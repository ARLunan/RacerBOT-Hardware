// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ============================================================================
// Minimalist firmware.cpp
//   - Board targets: pico / pico2 ONLY (no teensy*/esp32*/gendrv* code paths -
//     no WiFi, no OTA, no ESP32 watchdog, no ESP32 Serial buffer tuning).
//   - Base: ACKERMANN ONLY - a single drive motor (ESC, closed-loop via
//     encoder+PID) plus a steering servo/ESC (open-loop position), instead of
//     the generic up-to-4-wheel differential/skid/mecanum kinematics path.
//   - No RC passthrough (Traxxas_RemCtl) here - see TRAXXAS_REMCTL_INTEGRATION.md
//     if you also want that layered on top.
//   - No syslog, no JOINT_STATE_SUBSCRIBER, no ultrasonic range sensor - strip
//     these to keep the file to only what an ACKERMANN pico/pico2 build needs.
//     IMU, magnetometer, battery monitor, and LIDAR passthrough are kept, since
//     those are independent of drivetrain/MCU choice and pico2_config.h expects
//     a real (non-fake) IMU/MAG by default.
// ============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <i2cdetect.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"
#include "led.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "battery.h"
#include "lidar.h"
#include "pwm.h"
#include "steering.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // 50Hz
#endif
#ifndef BATTERY_TIMER
#define BATTERY_TIMER 2000 // 2 sec
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t battery_publisher;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
float prev_voltage;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// ---- Single drive motor (closed-loop via encoder + PID) ----
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// ---- Steering actuator (open-loop position, no encoder) ----
Motor motor_str_controller(PWM_FREQUENCY, PWM_BITS, MOTOR_STR_INV, MOTOR_STR_PWM, MOTOR_STR_IN_A, MOTOR_STR_IN_B);
Steering steering(STEERING_FULL_RANGE_DEG, motor_str_controller);

Kinematics kinematics(
    Kinematics::ACKERMANN,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    FR_WHEELS_DISTANCE,
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;
MAG mag;

static inline float sgnf(float val) { return (val > 0) - (val < 0); }

// Converts commanded Twist (linear.x, angular.z) into a steering angle (rad)
// using the bicycle/Ackermann model, clamped to the servo's mechanical range.
float rotational_vel_to_steering_angle(float x_vel, float w_vel, float wheelbase)
{
    if (w_vel == 0 || x_vel == 0)
        return 0;

    float radius = x_vel / w_vel;
    if (fabs(radius) < STEERING_MIN_TURN_RADIUS)
    {
        radius = STEERING_MIN_TURN_RADIUS * sgnf(radius);
    }
    return atan(wheelbase / radius);
}

// Commands the steering servo to a target angle (rad); returns the angle actually achieved.
float steer(float steering_angle)
{
    float angle_deg = -steering_angle * 180.0 / M_PI;
    angle_deg = steering.set_position_deg(angle_deg);
    return -angle_deg * M_PI / 180.0;
}

// Reads back the current commanded steering position as an angle (rad).
float getSteeringPos()
{
    return -steering.get_position_deg() * M_PI / 180.0;
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        setLed(HIGH);
        delay(150);
        setLed(LOW);
        delay(150);
    }
    delay(1000);
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    steering.update(false);
}

void rclErrorLoop()
{
    while(true)
    {
        flashLED(2); // flash 2 times
    }
}

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        setLed(HIGH);
    }

    // steering angle from commanded x velocity + angular z, and wheelbase
    float steering_angle = rotational_vel_to_steering_angle(
        twist_msg.linear.x, twist_msg.angular.z, FR_WHEELS_DISTANCE);

    // get the required drive-wheel rpm based on required linear velocity
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z
    );

    // get the current speed of the drive motor
    float current_rpm1 = motor1_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the ESC is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));

    // only actually change steering angle while moving (angle is 0 at rest)
    if (twist_msg.linear.x != 0.0)
    {
        steer(steering_angle);
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(getSteeringPos(), current_rpm1);

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z
    );

    steering.update(true);
}

bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
    timespec tp;
    tp.tv_sec = time_ns / 1000000000;
    tp.tv_nsec = time_ns % 1000000000;
    clock_settime(CLOCK_REALTIME, &tp);
#else
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - millis();
#endif
    return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void twistCallback(const void * msgin)
{
    setLed(!getLed());

    prev_cmd_time = millis();
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg = imu.getData();
#ifdef USE_FAKE_IMU
    imu_msg.angular_velocity.z = odom_msg.twist.twist.angular.z;
#endif
    mag_msg = mag.getData();
#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

#ifndef USE_FAKE_MAG
    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
#endif

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
#ifndef USE_FAKE_MAG
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
#endif
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
#if defined(BATTERY_PIN) || defined(USE_INA219)
    battery_msg = getBattery();
    battery_msg.header.stamp.sec = time_stamp.tv_sec;
    battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    battery_msg.voltage = prev_voltage = battery_msg.voltage * 0.01 + prev_voltage * 0.99;
    EXECUTE_EVERY_N_MS(BATTERY_TIMER, {
        getBatteryPercentage(&battery_msg);
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL)) });
#endif
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
       moveBase();
       publishData();
    }
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    // if we have magnetomter, use imu/data_raw for madgwick filter
#ifndef USE_FAKE_MAG
        TOPIC_PREFIX "imu/data_raw"
#else
        TOPIC_PREFIX "imu/data"
#endif
    ));
#ifndef USE_FAKE_MAG
    RCCHECK(rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        TOPIC_PREFIX "imu/mag"
    ));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219)
    // create battery publisher
    RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    TOPIC_PREFIX "battery"
    ));
#endif
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = CONTROL_TIMER;
#if defined(MICRO_ROS_DISTRO_HUMBLE) || defined(MICRO_ROS_DISTRO_FOXY)
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
#else
    RCCHECK(rclc_timer_init_default2(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback,
        true
    ));
#endif
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    setLed(HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
#ifndef USE_FAKE_MAG
    RCSOFTCHECK(rcl_publisher_fini(&mag_publisher, &node));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219)
    RCSOFTCHECK(rcl_publisher_fini(&battery_publisher, &node));
#endif
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

    setLed(HIGH);

    return true;
}

void setup()
{
    Serial.begin(BAUDRATE);
    initLed();
#ifdef BOARD_INIT // board specific setup, must include Wire.begin
    BOARD_INIT
#else
    Wire.begin();
#endif

    i2cdetect();  // default range from 0x03 to 0x77
    initPwm();
    motor1_controller.begin();
    motor_str_controller.begin();

    bool imu_ok = imu.init();
    if (!imu_ok) // take IMU failure as fatal
    {
        Serial.println("IMU init failed");
        while (1)
        {
            flashLED(3); // flash 3 times
        }
    }
    bool mag_ok = mag.init();
    if (!mag_ok) // take MAG failure as fatal
    {
        Serial.println("MAG init failed");
        while (1)
        {
            flashLED(4); // flash 4 times
        }
    }
    initBattery();
    initLidar(); // UART passthrough to the host

    battery_msg = getBattery();
    prev_voltage = battery_msg.voltage;

    set_microros_serial_transports(Serial);

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif
}

void loop() {
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
#ifdef BOARD_LOOP // board specific loop
    BOARD_LOOP
#endif
}
