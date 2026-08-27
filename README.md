# RacerBOT-Hardware
ROS2 Jazzy Ackerman drive packages to load into a RaspberryPi RP2350 Pico2 controller based on the linorobot2 micro-ROS architecture package configuration, refactored from repo https://github.com/hippo5329/linorobot2_hardware.git (that was the basis for the Pico 2 Controller code) and https://github.com/jimdinunzio/linorobot2_hardware.git (that was the basis for the ESC/Steering code functions).

Anthropic Claude Sonnet 5 Medium used to augment this developer's work to analyze and develop code.
 
 Design chassis is the Traxxas 1/18 LaTrax Rally ESC Drive and Steering Servo. Robot Controller is a RaspBerry PI 4/4GHz configured with Ubuntu 24.04/ROS 2 Jazzy and micro-ROS. A Lidar LD19 is connected with USB, Driving wheels equipped with A3144 HallEffect Encoders. Realtime Controller is a Raspberry Pi Pico2 install with micro-ROS connected by USB to the Raspberry PI4.

 Firmwware interaction with linorobot2 ROS nodes

The micro-ROS robot controller firmware subscribes to /cmd_vel, converts the Twist message and commands motor drivers to move the wheels under PID control. It publishes wheel odometry to /odom/unfiltered and IMU data to /imu/data. On the robot computer, an EKF filter fuses /odom/unfiltered and /imu/data to publish /odom consumed by  slam_toolbox and navigation2 . Robot state, joint state and TF transforms are published.

 This repository "purpose" defines for the single target Pico2 controller and does not have the other targets included in the other linorobot2 repositories.

This repository targets a MAC M1 Toolchain development machine configured with Homebrew PlatformIO and picotool, Microsoft VSCode with extensions: Remote:SSH, PlatformIO IDE, Micro Pico, Raspberry Pi Pico. A Linux Machine can be used with a number of appropriate changes.  

**RacerBOT-Robot** robot computer ROS2 Packages on the Raspberry Pi4 launch required "bringup" that include: micro0ROS robot_Agent, EKF_filter_node, LD19 Lidar, Robot Description (robot base URDF, robot_state_publisher, joint state publisher and rviz)

This RacerBOT Robot is expected to be used with a companion **RacerBOT-Desktop** running on a separate Linux Ubuntu 24.4/ROS 2 Jazzy based workstation configured with sets of slam_toolbox mapping and navigation2 launch packages.
