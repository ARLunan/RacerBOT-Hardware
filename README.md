# RacerBOT-Hardware
ROS2 Jazzy Ackerman drive packages to load into a RaspberryPi RP2350 Pico2 controller based on the linorobot2 micro-ROS architecture package configuration, refactored from repo https://github.com/hippo5329/linorobot2_hardware.git (that was the basis for the Pico 2 Controller code) and https://github.com/jimdinunzio/linorobot2_hardware.git (that was the basis for the ESC/Steering code functions).
 
 Design chassis is the Traxxas 1/18 LaTrax Rally ESC Drive and Steering Servo. Robot Controller is a RaspBerry PI 4/4GHz configured with Ubuntu 24.04/ROS 2 Jazzy and micro-ROS. A Lidar LD19 is connected with USB, Driving wheels equipped with AS5600 Encoders. Realtime Controller is a Raspberry Pi Pico2 install with micro-ROS connected by USB-C to the Raspberry PI4

 This repository "purpose" defines for the single target Pico2 controller and does not have the other targets included in the other linorobot2 repositories.

This repository targets a MAC M1 Toolchain development machine configured with Homebrew PlatformIO and picotool, Microsoft VSCode with extensions: Remote:SSH, PlatformIO IDE, Micro Pico, Raspberry Pi Pico. A Linux Machine can be used with a number of appropriate changes.  

**RacerBOT-Robot** ROS2 Packages on the Raspberry Pi4 launch required "bringup" that include: micro0ROS robot_Agent, EKF_node, LD19 Lidar, robot Description (robot base URDF, robot_state_publisher, joint state publisher and rviz) 

This RacerBOT Robot is expected to be used with a companion **RacerBOT-Desktop** running on a separate Linux Ubuntu 24.4/ROS 2 Jazzy based workstation configured with sets of slam_toolbox mapping and navigation2 launch packages.