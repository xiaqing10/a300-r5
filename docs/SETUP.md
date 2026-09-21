# A300 R5 ROS 2 setup

Target: Ubuntu 24.04 LTS + ROS 2 Jazzy.

## Workspace

    mkdir -p ~/a300_ws/src
    cd ~/a300_ws/src
    git clone https://github.com/xiaqing10/a300-r5.git

## Official R-5 driver

    git clone https://github.com/BlueSeaLidar/bluesea-ros2.git

The BlueSea driver remains an external dependency. This repository only wraps its launch/configuration.

## Dependencies and build

    cd ~/a300_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

## Verify R-5

    ls -l /dev/ttyUSB*
    ros2 launch a300_lidar r5.launch.py
    ros2 topic list
    ros2 topic hz /scan

Open RViz2 and add LaserScan on /scan.

## Safety layer

    ros2 launch a300_obstacle_avoidance obstacle_avoidance.launch.py

The safety layer consumes /scan and /cmd_vel_desired and publishes constrained /cmd_vel.

## Joystick

    sudo apt install ros-jazzy-joy
    ros2 launch a300_joystick joystick.launch.py

Default mapping: axis 1 -> linear X, axis 0 -> angular Z. Verify the actual joystick before any vehicle test.

## Safety

Thresholds in this repository are development defaults only. Do not drive a real wheelchair using them before validating latency, stopping distance, maximum speed, LiDAR mounting, controller behavior, emergency stop, watchdog, and applicable product safety requirements.
