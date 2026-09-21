# A300 R5 ROS 2

A300 intelligent wheelchair development workspace.

## Architecture

Real: LDS-E110-R-5 -> BlueSea official ROS 2 driver -> /scan -> A300 obstacle safety -> /cmd_vel.

Simulation: Gazebo LiDAR -> /scan -> the same A300 obstacle safety node -> /cmd_vel.

SLAM and Nav2 are intentionally separated from the final safety/velocity constraint layer.

## Target
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo
- LDS-E110-R-5
- SLAM Toolbox
- Nav2 (later)

## Packages
- a300_description: robot model and LiDAR mounting
- a300_gazebo: simulation
- a300_lidar: official BlueSea driver integration wrapper
- a300_obstacle_avoidance: real-time safety layer
- a300_joystick: joystick -> desired Twist
- a300_base_controller: future real differential-drive controller
- a300_slam: SLAM Toolbox integration
- a300_nav: future Nav2 integration
- a300_bringup: system launch

## First milestone
1. Verify R-5 /scan with the official driver.
2. Verify the scan in RViz2.
3. Run the A300 Gazebo model.
4. Feed simulated /scan into the same safety node.
5. Add SLAM and map the test world.
