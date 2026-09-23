# A300 R5 ROS 2

A300 intelligent wheelchair development workspace.

## Architecture

Real hardware:

`LDS-E110-R-5 -> BlueSea official ROS 2 driver -> /scan -> A300 obstacle safety -> /cmd_vel`

Simulation:

`Gazebo R-5-like LiDAR -> /scan -> the same A300 obstacle safety node -> /cmd_vel -> Gazebo DiffDrive`

SLAM and Nav2 remain outside the final safety/velocity constraint layer.

## Target

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic / the Gazebo version paired with the installed ROS 2 Jazzy packages
- LDS-E110-R-5
- SLAM Toolbox
- Nav2 (later)

## Documentation

| Document | Content |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Target layered architecture, topic contracts, TF tree, current-state mapping |
| [`docs/DESIGN.md`](docs/DESIGN.md) | Per-layer algorithm specification, state machines, parameter tables, interfaces of planned modules |
| [`docs/REVIEW.md`](docs/REVIEW.md) | Code-level review, defect list by severity (P0/P1/P2), fix recommendations |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | Milestones M0-M7 with acceptance criteria and architecture invariants |
| [`docs/SETUP.md`](docs/SETUP.md) | Environment setup |
| [`docs/SIMULATION.md`](docs/SIMULATION.md) | Simulation details |
| [`docs/REAL_LIDAR.md`](docs/REAL_LIDAR.md) | LDS-E110-R-5 integration |

## Packages

- `a300_description`: robot model, wheel geometry and LiDAR mounting
- `a300_gazebo`: Gazebo world, DiffDrive, simulated R-5 LiDAR and ROS/Gazebo bridge
- `a300_lidar`: official BlueSea driver integration wrapper
- `a300_obstacle_avoidance`: real-time safety layer
- `a300_joystick`: joystick -> desired Twist
- `a300_base_controller`: future hardware differential-drive controller
- `a300_slam`: SLAM Toolbox integration
- `a300_nav`: future Nav2 integration
- `a300_bringup`: system launch

## Gazebo simulation

Install the ROS/Gazebo integration recommended for the selected ROS 2 distribution:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

Then build the workspace:

```bash
cd ~/a300-r5
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Start the complete simulation:

```bash
ros2 launch a300_gazebo sim.launch.py
```

The launch file starts:

1. Gazebo test world
2. A300 robot model
3. `robot_state_publisher`
4. Gazebo <-> ROS 2 bridge
5. A300 obstacle safety controller
6. Robot spawning

Useful checks:

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /cmd_vel
ros2 topic echo /a300/obstacle_state
ros2 run tf2_tools view_frames
```

Drive the simulated wheelchair manually for a basic test:

```bash
ros2 topic pub --rate 10 /cmd_vel_desired geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

The safety node should pass the command when the path is clear and reduce/stop forward velocity when the simulated LiDAR detects an obstacle.

Stop the command with:

```bash
ros2 topic pub --once /cmd_vel_desired geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Simulation LiDAR model

The simulated sensor intentionally follows the main R-5 characteristics used by the A300 development model:

- 360 degree horizontal field of view
- 400 samples per scan
- approximately 0.9 degree angular resolution
- 10 Hz
- 0.05 m minimum range
- 12 m maximum range

These are simulation parameters, not a replacement for calibration against the physical LDS-E110-R-5.

## Important calibration note

The current A300 dimensions are development placeholders:

- body: 0.75 x 0.62 x 0.75 m
- wheel radius: 0.18 m
- wheel track: 0.55 m
- LiDAR position: x=0.20 m, z=0.85 m
- robot mass: 60 kg

Before using simulation results to tune hardware stopping distances, replace them with measured A300 values. In particular, measure wheel diameter/track, LiDAR x/y/z position, actual chassis footprint and caster geometry.

## Safety architecture

The simulation uses the same velocity-constraining safety node intended for real LiDAR data:

`/scan -> a300_safety_controller -> /cmd_vel`

The joystick or future Nav2 stack should publish a desired command, not bypass the safety layer.

This is a development simulation and is not safety-certified.

## Real R-5

For real hardware, use the official BlueSea ROS 2 driver and keep the ROS-side interface at `/scan`. The current repository wraps the vendor launch/configuration rather than reimplementing the LiDAR packet parser.

## Next milestones

1. Replace placeholder A300 geometry with measured CAD/mechanical data.
2. Validate simulated LiDAR frame and mounting height.
3. Validate forward/left/right obstacle sectors against the real wheelchair footprint.
4. Tune speed-dependent braking distance using measured A300 latency and deceleration.
5. Add simulated odometry noise/slip for more realistic SLAM testing.
6. Connect SLAM Toolbox.
7. Add Nav2 while keeping its velocity output behind the A300 safety layer.
8. Add the real motor controller interface.
