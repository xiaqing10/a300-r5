# A300 simulation

The simulation is intended to reproduce the same /scan interface used by the real LDS-E110-R-5.

Current test world:
- flat ground
- long wall
- box obstacle

Next implementation step:
- add Gazebo differential-drive system
- add a 2D LiDAR sensor configured close to the R-5: 360 deg, 10 Hz, 0.9 deg
- publish LaserScan
- connect /cmd_vel to the simulated base
- validate obstacle safety with the same a300_safety_controller node.