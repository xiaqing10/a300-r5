# LDS-E110-R-5 integration

The A300 project uses the official BlueSea ROS 2 driver instead of duplicating the UART protocol parser.

R-5 values from the supplied manual:
- UART: 230400, 8 data bits, 1 stop bit, no parity
- supply: 5.1 +/- 0.2 V
- 360 degree scan
- typical rotation: 10 Hz
- angular resolution: 0.9 degree
- 4000 measurements/s
- raw output contains distance, angle and intensity

The official driver publishes ROS 2 LaserScan data. A300 application nodes should depend on LaserScan rather than on the driver's internal packet format.

Before real tests:
1. Confirm the actual USB/UART device path.
2. Confirm the driver launch file/parameter names for the installed BlueSea driver revision.
3. Verify frame_id is laser_link.
4. Verify scan direction and zero angle in RViz2.
5. Record a rosbag for regression testing.