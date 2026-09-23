#!/usr/bin/env bash
# One-click startup for the a300-r5 obstacle-avoidance safety demo.
# Launches Gazebo sim + status_light (green/yellow/orange/red ball), then
# hands you keyboard teleop to drive the robot toward the wall.
#
# Usage:
#   bash start_demo.sh
#
# Drive with the keyboard in THIS same terminal:
#     i  forward            j  turn left
#     ,  backward           l  turn right
#     k  stop
#     q/z  faster/slower (all speeds)
#     w/x  faster/slower (linear only)
#     e/c  faster/slower (angular only)
#     Ctrl-C  quit teleop
#
# Watch: ball turns green -> yellow -> orange -> red and the robot auto-stops
# in front of the wall.

WS="/mnt/c/Users/夏青/Desktop/a300/a300-r5"

echo "==> Stopping any previous sim / bridge / status_light / teleop processes..."
ps aux | grep -E \
  "gz server|gz sim|gz sim gui|sim.launch|status_light|parameter_bridge|ros_gz_bridge|robot_state_pub|a300_safety_controller|gz-transport-service|ros_gz_sim|ros2 launch|teleop_twist_keyboard" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
sleep 3

echo "==> Sourcing ROS 2 + workspace..."
export PATH=/opt/ros/jazzy/bin:$PATH
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "==> Launching Gazebo simulation (background)..."
nohup ros2 launch a300_gazebo sim.launch.py > /tmp/sim_demo.log 2>&1 &
disown
echo "    sim log: /tmp/sim_demo.log"

echo "==> Waiting for the robot to spawn..."
sleep 16

echo "==> Launching status_light (background)..."
nohup ros2 run a300_visualization status_light > /tmp/status_light.log 2>&1 &
disown
echo "    status_light log: /tmp/status_light.log"

sleep 3
echo ""
echo "===================================================="
echo "  Demo is running. Keyboard teleop is active below."
echo ""
echo "  Drive the robot toward the wall with your keyboard:"
echo "      i  = forward       j  = turn left"
echo "      ,  = backward      l  = turn right"
echo "      k  = stop"
echo "      q/z = faster/slower"
echo "      Ctrl-C to quit teleop"
echo ""
echo "  Watch the ball: green -> yellow -> orange -> red,"
echo "  then the robot auto-stops in front of the wall."
echo "===================================================="
echo ""

echo "==> Starting keyboard teleop (type keys in this terminal)..."
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=cmd_vel_desired -p speed:=0.3
echo "==> Teleop ended. Demo processes (sim, status_light) are still running."
echo "    Re-run 'bash start_demo.sh' to start over."
