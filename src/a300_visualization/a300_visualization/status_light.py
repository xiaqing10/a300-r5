#!/usr/bin/env python3

import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

WORLD_NAME = "a300_test"

BALLS = {
    "SAFE": "status_green",
    "WARNING": "status_yellow",
    "SLOW": "status_orange",
    "AVOIDING": "status_orange",
    "STOP": "status_red",
}

COLORS = list(BALLS.values())

HIDDEN_X = 50.0
BALL_Z = 1.6


def gz_set_pose(name, x, y, z):
    req = f'name: "{name}"\nposition {{ x: {x} y: {y} z: {z} }}'
    cmd = [
        "gz", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--req", req,
    ]
    try:
        subprocess.run(cmd, capture_output=True, text=True, timeout=8)
    except Exception:
        pass


class StatusLight(Node):
    def __init__(self):
        super().__init__("a300_status_light")
        self.state = "SAFE"
        self.world_x = 0.0
        self.world_y = 0.0
        self.world_yaw = 0.0
        self.current = None

        self.sub_state = self.create_subscription(
            String, "/a300/obstacle_state", self.state_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10)

        # 5 Hz update (only acts on state changes)
        self.timer = self.create_timer(0.2, self.update)

    def state_cb(self, msg):
        s = msg.data.strip().upper()
        if s in BALLS:
            self.state = s

    def odom_cb(self, msg):
        # The odom frame is mirrored 180 deg about Z vs the gz world frame
        # (odom.x = -world.x, odom.y = -world.y) under the safety controller's
        # drive-direction fix, so negate to recover authoritative world coords.
        p = msg.pose.pose
        self.world_x = -p.position.x
        self.world_y = -p.position.y
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.world_yaw = yaw

    def update(self):
        active = BALLS[self.state]
        if self.current == active:
            return
        # Place the active ball above the robot; hide the previous one.
        if self.current is not None:
            gz_set_pose(self.current, HIDDEN_X, HIDDEN_X, BALL_Z)
        gz_set_pose(active, self.world_x, self.world_y, BALL_Z)
        self.current = active
        self.get_logger().info(
            "ball=%s at (%.2f, %.2f, %.2f)" %
            (active, self.world_x, self.world_y, BALL_Z))


def main(args=None):
    rclpy.init(args=args)
    node = StatusLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
