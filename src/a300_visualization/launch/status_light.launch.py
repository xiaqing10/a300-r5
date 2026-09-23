from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="a300_visualization",
            executable="status_light",
            name="a300_status_light",
            output="screen",
        ),
    ])
