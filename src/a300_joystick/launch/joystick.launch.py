from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
        Node(
            package="a300_joystick",
            executable="a300_joystick_mapper",
            name="a300_joystick_mapper",
            output="screen",
            parameters=["config/joystick.yaml"],
        ),
    ])