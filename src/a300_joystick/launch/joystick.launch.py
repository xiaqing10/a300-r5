from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("a300_joystick"), "config", "joystick.yaml"
    ])
    return LaunchDescription([
        Node(package="joy", executable="joy_node", name="joy_node", output="screen"),
        Node(
            package="a300_joystick",
            executable="a300_joystick_mapper",
            name="a300_joystick_mapper",
            output="screen",
            parameters=[config],
        ),
    ])
