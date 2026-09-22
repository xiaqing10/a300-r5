from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("a300_obstacle_avoidance"), "config", "obstacle.yaml"])
    return LaunchDescription([Node(package="a300_obstacle_avoidance", executable="a300_safety_controller", name="a300_safety_controller", output="screen", parameters=[config])])
