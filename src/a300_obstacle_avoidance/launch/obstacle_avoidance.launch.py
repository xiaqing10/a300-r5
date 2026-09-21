from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package="a300_obstacle_avoidance",executable="a300_safety_controller",name="a300_safety_controller",output="screen",parameters=["config/obstacle.yaml"])])