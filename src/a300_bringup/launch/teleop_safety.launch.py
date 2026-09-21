from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("a300_joystick"), "launch", "joystick.launch.py"])
        )
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("a300_obstacle_avoidance"),
                "launch", "obstacle_avoidance.launch.py"
            ])
        )
    )
    return LaunchDescription([joy, safety])