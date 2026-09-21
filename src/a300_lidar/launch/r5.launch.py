from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            FindPackageShare("a300_lidar"), "config", "r5.yaml"
        ])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("bluesea2"),
                    "launch", "uart_lidar.launch"
                ])
            ),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file")
            }.items()
        )
    ])
