from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port",default_value="/dev/ttyUSB0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("bluesea2"),"launch","uart_lidar.launch.py"])),
            launch_arguments={"port":LaunchConfiguration("port"),"scan_topic":"/scan","frame_id":"laser_link","output_scan":"true","output_360":"true","min_dist":"0.05","max_dist":"12.0"}.items())])