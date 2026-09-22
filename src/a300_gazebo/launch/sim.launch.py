from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_gazebo = FindPackageShare("a300_gazebo")
    pkg_description = FindPackageShare("a300_description")
    world = LaunchConfiguration("world")
    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "a300.urdf.xacro"])
    robot_description = Command(["xacro ", xacro_file, " use_sim:=true"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": True}],
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="a300_gz_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
    )
    safety = Node(
        package="a300_obstacle_avoidance", executable="a300_safety_controller",
        name="a300_safety_controller", output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("a300_obstacle_avoidance"), "config", "obstacle.yaml"]),
            {"use_sim_time": True},
        ],
    )
    spawn = TimerAction(period=2.0, actions=[
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "a300", "-x", "0", "-y", "0", "-z", "0.20"])
    ])
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([pkg_gazebo, "worlds", "a300_test.sdf"])),
        gazebo, robot_state_publisher, bridge, safety, spawn
    ])
