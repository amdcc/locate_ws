from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("pid_control"),
        "config",
        "pid_control_params.yaml",
    ])

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the ROS2 parameter file for pid_control nodes",
    )

    pid_control_node = Node(
        package="pid_control",
        executable="pid_control_node",
        name="pid_control_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        pid_control_node,
    ])
