from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pid_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("pid_control"),
                "launch",
                "pid_control_launch.py",
            ])
        )
    )

    uart_to_mcu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("uart_to_mcu"),
                "launch",
                "uart_to_mcu.launch.py",
            ])
        )
    )

    return LaunchDescription([
        pid_control_launch,
        uart_to_mcu_launch,
    ])
