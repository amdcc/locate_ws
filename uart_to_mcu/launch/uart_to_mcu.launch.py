from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	serial_port_arg = DeclareLaunchArgument(
		"serial_port",
		default_value="/dev/ttyS1",
		description="Serial device for sending wheel speed packets",
	)

	wheel_topic_arg = DeclareLaunchArgument(
		"wheel_speeds_topic",
		default_value="/wheel_speeds",
		description="Topic that publishes left/right wheel speeds",
	)

	uart_node = Node(
		package="uart_to_mcu",
		executable="uart_to_mcu_node",
		name="uart_to_mcu_node",
		output="screen",
		parameters=[{
			"serial_port": LaunchConfiguration("serial_port"),
			"wheel_speeds_topic": LaunchConfiguration("wheel_speeds_topic"),
		}],
	)

	return LaunchDescription([
		serial_port_arg,
		wheel_topic_arg,
		uart_node,
	])
