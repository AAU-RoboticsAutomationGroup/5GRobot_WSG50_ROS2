from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	
	config_param = DeclareLaunchArgument("config", default_value="true")

	return LaunchDescription([
		config_param,
		Node(
			package="wsg_ctrl",
			executable="wsg",
			output="screen",
			parameters=[{"config": LaunchConfiguration("config")}]
		),
	])
