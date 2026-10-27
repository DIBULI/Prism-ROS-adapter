"""Host SDK 1.3.0 over trusted Ethernet; password stays in the environment."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("prism_ros_driver"), "config", "default.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("host", description="Device numeric IPv4/IPv6 address"),
        DeclareLaunchArgument("port", default_value="7443"),
        DeclareLaunchArgument("token_env", default_value="PRISM_NETWORK_TOKEN"),
        Node(package="prism_ros_driver", executable="prism_ros_driver_node",
             name="prism_ros_driver", output="screen",
             parameters=[LaunchConfiguration("config"), {
                 "transport": "network",
                 "network_host": ParameterValue(LaunchConfiguration("host"), value_type=str),
                 "network_port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                 "network_token_env": ParameterValue(LaunchConfiguration("token_env"), value_type=str),
             }]),
    ])
