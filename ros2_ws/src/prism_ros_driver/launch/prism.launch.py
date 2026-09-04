from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("prism_ros_driver"), "config", "default.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="prism_ros_driver",
                executable="prism_ros_driver_node",
                name="prism_ros_driver",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
