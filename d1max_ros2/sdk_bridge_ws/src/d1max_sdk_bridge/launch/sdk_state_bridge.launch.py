from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_dir = get_package_share_directory("d1max_sdk_bridge")
    default_config = os.path.join(package_dir, "config", "bridge.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("robot_ip", default_value="192.168.168.168"),
        DeclareLaunchArgument("robot_port", default_value="8081"),
        Node(
            package="d1max_sdk_bridge",
            executable="sdk_state_bridge",
            name="d1max_sdk_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("config"),
                {
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robot_port": LaunchConfiguration("robot_port"),
                },
            ],
        ),
    ])
