from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="d1max_sdk_gui",
            executable="d1max_sdk_gui",
            name="d1max_sdk_gui",
            output="screen",
        ),
    ])
