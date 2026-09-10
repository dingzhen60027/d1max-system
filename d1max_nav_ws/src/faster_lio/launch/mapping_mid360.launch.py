import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # ---------- Livox MID360 driver ----------
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('livox_ros_driver2'),
                'launch_ROS2', 'msg_MID360_launch.py'
            ])
        )
    )

    # ---------- FASTer-LIO mapping node ----------
    config_file = PathJoinSubstitution([
        get_package_share_directory('faster_lio'), 'config', 'mid360.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('faster_lio'), 'rviz_cfg', 'loam_livox.rviz'
    ])

    faster_lio_node = Node(
        package='faster_lio',
        executable='run_mapping_online',
        output='screen',
        parameters=[config_file],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config,
                   '--ros-args', '--log-level', 'warn'],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)
    ld.add_action(livox_driver)
    ld.add_action(faster_lio_node)
    ld.add_action(rviz_node)

    return ld
