from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory('d1max_pct_planner')) / 'config/pct_scan_single_floor.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        Node(package='d1max_pct_planner', executable='pct_route_server',
             name='pct_route_server', parameters=[LaunchConfiguration('config')], output='screen'),
    ])
