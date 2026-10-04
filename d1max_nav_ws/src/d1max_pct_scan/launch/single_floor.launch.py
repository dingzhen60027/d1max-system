"""One supervisor owns the core graph; presentation lifetime is independent."""
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session'),ExecuteProcess(
        cmd=[sys.executable,'-m','d1max_pct_scan.single_floor_session','run',
             '--session',LaunchConfiguration('session')],output='screen')])
