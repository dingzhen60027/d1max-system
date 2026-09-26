"""Owned execution stack only. Never starts SDK or automatically enables it."""
import json
import os
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    directory = Path(LaunchConfiguration('session').perform(context))
    s = json.loads((directory/'session.json').read_text())
    if (s.get('mode') != 'LIVE_NAVIGATION' or s.get('motion_control_enabled') is not True
            or os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp'):
        raise RuntimeError('Explicit live motion session and Zenoh required')
    nodes = []
    for package, executable, name in (
        ('d1max_trajectory_tracker', 'trajectory_tracker', 'trajectory_tracker'),
        ('d1max_navigation', 'navigation_cloud_to_scan', 'navigation_cloud_to_scan'),
        ('d1max_motion_safety', 'continuous_collision_monitor', 'collision_monitor'),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'motion_lifecycle_manager'),
        ('d1max_navigation', 'navigation_command_gate', 'navigation_command_gate'),
    ):
        nodes.append(Node(package=package, executable=executable, name=name,
            namespace='/d1max/live_planning', output='screen',
            parameters=[str(directory/'motion.yaml')]))
    nodes.append(ExecuteProcess(cmd=[sys.executable, '-m', 'd1max_pct_scan.motion_coordinator',
                                    '--session', str(directory)], output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='A motion component exited; revoke execution'))])) for node in nodes]
    return handlers+nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session'), OpaqueFunction(function=start)])
