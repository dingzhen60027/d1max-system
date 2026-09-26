"""Nav2 map + RViz only. Existing Web owns LIO/PCD localization and live uplink."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise RuntimeError('D1 Max requires rmw_zenoh_cpp; middleware fallback is forbidden')
    settings = json.loads(Path(LaunchConfiguration('session').perform(context)).read_text())
    map_server = Node(package='nav2_map_server', executable='map_server',
        namespace='d1max/navigation', name='map_server', output='screen', parameters=[{
            'yaml_filename': settings['map_yaml'], 'frame_id': settings['map_frame'],
            'topic_name': settings['map_topic'], 'use_sim_time': False}])
    lifecycle = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        namespace='d1max/navigation', name='lifecycle_manager_map', output='screen', parameters=[{
            'autostart': True, 'node_names': ['map_server'],
            'bond_timeout': 4.0, 'use_sim_time': False}])
    bridge = Node(package='d1max_navigation', executable='rviz_initial_pose_bridge',
        name='rviz_initial_pose_bridge', output='screen', parameters=[{
            'web_url': settings['web_url'], 'map_frame': settings['map_frame'],
            'initial_pose_topic': settings['initial_pose_topic'],
            'expected_version_id': settings['version_id'],
            'expected_session_id': settings.get('localization_session_id', ''),
            'initial_pose_z': float(settings['initial_pose_z']),
            'offline': settings['mode'] == 'offline', 'use_sim_time': False}])
    rviz = Node(package='rviz2', executable='rviz2', name='d1max_navigation_rviz',
        arguments=['-d', settings['rviz_config']], output='screen',
        additional_env={'QT_QPA_PLATFORM': 'xcb'}, parameters=[{'use_sim_time': False}])
    status = Node(package='d1max_navigation', executable='navigation_test_status',
        name='navigation_test_status', output='screen', parameters=[{
            'map_frame': settings['map_frame'], 'offline': settings['mode'] == 'offline',
            'expected_version_id': settings['version_id'],
            'expected_session_id': settings.get('localization_session_id', ''),
            'anchor_x': float(settings['status_anchor_x']),
            'anchor_y': float(settings['status_anchor_y']), 'anchor_z': 1.0,
            'text_height': 0.8, 'use_sim_time': False}])
    nodes = [map_server, lifecycle, bridge, status, rviz]
    if settings['mode'] == 'offline':
        router = ExecuteProcess(cmd=[str(Path(get_package_prefix('rmw_zenoh_cpp')) /
            'lib/rmw_zenoh_cpp/rmw_zenohd')], output='screen')
        actions = [router, TimerAction(period=2.0, actions=nodes)]
        watched = [router] + nodes
    else:
        actions = nodes
        watched = nodes
    # Closing RViz or a child failure terminates this launch's complete process group.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='Navigation test window/node exited'))])) for node in watched]
    return handlers + actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session'), OpaqueFunction(function=start)])
