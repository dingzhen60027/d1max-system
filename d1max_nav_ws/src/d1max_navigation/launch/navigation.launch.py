"""Complete Nav2 software chain. Physical SDK is never started by this launch."""
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
from nav2_common.launch import RewrittenYaml

NS='d1max/navigation'


def start(context):
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp':
        raise RuntimeError('D1 Max forbids middleware fallback')
    cfg=json.loads(Path(LaunchConfiguration('session').perform(context)).read_text())
    sim=cfg['mode']=='sim'
    params=RewrittenYaml(source_file=cfg['nav2_config'],root_key=NS,
        param_rewrites={'use_sim_time':'false',
            'default_nav_to_pose_bt_xml':cfg['bt_to_pose'],
            'default_nav_through_poses_bt_xml':cfg['bt_through_poses']},convert_types=True)
    collision_params=RewrittenYaml(source_file=cfg['collision_monitor_config'],root_key=NS,
        param_rewrites={'use_sim_time':'false'},convert_types=True)
    nodes=[]
    def node(package, executable, name=None, parameters=None, remappings=None, **kwargs):
        value=Node(package=package,executable=executable,name=name or executable,
            namespace=NS,output='screen',parameters=parameters or [params],remappings=remappings or [],**kwargs)
        nodes.append(value); return value
    node('nav2_map_server','map_server',parameters=[{'yaml_filename':cfg['map_yaml'],
        'frame_id':cfg['map_frame'],'topic_name':'/d1max/navigation/map','use_sim_time':False}])
    node('nav2_lifecycle_manager','lifecycle_manager','lifecycle_manager_localization',parameters=[{
        'autostart':True,'node_names':['map_server'],'bond_timeout':4.0,'use_sim_time':False}])
    if sim:
        node('d1max_navigation','navigation_simulator',parameters=[{
            'allow_simulation':True,**cfg['simulation'],'map_version_id':cfg['version_id'],
            'max_linear':cfg['motion_limits']['max_forward'],'max_angular':cfg['motion_limits']['max_yaw'],
            'use_sim_time':False}])
    else:
        node('d1max_navigation','navigation_cloud_to_scan',parameters=[{**cfg['cloud_projection'],'use_sim_time':False}])
        node('d1max_navigation','rviz_initial_pose_bridge',parameters=[{
            'web_url':cfg['web_url'],'expected_version_id':cfg['version_id'],
            'expected_session_id':cfg['localization_session_id'],
            'initial_pose_z':float(cfg['initial_pose_z']),'offline':False,'use_sim_time':False}])
        node('d1max_navigation','navigation_test_status',parameters=[{
            'offline':False,'show_navigation_gate':True,'expected_version_id':cfg['version_id'],
            'expected_session_id':cfg['localization_session_id'],
            'anchor_x':float(cfg['status_anchor_x']),'anchor_y':float(cfg['status_anchor_y']),
            'anchor_z':1.0,'text_height':.8,'use_sim_time':False}])
    node('d1max_navigation','navigation_command_gate',parameters=[{
        'mode':'simulation' if sim else 'live', 'motion_enabled':bool(cfg['enable_motion']),
        'expected_version_id':cfg['version_id'],'expected_session_id':cfg['localization_session_id'],
        'navigation_session_id':cfg['navigation_session_id'],
        'max_planar_speed':cfg['motion_limits']['max_planar'],
        **cfg['command_gate_limits'],
        'use_sim_time':False}])
    node('nav2_controller','controller_server',remappings=[('cmd_vel','cmd_vel_raw')])
    node('nav2_planner','planner_server')
    node('nav2_smoother','smoother_server')
    node('nav2_behaviors','behavior_server',remappings=[('cmd_vel','cmd_vel_raw')])
    node('nav2_bt_navigator','bt_navigator')
    node('nav2_waypoint_follower','waypoint_follower')
    node('nav2_velocity_smoother','velocity_smoother',remappings=[
        ('cmd_vel','cmd_vel_raw'),('cmd_vel_smoothed','cmd_vel_smoothed')])
    node('nav2_collision_monitor','collision_monitor',parameters=[collision_params])
    node('nav2_lifecycle_manager','lifecycle_manager','lifecycle_manager_navigation',parameters=[{
        'autostart':True,'bond_timeout':4.0,'use_sim_time':False,
        'node_names':['controller_server','smoother_server','planner_server','behavior_server',
                      'velocity_smoother','collision_monitor','bt_navigator','waypoint_follower']}])
    if not cfg['headless']:
        node('rviz2','rviz2','navigation_rviz',parameters=[{'use_sim_time':False}],
            arguments=['-d',cfg['rviz_config']],additional_env={'QT_QPA_PLATFORM':'xcb'})
    if sim:
        router=ExecuteProcess(cmd=[str(Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd')],output='screen')
        watched=[router]+nodes; actions=[router,TimerAction(period=2.0,actions=nodes)]
    else:
        watched=nodes; actions=nodes
    handlers=[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[
        EmitEvent(event=Shutdown(reason='A managed navigation node/window exited'))])) for n in watched]
    return handlers+actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session'),OpaqueFunction(function=start)])
