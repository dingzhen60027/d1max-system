"""Localization only. No SDK ownership, robot controls, drivers, URDF or Nav2."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
import yaml
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from d1max_localization.estimation.configuration import navigation_parameters

def launch_nodes(context):
    config = LaunchConfiguration('config')
    session = LaunchConfiguration('session_dir')
    cloud = LaunchConfiguration('map_pcd')
    value=yaml.safe_load(Path(config.perform(context)).read_text())
    backend=value.get('localization_pipeline',{}).get('ros__parameters',{}).get('backend','legacy_ekf')
    if backend not in ('lio_pcd','legacy_ekf'):raise RuntimeError('Unknown localization backend')
    if backend=='lio_pcd':
        navigation=navigation_parameters(value)
        nodes=[
          Node(package='d1max_localization',executable='dual_lidar_adapter',name='dual_lidar_adapter',parameters=[config],output='screen'),
          Node(package='faster_lio',executable='run_mapping_online',namespace='d1max/localization/lio',name='laserMapping',parameters=[config],
               remappings=[('Odometry','odometry'),('cloud_registered_body','deskewed')],output='screen'),
          Node(package='d1max_localization',executable='fused_icp_matcher',name='lio_global_matcher',parameters=[config,{'map_pcd':cloud}],output='screen'),
          Node(package='d1max_localization',executable='lio_localizer',name='lio_localizer',parameters=[config,{'session_dir':session,'navigation_output_enabled':navigation is not None}],output='screen')]
        if navigation:
            prediction,output,ekf=navigation
            nodes.extend([
              Node(package='d1max_localization',executable='lio_predictor',name='lio_predictor',parameters=[prediction],output='screen'),
              Node(package='robot_localization',executable='ekf_node',name='ekf_navigation',parameters=[config,ekf],output='screen',
                   remappings=[('odometry/filtered','/d1max/localization/estimator/odometry_raw'),
                               ('set_pose','/d1max/localization/estimator/set_pose')]),
              Node(package='d1max_localization',executable='navigation_output',name='navigation_output',parameters=[output],output='screen')])
        return [*[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='A localization node exited; stopping the complete session'))])) for n in nodes],*nodes]
    nodes = [
        Node(package='d1max_localization', executable='dual_lidar_adapter', name='dual_lidar_adapter', parameters=[config], output='screen'),
        Node(package='d1max_localization', executable='fused_icp_matcher', name='fused_icp_matcher', parameters=[config, {'map_pcd':cloud}], output='screen'),
        Node(package='d1max_localization', executable='icp_fusion_bridge', name='icp_fusion_bridge', parameters=[config], output='screen'),
        Node(package='d1max_localization', executable='localization_supervisor', name='localization_supervisor', parameters=[config, {'session_dir':session}], output='screen'),
    ]
    for which in ('local','global'):
        nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_'+which, parameters=[config], output='screen', remappings=[('odometry/filtered','/d1max/localization/odometry/'+which),('set_pose','/d1max/localization/'+which+'/set_pose')]))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown(reason='A localization node exited; stopping the complete session'))])) for node in nodes]
    return [*handlers,*nodes]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(Path(get_package_share_directory('d1max_localization'))/'config/localization.yaml')),
        DeclareLaunchArgument('session_dir'), DeclareLaunchArgument('map_pcd'), OpaqueFunction(function=launch_nodes),
    ])
