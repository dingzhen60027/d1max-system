"""Motion-specific wiring; no duplication of localization/global/local planners."""
from pathlib import Path
import yaml
from .motion_execution import MotionConfig
from .robot_profile import load_robot_profile, safety_scan_parameters

PREFIX = '/d1max/live_planning/'


def execution_config(session):
    e = session['robot_profile_snapshot']['engineering']
    result = dict(body_height=session['body_height'], max_speed=.30, max_yaw=.50,
                  single_floor_height_span=.25)
    for name in MotionConfig('validation', 'validation').acceptance_fields:
        result[name] = e.get(name, False)
    MotionConfig(session_id=session['id'], map_version_id=session['version_id'], **result)
    return result


def parameters(session, workspace):
    """All runtime topics explicit. Raw tracker Twist is never an SDK input."""
    m = session['motion']
    profile = session.get('robot_profile_snapshot') or load_robot_profile(
        Path(workspace)/'src/d1max_scan_planner/config/d1max_robot.yaml')
    if abs(profile['engineering']['body_reference_height_m'] - m['body_height']) > 1e-9:
        raise ValueError('Motion and collision profile body height disagree')
    nodes = {
        'trajectory_tracker': {
            'session_id': session['id'], 'planning_frame': session['frame_id'],
            'base_frame': 'd1max_loc_base_link', 'max_speed': m['max_speed'],
            'max_yaw_rate': m['max_yaw'], 'max_acceleration': .35,
            'max_yaw_acceleration': .8, 'single_floor_max_height_change': m['single_floor_height_span'],
            'goal_tolerance': .20, 'goal_height_tolerance': .15,
            'task_timeout': .75, 'odom_timeout': .4,
            'trajectory_timeout': 2.,
            'task_topic': PREFIX+'execution_task',
            'trajectory_topic': PREFIX+'execution_bspline',
            'odom_topic': '/d1max/localization/odometry/global',
            'command_topic': PREFIX+'cmd_vel_tracker_diagnostic_only',
            'frozen_topic': PREFIX+'tracker_frozen',
            'status_topic': PREFIX+'tracker_status', 'stop_service': PREFIX+'tracker_stop',
        },
        'navigation_command_gate': {
            'mode': 'live', 'motion_enabled': True, 'require_execution_permit': True,
            'execution_permit_timeout': .35, 'execution_permit_topic': PREFIX+'execution_permit',
            'expected_session_id': session['id'], 'navigation_session_id': session['id'],
            'expected_version_id': session['version_id'], 'map_frame': session['frame_id'],
            'odom_frame': 'd1max_loc_odom', 'base_frame': 'd1max_loc_base_link',
            'max_forward': m['max_speed'], 'max_lateral': 0., 'max_yaw': m['max_yaw'],
            'max_planar_speed': m['max_speed'],
            'input_topic': PREFIX+'cmd_vel_collision_checked',
            'output_topic': PREFIX+'cmd_vel_safe', 'status_topic': PREFIX+'command_gate_status',
            'scan_topic': PREFIX+'safety_scan',
        },
        'navigation_cloud_to_scan': {
            'input_topic': '/d1max/localization/lio/deskewed',
            'output_topic': PREFIX+'safety_scan', 'target_frame': 'd1max_loc_base_link',
            **safety_scan_parameters(profile), 'range_min': .15,
            'range_max': 12., 'bins': 720, 'max_cloud_age': .5,
        },
    }
    collision = yaml.safe_load((Path(workspace)/'src/d1max_navigation/config/collision_monitor.yaml').read_text())
    p = collision['collision_monitor']['ros__parameters']
    p.update(cmd_vel_in_topic=PREFIX+'cmd_vel_admitted',
             cmd_vel_out_topic=PREFIX+'cmd_vel_collision_checked')
    p['scan']['topic'] = PREFIX+'safety_scan'
    p['StopFootprint']['polygon_pub_topic'] = PREFIX+'collision_stop_polygon'
    nodes['collision_monitor'] = p
    nodes['motion_lifecycle_manager'] = {
        'autostart': True, 'bond_timeout': 1., 'attempt_respawn_reconnection': False,
        'node_names': ['collision_monitor'],
    }
    return {PREFIX+name: {'ros__parameters': dict(p, use_sim_time=False)} for name, p in nodes.items()}
