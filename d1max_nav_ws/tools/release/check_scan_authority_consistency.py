#!/usr/bin/env python3
"""Passive source/install/default-policy check; never imports ROS or the SDK."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nav-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--app-root', type=Path, default=os.environ.get('D1MAX_APP_ROOT'))
    args = parser.parse_args()
    nav = args.nav_root.resolve(strict=True)
    selected = json.loads((nav/'deploy/single_floor_release.json').read_text())
    release = (nav/selected['release_directory']).resolve(strict=True)
    descriptor = json.loads((release/'release.json').read_text())
    manifest = release/descriptor['sealed_manifest']
    if digest(manifest) != selected['release_manifest_sha256']:
        raise ValueError('default_manifest_binding_mismatch')
    session = json.loads((release/descriptor['preflight_session']/'session.json').read_text())
    for key in ('scan_collision_policy', 'scan_planning_collision_policy', 'execution_collision_policy'):
        if session.get(key) != 'official':
            raise ValueError('formal_collision_policy_mismatch:'+key)
    if selected.get('physical_acceptance') is not False or session.get('physical_acceptance') is not False:
        raise ValueError('software_update_must_not_change_physical_acceptance')
    if session.get('execution_architecture')!='task_follower_v1' or session.get('handoff_contract')!='local_executor_v1':
        raise ValueError('default_must_have_one_local_executor_and_task_writer')
    pairs = []
    snapshot = release/'source_snapshot/nav_ws'
    for name in ('single_floor_session', 'execution_safety', 'execution_safety_node',
                 'execution_handoff', 'command_admission', 'continuous_reference', 'continuous_reference_node',
                 'continuous_reference_transport', 'atomic_projection_state', 'atomic_navigation_inbox',
                 'local_navigation_state', 'live_global_planner', 'bt_adapters',
                 'bt_follow_policy', 'formal_follow_status', 'navigation_runtime_policy',
                 'component_health', 'component_health_monitor', 'robot_profile',
                 'live_visualization', 'live_view', 'live_goal_selection',
                 'live_diagnostics', 'typed_view_state', 'committed_route_view',
                 'navigation_contract', 'control_frame_contract', 'tracking_profile', 'braking_model', 'execution_timing',
                 'interface_preflight', 'execution_view'):
        relative = Path('src/d1max_pct_scan/d1max_pct_scan')/(name+'.py')
        installed = release/'application/install/d1max_pct_scan/lib/python3.10/site-packages/d1max_pct_scan'/(name+'.py')
        pairs.append((nav/relative, snapshot/relative, installed))
    for name in ('navigation_wire', 'global_lio_localizer', 'startup_relocalization', 'lio_localizer', 'lio_fusion'):
        relative = Path('src/d1max_localization/d1max_localization')/(name+'.py')
        pairs.append((nav/relative, snapshot/relative,
            release/'application/install/d1max_localization/local/lib/python3.10/dist-packages/d1max_localization'/(name+'.py')))
    for name in ('localization.yaml', 'global_relocalization.yaml', 'realtime_navigation.yaml'):
        relative = Path('src/d1max_localization/config')/name
        pairs.append((nav/relative, snapshot/relative,
            release/'application/install/d1max_localization/share/d1max_localization/config'/name))
    relative = Path('src/d1max_localization/d1max_localization/estimation/causal_prediction.py')
    pairs.append((nav/relative, snapshot/relative,
        release/'application/install/d1max_localization/local/lib/python3.10/dist-packages/d1max_localization/estimation/causal_prediction.py'))
    relative = Path('src/d1max_pct_scan/config/motion_tracking.yaml')
    pairs.append((nav/relative, snapshot/relative,
                  release/'application/install/d1max_pct_scan/share/d1max_pct_scan/config/motion_tracking.yaml'))
    relative=Path('src/d1max_pct_scan/config/runtime_motion_profile.json')
    pairs.append((nav/relative,snapshot/relative,
        release/'application/install/d1max_pct_scan/share/d1max_pct_scan/config/runtime_motion_profile.json'))
    for name in ('navigation_diagnostics_panel', 'point_tools'):
        relative = Path('src/d1max_pct_rviz_tools/src')/(name+'.cpp')
        pairs.append((nav/relative, snapshot/relative))
        relative = Path('src/d1max_pct_rviz_tools/include/d1max_pct_rviz_tools')/(name+'.hpp')
        pairs.append((nav/relative, snapshot/relative,
            release/'rviz/install/d1max_pct_rviz_tools/include/d1max_pct_rviz_tools'/(name+'.hpp')))
    for package, component, names in (
        ('scan_planner_vendor/bspline_opt', 'native', ('bspline_opt/spline_heading_contract.hpp',
            'bspline_opt/near_curve_direction.hpp', 'bspline_opt/whole_spline_collision.hpp')),
        ('scan_planner_vendor/plan_env', 'native', ('plan_env/grid_map.h',)),
        ('scan_planner_vendor/plan_manage', 'native', ('plan_manage/execution_validator.hpp',
            'plan_manage/trajectory_collision.hpp', 'plan_manage/validation_cycle.hpp', 'plan_manage/motion_sweep.hpp',
            'plan_manage/motion_check_diagnostics.hpp',
            'plan_manage/scan_replan_fsm.h', 'plan_manage/reference_receipt_history.hpp',
            'plan_manage/reference_target.hpp', 'plan_manage/final_endpoint_reseed.hpp',
            'plan_manage/planner_manager.h',
            'plan_manage/forward_motion_boundary.hpp',
            'plan_manage/moving_splice.hpp', 'plan_manage/candidate_adoption.hpp')),
        ('d1max_trajectory_tracker', 'tracker', ('d1max_trajectory_tracker/execution_contract.hpp',
            'd1max_trajectory_tracker/tracker_core.hpp','d1max_trajectory_tracker/local_follower.hpp',
            'd1max_trajectory_tracker/velocity_smoother.hpp',
            'd1max_trajectory_tracker/forward_steering_controller.hpp',
            'd1max_trajectory_tracker/motion_command_conditioner.hpp',
            'd1max_trajectory_tracker/response_command_limiter.hpp')),
        ('d1max_navigation_bt', 'bt', ('d1max_navigation_bt/execution_owner.hpp',
            'd1max_navigation_bt/body_pair_admission.hpp', 'd1max_navigation_bt/runtime_policy.hpp',
            'd1max_navigation_bt/task_execution_authority.hpp',
            'd1max_navigation_bt/sdk_software_estop_fence.hpp')),
    ):
        installed_package = {'scan_planner_vendor/plan_manage': 'scan_planner',
                             'scan_planner_vendor/bspline_opt': 'bspline_opt',
                             'scan_planner_vendor/plan_env': 'plan_env'}.get(package, package)
        for name in names:
            relative = Path('src')/package/'include'/name
            pairs.append((nav/relative, snapshot/relative,
                release/component/'install'/installed_package/'include'/name))
    for relative in ('src/scan_planner_vendor/plan_manage/src/scan_replan_fsm.cpp',
                     'src/scan_planner_vendor/plan_manage/src/planner_manager.cpp',
                     'src/scan_planner_vendor/plan_manage/src/motion_sweep.cpp',
                     'src/d1max_trajectory_tracker/src/trajectory_tracker.cpp',
                     'src/d1max_trajectory_tracker/src/local_follower_node.cpp',
                     'src/d1max_navigation_bt/src/navigator_node.cpp'):
        pairs.append((nav/relative,snapshot/relative))
    for relative in ('src/faster_lio/src/laser_mapping.cc',
                     'src/faster_lio/src/robust_runtime.inc',
                     'src/faster_lio/include/registration_quality.hpp'):
        pairs.append((nav/relative,snapshot/relative))
    if args.app_root is not None:
        for name in ('execution_transport_core.hpp', 'execution_robot_health.hpp',
                     'execution_scheduler.hpp', 'sdk_write_transactions.hpp',
                     'mc_capture_time.hpp', 'mc_report_core.hpp',
                     'execution_kinematics.hpp', 'execution_acceptance.hpp', 'sdk_input_mapping.hpp',
                     'shared_estop_core.hpp', 'monitor_estop_core.hpp',
                     'measured_curve_entry.hpp',
                     'navigation_diagnostics_core.hpp',
                     'sdk_monitor_bridge.cpp', 'sdk_execution_mock.cpp', 'execution_acceptance_check.cpp'):
            relative = Path('sdk_bridge_ws/src/d1max_sdk_bridge/src')/name
            pairs.append((args.app_root.resolve(strict=True)/relative,
                          release/'source_snapshot'/relative))
    checked = []
    for sources in pairs:
        hashes = [digest(path) for path in sources]
        if len(set(hashes)) != 1:
            raise ValueError('source_snapshot_or_installed_mismatch:'+str(sources[0]))
        checked.append(dict(source=str(sources[0]), sha256=hashes[0],
                            copies=[str(path) for path in sources[1:]]))
    if session.get('controller_frequency_hz')!=20.:
        raise ValueError('default_controller_frequency_mismatch')
    installed_gate = (release/'application/install/d1max_pct_scan/lib/python3.10/site-packages/d1max_pct_scan/execution_safety_node.py').read_text()
    if 'ProjectedRays' in installed_gate:
        raise ValueError('duplicate_ray_subscription_still_installed')
    print(json.dumps(dict(schema=1, release=str(release),kinematic_profile=session.get('kinematic_profile','forward_yaw'),
        manifest_sha256=digest(manifest), collision_authority='native_scan',
        collision_policy='official', command_collision_authority='current_accepted_scan_curve',
        independent_motion_sweep_required=False, source_install_matches=checked,
        physical_acceptance=False, starts_services=False), indent=2))


if __name__ == '__main__':
    main()
