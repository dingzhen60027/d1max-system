"""Offline launch/config/seed contract tests. Never starts ROS or an SDK."""
from copy import deepcopy
import ast
import fcntl
import json
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from d1max_pct_planner.paths import expand_tree

from d1max_pct_scan import live_session
from d1max_pct_scan.live_ui_contract import (initial_pose_command, initial_pose_feedback,
                                            checked_session_status, source_status_fresh)


@pytest.fixture
def config():
    return expand_tree(yaml.safe_load(live_session.DEFAULT_CONFIG.read_text()))


def test_current_map_is_bound_to_original_coordinates():
    cfg = live_session.load_config(live_session.DEFAULT_CONFIG)
    assert cfg['frame_id'] == 'd1max_loc_map'
    assert 'crossfloor_complete' not in cfg['map_pcd']


def test_extended_perception_budget_propagates_without_relaxing_pose(tmp_path, config):
    # Exercise the timing gate independently of the default preview mask.
    config.pop('preview_ray_exclusion', None)
    config.update(perception_timeout_s=.75, perception_backend='per_sensor_rays',
                  scan_collision_policy='official')
    path = tmp_path / 'preview.yaml'
    path.write_text(yaml.safe_dump(config))
    cfg = live_session.load_config(path)
    scan = live_session.scan_parameters(cfg, 'test')
    assert scan['grid_map.preview_only'] is True
    assert scan['grid_map.cloud_pose_max_age'] == .75
    assert scan['grid_map.maximum_cloud_pose_dt'] == .25
    assert scan['fsm.odom_timeout'] == .5
    cfg.update({key: 'test' for key in ('ground_support_index', 'ground_support_sha256',
        'ground_support_source_pcd_sha256', 'ground_support_tomogram_sha256')})
    bridge = live_session.bridge_parameters(cfg, 'test')
    assert bridge['perception_timeout'] == .75
    assert 'input_timeout' not in bridge and 'pose_timeout' not in bridge
    with pytest.raises(ValueError, match='preview-only'):
        live_session.prepare_motion(path)


@pytest.mark.parametrize('override', [
    {'mode': 'LIVE_NAVIGATION', 'motion_control_enabled': True},
    {'motion_control_enabled': True}, {'perception_backend': 'deskewed_cloud'},
    {'scan_collision_policy': 'observed_free'}, {'perception_timeout_s': .750001},
    {'perception_timeout_s': float('nan')}, {'perception_timeout_s': True},
])
def test_perception_budget_rejects_non_preview_or_unbounded_override(config, override):
    config.update(perception_timeout_s=.75, perception_backend='per_sensor_rays',
                  scan_collision_policy='official')
    config.update(override)
    with pytest.raises(ValueError):
        live_session.perception_budget(config)


def test_default_perception_budget_remains_half_second(config):
    config.pop('perception_timeout_s', None)
    assert live_session.perception_budget(config) == .5
    params = live_session.scan_parameters(config, 'test')
    assert params['grid_map.cloud_pose_max_age'] == .5
    assert params['grid_map.preview_only'] == params['manager.preview_body_heading_contract']


def test_operator_preview_profile_uses_bounded_three_quarter_second_budget(config):
    assert live_session.perception_budget(config) == .75
    assert config['preview_ray_exclusion']['scope'] == 'near_body_preview'
    params = live_session.scan_parameters(config, 'test')
    assert params['grid_map.cloud_pose_max_age'] == .75
    assert params['grid_map.preview_only'] is True


@pytest.mark.parametrize('override,enabled', [
    ({}, True),
    ({'mode': 'LIVE_NAVIGATION', 'motion_control_enabled': True}, False),
    ({'motion_control_enabled': True}, False),
    ({'perception_backend': 'deskewed_cloud'}, False),
    ({'scan_collision_policy': 'observed_free'}, False),
])
def test_heading_contract_never_leaks_to_motion_or_other_backends(config, override, enabled):
    config.update(mode='LIVE_VISUALIZATION_NO_MOTION', motion_control_enabled=False,
                  perception_timeout_s=.5, perception_backend='per_sensor_rays',
                  scan_collision_policy='official')
    config.update(override)
    params=live_session.scan_parameters(config, 'test')
    assert params['manager.preview_body_heading_contract'] is enabled
    assert params['grid_map.preview_only'] is enabled
    assert params['manager.preview_direction_min_speed'] == .02
    assert params['grid_map.cloud_pose_max_age'] == .5


@pytest.mark.parametrize('key,value', [
    ('motion_control_enabled', True), ('mode', 'live_control'),
    ('frame_id', 'd1max_multifloor_planning'), ('current_floor', 'auto'),
    ('goal_floor', 'floor3'), ('scan_preview_speed_mps', 1.5),
    ('body_height_min_m', .8), ('result_timeout_s', float('nan')),
])
def test_unsafe_or_ambiguous_config_rejected(tmp_path, config, key, value):
    config[key] = value
    if key == 'body_height_min_m':
        config['body_height_max_m'] = .7
    file = tmp_path / 'config.yaml'
    file.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        live_session.load_config(file)


def test_prepare_snapshots_without_any_process_or_connection(tmp_path):
    with patch.object(live_session, 'ROOT', tmp_path), \
         patch.object(live_session.subprocess, 'Popen') as popen, \
         patch.object(live_session.subprocess, 'run') as run, \
         patch.object(live_session, 'build_opener') as network:
        directory, session = live_session.prepare()
        popen.assert_not_called()
        run.assert_not_called()
        network.assert_not_called()
    assert session['motion_control_enabled'] is False
    assert set(p.name for p in directory.iterdir()) == {
        'session.json', 'localization.yaml', 'global.yaml', 'bridge.yaml', 'scan.yaml', 'live.rviz',
        'pct_ground_support.npz', 'global_planning.rviz', 'local_planning.rviz',
        'navigation.xml', 'bt.yaml', 'bt_adapter.yaml', 'bt_lifecycle.yaml'}
    scan = yaml.safe_load((directory / 'scan.yaml').read_text())['/**']['ros__parameters']
    bridge = yaml.safe_load((directory / 'bridge.yaml').read_text())['/**']['ros__parameters']
    global_ = yaml.safe_load((directory / 'global.yaml').read_text())['/**']['ros__parameters']
    assert scan['fsm.navigation_session_id'] == bridge['session_id'] == global_['session_id'] == session['id']
    assert global_['retain_preview_task_on_soft_loss'] == live_session.official_ray_preview(session)
    assert scan['fsm.require_tagged_reference'] is True
    assert scan['fsm.max_replan_interval'] == 1.
    assert scan['fsm.planning_horizon'] == scan['manager.planning_horizon'] == 2.
    assert scan['manager.max_vel'] == .3
    assert session['scan_collision_policy'] == bridge['collision_policy'] == 'official'
    assert scan['grid_map.require_observed_free'] is False
    assert bridge['ground_support_sha256'] == session['ground_support_sha256']
    assert Path(bridge['ground_support_index']).is_file()


def test_two_view_files_share_session_and_do_not_spawn_processes(tmp_path):
    with patch.object(live_session.subprocess, 'Popen') as popen, \
         patch.object(live_session.subprocess, 'run') as run:
        live_session.write_view_configs(tmp_path, 'layout-audit')
        popen.assert_not_called()
        run.assert_not_called()
    global_view = yaml.safe_load((tmp_path/'global_planning.rviz').read_text())
    local_view = yaml.safe_load((tmp_path/'local_planning.rviz').read_text())
    assert (tmp_path/'live.rviz').read_text() == (tmp_path/'global_planning.rviz').read_text()
    for layout, config in (('global', global_view), ('local', local_view)):
        assert config['Panels'][0]['Layout'] == layout
        assert config['Panels'][0]['Session ID'] == 'layout-audit'
        assert config['Visualization Manager']['Global Options']['Fixed Frame'] == 'd1max_loc_map'
    assert global_view['Visualization Manager']['Views']['Current'] != local_view['Visualization Manager']['Views']['Current']


def test_export_views_only_updates_owned_preview_files(tmp_path):
    directory = tmp_path/'session'
    directory.mkdir(mode=0o700)
    session = dict(id='a'*32, mode='LIVE_VISUALIZATION_NO_MOTION',
                   motion_control_enabled=False, ui_reload_supported=1,
                   preview_freeze_owner='supervisor_child')
    live_session.write_owned_json(directory/'session.json', session)
    live_session.write_owned_json(tmp_path/'last_session.json', {'directory': str(directory)})
    original = (directory/'session.json').read_bytes()
    with patch.object(live_session, 'ROOT', tmp_path), \
         patch.object(live_session.subprocess, 'Popen') as popen, \
         patch.object(live_session.subprocess, 'run') as run, \
         patch.object(live_session, 'build_opener') as network:
        result = live_session.export_views()
        popen.assert_not_called()
        run.assert_not_called()
        network.assert_not_called()
        assert result['started_processes'] is False
        assert set(result['layouts']) == {'global', 'local'}
        assert all(Path(p).is_file() for p in result['layouts'].values())
        assert (directory/'session.json').read_bytes() == original
        session['motion_control_enabled'] = True
        live_session.write_owned_json(directory/'session.json', session)
        with pytest.raises(ValueError):
            live_session.export_views()


def test_start_obeys_shared_web_localization_lease(tmp_path):
    lease = tmp_path / '.localization-start.lock'
    with lease.open('a') as held:
        fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with patch.object(live_session, 'START_LOCK', lease), \
             patch.object(live_session, '_start_locked') as launch:
            with pytest.raises(RuntimeError, match='Another localization entry'):
                live_session.start()
            launch.assert_not_called()
    with patch.object(live_session, 'START_LOCK', lease), \
         patch.object(live_session, '_start_locked') as launch:
        live_session.start()
        launch.assert_called_once_with(live_session.DEFAULT_CONFIG)


def test_per_sensor_rays_is_one_explicit_preview_branch(tmp_path, config):
    config.pop('preview_ray_exclusion', None)
    config['perception_timeout_s'] = .5
    original = Path(config['localization_config']).read_text()
    config['perception_backend'] = 'per_sensor_rays'
    path = tmp_path / 'input.yaml'
    path.write_text(yaml.safe_dump(config))
    with patch.object(live_session, 'ROOT', tmp_path / 'sessions'), \
         patch.object(live_session.subprocess, 'Popen') as spawn:
        directory, session = live_session.prepare(path)
        spawn.assert_not_called()
    localization = yaml.safe_load((directory/'localization.yaml').read_text())
    assert localization['dual_lidar_adapter']['ros__parameters']['perception_rays.enabled'] is True
    assert Path(config['localization_config']).read_text() == original
    assert live_session.scan_parameters(session, session['id'])['grid_map.use_projected_rays'] is True
    assert live_session.bridge_parameters(session, session['id'])['perception_backend'] == 'per_sensor_rays'
    with pytest.raises(ValueError, match='preview-only'):
        live_session.prepare_motion(path)


def test_internal_run_cannot_bypass_owned_systemd_entry(tmp_path):
    with patch.object(live_session, 'unit', return_value={'MainPID': '0'}), \
         patch.object(live_session.subprocess, 'Popen') as launch:
        with pytest.raises(RuntimeError, match='internal owned-systemd'):
            live_session.run(tmp_path)
        launch.assert_not_called()


def test_rviz_topics_initial_and_goal_are_distinct():
    configuration = live_session.view_config('rviz-test-session')
    manager = configuration['Visualization Manager']
    tools = {t['Class']: t for t in manager['Tools']}
    assert tools['rviz_default_plugins/SetInitialPose']['Topic']['Value'].endswith('/initialpose')
    assert tools['rviz_default_plugins/SetGoal']['Topic']['Value'].endswith('/goal')
    assert tools['d1max_pct_rviz_tools/LiveGoal3D']['Session ID'] == 'rviz-test-session'
    displays = [child for group in manager['Displays'] for child in group['Displays']]
    assert [group['Name'] for group in manager['Displays']] == ['定位', '地图', '全局规划', '局部规划', '调试']
    topics = [d.get('Topic', {}).get('Value') for d in displays]
    assert '/d1max/live_planning/global_path_visual' in topics
    assert '/d1max/live_planning/reference_path' not in topics
    assert '/d1max/live_planning/scan_optimal' in topics
    assert '/d1max/live_planning/local_debug' in topics
    assert '/d1max/live_planning/local_attempt_debug' in topics
    assert '/d1max/live_planning/body_marker' in topics
    assert '/d1max/live_planning/traversable_surface' in topics
    assert '/d1max/live_planning/blocked_surface' in topics
    assert '/d1max/live_planning/view_status' not in topics  # No text wall over the map.
    assert manager['Global Options']['Fixed Frame'] == 'd1max_loc_map'
    assert not any('cmd_vel' in str(t) for t in topics)
    assert any(v['Class'] == 'rviz_default_plugins/Orbit' for v in manager['Views']['Saved'])
    names = [d['Name'] for d in displays]
    assert '全局路径' in names and '局部轨迹' in names
    assert '机器狗坐标系' in names
    assert not any('PCT' in name or 'SCAN' in name for name in names)
    assert configuration['Panels'][0]['Session ID'] == 'rviz-test-session'
    assert [v['Name'] for v in manager['Views']['Saved']] == ['全局总览', '局部跟随', '跨层 3D', '导航视角']
    for d in displays:
        if d['Class'] == 'rviz_default_plugins/Odometry':
            assert not d['Enabled']  # No stale Keep=1 robot arrow.
        if d.get('Topic', {}).get('Value') == '/d1max/localization/map_cloud':
            assert d['Color Transformer'] == 'AxisColor' and d['Axis'] == 'Z'
        if d.get('Topic', {}).get('Value') == '/d1max/localization/lio/deskewed':
            assert 0 < d['Decay Time'] <= .5


def test_retained_global_display_has_no_reference_authority_source_contract():
    # Inspect source without importing a ROS node or launching a process.
    source = Path(live_session.__file__).with_name('live_global_planner.py').read_text()
    tree = ast.parse(source)
    methods = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    def calls(name):
        return {ast.unparse(node.func) for node in ast.walk(methods[name]) if isinstance(node, ast.Call)}
    # Only explicitly configured previews retain owner intent. The default
    # branch still revokes; callback tests verify each branch and admission.
    assert 'self.publish_empty' in calls('pause_reference')
    assert 'self.clear_visual_path' not in calls('pause_reference')
    assert 'self.visual_path_pub.publish' not in calls('pause_reference')
    assert 'self.path_pub.publish' not in calls('pause_reference')
    assert 'self.clear_visual_path' in calls('revoke')
    assert 'self.publish_empty' in calls('revoke')
    commit = calls('_commit_cached_result')
    assert {'self.confirmed_identity', 'self.planning_context', 'self._start_moved',
            'self.path_pub.publish', 'self.visual_path_pub.publish'} <= commit


def test_axes_marker_retains_measurement_stamp_and_normalizes_duration():
    source = Path(live_session.__file__).with_name('live_view.py').read_text()
    tree = ast.parse(source)
    method = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef) and node.name == 'body_axes_marker')
    assignments = {ast.unparse(target): ast.unparse(node.value)
                   for node in ast.walk(method) if isinstance(node, ast.Assign)
                   for target in node.targets}
    assert assignments['marker.header'] == 'body.header'
    assert assignments['marker.pose'] == 'body.pose.pose'
    assert assignments['marker.lifetime'].startswith('Duration(')
    assert assignments['marker.lifetime'].endswith('.to_msg()')
    assert 'Marker.LINE_LIST' in ast.unparse(method)
    assert 'Marker.ARROW' not in ast.unparse(method)
    assert '上次位置' not in ast.unparse(method)
    assert not any(isinstance(node, ast.Call) and
                   ast.unparse(node.func).endswith(('odom_pub.publish', 'sendTransform'))
                   for node in ast.walk(method))


def test_seed_feedback_reports_rejection_not_an_old_acceptance():
    state = {'command_result': {'id': 'old', 'accepted': True, 'message': 'old'}}
    assert initial_pose_feedback(state, 'new') is None
    state['command_result'] = {'id': 'new', 'accepted': False, 'message': '轮次已变化'}
    result = initial_pose_feedback(state, 'new')
    assert '拒绝' in result and '轮次已变化' in result
    state['command_result']['accepted'] = True
    assert '仍需匹配确认' in initial_pose_feedback(state, 'new')


@pytest.mark.parametrize('stamp', [99., 101., float('nan'), float('inf'), True, '100'])
def test_status_source_timestamp_not_receipt_is_required(stamp):
    raw = json.dumps({'session_id': 's', 'wall_time': stamp})
    assert checked_session_status(raw, session_id='s', stamp_field='wall_time',
                                  now=100., timeout=.6) is None


def test_status_replay_and_bad_json_cannot_refresh_ui():
    raw = json.dumps({'session_id': 's', 'wall_time': 100.})
    arguments = dict(session_id='s', stamp_field='wall_time', now=100.1, timeout=.6)
    assert checked_session_status(raw, **arguments)['wall_time'] == 100.
    assert checked_session_status(raw, previous_stamp=100., **arguments) is None
    assert checked_session_status(raw.replace('"s"', '"old"'), **arguments) is None
    assert checked_session_status(raw, maximum_bytes=10, **arguments) is None
    assert checked_session_status('[]', **arguments) is None
    assert checked_session_status('{', **arguments) is None
    assert not source_status_fresh({'wall_time': 100.}, stamp_field='wall_time', now=101., timeout=.6)


def seed(**changes):
    args = dict(frame='d1max_loc_map', stamp=100., now=100., xy=[1., 2.],
                quaternion=[0., 0., 0., 1.], body_z=-.05,
                state={'session_id': 'test', 'initial_pose_ready': True, 'wall_time': 100.},
                session_id='test', state_age=.1, command_id='request1')
    args.update(changes)
    return initial_pose_command(**args)


def test_seed_is_only_body_reference_and_never_localization_success():
    value = seed()
    assert value['reference'] == 'body'
    assert value['z'] == -.05
    assert 'localized' not in value and 'navigation_ready' not in value


@pytest.mark.parametrize('change', [
    {'frame': 'map'}, {'stamp': 90.}, {'stamp': float('nan')}, {'state_age': 2.},
    {'quaternion': [0., 0., 0., 0.]}, {'xy': [float('inf'), 0.]},
    {'state': {'session_id': 'test', 'initial_pose_ready': False, 'wall_time': 100.}},
    {'state': {'session_id': 'other', 'initial_pose_ready': True, 'wall_time': 100.}},
    {'state': {'session_id': 'test', 'initial_pose_ready': True, 'wall_time': 99.}},
])
def test_seed_rejects_wrong_session_stale_and_unready(change):
    with pytest.raises(ValueError):
        seed(**change)
