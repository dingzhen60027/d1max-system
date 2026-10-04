from copy import deepcopy
import pytest
from d1max_pct_scan.live_diagnostics import diagnostics, map_identity, body_identity, body_visual_state


def states():
    loc = {'session_id': 's', 'wall_time': 99.9,
           'localized': True, 'map_localized': True, 'navigation_ready': False, 'local_epoch': 2,
           'active_seed_ns': 'seed', 'confirmed_seed_ns': 'seed', 'verified_confirmations': 3,
           'navigation': {'valid': True, 'navigation_ready': False, 'epoch': 2,
                          'seed_id': 'seed', 'received_at_unix': 99.9,
                          'target_hz': 50., 'global_observed_hz': 30.4,
                          'calibration': {'extrinsics_verified': False,
                                          'time_alignment_verified': False}}}
    glob = {'session_id': 's', 'active_reference': True, 'navigation_ready': False,
            'last_route': {'path_stamp': 90.}, 'generation': 23,
            'localization_epoch': 2, 'localization_seed_id': 'seed'}
    local = {'session_id': 's', 'ready': True, 'active_reference': True, 'local_debug_valid': True,
             'spline_visual_valid': True, 'last_spline_id': 4,
             'last_spline_stamp': 99.7, 'reference_stamp': 90., 'generation': 89,
             'local_debug_target': [1., 2., 3.], 'local_debug_plan_id': 4,
             'local_debug_progress_arc_m': 2., 'local_debug_target_arc_m': 8.,
             'localization_epoch': 2, 'localization_seed_id': 'seed',
             'body_age': .02, 'cloud_age': .11}
    return loc, glob, local


def sample(**local_changes):
    loc, glob, local = states()
    local.update(local_changes)
    return diagnostics(loc, glob, local, session_id='s', now=100.)


def test_cached_coast_metadata_cannot_hide_new_output_pause():
    loc, glob, local = states()
    loc.update(localized=False, state='output_waiting')
    loc['navigation'].update(valid=False, quality='paused',
        prediction={'prediction_mode': 'coasting', 'reason': 'predicting_coast', 'degraded': True})
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['output_quality'] == 'output_waiting'
    assert result['preview_ready'] is False
    assert result['stages']['localization']['tone'] == 'warning'


def test_confirmed_continuous_pose_does_not_disappear_when_matcher_waits():
    loc, glob, local = states()
    loc.update(localized=False, map_localized=False, continuous_pose_valid=True,
               state='tracking_degraded')
    loc['navigation']['valid'] = False
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['preview_ready'] is True
    assert result['output_quality'] == 'tracking'
    assert result['stages']['localization']['label'] == '已定位'
    assert result['stages']['localization']['tone'] == 'ready'
    assert result['stages']['localization']['detail'] == ''
    assert result['motion_enabled'] is False
    loc['continuous_pose_valid'] = False
    assert diagnostics(loc, glob, local, session_id='s', now=100.)['preview_ready'] is False


def test_paused_goal_is_visible_but_never_shown_as_an_active_route():
    loc, glob, local = states()
    glob.update(active_reference=False, goal_retained=True, paused_for_recovery=True)
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['global']['label'] == '等待定位恢复'
    assert result['stages']['global']['tone'] == 'warning'
    assert result['stages']['local']['tone'] != 'ready'


def test_committed_global_output_does_not_borrow_local_following_permission():
    loc, glob, local = states()
    glob.update(route_committed=True, visual_path_available=True, goal_retained=True,
        planning_phase='active', active_goal=dict(session_id='s', epoch=2, seed_id='seed'))
    baseline = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert baseline['stages']['global']['label'] == '路径已生成'
    loc.update(localized=False, continuous_pose_valid=False)
    loc['navigation']['valid'] = False
    glob.update(active_reference=False, paused_for_recovery=True,
                state='route_retained_following_paused')
    local.update(ready=False, spline_visual_valid=False)
    paused = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert paused['stages']['global'] == baseline['stages']['global']
    assert paused['stages']['local']['tone'] != 'ready'
    assert paused['motion_enabled'] is False and paused['plan_id'] is None
    for source, changes in ((loc, dict(reset_pending=True)),
                            (loc, dict(active_seed_ns='new')),
                            (glob, dict(localization_epoch=3))):
        original = deepcopy(source)
        source.update(changes)
        result = diagnostics(loc, glob, local, session_id='s', now=100.)
        assert result['stages']['global']['label'] != '路径已生成'
        source.clear(); source.update(original)


def retained_goal_state(reason='current_floor_requires_unique_measured_ground_support'):
    loc, glob, local = states()
    glob.update(state='goal_retained_waiting_recovery',
        planning_phase='goal_retained_waiting_recovery', active_reference=False,
        goal_retained=True, paused_for_recovery=False, last_route={},
        recovery_hold=dict(reason=reason, user_stamp=90.),
        active_goal=dict(session_id='s', epoch=2, seed_id='seed', user_stamp=90.))
    return loc, glob, local


@pytest.mark.parametrize('reason,global_label,local_label', [
    ('current_floor_requires_unique_measured_ground_support',
     '目标保留 · 等待可用起点', '等待可用起点'),
    ('snapshot_replan_limit_exceeded', '规划未完成', '等待重新规划'),
    ('total_goal_computation_deadline_expired', '规划未完成', '等待重新规划'),
    ('global_result_commit_wait_expired', '规划未完成', '等待重新规划'),
])
def test_retained_mission_labels_are_not_missing_goal_or_executable_path(reason, global_label, local_label):
    loc, glob, local = retained_goal_state(reason)
    # Cross-stream arrival may leave an older valid local curve in the view.
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['global']['label'] == global_label
    assert result['stages']['local']['label'] == local_label
    assert result['stages']['global']['tone'] == result['stages']['local']['tone'] == 'warning'
    assert result['plan_id'] is None and result['local_target'] is None
    assert result['reference_horizon_m'] is None and result['motion_enabled'] is False


def test_hold_remains_visible_during_soft_pose_gap_without_hiding_localization_wait():
    loc, glob, local = retained_goal_state()
    loc.update(localized=False, continuous_pose_valid=False, state='output_waiting')
    loc['navigation']['valid'] = False
    local.update(active_reference=False, ready=False)
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['global']['label'] == '目标保留 · 等待可用起点'
    assert result['stages']['local']['label'] == '等待可用起点'
    assert result['stages']['localization']['tone'] != 'ready'


@pytest.mark.parametrize('scope,change', [
    ('global', dict(session_id='other')), ('global', dict(goal_retained=False)),
    ('global', dict(active_reference=True)), ('global', dict(recovery_hold=None)),
    ('global', dict(active_goal=None)),
    ('goal', dict(user_stamp=89.)), ('goal', dict(session_id='other')),
    ('localization', dict(local_epoch=3)), ('localization', dict(active_seed_ns='new')),
    ('localization', dict(local_fault='imu_fault')),
])
def test_malformed_or_old_identity_hold_cannot_claim_retained_current_mission(scope, change):
    loc, glob, local = retained_goal_state()
    {'global': glob, 'goal': glob['active_goal'], 'localization': loc}[scope].update(change)
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['global']['label'] != '目标保留 · 等待可用起点'
    assert result['stages']['local']['label'] != '等待可用起点'


@pytest.mark.parametrize('phase,label', [
    ('initializing_map', '加载规划地图'),
    ('native_search_and_validation', '搜索与路径检查'),
    ('validating_source_frame', '核对路径坐标'),
    ('unknown', '计算中'),
])
def test_global_progress_distinguishes_loading_search_and_validation(phase, label):
    loc, glob, local = states()
    glob.update(active_reference=False, planning=True, native_elapsed_sec=2.,
                worker_metrics=dict(phase=phase))
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['global']['label'] == label + ' · 2s'
    assert result['stages']['global']['tone'] != 'ready'
    assert result['motion_enabled'] is False


def test_retained_visual_path_is_explicit_history_not_execution_reference():
    loc, glob, local = states()
    glob.update(active_reference=False, goal_retained=True, paused_for_recovery=True,
                visual_path_available=True, visual_path_execution_authorized=False,
                planning_phase='awaiting_navigation')
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['global']['label'] == '路线保留 · 等待定位'
    assert result['stages']['global']['tone'] == 'warning'
    assert result['stages']['local']['tone'] != 'ready'
    assert result['local_target'] is None and result['motion_enabled'] is False


def test_fresh_paired_plan_and_real_target_are_visible():
    value = sample()
    assert value['stages']['local']['tone'] == 'ready'
    assert value['local_target'] == [1., 2., 3.]
    assert value['reference_horizon_m'] == 6.
    assert value['motion_enabled'] is False


def revalidated_preview():
    loc, glob, local = states()
    local.update(execution_mode='preview', collision_policy='official',
        perception_backend='per_sensor_rays', last_spline_stamp=91.,
        trajectory_revalidation=dict(enabled=True, valid=True,
            proof_kind='revalidated', original_spline_stamp=91.,
            checked_at=99.9, plan_id=4))
    return loc, glob, local


def test_native_revalidated_preview_keeps_original_creation_time():
    loc, glob, local = revalidated_preview()
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['stages']['local']['label'] == '轨迹有效'
    assert local['last_spline_stamp'] == 91.
    assert value['motion_enabled'] is False


@pytest.mark.parametrize('changes', [dict(valid=False), dict(checked_at=97.),
    dict(checked_at=100.02), dict(plan_id=5), dict(original_spline_stamp=90.),
    dict(proof_kind='heartbeat')])
def test_stale_mismatched_or_missing_native_proof_cannot_validate_old_curve(changes):
    loc, glob, local = revalidated_preview()
    local['trajectory_revalidation'].update(changes)
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['stages']['local']['tone'] != 'ready'


@pytest.mark.parametrize('changes', [dict(execution_mode='execution'),
    dict(collision_policy='observed_free'), dict(perception_backend='deskewed_cloud'),
    dict(ready=False), dict(spline_visual_valid=False), dict(local_debug_valid=False)])
def test_preview_recheck_does_not_relax_other_modes_or_input_admission(changes):
    loc, glob, local = revalidated_preview()
    local.update(changes)
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['stages']['local']['tone'] != 'ready'


def test_preview_ready_is_separate_from_unverified_navigation_acceptance():
    loc, glob, local = states()
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['preview_ready'] is True
    assert value['stages']['localization']['label'] == '已定位'
    assert value['stages']['global']['tone'] == 'ready'
    acceptance = value['navigation_admission']
    assert acceptance['ready'] is False
    assert acceptance['minimum_hz'] == 40. and acceptance['maximum_hz'] == 60.
    assert acceptance['blockers'] == ['外参未完成实机验收', '传感器时间对齐未完成实机验收',
                                       '输出频率 30.4 Hz，要求 40–60 Hz']
    assert loc['navigation_ready'] is False
    assert loc['navigation']['calibration'] == {'extrinsics_verified': False,
                                               'time_alignment_verified': False}


def test_verified_navigation_is_only_a_read_only_report_never_motion_authorization():
    loc, glob, local = states()
    loc['navigation'].update(navigation_ready=True, global_observed_hz=50.,
        calibration={'extrinsics_verified': True, 'time_alignment_verified': True})
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['navigation_admission']['ready'] is True
    assert value['motion_enabled'] is False


def test_preview_still_requires_confirmations_same_session_epoch_seed_and_fresh_navigation():
    for key, changed in [('session_id', 'other'), ('wall_time', 90.),
                         ('verified_confirmations', 2), ('verified_confirmations', True),
                         ('active_seed_ns', 'new'), ('local_epoch', 3), ('local_fault', 'imu_gap')]:
        loc, glob, local = states()
        loc[key] = changed
        value = diagnostics(loc, glob, local, session_id='s', now=100.)
        assert value['preview_ready'] is False
        assert all(stage['tone'] != 'ready' for stage in value['stages'].values())
    for key, changed in [('epoch', 1), ('seed_id', 'old'), ('valid', False),
                         ('fault', 'reset'), ('received_at_unix', 99.)]:
        loc, glob, local = states()
        loc['navigation'][key] = changed
        value = diagnostics(loc, glob, local, session_id='s', now=100.)
        assert value['preview_ready'] is False
        assert all(stage['tone'] != 'ready' for stage in value['stages'].values())


def test_unpaired_or_absent_native_geometry_never_looks_valid():
    for changed in ({'spline_visual_valid': False}, {'last_spline_id': 5},
                    {'last_spline_id': True}, {'local_debug_plan_id': True},
                    {'session_id': 'other'}):
        assert sample(**changed)['stages']['local']['tone'] != 'ready'


def test_independent_generation_counters_use_source_reference_stamp():
    assert sample()['stages']['local']['tone'] == 'ready'
    assert sample(reference_stamp=90.1)['stages']['local']['tone'] != 'ready'


def test_owner_republication_stamp_pairs_display_without_changing_native_issue_time():
    loc, glob, local = states()
    glob['last_route']['path_stamp'] = 95.
    local['owner_reference_stamp'] = 95.
    assert local['reference_stamp'] == 90.
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['local']['tone'] == 'ready'
    local['owner_reference_stamp'] = 94.
    assert diagnostics(loc, glob, local, session_id='s', now=100.)['local_target'] is None


def test_invalid_stale_or_unpaired_never_shows_old_target():
    for change in ({'local_debug_valid': False}, {'last_spline_stamp': 97.},
                   {'ready': False}, {'active_reference': False}):
        value = sample(**change)
        assert value['stages']['local']['tone'] != 'ready'
        assert value['local_target'] is None
        assert value['reference_horizon_m'] is None


def test_absent_states_and_bad_numbers_cannot_look_healthy():
    empty = diagnostics({}, {}, {}, session_id='s', now=100.)
    assert all(s['tone'] != 'ready' for s in empty['stages'].values())
    value = sample(body_age=float('nan'), cloud_age=True, local_debug_target=[float('inf'), 2, 3])
    assert value['body_age'] is None and value['cloud_age'] is None
    assert value['local_target'] is None


def test_native_failure_is_distinct_from_waiting_for_reference():
    value = sample(local_debug_valid=False, local_debug_phase='failed')
    assert value['stages']['local']['label'] == '局部规划失败'
    assert value['stages']['local']['tone'] == 'warning'
    assert value['local_target'] is None


@pytest.mark.parametrize('phase,label', [
    ('failed_reference_geometry', '参考路径无效'),
    ('failed_reference_search_budget', '绕行搜索达到上限'),
    ('failed_reference_target_occupied', '局部目标被占用'),
    ('failed_reference_start_occupied', '绕行起点受阻'),
    ('failed_reference_lattice_occupied', '搜索格端点受阻'),
    ('failed_reference_outside_map', '局部端点超出地图'),
    ('waiting_observed_space', '绕行空间尚未观测'),
    ('failed_reference_search_collision', '绕行连接段碰撞'),
    ('failed_optimization', '轨迹优化未收敛'),
    ('failed_final_collision', '轨迹碰撞校验未通过'),
    ('failed_current_validation', '当前轨迹需重新规划'),
    ('waiting_sensor_map', '等待局部地图'),
    ('waiting_goal_reached', '已接近局部目标'),
])
def test_real_native_failure_phase_is_not_guessed_or_shown_as_success(phase, label):
    value = sample(local_debug_valid=False, spline_visual_valid=False, local_debug_phase=phase)
    assert value['stages']['local']['label'] == label
    assert value['stages']['local']['tone'] == 'warning'
    assert value['local_target'] is None and value['plan_id'] is None


def test_retained_task_waits_for_data_not_another_user_reference():
    value=sample(ready=False, preview_reference_paused=True,
                 spline_visual_valid=False, local_debug_valid=False,
                 local_debug_phase='inactive')
    assert value['stages']['local']['label'] == '等待局部数据'
    assert value['stages']['local']['tone'] != 'ready'
    assert value['local_target'] is None and value['plan_id'] is None


def test_lio_fault_overrides_old_localized_flag():
    value = diagnostics({'localized': True, 'local_fault': 'imu_gap'}, {}, {}, session_id='s', now=100.)
    assert value['stages']['localization']['tone'] == 'error'
    assert 'imu_gap' in value['stages']['localization']['detail']


def test_new_localization_context_cannot_show_old_planners_green():
    stale = {'session_id': 's', 'active_reference': True, 'navigation_ready': True, 'ready': True,
             'last_route': {'path_stamp': 90.}, 'reference_stamp': 90.,
             'localization_epoch': 1, 'localization_seed_id': 'old',
             'local_debug_valid': True, 'last_spline_stamp': 99.9}
    loc, _, _ = states()
    for state in (loc, {}):
        value = diagnostics(state, stale, stale, session_id='s', now=100.)
        assert value['stages']['global']['tone'] != 'ready'
        assert value['stages']['local']['tone'] != 'ready'
        assert value['local_target'] is None
    assert sample(localization_epoch=1)['stages']['local']['tone'] != 'ready'


def test_bad_status_shapes_do_not_crash_status_rendering():
    for value in (None, 1, [], {}, True):
        result = diagnostics({}, {'state': value, 'last_route': value}, {}, session_id='s', now=100.)
        assert result['stages']['global']['tone'] != 'ready'


def test_submitted_current_seed_waits_for_confirmation_not_another_seed():
    loc = {'session_id': 's', 'wall_time': 99.9, 'localized': False,
           'initial_pose_ready': True, 'state': 'acquiring', 'local_epoch': 2,
           'active_seed_ns': '100000000000', 'confirmed_seed_ns': None,
           'verified_confirmations': 0, 'frontend_ready': True,
           'frontend': {'epoch': 2, 'ready': True}}
    def label(value):
        return diagnostics(value, {}, {}, session_id='s', now=100.)['stages']['localization']['label']
    assert label(loc) == '匹配确认中'
    assert label(dict(loc, state='relocalizing')) == '匹配确认中'
    for change in ({'session_id': 'old'}, {'wall_time': 90.}, {'local_epoch': 3},
                   {'verified_confirmations': True}, {'verified_confirmations': -1},
                   {'confirmed_seed_ns': 'old'}, {'frontend_ready': False},
                   {'active_seed_ns': 'malformed'}, {'state': 'waiting_sensors'}):
        assert label(dict(loc, **change)) != '匹配确认中'
    assert label(dict(loc, active_seed_ns=None, state='waiting_initial_pose', local_epoch=3)) == '请给初值'
    assert label(dict(loc, state='filter_initializing', verified_confirmations=3,
                      confirmed_seed_ns=loc['active_seed_ns'])) == '等待融合输出'


def test_short_current_imu_gap_is_degraded_but_localization_remains_valid():
    loc, glob, local = states()
    loc['frontend'] = {'epoch': 2, 'scan_imu_gap_sec': .07, 'imu_gap_warning_sec': .015}
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['localization_degraded'] is True and value['preview_ready'] is True
    assert value['stages']['localization'] == {'label': '已定位', 'tone': 'ready', 'detail': ''}
    assert '70.0 ms' in value['localization_quality_detail']
    assert value['stages']['local']['tone'] == 'ready'
    assert loc['active_seed_ns'] == 'seed'
    loc['frontend'].update(scan_imu_gap_sec=.01, max_observed_gap=.5, degraded=False)
    assert diagnostics(loc, glob, local, session_id='s', now=100.)['localization_degraded'] is False
    loc['navigation']['prediction'] = {'degraded': True, 'largest_imu_gap_sec': .05}
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['localization_degraded'] is True
    assert '50.0 ms' in value['localization_quality_detail']
    loc['navigation']['prediction']['imu_gap'] = {'count': 1, 'max_sec': .07,
        'max_rotation_rad': .01, 'integrated_sec': .07, 'rejected_reason': ''}
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert '70.0 ms' in value['localization_quality_detail']
    assert '50.0 ms' not in value['localization_quality_detail']
    loc['local_fault'] = 'imu_gap_hard_limit'
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['localization_degraded'] is False
    assert value['stages']['localization']['tone'] == 'error'


def test_confirmed_map_with_paused_output_is_not_filter_initialization_or_ready():
    loc, glob, local = states()
    loc.update(localized=False, map_localized=True, state='output_waiting')
    loc['navigation'].update(valid=False, state='waiting_local', prediction={'reason': 'imu_stale'})
    value = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert value['stages']['localization']['label'] == '已定位 · 输出暂缓'
    assert value['stages']['localization']['detail'] == '等待新 IMU'
    assert value['preview_ready'] is False
    assert all(stage['tone'] != 'ready' for stage in value['stages'].values())


def test_old_epoch_or_historical_gap_alone_does_not_claim_current_degradation():
    loc, glob, local = states()
    for front in ({'epoch': 1, 'degraded': True, 'scan_imu_gap_sec': .08},
                  {'epoch': 2, 'max_observed_gap': .7},
                  {'epoch': 2, 'scan_imu_gap_sec': float('nan')}):
        loc['frontend'] = front
        loc['navigation']['prediction'] = {'degraded': False, 'largest_imu_gap_sec': .08,
                                           'imu_gap': {'max_sec': .07, 'count': 1}}
        assert diagnostics(loc, glob, local, session_id='s', now=100.)['localization_degraded'] is False


def test_prediction_nested_gap_malformed_or_absent_uses_bounded_legacy_fallback():
    loc, glob, local = states()
    for nested in (None, [], {'max_sec': True}, {'max_sec': float('nan')}, {'max_sec': 8.}):
        loc['navigation']['prediction'] = {'degraded': True, 'imu_gap': nested,
                                           'largest_imu_gap_sec': .06}
        value = diagnostics(loc, glob, local, session_id='s', now=100.)
        assert '60.0 ms' in value['localization_quality_detail']


def test_map_identity_survives_output_waiting_but_never_relaxes_preview_admission():
    loc, glob, local = states()
    initial = map_identity(loc, session_id='s', now=100.)
    loc.update(localized=False, state='output_waiting')
    loc['navigation'].update(valid=False, state='waiting_local', prediction={'reason': 'imu_stale'})
    result = diagnostics(loc, glob, local, session_id='s', now=100., feedback='旧初值仍需匹配确认')
    assert map_identity(loc, session_id='s', now=100.) == initial == ('s', 2, 'seed')
    assert result['notice'] == ''
    assert result['map_identity_valid'] is True and result['output_quality'] == 'output_waiting'
    assert result['stages']['localization']['label'] == '已定位 · 输出暂缓'
    assert result['stages']['localization']['tone'] == 'warning'
    assert result['preview_ready'] is False and result['navigation_admission']['ready'] is False
    assert result['motion_enabled'] is False and result['local_target'] is None
    assert all(stage['tone'] != 'ready' for stage in result['stages'].values())


def test_valid_bounded_prediction_is_located_without_mutating_quality_or_admission():
    loc, glob, local = states()
    loc['navigation']['prediction'] = {'prediction_mode': 'coasting', 'reason': 'predicting_coast',
        'degraded': True, 'coast_stats': {'duration_sec': .03, 'unsupported_sec': .005}}
    result = diagnostics(loc, glob, local, session_id='s', now=100.)
    assert result['stages']['localization'] == {'label': '已定位', 'tone': 'ready', 'detail': ''}
    assert result['output_quality'] == 'predicting_coast' and result['map_identity_valid']
    assert loc['navigation']['prediction']['prediction_mode'] == 'coasting'
    assert result['localization_degraded'] is True
    assert result['navigation_admission']['ready'] is False  # Acceptance flags are unchanged.
    # Valid fresh native output is still valid; presentation is not a new gate.
    assert result['preview_ready'] is True and result['stages']['local']['tone'] == 'ready'
    loc['navigation']['valid'] = False
    assert diagnostics(loc, glob, local, session_id='s', now=100.)['preview_ready'] is False


def body_state(loc, **overrides):
    args = dict(session_id='s', now=100., body_stamp=99.8, body_receipt_age=.2,
                state_receipt_age=.05, body_context=('s', 2, 'seed'), body_barrier=99.)
    args.update(overrides)
    return body_visual_state(loc, **args)


def test_output_status_interleaving_preserves_only_original_half_second_pose_lease():
    loc, _, _ = states()
    tracked = body_state(loc)
    assert tracked['visible'] and tracked['quality'] == 'tracking'
    assert tracked['last_known'] is False
    assert tracked['lifetime_sec'] == pytest.approx(.3)
    loc.update(localized=False, state='output_waiting')
    loc['navigation'].update(valid=False, state='waiting_local')
    held = body_state(loc)
    assert held['visible'] and held['held_or_degraded'] and held['quality'] == 'output_waiting'
    assert held['last_known'] is False and held['color'] == tracked['color']
    assert held['source_stamp'] == tracked['source_stamp'] == 99.8
    assert held['lifetime_sec'] == pytest.approx(.3)
    loc['wall_time'] = 100.1  # Fresh status cannot refresh the old pose's timestamp.
    later = body_state(loc, now=100.1, body_receipt_age=.3)
    assert later['visible'] and later['source_stamp'] == 99.8
    assert later['lifetime_sec'] < held['lifetime_sec']
    expired = body_state(loc, now=100.31, body_receipt_age=.51)
    assert not expired['visible'] and not expired['last_known']
    assert expired['source_stamp'] is None and expired['lifetime_sec'] == 0.
    loc['wall_time'] = 101.7
    assert not body_state(loc, now=101.8, body_receipt_age=2.)['visible']
    # A newly measured odometry message may replace the old one even during a short wait.
    loc['wall_time'] = 101.79
    newer = body_state(loc, now=101.8, body_stamp=101.78, body_receipt_age=.02)
    assert newer['visible'] and newer['source_stamp'] == 101.78 and not newer['last_known']


def test_body_fault_identity_switch_or_stale_status_cannot_be_held():
    original, _, _ = states()
    for changed in ({'session_id': 'other'}, {'local_epoch': 3}, {'active_seed_ns': 'new'},
                    {'confirmed_seed_ns': 'new'}, {'local_fault': 'imu_gap'}, {'wall_time': 97.99}):
        state = deepcopy(original); state.update(changed)
        assert not body_state(state)['visible']
    state = deepcopy(original); state['navigation']['fault'] = 'filter_jump'
    assert not body_state(state)['visible']
    assert not body_state(original, state_receipt_age=2.)['visible']
    assert not body_state(original, body_receipt_age=2.)['visible']
    assert not body_state(original, body_context=('s', 1, 'old'))['visible']
    assert not body_state(original, body_barrier=99.9)['visible']
    assert not body_state(original, body_stamp=float('nan'))['visible']


def test_body_display_context_does_not_relax_existing_output_or_preview_admission():
    original, glob, local = states()
    for change in ({'map_localized': False},
                   {'localized': False, 'map_localized': False, 'continuous_pose_valid': False}):
        loc = deepcopy(original); loc.update(change)
        visual = body_state(loc)
        assert visual['visible'] and not visual['last_known']
        assert visual['source_stamp'] == 99.8 and visual['lifetime_sec'] == pytest.approx(.3)
        assert body_identity(loc, session_id='s', now=100.) == ('s', 2, 'seed')
        report = diagnostics(loc, glob, local, session_id='s', now=100.)
        assert report['preview_ready'] is False
        assert report['local_target'] is None and report['motion_enabled'] is False
    for change in ({'verified_confirmations': 2}, {'verified_confirmations': True},
                   {'wall_time': 99.}):
        loc = deepcopy(original); loc.update(change)
        assert not body_state(loc)['visible']
        assert body_identity(loc, session_id='s', now=100.) is None


def test_display_expiry_uses_each_original_clock_not_only_latest_status_receipt():
    loc, _, _ = states()
    assert not body_state(loc, body_stamp=99.5)['visible']
    assert not body_state(loc, body_stamp=100.11)['visible']
    assert not body_state(loc, state_receipt_age=.6)['visible']
    assert not body_state(loc, body_receipt_age=.5)['visible']
    loc['wall_time'] = 99.39
    assert not body_state(loc)['visible']


def test_valid_quality_variants_never_recolor_a_fresh_pose():
    original, _, _ = states()
    tracking = body_state(original)
    for prediction in ({'prediction_mode': 'coasting', 'degraded': True},
                       {'degraded': True, 'largest_imu_gap_sec': .07}):
        loc = deepcopy(original)
        loc['navigation']['prediction'] = prediction
        visual = body_state(loc)
        assert visual['visible'] and visual['color'] == tracking['color']
        assert visual['source_stamp'] == tracking['source_stamp']
        assert visual['lifetime_sec'] == tracking['lifetime_sec']
