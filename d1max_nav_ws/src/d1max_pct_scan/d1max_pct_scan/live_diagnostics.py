"""Small read-only RViz status contract, with source freshness preserved."""
import math
from .live_ui_contract import (localization_preview_state, navigation_admission,
                               matching_confirmation_pending, localization_degradation,
                               source_status_fresh)


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def map_identity(localization, *, session_id, now):
    """Retain confirmed map identity independently of short output availability.

    This is display identity, never planning/control admission. Older status
    producers may omit map_localized, but an explicit false is authoritative.
    """
    if (not isinstance(localization, dict) or localization.get('session_id') != session_id
            or not source_status_fresh(localization, stamp_field='wall_time', now=now, timeout=.6)
            or (localization.get('continuous_pose_valid') is not True
                and localization.get('map_localized', localization.get('localized')) is not True)
            or localization.get('local_fault')):
        return None
    epoch, seed, count = (localization.get('local_epoch'), localization.get('confirmed_seed_ns'),
                          localization.get('verified_confirmations'))
    navigation = localization.get('navigation')
    if (type(epoch) is not int or epoch < 1 or not isinstance(seed, str) or not seed
            or localization.get('active_seed_ns') != seed or type(count) is not int or count < 3
            or isinstance(navigation, dict) and navigation.get('fault')):
        return None
    return session_id, epoch, seed


def body_identity(localization, *, session_id, now):
    """Confirmed display context, not output availability or motion admission.

    A fresh measured pose may precede the next 5 Hz availability snapshot.
    Short output waits do not change its epoch/seed or extend its source lease.
    """
    if (not isinstance(localization, dict) or localization.get('session_id') != session_id
            or not source_status_fresh(localization, stamp_field='wall_time', now=now, timeout=.6)
            or localization.get('local_fault')):
        return None
    epoch, seed, count = (localization.get('local_epoch'), localization.get('confirmed_seed_ns'),
                          localization.get('verified_confirmations'))
    navigation = localization.get('navigation')
    if (type(epoch) is not int or epoch < 1 or not isinstance(seed, str) or not seed
            or localization.get('active_seed_ns') != seed or type(count) is not int or count < 3
            or isinstance(navigation, dict) and navigation.get('fault')):
        return None
    return session_id, epoch, seed


def output_quality(localization, identity, *, now):
    if localization.get('local_fault'):
        return 'fault'
    navigation = localization.get('navigation')
    navigation = navigation if isinstance(navigation, dict) else {}
    if navigation.get('fault'):
        return 'fault'
    if identity is None:
        return 'unlocalized'
    if (not source_status_fresh(navigation, stamp_field='received_at_unix', now=now, timeout=.6)
            or navigation.get('epoch') != identity[1] or navigation.get('seed_id') != identity[2]):
        return 'output_waiting'
    continuous = localization.get('continuous_pose_valid', navigation.get('valid'))
    if continuous is not True:
        return 'output_waiting'  # cached prediction metadata cannot authorize output
    prediction = navigation.get('prediction')
    prediction = prediction if isinstance(prediction, dict) else {}
    if (navigation.get('quality') == 'coasting'
            or navigation.get('state') in ('coasting', 'predicting_coast')
            or prediction.get('prediction_mode') == 'coasting'
            or prediction.get('reason') == 'predicting_coast'):
        return 'predicting_coast'
    return 'tracking' if continuous is True else 'output_waiting'


def body_visual_state(localization, *, session_id, now, body_stamp, body_receipt_age,
                      state_receipt_age, body_context, body_barrier=0.):
    """Show only a fresh measured pose in its confirmed display context.

    Availability snapshots can interleave with odometry, so their quality must
    not hide a fresh sample. There is no historical-pose fallback and no motion
    authorization; all expiry clocks and the measurement stamp remain original.
    """
    identity = body_identity(localization, session_id=session_id, now=now)
    quality = output_quality(localization,
        map_identity(localization, session_id=session_id, now=now), now=now)
    finite = all(finite_number(v) for v in (
        now, body_stamp, body_receipt_age, state_receipt_age, body_barrier))
    visible = bool(finite and identity is not None and identity == body_context
        and body_stamp >= body_barrier and -.1 <= now-body_stamp < .5
        and 0 <= body_receipt_age < .5 and 0 <= state_receipt_age < .6)
    remaining = (min(.5-max(0., now-body_stamp), .5-body_receipt_age,
                     .6-state_receipt_age, .6-max(0., now-localization['wall_time']))
                 if visible else 0.)
    degraded = localization_degradation(localization)[0] if visible else False
    paused = visible and (quality != 'tracking' or degraded)
    return {'visible': visible, 'source_stamp': body_stamp if visible else None,
            'lifetime_sec': max(0., remaining), 'quality': quality,
            'last_known': False,
            'source_age_sec': max(0., now-body_stamp) if visible else None,
            # Compatibility metadata only; the pose marker uses fixed RGB axes.
            'color': (1., 1., 1., 1.),
            'held_or_degraded': paused}


def diagnostics(localization, global_state, local, *, session_id, now, feedback=None):
    # Inputs have already passed the same-session/source-age checks in LiveView.
    def stage(label, tone, reason=''):
        return {'label': label, 'tone': tone, 'detail': str(reason)[:500]}

    preview_ready, preview_reason = localization_preview_state(
        localization, session_id=session_id, now=now)
    degraded, degradation_detail = localization_degradation(localization)
    identity = map_identity(localization, session_id=session_id, now=now)
    quality = output_quality(localization, identity, now=now)
    preview_ready = preview_ready and identity is not None
    admission = navigation_admission(localization, preview_ready=preview_ready)
    degraded = identity is not None and (degraded or quality in ('predicting_coast', 'output_waiting'))
    loc = stage('等待数据', 'muted')
    if localization:
        if localization.get('local_fault'):
            loc = stage('里程计异常', 'error', localization['local_fault'])
        elif quality == 'fault':
            loc = stage('融合输出异常', 'error', localization['navigation'].get('fault', ''))
        elif preview_ready:
            loc = stage('已定位', 'ready')
        elif identity is not None and quality == 'output_waiting':
            nav = localization.get('navigation')
            nav = nav if isinstance(nav, dict) else {}
            prediction = nav.get('prediction')
            prediction = prediction if isinstance(prediction, dict) else {}
            reason = prediction.get('reason') if nav.get('state') == 'waiting_local' else nav.get('state')
            labels = {'imu_stale': '等待新 IMU', 'lio_stale': '等待新 LIO 后验',
                      'aligned_history_stale': '等待时效合格的对齐结果',
                      'waiting_aligned_history': '等待同时间的里程计', 'filter_stale': '融合输出超时'}
            loc = stage('已定位 · 输出暂缓', 'warning', labels.get(reason, str(reason or '等待新融合输出')))
        elif localization.get('localized') is True:
            loc = stage('定位状态待核对', 'warning', preview_reason)
        elif matching_confirmation_pending(localization, session_id=session_id, now=now):
            loc = stage('匹配确认中', 'warning', '初值已接收，等待连续匹配确认')
        elif localization.get('state') == 'filter_initializing':
            loc = stage('等待融合输出', 'warning')
        elif (localization.get('initial_pose_ready') is True
              and not localization.get('active_seed_ns')):
            loc = stage('请给初值', 'warning', feedback or '2D Pose Estimate')
        else:
            loc = stage('等待定位', 'warning', localization.get('state', ''))
    epoch = localization.get('local_epoch')
    seed = localization.get('confirmed_seed_ns')
    def same_context(value):
        return (preview_ready and type(epoch) is int and epoch > 0
                and isinstance(seed, str) and bool(seed)
                and seed == localization.get('active_seed_ns')
                and value.get('session_id') == session_id
                and type(value.get('localization_epoch')) is int and value.get('localization_epoch') == epoch
                and value.get('localization_seed_id') == seed)
    hold, goal = global_state.get('recovery_hold'), global_state.get('active_goal')
    # A retained user mission is not an executable reference. Its display
    # survives a soft pose-output gap, but not a contradictory identity/reset.
    retained_hold = (global_state.get('session_id') == session_id
        and global_state.get('state') == 'goal_retained_waiting_recovery'
        and global_state.get('planning_phase') == 'goal_retained_waiting_recovery'
        and global_state.get('goal_retained') is True
        and global_state.get('active_reference') is False
        and isinstance(hold, dict) and isinstance(goal, dict)
        and goal.get('session_id') == session_id
        and finite_number(goal.get('user_stamp'))
        and goal.get('user_stamp') == hold.get('user_stamp')
        and (epoch is None or goal.get('epoch') == epoch)
        and (not localization.get('active_seed_ns')
             or goal.get('seed_id') == localization.get('active_seed_ns'))
        and quality != 'fault' and not localization.get('local_fault'))
    waiting_support = (retained_hold and hold.get('reason')
        == 'current_floor_requires_unique_measured_ground_support')
    # Task output availability and permission to follow it are separate rows.
    # A confirmed route stays generated through a short current-pose gap;
    # this predicate never participates in trajectory_ok or motion admission.
    route_identity = body_identity(localization, session_id=session_id, now=now)
    committed_route = (global_state.get('route_committed') is True
        and global_state.get('visual_path_available') is True
        and global_state.get('goal_retained') is True
        and isinstance(goal, dict) and route_identity is not None
        and (global_state.get('session_id'), global_state.get('localization_epoch'),
             global_state.get('localization_seed_id')) == route_identity
        and (goal.get('session_id'), goal.get('epoch'), goal.get('seed_id')) == route_identity
        and not localization.get('reset_pending')
        and not (isinstance(localization.get('navigation'), dict)
                 and localization['navigation'].get('reset_pending')))
    glob = stage('等待目标', 'muted')
    if global_state:
        state = global_state.get('state', '')
        state = state if isinstance(state, str) else ''
        phase = global_state.get('planning_phase')
        reason = global_state.get('reason', state)
        if retained_hold:
            glob = stage('目标保留 · 等待可用起点' if waiting_support else '规划未完成',
                         'warning', hold.get('reason', ''))
        elif phase == 'expired' or state == 'expired' or 'timeout' in state:
            glob = stage('请求超时 · 请重新提交', 'error', reason)
        elif 'fail' in state or 'reject' in state:
            glob = stage('规划失败', 'error', reason)
        elif global_state.get('planning') or global_state.get('pending_worker_start'):
            elapsed = global_state.get('native_elapsed_sec')
            metrics = global_state.get('worker_metrics')
            metrics = metrics if isinstance(metrics, dict) else {}
            activity = {'worker_request_queued': '准备规划', 'initializing_map': '加载规划地图',
                        'native_search_and_validation': '搜索与路径检查',
                        'validating_source_frame': '核对路径坐标'}.get(metrics.get('phase'), '计算中')
            label = (f'{activity} · {elapsed:.0f}s' if finite_number(elapsed)
                     and 0 <= elapsed <= 300 else activity)
            glob = stage(label, 'warning', reason)
        elif committed_route:
            glob = stage('路径已生成', 'ready')
        elif phase == 'awaiting_navigation':
            glob = stage('路线保留 · 等待定位' if global_state.get('visual_path_available')
                         else '已算完 · 等待定位输出', 'warning', reason)
        elif global_state.get('active_reference') is True and same_context(global_state):
            glob = stage('路径有效', 'ready',
                '局部规划等待实时数据' if phase == 'awaiting_local_sensor' else '')
        elif global_state.get('paused_for_recovery') is True and global_state.get('goal_retained') is True:
            glob = stage('等待定位恢复', 'warning', reason)
        else:
            glob = stage('等待目标', 'muted', global_state.get('reason', ''))
    active = local.get('active_reference') is True
    paired = (local.get('local_debug_valid') is True and local.get('spline_visual_valid') is True
              and type(local.get('last_spline_id')) is int and local['last_spline_id'] >= 0
              and type(local.get('local_debug_plan_id')) is int
              and local['local_debug_plan_id'] == local['last_spline_id'])
    recent = finite_number(local.get('last_spline_stamp')) and -.1 <= now-local['last_spline_stamp'] <= 2.
    proof = local.get('trajectory_revalidation')
    # Geometry creation time is immutable. A no-motion preview may remain
    # current through an explicit native recheck of that SAME curve, not by
    # renewing its original timestamp or a UI heartbeat.
    if (local.get('execution_mode') == 'preview'
            and local.get('collision_policy') == 'official'
            and local.get('perception_backend') == 'per_sensor_rays'
            and isinstance(proof, dict) and proof.get('enabled') is True):
        checked_at = proof.get('checked_at')
        recent = (proof.get('valid') is True
            and proof.get('proof_kind') in ('accepted', 'revalidated')
            and type(proof.get('plan_id')) is int
            and proof['plan_id'] == local.get('last_spline_id')
            and proof.get('original_spline_stamp') == local.get('last_spline_stamp')
            and finite_number(checked_at) and -.01 <= now-checked_at <= 2.)
    trajectory_ok = (local.get('ready') is True and active and paired and recent
                     and same_context(local) and same_context(global_state))
    # Cross-stream status arrival order may differ. A green local row requires
    # matching source path stamp; global worker and native bridge generations
    # are deliberately independent counters, not interchangeable identifiers.
    route = global_state.get('last_route')
    route = route if isinstance(route, dict) else {}
    # An identical owner re-publication can be accepted without restarting
    # native SCAN. Its owner stamp and native generation issue time differ.
    reference_stamp = local.get('owner_reference_stamp', local.get('reference_stamp'))
    path_stamp = route.get('path_stamp')
    trajectory_ok = trajectory_ok and (global_state.get('active_reference') is True
        and all(finite_number(x) for x in (path_stamp, reference_stamp))
        and abs(path_stamp-reference_stamp) < 1e-6)
    if trajectory_ok:
        plan = stage('轨迹有效', 'ready')
    elif retained_hold:
        # The owner withdrew the executable reference. An older SCAN status
        # can arrive later; do not present its old valid/failed attempt as live.
        plan = stage('等待可用起点' if waiting_support else '等待重新规划', 'warning')
    elif active and local.get('preview_reference_paused') is True:
        plan = stage('等待局部数据', 'warning', local.get('last_error') or local.get('reason', ''))
    elif active and local.get('local_debug_phase') in {
            'failed', 'failed_reference_search', 'failed_rebound_search',
            'failed_optimization', 'failed_final_collision', 'failed_current_validation', 'failed_dynamics',
            'failed_reference_geometry', 'failed_reference_search_budget',
            'failed_reference_target_occupied', 'failed_reference_search_collision',
            'failed_reference_start_occupied', 'failed_reference_lattice_occupied',
            'failed_reference_outside_map',
            'waiting_environment', 'waiting_sensor_map', 'waiting_recheck', 'waiting_goal_reached',
            'waiting_observed_space', 'failed_ground_support'}:
        phase = local['local_debug_phase']
        labels = {'failed': '局部规划失败',
                  'failed_reference_search': '未找到绕行通道',
                  'failed_rebound_search': '绕障搜索未成功',
                  'failed_optimization': '轨迹优化未收敛',
                  'failed_final_collision': '轨迹碰撞校验未通过',
                  'failed_current_validation': '当前轨迹需重新规划',
                  'failed_dynamics': '轨迹动力学未通过',
                  'failed_reference_geometry': '参考路径无效',
                  'failed_reference_search_budget': '绕行搜索达到上限',
                  'failed_reference_target_occupied': '局部目标被占用',
                  # This is a detour search anchor, not necessarily the robot.
                  'failed_reference_start_occupied': '绕行起点受阻',
                  'failed_reference_lattice_occupied': '搜索格端点受阻',
                  'failed_reference_outside_map': '局部端点超出地图',
                  'failed_reference_search_collision': '绕行连接段碰撞',
                  'waiting_environment': '受阻 · 等待环境更新',
                  'waiting_sensor_map': '等待局部地图',
                  'waiting_recheck': '重新检查轨迹',
                  'waiting_observed_space': '绕行空间尚未观测',
                  'failed_ground_support': '轨迹离开可通行地面',
                  'waiting_goal_reached': '已接近局部目标'}
        plan = stage(labels[phase], 'warning', local.get('last_error', '') or phase)
    elif active and local.get('ready') is True:
        plan = stage('等待轨迹', 'warning', local.get('last_error', ''))
    elif local:
        plan = stage('等待参考' if local.get('ready') is True else '输入未就绪', 'muted',
                     local.get('last_error') or local.get('reason', ''))
    else:
        plan = stage('等待数据', 'muted')
    target = local.get('local_debug_target') if trajectory_ok else None
    if not (isinstance(target, (list, tuple)) and len(target) == 3
            and all(finite_number(x) and abs(x) < 10000 for x in target)):
        target = None
    ages = {k: local.get(k) if finite_number(local.get(k)) and 0 <= local[k] < 100 else None
            for k in ('body_age', 'cloud_age')}
    progress, target_arc = local.get('local_debug_progress_arc_m'), local.get('local_debug_target_arc_m')
    horizon = target_arc-progress if trajectory_ok and all(finite_number(x) for x in (progress, target_arc)) and target_arc >= progress else None
    return {'schema': 1, 'mode': 'LIVE_VISUALIZATION_NO_MOTION', 'motion_enabled': False,
            'session_id': session_id, 'stamp': now, 'frame_id': 'd1max_loc_map',
            'preview_ready': preview_ready, 'navigation_admission': admission,
            'localization_degraded': degraded,
            'localization_quality_detail': degradation_detail if degraded else '',
            'map_identity_valid': identity is not None, 'output_quality': quality,
            'stages': {'localization': loc, 'global': glob, 'local': plan},
            'body_age': ages['body_age'], 'cloud_age': ages['cloud_age'],
            'local_target': target, 'reference_horizon_m': horizon,
            'plan_id': local.get('local_debug_plan_id') if trajectory_ok else None,
            'notice': feedback if feedback and identity is None else ''}
