"""Pure RViz seed validation; a seed is not a localization result or a command."""
import math
import json


def floor_initial_body_z(bridge, floor_id, xy, body_height):
    """Seed height from the selected ORIGINAL-map floor, never a control TF.

    A seed is allowed to use an explicitly approximate standing height. It is
    not a measured pose, a calibration certificate, or authority to move.
    """
    import numpy as np
    if floor_id not in ('floor1', 'floor2') or floor_id not in bridge.floors:
        raise ValueError('请选择初始楼层')
    xy = np.asarray(xy, dtype=float)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        raise ValueError('初始位置无效')
    if type(body_height) not in (int, float) or not math.isfinite(body_height) or not .1 <= body_height <= 1.:
        raise ValueError('机身参考高度无效')
    field, distance = bridge.floors[floor_id].query(xy[None, :], bridge.limits)
    ground = np.r_[xy, float(field[0])]
    for region in bridge.protected_regions:
        if np.all(ground >= np.asarray(region['min'])) and np.all(ground <= np.asarray(region['max'])):
            raise ValueError('楼梯内部初值需要独立的支撑面确认，不能按平层高度猜测')
    result = float(field[0]) + body_height
    if not math.isfinite(result) or not -10 <= result <= 10:
        raise ValueError('初始高度超出有效范围')
    return result


def localization_degradation(value):
    """Current scan/prediction quality only, never the lifetime maximum gap."""
    frontend = value.get('frontend')
    frontend = frontend if isinstance(frontend, dict) else {}
    navigation = value.get('navigation')
    navigation = navigation if isinstance(navigation, dict) else {}
    prediction = navigation.get('prediction')
    prediction = prediction if isinstance(prediction, dict) else {}
    gap, warning = frontend.get('scan_imu_gap_sec'), frontend.get('imu_gap_warning_sec', .015)
    finite = lambda v: type(v) in (int, float) and math.isfinite(v)
    current_front = (type(frontend.get('epoch')) is int
                     and frontend.get('epoch') == value.get('local_epoch'))
    scan_degraded = (current_front and (frontend.get('degraded') is True
        or finite(gap) and finite(warning) and 0 <= warning < gap <= 1.))
    prediction_degraded = prediction.get('degraded') is True
    if not (scan_degraded or prediction_degraded):
        return False, ''
    details = []
    if scan_degraded:
        details.append(f'当前扫描 IMU 缺口 {gap*1000:.1f} ms' if finite(gap) and 0 <= gap <= 1.
                       else '当前扫描 IMU 短时缺帧')
    if prediction_degraded:
        imu_gap = prediction.get('imu_gap')
        predicted_gap = imu_gap.get('max_sec') if isinstance(imu_gap, dict) else None
        if not (finite(predicted_gap) and 0 <= predicted_gap <= 1.):
            predicted_gap = prediction.get('largest_imu_gap_sec')  # Older status producers.
        details.append(f'运动预测 IMU 缺口 {predicted_gap*1000:.1f} ms'
                       if finite(predicted_gap) and 0 <= predicted_gap <= 1. else '运动预测短时数据降级')
    return True, '；'.join(details) + '；定位预览仍有效'


def matching_confirmation_pending(value, *, session_id, now, maximum_age=.6):
    """A submitted current seed is not a request to submit another seed.

    The localizer exposes only zero or the accepted >=3 confirmations, not
    intermediate matcher progress. Do not invent a numeric progress display.
    """
    if (not isinstance(value, dict) or value.get('session_id') != session_id
            or not source_status_fresh(value, stamp_field='wall_time', now=now,
                                       timeout=maximum_age)
            or value.get('localized') is not False or value.get('local_fault')
            or value.get('state') not in ('acquiring', 'relocalizing')):
        return False
    epoch, seed = value.get('local_epoch'), value.get('active_seed_ns')
    count, frontend = value.get('verified_confirmations'), value.get('frontend')
    return (type(epoch) is int and epoch > 0 and isinstance(seed, str)
            and 1 <= len(seed) <= 20 and seed.isdecimal() and int(seed) > 0
            and value.get('confirmed_seed_ns') is None
            and type(count) is int and 0 <= count < 3
            and value.get('frontend_ready') is True and isinstance(frontend, dict)
            and type(frontend.get('epoch')) is int and frontend.get('epoch') == epoch
            and frontend.get('ready') is True and not frontend.get('fault'))


def localization_preview_state(value, *, session_id, now, maximum_age=.6):
    """Read-only preview admission, separate from hardware/navigation acceptance."""
    if (not isinstance(value, dict) or value.get('session_id') != session_id
            or not source_status_fresh(value, stamp_field='wall_time', now=now,
                                       timeout=maximum_age)):
        return False, '定位状态已过期或会话不一致'
    if value.get('local_fault'):
        return False, str(value['local_fault'])[:300]
    # UI snapshot only. The native planning bridge separately checks the
    # high-rate source-timed pose lease; never use this 5 Hz view as authority.
    if value.get('continuous_pose_valid', value.get('localized')) is not True:
        return False, '定位尚未完成'
    confirmations = value.get('verified_confirmations')
    if type(confirmations) is not int or confirmations < 3:
        return False, '尚未完成连续匹配确认'
    epoch, seed = value.get('local_epoch'), value.get('active_seed_ns')
    navigation = value.get('navigation')
    if (type(epoch) is not int or epoch < 1 or not isinstance(seed, str) or not seed
            or seed != value.get('confirmed_seed_ns') or not isinstance(navigation, dict)
            or type(navigation.get('epoch')) is not int or navigation.get('epoch') != epoch
            or navigation.get('seed_id') != seed):
        return False, '定位轮次或初值尚未同步'
    if ((value.get('continuous_pose_valid') is not True and navigation.get('valid') is not True)
            or navigation.get('fault')
            or not source_status_fresh(navigation, stamp_field='received_at_unix',
                                       now=now, timeout=maximum_age)):
        return False, str(navigation.get('fault') or '融合定位输出尚未有效或已过期')[:300]
    return True, ''


def navigation_admission(value, *, preview_ready):
    """Explain existing acceptance flags; never set them or authorize motion."""
    navigation = value.get('navigation') if isinstance(value, dict) else None
    navigation = navigation if isinstance(navigation, dict) else {}
    calibration = navigation.get('calibration')
    calibration = calibration if isinstance(calibration, dict) else {}
    def number(key):
        result = navigation.get(key)
        return result if type(result) in (int, float) and math.isfinite(result) else None
    target, observed = number('target_hz'), number('global_observed_hz')
    target = target if target is not None and target > 0 else None
    observed = observed if observed is not None and observed >= 0 else None
    minimum, maximum = (.8*target, 1.2*target) if target else (None, None)
    blockers = []
    if not preview_ready:
        blockers.append('定位输出尚未有效')
    for key, label in (('extrinsics_verified', '外参'),
                       ('time_alignment_verified', '传感器时间对齐')):
        if calibration.get(key) is not True:
            blockers.append(label + ('未完成实机验收' if calibration.get(key) is False else '验收状态未知'))
    if observed is None or target is None:
        blockers.append('输出频率验收数据缺失')
    elif not minimum <= observed <= maximum:
        blockers.append(f'输出频率 {observed:.1f} Hz，要求 {minimum:g}–{maximum:g} Hz')
    if navigation.get('navigation_ready') is not True and not blockers:
        blockers.append('导航输出尚未报告验收通过')
    ready = preview_ready and navigation.get('navigation_ready') is True and not blockers
    return {'ready': ready, 'label': '已通过导航验收' if ready else '未通过导航验收',
            'tone': 'ready' if ready else 'warning', 'detail': '；'.join(blockers),
            'blockers': blockers, 'target_hz': target, 'global_observed_hz': observed,
            'minimum_hz': minimum, 'maximum_hz': maximum}


def source_status_fresh(value, *, stamp_field, now, timeout):
    stamp = value.get(stamp_field) if isinstance(value, dict) else None
    return (type(stamp) in (int, float) and math.isfinite(stamp)
            and -.1 <= now-stamp <= timeout)


def checked_session_status(raw, *, session_id, stamp_field, now, timeout,
                           previous_stamp=0., maximum_bytes=262144):
    """Receipt of an old packet must not make old status look healthy again."""
    if not isinstance(raw, str) or len(raw) > maximum_bytes:
        return None
    try:
        value = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if (not isinstance(value, dict) or value.get('session_id') != session_id
            or not source_status_fresh(value, stamp_field=stamp_field, now=now, timeout=timeout)
            or value[stamp_field] <= previous_stamp):
        return None
    return value


def initial_pose_feedback(state, command_id):
    """Only acknowledge the seed submitted by this view, never an old seed."""
    result = state.get('command_result')
    if (not command_id or not isinstance(result, dict)
            or result.get('id') != command_id or type(result.get('accepted')) is not bool):
        return None
    prefix = '初值已接收（仍需匹配确认）' if result['accepted'] else '初值被拒绝，请重新给初值'
    return prefix + '：' + str(result.get('message', ''))[:300]


def initial_pose_command(*, frame, stamp, now, xy, quaternion, body_z,
                         state, session_id, state_age, command_id):
    if frame != 'd1max_loc_map' or not math.isfinite(stamp) or not -.1 <= now-stamp <= 2:
        raise ValueError('初值坐标系或时间不正确')
    if (state.get('session_id') != session_id or state.get('initial_pose_ready') is not True
            or not 0 <= state_age <= .6 or not math.isfinite(state.get('wall_time', 0))
            or not -.1 <= now-state.get('wall_time', 0) <= .6):
        raise ValueError('LIO/机身状态尚未就绪；保持静止后重试初值')
    if len(xy) != 2 or len(quaternion) != 4 or not all(
            math.isfinite(v) for v in (*xy, *quaternion, body_z)):
        raise ValueError('初值包含无效数字')
    if max(abs(v) for v in xy) > 10000 or abs(body_z) > 100:
        raise ValueError('初值超出允许范围')
    if abs(math.hypot(*quaternion)-1.) > .01:
        raise ValueError('初值四元数不是单位四元数')
    qx, qy, qz, qw = quaternion
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return {'id': command_id, 'session_id': session_id, 'created_at': now,
            'x': xy[0], 'y': xy[1], 'z': body_z, 'yaw': yaw, 'reference': 'body'}
