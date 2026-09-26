import pytest

from d1max_navigation.status_display import classify_status


def healthy():
    return {'session_id': 'session-a', 'map_version_id': 'grid-a', 'wall_time': 100.,
            'state': 'tracking', 'localized': True,
            'frontend_ready': True, 'local_ekf_fresh': True, 'global_ekf_fresh': True,
            'frames': {'map': 'd1max_loc_map'}, 'sensors': {'imu': True, 'cloud': True},
            'navigation': {'valid': True},
            'calibration': {'extrinsics_verified': False, 'time_alignment_verified': False}}


def classify(status=None, **overrides):
    settings = dict(offline=False, expected_version_id='grid-a', now_wall=100.5,
                    now_monotonic=10.5, received_monotonic=10.0)
    settings.update(overrides)
    return classify_status(healthy() if status is None else status, **settings)


def test_offline_cannot_turn_localized_even_with_valid_messages():
    result = classify(offline=True)
    assert result.state == 'offline'
    assert 'OFFLINE' in result.text and 'MOTION DISABLED' in result.text


def test_waiting_until_first_message():
    assert classify(received_monotonic=None).state == 'waiting_sensors'


def test_localized_is_not_navigation_or_calibration_acceptance():
    result = classify()
    assert result.state == 'localized'
    assert 'MOTION DISABLED' in result.text
    assert 'CALIBRATION NOT VERIFIED' in result.text


@pytest.mark.parametrize('overrides', [
    {'now_monotonic': 12.0}, {'now_monotonic': 9.0}, {'now_wall': 103.},
    {'now_wall': 99.}, {'now_wall': float('nan')}, {'max_age': 2.0},
])
def test_staleness_checked_on_timer_without_new_data(overrides):
    assert classify(**overrides).state == 'stale'


@pytest.mark.parametrize('change', [
    lambda s: s.update(map_version_id='other'),
    lambda s: s.update(session_id=''),
    lambda s: s.update(frames={'map': 'map'}),
    lambda s: s.update(local_fault='bad local odometry'),
    lambda s: s.update(navigation={'fault': 'stale'}),
    lambda s: s.update(frontend_ready=False),
    lambda s: s.update(global_ekf_fresh=False),
    lambda s: s.update(local_ekf_fresh=False),
    lambda s: s.update(state='lost'),
    lambda s: s.update(state='degraded'),
    lambda s: s.update(state='unrecognized'),
])
def test_mismatch_and_invalid_output_fail_closed(change):
    state = healthy()
    change(state)
    assert classify(state).state == 'fault'


def test_pin_session_and_require_version():
    assert classify(expected_session_id='other').state == 'fault'
    assert classify(expected_version_id='').state == 'fault'


def test_false_sensor_flags_do_not_show_healthy_tracking():
    state = healthy()
    state['sensors']['imu'] = False
    assert classify(state).state == 'waiting_sensors'


@pytest.mark.parametrize('state_name,ready,expected', [
    ('waiting_sensors', False, 'waiting_sensors'),
    ('calibrating', False, 'waiting_sensors'),
    ('recovering_local', False, 'waiting_sensors'),
    ('waiting_initial_pose', True, 'set_initial_pose'),
    ('waiting_initial_pose', False, 'waiting_sensors'),
    ('acquiring', True, 'localizing'),
    ('relocalizing', True, 'localizing'),
    ('filter_initializing', True, 'localizing'),
])
def test_expected_live_states(state_name, ready, expected):
    state = healthy()
    state.update(state=state_name, localized=False, initial_pose_ready=ready)
    assert classify(state).state == expected


def test_malformed_status_is_not_healthy():
    assert classify({}).state == 'fault'
    assert classify([]).state == 'fault'
