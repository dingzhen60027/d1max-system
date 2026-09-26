from dataclasses import replace

import pytest

from d1max_navigation.command_gate import CommandGate, GateConfig, monitor_endpoint, sdk_command_envelope, arm_generation_revoked, limit_velocity, INITIAL_COMMAND_TIMEOUT, preflight_status


def healthy(gate, now=10., wall=100.):
    gate.observe('odom', {'valid': True, 'frame': gate.config.odom_frame,
                         'child_frame': gate.config.base_frame}, now)
    gate.observe('scan', {'valid': True, 'frame': gate.config.base_frame}, now)
    gate.observe('localization', {'session_id': 'session-a', 'map_version_id': 'grid-a',
        'wall_time': wall, 'frames': {'map': gate.config.map_frame}, 'state': 'tracking',
        'localized': True, 'navigation_ready': True, 'navigation': {'valid': True},
        'calibration': {'extrinsics_verified': True, 'time_alignment_verified': True}}, now)
    gate.observe('robot', {'received_at_unix': wall, 'software_emergency_status': 1,
        'hardware_emergency_status': 1, 'control_source': 2, 'sport_mode': 1,
        'motion_status': 5, 'speed_level': 1, 'head_direction': 1}, now)
    gate.observe('behavior', {'telemetry_only': False, 'ready_for_navigation': True,
        'sdk_has_control': True, 'replay_latched': False, 'fault_latched': False,
        'requires_review': False}, now)
    gate.observe('mc', {'source': 'sdk_mc', 'callback': 'OnMcData',
        'stream_fresh': True, 'rate_ok': True, 'received_at_unix': wall}, now)
    return gate


def live(**overrides):
    config = GateConfig(mode='live', motion_enabled=True,
                        expected_version_id='grid-a', expected_session_id='session-a')
    return healthy(CommandGate(replace(config, **overrides)))


def test_default_is_hardware_locked_even_with_good_data():
    gate = healthy(CommandGate(GateConfig()))
    assert gate.arm(True, 10., 100.) == (False, 'live_motion_disabled')
    gate.receive_command((.2, 0., .2), 10.)
    assert gate.output(10., 100.).velocity == (0., 0., 0.)


def test_simulation_does_not_need_sdk_or_localizer():
    gate = healthy(CommandGate(GateConfig(mode='simulation')))
    gate.samples = {k: v for k, v in gate.samples.items() if k in ('odom', 'scan')}
    gate.receive_command((.2, 0., .1), 10.)
    assert gate.output(10., 100.).allowed
    assert gate.output(10., 100.).reason == 'simulation_only'


def test_live_requires_explicit_arm_and_new_command_after_arm():
    gate = live()
    gate.receive_command((.1, 0., .1), 10.)
    assert gate.output(10., 100.).reason == 'not_armed'
    assert gate.arm(True, 10., 100.)[0]
    assert gate.output(10., 100.).reason == 'command_missing_or_stale'
    gate.receive_command((.1, 0., .1), 10.)
    assert gate.output(10., 100.).allowed


def test_limit_is_si_not_sdk_normalized_and_lateral_disabled():
    gate = healthy(CommandGate(GateConfig(mode='simulation')))
    gate.receive_command((2., -.3, 9.), 10.)
    assert gate.output(10., 100.).velocity == (.3, 0., .5)


def test_planar_limit_applies_to_vector_not_axes_independently():
    import math
    velocity = limit_velocity((10., 10., 9.), 10., 10., .5, 99.)
    assert math.hypot(*velocity[:2]) == pytest.approx(1.5)
    assert velocity[2] == .5
    gate = healthy(CommandGate(GateConfig(mode='simulation', max_forward=1.,
                                         max_lateral=.5, max_planar_speed=.4)))
    gate.receive_command((1., .5, .1), 10.)
    assert math.hypot(*gate.output(10., 100.).velocity[:2]) == pytest.approx(.4)


@pytest.mark.parametrize('value', [1.500001, float('inf'), 0., -1., True])
def test_user_speed_ceiling_cannot_be_relaxed_by_configuration(value):
    with pytest.raises(ValueError):
        GateConfig(max_planar_speed=value)


def test_measured_overspeed_stops_and_requires_explicit_rearm():
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    gate.receive_command((.2, 0., 0.), 10.)
    gate.samples['odom'][0]['planar_speed'] = 1.51
    assert gate.output(10., 100.).reason == 'measured_speed_exceeds_hard_limit'
    assert not gate.armed
    healthy(gate)
    assert gate.output(10., 100.).reason == 'not_armed'


def test_sdk_envelope_refuses_bypass_of_hard_planar_limit():
    with pytest.raises(ValueError):
        sdk_command_envelope(sdk_session='1234567890abcdef', generation=1, sequence=1,
            navigation_session='n', map_version_id='m', velocity=(1.5, .01, 0.),
            wall=100., command_source='a'*32)


@pytest.mark.parametrize('kind', ['odom', 'scan', 'localization', 'robot', 'behavior', 'mc'])
def test_evidence_dropout_zeroes_and_latches_live_disarm(kind):
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    gate.receive_command((.2, 0., .2), 10.)
    gate.samples.pop(kind)
    result = gate.output(10., 100.)
    assert not result.allowed
    assert result.velocity == (0., 0., 0.)
    assert not gate.armed
    healthy(gate)
    assert gate.output(10., 100.).reason == 'not_armed'


def test_current_monitor_bridge_never_passes_physical_gate():
    gate = live()
    gate.observe('behavior', {'fsm_state': 'MONITOR_ONLY', 'telemetry_only': True,
        'motion_control_enabled': False, 'ready_for_navigation': None,
        'sdk_has_control': True, 'replay_latched': False}, 10.)
    assert gate.arm(True, 10., 100.) == (False, 'sdk_motion_adapter_not_ready')


@pytest.mark.parametrize('kind,field,value', [
    ('localization', 'session_id', 'different'),
    ('localization', 'map_version_id', 'different'),
    ('localization', 'wall_time', 90.),
    ('localization', 'wall_time', 100.01),
    ('localization', 'navigation_ready', False),
    ('localization', 'calibration', {'extrinsics_verified': False, 'time_alignment_verified': True}),
    ('localization', 'navigation', {'valid': True, 'fault': 'bad'}),
    ('localization', 'state', 'acquiring'),
    ('robot', 'software_emergency_status', 0),
    ('robot', 'software_emergency_status', 2),
    ('robot', 'hardware_emergency_status', 2),
    ('robot', 'control_source', 1),
    ('robot', 'motion_status', 7),
    ('robot', 'speed_level', 2),
    ('robot', 'head_direction', 2),
    ('robot', 'received_at_unix', 90.),
    ('behavior', 'sdk_has_control', False),
    ('behavior', 'replay_latched', True),
    ('behavior', 'fault_latched', True),
    ('behavior', 'requires_review', True),
    ('mc', 'callback', 'OnSpeedData'),
    ('mc', 'stream_fresh', False),
    ('mc', 'rate_ok', False),
    ('mc', 'received_at_unix', 90.),
    ('odom', 'frame', 'map'),
    ('odom', 'child_frame', 'tracking'),
    ('scan', 'frame', 'laser'),
    ('scan', 'valid', False),
])
def test_unsafe_health_is_not_armable(kind, field, value):
    gate = live()
    gate.samples[kind][0][field] = value
    assert not gate.arm(True, 10., 100.)[0]


@pytest.mark.parametrize('kind', ['odom', 'scan', 'localization', 'robot', 'behavior', 'mc'])
def test_stale_and_future_receipt_rejected(kind):
    gate = live()
    payload = gate.samples[kind][0]
    for receipt in (0., 11., float('nan')):
        gate.observe(kind, payload, receipt)
        assert not gate.arm(True, 10., 100.)[0]


def test_timeout_stops_even_without_new_command_and_does_not_replay():
    gate = healthy(CommandGate(GateConfig(mode='simulation')))
    gate.receive_command((.2, 0., .2), 10.)
    assert gate.output(10., 100.).allowed
    assert gate.output(10.26, 100.26).reason == 'command_missing_or_stale'
    assert gate.output(10.26, 100.26).velocity == (0., 0., 0.)


@pytest.mark.parametrize('velocity', [(0., 0., 0.), (.2, 0., .1)])
def test_live_command_timeout_latches_disarm_including_zero_heartbeat(velocity):
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    assert gate.receive_command(velocity, 10.)
    assert gate.output(10., 100.).allowed
    result = gate.output(10.26, 100.26)
    assert result.reason == 'command_timeout_rearm_required'
    assert result.velocity == (0., 0., 0.)
    assert not result.allowed and not gate.armed
    healthy(gate, 10.3, 100.3)
    gate.receive_command((.2, 0., 0.), 10.3)
    assert gate.output(10.3, 100.3).reason == 'not_armed'
    assert gate.arm(True, 10.3, 100.3)[0]
    assert not gate.output(10.3, 100.3).allowed  # Pre-arm input was discarded.
    assert gate.receive_command((.2, 0., 0.), 10.31)
    assert gate.output(10.31, 100.31).allowed


def test_live_initial_command_wait_is_bounded_and_does_not_resume():
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    healthy(gate, 10.5, 100.5)
    assert gate.output(10.5, 100.5).reason == 'command_missing_or_stale'
    assert gate.armed
    elapsed = INITIAL_COMMAND_TIMEOUT + .01
    healthy(gate, 10. + elapsed, 100. + elapsed)
    assert gate.output(10. + elapsed, 100. + elapsed).reason == 'initial_command_timeout_rearm_required'
    assert not gate.armed
    gate.receive_command((.2, 0., 0.), 10. + elapsed)
    assert not gate.output(10. + elapsed, 100. + elapsed).allowed


@pytest.mark.parametrize('first_command', [False, True])
def test_new_receipt_cannot_hide_live_timeout_before_timer_runs(first_command):
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    if first_command:
        assert gate.receive_command((.2, 0., 0.), 10.)
    elapsed = gate.config.command_timeout + .01 if first_command else INITIAL_COMMAND_TIMEOUT + .01
    healthy(gate, 10. + elapsed, 100. + elapsed)
    assert not gate.receive_command((.2, 0., 0.), 10. + elapsed)
    assert not gate.armed and gate.command is None
    assert not gate.output(10. + elapsed, 100. + elapsed).allowed


def test_live_initial_command_within_grace_starts_normal_timeout():
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    healthy(gate, 10.5, 100.5)
    assert gate.receive_command((.2, 0., 0.), 10.5)
    assert gate.output(10.7, 100.7).allowed
    assert gate.output(10.76, 100.76).reason == 'command_timeout_rearm_required'


@pytest.mark.parametrize('now', [9.99, float('nan'), float('inf')])
def test_live_command_clock_anomaly_revokes_arm(now):
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    assert not gate.receive_command((.2, 0., 0.), now)
    assert not gate.armed


def test_live_malformed_command_requires_rearm():
    gate = live()
    assert gate.arm(True, 10., 100.)[0]
    assert gate.receive_command((.2, 0., 0.), 10.)
    assert not gate.receive_command((float('nan'), 0., 0.), 10.1)
    assert not gate.armed
    gate.receive_command((.2, 0., 0.), 10.2)
    assert not gate.output(10.2, 100.2).allowed


def test_simulation_keeps_existing_timeout_and_resume_contract():
    gate = healthy(CommandGate(GateConfig(mode='simulation')))
    healthy(gate, 12., 102.)
    assert gate.output(12., 102.).reason == 'command_missing_or_stale'
    assert gate.armed
    assert gate.receive_command((.2, 0., 0.), 12.)
    assert not gate.output(12.26, 102.26).allowed
    assert gate.armed
    assert gate.receive_command((.2, 0., 0.), 12.3)
    assert gate.output(12.3, 102.3).allowed


@pytest.mark.parametrize('velocity', [(float('nan'), 0., 0.), (0., float('inf'), 0.), (0., 0., True)])
def test_malformed_command_replaces_previous_with_stop(velocity):
    gate = healthy(CommandGate(GateConfig(mode='simulation')))
    gate.receive_command((.2, 0., .2), 10.)
    assert not gate.receive_command(velocity, 10.)
    assert gate.output(10., 100.).velocity == (0., 0., 0.)


@pytest.mark.parametrize('kwargs', [
    {'mode': 'offline'}, {'command_timeout': .3}, {'max_forward': 10.},
    {'max_yaw': float('nan')}, {'motion_enabled': 'false'}, {'odom_timeout': 0.},
])
def test_unsafe_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        GateConfig(**kwargs)


def test_live_map_and_session_pins_required():
    for kwargs in ({'expected_version_id': ''}, {'expected_session_id': ''}):
        assert live(**kwargs).arm(True, 10., 100.) == (False, 'localization_context_not_pinned')


def monitor_status():
    return {'session': '1234567890abcdef', 'service_prefix': '/d1max/monitor/s_1234567890abcdef',
            'wall_time': 100., 'motion_control_enabled': True, 'mode': 'monitor'}


def test_sdk_endpoint_is_session_scoped_and_monitor_readiness_not_assumed():
    assert monitor_endpoint(monitor_status(), 100.) == ('1234567890abcdef', '/d1max/monitor/s_1234567890abcdef')


@pytest.mark.parametrize('field,value', [('session', 'other'), ('service_prefix', '/evil'),
    ('wall_time', 98.), ('wall_time', 101.), ('mode', 'replay'), ('motion_control_enabled', False)])
def test_sdk_endpoint_fail_closed(field, value):
    status = monitor_status()
    status[field] = value
    assert monitor_endpoint(status, 100.) is None


def test_sdk_envelope_contains_epoch_context_and_unmodified_si():
    packet = sdk_command_envelope(sdk_session='1234567890abcdef', generation=2, sequence=17,
        navigation_session='localization-a', map_version_id='grid-a', velocity=(.2, 0., .3), wall=100.,
        command_source='1234567890abcdef1234567890abcdef')
    assert packet == {'sdk_session': '1234567890abcdef', 'arm_generation': 2, 'seq': 17,
        'navigation_session': 'localization-a', 'map_version_id': 'grid-a', 'stamp': 100.,
        'x': .2, 'y': 0., 'yaw': .3, 'command_source': '1234567890abcdef1234567890abcdef'}


@pytest.mark.parametrize('observed,armed,age,revoked', [
    (1, False, .01, False), (1, False, .59, False), (1, False, .61, True),
    (2, True, .01, False), (2, True, 2., False), (2, False, .01, True),
    (3, True, .01, True), (None, False, .01, True), (2, True, float('nan'), True),
])
def test_queued_prearm_status_has_bounded_grace_and_never_overrides_a_revocation(observed, armed, age, revoked):
    assert arm_generation_revoked(observed, armed, 2, age) is revoked


def execution_permit(**overrides):
    return {'schema': 1, 'session_id': 'pct-session', 'map_version_id': 'grid-a',
            'allow': True, 'seq': 1, 'generation': 1, 'received_at_unix': 100., **overrides}


def permitted_live():
    gate = live(require_execution_permit=True, navigation_session_id='pct-session')
    assert gate.observe('execution_permit', execution_permit(), 10.)
    return gate


def test_execution_permit_is_optional_by_default_and_cannot_bypass_sdk_health():
    assert not GateConfig().require_execution_permit
    assert live().arm(True, 10., 100.)[0]
    gate = permitted_live()
    gate.samples['robot'][0]['control_source'] = 1
    assert gate.arm(True, 10., 100.) == (False, 'sdk_control_not_owned')


def test_required_execution_permit_missing_blocks_arm():
    gate = live(require_execution_permit=True, navigation_session_id='pct-session')
    assert gate.arm(True, 10., 100.) == (False, 'execution_permit_missing_or_stale')


@pytest.mark.parametrize('field,value', [
    ('schema', 2), ('schema', True), ('allow', 1), ('session_id', 'other'),
    ('map_version_id', 'other'), ('seq', 0), ('seq', True), ('generation', 0),
    ('generation', 1.5), ('received_at_unix', float('nan')),
])
def test_malformed_execution_permit_revokes_armed_gate(field, value):
    gate = permitted_live()
    assert gate.arm(True, 10., 100.)[0]
    payload = execution_permit(seq=2)
    payload[field] = value
    assert not gate.observe('execution_permit', payload, 10.1)
    assert not gate.armed
    assert gate.output(10.1, 100.1).velocity == (0., 0., 0.)


def test_execution_permit_requires_fresh_source_time_and_receipt():
    for payload, receipt, reason in [
        (execution_permit(received_at_unix=99.), 10., 'execution_permit_timestamp_invalid'),
        (execution_permit(received_at_unix=101.), 10., 'execution_permit_timestamp_invalid'),
        (execution_permit(), 9., 'execution_permit_missing_or_stale'),
        (execution_permit(), 11., 'execution_permit_missing_or_stale'),
    ]:
        gate = live(require_execution_permit=True, navigation_session_id='pct-session')
        gate.observe('execution_permit', payload, receipt)
        assert gate.arm(True, 10., 100.) == (False, reason)


def test_execution_permit_timeout_revokes_even_while_zero_commands_continue():
    gate = permitted_live()
    assert gate.arm(True, 10., 100.)[0]
    assert gate.receive_command((0., 0., 0.), 10.2)
    healthy(gate, 10.36, 100.36)
    assert gate.receive_command((0., 0., 0.), 10.36)
    result = gate.output(10.36, 100.36)
    assert result.reason == 'execution_permit_missing_or_stale'
    assert not gate.armed and not result.allowed
    gate.observe('execution_permit', execution_permit(seq=2, received_at_unix=100.37), 10.37)
    gate.receive_command((.2, 0., 0.), 10.37)
    assert gate.output(10.37, 100.37).reason == 'not_armed'


@pytest.mark.parametrize('sequence', [2, 3])
def test_execution_permit_duplicate_or_older_sequence_never_renews_freshness(sequence):
    gate = permitted_live()
    assert gate.observe('execution_permit', execution_permit(seq=3), 10.)
    assert gate.arm(True, 10., 100.)[0]
    assert gate.receive_command((0., 0., 0.), 10.2)
    before = gate.samples['execution_permit']
    assert not gate.observe('execution_permit', execution_permit(seq=sequence, received_at_unix=100.3), 10.3)
    assert gate.samples['execution_permit'] == before
    assert gate.output(10.36, 100.36).reason == 'execution_permit_missing_or_stale'
    assert not gate.armed


def test_new_permit_cannot_hide_expired_lease_before_timer_runs():
    gate = permitted_live()
    assert gate.arm(True, 10., 100.)[0]
    healthy(gate, 10.36, 100.36)
    assert gate.observe('execution_permit', execution_permit(seq=2, received_at_unix=100.36), 10.36)
    assert not gate.armed
    assert gate.output(10.36, 100.36).reason == 'not_armed'


@pytest.mark.parametrize('update', [{'allow': False}, {'generation': 2}])
def test_execution_revocation_or_generation_change_requires_explicit_rearm(update):
    gate = permitted_live()
    assert gate.arm(True, 10., 100.)[0]
    assert gate.receive_command((.2, 0., 0.), 10.)
    assert gate.observe('execution_permit', execution_permit(seq=2, received_at_unix=100.1, **update), 10.1)
    assert not gate.armed and gate.command is None
    assert gate.observe('execution_permit', execution_permit(seq=3, generation=2, received_at_unix=100.2), 10.2)
    assert not gate.output(10.2, 100.2).allowed
    assert gate.arm(True, 10.2, 100.2)[0]
    assert gate.receive_command((.2, 0., 0.), 10.21)
    assert gate.output(10.21, 100.21).allowed


def test_same_generation_permit_renewal_preserves_arm():
    gate = permitted_live()
    assert gate.arm(True, 10., 100.)[0]
    assert gate.receive_command((.2, 0., 0.), 10.)
    for sequence in range(2, 12):
        delta = (sequence-1) * .1
        healthy(gate, 10.+delta, 100.+delta)
        assert gate.observe('execution_permit', execution_permit(seq=sequence,
            received_at_unix=100.+delta), 10.+delta)
        assert gate.receive_command((.2, 0., 0.), 10.+delta)
        assert gate.output(10.+delta, 100.+delta).allowed


@pytest.mark.parametrize('overrides', [
    {'require_execution_permit': 'true'}, {'execution_permit_timeout': .36},
    {'execution_permit_timeout': 0.}, {'execution_permit_timeout': float('nan')},
])
def test_execution_permit_configuration_fails_closed(overrides):
    with pytest.raises(ValueError):
        GateConfig(**overrides)


def ready_preflight(gate, **overrides):
    params = {'monitor': {**monitor_status(), 'navigation_armed': False},
              'monitor_received': 10., 'sdk_session': '1234567890abcdef',
              'sdk_service_ready': True, 'sdk_arm_pending': False, **overrides}
    return preflight_status(gate, 10., 100., **params)


@pytest.mark.parametrize('permit', [None, execution_permit(allow=False),
                                  execution_permit(received_at_unix=99.)])
def test_preflight_excludes_permit_without_authorizing_arm_or_motion(permit):
    gate = live(require_execution_permit=True, navigation_session_id='pct-session')
    if permit is not None:
        gate.observe('execution_permit', permit, 10.)
    original_health = gate.health_reason(10., 100.)
    assert original_health
    before = (gate.armed, dict(gate.samples), gate.execution_permit_sequence,
              gate.execution_permit_generation, gate.command, gate.command_at)
    assert ready_preflight(gate) == {'preflight_ready': True, 'preflight_block_reason': ''}
    assert gate.health_reason(10., 100., check_execution_permit=False) == ''
    assert before == (gate.armed, gate.samples, gate.execution_permit_sequence,
                      gate.execution_permit_generation, gate.command, gate.command_at)
    assert gate.arm(True, 10., 100.) == (False, original_health)
    gate.receive_command((.2, 0., 0.), 10.)
    assert not gate.output(10., 100.).allowed
    assert gate.output(10., 100.).velocity == (0., 0., 0.)


@pytest.mark.parametrize('kind', ['odom', 'scan', 'localization', 'robot', 'behavior', 'mc'])
def test_preflight_retains_every_existing_health_evidence_requirement(kind):
    gate = permitted_live()
    gate.observe('execution_permit', execution_permit(seq=2, allow=False), 10.)
    gate.samples.pop(kind)
    status = ready_preflight(gate)
    assert status['preflight_ready'] is False
    assert status['preflight_block_reason'] == gate.health_reason(10., 100., check_execution_permit=False)
    assert status['preflight_block_reason'] != 'execution_not_permitted'


@pytest.mark.parametrize('kind,field,value,reason', [
    ('localization', 'calibration', {'extrinsics_verified': False, 'time_alignment_verified': True}, 'localization_not_verified'),
    ('localization', 'navigation_ready', False, 'localization_not_navigation_ready'),
    ('robot', 'control_source', 1, 'sdk_control_not_owned'),
    ('robot', 'hardware_emergency_status', 2, 'sdk_estop_active_or_unknown'),
    ('behavior', 'fault_latched', True, 'sdk_motion_adapter_not_ready'),
    ('mc', 'rate_ok', False, 'sdk_mc_missing_or_stale'),
])
def test_preflight_exposes_real_blocker_while_execution_permit_withheld(kind, field, value, reason):
    gate = permitted_live()
    gate.observe('execution_permit', execution_permit(seq=2, allow=False), 10.)
    gate.samples[kind][0][field] = value
    assert ready_preflight(gate) == {'preflight_ready': False, 'preflight_block_reason': reason}


@pytest.mark.parametrize('overrides,reason', [
    ({'monitor': None}, 'navigation_enabled_sdk_monitor_required'),
    ({'monitor_received': 8.9}, 'navigation_enabled_sdk_monitor_required'),
    ({'monitor_received': 10.1}, 'navigation_enabled_sdk_monitor_required'),
    ({'sdk_session': 'other'}, 'sdk_arm_service_unavailable'),
    ({'sdk_service_ready': False}, 'sdk_arm_service_unavailable'),
    ({'monitor': monitor_status()}, 'sdk_arm_state_unknown'),
    ({'monitor': {**monitor_status(), 'navigation_armed': True}}, 'sdk_already_armed'),
    ({'monitor': {**monitor_status(), 'navigation_armed': False, 'wall_time': 98.}}, 'navigation_enabled_sdk_monitor_required'),
    ({'monitor': {**monitor_status(), 'navigation_armed': False, 'service_prefix': '/other'}}, 'navigation_enabled_sdk_monitor_required'),
    ({'sdk_arm_pending': True}, 'sdk_arm_pending'),
])
def test_preflight_requires_current_unarmed_sdk_endpoint_and_service(overrides, reason):
    assert ready_preflight(permitted_live(), **overrides) == {
        'preflight_ready': False, 'preflight_block_reason': reason}


def test_preflight_already_armed_does_not_mutate_running_gate():
    gate = permitted_live()
    assert gate.arm(True, 10., 100.)[0]
    assert gate.receive_command((.2, 0., 0.), 10.)
    assert ready_preflight(gate) == {
        'preflight_ready': False, 'preflight_block_reason': 'gate_already_armed'}
    assert gate.armed and gate.output(10., 100.).allowed


def test_preflight_does_not_bypass_default_motion_disable():
    gate = healthy(CommandGate(GateConfig()))
    assert ready_preflight(gate) == {
        'preflight_ready': False, 'preflight_block_reason': 'live_motion_disabled'}
