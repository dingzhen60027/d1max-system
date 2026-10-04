from dataclasses import replace
import math
import pytest

from d1max_navigation.mode_transaction import (
    ModeTransaction, ModeCapability, ModePolicy, RobotModeState, documented_capabilities)
from d1max_navigation.stop_verifier import StopPolicy, StopVerifier, MotionSample


def state(at=10., seq=0, mode=1, status=5, **overrides):
    return replace(RobotModeState('sdk', seq, at, mode, status, 1), **overrides)


def sample(at, raw=None, **overrides):
    return replace(MotionSample('sdk', 'clock', int(at*1e9) if raw is None else raw,
        at, at, (0., 0., 0.), (0., 0., 0.), True), **overrides)


def profiles():
    return {p.name: replace(p, mode_switch_verified=True, zero_hold_verified=True,
        move_verified=True, speed_mapping_verified=True, evidence_id='offline-test-fixture')
        for p in documented_capabilities().values()}


def transaction(**overrides):
    m = ModeTransaction(profiles(), stop_policy=StopPolicy(physical_acceptance_verified=True))
    kwargs = dict(transaction_id='tx', target_mode='stair', current_mode='general', sdk_session='sdk',
                  clock_epoch='clock', now=10., owned=True, initial_state=state())
    kwargs.update(overrides)
    assert m.begin(**kwargs)
    return m


def submit_mode(m):
    assert [e.kind for e in m.take_effects()] == ['revoke_permission', 'request_zero_hold']
    assert m.stop_submitted('tx', 10.01)
    for at in (10.05, 10.20, 10.36):
        m.observe_mc(sample(at), at)
    effects = m.take_effects()
    assert [e.kind for e in effects] == ['request_set_mode']
    assert effects[0].sdk_mode == 3
    assert m.mode_submitted('tx', 10.37)


def test_documented_api_is_not_verified_capability():
    m = ModeTransaction()
    assert m.begin(transaction_id='tx', target_mode='stair', current_mode='general', sdk_session='sdk',
                   clock_epoch='clock', now=10., owned=True, initial_state=state())
    assert m.phase == 'needs_review'
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']
    assert not m.status(10.)['motion_authorized']
    assert not m.status(10.)['adapter_wired']


def test_success_requires_stop_ack_and_two_new_actual_states():
    m = transaction(); submit_mode(m)
    # Matching states before ACK never complete a mode transaction.
    assert not m.robot_state(state(10.38, 1, 3, 7), 10.38)
    assert m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.40, now=10.40)
    assert not m.robot_state(state(10.42, 2, 3, 7), 10.42)
    assert not m.robot_state(state(10.42, 2, 3, 7), 10.42)
    assert m.robot_state(state(10.44, 3, 3, 7), 10.44)
    assert m.phase == 'ready'
    assert [e.kind for e in m.take_effects()] == ['mode_ready']
    assert m.mode_epoch == 1
    assert m.status(10.44)['mode_confirmed']
    assert not m.status(10.44)['motion_authorized']


def test_cancelled_pending_intent_cannot_dispatch_and_never_rolls_back():
    m = transaction()
    assert m.stop_submitted('tx', 10.01)
    for at in (10.05, 10.20, 10.36): m.observe_mc(sample(at), at)
    m.cancel()
    assert m.phase == 'needs_review'
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']
    assert not m.mode_submitted('tx', 10.37)
    assert not m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.4, now=10.4)
    assert not m.begin(transaction_id='new', target_mode='general', current_mode='stair',
                       sdk_session='sdk', clock_epoch='clock', now=11., owned=True)


def test_cannot_replace_active_transaction_or_replay_id():
    m = transaction()
    assert not m.begin(transaction_id='other', target_mode='general', current_mode='general',
        sdk_session='sdk', clock_epoch='clock', now=10.1, owned=True, initial_state=state(10.1))
    submit_mode(m)
    assert m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.4, now=10.4)
    m.robot_state(state(10.42, 1, 3, 7), 10.42); m.robot_state(state(10.44, 2, 3, 7), 10.44)
    assert not m.begin(transaction_id='tx', target_mode='general', current_mode='stair',
        sdk_session='sdk', clock_epoch='clock', now=10.5, owned=True, initial_state=state(10.5, 3, 3, 7))
    assert m.begin(transaction_id='next', target_mode='general', current_mode='stair',
        sdk_session='sdk', clock_epoch='clock', now=10.5, owned=True, initial_state=state(10.5, 3, 3, 7))
    assert m.mode_epoch == 2
    assert not m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.51, now=10.51)


@pytest.mark.parametrize('change', [dict(control_source=1), dict(software_emergency_status=2),
    dict(hardware_emergency_status=0), dict(sdk_session='different')])
def test_lost_authority_or_emergency_invalidates_transaction(change):
    m = transaction(); submit_mode(m)
    assert not m.robot_state(state(10.4, 1, **change), 10.4)
    assert m.phase == 'needs_review'
    assert not m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.41, now=10.41)


@pytest.mark.parametrize('change', [dict(initial_state=None), dict(owned=False),
    dict(initial_state=state(1.)), dict(initial_state=state(mode=3, status=7)),
    dict(initial_state=state(status=2))])
def test_initial_state_cannot_be_assumed(change):
    m = transaction(**change)
    assert m.phase == 'needs_review'
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']


def test_mismatching_actual_states_reset_two_sample_confirmation():
    m = transaction(); submit_mode(m)
    m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.4, now=10.4)
    assert not m.robot_state(state(10.42, 1, 3, 7), 10.42)
    assert not m.robot_state(state(10.43, 2, 1, 5), 10.43)
    assert not m.robot_state(state(10.44, 3, 3, 7), 10.44)
    assert m.robot_state(state(10.45, 4, 3, 7), 10.45)
    assert not m.robot_state(state(10.46, 5, 1, 5), 10.46)
    assert m.phase == 'needs_review'


def test_single_send_and_late_ack_timeout_never_retries():
    m = transaction(); submit_mode(m)
    assert not m.mode_submitted('tx', 10.38)
    m.tick(13.5)
    assert m.reason == 'mode_ack_timeout'
    assert not m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=13.6, now=13.6)
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']
    for at in (14., 20., 200.): m.tick(at)
    assert m.take_effects() == ()


def test_dispatch_preconditions_are_rechecked_and_uncertain_write_never_retries():
    m = transaction()
    m.take_effects(); assert m.stop_submitted('tx', 10.01)
    for at in (10.05, 10.20, 10.36): m.observe_mc(sample(at), at)
    assert m.dispatch_ready('tx', 10.37)
    assert not m.dispatch_ready('tx', 11.)
    assert not m.mode_submitted('tx', 11.)
    assert m.phase == 'needs_review'
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']
    m = transaction(); submit_mode(m); m.submission_failed('tx')
    assert m.phase == 'needs_review'
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']


def test_cancellation_of_ready_mode_revokes_without_posture_or_motion_commands():
    m = transaction(); submit_mode(m)
    m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.4, now=10.4)
    m.robot_state(state(10.42, 1, 3, 7), 10.42); m.robot_state(state(10.44, 2, 3, 7), 10.44)
    m.take_effects(); m.cancel()
    assert m.phase == 'needs_review'
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']


def test_source_gap_cannot_be_hidden_by_robot_state_or_reception_clock():
    m = transaction(); submit_mode(m)
    m.robot_state(state(10.7, 1), 10.7)
    m.tick(10.7)
    assert m.reason == 'mc_source_stale'


def test_stair_profile_does_not_inherit_general_zero_hold():
    ps = profiles(); ps['stair'] = replace(ps['stair'], zero_hold_verified=False)
    m = ModeTransaction(ps, stop_policy=StopPolicy(physical_acceptance_verified=True))
    assert m.begin(transaction_id='tx', target_mode='general', current_mode='stair', sdk_session='sdk',
        clock_epoch='clock', now=10., owned=True, initial_state=state(10., 0, 3, 7))
    assert m.reason == 'current_mode_zero_hold_unverified'
    assert [e.kind for e in m.take_effects()] == ['revoke_permission']


def stopped_verifier():
    v = StopVerifier()
    v.begin(transaction_id='tx', sdk_session='sdk', clock_epoch='clock', now=10.)
    assert v.mark_submitted('tx', 10.01)
    return v


def test_submission_and_real_measurement_are_distinct():
    v = StopVerifier()
    v.begin(transaction_id='tx', sdk_session='sdk', clock_epoch='clock', now=10.)
    for at in (10.05, 10.20, 10.36): assert not v.observe(sample(at), at)
    assert v.status(10.36)['nonzero_blocked'] and not v.status(10.36)['stop_submitted']
    assert v.mark_submitted('tx', 10.4)
    assert not v.status(10.4)['measured_stop_confirmed']
    for at in (10.45, 10.60, 10.76): v.observe(sample(at), at)
    assert v.status(10.76)['measured_stop_confirmed']
    assert not v.status(10.76)['physical_acceptance_verified']
    assert not v.status(11.1)['measured_stop_confirmed']


@pytest.mark.parametrize('change', [dict(source_clock_verified=False), dict(clock_epoch='different'),
    dict(sdk_session='old'), dict(source_time=8.), dict(source_time=11.),
    dict(raw_stamp_ns=0), dict(velocity_body=(math.nan,0,0)), dict(velocity_body=(0.,0.,.1)),
    dict(angular_velocity_body=(.1,0,0))])
def test_bad_or_moving_mc_never_proves_stopped(change):
    v = stopped_verifier()
    for at in (10.05, 10.20, 10.36): assert not v.observe(sample(at, **change), at)
    assert not v.confirmed(10.36)


def test_republished_mc_does_not_extend_proof_source_age():
    v = stopped_verifier()
    for at in (10.05, 10.20, 10.36): v.observe(sample(at), at)
    assert v.confirmed(10.36)
    for now in (10.4, 10.5, 10.7):
        assert not v.observe(sample(10.36, received_time=now), now)
    assert not v.confirmed(10.7)


def test_reception_as_source_cannot_sneak_through_incremented_raw_stamp():
    v = stopped_verifier()
    assert not v.observe(sample(10.05), 10.05)
    assert not v.observe(sample(10.20, raw=int(10.05*1e9)+1), 10.20)
    assert v.reason == 'stop_source_clock_mapping_discontinuous'
    assert not v.confirmed(10.20)


def test_mc_gap_resets_stationary_dwell_and_motion_invalidates_proof():
    v = stopped_verifier()
    for at in (10.05, 10.20, 10.36): v.observe(sample(at), at)
    assert v.confirmed(10.36)
    assert not v.observe(sample(10.4, velocity_body=(.1,0,0)), 10.4)
    assert not v.confirmed(10.4)
    assert not v.observe(sample(10.5), 10.5)
    assert not v.observe(sample(10.9), 10.9)
    assert not v.confirmed(10.9)


def test_physical_flags_require_explicit_evidence_and_valid_parameters():
    with pytest.raises(ValueError): ModeCapability('stair', 3, 7, move_verified=True)
    with pytest.raises(ValueError): ModePolicy(ack_timeout_s=float('nan'))
    with pytest.raises(ValueError): StopPolicy(dwell_seconds=0)
    with pytest.raises(ValueError): ModeCapability('wheel', 4, 8)


def test_stop_and_state_wait_are_bounded_without_sdk():
    m = transaction(); m.tick(14.); assert m.reason == 'stop_or_dispatch_timeout'
    m = transaction(); submit_mode(m)
    m.mode_ack(sdk_session='sdk', sdk_mode=3, callback_time=10.4, now=10.4)
    m.tick(14.5); assert m.reason == 'mode_state_confirmation_timeout'
