"""Original command lifetime and current sensor evidence are different clocks.

The 125 ms delivery uses the same command bytes as time zero. Only native
proofs contain a newer actual-body sample; callbacks must not restamp input.
"""
from copy import deepcopy

import pytest

from d1max_pct_scan.command_admission import CommandAdmission
from d1max_pct_scan.execution_safety import ExecutionSafety
from test_execution_safety import fixture, motion_proof, NOW, stamp
from test_execution_handoff import ready, ack
from d1max_pct_scan.execution_handoff import ExecutionHandoffAdmission


def staged(budget=.2):
    old, permit, demand, curve, *_ = fixture()
    core = ExecutionSafety(old.session, old.mode,
        braking_model_sha256=old.braking_model_sha256,
        stationary_policy=old.stationary_policy,
        command_pipeline_bound_s=budget)
    curve.check_begin = stamp(NOW + 64_000_000)
    curve.check_end = curve.body_source_stamp = stamp(NOW + 65_000_000)
    curve.valid_until = stamp(NOW + 180_000_000)
    assert core.on_permit(permit) and core.on_validation(curve)
    native = motion_proof(demand)
    native.check_begin = stamp(NOW + 64_000_000)
    native.check_end = native.body_source_stamp = stamp(NOW + 65_000_000)
    native.valid_until = stamp(NOW + 165_000_000)
    return core, permit, demand, curve, native


def test_two_20hz_phases_accept_original_command_without_restamping():
    core, _, demand, _, native = staged()
    queue = CommandAdmission(core)
    original = deepcopy(demand)
    assert queue.demand(demand, NOW).reason == 'waiting_command_sweep'
    result = queue.proof(native, NOW + 125_000_000)
    assert result.allowed and result.output.safety_checked
    assert result.output.source_stamp == original.source_stamp
    assert result.output.body_source_stamp == original.body_source_stamp
    assert result.output.valid_until == original.valid_until
    assert demand == original and not queue.pending
    # A repeated callback is not another 200 ms command or proof lease.
    assert not queue.proof(native, NOW + 201_000_000).allowed


def test_same_input_legacy_100ms_budget_expires_instead_of_borrowing_new_proof():
    core, _, demand, _, native = staged(.1)
    queue = CommandAdmission(core)
    assert queue.demand(demand, NOW).reason == 'waiting_command_sweep'
    assert not queue.proof(native, NOW + 125_000_000).allowed
    assert not queue.pending


@pytest.mark.parametrize('fault', ['deadline', 'original_body', 'native_expired',
    'negative', 'revoked', 'velocity', 'foreign_anchor', 'model'])
def test_pipeline_budget_does_not_extend_evidence_or_remove_safety_fences(fault):
    core, permit, demand, _, native = staged()
    if fault == 'deadline': demand.valid_until = native.demand_valid_until = stamp(NOW + 124_000_000)
    elif fault == 'original_body':
        demand.body_source_stamp = native.demand_body_source_stamp = stamp(NOW - 76_000_000)
    elif fault == 'native_expired': native.valid_until = stamp(NOW + 124_000_000)
    elif fault == 'negative': native.valid = False
    elif fault == 'revoked':
        permit = deepcopy(permit); permit.sequence += 1; permit.revoked = True
        assert core.on_permit(permit)
    elif fault == 'velocity': native.velocity.linear.x = .21
    elif fault == 'foreign_anchor': native.version.anchor_revision += 1
    else: native.braking_model_sha256 = 'd' * 64
    core.on_motion_validation(native)
    assert not core.admit(demand, NOW + 125_000_000).allowed


@pytest.mark.parametrize('kinematic_profile', ['forward_yaw', 'holonomic_xyyaw'])
def test_prepared_handoff_uses_bound_original_entry_but_never_restamps_it(kinematic_profile):
    handoff, incumbent, _, grant, prepared, native = ready()
    handoff.core.command_pipeline_bound_ns = 200_000_000
    handoff.core.kinematic_profile = kinematic_profile
    if kinematic_profile == 'holonomic_xyyaw':
        prepared.demand.velocity.linear.x = native.velocity.linear.x = -.1
        prepared.demand.velocity.linear.y = native.velocity.linear.y = .2
    prepared.entry_admission.valid_until = stamp(NOW + 180_000_000)
    native.check_begin = stamp(NOW + 64_000_000)
    native.check_end = native.body_source_stamp = stamp(NOW + 65_000_000)
    native.valid_until = stamp(NOW + 165_000_000)
    # The exact candidate curve has its own fresh native lease, independent
    # of the unchanged original entry/command source.
    curve = deepcopy(next(p for p in handoff.core.proofs.values()
        if p.trajectory_id == grant.candidate.trajectory_id))
    curve.sequence += 1
    curve.check_end = curve.body_source_stamp = stamp(NOW + 65_000_000)
    curve.valid_until = stamp(NOW + 180_000_000)
    native.trajectory_validation_sequence = curve.sequence
    assert handoff.core.on_validation(curve)
    assert handoff.core.on_motion_validation(native)
    # The transaction had already been constructed by ready() under the
    # compatibility profile. Re-admit this same original grant on a new gate
    # so candidate admission copies the configured 200 ms profile itself.
    handoff = ExecutionHandoffAdmission(handoff.core, enabled=True)
    assert handoff.on_ack(ack(incumbent), NOW)
    assert handoff.on_grant(grant, NOW)
    assert handoff.candidate.command_pipeline_bound_ns == 200_000_000
    assert handoff.candidate.kinematic_profile == kinematic_profile
    original = deepcopy(prepared)
    result = handoff.prepared_demand(prepared, NOW + 125_000_000)
    assert result.allowed and result.output.demand.source_stamp == original.demand.source_stamp
    assert result.output.entry_source_stamp == original.entry_source_stamp
    assert result.output.demand.valid_until == original.demand.valid_until
    assert handoff.applied == incumbent and prepared == original
    assert not handoff.prepared_demand(prepared, NOW + 201_000_000).allowed


@pytest.mark.parametrize('invalid', [True, False, .099, .501, float('nan'), float('inf')])
def test_invalid_pipeline_bound_is_not_silently_defaulted(invalid):
    with pytest.raises(ValueError, match='invalid_bound_command_pipeline'):
        ExecutionSafety('s', 'isolated_mock', braking_model_sha256='c' * 64,
            command_pipeline_bound_s=invalid)
