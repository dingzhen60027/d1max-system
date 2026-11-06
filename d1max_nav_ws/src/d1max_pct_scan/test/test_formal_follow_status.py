"""Pure typed workflow projection; no generated messages, ROS graph or SDK."""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace as Message

import pytest

from d1max_pct_scan.bt_follow_policy import FollowRecovery, FollowStage
from d1max_pct_scan.formal_follow_status import (
    FormalFollowContext, FormalFollowStatus, classify_formal_follow)


NOW = 1_800_000_000_123_456_789
CONTEXT = FormalFollowContext('session', 'task', 'route', 'a'*64, 'map', 2, 'seed', 'live')


def stamp(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return Message(sec=sec, nanosec=nanosec)


def inputs():
    receipt = Message(schema_version=1, session_id='session', task_id='task', route_id='route',
        route_hash='a'*64, delivery_sequence=7, active=True, status=1,
        native_retirement_required=False, native_retirement_confirmed=False, native_generation=0,
        source_stamp=stamp(NOW-600_000_000_000), receipt_stamp=stamp(NOW-599_000_000_000), reason='accepted')
    version = Message(schema_version=3, **{name: getattr(CONTEXT, name) for name in (
        'session_id', 'task_id', 'route_id', 'route_hash', 'map_version_id',
        'localization_epoch', 'localization_seed_id')})
    local = Message(schema_version=1, version=version, transport_mode='live', sequence=31,
        source_stamp=stamp(NOW-10_000_000), body_source_stamp=stamp(NOW-20_000_000),
        body_valid_until=stamp(NOW+80_000_000), trajectory_id=42, commit_sequence=9,
        ready=True, holding=False, software_retired=False, phase='following', reason='',
        at_goal=False, request_stop=False)
    return dict(context=CONTEXT, pose_ready=True, published=True, subscriber_ready=True,
        active_delivery=('session', 'task', 'route', 'a'*64, 7, NOW-600_000_000_000),
        active_receipt=receipt, local_status=local, local_status_received=9.9,
        now_ns=NOW, now_monotonic=10.)


def assert_waiting(result, reason=None):
    assert result.stage.phase != 'following' and not result.stage.terminal
    assert result.stage.budget
    assert result.evidence_source_ns == 0 and result.trajectory_id == -1
    if reason is not None:
        assert result.stage.reason == reason


def test_current_typed_status_uses_original_integer_ns_and_durable_delivery_without_mutating_inputs():
    args = inputs(); before = deepcopy(args)
    result = classify_formal_follow(**args)
    assert result == FormalFollowStatus(
        FollowStage('following', 'following_local_executor_trajectory'), NOW-20_000_000, 42)
    assert args == before
    with pytest.raises(FrozenInstanceError):
        result.trajectory_id = 0


@pytest.mark.parametrize('field,value,reason', [
    ('pose_ready', False, 'waiting_current_localization'),
    ('published', False, 'delivering_committed_route'),
    ('active_receipt', None, 'waiting_reference_acceptance'),
    ('active_delivery', None, 'waiting_reference_acceptance'),
    ('local_status', None, 'waiting_native_reference')])
def test_workflow_prerequisites_cannot_be_overridden_by_positive_local_status(field, value, reason):
    args = inputs(); args[field] = value
    assert_waiting(classify_formal_follow(**args), reason)


def test_subscriber_only_matters_before_initial_reference_and_does_not_revoke_accepted_route():
    args = inputs(); args['subscriber_ready'] = False
    assert classify_formal_follow(**args).stage.phase == 'following'
    args['published'] = False
    assert_waiting(classify_formal_follow(**args), 'waiting_reference_subscriber')


@pytest.mark.parametrize('field,value', [
    ('schema_version', 2), ('session_id', 'foreign'), ('task_id', 'old'), ('route_id', 'old'),
    ('route_hash', 'b'*64), ('delivery_sequence', 6), ('active', False), ('status', 3),
    ('source_stamp', stamp(NOW-600_000_000_001))])
def test_receipt_must_match_current_exact_delivery_even_with_current_local_status(field, value):
    args = inputs(); setattr(args['active_receipt'], field, value)
    assert_waiting(classify_formal_follow(**args), 'waiting_reference_acceptance')


def test_only_explicit_unowned_temporary_route_rejection_is_recoverable():
    args = inputs(); args['active_receipt'].status = 2
    args['active_receipt'].reason = 'prior_route_retirement_pending'
    result = classify_formal_follow(**args)
    assert result.stage == FollowStage('waiting_reference',
        'reference_delivery_rejected:prior_route_retirement_pending', 'reference')
    assert_waiting(result)


@pytest.mark.parametrize('pose_ready', [False, True])
def test_rejected_route_contract_is_explicit_even_during_localization_wait(pose_ready):
    args = inputs(); args['pose_ready'] = pose_ready
    args['active_receipt'].status = 2
    args['active_receipt'].reason = 'committed_route_identity_mismatch'
    result = classify_formal_follow(**args)
    assert result.stage.terminal
    assert result.stage.reason == 'reference_delivery_rejected:committed_route_identity_mismatch'
    assert result.evidence_source_ns == 0 and result.trajectory_id == -1


@pytest.mark.parametrize('field,value', [('native_retirement_required', True),
    ('native_retirement_confirmed', True), ('native_generation', 1)])
def test_temporary_reason_cannot_hide_native_ownership(field, value):
    args = inputs(); args['active_receipt'].status = 2
    args['active_receipt'].reason = 'prior_route_retirement_pending'
    setattr(args['active_receipt'], field, value)
    assert classify_formal_follow(**args).stage.terminal


def test_geometry_ready_preview_is_not_movement_or_a_failed_trajectory():
    args = inputs(); args['local_status'].phase = 'preview'
    result = classify_formal_follow(**args)
    assert result.stage == FollowStage('waiting_execution', 'trajectory_ready_for_execution')
    assert result.evidence_source_ns == 0 and result.trajectory_id == -1


@pytest.mark.parametrize('field,value', [('holding', True), ('software_retired', True),
    ('trajectory_id', -1), ('commit_sequence', 0), ('ready', False)])
def test_preview_label_alone_does_not_claim_geometry_admission(field, value):
    args = inputs(); args['local_status'].phase = 'preview'
    setattr(args['local_status'], field, value)
    result = classify_formal_follow(**args)
    assert result.stage.phase == 'recovering_local_trajectory'


@pytest.mark.parametrize('field,value', [
    ('schema_version', 2), ('session_id', 'foreign'), ('task_id', 'old'), ('route_id', 'old'),
    ('route_hash', 'b'*64), ('map_version_id', 'old'), ('localization_epoch', 3),
    ('localization_seed_id', 'old')])
def test_foreign_local_context_cannot_supply_following_or_failure_reason(field, value):
    args = inputs(); setattr(args['local_status'].version, field, value)
    args['local_status'].reason = 'foreign_failure'
    assert_waiting(classify_formal_follow(**args), 'waiting_current_local_executor_status')


@pytest.mark.parametrize('field,value', [('schema_version', 2), ('transport_mode', 'isolated_mock')])
def test_local_status_schema_and_transport_are_part_of_identity(field, value):
    args = inputs(); setattr(args['local_status'], field, value)
    assert_waiting(classify_formal_follow(**args), 'waiting_current_local_executor_status')


@pytest.mark.parametrize('fault', [
    'future_source', 'old_source', 'zero_source', 'bad_stamp', 'future_receipt', 'old_receipt',
    'nan_receipt', 'infinite_receipt', 'expired_body', 'body_after_source', 'extended_body_lease'])
def test_original_source_body_and_receipt_deadlines_cannot_be_replaced_by_a_heartbeat(fault):
    args = inputs(); local = args['local_status']
    if fault == 'future_source': local.source_stamp = stamp(NOW+1)
    elif fault == 'old_source': local.source_stamp = stamp(NOW-500_000_001)
    elif fault == 'zero_source': local.source_stamp = stamp(0)
    elif fault == 'bad_stamp': local.source_stamp.nanosec = 1_000_000_000
    elif fault == 'future_receipt': args['local_status_received'] = 10.000001
    elif fault == 'old_receipt': args['local_status_received'] = 9.499999
    elif fault == 'nan_receipt': args['local_status_received'] = float('nan')
    elif fault == 'infinite_receipt': args['local_status_received'] = -float('inf')
    elif fault == 'expired_body': local.body_valid_until = stamp(NOW-1)
    elif fault == 'body_after_source': local.body_source_stamp = stamp(NOW)
    else: local.body_valid_until = stamp(NOW+80_000_001)
    assert_waiting(classify_formal_follow(**args), 'waiting_current_local_executor_status')


def test_exact_body_and_receipt_deadlines_are_inclusive_then_expire_without_source_refresh():
    args = inputs(); local = args['local_status']
    local.source_stamp = local.body_source_stamp = stamp(NOW-100_000_000)
    local.body_valid_until = stamp(NOW)
    args['local_status_received'] = 9.5
    result = classify_formal_follow(**args)
    assert result.stage.phase == 'following' and result.evidence_source_ns == NOW-100_000_000
    args['now_ns'] += 1
    assert_waiting(classify_formal_follow(**args), 'waiting_current_local_executor_status')


@pytest.mark.parametrize('phase', ['failed_optimization', 'failed_reference_search',
    'failed_reference_start_occupied', 'EMERGENCY_STOP', 'failed_reference_geometry', 'holding'])
def test_ordinary_local_reasons_remain_recoverable_beyond_legacy_budget(phase):
    args = inputs(); args['local_status'].ready = False; args['local_status'].phase = phase
    recovery = FollowRecovery({'keep_committed_route_on_local_failure': True})
    for tick in (0., 30., 120., 600.):
        result = classify_formal_follow(**args)
        assert_waiting(result, phase)
        assert recovery.observe(result.stage, tick, evidence_stamp=result.evidence_source_ns) == ''


@pytest.mark.parametrize('field,value', [('ready', False), ('holding', True),
    ('software_retired', True), ('trajectory_id', -1), ('commit_sequence', 0)])
def test_non_executable_typed_status_cannot_be_overridden_by_display_fields(field, value):
    args = inputs(); local = args['local_status']; setattr(local, field, value)
    local.reason = 'waiting_recheck'
    local.spline_visual_valid = local.active_reference = True  # Deliberately ignored.
    assert_waiting(classify_formal_follow(**args), 'waiting_recheck')


def test_new_receipt_time_or_sequence_cannot_turn_duplicate_source_into_new_recovery_evidence():
    args = inputs(); first = classify_formal_follow(**args)
    args['now_ns'] += 10_000_000; args['now_monotonic'] += .01
    args['local_status_received'] = args['now_monotonic']; args['local_status'].sequence += 1
    assert classify_formal_follow(**args) == first


def test_new_reporting_heartbeat_cannot_count_the_same_body_source_twice():
    args = inputs(); first = classify_formal_follow(**args)
    args['now_ns'] += 10_000_000; args['now_monotonic'] += .01
    args['local_status_received'] = args['now_monotonic']
    args['local_status'].source_stamp = stamp(args['now_ns'])
    args['local_status'].sequence += 1
    assert classify_formal_follow(**args) == first


def test_goal_flags_never_produce_arrival_or_stop_authority():
    args = inputs(); args['local_status'].at_goal = args['local_status'].request_stop = True
    result = classify_formal_follow(**args)
    assert result.stage == FollowStage('following', 'following_local_executor_trajectory')
    assert set(result.__dataclass_fields__) == {'stage', 'evidence_source_ns', 'trajectory_id'}


def test_context_is_immutable_and_new_context_does_not_reuse_old_task_feedback():
    args = inputs(); args['context'] = replace(CONTEXT, task_id='new-task')
    assert_waiting(classify_formal_follow(**args), 'waiting_reference_acceptance')
    with pytest.raises(FrozenInstanceError):
        CONTEXT.task_id = 'new-task'
