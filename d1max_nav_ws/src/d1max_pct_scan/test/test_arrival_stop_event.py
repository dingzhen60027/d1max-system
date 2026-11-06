"""Terminal stop is a bound event; current position/BT authority stay live."""
from copy import deepcopy

import pytest
from d1max_planning_interfaces.msg import StopReport
from test_bt_action_adapters import (formal_follow_before_first_reference, delivery_receipt,
    follower_feedback, time_msg, worker_retired)
from test_continuous_reference_transport import state
from test_continuous_reference_node import local_state_from_pair


def position(node, clock, *, global_update=True, x=2.):
    pair = state(t=clock.wall-10., x=x)
    if global_update: node.on_atomic_state(pair)
    node.on_local_state(local_state_from_pair(pair))


def setup_goal_stop(monkeypatch):
    node, clock, handle, _ = formal_follow_before_first_reference(monkeypatch)
    node.p.update(execution_architecture='task_follower_v1', transport_mode='live')
    node.follow_tick()
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    lease, status = follower_feedback(node, clock)
    assert node.on_task_lease(lease)
    position(node, clock)
    status.at_goal = status.request_stop = True
    assert node.on_local_execution_status(status)
    goal = deepcopy(node.local_goal_event)
    clock.wall += 1.1
    clock.mono += 1.1
    stop = StopReport(version=deepcopy(lease.version), execution_id=lease.execution_id,
        control_epoch=lease.control_epoch, sdk_session=lease.sdk_session,
        sdk_arm_generation=lease.sdk_arm_generation, stop_request_id=lease.confirmation_id,
        sequence=1, source_stamp=time_msg(clock.wall), nonzero_blocked=True,
        stop_submitted=True, measured_stop_confirmed=True, mc_clock_epoch='mc-fixture',
        mc_raw_stamp_ns=1_100_000_000, stationary_samples=30, stationary_duration_sec=1.,
        measured_linear_mps=0., measured_angular_radps=0., transport_mode='live',
        time_basis='source_delta_host_anchor_approximate')
    return node, clock, handle, lease, status, goal, stop


def terminal_lease(node, clock, lease, *, reason='goal_reached'):
    lease = deepcopy(lease)
    lease.sequence = (node.task_lease.sequence if node.task_lease else lease.sequence)+1
    lease.source_stamp, lease.valid_until = time_msg(clock.wall), time_msg(clock.wall+.35)
    lease.phase, lease.reason = 'terminal', reason
    lease.allowed, lease.revoked = False, True
    assert node.on_task_lease(lease)
    return lease


def received_stop(node, stop):
    node.on_stop_report(stop)
    return node.stop_report == stop


@pytest.mark.parametrize('order', ['stop_first', 'terminal_lease_first'])
def test_stop_event_survives_global_pause_and_delayed_delivery_without_restamping(monkeypatch, order):
    node, clock, handle, lease, status, goal, stop = setup_goal_stop(monkeypatch)
    original = deepcopy(stop)
    if order == 'stop_first':
        assert received_stop(node, stop)
    else:
        terminal_lease(node, clock, lease)
    # Pose becomes unavailable after the actual stop. No arrival is invented
    # from the goal event, planned curve or a terminal authority heartbeat.
    for _ in range(4):
        clock.wall += .2
        clock.mono += .2
        position(node, clock, global_update=False)
        terminal_lease(node, clock, lease)
        node.follow_tick()
        assert node.follow['cancel'] is None
    if order == 'terminal_lease_first':
        assert received_stop(node, stop)  # The original event is already 800 ms old.
    assert stop == original and node.stop_report == original
    position(node, clock)
    node.follow_tick()
    assert node.follow['cancel']['reason'] == 'goal_reached'
    assert node.local_goal_event == goal
    assert node.stop_report == original
    clock.wall += .1
    clock.mono += .1
    worker_retired(node, clock)
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock, active=False))
    result = node.executions[id(handle)]['future'].result()
    assert handle.terminal == 'succeeded' and result.success
    assert result.retirement_confirmed and result.physical_stop_confirmed


def test_repeated_goal_status_cannot_move_original_stop_boundary(monkeypatch):
    node, clock, _, lease, status, goal, stop = setup_goal_stop(monkeypatch)
    assert received_stop(node, stop)
    terminal_lease(node, clock, lease)
    clock.wall += .5
    clock.mono += .5
    position(node, clock)
    status.sequence += 1
    status.source_stamp = status.body_source_stamp = time_msg(clock.wall)
    status.body_valid_until = time_msg(clock.wall+.1)
    assert node.on_local_execution_status(status)
    assert node.local_goal_event == goal
    terminal_lease(node, clock, lease)
    node.follow_tick()
    assert node.follow['cancel']['reason'] == 'goal_reached'


@pytest.mark.parametrize('reason', ['task_tracking', 'stopped', 'user_cancel', 'sdk_authority_lost'])
def test_matching_stop_without_authoritative_goal_completion_cannot_succeed(monkeypatch, reason):
    node, clock, handle, lease, _, _, stop = setup_goal_stop(monkeypatch)
    assert received_stop(node, stop)
    terminal_lease(node, clock, lease, reason=reason)
    position(node, clock)
    node.follow_tick()
    assert node.follow['cancel'] is None and handle.terminal is None


@pytest.mark.parametrize('field,value', [('task_id', 'old-task'), ('route_hash', 'f'*64),
    ('localization_epoch', 2), ('localization_seed_id', 'old-seed'),
    ('execution_id', 'old-execution'), ('control_epoch', 2), ('sdk_session', 'old-sdk'),
    ('sdk_arm_generation', 1), ('stop_request_id', 'old-confirmation'), ('sequence', 0),
    ('transport_mode', 'isolated_mock'), ('source_stamp', time_msg(1_000.))])
def test_old_foreign_or_invalid_stop_cannot_become_current_goal_completion(monkeypatch, field, value):
    node, clock, handle, lease, _, _, stop = setup_goal_stop(monkeypatch)
    target = stop.version if field in ('task_id', 'route_hash', 'localization_epoch', 'localization_seed_id') else stop
    setattr(target, field, value)
    assert not node.on_stop_report(stop)
    terminal_lease(node, clock, lease)
    position(node, clock)
    node.follow_tick()
    assert node.follow['cancel'] is None and handle.terminal is None
    assert node.stop_report is None


@pytest.mark.parametrize('case', ['stale_lease', 'unsubmitted_stop', 'old_stop_boundary', 'position_mismatch'])
def test_immutable_stop_does_not_remove_live_authority_or_measured_position_contract(monkeypatch, case):
    node, clock, handle, lease, _, goal, stop = setup_goal_stop(monkeypatch)
    if case == 'unsubmitted_stop': stop.stop_submitted = False
    if case == 'old_stop_boundary': stop.source_stamp = time_msg(100.)
    assert received_stop(node, stop)
    terminal_lease(node, clock, lease)
    if case == 'stale_lease':
        clock.wall += .5
        clock.mono += .5
    clock.wall += .01
    clock.mono += .01
    position(node, clock, x=1.75 if case == 'position_mismatch' else 2.)
    node.follow_tick()
    if case == 'position_mismatch':
        assert node.follow['cancel']['reason'] == 'post_stop_goal_mismatch'
    else:
        assert node.follow['cancel'] is None
    assert handle.terminal is None


def test_user_cancel_wins_over_late_goal_stop_and_terminal_lease(monkeypatch):
    node, clock, handle, lease, _, _, stop = setup_goal_stop(monkeypatch)
    node.cancel_follow(handle)
    assert received_stop(node, stop)
    terminal_lease(node, clock, lease)
    position(node, clock)
    node.follow_tick()
    assert node.follow['cancel']['reason'] == 'user_cancel'
    clock.wall += .1
    clock.mono += .1
    worker_retired(node, clock)
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock, active=False))
    result = node.executions[id(handle)]['future'].result()
    assert handle.terminal == 'canceled' and not result.success


def test_stop_event_cannot_extend_existing_final_position_wait_budget(monkeypatch):
    node, clock, handle, lease, _, _, stop = setup_goal_stop(monkeypatch)
    assert received_stop(node, stop)
    clock.wall += 3.
    clock.mono += 3.
    terminal_lease(node, clock, lease)
    position(node, clock)
    node.follow_tick()
    assert node.follow['cancel']['reason'] == 'final_arrival_evidence_timeout'
    assert handle.terminal is None
