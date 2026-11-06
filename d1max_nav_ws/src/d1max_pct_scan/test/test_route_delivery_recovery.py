"""Actual adapter/reference callbacks; no graph, SDK or synthetic map authority."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from rclpy.action import CancelResponse
from d1max_planning_interfaces.msg import RouteDeliveryReceipt
from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from test_bt_action_adapters import (formal_follow_before_first_reference, typed_follow,
    delivery_receipt, refresh, time_msg, worker_retired, follower_feedback, commit)
from test_continuous_reference_transport import state
from test_continuous_reference_node import local_state_from_pair, reset_debug


def formal(monkeypatch):
    node, clock, handle = typed_follow(monkeypatch)
    node.p.update(execution_architecture='task_follower_v1', readiness_timeout_s=30.,
                  keep_committed_route_on_local_failure=True)
    return node, clock, handle


def reject(node, clock, reason='prior_route_retirement_pending'):
    value = delivery_receipt(node, clock, status=RouteDeliveryReceipt.REJECTED)
    value.reason = reason
    assert node.on_route_delivery_receipt(value)
    return value


def advance(node, clock, dt=.21):
    clock.wall += dt
    clock.mono += dt
    refresh(node, clock)


@pytest.mark.parametrize('cancel_current', [False, True])
def test_exact_temporary_rejection_recovers_same_route_once_receiver_retires(monkeypatch, cancel_current):
    adapter, clock, handle, pair = formal_follow_before_first_reference(monkeypatch)
    adapter.p.update(execution_architecture='task_follower_v1', transport_mode='isolated_mock',
                     readiness_timeout_s=30.)
    adapter.reference_delivery_sequence = 2
    adapter.follow_tick()
    delivery = deepcopy(adapter.route_reference_pub.messages[-1])
    slot, snapshot = adapter.follow, adapter.snapshot
    output = []
    reference = ReferenceCallbacks(dict(session_id='session', map_version_id='map',
        expected_source_map_sha256='a'*64, expected_tomogram_sha256='c'*64,
        expected_conditioning_sha256='b'*64, body_height_m=.55,
        body_height_calibration_id='fixture', transport_mode='isolated_mock',
        floor_id='floor1', local_state_enabled=True, geometry_owner='local_executor',
        async_support_preparation=False),
        lambda kind, value: output.append((kind, deepcopy(value))),
        lambda: adapter.get_clock().now().nanoseconds, lambda: clock.mono)
    def receipt():
        return [v for k, v in output if k == 'route_receipt'][-1]
    def update():
        new_pair = state(t=clock.wall-10.)
        adapter.on_atomic_state(new_pair)
        adapter.on_local_state(local_state_from_pair(new_pair))
        reference.on_navigation(new_pair)
        reference.on_local_navigation(local_state_from_pair(new_pair))
        reference.on_map(dict(reference.context, valid=True,
            source_stamp_ns=reference.now_ns(), sources=[dict(sensor_id=i,
                integrated_stamp_ns=reference.now_ns()) for i in (0, 1)]))
    try:
        reference.on_navigation(pair)
        reference.on_local_navigation(local_state_from_pair(pair))
        reference.on_context_ack(reference.context)
        update()
        old = deepcopy(delivery)
        old.delivery_sequence = 1
        old.task_id = old.snapshot.task_id = 'old-task'
        old.route_id = old.snapshot.route_id = 'old-route'
        reference.on_route(old)
        assert reference.proposal is not None
        assert reference.route_identity(old) in reference.native_route_exposures
        cancel_old = deepcopy(old)
        cancel_old.active, cancel_old.delivery_sequence = False, 2
        reference.on_route(cancel_old)
        pending = reference.pending_route_retirement
        assert pending is not None and pending['required']

        reference.on_route(delivery)
        rejected = deepcopy(receipt())
        assert rejected.status == rejected.REJECTED
        assert rejected.reason == 'prior_route_retirement_pending'
        assert adapter.on_route_delivery_receipt(rejected)
        assert slot['cancel'] is None and not slot['committed']
        for _ in range(2):
            advance(adapter, clock)
            update()
            adapter.follow_tick()
            retry = deepcopy(adapter.route_reference_pub.messages[-1])
            assert retry.active and retry.delivery_sequence > delivery.delivery_sequence
            assert retry.route_hash == delivery.route_hash and retry.task_id == delivery.task_id
            assert reference.pending_route_retirement is pending
            reference.on_route(retry)
            assert receipt().status == receipt().REJECTED  # Retry does not bypass the gate.
            assert adapter.on_route_delivery_receipt(receipt())
            delivery = retry
        if cancel_current:
            assert adapter.cancel_follow(handle) == CancelResponse.ACCEPT
            reference.on_route(deepcopy(adapter.route_reference_pub.messages[-1]))
            retired = deepcopy(receipt())
            assert retired.status == retired.RETIRED
            assert not retired.native_retirement_required
            assert reference.pending_route_retirement is pending  # Not our old owner's barrier.
            assert adapter.on_route_delivery_receipt(retired)
            advance(adapter, clock)
            worker_retired(adapter, clock)
            adapter.follow_tick()
            assert handle.terminal == 'canceled'
            assert reference.on_debug(reset_debug(reference,
                SimpleNamespace(ns=reference.now_ns()), reference.cancel_generation))
            assert reference.pending_route_retirement is None
            reference.on_route(delivery)
            assert receipt().status == receipt().REJECTED
            assert reference.transport.route is None
            return
        # Only the original native cancellation ACK opens the old-owner barrier.
        assert reference.on_debug(reset_debug(reference,
            SimpleNamespace(ns=reference.now_ns()), reference.cancel_generation))
        assert reference.pending_route_retirement is None
        advance(adapter, clock)
        update()
        adapter.follow_tick()
        latest = deepcopy(adapter.route_reference_pub.messages[-1])
        reference.on_route(latest)
        assert receipt().status == receipt().ACCEPTED
        assert adapter.on_route_delivery_receipt(receipt())
        assert adapter.follow is slot and adapter.snapshot is snapshot and slot['committed']
        assert reference.transport.route.task_id == latest.task_id
        assert reference.proposal is not None
        assert reference.proposal.version.task_id == latest.task_id
        assert not adapter.on_route_delivery_receipt(rejected)
        assert not adapter.worker_cancel_pub.messages and handle.terminal is None
        assert all(m.active for m in adapter.route_reference_pub.messages)
        lease, status = follower_feedback(adapter, clock)
        lease.transport_mode = status.transport_mode = 'isolated_mock'
        assert adapter.on_task_lease(lease)
        assert adapter.on_local_execution_status(status)
        adapter.follow_tick()
        assert slot['wait_phase'] == 'following'
    finally:
        assert reference.close()


def test_retry_is_one_in_flight_and_old_receipts_cannot_commit_new_delivery(monkeypatch):
    node, clock, handle = formal(monkeypatch)
    old = reject(node, clock)
    slot, snapshot = node.follow, node.snapshot
    for _ in range(10): node.follow_tick()
    assert len(node.route_reference_pub.messages) == 1
    advance(node, clock)
    node.follow_tick()
    assert len(node.route_reference_pub.messages) == 2
    for _ in range(5):
        advance(node, clock)
        node.follow_tick()
    assert len(node.route_reference_pub.messages) == 2  # Missing response is not another rejection.
    assert not node.on_route_delivery_receipt(old)
    old.status = old.ACCEPTED
    assert not node.on_route_delivery_receipt(old)
    assert not slot['committed']
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    assert node.follow is slot and node.snapshot is snapshot and slot['committed']
    advance(node, clock, 31.)
    node.follow_tick()
    assert slot['cancel'] is None and handle.terminal is None
    assert len(node.route_reference_pub.messages) == 2


@pytest.mark.parametrize('in_flight', [False, True])
def test_cancel_during_retry_fences_active_late_ack_and_new_task(monkeypatch, in_flight):
    node, clock, handle = formal(monkeypatch)
    reject(node, clock)
    if in_flight:
        advance(node, clock)
        node.follow_tick()
    late = delivery_receipt(node, clock)
    assert node.cancel_follow(handle) == CancelResponse.ACCEPT
    count = len(node.route_reference_pub.messages)
    advance(node, clock)
    node.follow_tick()
    assert len(node.route_reference_pub.messages) == count
    assert not node.route_reference_pub.messages[-1].active
    assert not node.on_route_delivery_receipt(late)
    worker_retired(node, clock)
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock, active=False))
    assert handle.terminal == 'canceled' and node.follow is None
    advance(node, clock)
    worker_retired(node, clock)
    _, next_handle = commit(node, clock, task='next-task',
        completion_stamp=clock.wall+.1, completion_mono=clock.mono+.1)
    assert not node.on_route_delivery_receipt(late)
    assert node.follow['handle'] is next_handle and not node.follow['committed']


@pytest.mark.parametrize('reason', ['staging_context_quarantined', 'retired_committed_route_cannot_resume',
    'committed_route_not_ready', 'route_delivery_not_accepted', 'old_committed_route_not_retired'])
def test_nonretryable_rejection_reports_explicit_contract_failure_and_real_retirement(monkeypatch, reason):
    node, clock, handle = formal(monkeypatch)
    reject(node, clock, reason)
    assert node.follow['cancel'] is not None
    assert node.follow['cancel']['reason'] == 'reference_delivery_rejected:'+reason
    assert len(node.route_reference_pub.messages) == 2
    assert not node.route_reference_pub.messages[-1].active
    advance(node, clock)
    worker_retired(node, clock)
    assert handle.terminal is None  # Failure is not an invented retirement ACK.
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock, active=False))
    assert handle.terminal == 'aborted'
    assert node.executions[id(handle)]['future'].result().retirement_confirmed


@pytest.mark.parametrize('response', ['reject', 'missing'])
def test_delivery_retry_uses_original_existing_readiness_budget(monkeypatch, response):
    node, clock, handle = formal(monkeypatch)
    reject(node, clock)
    for _ in range(29):
        advance(node, clock, 1.)
        node.follow_tick()
        if response == 'reject': reject(node, clock)
        assert node.follow['cancel'] is None
    advance(node, clock, 1.)
    node.follow_tick()
    assert node.follow['cancel']['reason'] == 'reference_delivery_retry_timeout:prior_route_retirement_pending'
    assert not node.route_reference_pub.messages[-1].active
    assert handle.terminal is None
    assert len([v for v in node.route_reference_pub.messages if v.active]) <= 30


def test_duplicate_rejection_does_not_restart_retry_delay_or_episode(monkeypatch):
    node, clock, _ = formal(monkeypatch)
    receipt = reject(node, clock)
    advance(node, clock, .19)
    receipt.receipt_stamp = time_msg(clock.wall)
    assert not node.on_route_delivery_receipt(receipt)
    node.follow_tick()
    assert len(node.route_reference_pub.messages) == 1
    advance(node, clock, .02)
    node.follow_tick()
    assert len(node.route_reference_pub.messages) == 2


@pytest.mark.parametrize('reply', [RouteDeliveryReceipt.ACCEPTED, RouteDeliveryReceipt.REJECTED])
def test_immediate_retry_reply_sees_exact_new_binding_without_reentrant_resend(monkeypatch, reply):
    node, clock, handle = formal(monkeypatch)
    reject(node, clock)
    publish = node.route_reference_pub.publish
    def synchronous_reply(message):
        publish(message)
        value = delivery_receipt(node, clock, status=reply)
        value.reason = 'prior_route_retirement_pending' if reply == value.REJECTED else ''
        assert node.on_route_delivery_receipt(value)
    node.route_reference_pub.publish = synchronous_reply
    advance(node, clock)
    node.follow_tick()
    assert len(node.route_reference_pub.messages) == 2
    assert node.follow['committed'] is (reply == RouteDeliveryReceipt.ACCEPTED)
    assert node.follow['cancel'] is None and handle.terminal is None


def test_late_acceptance_cannot_reopen_expired_retry_episode(monkeypatch):
    node, clock, handle = formal(monkeypatch)
    reject(node, clock)
    advance(node, clock)
    node.follow_tick()
    acceptance = delivery_receipt(node, clock)
    advance(node, clock, 30.)
    assert node.on_route_delivery_receipt(acceptance)
    assert not node.follow['committed']
    assert node.follow['cancel']['reason'].startswith('reference_delivery_retry_timeout:')
    assert handle.terminal is None and not node.route_reference_pub.messages[-1].active


@pytest.mark.parametrize('block', ['pose', 'subscriber'])
def test_retry_cannot_create_fake_localization_or_subscriber_readiness(monkeypatch, block):
    node, clock, handle = formal(monkeypatch)
    reject(node, clock)
    if block == 'pose':
        def unavailable(): raise ValueError('waiting_local_pose')
        node.ready_local_task_identity = unavailable
    else:
        node.route_reference_pub.subscribers = 0
    advance(node, clock)
    node.follow_tick()
    assert len(node.route_reference_pub.messages) == 1
    assert not node.follow['committed'] and handle.terminal is None


def test_accepted_route_waiting_native_is_never_redelivered(monkeypatch):
    node, clock, handle = formal(monkeypatch)
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    for dt in (.21, 1., 30., 60.):
        advance(node, clock, dt)
        node.follow_tick()
    assert len(node.route_reference_pub.messages) == 1
    assert node.follow['committed'] and node.follow['cancel'] is None
    assert handle.terminal is None
