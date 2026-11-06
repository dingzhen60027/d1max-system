"""Passive formal task/reference handoffs using production callbacks.

Generated messages and fake publishers/Action handles exercise the existing
adapter and reference manager without ROS initialization, a graph or SDK.
Route delivery, map admission and Native collision acceptance remain distinct:
these tests prove software handoffs, never permission to move.
"""
from copy import deepcopy

from rclpy.action import CancelResponse

from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from d1max_pct_scan.continuous_reference_transport import nanoseconds
from test_bt_action_adapters import (
    formal_follow_before_first_reference, worker_retired)
from test_continuous_reference_node import local_state_from_pair


class NoScanDiagnosticReads(dict):
    """Fail if a formal workflow decision consults downstream display JSON."""

    def get(self, *args, **kwargs):
        raise AssertionError('formal handoff must not read SCAN display JSON')

    def __getitem__(self, key):
        raise AssertionError('formal handoff must not read SCAN display JSON')


def first_handoff(monkeypatch):
    adapter, clock, handle, pair = formal_follow_before_first_reference(monkeypatch)
    adapter.p.update(execution_architecture='task_follower_v1', transport_mode='isolated_mock')
    adapter.scan = NoScanDiagnosticReads()

    def forbidden_local_ready(_context):
        raise AssertionError('formal delivery must not await SCAN JSON local_ready')

    adapter.local_ready = forbidden_local_ready
    adapter.follow_tick()
    assert adapter.follow['published'] and not adapter.follow['committed']
    assert len(adapter.route_reference_pub.messages) == 1
    delivery = deepcopy(adapter.route_reference_pub.messages[0])

    output = []
    params = dict(session_id='session', map_version_id='map',
        expected_source_map_sha256='a'*64, expected_tomogram_sha256='c'*64,
        expected_conditioning_sha256='b'*64, body_height_m=.55,
        body_height_calibration_id='fixture', transport_mode='isolated_mock',
        floor_id='floor1', local_state_enabled=True, geometry_owner='local_executor',
        async_support_preparation=False)
    reference = ReferenceCallbacks(params,
        lambda kind, value: output.append((kind, deepcopy(value))),
        lambda: adapter.get_clock().now().nanoseconds, lambda: clock.mono)
    reference.on_navigation(pair)
    assert reference.on_local_navigation(local_state_from_pair(pair))
    assert reference.context is not None and not reference.context_ready
    assert reference.map_report is None and not reference.map_fresh()

    reference.on_route(delivery)
    receipt = messages(output, 'route_receipt')[-1]
    assert receipt.status == receipt.ACCEPTED and receipt.active
    assert receipt.delivery_sequence == delivery.delivery_sequence
    assert receipt.source_stamp == delivery.source_stamp
    assert reference.transport.route == delivery
    assert reference.transport.core is not None
    assert reference.proposal is None and reference.transport.pending is None
    assert not reference.native_route_exposures
    assert not messages(output, 'proposal') and not messages(output, 'support')
    assert adapter.on_route_delivery_receipt(receipt)
    assert adapter.follow['committed']
    assert adapter.follow['wait_phase'] == 'recovering_local_trajectory'
    assert adapter.follow['wait_reason'] == 'waiting_native_reference'
    assert adapter.local_execution_status is None and adapter.task_lease is None
    assert adapter.follow['cancel'] is None and handle.terminal is None
    return adapter, clock, handle, reference, output, delivery


def messages(output, kind):
    return [value for key, value in output if key == kind]


def map_report(reference, now_ns):
    return dict(reference.context, valid=True, source_stamp_ns=now_ns,
        sources=[dict(sensor_id=sensor, integrated_stamp_ns=now_ns) for sensor in (0, 1)])


def test_pre_map_formal_delivery_receipt_and_cancel_retire_without_nonexistent_native_ack(monkeypatch):
    adapter, clock, handle, reference, output, delivery = first_handoff(monkeypatch)
    slot, snapshot = adapter.follow, adapter.snapshot

    # Re-delivery only reaffirms the reference manager's durable ownership
    # disposition. It cannot create a proposal or a new sensor/motion lease.
    reference.on_route(deepcopy(delivery))
    duplicate = messages(output, 'route_receipt')[-1]
    assert duplicate.source_stamp == delivery.source_stamp
    assert duplicate.delivery_sequence == delivery.delivery_sequence
    assert not adapter.on_route_delivery_receipt(duplicate)
    assert adapter.follow is slot and adapter.snapshot is snapshot
    assert not messages(output, 'proposal') and not reference.native_route_exposures

    # Keep the strict diagnostic sentinel installed through cancellation:
    # typed retirement must not consume a legacy generation or JSON ACK.
    assert adapter.cancel_follow(handle) == CancelResponse.ACCEPT
    cancellation = deepcopy(adapter.route_reference_pub.messages[-1])
    assert not cancellation.active and cancellation.delivery_sequence > delivery.delivery_sequence
    assert cancellation.route_hash == delivery.route_hash
    reference.on_route(cancellation)
    retirement = messages(output, 'route_receipt')[-1]
    assert retirement.status == retirement.RETIRED and not retirement.active
    assert retirement.source_stamp == cancellation.source_stamp
    assert retirement.delivery_sequence == cancellation.delivery_sequence
    assert not retirement.native_retirement_required
    assert not retirement.native_retirement_confirmed and retirement.native_generation == 0
    assert reference.pending_route_retirement is None and reference.transport.route is None
    assert not messages(output, 'cancel_reference') and not messages(output, 'proposal')
    assert adapter.native_cancel_ack is None
    assert adapter.on_route_delivery_receipt(retirement)
    assert handle.terminal is None  # Private global work must still retire.

    clock.wall += .1
    clock.mono += .1
    worker_retired(adapter, clock)
    adapter.follow_tick()
    result = adapter.executions[id(handle)]['future'].result()
    assert handle.terminal == 'canceled' and result.retirement_confirmed
    assert not result.success and not result.physical_stop_confirmed
    assert adapter.follow is None and adapter.snapshot is None and not adapter.quarantine

    # A queued reliable active delivery cannot resurrect this canceled task,
    # even when the matching map ACK subsequently arrives.
    assert reference.on_context_ack(deepcopy(reference.context))
    assert reference.on_map(map_report(reference, reference.now_ns()))
    reference.on_route(deepcopy(delivery))
    reference.tick()
    assert reference.transport.route is None and reference.transport.core is None
    assert not reference.native_route_exposures
    assert not messages(output, 'proposal') and not messages(output, 'cancel_reference')
    assert reference.close()


def test_pre_map_acceptance_still_requires_exact_ack_and_fresh_sensor_pair_for_native_prepare(monkeypatch):
    adapter, _clock, handle, reference, output, delivery = first_handoff(monkeypatch)
    slot, snapshot = adapter.follow, adapter.snapshot
    original_source = nanoseconds(delivery.source_stamp)
    now_ns = reference.now_ns()

    # A map report alone, or a different context's ACK, cannot admit native
    # work. The task remains stored while the actual context handshake waits.
    report = map_report(reference, now_ns)
    assert reference.on_map(report)
    wrong_ack = dict(reference.context, sequence=reference.context['sequence']+1)
    assert not reference.on_context_ack(wrong_ack)
    reference.tick()
    assert not reference.map_fresh() and not messages(output, 'proposal')

    # Even the correct context ACK is insufficient when one original sensor
    # source is stale; a fresh relay/status timestamp does not replace it.
    stale = deepcopy(report)
    stale['sources'][1]['integrated_stamp_ns'] -= 600_000_000
    assert reference.on_map(stale)
    assert reference.on_context_ack(deepcopy(reference.context))
    reference.tick()
    assert not reference.map_fresh() and reference.proposal is None
    assert not messages(output, 'proposal') and not reference.native_route_exposures
    adapter.follow_tick()
    assert adapter.follow is slot and adapter.snapshot is snapshot
    assert slot['wait_reason'] == 'waiting_native_reference' and slot['cancel'] is None

    # Only genuine current map/context evidence permits the SAME stored route
    # to prepare support and a typed proposal. That is not a SCAN verdict,
    # a local geometry commit, a task lease or an arrival certificate.
    assert reference.on_map(deepcopy(report)) and reference.map_fresh()
    reference.tick()
    proposals, supports = messages(output, 'proposal'), messages(output, 'support')
    assert len(proposals) == len(supports) == 1
    proposal = proposals[0]
    assert proposal == reference.proposal
    assert proposal.version.task_id == delivery.task_id
    assert proposal.version.route_id == delivery.route_id
    assert proposal.version.route_hash == delivery.route_hash
    assert proposal.reference.path.header.frame_id == 'd1max_loc_odom'
    assert proposal.reference.point_reference == 'body_center'
    assert supports[0].version == proposal.version
    assert reference.transport.route == delivery
    assert nanoseconds(reference.transport.route.source_stamp) == original_source
    assert reference.transport.accepted is None and reference.local_commit is None
    assert reference.permit is None
    assert reference.native_route_exposures == {(delivery.task_id, delivery.route_id, delivery.route_hash)}
    reference.tick()
    adapter.follow_tick()
    assert len(messages(output, 'proposal')) == 1
    assert adapter.follow is slot and adapter.snapshot is snapshot
    assert slot['wait_phase'] == 'recovering_local_trajectory' and slot['cancel'] is None
    assert adapter.local_execution_status is None and adapter.task_lease is None
    assert adapter.stop_report is None and handle.terminal is None
    assert len(adapter.route_reference_pub.messages) == 1
    assert reference.close()
