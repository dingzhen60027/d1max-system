"""BT capability separation uses real adapters and generated messages only."""
from copy import deepcopy

import pytest

from test_bt_action_adapters import harness, commit, compute_started, worker_retired
from test_continuous_reference_transport import state
from test_system_v2_reference_admission import local_message


def wire(clock, *, x=0., correction=0., usable=True):
    return state(t=clock.wall-10., x=x, correction=correction, usable=usable)


def enable_atomic(node):
    node.p.update(pipeline_contract='single_floor_v3', map_version_id='map',
                  local_state_enabled=True, transport_mode='isolated_mock')


def feed(node, clock, *, global_sample=True, local_sample=True, x=0., correction=0.):
    message = wire(clock, x=x, correction=correction)
    if global_sample: node.on_atomic_state(message)
    if local_sample: node.on_local_state(local_message(message))


def accepted_scan(node, clock):
    slot = node.follow
    node.scan.update(received_at_unix=clock.wall, sensor_ready=True,
        native_map_context_ready=True, native_map_context_fault='',
        localization_epoch=1, localization_seed_id='seed', active_reference=True,
        spline_visual_valid=True, local_debug_phase='accepted',
        owner_task_id=slot['handle'].request.task_id, owner_route_id=node.snapshot['route_id'],
        owner_route_hash=node.snapshot['source_snapshot'].route_hash,
        owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=slot['reference_stamp_ns'])
    node.received['scan'] = clock.mono


def running(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    enable_atomic(node); feed(node, clock); accepted_scan(node, clock)
    node.follow_tick()
    assert node.follow['wait_phase'] == 'following'
    return node, clock, handle


@pytest.mark.parametrize('global_failure', ['soft_gap', 'expired', 'invalid_packet'])
def test_global_unavailable_leaves_local_following_and_route_identity_intact(monkeypatch, global_failure):
    node, clock, handle = running(monkeypatch)
    route = deepcopy(node.snapshot['wire'])
    context = node.follow['context']
    count = len(node.route_reference_pub.messages)
    clock.wall += .6 if global_failure == 'expired' else .02
    clock.mono += .6 if global_failure == 'expired' else .02
    feed(node, clock, global_sample=False)
    if global_failure != 'expired':
        bad = wire(clock, usable=global_failure != 'soft_gap')
        if global_failure == 'soft_gap': bad.reason = 'global_matching_temporarily_unavailable'
        else: bad.global_odometry.header.frame_id = 'invalid_map'
        node.on_atomic_state(bad)
    accepted_scan(node, clock)
    node.tick()
    health = node.health_pub.messages[-1]
    assert health.local_control_ready and not health.hard_fault
    # An invalid packet is rejected, so the still fresh prior global is valid.
    assert health.global_planning_ready is (global_failure == 'invalid_packet')
    assert node.follow['wait_phase'] == 'following' and not node.follow['cancel']
    assert node.follow['context'] == context and node.snapshot['wire'] == route
    assert handle.terminal is None and len(node.route_reference_pub.messages) == count
    assert not node.worker_cancel_pub.messages


def test_fresh_global_does_not_substitute_for_missing_local_control_evidence(monkeypatch):
    node, clock = harness(monkeypatch); enable_atomic(node)
    feed(node, clock, local_sample=False)
    node.tick()
    health = node.health_pub.messages[-1]
    assert health.ready and health.global_planning_ready
    assert not health.local_control_ready and not health.hard_fault
    with pytest.raises(ValueError, match='waiting_fresh_local_navigation_state'):
        node.ready_local_identity()


def test_compute_remains_global_gated_even_with_fresh_local_state(monkeypatch):
    node, clock = harness(monkeypatch)
    enable_atomic(node); feed(node, clock, global_sample=False)
    handle = compute_started(node)
    before = len(node.goal3d_pub.messages)
    worker_retired(node, clock)
    node.compute_tick()
    assert node.compute['wait_phase'] == 'waiting_localization'
    assert not node.compute['dispatched'] and len(node.goal3d_pub.messages) == before
    assert handle.terminal is None


def test_local_at_endpoint_without_global_measurement_cannot_claim_arrival(monkeypatch):
    node, clock, handle = running(monkeypatch)
    for _ in range(8):
        clock.wall += .1; clock.mono += .1
        feed(node, clock, global_sample=False, x=2.)
        bad = wire(clock, usable=False); bad.reason = 'global_matching_temporarily_unavailable'
        node.on_atomic_state(bad); accepted_scan(node, clock); node.follow_tick()
    assert node.body is None and node.follow['cancel'] is None
    assert node.follow['wait_phase'] == 'following' and handle.terminal is None
    assert node.arrival.samples == 0


def test_local_soft_gap_recovers_same_follow_task_no_route_redelivery(monkeypatch):
    node, clock, handle = running(monkeypatch)
    slot = node.follow
    route = deepcopy(node.snapshot['wire'])
    count = len(node.route_reference_pub.messages)
    clock.wall += .02; clock.mono += .02
    bad = local_message(wire(clock, usable=False)); bad.reason = 'imu_temporarily_unavailable'
    node.on_local_state(bad); accepted_scan(node, clock); node.tick()
    assert not node.health_pub.messages[-1].local_control_ready
    assert slot['wait_phase'] == 'waiting_localization' and slot['cancel'] is None
    for _ in range(4):
        clock.wall += .2; clock.mono += .2
        feed(node, clock); accepted_scan(node, clock); node.tick()
    assert slot is node.follow and slot['wait_phase'] == 'following'
    assert node.snapshot['wire'] == route and len(node.route_reference_pub.messages) == count
    assert handle.terminal is None and not node.worker_cancel_pub.messages


@pytest.mark.parametrize('hard_source', ['local', 'global'])
def test_hard_reset_does_revoke_follow_and_is_not_hidden_by_other_healthy_stream(monkeypatch, hard_source):
    node, clock, handle = running(monkeypatch)
    clock.wall += .02; clock.mono += .02
    bad = wire(clock, usable=False); bad.reason = 'lio_reset'
    if hard_source == 'local': node.on_local_state(local_message(bad))
    else: node.on_atomic_state(bad)
    node.tick()
    assert node.health_pub.messages[-1].hard_fault
    assert not node.health_pub.messages[-1].local_control_ready
    assert node.follow['cancel'] is not None
    assert node.execution_permit is None and node.stop_report is None
    assert handle.terminal is None  # retirement/physical stop still separate


def test_normal_map_correction_does_not_mutate_route_or_restart_follow(monkeypatch):
    node, clock, handle = running(monkeypatch)
    slot = node.follow; route = deepcopy(node.snapshot['wire'])
    count = len(node.route_reference_pub.messages)
    for i in range(1, 9):
        clock.wall += .1; clock.mono += .1
        feed(node, clock, x=i*.03, correction=i*.04)
        accepted_scan(node, clock); node.tick()
    assert node.follow is slot and slot['cancel'] is None
    assert slot['wait_phase'] == 'following' and node.snapshot['wire'] == route
    assert len(node.route_reference_pub.messages) == count and handle.terminal is None


def test_timer_phase_hold_does_not_reset_follow_workflow_or_authorize_motion(monkeypatch):
    node, clock, handle = running(monkeypatch)
    slot = node.follow
    route = deepcopy(node.snapshot['wire'])
    clock.wall += .101; clock.mono += .101
    accepted_scan(node, clock)
    node.tick()
    assert not node.health_pub.messages[-1].local_control_ready
    assert node.ready_local_task_identity() == slot['context']
    with pytest.raises(ValueError, match='waiting_fresh_local_navigation_state'):
        node.ready_local_identity()
    assert node.follow is slot and slot['wait_phase'] == 'following'
    assert slot['cancel'] is None and node.snapshot['wire'] == route
    assert handle.terminal is None
