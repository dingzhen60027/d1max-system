from copy import deepcopy
import json
import math

import pytest
from rclpy.action import GoalResponse
from std_msgs.msg import String

from d1max_pct_scan.source_route_ros import from_message, to_message
from test_bt_action_adapters import harness, candidate, commit, compute_started, request, refresh, route


def test_codec_roundtrip_exact_hash_and_defensive_geometry_copy():
    wire = candidate()
    snapshot = from_message(wire)
    copied = to_message(snapshot,session_id='session2',task_id='task2',route_id='task2:'+snapshot.route_hash,
        epoch=99,seed_id='newseed',stamp=wire.path.header.stamp)
    assert copied.route_hash == wire.route_hash
    assert from_message(copied).encoded == snapshot.encoded
    wire.path.poses[0].pose.position.x = 123.
    assert snapshot.payload()['xyz'][0][0] == 0.


@pytest.mark.parametrize('field', ['route_hash','source_map_sha256','point_reference','layer_ids','point_floor_ids'])
def test_wire_geometry_or_semantics_tamper_is_rejected(field):
    wire = candidate()
    if field == 'layer_ids': wire.layer_ids[0] = 12
    elif field == 'point_floor_ids': wire.point_floor_ids[0] = 'floor2'
    elif field == 'point_reference': wire.point_reference = 'body'
    else: setattr(wire,field,'f'*64)
    with pytest.raises(ValueError):
        from_message(wire)


def test_path_only_private_result_can_never_complete_v2_compute(monkeypatch):
    node, clock = harness(monkeypatch)
    handle = compute_started(node)
    node.on_candidate(route())
    assert not node.candidates and handle.terminal is None


def test_foreign_source_hash_cannot_enter_even_with_valid_recomputed_snapshot(monkeypatch):
    node, clock = harness(monkeypatch)
    compute_started(node)
    value = from_message(candidate()).payload()
    value['source_map_sha256'] = 'd'*64
    value['geometry_evidence']['source_map_sha256'] = 'd'*64
    from d1max_pct_scan.source_route import RouteSnapshot
    altered = RouteSnapshot.create(value)
    wire = candidate()
    altered_wire = to_message(altered,session_id='session',task_id='',route_id='',
        epoch=1,seed_id='seed',stamp=wire.path.header.stamp)
    node.on_candidate(altered_wire)
    assert not node.candidates


def test_follow_matches_full_snapshot_not_only_visible_path(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    assert handle.request.schema_version == 2
    node.follow = None
    bad = deepcopy(handle.request)
    bad.snapshot.point_floor_ids[0] = 'floor2'
    assert node.follow_goal(bad) == GoalResponse.REJECT
    old = deepcopy(handle.request)
    old.schema_version = 1
    assert node.follow_goal(old) == GoalResponse.REJECT
    executable = deepcopy(handle.request)
    executable.execution_confirmed = True
    assert node.follow_goal(executable) == GoalResponse.REJECT


def test_typed_reference_active_and_cancel_have_same_immutable_task_route(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    delivery = node.route_reference_pub.messages[-1]
    assert delivery.active and delivery.schema_version == 2
    assert delivery.snapshot.preview_ready and not delivery.snapshot.execution_eligible
    assert from_message(delivery.snapshot).route_hash == delivery.route_hash
    node.cancel_follow(handle)
    cancel = node.route_reference_pub.messages[-1]
    assert not cancel.active and cancel.delivery_sequence > delivery.delivery_sequence
    assert cancel.task_id == delivery.task_id and cancel.route_hash == delivery.route_hash
    assert from_message(cancel.snapshot).encoded == from_message(delivery.snapshot).encoded


def test_compute_rejects_old_schema_and_invalid_yaw_contract(monkeypatch):
    node, _ = harness(monkeypatch)
    old = request(); old.schema_version = 0
    assert node.compute_goal(old) == GoalResponse.REJECT
    bad = request(); bad.goal_yaw_tolerance_rad = 0.
    assert node.compute_goal(bad) == GoalResponse.REJECT


def test_ordinary_same_context_correction_never_changes_committed_route(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node,clock)
    initial = deepcopy(node.snapshot['wire'])
    for index in range(10):
        clock.wall += .05; clock.mono += .05
        refresh(node,clock,xyz=(.01*index,0.,.55))
        node.navigation['alignment_stamp_sec'] = clock.wall
        node.follow_tick()
    assert node.snapshot['wire'] == initial
    assert len(node.route_reference_pub.messages) == 1
    assert handle.terminal is None


def test_goal_heading_is_frozen_in_snapshot_and_arrival_requires_measured_heading(monkeypatch):
    import test_bt_action_adapters as callbacks
    original = callbacks.request
    def heading_request(*args, **kwargs):
        goal = original(*args, **kwargs)
        goal.has_goal_yaw = True
        goal.goal.pose.orientation.z = math.sin(math.pi/4)
        goal.goal.pose.orientation.w = math.cos(math.pi/4)
        return goal
    monkeypatch.setattr(callbacks, 'request', heading_request)
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    source = node.snapshot['source_snapshot'].payload()
    assert source['has_goal_yaw'] and source['goal_yaw'] == pytest.approx(math.pi/2)
    stamp = node.follow['reference_stamp']
    for heading in (0., math.pi/2):
        for _ in range(4):
            clock.wall += .1; clock.mono += .1
            refresh(node, clock, xyz=(2., 0., .55))
            node.body_yaw = heading
            node.scan.update(active_reference=True, owner_reference_stamp=stamp)
            node.follow_tick()
            if node.follow['cancel'] is not None:
                break
        if heading == 0.:
            assert node.follow['cancel'] is None and handle.terminal is None
    assert node.follow['cancel']['reason'] == 'goal_reached'
