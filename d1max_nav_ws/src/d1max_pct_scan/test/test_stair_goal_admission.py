"""Production goal callbacks with measured stair surfaces, without ROS startup."""
from types import MethodType

import numpy as np
import pytest
from geometry_msgs.msg import PointStamped

from d1max_pct_scan.live_global_planner import LiveGlobalPlanner
from test_global_mission_lifecycle import lifecycle
from test_navigation_surfaces import stair_model, regularized_stair_model


def stair_owner(monkeypatch):
    node, clock, actions, published, visuals, attempts = lifecycle(monkeypatch)
    resolver, _ = stair_model()
    node.bridge = resolver.bridge
    node.tomogram = resolver.tomogram
    node.route_settings = resolver.settings
    node.surface_resolver = resolver
    node.body = np.array([.6, 0., .4])  # measured floor -.1, body offset .5
    node.p.update(body_height_min_m=.4, body_height_max_m=.6)
    node._new_goal = MethodType(LiveGlobalPlanner._new_goal, node)
    return node, clock, actions, published, visuals, attempts


def submit(node, clock, xyz, frame=None):
    clock.wall += .1
    clock.mono += .1
    message = PointStamped()
    message.header.frame_id = frame or node.bridge.source_frame
    message.header.stamp = node.get_clock().now().to_msg()
    message.point.x, message.point.y, message.point.z = map(float, xyz)
    node._new_goal(message, kind='3d')
    return message


@pytest.mark.parametrize('xyz,region', [([0., -.4, 1.], 'stair_lower'),
    ([0., 0., 1.5], 'landing'), ([0., -.4, 2.], 'stair_upper')])
def test_stair_goal_reaches_native_request_with_selected_ground(monkeypatch, xyz, region):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    submit(node, clock, xyz)
    assert node.pause.intent is not None, node.reason
    assert node.pause.intent.floor_id == region
    assert len(attempts) == 1
    start, goal, _ = attempts[-1]
    assert start['xyz'] == pytest.approx([.6, 0., 0.])
    assert goal['xyz'] == pytest.approx(xyz)
    assert node.current.start_body_xyz == pytest.approx([.6, 0., .4])


def test_new_goal_from_midstairs_uses_current_body_not_old_floor(monkeypatch):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    submit(node, clock, [0., 0., 1.5])
    old_generation = node.current.generation
    node.body = np.array([0., -.4, 1.5])  # real stair support z=1, not floor1
    submit(node, clock, [.6, 0., 3.07])
    assert node.pause.intent.floor_id == 'floor2', node.reason
    assert node.current.generation > old_generation
    assert attempts[-1][0]['xyz'] == pytest.approx([0., -.4, 1.])
    assert attempts[-1][1]['xyz'] == pytest.approx([.6, 0., 3.])
    # Retired result cannot replace the new request even if it arrived late.
    LiveGlobalPlanner._finish(node, {'kind': 'planned', 'generation': old_generation,
                                   'result': {'path_xyz': [[0., 0., 0.], [1., 1., 1.]]}})
    assert node.current.generation == attempts[-1][2]


def test_moving_uncommitted_start_on_stairs_retries_same_intent(monkeypatch):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    submit(node, clock, [0., 0., 1.5])
    intent = node.pause.intent
    node.body = np.array([0., -.4, 1.5])
    node._replan_from_fresh_body((4, 'seed-a'), 'moving_snapshot')
    assert node.pause.intent is intent and node.recovery_hold is None
    assert len(attempts) == 2
    assert attempts[-1][0]['xyz'] == pytest.approx([0., -.4, 1.])


def test_same_stair_goal_works_in_legacy_planning_frame(monkeypatch):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    submit(node, clock, [0., -.4, 2.], node.bridge.planning_frame)
    assert node.pause.intent.floor_id == 'stair_upper', node.reason
    assert attempts[-1][1]['xyz'] == pytest.approx([0., -.4, 2.])


def test_free_xyz_draft_uses_measured_ground_for_native_endpoint(monkeypatch):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    submit(node, clock, [0., -.4, 2.12])
    assert node.pause.intent.floor_id == 'stair_upper', node.reason
    assert node.pause.intent.xyz == pytest.approx([0., -.4, 2.12])
    goal = attempts[-1][1]
    assert goal['xyz'] == pytest.approx([0., -.4, 2.])
    node.tomogram.validate_endpoint(goal['xyz'], goal['layer_id'])


def test_invalid_stair_goal_never_launches_new_native_work(monkeypatch):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    submit(node, clock, [0., 0., 1.5])
    node.tomogram.cost[1, 10, 6] = 100.
    submit(node, clock, [0., -.4, 1.])
    assert node.state == 'goal_rejected'
    assert node.pause.intent is None and len(attempts) == 1


def test_committed_stair_route_is_not_recomputed_during_body_progress(monkeypatch):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    submit(node, clock, [0., 0., 1.5])
    intent = node.pause.intent
    node.route_committed = True
    node.body = np.array([0., -.4, 1.5])
    node._replan_from_fresh_body((4, 'seed-a'), 'body_progress')
    assert len(attempts) == 1 and node.pause.intent is intent
    assert node.reason == 'committed_route_retained_during_body_progress'


@pytest.mark.parametrize('frame', ['source', 'planning'])
def test_regularized_goal_and_current_support_reach_native_planning_coordinates(monkeypatch, frame):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    resolver, _ = regularized_stair_model()
    node.bridge, node.tomogram, node.surface_resolver = resolver.bridge, resolver.tomogram, resolver
    node.route_settings = resolver.settings
    node.body = np.array([0., -.4, 1.42])  # Literal support .92 + measured .50 body height.
    node.p.update(body_height_min_m=.49, body_height_max_m=.51)
    point = [0., -.4, 1.92] if frame == 'source' else [0., -.4, 2.]
    submit(node, clock, point, resolver.bridge.source_frame if frame == 'source'
           else resolver.bridge.planning_frame)
    assert node.pause.intent is not None, node.reason
    # Retain the user's original frame/coordinate intent, while the native
    # endpoint is resolved onto the same measured/conditioned ground pair.
    assert node.pause.intent.xyz == pytest.approx(point)
    assert node.pause.intent.frame_id == (resolver.bridge.source_frame if frame == 'source'
                                         else resolver.bridge.planning_frame)
    assert attempts[-1][0]['xyz'] == pytest.approx([0., -.4, 1.])
    assert attempts[-1][1]['xyz'] == pytest.approx([0., -.4, 2.])
    assert node.current.start_body_xyz == pytest.approx(node.body)


def test_regularized_goal_without_observed_correspondence_does_not_launch_native(monkeypatch):
    node, clock, _, _, _, attempts = stair_owner(monkeypatch)
    resolver, _ = regularized_stair_model()
    node.bridge, node.tomogram, node.surface_resolver = resolver.bridge, resolver.tomogram, resolver
    node.route_settings = resolver.settings
    submit(node, clock, [0., .5, 1.42])
    assert node.state == 'goal_rejected'
    assert node.pause.intent is None and not attempts
