"""Production owner callbacks; finite planning cycles versus durable user intent.

No ROS initialization, worker process, service, SDK, or robot commands. Native
geometry generation is outside this state-machine test; the actual ground
bridge's protected stair/height checks are exercised for recovery preflight.
"""
from copy import deepcopy
from dataclasses import replace
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from d1max_pct_scan.live_global_contract import RequestEvidence
from d1max_pct_scan.live_global_planner import LiveGlobalPlanner
from d1max_pct_scan.live_goal_pause import RetainedGlobalComputation
from test_live_goal_pause import IDENTITY, INTENT, _localizer, _navigation
from d1max_pct_scan.pointcloud_helpers.ground_path_bridge import GroundPathBridge


def lifecycle(monkeypatch, *, implementation=LiveGlobalPlanner):
    clock = NS(mono=60., wall=160.)
    monkeypatch.setattr('d1max_pct_scan.live_global_planner.time.monotonic', lambda: clock.mono)
    actions, published, visuals, attempts = [], [], [], []
    pause = RetainedGlobalComputation(); pause.install(INTENT); pause.pause(59.8)
    floors = {name: NS(reference_z_m=z,
        query=lambda xy, _limits, z=z: (np.full(len(xy), z), np.zeros(len(xy))))
        for name, z in [('floor1', 0.), ('floor2', 3.)]}
    bridge = GroundPathBridge(floors, [dict(min=[2., -1., .2], max=[3., 1., 2.5])])
    path = Path(); path.header.frame_id = bridge.source_frame
    path.header.stamp = Time(sec=101)
    path.poses = [PoseStamped(), PoseStamped()]
    node = NS(p=dict(session_id='session-a', max_start_move_m=.3, result_timeout_s=45.,
                     commit_wait_timeout_s=10., body_height_min_m=.25, body_height_max_m=.85,
                     retain_preview_task_on_soft_loss=True),
        pause=pause, pause_wall=159.8, current=None, pending_goal=None,
        cached_evidence=RequestEvidence(8, 4, 'seed-a', 100.1, (0., 0., .55), 0.),
        cached_result=dict(path_message=path, source_tomogram_sha256='hash-a', diagnostics={}),
        body=np.array([1., 0., .55]), body_stamp=160., body_received=60.,
        bridge=bridge, route_settings={'floor_z_ranges':{'lower':[-.1,.1], 'upper':[2.9,3.1]}},
        tomogram=NS(sha256='hash-a', sample_surfaces=lambda xy, **_: [
            dict(xyz=[float(xy[0]),float(xy[1]),z], ground_z=z, layer_id=i)
            for i,z in enumerate([0.,3.])]),
        generation=8, replan_count=0, computation_cycle=1, route_committed=True,
        goal_started_monotonic=0., goal_deadline_monotonic=45., recovery_hold=None,
        state='active', reason='active', active_reference=True, visual_path_available=True,
        active_context=(4,'seed-a'), last_route={'path_stamp':101.},
        last_reference_stamp=101., commit_wait_started=None, refresh_requested=False,
        worker_metrics={}, goal_kind='2d', goal_stamp=100.1,
        static_validator=NS(invalidate=lambda: actions.append('invalidate_result')),
        idle_worker_available=lambda: True, stop_child=lambda: actions.append('stop_child'),
        publish_status=lambda:actions.append('status'),
        planning_context=lambda:(4,'seed-a'), confirmed_identity=lambda:IDENTITY,
        now_s=lambda:clock.wall, get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(
            sec=int(clock.wall), nanosec=round((clock.wall%1)*1e9)))),
        path_pub=NS(publish=lambda value:published.append(deepcopy(value))),
        visual_path_pub=NS(publish=lambda value:visuals.append(deepcopy(value))),
        localizer=_localizer(160.), navigation=_navigation(160.), pose_status=_navigation(160.),
        localizer_received=60., startup_stamp=99., last_goal_stamp=100.1,
        get_parameter=lambda _name:NS(value='floor1'))
    for name in ('resume_intent', '_start_moved', '_replan_from_fresh_body', 'plan_intent',
                 'revoke', 'publish_empty', 'clear_visual_path', '_commit_cached_result',
                 '_revoke_if_context_lost', '_pause_barrier_passed', 'pause_reference'):
        setattr(node, name, MethodType(getattr(implementation, name), node))
    node._maybe_launch_pending = lambda: attempts.append(deepcopy(node.pending_goal))
    return node, clock, actions, published, visuals, attempts


def test_committed_route_recovers_after_old_computation_deadline_with_same_goal(monkeypatch):
    node, _, _, published, visuals, attempts = lifecycle(monkeypatch)
    original_intent = node.pause.intent
    original_geometry = deepcopy(node.cached_result['path_message'].poses)
    node.resume_intent()
    assert node.pause.intent is original_intent
    assert node.current is None and not attempts
    assert node.computation_cycle == 1 and node.generation == 8
    assert node.goal_deadline_monotonic is None
    assert node.active_reference and published[-1].poses == original_geometry
    assert not visuals  # Same reference ACK is not a new global display route.
    assert node.recovery_hold is None


def test_successful_commit_ends_calculation_not_user_mission(monkeypatch):
    node, _, _, published, visuals, _ = lifecycle(monkeypatch)
    node.route_committed = False
    node.body = np.array([.1, 0., .55]); node.pause.resumed()
    node._commit_cached_result()
    assert node.route_committed and node.active_reference
    assert node.goal_deadline_monotonic is None and node.goal_started_monotonic is None
    assert node.pause.intent is INTENT and published[-1].poses and visuals[-1].poses


def test_commit_immediately_notifies_only_after_complete_route_state(monkeypatch):
    node,clock,_,published,_,_=lifecycle(monkeypatch)
    node.route_committed=False;node.body=np.array([.1,0.,.55]);node.pause.resumed()
    events=[]
    node.publish_status=lambda:events.append(dict(committed=node.route_committed,
        context=node.active_context,state=node.state,route=deepcopy(node.last_route),
        published=len(published),deadline=node.goal_deadline_monotonic))
    node._commit_cached_result()
    assert len(events)==1 and events[0]['committed'] and events[0]['state']=='active'
    assert events[0]['context']==(4,'seed-a') and events[0]['published']==1
    assert events[0]['route']['goal_stamp']==INTENT.user_stamp
    assert events[0]['route']['path_stamp']==node.last_reference_stamp
    assert events[0]['deadline'] is None and node.last_status_at==clock.mono


def test_rejected_commit_never_sends_success_notification(monkeypatch):
    node,_,_,_,_,_=lifecycle(monkeypatch)
    node.route_committed=False;node.pause.resumed()
    node.confirmed_identity=lambda:replace(IDENTITY,epoch=99)
    events=[];node.publish_status=lambda:events.append('commit')
    node._commit_cached_result()
    assert not events and not node.route_committed


def test_committed_route_soft_pause_is_not_an_unpublished_result_timeout(monkeypatch):
    node, clock, _, _, _, _ = lifecycle(monkeypatch)
    node.body = np.array([.1, 0., .55]); node.pause.resumed()
    node._commit_cached_result()
    node.pause_reference('temporary_source_loss')
    assert node.commit_wait_started is None
    assert node.goal_deadline_monotonic is None and node.pause.intent is INTENT
    clock.mono += 90.; clock.wall += 90.
    node.resume_intent()
    assert node.pause.intent is INTENT and node.active_reference and node.route_committed
    assert node.recovery_hold is None


def test_repeated_moving_start_retries_do_not_extend_same_cycle_budget(monkeypatch):
    node, clock, _, _, _, attempts = lifecycle(monkeypatch)
    node.route_committed = False; node.goal_deadline_monotonic = 105.
    node.resume_intent()
    deadline, cycle = node.goal_deadline_monotonic, node.computation_cycle
    for distance in (1.5, 2.):
        clock.mono += 2.; clock.wall += 2.
        node.body = np.array([distance, 2., .55])
        node._replan_from_fresh_body((4,'seed-a'), 'moving_snapshot')
        assert node.goal_deadline_monotonic == deadline and node.computation_cycle == cycle
    assert len(attempts) == 3
    node._replan_from_fresh_body((4,'seed-a'), 'moving_snapshot_again')
    assert node.pause.intent is INTENT and node.current is None
    assert node.recovery_hold['reason'] == 'snapshot_replan_limit_exceeded'
    assert node.recovery_hold['retry_on_supported_body_change'] is False
    # Good sensor status cannot repeatedly create a fresh budget after failure.
    clock.mono += 100.; clock.wall += 100.; node.body[0] += 2.
    node._revoke_if_context_lost(); node.resume_intent()
    assert len(attempts) == 3 and node.computation_cycle == cycle


def test_unfinished_cycle_deadline_latches_goal_without_auto_budget_refresh(monkeypatch):
    node, clock, _, _, _, attempts = lifecycle(monkeypatch)
    node.route_committed = False; node.goal_deadline_monotonic = 105.
    node.resume_intent()
    clock.mono = node.goal_deadline_monotonic+.1; clock.wall += 46.
    cycle = node.computation_cycle
    node._replan_from_fresh_body((4,'seed-a'), 'slow_computation')
    assert node.pause.intent is INTENT and node.current is None and not node.active_reference
    assert node.recovery_hold['reason'] == 'total_goal_computation_deadline_expired'
    for _ in range(20): node._revoke_if_context_lost()
    assert node.computation_cycle == cycle and len(attempts) == 1


def test_progress_into_stairs_does_not_reproject_the_committed_route_start(monkeypatch):
    node, clock, _, published, visuals, attempts = lifecycle(monkeypatch)
    node.body = np.array([2.5, 0., 1.5])
    node.resume_intent()
    assert node.pause.intent is INTENT and not attempts
    assert node.current is None and node.active_reference and node.visual_path_available
    assert node.cached_result is not None and published[-1].poses and not visuals
    assert node.recovery_hold is None
    assert node.goal_deadline_monotonic is None and node.computation_cycle == 1
    # Unchanged body or pose jitter causes no budget allocation or native work.
    for delta in (0., .001, -.001, .01):
        node.body[0] = 2.5+delta
        node._revoke_if_context_lost()
    assert not attempts and node.computation_cycle == 1
    # Progress onto another floor is following, not a new global request.
    clock.mono += 20.; clock.wall += 20.
    node.body = np.array([3.2, 0., 3.55])
    node._revoke_if_context_lost()
    assert node.pause.intent is INTENT and node.current is None and not attempts
    assert node.computation_cycle == 1 and node.goal_deadline_monotonic is None


def test_direct_body_progress_replan_call_cannot_replace_a_committed_route(monkeypatch):
    node, clock, _, _, _, attempts = lifecycle(monkeypatch)
    node.body = np.array([2.99, 0., .55]); node.resume_intent()
    assert node.recovery_hold is None and not attempts
    node.body = np.array([3.01, 0., .55])
    node._replan_from_fresh_body((4, 'seed-a'), 'incidental_body_motion')
    assert not attempts
    clock.mono += 1.; clock.wall += 1.
    node._revoke_if_context_lost()
    assert not attempts and node.pause.intent is INTENT and node.recovery_hold is None
    assert node.route_committed and node.generation == 8 and node.computation_cycle == 1


def test_moving_into_stairs_during_unfinished_first_calculation_retains_goal_but_not_budget(monkeypatch):
    node, clock, _, _, _, attempts = lifecycle(monkeypatch)
    node.route_committed = False; node.goal_deadline_monotonic = 105.
    node.body = np.array([2.5, 0., 1.5])
    node._replan_from_fresh_body((4,'seed-a'), 'snapshot_moved_into_stairs')
    assert node.pause.intent is INTENT and node.recovery_hold is not None and not attempts
    assert node.recovery_hold['retry_on_supported_body_change'] is False
    assert node.goal_deadline_monotonic is None
    clock.mono += 100.; node.body = np.array([3.2, 0., 3.55])
    node._revoke_if_context_lost()
    assert not attempts and node.computation_cycle == 1


@pytest.mark.parametrize('change', ['epoch', 'seed', 'map', 'fault', 'cancel'])
def test_hard_identity_or_user_cancel_still_destroys_held_goal_and_history(monkeypatch, change):
    node, _, _, _, visuals, attempts = lifecycle(monkeypatch)
    node.body = np.array([2.5, 0., 1.5]); node.resume_intent()
    if change == 'epoch': node.localizer['local_epoch'] += 1
    if change == 'seed': node.localizer['confirmed_seed_ns'] = 'another'
    if change == 'map': node.tomogram.sha256 = 'changed'
    if change == 'fault': node.navigation['fault'] = 'hard_fault'
    if change == 'cancel': LiveGlobalPlanner.on_cancel(node, None)
    else: node._revoke_if_context_lost()
    assert node.pause.intent is None and node.recovery_hold is None and not attempts
    assert node.cached_result is None and not node.active_reference
    assert visuals and not visuals[-1].poses and not node.visual_path_available


def test_old_result_after_hold_cannot_commit_and_new_user_request_gets_new_budget(monkeypatch):
    node, clock, _, _, _, attempts = lifecycle(monkeypatch)
    node.route_committed = False; node.goal_deadline_monotonic = 105.
    node.resume_intent(); old = node.current
    clock.mono = node.goal_deadline_monotonic+.1; clock.wall += 46.
    node._replan_from_fresh_body((4,'seed-a'), 'timeout')
    LiveGlobalPlanner._finish(node, dict(kind='planned', generation=old.generation, result={}))
    assert node.current is None and node.recovery_hold is not None
    message = PoseStamped(); message.header.frame_id = node.bridge.source_frame
    message.header.stamp = Time(sec=int(clock.wall))
    message.pose.position.x, message.pose.position.y = 4., 2.
    LiveGlobalPlanner._new_goal(node, message, kind='2d')
    assert node.pause.intent is not INTENT and node.pause.intent.xyz[:2] == (4.,2.)
    assert node.current is not None and node.recovery_hold is None and len(attempts) == 2
    assert node.goal_deadline_monotonic == clock.mono+45.
