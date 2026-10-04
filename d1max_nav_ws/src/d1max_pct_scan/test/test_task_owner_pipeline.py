"""One owner -> reference bridge regression, without a ROS graph or robot IO.

Unlike stationary callback tests, these move the measured body beyond both the
global request-start tolerance and the bridge's new-reference start tolerance.
The real global recovery/commit callbacks publish into the real bridge callback;
native planning is represented only by a counted worker-request boundary.
"""
from copy import deepcopy
from types import MethodType, SimpleNamespace as NS
import json

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from d1max_pct_scan.live_global_contract import RequestEvidence
from d1max_pct_scan.live_global_planner import LiveGlobalPlanner
from d1max_pct_scan.live_goal_pause import GoalIdentity, GoalIntent, RetainedGlobalComputation
from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from test_preview_task_continuity import real_contract_harness, sample


def coordinates(message):
    return tuple((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                 for pose in message.poses)


def pipeline(monkeypatch):
    bridge, clock, native_paths, clears, _ = real_contract_harness(monkeypatch)
    bridge.gate.active = False
    publications, visuals, worker_requests = [], [], []
    identity = GoalIdentity('session', 'map-a', 1, 'seed', 'd1max_loc_map', 'hash-a')
    pause = RetainedGlobalComputation()
    pause.install(GoalIntent(100.1, '2d', identity.frame_id, (4., 0., 0.), 'floor1', identity))
    message = Path()
    message.header.frame_id = identity.frame_id
    for x in range(5):
        point = PoseStamped()
        point.header = message.header
        point.pose.position.x = float(x)
        point.pose.orientation.w = 1.
        message.poses.append(point)
    owner = NS(
        p=dict(session_id='session', freshness_s=.5, max_start_move_m=.3,
               commit_wait_timeout_s=10., result_timeout_s=45.,
               body_height_min_m=.25, body_height_max_m=.85,
               retain_preview_task_on_soft_loss=True),
        pause=pause, pause_wall=None, generation=3, current=None, pending_goal=None,
        cached_evidence=RequestEvidence(3, 1, 'seed', 100.1, (0., 0., .55), 10.1),
        cached_result=dict(path_message=message, source_tomogram_sha256='hash-a', diagnostics={}),
        bridge=NS(source_frame=identity.frame_id, floors={'floor1':object()},
                  project_live_pose_to_ground=lambda body, floor, **kw:
                      NS(xyz=np.array([body[0], body[1], 0.]))),
        tomogram=NS(sha256='hash-a'),
        recovery_hold=None, commit_wait_started=None, route_committed=False,
        active_context=None, active_reference=False, visual_path_available=False,
        last_route={}, last_reference_stamp=0., refresh_requested=False, last_refresh_id=0,
        goal_started_monotonic=None, goal_deadline_monotonic=None,
        startup_stamp=100., last_goal_stamp=100.1, replan_count=0,
        static_validator=NS(invalidate=lambda:None),
        now_s=lambda:clock.wall, get_clock=bridge.get_clock,
        get_parameter=lambda name:NS(value='floor1'),
        stop_child=lambda:None, idle_worker_available=lambda:False,
        publish_status=lambda:None,
        plan_intent=lambda *args, **kwargs:worker_requests.append((args, kwargs)),
        visual_path_pub=NS(publish=lambda value:visuals.append(deepcopy(value))),
    )
    for name in ('publish_empty', 'clear_visual_path', 'revoke', 'confirmed_identity',
                 'planning_context', 'pause_reference', 'resume_intent',
                 '_start_moved', '_commit_cached_result', '_revoke_if_context_lost',
                 '_pause_barrier_passed', '_replan_from_fresh_body'):
        setattr(owner, name, MethodType(getattr(LiveGlobalPlanner, name), owner))

    def publish(value):
        publications.append(deepcopy(value))
        LiveScanBridge.on_reference(bridge, value)

    owner.path_pub = NS(publish=publish)

    def observe(wall, *, x=0., valid=True):
        sample(bridge, clock, wall, valid=valid)
        bridge.body.pose.pose.position.x = x
        bridge.body.pose.pose.position.y = 0.
        bridge.localizer['map_version_id'] = identity.map_version_id
        owner.localizer = deepcopy(bridge.localizer)
        owner.navigation = deepcopy(bridge.navigation)
        owner.pose_status = deepcopy(bridge.pose_status)
        owner.localizer_received = owner.pose_received = owner.body_received = clock.mono
        owner.body_stamp = wall-.01
        owner.body = np.array([x, 0., .55])
        owner.body_context = identity
        bridge.update_gate()

    observe(100.26)
    owner._commit_cached_result()
    assert owner.route_committed and bridge.gate.active
    assert len(publications) == len(visuals) == len(native_paths) == 1
    return NS(owner=owner, bridge=bridge, clock=clock, observe=observe,
              publications=publications, visuals=visuals, native=native_paths,
              workers=worker_requests, clears=clears, coordinates=coordinates(message))


def test_moving_committed_route_survives_eight_pose_gaps_and_owner_refresh(monkeypatch):
    run = pipeline(monkeypatch)
    native_identity = (run.bridge.gate.generation, run.bridge.gate.digest,
                       run.bridge.gate.issued_at)
    owner_generation = run.owner.generation
    for cycle in range(8):
        wall = 100.4 + cycle*.8
        # A real displacement, not synthetic trajectory-clock progress.
        x = 1.2 + cycle*.1
        run.observe(wall, x=x, valid=False)
        run.owner._revoke_if_context_lost()
        assert run.owner.pause.paused_at is not None
        assert run.bridge.gate.preview_paused and run.bridge.gate.active
        # Three distinct fresh samples and the existing post-pause barrier.
        for delta in (.1, .21, .32):
            run.observe(wall+delta, x=x)
            run.owner._revoke_if_context_lost()
        assert run.owner.pause.paused_at is None
        assert run.owner.route_committed
        assert run.bridge.gate.ready and run.bridge.gate.active
        # An explicit reference-receipt retry must not become a new plan.
        request = dict(schema=1, motion_enabled=False, request_id=cycle+1,
            session_id='session', localization_epoch=1, localization_seed_id='seed',
            rejected_reference_stamp=run.owner.last_reference_stamp,
            received_at_unix=run.clock.wall)
        LiveGlobalPlanner.on_reference_refresh_request(
            run.owner, String(data=json.dumps(request)))
        run.observe(wall+.34, x=x)
        run.owner._revoke_if_context_lost()
        assert run.owner.generation == owner_generation
        assert (run.bridge.gate.generation, run.bridge.gate.digest,
                run.bridge.gate.issued_at) == native_identity
        assert not run.workers
    assert all(coordinates(value) == run.coordinates for value in run.publications)
    assert all(coordinates(value) == run.coordinates for value in run.visuals)
    assert len(run.publications) == 17  # first commit + 8 recoveries + 8 receipt retries
    assert len(run.visuals) == 1  # the fixed global route is not repainted on gaps
    assert len(run.native) == 1  # no cancel/new native generation or progress reset


@pytest.mark.parametrize('event', ['cancel', 'new_goal', 'reset', 'new_seed'])
def test_task_owner_hard_events_still_cancel_the_retained_route(monkeypatch, event):
    run = pipeline(monkeypatch)
    generation = run.bridge.gate.generation
    run.observe(100.4, x=1.2, valid=False)
    run.owner._revoke_if_context_lost()
    run.observe(100.45, x=1.2)
    if event == 'cancel':
        LiveGlobalPlanner.on_cancel(run.owner, None)
    elif event == 'new_goal':
        goal = PoseStamped()
        goal.header.frame_id = 'd1max_loc_map'
        goal.header.stamp = run.owner.get_clock().now().to_msg()
        goal.pose.position.x = 3.
        goal.pose.orientation.w = 1.
        LiveGlobalPlanner._new_goal(run.owner, goal, kind='2d')
    else:
        if event == 'reset':
            run.owner.navigation['reset_pending'] = True
        else:
            run.owner.localizer['active_seed_ns'] = 'different'
        run.owner._revoke_if_context_lost()
    assert not run.bridge.gate.active
    assert run.bridge.gate.generation == generation+1
    assert len(run.native) == 2 and not run.native[-1].path.poses
    assert not run.publications[-1].poses and not run.visuals[-1].poses
    assert len(run.workers) == (1 if event == 'new_goal' else 0)
