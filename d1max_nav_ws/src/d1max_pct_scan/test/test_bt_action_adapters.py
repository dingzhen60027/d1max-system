"""Production callbacks, generated messages, fake handles; no ROS graph/SDK."""
from copy import deepcopy
import asyncio
import json
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from rclpy.action import GoalResponse, CancelResponse
from d1max_navigation_bt_interfaces.action import ComputeRoute, FollowRoute
from d1max_planning_interfaces.msg import LocalPlanDebug
from d1max_pct_scan import bt_adapters as module
from d1max_pct_scan.bt_adapters import NavigationBTAdapters
from d1max_pct_scan.bt_adapter_contract import ArrivalEvidence, route_fingerprint
from d1max_pct_scan.source_route import SourceRouteBuilder
from d1max_pct_scan.source_route_ros import to_message as encode_route


class Handle:
    def __init__(self, request):
        self.request, self.terminal, self.feedback, self.executed = request, None, [], 0
    def execute(self): self.executed += 1
    def succeed(self): self.terminal = 'succeeded'
    def abort(self): self.terminal = 'aborted'
    def canceled(self): self.terminal = 'canceled'
    def publish_feedback(self, message): self.feedback.append(message)


class Publisher:
    def __init__(self): self.messages, self.subscribers = [], 1
    def publish(self, value): self.messages.append(deepcopy(value))
    def get_subscription_count(self): return self.subscribers


def native_cancel(node, clock, generation=4):
    message = LocalPlanDebug()
    message.session_id, message.phase, message.valid = 'session', 'cancelled', False
    message.header.frame_id, message.header.stamp = 'd1max_loc_map', time_msg(clock.wall)
    message.generation = generation
    node.on_native_debug(message)


def worker_retired(node, clock):
    node.on_worker_status(String(data=json.dumps(dict(session_id='session',
        received_at_unix=clock.wall, planning=False, pending_worker_start=False,
        retired_native_workers=0, active_goal=None, active_reference=False))))


def time_msg(value):
    return module.rclpy.time.Time(nanoseconds=round(value*1e9)).to_msg()


def harness(monkeypatch):
    clock = SimpleNamespace(wall=100., mono=10.)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.mono)
    node = object.__new__(NavigationBTAdapters)
    node.p = dict(session_id='session', localization_session_id='session', execution_mode='preview',
        expected_source_map_sha256='a'*64, expected_tomogram_sha256='c'*64,
        expected_conditioning_sha256='b'*64,
        map_frame='d1max_loc_map', tracking_frame='d1max_loc_tracking', body_frame='d1max_loc_base_link',
        freshness_s=.5, result_timeout_s=60., cancel_timeout_s=2.5, body_height=.55,
        goal_xy_tolerance_m=.2, goal_z_tolerance_m=.15)
    node.get_clock = lambda: SimpleNamespace(now=lambda: module.rclpy.time.Time(
        nanoseconds=round(clock.wall*1e9)))
    node.cancel_timeout_warnings=[]
    node.get_logger=lambda:SimpleNamespace(warning=node.cancel_timeout_warnings.append)
    node.initialize_state()
    for name in ('goal_pub', 'goal3d_pub', 'worker_cancel_pub', 'reference_pub', 'route_reference_pub', 'health_pub'):
        setattr(node, name, Publisher())
    refresh(node, clock)
    worker_retired(node, clock)
    return node, clock


def refresh(node, clock, xyz=(0., 0., .55)):
    node.navigation = dict(epoch=1, seed_id='seed', received_at_unix=clock.wall,
                           reset_pending=False, fault='')
    node.localizer = dict(session_id='session', local_epoch=1, active_seed_ns='seed',
        confirmed_seed_ns='seed', verified_confirmations=3, map_version_id='map',
        wall_time=clock.wall, navigation=deepcopy(node.navigation), local_fault='',
        frames=dict(map='d1max_loc_map', tracking='d1max_loc_tracking'))
    node.pose = dict(schema=1, epoch=1, seed_id='seed', valid=True, pose_valid=True,
        reset_pending=False, motion_control_enabled=False, frame_id='d1max_loc_map',
        body_frame='d1max_loc_base_link', pose_timeout_sec=.08, received_at_unix=clock.wall,
        output_stamp_sec=clock.wall, fault='')
    node.scan = dict(session_id='session', received_at_unix=clock.wall, sensor_ready=True,
        native_map_context_ready=True, native_map_context_fault='', localization_epoch=1,
        localization_seed_id='seed', active_reference=False, generation=3,
        motion_enabled=False, last_spline_id=4)
    node.received.update({key: clock.mono for key in ('navigation', 'localizer', 'pose', 'scan')})
    node.body, node.body_identity, node.body_source = xyz, (1, 'seed', 'map'), clock.wall
    node.body_received = clock.mono


def request(task='task-1', kind='3d'):
    goal = PoseStamped()
    goal.header.frame_id = 'd1max_loc_map'
    goal.pose.position.x = 2.
    goal.pose.orientation.w = 1.
    return ComputeRoute.Goal(schema_version=2, session_id='session', task_id=task, goal_kind=kind, goal=goal,
                             has_goal_yaw=False, goal_yaw_tolerance_rad=.15)


def route(stamp=100.1):
    result = Path()
    result.header.frame_id, result.header.stamp = 'd1max_loc_map', time_msg(stamp)
    for x in (0., .5, 1., 1.5, 2.):
        pose = PoseStamped()
        pose.header = result.header
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        result.poses.append(pose)
    return result


def candidate(stamp=100.1):
    path = route(stamp)
    xyz = [[point.pose.position.x,0.,0.] for point in path.poses]
    bridge = SimpleNamespace(source_frame='d1max_loc_map', planning_frame='d1max_multifloor_planning',
        to_localization_ground=lambda points, labels:SimpleNamespace(xyz=points,diagnostics={}))
    snapshot = SourceRouteBuilder(bridge=bridge,source_map_sha256='a'*64,
        conditioning_sha256='b'*64,tomogram_sha256='c'*64).build(
            dict(route_type='same_floor',layer_ids=[4]*len(xyz),source_layer_ids=[4]*len(xyz)),
            xyz,['floor1']*len(xyz),map_version_id='map')
    return encode_route(snapshot,session_id='session',task_id='',route_id='',epoch=1,
                        seed_id='seed',stamp=path.header.stamp)


def compute_started(node):
    handle = Handle(request())
    assert node.compute_goal(handle.request) == GoalResponse.ACCEPT
    node.accept_compute(handle)
    node.compute_tick()
    return handle


def commit(node, clock, order='path_first'):
    compute_handle = compute_started(node)
    stamp = node.compute['goal_stamp']
    clock.wall, clock.mono = 100.1, 10.1
    refresh(node, clock)
    status = dict(session_id='session', received_at_unix=100.1, last_goal_stamp=stamp,
        route_committed=True, active_goal=dict(user_stamp=stamp, epoch=1, seed_id='seed', map_version_id='map'),
        last_route=dict(path_stamp=100.1,route_hash=candidate().route_hash), state='active', generation=7)
    if order == 'path_first':
        node.on_candidate(candidate())
        node.on_worker_status(String(data=json.dumps(status)))
    else:
        node.on_worker_status(String(data=json.dumps(status)))
        node.compute_tick()
        assert compute_handle.terminal is None
        node.on_candidate(candidate())
    node.compute_tick()
    assert compute_handle.terminal == 'succeeded'
    value = node.snapshot
    follow_request = FollowRoute.Goal(schema_version=2, session_id='session', task_id='task-1', route_id=value['route_id'],
        localization_epoch=1, localization_seed_id='seed', route=deepcopy(value['route']),
        snapshot=deepcopy(value['wire']),execution_confirmed=False,confirmation_id='')
    handle = Handle(follow_request)
    assert node.follow_goal(follow_request) == GoalResponse.ACCEPT
    node.accept_follow(handle)
    node.follow_tick()
    return compute_handle, handle


@pytest.mark.parametrize('order', ['path_first', 'status_first'])
def test_real_callbacks_buffer_worker_pair_and_commit_route_once(monkeypatch, order):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock, order)
    assert len(node.goal3d_pub.messages) == 1
    assert len(node.reference_pub.messages) == 1
    original = deepcopy(node.reference_pub.messages[0])
    for i in range(30):
        clock.wall += .05; clock.mono += .05
        refresh(node, clock, xyz=(i*.03, 0., .55))
        node.follow_tick()
    assert len(node.reference_pub.messages) == 1
    assert node.reference_pub.messages[0] == original
    assert handle.terminal is None and node.follow is not None


def test_v3_reference_ack_uses_exact_delivery_identity_not_float_clock(monkeypatch):
    import math
    node,clock=harness(monkeypatch);commit(node,clock)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map')
    node.ready_identity=lambda:(1,'seed','map')
    node.local_ready=lambda context:True
    slot=node.follow
    node.scan.update(active_reference=True,spline_visual_valid=True,local_debug_phase='accepted',
        owner_task_id=slot['handle'].request.task_id,owner_route_id=node.snapshot['route_id'],
        owner_route_hash=node.snapshot['source_snapshot'].route_hash,
        owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=slot['reference_stamp_ns'],
        owner_reference_stamp=math.nextafter(slot['reference_stamp'],math.inf))
    node.follow_tick()
    assert slot['wait_reason']!='waiting_reference_acceptance'
    # Same nanoseconds but a stale/foreign delivery must not acknowledge work.
    for key,value in [('owner_delivery_sequence',slot['reference_delivery_sequence']-1),
                      ('owner_task_id','old-task'),('owner_reference_stamp_ns',slot['reference_stamp_ns']-1)]:
        original=node.scan[key];node.scan[key]=value
        node.follow_tick()
        assert slot['wait_reason']=='waiting_reference_acceptance'
        node.scan[key]=original


def test_v3_received_candidate_wait_feedback_requires_exact_owner_and_current_debug(monkeypatch):
    node,clock=harness(monkeypatch);commit(node,clock)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map')
    node.ready_identity=lambda:(1,'seed','map')
    node.local_ready=lambda context:True
    slot=node.follow
    node.scan.update(active_reference=False,spline_visual_valid=False,
        native_reference_received=True,pending_reference_generation=2,
        local_debug_generation=2,local_debug_fresh=True,local_debug_phase='waiting_observed_space',
        native_pending_phase='waiting_observed_space',
        owner_task_id=slot['handle'].request.task_id,owner_route_id=node.snapshot['route_id'],
        owner_route_hash=node.snapshot['source_snapshot'].route_hash,
        owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=slot['reference_stamp_ns'])
    node.follow_tick()
    assert slot['wait_phase']=='recovering_local_trajectory'
    assert slot['wait_reason']=='waiting_observed_space'
    assert not node.scan['active_reference'] and not node.scan['spline_visual_valid']
    node.scan['local_debug_fresh']=False
    node.follow_tick()
    assert slot['wait_reason']=='waiting_observed_space' # diagnostic event, not permission lease
    for key,value in [('owner_task_id','old-task'),('local_debug_generation',1),
                      ('native_pending_phase',''),('native_reference_received',False)]:
        original=node.scan[key];node.scan[key]=value
        node.follow_tick()
        assert slot['wait_reason']=='waiting_reference_acceptance'
        node.scan[key]=original


def test_soft_pose_and_map_gaps_never_cancel_route_or_finish_follow(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    for i in range(20):
        clock.wall += .2; clock.mono += .2
        if i % 2:
            refresh(node, clock)
        else:
            node.pose['valid'] = False
            node.scan['sensor_ready'] = False
        node.follow_tick()
    assert node.follow['cancel'] is None and handle.terminal is None
    assert len(node.reference_pub.messages) == 1 and not node.worker_cancel_pub.messages


def test_compute_late_foreign_candidates_cannot_complete_new_task(monkeypatch):
    node, clock = harness(monkeypatch)
    handle = compute_started(node)
    clock.wall, clock.mono = 100.1, 10.1
    node.on_candidate(candidate())
    node.on_worker_status(String(data=json.dumps(dict(session_id='session', received_at_unix=100.1,
        last_goal_stamp=99., route_committed=True, last_route=dict(path_stamp=100.1),
        active_goal=dict(user_stamp=99., epoch=1, seed_id='seed', map_version_id='map')))))
    node.compute_tick()
    assert handle.terminal is None and node.snapshot is None and not node.reference_pub.messages


@pytest.mark.parametrize('candidate_first',[True,False])
def test_compute_completes_on_second_matching_event_without_timer(monkeypatch,candidate_first):
    node,clock=harness(monkeypatch)
    handle=Handle(request())
    assert node.compute_goal(handle.request)==GoalResponse.ACCEPT
    node.accept_compute(handle)
    assert node.compute['dispatched'] and len(node.goal3d_pub.messages)==1
    submitted=node.compute['goal_stamp']
    clock.wall,clock.mono=100.1,10.1
    refresh(node,clock)
    message=candidate()
    original_hash=message.route_hash
    status=String(data=json.dumps(dict(session_id='session',received_at_unix=100.1,
        last_goal_stamp=submitted,route_committed=True,state='active',generation=7,
        active_goal=dict(user_stamp=submitted,epoch=1,seed_id='seed',map_version_id='map'),
        last_route=dict(path_stamp=100.1,route_hash=original_hash))))
    if candidate_first:
        node.on_candidate(message)
        assert handle.terminal is None
        # The inbox owns canonical geometry, not an alias of a received Path.
        message.path.poses[-1].pose.position.x=999.
        message.route_hash='f'*64
        node.on_worker_status(status)
    else:
        node.on_worker_status(status)
        assert handle.terminal is None
        node.on_candidate(message)
    assert handle.terminal=='succeeded'
    assert node.snapshot['route'].poses[-1].pose.position.x==2.
    assert node.snapshot['digest']==original_hash
    assert node.compute is None


def test_compute_cancel_waits_for_actual_worker_retirement(monkeypatch):
    node, clock = harness(monkeypatch)
    handle = compute_started(node)
    assert node.cancel_compute(handle) == CancelResponse.ACCEPT
    assert len(node.worker_cancel_pub.messages) == 1
    node.compute_tick()
    assert handle.terminal is None
    clock.wall, clock.mono = 100.1, 10.1
    status = dict(session_id='session', received_at_unix=100.1, planning=False,
        pending_worker_start=False, retired_native_workers=1, active_goal=None, active_reference=False)
    node.on_worker_status(String(data=json.dumps(status)))
    node.compute_tick()
    assert handle.terminal is None
    clock.wall, clock.mono = 100.2, 10.2
    status.update(received_at_unix=100.2, retired_native_workers=0)
    node.on_worker_status(String(data=json.dumps(status)))
    node.compute_tick()
    assert handle.terminal == 'canceled' and node.compute is None
    assert node.compute_goal(request('task-2')) == GoalResponse.ACCEPT


def test_follow_cancel_waits_new_bridge_ack_and_empty_only_once(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    assert node.cancel_follow(handle) == CancelResponse.ACCEPT
    node.cancel_follow(handle)
    for _ in range(3): node.follow_tick()
    assert handle.terminal is None and len(node.reference_pub.messages) == 2
    assert not node.reference_pub.messages[-1].poses
    clock.wall, clock.mono = 100.2, 10.2
    status = dict(session_id='session', received_at_unix=100.2, active_reference=False, generation=4)
    node.on_scan_status(String(data=json.dumps(status)))
    node.follow_tick()
    assert handle.terminal is None  # bridge ACK alone cannot retire native work
    native_cancel(node, clock)
    node.follow_tick()
    assert handle.terminal is None  # native ACK alone leaves the PCT intent alive
    worker_retired(node, clock)
    node.follow_tick()
    assert handle.terminal == 'canceled' and node.follow is None
    assert len(node.worker_cancel_pub.messages) == 1


def test_cancel_timeout_quarantines_session_instead_of_overlapping_tasks(monkeypatch):
    node, clock = harness(monkeypatch)
    handle = compute_started(node)
    node.cancel_compute(handle)
    clock.wall += 3; clock.mono += 3
    node.compute_tick()
    assert handle.terminal == 'aborted' and node.quarantine
    assert node.compute_goal(request('new-task')) == GoalResponse.REJECT


@pytest.mark.parametrize('missing_ack', ['scan', 'native', 'worker'])
def test_follow_cancel_timeout_diagnostic_preserves_failed_gate_and_logs_once(monkeypatch, missing_ack):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    assert node.cancel_follow(handle) == CancelResponse.ACCEPT
    slot=node.follow
    cancel_mono=slot['cancel']['mono']
    # ROS source advances only .2 s while the actual 2.5 s wall fence expires.
    clock.wall,clock.mono=100.3,cancel_mono+2.6
    if missing_ack!='scan':
        node.on_scan_status(String(data=json.dumps(dict(session_id='session',
            received_at_unix=clock.wall,active_reference=False,generation=4))))
    if missing_ack!='native': native_cancel(node,clock)
    worker_retired(node,clock)
    if missing_ack=='worker': node.worker['retired_native_workers']=1
    node.follow_tick()
    result=slot['future'].result()
    assert handle.terminal=='aborted' and not result.retirement_confirmed
    assert result.reason=='cancel_ack_timeout_requires_session_restart'
    assert node.p['cancel_timeout_s']==2.5
    assert len(node.cancel_timeout_warnings)==1
    prefix,encoded=node.cancel_timeout_warnings[0].split(' ',1)
    assert prefix=='cancel_ack_timeout_diagnostic'
    diagnostic=json.loads(encoded)
    assert diagnostic['event']=='cancel_ack_timeout' and diagnostic['worker_kind']=='follow'
    assert diagnostic['deadline_mono']==cancel_mono+2.5
    assert diagnostic['cancel_source_stamp']==pytest.approx(100.1)
    assert diagnostic['source_now']==pytest.approx(100.3)
    assert diagnostic['scan']['fresh_status']==(missing_ack!='scan')
    assert diagnostic['native']['present']==(missing_ack!='native')
    assert diagnostic['worker']['retirement_ack']==(missing_ack!='worker')
    if missing_ack=='worker':
        assert diagnostic['worker']['retired_native_workers']==1
        assert not diagnostic['worker']['retired_conditions']['retired_native_workers_zero']
    node.cancellation_tick(slot,compute=False)
    assert len(node.cancel_timeout_warnings)==1


@pytest.mark.parametrize('change', ['route', 'task', 'epoch', 'seed'])
def test_follow_rejects_not_computed_route_identity(monkeypatch, change):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    node.follow = None  # only test admission against the retained computed snapshot
    modified = deepcopy(handle.request)
    if change == 'route': modified.route.poses[1].pose.position.y = .1
    if change == 'task': modified.task_id = 'other'
    if change == 'epoch': modified.localization_epoch = 2
    if change == 'seed': modified.localization_seed_id = 'other'
    assert node.follow_goal(modified) == GoalResponse.REJECT


def test_refresh_only_renews_same_route_on_exact_bridge_request(monkeypatch):
    node, clock = harness(monkeypatch)
    commit(node, clock)
    old_stamp = node.follow['reference_stamp']
    clock.wall += .05; clock.mono += .05
    refresh(node, clock)
    message = dict(session_id='session', received_at_unix=clock.wall, motion_enabled=False,
        localization_epoch=1, localization_seed_id='seed', rejected_reference_stamp=old_stamp, request_id=1)
    node.on_refresh(String(data=json.dumps(message)))
    assert len(node.reference_pub.messages) == 2
    first, second = node.reference_pub.messages
    assert route_fingerprint(first, 'd1max_loc_map') == route_fingerprint(second, 'd1max_loc_map')
    assert first.header.stamp != second.header.stamp
    node.on_refresh(String(data=json.dumps(message)))
    assert len(node.reference_pub.messages) == 2


def test_arrival_uses_fresh_actual_body_and_same_floor_not_spline_clock(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    stamp = node.follow['reference_stamp']
    # Local planner can report completed/progress while the robot is stationary.
    for i in range(5):
        clock.wall += .1; clock.mono += .1
        refresh(node, clock, xyz=(0., 0., .55))
        node.scan.update(active_reference=True, owner_reference_stamp=stamp,
            local_debug_progress_arc_m=100., spline_visual_valid=True)
        node.follow_tick()
    assert handle.terminal is None and node.follow['cancel'] is None
    # Same XY on a different floor cannot finish either.
    for i in range(5):
        clock.wall += .1; clock.mono += .1
        refresh(node, clock, xyz=(2., 0., 3.55))
        node.scan.update(active_reference=True, owner_reference_stamp=stamp)
        node.follow_tick()
    assert node.follow['cancel'] is None
    for i in range(3):
        clock.wall += .1; clock.mono += .1
        refresh(node, clock, xyz=(2., 0., .55))
        node.scan.update(active_reference=True, owner_reference_stamp=stamp)
        node.follow_tick()
    assert node.follow['cancel']['reason'] == 'goal_reached'
    assert handle.terminal is None  # stop is not acknowledged yet
    clock.wall += .1; clock.mono += .1
    node.on_scan_status(String(data=json.dumps(dict(session_id='session', received_at_unix=clock.wall,
        active_reference=False, generation=4))))
    native_cancel(node, clock)
    worker_retired(node, clock)
    node.follow_tick()
    assert handle.terminal == 'succeeded'
    arrival_result = node.executions[id(handle)]['future'].result()
    assert module.stamp_seconds(arrival_result.measured_arrival_stamp) == pytest.approx(clock.wall-.1)


def test_explicit_epoch_change_cancels_instead_of_soft_wait(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    node.pose['epoch'] = 2
    node.follow_tick()
    assert node.follow['cancel']['reason'] == 'pose_identity_changed'
    assert not node.reference_pub.messages[-1].poses and handle.terminal is None


def test_arrival_duplicate_samples_cannot_build_confirmation():
    evidence = ArrivalEvidence()
    args = dict(body=(2., 0., .55), endpoint=(2., 0., 0.), stamp=100., valid=True,
        body_height=.55, xy_tolerance=.2, z_tolerance=.15)
    for _ in range(100):
        assert not evidence.observe(**args)
    assert evidence.samples == 1


def test_completed_result_can_be_awaited_after_active_slot_retired(monkeypatch):
    node, clock = harness(monkeypatch)
    compute_handle, _ = commit(node, clock)
    assert node.compute is None
    result = asyncio.run(node.execute_compute(compute_handle))
    assert result.success and result.route_id == node.snapshot['route_id']
    assert compute_handle.terminal == 'succeeded'


def test_late_old_cancel_handle_cannot_cancel_current_follow(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    assert node.cancel_follow(Handle(handle.request)) == CancelResponse.REJECT
    assert node.follow['cancel'] is None and len(node.reference_pub.messages) == 1


@pytest.mark.parametrize('change', ['session', 'kind', 'frame', 'nan', 'oversize_task'])
def test_invalid_action_goals_are_rejected_without_publishing(monkeypatch, change):
    node, _ = harness(monkeypatch)
    value = request()
    if change == 'session': value.session_id = 'foreign'
    if change == 'kind': value.goal_kind = 'teleop'
    if change == 'frame': value.goal.header.frame_id = 'odom'
    if change == 'nan': value.goal.pose.position.x = float('nan')
    if change == 'oversize_task': value.task_id = 'x'*129
    assert node.compute_goal(value) == GoalResponse.REJECT
    assert not node.goal_pub.messages and not node.goal3d_pub.messages


def test_body_samples_must_cross_new_identity_barrier_and_be_monotonic(monkeypatch):
    node, clock = harness(monkeypatch)
    message = Odometry()
    message.header.frame_id, message.child_frame_id = node.p['map_frame'], node.p['body_frame']
    message.pose.pose.position.z = .55
    message.header.stamp = time_msg(100.)
    node.on_body(message)
    assert node.body is None  # first identity establishes a new barrier
    clock.wall += .02; clock.mono += .02
    # Refresh identity/pose only, not the body which the callback must establish.
    node.pose.update(received_at_unix=clock.wall, output_stamp_sec=clock.wall)
    node.received['pose'] = clock.mono
    message.header.stamp = time_msg(clock.wall)
    node.on_body(message)
    assert node.body_source == pytest.approx(clock.wall)
    assert node.body == (0., 0., .55)
    message.pose.pose.position.x = 100.
    node.on_body(message)
    assert node.body == (0., 0., .55)


def test_reference_stamps_remain_distinct_at_current_unix_epoch(monkeypatch):
    node, clock = harness(monkeypatch)
    clock.wall = 1790490000.
    first = node.stamped_path(route())
    second = node.stamped_path(route())
    assert module.stamp_seconds(second.header.stamp) > module.stamp_seconds(first.header.stamp)


def test_many_successful_recovery_episodes_do_not_exhaust_task_refresh_budget(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    for index in range(30):
        clock.wall += .05; clock.mono += .05
        refresh(node, clock)
        message = dict(session_id='session', received_at_unix=clock.wall, motion_enabled=False,
            localization_epoch=1, localization_seed_id='seed',
            rejected_reference_stamp=node.follow['reference_stamp'], request_id=index+1)
        node.on_refresh(String(data=json.dumps(message)))
        node.scan.update(active_reference=True, owner_reference_stamp=node.follow['reference_stamp'])
        node.follow_tick()
    assert handle.terminal is None and node.follow['cancel'] is None
    assert node.follow['refresh_count'] == 0
    assert len(node.reference_pub.messages) == 31
    assert len({route_fingerprint(p, 'd1max_loc_map') for p in node.reference_pub.messages}) == 1


def test_native_cancel_ack_must_match_new_cancel_generation(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    node.cancel_follow(handle)
    clock.wall += .1; clock.mono += .1
    node.on_scan_status(String(data=json.dumps(dict(session_id='session', received_at_unix=clock.wall,
        active_reference=False, generation=4))))
    native_cancel(node, clock, generation=3)
    node.follow_tick()
    assert handle.terminal is None
    clock.wall += .02; clock.mono += .02
    native_cancel(node, clock, generation=4)
    worker_retired(node, clock)
    node.follow_tick()
    assert handle.terminal == 'canceled'


def test_worker_heartbeat_slack_does_not_extend_pose_lease(monkeypatch):
    node, clock = harness(monkeypatch)
    clock.wall += .7; clock.mono += .7
    node.on_worker_status(String(data=json.dumps(dict(session_id='session', received_at_unix=100.))))
    assert node.fresh_status('worker')
    with pytest.raises(ValueError):
        node.ready_identity()


def test_native_cancel_event_remains_proof_while_waiting_worker_retirement(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    node.cancel_follow(handle)
    clock.wall += .1; clock.mono += .1
    native_cancel(node, clock)
    clock.wall += .7; clock.mono += .7
    # Native publishes cancellation only once. The event was fresh at receipt;
    # it cannot be undone because the action fence forbids a replacement goal.
    node.on_scan_status(String(data=json.dumps(dict(session_id='session', received_at_unix=clock.wall,
        active_reference=False, generation=4))))
    node.follow_tick()
    assert handle.terminal is None
    worker_retired(node, clock)
    node.follow_tick()
    assert handle.terminal == 'canceled'


@pytest.mark.parametrize('kind', ['2d', '3d'])
def test_first_goal_waits_worker_heartbeat_and_matching_subscriber(monkeypatch, kind):
    node, clock = harness(monkeypatch)
    node.worker = {}
    node.received['worker'] = -float('inf')
    publisher = node.goal_pub if kind == '2d' else node.goal3d_pub
    publisher.subscribers = 0
    handle = Handle(request(kind=kind))
    assert node.compute_goal(handle.request) == GoalResponse.ACCEPT
    node.accept_compute(handle)
    node.compute_tick()
    assert not node.compute['dispatched'] and not publisher.messages
    # Publisher graph discovery alone does not prove the worker finished map load.
    publisher.subscribers = 1
    node.compute_tick()
    assert not node.compute['dispatched'] and not publisher.messages
    publisher.subscribers = 0
    worker_retired(node, clock)
    node.compute_tick()
    assert not node.compute['dispatched'] and not publisher.messages
    publisher.subscribers = 1
    node.compute_tick()
    assert node.compute['dispatched'] and len(publisher.messages) == 1
    publisher.subscribers = 0
    for _ in range(5): node.compute_tick()
    assert len(publisher.messages) == 1  # never redispatch an accepted task


def pending_compute_for_deadline(monkeypatch):
    node,clock=harness(monkeypatch)
    node.p.update(result_timeout_s=10.,worker_startup_timeout_s=60.)
    node.worker={};node.received['worker']=-float('inf')
    handle=Handle(request())
    assert node.compute_goal(handle.request)==GoalResponse.ACCEPT
    node.accept_compute(handle)
    return node,clock,handle,node.compute


def advance_compute_clock(node,clock,offset):
    clock.wall,clock.mono=100.+offset,10.+offset
    refresh(node,clock)


def unavailable_localization():
    raise ValueError('waiting_current_global_localization')


def test_compute_worker_prewarm_uses_independent_bounded_sixty_seconds(monkeypatch):
    node,clock,_,slot=pending_compute_for_deadline(monkeypatch)
    advance_compute_clock(node,clock,59.999);node.compute_tick()
    assert slot['cancel'] is None and slot['wait_phase']=='waiting_worker'
    assert not slot['dispatched'] and not node.goal3d_pub.messages
    advance_compute_clock(node,clock,60.001);node.compute_tick()
    assert slot['cancel']['reason']=='compute_worker_startup_timeout'
    assert not node.goal3d_pub.messages and not node.worker_cancel_pub.messages


def test_worker_ready_waiting_localization_starts_the_same_ten_second_work_deadline(monkeypatch):
    node,clock,_,slot=pending_compute_for_deadline(monkeypatch)
    node.ready_identity=unavailable_localization
    advance_compute_clock(node,clock,40.);worker_retired(node,clock)
    node.compute_tick()
    assert slot['wait_phase']=='waiting_localization' and slot['cancel'] is None
    advance_compute_clock(node,clock,49.999);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel'] is None
    advance_compute_clock(node,clock,50.001);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel']['reason']=='compute_route_timeout'
    assert not slot['dispatched'] and not node.goal3d_pub.messages


def test_retreat_to_waiting_worker_does_not_reenter_or_renew_startup_budget(monkeypatch):
    node,clock,_,slot=pending_compute_for_deadline(monkeypatch)
    node.ready_identity=unavailable_localization
    advance_compute_clock(node,clock,40.);worker_retired(node,clock)
    node.compute_tick()
    advance_compute_clock(node,clock,45.)
    node.worker={};node.received['worker']=-float('inf');node.compute_tick()
    assert slot['wait_phase']=='waiting_worker' and slot['cancel'] is None
    advance_compute_clock(node,clock,50.001);node.compute_tick()
    assert slot['cancel']['reason']=='compute_route_timeout'
    assert not node.goal3d_pub.messages


def test_goal_dispatch_cannot_regrant_time_spent_waiting_for_localization(monkeypatch):
    node,clock,_,slot=pending_compute_for_deadline(monkeypatch)
    node.ready_identity=unavailable_localization
    advance_compute_clock(node,clock,40.);worker_retired(node,clock)
    node.compute_tick()
    advance_compute_clock(node,clock,48.);worker_retired(node,clock)
    node.ready_identity=lambda:(1,'seed','map');node.compute_tick()
    assert slot['dispatched'] and len(node.goal3d_pub.messages)==1
    advance_compute_clock(node,clock,49.999);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel'] is None
    advance_compute_clock(node,clock,50.001);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel']['reason']=='compute_route_timeout'
    assert len(node.goal3d_pub.messages)==1 and len(node.worker_cancel_pub.messages)==1


def test_first_reference_waits_actual_bridge_subscription(monkeypatch):
    node, clock = harness(monkeypatch)
    node.route_reference_pub.subscribers = 0
    _, handle = commit(node, clock)
    assert node.follow is not None and not node.follow['committed']
    assert not node.reference_pub.messages and handle.terminal is None
    node.route_reference_pub.subscribers = 1
    node.follow_tick()
    assert node.follow['committed'] and len(node.reference_pub.messages) == 1
    for _ in range(5): node.follow_tick()
    assert len(node.reference_pub.messages) == 1


def test_cancel_before_worker_startup_has_no_side_effect_or_ack_requirement(monkeypatch):
    node, clock = harness(monkeypatch)
    node.worker = {}
    node.received['worker'] = -float('inf')
    node.goal3d_pub.subscribers = 0
    handle = Handle(request())
    assert node.compute_goal(handle.request) == GoalResponse.ACCEPT
    node.accept_compute(handle)
    node.compute_tick()
    assert not node.compute['dispatched']
    assert node.cancel_compute(handle) == CancelResponse.ACCEPT
    assert handle.terminal is None
    node.compute_tick()
    assert handle.terminal == 'canceled' and not node.quarantine
    assert not node.worker_cancel_pub.messages and not node.goal3d_pub.messages
    assert node.compute is None


def test_new_task_may_replace_only_explicitly_completed_private_worker_intent(monkeypatch):
    node, _ = harness(monkeypatch)
    node.worker.update(active_goal={'user_stamp': 99.}, route_committed=False,
                       planning=False, pending_worker_start=False, retired_native_workers=0)
    handle = Handle(request())
    assert node.compute_goal(handle.request) == GoalResponse.ACCEPT
    node.accept_compute(handle)
    node.compute_tick()
    assert not node.compute['dispatched']
    # Finished calculation whose Follow action was not yet launched before a
    # user preemption. Existing worker _new_goal atomically replaces this intent.
    node.worker['route_committed'] = True
    node.compute_tick()
    assert node.compute['dispatched'] and len(node.goal3d_pub.messages) == 1


@pytest.mark.parametrize('busy', ['planning', 'pending_worker_start', 'retired_native_workers'])
def test_completed_flag_does_not_allow_takeover_of_still_busy_worker(monkeypatch, busy):
    node, _ = harness(monkeypatch)
    node.worker.update(active_goal={'user_stamp': 99.}, route_committed=True,
                       planning=False, pending_worker_start=False, retired_native_workers=0)
    node.worker[busy] = 1 if busy == 'retired_native_workers' else True
    handle = Handle(request())
    assert node.compute_goal(handle.request) == GoalResponse.ACCEPT
    node.accept_compute(handle)
    node.compute_tick()
    assert not node.compute['dispatched'] and not node.goal3d_pub.messages


@pytest.mark.parametrize('failure,seconds,expected', [
    ('map', 15.1, 'follow_map_timeout:'),
    ('reference', 5.1, 'follow_reference_timeout:'),
    ('trajectory', 20.1, 'follow_trajectory_timeout:')])
def test_follow_failure_is_bounded_and_uses_existing_cleanup_fence(monkeypatch, failure, seconds, expected):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    for delta in (0., seconds):
        clock.wall += delta; clock.mono += delta
        refresh(node, clock)
        node.scan.update(active_reference=failure != 'reference',
            owner_reference_stamp=node.follow['reference_stamp'],
            sensor_ready=failure != 'map', spline_visual_valid=False,
            local_debug_phase='failed_reference_target_occupied')
        node.follow_tick()
    assert node.follow['cancel']['reason'].startswith(expected)
    assert handle.terminal is None  # cancel request is not retirement confirmation
    assert len(node.goal3d_pub.messages) == 1 and len(node.reference_pub.messages) == 2
    assert not node.reference_pub.messages[-1].poses
    clock.wall += .1; clock.mono += .1
    node.on_scan_status(String(data=json.dumps(dict(session_id='session', received_at_unix=clock.wall,
        active_reference=False, generation=4))))
    native_cancel(node, clock)
    worker_retired(node, clock)
    node.follow_tick()
    assert handle.terminal == 'aborted' and not node.quarantine


def test_blockage_then_new_proved_curve_recovers_same_route_without_empty_publish(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    fingerprint = node.snapshot['digest']
    for index in range(50):
        clock.wall += .1; clock.mono += .1
        refresh(node, clock)
        node.scan.update(active_reference=True, owner_reference_stamp=node.follow['reference_stamp'],
            spline_visual_valid=index >= 30, local_debug_phase='failed_optimization')
        node.tick()
    assert handle.terminal is None and node.follow['cancel'] is None
    assert node.snapshot['digest'] == fingerprint and node.follow['recovery'].since is None
    assert len(node.goal3d_pub.messages) == 1 and len(node.reference_pub.messages) == 1
    phases = {feedback.phase for feedback in handle.feedback}
    assert 'recovering_local_trajectory' in phases and 'following' in phases
    assert handle.feedback[-1].reason == 'following_validated_local_trajectory'


def test_cancel_undelivered_follow_needs_worker_retirement_not_nonexistent_native_ack(monkeypatch):
    node, clock = harness(monkeypatch)
    node.route_reference_pub.subscribers = 0
    _, handle = commit(node, clock)
    assert not node.follow['committed']
    node.cancel_follow(handle)
    assert not node.reference_pub.messages
    assert len(node.worker_cancel_pub.messages) == 1
    clock.wall += .1; clock.mono += .1
    worker_retired(node, clock)
    node.follow_tick()
    assert handle.terminal == 'canceled' and not node.quarantine
    assert node.executions[id(handle)]['future'].result().retirement_confirmed


def exact_epoch_cancel_harness(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    # Use the real v13 clock domain, retaining all native header nanoseconds.
    clock.source_ns = 1791129505309507931
    clock.wall = clock.source_ns*1e-9
    node.get_clock = lambda: SimpleNamespace(now=lambda: module.rclpy.time.Time(
        nanoseconds=clock.source_ns))
    refresh(node, clock)
    worker_retired(node, clock)
    return node, clock, handle


def exact_native_cancel(node, clock, stamp_ns, generation=4):
    message = LocalPlanDebug()
    message.session_id, message.phase, message.valid = 'session', 'cancelled', False
    message.header.frame_id = 'd1max_loc_map'
    message.header.stamp = module.rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
    message.generation = generation
    original = deepcopy(message.header.stamp)
    node.on_native_debug(message)
    assert message.header.stamp == original  # The ACK source is never rewritten.


def exact_cancel_peer_retirement(node, clock, *, generation=4, active=False, worker_running=False):
    clock.source_ns += 100_000_000
    clock.wall = clock.source_ns*1e-9
    clock.mono += .1
    node.on_scan_status(String(data=json.dumps(dict(session_id='session',
        received_at_unix=clock.wall, active_reference=active, generation=generation))))
    worker_retired(node, clock)
    if worker_running:
        node.worker['retired_native_workers'] = 1


@pytest.mark.parametrize('ack_source_delta_ns', [0, 1])
def test_follow_cancel_epoch_ack_can_be_causally_later_in_same_source_tick(monkeypatch, ack_source_delta_ns):
    node, clock, handle = exact_epoch_cancel_harness(monkeypatch)
    assert node.cancel_follow(handle) == CancelResponse.ACCEPT
    slot = node.follow
    cancel_ns = slot['cancel']['source_ns']
    clock.mono += .001
    clock.source_ns += ack_source_delta_ns
    exact_native_cancel(node, clock, clock.source_ns)
    assert node.native_cancel_ack['stamp_ns'] == cancel_ns+ack_source_delta_ns
    exact_cancel_peer_retirement(node, clock)
    node.follow_tick()
    assert handle.terminal == 'canceled' and node.follow is None
    assert slot['future'].result().retirement_confirmed
    assert not node.quarantine and not node.cancel_timeout_warnings


@pytest.mark.parametrize('invalid_guard', ['before_source', 'future_source', 'before_receipt',
    'wrong_generation', 'unchanged_generation', 'worker_running', 'reference_active'])
def test_follow_cancel_same_source_ack_does_not_bypass_other_retirement_proofs(monkeypatch, invalid_guard):
    node, clock, handle = exact_epoch_cancel_harness(monkeypatch)
    source_ns = clock.source_ns
    if invalid_guard == 'before_receipt':
        exact_native_cancel(node, clock, source_ns)
    assert node.cancel_follow(handle) == CancelResponse.ACCEPT
    clock.mono += .001
    if invalid_guard != 'before_receipt':
        delta = -1 if invalid_guard == 'before_source' else 1 if invalid_guard == 'future_source' else 0
        generation = 5 if invalid_guard == 'wrong_generation' else 3 if invalid_guard == 'unchanged_generation' else 4
        exact_native_cancel(node, clock, source_ns+delta, generation=generation)
    if invalid_guard == 'future_source':
        assert node.native_cancel_ack is None  # One genuinely future nanosecond.
    exact_cancel_peer_retirement(node, clock,
        generation=3 if invalid_guard == 'unchanged_generation' else 4,
        active=invalid_guard == 'reference_active', worker_running=invalid_guard == 'worker_running')
    node.follow_tick()
    assert handle.terminal is None and node.follow is not None
    assert not node.follow['future'].done()


def test_native_cancel_ack_same_tick_generation_order_and_duplicates(monkeypatch):
    node, clock, _ = exact_epoch_cancel_harness(monkeypatch)
    source_ns = clock.source_ns
    exact_native_cancel(node, clock, source_ns, generation=4)
    original_receipt = node.native_cancel_ack['received']
    clock.mono += .001
    exact_native_cancel(node, clock, source_ns, generation=4)
    assert node.native_cancel_ack['received'] == original_receipt
    exact_native_cancel(node, clock, source_ns, generation=5)
    assert node.native_cancel_ack['generation'] == 5
    assert node.native_cancel_ack['stamp_ns'] == source_ns
    exact_native_cancel(node, clock, source_ns-1, generation=6)
    assert node.native_cancel_ack['generation'] == 5
    clock.source_ns += 1
    exact_native_cancel(node, clock, clock.source_ns, generation=4)
    assert node.native_cancel_ack['generation'] == 5
