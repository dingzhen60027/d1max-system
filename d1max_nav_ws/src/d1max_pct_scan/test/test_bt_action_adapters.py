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
from d1max_planning_interfaces.msg import LocalPlanDebug, RouteDeliveryReceipt
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
        retired_native_workers=0, active_goal=None, active_reference=False,
        native_warmup={'phase':'ready'}))))


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
        goal_xy_tolerance_m=.2, goal_z_tolerance_m=.15,arrival_evidence_timeout_s=4.)
    node.get_clock = lambda: SimpleNamespace(now=lambda: module.rclpy.time.Time(
        nanoseconds=round(clock.wall*1e9)))
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


def compute_started(node, task='task-1'):
    handle = Handle(request(task=task))
    assert node.compute_goal(handle.request) == GoalResponse.ACCEPT
    node.accept_compute(handle)
    node.compute_tick()
    return handle


def commit(node, clock, order='path_first', task='task-1', completion_stamp=100.1, completion_mono=10.1):
    compute_handle = compute_started(node, task=task)
    stamp = node.compute['goal_stamp']
    clock.wall, clock.mono = completion_stamp, completion_mono
    refresh(node, clock)
    source=candidate(completion_stamp)
    status = dict(session_id='session', received_at_unix=completion_stamp, last_goal_stamp=stamp,
        route_committed=True, active_goal=dict(user_stamp=stamp, epoch=1, seed_id='seed', map_version_id='map'),
        last_route=dict(path_stamp=module.stamp_seconds(source.path.header.stamp),route_hash=source.route_hash),
        state='active', generation=7)
    if order == 'path_first':
        node.on_candidate(source)
        node.on_worker_status(String(data=json.dumps(status)))
    else:
        node.on_worker_status(String(data=json.dumps(status)))
        node.compute_tick()
        assert compute_handle.terminal is None
        node.on_candidate(source)
    node.compute_tick()
    assert compute_handle.terminal == 'succeeded'
    value = node.snapshot
    follow_request = FollowRoute.Goal(schema_version=2, session_id='session', task_id=task, route_id=value['route_id'],
        localization_epoch=1, localization_seed_id='seed', route=deepcopy(value['route']),
        snapshot=deepcopy(value['wire']),execution_confirmed=False,confirmation_id='')
    handle = Handle(follow_request)
    assert node.follow_goal(follow_request) == GoalResponse.ACCEPT
    node.accept_follow(handle)
    node.follow_tick()
    return compute_handle, handle


def typed_follow(monkeypatch):
    node,clock=harness(monkeypatch)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map')
    node.ready_identity=lambda:(1,'seed','map')
    node.local_ready=lambda context:True
    _,handle=commit(node,clock)
    return node,clock,handle


def follower_feedback(node,clock):
    from d1max_planning_interfaces.msg import TaskExecutionLease,LocalExecutionStatus,ExecutionVersion
    snapshot=node.snapshot
    v=ExecutionVersion(schema_version=3,session_id='session',task_id=snapshot['task_id'],route_id=snapshot['route_id'],
        route_hash=snapshot['source_snapshot'].route_hash,map_version_id='map',localization_epoch=1,localization_seed_id='seed')
    lease=TaskExecutionLease(schema_version=1,version=v,execution_id='execution',control_epoch=3,
        confirmation_id='confirmation',sdk_session='sdk',sdk_arm_generation=2,sequence=1,
        source_stamp=time_msg(clock.wall),valid_until=time_msg(clock.wall+.35),allowed=True,phase='tracking',transport_mode='live')
    status=LocalExecutionStatus(schema_version=1,version=deepcopy(v),execution_id='execution',control_epoch=3,
        sequence=1,source_stamp=time_msg(clock.wall),body_source_stamp=time_msg(clock.wall),
        body_valid_until=time_msg(clock.wall+.1),trajectory_id=1,commit_sequence=1,ready=True,phase='following',transport_mode='live')
    return lease,status


def test_task_follower_readiness_does_not_wait_for_sdk_curve_cas(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch);node.p['execution_architecture']='task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
    lease,status=follower_feedback(node,clock)
    lease.execution_id='';lease.control_epoch=0;lease.allowed=False;lease.sdk_session='';lease.sdk_arm_generation=0
    status.execution_id='';status.control_epoch=0
    assert node.on_task_lease(lease) and node.on_local_execution_status(status)
    node.follow_tick()
    assert node.follow['wait_phase']=='following' and node.follow['cancel'] is None
    assert node.execution_permit is None and node.stop_report is None and handle.terminal is None


def test_task_follower_same_task_new_curve_keeps_single_fixed_route(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch);node.p['execution_architecture']='task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
    lease,status=follower_feedback(node,clock);assert node.on_task_lease(lease);assert node.on_local_execution_status(status)
    slot=node.follow;digest=node.snapshot['digest'];messages=len(node.route_reference_pub.messages)
    for i in range(1,8):
        clock.wall+=.1;clock.mono+=.1;refresh(node,clock)
        lease.source_stamp=time_msg(clock.wall);lease.valid_until=time_msg(clock.wall+.35);lease.sequence+=1
        assert node.on_task_lease(lease)
        status.source_stamp=status.body_source_stamp=time_msg(clock.wall);status.body_valid_until=time_msg(clock.wall+.1)
        status.version.anchor_id='map_correction_'+str(i);status.version.reference_generation=i+1
        status.trajectory_id=i+1;status.commit_sequence=i+1;status.sequence+=1
        assert node.on_local_execution_status(status);node.follow_tick()
        assert node.follow is slot and slot['wait_phase']=='following' and slot['cancel'] is None
    assert node.snapshot['digest']==digest and len(node.route_reference_pub.messages)==messages
    assert not node.worker_cancel_pub.messages and handle.terminal is None


def test_task_follower_goal_requires_matching_measured_stop_and_global_position(monkeypatch):
    from d1max_planning_interfaces.msg import StopReport
    node,clock,handle=typed_follow(monkeypatch);node.p['execution_architecture']='task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
    lease,status=follower_feedback(node,clock);assert node.on_task_lease(lease)
    status.at_goal=status.request_stop=True;assert node.on_local_execution_status(status)
    refresh(node,clock,xyz=(2.,0.,.55))
    node.follow_tick();assert node.follow['cancel'] is None and handle.terminal is None
    stop=StopReport(version=lease.version,execution_id=lease.execution_id,control_epoch=lease.control_epoch,
        sdk_session=lease.sdk_session,sdk_arm_generation=lease.sdk_arm_generation,sequence=1,
        stop_request_id=lease.confirmation_id,
        source_stamp=time_msg(clock.wall),nonzero_blocked=True,stop_submitted=True,transport_mode='live')
    node.on_stop_report(stop);node.follow_tick();assert node.follow['cancel'] is None
    stop.sequence=2;stop.measured_stop_confirmed=True;node.on_stop_report(stop)
    node.follow_tick();assert node.follow['cancel'] is None  # SDK bool alone is not BT's checked stop.
    lease.sequence+=1;lease.phase='terminal';lease.reason='goal_reached'
    lease.allowed=False;lease.revoked=True
    assert node.on_task_lease(lease)
    node.follow_tick()
    assert node.follow['cancel']['reason']=='goal_reached' and node.follow['measured_arrival_stamp']==clock.wall


@pytest.mark.parametrize('bad',['task','route','hash','epoch','seed','mode','execution','control','future'])
def test_task_follower_foreign_status_cannot_fail_or_retarget_route(monkeypatch,bad):
    node,clock,handle=typed_follow(monkeypatch);node.p['execution_architecture']='task_follower_v1'
    lease,status=follower_feedback(node,clock);assert node.on_task_lease(lease)
    if bad=='task':status.version.task_id='old_task'
    elif bad=='route':status.version.route_id='old_route'
    elif bad=='hash':status.version.route_hash='wrong'
    elif bad=='epoch':status.version.localization_epoch+=1
    elif bad=='seed':status.version.localization_seed_id='old_seed'
    elif bad=='mode':status.transport_mode='isolated_mock'
    elif bad=='execution':status.execution_id='old_execution'
    elif bad=='control':status.control_epoch+=1
    else:status.source_stamp=time_msg(clock.wall+.001)
    status.at_goal=status.request_stop=True
    assert not node.on_local_execution_status(status)
    assert node.local_goal_event is None and node.follow['cancel'] is None and handle.terminal is None


@pytest.mark.parametrize('bad',['expired','body_future','body_stamp','deadline_extended'])
def test_task_follower_ready_status_cannot_replace_original_body_lease(monkeypatch,bad):
    node,clock,handle=typed_follow(monkeypatch);node.p['execution_architecture']='task_follower_v1'
    lease,status=follower_feedback(node,clock);assert node.on_task_lease(lease)
    if bad=='expired':
        status.body_source_stamp=time_msg(clock.wall-.2);status.body_valid_until=time_msg(clock.wall-.1)
    elif bad=='body_future':status.body_source_stamp=time_msg(clock.wall+.001)
    elif bad=='body_stamp':status.body_source_stamp.nanosec=1_000_000_000
    else:status.body_valid_until=time_msg(clock.wall+.100000001)
    status.at_goal=status.request_stop=True
    assert not node.on_local_execution_status(status)
    assert node.local_goal_event is None and node.follow['cancel'] is None and handle.terminal is None


def test_task_follower_following_display_expires_with_original_body_not_status_heartbeat(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch);node.p['execution_architecture']='task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
    lease,status=follower_feedback(node,clock);assert node.on_task_lease(lease);assert node.on_local_execution_status(status)
    node.follow_tick();assert node.follow['wait_phase']=='following'
    clock.wall+=.11;clock.mono+=.11;refresh(node,clock)
    node.follow_tick()
    assert node.follow['wait_phase']!='following' and node.follow['cancel'] is None and handle.terminal is None


def scan_delivery_diagnostic(node):
    slot = node.follow
    node.scan.update(active_reference=True, owner_reference_stamp=slot['reference_stamp'],
        owner_task_id=node.snapshot['task_id'], owner_route_id=node.snapshot['route_id'],
        owner_route_hash=node.snapshot['digest'], owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=slot['reference_stamp_ns'])


@pytest.mark.parametrize('diagnostic_phase', ['failed_reference_geometry', 'reference_rejected_frame'])
@pytest.mark.parametrize('local_ready', [True, False])
def test_formal_local_feedback_is_not_overruled_by_diagnostic_errors(monkeypatch, diagnostic_phase, local_ready):
    node, clock, handle = typed_follow(monkeypatch)
    node.p['execution_architecture'] = 'task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    scan_delivery_diagnostic(node)
    node.scan.update(local_debug_phase=diagnostic_phase, spline_visual_valid=False)
    _, status = follower_feedback(node, clock)
    status.ready = local_ready
    status.holding = not local_ready
    status.reason = 'waiting_native_trajectory' if not local_ready else ''
    assert node.on_local_execution_status(status)
    node.follow_tick()
    assert node.follow['wait_phase'] == ('following' if local_ready else 'recovering_local_trajectory')
    assert node.follow['cancel'] is None and handle.terminal is None
    assert len(node.route_reference_pub.messages) == 1 and not node.worker_cancel_pub.messages


@pytest.mark.parametrize('local_case', ['missing', 'holding', 'expired'])
def test_diagnostic_green_cannot_substitute_for_formal_local_feedback(monkeypatch, local_case):
    node, clock, handle = typed_follow(monkeypatch)
    node.p['execution_architecture'] = 'task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    scan_delivery_diagnostic(node)
    node.scan['spline_visual_valid'] = True
    if local_case != 'missing':
        _, status = follower_feedback(node, clock)
        status.ready = local_case != 'holding'
        status.holding = local_case == 'holding'
        assert node.on_local_execution_status(status)
    if local_case == 'expired':
        clock.wall += .11; clock.mono += .11
        refresh(node, clock)
        scan_delivery_diagnostic(node)
        node.scan['spline_visual_valid'] = True
    node.follow_tick()
    assert node.follow['wait_phase'] == 'recovering_local_trajectory'
    assert node.follow['feedback_plan_id'] == -1
    assert node.follow['cancel'] is None and handle.terminal is None


@pytest.mark.parametrize('bad_progress', ['not-a-number', {}, [], float('nan'), float('inf'), -1., 10**1000])
def test_formal_display_diagnostics_cannot_interrupt_task_loop(monkeypatch, bad_progress):
    node, clock, handle = typed_follow(monkeypatch)
    node.p['execution_architecture'] = 'task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    _, status = follower_feedback(node, clock)
    status.trajectory_id = 31
    assert node.on_local_execution_status(status)
    node.scan.update(last_spline_id='bad-display-id', local_debug_progress_arc_m=bad_progress)
    node.tick()
    assert node.follow['wait_phase'] == 'following' and node.follow['cancel'] is None
    assert handle.feedback[-1].local_plan_id == 31 and handle.feedback[-1].progress_m == 0.
    assert handle.terminal is None and not node.worker_cancel_pub.messages


def formal_follow_before_first_reference(monkeypatch):
    """Actual typed callbacks, without replacing any freshness/identity method."""
    from test_continuous_reference_transport import state
    from test_continuous_reference_node import local_state_from_pair
    node,clock=harness(monkeypatch)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map',
                  local_state_enabled=True,freshness_s=.4)
    pair=state(t=clock.wall-10.)
    node.on_atomic_state(pair)
    node.on_local_state(local_state_from_pair(pair))
    # No consumer yet: retain the real prepared FollowRoute session without
    # publishing/committing a reference or fabricating an execution permit.
    node.route_reference_pub.subscribers=0
    _,handle=commit(node,clock)
    assert not node.follow['published'] and not node.route_reference_pub.messages
    clock.wall,clock.mono=100.1459,10.1459
    refresh(node,clock)
    pair=state(t=90.12)
    pair.posterior_stamp=time_msg(99.73)  # At-source posterior age: 390 ms.
    pair.imu_stamp=time_msg(100.0254)     # At-source propagation: 94.6 ms.
    pair.extrapolation_sec=.0946
    node.on_atomic_state(pair)
    node.on_local_state(local_state_from_pair(pair))
    assert node.local_inbox.latest.state.source_ns==100_120_000_000
    node.route_reference_pub.subscribers=1
    return node,clock,handle,pair


def test_formal_first_delivery_does_not_wait_for_downstream_map_diagnostic(monkeypatch):
    node, clock, handle, pair = formal_follow_before_first_reference(monkeypatch)
    node.p['execution_architecture'] = 'task_follower_v1'
    node.scan.update(sensor_ready=False, native_map_context_ready=False,
        native_map_context_fault='waiting_map_context')
    node.follow_tick()
    assert node.follow['published'] and not node.follow['committed']
    assert len(node.route_reference_pub.messages) == 1
    assert node.follow['wait_phase'] == 'waiting_reference'
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    assert node.follow['committed'] and node.follow['wait_phase'] == 'recovering_local_trajectory'
    assert node.local_execution_status is None and node.task_lease is None
    assert node.follow['cancel'] is None and handle.terminal is None
    node.follow_tick()
    assert len(node.route_reference_pub.messages) == 1


def test_formal_stable_recovery_counts_typed_source_not_diagnostic_heartbeat(monkeypatch):
    node, clock, handle = typed_follow(monkeypatch)
    node.p['execution_architecture'] = 'task_follower_v1'
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    slot = node.follow
    node.follow_tick()
    assert slot['recovery'].since is not None
    _, status = follower_feedback(node, clock)
    assert node.on_local_execution_status(status)
    node.follow_tick()
    assert slot['recovery'].good_samples == 1
    for _ in range(3):
        clock.wall += .01; clock.mono += .01
        refresh(node, clock)
        node.follow_tick()
    assert slot['recovery'].good_samples == 1
    for index in range(1, 5):
        clock.wall += .2; clock.mono += .2
        refresh(node, clock)
        status.sequence += 1
        status.source_stamp = status.body_source_stamp = time_msg(clock.wall)
        status.body_valid_until = time_msg(clock.wall+.1)
        assert node.on_local_execution_status(status)
        node.follow_tick()
    assert slot['recovery'].since is None and slot['recovery'].good_samples >= 3
    assert slot['cancel'] is None and handle.terminal is None


def test_formal_first_reference_accepts_valid_coast_without_execution_permit(monkeypatch):
    node,clock,handle,pair=formal_follow_before_first_reference(monkeypatch)
    slot=node.follow;digest=node.snapshot['digest']
    assert node.ready_local_task_identity()==(1,'seed','map')
    assert node.ready_local_identity()==(1,'seed','map')
    node.follow_tick()
    assert node.follow is slot and slot['published'] and not slot['committed']
    assert len(node.route_reference_pub.messages)==1
    sent=node.route_reference_pub.messages[0]
    assert sent.active and sent.task_id==handle.request.task_id
    assert sent.route_hash==digest and node.snapshot['digest']==digest
    assert node.execution_permit is None and not node.worker_cancel_pub.messages
    state=node.local_inbox.latest.state
    assert state.posterior_ns==99_730_000_000 and state.imu_ns==100_025_400_000
    assert state.valid_until_ns==100_220_000_000  # Not receipt + 100 ms.


@pytest.mark.parametrize('expired',['source','receipt'])
def test_formal_task_observation_lease_does_not_extend_control_or_refresh_duplicates(monkeypatch,expired):
    from test_continuous_reference_node import local_state_from_pair
    node,clock,handle,pair=formal_follow_before_first_reference(monkeypatch)
    slot=node.follow;event=node.local_inbox.latest
    # The 100 ms provider control lease is already over; the same identifiable
    # task may still submit its first workflow reference inside 400 ms.
    clock.wall,clock.mono=100.49,10.49
    refresh(node,clock)
    node.on_local_state(local_state_from_pair(pair))
    assert node.local_inbox.latest is event and event.received==10.1459
    assert node.ready_local_task_identity()==(1,'seed','map')
    with pytest.raises(ValueError,match='waiting_fresh_local_navigation_state'):
        node.ready_local_identity()
    node.follow_tick()
    assert slot['published'] and node.execution_permit is None
    assert handle.terminal is None and slot['cancel'] is None
    # Independently exhaust the source or receipt observation lease. Never
    # accept a duplicate callback as new acquisition/receipt evidence.
    clock.wall=100.520000001 if expired=='source' else 100.49
    clock.mono=10.51 if expired=='source' else 10.545900001
    refresh(node,clock)
    node.on_local_state(local_state_from_pair(pair))
    assert node.local_inbox.latest is event
    with pytest.raises(ValueError,match='waiting_fresh_local_navigation_state'):
        node.ready_local_task_identity()
    node.follow_tick()
    assert node.follow is slot and slot['cancel'] is None and handle.terminal is None
    assert len(node.route_reference_pub.messages)==1
    assert slot['wait_reason']=='waiting_current_localization'
    assert node.execution_permit is None and not node.worker_cancel_pub.messages


def test_formal_hard_reset_fences_coasting_task_and_queued_reference(monkeypatch):
    from test_continuous_reference_transport import state
    from test_continuous_reference_node import local_state_from_pair
    node,clock,handle,pair=formal_follow_before_first_reference(monkeypatch)
    slot=node.follow
    reset=state(t=90.15,epoch=2,usable=False)
    reset.localization_seed_id='reset'
    clock.wall,clock.mono=100.16,10.16
    node.on_local_state(local_state_from_pair(reset,usable=False,reason='lio_reset'))
    event=node.local_inbox.latest
    assert event.hard_failure and slot['cancel'] is not None
    node.on_local_state(local_state_from_pair(pair))
    assert node.local_inbox.latest is event
    node.follow_tick()
    assert not [v for v in node.route_reference_pub.messages if v.active]
    assert node.execution_permit is None
    assert slot['cancel']['reason']=='local_navigation_hard_loss'


def delivery_receipt(node,clock,*,active=True,status=None,required=False,confirmed=False,generation=0):
    delivery=next(value for value in reversed(node.route_reference_pub.messages) if value.active==active)
    return RouteDeliveryReceipt(schema_version=1,source_stamp=deepcopy(delivery.source_stamp),
        receipt_stamp=time_msg(clock.wall),session_id=delivery.session_id,task_id=delivery.task_id,
        route_id=delivery.route_id,route_hash=delivery.route_hash,delivery_sequence=delivery.delivery_sequence,
        active=active,status=(RouteDeliveryReceipt.ACCEPTED if active else RouteDeliveryReceipt.RETIRED)
            if status is None else status,native_retirement_required=required,
        native_retirement_confirmed=confirmed,native_generation=generation,reason='test_receipt')


def test_typed_transmission_does_not_commit_or_allow_json_acceptance(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch)
    slot=node.follow
    assert slot['published'] and not slot['committed']
    node.scan.update(active_reference=True,spline_visual_valid=True,local_debug_phase='accepted',
        owner_task_id=handle.request.task_id,owner_route_id=node.snapshot['route_id'],
        owner_route_hash=node.snapshot['digest'],owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=slot['reference_stamp_ns'])
    node.follow_tick()
    assert not slot['committed'] and slot['wait_reason']=='waiting_reference_acceptance'
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
    assert slot['committed'] and slot['wait_phase']=='following'
    assert len(node.route_reference_pub.messages)==1 and not node.worker_cancel_pub.messages


def test_typed_acceptance_without_native_reference_is_not_executable_trajectory(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch)
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
    slot=node.follow
    assert slot['committed'] and slot['wait_reason']=='waiting_native_reference'
    assert slot['wait_phase']=='recovering_local_trajectory'
    assert handle.terminal is None and node.execution_permit is None


@pytest.mark.parametrize('status',[RouteDeliveryReceipt.ACCEPTED,RouteDeliveryReceipt.REJECTED])
def test_delayed_typed_disposition_keeps_original_software_fact_not_motion_lease(monkeypatch,status):
    node,clock,handle=typed_follow(monkeypatch)
    slot=node.follow
    receipt=delivery_receipt(node,clock,status=status)
    original=deepcopy(receipt)
    source_ns=slot['reference_stamp_ns']
    clock.wall+=.6;clock.mono+=.6
    refresh(node,clock)
    node.scan.update(active_reference=True,spline_visual_valid=True,local_debug_phase='accepted',
        owner_task_id=handle.request.task_id,owner_route_id=node.snapshot['route_id'],
        owner_route_hash=node.snapshot['digest'],owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=source_ns)
    assert node.on_route_delivery_receipt(receipt)
    assert slot['active_receipt']==original and receipt==original
    assert slot['committed'] is (status==RouteDeliveryReceipt.ACCEPTED)
    assert slot['wait_phase']==('following' if status==RouteDeliveryReceipt.ACCEPTED else 'waiting_reference')
    assert handle.terminal is None and slot['cancel'] is None
    assert node.execution_permit is None and node.stop_report is None
    assert slot['reference_stamp_ns']==source_ns and len(node.route_reference_pub.messages)==1


def test_delayed_formal_route_acceptance_does_not_refresh_expired_local_control_evidence(monkeypatch):
    node,clock,handle,pair=formal_follow_before_first_reference(monkeypatch)
    node.follow_tick()
    slot=node.follow
    receipt=delivery_receipt(node,clock)
    event=node.local_inbox.latest
    original_state=deepcopy(event.state)
    original_received=event.received
    clock.wall+=.6;clock.mono+=.6
    assert node.on_route_delivery_receipt(receipt)
    assert slot['committed'] and slot['active_receipt']==receipt
    assert node.local_inbox.latest is event and event.state==original_state
    assert event.received==original_received
    assert slot['wait_reason']=='waiting_current_localization'
    assert node.execution_permit is None and node.stop_report is None
    assert not node.worker_cancel_pub.messages and len(node.route_reference_pub.messages)==1
    assert handle.terminal is None and slot['cancel'] is None


@pytest.mark.parametrize('field,value',[
    ('schema_version',2),('session_id','foreign'),('task_id','old-task'),('route_id','foreign-route'),
    ('route_hash','f'*64),('delivery_sequence',0),('active',False),('status',RouteDeliveryReceipt.RETIRED),
    ('source_stamp',time_msg(100.09)),('receipt_stamp',time_msg(99.5)),
    ('receipt_stamp',time_msg(100.100001))])
def test_typed_acceptance_rejects_foreign_queued_and_wrong_source(monkeypatch,field,value):
    node,clock,handle=typed_follow(monkeypatch)
    receipt=delivery_receipt(node,clock)
    setattr(receipt,field,value)
    assert not node.on_route_delivery_receipt(receipt)
    assert not node.follow['committed'] and node.follow['active_receipt'] is None
    assert handle.terminal is None and len(node.route_reference_pub.messages)==1


def test_sent_but_rejected_route_retires_without_inventing_native_ack(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch)
    original=deepcopy(node.route_reference_pub.messages[0])
    rejection=delivery_receipt(node,clock,status=RouteDeliveryReceipt.REJECTED)
    rejection.reason='staging_context_quarantined'
    assert node.on_route_delivery_receipt(rejection)
    assert node.follow['published'] and not node.follow['committed']
    assert node.follow['wait_reason']=='reference_delivery_rejected:staging_context_quarantined'
    assert node.cancel_follow(handle)==CancelResponse.ACCEPT
    assert len(node.route_reference_pub.messages)==2 and not node.route_reference_pub.messages[-1].active
    clock.wall+=.1;clock.mono+=.1
    worker_retired(node,clock)
    node.follow_tick()
    # A rejected active delivery or empty JSON status is not retirement.
    assert handle.terminal is None and node.follow is not None
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock,active=False))
    assert handle.terminal=='canceled' and node.follow is None and not node.quarantine
    assert node.native_cancel_ack is None and node.snapshot is None
    assert original.snapshot.route_hash==node.route_reference_pub.messages[-1].snapshot.route_hash


def test_typed_retirement_waits_private_worker_and_retains_exact_event(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch)
    node.cancel_follow(handle)
    clock.wall+=.1;clock.mono+=.1
    assert node.on_route_delivery_receipt(delivery_receipt(node,clock,active=False))
    assert handle.terminal is None
    clock.wall+=.7;clock.mono+=.7
    worker_retired(node,clock)
    node.follow_tick()
    assert handle.terminal=='canceled' and not node.quarantine


@pytest.mark.parametrize('required,confirmed,generation',[(False,False,0),(True,True,4)])
def test_delayed_typed_retirement_closes_exact_old_follow_without_quarantine(monkeypatch,required,confirmed,generation):
    node,clock,handle=typed_follow(monkeypatch)
    slot=node.follow
    node.cancel_follow(handle)
    receipt=delivery_receipt(node,clock,active=False,required=required,
        confirmed=confirmed,generation=generation)
    original=deepcopy(receipt)
    clock.wall+=.6;clock.mono+=.6
    worker_retired(node,clock)
    assert node.on_route_delivery_receipt(receipt)
    assert receipt==original and slot['cancel']['receipt']==original
    assert handle.terminal=='canceled' and node.follow is None and not node.quarantine
    result=node.executions[id(handle)]['future'].result()
    assert result.retirement_confirmed and not result.physical_stop_confirmed
    assert len(node.route_reference_pub.messages)==2 and node.execution_permit is None


@pytest.mark.parametrize('field,value',[
    ('session_id','foreign'),('task_id','foreign'),('route_id','foreign'),('route_hash','d'*64),
    ('delivery_sequence',0),('source_stamp',time_msg(100.09)),
    ('receipt_stamp',time_msg(100.700001)),('receipt_stamp',time_msg(99.999999)),
    ('status',RouteDeliveryReceipt.ACCEPTED),('native_retirement_confirmed',True),
    ('native_generation',4)])
def test_delayed_retirement_preserves_delivery_time_identity_and_native_contract(monkeypatch,field,value):
    node,clock,handle=typed_follow(monkeypatch)
    node.cancel_follow(handle)
    receipt=delivery_receipt(node,clock,active=False)
    clock.wall+=.6;clock.mono+=.6
    worker_retired(node,clock)
    setattr(receipt,field,value)
    assert not node.on_route_delivery_receipt(receipt)
    assert node.follow is not None and node.follow['cancel'].get('receipt') is None
    assert handle.terminal is None and not node.quarantine


@pytest.mark.parametrize('required,confirmed,generation,valid',[
    (False,False,0,True),(False,True,0,False),(False,False,1,False),
    (True,False,4,False),(True,True,0,False),(True,True,4,True)])
def test_typed_native_retirement_flags_are_not_stop_or_empty_status(monkeypatch,required,confirmed,generation,valid):
    node,clock,handle=typed_follow(monkeypatch)
    node.cancel_follow(handle)
    clock.wall+=.1;clock.mono+=.1
    worker_retired(node,clock)
    result=node.on_route_delivery_receipt(delivery_receipt(node,clock,active=False,
        required=required,confirmed=confirmed,generation=generation))
    assert result is valid
    assert (handle.terminal=='canceled') is valid
    if valid:
        assert not node.executions[id(handle)]['future'].result().physical_stop_confirmed
    else:
        assert node.follow is not None and node.follow['cancel'].get('receipt') is None


@pytest.mark.parametrize('field,value',[
    ('delivery_sequence',1),('source_stamp',time_msg(100.09)),('task_id','old-task'),
    ('route_hash','e'*64),('active',True),('status',RouteDeliveryReceipt.REJECTED),
    ('receipt_stamp',time_msg(99.5))])
def test_typed_cancel_receipt_must_match_inactive_delivery_not_old_active(monkeypatch,field,value):
    node,clock,handle=typed_follow(monkeypatch)
    node.cancel_follow(handle)
    clock.wall+=.1;clock.mono+=.1
    worker_retired(node,clock)
    receipt=delivery_receipt(node,clock,active=False)
    setattr(receipt,field,value)
    assert not node.on_route_delivery_receipt(receipt)
    assert handle.terminal is None and node.follow['cancel'].get('receipt') is None


def test_late_acceptance_or_rejection_cannot_revive_canceled_or_retired_task(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch)
    acceptance=delivery_receipt(node,clock)
    node.cancel_follow(handle)
    assert not node.on_route_delivery_receipt(acceptance)
    assert not node.follow['committed'] and len(node.route_reference_pub.messages)==2
    clock.wall+=.1;clock.mono+=.1
    worker_retired(node,clock)
    retirement=delivery_receipt(node,clock,active=False)
    assert node.on_route_delivery_receipt(retirement)
    assert not node.on_route_delivery_receipt(retirement)
    assert not node.on_route_delivery_receipt(acceptance)
    assert handle.terminal=='canceled' and not node.snapshot


def test_old_receipts_cannot_retire_a_new_task_with_same_route_geometry(monkeypatch):
    node,clock,first=typed_follow(monkeypatch)
    acceptance=delivery_receipt(node,clock)
    node.cancel_follow(first)
    clock.wall+=.1;clock.mono+=.1
    worker_retired(node,clock)
    retirement=delivery_receipt(node,clock,active=False)
    assert node.on_route_delivery_receipt(retirement)
    # Preserve actual counters and task tombstones; identical XYZ never
    # implies identical ownership, even while old receipts are still fresh.
    clock.wall+=.01;clock.mono+=.01
    refresh(node,clock);worker_retired(node,clock)
    _,second=commit(node,clock,task='task-2',completion_stamp=clock.wall+.1,
                    completion_mono=clock.mono+.1)
    slot=node.follow
    assert slot['published'] and not slot['committed']
    assert not node.on_route_delivery_receipt(acceptance)
    assert not node.on_route_delivery_receipt(retirement)
    assert node.follow is slot and slot['cancel'] is None and second.terminal is None


def test_arrival_then_b_c_replacement_never_inherits_old_terminal_or_retirement(monkeypatch):
    from d1max_planning_interfaces.msg import StopReport
    node,clock,first=typed_follow(monkeypatch)
    node.p['execution_architecture']='task_follower_v1'
    first_acceptance=delivery_receipt(node,clock)
    assert node.on_route_delivery_receipt(first_acceptance)
    lease,status=follower_feedback(node,clock)
    assert node.on_task_lease(lease)
    status.at_goal=status.request_stop=True
    assert node.on_local_execution_status(status)
    refresh(node,clock,xyz=(2.,0.,.55))
    stop=StopReport(version=deepcopy(lease.version),execution_id=lease.execution_id,
        control_epoch=lease.control_epoch,sdk_session=lease.sdk_session,
        sdk_arm_generation=lease.sdk_arm_generation,sequence=1,
        stop_request_id=lease.confirmation_id,
        source_stamp=time_msg(clock.wall),nonzero_blocked=True,stop_submitted=True,
        measured_stop_confirmed=True,transport_mode='live')
    lease.sequence+=1;lease.phase='terminal';lease.reason='goal_reached'
    lease.allowed=False;lease.revoked=True
    assert node.on_task_lease(lease)
    node.on_stop_report(stop);node.follow_tick()
    assert node.follow['cancel']['reason']=='goal_reached'
    clock.wall+=.01;clock.mono+=.01
    worker_retired(node,clock)
    first_retirement=delivery_receipt(node,clock,active=False)
    assert node.on_route_delivery_receipt(first_retirement)
    first_result=node.executions[id(first)]['future'].result()
    assert first.terminal=='succeeded' and first_result.retirement_confirmed
    assert node.follow is None and node.snapshot is None

    retired_receipts=[first_retirement]
    for task in ('task-b','task-c'):
        clock.wall+=.01;clock.mono+=.01
        refresh(node,clock);worker_retired(node,clock)
        _,current=commit(node,clock,task=task,completion_stamp=clock.wall+.05,
                         completion_mono=clock.mono+.05)
        slot=node.follow
        assert node.task_lease is None and node.local_goal_event is None and node.stop_report is None
        # A's still-fresh final event and stop use the same session/geometry;
        # task identity, not source freshness, excludes them from B and C.
        assert not node.on_task_lease(lease)
        assert not node.on_local_execution_status(status)
        node.on_stop_report(stop)
        assert not node.on_route_delivery_receipt(first_acceptance)
        for receipt in retired_receipts:
            assert not node.on_route_delivery_receipt(receipt)
        assert node.cancel_follow(first)==CancelResponse.REJECT
        assert node.executions[id(first)]['future'].result() is first_result
        assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
        current_lease,current_status=follower_feedback(node,clock)
        assert node.on_task_lease(current_lease) and node.on_local_execution_status(current_status)
        node.follow_tick()
        assert node.follow is slot and slot['wait_phase']=='following'
        assert slot['cancel'] is None and current.terminal is None and not node.quarantine
        if task=='task-b':
            assert node.cancel_follow(current)==CancelResponse.ACCEPT
            clock.wall+=.01;clock.mono+=.01;worker_retired(node,clock)
            retired_receipts.append(delivery_receipt(node,clock,active=False))
            assert node.on_route_delivery_receipt(retired_receipts[-1])
            assert current.terminal=='canceled' and node.follow is None
    assert len(node.goal3d_pub.messages)==3
    assert len(node.route_reference_pub.messages)==5  # A active/inactive, B active/inactive, C active.


def test_duplicate_typed_acceptance_does_not_renew_or_redeliver_route(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch)
    receipt=delivery_receipt(node,clock)
    assert node.on_route_delivery_receipt(receipt)
    saved=deepcopy(node.follow['active_receipt'])
    since=node.follow['recovery'].since
    clock.wall+=.1;clock.mono+=.1
    duplicate=deepcopy(receipt);duplicate.receipt_stamp=time_msg(clock.wall)
    assert not node.on_route_delivery_receipt(duplicate)
    assert node.follow['active_receipt']==saved and node.follow['recovery'].since==since
    assert len(node.route_reference_pub.messages)==1 and handle.terminal is None


def test_typed_delivery_binding_exists_before_synchronous_receipt(monkeypatch):
    node,clock=harness(monkeypatch)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map')
    node.ready_identity=lambda:(1,'seed','map');node.local_ready=lambda context:True
    original_publish=node.route_reference_pub.publish
    def publish_with_receipt(message):
        original_publish(message)
        if message.active:
            assert node.on_route_delivery_receipt(delivery_receipt(node,clock))
    node.route_reference_pub.publish=publish_with_receipt
    _,handle=commit(node,clock)
    assert node.follow['published'] and node.follow['committed']
    assert len(node.route_reference_pub.messages)==1 and handle.terminal is None


def test_typed_cancel_timeout_requires_retired_receipt_even_with_native_json_ack(monkeypatch):
    node,clock,handle=typed_follow(monkeypatch)
    node.cancel_follow(handle)
    clock.wall+=.1;clock.mono+=.1
    node.scan.update(received_at_unix=clock.wall,active_reference=False,generation=4)
    node.received['scan']=clock.mono
    debug=LocalPlanDebug(session_id='session',phase='cancelled',valid=False,generation=4)
    debug.header.frame_id='d1max_loc_odom';debug.header.stamp=time_msg(clock.wall)
    node.on_native_debug(debug)
    worker_retired(node,clock)
    node.follow_tick()
    assert handle.terminal is None
    clock.wall+=2.5;clock.mono+=2.5
    node.follow_tick()
    assert handle.terminal=='aborted' and node.quarantine=='cancel_ack_timeout_requires_session_restart'


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


def test_v3_received_candidate_wait_feedback_requires_exact_owner_and_candidate_debug(monkeypatch):
    node,clock=harness(monkeypatch);commit(node,clock)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map')
    node.ready_identity=lambda:(1,'seed','map')
    node.local_ready=lambda context:True
    slot=node.follow
    node.scan.update(active_reference=False,spline_visual_valid=False,
        native_reference_received=True,pending_reference_generation=2,
        local_debug_generation=1,local_debug_fresh=True,local_debug_phase='accepted',
        native_pending_debug_generation=2,native_pending_debug_fresh=True,
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
    node.scan['native_pending_debug_fresh']=False
    node.follow_tick()
    assert slot['wait_reason']=='waiting_observed_space' # diagnostic event, not permission lease
    for key,value in [('owner_task_id','old-task'),('native_pending_debug_generation',1),
                      ('native_pending_phase',''),('native_reference_received',False)]:
        original=node.scan[key];node.scan[key]=value
        node.follow_tick()
        assert slot['wait_reason']=='waiting_reference_acceptance'
        node.scan[key]=original
    node.scan.update(active_reference=True,spline_visual_valid=True)
    node.follow_tick()
    assert slot['wait_phase']=='following'
    assert slot['wait_reason']=='following_validated_local_trajectory'


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


def test_typed_unknown_wait_ignores_legacy_refresh_and_keeps_fixed_route(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    node.p.update(pipeline_contract='single_floor_v3', map_version_id='map')
    node.ready_identity = lambda: (1, 'seed', 'map')
    node.local_ready = lambda context: True
    slot = node.follow
    original = deepcopy(node.route_reference_pub.messages[0])
    node.scan.update(active_reference=False, spline_visual_valid=False,
        native_reference_received=True, pending_reference_generation=2,
        local_debug_generation=2, native_pending_debug_generation=2,
        native_pending_phase='waiting_observed_space',
        owner_task_id=handle.request.task_id, owner_route_id=node.snapshot['route_id'],
        owner_route_hash=node.snapshot['source_snapshot'].route_hash,
        owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=slot['reference_stamp_ns'])
    node.follow_tick()
    # Replay the old callback's actual count/deadline triggers while a native
    # UNKNOWN outcome waits under the BT's common 30 s recovery budget.
    for index, elapsed in enumerate((.1, 12.1, 13., 29.), 1):
        clock.wall, clock.mono = 100.+elapsed, 10.+elapsed
        node.scan['received_at_unix'] = clock.wall
        node.received['scan'] = clock.mono
        node.on_refresh(String(data=json.dumps(dict(session_id='session',
            received_at_unix=clock.wall, motion_enabled=False,
            localization_epoch=1, localization_seed_id='seed',
            rejected_reference_stamp=slot['reference_stamp'], request_id=index))))
        node.follow_tick()
        assert slot['wait_reason'] == 'waiting_observed_space'
        assert slot['cancel'] is None and handle.terminal is None
    assert node.route_reference_pub.messages == [original]
    assert len(node.reference_pub.messages) == 1
    assert len(node.goal3d_pub.messages) == 1  # no autonomous global retry
    assert not node.worker_cancel_pub.messages
    assert slot['refresh_count'] == 0


def test_legacy_refresh_count_does_not_preempt_shared_recovery_budget(monkeypatch):
    from d1max_pct_scan.navigation_runtime_policy import NavigationRuntimePolicy
    from d1max_pct_scan.bt_follow_policy import FollowRecovery
    node, clock = harness(monkeypatch)
    node.p.update(NavigationRuntimePolicy(
        keep_committed_route_on_local_failure=False).follow_parameters())
    _, handle = commit(node, clock)
    slot = node.follow
    assert isinstance(slot['recovery'], FollowRecovery)
    original_hash = node.snapshot['digest']
    started_wall, started_mono = clock.wall, clock.mono
    node.follow_tick()  # starts the sole monotonic episode at t=0
    for index in range(1, 27):
        clock.wall, clock.mono = started_wall+index*.5, started_mono+index*.5
        refresh(node, clock)
        node.on_refresh(String(data=json.dumps(dict(session_id='session',
            received_at_unix=clock.wall, motion_enabled=False,
            localization_epoch=1, localization_seed_id='seed',
            rejected_reference_stamp=slot['reference_stamp'], request_id=index))))
        node.follow_tick()
        assert slot['cancel'] is None and handle.terminal is None
        assert node.snapshot['digest'] == original_hash
    assert slot['refresh_count'] == 26  # diagnostics, not a terminal counter
    # Refreshing cannot extend the centralized deadline indefinitely either.
    clock.wall, clock.mono = started_wall+30.01, started_mono+30.01
    refresh(node, clock)
    node.follow_tick()
    assert slot['cancel']['reason'].startswith('follow_reference_timeout:')
    assert slot['recovery'].since == pytest.approx(started_mono)


@pytest.mark.parametrize('phase', [
    'failed_optimization', 'failed_reference_search', 'failed_reference_start_occupied',
    'EMERGENCY_STOP', 'waiting_native_trajectory'])
def test_formal_follow_keeps_route_through_long_local_failure_then_recovers_and_cancels(
        monkeypatch, phase):
    from d1max_pct_scan.navigation_runtime_policy import NavigationRuntimePolicy
    from test_continuous_reference_transport import state
    from test_continuous_reference_node import local_state_from_pair

    node, clock = harness(monkeypatch)
    node.p.update(NavigationRuntimePolicy().follow_parameters())
    node.p.update(pipeline_contract='single_floor_v3', map_version_id='map',
                  local_state_enabled=True, execution_architecture='task_follower_v1')

    def navigation_sample():
        pair = state(t=clock.wall-10.)
        node.on_atomic_state(pair)
        node.on_local_state(local_state_from_pair(pair))
        assert node.ready_identity() == node.ready_local_identity() == (1, 'seed', 'map')

    navigation_sample()
    compute, handle = commit(node, clock, completion_stamp=100.05, completion_mono=10.05)
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock))
    slot, snapshot = node.follow, node.snapshot
    assert slot['recovery'].params['keep_committed_route_on_local_failure'] is True
    original = deepcopy(node.route_reference_pub.messages[0])
    original_path = deepcopy(node.reference_pub.messages[0])
    scan, worker = deepcopy(node.scan), deepcopy(node.worker)
    started_wall, started_mono = clock.wall, clock.mono
    scan.update(active_reference=False, spline_visual_valid=False,
        native_reference_received=True, pending_reference_generation=2,
        native_pending_debug_generation=2, native_pending_phase=phase,
        owner_task_id=handle.request.task_id, owner_route_id=snapshot['route_id'],
        owner_route_hash=snapshot['digest'],
        owner_delivery_sequence=slot['reference_delivery_sequence'],
        owner_reference_stamp_ns=slot['reference_stamp_ns'])

    # Exercise production callbacks with new sensor/status samples, not a
    # replaced readiness predicate. The robot remains at the original pose.
    for index in range(1, 662):
        clock.wall, clock.mono = started_wall+index*.2, started_mono+index*.2
        navigation_sample()
        scan['received_at_unix'] = worker['received_at_unix'] = clock.wall
        node.on_scan_status(String(data=json.dumps(scan)))
        node.on_worker_status(String(data=json.dumps(worker)))
        _, local = follower_feedback(node, clock)
        local.sequence = index
        local.ready = False
        local.holding = True
        local.phase = 'holding'
        local.reason = phase
        assert node.on_local_execution_status(local)
        node.on_refresh(String(data=json.dumps(dict(session_id='session',
            received_at_unix=clock.wall, motion_enabled=False,
            localization_epoch=1, localization_seed_id='seed',
            rejected_reference_stamp=slot['reference_stamp'], request_id=index))))
        node.follow_tick()
        assert node.follow is slot and node.snapshot is snapshot
        assert slot['wait_phase'] == 'recovering_local_trajectory'
        assert slot['wait_reason'] == phase
        assert slot['cancel'] is None and handle.terminal is None and not slot['future'].done()
        assert node.snapshot['digest'] == original.route_hash
        assert slot['active_delivery'] == (original.session_id, original.task_id,
            original.route_id, original.route_hash, original.delivery_sequence,
            original.source_stamp.sec*1_000_000_000+original.source_stamp.nanosec)

    assert clock.mono-started_mono > 120.
    assert slot['recovery'].waits['trajectory'] > 120.
    assert len(node.goal3d_pub.messages) == 1 and compute.terminal == 'succeeded'
    assert node.route_reference_pub.messages == [original]
    assert node.reference_pub.messages == [original_path]
    assert slot['refresh_count'] == 0 and not node.worker_cancel_pub.messages
    assert node.execution_permit is None and node.local_goal_event is None
    assert node.stop_report is None and slot['measured_arrival_stamp'] == 0.

    # Typed local-executor readiness must pass the actual identity/body lease
    # validator; a fresh status with expired body evidence cannot resume it.
    lease, status = follower_feedback(node, clock)
    status.sequence = 662
    lease.allowed = False  # Workflow readiness does not grant SDK motion.
    assert node.on_task_lease(lease)
    expired = deepcopy(status)
    expired.body_source_stamp = time_msg(clock.wall-.2)
    expired.body_valid_until = time_msg(clock.wall-.1)
    assert not node.on_local_execution_status(expired)
    node.follow_tick()
    assert slot['wait_phase'] == 'recovering_local_trajectory'
    assert node.on_local_execution_status(status)
    node.follow_tick()
    assert slot['wait_phase'] == 'following'
    assert slot['wait_reason'] == 'following_local_executor_trajectory'
    assert node.follow is slot and node.snapshot is snapshot and slot['cancel'] is None
    assert handle.terminal is None and node.local_goal_event is None
    assert node.route_reference_pub.messages == [original]
    assert node.reference_pub.messages == [original_path] and not node.worker_cancel_pub.messages

    # Persistence cannot swallow explicit cancellation or bypass retirement.
    assert node.cancel_follow(handle) == CancelResponse.ACCEPT
    assert slot['cancel']['reason'] == 'user_cancel' and handle.terminal is None
    assert len(node.worker_cancel_pub.messages) == 1
    assert len(node.route_reference_pub.messages) == len(node.reference_pub.messages) == 2
    assert not node.route_reference_pub.messages[-1].active
    assert not node.reference_pub.messages[-1].poses
    clock.wall += .1; clock.mono += .1
    worker_retired(node, clock)
    assert node.on_route_delivery_receipt(delivery_receipt(node, clock, active=False,
        required=True, confirmed=True, generation=4))
    assert handle.terminal == 'canceled' and node.follow is None and not node.quarantine
    result = node.executions[id(handle)]['future'].result()
    assert result.retirement_confirmed and not result.physical_stop_confirmed


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
    node.p.update(result_timeout_s=10.,worker_startup_timeout_s=60.,readiness_timeout_s=30.)
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


def test_worker_ready_localization_wait_has_its_own_fixed_budget(monkeypatch):
    node,clock,_,slot=pending_compute_for_deadline(monkeypatch)
    node.ready_identity=unavailable_localization
    advance_compute_clock(node,clock,40.);worker_retired(node,clock)
    node.compute_tick()
    assert slot['wait_phase']=='waiting_localization' and slot['cancel'] is None
    advance_compute_clock(node,clock,49.999);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel'] is None
    advance_compute_clock(node,clock,69.999);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel'] is None
    advance_compute_clock(node,clock,70.001);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel']['reason']=='compute_localization_wait_timeout'
    assert not slot['dispatched'] and not node.goal3d_pub.messages


def test_retreat_to_waiting_worker_does_not_reenter_or_renew_startup_budget(monkeypatch):
    node,clock,_,slot=pending_compute_for_deadline(monkeypatch)
    node.ready_identity=unavailable_localization
    advance_compute_clock(node,clock,40.);worker_retired(node,clock)
    node.compute_tick()
    advance_compute_clock(node,clock,45.)
    node.worker={};node.received['worker']=-float('inf');node.compute_tick()
    assert slot['wait_phase']=='waiting_worker' and slot['cancel'] is None
    advance_compute_clock(node,clock,70.001);node.compute_tick()
    assert slot['cancel']['reason']=='compute_localization_wait_timeout'
    assert not node.goal3d_pub.messages


def test_goal_dispatch_starts_bounded_solve_after_localization_wait(monkeypatch):
    node,clock,_,slot=pending_compute_for_deadline(monkeypatch)
    node.ready_identity=unavailable_localization
    advance_compute_clock(node,clock,40.);worker_retired(node,clock)
    node.compute_tick()
    advance_compute_clock(node,clock,48.);worker_retired(node,clock)
    node.ready_identity=lambda:(1,'seed','map');node.compute_tick()
    assert slot['dispatched'] and len(node.goal3d_pub.messages)==1
    advance_compute_clock(node,clock,57.999);worker_retired(node,clock);node.compute_tick()
    assert slot['cancel'] is None
    advance_compute_clock(node,clock,58.001);worker_retired(node,clock);node.compute_tick()
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
