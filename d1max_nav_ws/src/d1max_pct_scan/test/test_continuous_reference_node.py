from copy import deepcopy
from types import SimpleNamespace
import pytest
from d1max_planning_interfaces.msg import ReferenceReceipt,ExecutionPermit
from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from d1max_pct_scan.continuous_reference_transport import set_stamp
from test_continuous_reference_transport import state,envelope,BASE_NS


def harness():
    clock=SimpleNamespace(ns=BASE_NS,mono=100.)
    output=[]
    p=dict(session_id='session',map_version_id='map',expected_source_map_sha256='a'*64,
        expected_tomogram_sha256='c'*64,expected_conditioning_sha256='b'*64,body_height_m=.55,
        body_height_calibration_id='fixture',transport_mode='isolated_mock',floor_id='floor1')
    c=ReferenceCallbacks(p,lambda k,v:output.append((k,deepcopy(v))),lambda:clock.ns,lambda:clock.mono)
    c.on_navigation(state())
    c.on_context_ack(c.context)
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    return c,clock,output


def admit(c, sequence=1):
    proposal=c.proposal
    receipt=ReferenceReceipt(version=proposal.version,proposal_id=proposal.proposal_id,
        expected_version=proposal.expected_version,expected_trajectory_id=proposal.expected_trajectory_id,
        source_stamp=proposal.source_stamp,valid_until=proposal.valid_until,accepted=True,transport_mode='isolated_mock')
    assert c.on_receipt(receipt)
    permit=ExecutionPermit(version=proposal.version,source_stamp=proposal.source_stamp,
        valid_until=proposal.valid_until,phase='preview',geometry_committed=True,allowed=False,
        sequence=sequence,trajectory_id=sequence,transport_mode='isolated_mock')
    assert c.on_permit(permit)
    return permit


def test_async_support_does_not_block_body_updates_and_late_result_cannot_publish():
    from threading import Event
    from d1max_pct_scan.bounded_preparation import LatestPreparation
    c,clock,out=harness();c.p['local_state_enabled']=True
    started,release=Event(),Event()
    def prepare(request):
        started.set();assert release.wait(1.)
        return c.build_support(request)
    c.preparation=LatestPreparation(prepare)
    try:
        c.on_route(envelope());assert started.wait(1.)
        assert not [v for k,v in out if k in ('proposal','support')]
        clock.ns+=20_000_000;clock.mono+=.02
        assert c.on_local_navigation(local_state_from_pair(state(t=.02,x=.005)))
        assert [v for k,v in out if k=='body'][-1].header.stamp==state(t=.02).source_stamp
        # Expire the original geometry, then complete the worker. No restamp.
        clock.ns+=500_000_000;clock.mono+=.5
        release.set()
        with c.preparation._cv:
            assert c.preparation._cv.wait_for(lambda:c.preparation._result is not None,timeout=1.)
        c.tick()
        assert not [v for k,v in out if k in ('proposal','support')]
    finally:release.set();assert c.close()


def test_async_support_uses_same_geometry_and_source_as_synchronous_owner():
    from d1max_pct_scan.bounded_preparation import LatestPreparation
    expected,_,sync=harness();expected.on_route(envelope())
    c,clock,out=harness();c.preparation=LatestPreparation(c.build_support)
    try:
        c.on_route(envelope())
        with c.preparation._cv:
            assert c.preparation._cv.wait_for(lambda:c.preparation._result is not None,timeout=1.)
        c.tick()
        assert [v for k,v in out if k=='proposal']==[v for k,v in sync if k=='proposal']
        assert [v for k,v in out if k=='support']==[v for k,v in sync if k=='support']
    finally:assert c.close()


def test_cancel_during_async_support_retirement_cannot_publish_old_proposal():
    from threading import Event
    from d1max_pct_scan.bounded_preparation import LatestPreparation
    c,clock,out=harness();started,release,done=Event(),Event(),Event()
    def prepare(request):
        started.set();assert release.wait(1.)
        result=c.build_support(request);done.set();return result
    c.preparation=LatestPreparation(prepare)
    try:
        c.on_route(envelope());assert started.wait(1.)
        c.on_route(envelope(sequence=2,active=False))
        release.set();assert done.wait(1.)
        c.tick()
        assert not [v for k,v in out if k in ('proposal','support')]
        assert c.transport.route is None
    finally:release.set();assert c.close()


def test_real_callbacks_prepare_once_receipt_is_not_commit_no_duplicate_on_correction():
    c,clock,out=harness();c.on_route(envelope())
    assert c.proposal.reference.path.header.frame_id=='d1max_loc_odom'
    assert c.proposal.reference.point_reference=='body_center'
    assert c.transport.accepted is None
    p=admit(c)
    accepted=deepcopy(c.transport.accepted)
    clock.ns+=100_000_000;clock.mono+=.1
    c.on_navigation(state(t=.1,x=.05,correction=.01))
    assert c.transport.accepted==accepted and c.transport.core.candidate_anchor
    assert len([x for x in out if x[0]=='proposal'])==1
    assert not p.allowed


def test_wrong_or_uncommitted_permit_cannot_admit_candidate():
    c,clock,out=harness();c.on_route(envelope())
    p=c.proposal
    permit=ExecutionPermit(version=p.version,source_stamp=p.source_stamp,valid_until=p.valid_until,
        geometry_committed=False,sequence=1,transport_mode='isolated_mock')
    assert not c.on_permit(permit)
    permit.geometry_committed=True
    assert not c.on_permit(permit) # no receipt
    assert c.transport.accepted is None


def test_route_progress_only_reports_accepted_core_original_body_source():
    c,clock,out=harness();c.on_route(envelope())
    assert not [v for k,v in out if k=='route_progress']
    p=admit(c)
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==1
    r=reports[-1]
    assert r.schema_version==1 and r.version==p.version
    assert r.body_source_stamp==r.odom_body_pose.header.stamp==state().source_stamp
    assert r.odom_body_pose.pose==state().local_odometry.pose.pose
    assert r.anchor_source_stamp==state().source_stamp
    assert r.measured_arc_m==r.confirmed_arc_m==pytest.approx(0.)
    assert r.source_map_body_xyz.z==pytest.approx(.55)
    assert not c.publish_route_progress()
    clock.ns+=100_000_000;clock.mono+=.1
    c.on_navigation(state(t=.1,x=.05,correction=.1))
    latest=[v for k,v in out if k=='route_progress'][-1]
    assert latest.version==p.version  # correction is staged, not the incumbent anchor
    assert latest.source_map_body_xyz.x==latest.measured_arc_m==pytest.approx(.05)
    assert latest.map_from_odom==r.map_from_odom
    assert latest.body_source_stamp==state(t=.1).source_stamp
    assert not c.publish_route_progress()


def test_local_only_progress_survives_global_gap_and_has_no_restamped_source():
    from d1max_planning_interfaces.msg import LocalNavigationState
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());admit(c)
    global_source=deepcopy(c.state.source_ns)
    for i in range(1,11):
        clock.ns=BASE_NS+i*100_000_000;clock.mono=100.+i*.1
        pair=state(t=i*.1,x=i*.03)
        m=LocalNavigationState(schema_version=1,session_id='session',map_version_id='map',
            localization_epoch=1,localization_seed_id='seed',usable=True,
            local_odometry=pair.local_odometry,source_stamp=pair.source_stamp,
            posterior_stamp=pair.posterior_stamp,imu_stamp=pair.imu_stamp)
        assert c.on_local_navigation(m)
    r=[v for k,v in out if k=='route_progress'][-1]
    assert c.state.source_ns==global_source
    assert r.measured_arc_m==pytest.approx(.3)
    assert r.body_source_stamp==m.source_stamp and r.odom_body_pose.pose==m.local_odometry.pose.pose
    before=len([v for k,v in out if k=='route_progress'])
    assert not c.on_local_navigation(m)
    assert len([v for k,v in out if k=='route_progress'])==before


def local_state_from_pair(pair, *, usable=True, reason=''):
    from d1max_planning_interfaces.msg import LocalNavigationState
    return LocalNavigationState(schema_version=1,session_id=pair.session_id,
        map_version_id=pair.map_version_id,localization_epoch=pair.localization_epoch,
        localization_seed_id=pair.localization_seed_id,usable=usable,reason=reason,
        local_odometry=deepcopy(pair.local_odometry),source_stamp=deepcopy(pair.source_stamp),
        posterior_stamp=deepcopy(pair.posterior_stamp),imu_stamp=deepcopy(pair.imu_stamp),
        extrapolation_sec=pair.extrapolation_sec)


def test_global_pair_ahead_of_local_cannot_replace_literal_route_progress_source():
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());permit=admit(c)
    assert not [v for k,v in out if k=='route_progress']
    assert c.on_local_navigation(local_state_from_pair(state()))
    first=[v for k,v in out if k=='route_progress'][-1]
    for i in range(1,6):
        clock.ns=BASE_NS+i*100_000_000;clock.mono=100.+i*.1
        pair=state(t=i*.1,x=i*.03,correction=.1)
        count=len([v for k,v in out if k=='route_progress'])
        assert c.on_navigation(pair)
        assert c.transport.core.candidate_anchor  # Real pairs still stage the anchor.
        heartbeat=deepcopy(permit);heartbeat.sequence=i+1
        heartbeat.source_stamp=deepcopy(pair.source_stamp)
        set_stamp(heartbeat.valid_until,clock.ns+400_000_000)
        assert c.on_permit(heartbeat)
        assert not c.publish_route_progress()  # Permit/timer paths cannot bypass the source fence.
        assert len([v for k,v in out if k=='route_progress'])==count
        assert c.on_local_navigation(local_state_from_pair(pair))
        current=[v for k,v in out if k=='route_progress'][-1]
        assert current.body_source_stamp==pair.source_stamp
        assert current.odom_body_pose.pose==pair.local_odometry.pose.pose
        assert current.map_from_odom==first.map_from_odom
        assert current.measured_arc_m==pytest.approx(i*.03)
        assert len([v for k,v in out if k=='route_progress'])==count+1
        assert not c.publish_route_progress()  # Exact duplicates never renew the observation.


def test_global_projection_same_source_wrong_local_pose_cannot_publish_progress():
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());admit(c)
    assert c.on_local_navigation(local_state_from_pair(state()))
    clock.ns+=100_000_000;clock.mono+=.1
    pair=state(t=.1,x=.03)
    assert c.on_navigation(pair)
    wrong_pose=local_state_from_pair(state(t=.1,x=.031))
    assert c.on_local_navigation(wrong_pose)
    assert not c.publish_route_progress()
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==1 and reports[0].body_source_stamp==state().source_stamp
    clock.ns+=20_000_000;clock.mono+=.02
    actual=local_state_from_pair(state(t=.12,x=.036))
    assert c.on_local_navigation(actual)
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==2 and reports[-1].body_source_stamp==actual.source_stamp
    assert reports[-1].measured_arc_m==pytest.approx(.036)


def test_global_pair_cannot_bypass_explicit_unusable_local_progress_source():
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());admit(c)
    assert c.on_local_navigation(local_state_from_pair(state()))
    clock.ns+=100_000_000;clock.mono+=.1
    pair=state(t=.1,x=.03)
    assert not c.on_local_navigation(local_state_from_pair(pair,usable=False,reason='imu_age'))
    assert c.on_navigation(pair)
    assert not c.publish_route_progress()
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==1 and reports[0].body_source_stamp==state().source_stamp


def test_pending_anchor_does_not_report_reprojected_candidate_as_task_motion():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old=deepcopy(c.transport.accepted)
    clock.ns+=100_000_000;clock.mono+=.1
    c.on_navigation(state(t=.1,correction=.1))
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    assert c.transport.pending_core.anchor!=c.transport.core.anchor
    assert c.transport.pending_core._progress.confirmed_arc_m==pytest.approx(.1)
    r=[v for k,v in out if k=='route_progress'][-1]
    assert r.version.anchor_id==old.anchor_id and r.confirmed_arc_m==pytest.approx(0.)
    assert r.version.reference_generation==old.generation
    c.transport.quarantined=True
    assert not c.publish_route_progress()


def test_cancelled_route_cannot_refresh_old_progress():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    count=len([v for k,v in out if k=='route_progress'])
    c.on_route(envelope(sequence=2,active=False))
    assert not c.publish_route_progress()
    assert len([v for k,v in out if k=='route_progress'])==count


def test_stale_one_sensor_does_not_allow_reference_even_fresh_map_heartbeat():
    c,clock,out=harness()
    c.map_report['sources'][1]['integrated_stamp_ns']-=600_000_000
    c.on_route(envelope())
    assert c.transport.route is not None and c.proposal is None
    assert c.error=='reference_requires_acknowledged_fresh_native_map'
    assert not [v for k,v in out if k=='proposal']
    c.map_report['sources'][1]['integrated_stamp_ns']=clock.ns
    c.tick()
    assert c.proposal is not None
    assert c.error==''
    assert c.transport.route.source_stamp==envelope().source_stamp


def test_current_native_pending_debug_is_reported_without_claiming_valid_trajectory():
    from d1max_planning_interfaces.msg import LocalPlanDebug
    c,clock,_=harness();c.on_route(envelope())
    proposal=c.proposal
    receipt=ReferenceReceipt(version=proposal.version,proposal_id=proposal.proposal_id,
        expected_version=proposal.expected_version,expected_trajectory_id=proposal.expected_trajectory_id,
        source_stamp=proposal.source_stamp,valid_until=proposal.valid_until,accepted=True,transport_mode='isolated_mock')
    assert c.on_receipt(receipt)
    debug=LocalPlanDebug(session_id='session',generation=c.proposal.version.reference_generation,
        valid=False,phase='waiting_observed_space')
    debug.header.frame_id='d1max_loc_odom';set_stamp(debug.header.stamp,clock.ns)
    assert c.on_debug(debug)
    status=c.tick()
    assert status['local_debug_fresh'] and status['local_debug_generation']==debug.generation
    assert status['native_pending_phase']=='waiting_observed_space'
    assert not status['spline_visual_valid'] and not status['active_reference']
    clock.ns+=450_000_000;clock.mono+=.45
    status=c.tick()
    assert not status['local_debug_fresh']
    assert status['native_pending_phase']=='waiting_observed_space'
    assert not status['spline_visual_valid'] and not status['active_reference']
    assert c.debug.header.stamp==debug.header.stamp # no fabricated heartbeat
    c.debug.generation+=1
    assert c.tick()['native_pending_phase']==''


def test_candidate_expiry_keeps_committed_reference_and_context_ack_is_exact():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old=deepcopy(c.transport.accepted)
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    clock.ns+=500_000_000;clock.mono+=.5;c.tick()
    assert c.transport.accepted==old and c.transport.pending is None
    assert not c.on_context_ack(dict(c.context,epoch=2))


def test_cancel_same_source_stamp_new_sequence_retires_owner_without_motion():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    c.on_route(envelope(sequence=2,active=False))
    assert c.transport.accepted is None
    assert c.permit is None
    assert [v for k,v in out if k=='cancel_reference'][-1].path.poses==[]


def test_soft_delivery_refresh_acknowledges_latest_integer_sequence_without_moving_curve():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old=deepcopy(c.transport.accepted)
    clock.ns+=1;clock.mono+=.000000001
    c.on_route(envelope(t=1e-9,sequence=2))
    status=c.tick()
    assert status['owner_reference_stamp_ns']==BASE_NS+1
    assert status['owner_delivery_sequence']==2
    assert status['owner_task_id']=='task' and status['owner_route_id']=='route'
    assert status['owner_route_hash']==old.route_hash and c.transport.accepted==old


def test_cancel_before_map_ack_cannot_resurrect_cached_active_route():
    c,clock,out=harness();c.context_ready=False
    c.on_route(envelope())
    assert c.proposal is None and c.transport.route is not None
    c.on_route(envelope(sequence=2,active=False))
    c.on_context_ack(c.context);c.tick()
    assert c.transport.route is None
    assert not [v for k,v in out if k=='proposal']
    assert c.on_route(envelope()) is None


def test_goal_yaw_uses_frozen_map_goal_and_fixed_anchor_not_window_tangent():
    import math
    from d1max_pct_scan.source_route_ros import from_message,to_message
    c,clock,out=harness()
    message=state(t=.01)
    message.global_odometry.pose.pose.orientation.z=math.sin(.3/2)
    message.global_odometry.pose.pose.orientation.w=math.cos(.3/2)
    clock.ns+=10_000_000;clock.mono+=.01;c.on_navigation(message)
    route=envelope(t=.01)
    snapshot=from_message(route.snapshot).with_goal(has_goal_yaw=True,goal_yaw=.8,goal_yaw_tolerance_rad=.12)
    route.snapshot=to_message(snapshot,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=route.source_stamp)
    route.route_hash=snapshot.route_hash
    c.on_route(route)
    assert c.proposal.has_goal_yaw
    assert c.proposal.goal_yaw==pytest.approx(.5)
    assert c.proposal.goal_yaw_tolerance_rad==pytest.approx(.12)


def correction(c,clock,*,t,offset):
    clock.ns=BASE_NS+int(t*1e9);clock.mono=100.+t
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    return c.on_navigation(state(t=t,correction=offset))


def test_accumulated_map_correction_prepares_new_anchor_then_atomic_owner_commit():
    c,clock,out=harness();c.on_route(envelope());old_permit=admit(c)
    old_core=c.transport.core;old=deepcopy(c.transport.accepted)
    assert correction(c,clock,t=.1,offset=.03)
    assert c.proposal is None
    assert correction(c,clock,t=.6,offset=.12)
    candidate=c.proposal
    assert candidate is not None and candidate.version.anchor_id!=old.anchor_id
    assert candidate.expected_version==old_permit.version
    assert candidate.expected_trajectory_id==old_permit.trajectory_id
    assert candidate.reference.route_hash==old.route_hash
    assert candidate.reference.task_id==old.task_id
    assert candidate.goal_position.x==pytest.approx(1.88)
    assert candidate.reference.path.poses[0].pose.position.z==pytest.approx(.55)
    assert c.transport.core is old_core and c.transport.accepted==old
    new_permit=admit(c,sequence=2)
    assert c.transport.core is not old_core
    assert c.transport.core.anchor.anchor_id==new_permit.version.anchor_id
    assert c.transport.core.snapshot.route_hash==old_core.snapshot.route_hash
    assert c.transport.core._progress.measured_arc_m==pytest.approx(.12)
    assert old_core.anchor.anchor_id==old.anchor_id # old curve was never moved


def test_rejected_reanchor_retains_incumbent_and_late_old_commit_cannot_overwrite_new():
    c,clock,out=harness();c.on_route(envelope());old_permit=admit(c)
    old_core=c.transport.core;old=deepcopy(c.transport.accepted)
    correction(c,clock,t=.6,offset=.12)
    rejected=deepcopy(c.proposal)
    negative=ReferenceReceipt(version=rejected.version,proposal_id=rejected.proposal_id,
        expected_version=rejected.expected_version,expected_trajectory_id=rejected.expected_trajectory_id,
        source_stamp=rejected.source_stamp,valid_until=rejected.valid_until,
        accepted=False,transport_mode='isolated_mock')
    assert not c.on_receipt(negative)
    assert c.transport.core is old_core and c.transport.accepted==old
    assert c.transport.pending_core is None
    correction(c,clock,t=1.2,offset=.13)
    assert c.proposal and c.proposal.version!=rejected.version
    current=admit(c,sequence=3)
    committed_core=c.transport.core;committed=deepcopy(c.transport.accepted)
    late=deepcopy(current);late.version=rejected.version;late.sequence=4
    assert not c.on_permit(late)
    assert c.transport.core is committed_core and c.transport.accepted==committed
    late.version=old_permit.version
    assert not c.on_permit(late)


def test_expired_or_unprojectable_reanchor_never_replaces_incumbent():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old_core=c.transport.core;old=deepcopy(c.transport.accepted)
    correction(c,clock,t=.6,offset=.12)
    assert c.transport.pending_core is not old_core
    clock.ns+=500_000_000;clock.mono+=.5;c.tick()
    assert c.transport.pending_core is None and c.transport.core is old_core
    assert c.transport.accepted==old
    correction(c,clock,t=1.2,offset=.8)
    assert c.proposal is None and c.transport.pending_core is None
    assert c.transport.core is old_core and c.transport.accepted==old
    assert c.error=='measured_body_outside_current_semantic_segment'


def reset_debug(c,clock,generation):
    from d1max_planning_interfaces.msg import LocalPlanDebug
    message=LocalPlanDebug(session_id='session',generation=generation,phase='cancelled',valid=False)
    message.header.frame_id='d1max_loc_odom';set_stamp(message.header.stamp,clock.ns)
    return message


def test_hard_reset_requires_old_retirement_and_new_context_ack_then_new_task_only():
    from d1max_pct_scan.source_route_ros import from_message,to_message
    c,clock,out=harness();c.on_route(envelope());old_permit=admit(c)
    old_context=deepcopy(c.context)
    clock.ns+=100_000_000;clock.mono+=.1
    assert not c.on_navigation(state(t=.1,epoch=2))
    assert c.transport.quarantined and c.permit is None and c.transport.core._progress is None
    assert not c.on_permit(old_permit)
    assert not c.on_context_ack(old_context)
    assert c.on_context_ack(c.context)
    assert c.transport.quarantined # ACK alone cannot retire old task
    c.on_route(envelope(t=.1,sequence=2,active=False))
    c.on_debug(reset_debug(c,clock,c.cancel_generation-1))
    assert c.transport.quarantined # wrong generation cannot recover
    c.on_debug(reset_debug(c,clock,c.cancel_generation))
    assert not c.transport.quarantined and c.transport.core is None and c.transport.route is None
    assert c.permit is None and c.receipt is None and c.transport.accepted is None
    assert not c.on_permit(old_permit)
    with pytest.raises(ValueError,match='retired_committed_route'):
        c.on_route(envelope(t=.1,sequence=3))
    new=envelope(t=.1,sequence=3)
    new.task_id,new.route_id='new-task','new-route'
    new.snapshot=to_message(from_message(new.snapshot),session_id='session',task_id=new.task_id,
        route_id=new.route_id,epoch=2,seed_id='seed',stamp=new.source_stamp)
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    c.on_route(new)
    assert c.proposal.version.task_id=='new-task' and c.proposal.version.localization_epoch==2
    assert c.permit is None # a new owner confirmation is still required
    assert c.proposal.expected_trajectory_id==-1


@pytest.mark.parametrize('missing',['context_ack','fresh_body','native_ack'])
def test_hard_reset_cannot_recover_with_partial_or_stale_evidence(missing):
    c,clock,out=harness();c.on_route(envelope());admit(c)
    clock.ns+=100_000_000;clock.mono+=.1;c.on_navigation(state(t=.1,epoch=2))
    c.on_route(envelope(t=.1,sequence=2,active=False))
    if missing!='context_ack':c.on_context_ack(c.context)
    if missing=='fresh_body':clock.ns+=500_000_000;clock.mono+=.5
    if missing!='native_ack':c.on_debug(reset_debug(c,clock,c.cancel_generation))
    assert c.transport.quarantined and c.transport.core is None
    assert not c.try_recover_context()


def native_received(c):
    p=c.proposal
    ack=ReferenceReceipt(version=p.version,proposal_id=p.proposal_id,expected_version=p.expected_version,
        expected_trajectory_id=p.expected_trajectory_id,source_stamp=p.source_stamp,valid_until=p.valid_until,
        accepted=True,transport_mode='isolated_mock')
    assert c.on_receipt(ack)
    return deepcopy(p)


def fresh_commit(c,clock,proposal,sequence=1):
    p=ExecutionPermit(version=proposal.version,phase='preview',geometry_committed=True,allowed=False,
        sequence=sequence,trajectory_id=1,transport_mode='isolated_mock')
    set_stamp(p.source_stamp,clock.ns);set_stamp(p.valid_until,clock.ns+250_000_000)
    return c.on_permit(p)


def test_native_receipt_geometry_survives_delivery_deadline_until_fresh_owner_commit():
    # graph06: native accepted generation1, owner commit arrived 477ms after
    # proposal source. A 400ms delivery TTL must not discard already received
    # immutable geometry and start a generation flood.
    c,clock,out=harness();c.on_route(envelope());proposal=native_received(c)
    for i in range(1,31):
        correction(c,clock,t=.05*i,offset=0.)
        c.tick()
    assert c.transport.generation==1 and c.transport.pending is not None
    assert c.transport.accepted is None
    assert len([v for k,v in out if k=='proposal'])==1
    assert not [v for k,v in out if k=='cancel_reference']
    assert fresh_commit(c,clock,proposal)
    assert c.transport.accepted.generation==1
    assert c.transport.accepted.path.header.stamp==proposal.source_stamp # not renewed
    assert c.permit is not None and not c.permit.allowed


def test_received_geometry_is_not_permission_when_current_pose_is_stale():
    c,clock,out=harness();c.on_route(envelope());proposal=native_received(c)
    clock.ns+=600_000_000;clock.mono+=.6;c.tick()
    assert not fresh_commit(c,clock,proposal)
    assert c.transport.accepted is None and c.transport.pending is not None
    correction(c,clock,t=.65,offset=0.)
    assert fresh_commit(c,clock,proposal,sequence=2)


def test_missing_receipt_retry_is_bounded_and_never_a_cancel():
    c,clock,out=harness();c.on_route(envelope())
    for i in range(1,61):
        correction(c,clock,t=.05*i,offset=0.);c.tick()
    assert c.transport.generation<=7
    assert c.transport.accepted is None
    assert not [v for k,v in out if k=='cancel_reference']


def test_late_epoch_or_seed_cannot_replace_new_atomic_context():
    c,clock,out=harness()
    clock.ns+=100_000_000;clock.mono+=.1
    assert c.on_navigation(state(t=.1,epoch=2))
    current=deepcopy(c.context);body_source=c.state.source_ns
    assert not c.on_navigation(state(t=.09,epoch=1))
    clock.ns+=20_000_000;clock.mono+=.02
    assert not c.on_navigation(state(t=.11,epoch=1))
    message=state(t=.09,epoch=2);message.localization_seed_id='old-seed'
    assert not c.on_navigation(message)
    assert c.context==current and c.state.source_ns==body_source


def long_envelope(length=5):
    from d1max_pct_scan.source_route_ros import to_message
    from test_continuous_reference import snapshot
    route=snapshot(points=[[float(x),0.,0.] for x in range(length+1)])
    message=envelope()
    message.route_hash=route.route_hash
    message.snapshot=to_message(route,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=message.source_stamp)
    return message


def drive(c,clock,*,t,x):
    clock.ns=BASE_NS+int(t*1e9);clock.mono=100.+t
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    assert c.on_navigation(state(t=t,x=x))
    c.tick()


def test_late_owner_commit_cannot_exhaust_window_before_next_reissue():
    # long_straight_00_newnative: generation 3 was proposed ~1 m into its
    # predecessor but committed ~0.9 m later. Commit-relative progress then
    # needed 1 m more, native reached the 2 m window end first and waited
    # forever. The next window must follow remaining accepted look-ahead.
    c,clock,out=harness();c.on_route(long_envelope());admit(c)
    assert c.transport.accepted.path.poses[-1].pose.position.x==pytest.approx(2.)
    assert c.transport.remaining_window_arc_m()==pytest.approx(2.)
    t=x=0.
    while c.transport.pending is None:
        t+=.1;x+=.025;drive(c,clock,t=t,x=x)
        assert x<1.05, 'next window must be proposed at 1 m remaining look-ahead'
    assert x==pytest.approx(1.,abs=.03)
    second=native_received(c)
    end=second.reference.path.poses[-1].pose.position.x
    assert end==pytest.approx(x+2.,abs=.03)
    while x<1.9:  # native accepted geometry, owner commit arrives late
        t+=.1;x+=.025;drive(c,clock,t=t,x=x)
    assert c.transport.pending is not None
    assert fresh_commit(c,clock,second,sequence=2)
    assert c.transport.accepted.generation==second.version.reference_generation
    assert c.transport.remaining_window_arc_m()==pytest.approx(end-x,abs=.03)
    before=len([v for k,v in out if k=='proposal'])
    while c.transport.pending is None:
        t+=.1;x+=.025;drive(c,clock,t=t,x=x)
        assert x<end-.9, 'late commit must not delay the next window to the window end'
    assert end-x==pytest.approx(1.,abs=.03)
    assert len([v for k,v in out if k=='proposal'])==before+1


def test_window_covering_segment_end_is_not_reissued_without_correction():
    c,clock,out=harness();c.on_route(long_envelope(length=2));admit(c)
    assert c.transport.covers_current_segment_end()
    for i in range(1,30):
        drive(c,clock,t=.1*i,x=.025*i)
    assert c.transport.pending is None
    assert len([v for k,v in out if k=='proposal'])==1
