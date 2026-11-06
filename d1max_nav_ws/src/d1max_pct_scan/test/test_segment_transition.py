"""Software portal integration only. No ROS graph, SDK, or physical mode test."""
from copy import deepcopy
from dataclasses import replace

import pytest

from d1max_pct_scan.continuous_reference import (
    PreviewSegmentTransition, ReceiptModeConfirmation)
from d1max_pct_scan.continuous_reference_transport import set_stamp, nanoseconds
from d1max_pct_scan.continuous_reference_node import version_of
from d1max_pct_scan.segment_transition import SegmentTransitionInbox
from test_continuous_reference import at_portal, BASE_NS, CONTEXT
from test_continuous_reference_node import (harness, admit, drive, native_received,
    local_geometry_commit)
from test_continuous_reference_transport import envelope


def install_protocol_support_fixture(callbacks):
    """Explicit source double for wire/state tests, not a geometry benchmark.

    The separate reference-support tests exercise real classification/loaders.
    These predeclared rows let this suite exercise the production mixed packet
    builder without inventing support from the candidate's interpolated path.
    """
    import numpy as np
    from d1max_pct_scan.reference_support import ConditionedReferenceSupport
    from d1max_pct_scan.stair_support import StairSupportSelection
    class ProtocolSupport(ConditionedReferenceSupport):
        stair_model=None
        stair_model_sha256=''
        def __init__(self):
            self.rows={
                'floor1':np.array([[0.,0.,0.],[.5,0.,0.],[1.,0.,0.]]),
                'floor2':np.array([[0.,0.,1.],[.5,0.,1.],[1.,0.,1.]]),
                'stair_lower':np.array([[1.,0.,0.],[1.,0.,.25],[1.,0.,.5]]),
                'stair_upper':np.array([[1.25,0.,.5],[1.125,0.,.75],[1.,0.,1.]]),
                'stairs':np.array([[1.,0.,.5],[1.125,0.,.5],[1.25,0.,.5]])}
            from types import SimpleNamespace
            from scipy.spatial import cKDTree
            from d1max_pct_scan.source_route import SourceSupport
            stair=np.unique(np.concatenate([self.rows[k] for k in ('stair_lower','stair_upper','stairs')]),axis=0)
            self.source_support=SourceSupport({'floor1':self.rows['floor1'],
                'floor2':self.rows['floor2'],'stairs':stair},'a'*64)
            self.stair_index=SimpleNamespace(tree=cKDTree(stair),points=stair)
            self.stair_contact_source=frozenset()
        def nearby(self,center,radius,floor_id):return self.rows[floor_id]
        def classified_ground(self,*,source_map_sha256,floor_id,segment_kind,required_mode):
            return source_map_sha256=='a'*64 and floor_id in ('floor1','floor2') and required_mode=='general'
        def stair_surface(self,ground_reference,*,source_map_sha256,floor_id,segment_kind,required_mode):
            return StairSupportSelection(self.rows[floor_id],source_map_sha256=='a'*64 and required_mode=='stair',
                'original_stair_surface_returns_associated')
    callbacks.support_map=ProtocolSupport()


def test_sdk_receipt_confirmation_does_not_claim_body_source_time():
    ref=at_portal()
    # RobotState is slower than 50 Hz body state: a one-second-old real mode
    # callback remains distinct from the newer pose, not re-stamped as that pose.
    mode=ReceiptModeConfirmation(CONTEXT,'task',ref.snapshot.route_hash,'stair_lower','stair',
        BASE_NS-200_000_000,BASE_NS+2_000_000_000,100.,12,'sdk')
    assert ref.advance_segment(current_source_ns=BASE_NS+800_000_000,
        now_monotonic=100.8,mode=mode)=='stair_lower'
    assert mode.callback_ns==BASE_NS-200_000_000


@pytest.mark.parametrize('bad',['task','segment','mode','source','expired','receipt','basis','sequence'])
def test_callback_mode_confirmation_is_bound_and_fresh(bad):
    ref=at_portal()
    mode=ReceiptModeConfirmation(CONTEXT,'task',ref.snapshot.route_hash,'stair_lower','stair',
        BASE_NS,BASE_NS+2_000_000_000,100.,12,'sdk')
    changes={'task':dict(task_id='old-task'),'segment':dict(segment_id='stair_upper'),
        'mode':dict(mode='general'),'source':dict(callback_ns=BASE_NS-3_000_000_000),
        'expired':dict(valid_until_ns=BASE_NS+700_000_000),
        'receipt':dict(received_monotonic=90.),'basis':dict(time_basis='invented_source'),
        'sequence':dict(callback_sequence=0)}
    with pytest.raises(ValueError,match='portal_requires_bound_mode_evidence'):
        ref.advance_segment(current_source_ns=BASE_NS+800_000_000,now_monotonic=100.8,
            mode=replace(mode,**changes[bad]))


def test_preview_transition_cannot_be_used_by_an_execution_core():
    ref=at_portal()
    mode=PreviewSegmentTransition(CONTEXT,'task',ref.snapshot.route_hash,'stair_lower')
    with pytest.raises(ValueError,match='portal_requires_bound_mode_evidence'):
        ref.advance_segment(current_source_ns=BASE_NS+800_000_000,now_monotonic=100.8,mode=mode)
    ref.preview_only=True  # Fixture capability; production is fixed at session construction.
    assert ref.advance_segment(current_source_ns=BASE_NS+800_000_000,now_monotonic=100.8,mode=mode)=='stair_lower'


def task_lease(c,clock):
    from d1max_planning_interfaces.msg import TaskExecutionLease
    message=TaskExecutionLease(schema_version=1,version=version_of(c.transport.accepted),
        execution_id='execution',control_epoch=1,confirmation_id='confirmation',sdk_session='sdk',
        sdk_arm_generation=2,sequence=1,allowed=True,transport_mode='isolated_mock')
    set_stamp(message.source_stamp,clock.ns)
    set_stamp(message.valid_until,clock.ns+350_000_000)
    return message


def confirmed_state(request,clock,**overrides):
    from d1max_planning_interfaces.msg import ModeTransitionState
    message=ModeTransitionState(schema_version=1,version=deepcopy(request.version),
        request_id=request.request_id,request_sequence=request.sequence,sequence=1,
        target_segment_id=request.target_segment_id,target_mode=request.target_mode,
        transport_mode=request.transport_mode,execution_id=request.execution_id,
        control_epoch=request.control_epoch,sdk_session=request.sdk_session,
        sdk_arm_generation=request.sdk_arm_generation,phase='ready',reason='mock_confirmed',
        confirmed=True,capability_verified=True,time_basis='isolated_mock_callback_receipt',
        robot_callback_sequence=9,sdk_mode=3,motion_status=7,speed_level=1,
        control_owned=True,emergency_clear=True)
    for key in ('source_stamp','robot_callback_stamp'):set_stamp(getattr(message,key),clock.ns)
    set_stamp(message.valid_until,clock.ns+1_000_000_000)
    for key,value in overrides.items():setattr(message,key,value)
    return message


def commit_reference(c,clock,sequence=1):
    if c.geometry_owner=='local_executor':
        proposal=native_received(c)
        assert c.on_local_commit(local_geometry_commit(c,clock,proposal,
            sequence=sequence,trajectory=sequence))
    else:
        admit(c,sequence=sequence)


def execution_at_portal(**overrides):
    c,clock,out=harness(**overrides);c.on_route(envelope(crossfloor=True))
    commit_reference(c,clock)
    drive(c,clock,t=.8,x=.95)
    lease=task_lease(c,clock)
    assert c.on_task_lease(lease)
    assert c.continue_same_floor_segment()
    return c,clock,out,lease


def test_formal_reference_requests_mode_once_and_waits_without_cancelling_route():
    c,clock,out,lease=execution_at_portal()
    request=c.mode_transition.request
    assert request.target_mode=='stair' and request.current_mode=='general'
    assert request.version==version_of(c.transport.accepted)
    assert request.body_source_stamp==c.transport.accepted.path.header.stamp or nanoseconds(request.body_source_stamp)==clock.ns
    assert not request.preview_only and request.execution_id==lease.execution_id
    original=deepcopy(request)
    for _ in range(5):c.refresh_window()
    assert c.mode_transition.request==original
    assert len([value for key,value in out if key=='mode_request'])==1
    assert c.transport.accepted.segment_id=='lower_floor' and c.transport.pending is None
    assert not [value for key,value in out if key=='cancel_reference']


def test_actual_writer_mode_result_prepares_stair_then_geometry_owner_commits():
    c,clock,out,lease=execution_at_portal()
    request=c.mode_transition.request
    assert c.on_mode_transition(confirmed_state(request,clock))
    assert c.transport.pending.segment_id=='stair_lower'
    assert c.transport.accepted.segment_id=='lower_floor'
    assert c.proposal.reference.required_mode=='stair'
    assert c.proposal.reference.path.header.stamp==request.body_source_stamp
    admit(c,sequence=2)
    assert c.transport.accepted.segment_id=='stair_lower'
    assert c.transport.accepted.route_hash==request.version.route_hash


@pytest.mark.parametrize('bad',['capability','confirmed','target','request','sdk','mode','basis','callback','revoked'])
def test_wrong_or_unaccepted_mode_cannot_change_the_segment(bad):
    c,clock,out,lease=execution_at_portal()
    state=confirmed_state(c.mode_transition.request,clock)
    if bad=='capability':state.capability_verified=False
    elif bad=='confirmed':state.confirmed=False
    elif bad=='target':state.target_segment_id='stair_upper'
    elif bad=='request':state.request_id='old'
    elif bad=='sdk':state.sdk_session='old'
    elif bad=='mode':state.sdk_mode=1
    elif bad=='basis':state.time_basis='fake_vendor_stamp'
    elif bad=='callback':set_stamp(state.robot_callback_stamp,clock.ns-3_000_000_000)
    else:
        lease.sequence+=1;lease.revoked=True;assert c.on_task_lease(lease)
    c.on_mode_transition(state)
    assert c.transport.pending is None and c.transport.accepted.segment_id=='lower_floor'


def test_planning_only_crosses_portal_without_mode_or_execution_permission():
    c,clock,out=harness(execution_purpose='planning_only')
    c.on_route(envelope(crossfloor=True));admit(c)
    drive(c,clock,t=.8,x=.95)
    assert c.transport.pending.segment_id=='stair_lower'
    assert c.transport.accepted.segment_id=='lower_floor'
    assert c.proposal.reference.required_mode=='stair'
    assert not [value for key,value in out if key=='mode_request']
    assert c.task_lease is None and c.mode_transition.state is None


def test_mode_state_duplicates_never_refresh_callback_evidence():
    c,clock,_,lease=execution_at_portal()
    inbox=SegmentTransitionInbox('isolated_mock');inbox.begin(c.mode_transition.request)
    state=confirmed_state(c.mode_transition.request,clock)
    assert inbox.observe(state,now_ns=clock.ns,received_monotonic=clock.mono)
    assert not inbox.observe(state,now_ns=clock.ns+100_000_000,received_monotonic=clock.mono+.1)
    assert inbox.received==clock.mono
    assert nanoseconds(inbox.state.robot_callback_stamp)==clock.ns


def test_late_task_lease_cannot_restore_previous_execution_identity():
    c,clock,_,lease=execution_at_portal()
    newer=deepcopy(lease)
    newer.control_epoch+=1;newer.execution_id='new-execution';newer.sequence+=1
    assert c.on_task_lease(newer)
    lease.sequence=1000
    assert not c.on_task_lease(lease)
    assert c.task_lease.execution_id=='new-execution'


def test_mode_response_cannot_predate_the_bound_request():
    c,clock,_,_=execution_at_portal()
    state=confirmed_state(c.mode_transition.request,clock)
    set_stamp(state.source_stamp,clock.ns-1)
    assert not c.on_mode_transition(state)
    assert c.transport.pending is None


def advance_measured_body(c,clock,lease,*,t,x,z):
    """Separate real pose progression from mode and geometry acknowledgements."""
    from test_continuous_reference_transport import state
    clock.ns=BASE_NS+int(t*1e9);clock.mono=100.+t
    lease=deepcopy(lease)
    lease.version=version_of(c.transport.accepted)
    lease.sequence+=1
    set_stamp(lease.source_stamp,clock.ns)
    set_stamp(lease.valid_until,clock.ns+350_000_000)
    assert c.on_task_lease(lease)
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    message=state(t=t,x=x)
    for field in ('local_odometry','global_odometry'):
        getattr(message,field).pose.pose.position.z=z
    assert c.on_navigation(message)
    c.tick()
    return lease


@pytest.mark.parametrize('geometry_owner',['bt_writer','local_executor'])
def test_full_stair_entry_platform_exit_keep_task_after_rejected_candidate(geometry_owner):
    from d1max_planning_interfaces.msg import ReferenceReceipt
    c,clock,out,lease=execution_at_portal(geometry_owner=geometry_owner)
    original_route=deepcopy(c.transport.route)
    first_request=deepcopy(c.mode_transition.request)
    assert c.on_mode_transition(confirmed_state(first_request,clock))
    rejected=deepcopy(c.proposal)
    assert not c.on_receipt(ReferenceReceipt(version=rejected.version,
        proposal_id=rejected.proposal_id,expected_version=rejected.expected_version,
        expected_trajectory_id=rejected.expected_trajectory_id,
        source_stamp=rejected.source_stamp,valid_until=rejected.valid_until,
        accepted=False,reason='candidate_solver_failed',transport_mode='isolated_mock'))
    assert c.transport.accepted.segment_id=='lower_floor' and c.transport.pending is None
    # Retry geometrical preparation under the SAME confirmed mode transaction.
    c.refresh_window()
    assert c.transport.pending.segment_id=='stair_lower'
    assert c.mode_transition.request==first_request
    assert c.proposal.version.reference_generation>rejected.version.reference_generation
    commit_reference(c,clock,sequence=2)
    assert c.transport.accepted.segment_id=='stair_lower'

    # Walk the first flight. The landing boundary is stair->stair, so request
    # source semantics remain stair; SDK may confirm without another SetMode.
    for i in range(1,10):
        lease=advance_measured_body(c,clock,lease,t=.8+.1*i,x=1.,z=.55+.05*i)
    middle=deepcopy(c.mode_transition.request)
    assert middle.request_id!=first_request.request_id
    assert middle.current_mode==middle.target_mode=='stair'
    assert middle.target_segment_id=='stair_upper'
    assert c.transport.accepted.segment_id=='stair_lower'
    assert c.on_mode_transition(confirmed_state(middle,clock))
    commit_reference(c,clock,sequence=3)
    assert c.transport.accepted.segment_id=='stair_upper'

    # Exit requests GENERAL from fresh real mode feedback, not a stationary
    # flag or the old admission's general_low_speed_confirmed boolean.
    for i in range(1,11):
        lease=advance_measured_body(c,clock,lease,t=1.7+.1*i,x=1.,z=1.+.05*i)
    exit_request=deepcopy(c.mode_transition.request)
    assert exit_request.target_segment_id=='upper_floor'
    assert (exit_request.current_mode,exit_request.target_mode)==('stair','general')
    assert c.on_mode_transition(confirmed_state(exit_request,clock,sdk_mode=1,motion_status=5))
    commit_reference(c,clock,sequence=4)
    assert c.transport.accepted.segment_id=='upper_floor'
    assert c.transport.accepted.required_mode=='general'
    assert c.transport.route==original_route
    assert c.transport.accepted.task_id==first_request.version.task_id
    assert c.transport.accepted.route_hash==first_request.version.route_hash
    assert not [message for key,message in out if key=='cancel_reference']


def test_mode_hold_lease_does_not_deadlock_existing_execution_transaction():
    c,clock,_,lease=execution_at_portal()
    # The existing writer owns execution; mode switching intentionally holds
    # movement while retaining that grant. It must not need movement allowed.
    held=deepcopy(lease);held.sequence+=1;held.allowed=False;held.phase='holding'
    assert c.on_task_lease(held)
    assert c.on_mode_transition(confirmed_state(c.mode_transition.request,clock))
    assert c.transport.pending.segment_id=='stair_lower'


def test_downstairs_keeps_xyz_segment_order_and_returns_to_general_same_task():
    from d1max_pct_scan.source_route import RouteSnapshot
    from d1max_pct_scan.source_route_ros import from_message,to_message
    from test_continuous_reference_transport import state
    c,clock,out=harness(geometry_owner='local_executor')
    message=state(t=.02)
    for field in ('local_odometry','global_odometry'):
        getattr(message,field).pose.pose.position.z=1.55
    clock.ns+=20_000_000;clock.mono+=.02
    assert c.on_navigation(message)
    delivery=envelope(t=.02,crossfloor=True)
    value=from_message(delivery.snapshot).payload()
    count=len(value['xyz'])
    for key in ('xyz','layer_ids','source_layer_ids'):
        value[key].reverse()
    value['direction']='upper_to_lower';value['segments'].reverse()
    value['edge_segments']=[];value['point_floor_ids']=[]
    for segment in value['segments']:
        segment['begin_index'],segment['end_index']=(count-1-segment['end_index'],
            count-1-segment['begin_index'])
        segment['entry_portal_id'],segment['exit_portal_id']=(
            segment['exit_portal_id'],segment['entry_portal_id'])
        segment['source_layer_id'],segment['target_layer_id']=(
            segment['target_layer_id'],segment['source_layer_id'])
        segment['direction']=value['direction']
        if segment['required_mode']=='stair':segment['kind']='stair_down'
        edges=segment['end_index']-segment['begin_index']
        value['edge_segments'] += [segment['segment_id']]*edges
        value['point_floor_ids'] += [segment['floor_id']]*edges
    value['point_floor_ids'].append(value['segments'][-1]['floor_id'])
    snapshot=RouteSnapshot.create(value)
    delivery.route_hash=snapshot.route_hash
    delivery.snapshot=to_message(snapshot,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=delivery.source_stamp)
    c.on_route(delivery);commit_reference(c,clock)
    lease=task_lease(c,clock)
    lease=advance_measured_body(c,clock,lease,t=.8,x=.95,z=1.55)
    for sequence,target,start_z,start_t in (
            (2,'stair_upper',1.55,.8),(3,'stair_lower',1.1,1.7),
            (4,'lower_floor',.6,2.7)):
        request=deepcopy(c.mode_transition.request)
        assert request.target_segment_id==target
        mode,motion=(1,5) if target=='lower_floor' else (3,7)
        assert c.on_mode_transition(confirmed_state(request,clock,sdk_mode=mode,motion_status=motion))
        commit_reference(c,clock,sequence)
        assert c.transport.accepted.segment_id==target
        assert c.transport.accepted.route_hash==snapshot.route_hash
        if target=='lower_floor':break
        for i in range(1,10 if sequence==2 else 11):
            lease=advance_measured_body(c,clock,lease,t=start_t+.1*i,x=1.,z=start_z-.05*i)
    assert c.transport.accepted.required_mode=='general'
    assert not [message for key,message in out if key=='cancel_reference']


def stair_delivery_with_landing():
    from d1max_pct_scan.source_route import RouteSnapshot
    from d1max_pct_scan.source_route_ros import from_message,to_message
    delivery=envelope(t=.02,crossfloor=True)
    value=from_message(delivery.snapshot).payload()
    value['xyz'].insert(3,[1.25,0.,.5])
    value['layer_ids'].insert(3,8);value['source_layer_ids'].insert(3,8)
    platform=deepcopy(value['segments'][1])
    platform.update(segment_id='middle_landing',kind='landing',floor_id='stairs',
        begin_index=2,end_index=3,source_layer_id=8,target_layer_id=8,
        entry_portal_id='portal2',exit_portal_id='platform_exit')
    for segment in value['segments'][2:]:
        segment['begin_index']+=1;segment['end_index']+=1
    value['segments'][2]['entry_portal_id']='platform_exit'
    value['segments'].insert(2,platform)
    value['edge_segments']=[s['segment_id'] for s in value['segments']]
    value['point_floor_ids']=[s['floor_id'] for s in value['segments']]+['floor2']
    snapshot=RouteSnapshot.create(value)
    delivery.route_hash=snapshot.route_hash
    delivery.snapshot=to_message(snapshot,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=delivery.source_stamp)
    return delivery


def reversed_delivery(delivery):
    """Reverse literal route geometry; preserve terrain mode ownership."""
    from d1max_pct_scan.source_route import RouteSnapshot
    from d1max_pct_scan.source_route_ros import from_message,to_message
    value=from_message(delivery.snapshot).payload()
    count=len(value['xyz'])
    for key in ('xyz','layer_ids','source_layer_ids'):
        value[key].reverse()
    value['direction']='upper_to_lower';value['segments'].reverse()
    value['edge_segments']=[];value['point_floor_ids']=[]
    for segment in value['segments']:
        segment['begin_index'],segment['end_index']=(count-1-segment['end_index'],
            count-1-segment['begin_index'])
        segment['entry_portal_id'],segment['exit_portal_id']=(
            segment['exit_portal_id'],segment['entry_portal_id'])
        segment['source_layer_id'],segment['target_layer_id']=(
            segment['target_layer_id'],segment['source_layer_id'])
        segment['direction']=value['direction']
        if segment['kind'].startswith('stair_'):segment['kind']='stair_down'
        edges=segment['end_index']-segment['begin_index']
        value['edge_segments'] += [segment['segment_id']]*edges
        value['point_floor_ids'] += [segment['floor_id']]*edges
    value['point_floor_ids'].append(value['segments'][-1]['floor_id'])
    snapshot=RouteSnapshot.create(value)
    delivery.route_hash=snapshot.route_hash
    delivery.snapshot=to_message(snapshot,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=delivery.source_stamp)
    return delivery


@pytest.mark.parametrize('downstairs',[False,True])
@pytest.mark.parametrize('geometry_owner',['bt_writer','local_executor'])
@pytest.mark.parametrize('execution_purpose',['execution','planning_only'])
def test_fixed_stair_crosses_all_terrain_without_mode_transactions_and_retries_geometry(
        downstairs,geometry_owner,execution_purpose):
    from d1max_planning_interfaces.msg import ReferenceReceipt
    from d1max_pct_scan.source_route_ros import from_message
    from test_continuous_reference_transport import state
    c,clock,out=harness(geometry_owner=geometry_owner,execution_purpose=execution_purpose,
        execution_mode_policy='fixed_stair')
    install_protocol_support_fixture(c)
    clock.ns+=20_000_000;clock.mono+=.02
    initial=state(t=.02)
    if downstairs:
        for field in ('local_odometry','global_odometry'):
            getattr(initial,field).pose.pose.position.z=1.55
    assert c.on_navigation(initial)
    delivery=stair_delivery_with_landing()
    if downstairs:delivery=reversed_delivery(delivery)
    original=deepcopy(delivery)
    value=from_message(delivery.snapshot).payload()
    c.on_route(delivery);commit_reference(c,clock)
    assert c.transport.core.execution_mode_policy=='fixed_stair'
    assert not c.continue_same_floor_segment()  # Policy cannot bypass the measured XYZ portal.
    assert c.transport.accepted.required_mode=='general'
    assert c.prepared_for(version_of(c.transport.accepted)).support.required_mode=='general'
    sequence=1;rejected_once=False;t=.02
    observed=[c.transport.accepted.segment_id]
    for start,end in zip(value['xyz'],value['xyz'][1:]):
        for step in range(1,21):
            t+=.1;clock.ns=BASE_NS+round(t*1e9);clock.mono=100.+t
            c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
                sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
            sample=state(t=t)
            xyz=[a+(b-a)*step/20 for a,b in zip(start,end)]
            for field in ('local_odometry','global_odometry'):
                p=getattr(sample,field).pose.pose.position;p.x,p.y,p.z=xyz[0],xyz[1],xyz[2]+.55
            assert c.on_navigation(sample)
            c.tick()
            if c.transport.pending is not None:
                if not rejected_once and c.transport.pending.segment_id!=c.transport.accepted.segment_id:
                    rejected=deepcopy(c.proposal);old=deepcopy(c.transport.accepted)
                    assert not c.on_receipt(ReferenceReceipt(version=rejected.version,
                        proposal_id=rejected.proposal_id,expected_version=rejected.expected_version,
                        expected_trajectory_id=rejected.expected_trajectory_id,
                        source_stamp=rejected.source_stamp,valid_until=rejected.valid_until,
                        accepted=False,reason='candidate_solver_failed',transport_mode='isolated_mock'))
                    assert c.transport.pending is None and c.transport.accepted==old
                    c.refresh_window()
                    assert c.transport.pending is not None
                    rejected_once=True
                sequence+=1;commit_reference(c,clock,sequence)
                if observed[-1]!=c.transport.accepted.segment_id:
                    observed.append(c.transport.accepted.segment_id)
            assert c.transport.route==original
            assert c.transport.accepted.task_id==original.task_id
            assert c.transport.accepted.route_hash==original.route_hash
            segment=next(s for s in value['segments'] if s['segment_id']==c.transport.accepted.segment_id)
            assert c.transport.accepted.required_mode==segment['required_mode']
            support=c.prepared_for(version_of(c.transport.accepted)).support
            assert support.required_mode==segment['required_mode']
            assert support.segment_kind==segment['kind']
            assert c.mode_transition.request is None and c.mode_transition.state is None
            assert c.task_lease is None  # Geometry continuity does not manufacture SDK authorization.
    assert rejected_once
    assert observed==[segment['segment_id'] for segment in value['segments']]
    assert c.transport.accepted.required_mode=='general'
    assert c.prepared_for(version_of(c.transport.accepted)).support.required_mode=='general'
    assert not [message for key,message in out if key in ('mode_request','cancel_reference')]


def test_fixed_stair_rejects_late_mode_callbacks_and_cannot_request_a_switch():
    c,clock,out=harness(execution_mode_policy='fixed_stair')
    install_protocol_support_fixture(c)
    c.on_route(envelope(crossfloor=True));admit(c)
    before=(c.error,c.transport.core,c.transport.pending)
    from d1max_planning_interfaces.msg import ModeTransitionState
    assert not c.on_mode_transition(ModeTransitionState(reason='late_mode_switch_failed'))
    assert (c.error,c.transport.core,c.transport.pending)==before
    assert not c.request_mode_transition({}, {}, ())
    assert not [message for key,message in out if key=='mode_request']


@pytest.mark.parametrize('policy',['', 'stair', 'fixed_general', None, 1])
def test_unknown_execution_mode_policy_is_not_silently_interpreted(policy):
    with pytest.raises(ValueError,match='explicit_execution_mode_policy_required'):
        harness(execution_mode_policy=policy)
