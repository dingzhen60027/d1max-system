"""Measured portal events survive candidate latency, without a ROS graph."""
import pytest

from test_continuous_reference import observation, project, reference


def fixed_reference():
    core = reference(crossfloor=True)
    core.execution_mode_policy = 'fixed_stair'
    project(core, observation())
    return core


def rounded_stair_reference():
    from d1max_pct_scan.source_route import RouteSnapshot
    payload=reference(crossfloor=True).snapshot.payload()
    payload['xyz']=[[0.,0.,0.],[1.,0.,0.],[1.,.8,.2],[1.4,.8,1.],[2.2,.8,1.]]
    core=reference(source=RouteSnapshot.create(payload))
    core.execution_mode_policy='fixed_stair'
    project(core,observation())
    project(core,observation(x=.6,t=.5))
    project(core,observation(x=.85,y=.02,z=.554,t=.7))
    return core


def test_rounded_stair_crossing_uses_adjacent_xyz_not_a_centerline_stop_point():
    import math
    core=rounded_stair_reference()
    sample=observation(x=.85,y=.18,z=.586,t=.9)
    before=core._segment
    progress=project(core,sample)
    assert math.dist(sample.body.pose.xyz,(1.,0.,.55))>core.portal_radius
    assert core._arc[1]-progress.measured_arc_m>core.portal_radius
    assert core._portal_observation is progress
    assert core.reached_adjacent_portal(current_source_ns=sample.body.source_ns,
        now_monotonic=sample.received_monotonic)[1]['segment_id']=='stair_lower'
    staged=core.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,
        now_monotonic=sample.received_monotonic)
    assert core._segment==before==0 and staged._segment==1
    assert staged._progress.source_ns==sample.body.source_ns
    assert staged._progress.received_monotonic==sample.received_monotonic
    assert staged._progress.body_source_xyz==sample.body.pose.xyz
    assert staged.snapshot.route_hash==core.snapshot.route_hash
    assert staged.anchor==core.anchor


@pytest.mark.parametrize('problem',['wrong_floor','side_branch','duplicate','old_source','context'])
def test_rounded_portal_cannot_relabel_wrong_xyz_or_old_identity(problem):
    from dataclasses import replace
    core=rounded_stair_reference()
    sample=observation(x=.85,y=.18,z=.586,t=.9)
    if problem=='wrong_floor':sample=observation(x=.85,y=.18,z=1.586,t=.9)
    elif problem=='side_branch':sample=observation(x=.75,y=.4,z=.63,t=.9)
    elif problem=='duplicate':sample=observation(x=.85,y=.18,z=.586,t=.7)
    elif problem=='old_source':sample=observation(x=.85,y=.18,z=.586,t=.6)
    else:sample=replace(sample,body=replace(sample.body,context=replace(sample.body.context,epoch=2)))
    try:project(core,sample)
    except ValueError:pass
    assert core._portal_observation is None and core._segment==0
    with pytest.raises(ValueError):
        core.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,
            now_monotonic=sample.received_monotonic)


def test_return_across_rounded_boundary_retires_witness_before_geometry_commit():
    core=rounded_stair_reference()
    project(core,observation(x=.9,y=.14,z=.578,t=.9))
    assert core._portal_observation is not None
    # Less than 12 cm of along-route retreat still crosses the actual section.
    sample=observation(x=.86,y=.03,z=.556,t=1.1)
    project(core,sample)
    assert core._portal_observation is None
    with pytest.raises(ValueError,match='not_physically_reached'):
        core.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,
            now_monotonic=sample.received_monotonic)


def test_rounded_boundary_does_not_change_mode_transaction_or_flat_rules():
    from d1max_pct_scan.source_route import RouteSnapshot
    for flat in (False,True):
        core=rounded_stair_reference()
        if flat:
            payload=core.snapshot.payload()
            payload['point_floor_ids']=['floor1']*len(payload['xyz'])
            for segment in payload['segments']:
                segment.update(kind='floor',required_mode='general',floor_id='floor1')
            core=reference(source=RouteSnapshot.create(payload))
            core.execution_mode_policy='fixed_stair'
            project(core,observation());project(core,observation(x=.85,y=.02,z=.554,t=.7))
        else:core.execution_mode_policy='segment_modes'
        sample=observation(x=.85,y=.18,z=.586,t=.9)
        project(core,sample)
        assert core._portal_observation is None
        with pytest.raises(ValueError,match='not_physically_reached'):
            core.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,
                now_monotonic=sample.received_monotonic)


def test_measured_portal_crossing_survives_a_pending_candidate_delay():
    core = fixed_reference()
    # A candidate currently owns the publication slot. These remain real body
    # samples; acknowledging/rejecting that candidate does not move the robot.
    project(core, observation(x=.85, t=.7))
    portal = project(core, observation(x=.95, t=.8))
    sample = observation(x=1., z=.75, t=1.1)
    project(core, sample)
    assert core._segment == 0
    staged = core.prepare_segment_transition(sample,
        current_source_ns=sample.body.source_ns, now_monotonic=sample.received_monotonic)
    assert staged._segment == 1 and core._segment == 0
    assert staged._progress.body_source_xyz == sample.body.pose.xyz
    assert staged._progress.source_ns == sample.body.source_ns > portal.source_ns
    assert staged._progress.received_monotonic == sample.received_monotonic
    assert staged._progress.segment_id == 'stair_lower'
    assert staged.snapshot.route_hash == core.snapshot.route_hash


def test_nearby_upper_body_without_a_measured_portal_cannot_switch_segments():
    core = fixed_reference()
    project(core, observation(x=.7, t=.6))
    sample = observation(x=1., z=.75, t=1.)
    project(core, sample)
    with pytest.raises(ValueError, match='not_physically_reached'):
        core.prepare_segment_transition(sample, current_source_ns=sample.body.source_ns,
            now_monotonic=sample.received_monotonic)


def test_old_portal_observation_does_not_refresh_the_latest_body_timestamp():
    core = fixed_reference()
    sample = observation(x=.95, t=.8)
    project(core, sample)
    with pytest.raises(ValueError, match='fresh'):
        core.prepare_segment_transition(sample,
            current_source_ns=sample.body.source_ns+600_000_000,
            now_monotonic=sample.received_monotonic+.6)


def test_late_commit_can_project_current_successor_body_after_old_segment_ends():
    core=fixed_reference()
    project(core,observation(x=.95,t=.8))
    last_progress=core._progress
    sample=observation(x=1.,z=.95,t=1.2)
    with pytest.raises(ValueError,match='outside_current_semantic_segment'):
        project(core,sample)
    staged=core.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,
        now_monotonic=sample.received_monotonic)
    assert core._progress==last_progress  # No made-up incumbent progress.
    assert staged._progress.body_source_xyz==sample.body.pose.xyz
    assert staged._progress.source_ns==sample.body.source_ns
    assert staged._progress.segment_id=='stair_lower'


def test_delayed_portal_transition_also_recovers_when_old_projection_still_succeeds():
    from d1max_pct_scan.source_route import RouteSnapshot
    core=fixed_reference()
    payload=core.snapshot.payload()
    # A gently rising real stair/approach can remain inside the old floor's
    # height range while the body has travelled beyond the portal radius.
    payload['xyz'][2]=[1.8,0.,.2]
    payload['xyz'][3]=[1.8,0.,1.]
    payload['xyz'][4]=[.8,0.,1.]
    core=reference(source=RouteSnapshot.create(payload))
    core.execution_mode_policy='fixed_stair'
    project(core,observation());project(core,observation(x=.95,t=.8))
    sample=observation(x=1.5,z=.675,t=1.3)
    project(core,sample)  # Current floor projection legitimately still exists.
    staged=core.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,
        now_monotonic=sample.received_monotonic)
    assert staged._segment==1 and core._segment==0
    assert staged._progress.body_source_xyz==sample.body.pose.xyz
    assert staged._progress.source_ns==sample.body.source_ns


@pytest.mark.parametrize('problem',['teleport','upper_floor','context','stale'])
def test_witness_does_not_allow_foreign_stale_or_skipped_floor_body(problem):
    from dataclasses import replace
    core=fixed_reference();project(core,observation(x=.95,t=.8))
    sample=observation(x=1.,z=.95,t=1.2)
    current=sample.body.source_ns;now=sample.received_monotonic
    if problem=='teleport':sample=observation(x=1.,z=.95,t=.81);current=sample.body.source_ns;now=sample.received_monotonic
    elif problem=='upper_floor':sample=observation(x=1.,z=1.55,t=1.8);current=sample.body.source_ns;now=sample.received_monotonic
    elif problem=='context':sample=replace(sample,body=replace(sample.body,context=replace(sample.body.context,epoch=2)))
    else:current+=600_000_000;now+=.6
    with pytest.raises(ValueError):
        core.prepare_segment_transition(sample,current_source_ns=current,now_monotonic=now)
    assert core._segment==0


def test_retreat_discards_crossing_instead_of_reusing_old_portal_event():
    core=fixed_reference();project(core,observation(x=.95,t=.8))
    project(core,observation(x=.7,t=1.1))
    sample=observation(x=1.,z=.75,t=1.5);project(core,sample)
    with pytest.raises(ValueError,match='not_physically_reached'):
        core.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,
            now_monotonic=sample.received_monotonic)


def test_fixed_stair_lookahead_crosses_portals_without_advancing_measured_segment():
    core=fixed_reference()
    window=core.window(current_source_ns=core._progress.source_ns,
        now_monotonic=core._progress.received_monotonic,horizon_m=4.)
    assert window.ground_source_xyz[-1]==(0.,0.,1.)
    assert set(window.point_segment_ids)=={'lower_floor','stair_lower','stair_upper','upper_floor'}
    assert set(window.point_required_modes)=={'general','stair'}
    assert core._segment==0 and window.segment_id=='lower_floor'
    assert all(body[2]==pytest.approx(ground[2]+.55)
        for ground,body in zip(window.ground_source_xyz,window.body_odom_xyz))
    assert len(window.point_segment_ids)==len(window.body_odom_xyz)


def test_floor_only_and_mode_transaction_windows_keep_original_boundaries():
    from test_continuous_reference import segmented_snapshot
    for core in (reference(source=segmented_snapshot()),reference(crossfloor=True)):
        if core.snapshot.payload()['direction']=='same_floor':core.execution_mode_policy='fixed_stair'
        project(core,observation())
        window=core.window(current_source_ns=core._progress.source_ns,
            now_monotonic=core._progress.received_monotonic,horizon_m=4.)
        assert window.ground_source_xyz[-1]==(1.,0.,0.)
        assert set(window.point_segment_ids)=={core._segments[0]['segment_id']}


def test_exact_portal_first_sample_retains_current_owner_and_height_once():
    core=fixed_reference();sample=observation(x=1.,t=.8);project(core,sample)
    window=core.window(current_source_ns=sample.body.source_ns,
        now_monotonic=sample.received_monotonic,horizon_m=2.)
    assert window.point_segment_ids[0]==window.segment_id=='lower_floor'
    assert window.ground_source_xyz[0]==(1.,0.,0.)
    assert window.body_odom_xyz[0]==(1.,0.,.55)
    assert window.edge_indices[0]==0 and window.edge_indices[1]==1
    assert window.point_segment_ids[1]=='stair_lower'


@pytest.mark.parametrize('past_projection_region',[False,True])
def test_pending_native_result_cannot_lose_a_real_portal_event(past_projection_region):
    from copy import deepcopy
    from d1max_planning_interfaces.msg import ReferenceReceipt
    from test_continuous_reference_node import harness,drive
    from test_continuous_reference_transport import envelope,state,BASE_NS
    from test_segment_transition import install_protocol_support_fixture,commit_reference
    c,clock,out=harness(execution_mode_policy='fixed_stair',geometry_owner='local_executor')
    install_protocol_support_fixture(c)
    c.on_route(envelope(crossfloor=True));commit_reference(c,clock)
    original=deepcopy(c.transport.accepted)
    assert c.transport.accepted.path.poses[-1].pose.position.z>original.path.poses[0].pose.position.z
    assert not c.transport.covers_current_segment_end()  # No artificial finite portal target.
    drive(c,clock,t=.7,x=.85)
    pending=deepcopy(c.proposal)
    assert pending.reference.segment_id=='lower_floor'
    drive(c,clock,t=.8,x=.95)
    assert c.transport.core._portal_observation is not None
    clock.ns=BASE_NS+1_200_000_000;clock.mono=101.2
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    sample=state(t=1.2,x=1.)
    for name in ('local_odometry','global_odometry'):
        getattr(sample,name).pose.pose.position.z=.95 if past_projection_region else .75
    c.on_navigation(sample)
    # A late native rejection closes that exact delivery, not the current
    # accepted curve. No direct manipulation of its pending ownership.
    assert not c.on_receipt(ReferenceReceipt(version=pending.version,
        proposal_id=pending.proposal_id,expected_version=pending.expected_version,
        expected_trajectory_id=pending.expected_trajectory_id,
        source_stamp=pending.source_stamp,valid_until=pending.valid_until,
        accepted=False,reason='candidate_delivery_expired',transport_mode='isolated_mock'))
    assert c.transport.accepted==original
    c.refresh_window()
    assert c.transport.pending.segment_id=='stair_lower'
    assert c.transport.pending.path.header.stamp==sample.source_stamp
    assert c.transport.pending_core._progress.body_source_xyz==(1.,0.,.95 if past_projection_region else .75)
    commit_reference(c,clock,sequence=2)
    assert c.transport.accepted.segment_id=='stair_lower'
    assert c.transport.accepted.task_id==original.task_id
    assert c.transport.accepted.route_hash==original.route_hash
    assert not [value for key,value in out if key in ('mode_request','cancel_reference')]


@pytest.mark.parametrize('retire_before_commit',[False,True])
def test_late_reanchor_commit_keeps_its_own_measured_portal_not_old_anchor_progress(retire_before_commit):
    from test_continuous_reference_node import harness,native_received,local_geometry_commit,local_state_from_pair
    from test_continuous_reference_transport import envelope,state,BASE_NS
    from test_segment_transition import install_protocol_support_fixture,commit_reference
    c,clock,out=harness(execution_mode_policy='fixed_stair',geometry_owner='local_executor')
    install_protocol_support_fixture(c)
    c.on_route(envelope(crossfloor=True));commit_reference(c,clock)
    original=c.transport.accepted
    clock.ns=BASE_NS+700_000_000;clock.mono=100.7
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    assert c.on_navigation(state(t=.7,x=.8,correction=.06))
    proposal=native_received(c)
    assert proposal.version.anchor_id!=original.anchor_id
    c.p['local_state_enabled']=True
    clock.ns=BASE_NS+800_000_000;clock.mono=100.8
    assert c.on_local_navigation(local_state_from_pair(state(t=.8,x=.9)))
    pending_witness=c.transport.pending_core._portal_observation
    assert pending_witness is not None
    assert pending_witness.body_source_xyz[0]==pytest.approx(.96)
    clock.ns=BASE_NS+1_200_000_000;clock.mono=101.2
    sample=state(t=1.2,x=1.)
    for name in ('local_odometry','global_odometry'):getattr(sample,name).pose.pose.position.z=.95
    c.on_local_navigation(local_state_from_pair(sample))
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    if retire_before_commit:
        assert c.retire_proposal('late_native_owner_fact')
        assert c.transport.pending is None
    assert c.on_local_commit(local_geometry_commit(c,clock,proposal,sequence=2,trajectory=2))
    assert c.transport.core.anchor.anchor_id==proposal.version.anchor_id
    assert c.transport.core._portal_observation==pending_witness
    c.refresh_window()
    assert c.transport.pending.segment_id=='stair_lower'
    assert c.transport.pending_core.anchor.anchor_id==proposal.version.anchor_id
    assert c.transport.pending_core._progress.body_source_xyz[0]==pytest.approx(1.06)
    assert c.transport.pending.path.header.stamp==sample.source_stamp
