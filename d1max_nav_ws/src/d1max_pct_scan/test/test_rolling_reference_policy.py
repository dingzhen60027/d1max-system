"""Passive production-callback tests: no ROS graph or replacement planner."""
from copy import deepcopy
import math

import pytest

from d1max_pct_scan.continuous_reference_transport import set_stamp
from test_continuous_reference_node import (
    harness, drive, long_envelope, native_received, local_geometry_commit,
    local_state_from_pair)
from test_continuous_reference_transport import state, BASE_NS
from d1max_planning_interfaces.msg import ReferenceReceipt


def committed_harness(length=12):
    c, clock, out = harness(geometry_owner='local_executor', reference_horizon_m=4.)
    route = long_envelope(length)
    c.on_route(route)
    proposal = native_received(c)
    assert c.on_local_commit(local_geometry_commit(c, clock, proposal))
    return c, clock, out, route


def step(c, clock, index, x):
    drive(c, clock, t=.05*index, x=x)


@pytest.mark.parametrize('span,step_m',[(4.,.25),(2.,.25),(.5,.125)])
def test_policy_uses_distance_and_time_with_short_window_overlap(span,step_m):
    from d1max_pct_scan.rolling_reference_policy import RollingReferencePolicy, WindowProgress
    p=RollingReferencePolicy()
    assert p.reason(WindowProgress(2.,2.+span,2.,2.+step_m-.001),elapsed_s=1.) is None
    assert p.reason(WindowProgress(2.,2.+span,2.,2.+step_m),elapsed_s=.49) is None
    assert p.reason(WindowProgress(2.,2.+span,2.,2.+step_m),elapsed_s=.5)=='measured_progress'
    assert (span-step_m)/span>=.75


def test_policy_static_reverse_low_lookahead_and_segment_boundary():
    from d1max_pct_scan.rolling_reference_policy import RollingReferencePolicy, WindowProgress
    p=RollingReferencePolicy()
    assert p.reason(WindowProgress(.8,4.8,1.,1.),elapsed_s=10.) is None
    assert p.reason(WindowProgress(.8,4.8,1.,.9),elapsed_s=10.) is None
    assert p.reason(WindowProgress(0.,4.,0.,2.1),elapsed_s=1.)=='recover_lookahead'
    assert p.reason(WindowProgress(0.,4.,0.,2.1),elapsed_s=1.,covers_segment_end=True) is None
    assert p.reason(WindowProgress(0.,4.,0.,0.),elapsed_s=1.,covers_segment_end=True,
        correction_due=True)=='coordinate_anchor_update'
    assert p.reason(None,elapsed_s=math.inf)=='initial_window'
    assert p.reason(None,elapsed_s=math.nan) is None


def test_formal_callback_rolls_at_25cm_not_half_window_without_new_task():
    """Also runnable unchanged with the frozen old callbacks as a red control."""
    c,clock,out,route=committed_harness()
    original=deepcopy(c.transport.accepted)
    for i in range(1,18):
        step(c,clock,i,.015*i)
    # At 0.3 m/s, 0.85 s advances 25.5 cm, not the old 2 m threshold.
    assert c.transport.pending is not None, 'rolling window must already be staged at 25 cm'
    assert c.transport.accepted==original
    proposal=c.proposal
    assert proposal.reference.path.poses[0].pose.position.x==pytest.approx(.255)
    assert proposal.reference.path.poses[-1].pose.position.x==pytest.approx(4.255)
    assert proposal.version.task_id==route.task_id
    assert proposal.version.route_id==route.route_id
    assert proposal.version.route_hash==route.route_hash
    assert proposal.source_stamp==state(t=.85,x=.255).source_stamp
    assert not [v for k,v in out if k in ('cancel_reference','handoff','sdk','motion')]


def test_formal_callbacks_commit_many_overlapping_windows_from_original_arcs():
    c,clock,out,route=committed_harness()
    starts=[0.];ends=[4.];issued=[0.];times=[0.];overlaps=[]
    sequence=1
    for i in range(1,205):
        step(c,clock,i,.015*i)
        if c.transport.pending is None:
            continue
        previous=deepcopy(c.transport.accepted)
        proposal=native_received(c)
        start=proposal.reference.path.poses[0].pose.position.x
        end=proposal.reference.path.poses[-1].pose.position.x
        assert c.transport.accepted==previous
        assert start-starts[-1]==pytest.approx(.255,abs=.016)
        overlap=(ends[-1]-start)/(ends[-1]-starts[-1])
        assert overlap>=.75
        assert .05*i-times[-1]>=.5-1e-9
        assert end-start==pytest.approx(4.)
        assert proposal.version.task_id==route.task_id and proposal.version.route_id==route.route_id
        assert proposal.version.route_hash==route.route_hash
        sequence+=1
        assert c.on_local_commit(local_geometry_commit(c,clock,proposal,sequence,sequence))
        facts=c.transport.rolling_window_progress()
        assert facts.start_arc_m==pytest.approx(start)
        assert facts.end_arc_m==pytest.approx(end)
        starts.append(start);ends.append(end);times.append(.05*i);overlaps.append(overlap)
        issued.append(facts.issued_confirmed_arc_m)
    assert sequence>=10
    assert starts==sorted(starts) and issued==sorted(issued)
    assert c.transport.route==route
    assert not [v for k,v in out if k in ('cancel_reference','handoff','sdk','motion')]
    print(f'rolling_reference windows_committed={sequence} measured_motion_m={.015*204:.3f} '
        f'min_overlap={min(overlaps):.6f} min_interval_s={min(b-a for a,b in zip(times,times[1:])):.3f} '
        f'last_window=[{starts[-1]:.3f},{ends[-1]:.3f}] task_and_route_unchanged=1')


def test_delayed_pending_stays_literal_then_commit_catches_up_without_waiting_half_window():
    c,clock,out,_=committed_harness()
    for i in range(1,18):step(c,clock,i,.015*i)
    pending=native_received(c)
    literal=deepcopy(c.transport.pending)
    before=len([v for k,v in out if k=='proposal'])
    for i in range(18,59):
        step(c,clock,i,.015*i)
        assert c.transport.pending==literal
        assert c.transport.generation==pending.version.reference_generation
    assert len([v for k,v in out if k=='proposal'])==before
    assert c.on_local_commit(local_geometry_commit(c,clock,pending,2,2))
    assert c.transport.accepted.path.header.stamp==pending.source_stamp
    assert c.transport._accepted_start_arc_m==pytest.approx(.255)
    step(c,clock,59,.885)
    assert c.transport.pending is not None
    assert c.proposal.version.reference_generation==pending.version.reference_generation+1
    assert c.proposal.reference.path.poses[0].pose.position.x==pytest.approx(.885)
    assert c.proposal.expected_trajectory_id==2
    assert c.transport.remaining_window_arc_m()>3.


def test_static_and_small_reverse_do_not_churn_reference_generations():
    c,clock,out,_=committed_harness()
    for i in range(1,18):step(c,clock,i,.015*i)
    proposal=native_received(c)
    assert c.on_local_commit(local_geometry_commit(c,clock,proposal,2,2))
    generation=c.transport.generation
    for i in range(18,100):
        x=.255-.06*min(1.,(i-17)/5.) if i<50 else .255
        step(c,clock,i,x)
        assert c.transport.pending is None
        assert c.transport.generation==generation
    assert len([v for k,v in out if k=='proposal'])==2


def test_negative_native_busy_receipt_releases_pending_and_retries_same_route():
    c,clock,out,route=committed_harness()
    for i in range(1,18):step(c,clock,i,.015*i)
    p=deepcopy(c.proposal)
    negative=ReferenceReceipt(version=p.version,proposal_id=p.proposal_id,
        expected_version=p.expected_version,expected_trajectory_id=p.expected_trajectory_id,
        source_stamp=p.source_stamp,valid_until=p.valid_until,accepted=False,
        reason='same_task_splice_pending',transport_mode='isolated_mock')
    original=deepcopy(c.transport.accepted)
    assert not c.on_receipt(negative)
    assert c.transport.pending is None and c.transport.accepted==original
    for i in range(18,27):
        step(c,clock,i,.015*i)
        assert c.transport.pending is None
    step(c,clock,27,.405)
    assert c.transport.pending is not None
    assert c.proposal.version.reference_generation==p.version.reference_generation+1
    assert c.proposal.source_stamp==state(t=1.35,x=.405).source_stamp
    assert c.proposal.version.route_hash==route.route_hash
    assert c.transport.accepted==original
    assert not [v for k,v in out if k=='cancel_reference']
    print(f'rolling_native_defer rejected_generation={p.version.reference_generation} '
        f'retry_generation={c.proposal.version.reference_generation} '
        'retry_interval_s=0.500 original_incumbent_preserved=1 task_and_route_unchanged=1')


def test_consumed_pending_releases_once_instead_of_dead_wait():
    c,clock,out,_=committed_harness()
    for i in range(1,18):step(c,clock,i,.015*i)
    p=native_received(c)
    end=c.transport._pending_end_arc_m
    for i in range(18,272):
        step(c,clock,i,.015*i)
        if .015*i < end-.2:
            assert c.transport.pending.generation==p.version.reference_generation
    assert c.transport.pending.generation==p.version.reference_generation+1
    assert c.proposal.reference.path.poses[0].pose.position.x>4.
    assert len([v for k,v in out if k=='proposal'])==3


def test_segment_end_and_cancel_do_not_become_window_updates():
    c,clock,out,route=committed_harness(length=4)
    assert c.transport.covers_current_segment_end()
    for i in range(1,121):step(c,clock,i,.015*i)
    assert c.transport.generation==1 and c.transport.pending is None
    cancel=deepcopy(route);cancel.active=False;cancel.delivery_sequence+=1
    set_stamp(cancel.source_stamp,clock.ns)
    c.on_route(cancel)
    generation=c.transport.generation
    assert c.transport.core is None and c.transport.accepted is None
    for i in range(121,131):step(c,clock,i,1.8)
    assert c.transport.generation==generation and c.transport.pending is None


@pytest.mark.parametrize('boundary',['foreign_task','reset','cancel'])
def test_late_window_commit_cannot_carry_a_correction_across_task_boundaries(boundary):
    c,clock,_,route=committed_harness()
    for i in range(1,18):step(c,clock,i,.015*i)
    proposal=native_received(c)
    event=local_geometry_commit(c,clock,proposal,2,2)
    # A genuinely new same-time paired observation can stage an anchor while
    # an earlier local window is pending. Neither is permission to cross a
    # later cancellation, reset or a different task's identity.
    clock.ns=BASE_NS+900_000_000;clock.mono=100.9
    assert c.on_navigation(state(t=.9,x=.27,correction=.12))
    correction=c.transport.core.candidate_anchor
    assert correction is not None
    if boundary=='foreign_task':
        event.version.task_id='foreign'
        accepted=deepcopy(c.transport.accepted)
    elif boundary=='reset':
        c.p['local_state_enabled']=True
        clock.ns+=50_000_000;clock.mono+=.05
        assert not c.on_local_navigation(local_state_from_pair(state(t=.95,x=.27,epoch=2)))
    else:
        cancel=deepcopy(route);cancel.active=False;cancel.delivery_sequence+=1
        set_stamp(cancel.source_stamp,clock.ns)
        c.on_route(cancel)
    assert not c.on_local_commit(event)
    if boundary=='foreign_task':
        assert c.transport.accepted==accepted
        assert c.transport.core.candidate_anchor==correction
        assert c.transport.core.anchor.anchor_id!=correction.anchor_id
    elif boundary=='reset':
        assert c.transport.quarantined and c.transport.pending is None
    else:
        assert c.transport.core is None and c.transport.accepted is None
