"""Literal pending-segment observations survive incumbent projection loss.

No ROS initialization, graph, SDK, or occupancy bypass. Protocol support and
owner receipts are explicit doubles; these tests prove reference bookkeeping,
not collision acceptance or physical stair traversal.
"""
from copy import deepcopy

import pytest

from d1max_pct_scan.continuous_reference_transport import nanoseconds
from test_continuous_reference_transport import BASE_NS, envelope, state, transport
from test_continuous_reference_node import (
    harness, local_state_from_pair, native_received, local_geometry_commit)
from test_segment_transition import (
    reversed_delivery, install_protocol_support_fixture, commit_reference)


def measured(t, x, z, *, epoch=1):
    sample=state(t=t,x=x,epoch=epoch)
    for field in ('local_odometry','global_odometry'):
        getattr(sample,field).pose.pose.position.z=z
    return sample


def observe(value,sample,lane):
    stamp=nanoseconds(sample.source_stamp)
    now=100.+(stamp-BASE_NS)*1e-9
    callback=value.on_local if lane=='local' else value.on_navigation
    wire=local_state_from_pair(sample) if lane=='local' else sample
    return callback(wire,received_monotonic=now,current_source_ns=stamp,now_monotonic=now)


def staged_stair(downstairs=False):
    value,_=transport(execution_mode_policy='fixed_stair')
    z=1.55 if downstairs else .55
    observe(value,measured(0.,0.,z),'pair')
    route=envelope(crossfloor=True)
    if downstairs:route=reversed_delivery(route)
    value.on_route(route,current_source_ns=BASE_NS,now_monotonic=100.)
    value.commit_pending(current_source_ns=BASE_NS,now_monotonic=100.)
    observe(value,measured(.8,.95,z),'pair')
    value.request_segment_transition(current_source_ns=BASE_NS+800_000_000,now_monotonic=100.8)
    return value,z


@pytest.mark.parametrize('downstairs',[False,True])
@pytest.mark.parametrize('lane',['local','pair'])
def test_pending_portal_keeps_original_samples_when_incumbent_cannot_project(downstairs,lane):
    value,z=staged_stair(downstairs)
    incumbent=deepcopy(value.accepted)
    pending=deepcopy(value.pending)
    candidate=value.pending_core
    sign=-1. if downstairs else 1.
    # 25 cm/s literal movement. The incumbent floor loses height
    # correspondence before the pending flight reaches its exit portal.
    for index in range(1,19):
        sample=measured(.8+.2*index,1.,z+sign*.05*index)
        if index>=7:
            with pytest.raises(ValueError,match='outside_current_semantic_segment'):
                observe(value,sample,lane)
        else:
            try:observe(value,sample,lane)
            except ValueError as error:
                assert str(error)=='measured_body_outside_current_semantic_segment'
        assert value.accepted==incumbent and value.pending==pending
    assert candidate._portal_observation is not None
    witness=candidate._portal_observation
    assert witness.segment_id==pending.segment_id
    assert witness.source_ns>nanoseconds(pending.path.header.stamp)
    assert witness.received_monotonic==pytest.approx(100.+(witness.source_ns-BASE_NS)*1e-9)
    assert candidate._progress is None  # Current pose has also left this flight.
    stamp=nanoseconds(sample.source_stamp);now=100.+(stamp-BASE_NS)*1e-9
    value.commit_pending(current_source_ns=stamp,now_monotonic=now,owner_committed=True)
    assert value.core._progress is None  # Commit cannot fabricate current progress.
    assert value.core._portal_observation==witness
    next_id=value.request_segment_transition(current_source_ns=stamp,now_monotonic=now)
    assert next_id==('stair_lower' if downstairs else 'stair_upper')
    assert value.accepted==pending  # Preparing the next segment is not committing it.
    assert value.pending_core._progress.body_source_xyz==(1.,0.,z+sign*.9)
    assert value.pending.path.header.stamp==sample.source_stamp
    assert value.pending_core._progress.received_monotonic==pytest.approx(now)
    assert value.pending.route_hash==incumbent.route_hash


@pytest.mark.parametrize('lane',['local','pair'])
@pytest.mark.parametrize('fault',['stale','receipt','epoch'])
def test_invalid_measurement_cannot_update_the_pending_segment(lane,fault):
    value,_=staged_stair()
    candidate=value.pending_core
    before=(candidate._last_sample,candidate._progress,candidate._portal_observation)
    sample=measured(1.,1.,1.05,epoch=2 if fault=='epoch' else 1)
    callback=value.on_local if lane=='local' else value.on_navigation
    wire=local_state_from_pair(sample) if lane=='local' else sample
    with pytest.raises(ValueError):
        callback(wire,received_monotonic=100. if fault=='receipt' else 101.,current_source_ns=BASE_NS+1_000_000_000+
            (600_000_000 if fault=='stale' else 0),now_monotonic=101.6 if fault=='stale' else 101.)
    assert (candidate._last_sample,candidate._progress,candidate._portal_observation)==before
    assert not candidate._portal_observation


def test_pending_retreat_clears_the_real_portal_event_while_old_floor_is_unavailable():
    value,z=staged_stair()
    for index in range(1,10):
        sample=measured(.8+.2*index,1.,z+.05*index)
        try:observe(value,sample,'local')
        except ValueError:pass  # The pending segment, not the old floor, owns this geometry.
    candidate=value.pending_core
    assert candidate._portal_observation is not None
    retreat=measured(3.2,1.,.86)
    with pytest.raises(ValueError,match='outside_current_semantic_segment'):
        observe(value,retreat,'local')
    assert candidate._progress.body_source_xyz==(1.,0.,.86)
    assert candidate._portal_observation is None


def test_delayed_global_pair_cannot_erase_newer_pending_local_progress():
    value,_=staged_stair()
    with pytest.raises(ValueError,match='outside_current_semantic_segment'):
        observe(value,measured(1.02,1.,.9),'local')
    candidate=value.pending_core
    before=(candidate._last_sample,candidate._progress,candidate._portal_observation)
    pair=measured(1.,1.,.85)
    try:
        value.on_navigation(pair,received_monotonic=101.02,
            current_source_ns=BASE_NS+1_020_000_000,now_monotonic=101.02)
    except ValueError as error:
        assert str(error)=='measured_body_outside_current_semantic_segment'
    assert (candidate._last_sample,candidate._progress,candidate._portal_observation)==before


def test_pending_projection_programming_error_is_not_swallowed(monkeypatch):
    value,_=staged_stair()
    def broken(*args,**kwargs):raise RuntimeError('unexpected_projection_failure')
    monkeypatch.setattr(value.pending_core,'project',broken)
    with pytest.raises(RuntimeError,match='unexpected_projection_failure'):
        observe(value,measured(1.,1.,.6),'local')


@pytest.mark.parametrize('downstairs',[False,True])
def test_callback_late_owner_commit_can_continue_after_pending_exit_crossing(downstairs):
    c,clock,out=harness(execution_mode_policy='fixed_stair',geometry_owner='local_executor',
        local_state_enabled=True)
    install_protocol_support_fixture(c)
    z=1.55 if downstairs else .55
    clock.ns=BASE_NS+20_000_000;clock.mono=100.02
    initial=measured(.02,0.,z)
    assert c.on_navigation(initial)
    assert c.on_local_navigation(local_state_from_pair(initial))
    route=envelope(t=.02,crossfloor=True)
    if downstairs:route=reversed_delivery(route)
    c.on_route(route);commit_reference(c,clock)
    clock.ns=BASE_NS+800_000_000;clock.mono=100.8
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    assert c.on_local_navigation(local_state_from_pair(measured(.8,.95,z)))
    proposal=native_received(c)
    assert proposal.reference.segment_id==('stair_upper' if downstairs else 'stair_lower')
    incumbent=deepcopy(c.transport.accepted)
    sign=-1. if downstairs else 1.
    for index in range(1,19):
        sample=measured(.8+.2*index,1.,z+sign*.05*index)
        clock.ns=nanoseconds(sample.source_stamp);clock.mono=100.+(clock.ns-BASE_NS)*1e-9
        c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
            sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
        accepted=c.on_local_navigation(local_state_from_pair(sample))
        if index>=7:assert accepted is False
        assert c.transport.accepted==incumbent
    assert c.on_local_commit(local_geometry_commit(c,clock,proposal,sequence=2,trajectory=2))
    assert c.transport.core._progress is None
    c.refresh_window()
    assert c.transport.pending is not None
    assert c.transport.pending.segment_id==('stair_lower' if downstairs else 'stair_upper')
    assert c.transport.pending.path.header.stamp==sample.source_stamp
    assert c.transport.pending.route_hash==incumbent.route_hash
    assert not [key for key,_ in out if key in ('mode_request','cancel_reference','sdk','motion','permit')]
