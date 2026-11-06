"""Stable rigid corrections must recover using new measured pairs, not new leases.

Passive production-reference callbacks only; no ROS graph, planner replacement,
SDK connection, or fabricated collision result is needed to reproduce the gap.
"""
from copy import deepcopy

import pytest

from test_continuous_reference import observation, project, reference
from test_continuous_reference_node import (
    local_geometry_commit, local_state_from_pair, native_received,
)
from test_continuous_reference_transport import BASE_NS, state
from test_rolling_reference_policy import committed_harness


def observe(core, t, *, correction=.2, revision=2):
    local = observation(t=t)
    return core.observe_pair(observation(z=.55+correction, frame='d1max_loc_map', t=t),
        local, revision=revision, current_source_ns=local.body.source_ns,
        now_monotonic=local.received_monotonic)


def feed(c, clock, t, z=.55, *, global_z=.75):
    clock.ns, clock.mono = BASE_NS+round(t*1e9), 100+t
    c.on_map(dict(c.context, valid=True, source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i, integrated_stamp_ns=clock.ns) for i in (0, 1)]))
    pair = state(t=t, x=0.)
    pair.local_odometry.pose.pose.position.z = z
    if global_z is None:
        return c.on_local_navigation(local_state_from_pair(pair))
    pair.global_odometry.pose.pose.position.z = global_z
    return c.on_navigation(pair)


def test_new_measured_pair_refreshes_only_uncommitted_same_geometry_candidate():
    core = reference()
    project(core, observation())
    accepted, progress, route = core.anchor, core._progress, core.snapshot
    old = observe(core, .1)
    fresh = observe(core, .9, revision=3)
    assert fresh.source_ns == observation(t=.9).body.source_ns
    assert fresh.revision == 3 and fresh.anchor_id != old.anchor_id
    assert fresh.map_from_odom == old.map_from_odom
    assert core.anchor == accepted and core._progress == progress and core.snapshot is route
    staged = core.prepare_reanchor(observation(t=.9),
        current_source_ns=fresh.source_ns, now_monotonic=100.9)
    assert staged.anchor == fresh
    assert staged.snapshot.route_hash == route.route_hash and staged.task_id == core.task_id


@pytest.mark.parametrize('t', [.09, .1])
def test_old_or_duplicate_pair_never_renews_candidate(t):
    core = reference()
    project(core, observation())
    original = observe(core, .1)
    with pytest.raises(ValueError, match='correction_context_or_version_changed'):
        observe(core, t, revision=3)
    assert core.candidate_anchor is original


def test_stale_received_pair_never_renews_candidate():
    core = reference()
    project(core, observation())
    original = observe(core, .1)
    with pytest.raises(ValueError, match='must_both_be_fresh'):
        core.observe_pair(observation(z=.75, frame='d1max_loc_map', t=.9),
            observation(t=.9), revision=3,
            current_source_ns=observation(t=1.5).body.source_ns, now_monotonic=101.5)
    assert core.candidate_anchor is original


def test_accepted_identical_rigid_still_does_not_create_a_correction_each_tick():
    core = reference()
    project(core, observation())
    original = core.anchor
    for t in (.1, .9, 1.8):
        assert observe(core, t, correction=0.) is original
        assert core.candidate_anchor is None


def test_failed_pending_projection_recovers_after_stable_correction_without_new_task():
    c, clock, output, route = committed_harness()
    accepted = deepcopy(c.transport.accepted)
    feed(c, clock, .6)
    failed = native_received(c)
    feed(c, clock, .8, .70, global_z=None)
    assert c.proposal is None and c.transport.pending is None
    assert c.transport.accepted == accepted
    assert c.error == 'measured_body_outside_current_semantic_segment'
    # Same transform, but genuinely new measured global/local pair. Waiting
    # > original 400 ms lease must not make this correction unrecoverable.
    feed(c, clock, 1.4)
    assert c.proposal is not None, c.error
    recovered = native_received(c)
    assert recovered.version.reference_generation > failed.version.reference_generation
    assert recovered.version.anchor_id != failed.version.anchor_id
    assert recovered.source_stamp == state(t=1.4).source_stamp
    assert recovered.version.task_id == route.task_id
    assert recovered.version.route_id == route.route_id
    assert recovered.version.route_hash == route.route_hash
    assert c.transport.accepted == accepted
    assert c.on_local_commit(local_geometry_commit(c, clock, recovered, sequence=2, trajectory=2))
    assert c.transport.accepted == recovered.reference
    assert c.transport.core.anchor.source_ns == clock.ns
    assert not [k for k, _ in output if k in ('sdk', 'motion', 'cancel_reference', 'handoff')]


def test_new_pair_does_not_rewrite_published_pending_identity_or_deadline():
    c, clock, _, _ = committed_harness()
    feed(c, clock, .6)
    pending = deepcopy(native_received(c))
    core = c.transport.pending_core
    old_anchor = core.anchor
    owner = deepcopy(c.transport.accepted)
    for t in (.8, 1., 1.4):
        feed(c, clock, t)
        assert c.proposal == pending
        assert c.transport.pending == pending.reference
        assert c.transport.pending_core is core and core.anchor == old_anchor
        assert c.transport.accepted == owner
    assert c.transport.core.candidate_anchor.source_ns == clock.ns
    # Native may still commit the literal previously prepared geometry. Fresh
    # candidate observations do not substitute a different anchor under it.
    assert c.on_local_commit(local_geometry_commit(c, clock, pending, sequence=2, trajectory=2))
    assert c.transport.core.anchor == old_anchor
    assert c.transport.accepted == pending.reference


def test_cancelled_route_is_not_revived_by_new_pair_for_stable_correction():
    from d1max_pct_scan.continuous_reference_transport import set_stamp
    c, clock, output, route = committed_harness()
    feed(c, clock, .6)
    old = deepcopy(native_received(c))
    late_commit = local_geometry_commit(c, clock, old, sequence=2, trajectory=2,
                                       publish_proof=False)
    cancel = deepcopy(route)
    cancel.active = False
    cancel.delivery_sequence += 1
    set_stamp(cancel.source_stamp, clock.ns)
    c.on_route(cancel)
    count = len([v for k, v in output if k == 'proposal'])
    for t in (.8, 1.4, 2.):
        feed(c, clock, t)
        c.tick()
        assert c.proposal is None and c.transport.pending is None
        assert c.transport.route is None
    assert len([v for k, v in output if k == 'proposal']) == count
    assert not c.on_local_commit(late_commit)
    assert c.transport.accepted is None


@pytest.mark.parametrize('first_error', [False, True])
def test_late_support_keeps_admitted_anchor_and_failed_build_retries_literal_request(first_error):
    from threading import Event
    from d1max_pct_scan.bounded_preparation import LatestPreparation

    c, clock, output, route = committed_harness()
    started, release = Event(), Event()
    calls = []

    def prepare(request):
        calls.append(deepcopy(request[0]))
        if len(calls) == 1:
            started.set()
            assert release.wait(2.)
            if first_error:
                raise ValueError('isolated_first_support_failure')
        return c.build_support(request)

    c.preparation = LatestPreparation(prepare)
    try:
        feed(c, clock, .6)
        assert started.wait(1.)
        original = deepcopy(c.proposal)
        assert native_received(c)==original
        ticket = c.preparation._ticket
        feed(c, clock, 1.4)
        assert c.proposal == original and c.preparation._ticket == ticket
        release.set()
        with c.preparation._cv:
            assert c.preparation._cv.wait_for(lambda: c.preparation._result is not None, timeout=1.)
        c.tick()  # Static evidence has no new sensor/admission deadline.
        assert c.proposal==original and c.receipt.accepted
        if first_error:
            with c.preparation._cv:
                assert c.preparation._cv.wait_for(lambda:c.preparation._result is not None,timeout=1.)
            assert c.harvest_preparation()
        assert [v for k,v in output if k=='proposal'][-1]==original
        assert calls==([original,original] if first_error else [original])
        assert original.source_stamp==state(t=.6).source_stamp
        assert original.version.task_id==route.task_id
        assert c.on_local_commit(local_geometry_commit(c,clock,original,sequence=2,trajectory=2))
    finally:
        release.set()
        assert c.close()
