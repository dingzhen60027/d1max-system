"""System-v2 real reference callbacks with generated contracts, no ROS graph.

Local acquisition evidence advances a fixed route independently of global
matching. A received proposal is not an execution permit or a collision proof.
"""
from copy import deepcopy

import pytest
from d1max_planning_interfaces.msg import LocalNavigationState

from d1max_pct_scan.source_route_ros import to_message
from test_continuous_reference import snapshot
from test_continuous_reference_node import harness, admit, native_received, fresh_commit
from test_continuous_reference_transport import BASE_NS, envelope, state


def local_message(pair):
    result = LocalNavigationState(schema_version=1)
    for key in ('session_id', 'map_version_id', 'localization_epoch',
                'localization_seed_id', 'source_stamp', 'posterior_stamp',
                'imu_stamp', 'extrapolation_sec', 'local_odometry', 'usable', 'reason'):
        setattr(result, key, deepcopy(getattr(pair, key)))
    return result


def local_harness():
    callback, clock, output = harness()
    callback.p['local_state_enabled'] = True
    assert callback.on_local_navigation(local_message(state()))
    return callback, clock, output


def long_route():
    message = envelope()
    route = snapshot(points=[[float(x), 0., 0.] for x in range(6)])
    message.route_hash = route.route_hash
    message.snapshot = to_message(route, session_id='session', task_id='task',
        route_id='route', epoch=1, seed_id='seed', stamp=message.source_stamp)
    return message


def local_update(callback, clock, t, x):
    clock.ns, clock.mono = BASE_NS+round(t*1e9), 100.+t
    assert callback.on_map(dict(callback.context, valid=True, source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i, integrated_stamp_ns=clock.ns) for i in (0, 1)]))
    assert callback.on_local_navigation(local_message(state(t=t, x=x)))


@pytest.mark.parametrize('kind', ['unusable', 'stale', 'frame', 'duplicate'])
def test_bad_global_never_erases_fresh_local_progress_or_incumbent(kind):
    callback, clock, output = local_harness()
    callback.on_route(long_route()); admit(callback)
    old = deepcopy(callback.transport.accepted)
    local_update(callback, clock, .6, .4)
    latest = deepcopy(callback.transport.latest_local)
    measured = callback.transport.core._progress
    body_count = len([value for key, value in output if key == 'body'])
    bad = state(t=.6, x=.4)
    if kind == 'unusable': bad.usable = False
    elif kind == 'stale': bad = state()
    elif kind == 'frame': bad.global_odometry.header.frame_id = 'wrong_map'
    elif kind == 'duplicate':
        # An exact old pair is ignored, not relabelled as a fresh local sample.
        bad = state(t=.59, x=.39)
        assert callback.on_navigation(bad)
        measured = callback.transport.core._progress
        latest = deepcopy(callback.transport.latest_local)
    if kind == 'duplicate':
        assert callback.on_navigation(bad) is False
    else:
        with pytest.raises(ValueError): callback.on_navigation(bad)
    assert callback.transport.latest_local == latest
    assert callback.transport.core._progress == measured
    assert callback.transport.accepted == old
    assert len([value for key, value in output if key == 'body']) == body_count
    assert not callback.transport.quarantined


def test_global_pair_behind_local_cannot_regress_actual_progress():
    callback, clock, _ = local_harness()
    callback.on_route(long_route()); admit(callback)
    local_update(callback, clock, .6, .4)
    progress = callback.transport.core._progress
    assert callback.on_navigation(state(t=.55, x=.36, correction=.12))
    assert callback.transport.latest_pair[0].body.source_ns == BASE_NS+550_000_000
    assert callback.transport.latest_local.body.source_ns == clock.ns
    assert callback.transport.core._progress == progress
    assert callback.proposal.reference.path.header.stamp == local_message(state(t=.6)).source_stamp


@pytest.mark.parametrize('has_incumbent', [False, True])
def test_consumed_pending_rebuilds_same_route_new_generation_and_fences_late_commit(has_incumbent):
    callback, clock, _ = local_harness()
    route = long_route(); callback.on_route(route)
    old = None
    if has_incumbent:
        admit(callback)
        old = deepcopy(callback.transport.accepted)
        # Reissue when the accepted two-metre window has one metre remaining.
        for i in range(1, 6): local_update(callback, clock, i*.2, i*.2)
    proposal = native_received(callback)
    frozen_route = deepcopy(callback.transport.route)
    generation = proposal.version.reference_generation
    first_end = callback.transport._pending_end_arc_m
    # No global matching refresh occurs while actual odom consumes the window.
    t = (clock.ns-BASE_NS)*1e-9
    x = callback.transport.core._progress.measured_arc_m
    while callback.proposal.version.reference_generation == generation:
        x = min(first_end-.19, x+.2); t += .2
        local_update(callback, clock, t, x)
    successor = callback.proposal
    assert successor is not None
    assert successor.version.reference_generation > generation
    assert successor.version.task_id == proposal.version.task_id
    assert successor.version.route_hash == proposal.version.route_hash == route.route_hash
    assert callback.transport.route == frozen_route
    assert callback.transport.accepted == old
    assert successor.reference.path.poses[0].pose.position.x == pytest.approx(x)
    assert callback.transport.pending_window_remaining() > 1.9
    assert not fresh_commit(callback, clock, proposal, sequence=2)
    assert callback.proposal == successor and callback.transport.accepted == old


def test_global_correction_only_prepares_anchor_candidate_incumbent_geometry_unchanged():
    callback, clock, _ = local_harness()
    callback.on_route(long_route()); old_permit = admit(callback)
    old = deepcopy(callback.transport.accepted)
    core = callback.transport.core
    local_update(callback, clock, .6, .3)
    assert callback.on_navigation(state(t=.6, x=.3, correction=.12))
    candidate = callback.proposal
    assert candidate is not None and candidate.version.anchor_id != old.anchor_id
    assert candidate.expected_version == old_permit.version
    assert candidate.expected_trajectory_id == old_permit.trajectory_id
    assert candidate.version.route_hash == old.route_hash
    assert callback.transport.accepted == old and callback.transport.core is core
    assert callback.transport.pending_core is not core
    # Native delivery receipt still cannot replace the old executable geometry.
    native_received(callback)
    assert callback.transport.accepted == old and callback.transport.core is core


def test_expired_local_does_not_fabricate_progress_or_rebuild_pending_from_receipt_time():
    callback, clock, _ = local_harness()
    callback.on_route(long_route()); admit(callback)
    old = deepcopy(callback.transport.accepted)
    progress = callback.transport.core._progress
    clock.ns += 600_000_000; clock.mono += .6
    assert not callback.on_local_navigation(local_message(state()))
    callback.tick()
    assert callback.transport.accepted == old
    assert callback.transport.core._progress == progress
    assert callback.proposal is None


def test_local_hard_reset_quarantines_without_faking_new_anchor_or_resuming_route():
    callback, clock, _ = local_harness()
    callback.on_route(long_route()); old_permit = admit(callback)
    old = deepcopy(callback.transport.accepted)
    clock.ns += 20_000_000; clock.mono += .02
    message = local_message(state(t=.02, usable=False))
    message.reason = 'lio_reset'
    assert not callback.on_local_navigation(message)
    assert callback.transport.quarantined and callback.permit is None
    assert callback.transport.accepted == old  # review only, not a lease
    assert not callback.on_permit(old_permit)
    assert callback.transport.pending is None
