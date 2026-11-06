"""Reanchoring after an unavailable old projection must follow real odom motion.

No ROS graph, synthetic navigation success, or collision replacement is used:
these tests exercise the production immutable reference/anchor implementation.
"""
import pytest

from test_continuous_reference import observation, project, reference, snapshot


def displaced_reference(*, x=.9, t=2., correction_x=0., z=.95):
    core = reference(source=snapshot(points=[[float(i), 0., 0.] for i in range(8)]))
    project(core, observation())
    local = observation(x=x, z=z, t=t)
    global_ = observation(x=x+correction_x, frame='d1max_loc_map', t=t)
    core.observe_pair(global_, local, revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    with pytest.raises(ValueError, match='outside_current_semantic_segment|measured_body_discontinuity'):
        project(core, local)
    return core, local


@pytest.mark.parametrize('missing_progress', [False, True])
def test_reanchor_recovers_measured_forward_motion_after_old_height_projection_failed(missing_progress):
    core, local = displaced_reference()
    old_anchor, old_source = core.anchor, core._last_sample[0]
    if missing_progress:
        core._progress = None  # Native owner commit cannot undo admitted geometry.
    candidate = core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                     now_monotonic=local.received_monotonic)
    assert candidate._progress.measured_arc_m == pytest.approx(.9)
    assert candidate._progress.body_source_xyz == pytest.approx((.9, 0., .55))
    assert candidate._progress.source_ns == local.body.source_ns
    assert candidate.snapshot.route_hash == core.snapshot.route_hash
    assert candidate.window(current_source_ns=local.body.source_ns,
                            now_monotonic=local.received_monotonic).end_arc_m == pytest.approx(4.9)
    assert core.anchor == old_anchor and core._last_sample[0] == old_source
    assert core._measured == 0.  # Only the new candidate owns recovered progress.


@pytest.mark.parametrize('case', ['teleport', 'map_jump', 'other_floor', 'stale'])
def test_reanchor_recovery_does_not_turn_elapsed_time_into_unobserved_route_progress(case):
    core, local = displaced_reference(t=.1 if case == 'teleport' else 2.,
                                     correction_x=1. if case == 'map_jump' else 0.)
    if case == 'other_floor':
        core.observe_pair(observation(x=.9, z=3.55, frame='d1max_loc_map', t=2.1),
                          observation(x=.9, z=.95, t=2.1), revision=3,
                          current_source_ns=local.body.source_ns+100_000_000,
                          now_monotonic=local.received_monotonic+.1)
        local = observation(x=.9, z=.95, t=2.1)
    delay = .6 if case == 'stale' else 0.
    with pytest.raises(ValueError):
        core.prepare_reanchor(local, current_source_ns=local.body.source_ns+int(delay*1e9),
                              now_monotonic=local.received_monotonic+delay)
    assert core._measured == 0. and core.anchor.revision == 1


@pytest.mark.parametrize('t', [.1, 10.])
def test_pure_map_correction_does_not_borrow_elapsed_time_as_forward_motion(t):
    core = reference()
    project(core, observation())
    local = observation(t=t)
    core.observe_pair(observation(x=.9, frame='d1max_loc_map', t=t), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                              now_monotonic=local.received_monotonic)
    assert core._measured == 0.


@pytest.mark.parametrize('source_delta', [-.01, 0.])
def test_unchanged_or_old_source_cannot_create_new_measured_displacement(source_delta):
    core = reference()
    project(core, observation())
    project(core, observation(x=.1, t=.1))
    local = observation(x=.12, t=.1+source_delta)
    core.observe_pair(observation(x=.2, frame='d1max_loc_map', t=.1+source_delta), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    with pytest.raises(ValueError):
        core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                              now_monotonic=local.received_monotonic)
    assert core._progress.measured_arc_m == pytest.approx(.1)


def test_same_literal_source_can_still_stage_an_ordinary_flat_anchor_correction():
    core = reference()
    project(core, observation())
    local = observation(x=.1, t=.1)
    project(core, local)
    core.observe_pair(observation(x=.2, frame='d1max_loc_map', t=.1), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    candidate = core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                     now_monotonic=local.received_monotonic)
    assert candidate._progress.measured_arc_m == pytest.approx(.2)
    assert candidate._progress.source_ns == core._progress.source_ns
    assert core._progress.measured_arc_m == pytest.approx(.1)


def test_normal_flat_corner_keeps_exact_current_projection_and_fixed_route():
    core = reference(source=snapshot(points=[[0., 0., 0.], [1., 0., 0.],
                                             [1., 1., 0.], [2., 1., 0.]]))
    project(core, observation())
    project(core, observation(x=.9, t=1.))
    local = observation(x=1., y=.1, t=1.2)
    core.observe_pair(observation(x=1.02, y=.11, frame='d1max_loc_map', t=1.2),
                      local, revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    candidate = core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                     now_monotonic=local.received_monotonic)
    assert candidate._progress.measured_arc_m == pytest.approx(1.11)
    assert candidate._progress.cross_track_m == pytest.approx(.02)
    assert core.snapshot.route_hash == candidate.snapshot.route_hash
    assert core._progress.measured_arc_m == pytest.approx(.9)


def test_reanchor_recovers_actual_motion_past_a_long_ninety_degree_corner():
    core = reference(source=snapshot(points=[[0., 0., 0.], [1., 0., 0.],
                                             [1., 1., 0.], [1., 2., 0.], [2., 2., 0.]]))
    project(core, observation())
    project(core, observation(x=.9, t=1.))
    local = observation(x=1., y=.8, z=.95, t=2.)
    core.observe_pair(observation(x=1., y=.8, frame='d1max_loc_map', t=2.), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        project(core, local)
    candidate = core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                     now_monotonic=local.received_monotonic)
    assert candidate._progress.measured_arc_m == pytest.approx(1.8)
    assert core._progress.measured_arc_m == pytest.approx(.9)
    assert candidate.snapshot.route_hash == core.snapshot.route_hash


@pytest.mark.parametrize('duplicate', [False, True])
def test_duplicate_route_vertices_do_not_block_measured_reanchor_progress(duplicate):
    points = [[0., 0., 0.], [1., 0., 0.]]
    if duplicate:
        points.append([1., 0., 0.])
    points.extend([[2., 0., 0.], [3., 0., 0.]])
    core = reference(source=snapshot(points=points))
    project(core, observation())
    project(core, observation(x=.9, t=1.))
    local = observation(x=1.8, z=.95, t=2.)
    core.observe_pair(observation(x=1.8, frame='d1max_loc_map', t=2.), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    candidate = core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                     now_monotonic=local.received_monotonic)
    assert candidate._progress.measured_arc_m == pytest.approx(1.8)
    assert core._progress.measured_arc_m == pytest.approx(.9)


@pytest.mark.parametrize('direction', [-1., 1.])
@pytest.mark.parametrize('wrong_floor', [False, True])
def test_reanchor_preserves_actual_ascending_descending_support_and_rejects_other_floor(direction, wrong_floor):
    import math
    from d1max_pct_scan.source_route import RouteSnapshot

    payload = snapshot(points=[[.5*i, 0., direction*.25*i] for i in range(9)]).payload()
    payload['segments'][0].update(kind='stair_up' if direction > 0. else 'stair_down',
                                  required_mode='stair', floor_id='stair_lower')
    payload['point_floor_ids'] = ['stair_lower']*len(payload['xyz'])
    core = reference(source=RouteSnapshot.create(payload))
    project(core, observation())
    local = observation(x=.9, z=.55+direction*.45+.4, t=2.)
    core.observe_pair(observation(x=.9, z=.55+direction*.45+(3. if wrong_floor else 0.),
                                    frame='d1max_loc_map', t=2.), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    if wrong_floor:
        with pytest.raises(ValueError):
            core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                  now_monotonic=local.received_monotonic)
    else:
        candidate = core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                         now_monotonic=local.received_monotonic)
        assert candidate._progress.measured_arc_m == pytest.approx(.9*math.sqrt(1.25))
        assert candidate._progress.body_source_xyz == pytest.approx((.9, 0., .55+direction*.45))
        assert candidate._segment == core._segment == 0
    assert core._progress.measured_arc_m == 0.


@pytest.mark.parametrize('motion', ['lateral', 'vertical'])
def test_nonforward_motion_cannot_fund_a_large_forward_map_correction(motion):
    core = reference()
    project(core, observation())
    local = observation(y=.9 if motion == 'lateral' else 0.,
                        z=.95 if motion == 'vertical' else .55, t=2.)
    core.observe_pair(observation(x=1., frame='d1max_loc_map', t=2.), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                              now_monotonic=local.received_monotonic)
    assert core._progress.measured_arc_m == 0.


@pytest.mark.parametrize('vertical_delta', [0., .2])
def test_no_planar_motion_does_not_expand_search_even_with_old_slope_projection_offset(vertical_delta, monkeypatch):
    core = reference(source=snapshot(points=[[.5*i, 0., .25*i] for i in range(9)]))
    project(core, observation(x=.3))  # Old slope projection has an XY residual.
    local = observation(x=.3, z=.55+vertical_delta, t=.1)
    core.observe_pair(observation(x=.3, z=.7, frame='d1max_loc_map', t=.1), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    staged_bounds = []
    original_project = type(core).project

    def capture_bound(self, *args, **kwargs):
        staged_bounds.append(self.forward)
        return original_project(self, *args, **kwargs)

    monkeypatch.setattr(type(core), 'project', capture_bound)
    core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                          now_monotonic=local.received_monotonic)
    assert staged_bounds == [min(core.forward, core.backward)]


@pytest.mark.parametrize('dx', [0., -1e-6, 1e-6, -.01, .01])
def test_near_zero_planar_motion_cannot_convert_old_slope_xy_residual_into_forward_travel(dx):
    core = reference(source=snapshot(points=[[.5*i, 0., .5*i] for i in range(8)]))
    project(core, observation(x=.59))
    before = core._progress
    local = observation(x=.59+dx, t=.1)
    core.observe_pair(observation(x=.89, z=1.44, frame='d1max_loc_map', t=.1), local,
                      revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                              now_monotonic=local.received_monotonic)
    assert core._progress == before


@pytest.mark.parametrize('direction', [-1., 1.])
def test_real_slope_motion_still_recovers_despite_an_old_xyz_projection_residual(direction):
    import math

    core = reference(source=snapshot(points=[[.5*i, 0., direction*.5*i] for i in range(8)]))
    project(core, observation(x=.35))
    before = core._progress
    local = observation(x=.95, z=.55+direction*.6, t=2.)
    core.observe_pair(observation(x=.95, z=.55+direction*.95, frame='d1max_loc_map', t=2.),
                      local, revision=2, current_source_ns=local.body.source_ns,
                      now_monotonic=local.received_monotonic)
    candidate = core.prepare_reanchor(local, current_source_ns=local.body.source_ns,
                                     now_monotonic=local.received_monotonic)
    assert candidate._progress.measured_arc_m == pytest.approx(.95*math.sqrt(2.))
    assert core._progress == before


def test_current_node_recovers_window_delivery_without_task_or_sdk_restart():
    from copy import deepcopy
    from test_continuous_reference_node import (
        harness, local_geometry_commit, local_state_from_pair, long_envelope, native_received)
    from test_continuous_reference_transport import BASE_NS, state

    node, clock, output = harness(geometry_owner='local_executor', local_state_enabled=True,
                                  reference_horizon_m=4.)
    node.on_route(long_envelope())
    first = native_received(node)
    assert node.on_local_commit(local_geometry_commit(node, clock, first))
    original = deepcopy(node.transport.accepted)
    clock.ns, clock.mono = BASE_NS+2_000_000_000, 102.
    pair = state(t=2., x=.9)
    pair.local_odometry.pose.pose.position.z = .95
    node.map_report = None  # Preserve source arrival order without minting a fresh map.
    assert node.on_navigation(pair) is False
    node.on_local_navigation(local_state_from_pair(pair))
    # Map-report leases are no longer reference-delivery gates. This also
    # holds in the frozen pre-portal implementation: preserve the current
    # contract rather than restoring its obsolete map-proof dependency.
    assert node.map_report is None and node.proposal is not None
    node.tick()
    recovered = native_received(node)
    assert recovered.version.task_id == original.task_id
    assert recovered.version.route_hash == original.route_hash
    assert recovered.version.anchor_id != original.anchor_id
    point = recovered.reference.path.poses[0].pose.position
    assert (point.x, point.y, point.z) == pytest.approx((.9, 0., .95))
    assert node.transport.accepted == original  # Preview is not a native commit.
    assert node.on_local_commit(local_geometry_commit(node, clock, recovered, sequence=2, trajectory=2))
    assert node.transport.core._progress.measured_arc_m == pytest.approx(.9)
    assert node.transport.rolling_window_progress().end_arc_m == pytest.approx(4.9)
    assert not [value for kind, value in output if kind in ('handoff', 'sdk', 'motion', 'cancel_reference')]
