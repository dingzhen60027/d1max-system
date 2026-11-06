from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from copy import deepcopy

from d1max_pct_scan.control_frame_contract import BodySample, Context, Rigid, create_anchor
from d1max_pct_scan.continuous_reference import ContinuousReference, ModeObservation, Observation
from d1max_pct_scan.source_route import SourceRouteBuilder, RouteSnapshot


class IdentityGroundBridge:
    source_frame = 'd1max_loc_map'
    planning_frame = 'd1max_multifloor_planning'

    def to_localization_ground(self, points, labels):
        return SimpleNamespace(xyz=np.asarray(points), diagnostics={})


def snapshot(crossfloor=False, points=None):
    if crossfloor:
        points = [[0., 0., 0.], [1., 0., 0.], [1., 0., .5], [1., 0., 1.], [0., 0., 1.]]
        names = ['lower_floor', 'stair_lower', 'stair_upper', 'upper_floor']
        labels = ['floor1', 'stair_lower', 'stair_upper', 'floor2', 'floor2']
        result = dict(layer_ids=[4, 4, 8, 12, 12], source_layer_ids=[4, 4, 8, 12, 12],
            direction='lower_to_upper', segments=[dict(name=name, first_index=i, last_index=i+1,
                **{'from': f'portal{i}', 'to': f'portal{i+1}'}) for i, name in enumerate(names)])
    else:
        points = points or [[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]
        labels = ['floor1']*len(points)
        result = dict(route_type='same_floor', layer_ids=[4]*len(points), source_layer_ids=[4]*len(points))
    return SourceRouteBuilder(bridge=IdentityGroundBridge(), source_map_sha256='a'*64,
        conditioning_sha256='b'*64, tomogram_sha256='c'*64).build(result, points, labels, map_version_id='map')


CONTEXT = Context('session', 1, 'seed', 'map')
BASE_NS = 10_000_000_000


def observation(x=0., y=0., z=.55, *, t=0., frame='d1max_loc_odom', context=CONTEXT):
    return Observation(BodySample(context, frame, 'd1max_loc_base_link',
        BASE_NS+int(t*1e9), Rigid((x, y, z), (0., 0., 0., 1.))), 100.+t)


def reference(crossfloor=False, source=None, anchor=None):
    anchor = anchor or create_anchor(observation(frame='d1max_loc_map').body, observation().body, 1)
    return ContinuousReference(source or snapshot(crossfloor), context=CONTEXT, task_id='task',
        anchor=anchor, body_height_m=.55, body_height_calibration_id='unverified-preview-config')


def segmented_snapshot(*, landing=False):
    value=snapshot().payload()
    first,second=deepcopy(value['segments'][0]),deepcopy(value['segments'][0])
    first.update(segment_id='corridor_a',end_index=1,entry_portal_id='start',exit_portal_id='join')
    second.update(segment_id='corridor_b',begin_index=1,entry_portal_id='join',exit_portal_id='goal',
                  kind='landing' if landing else 'floor')
    value['segments']=[first,second]
    value['edge_segments']=['corridor_a','corridor_b']
    return RouteSnapshot.create(value)


def project(ref, sample):
    return ref.project(sample, current_source_ns=sample.body.source_ns, now_monotonic=sample.received_monotonic)


def window(ref, t=0., **kwargs):
    return ref.window(current_source_ns=BASE_NS+int(t*1e9), now_monotonic=100.+t, **kwargs)


def test_reference_adds_height_once_then_fixed_rigid_transform_and_keeps_source_times():
    # map <- odom is translation (10, 0, 0): do not mistake it for the
    # non-rigid planning-map correction, which has happened before this input.
    anchor = create_anchor(observation(frame='d1max_loc_map').body, observation(x=-10.).body, 1)
    ref = reference(anchor=anchor)
    project(ref, observation(x=-10.))
    result = window(ref)
    assert result.body_odom_xyz[0] == (-10., 0., .55)
    assert result.ground_source_xyz[0] == (0., 0., 0.)
    assert result.ground_to_body_height_applications == 1
    assert result.source_ns == BASE_NS and result.received_monotonic == 100.
    assert result.frame_id == 'd1max_loc_odom' and not result.execution_eligible
    assert result.route_hash == ref.snapshot.route_hash


def test_sender_detour_bound_matches_native_input_sphere_after_rigid_anchor():
    from d1max_pct_scan.reference_geometry_contract import (
        LATERAL_RADIUS_M, PROJECTION_RADIUS_M, NATIVE_START_DISTANCE_M)
    # A translated map/odom pair makes a duplicate transform conspicuous.
    anchor = create_anchor(observation(frame='d1max_loc_map').body,
                           observation(x=-10.).body, 1)
    ref = reference(anchor=anchor)
    body = observation(x=-10., y=LATERAL_RADIUS_M-.01,
                       z=.55+PROJECTION_RADIUS_M-.01)
    progress = project(ref, body)
    out = window(ref)
    gap = np.linalg.norm(np.asarray(out.body_odom_xyz[0])-body.body.pose.xyz)
    assert 1. < gap <= NATIVE_START_DISTANCE_M
    assert ref.lateral == LATERAL_RADIUS_M and ref.radius == PROJECTION_RADIUS_M
    assert out.body_odom_xyz[0] == (-10., 0., .55)
    assert out.ground_to_body_height_applications == 1
    assert progress.measured_arc_m == 0.
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        project(reference(anchor=anchor), observation(x=-10., y=LATERAL_RADIUS_M+.01))


def test_candidate_shares_only_immutable_geometry_and_keeps_dynamic_state_separate():
    ref = reference()
    project(ref, observation())
    staged = deepcopy(ref)
    for name in ('snapshot', '_points', '_arc', '_segments', 'support_evidence'):
        assert getattr(staged, name) is getattr(ref, name)
    for array in (staged._points, staged._arc, staged.support_evidence.observed_source_support_xyz):
        with pytest.raises(ValueError):
            array.flags.writeable = True
    with pytest.raises(TypeError):
        staged._segments[0]['kind'] = 'stair'
    assert staged._history is not ref._history
    staged._history.clear()
    assert len(ref._history) == 1
    project(staged, observation(x=.1, t=.1))
    assert ref._progress.source_ns == BASE_NS
    assert staged._progress.source_ns == BASE_NS + 100_000_000


def test_small_reverse_is_measured_but_confirmed_progress_never_goes_backward():
    ref = reference()
    project(ref, observation())
    forward = project(ref, observation(x=.5, t=.5))
    reverse = project(ref, observation(x=.4, t=.6))
    assert forward.measured_arc_m == pytest.approx(.5)
    assert reverse.measured_arc_m == pytest.approx(.4)
    assert reverse.confirmed_arc_m == pytest.approx(.5)
    assert window(ref, .6).body_odom_xyz[0][0] == pytest.approx(.4)


def test_large_reverse_or_spatial_teleport_does_not_update_progress():
    ref = reference()
    project(ref, observation())
    before = project(ref, observation(x=.5, t=.5))
    with pytest.raises(ValueError, match='discontinuity'):
        project(ref, observation(x=1.8, t=.51))
    assert ref._progress == before


@pytest.mark.parametrize('mode', ['source_stale', 'receipt_stale', 'future_source', 'duplicate', 'old_epoch'])
def test_freshness_and_epoch_fail_closed_without_refreshing_admitted_window(mode):
    ref = reference()
    project(ref, observation())
    sample = observation(t=.1)
    current, now = sample.body.source_ns, sample.received_monotonic
    if mode == 'source_stale':
        current += 1_000_000_000
    elif mode == 'receipt_stale':
        now += 1.
    elif mode == 'future_source':
        current -= 1
    elif mode == 'duplicate':
        sample = observation()
    else:
        sample = replace(sample, body=replace(sample.body, context=replace(CONTEXT, epoch=2)))
    with pytest.raises(ValueError):
        ref.project(sample, current_source_ns=current, now_monotonic=now)
    assert ref._progress.source_ns == BASE_NS
    with pytest.raises(ValueError, match='fresh'):
        window(ref, 1.)


def test_same_xy_upper_floor_cannot_be_selected_from_lower_segment():
    ref = reference(crossfloor=True)
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        project(ref, observation(z=1.55))
    assert ref._progress is None and ref._segment == 0


def test_route_crossing_does_not_choose_arbitrary_far_along_branch():
    route = snapshot(points=[[0., 0., 0.], [.4, 0., 0.], [0., 0., 0.], [0., .4, 0.]])
    ref = reference(source=route)
    with pytest.raises(ValueError, match='ambiguous'):
        project(ref, observation())


def test_window_stops_at_semantic_boundary_and_cannot_skip_stair_portal():
    ref = reference(crossfloor=True)
    project(ref, observation())
    result = window(ref, horizon_m=8.)
    assert result.segment_id == 'lower_floor'
    assert result.ground_source_xyz[-1] == (1., 0., 0.)
    assert set(result.edge_indices) == {0}
    with pytest.raises(ValueError, match='not_physically_reached'):
        ref.advance_segment(current_source_ns=BASE_NS, now_monotonic=100.)


def at_portal():
    ref = reference(crossfloor=True)
    project(ref, observation())
    project(ref, observation(x=.95, t=.8))
    return ref


def test_stair_portal_requires_actual_bound_mode_even_when_stationary():
    ref = at_portal()
    with pytest.raises(ValueError, match='requires_bound_mode'):
        ref.advance_segment(current_source_ns=BASE_NS+800_000_000, now_monotonic=100.8)
    project(ref, observation(x=.95, t=.9))
    project(ref, observation(x=.95, t=1.))
    with pytest.raises(ValueError, match='requires_bound_mode'):
        ref.advance_segment(current_source_ns=BASE_NS+1_000_000_000, now_monotonic=101.)
    mode=ModeObservation(CONTEXT,'task',ref.snapshot.route_hash,'stair_lower','stair',
        BASE_NS+1_000_000_000,101.)
    assert ref.advance_segment(current_source_ns=mode.source_ns,now_monotonic=101.,mode=mode)=='stair_lower'
    with pytest.raises(ValueError, match='fresh_measured_projection'):
        window(ref, 1.)
    with pytest.raises(ValueError, match='not_newer'):
        project(ref, observation(x=.95, t=1.))
    p = project(ref, observation(x=1., t=1.1))
    assert p.segment_id == 'stair_lower'
    assert window(ref, 1.1).required_mode == 'stair'


def test_fresh_mode_evidence_must_bind_target_segment_task_hash_and_context():
    ref = at_portal()
    mode = ModeObservation(CONTEXT, 'other-task', ref.snapshot.route_hash,
        'stair_lower', 'stair', BASE_NS+800_000_000, 100.8)
    with pytest.raises(ValueError, match='requires_bound_mode'):
        ref.advance_segment(current_source_ns=mode.source_ns, now_monotonic=100.8, mode=mode)
    assert ref.advance_segment(current_source_ns=mode.source_ns, now_monotonic=100.8,
                               mode=replace(mode, task_id='task')) == 'stair_lower'


@pytest.mark.parametrize('landing',[False,True])
def test_same_floor_transition_is_a_continuous_candidate_with_literal_measured_source(landing):
    ref=reference(source=segmented_snapshot(landing=landing))
    project(ref,observation())
    sample=observation(x=.95,t=.8)
    original=project(ref,sample)
    staged=ref.prepare_segment_transition(sample,current_source_ns=sample.body.source_ns,now_monotonic=100.8)
    assert ref._segment==0 and ref._progress==original
    assert staged._segment==1 and staged.snapshot is ref.snapshot and staged.anchor==ref.anchor
    assert staged._progress.segment_id=='corridor_b'
    assert staged._progress.source_ns==sample.body.source_ns
    assert staged._progress.received_monotonic==sample.received_monotonic
    assert staged._progress.body_source_xyz==sample.body.pose.xyz
    assert staged._confirmed>=original.confirmed_arc_m
    assert window(staged,.8).body_odom_xyz[-1]==(2.,0.,.55)
    assert not ref._stationary()  # No stop is manufactured at an ordinary join.


def test_ordinary_segment_classification_is_not_a_floor_name_allowlist():
    ref=reference(source=segmented_snapshot())
    ref._segments=tuple(dict(segment,floor_id='outdoor_west') for segment in ref._segments)
    assert ref.adjacent_general_segment()
    ref._segments[1]['floor_id']='outdoor_east'
    assert not ref.adjacent_general_segment()
    ref._segments[1]['floor_id']='outdoor_west';ref._segments[1]['required_mode']='stair'
    assert not ref.adjacent_general_segment()


@pytest.mark.parametrize('bad',['far','future','foreign','changed_pose','upper_z','old_receipt'])
def test_adjacent_candidate_cannot_use_foreign_or_unmeasured_portal_source(bad):
    ref=reference(source=segmented_snapshot());project(ref,observation())
    sample=observation(x=.95 if bad!='far' else .5,t=.8);before=project(ref,sample)
    if bad=='future':sample=observation(x=.95,t=.81)
    elif bad=='foreign':sample=replace(sample,body=replace(sample.body,context=replace(CONTEXT,epoch=2)))
    elif bad=='changed_pose':sample=observation(x=.94,t=.8)
    elif bad=='upper_z':sample=observation(x=.95,z=1.55,t=.8)
    elif bad=='old_receipt':sample=replace(sample,received_monotonic=100.79)
    with pytest.raises(ValueError):
        ref.prepare_segment_transition(sample,current_source_ns=BASE_NS+800_000_000,now_monotonic=100.8)
    assert ref._segment==0 and ref._progress==before


def test_normal_map_correction_is_only_a_candidate_and_does_not_drag_curve_or_route():
    ref = reference()
    project(ref, observation())
    before = window(ref)
    global_sample = observation(x=.2, frame='d1max_loc_map', t=.1)
    candidate = ref.observe_pair(global_sample, observation(t=.1), revision=2,
        current_source_ns=global_sample.body.source_ns, now_monotonic=100.1)
    after = window(ref, .1)
    assert candidate.map_from_odom.xyz[0] == pytest.approx(.2)
    assert candidate.anchor_id != ref.anchor.anchor_id
    assert before == after
    assert ref.snapshot.route_hash == before.route_hash
    with pytest.raises(ValueError, match='same-source-time'):
        ref.observe_pair(global_sample, observation(t=.2), revision=3,
            current_source_ns=BASE_NS+200_000_000, now_monotonic=100.2)


def test_epoch_reset_cannot_be_disguised_as_regular_map_correction():
    ref = reference()
    changed = replace(CONTEXT, epoch=2)
    with pytest.raises(ValueError, match='context_or_version_changed'):
        ref.observe_pair(observation(frame='d1max_loc_map', t=.1, context=changed),
            observation(t=.1, context=changed), revision=2,
            current_source_ns=BASE_NS+100_000_000, now_monotonic=100.1)
    assert ref.candidate_anchor is None


def test_endpoint_does_not_emit_a_zero_length_fake_trajectory():
    ref = reference()
    project(ref, observation(x=1.))
    project(ref, observation(x=2., t=1.))
    with pytest.raises(ValueError, match='endpoint_reached'):
        window(ref, 1.)


def test_bounded_side_detour_keeps_progress_but_height_and_far_offsets_fail_closed():
    ref = reference()
    project(ref, observation())
    t = 0.
    # A box bypass: drift sideways to 0.9 m at 0.3 m/s while advancing.
    for i in range(1, 16):
        t = .2*i
        progress = project(ref, observation(x=.04*i, y=.06*i, t=t))
    assert progress.cross_track_m == pytest.approx(.9)
    assert progress.measured_arc_m == pytest.approx(.6)
    assert window(ref, t).body_odom_xyz[0][0] == pytest.approx(.6)
    # Height mismatch keeps the tight correspondence radius.
    high = reference()
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        project(high, observation(y=.2, z=.55+.35))
    assert high._progress is None
    # Beyond the bounded lateral radius the reference still fails closed.
    far = reference()
    project(far, observation())
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        project(far, observation(y=1.45, t=5.))
    assert far._progress.measured_arc_m == 0.


def test_reanchor_cannot_use_detour_allowance_to_slide_sideways():
    ref = reference()
    project(ref, observation())
    project(ref, observation(y=.2, t=.5))
    global_sample = observation(y=.2+.6, frame='d1max_loc_map', t=.6)
    ref.observe_pair(global_sample, observation(y=.2, t=.6), revision=2,
        current_source_ns=global_sample.body.source_ns, now_monotonic=100.6)
    with pytest.raises(ValueError, match='outside_current_semantic_segment'):
        ref.prepare_reanchor(observation(y=.2, t=.6), current_source_ns=global_sample.body.source_ns,
                             now_monotonic=100.6)


@pytest.mark.parametrize('installed_without_progress',[False,True])
def test_fresh_correction_can_recover_stale_old_anchor_correspondence(installed_without_progress):
    ref=reference();project(ref,observation())
    old_anchor=ref.anchor
    # Literal local pose leaves the old anchor's vertical association while
    # the same-time global pose stays on this exact semantic route. This is
    # an anchor correction, not a fabricated fresh projection of the old one.
    local=observation(z=.95,t=1.)
    global_=observation(frame='d1max_loc_map',t=1.)
    ref.observe_pair(global_,local,revision=2,current_source_ns=local.body.source_ns,
        now_monotonic=local.received_monotonic)
    with pytest.raises(ValueError,match='outside_current_semantic_segment'):
        project(ref,local)
    if installed_without_progress:
        ref._progress=None  # exact production owner-commit failure disposition
    staged=ref.prepare_reanchor(local,current_source_ns=local.body.source_ns,
        now_monotonic=local.received_monotonic)
    assert staged._progress.source_ns==local.body.source_ns
    assert staged._progress.measured_arc_m==0.
    assert staged._progress.body_source_xyz==(0.,0.,.55)
    assert ref.anchor==old_anchor and staged.anchor!=old_anchor
    assert ref.snapshot.route_hash==staged.snapshot.route_hash
    assert ref._last_sample[0]==BASE_NS
    if installed_without_progress:assert ref._progress is None
    else:assert ref._progress.source_ns==BASE_NS


@pytest.mark.parametrize('case',['stale_correction','wrong_height','far_along','wrong_floor'])
def test_old_progress_recovery_never_bypasses_current_candidate_correspondence(case):
    ref=reference();project(ref,observation())
    local=observation(z=.95,t=1.)
    global_=observation(frame='d1max_loc_map',t=1.)
    if case in ('wrong_height','wrong_floor'):
        global_=observation(frame='d1max_loc_map',z=.9 if case=='wrong_height' else 3.55,t=1.)
    elif case=='far_along':global_=observation(frame='d1max_loc_map',x=1.,t=1.)
    ref.observe_pair(global_,local,revision=2,current_source_ns=local.body.source_ns,now_monotonic=101.)
    ref._progress=None
    if case=='stale_correction':local=observation(z=.95,t=2.)
    with pytest.raises(ValueError):
        ref.prepare_reanchor(local,current_source_ns=local.body.source_ns,
            now_monotonic=local.received_monotonic)
    assert ref.anchor.revision==1 and ref._progress is None
