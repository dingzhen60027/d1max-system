import json
from pathlib import Path

import numpy as np
import pytest

from d1max_pct_planner.corridor_refinement import (
    refine_corridor, _corridor_quality, _quality_not_worse, _curve_peak_curvature,
    _rounded_segments, _conditioned_corridor_map,
    _corner_intersection_candidates, MAX_CORNER_ALTERNATIVES,
)
from d1max_pct_planner.tomogram_map import TomogramMap


def scene(allowed=None, layers=1):
    coords=(np.arange(161)-80)*.1
    x,y=np.meshgrid(coords,coords,indexing='ij')
    mask=np.ones(x.shape,bool) if allowed is None else allowed(x,y)
    data=np.zeros((5,layers,161,161),np.float32)
    data[0]=np.where(mask,0,50)
    data[4]=2.
    return TomogramMap({'data':data,'resolution':.1,'center':[0.,0.],
                        'slice_h0':.15,'slice_dh':.5})


def wavy_line():
    x=np.linspace(-4,4,161)
    return np.c_[x,.15*np.sin(np.pi*(x+4)),np.zeros(len(x))]


def wavy_l():
    s=np.linspace(0,3,81)
    horizontal=np.c_[s-3,.09*np.sin(5*np.pi*s/3),np.zeros(len(s))]
    vertical=np.c_[.09*np.sin(5*np.pi*s/3),s,np.zeros(len(s))]
    return np.vstack((horizontal,vertical[1:]))


def test_wavy_open_corridor_becomes_exact_straight_xy_and_keeps_endpoints():
    tomo=scene();path=wavy_line();path[0,2]=.04;path[-1,2]=-.03
    before=path.copy();r=refine_corridor(tomo,path,np.zeros(len(path),int))
    assert r['applied'] and r['reason']=='checked_straight_line'
    output=np.array(r['path'])
    assert np.array_equal(output[[0,-1]],path[[0,-1]])
    assert np.array_equal(path,before)
    assert np.max(abs(output[:,1]))<1e-12
    assert np.max(np.linalg.norm(np.diff(output[:,:2],axis=0),axis=1))<=.100000001
    assert r['after_quality']['curvature_max_per_m']<1e-9
    assert r['after_quality']['total_abs_turn_rad']<r['before_quality']['total_abs_turn_rad']
    assert not r['is_native_gpmp']
    assert r['segments'][0]['kind']=='line'
    tomo.validate_path(output,r['layer_ids'])
    json.dumps(r,allow_nan=False)


def test_l_corridor_gets_checked_zero_endpoint_curvature_bezier_corners():
    tomo=scene(lambda x,y: ((x>=-3.6)&(x<=.6)&(abs(y)<=.6))|((abs(x)<=.6)&(y>=-.6)&(y<=3.6)))
    path=wavy_l();r=refine_corridor(tomo,path,np.zeros(len(path),int),corner_cut_m=1.5)
    assert r['applied'],r
    assert r['reason']=='checked_c2_corners'
    assert any(segment['kind']=='quintic_bezier' for segment in r['segments'])
    output=np.asarray(r['path']);tomo.validate_path(output,r['layer_ids'])
    assert np.max(np.linalg.norm(np.diff(output[:,:2],axis=0),axis=1))<=.100000001
    for segment in r['segments']:
        if segment['kind']!='quintic_bezier':continue
        coefficients=np.asarray(segment['coefficients_xy'])
        acceleration=np.polynomial.polynomial.polyder(coefficients,m=2,axis=0)
        np.testing.assert_allclose(np.polynomial.polynomial.polyval([0.,1.],acceleration),0,atol=1e-10)
    for key in ('xy_length_m','xyz_length_m','total_abs_turn_rad','curvature_p95_per_m','curvature_max_per_m'):
        assert r['after_quality'][key] <= r['before_quality'][key]+1e-5


def test_blocked_direct_connection_cannot_be_published_through_a_wall():
    tomo=scene(lambda x,y: ~((abs(x)<=.5)&(abs(y)<=1.)))
    path=np.array([[-3.,0.,0.],[-1.,-2.,0.],[1.,-2.,0.],[3.,0.,0.]])
    r=refine_corridor(tomo,path,[0]*len(path))
    assert r.get('reason')!='checked_straight_line'
    if r['applied']:
        tomo.validate_path(r['path'],r['layer_ids'])
        assert any(point[1]<-1 for point in r['path'])
    else:
        assert r['reason']=='no_valid_nonworsening_smooth_candidate'
        assert 'path' not in r


def test_invalid_native_input_never_gets_repaired_by_bypassing_validation():
    tomo=scene(lambda x,y: abs(x)>.3)
    r=refine_corridor(tomo,[[-2.,0.,0.],[2.,0.,0.]],[0,0])
    assert not r['applied']
    assert r['reason']=='invalid_input_path'
    assert 'path' not in r


def test_unknown_ground_is_not_treated_as_clear_space():
    tomo=scene();tomo.ground[:,80,:]=np.nan;tomo.ground_known[:,80,:]=False;tomo.valid[:,80,:]=False
    r=refine_corridor(tomo,[[-2.,0.,0.],[2.,0.,0.]],[0,0])
    assert not r['applied'] and r['reason']=='invalid_input_path'


def test_layer_transitions_are_not_refined_even_with_geometric_overlap():
    tomo=scene(layers=2)
    path=[[-1.,0.,0.],[0.,0.,0.],[1.,0.,0.]]
    tomo.validate_path(path,[0,0,1])
    r=refine_corridor(tomo,path,[0,0,1])
    assert not r['applied'] and r['reason']=='multi_layer_path_not_supported'


def test_all_corner_scales_fail_without_falling_back_to_a_sharp_polyline():
    tomo=scene(lambda x,y: ((abs(y)<.001)&(x<=0)&(x>=-3.1))|((abs(x)<.001)&(y>=0)&(y<=3.1)))
    path=np.array([[-3.,0.,0.],[0.,0.,0.],[0.,3.,0.]])
    tomo.validate_path(path,[0,0,0])
    r=refine_corridor(tomo,path,[0,0,0],corner_cut_m=1.5)
    assert not r['applied'] and r['reason']=='no_valid_nonworsening_smooth_candidate'
    assert len(r['candidate_failures'])==3
    assert 'path' not in r


def test_duplicates_do_not_lose_exact_endpoints():
    tomo=scene();path=wavy_line();path=np.vstack((path[0],path,path[-1]))
    r=refine_corridor(tomo,path,[0]*len(path))
    assert r['applied']
    assert r['path'][0]==path[0].tolist() and r['path'][-1]==path[-1].tolist()


def test_short_straight_line_remains_valid_and_exact():
    tomo=scene();path=[[.01,.02,0.],[.06,.02,0.]]
    r=refine_corridor(tomo,path,[0,0])
    assert r['applied'] and r['path'][0]==path[0] and r['path'][-1]==path[-1]


def test_recorded_wall_side_exact_endpoints_survive_horner_roundoff():
    # Recorded same-floor input: a+(b-a) differs from b by one ULP. Optional
    # route-quality work must not turn this into refinement_endpoint_mismatch.
    start=np.array([-3.316805934834993,41.411893695942496,-.6499999761581421])
    goal=np.array([-15.001619339078106,44.14413374908944,-.6499999761581421])
    assert np.any(start[:2]+(goal[:2]-start[:2]) != goal[:2])
    data=np.zeros((5,1,181,181),np.float32);data[3]=start[2];data[4]=2.
    tomo=TomogramMap(dict(data=data,resolution=.1,center=[-9.,43.],slice_h0=0.,slice_dh=.5))
    middle=(start+goal)/2;middle[1]+=.1
    raw=np.vstack((start,middle,goal))
    r=refine_corridor(tomo,raw,[0,0,0])
    assert r['applied'],r
    np.testing.assert_array_equal(np.asarray(r['path'])[[0,-1]],raw[[0,-1]])
    tomo.validate_path(r['path'],r['layer_ids'])


def test_pure_z_route_is_not_a_corridor():
    r=refine_corridor(scene(),[[0.,0.,0.],[0.,0.,.02]],[0,0])
    assert not r['applied'] and r['reason']=='degenerate_or_closed_xy_route'


def test_invalid_layer_and_nonfinite_points_have_explicit_rejection():
    assert not refine_corridor(scene(),[[0.,0.,0.],[1.,0.,0.]],[0,99])['applied']
    assert not refine_corridor(scene(),[[0.,0.,0.],[np.nan,0.,0.]],[0,0])['applied']


def test_bad_cut_has_explicit_rejection():
    for cut in [0,-1,np.nan,True]:
        r=refine_corridor(scene(),[[0.,0.,0.],[1.,0.,0.]],[0,0],cut)
        assert not r['applied'] and r['reason']=='invalid_corner_cut'


def test_soft_cost_tradeoff_is_audited_without_changing_hard_limits():
    tomo=scene();tomo.cost[0,:,80]=15.
    x=np.linspace(-4,4,121)
    path=np.c_[x,.4*np.sin(np.pi*(x+4)/8),np.zeros(len(x))]
    r=refine_corridor(tomo,path,[0]*len(path))
    assert r['applied'] and r['reason']=='checked_straight_line'
    assert r['after_cost_audit']['mean_touched_cell_cost'] > r['before_cost_audit']['mean_touched_cell_cost']
    assert r['after_cost_audit']['max_cost']==15
    assert r['after_cost_audit']['hard_cost_threshold']==20
    assert r['soft_cost_policy']=='prefer_geometric_simplicity_within_unchanged_hard_pct_mask'
    assert not r['physical_safety_certified']


def test_length_budget_precedes_expensive_boundary_validation():
    class NoValidationAllowed:
        def validate_path(self,*args,**kwargs):
            raise AssertionError('Oversized path must be rejected before grid walking')
    path=np.array([[0.,0.,0.],[2000.,0.,0.]])
    r=refine_corridor(NoValidationAllowed(),path,[0,0])
    assert not r['applied'] and r['reason']=='input_length_limit'


def test_direct_visibility_reuses_geometry_but_revalidates_changed_map(monkeypatch):
    tomo=scene();path=wavy_line()
    original=tomo.validate_path
    calls=[]

    def mutate_after_visibility(*args,**kwargs):
        result=original(*args,**kwargs)
        calls.append(1)
        if len(calls)==2:  # Input route, then fully expanded visibility line.
            tomo.cost[0,80,80]=50.
        return result

    monkeypatch.setattr(tomo,'validate_path',mutate_after_visibility)
    result=refine_corridor(tomo,path,np.zeros(len(path),int))
    assert not result['applied']
    assert result['candidate_failures'][0]['reason']=='invalid_curve'
    assert 'path' not in result


def test_direct_line_performs_only_one_geometry_expansion(monkeypatch):
    tomo=scene();path=wavy_line();calls=[]
    original=tomo.surface_ground_z_many

    def counted(*args,**kwargs):
        calls.append(1)
        return original(*args,**kwargs)

    monkeypatch.setattr(tomo,'surface_ground_z_many',counted)
    result=refine_corridor(tomo,path,np.zeros(len(path),int))
    assert result['applied'] and calls==[1]
    tomo.validate_path(result['path'],result['layer_ids'])


def long_corridor_quality_regression():
    # Metrics from the real native route on the reviewed 2026-10-14 map. Full
    # native/map validation remains in the offline comparison, not this unit.
    before = dict(xy_length_m=56.6648492934, xyz_length_m=56.7018290129,
                  total_abs_turn_rad=2.9986802479, curvature_p95_per_m=.2758976184,
                  curvature_max_per_m=.7161442620)
    after = dict(xy_length_m=55.5202711516, xyz_length_m=56.0069852864,
                 total_abs_turn_rad=1.4775406447, curvature_p95_per_m=.0005953953,
                 curvature_max_per_m=1.2239049127)
    return before, after


def test_local_peak_alone_does_not_preserve_long_distributed_native_waviness():
    before, after = long_corridor_quality_regression()
    failures, audit = _corridor_quality(before, after, conditioned=True, curve_peak=1.24)
    assert failures == []
    assert audit['before'] == before['curvature_max_per_m']
    assert audit['after'] == after['curvature_max_per_m']
    assert not audit['dynamic_feasibility_certified']
    # The source-identity caller's existing strict contract is unchanged.
    assert _quality_not_worse(before, after) == ['curvature_max_per_m']


def test_local_peak_tradeoff_requires_both_length_and_turn_improvement():
    before, after = long_corridor_quality_regression()
    for key in ('xy_length_m', 'total_abs_turn_rad', 'curvature_p95_per_m'):
        candidate = {**after, key: before[key]}
        failures, audit = _corridor_quality(before, candidate, conditioned=True, curve_peak=1.24)
        assert failures == ['curvature_max_per_m'] and audit is None


def test_local_peak_tradeoff_cannot_hide_any_other_worse_metric():
    before, after = long_corridor_quality_regression()
    for key in ('xy_length_m', 'xyz_length_m', 'total_abs_turn_rad', 'curvature_p95_per_m'):
        candidate = {**after, key: before[key] + 1.}
        failures, audit = _corridor_quality(before, candidate, conditioned=True, curve_peak=1.24)
        assert key in failures and 'curvature_max_per_m' in failures and audit is None


def test_undefined_or_infinite_peak_is_not_a_permitted_tradeoff():
    before, after = long_corridor_quality_regression()
    for value in (None, float('inf')):
        failures, audit = _corridor_quality(before, {**after, 'curvature_max_per_m': value},
                                          conditioned=True, curve_peak=1.24)
        assert 'curvature_max_per_m' in failures and audit is None


def test_tiny_sharp_c2_corner_cannot_use_geometric_tradeoff():
    before, after = long_corridor_quality_regression()
    segments = _rounded_segments(np.array([[-1., 0.], [0., 0.], [0., 1.]]), .01)
    peak = _curve_peak_curvature(segments)
    assert peak > 100
    failures, audit = _corridor_quality(before, after, conditioned=True, curve_peak=peak)
    assert failures == ['curvature_max_per_m'] and audit is None


def test_unconditioned_map_or_missing_analytic_peak_keeps_strict_rule():
    before, after = long_corridor_quality_regression()
    for options in ({}, {'curve_peak': 1.24}, {'conditioned': True},
                    {'conditioned': True, 'curve_peak': 1.25001}):
        failures, audit = _corridor_quality(before, after, **options)
        assert failures == ['curvature_max_per_m'] and audit is None
    tomo = scene()
    assert not _conditioned_corridor_map(tomo)
    tomo.provenance = dict(geometry_operation='source_identity', frame_id='d1max_loc_map',
                          processing_schema='d1max.crossfloor_planning/v1')
    assert not _conditioned_corridor_map(tomo)


def test_conditioned_provenance_cannot_override_wall_or_layer_transition():
    provenance = dict(processing_schema='d1max.crossfloor_planning/v1',
                      geometry_operation='independent_floor_conditioning_visibility_filter_and_measured_return_restore',
                      frame_id='d1max_multifloor_planning')
    blocked = scene(lambda x,y: abs(x)>.3)
    blocked.provenance = provenance
    assert _conditioned_corridor_map(blocked)
    assert refine_corridor(blocked, [[-2.,0.,0.],[2.,0.,0.]], [0,0])['reason'] == 'invalid_input_path'
    layers = scene(layers=2)
    layers.provenance = provenance
    result = refine_corridor(layers, [[-1.,0.,0.],[0.,0.,0.],[1.,0.,0.]], [0,0,1])
    assert result['reason'] == 'multi_layer_path_not_supported' and 'path' not in result


def measured_clipped_corner():
    # One unchanged layer/crop from the real failed 2026-10-15 native request.
    # The file stores original map/route SHA256 and source crop bounds. Exporter:
    # experiments/corridor_residual_20261015.rPRDGm/export_refinement_fixture.py
    fixture = Path(__file__).parent/'fixtures/corridor_clipped_anchor_20261015.npz'
    with np.load(fixture, allow_pickle=False) as data:
        path, layers = data['path'].copy(), data['layers'].copy()
    tomo = TomogramMap(fixture, max_ground_step_m=.17)
    assert tomo.source_layers.tolist() == [4]
    return tomo, path, layers


def test_measured_greedy_cut_corner_gets_validated_alternative_not_native_waviness():
    tomo, path, layers = measured_clipped_corner()
    before_arrays = [array.copy() for array in (tomo.cost, tomo.ground, tomo.ceiling)]
    result = refine_corridor(tomo, path, layers)
    assert result['applied'], result
    assert result['visibility_candidate_index'] == 1
    assert result['alternative_polylines_checked'] == 1
    # Same input on the frozen old implementation fails these three candidates.
    assert [failure['reason'] for failure in result['prior_candidate_failures']] == [
        'pct_cost_blocked', 'pct_cost_blocked', 'quality_not_improved']
    assert result['prior_candidate_failures'][-1]['worse_metrics'] == ['curvature_max_per_m']
    output = np.asarray(result['path'])
    np.testing.assert_array_equal(output[[0,-1]], path[[0,-1]])
    assert result['after_quality']['xy_length_m'] < result['before_quality']['xy_length_m']
    assert result['after_quality']['total_abs_turn_rad'] < result['before_quality']['total_abs_turn_rad']
    tradeoff = result['quality_acceptance']['local_curvature_peak_tradeoff']
    assert tradeoff['polynomial_peak_per_m'] <= 1.25
    tomo.validate_path(output, result['layer_ids'])
    for before, after in zip(before_arrays, (tomo.cost, tomo.ground, tomo.ceiling)):
        np.testing.assert_array_equal(before, after)
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('unknown', [False, True])
def test_corner_alternative_never_crosses_real_wall_or_missing_ground(unknown):
    from scipy.spatial import cKDTree
    tomo, path, layers = measured_clipped_corner()
    valid = refine_corridor(tomo, path, layers)
    output = np.asarray(valid['path'])
    # Place the counterexample on the alternative's long straight segment,
    # outside the old native curve. The original input remains a valid route.
    separation, _ = cKDTree(path[:, :2]).query(output[:, :2])
    chosen = int(np.argmax(separation))
    assert separation[chosen] > .2
    cell = (0, *tomo.index(output[chosen, :2]))
    if unknown:
        tomo.ground[cell] = np.nan
        tomo.ground_known[cell] = False
    else:
        tomo.cost[cell] = 50.
    tomo.valid[cell] = False
    tomo.validate_path(path, layers)
    result = refine_corridor(tomo, path, layers)
    if result['applied']:
        # A newly checked detour is allowed; deleting the blocked/unknown cell
        # or publishing the original intersecting shortcut is not.
        candidate = np.asarray(result['path'])
        tomo.validate_path(candidate, result['layer_ids'])
        for a, b in zip(candidate[:-1], candidate[1:]):
            for _, cells in tomo._segment_samples_f64(a, b):
                assert cell[1:] not in cells
    else:
        assert 'path' not in result
        assert result['reason'] == 'no_valid_nonworsening_smooth_candidate'
    assert not tomo.valid[cell]
    if unknown:
        assert np.isnan(tomo.ground[cell])
    else:
        assert tomo.cost[cell] == 50.


def test_adjacent_corner_intersection_keeps_endpoints_and_reverse_symmetry():
    polyline = np.array([[-4.,0.],[-.3,0.],[.1,.3],[1.5,5.]])
    saved = polyline.copy()
    forward = _corner_intersection_candidates(polyline, 1.5)
    reverse = _corner_intersection_candidates(polyline[::-1], 1.5)
    assert len(forward) == len(reverse) == 1
    np.testing.assert_allclose(forward[0], reverse[0][::-1], atol=1e-12)
    np.testing.assert_array_equal(forward[0][[0,-1]], polyline[[0,-1]])
    np.testing.assert_array_equal(polyline, saved)
    assert _curve_peak_curvature(_rounded_segments(polyline, 1.5)) > 2.
    assert _curve_peak_curvature(_rounded_segments(forward[0], 1.5)) < 1.25


@pytest.mark.parametrize('polyline', [
    [[-1.,0.],[0.,0.],[1.,0.],[2.,0.]],  # parallel / collinear
    [[-1.,0.],[0.,0.],[.2,.2],[1.,.2]],  # S-turn, not a clipped corner
    [[-1.,0.],[0.,0.],[0.,1.],[-1.,1.]],  # U-turn
    [[-1.,0.],[0.,0.],[0.,1.],[-1.,1.1]],  # remote ray intersection
    [[-1.,0.],[0.,0.],[0.,0.],[0.,1.]],  # duplicate vertex
    [[-1.,0.],[0.,0.],[0.,1.],[np.nan,2.]],
])
def test_nonlocal_or_reversing_corner_is_not_an_alternative(polyline):
    assert _corner_intersection_candidates(np.asarray(polyline), 1.5) == []


def test_corner_alternative_count_is_bounded():
    angle = np.linspace(0., np.pi/2, 30)
    polyline = 10.*np.c_[np.cos(angle), np.sin(angle)]
    assert len(_corner_intersection_candidates(polyline, 1.5)) == MAX_CORNER_ALTERNATIVES
