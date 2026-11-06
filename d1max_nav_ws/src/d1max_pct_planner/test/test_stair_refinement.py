import numpy as np
import pytest
from copy import deepcopy

from d1max_pct_planner.tomogram_map import TomogramMap, TomogramError
from d1max_pct_planner.stair_refinement import (
    refine_stair_route, _clearance_quality, _quality_admissible, MAX_INPUT_POINTS,
)
from d1max_pct_planner.corridor_centering import ClearanceFieldCache


LIMITS = dict(max_ground_step_m=.17, max_vertical_grade=1.,
              vertical_step_allowance_m=.17, max_reverse_height_m=.35,
              max_selection_height_error_m=.08)
ROI = ([-4., -4., -.1], [4., 4., .1])


def scene():
    data = np.zeros((5, 2, 91, 91), dtype=np.float32)
    data[4] = 2.
    x, y = np.indices((91,91))*.1-4.5
    mask = (np.abs(x) < 1.) & (y < 1.5)
    data[0,:,mask] = 50.
    data[0,:,:2] = data[0,:,-2:] = 50.
    data[0,:,:,:2] = data[0,:,:,-2:] = 50.
    grid = TomogramMap(dict(data=data, resolution=.1, center=[0.,0.], slice_h0=.5, slice_dh=.5))
    points = np.asarray([[-2.,-2.,0.], [-2.,1.,0.], [-1.3,1.8,0.],
                         [-.2,2.,0.], [0.,3.2,0.], [.2,2.,0.],
                         [1.3,1.8,0.], [2.,1.,0.], [2.,-2.,0.]])
    return grid, points, np.zeros(len(points), dtype=int), 4


def run(grid, points, layers, landing, **kwargs):
    return refine_stair_route(grid, points, layers, landing_index=landing,
                              stair_roi=ROI, limits=LIMITS, **kwargs)


def test_joint_platform_rounding_preserves_geometry_and_reports_effective_landing():
    grid, points, layers, landing = scene()
    original = grid.data.copy()
    result = run(grid, points, layers, landing)
    assert result['applied'], result['reason']
    candidate = np.asarray(result['path'])
    assert np.array_equal(candidate[[0,-1]], points[[0,-1]])
    assert result['landing_displacement_m'] > .01
    assert result['requested_landing_xyz'] == points[landing].tolist()
    assert result['effective_landing_xyz'] == result['path'][result['landing_index']]
    assert result['after_quality']['curvature_max_per_m'] < result['before_quality']['curvature_max_per_m']
    assert result['after_clearance']['continuous_clearance_min_m'] >= (
        result['before_clearance']['continuous_clearance_min_m']-.001)
    assert grid.validate_path(candidate,result['layer_ids'])['checked_cells'] > 0
    np.testing.assert_array_equal(grid.data,original)
    assert np.all(candidate[:,2] == 0.)


def test_downward_order_remains_valid_and_exact_endpoints_are_reversed():
    grid, points, layers, landing = scene()
    result = run(grid,points[::-1],layers[::-1],len(points)-1-landing)
    assert result['applied']
    np.testing.assert_array_equal(np.asarray(result['path'])[[0,-1]],points[[-1,0]])
    grid.validate_path(result['path'],result['layer_ids'])


def test_original_slice_transition_pair_is_never_shifted():
    grid, points, layers, landing = scene()
    layers[2:] = 1
    result = run(grid, points, layers, landing)
    candidate, ids = np.asarray(result['path']),np.asarray(result['layer_ids'])
    transition = np.flatnonzero(np.diff(ids))
    assert len(transition) == 1
    np.testing.assert_array_equal(candidate[transition[0]:transition[0]+2],points[1:3])
    assert result['layer_ids'][result['landing_index']] == layers[landing]


def test_invalid_input_support_is_not_repaired_or_cleared():
    grid, points, layers, landing = scene()
    points[3] = [0.,0.,0.]
    with pytest.raises(TomogramError):
        run(grid, points, layers, landing)


def test_invalid_vertical_input_is_not_flattened():
    grid, points, layers, landing = scene()
    points[3,2] = 1.
    with pytest.raises(TomogramError):
        run(grid, points, layers, landing)


def test_work_budget_returns_identical_input_not_navigation_failure():
    grid, points, layers, landing = scene()
    points = np.repeat(points, MAX_INPUT_POINTS//len(points)+1,axis=0)
    layers = np.zeros(len(points),dtype=int)
    result = run(grid,points,layers,len(points)//2)
    assert not result['applied'] and result['reason'] == 'stair_refinement_work_limit'
    np.testing.assert_array_equal(result['path'],points)


def test_failed_optional_geometry_keeps_valid_original(monkeypatch):
    from d1max_pct_planner import stair_refinement
    grid, points, layers, landing = scene()
    monkeypatch.setattr(stair_refinement,'refine_corridor',
                        lambda *a,**k: dict(applied=False,reason='blocked_candidate'))
    monkeypatch.setattr(stair_refinement,'center_corridor',
                        lambda *a,**k: dict(applied=False,reason='no_better_candidate'))
    result=run(grid,points,layers,landing)
    assert not result['applied']
    np.testing.assert_array_equal(result['path'],points)
    assert result['landing_index']==landing


def test_continuous_clearance_min_does_not_depend_on_vertex_density():
    grid, points, layers, landing = scene()
    first = _clearance_quality(grid,points,layers,ClearanceFieldCache(grid))
    densified = np.vstack([a+(b-a)*np.arange(20)[:,None]/20 for a,b in zip(points[:-1],points[1:])]
                          + [points[-1:]])
    second = _clearance_quality(grid,densified,np.zeros(len(densified),int),ClearanceFieldCache(grid))
    assert abs(first['continuous_clearance_min_m']-second['continuous_clearance_min_m']) < 1e-6


def test_geometry_score_cannot_buy_worse_minimum_clearance():
    grid, points, layers, landing=scene()
    quality=_clearance_quality(grid,points,layers,ClearanceFieldCache(grid))
    worse=dict(quality,score=0.,continuous_clearance_min_m=quality['continuous_clearance_min_m']-.05)
    assert not _quality_admissible(quality,worse)


def test_small_turn_tradeoff_requires_both_curvatures_clearance_and_score_improve():
    grid, points, layers, landing=scene()
    old=_clearance_quality(grid,points,layers,ClearanceFieldCache(grid))
    new=deepcopy(old)
    new['score']-=.1
    new['geometry_quality']['total_abs_turn_rad']+=np.deg2rad(5.)
    new['geometry_quality']['curvature_p95_per_m']*=.8
    new['geometry_quality']['curvature_max_per_m']*=.5
    assert not _quality_admissible(old,new)  # Whole joined route cannot worsen.
    assert _quality_admissible(old,new,allow_small_turn_tradeoff=True)
    for key,value in [('continuous_clearance_min_m',old['continuous_clearance_min_m']-.02),
                      ('score',old['score'])]:
        bad=deepcopy(new);bad[key]=value
        assert not _quality_admissible(old,bad,allow_small_turn_tradeoff=True)
    for key,value in [('total_abs_turn_rad',old['geometry_quality']['total_abs_turn_rad']+np.deg2rad(11.)),
                      ('curvature_p95_per_m',old['geometry_quality']['curvature_p95_per_m'])]:
        bad=deepcopy(new);bad['geometry_quality'][key]=value
        assert not _quality_admissible(old,bad,allow_small_turn_tradeoff=True)


def test_stair_policy_does_not_change_default_floor_refinement(monkeypatch):
    from d1max_pct_planner.corridor_refinement import refine_corridor
    grid, points, layers, landing=scene()
    normal=refine_corridor(grid,points,layers)
    explicit=refine_corridor(grid,points,layers,quality_policy='floor')
    assert normal==explicit
    with pytest.raises(ValueError):
        refine_corridor(grid,points,layers,quality_policy='unknown')


@pytest.mark.parametrize('failure',['occupied','unknown','low_ceiling'])
def test_bad_shortcut_keeps_original_support_and_obstacle_boundaries(monkeypatch, failure):
    from d1max_pct_planner import stair_refinement
    grid, points, layers, landing=scene()
    cell=tuple(grid.index([0.,0.]))
    if failure=='unknown':
        grid.ground[(0,*cell)]=np.nan
        grid.ground_known[(0,*cell)]=False
    elif failure=='low_ceiling':
        grid.cost[(0,*cell)]=0.
        grid.valid[(0,*cell)]=True
        grid.headroom[(0,*cell)]=.1
        grid.ceiling_known[(0,*cell)]=True
    bad=dict(applied=True,reason='injected_invalid_shortcut',
             path=[points[0].tolist(),[0.,0.,0.],points[-1].tolist()],
             layer_ids=[0,0,0],curve_validation='injected',curve_partition_points=3)
    monkeypatch.setattr(stair_refinement,'refine_corridor',lambda *a,**k:bad)
    monkeypatch.setattr(stair_refinement,'center_corridor',
                        lambda *a,**k:dict(applied=False,reason='no_other_candidate'))
    result=run(grid,points,layers,landing)
    assert not result['applied']
    np.testing.assert_array_equal(result['path'],points)
    assert grid.validate_path(result['path'],result['layer_ids'])['checked_cells']>0


def test_real_height_gradient_and_reverse_are_never_flattened():
    grid, points, layers, landing=scene()
    # Both retained aliases observe the same measured incline. This test
    # verifies Z lifting, not stair dynamics or a simulated control rollout.
    y=(np.arange(grid.ny)-grid.offset[1])*.1
    grid.data[3]=np.broadcast_to(.05*y,(grid.layers,grid.nx,grid.ny))
    grid.ground[:]=grid.data[3]
    points[:,2]=[grid.surface_ground_z(p[:2],0) for p in points]
    wider_roi=([-4.,-4.,-.5],[4.,4.,.5])
    for path in [points,points[::-1]]:
        result=refine_stair_route(grid,path,layers,landing_index=landing,stair_roi=wider_roi,limits=LIMITS)
        for xyz,layer in zip(result['path'],result['layer_ids']):
            assert abs(xyz[2]-grid.surface_ground_z(xyz[:2],layer))<1e-6
        np.testing.assert_array_equal(np.asarray(result['path'])[[0,-1]],path[[0,-1]])


@pytest.mark.parametrize('swap_alias_ids', [False, True])
def test_shared_platform_slice_alias_does_not_lock_hairpin(swap_alias_ids):
    grid, points, layers, landing = scene()
    layers[5:] = 1  # Same measured platform; retained slice changes at the turn.
    if swap_alias_ids:
        layers = 1-layers
    original = grid.data.copy()
    result = run(grid, points, layers, landing)
    assert result['after_quality']['curvature_max_per_m'] < 6., result['after_quality']
    assert result['platform_alias_refinement']['applied']
    assert result['platform_alias_refinement']['alias_layers'] == [0, 1]
    np.testing.assert_array_equal(np.asarray(result['path'])[[0,-1]], points[[0,-1]])
    assert result['layer_ids'][result['landing_index']] == layers[landing]
    grid.validate_path(result['path'], result['layer_ids'])
    np.testing.assert_array_equal(grid.data, original)


def test_reverse_platform_candidate_cannot_trade_away_existing_p95_quality():
    from d1max_pct_planner.stair_refinement import _refine_stair_spans
    grid, points, layers, landing = scene()
    layers[5:] = 1
    points, layers, landing = points[::-1], layers[::-1], len(points)-1-landing
    preceding = _refine_stair_spans(grid, points, layers, landing_index=landing,
                                  stair_roi=ROI, limits=LIMITS)
    result = run(grid, points, layers, landing)
    # This deliberately sparse reverse fixture has a good existing P95 but a
    # high isolated peak. A lower peak alone is not permission to worsen the
    # established whole-route P95 or clearance contract.
    assert not result['platform_alias_refinement']['applied']
    assert any(not item['quality_accepted']
               for item in result['platform_alias_refinement']['candidates'])
    np.testing.assert_array_equal(result['path'], preceding['path'])
    assert result['after_quality'] == preceding['after_quality']


@pytest.mark.parametrize('problem', ['pillar', 'missing_support', 'low_ceiling', 'different_surface'])
def test_shared_platform_cannot_borrow_bad_or_different_alias_cells(problem):
    grid, points, layers, landing = scene()
    layers[5:] = 1
    # The tempting platform shortcut crosses this cell, but the original U
    # route goes around it. Keep both original paths valid in every case.
    cell = tuple(grid.index([0., 2.2]))
    if problem == 'pillar':
        grid.cost[(1,*cell)] = 50.
        grid.valid[(1,*cell)] = False
    elif problem == 'missing_support':
        grid.ground[(1,*cell)] = np.nan
        grid.ground_known[(1,*cell)] = False
        grid.valid[(1,*cell)] = False
    elif problem == 'low_ceiling':
        grid.ceiling[(1,*cell)] = .1
        grid.headroom[(1,*cell)] = .1
        grid.valid[(1,*cell)] = False
    else:
        grid.ground[(1,*cell)] = .30
        grid.headroom[(1,*cell)] = 1.70
    result = run(grid, points, layers, landing)
    grid.validate_path(result['path'], result['layer_ids'])
    # The alias optimization may retain a longer valid route or do nothing,
    # but must never erase the only-slice obstacle/support discrepancy.
    alias = result.get('platform_alias_refinement', {})
    if alias.get('applied'):
        first, last = alias['output_first_index'], alias['output_last_index']
        path = np.asarray(result['path'])[first:last+1]
        touched = set()
        for a, b in zip(path[:-1],path[1:]):
            for _, cells in grid._segment_samples_f64(a,b):
                touched.update(cells)
        assert cell not in touched


def test_platform_aliasing_never_merges_different_floor_heights():
    grid, points, layers, landing = scene()
    layers[5:] = 1
    grid.ground[1] = .10
    grid.headroom[1] = 1.90
    points[5:,2] = .10
    result = run(grid, points, layers, landing)
    assert not result.get('platform_alias_refinement', {}).get('applied', False)
    assert set(result['layer_ids']) == {0,1}
    assert max(np.asarray(result['path'])[:,2]) == .10
