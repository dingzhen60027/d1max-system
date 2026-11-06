import numpy as np
import pytest

from tools.pointcloud_preprocessing.stair_parametric_geometry import (
    _regular_wall_strips, _clip_polygon, _polygon_difference, _triangle_surface)


def test_outboard_wall_thickness_does_not_double_as_inward_margin():
    s,z=np.meshgrid(np.arange(-1,1.001,.05),np.arange(.3,1.01,.1))
    cloud=np.r_[np.c_[s.ravel(),np.full(s.size,-.7),z.ravel()],
                np.c_[s.ravel(),np.full(s.size,-1.0),z.ravel()],
                np.c_[s.ravel(),np.full(s.size,.7),z.ravel()]]
    bins=[[v,t,10] for v in np.arange(-.9,.91,.2) for t in []]
    fitted={'sides':{name:dict(lateral_slope_intercept=[0,t],
         bins=[[v,t,10] for v in np.arange(-.9,.91,.2)],accepted_bins=list(range(10)))
         for name,t in [('negative',-.7),('positive',.7)]}}
    boxes,evidence=_regular_wall_strips(cloud,np.arange(len(cloud)),np.array([0.,0.]),
        np.array([1.,0.]),[-1,1],0.,fitted,'flight','a'*64)
    negative=[b for b in boxes if 'negative_fitted' in b['structure_id']]
    assert negative
    for box in negative:
        assert box['pose']['position'][1]+box['half_extents_xyz'][1] <= -.6999
        assert box['pose']['position'][1]-box['half_extents_xyz'][1] <= -1.
    assert evidence[0]['coherent_inward_records']==0


def test_coherent_inward_structure_retained_but_single_jitter_not_wall_outline():
    s,z=np.meshgrid(np.arange(-1,1.001,.05),np.arange(.3,1.01,.1))
    cloud=np.r_[np.c_[s.ravel(),np.full(s.size,-.7),z.ravel()],
                np.c_[s.ravel(),np.full(s.size,.7),z.ravel()],
                [[0.8,-.5,.6]]]
    ps,pz=np.meshgrid(np.arange(-.1,.11,.04),np.arange(.4,.61,.04))
    cloud=np.r_[cloud,np.c_[ps.ravel(),np.full(ps.size,-.5),pz.ravel()]]
    fitted={'sides':{name:dict(lateral_slope_intercept=[0,t],
         bins=[[v,t,10] for v in np.arange(-.9,.91,.2)],accepted_bins=list(range(10)))
         for name,t in [('negative',-.7),('positive',.7)]}}
    boxes,evidence=_regular_wall_strips(cloud,np.arange(len(cloud)),np.array([0.,0.]),
        np.array([1.,0.]),[-1,1],0.,fitted,'flight','a'*64)
    assert any('protrusion' in b['structure_id'] for b in boxes)
    assert evidence[0]['isolated_residuals_retained_in_source_not_wall_silhouette']==1


def _area(polygon):
    return abs(np.cross(polygon,np.roll(polygon,-1,axis=0)).sum())/2


def test_landing_subtraction_preserves_area_without_overlapping_interior():
    corridor=np.array([[0.,-1.],[3.,-1.],[3.,1.],[0.,1.]])
    landing=np.array([[2.,-.8],[4.,-.8],[4.,1.2],[2.,1.2]])
    pieces=_polygon_difference(corridor,landing)
    assert sum(_area(p) for p in pieces)==pytest.approx(4.2)
    for piece in pieces:
        intersection=piece.copy()
        for a,b in zip(landing,np.roll(landing,-1,axis=0)):
            intersection=_clip_polygon(intersection,a,b)
        assert len(intersection)<3 or _area(intersection)<1e-10


def test_shared_triangle_join_has_exact_height_on_whole_common_edge():
    from d1max_pct_planner.stair_model_geometry import ParametricStairModel,SCHEMA
    template=dict(source={},segment_ids=['stair_lower'],contact_tolerance_m=0.,
        ground_replacement_band_m=.25,static_terrain_band_m=.25,
        static_stair_interior_assumed_clear=True,tread_stations_m=[],tread_heights_m=[])
    xy=np.array([[10.,20.],[11.,20.],[11.,21.],[10.,21.]])
    z=np.array([.4,.6,.6,.4])
    surfaces=[_triangle_surface(template,xy[t],z[t],f'join{i}',i+1)
        for i,t in enumerate(([0,1,2],[0,2,3]))]
    model=ParametricStairModel(dict(schema=SCHEMA,frame_id='d1max_loc_map',
        source_map_sha256='a'*64,planning_map_sha256='b'*64,
        model_points_are_observations=False,surfaces=surfaces,obstacles=[]))
    edge=xy[0]+np.linspace(0,1,31)[:,None]*(xy[2]-xy[0])
    np.testing.assert_allclose(model.ground_at(model.surfaces[0],edge),
        model.ground_at(model.surfaces[1],edge),atol=1e-12)
    for p in edge:
        assert model.surface_at(p)['surface_id']=='join0'
    assert all(s['source']['transition_semantics'].endswith('not lidar observations') for s in surfaces)


def test_transition_rejects_vertical_or_excessively_steep_plane():
    template={'source':{}}
    with pytest.raises(ValueError,match='excess_grade'):
        _triangle_surface(template,np.array([[0.,0.],[1.,0.],[1.,1.]]),
            np.array([0.,2.,2.]),'bad',1)


def test_ideal_wall_reclusters_guard_residuals_without_erasing_true_protrusion():
    from d1max_pct_planner.stair_model_geometry import IDEAL_STATIC_POLICY
    s, z = np.meshgrid(np.arange(-1.5, 1.501, .05), np.arange(.4, .81, .05))
    # Dense 2 cm inward residual links the entire measured wall. A real
    # local 15 cm protrusion must remain, but not become a 3 m long box.
    cloud = np.r_[np.c_[s.ravel(), np.full(s.size, -.68), z.ravel()],
                  np.c_[s.ravel(), np.full(s.size, .7), z.ravel()]]
    ps, pz = np.meshgrid(np.arange(-.15, .151, .05), np.arange(.5, .71, .05))
    cloud = np.r_[cloud, np.c_[ps.ravel(), np.full(ps.size, -.55), pz.ravel()]]
    # A connecting stripe makes the old p95 component span the full wall.
    cs, cz = np.meshgrid([-.65, -.60], np.arange(.5, .71, .05))
    cloud = np.r_[cloud, np.c_[np.zeros(cs.size), cs.ravel(), cz.ravel()]]
    original = cloud.copy()
    fitted = {'sides': {name: dict(lateral_slope_intercept=[0, t],
        bins=[[v, t, 10] for v in np.arange(-1.4, 1.41, .2)],
        accepted_bins=list(range(15))) for name, t in [('negative', -.7), ('positive', .7)]}}
    args = (cloud, np.arange(len(cloud)), np.array([0., 0.]), np.array([1., 0.]),
            [-1.5, 1.5], 0., fitted, 'flight', 'a'*64)
    legacy, _ = _regular_wall_strips(*args)
    actual, evidence = _regular_wall_strips(*args, static_geometry_policy=IDEAL_STATIC_POLICY)
    old_protrusions = [b for b in legacy if 'protrusion' in b['structure_id']]
    new_protrusions = [b for b in actual if 'protrusion' in b['structure_id']]
    assert max(b['half_extents_xyz'][0]*2 for b in old_protrusions) > 2.9
    assert new_protrusions
    assert max(b['half_extents_xyz'][0]*2 for b in new_protrusions) < .4
    assert any(b['pose']['position'][1]+b['half_extents_xyz'][1] > -.551
               for b in new_protrusions)
    assert [b for b in legacy if 'fitted' in b['structure_id']] == [
        b for b in actual if 'fitted' in b['structure_id']]
    assert evidence[0]['near_wall_records_assigned_to_fitted_boundary'] > 0
    np.testing.assert_array_equal(cloud, original)
