import copy

import numpy as np
import pytest

from d1max_pct_planner.cpu_tomography import traversal_cost, _inflate
from d1max_pct_planner.stair_model_geometry import (SCHEMA, ParametricStairModel,
                                                   apply_parametric_stair_geometry)


def model():
    return dict(schema=SCHEMA, source_map_sha256='a'*64, planning_map_sha256='b'*64,
        frame_id='d1max_loc_map', model_points_are_observations=False,
        surfaces=[dict(surface_id='flight', source_surface_id=1, segment_ids=['stair_lower'],
            kind='continuous_support_envelope', frame=dict(origin_xyz=[0,0,0],
                axis_u=[1,0,0], axis_v=[0,1,0], axis_up=[0,0,1]),
            polygon_xy=[[-1,-.7],[1,-.7],[1,.7],[-1,.7]], support_grade=.5,
            support_intercept_m=1., contact_tolerance_m=0.,
            ground_replacement_band_m=.25, static_terrain_band_m=.25,
            static_stair_interior_assumed_clear=True,
            tread_stations_m=[], tread_heights_m=[])], obstacles=[])


def fixture():
    cfg = dict(traversability=dict(kernel_size=7, interval_min=.55, interval_free=.7,
        slope_max_rad=.6, step_max=.17, standable_ratio=.2, cost_barrier=50.,
        safe_margin=.1, inflation=.1))
    x,y = np.meshgrid(np.arange(-2,2.01,.1), np.arange(-1.5,1.51,.1), indexing='ij')
    ground = (1+.5*x)[None].astype(np.float32)
    # A finite static map height ripple, not a modified source observation.
    ground[:,10:30,8:22] += (.045*np.sin(x[10:30,8:22]*35)).astype(np.float32)
    ceiling = np.full_like(ground, 10.)
    raw = traversal_cost(ground, ceiling, .1, cfg['traversability'])
    cost = np.asarray([_inflate(raw[0], .1,.1,.1)])
    gx,gy = np.zeros_like(cost),np.zeros_like(cost)
    gx[:,1:-1] = cost[:,2:]-cost[:,:-2]; gy[:,:,1:-1] = cost[:,:,2:]-cost[:,:,:-2]
    payload = dict(data=np.stack([cost,gx,gy,ground,ceiling]), resolution=.1,
        center=np.array([0.,0.]), slice_h0=3., slice_dh=.5, source_sha256='b'*64)
    xyz = np.c_[x.ravel(), y.ravel(), (1+.5*x).ravel()]
    return payload,cfg,xyz


def test_loader_surface_explicit_frame_and_kind():
    m=ParametricStairModel(model())
    s=m.surface_at([.2,0],source_surface_id=1,height_hint=1.1)
    assert s['surface_id']=='flight'
    assert m.ground_at(s,[.2,0])==pytest.approx(1.1)
    assert m.surface_at([1.1,0]) is None
    assert m.surface_at([0,0],source_surface_id=3) is None
    assert m.surface_at([0,0],height_hint=4.) is None


@pytest.mark.parametrize('mutation',[lambda m:m.update(source_map_sha256='bad'),
    lambda m:m.update(model_points_are_observations=True),
    lambda m:m['surfaces'][0].update(segment_ids=[]),
    lambda m:m['surfaces'][0].update(tread_stations_m=[0,1]),
    lambda m:m['surfaces'][0].update(polygon_xy=[[-1,-1],[1,1],[1,-1],[-1,1]]),
    lambda m:m['surfaces'][0]['frame'].update(axis_up=[0,1,0])])
def test_invalid_or_ambiguous_model_rejected(mutation):
    value=model();mutation(value)
    with pytest.raises(ValueError): ParametricStairModel(value)


def test_ground_changes_not_only_cost_with_exact_outside_halo_and_ceiling():
    payload,cfg,xyz=fixture(); old=payload['data'].copy()
    result=apply_parametric_stair_geometry(payload,cfg,model(),xyz)
    assert result['diagnostics']['changed_ground_cells']>0
    assert result['diagnostics']['changed_cost_cells']>0
    mask=result['arrays']['modeled_ground']; halo=result['arrays']['derivative_halo']
    np.testing.assert_array_equal(result['data'][:,~(mask|halo)],old[:,~(mask|halo)])
    np.testing.assert_array_equal(result['data'][4],old[4])
    np.testing.assert_array_equal(payload['data'],old)
    x,y=np.meshgrid(np.arange(-2,2.01,.1),np.arange(-1.5,1.51,.1),indexing='ij')
    np.testing.assert_allclose(result['data'][3][mask],(1+.5*x)[None][mask],atol=1e-6)


def test_other_floor_unknown_low_ceiling_and_real_entity_are_not_replaced():
    payload,cfg,xyz=fixture()
    payload['data'][3,0,20,15]=np.nan
    payload['data'][3,0,21,15]=4.
    payload['data'][4,0,22,15]=1.5
    xyz=np.r_[xyz,[[.3,0,1.65]]]
    old=payload['data'].copy()
    result=apply_parametric_stair_geometry(payload,cfg,model(),xyz)
    for x in (20,21,22,23):
        assert not result['arrays']['modeled_ground'][0,x,15]
        np.testing.assert_array_equal(result['data'][[0,3,4],0,x,15],old[[0,3,4],0,x,15])


def test_finite_measured_wall_not_erased_and_no_entrance_cap():
    payload,cfg,xyz=fixture(); value=model()
    value['obstacles']=[dict(structure_id='wall',pose=dict(position=[0,.6,1.3],
        orientation_xyzw=[0,0,0,1]),half_extents_xyz=[.8,.04,.8],source={'kind':'measured'})]
    result=apply_parametric_stair_geometry(payload,cfg,value,xyz)
    assert not result['arrays']['modeled_ground'][0,20,21]
    assert result['arrays']['modeled_ground'][0,20,15]
    assert result['arrays']['modeled_ground'][0,11,15]


def test_hash_mismatch_rejected_before_geometry():
    p,c,x=fixture();p['source_sha256']='c'*64
    with pytest.raises(ValueError,match='identity'):apply_parametric_stair_geometry(p,c,model(),x)


def test_unimplemented_discrete_step_contract_is_not_silently_a_ramp():
    value=model()
    value['surfaces'][0].update(kind='standard_steps',tread_stations_m=[-1.,0.,1.],
        tread_heights_m=[.5,1.5])
    with pytest.raises(ValueError,match='surface_identity'):ParametricStairModel(value)


def test_finite_wall_uses_original_continuous_inflation_once():
    p,c,x=fixture();v=model()
    v['obstacles']=[dict(structure_id='wall',pose=dict(position=[0,.6,1.3],
        orientation_xyzw=[0,0,0,1]),half_extents_xyz=[.8,.04,.8],source={'kind':'test_measurement'})]
    m=ParametricStairModel(v)
    np.testing.assert_allclose(m.obstacle_distance(np.array([[0,.3],[0,.6],[1.,.6]]),
        np.array([1.,1.,1.])),[.26,0.,.2],atol=1e-12)
    result=apply_parametric_stair_geometry(p,c,v,x)
    # Exact 26 cm from inner face: original kernel gives cost 10. There is
    # no added half-cell raster margin (which would incorrectly make 25).
    assert result['data'][0,0,20,18]==pytest.approx(10.,abs=1e-4)
    assert result['data'][0,0,20,21]==50.
    assert result['data'][0,0,20,18]<=20


@pytest.mark.parametrize('duplicate',[False,True])
def test_model_wall_identity_matches_native_contract(duplicate):
    value=model();box=dict(structure_id='wall' if duplicate else '',
        pose=dict(position=[0,.6,1.3],orientation_xyzw=[0,0,0,1]),
        half_extents_xyz=[.8,.04,.8],source={'kind':'test_measurement'})
    value['obstacles']=[box,copy.deepcopy(box)] if duplicate else [box]
    with pytest.raises(ValueError,match='obstacle_identity'):ParametricStairModel(value)


def test_shared_equal_height_edge_only_has_stable_owner():
    value=model(); second=copy.deepcopy(value['surfaces'][0])
    second.update(surface_id='second',source_surface_id=2,polygon_xy=[[1,-.7],[2,-.7],[2,.7],[1,.7]])
    value['surfaces'].append(second)
    m=ParametricStairModel(value)
    assert m.surface_at([1.,0])['surface_id']=='flight'
    second['support_intercept_m']+=.1
    with pytest.raises(ValueError,match='ambiguous'):ParametricStairModel(value).surface_at([1.,0])
    second['support_intercept_m']-=.1
    second['polygon_xy'][0][0]=.9;second['polygon_xy'][3][0]=.9
    with pytest.raises(ValueError,match='ambiguous'):ParametricStairModel(value).surface_at([.95,0])


def test_old_tread_ceiling_reclassified_but_real_overhead_and_other_floor_remain():
    payload,cfg,xyz=fixture()
    payload['data'][4,0,20,15]=1.1 # original adjacent tread in the same static prior band
    xyz=np.r_[xyz,[[0,0,1.1],[0,0,2.2]]]
    old=payload['data'].copy()
    result=apply_parametric_stair_geometry(payload,cfg,model(),xyz)
    assert result['arrays']['terrain_ceiling_reclassified'][0,20,15]
    assert result['data'][4,0,20,15]==pytest.approx(2.2)
    assert result['data'][3,0,20,15]==pytest.approx(1.)
    assert not result['arrays']['terrain_ceiling_reclassified'][0,19,15]
    assert result['data'][4,0,19,15]==old[4,0,19,15]


def test_actual_low_overhead_above_static_band_still_blocks():
    payload,cfg,xyz=fixture()
    payload['data'][4,0,20,15]=1.1
    xyz=np.r_[xyz,[[0,0,1.1],[0,0,1.4]]]
    result=apply_parametric_stair_geometry(payload,cfg,model(),xyz)
    assert not result['arrays']['modeled_ground'][0,20,15]
    assert result['data'][0,0,20,15]>20


def test_pipeline_without_explicit_model_is_exact_noop(tmp_path):
    from d1max_pct_planner.official_pipeline import apply_parametric_geometry
    payload,config,_=fixture()
    for processing in (None, {}, {'unrelated_stage': True}):
        actual,diagnostics=apply_parametric_geometry(payload, {'pct':config}, processing, tmp_path)
        assert actual is payload
        assert diagnostics is None
    assert not list(tmp_path.iterdir())


def test_pipeline_reads_full_retained_cloud_and_seals_both_stages(tmp_path, monkeypatch):
    import hashlib
    import json
    import sys
    from types import ModuleType, SimpleNamespace
    from d1max_pct_planner.official_pipeline import apply_parametric_geometry, sha256
    payload,config,xyz=fixture()
    payload['data'][4,0,20,15]=1.1
    # The true overhead exists in the retained input, not a preprocessing
    # subset. The production hook must consume that exact named file.
    xyz=np.r_[xyz,[[0,0,1.1],[0,0,2.2]]]
    expected=apply_parametric_stair_geometry(payload,config,model(),xyz)
    model_path=tmp_path/'model.json'
    model_path.write_text(json.dumps(model()))
    calls=[]
    io=ModuleType('d1max_pct_scan.pointcloud_helpers.pcd_io')
    def read_pcd(path):
        calls.append(path)
        return SimpleNamespace(xyz=xyz)
    io.read_pcd=read_pcd
    monkeypatch.setitem(sys.modules, io.__name__, io)
    retained=tmp_path/'retained.pcd'
    processing=dict(manifest_path=str(tmp_path/'manifest.json'),output_path=str(retained),
        parametric_stair_geometry=dict(model_file=model_path.name,model_sha256=sha256(model_path)))
    original=payload['data'].copy()
    actual,diagnostics=apply_parametric_geometry(payload,{'pct':config},processing,tmp_path)
    assert calls==[str(retained)]
    np.testing.assert_array_equal(actual['data'],expected['data'])
    np.testing.assert_array_equal(payload['data'],original)
    with np.load(tmp_path/'parametric_geometry_base.npz',allow_pickle=False) as archive:
        np.testing.assert_array_equal(archive['data'],original)
    with np.load(tmp_path/'parametric_geometry_evidence.npz',allow_pickle=False) as archive:
        for name,value in expected['arrays'].items():
            np.testing.assert_array_equal(archive[name],value)
    assert actual['parametric_stair_model_sha256']==sha256(model_path)
    assert actual['parametric_stair_base_sha256']==sha256(tmp_path/'parametric_geometry_base.npz')
    assert actual['parametric_stair_evidence_sha256']==sha256(tmp_path/'parametric_geometry_evidence.npz')
    assert actual['parametric_stair_data_sha256']==hashlib.sha256(actual['data'].tobytes()).hexdigest()
    assert diagnostics['original_points_unchanged'] is True
    assert diagnostics['model_points_are_observations'] is False


def ideal_model():
    from d1max_pct_planner.stair_model_geometry import IDEAL_STATIC_POLICY
    value = model()
    value['static_geometry_policy'] = IDEAL_STATIC_POLICY
    return value


def test_explicit_ideal_prior_does_not_reinflate_outside_raw_terrain_into_stairs():
    payload, cfg, xyz = fixture()
    payload['data'][3, 0, 9, 15] += .6  # old terrain seed, outside finite model
    original = payload['data'].copy()
    legacy = apply_parametric_stair_geometry(payload, cfg, model(), xyz)
    actual = apply_parametric_stair_geometry(payload, cfg, ideal_model(), xyz)
    assert legacy['data'][0, 0, 11, 15] > 20
    assert actual['data'][0, 0, 11, 15] < 20
    outside = ~actual['arrays']['model_domain']
    for channel in (0, 3, 4):
        np.testing.assert_array_equal(actual['data'][channel][outside], original[channel][outside])
    np.testing.assert_array_equal(payload['data'], original)


@pytest.mark.parametrize('kind', ['entity', 'overhead', 'wall'])
def test_ideal_prior_preserves_real_entity_low_overhead_and_fitted_structure(kind):
    payload, cfg, xyz = fixture()
    value = ideal_model()
    if kind == 'entity':
        xyz = np.r_[xyz, [[0., 0., 1.4]]]
    elif kind == 'overhead':
        payload['data'][4, 0, 20, 15] = 1.4
    else:
        value['obstacles'] = [dict(structure_id='real_handrail',
            pose=dict(position=[0, 0, 1.3], orientation_xyzw=[0, 0, 0, 1]),
            half_extents_xyz=[.1, .04, .3], source={'kind': 'measured'})]
    actual = apply_parametric_stair_geometry(payload, cfg, value, xyz)
    assert not actual['arrays']['modeled_ground'][0, 20, 15]
    # Neighboring cells still receive real obstacle/clearance inflation.
    assert actual['data'][0, 0, 20, 16] > 20


def test_ideal_prior_still_does_not_invent_unknown_or_other_floor_support():
    payload, cfg, xyz = fixture()
    payload['data'][3, 0, 20, 15] = np.nan
    payload['data'][3, 0, 21, 15] = 4.
    original = payload['data'].copy()
    actual = apply_parametric_stair_geometry(payload, cfg, ideal_model(), xyz)
    for ix in (20, 21):
        assert not actual['arrays']['modeled_ground'][0, ix, 15]
        np.testing.assert_array_equal(actual['data'][[0, 3, 4], 0, ix, 15],
                                      original[[0, 3, 4], 0, ix, 15])


def test_unknown_ideal_policy_is_rejected_not_silently_enabled():
    value = model()
    value['static_geometry_policy'] = 'unbounded_clear_stairs'
    with pytest.raises(ValueError, match='policy'):
        ParametricStairModel(value)


@pytest.mark.parametrize('kind', ['entity', 'overhead'])
def test_real_obstacle_outside_model_still_inflates_inward(kind):
    payload, cfg, xyz = fixture()
    # x=-1.1 is outside the finite [-1, 1] model, in the original PCT layer.
    if kind == 'entity':
        xyz = np.r_[xyz, [[-1.1, 0., .85]]]
    else:
        payload['data'][4, 0, 9, 15] = .85
    original = payload['data'].copy()
    actual = apply_parametric_stair_geometry(payload, cfg, ideal_model(), xyz)
    assert actual['data'][0, 0, 11, 15] > 20
    np.testing.assert_array_equal(actual['data'][[0, 3, 4], 0, 9, 15],
                                  original[[0, 3, 4], 0, 9, 15])
