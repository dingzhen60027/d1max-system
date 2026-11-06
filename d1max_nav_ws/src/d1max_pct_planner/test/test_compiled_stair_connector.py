"""A compiled planner polyline is current-map data, never a stale motion proof."""
from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from d1max_pct_planner import crossfloor_preview as preview
from d1max_pct_planner.compiled_stair_connector import (
    MAX_POINTS, PROOF, SCHEMA, load_compiled_stair_connector, map_settings_sha256,
)
from d1max_pct_planner.crossfloor_route import plan_crossfloor, validate_config
from d1max_pct_planner.tomogram_map import TomogramMap
from test_crossfloor_route import config, paths, proven_route_factory, scene, xyz


@pytest.fixture
def compiled(tmp_path):
    source = tmp_path/'map.pcd'
    source.write_bytes(b'current measured planning PCD fixture')
    payload = scene(source)
    payload['original_source_sha256'] = hashlib.sha256(b'original source').hexdigest()
    map_file = tmp_path/'map.npz'
    np.savez_compressed(map_file, **payload)
    raw = config()
    raw.update(tomogram_path=str(map_file), source_pcd=str(source), vendor_root=str(tmp_path))
    settings = validate_config(raw)
    grid = TomogramMap(map_file, unknown_ceiling_policy=settings['unknown_ceiling_policy'],
                       max_ground_step_m=settings['limits']['max_ground_step_m'])
    lower, upper = paths()['stair_lower'], paths()['stair_upper']
    value = dict(schema=SCHEMA, tomogram_sha256=grid.sha256,
        source_pcd_sha256=grid.provenance['source_sha256'],
        original_source_sha256=grid.provenance['original_source_sha256'],
        frame_id=settings['frame_id'], map_settings_sha256=map_settings_sha256(settings),
        minimum_headroom_m=grid.minimum_headroom_m, source_layer_ids=grid.source_layers.tolist(),
        portals={key:raw['anchors'][key] for key in ('entry', 'landing', 'exit')},
        path=lower[0]+upper[0][1:], layer_ids=lower[1]+upper[1][1:], landing_index=len(lower[0])-1,
        source_route_sha256='1'*64, origin_tomogram_sha256='2'*64)
    bundle_file = tmp_path/'connector.json'
    cfg_file = tmp_path/'route.yaml'
    def write(new_value=None):
        bundle_file.write_text(json.dumps(value if new_value is None else new_value))
        raw['compiled_stair_connector'] = dict(file=str(bundle_file),
            sha256=hashlib.sha256(bundle_file.read_bytes()).hexdigest())
        cfg_file.write_text(yaml.safe_dump(raw))
        return validate_config(raw)
    write()
    return SimpleNamespace(raw=raw, grid=grid, value=value, write=write, path=cfg_file,
                           bundle=bundle_file, source=source, map_payload=payload)


def test_current_polyline_has_its_own_proof_and_not_quintic_or_motion_authority(compiled):
    result = load_compiled_stair_connector(compiled.grid, compiled.write())
    assert result['diagnostics']['curve_validation'] == PROOF
    assert result['diagnostics']['physical_safety_certified'] is False
    for name, raw in result['legs'].items():
        assert raw['quintic_segments'] == 0
        assert raw['curve_validation'] == PROOF
        assert raw['checks']
        assert np.array_equal(raw['path'], paths()[name][0])
    assert result['dependency_files'][str(compiled.bundle)] == compiled.raw['compiled_stair_connector']['sha256']


@pytest.mark.parametrize('key,value', [
    ('tomogram_sha256','0'*64), ('source_pcd_sha256','0'*64),
    ('original_source_sha256','0'*64), ('map_settings_sha256','0'*64),
    ('frame_id','other_map'), ('minimum_headroom_m',.123),
    ('source_layer_ids',[0,7]), ('landing_index',0), ('schema','old_schema')])
def test_foreign_identity_and_bad_geometry_contract_are_not_silent_fallback(compiled,key,value):
    changed = deepcopy(compiled.value)
    changed[key] = value
    with pytest.raises(ValueError):
        load_compiled_stair_connector(compiled.grid, compiled.write(changed))


def test_current_pcd_bytes_and_literal_bundle_hash_are_checked(compiled):
    settings = compiled.write()
    compiled.bundle.write_text(compiled.bundle.read_text()+' ')
    with pytest.raises(ValueError, match='hash_mismatch'):
        load_compiled_stair_connector(compiled.grid, settings)
    settings = compiled.write()
    compiled.source.write_bytes(b'changed planning map')
    with pytest.raises(ValueError):
        load_compiled_stair_connector(compiled.grid, settings)


@pytest.mark.parametrize('mutation', ['obstacle', 'missing_support', 'low_ceiling'])
def test_bundle_is_rechecked_against_current_geometry_even_with_rebound_hash(compiled,mutation):
    payload = deepcopy(compiled.map_payload)
    if mutation == 'obstacle':
        payload['data'][0,0,12,1] = 50.
    elif mutation == 'missing_support':
        payload['data'][3,0,12,1] = np.nan
    else:
        payload['data'][4,0,12,1] = payload['data'][3,0,12,1]+.1
    np.savez_compressed(compiled.raw['tomogram_path'], **payload)
    grid = TomogramMap(compiled.raw['tomogram_path'], max_ground_step_m=.17)
    changed = deepcopy(compiled.value)
    changed['tomogram_sha256'] = grid.sha256
    with pytest.raises(ValueError):
        load_compiled_stair_connector(grid, compiled.write(changed))


def test_bounded_points_exact_portals_and_integer_layers(compiled):
    for key, value in [('path', [compiled.value['path'][0]]*(MAX_POINTS+1)),
                       ('layer_ids',[float(v) for v in compiled.value['layer_ids']]),
                       ('landing_index',True)]:
        changed = deepcopy(compiled.value)
        changed[key] = value
        with pytest.raises(ValueError):
            load_compiled_stair_connector(compiled.grid, compiled.write(changed))
    changed = deepcopy(compiled.value)
    changed['path'][0][0] += .001
    with pytest.raises(ValueError,match='exact_endpoint'):
        load_compiled_stair_connector(compiled.grid, compiled.write(changed))


def test_compiled_cache_is_copied_and_reloads_after_cancel_or_unrelated_failure(compiled):
    settings = compiled.write()
    cache = preview._FixedStairCache(compiled.grid,settings)
    begin,end = settings['anchors']['entry'],settings['anchors']['landing']
    first = cache.get(compiled.grid,'stair_lower',begin,end,settings)
    first['path'][3][1] = 999.
    cache.clear()
    next_value = cache.get(compiled.grid,'stair_lower',begin,end,settings)
    assert next_value['path'] == paths()['stair_lower'][0]
    assert next_value['curve_validation'] == PROOF
    foreign = deepcopy(settings)
    foreign['planning']['max_iterations'] = 1
    with pytest.raises(ValueError,match='original map/configuration'):
        cache.get(compiled.grid,'stair_lower',begin,end,foreign)
    alien = deepcopy(begin)
    alien['xyz'] = alien['xyz']+np.array([.001,0,0])
    with pytest.raises(ValueError,match='exact configured portal'):
        cache.get(compiled.grid,'stair_lower',alien,end,settings)


def install_native(compiled, monkeypatch):
    calls=[]
    def native(grid,name,settings,**_):
        class Native:
            native_runtime={}
            native_parameters={}
            def plan(self,start,goal,a,b):
                calls.append((name,list(start),list(goal)))
                first,last = grid.index(np.asarray(start)[:2])[0],grid.index(np.asarray(goal)[:2])[0]
                step=1 if last>=first else -1
                points=[xyz(i) for i in range(first,last+step,step)]
                layers=[a if i<=19 else b for i in range(first,last+step,step)] if a!=b else [a]*len(points)
                return dict(path=points,layer_ids=layers,algorithm='native fixture',
                    curve_validation='quintic_cell_boundary_roots_and_interval_interiors',
                    quintic_segments=2,curve_partition_points=len(points))
        return Native()
    monkeypatch.setattr(preview,'_native_route',native)
    return calls


def test_up_down_clear_flat_and_partial_keep_their_respective_paths(compiled,monkeypatch):
    calls=install_native(compiled,monkeypatch)
    route=preview.CrossfloorPreviewRoute(compiled.grid,compiled.path)
    up=route.plan(xyz(1),xyz(39),0,1)
    assert [name for name,*_ in calls] == ['lower_floor','upper_floor']
    assert up['stair_refinement']['curve_validation'] == PROOF
    down=route.plan(xyz(39),xyz(1),1,0)
    assert down['path'] == up['path'][::-1]
    assert down['stair_refinement']['output_direction'] == 'upper_to_lower'
    assert down['stair_refinement']['landing_index'] == len(compiled.value['path'])-1-compiled.value['landing_index']
    with pytest.raises(ValueError):
        route.plan([999,999,999],xyz(39),0,1)
    assert not route._fixed_stairs._results
    again=route.plan(xyz(1),xyz(39),0,1)
    assert again['path'] == up['path']
    calls.clear()
    same=route.plan(xyz(1),xyz(4),0,0)
    assert same['route_type'] == 'same_floor' and [v[0] for v in calls] == ['lower_floor']
    calls.clear()
    partial=route.plan(xyz(15),xyz(39),0,1)
    assert partial['route_type'] == 'partial_stair'
    assert any(v[0].startswith('stair_') for v in calls)


def test_absent_bundle_keeps_native_queries_and_native_cache_proof_rules(compiled,monkeypatch):
    raw=deepcopy(compiled.raw)
    del raw['compiled_stair_connector']
    settings=validate_config(raw)
    assert load_compiled_stair_connector(compiled.grid,settings) is None
    cache=preview._FixedStairCache(compiled.grid,settings)
    begin,end=settings['anchors']['entry'],settings['anchors']['landing']
    cache.put(compiled.grid,'stair_lower',begin,end,settings,
              dict(curve_validation=PROOF,quintic_segments=0,curve_partition_points=100))
    assert cache.get(compiled.grid,'stair_lower',begin,end,settings) is None
    raw['planning']['path_refinement']='none'
    queried=[]
    factory=proven_route_factory(paths())
    def counted(grid,name,cfg):
        queried.append(name)
        return factory(grid,name,cfg)
    result=plan_crossfloor(raw,compiled.grid,counted)
    assert queried == ['lower_floor','stair_lower','stair_upper','upper_floor']
    assert all(s['curve_validation'] != PROOF for s in result['segments'])


def test_restore_requires_literal_current_compiled_geometry_not_proof_string(compiled,tmp_path):
    result=plan_crossfloor(compiled.raw,compiled.grid,proven_route_factory(paths()))
    result['config_sha256']=hashlib.sha256(compiled.path.read_bytes()).hexdigest()
    audit=tmp_path/'audit.json'
    audit.write_text(json.dumps(result))
    restored=preview.restore_crossfloor(compiled.grid,compiled.path,audit)
    assert restored['path'] == result['path_xyz']
    # Still inside the same admissible cell; geometry validation alone is not
    # permission to impersonate the separately compiled literal connector.
    altered=deepcopy(result)
    altered['path_xyz'][7][0] += .001
    audit.write_text(json.dumps(altered))
    with pytest.raises(ValueError,match='current-map bundle'):
        preview.restore_crossfloor(compiled.grid,compiled.path,audit)
    altered=deepcopy(result)
    altered['segments'][2]['curve_validation']='quintic_cell_boundary_roots_and_interval_interiors'
    altered['segments'][2]['quintic_segments']=2
    audit.write_text(json.dumps(altered))
    with pytest.raises(ValueError,match='both exact'):
        preview.restore_crossfloor(compiled.grid,compiled.path,audit)


@pytest.fixture
def model_derivation(tmp_path):
    from d1max_pct_planner.compiled_stair_connector import (
        MODEL_DERIVATION_SCHEMA,derive_model_ground,polyline_sha256)
    from d1max_pct_planner.stair_model_geometry import ParametricStairModel,SCHEMA as MODEL_SCHEMA
    model=dict(schema=MODEL_SCHEMA,frame_id='d1max_loc_map',source_map_sha256='a'*64,
        planning_map_sha256='b'*64,model_points_are_observations=False,surfaces=[dict(
            surface_id='flight',source_surface_id=1,segment_ids=['stair_lower'],kind='continuous_support_envelope',
            frame=dict(origin_xyz=[0.,0.,0.],axis_u=[1.,0.,0.],axis_v=[0.,1.,0.],axis_up=[0.,0.,1.]),
            polygon_xy=[[0.,-.7],[2.,-.7],[2.,.7],[0.,.7]],support_grade=.1,support_intercept_m=.02,
            contact_tolerance_m=0.,ground_replacement_band_m=.25,static_terrain_band_m=.25,
            static_stair_interior_assumed_clear=True,tread_stations_m=[],tread_heights_m=[])],obstacles=[])
    points=[[-1.,0.,0.],[0.,0.,0.],[1.,0.,.1],[2.,0.,.2],[3.,0.,.3]];layers=[0]*5
    base=dict(schema=SCHEMA,path=points,layer_ids=layers,source_pcd_sha256='b'*64,
        original_source_sha256='a'*64,frame_id='planning',source_route_sha256='c'*64,
        origin_tomogram_sha256='d'*64,landing_index=2,source_layer_ids=[0],origin={'route_file':'native.json'})
    mp=tmp_path/'model.json';mp.write_text(json.dumps(model));mh=hashlib.sha256(mp.read_bytes()).hexdigest()
    bp=tmp_path/'base.json';bp.write_text(json.dumps(base));bh=hashlib.sha256(bp.read_bytes()).hexdigest()
    new=derive_model_ground(points,ParametricStairModel(model));payload=deepcopy(base);payload['path']=new.tolist()
    payload['origin']['parametric_geometry']=dict(schema=MODEL_DERIVATION_SCHEMA,
        base_file=bp.name,base_sha256=bh,model_file=mp.name,model_sha256=mh,
        input_geometry_sha256=polyline_sha256(points,layers),output_geometry_sha256=polyline_sha256(new,layers),
        changed_vertices=[1,2,3])
    grid=SimpleNamespace(provenance={'parametric_stair_model_sha256':mh})
    return payload,grid,tmp_path,base


def test_model_z_derivative_is_explicit_and_keeps_native_xy_layers_and_flat(model_derivation):
    from d1max_pct_planner.compiled_stair_connector import _model_derivation
    payload,grid,directory,base=model_derivation
    original,dependencies=_model_derivation(payload,grid,directory)
    assert original['path']==base['path']
    np.testing.assert_array_equal(np.asarray(payload['path'])[:,:2],np.asarray(base['path'])[:,:2])
    assert payload['path'][0]==base['path'][0] and payload['path'][-1]==base['path'][-1]
    assert len(dependencies)==2


@pytest.mark.parametrize('mutation',['xy','z','layer','source','model','base_path','changed','old_origin'])
def test_model_z_derivative_cannot_impersonate_new_geometry_or_drop_lineage(model_derivation,mutation):
    from d1max_pct_planner.compiled_stair_connector import _model_derivation,polyline_sha256
    payload,grid,directory,base=model_derivation;value=deepcopy(payload)
    if mutation=='xy':value['path'][2][0]+=.01
    if mutation=='z':value['path'][2][2]+=.01
    if mutation=='layer':value['layer_ids'][2]=1
    if mutation=='source':value['original_source_sha256']='f'*64
    if mutation=='model':grid.provenance['parametric_stair_model_sha256']='f'*64
    if mutation=='base_path':value['origin']['parametric_geometry']['base_file']='../base.json'
    if mutation=='changed':value['origin']['parametric_geometry']['changed_vertices']=[]
    if mutation=='old_origin':value['origin']['route_file']='different.json'
    # A self-consistent new path hash does not replace source/model derivation.
    value['origin']['parametric_geometry']['output_geometry_sha256']=polyline_sha256(value['path'],value['layer_ids'])
    with pytest.raises(ValueError):_model_derivation(value,grid,directory)
