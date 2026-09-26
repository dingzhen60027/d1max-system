"""Independent scalar checks of official PCT kernels and bounded build artifacts."""
import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from d1max_pct_planner import cpu_tomography as cpu
from d1max_pct_planner.official_pipeline import build, preprocess, sha256, unique_output


def config():
    return {'resolution': .2, 'slice_dh': .5, 'ground_height': 'auto', 'simplify_layers': True,
            'traversability': {'kernel_size': 3, 'interval_min': .55, 'interval_free': .7,
                              'slope_max_rad': .4, 'step_max': .17, 'standable_ratio': .2,
                              'cost_barrier': 50., 'safe_margin': .4, 'inflation': .2}}


def test_cuda_point_rounding_is_not_ties_to_even():
    np.testing.assert_array_equal(cpu.round_half_away_from_zero([-2.5, -1.5, -.5, 0, .5, 1.5, 2.5]),
                                  [-3, -2, -1, 0, 1, 2, 3])


def test_actual_ground_and_ceiling_per_slice_not_fabricated_headroom():
    points = np.array([[0, 0, 0], [0, 0, .4], [0, 0, 1.3],
                       [.2, 0, -.1], [.2, 0, .8]], dtype=np.float32)
    geometry = {'shape': (7, 7), 'n_slices': 3, 'center': np.zeros(2, np.float32), 'slice_h0': .5}
    ground, ceiling = cpu.height_layers(points, geometry, .2, .5)
    np.testing.assert_allclose(ground[:, 3, 3], [.4, .4, 1.3])
    np.testing.assert_allclose(ceiling[:2, 3, 3], [1.3, 1.3])
    assert ceiling[2, 3, 3] == cpu.SENTINEL
    np.testing.assert_allclose(ground[:, 4, 3], [-.1, .8, .8])
    assert ground[:, 0, 0].tolist() == [-cpu.SENTINEL] * 3


def scalar_traversal(ground, ceiling, resolution, cfg):
    """Literal per-cell/neighbor interpretation of kernels.py, no ndimage."""
    result = np.zeros_like(ground)
    gradients, maxes = np.zeros_like(ground), np.zeros_like(ground)
    n, rows, cols = ground.shape
    for layer in range(n):
        for x in range(1, rows-1):
            for y in range(1, cols-1):
                dx = max((float(ground[layer,x,y])-ground[layer,x-1,y])**2,
                         (float(ground[layer,x,y])-ground[layer,x+1,y])**2)
                dy = max((float(ground[layer,x,y])-ground[layer,x,y-1])**2,
                         (float(ground[layer,x,y])-ground[layer,x,y+1])**2)
                gradients[layer,x,y], maxes[layer,x,y] = dx+dy, max(dx,dy)
    stand_sq = (1.2*resolution*math.tan(cfg['slope_max_rad']))**2
    cross_sq = cfg['step_max']**2
    half = cfg['kernel_size']//2
    required = int(cfg['standable_ratio']*(2*half+1)**2)-1
    for layer in range(n):
        for x in range(rows):
            for y in range(cols):
                gap = float(ceiling[layer,x,y]-ground[layer,x,y])
                if gap < cfg['interval_min']:
                    result[layer,x,y] = cfg['cost_barrier']
                    continue
                value = max(0.,20*(cfg['interval_free']-gap))
                if gradients[layer,x,y] <= stand_sq:
                    value += 15*gradients[layer,x,y]/stand_sq
                elif maxes[layer,x,y] <= cross_sq:
                    neighbors = sum(gradients[layer,u,v] < stand_sq
                                    for u in range(max(0,x-half),min(rows,x+half+1))
                                    for v in range(max(0,y-half),min(cols,y+half+1)))
                    value = (value + 20*maxes[layer,x,y]/cross_sq
                             if neighbors >= required else cfg['cost_barrier'])
                else:
                    value = cfg['cost_barrier']
                result[layer,x,y] = value
    return result


def test_slope_step_clearance_standability_and_boundaries_match_official_scalar_kernel():
    cfg = config()['traversability']
    ground = np.zeros((2, 11, 10), np.float32)
    ground[0, 3:, :] = .14  # Crossable step branch.
    ground[0, 7:, :] = .5   # Step too high.
    ground[1] = np.arange(11, dtype=np.float32)[:,None]*.025  # Gentle slope.
    ground[1, 7, 7] = -cpu.SENTINEL
    ceiling = ground + 1.2
    ceiling[:, 3, 5] = ground[:, 3, 5] + .5   # Net clearance barrier.
    ceiling[:, 4, 5] = ground[:, 4, 5] + .6   # Soft clearance cost.
    expected = scalar_traversal(ground, ceiling, .2, cfg)
    actual = cpu.traversal_cost(ground, ceiling, .2, cfg)
    np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=2e-6)
    assert actual[0,3,5] == 50
    assert 0 < actual[1,4,5] < 20
    assert actual[0,7,5] == 50


def test_radial_max_cost_inflation_matches_official_scalar_kernel():
    costs = np.zeros((11,9), np.float32)
    costs[5,4], costs[0,0], costs[3,2] = 50, 30, 14
    cfg = config()['traversability']
    r, margin, inflation = .2, cfg['safe_margin'], cfg['inflation']
    radius = int((margin+inflation)/r)
    expected = np.zeros_like(costs)
    for x in range(costs.shape[0]):
        for y in range(costs.shape[1]):
            for dx in range(-radius,radius+1):
                for dy in range(-radius,radius+1):
                    if 0 <= x+dx < costs.shape[0] and 0 <= y+dy < costs.shape[1]:
                        weight = np.float32(np.clip(1-(r*math.hypot(dx,dy)-inflation)/(margin+r),0,1))
                        expected[x,y] = max(expected[x,y],costs[x+dx,y+dy]*weight)
    np.testing.assert_allclose(cpu._inflate(costs,r,margin,inflation), expected, atol=1e-6)
    assert expected[5,4] == expected[6,4] == 50


def test_inflation_on_tiny_map_skips_far_outside_neighbors():
    values = np.array([[0, 50], [0, 0]], dtype=np.float32)
    inflated = cpu._inflate(values, .1, .4, .2)
    np.testing.assert_array_equal(inflated, np.full((2, 2), 50, dtype=np.float32))


def test_official_layer_selection_preserves_unique_surfaces_and_source_indices():
    ground, cost = np.zeros((6,5,5),np.float32), np.zeros((6,5,5),np.float32)
    ground[2:] = 1
    ground[3:] = 2
    # Unique middle source layer2 lies strictly above lower and below upper.
    np.testing.assert_array_equal(cpu.select_layers(ground,cost,50), [0,2,4])
    np.testing.assert_array_equal(cpu.select_layers(ground,cost,50,False), np.arange(6))
    np.testing.assert_array_equal(cpu.select_layers(ground[:1],cost[:1],50), [0])
    np.testing.assert_array_equal(cpu.select_layers(ground[:2],cost[:2],50), [0,1])


def plane_points():
    # Dense samples, not one point exactly on every cell boundary (the
    # official float32/round insertion is not a hole-filling operation).
    x,y=np.meshgrid(np.arange(81)*.05,np.arange(81)*.05,indexing='ij')
    return np.concatenate([np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size))),
                           np.column_stack((x.ravel(),y.ravel(),np.full(x.size,1.3)))],axis=0).astype(np.float32)


def test_complete_tensor_contains_real_ceiling_and_unknown_nan_without_floor_fill():
    payload, stats = cpu.tomogram_from_points(plane_points(), config())
    assert payload['data'].dtype == np.float32 and payload['data'].shape[0] == 5
    ground,ceiling=payload['data'][3:]
    assert np.isnan(ground).any() and np.isnan(ceiling).any()
    assert np.allclose(ceiling[np.isfinite(ceiling)],1.3)
    assert np.allclose(ground[np.isfinite(ground)],0)
    cost=payload['data'][0]
    assert ((cost<=20)&np.isfinite(ground)).any()
    assert stats['selected_source_layers'] == [0,1]
    assert payload['minimum_headroom_m'] == .55
    np.testing.assert_allclose(payload['data'][1,:,1:-1], cost[:,2:]-cost[:,:-2])


def test_allocation_limits_fail_before_mapping_arrays(monkeypatch):
    cfg=config(); cfg['limits']={'max_total_cells':5}
    monkeypatch.setattr(cpu,'height_layers',lambda *_: pytest.fail('allocated before checking limits'))
    with pytest.raises(ValueError,match='resource limits'):
        cpu.tomogram_from_points(plane_points(),cfg)


@pytest.mark.parametrize('key,value',[('resolution',0),('slice_dh',-1),('ground_height',float('inf'))])
def test_invalid_top_level_parameters(key,value):
    cfg=config();cfg[key]=value
    with pytest.raises(ValueError): cpu.validate_config(cfg)


@pytest.mark.parametrize('key,value',[('kernel_size',4),('cost_barrier',20),('interval_min',1.0),
                                    ('slope_max_rad',0),('standable_ratio',1.2),('inflation',-.1)])
def test_invalid_traversability_parameters(key,value):
    cfg=config();cfg['traversability'][key]=value
    with pytest.raises(ValueError): cpu.validate_config(cfg)


def test_default_preprocessing_only_removes_nonfinite_without_moving_points():
    original=np.array([[1,2,3],[2,4,6],[0,float('nan'),1]],np.float32)
    points,stats=preprocess(original,{})
    np.testing.assert_array_equal(points,original[:2])
    assert stats[0]['points']==3 and stats[-1]['points']==2
    assert not any(item.get('enabled',False) for item in stats)


def test_unique_output_does_not_overwrite_existing_map(tmp_path):
    a=unique_output(tmp_path/'map'); sentinel=a/'keep'
    sentinel.write_text('user data')
    b=unique_output(tmp_path/'map')
    assert a!=b and b.name=='map_001' and sentinel.read_text()=='user data'


def test_one_yaml_build_produces_safe_npz_official_pickle_manifest_and_immutable_source(tmp_path):
    import open3d as o3d
    source=tmp_path/'input.pcd'
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(plane_points()))
    assert o3d.io.write_point_cloud(str(source),cloud)
    original_hash=sha256(source)
    cfg={'schema_version':1,'source_pcd':str(source),'output_directory':str(tmp_path/'result'),
         'vendor_root':'/home/dndx/d1max_nav_ws/src/pct_planner_vendor',
         'frame_id':'d1max_loc_map','pct':config(),'export':{'traversable_pcd':True}}
    path=tmp_path/'config.yaml';path.write_text(yaml.safe_dump(cfg))
    output,manifest=build(path)
    assert sha256(source)==original_hash and manifest['source_unchanged']
    assert manifest['source_points']==len(plane_points()) and manifest['used_points']==len(plane_points())
    assert manifest['hardware']['cuda_executed_for_build'] is False
    assert (output/'tomogram.pickle').is_file() and (output/'traversable.pcd').is_file()
    with np.load(output/'tomogram.npz',allow_pickle=False) as data:
        assert all(data[k].dtype.kind!='O' for k in data.files)
        assert data['data'].shape[0]==5
        assert str(data['backend'])==cpu.BACKEND
        assert float(data['minimum_headroom_m'])==.55
    saved=json.loads((output/'manifest.json').read_text())
    assert saved['config_sha256']==sha256(path)
    assert saved['files']['tomogram.npz']['sha256']==sha256(output/'tomogram.npz')
