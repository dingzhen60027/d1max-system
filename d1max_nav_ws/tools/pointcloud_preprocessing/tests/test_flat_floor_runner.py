import copy
from pathlib import Path

import pytest
import yaml
import numpy as np

from pointcloud_preprocessing.flat_floor_runner import validate, structural_cleanup
from pointcloud_preprocessing.flat_floor import DEFAULTS, _config


def config():
    return yaml.safe_load((Path(__file__).parents[1]/'configs/sc_pgo_0919_flat_floor.yaml').read_text())


def test_actual_profile_has_explicit_parameters_and_scene_prior():
    cfg = config()
    assert validate(cfg) == cfg
    assert set(cfg['conditioning']) == set(DEFAULTS)
    assert _config(cfg['conditioning']) == cfg['conditioning']
    assert cfg['conditioning']['structure_above_min_m'] > cfg['conditioning']['near_ground_max_m']
    assert cfg['pct_pipeline']['pct']['ground_height'] + cfg['pct_pipeline']['pct']['slice_dh'] == pytest.approx(.15)
    assert cfg['pct_pipeline']['pct']['traversability']['safe_margin'] == .20
    assert cfg['pct_pipeline']['pct']['traversability']['inflation'] == .20
    from pointcloud_preprocessing.structural_obstacles import DEFAULTS as STRUCTURAL_DEFAULTS
    assert set(cfg['obstacle_cleanup']) == set(STRUCTURAL_DEFAULTS)
    assert cfg['obstacle_cleanup']['enabled'] is True


def test_structural_stage_maps_subset_indices_without_altering_coordinates():
    xy = np.array([[x,y] for x in np.linspace(0,.12,7) for y in np.linspace(0,.12,7)])
    floor = np.c_[xy, np.zeros(len(xy))]
    points = np.vstack([[100,100,100], floor, [.055,.055,.45]])
    original = points.copy()
    result = {'xyz': points, 'planning_mask': np.r_[False, np.ones(len(points)-1, bool)],
              'keep_mask': np.ones(len(points), bool), 'statistics': {},
              'config': {'reference_z_m': 0.}}
    indices, stage = structural_cleanup(result, {'enabled': True})
    assert indices[0] == 1 and indices[-1] == len(points)-1
    assert np.array_equal(points, original)
    assert result['keep_mask'][0] and not result['planning_mask'][0]
    assert not result['keep_mask'][-1] and not result['planning_mask'][-1]
    assert result['keep_mask'][1:-1].all() and result['planning_mask'][1:-1].all()
    assert stage['statistics']['removed_points'] == 1


def test_structural_stage_rejects_different_floor_reference():
    result = {'xyz': np.zeros((12,3)), 'planning_mask': np.ones(12,bool),
              'keep_mask': np.ones(12,bool), 'statistics': {}, 'config': {'reference_z_m': 1.}}
    with pytest.raises(ValueError, match='same floor reference'):
        structural_cleanup(result, {'enabled': False, 'floor_reference_z_m': 0.})
    assert result['keep_mask'].all()


@pytest.mark.parametrize('section,key,value', [
    ('prior', 'single_level_flat_floor_confirmed', False),
    ('prior', 'single_level_flat_floor_confirmed', 'true'),
    ('output', 'planning_only', 1), ('output', 'frame_id', 'd1max_loc_map'),
    ('output', 'directory', 'relative'), ('input', 'path', 'relative.pcd'),
    ('trajectory', 'format', 'tum'), ('output', 'maximum_removed_fraction', True),
    ('output', 'maximum_unsupported_fraction', float('nan')),
])
def test_reject_unconfirmed_or_unsafe_profile(section, key, value):
    cfg = copy.deepcopy(config())
    cfg[section][key] = value
    with pytest.raises(ValueError):
        validate(cfg)
