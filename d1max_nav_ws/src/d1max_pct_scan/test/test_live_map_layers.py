"""Offline display contracts; no ROS nodes, sensors, planner or controls."""
from pathlib import Path
from types import SimpleNamespace
import math
import numpy as np
import pytest
import yaml
from d1max_pct_scan.live_goal_editor import goal_position
from d1max_pct_scan.live_map_geometry import (map_surfaces, colored_records, build_layers,
                                             initial_goal_from_candidates, validate_map_binding)
from tools.pointcloud_preprocessing.ground_path_bridge import GroundPathBridge, FloorField


def test_separately_valid_same_frame_maps_cannot_be_mixed(tmp_path):
    bridge = SimpleNamespace(planning_frame='planning', source_frame='original')
    manifest = tmp_path/'version_a'/'manifest.json'
    settings = dict(frame_id='planning', source_pcd=str(manifest.parent/'processed_map.pcd'))
    validate_map_binding(manifest, bridge, settings, source_frame='original')
    wrong_version = {**settings, 'source_pcd': str(tmp_path/'version_b'/'processed_map.pcd')}
    for candidate, source_frame in ((wrong_version, 'original'),
            ({**settings, 'frame_id': 'different'}, 'original'), (settings, 'different')):
        with pytest.raises(ValueError, match='PCT source/frame'):
            validate_map_binding(manifest, bridge, candidate, source_frame=source_frame)


def test_independent_surfaces_use_bounded_real_inverse_and_omit_unsupported_cells():
    anchors = {'xy': np.array([[0., 0.], [1., 0.], [0., 1.], [1., 1.]]),
               'z': np.full(4, -.75), 'slopes': np.zeros((4, 2))}
    one = FloorField('floor1', -.65, (-2., 2.3), anchors, anchors['xy'],
                     {'conditioning_support_radius_m': .5})
    two = FloorField('floor2', 3.15, (2.7, 7.), {**anchors, 'z': np.full(4, 3.05)},
                     anchors['xy'], {'conditioning_support_radius_m': .5})
    bridge = GroundPathBridge({'floor1': one, 'floor2': two},
                             [{'min': [.4, .4, -2.], 'max': [.6, .6, 7.]}])
    points = np.array([[.1, .1, -.65], [.1, .1, 3.15], [.5, .5, 1.5], [20., 20., -.65]])
    mapped, ids = map_surfaces(points, bridge, {'stair_roi': ([-1., -1., -2.], [1., 1., 7.])})
    actual = {int(i): p for i, p in zip(ids, mapped)}
    assert set(actual) == {0, 1, 2}
    np.testing.assert_allclose(actual[0], [.1, .1, -.75])
    np.testing.assert_allclose(actual[1], [.1, .1, 3.05])
    np.testing.assert_allclose(actual[2], points[2])
    np.testing.assert_allclose(points[0], [.1, .1, -.65])  # No map rewrite.


def test_real_cost_colors_and_rgb_payload_are_not_flat_white():
    xyz = np.array([[0., 1., 2.], [1., 2., 3.], [2., 3., 4.]])
    values = colored_records(xyz, [0., 10., 20.])
    assert values.dtype.itemsize == 16
    assert values['rgb'].tolist() == [(45 << 16) | (225 << 8) | 65,
        (145 << 16) | (202 << 8) | 65, (245 << 16) | (180 << 8) | 65]
    assert np.all(colored_records(xyz)['rgb'] == (220 << 16) | (68 << 8) | 68)
    for costs in ([0., 21., 0.], [0., float('nan'), 0.], [0.]):
        with pytest.raises(ValueError):
            colored_records(xyz, costs)


def test_goal_activation_preserves_xyz_and_prefers_nearby_true_floor_not_other_floor():
    assert goal_position(existing=(1., 2., 3.)) == (1., 2., 3.)
    assert goal_position(fallback=(4.9, 22.2, -.57)) == (4.9, 22.2, -.57)
    floors = np.array([[1.5, 0., 3.], [1.6, 0., 0.]])
    assert goal_position(body=(0., 0., .55, 0.), surfaces=floors) == (1.6, 0., 0.)
    assert np.allclose(goal_position(body=(0., 0., .55, math.pi/2)), [0., 1.5, 0.])
    with pytest.raises(ValueError):
        goal_position(existing=(float('nan'), 0., 0.))


def test_initial_anchor_selection_compares_only_planning_coordinates():
    planning = np.array([[0., 0., 0.], [10., 0., 0.]])
    original = np.array([[100., 0., 1.], [0., 0., 1.]])
    assert initial_goal_from_candidates(planning, original, np.array([0, 1]),
        np.array([0, 1]), [0., 0., 0.]) == [100., 0., 1.]


def test_real_saved_multifloor_layers_are_bound_to_original_map_and_true_costs():
    config = Path(__file__).resolve().parents[1]/'config/live_visualization.yaml'
    session = yaml.safe_load(config.read_text())
    if not Path(session['planning_manifest']).is_file():
        pytest.skip('Audited large local map is not installed')
    xyz, costs, blocked, info = build_layers(session)
    assert 10000 < len(xyz) < 1000000 and 1000 < len(blocked) < 1000000
    assert len(costs) == len(xyz) and np.all((costs >= 0) & (costs <= 20))
    assert np.any(xyz[:, 2] < 0) and np.any(xyz[:, 2] > 3.)
    assert np.any(np.abs(xyz[xyz[:, 2] < 0, 2] + .65) > .03)
    assert info['frame_id'] == 'd1max_loc_map'
    assert info['estimated_inverse_not_tf'] and info['motion_enabled'] is False
    assert info['unsupported_omitted_cells'] == info['unsupported_traversable_cells']+info['unsupported_blocked_cells']
    assert len(info['initial_goal_xyz']) == 3
    assert len(np.unique(colored_records(xyz, costs)['rgb'])) > 2
