import csv
from pathlib import Path

import numpy as np
import pytest

from tools.pointcloud_preprocessing.ground_path_bridge import (
    FloorField, GroundBridgeError, GroundPathBridge, ProjectionLimits,
)


def _floor(floor_id='floor1', *, source_z=-0.75, reference_z=-0.65,
           support_xy=None, slope=0.02):
    xy = np.array([[0., 0.], [1., 0.], [0., 1.], [1., 1.]])
    anchors = {'xy': xy, 'z': source_z + slope * xy[:, 0],
               'slopes': np.tile([slope, 0.], (len(xy), 1))}
    support = xy if support_xy is None else np.asarray(support_xy, float)
    return FloorField(floor_id, reference_z, (-2., 2.3) if floor_id == 'floor1' else (2.7, 7.),
                      anchors, support, {'conditioning_support_radius_m': .5})


def _bridge(*, protected=(), floor1=None):
    return GroundPathBridge({'floor1': floor1 or _floor(),
                             'floor2': _floor('floor2', source_z=3.04,
                                              reference_z=3.15)}, protected)


def test_observed_ground_round_trip_and_diagnostics():
    bridge = _bridge()
    original = np.array([[.10, .10, -.748], [.20, .10, -.746]])
    forward = bridge.to_planning_ground(original, ['floor1'] * 2)
    assert np.allclose(forward.xyz[:, 2], -.65, atol=1e-10)
    backward = bridge.to_localization_ground(forward.xyz, ['floor1'] * 2)
    assert np.allclose(backward.xyz, original, atol=1e-10)
    assert backward.diagnostics['maximum_support_distance_m'] < .25
    assert not backward.diagnostics['exact_inverse_of_snapped_records']
    assert backward.diagnostics['target_frame'] == 'd1max_loc_map'


def test_protected_stair_points_are_unchanged_in_both_directions():
    region = {'min': [.4, .4, -2.], 'max': [1.5, 1.5, 7.]}
    bridge = _bridge(protected=[region])
    step = [[.5, .5, .7], [.6, .6, 1.1]]
    assert np.array_equal(bridge.to_localization_ground(step, ['stairs'] * 2).xyz, step)
    assert np.array_equal(bridge.to_planning_ground(step, ['stairs'] * 2).xyz, step)


def test_unobserved_or_wrong_floor_rejected():
    bridge = _bridge()
    with pytest.raises(GroundBridgeError, match='no bounded observed'):
        bridge.to_localization_ground([[20., 20., -.65]], ['floor1'])
    with pytest.raises(GroundBridgeError, match='not near its floor'):
        bridge.to_localization_ground([[.1, .1, -.65]], ['floor2'])
    with pytest.raises(GroundBridgeError, match='explicit floor'):
        bridge.to_localization_ground([[.1, .1, -.65]], [])
    with pytest.raises(GroundBridgeError, match='Unprotected stair'):
        bridge.to_localization_ground([[.1, .1, .0]], ['stairs'])


def test_protected_boundary_jump_is_not_hidden():
    floor = _floor(source_z=-.95, reference_z=-.65)
    region = {'min': [.5, -.1, -2.], 'max': [1.5, 1.5, 7.]}
    bridge = _bridge(protected=[region], floor1=floor)
    with pytest.raises(GroundBridgeError, match='correction jumps'):
        bridge.to_localization_ground([[.49, 0., -.65], [.51, 0., -.65]],
                                      ['floor1', 'stairs'])


def test_live_body_pose_only_projects_ground_after_height_and_floor_checks():
    bridge = _bridge()
    projected = bridge.project_live_pose_to_ground([.1, .1, -.23], 'floor1',
                                                    body_height_interval_m=(.4, .7))
    assert np.allclose(projected.xyz, [.1, .1, -.65])
    assert np.isclose(projected.diagnostics['source_ground_xyz'][2], -.748)
    with pytest.raises(GroundBridgeError, match='calibrated interval'):
        bridge.project_live_pose_to_ground([.1, .1, -.23], 'floor1',
                                           body_height_interval_m=(.1, .2))
    with pytest.raises(GroundBridgeError, match='explicit floor1/floor2'):
        bridge.project_live_pose_to_ground([.1, .1, -.23], 'stairs',
                                           body_height_interval_m=(.4, .7))


def test_nonfinite_and_invalid_limits_fail_closed():
    bridge = _bridge()
    with pytest.raises(GroundBridgeError, match='finite Nx3'):
        bridge.to_localization_ground([[np.nan, 0., 0.]], ['floor1'])
    with pytest.raises(GroundBridgeError, match='finite and positive'):
        ProjectionLimits(max_boundary_jump_m=float('nan'))


def test_saved_0923_crossfloor_route_projects_to_original_map_if_available():
    output = (Path(__file__).resolve().parents[3] / 'maps/processed/'
              'sc_pgo_20260923_crossfloor_complete_001')
    if not (output / 'manifest.json').exists():
        pytest.skip('Large local 09-23 map artifact is not installed')
    bridge = GroundPathBridge.from_artifacts(output / 'manifest.json')
    with (output / 'route/path.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    planned = np.asarray([[float(row[key]) for key in ('x_m', 'y_m', 'z_m')]
                          for row in rows])
    labels = [{'lower_floor': 'floor1', 'upper_floor': 'floor2'}.get(
        row['outgoing_leg'] or row['incoming_leg'], row['outgoing_leg'] or row['incoming_leg'])
        for row in rows]
    original = bridge.to_localization_ground(planned, labels)
    restored = bridge.to_planning_ground(original.xyz, labels)
    assert len(planned) > 3000
    assert original.diagnostics['protected_stair_points'] > 0
    assert original.diagnostics['maximum_support_distance_m'] < .15
    assert original.diagnostics['maximum_adjacent_correction_jump_m'] < .02
    assert np.max(np.abs(restored.xyz - planned)) < 1e-8
