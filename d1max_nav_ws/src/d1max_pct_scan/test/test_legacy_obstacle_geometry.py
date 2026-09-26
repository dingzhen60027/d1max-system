"""Pure geometry/config contracts; never imports/starts the ROS simulator."""
import ast
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from d1max_navigation.cloud_to_scan import project_points
from d1max_pct_scan.robot_profile import load_robot_profile, scan_robot_parameters, safety_scan_parameters


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'d1max_pct_scan/d1max_pct_scan/simulation.py'
TREE = ast.parse(SOURCE.read_text())
FUNCTION = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'simulation_scan_parameters')
NAMESPACE = {'math': math}
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(SOURCE), 'exec'), NAMESPACE)
slice_parameters = NAMESPACE['simulation_scan_parameters']


def test_default_legacy_scan_and_simulator_share_live_vertical_geometry():
    profile = load_robot_profile(ROOT/'d1max_scan_planner/config/d1max_robot.yaml')
    legacy = next(iter(yaml.safe_load((ROOT/'d1max_scan_planner/config/d1max_scan_planner.yaml').read_text()).values()))['ros__parameters']
    live = scan_robot_parameters(profile)
    for key in ('grid_map.body_height', 'grid_map.obstacles_inflation_z_up', 'grid_map.obstacles_inflation_z_down'):
        assert legacy[key] == live[key]
    assert slice_parameters(legacy['grid_map.body_height']) == pytest.approx(safety_scan_parameters(profile))
    assert legacy.get('grid_map.require_observed_free', False) is False  # policy not silently changed
    assert profile['engineering']['motion_authorized'] is False
    assert profile['engineering']['vertical_envelope_validated'] is False


@pytest.mark.parametrize('body_height', [.40, .55, .61])
@pytest.mark.parametrize('ground_z', [-3.17, 0., 4.31])
def test_simulation_does_not_drop_20cm_object_or_turn_missing_rays_free(body_height, ground_z):
    params = slice_parameters(body_height)
    # Ground and a 20 cm box top at distinct bearings; only the box top survives.
    cloud = [[2., 0., ground_z], [0., 2., ground_z+.20]]
    ranges = project_points(cloud, [0., 0., -ground_z-body_height], [0., 0., 0., 1.], **params)
    assert np.count_nonzero(np.isfinite(ranges)) == 1
    assert ranges[np.isfinite(ranges)][0] == pytest.approx(2.)
    assert np.count_nonzero(np.isnan(ranges)) == 719
    assert not np.isinf(ranges).any()


@pytest.mark.parametrize('height', [float('nan'), float('inf'), -.1, 0., .10, 1.01, True, '0.55'])
def test_invalid_simulated_height_is_rejected(height):
    with pytest.raises(ValueError, match='body reference height'):
        slice_parameters(height)


def test_simulator_runtime_uses_the_checked_slice_instead_of_a_literal():
    simulator = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'Simulator')
    constructor = next(n for n in simulator.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    assert any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'simulation_scan_parameters'
               for n in ast.walk(constructor))
    clouds = next(n for n in simulator.body if isinstance(n, ast.FunctionDef) and n.name == 'clouds')
    projection = next(n for n in ast.walk(clouds) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'project_points')
    assert not any(keyword.arg in ('min_height', 'max_height') for keyword in projection.keywords)
    assert any(keyword.arg is None and isinstance(keyword.value, ast.Attribute) and keyword.value.attr == 'scan_slice'
               for keyword in projection.keywords)
