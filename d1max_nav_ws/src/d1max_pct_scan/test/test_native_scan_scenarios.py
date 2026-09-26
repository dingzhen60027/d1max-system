"""Pure fixture/checker tests; importing the smoke never initializes ROS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest


SPEC = importlib.util.spec_from_file_location('native_scan_synthetic_fixture',
    Path(__file__).with_name('offline_native_scan_scenarios.py'))
SMOKE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SMOKE)


def spline_fixture(z=.55):
    x = np.arange(-.3, 4.81, .3)
    trajectory = NS(order=3, knots=((np.arange(len(x)+4)-3)*3.).tolist(),
                    pos_pts=[NS(x=float(value), y=0., z=z) for value in x])
    tag = NS(trajectory=trajectory)
    debug = NS(plan_id=1, generation=1, local_target=NS(x=4.5, y=0., z=z))
    params = {'manager.max_vel': .30, 'manager.max_acc': .35, 'fsm.planning_horizon': 6.,
              'grid_map.double_cylinder_radius': .29, 'grid_map.double_cylinder_offset': .20,
              'grid_map.obstacles_inflation_z_up': .45, 'grid_map.obstacles_inflation_z_down': .45}
    return tag, debug, params


def test_first_return_has_front_face_and_no_invented_hidden_box_faces():
    cloud = SMOKE.first_return_cloud(np.array([0., 0., .55]), SMOKE.CASES['box_occluded'])
    assert np.isfinite(cloud).all() and 1000 < len(cloud) <= 250000
    assert np.count_nonzero(np.isclose(cloud[:, 0], 1.5, atol=1e-5)) > 1000
    hidden_side = ((cloud[:, 0] > 1.5001) & (cloud[:, 0] < 2.2001)
                   & (np.abs(np.abs(cloud[:, 1])-.4) < 1e-5))
    assert not hidden_side.any()
    assert not np.any((cloud[:, 0] > 1.5001) & (cloud[:, 0] < 2.1999)
                      & (np.abs(cloud[:, 1]) < .3999))


def test_closed_wall_returns_do_not_pass_through_the_wall():
    cloud = SMOKE.first_return_cloud(np.array([0., 0., .55]), SMOKE.CASES['closed_corridor'])
    assert cloud[:, 0].max() <= 1.50001


def test_other_physical_view_observes_box_sides_but_never_interior():
    cloud = SMOKE.first_return_cloud(np.array([2., 1.2, .55]), SMOKE.CASES['box_detour_multiview'])
    assert np.count_nonzero(np.isclose(cloud[:, 1], .4, atol=1e-5)
                           & (cloud[:, 0] > 1.5) & (cloud[:, 0] < 2.2)) > 100
    assert not np.any((cloud[:, 0] > 1.5001) & (cloud[:, 0] < 2.1999)
                      & (np.abs(cloud[:, 1]) < .3999))


def test_independent_checker_accepts_straight_safe_full_curve():
    result = SMOKE.check_native_curve(*spline_fixture(), obstacle=None)
    assert result['minimum_analytic_clearance_m'] > 1.
    assert result['max_sample_spacing_m'] < .005
    assert result['end'] == pytest.approx([4.5, 0., .55])


def test_independent_checker_rejects_a_curve_through_solid_box():
    with pytest.raises(AssertionError, match='collides with analytic box'):
        SMOKE.check_native_curve(*spline_fixture(), obstacle=SMOKE.CASES['box_occluded'])


def test_independent_checker_does_not_treat_floor_penetration_as_free():
    with pytest.raises(AssertionError, match='floor/ceiling'):
        SMOKE.check_native_curve(*spline_fixture(z=.1), obstacle=None)


def test_independent_checker_rejects_speed_violation_without_relaxing_limits():
    tag, debug, params = spline_fixture()
    tag.trajectory.knots = (np.asarray(tag.trajectory.knots)*.1).tolist()
    with pytest.raises(AssertionError, match='speed violation'):
        SMOKE.check_native_curve(tag, debug, params, obstacle=None)
