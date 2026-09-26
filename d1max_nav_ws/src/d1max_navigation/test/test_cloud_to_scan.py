import math
import numpy as np
import pytest
from d1max_navigation.cloud_to_scan import project_points


def test_nearest_ray_and_height_filter():
    scan = project_points([[2, 0, 0], [1, 0, .1], [0, 1, -.5], [0, 2, 1.2]], [0,0,0], [0,0,0,1])
    assert scan[360] == pytest.approx(1.)
    assert np.count_nonzero(np.isfinite(scan)) == 1


def test_unobserved_is_unknown_not_infinite_clearing_ray():
    scan = project_points([], [0,0,0], [0,0,0,1])
    assert np.isnan(scan).all() and not np.isinf(scan).any()


def test_rotation_and_translation_are_applied_once():
    scan = project_points([[1,0,0]], [0,1,0], [0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
    assert scan[540] == pytest.approx(2.)


def test_nan_and_far_returns_rejected():
    scan = project_points([[np.nan,1,0],[20,0,0],[.05,0,0]], [0,0,0], [0,0,0,1])
    assert np.isnan(scan).all()


def test_bad_transform_rejected():
    with pytest.raises(ValueError): project_points([], [0,0,0], [0,0,0,0])


def test_low_obstacle_inside_stop_polygon_survives_while_ground_is_excluded():
    # Standing profile's .55 m reference height is an assumption, not a passed
    # calibration. Cloud is in a ground-origin frame, transformed to base once.
    low = [[.5, y, .20] for y in np.linspace(-.15, .15, 25)]
    ground = [[.4, y, 0.] for y in np.linspace(-.2, .2, 25)]
    scan = project_points(low + ground + [[3., 0., .6]], [0, 0, -.55], [0, 0, 0, 1])
    assert np.count_nonzero(np.isfinite(scan) & (scan < .6)) >= 20
    floor_only = project_points(ground, [0, 0, -.55], [0, 0, 0, 1])
    assert np.isnan(floor_only).all()


def test_ground_exclusion_does_not_filter_obstacles_above_its_bound():
    for height in [.101, .12, .20, .34]:
        scan = project_points([[.5, 0, height]], [0, 0, -.55], [0, 0, 0, 1])
        assert scan[360] == pytest.approx(.5)
