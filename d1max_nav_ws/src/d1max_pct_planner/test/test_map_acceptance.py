import numpy as np
import pytest

from d1max_pct_planner.map_acceptance import floor_components, densify_xy


def test_diagonal_touch_does_not_join_two_rooms():
    allowed = np.eye(3, dtype=bool)
    labels, sizes = floor_components(allowed, np.zeros((3,3)), .17)
    assert sorted(sizes.tolist()) == [1,1,1]
    assert len(set(labels[allowed])) == 3


def test_known_unbroken_corridor_is_one_component():
    allowed = np.ones((3,20), dtype=bool)
    _, sizes = floor_components(allowed, np.zeros((3,20)), .17)
    assert sizes.tolist() == [60]


def test_wall_and_unknown_are_not_bridged():
    allowed = np.ones((3,8), dtype=bool)
    ground = np.zeros((3,8))
    allowed[:, 3] = False
    ground[:, 6] = np.nan
    _, sizes = floor_components(allowed, ground, .17)
    assert sorted(sizes.tolist()) == [3,6,9]


def test_height_step_remains_disconnected_even_when_cost_allows():
    ground = np.zeros((3,6))
    ground[:, 3:] = .5
    _, sizes = floor_components(np.ones_like(ground, dtype=bool), ground, .17)
    assert sizes.tolist() == [9,9]


def test_small_ground_step_allowed():
    ground = np.zeros((3,6))
    ground[:, 3:] = .1
    _, sizes = floor_components(np.ones_like(ground, dtype=bool), ground, .17)
    assert sizes.tolist() == [18]


def test_densify_preserves_reference_endpoints_without_cost_mutation():
    points = np.array([[0.,0.,4.], [1.,0.,6.], [1.,1.,3.]])
    dense = densify_xy(points, .1)
    np.testing.assert_array_equal(dense[[0,-1]], points[[0,-1], :2])
    assert np.max(np.linalg.norm(np.diff(dense, axis=0), axis=1)) <= .1+1e-12


def test_invalid_graph_parameters_fail():
    with pytest.raises(ValueError):
        floor_components(np.ones((3,3)), np.zeros((3,2)), .17)
    with pytest.raises(ValueError):
        floor_components(np.ones((3,3)), np.zeros((3,3)), 0.)
