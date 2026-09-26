import numpy as np
import pytest

from d1max_pct_planner.measured_grid import MeasuredGrid
from d1max_pct_planner.route_engine import plan_checked


def save_grid(tmp_path, free=None, resolution=0.2, origin=(10.0, -5.0), height=None):
    if free is None:
        free = np.ones((30, 30), dtype=bool)
        free[[0, -1], :] = False
        free[:, [0, -1]] = False
    target = tmp_path / 'measured.npz'
    np.savez(target, free=free, support=free, obstacles=~free,
             height=np.zeros(free.shape) if height is None else height,
             origin=np.asarray(origin), resolution=resolution)
    return target


def test_grid_center_preserves_pcd_coordinates(tmp_path):
    grid = MeasuredGrid(save_grid(tmp_path))
    payload = grid.payload()
    for index in [(2, 5), (18, 12), (25, 25)]:
        xyz = grid.origin + (np.array(index) + 0.5) * grid.resolution
        pct_index = np.rint((xyz - payload['center']) / grid.resolution).astype(int)
        pct_index += np.array(grid.free.shape) // 2
        np.testing.assert_array_equal(pct_index, index)


def test_does_not_make_unknown_or_obstacle_free(tmp_path):
    grid = MeasuredGrid(save_grid(tmp_path))
    assert not np.any(grid.planner_free & ~grid.free)
    assert np.all(grid.cost[~grid.free] == 50)
    assert grid.planner_free.sum() < grid.free.sum()
    with pytest.raises(ValueError, match='outside measured'):
        grid.validate_point((10.05, -4.95))


def test_validation_is_not_just_waypoint_validation(tmp_path):
    free = np.ones((30, 30), dtype=bool)
    free[15, 15] = False
    grid = MeasuredGrid(save_grid(tmp_path, free))
    with pytest.raises(ValueError, match='crosses blocked'):
        grid.validate_path([[11.1, -1.9, 0], [15.1, -1.9, 0]])


def test_diagonal_corner_cutting_is_rejected(tmp_path):
    free = np.ones((30, 30), dtype=bool)
    free[6, 5] = False
    grid = MeasuredGrid(save_grid(tmp_path, free))
    with pytest.raises(ValueError, match='crosses blocked'):
        grid.validate_path([[11.1, -3.9, 0], [11.3, -3.7, 0]])


def test_ground_step_but_not_gradual_slope_is_rejected(tmp_path):
    height = np.indices((30, 30))[0] * 0.05
    grid = MeasuredGrid(save_grid(tmp_path, height=height))
    assert grid.validate_path([[11.1, -3.9, 0.25], [14.1, -3.9, 1.0]])['checked_cells'] > 2
    grid.height[12:] += 0.5
    with pytest.raises(ValueError, match='local ground step'):
        grid.validate_path([[11.1, -3.9, 0.25], [14.1, -3.9, 1.5]])


@pytest.mark.parametrize('point', [[np.nan, 0], [np.inf, 1], [0], [0, 0, 0]])
def test_bad_point_rejected(tmp_path, point):
    grid = MeasuredGrid(save_grid(tmp_path))
    with pytest.raises(ValueError):
        grid.validate_point(point)


def test_exact_endpoints_restored_and_checked(tmp_path):
    grid = MeasuredGrid(save_grid(tmp_path))
    class NativeStub:
        def plan(self, start, goal):
            return np.array([[11.5, -3.5, 0], [12.3, -3.5, 0]])
    start, goal = [11.3, -3.5], [12.5, -3.5]
    result = plan_checked(NativeStub(), grid, start, goal)
    assert result['path'][0][:2] == start
    assert result['path'][-1][:2] == goal
    assert result['height_semantics'] == 'ground_z'


def test_unsafe_native_result_has_no_fallback(tmp_path):
    free = np.ones((30, 30), dtype=bool)
    free[15, 15] = False
    grid = MeasuredGrid(save_grid(tmp_path, free))
    class NativeStub:
        def plan(self, start, goal):
            return np.array([[*start, 0], [*goal, 0]])
    with pytest.raises(ValueError, match='crosses blocked'):
        plan_checked(NativeStub(), grid, [11.1, -1.9], [15.1, -1.9])


@pytest.mark.parametrize('guard', [-1, 1.5, 6])
def test_guard_is_bounded(tmp_path, guard):
    with pytest.raises(ValueError, match='optimization_guard_cells'):
        MeasuredGrid(save_grid(tmp_path), optimization_guard_cells=guard)
