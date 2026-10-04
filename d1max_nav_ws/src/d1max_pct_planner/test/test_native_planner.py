from pathlib import Path

import numpy as np
import pytest

from d1max_pct_planner.measured_grid import MeasuredGrid
from d1max_pct_planner.planner_core import TomogramPlanner
from d1max_pct_planner.route_engine import plan_checked


from d1max_pct_planner.paths import nav_root, vendor_checkout

VENDOR = vendor_checkout()
GRID = nav_root()/'maps/processed/sc_pgo_20260919_ground_20260921/global_path_reachable/planning_grid.npz'
pytestmark = pytest.mark.skipif(not (VENDOR / 'planner/lib/ele_planner.cpython-310-x86_64-linux-gnu.so').exists(),
                                reason='Native PCT extension is not built')


@pytest.fixture
def planner():
    grid = MeasuredGrid(GRID)
    native = TomogramPlanner(VENDOR, ground_z=True)
    native.load_payload(grid.payload())
    return native, grid


@pytest.mark.skipif(not GRID.exists(), reason='Recorded PCD planning grid unavailable')
def test_recorded_corridor_native_gpmp_safe_short_route(planner):
    native, grid = planner
    result = plan_checked(native, grid, [-4.100001525878895, 6.9], [-5.500001525878901, 10.5])
    assert 3.8 < result['length_m'] < 4.5
    assert result['checked_cells'] >= 20
    assert result['minimum_clearance_m'] > 0.4
    # Ground heights, not the native visualization/body +0.1/+0.5 offsets.
    assert abs(result['path'][0][2] + 0.63750159740448) < 1e-6
    assert abs(result['path'][-1][2] + 0.7147215604782104) < 1e-6


@pytest.mark.parametrize('point', [[-1000, 0], [0, 1000], [np.nan, 0]])
@pytest.mark.skipif(not GRID.exists(), reason='Recorded PCD planning grid unavailable')
def test_native_out_of_bounds_blocked_before_cpp(planner, point):
    native, _ = planner
    with pytest.raises(ValueError):
        native.plan(point, [-5.500001525878901, 10.5])


@pytest.mark.skipif(not GRID.exists(), reason='Recorded PCD planning grid unavailable')
def test_native_same_cell_no_empty_vector_crash(planner):
    native, _ = planner
    point = [-4.100001525878895, 6.9]
    result = native.plan(point, point)
    assert result.shape == (2, 3)
    assert np.isfinite(result).all()
