from pathlib import Path

import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramError, TomogramMap
from d1max_pct_planner.tomogram_route import TomogramRoute


def scene():
    data = np.zeros((5, 2, 21, 21), dtype=np.float32)
    data[3, 0], data[3, 1] = 0, 3
    data[4, 0], data[4, 1] = 2, 5
    return TomogramMap({'data': data, 'resolution': .2, 'center': [12.3, -4.2],
                        'slice_h0': .5, 'slice_dh': .5, 'selected_source_layers': [0, 6]})


class FakeNative:
    def __init__(self, path, layers):
        self.path, self.layers = np.asarray(path), np.asarray(layers)
        self.calls = []

    def plan(self, start, goal, start_layer, goal_layer, return_details=False):
        self.calls.append((start, goal, start_layer, goal_layer, return_details))
        return {'path': self.path, 'layer_ids': self.layers,
                'native_states': None, 'native_sample_dt': None}


def route_with_fake(tomogram, path, layers):
    route = TomogramRoute.__new__(TomogramRoute)
    route.tomogram, route.height_tolerance_m = tomogram, .08
    route.planner = FakeNative(path, layers)
    return route


def test_layered_route_preserves_both_free_xyz_endpoints_and_requested_layers():
    tomo = scene()
    a, b = tomo.world([6, 10]), tomo.world([14, 10])
    route = route_with_fake(tomo, [[*a, 3], [*b, 3]], [1, 1])
    start, goal = [*a, 3.04], [*b, 2.96]
    result = route.plan(start, goal, 1, 1)
    assert result['path'][0] == start and result['path'][-1] == goal
    assert set(result['layer_ids']) == {1}
    assert set(result['source_layer_ids']) == {6}
    assert route.planner.calls[0][2:] == (1, 1, True)


def test_connector_is_checked_and_not_forced_to_pass():
    tomo = scene()
    tomo.valid[0, 8, 10] = False
    tomo.cost[0, 8, 10] = 50
    a, b, shifted = tomo.world([6, 10]), tomo.world([14, 10]), tomo.world([10, 10])
    route = route_with_fake(tomo, [[*shifted, 0], [*b, 0]], [0, 0])
    with pytest.raises(TomogramError):
        route.plan([*a, 0], [*b, 0], 0, 0)


def test_optional_refinement_reports_real_algorithm_and_exact_endpoints():
    tomo = scene()
    a, b = tomo.world([3, 10]), tomo.world([17, 10])
    middle = tomo.world([10, 12])
    route = route_with_fake(tomo, [[*a, 0], [*middle, 0], [*b, 0]], [0, 0, 0])
    route.path_refinement, route.refinement_corner_cut_m = 'visibility_c2', 1.5
    start, goal = [*a, .03], [*b, -.03]
    result = route.plan(start, goal, 0, 0)
    assert result['path'][0] == start and result['path'][-1] == goal
    assert result['path_refinement']['applied']
    assert result['native_geometry'] is not None
    assert result['quintic_segments'] == 0
    assert 'corridor refinement' in result['algorithm']
    assert result['path_quality']['lateral_deviation_max_m'] < 1e-8
    assert result['cost_threshold'] == 20


def test_refinement_cannot_rescue_an_invalid_native_curve():
    tomo = scene()
    tomo.valid[0, 10, 10] = False
    tomo.cost[0, 10, 10] = 50
    a, b = tomo.world([6, 10]), tomo.world([14, 10])
    route = route_with_fake(tomo, [[*a, 0], [*b, 0]], [0, 0])
    route.path_refinement, route.refinement_corner_cut_m = 'visibility_c2', 1.5
    with pytest.raises(TomogramError):
        route.plan([*a, 0], [*b, 0], 0, 0)


def test_curve_cannot_switch_to_a_different_floor_at_same_xy():
    tomo = scene()
    a, b = tomo.world([6, 10]), tomo.world([14, 10])
    route = route_with_fake(tomo, [[*a, 0], [*b, 3]], [0, 1])
    with pytest.raises(TomogramError):
        route.plan([*a, 0], [*b, 3], 0, 1)


def test_same_cell_has_no_fabricated_native_gpmp_result():
    tomo = scene()
    point = tomo.world([10, 10])
    route = route_with_fake(tomo, [[*point, 0], [*point, 0]], [0, 0])
    with pytest.raises(TomogramError, match='distinct cells'):
        route.plan([*point, 0], [*point + [.01, 0], 0], 0, 0)
    assert not route.planner.calls


def test_real_native_nonzero_layer_and_free_endpoint_z():
    vendor = Path(__file__).resolve().parents[2] / 'pct_planner_vendor'
    if not list((vendor / 'planner/lib').glob('ele_planner*.so')):
        pytest.skip('Native PCT extension is not installed')
    tomo = scene()
    route = TomogramRoute(tomo, vendor)
    a, b = tomo.world([6, 10]), tomo.world([14, 10])
    start, goal = [*a, 3.03], [*b, 2.97]
    result = route.plan(start, goal, 1, 1)
    assert result['path'][0] == start and result['path'][-1] == goal
    assert set(result['layer_ids']) == {1}
    assert result['quintic_segments'] > 0
    assert result['curve_validation'] == 'quintic_cell_boundary_roots_and_interval_interiors'
    assert result['extra_erosion_cells'] == 0
    assert result['native_parameters']['astar_cost_weight'] == .2
    assert result['native_parameters']['optimizer_cost_margin'] == 15.0


def test_crossfloor_worker_never_allocates_discarded_full_native_map(monkeypatch):
    import sys
    from queue import Queue
    from types import SimpleNamespace
    import d1max_pct_planner.tomogram_route as module

    tomo = scene()
    constructed = []

    def coordinator(actual_map, config):
        assert actual_map is tomo and config == 'crossfloor.yaml'
        constructed.append(config)
        return SimpleNamespace(native_runtime={}, native_parameters={})

    def forbidden(*args, **kwargs):
        raise AssertionError('Crossfloor startup must not instantiate a discarded native full map')

    monkeypatch.setattr(module, 'TomogramMap', lambda *args, **kwargs: tomo)
    monkeypatch.setattr(module, 'TomogramRoute', forbidden)
    monkeypatch.setitem(sys.modules, 'd1max_pct_planner.crossfloor_preview',
                        SimpleNamespace(CrossfloorPreviewRoute=coordinator))
    requests, results = Queue(), Queue()
    requests.put(None)
    module.worker_main({'tomogram_path': 'map.npz', 'crossfloor_route_config': 'crossfloor.yaml'},
                       requests, results)
    assert results.get_nowait()['kind'] == 'ready'
    assert constructed == ['crossfloor.yaml']
