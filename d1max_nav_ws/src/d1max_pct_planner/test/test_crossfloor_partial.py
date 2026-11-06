"""Partial-stair topology exercises production map checks with a native seam."""
from copy import deepcopy

import numpy as np
import pytest

from d1max_pct_planner import crossfloor_preview as preview
from d1max_pct_planner.crossfloor_partial import endpoint_region, plan_partial_stair
from d1max_pct_planner.crossfloor_route import CrossfloorError, _masked_tomogram, validate_config
from d1max_pct_planner.tomogram_map import TomogramMap
from test_crossfloor_preview import prepared, fixed_stair_native
from test_crossfloor_route import config, scene, xyz


@pytest.mark.parametrize('start,goal,expected', [
    (1, 15, [(1, 5), (5, 15)]),
    (15, 1, [(15, 5), (5, 1)]),
    (39, 27, [(39, 35), (35, 27)]),
    (27, 39, [(27, 35), (35, 39)]),
    (1, 27, [(1, 5), (5, 21), (21, 27)]),
    (27, 1, [(27, 21), (21, 5), (5, 1)]),
    (15, 39, [(15, 21), (21, 35), (35, 39)]),
    (39, 15, [(39, 35), (35, 21), (21, 15)]),
    (15, 27, [(15, 21), (21, 27)]),
    (27, 15, [(27, 21), (21, 15)]),
    (15, 18, [(15, 18)]),
    (18, 15, [(18, 15)]),
    (25, 31, [(25, 31)]),
    (31, 25, [(31, 25)]),
    (1, 21, [(1, 5), (5, 21)]),
    (21, 1, [(21, 5), (5, 1)]),
    (21, 39, [(21, 35), (35, 39)]),
    (39, 21, [(39, 35), (35, 21)]),
    (21, 15, [(21, 15)]),
    (27, 21, [(27, 21)]),
])
def test_partial_endpoints_use_ordered_native_queries_and_complete_metadata(
        prepared, monkeypatch, start, goal, expected):
    calls = fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    result = route.plan(xyz(start), xyz(goal), int(start > 19), int(goal > 19))
    assert calls == (expected if goal > start else [(b, a) for a, b in expected[::-1]])
    assert result['route_type'] == 'partial_stair'
    assert result['direction'] == ('lower_to_upper' if goal > start else 'upper_to_lower')
    assert result['path'][0] == xyz(start) and result['path'][-1] == xyz(goal)
    assert result['source_layer_ids'] == [int(i)*6 for i in result['layer_ids']]
    assert len(result['edge_legs']) == len(result['path'])-1
    cursor = 0
    for segment in result['segments']:
        a, b = segment['first_index'], segment['last_index']
        assert a == cursor and b > a
        assert result['edge_legs'][a:b] == [segment['name']]*(b-a)
        assert result['path'][a] == result['anchors'][segment['from']]['xyz']
        assert result['path'][b] == result['anchors'][segment['to']]['xyz']
        assert segment['checks']['original']['checked_cells'] > 0
        assert segment['curve_validation'] == 'quintic_cell_boundary_roots_and_interval_interiors'
        cursor = b
    assert cursor == len(result['path'])-1
    assert result['fixed_stair_cache']['entries'] == 0
    assert result['execution_authorized'] is False


def test_same_flight_partial_need_not_invent_a_layer_transition(prepared, monkeypatch):
    fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    result = route.plan(xyz(11), xyz(15), 0, 0)
    assert result['layer_transitions'] == []
    assert result['edge_legs'] == ['stair_lower']*(len(result['path'])-1)


def test_partial_request_does_not_poison_original_floor_floor_caches(prepared, monkeypatch):
    calls = fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    original = route.plan(xyz(1), xyz(39), 0, 1)
    cached = deepcopy(route._fixed_stairs._results)
    route.plan(xyz(15), xyz(27), 0, 1)
    after = route.plan(xyz(1), xyz(39), 0, 1)
    assert after['path'] == original['path']
    assert after['edge_legs'] == original['edge_legs']
    assert route._fixed_stairs._results == cached
    assert calls[-2:] == [(1, 5), (35, 39)]
    assert after['native_map_cache']['entries'] <= 3


def test_native_partial_failure_drops_resources_and_never_falls_back_to_a_line(prepared, monkeypatch):
    fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    _, native = route._resources('stair_lower', route.settings)
    def blocked(*_):
        raise ValueError('native search found disconnected support')
    monkeypatch.setattr(native, 'plan', blocked)
    with pytest.raises(CrossfloorError, match='Native PCT failed') as exc:
        route.plan(xyz(15), xyz(27), 0, 1)
    assert exc.value.code == 'segment_planning_failed'
    assert route._native_maps == {} and route._fixed_stairs._results == {}


def test_intermediate_height_outside_stair_roi_is_not_an_arbitrary_third_floor():
    with pytest.raises(CrossfloorError, match='起终点'):
        endpoint_region([100., 100., 1.], validate_config(config()))


@pytest.mark.parametrize('z,expected', [
    (0., 'lower'), (.08, 'lower'), (.2, 'stair_lower'),
    (2.8, 'stair_upper'), (2.92, 'upper'), (3., 'upper'),
])
def test_floor_search_bands_do_not_relabel_first_and_last_treads(z, expected):
    settings = validate_config(config())
    settings['floor_z_ranges'] = {'lower': [-.3, .3], 'upper': [2.7, 3.3]}
    assert endpoint_region([0., 0., z], settings) == expected


@pytest.mark.parametrize('z,floor', [(.2, 'lower'), (2.8, 'upper')])
def test_height_offset_outside_stair_roi_remains_ordinary_floor(z, floor):
    settings = validate_config(config())
    settings['floor_z_ranges'] = {'lower': [-.3, .3], 'upper': [2.7, 3.3]}
    assert endpoint_region([0., 5., z], settings) == floor


@pytest.mark.parametrize('start,goal,leg', [(6, 7, 'stair_lower'), (7, 6, 'stair_lower'),
    (33, 34, 'stair_upper'), (34, 33, 'stair_upper')])
def test_overlapping_floor_band_treads_keep_native_stair_semantics(prepared, monkeypatch, start, goal, leg):
    import yaml
    # Broaden only the declared floor search bands as in the actual map;
    # the fixture's original measured stair cells and costs remain unchanged.
    prepared.raw['floor_z_ranges'] = {'lower': [-.3, .3], 'upper': [2.7, 3.3]}
    prepared.path.write_text(yaml.safe_dump(prepared.raw))
    fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    result = route.plan(xyz(start), xyz(goal), int(start > 19), int(goal > 19))
    assert result['route_type'] == 'partial_stair'
    assert result['edge_legs'] == [leg]*(len(result['path'])-1)
    assert result['path'][0] == xyz(start) and result['path'][-1] == xyz(goal)


def platform_fixture():
    values = scene()
    for y in range(3):
        values['data'][0, 1, 21, y] = 0.
        values['data'][3, 1, 21, y] = xyz(21)[2]
        values['data'][4, 1, 21, y] = xyz(21)[2]+2.
    settings = validate_config(config())
    settings['stair_roi'] = (np.array([-3.2, -.3, -.1]), np.array([3.2, .3, 3.1]))
    grid = TomogramMap(values)
    mask = _masked_tomogram(grid, 'stair_lower', settings)
    calls = []
    class Native:
        def plan(self, start, goal, a, b):
            calls.append((np.array(start), np.array(goal)))
            # The fixture platform is a straight three-cell strip. Production
            # passes the same exact endpoints to the real native route object.
            return dict(path=np.linspace(start, goal, 3).tolist(), layer_ids=[a, a, b],
                        algorithm='native-seam', curve_validation='fixture-platform')
    return grid, settings, lambda name, cfg: (mask, Native()), calls


def test_arbitrary_platform_endpoints_keep_landing_semantics_without_fake_height():
    grid, settings, resources, calls = platform_fixture()
    a, b = np.array(xyz(21)), np.array(xyz(21))
    a[1], b[1] = -.2, .2
    result = plan_partial_stair(grid, settings, a, b, 1, 1, resources=resources)
    assert len(calls) == 1
    assert result['direction'] == 'same_floor'
    assert result['start_region'] == result['goal_region'] == 'landing'
    assert result['edge_legs'] == ['stair_landing', 'stair_landing']
    assert result['segments'][0]['kind'] == 'landing'
    assert result['path_xyz'][0] == a.tolist() and result['path_xyz'][-1] == b.tolist()


def test_platform_native_shortcut_across_missing_support_is_rejected():
    grid, settings, resources, calls = platform_fixture()
    grid.data[0, 1, 21, 1] = 50.
    grid.valid[1, 21, 1] = False
    a, b = np.array(xyz(21)), np.array(xyz(21))
    a[1], b[1] = -.2, .2
    with pytest.raises(CrossfloorError, match='full geometry'):
        plan_partial_stair(grid, settings, a, b, 1, 1, resources=resources)


def test_wrong_native_endpoint_cannot_be_hidden_by_joining(prepared, monkeypatch):
    fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    _, native = route._resources('stair_lower', route.settings)
    original = native.plan
    def wrong(*args):
        result = original(*args)
        result['path'][0][0] += .01
        return result
    monkeypatch.setattr(native, 'plan', wrong)
    with pytest.raises(CrossfloorError, match='exact portal endpoint'):
        route.plan(xyz(15), xyz(27), 0, 1)


@pytest.mark.parametrize('reverse', [False, True])
def test_user_endpoint_inside_portal_cell_is_planned_exactly_without_tiny_connector(
        prepared, monkeypatch, reverse):
    calls = fixed_stair_native(prepared, monkeypatch)
    factory = preview._native_route
    def native(*args, **kwargs):
        value = factory(*args, **kwargs)
        original = value.plan
        def plan(start, goal, a, b):
            grid = args[0]
            assert not (a == b and np.array_equal(grid.index(start[:2]), grid.index(goal[:2])))
            result = original(start, goal, a, b)
            result['path'][0], result['path'][-1] = list(start), list(goal)
            return result
        value.plan = plan
        return value
    monkeypatch.setattr(preview, '_native_route', native)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    landing_side = xyz(21)
    landing_side[0] += .02
    start, goal = (landing_side, xyz(15)) if reverse else (xyz(15), landing_side)
    result = route.plan(start, goal, int(reverse), int(not reverse))
    assert calls == [(15, 21)]
    assert result['path'][0] == start and result['path'][-1] == goal
    assert len(result['segments']) == 1 and result['segments'][0]['name'] == 'stair_lower'
    assert result['anchors']['landing']['xyz'] == xyz(21)  # not silently moved
