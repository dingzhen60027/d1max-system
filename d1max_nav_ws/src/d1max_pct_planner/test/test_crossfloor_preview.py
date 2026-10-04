"""Offline editor keeps physical floors, directions and saved provenance explicit."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from d1max_pct_planner import crossfloor_preview as preview
from d1max_pct_planner.crossfloor_route import plan_crossfloor, validate_config
from d1max_pct_planner.tomogram_map import TomogramMap
from test_crossfloor_route import config, paths, route_factory, scene, xyz


@pytest.fixture
def prepared(tmp_path, monkeypatch):
    source = tmp_path / 'map.pcd'
    source.write_bytes(b'fixture map identity')
    map_path = tmp_path / 'map.npz'
    np.savez_compressed(map_path, **scene(source))
    raw = config()
    settings = validate_config(raw)
    tomo = TomogramMap(map_path, unknown_ceiling_policy=settings['unknown_ceiling_policy'],
                      max_ground_step_m=settings['limits']['max_ground_step_m'])
    raw.update(source_pcd=str(source), tomogram_path=str(map_path))
    route_path = tmp_path / 'route.yaml'
    route_path.write_text(yaml.safe_dump(raw))
    calls = []
    def native(masked, leg, settings, **options):
        calls.append(leg)
        class Native:
            native_runtime = {'runtime_verified': True}
            native_parameters = {'optimizer_sample_interval': 10}
            def plan(self, start, goal, a, b):
                start, goal = np.asarray(start), np.asarray(goal)
                ia, ib = masked.index(start[:2])[0], masked.index(goal[:2])[0]
                step = 1 if ib >= ia else -1
                points = [xyz(i) for i in range(ia, ib + step, step)]
                layers = [a] * len(points)
                masked.validate_path(points, layers)
                return {'path': points, 'layer_ids': layers}
        return Native()
    monkeypatch.setattr(preview, '_native_route', native)
    monkeypatch.setattr(preview, 'plan_crossfloor',
                        lambda cfg, grid, **kwargs: plan_crossfloor(cfg, grid, route_factory(paths())))
    return SimpleNamespace(raw=raw, path=route_path, tomo=tomo, source=source, calls=calls)


@pytest.mark.parametrize('layer,begin,end,name', [(0, 1, 4, 'lower'), (1, 36, 39, 'upper')])
def test_same_floor_never_routes_via_stairs(prepared, monkeypatch, layer, begin, end, name):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    prepared.calls.clear()
    def forbidden(*_):
        raise AssertionError('same-floor request called cross-floor routing')
    monkeypatch.setattr(preview, 'plan_crossfloor', forbidden)
    result = route.plan(xyz(begin), xyz(end), layer, layer)
    assert prepared.calls == [name + '_floor']
    assert result['route_type'] == 'same_floor'
    assert result['floor'] == name
    assert result['path'][0] == xyz(begin) and result['path'][-1] == xyz(end)
    assert result['layer_ids'] == [layer] * len(result['path'])
    assert result['execution_authorized'] is False


def test_reverse_route_preserves_every_edge_and_reverses_transitions(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    forward = route.plan(xyz(1), xyz(39), 0, 1)
    reverse = route.plan(xyz(39), xyz(1), 1, 0)
    assert reverse['path'] == forward['path'][::-1]
    assert reverse['layer_ids'] == forward['layer_ids'][::-1]
    assert reverse['source_layer_ids'] == forward['source_layer_ids'][::-1]
    assert reverse['edge_legs'] == forward['edge_legs'][::-1]
    assert reverse['start_xyz'] == xyz(39) and reverse['goal_xyz'] == xyz(1)
    assert reverse['anchors']['start']['xyz'] == reverse['start_xyz']
    assert reverse['anchors']['goal']['xyz'] == reverse['goal_xyz']
    assert reverse['start_layer'] == 1 and reverse['goal_layer'] == 0
    assert reverse['direction'] == 'upper_to_lower'
    assert reverse['stair_profile']['direction'] == 'down'
    assert reverse['stair_profile']['height_change_m'] == -3.
    count = len(forward['path'])
    for segment, original in zip(reverse['segments'], forward['segments'][::-1]):
        assert segment['first_index'] == count - 1 - original['last_index']
        assert segment['last_index'] == count - 1 - original['first_index']
        assert reverse['path'][segment['first_index']] == reverse['anchors'][segment['from']]['xyz']
        assert reverse['path'][segment['last_index']] == reverse['anchors'][segment['to']]['xyz']
    for event, original in zip(reverse['layer_transitions'], forward['layer_transitions'][::-1]):
        assert event['edge'] == count - 2 - original['edge']
        assert event['from_layer'] == original['to_layer']
        assert event['to_layer'] == original['from_layer']
    assert reverse['execution_authorized'] is False


def test_stair_surface_cannot_be_requested_as_a_building_floor(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    with pytest.raises(ValueError, match='起终点'):
        route.plan(xyz(15), xyz(39), 0, 1)


def test_fixed_map_cache_reuses_native_not_old_goal_results(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    first = route.plan(xyz(1), xyz(4), 0, 0)
    second = route.plan(xyz(4), xyz(2), 0, 0)
    fresh = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    expected = fresh.plan(xyz(4), xyz(2), 0, 0)
    assert second['path'] == expected['path'] != first['path']
    assert second['layer_ids'] == expected['layer_ids']
    assert second['native_map_cache']['entries'] == 1
    assert second['native_map_cache']['hits'] == 1
    assert second['native_map_cache']['misses'] == 1


def test_three_fixed_maps_share_identical_stair_resource_and_bound_memory(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    resources = [route._resources(leg[0], route.settings) for leg in preview.LEGS]
    for _ in range(50):
        for leg, existing in zip(preview.LEGS, resources):
            assert route._resources(leg[0], route.settings) is existing
    assert resources[1] is resources[2]
    assert len(route._native_maps) == 3
    assert route.cache_stats['misses'] == 3
    assert route.cache_stats['hits'] == 201
    assert route.cache_stats['max_entries'] == 3
    stats = route._resource_stats()
    assert stats['resource_keys'] == ['lower_floor', 'stairs', 'upper_floor']
    assert stats['grid_nodes_per_native_map'] == int(np.prod(route.tomogram.data.shape[1:]))
    assert stats['native_grid_nodes_total'] == 3 * stats['grid_nodes_per_native_map']
    assert stats['tomogram_numpy_bytes'] > 4 * route.tomogram.data.nbytes
    assert 'excludes_native' in stats['memory_scope']
    with pytest.raises(ValueError, match='Unknown fixed'):
        route._resources('goal_201', route.settings)


def test_warmup_materializes_three_resources_but_never_plans_a_route(prepared,monkeypatch):
    native_calls=fixed_stair_native(prepared,monkeypatch)
    route=preview.CrossfloorPreviewRoute(prepared.tomo,prepared.path)
    metrics=route.warmup_resources()
    assert metrics['native_map_cache']['entries']==3
    assert metrics['fixed_stair_cache']['entries']==0
    assert native_calls==[]
    old_resources=dict(route._native_maps)
    route.plan(xyz(1),xyz(4),0,0)
    assert native_calls==[(1,4)]
    assert all(route._native_maps[key] is value for key,value in old_resources.items())


def test_warmup_rejects_changed_configuration_without_lazy_partial_cache(prepared):
    route=preview.CrossfloorPreviewRoute(prepared.tomo,prepared.path)
    route.raw['anchors']['start']['xyz'][0]+=1.
    with pytest.raises(ValueError,match='configuration changed'):
        route.warmup_resources()
    assert route._native_maps=={} and route.cache_stats['invalidations']==1


def fixed_stair_native(prepared, monkeypatch):
    """Native seam that serves arbitrary floor endpoints and both shared stairs."""
    calls = []
    def native(masked, leg, settings, **options):
        class Native:
            native_runtime = {'runtime_verified': True}
            native_parameters = {'optimizer_sample_interval': 10}
            def plan(self, start, goal, a, b):
                ia, ib = (int(masked.index(np.asarray(point)[:2])[0]) for point in (start, goal))
                calls.append((ia, ib))
                step = 1 if ib >= ia else -1
                cells = list(range(ia, ib + step, step))
                return {'path': list(map(xyz, cells)),
                        'layer_ids': [0 if x <= 19 else 1 for x in cells],
                        'curve_validation': 'quintic_cell_boundary_roots_and_interval_interiors',
                        'quintic_segments': 1, 'curve_partition_points': len(cells)}
        return Native()
    monkeypatch.setattr(preview, '_native_route', native)
    monkeypatch.setattr(preview, 'plan_crossfloor', plan_crossfloor)
    return calls


def test_fixed_stair_cache_skips_only_two_native_calls_not_new_floor_goals(prepared, monkeypatch):
    calls = fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    first = route.plan(xyz(1), xyz(39), 0, 1)
    assert calls == [(1, 5), (5, 21), (21, 35), (35, 39)]
    # Public results must never alias the cached stair geometry.
    first['path'][10][0] = 999.
    next_ = route.plan(xyz(2), xyz(38), 0, 1)
    assert calls[4:] == [(2, 5), (35, 38)]
    assert next_['path'][0] == xyz(2) and next_['path'][-1] == xyz(38)
    assert all(abs(p[0]) < 999. for p in next_['path'])
    assert next_['fixed_stair_cache'] == dict(hits=2, misses=2, entries=2, max_entries=2)
    assert [item['fixed_leg_cache_hit'] for item in next_['segments']] == [False, True, True, False]
    assert all(item['checks']['original']['checked_cells'] > 0 for item in next_['segments'])
    reverse = route.plan(xyz(38), xyz(2), 1, 0)
    assert reverse['path'] == next_['path'][::-1]
    assert len(calls) == 8
    assert reverse['fixed_stair_cache']['hits'] == 4


def test_stair_cache_refuses_other_map_parameters_or_portals(prepared, monkeypatch):
    fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    route.plan(xyz(1), xyz(39), 0, 1)
    cache = route._fixed_stairs
    begin, end = route.settings['anchors']['entry'], route.settings['anchors']['landing']
    with pytest.raises(ValueError, match='original map/configuration'):
        cache.get(prepared.tomo, 'stair_lower', begin, end, route.settings)
    changed = deepcopy(route.settings)
    changed['planning']['astar_cost_weight'] += .1
    with pytest.raises(ValueError, match='original map/configuration'):
        cache.get(route.tomogram, 'stair_lower', begin, end, changed)
    changed = deepcopy(begin)
    changed['xyz'][0] += .1
    with pytest.raises(ValueError, match='exact configured portal'):
        cache.get(route.tomogram, 'stair_lower', changed, end, route.settings)
    # Caller corruption of a looked-up copy cannot corrupt future retrievals.
    read = cache.get(route.tomogram, 'stair_lower', begin, end, route.settings)
    read['path'][1][2] = 99.
    assert cache.get(route.tomogram, 'stair_lower', begin, end, route.settings)['path'][1] == xyz(6)


def test_stair_cache_still_revalidates_geometry_and_clears_on_any_failure(prepared, monkeypatch):
    fixed_stair_native(prepared, monkeypatch)
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    route.plan(xyz(1), xyz(39), 0, 1)
    # Deliberate private corruption proves cached routes cannot bypass checks.
    stored = next(iter(route._fixed_stairs._results.values()))
    stored['path'][1][2] += 99.
    with pytest.raises(ValueError, match='geometry validation'):
        route.plan(xyz(1), xyz(39), 0, 1)
    assert not route._native_maps and not route._fixed_stairs._results


def test_stair_cache_does_not_retain_sampled_only_curve_proof(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    cache = route._fixed_stairs
    begin, end = route.settings['anchors']['entry'], route.settings['anchors']['landing']
    cache.put(route.tomogram, 'stair_lower', begin, end, route.settings,
              {'path': [begin['xyz'], end['xyz']], 'curve_validation': 'sampled_points_only'})
    assert not cache._results


def test_numpy_memory_metadata_counts_views_and_aliases_once():
    data = np.zeros((5, 2, 3), np.float32)
    owner = SimpleNamespace(data=data, cost=data[0], valid=np.ones((2, 3), bool))
    owner.allowed = owner.valid
    assert preview._unique_array_bytes([owner, owner]) == data.nbytes + owner.valid.nbytes


def test_snapshot_is_independent_of_caller_and_read_only(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    expected = route.tomogram.data.copy()
    prepared.tomo.data[:] = 42
    np.testing.assert_array_equal(route.tomogram.data, expected)
    with pytest.raises(ValueError, match='read-only'):
        route.tomogram.data[:] = 1
    masked, _ = route._resources('lower_floor', route.settings)
    with pytest.raises(ValueError, match='read-only'):
        masked.cost[:] = 0


def test_modified_configuration_requires_new_snapshot(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    route.plan(xyz(1), xyz(4), 0, 0)
    route.raw['anchors']['entry']['xyz'][0] += .1
    with pytest.raises(ValueError, match='configuration changed'):
        route.plan(xyz(1), xyz(4), 0, 0)
    assert not route._native_maps


def test_failed_request_clears_native_state_before_next_goal(prepared, monkeypatch):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    expected = route.plan(xyz(1), xyz(4), 0, 0)
    old = route._native_maps['lower_floor'][1]
    monkeypatch.setattr(old, 'plan', lambda *args: (_ for _ in ()).throw(ValueError('dirty failed A*')))
    with pytest.raises(ValueError, match='dirty failed'):
        route.plan(xyz(1), xyz(4), 0, 0)
    assert not route._native_maps
    actual = route.plan(xyz(1), xyz(4), 0, 0)
    assert actual['path'] == expected['path']
    assert route._native_maps['lower_floor'][1] is not old
    assert route.cache_stats['invalidations'] == 1


def test_snapshot_rejects_overlapping_native_requests(prepared):
    route = preview.CrossfloorPreviewRoute(prepared.tomo, prepared.path)
    route._planning_lock.acquire()
    try:
        with pytest.raises(RuntimeError, match='active request'):
            route.plan(xyz(1), xyz(4), 0, 0)
    finally:
        route._planning_lock.release()


def test_visible_surfaces_filters_intermediate_ceiling_without_mutating_data():
    settings = {'floor_z_ranges': {'lower': [-.8, -.5], 'upper': [3., 3.3]},
                'stair_roi': (np.array([-2., 10., -.8]), np.array([2., 20., 3.3]))}
    points = np.array([[100., 100., -.65], [100., 100., 3.15],
                       [100., 100., 1.7], [0., 15., 1.7], [0., 25., 1.7],
                       [0., 15., 5.], [0., 15., -2.]])
    before = points.copy()
    mask = preview.visible_surfaces(points, settings)
    np.testing.assert_array_equal(mask, [True, True, False, True, False, False, False])
    np.testing.assert_array_equal(points, before)
    assert preview.visible_surfaces(np.empty((0, 3)), settings).shape == (0,)


def test_coordinator_rejects_different_tomogram_or_changed_source(prepared):
    prepared.raw['tomogram_path'] = str(prepared.path.parent / 'other.npz')
    prepared.path.write_text(yaml.safe_dump(prepared.raw))
    with pytest.raises(ValueError, match='same tomogram'):
        preview.load_config(prepared.path, prepared.tomo)
    prepared.raw['tomogram_path'] = str(prepared.tomo.source)
    prepared.path.write_text(yaml.safe_dump(prepared.raw))
    prepared.source.write_bytes(b'changed source')
    with pytest.raises(ValueError, match='hash|source|PCD'):
        preview.load_config(prepared.path, prepared.tomo)


@pytest.mark.parametrize('policy,step,error', [
    ('reject', .17, 'unknown_ceiling_policy mismatch'),
    ('allow_unobserved', .15, 'max_ground_step_m mismatch'),
    ('allow_unobserved', .20, 'max_ground_step_m mismatch'),
])
def test_runtime_map_policy_must_match_route_before_native_allocation(prepared, policy, step, error):
    other = TomogramMap(prepared.tomo.source, unknown_ceiling_policy=policy,
                        max_ground_step_m=step)
    assert other.sha256 == prepared.tomo.sha256  # Same artifact is insufficient.
    with pytest.raises(ValueError, match=error):
        preview.CrossfloorPreviewRoute(other, prepared.path)
    assert prepared.calls == []


def test_changed_route_policy_is_not_silently_applied_to_an_old_map_mask(prepared):
    prepared.raw['unknown_ceiling_policy'] = 'reject'
    prepared.path.write_text(yaml.safe_dump(prepared.raw))
    with pytest.raises(ValueError, match='unknown_ceiling_policy mismatch'):
        preview.load_config(prepared.path, prepared.tomo)
    matching = TomogramMap(prepared.tomo.source, unknown_ceiling_policy='reject',
                           max_ground_step_m=.17)
    _, settings = preview.load_config(prepared.path, matching)
    assert settings['unknown_ceiling_policy'] == 'reject'


def test_headroom_remains_map_owned_and_exactly_preserved_without_new_default(prepared):
    custom = TomogramMap(prepared.tomo.source, minimum_headroom_m=.87,
                         unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    route = preview.CrossfloorPreviewRoute(custom, prepared.path)
    masked, _ = route._resources('lower_floor', route.settings)
    assert route.tomogram.minimum_headroom_m == .87
    assert masked.minimum_headroom_m == .87
    assert route.tomogram.max_ground_step_m == masked.max_ground_step_m == .17


def saved_record(prepared):
    result = plan_crossfloor(prepared.raw, prepared.tomo, route_factory(paths()))
    result.update(config_sha256=hashlib.sha256(prepared.path.read_bytes()).hexdigest())
    for segment in result['segments']:
        if segment['name'].startswith('stair_'):
            segment.update(curve_validation='quintic_cell_boundary_roots_and_interval_interiors',
                           quintic_segments=4, curve_partition_points=20)
        else:
            segment.update(curve_validation='polynomial_cell_boundary_roots_and_interval_interiors_plus_validate_path',
                           quintic_segments=0, curve_partition_points=20)
    return result


@pytest.mark.parametrize('key,value', [('config_sha256', 'different'), ('tomogram_sha256', 'different'),
                                     ('source_pcd_sha256', 'different'), ('frame_id', 'wrong'),
                                     ('status', 'candidate_only')])
def test_saved_route_rejects_stale_provenance(prepared, key, value):
    record = saved_record(prepared)
    record[key] = value
    audit = prepared.path.parent / 'audit.json'
    audit.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='does not match'):
        preview.restore_crossfloor(prepared.tomo, prepared.path, audit)


def test_saved_exact_map_accepts_both_complete_curve_proofs(prepared):
    record = saved_record(prepared)
    audit = prepared.path.parent / 'audit.json'
    audit.write_text(json.dumps(record))
    result = preview.restore_crossfloor(prepared.tomo, prepared.path, audit)
    assert result['path'] == record['path_xyz']
    assert result['route_origin'] == 'saved_crossfloor_route_exact_map_revalidated'
    assert result['execution_authorized'] is False
    record['segments'][1]['curve_validation'] = 'sampled_points_only'
    audit.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='validation evidence'):
        preview.restore_crossfloor(prepared.tomo, prepared.path, audit)


@pytest.mark.parametrize('tamper', ['coverage_gap', 'wrong_portal', 'wrong_edge_identity'])
def test_saved_route_rechecks_segment_coverage_and_physical_portals(prepared, tamper):
    record = saved_record(prepared)
    if tamper == 'coverage_gap':
        record['segments'][1]['first_index'] += 1
    elif tamper == 'wrong_portal':
        record['segments'][1]['from'] = 'exit'
    else:
        record['edge_legs'][record['segments'][1]['first_index']] = 'upper_floor'
    audit = prepared.path.parent / 'audit.json'
    audit.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='segment order/coverage|edge identities'):
        preview.restore_crossfloor(prepared.tomo, prepared.path, audit)
