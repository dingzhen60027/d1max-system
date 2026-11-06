"""Validate a bounded, hash-bound stair artifact without inventing observations.

The package loader owns file hashes and original-cloud lineage. This module
checks the internal evidence relations once at package initialization; it does
not fit a new surface, change a cloud, or participate in a control callback.
Returned rows are literal measured support, NOT free space. The caller must
intersect them with the separately validated retained-entity/cost witnesses
before supplying additional support to SCAN. Model grid samples are never
returned. Existing evidence stores entity indices, not their coordinates, so
the entity mask's physical truth is deliberately not inferred here.
"""
from collections import deque

import numpy as np
from scipy.spatial import cKDTree


SCHEMA = 'd1max.structured_stair_support.v1'
_MAX_SOURCE = 1_000_000
_MAX_GRID = 500_000
_MAX_FLIGHT_SOURCE = 250_000
_MAX_NEIGHBOUR_VISITS = 8_000_000


def _require(condition, reason):
    if not condition:
        raise ValueError('structured_stair_evidence_' + reason)


def _array(arrays, name, shape, kind, *, finite=True):
    _require(name in arrays, 'missing_' + name)
    value = np.asarray(arrays[name])
    _require(value.shape == shape and value.dtype.kind in kind, 'invalid_' + name)
    if finite:
        _require(np.isfinite(value).all(), 'nonfinite_' + name)
    return value


def _vector(value, size, name):
    result = np.asarray(value, dtype=np.float64)
    _require(result.shape == (size,) and np.isfinite(result).all(), 'invalid_' + name)
    return result


def _connected(points, anchors, radius):
    """Bound each neighbourhood before allocation, as in the offline builder."""
    _require(len(points) <= _MAX_FLIGHT_SOURCE and anchors.any(), 'connectivity_anchor_or_budget')
    tree = cKDTree(points)
    reached = anchors.copy()
    queue = deque(np.flatnonzero(anchors).tolist())
    visits = 0
    while queue:
        row = queue.popleft()
        visits += int(tree.query_ball_point(points[row], radius, return_length=True))
        _require(visits <= _MAX_NEIGHBOUR_VISITS, 'connectivity_budget')
        for other in tree.query_ball_point(points[row], radius):
            if not reached[other]:
                reached[other] = True
                queue.append(other)
    _require(reached.all(), 'unanchored_source_component')


def validate_structured_evidence(model, arrays, mapping):
    """Return an independent copy of exact source XYZ, never fitted grid XYZ.

    ``mapping`` is the already source/hash-validated StairGroundMapping. Its
    surface 0 denotes historical unmodified observed support, not a foreign
    floor. Those rows may be classified by a flight only after all geometric
    tests below. Nonzero foreign owners and unanchored components are rejected.
    New planning XYZ in this evidence is an offline *proposal*: the package's
    slice-preserving final mapping may retain its original Z. It is not used
    as measured ground here.
    """
    try:
        return _validate(model, arrays, mapping)
    except (KeyError, TypeError, AttributeError, IndexError, OverflowError) as error:
        raise ValueError('structured_stair_evidence_malformed') from error


def _validate(model, arrays, mapping):
    _require(isinstance(model, dict) and model.get('schema') == SCHEMA,
             'schema')
    _require(model.get('source_records_are_literal') is True
             and model.get('model_samples_are_observations') is False
             and model.get('original_source_modified') is False
             and model.get('runtime_collision_authority_unchanged') is True,
             'semantics')
    _require(model.get('source_sha256') == mapping.source_hash, 'source_identity')
    policy = model['policy']
    names = ('grid_spacing_m', 'source_support_radius_m', 'wall_boundary_guard_m',
             'maximum_support_residual_m', 'support_neighbor_m', 'endpoint_blend_m',
             'retained_entity_height_m', 'retained_entity_top_m', 'retained_entity_radius_m')
    _require(all(not isinstance(policy[name], bool) and np.isfinite(policy[name])
                 and policy[name] > 0 for name in names), 'policy')
    _require(policy['maximum_support_residual_m'] <= .08
             and policy['source_support_radius_m'] <= .12
             and policy['support_neighbor_m'] <= .16
             and policy['retained_entity_height_m'] >= policy['maximum_support_residual_m']
             and policy['retained_entity_top_m'] > policy['retained_entity_height_m']
             and type(policy['maximum_grid_points']) is int
             and 0 < policy['maximum_grid_points'] <= _MAX_GRID, 'policy')

    ids = np.asarray(arrays['source_indices'])
    grid = np.asarray(arrays['grid_model_xyz'])
    _require(ids.ndim == 1 and ids.dtype.kind in 'iu' and 0 < len(ids) <= _MAX_SOURCE
             and np.all(ids >= 0) and len(np.unique(ids)) == len(ids), 'source_indices')
    _require(grid.ndim == 2 and grid.shape[1:] == (3,) and grid.dtype.kind == 'f'
             and 0 < len(grid) <= _MAX_GRID and np.isfinite(grid).all(), 'grid_geometry')
    n, g = len(ids), len(grid)
    xyz = _array(arrays, 'source_xyz', (n, 3), 'f')
    proposed = _array(arrays, 'planning_xyz', (n, 3), 'f')
    owner = _array(arrays, 'surface_id', (n,), 'iu')
    normal = _array(arrays, 'source_normal_z', (n,), 'f')
    rms = _array(arrays, 'source_plane_rms_m', (n,), 'f')
    immutable = _array(arrays, 'immutable_existing_mapping', (n,), 'b')
    new = _array(arrays, 'source_is_new_mapping_row', (n,), 'b')
    grid_owner = _array(arrays, 'grid_surface_id', (g,), 'iu')
    nearest = _array(arrays, 'grid_nearest_source_index', (g,), 'iu')
    distance = _array(arrays, 'grid_source_distance_m', (g,), 'f')
    covered = _array(arrays, 'grid_source_covered', (g,), 'b')
    blocked = _array(arrays, 'grid_retained_entity_blocked', (g,), 'b')
    available = _array(arrays, 'grid_available', (g,), 'b')
    _require(np.array_equal(immutable, ~new), 'mapping_age_flags')
    _require(np.all((normal >= .70) & (normal <= 1.))
             and np.all((rms >= 0.) & (rms <= .035)), 'source_plane_evidence')
    _require(np.array_equal(proposed[:, :2], xyz[:, :2])
             and np.all(abs(proposed[:, 2]-xyz[:, 2]) <= .08 + 1e-9), 'planning_proposal')
    _require(np.all(distance >= 0.) and np.array_equal(available, covered & ~blocked),
             'availability_relation')

    order = np.argsort(mapping.source_indices)
    offsets = np.searchsorted(mapping.source_indices[order], ids)
    _require(np.all(offsets < len(order)), 'source_not_in_mapping')
    rows = order[offsets]
    _require(np.array_equal(mapping.source_indices[rows], ids)
             and np.array_equal(mapping.source_xyz[rows], xyz), 'source_not_literal_mapping')
    mapped_owner = np.asarray(mapping.surface_ids)[rows]
    _require(np.all((mapped_owner == owner) | (mapped_owner == 0)), 'foreign_mapping_owner')
    _require(np.array_equal(mapping.planning_xyz[rows[immutable]], proposed[immutable]),
             'immutable_mapping_changed')

    flights = model['flights']
    _require(isinstance(flights, list) and 0 < len(flights) <= 64, 'flight_count')
    flight_ids = [flight['surface_id'] for flight in flights]
    _require(all(type(identity) is int and identity > 0 for identity in flight_ids)
             and len(set(flight_ids)) == len(flight_ids)
             and set(np.unique(owner)) == set(flight_ids)
             and set(np.unique(grid_owner)) == set(flight_ids), 'surface_ownership')

    for flight in flights:
        identity = flight['surface_id']
        selected, sampled = owner == identity, grid_owner == identity
        points, samples = xyz[selected], grid[sampled]
        frame = flight['frame']
        center = _vector(frame['center_xy'], 2, 'frame_center')
        axis = _vector(frame['uphill_axis_xy'], 2, 'frame_axis')
        bounds = _vector(frame['along_range_m'], 2, 'frame_bounds')
        _require(abs(np.linalg.norm(axis)-1.) <= 1e-6 and bounds[0] < bounds[1], 'frame')
        lateral = np.array([-axis[1], axis[0]])
        height = flight['height_model']
        grade, intercept = height['grade'], height['intercept_m']
        _require(height['kind'] == 'planning_support_envelope_not_physical_treads'
                 and np.isfinite([grade, intercept]).all() and .15 <= grade <= 1., 'height_model')
        sides = flight['retained_structure']['sides']
        negative = _vector(sides['negative']['lateral_slope_intercept'], 2, 'negative_side')
        positive = _vector(sides['positive']['lateral_slope_intercept'], 2, 'positive_side')
        guard = policy['wall_boundary_guard_m']
        _require(flight['wall_boundary_guard_m'] == guard, 'boundary_guard')
        s, t = (points[:, :2]-center)@axis, (points[:, :2]-center)@lateral
        _require(np.all((s >= bounds[0]-1e-9) & (s <= bounds[1]+1e-9)
                        & (t > np.polyval(negative, s)+guard-1e-9)
                        & (t < np.polyval(positive, s)-guard+1e-9)
                        & (abs(points[:, 2]-(grade*s+intercept))
                           <= policy['maximum_support_residual_m']+1e-9)), 'source_outside_flight')
        anchors = immutable[selected] & (mapped_owner[selected] == identity)
        connectivity = flight['connectivity']
        _require(connectivity['policy'] == 'same_surface_measured_3d_component_with_existing_support_anchor'
                 and connectivity['neighbour_radius_m'] == policy['support_neighbor_m']
                 and connectivity['connected_points'] == len(points)
                 and connectivity['anchor_points'] == int(anchors.sum()), 'connectivity_report')
        _connected(points, anchors, policy['support_neighbor_m'])

        # Regenerate only the small, explicit model grid, not a PCD or its fit.
        _require((bounds[1]-bounds[0])/policy['grid_spacing_m']+1
                 <= policy['maximum_grid_points'], 'grid_sampling_budget')
        stations = np.r_[np.arange(bounds[0], bounds[1], policy['grid_spacing_m']), bounds[1]]
        lo, hi = np.polyval(negative, stations)+guard, np.polyval(positive, stations)-guard
        _require(np.all(lo < hi), 'crossing_sides')
        count_values = np.ceil((hi-lo)/policy['grid_spacing_m'])+1
        _require(np.isfinite(count_values).all() and count_values.sum() <= policy['maximum_grid_points'],
                 'grid_sampling_budget')
        counts = count_values.astype(np.int64)
        _require(int(counts.sum()) == len(samples)
                 and len(samples) <= policy['maximum_grid_points'], 'grid_sampling_count')
        st = np.concatenate([np.c_[np.full(count, station), np.linspace(low, high, count)]
                             for station, low, high, count in zip(stations, lo, hi, counts)])
        expected = np.c_[center+st[:, :1]*axis+st[:, 1:]*lateral, grade*st[:, 0]+intercept]
        _require(np.allclose(samples, expected, atol=1e-9, rtol=0), 'grid_not_declared_model')
        measured_ids = ids[selected]
        sorted_rows = np.argsort(measured_ids)
        positions = np.searchsorted(measured_ids[sorted_rows], nearest[sampled])
        _require(np.all(positions < len(sorted_rows)), 'grid_nearest_foreign_source')
        witness = sorted_rows[positions]
        _require(np.array_equal(measured_ids[witness], nearest[sampled]), 'grid_nearest_foreign_source')
        shortest = cKDTree(points).query(samples, workers=1)[0]
        explicit = np.linalg.norm(samples-points[witness], axis=1)
        _require(np.allclose(distance[sampled], shortest, atol=1e-9, rtol=0)
                 and np.allclose(explicit, shortest, atol=1e-9, rtol=0), 'grid_nearest_distance')
        _require(np.array_equal(covered[sampled], shortest <= policy['source_support_radius_m']),
                 'grid_coverage')
        counts_to_check = dict(connected_support_points=int(selected.sum()),
            new_source_rows=int(new[selected].sum()), immutable_existing_mapping_rows=int(immutable[selected].sum()),
            model_samples=int(sampled.sum()), observed_supported_samples=int(covered[sampled].sum()),
            excluded_by_retained_entities=int((covered[sampled] & blocked[sampled]).sum()),
            available_model_samples=int(available[sampled].sum()))
        _require(all(flight[key] == count for key, count in counts_to_check.items()), 'count_report')
    return xyz.copy()
