"""Offline stair support domains bounded by retained walls, not old PCT costs.

Two representations are deliberately separate: ``source_*`` are literal
observed records and ``grid_model_xyz`` is a planning model sampled between
measured sides. A model sample is never an observation or a free-space claim.
No input cloud is modified and this module has no runtime/SCAN dependency.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from scipy.spatial import cKDTree

from .stair_regularization import (RegularizationLimits, StairRegularization,
                                  _surface_normals, connected_measured_support)


@dataclass(frozen=True)
class StructuredSupportPolicy:
    grid_spacing_m: float = .05
    source_support_radius_m: float = .12
    wall_boundary_guard_m: float = .06
    maximum_support_residual_m: float = .08
    support_neighbor_m: float = .16
    endpoint_blend_m: float = .15
    retained_entity_height_m: float = .08
    retained_entity_top_m: float = .70
    retained_entity_radius_m: float = .05
    maximum_grid_points: int = 250_000

    def __post_init__(self):
        values = asdict(self)
        if (any(isinstance(v, bool) or not np.isfinite(v) or v <= 0 for v in values.values())
                or self.maximum_support_residual_m > .08
                or self.source_support_radius_m > .12
                or self.support_neighbor_m > .16
                or self.retained_entity_height_m < self.maximum_support_residual_m
                or self.retained_entity_top_m <= self.retained_entity_height_m
                or type(self.maximum_grid_points) is not int):
            raise ValueError('invalid_structured_stair_policy')


def _cloud(value, label):
    array = np.asarray(value, dtype=np.float64)
    if array.ndim != 2 or array.shape[1:] != (3,) or not np.isfinite(array).all():
        raise ValueError(label + '_must_be_finite_xyz')
    return array


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _mapping(value, source):
    required = {'source_indices', 'source_xyz', 'planning_xyz', 'surface_id'}
    if not isinstance(value, dict) or not required <= set(value):
        raise ValueError('structured_stair_existing_mapping_missing')
    ids = np.asarray(value['source_indices'])
    original = _cloud(value['source_xyz'], 'existing_source')
    planning = _cloud(value['planning_xyz'], 'existing_planning')
    owners = np.asarray(value['surface_id'])
    if (ids.ndim != 1 or ids.dtype.kind not in 'iu' or owners.dtype.kind not in 'iu'
            or owners.shape != ids.shape or np.any(owners < 0)
            or len(np.unique(ids)) != len(ids) or np.any(ids < 0) or np.any(ids >= len(source))
            or original.shape != (len(ids), 3) or planning.shape != original.shape
            or not np.array_equal(original, source[ids])
            or not np.array_equal(original[:, :2], planning[:, :2])):
        raise ValueError('structured_stair_existing_mapping_identity')
    return ids, original, planning, owners


def _frame(model):
    center, axis = np.asarray(model['center_xy'], float), np.asarray(model['uphill_axis_xy'], float)
    bounds = np.asarray(model['along_range_m'], float)
    if (center.shape != (2,) or axis.shape != (2,) or bounds.shape != (2,)
            or not np.isfinite(np.r_[center, axis, bounds, model['grade'], model['intercept_m']]).all()
            or abs(np.linalg.norm(axis)-1.) > 1e-6 or bounds[0] >= bounds[1]
            or not .15 <= model['grade'] <= 1.):
        raise ValueError('structured_stair_flight_model_invalid')
    return center, axis, np.array([-axis[1], axis[0]]), bounds


def _grid(center, axis, lateral, bounds, negative, positive, model, policy):
    # Sampling is only a serializable model, not a point-cloud insertion.
    stations = np.r_[np.arange(bounds[0], bounds[1], policy.grid_spacing_m), bounds[1]]
    low = np.polyval(negative, stations) + policy.wall_boundary_guard_m
    high = np.polyval(positive, stations) - policy.wall_boundary_guard_m
    if np.any(low >= high):
        raise ValueError('structured_stair_sides_cross_or_no_interior')
    counts = np.ceil((high-low)/policy.grid_spacing_m).astype(int)+1
    if counts.sum() > policy.maximum_grid_points:
        raise ValueError('structured_stair_grid_work_limit')
    st = np.concatenate([np.c_[np.full(n, s), np.linspace(a, b, n)]
                         for s, a, b, n in zip(stations, low, high, counts)])
    xy = center + st[:, :1] * axis + st[:, 1:] * lateral
    xyz = np.c_[xy, model['grade']*st[:, 0]+model['intercept_m']]
    return xyz, st


def build_structured_stair_support(source_xyz, planning_xyz, retained_source_indices,
        existing_mapping, models, *, source_sha256, planning_sha256,
        policy=None, measured_flights=None):
    """Return a structural prior and separately auditable literal support rows.

    ``retained_source_indices`` corresponds to the original-record prefix of
    the current planning PCD. Extra measured-restoration rows may follow it;
    they are considered entities but never given an invented original index.
    Every existing mapping row stays immutable. Only previously unmapped
    retained literal source points enter ``addition``; the caller must merge,
    preserve PCT layer membership and revalidate the map.
    Flight structure extraction uses current retained PCD (not removed ghosts).
    """
    p = policy or StructuredSupportPolicy()
    if not isinstance(p, StructuredSupportPolicy) or not _digest(source_sha256) or not _digest(planning_sha256):
        raise ValueError('structured_stair_identity_or_policy_invalid')
    source, planning = _cloud(source_xyz, 'source'), _cloud(planning_xyz, 'planning')
    retained = np.asarray(retained_source_indices)
    if (retained.ndim != 1 or retained.dtype.kind not in 'iu' or len(retained) > len(planning)
            or np.any(retained < 0) or np.any(retained >= len(source))
            or len(np.unique(retained)) != len(retained)
            or not np.array_equal(planning[:len(retained), :2], source[retained, :2])):
        raise ValueError('structured_stair_record_lineage_invalid')
    old_ids, old_source, old_planning, old_owner = _mapping(existing_mapping, source)
    owner = np.full(len(source), -1, np.int32)
    owner[old_ids] = old_owner
    immutable = np.zeros(len(source), bool)
    immutable[old_ids] = True
    to_planning = np.full(len(source), -1, np.int64)
    to_planning[retained] = np.arange(len(retained))
    # Bind the prefix to the original records or its existing mapping. A
    # foreign planning Z may not silently become the input for another edit.
    expected = source[retained].copy()
    present = to_planning[old_ids] >= 0
    expected[to_planning[old_ids[present]]] = old_planning[present]
    if not np.allclose(planning[:len(retained)], expected, rtol=0, atol=2e-6):
        # Other floor-conditioning edits are valid but cannot be mistaken for
        # literal stair records. Only the selected per-flight source rows are
        # later required to be equal; preserve other floors untouched.
        literal = np.max(abs(planning[:len(retained)]-expected), axis=1) <= 2e-6
    else:
        literal = np.ones(len(retained), bool)
    from d1max_pct_planner.stair_centering import extract_measured_flight_structure, fit_measured_sides
    flights = (extract_measured_flight_structure(planning, models)
               if measured_flights is None else measured_flights)
    if not flights or len({f['id'] for f in flights}) != len(flights):
        raise ValueError('structured_stair_flight_ownership_invalid')
    known_models = {int(m['id']): m for m in models}
    if len(known_models) != len(models):
        raise ValueError('structured_stair_duplicate_model_identity')
    for flight in flights:
        reference = known_models.get(int(flight['id']))
        rows = np.asarray(flight.get('structure_source_indices'))
        fixed = ('center_xy', 'uphill_axis_xy', 'along_range_m', 'grade', 'intercept_m')
        if (reference is None or reference.get('kind') != 'flight_support_envelope'
                or any(not np.array_equal(flight.get(key), reference.get(key)) for key in fixed)
                or rows.ndim != 1 or rows.dtype.kind not in 'iu'
                or np.any(rows < 0) or np.any(rows >= len(planning))
                or len(np.unique(rows)) != len(rows)
                or not np.array_equal(_cloud(flight.get('structure_xyz'), 'retained_structure'), planning[rows])):
            raise ValueError('structured_stair_retained_structure_identity')
    arrays, reports, claimed = [], [], set()
    widened = {int(model['id']): dict(model) for model in models}
    limits = RegularizationLimits(maximum_displacement_m=p.maximum_support_residual_m,
                                  support_neighbor_m=p.support_neighbor_m)
    for flight in flights:
        surface = int(flight['id'])
        center, axis, lateral, bounds = _frame(flight)
        fit = fit_measured_sides(flight)
        negative = fit['sides']['negative']['lateral_slope_intercept']
        positive = fit['sides']['positive']['lateral_slope_intercept']
        xy = source[retained, :2]-center
        s, t = xy@axis, xy@lateral
        expected_z = flight['grade']*s+flight['intercept_m']
        low = np.polyval(negative, s)+p.wall_boundary_guard_m
        high = np.polyval(positive, s)-p.wall_boundary_guard_m
        pad = limits.maximum_neighbor_radius_m * 2
        region = ((s >= bounds[0]-pad) & (s <= bounds[1]+pad)
                  & (t >= low-pad) & (t <= high+pad)
                  & (abs(source[retained, 2]-expected_z) <= .7+pad))
        rows, ids = np.flatnonzero(region), retained[region]
        xyz = source[ids]
        if len(rows) < limits.normal_neighbors:
            raise ValueError('structured_stair_source_region_too_small')
        normal_ok, normal_z, rms = _surface_normals(xyz, limits)
        eligible = (normal_ok & literal[rows] & (s[rows] >= bounds[0]) & (s[rows] <= bounds[1])
                    & (t[rows] > low[rows]) & (t[rows] < high[rows])
                    & (abs(xyz[:, 2]-expected_z[rows]) <= p.maximum_support_residual_m)
                    & ((owner[ids] == -1) | (owner[ids] == 0) | (owner[ids] == surface)))
        reached, connected = connected_measured_support(xyz, eligible,
            eligible & (owner[ids] == surface), p.support_neighbor_m)
        support_ids, support_xyz = ids[reached], xyz[reached]
        if not len(support_ids):
            raise ValueError('structured_stair_no_connected_source_support')
        if claimed.intersection(support_ids):
            raise ValueError('structured_stair_cross_flight_support_alias')
        claimed.update(support_ids)
        target = expected_z[rows][reached]
        edge = np.minimum(s[rows][reached]-bounds[0], bounds[1]-s[rows][reached])
        blend = np.clip(edge/p.endpoint_blend_m, 0, 1)
        blend = blend*blend*(3-2*blend)
        proposed = support_xyz.copy()
        proposed[:, 2] += (target-support_xyz[:, 2])*blend
        frozen = immutable[support_ids]
        proposed[frozen] = planning[to_planning[support_ids[frozen]]]
        model_xyz, st = _grid(center, axis, lateral, bounds, negative, positive, flight, p)
        distance, nearest = cKDTree(support_xyz).query(model_xyz, workers=1)
        source_covered = distance <= p.source_support_radius_m
        # Retained elevated returns stay exclusions, including unclassified
        # low entities. Never use the original (pre-cleanup) cloud as obstacles.
        rel = planning[:, :2]-center
        ps, pt = rel@axis, rel@lateral
        dz = planning[:, 2]-(flight['grade']*ps+flight['intercept_m'])
        entity = ((ps >= bounds[0]) & (ps <= bounds[1])
                  & (pt >= np.polyval(negative, ps)+p.wall_boundary_guard_m)
                  & (pt <= np.polyval(positive, ps)-p.wall_boundary_guard_m)
                  & (dz > p.retained_entity_height_m) & (dz <= p.retained_entity_top_m))
        entity_rows = np.flatnonzero(entity)
        entity_distance = (cKDTree(planning[entity, :2]).query(model_xyz[:, :2], workers=1)[0]
                           if entity.any() else np.full(len(model_xyz), np.inf))
        entity_blocked = entity_distance <= p.retained_entity_radius_m
        available = source_covered & ~entity_blocked
        legacy = flight.get('interior_lateral_range_m', [np.nan, np.nan])
        old_band = (st[:, 1] > legacy[0]) & (st[:, 1] < legacy[1])
        corrected = np.any(proposed != source[support_ids], axis=1)
        new_ids = ~np.isin(support_ids, old_ids)
        side_at_bounds = (np.polyval(negative, bounds)+p.wall_boundary_guard_m,
                          np.polyval(positive, bounds)-p.wall_boundary_guard_m)
        corners = np.array([center+u*axis+v*lateral for u, lo, hi in zip(
            bounds, *side_at_bounds) for v in (lo, hi)])
        expanded = dict(widened[surface])
        expanded['interior_lateral_range_m'] = [float(max(side_at_bounds[0])), float(min(side_at_bounds[1]))]
        expanded['scope_minimum'] = [*np.minimum(corners.min(axis=0), flight['scope_minimum'][:2]).tolist(), flight['scope_minimum'][2]]
        expanded['scope_maximum'] = [*np.maximum(corners.max(axis=0), flight['scope_maximum'][:2]).tolist(), flight['scope_maximum'][2]]
        expanded['structural_boundary'] = dict(negative_slope_intercept=negative,
            positive_slope_intercept=positive, inward_guard_m=p.wall_boundary_guard_m,
            source='retained_planning_pcd_vertical_returns', planning_sha256=planning_sha256)
        widened[surface] = expanded
        reports.append(dict(surface_id=surface, leg=flight['leg'],
            frame=dict(center_xy=center.tolist(), uphill_axis_xy=axis.tolist(), along_range_m=bounds.tolist()),
            height_model=dict(kind='planning_support_envelope_not_physical_treads', grade=flight['grade'], intercept_m=flight['intercept_m']),
            retained_structure=dict(**fit, planning_indices=np.asarray(flight['structure_source_indices'], int).tolist()),
            wall_boundary_guard_m=p.wall_boundary_guard_m,
            width_after_guard_at_endpoints_m=(np.asarray(fit['width_at_endpoints_m'])-2*p.wall_boundary_guard_m).tolist(),
            previous_support_width_m=float(legacy[1]-legacy[0]),
            connected_support_points=len(support_ids), new_source_rows=int(new_ids.sum()),
            immutable_existing_mapping_rows=int(frozen.sum()),
            proposed_original_z_changes=int(corrected.sum()),
            maximum_proposed_displacement_m=float(abs(proposed[:, 2]-support_xyz[:, 2]).max()),
            model_samples=len(model_xyz), observed_supported_samples=int(source_covered.sum()),
            excluded_by_retained_entities=int((source_covered & entity_blocked).sum()),
            available_model_samples=int(available.sum()), available_outside_previous_band=int((available & ~old_band).sum()),
            retained_entity_planning_indices=entity_rows.tolist(), connectivity=connected,
            grid_semantics='explicit_planning_prior_with_literal_source_witness_not_observations_or_freespace'))
        arrays.append(dict(source_indices=support_ids.astype(np.int64), source_xyz=support_xyz,
            planning_xyz=proposed, surface_id=np.full(len(support_ids), surface, np.int32),
            source_normal_z=normal_z[reached], source_plane_rms_m=rms[reached],
            immutable_existing_mapping=frozen, source_is_new_mapping_row=new_ids,
            grid_model_xyz=model_xyz, grid_surface_id=np.full(len(model_xyz), surface, np.int32),
            grid_nearest_source_index=support_ids[nearest].astype(np.int64),
            grid_source_distance_m=distance, grid_source_covered=source_covered,
            grid_retained_entity_blocked=entity_blocked, grid_available=available))
    merged = {key: np.concatenate([a[key] for a in arrays]) for key in arrays[0]}
    additions = merged['source_is_new_mapping_row']
    added = StairRegularization(merged['source_indices'][additions].copy(),
        merged['source_xyz'][additions].copy(), merged['planning_xyz'][additions].copy(),
        merged['surface_id'][additions].copy(), tuple(widened.values()),
        dict(additional_points=int(additions.sum()),
             changed_points=int(np.any(merged['source_xyz'][additions] != merged['planning_xyz'][additions], axis=1).sum()),
             fixed_height_models_unchanged=True, existing_mapping_rows_unchanged=True,
             selected_surface_ids=[int(f['id']) for f in flights],
             synthetic_points=0, removed_points=0, xy_changed_points=0,
             boundary_source='retained_planning_pcd_walls_and_rails_not_pct_valid_mask'))
    return dict(addition=added, widened_models=list(widened.values()),
        model=dict(schema='d1max.structured_stair_support.v1', source_sha256=source_sha256,
        planning_sha256=planning_sha256, policy=asdict(p), flights=reports,
        source_records_are_literal=True, model_samples_are_observations=False,
        original_source_modified=False, planning_cloud_modified=False,
        physical_tread_dimensions_recovered=False, runtime_collision_authority_unchanged=True), arrays=merged)
