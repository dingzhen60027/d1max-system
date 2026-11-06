"""Observed stair-support regularization for a *planning-only* PCD derivative.

This module does not reconstruct a staircase from a nominal rise/run.  It fits
separate flight support envelopes and a landing to measured, classified support.
Only bounded Z corrections of existing surface records are returned.  Wall and
railing surfaces, unobserved cells and the localization PCD are not replaced.
The caller must persist the source-row mapping: these idealized planning points
must never be sent to SCAN as if they were live or original observations.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from typing import Literal, Sequence

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class StairSurface:
    id: int
    name: str
    kind: Literal['flight_support_envelope', 'landing_plane']
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    def mask(self, xyz):
        low, high = np.asarray(self.minimum, float), np.asarray(self.maximum, float)
        if (self.id < 0 or not self.name or self.kind not in
                ('flight_support_envelope', 'landing_plane') or low.shape != (3,)
                or high.shape != (3,) or not np.isfinite([low, high]).all()
                or np.any(low >= high)):
            raise ValueError('invalid_stair_surface')
        return np.all((xyz >= low) & (xyz < high), axis=1)


@dataclass(frozen=True)
class RegularizationLimits:
    maximum_displacement_m: float = .08
    support_neighbor_m: float = .16
    normal_neighbors: int = 12
    minimum_normal_z: float = .70
    maximum_plane_rms_m: float = .035
    maximum_neighbor_radius_m: float = .20
    minimum_fit_points: int = 40
    minimum_profile_bins: int = 6
    lateral_edge_guard_m: float = .06
    maximum_fit_p95_m: float = .07
    boundary_blend_m: float = .15

    def __post_init__(self):
        values = (self.maximum_displacement_m, self.support_neighbor_m,
                  self.minimum_normal_z, self.maximum_plane_rms_m,
                  self.maximum_neighbor_radius_m, self.lateral_edge_guard_m,
                  self.maximum_fit_p95_m, self.boundary_blend_m)
        if (not np.isfinite(values).all() or min(values) < 0
                or not 0 < self.maximum_displacement_m <= .15
                or not 0 < self.minimum_normal_z <= 1
                or self.minimum_fit_points < 12 or self.normal_neighbors < 6
                or self.minimum_profile_bins < 4):
            raise ValueError('invalid_stair_regularization_limits')


@dataclass(frozen=True)
class StairRegularization:
    """One-to-one rows suitable for an audited source/planning mapping NPZ."""
    source_indices: np.ndarray
    source_xyz: np.ndarray
    planning_xyz: np.ndarray
    surface_id: np.ndarray
    models: tuple[dict, ...]
    diagnostics: dict


def _quantile(values, q):
    return float(np.quantile(values, q)) if len(values) else 0.


def _surface_normals(xyz, limits):
    """Original-return neighbourhoods; never normals of the fitted surface."""
    k = min(limits.normal_neighbors, len(xyz))
    distances, indices = cKDTree(xyz).query(xyz, k=k, workers=1)
    neighbours = xyz[indices]
    centered = neighbours - neighbours.mean(axis=1)[:, None, :]
    covariance = np.einsum('nki,nkj->nij', centered, centered) / k
    values, vectors = np.linalg.eigh(covariance)
    normal_z = np.abs(vectors[:, 2, 0])
    planar_rms = np.sqrt(np.maximum(0., values[:, 0]))
    enough_spread = values[:, 1] > 1e-6
    return ((normal_z >= limits.minimum_normal_z)
            & (planar_rms <= limits.maximum_plane_rms_m)
            & (distances[:, -1] <= limits.maximum_neighbor_radius_m)
            & enough_spread), normal_z, planar_rms


def connected_measured_support(xyz, eligible, anchors, radius, *,
                               maximum_points=250_000, maximum_neighbour_visits=8_000_000):
    """Reach only existing same-surface returns through bounded 3D edges.

    Eligibility (measured normal, model residual and surface bounds) must be
    established before this function. A component without an eligible anchor
    stays unselected. The radius is the original one-hop distance, not a new
    allowance for a gap. Work limits apply only to this offline map operation.
    """
    points, allowed, seeds = np.asarray(xyz), np.asarray(eligible), np.asarray(anchors)
    if (points.ndim != 2 or points.shape[1:] != (3,) or not np.isfinite(points).all()
            or allowed.dtype != np.bool_ or seeds.dtype != np.bool_
            or allowed.shape != (len(points),) or seeds.shape != allowed.shape
            or not np.isfinite(radius) or radius <= 0
            or maximum_points < 1 or maximum_neighbour_visits < 1):
        raise ValueError('invalid_measured_support_connectivity')
    indices = np.flatnonzero(allowed)
    if len(indices) > maximum_points:
        raise ValueError('measured_support_connectivity_work_limit')
    reached = np.zeros(len(points), bool)
    selected = seeds[indices].copy()
    queue = deque(np.flatnonzero(selected).tolist())
    tree = cKDTree(points[indices]) if len(indices) else None
    visits = 0
    while queue:
        index = queue.popleft()
        # Count before allocating a neighbourhood: dense input cannot create
        # an unbounded adjacency list or a full quadratic pair matrix.
        count = int(tree.query_ball_point(points[indices[index]], radius, return_length=True))
        if visits + count > maximum_neighbour_visits:
            raise ValueError('measured_support_connectivity_work_limit')
        visits += count
        neighbours = tree.query_ball_point(points[indices[index]], radius, return_sorted=True)
        for other in neighbours:
            if not selected[other]:
                selected[other] = True
                queue.append(other)
    reached[indices] = selected
    return reached, dict(eligible_points=len(indices), anchor_points=int((allowed & seeds).sum()),
        connected_points=int(reached.sum()), neighbour_visits=visits,
        neighbour_radius_m=float(radius), maximum_points=maximum_points,
        maximum_neighbour_visits=maximum_neighbour_visits,
        policy='same_surface_measured_3d_component_with_existing_support_anchor')


def _model_support(xyz, surface, model, normal_ok, limits):
    """Use original returns and fixed model bounds; never a fitted-point normal."""
    eligible = surface.mask(xyz) & normal_ok
    if surface.kind == 'flight_support_envelope':
        axis = np.asarray(model['uphill_axis_xy'], float)
        center = np.asarray(model['center_xy'], float)
        left, right = model['interior_lateral_range_m']
        begin, end = model['along_range_m']
        if (axis.shape != (2,) or center.shape != (2,)
                or not np.isfinite([*axis, *center, left, right, begin, end,
                                    model['grade'], model['intercept_m']]).all()
                or abs(np.linalg.norm(axis)-1.) > 1e-6 or not left < right or not begin < end):
            raise ValueError('invalid_fixed_stair_support_model')
        rel = xyz[:, :2]-center
        along, lateral = rel @ axis, rel @ [-axis[1], axis[0]]
        eligible &= ((lateral > left) & (lateral < right) & (along >= begin) & (along <= end))
        target = model['grade']*along+model['intercept_m']
        edge_distance = np.minimum(along-begin, end-along)
    else:
        if not np.isfinite(model['height_m']):
            raise ValueError('invalid_fixed_stair_support_model')
        target = np.full(len(xyz), model['height_m'])
        low, high = np.asarray(surface.minimum), np.asarray(surface.maximum)
        edge_distance = np.min(np.column_stack((xyz[:, :2]-low[:2], high[:2]-xyz[:, :2])), axis=1)
    residual = target-xyz[:, 2]
    eligible &= abs(residual) <= limits.maximum_displacement_m
    blend = np.clip(edge_distance/max(limits.boundary_blend_m, 1e-9), 0., 1.)
    blend = blend*blend*(3.-2.*blend)
    return eligible, residual, xyz[:, 2]+residual*blend


def _fit_flight(points, limits):
    # Height-stratified medians prevent a densely scanned landing or one wall
    # from rotating the flight axis. Direction is always low -> high.
    low, high = np.quantile(points[:, 2], [.02, .98])
    edges = np.linspace(low, high, 17)
    rows = [points[(points[:, 2] >= a) & (points[:, 2] < b)]
            for a, b in zip(edges[:-1], edges[1:])]
    medians = np.array([np.median(row, axis=0) for row in rows if len(row) >= 4])
    if len(medians) < limits.minimum_profile_bins:
        raise ValueError('insufficient_observed_flight_profile')
    center = medians[:, :2].mean(axis=0)
    _, _, vectors = np.linalg.svd(medians[:, :2] - center, full_matrices=False)
    axis = vectors[0]
    if np.dot(axis, medians[-1, :2] - medians[0, :2]) < 0:
        axis = -axis
    lateral_axis = np.array([-axis[1], axis[0]])
    along = (points[:, :2] - center) @ axis
    lateral = (points[:, :2] - center) @ lateral_axis
    fit = least_squares(lambda p: p[0] * along + p[1] - points[:, 2],
                        [.5, np.median(points[:, 2])],
                        loss='soft_l1', f_scale=.025)
    grade, intercept = fit.x
    residual = points[:, 2] - (grade * along + intercept)
    if (not fit.success or not .15 <= grade <= 1.0
            or _quantile(np.abs(residual), .95) > limits.maximum_fit_p95_m):
        raise ValueError('flight_support_model_not_observed')
    side_low, side_high = np.quantile(lateral, [.02, .98])
    side_low += limits.lateral_edge_guard_m
    side_high -= limits.lateral_edge_guard_m
    if side_high - side_low < .20:
        raise ValueError('insufficient_interior_stair_width')
    return dict(center_xy=center.tolist(), uphill_axis_xy=axis.tolist(),
                grade=float(grade), intercept_m=float(intercept),
                along_range_m=np.quantile(along, [.01, .99]).tolist(),
                interior_lateral_range_m=[float(side_low), float(side_high)],
                fit_points=len(points), profile_bins=len(medians),
                input_absolute_residual_p95_m=_quantile(np.abs(residual), .95),
                actual_tread_rise_m=None, actual_tread_depth_m=None,
                tread_period_status='not_identified_do_not_invent_nominal_steps')


def _fit_landing(points, limits):
    # The densest measured height cluster, not the entire ROI mean (railings
    # and tread junctions can occur in the same broad landing bounds).
    heights = np.sort(points[:, 2])
    ends = np.searchsorted(heights, heights + .06, side='right')
    first = int(np.argmax(ends - np.arange(len(heights))))
    cluster = heights[first:ends[first]]
    if len(cluster) < limits.minimum_fit_points:
        raise ValueError('landing_height_consensus_missing')
    height = float(np.median(cluster))
    residual = np.abs(points[:, 2] - height)
    consensus = residual <= .04
    if np.count_nonzero(consensus) < limits.minimum_fit_points:
        raise ValueError('landing_height_consensus_missing')
    return dict(height_m=height, fit_points=int(consensus.sum()),
                input_absolute_residual_p95_m=_quantile(residual[consensus], .95))


def regularize_stair_support(source_xyz, source_indices, support_seed_mask,
                             surfaces: Sequence[StairSurface], *, limits=None):
    """Fit and correct bounded observed support, retaining exact source identity.

    ``source_xyz`` contains the original records of the bounded stair area,
    including neighbouring structural points for normal estimation.
    ``support_seed_mask`` names existing classified/measured support, not all
    ROI points. Nearby original returns may join only with surface-normal and
    bounded residual evidence, so PCT's per-cell maximum is considered without
    converting the entire stair volume to a new solid ramp.

    Returns only associated support rows (some unchanged); no row is created,
    deleted, translated in XY or associated across two surfaces. A flight model
    is a global-search envelope, not a claim about physical step dimensions.
    """
    limits = limits or RegularizationLimits()
    xyz = np.asarray(source_xyz, dtype=np.float64)
    ids = np.asarray(source_indices)
    seeds = np.asarray(support_seed_mask)
    if (xyz.ndim != 2 or xyz.shape[1:] != (3,) or len(xyz) < 12
            or not np.isfinite(xyz).all() or ids.shape != (len(xyz),)
            or ids.dtype.kind not in 'iu' or np.any(ids < 0)
            or len(np.unique(ids)) != len(ids) or seeds.shape != ids.shape
            or seeds.dtype.kind != 'b' or not seeds.any()):
        raise ValueError('invalid_original_stair_support')
    if not surfaces or len({s.id for s in surfaces}) != len(surfaces):
        raise ValueError('ambiguous_stair_surface_identity')
    owner = np.full(len(xyz), -1, np.int32)
    output = xyz.copy()
    normal_ok, normal_z, planar_rms = _surface_normals(xyz, limits)
    models, skipped = [], []
    for surface in surfaces:
        region = surface.mask(xyz)
        fit_mask = seeds & region
        if np.count_nonzero(fit_mask) < limits.minimum_fit_points:
            skipped.append(dict(id=surface.id, reason='insufficient_classified_support'))
            continue
        try:
            model = (_fit_flight(xyz[fit_mask], limits)
                     if surface.kind == 'flight_support_envelope'
                     else _fit_landing(xyz[fit_mask], limits))
        except ValueError as exc:
            skipped.append(dict(id=surface.id, reason=str(exc)))
            continue
        eligible, residual, corrected_z = _model_support(xyz, surface, model, normal_ok, limits)
        near, _ = cKDTree(xyz[fit_mask]).query(xyz, workers=1)
        original_one_hop = eligible & (near <= limits.support_neighbor_m)
        if surface.kind == 'flight_support_envelope':
            connected, connectivity = connected_measured_support(
                xyz, eligible, fit_mask & eligible, limits.support_neighbor_m)
        else:
            # The confirmed omission is a measured flight patch. Keep the
            # established landing selection unchanged in initial full builds.
            connected = original_one_hop.copy()
            connectivity = dict(policy='landing_original_one_hop_unchanged',
                                neighbour_radius_m=limits.support_neighbor_m)
        candidate = original_one_hop | connected
        connectivity.update(original_one_hop_points=int(original_one_hop.sum()),
                            additional_connected_points=int((connected & ~original_one_hop).sum()))
        if np.any(candidate & (owner >= 0)):
            raise ValueError('overlapping_stair_surface_ownership')
        # Smooth spatial taper at segment endpoints, no detached height step
        # where the unchanged entrance/platform/exit joins the fitted envelope.
        output[candidate, 2] = corrected_z[candidate]
        owner[candidate] = surface.id
        changed = candidate & (output[:, 2] != xyz[:, 2])
        model.update(id=surface.id, name=surface.name, kind=surface.kind,
                     scope_minimum=list(surface.minimum), scope_maximum=list(surface.maximum),
                     associated_points=int(candidate.sum()), changed_points=int(changed.sum()),
                     maximum_displacement_m=_quantile(abs(output[changed, 2]-xyz[changed, 2]), 1.),
                     residual_before_rms_m=float(np.sqrt(np.mean(residual[candidate]**2))) if candidate.any() else 0.,
                    residual_after_rms_m=float(np.sqrt(np.mean((xyz[candidate, 2]+residual[candidate]-output[candidate, 2])**2))) if candidate.any() else 0.,
                    support_connectivity=connectivity)
        models.append(model)
    selected = owner >= 0
    return StairRegularization(
        source_indices=ids[selected].astype(np.int64), source_xyz=xyz[selected].copy(),
        planning_xyz=output[selected].copy(), surface_id=owner[selected].copy(),
        models=tuple(models), diagnostics=dict(input_records=len(xyz),
            seed_points=int(seeds.sum()), associated_points=int(selected.sum()),
            changed_points=int(np.count_nonzero(output[:, 2] != xyz[:, 2])),
            rejected_non_surface_records=int(np.count_nonzero(~normal_ok)),
            synthetic_points=0, xy_changed_points=0, removed_points=0,
            skipped_surfaces=skipped, planning_only=True,
            physical_tread_dimensions_recovered=False))


def extend_measured_stair_support(source_xyz, source_indices, existing_surface_ids,
                                  models, *, limits=None, surface_ids=None):
    """Find additional literal support without refitting or revising old rows.

    ``existing_surface_ids`` is aligned to these original source rows; -1
    means no previous mapping row. Existing measured one-hop support, after
    the same normal/model/bounds check, anchors each separate component.
    Previously mapped identity or corrected records are never returned.
    The caller verifies that every returned record is still literal in the
    current planning PCD and merges it without replacing any old mapping row.
    """
    limits = limits or RegularizationLimits()
    xyz, ids, owners = np.asarray(source_xyz, float), np.asarray(source_indices), np.asarray(existing_surface_ids)
    if (xyz.ndim != 2 or xyz.shape[1:] != (3,) or len(xyz) < 12
            or not np.isfinite(xyz).all() or ids.shape != (len(xyz),)
            or ids.dtype.kind not in 'iu' or np.any(ids < 0) or len(np.unique(ids)) != len(ids)
            or owners.shape != ids.shape or owners.dtype.kind not in 'iu' or np.any(owners < -1)):
        raise ValueError('invalid_original_stair_extension')
    normal_ok, _, _ = _surface_normals(xyz, limits)
    available = {model['id'] for model in models if model.get('kind') != 'unmodified_observed_support'}
    requested = available if surface_ids is None else set(surface_ids)
    if not requested or not requested <= available:
        raise ValueError('invalid_stair_extension_surface_selection')
    selected_owner = np.full(len(xyz), -1, np.int32)
    output = xyz.copy()
    reports = []
    for fixed in models:
        if fixed.get('kind') == 'unmodified_observed_support':
            continue
        if fixed['id'] not in requested:
            continue
        surface = StairSurface(fixed['id'], fixed['name'], fixed['kind'],
                               tuple(fixed['scope_minimum']), tuple(fixed['scope_maximum']))
        eligible, _, corrected_z = _model_support(xyz, surface, fixed, normal_ok, limits)
        reached, report = connected_measured_support(
            xyz, eligible, eligible & (owners == surface.id), limits.support_neighbor_m)
        added = reached & (owners == -1)
        if np.any(added & (selected_owner >= 0)):
            raise ValueError('overlapping_stair_surface_ownership')
        selected_owner[added] = surface.id
        output[added, 2] = corrected_z[added]
        reports.append(dict(surface_id=surface.id, additional_points=int(added.sum()), **report))
    selected = selected_owner >= 0
    return StairRegularization(ids[selected].astype(np.int64), xyz[selected].copy(),
        output[selected], selected_owner[selected], tuple(dict(model) for model in models),
        dict(input_records=len(xyz), additional_points=int(selected.sum()),
             changed_points=int(np.count_nonzero(output[selected, 2] != xyz[selected, 2])),
             fixed_models_unchanged=True, existing_mapping_rows_unchanged=True,
             selected_surface_ids=sorted(requested), surfaces=reports,
             synthetic_points=0, removed_points=0, xy_changed_points=0,
             planning_only=True, physical_tread_dimensions_recovered=False))
