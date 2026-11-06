"""Bounded soft-clearance refinement of an already valid PCT floor route.

Neither the native search/optimizer nor the hard map is changed.  Distance is
to the centres of inadmissible cells in the selected slice, NOT measured wall
or robot-envelope clearance.  A failed quality improvement retains the caller's
original route.  Every accepted cubic is checked by the production polynomial
cell-boundary validator; the distance field never grants traversability.
"""
from __future__ import annotations

from collections import OrderedDict
import hashlib

import numpy as np
from scipy import ndimage
from scipy.interpolate import CubicSpline

from .corridor_refinement import _curve_peak_curvature, _expanded_candidate, _expand_segment
from .path_quality import path_quality
from .tomogram_map import TomogramError


MAX_INPUT_POINTS = 30_000
MAX_OUTPUT_POINTS = 20_000
MAX_ANCHORS = 256
MAX_LENGTH_M = 1000.
MAX_QUALITY_SAMPLES = 30_000
SECTION_RADIUS_M = 2.
MIN_MAP_RESOLUTION_M = .01
TANGENT_SMOOTHING_M = 1.
OFFSET_SMOOTHING_M = 2.
ENDPOINT_TRANSITION_M = 4.
TURN_WEIGHT_M_PER_RAD = .35
MAX_ADDITIONAL_TURN_RAD = 1.5
MAX_INTERIOR_ADDITIONAL_TURN_RAD = .5
MAX_LOCAL_REPAIR_ROUNDS = 4
LOCAL_REPAIR_TAPER_M = 2.
DISTANCE_SEMANTICS = 'selected_slice_distance_to_inadmissible_cell_centres_not_physical_wall_clearance'
OBJECTIVE = 'xy_arclength_integral(1+2*exp(-slice_clearance_m/0.6))+0.35*absolute_turn_rad'


def _admissible_mask(tomogram, layer):
    # Include the retained floor/ROI mask, and re-read mutable predicate arrays.
    # This is only a quality field: original surface/curve checks remain the
    # authority and are run again before returning a candidate.
    shape = (tomogram.nx, tomogram.ny)
    arrays = (tomogram.valid, tomogram.ground_known, tomogram.ground, tomogram.cost,
              tomogram.ceiling_known, tomogram.headroom, tomogram.open_sky_verified)
    if any(np.shape(value) != (tomogram.layers, *shape) for value in arrays):
        raise ValueError('clearance_map_array_shape_changed')
    if not np.isfinite(tomogram.resolution) or tomogram.resolution <= 0:
        raise ValueError('clearance_map_resolution_invalid')
    cost = tomogram.cost[layer]
    mask = (tomogram.valid[layer] & tomogram.ground_known[layer]
            & np.isfinite(tomogram.ground[layer]) & np.isfinite(cost)
            & (cost >= 0) & (cost <= tomogram.COST_THRESHOLD))
    ceiling = tomogram.ceiling_known[layer]
    mask &= ~(ceiling & (tomogram.headroom[layer] < tomogram.minimum_headroom_m - 1e-6))
    if tomogram.unknown_ceiling_policy == 'reject':
        mask &= ceiling | tomogram.open_sky_verified[layer]
    return np.ascontiguousarray(mask, dtype=bool)


class ClearanceFieldCache:
    """At most two floor fields by default, tied to one map object and mask.

    Fingerprinting is intentionally repeated on reads: callers may edit a
    TomogramMap, including an unchanged object's cost/headroom arrays.  A field
    from the old mask must not silently rank the new map.  No route is cached.
    """
    def __init__(self, tomogram, max_layers=2):
        if isinstance(max_layers, bool) or int(max_layers) != max_layers or not 1 <= max_layers <= 16:
            raise ValueError('max_layers must be an integer in [1, 16]')
        self.tomogram, self.max_layers = tomogram, int(max_layers)
        self._fields = OrderedDict()
        self._hits = self._misses = self._invalidations = self._evictions = 0

    def get(self, layer):
        layer = self.tomogram.layer(layer)
        mask = _admissible_mask(self.tomogram, layer)
        fingerprint = (mask.shape, float(self.tomogram.resolution),
                       hashlib.blake2b(mask, digest_size=16).digest())
        old = self._fields.get(layer)
        if old is not None and old[0] == fingerprint:
            self._hits += 1
            self._fields.move_to_end(layer)
            return old[1]
        if old is not None:
            self._invalidations += 1
        self._misses += 1
        # Padding makes the outside of a finite map an explicit boundary; it
        # does not alter any cell of the planner's map or its inflation.
        field = (ndimage.distance_transform_edt(np.pad(mask, 1))[1:-1, 1:-1]
                 * self.tomogram.resolution).astype(np.float32)
        field.flags.writeable = False
        self._fields[layer] = (fingerprint, field)
        self._fields.move_to_end(layer)
        while len(self._fields) > self.max_layers:
            self._fields.popitem(last=False)
            self._evictions += 1
        return field

    @property
    def stats(self):
        return dict(hits=self._hits, misses=self._misses, invalidations=self._invalidations,
                    evictions=self._evictions, entries=len(self._fields), max_layers=self.max_layers,
                    field_bytes=sum(value[1].nbytes for value in self._fields.values()),
                    distance_semantics=DISTANCE_SEMANTICS)


def _cache(tomogram, cache):
    if cache is None:
        return ClearanceFieldCache(tomogram)
    if not isinstance(cache, ClearanceFieldCache) or cache.tomogram is not tomogram:
        raise ValueError('clearance cache belongs to a different map')
    return cache


def _input(tomogram, path_xyz, layer_ids):
    path = np.asarray(path_xyz, dtype=float)
    layers = np.asarray(layer_ids)
    if (path.ndim != 2 or path.shape[1] != 3 or not 2 <= len(path) <= MAX_INPUT_POINTS
            or not np.isfinite(path).all() or layers.shape != (len(path),)):
        raise ValueError('invalid_path_shape_or_values')
    layers = np.array([tomogram.layer(value) for value in layers], dtype=int)
    with np.errstate(over='ignore', invalid='ignore'):
        lengths = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
        length = float(lengths.sum())
    if not np.isfinite(length) or length > MAX_LENGTH_M:
        raise ValueError('path_length_limit')
    if length <= 1e-8:
        raise ValueError('no_horizontal_route')
    return path, layers, lengths


def _field_values(tomogram, field, xy):
    coordinates = ((np.asarray(xy) - tomogram.center) / tomogram.resolution + tomogram.offset).T
    # Linear interpolation is continuous across grid cells, avoiding an
    # objective that changes merely because sample positions cross rint ties.
    return ndimage.map_coordinates(field, coordinates, order=1, mode='constant', cval=0., prefilter=False)


def _interior_turn(path, lengths):
    """Audit the route interior separately from fixed-endpoint transitions.

    A wall-side selected endpoint necessarily requires an S-shaped transition
    into and out of a corridor centreline.  That does not justify repeatedly
    following door recesses in the interior.  Trim by XY arc length, not point
    indices, so this quality metric is invariant to route sampling density.
    """
    arc = np.r_[0., np.cumsum(lengths)]
    length = float(arc[-1])
    if length <= 2*ENDPOINT_TRANSITION_M + 1e-8:
        return dict(available=False, endpoint_exclusion_m=ENDPOINT_TRANSITION_M,
                    total_abs_turn_rad=None)
    retained = np.r_[True, lengths > 1e-10]
    stations = np.r_[ENDPOINT_TRANSITION_M,
                     arc[(arc > ENDPOINT_TRANSITION_M) & (arc < length-ENDPOINT_TRANSITION_M)],
                     length-ENDPOINT_TRANSITION_M]
    interior = np.column_stack([np.interp(stations, arc[retained], path[retained, axis])
                                for axis in range(3)])
    return dict(available=True, endpoint_exclusion_m=ENDPOINT_TRANSITION_M,
                total_abs_turn_rad=float(path_quality(interior)['total_abs_turn_rad'] or 0.))


def _quality(tomogram, path, layers, lengths, cache):
    length = float(lengths.sum())
    spacing = min(.05, max(.01, tomogram.resolution / 2))
    count = max(1, int(np.ceil(length / spacing)))
    if count > MAX_QUALITY_SAMPLES:
        raise ValueError('quality_sample_limit')
    edges = np.r_[0., np.cumsum(lengths)]
    stations = (np.arange(count) + .5) * length / count
    indices = np.minimum(np.searchsorted(edges, stations, side='right') - 1, len(path)-2)
    alpha = (stations-edges[indices]) / lengths[indices]
    xy = (1-alpha[:, None])*path[indices, :2] + alpha[:, None]*path[indices+1, :2]
    clearances = np.full(count, np.inf)
    # A genuine layer connector is scored in each of its two actual slices;
    # it never borrows the clearance of a different floor at the same XY.
    for layer in np.unique(layers):
        selected = (layers[indices] == layer) | (layers[indices+1] == layer)
        if selected.any():
            values = _field_values(tomogram, cache.get(int(layer)), xy[selected])
            clearances[selected] = np.minimum(clearances[selected], values)
    if not np.isfinite(clearances).all():
        raise ValueError('invalid_clearance_samples')
    geometry = path_quality(path)
    turn = float(geometry['total_abs_turn_rad'] or 0.)
    exposure = float(np.mean(2*np.exp(-clearances/.6))*length)
    interior = clearances[(stations >= 2.) & (stations <= length-2.)]
    metrics = dict(min_m=float(clearances.min()), p10_m=float(np.percentile(clearances, 10)),
                   mean_m=float(clearances.mean()), samples=count,
                   interior_excluding_2m=dict(available=bool(len(interior)),
                       min_m=float(interior.min()) if len(interior) else None,
                       p10_m=float(np.percentile(interior, 10)) if len(interior) else None,
                       mean_m=float(interior.mean()) if len(interior) else None),
                   distance_semantics=DISTANCE_SEMANTICS)
    return dict(available=True, score=length+exposure+TURN_WEIGHT_M_PER_RAD*turn, objective=OBJECTIVE,
                xy_length_m=length, length_m=length, xyz_length_m=geometry['xyz_length_m'],
                total_abs_turn_rad=turn, curvature_max_per_m=geometry['curvature_max_per_m'],
                interior_turn=_interior_turn(path, lengths),
                clearance_exposure_m=exposure, clearance=metrics,
                clearance_min_m=metrics['min_m'], clearance_p10_m=metrics['p10_m'],
                clearance_mean_m=metrics['mean_m'], physical_safety_certified=False,
                geometry_quality=geometry)


def route_clearance_quality(tomogram, path_xyz, layer_ids, cache=None):
    """One comparable soft objective, retaining every original layer identity.

    Full input validation is deliberate: no quality score makes an invalid
    route a selectable fallback.  Exact polynomial validation of a new curve
    remains the producer's responsibility, not this polyline audit.
    """
    cache = _cache(tomogram, cache)
    try:
        path, layers, lengths = _input(tomogram, path_xyz, layer_ids)
        tomogram.validate_path(path, layers)
        return {**_quality(tomogram, path, layers, lengths, cache), 'cache_stats': cache.stats}
    except (TypeError, ValueError, TomogramError) as error:
        return dict(available=False, reason=getattr(error, 'code', str(error)),
                    objective=OBJECTIVE, cache_stats=cache.stats)


def _section_extent(tomogram, origin, normal, layer, direction):
    """First boundary of the connected, supported normal ray containing origin."""
    end = origin + direction*SECTION_RADIUS_M*normal
    previous_heights = [tomogram.surface_ground_z(origin, layer)]
    previous_distance = 0.
    for fraction, cells in tomogram._segment_samples_f64(origin, end, check_endpoints=False):
        heights = []
        for cell in cells:
            if tomogram._reason(layer, cell):
                return previous_distance, True
            value = float(tomogram.ground[(layer, *cell)])
            if not np.isfinite(value):
                return previous_distance, True
            heights.append(value)
        if (max(heights)-min(heights) > tomogram.max_ground_step_m + 1e-6
                or max(abs(a-b) for a in heights for b in previous_heights)
                > tomogram.max_ground_step_m + 1e-6):
            return previous_distance, True
        previous_heights = heights
        previous_distance = float(fraction)*SECTION_RADIUS_M
    return SECTION_RADIUS_M, False


def _cubic_segments(stations, xy):
    spline = CubicSpline(stations, xy, bc_type='natural')
    segments = []
    for index, width in enumerate(np.diff(stations)):
        # scipy uses descending powers of (s-s_i).  The common curve checker
        # consumes ascending powers of u in [0,1], for every entire cubic.
        coefficients = spline.c[:, index, :][::-1] * width**np.arange(4)[:, None]
        bound = sum(k*np.linalg.norm(coefficients[k]) for k in range(1, 4))
        segments.append(dict(kind='natural_cubic', coefficients_xy=coefficients,
                             derivative_bound_m=float(bound)))
    return segments


def _locally_repaired_segments(tomogram, stations, xy, offsets, normals, layer, start, goal):
    """Retreat only conflicting centering offsets, never change the hard map.

    The ordinary three candidates are tried first. This bounded fallback keeps
    long straight sections centred when a single corner/notch rejects their
    full-route spline. Each conflicting span is found by the *same* production
    polynomial expansion used for final admission. A smooth local taper moves
    its anchors toward the original valid route; neighbouring spans are checked
    again because a natural cubic has global coupling. A still-invalid curve is
    never returned, even when its offsets have reached zero.
    """
    weights = np.ones(len(stations))
    rounds = []
    for attempt in range(MAX_LOCAL_REPAIR_ROUNDS + 1):
        shifted = xy + (offsets*weights)[:, None]*normals
        shifted[0], shifted[-1] = start[:2], goal[:2]
        segments = _cubic_segments(stations, shifted)
        blocked = []
        for index, segment in enumerate(segments):
            try:
                _expand_segment(tomogram, segment, layer,
                                start[2] if index == 0 else None,
                                goal[2] if index == len(segments)-1 else None)
            except TomogramError as error:
                blocked.append(dict(index=index, reason=error.code,
                                    details=error.details))
        rounds.append(dict(attempt=attempt, blocked_spans=blocked))
        if not blocked:
            return segments, dict(applied=attempt > 0, rounds=rounds,
                                  modified_anchor_count=int(np.count_nonzero(weights < 1.)),
                                  taper_m=LOCAL_REPAIR_TAPER_M,
                                  max_rounds=MAX_LOCAL_REPAIR_ROUNDS)
        if attempt == MAX_LOCAL_REPAIR_ROUNDS:
            break
        old_weights = weights.copy()
        for failure in blocked:
            index = failure['index']
            # Zero over the conflicting span and use a C2 taper at both sides.
            # Widen only after repeated conflict, bounded by the round count.
            radius = LOCAL_REPAIR_TAPER_M*(1.+.5*attempt)
            distance = np.maximum(stations[index]-stations,
                                  stations-stations[index+1])
            fraction = np.clip(distance/radius, 0., 1.)
            taper = fraction**3*(10.-15.*fraction+6.*fraction**2)
            weights = np.minimum(weights, taper)
        if np.array_equal(old_weights, weights):
            break
    return None, dict(applied=False, reason='local_offsets_cannot_repair_curve',
                      rounds=rounds, max_rounds=MAX_LOCAL_REPAIR_ROUNDS)


def center_corridor(tomogram, path_xyz, layer_ids, cache=None, *, allow_graded_support=False):
    """Try bounded geometric candidates, retaining ordinary successful curves."""
    cache = _cache(tomogram, cache)
    base = dict(applied=False, algorithm='checked_slice_clearance_cubic_centering',
                is_native_gpmp=False, objective=OBJECTIVE, physical_safety_certified=False,
                distance_semantics=DISTANCE_SEMANTICS)
    def reject(reason, **details):
        return {**base, 'reason': reason, 'cache_stats': cache.stats, **details}
    try:
        path, layers, lengths = _input(tomogram, path_xyz, layer_ids)
        tomogram.validate_path(path, layers)
        if np.any(layers != layers[0]):
            return reject('multi_layer_path_not_applicable')
        if tomogram.resolution < MIN_MAP_RESOLUTION_M:
            return reject('cross_section_work_limit')
        # Caller selects floor legs.  Also decline obvious stairs/ramps if used
        # directly; this is a quality-only no-op, not a navigation rejection.
        if not allow_graded_support and np.ptp(path[:, 2]) > max(.25, 2*tomogram.max_ground_step_m):
            return reject('nonfloor_height_profile_not_applicable')
        before = _quality(tomogram, path, layers, lengths, cache)
    except (TypeError, ValueError, TomogramError) as error:
        return reject('invalid_input_path', error_code=getattr(error, 'code', str(error)))
    before_data = dict(before_quality=before, before_clearance=before['clearance'], score_before=before['score'])
    total = before['xy_length_m']
    if total < .5 or np.linalg.norm(path[-1, :2]-path[0, :2]) < 1e-8:
        return reject('short_or_closed_route_not_applicable', **before_data)
    stations = np.linspace(0., total, min(MAX_ANCHORS, max(3, int(np.ceil(total/.5))+1)))
    source_arc = np.r_[0., np.cumsum(lengths)]
    retained = np.r_[True, lengths > 1e-10]
    xy = np.column_stack([np.interp(stations, source_arc[retained], path[retained, axis])
                          for axis in range(2)])
    tangent = np.gradient(xy, stations, axis=0)
    spacing = float(stations[1]-stations[0])
    tangent = ndimage.gaussian_filter1d(tangent, max(TANGENT_SMOOTHING_M/spacing, .25), axis=0, mode='nearest')
    norms = np.linalg.norm(tangent, axis=1)
    if np.any(norms < 1e-8):
        return reject('nonregular_anchor_tangent', **before_data)
    normals = np.c_[-tangent[:, 1], tangent[:, 0]]/norms[:, None]
    offsets, counts = np.zeros(len(xy)), dict(two_sided=0, one_sided=0, open=0)
    layer = int(layers[0])
    try:
        for index, (point, normal) in enumerate(zip(xy, normals)):
            left, left_closed = _section_extent(tomogram, point, normal, layer, -1)
            right, right_closed = _section_extent(tomogram, point, normal, layer, 1)
            if left_closed and right_closed:
                offsets[index] = (right-left)/2
                counts['two_sided'] += 1
            elif left_closed or right_closed:
                # A single wall does not define a corridor centreline. Move
                # away only within one metre; do not chase distant open space.
                offsets[index] = max(0., 1.-left) if left_closed else -max(0., 1.-right)
                counts['one_sided'] += 1
            else:
                counts['open'] += 1
    except TomogramError as error:
        return reject('anchor_cross_section_rejected', error_code=error.code, **before_data)
    # Follow the corridor at metre scale, not every recess or cell-width
    # fluctuation.  Endpoint transitions also need enough length to move away
    # from a wall gently while keeping both selected endpoints exactly fixed.
    offsets = ndimage.gaussian_filter1d(offsets, max(OFFSET_SMOOTHING_M/spacing, .25), mode='nearest')
    endpoint = np.clip(np.minimum(stations, total-stations)/min(ENDPOINT_TRANSITION_M, total/2), 0., 1.)
    endpoint = endpoint**3*(10-15*endpoint+6*endpoint**2)
    offsets *= endpoint
    offsets[[0, -1]] = 0.
    if np.max(np.abs(offsets)) < .02:
        return reject('no_supported_centering_shift', sections=counts, **before_data)
    peak_limit = max(1.25, float(before['curvature_max_per_m'] or 0.))
    failures, accepted = [], []
    for blend in (1., .7, .4, .2, .1, .05, 'local'):
        repair = None
        if isinstance(blend, float) and blend < .4:
            # A short route with fixed wall-side endpoints cannot make the
            # same lateral excursion as a long corridor. The original coarse
            # blends can all exceed curvature/turn limits although a modest
            # improvement exists. Backtrack at most three times, and only
            # when those endpoint transitions occupy the entire route.
            # Preserve every previously successful result and do not rescue
            # long doorway-following snakes by merely shrinking their wiggle.
            if accepted or allow_graded_support or total > 2*ENDPOINT_TRANSITION_M or not any(
                    failure['reason'] in ('analytic_curvature_limit',
                        'bounded_detour_exceeded', 'excess_additional_turn')
                    for failure in failures):
                continue
        if blend == 'local':
            # Do not change a previously successful result. Local repair is a
            # route-quality fallback, not another validity/clearance gate.
            if accepted or not any(failure['reason'] in (
                    'pct_cost_blocked', 'invalid_curve', 'unobserved_ground',
                    'insufficient_headroom', 'unobserved_ceiling', 'ground_step')
                    for failure in failures):
                break
            # If the map changed under the optional refinement, its old route
            # is no longer a valid fallback to anchor to.
            try:
                tomogram.validate_path(path, layers)
            except TomogramError:
                break
            try:
                segments, repair = _locally_repaired_segments(
                    tomogram, stations, xy, offsets, normals, layer, path[0], path[-1])
            except (ValueError, TomogramError) as error:
                failures.append(dict(blend=blend, reason=getattr(error, 'code', str(error))))
                continue
            if segments is None:
                failures.append(dict(blend=blend, reason=repair['reason'], local_repair=repair))
                continue
        else:
            shifted = xy + blend*offsets[:, None]*normals
            shifted[0], shifted[-1] = path[0, :2], path[-1, :2]
            segments = _cubic_segments(stations, shifted)
        peak = _curve_peak_curvature(segments)
        if not np.isfinite(peak) or peak > peak_limit + 1e-6:
            failures.append(dict(blend=blend, reason='analytic_curvature_limit',
                                 peak_per_m=float(peak) if np.isfinite(peak) else None))
            continue
        try:
            candidate, partition_points, checks = _expanded_candidate(tomogram, segments, layer, path[0], path[-1])
            if len(candidate) > MAX_OUTPUT_POINTS:
                failures.append(dict(blend=blend, reason='published_point_limit'))
                continue
            candidate_layers = np.full(len(candidate), layer, dtype=int)
            candidate_lengths = np.linalg.norm(np.diff(candidate[:, :2], axis=0), axis=1)
            after = _quality(tomogram, candidate, candidate_layers, candidate_lengths, cache)
            if after['xy_length_m'] > total + max(.2, .08*total) + 1e-8:
                failures.append(dict(blend=blend, reason='bounded_detour_exceeded'))
                continue
            if after['total_abs_turn_rad'] > before['total_abs_turn_rad'] + MAX_ADDITIONAL_TURN_RAD + 1e-8:
                failures.append(dict(blend=blend, reason='excess_additional_turn',
                    added_turn_rad=after['total_abs_turn_rad']-before['total_abs_turn_rad']))
                continue
            original_interior, candidate_interior = before['interior_turn'], after['interior_turn']
            if (original_interior['available'] and candidate_interior['available']
                    and candidate_interior['total_abs_turn_rad'] > original_interior['total_abs_turn_rad']
                        + MAX_INTERIOR_ADDITIONAL_TURN_RAD + 1e-8):
                failures.append(dict(blend=blend, reason='excess_interior_turn',
                    added_turn_rad=candidate_interior['total_abs_turn_rad']-original_interior['total_abs_turn_rad']))
                continue
            if after['score'] >= before['score'] - max(.01, .001*before['score']):
                failures.append(dict(blend=blend, reason='no_clear_soft_objective_improvement', score=after['score']))
                continue
            audit = [{key: value.tolist() if isinstance(value, np.ndarray) else value
                      for key, value in segment.items()} for segment in segments]
            accepted.append({**base, **before_data, 'applied': True,
                'reason': 'checked_clearance_improvement', 'path': candidate.tolist(),
                'layer_ids': candidate_layers.tolist(), 'segments': audit,
                'curve_validation': 'polynomial_cell_boundary_roots_and_interval_interiors_plus_validate_path',
                'curve_partition_points': partition_points, 'after_quality': after,
                'after_clearance': after['clearance'], 'score_after': after['score'],
                'blend': blend, 'anchor_count': len(xy), 'sections': counts,
                'local_repair': repair,
                'analytic_curvature_peak_per_m': peak, 'analytic_curvature_limit_per_m': peak_limit,
                'extra_length_limit_m': max(.2, .08*total),
                'quality_turn_budget': dict(additional_total_rad=MAX_ADDITIONAL_TURN_RAD,
                    additional_interior_rad=MAX_INTERIOR_ADDITIONAL_TURN_RAD,
                    endpoint_exclusion_m=ENDPOINT_TRANSITION_M,
                    rejection_retains_original_route=True),
                'centering_scales_m': dict(tangent_sigma=TANGENT_SMOOTHING_M,
                    offset_sigma=OFFSET_SMOOTHING_M, endpoint_transition=min(ENDPOINT_TRANSITION_M, total/2)),
                'continuity': 'XY C2 natural cubic in route station; measured ground Z, not dynamic feasibility',
                'height_semantics': 'measured_ground_with_exact_selected_endpoints', **checks})
        except (ValueError, TomogramError) as error:
            failures.append(dict(blend=blend, reason=getattr(error, 'code', str(error))))
    if not accepted:
        return reject('no_valid_clearance_improvement', candidate_failures=failures, sections=counts, **before_data)
    result = min(accepted, key=lambda candidate: (candidate['score_after'], candidate['after_quality']['xy_length_m']))
    result.update(candidate_failures=failures, cache_stats=cache.stats)
    return result
