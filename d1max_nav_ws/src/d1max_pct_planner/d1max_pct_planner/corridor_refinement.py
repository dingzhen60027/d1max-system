"""Explicit, bounded visibility + XY geometric-C2 refinement of a valid route.

This is not a native GPMP curve. Z follows measured tomogram ground and exact
selected endpoints; neither 3D C2 continuity nor dynamic feasibility is claimed.
All polynomial cell boundaries/interiors and the final published polyline pass
the original TomogramMap checks. Rejection never returns an unchecked fallback.
"""
from __future__ import annotations

from math import comb

import numpy as np

from .path_quality import path_quality
from .tomogram_map import TomogramError


MAX_INPUT_POINTS = 20_000
MAX_OUTPUT_POINTS = 30_000
MAX_XY_LENGTH_M = 1000.0
MAX_VISIBILITY_CHECKS = 2048
MAX_POLYLINE_VERTICES = 512
MAX_CORNER_ALTERNATIVES = 4
OUTPUT_SPACING_M = 0.10
VISIBILITY_ANCHOR_SPACING_M = 0.50
# Geometric selection bound for this conditioned-map tradeoff (radius >= .8m),
# not a robot steering/dynamic limit. Native heading-rate parameters are not
# curvature bounds and must not be relabelled as one.
MAX_CONDITIONED_CORNER_CURVATURE_PER_M = 1.25


def _rejected(reason, before=None, **details):
    return {'applied': False, 'reason': reason, 'before_quality': before,
            'after_quality': None, 'algorithm': 'explicit_visibility_c2_refinement',
            'is_native_gpmp': False,
            'soft_cost_policy': 'prefer_geometric_simplicity_within_unchanged_hard_pct_mask',
            'physical_safety_certified': False, **details}


def _bezier_power(control):
    coefficients = np.zeros((6, 2), dtype=float)
    for i in range(6):
        for k in range(i, 6):
            coefficients[k] += comb(5, i) * comb(5-i, k-i) * (-1)**(k-i) * control[i]
    return coefficients


def _line(a, b):
    control = np.array([a, b], dtype=float)
    return {'kind': 'line', 'coefficients_xy': np.array([control[0], control[1]-control[0]]),
            'control_points_xy': control, 'derivative_bound_m': float(np.linalg.norm(control[1]-control[0]))}


def _rounded_segments(polyline, cut_m):
    segments, current = [], polyline[0]
    for index in range(1, len(polyline)-1):
        before, vertex, after = polyline[index-1:index+2]
        incoming, outgoing = vertex-before, after-vertex
        len_a, len_b = float(np.linalg.norm(incoming)), float(np.linalg.norm(outgoing))
        a, b = incoming/len_a, outgoing/len_b
        cosine = float(np.clip(np.dot(a, b), -1, 1))
        if cosine > 1 - 1e-10:
            continue
        if cosine < -.999:
            raise TomogramError('refinement_hairpin', 'Cannot round a near-reversing corner without a regular tangent')
        d = min(cut_m, .45*len_a, .45*len_b)
        if d <= 1e-8:
            raise TomogramError('refinement_tiny_corner', 'Corner is too short for a checked smooth connector')
        control = np.array([vertex-d*a, vertex-.6*d*a, vertex-.2*d*a,
                            vertex+.2*d*b, vertex+.6*d*b, vertex+d*b])
        if np.linalg.norm(control[0]-current) > 1e-10:
            segments.append(_line(current, control[0]))
        coefficients = _bezier_power(control)
        # Reject stationary/cusped curves, including interior critical speeds.
        derivatives = np.array([np.polynomial.polynomial.polyder(coefficients[:, axis]) for axis in range(2)])
        speed_sq = np.polynomial.polynomial.polyadd(
            np.polynomial.polynomial.polymul(derivatives[0], derivatives[0]),
            np.polynomial.polynomial.polymul(derivatives[1], derivatives[1]))
        roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(speed_sq))
        times = [0., 1., *[float(v.real) for v in roots if abs(v.imag)<1e-8 and 0<v.real<1]]
        if min(np.polynomial.polynomial.polyval(times, speed_sq)) <= 1e-12:
            raise TomogramError('refinement_nonregular_corner', 'Rounded corner has a stationary tangent')
        segments.append({'kind': 'quintic_bezier', 'coefficients_xy': coefficients,
                         'control_points_xy': control, 'corner_cut_m': d,
                         'derivative_bound_m': float(5*np.max(np.linalg.norm(np.diff(control, axis=0), axis=1)))})
        current = control[-1]
    if np.linalg.norm(polyline[-1]-current) > 1e-10:
        segments.append(_line(current, polyline[-1]))
    return segments


def _corner_intersection_candidates(polyline, cut_m):
    """Recover the full corner hidden by two adjacent visibility cut points.

    Farthest-visible anchors can clip the inside of a turn so tightly that no
    sufficiently smooth rounded curve fits. Intersect only the forward incoming
    and backward outgoing supporting rays; do not change the route endpoints or
    turn direction. These are *hypotheses*, not checked shortcuts: the caller
    still validates every complete polynomial against the unchanged tomogram.
    """
    alternatives = []
    for index in range(1, len(polyline)-2):
        a, b, c, d = polyline[index-1:index+3]
        incoming, middle, outgoing = b-a, c-b, d-c
        lengths = np.linalg.norm([incoming, middle, outgoing], axis=1)
        if not np.isfinite(lengths).all() or min(lengths) <= 1e-10:
            continue
        incoming, middle, outgoing = (v/length for v, length in
                                      zip((incoming, middle, outgoing), lengths))
        cross = lambda u, v: float(u[0]*v[1]-u[1]*v[0])
        first_turn, second_turn = cross(incoming, middle), cross(middle, outgoing)
        determinant = cross(incoming, outgoing)
        # No U/S turns, straight-line aliases or ill-conditioned intersections.
        if (first_turn*second_turn <= 1e-12 or abs(determinant) <= 1e-8
                or float(np.dot(incoming, outgoing)) <= -.999):
            continue
        distance = c-b
        forward = cross(distance, outgoing)/determinant
        backward = cross(distance, incoming)/determinant
        if forward < -1e-9 or backward > 1e-9:
            continue
        corner = b + forward*incoming
        if not np.isfinite(corner).all():
            continue
        along = np.clip(np.dot(corner-b, middle), 0., lengths[1])
        displacement = float(np.linalg.norm(corner-(b+along*middle)))
        # A local clipped-corner repair, never an arbitrarily remote ray join.
        if displacement > cut_m or max(forward, -backward) > lengths[1] + cut_m:
            continue
        candidate = np.vstack((polyline[:index], corner, polyline[index+2:]))
        turning = abs(np.arctan2(determinant, np.dot(incoming, outgoing)))
        alternatives.append((turning, index, candidate))
    alternatives.sort(key=lambda item: (-item[0], item[1]))
    return [item[2] for item in alternatives[:MAX_CORNER_ALTERNATIVES]]


def _expand_segment(tomogram, segment, layer, first_z=None, last_z=None):
    # Lazy import avoids TomogramRoute -> refinement -> TomogramRoute cycles.
    from .tomogram_route import curve_partition
    coefficients = segment['coefficients_xy']
    bound = segment['derivative_bound_m']
    density_steps = max(1, int(np.ceil(bound / OUTPUT_SPACING_M)))
    if density_steps + 1 > MAX_OUTPUT_POINTS:
        raise TomogramError('refinement_output_limit', 'A segment exceeds the output point budget')
    partition = curve_partition(coefficients, tomogram)
    times = sorted(set(partition) | set(np.linspace(0, 1, density_steps+1)))
    # Evaluate Horner arithmetic in one array operation, retaining exactly the
    # same partition/density samples. Ground and every adjacent boundary cell
    # remain checked individually; no sparse chord replaces a quintic check.
    positions = np.polynomial.polynomial.polyval(times, coefficients).T
    path = np.empty((len(times), 3), dtype=float)
    path[:, :2] = positions
    path[:, 2] = tomogram.surface_ground_z_many(positions, layer)
    if first_z is not None:
        path[0, 2] = first_z
    if last_z is not None:
        path[-1, 2] = last_z
    tomogram.validate_path(path, np.full(len(path), layer))
    return path, len(partition)


def _expanded_candidate(tomogram, segments, layer, start, goal):
    paths, partition_points, total = [], 0, 0
    for index, segment in enumerate(segments):
        path, count = _expand_segment(tomogram, segment, layer,
                                     start[2] if index == 0 else None,
                                     goal[2] if index == len(segments)-1 else None)
        if paths and np.linalg.norm(path[0]-paths[-1][-1]) <= 1e-8:
            path = path[1:]
        paths.append(path)
        partition_points += count
        total += len(path)
        if total > MAX_OUTPUT_POINTS:
            raise TomogramError('refinement_output_limit', 'Refined curve exceeds the output point budget')
    result = np.vstack(paths)
    result[0], result[-1] = start, goal
    checks = tomogram.validate_path(result, np.full(len(result), layer))
    return result, partition_points, checks


def _quality_not_worse(before, after):
    failures = []
    for key in ('xy_length_m', 'xyz_length_m', 'total_abs_turn_rad', 'curvature_p95_per_m', 'curvature_max_per_m'):
        old, new = before[key], after[key]
        if new is None:
            if old is not None and old > 1e-8:
                failures.append(key)
        elif old is None:
            if new > 1e-8:
                failures.append(key)
        elif new > old + 1e-6 * max(1., abs(old)):
            failures.append(key)
    return failures


def _curve_peak_curvature(segments):
    """Evaluate polynomial curvature extrema, not sparse chord samples."""
    peak = 0.
    for segment in segments:
        c = segment['coefficients_xy']
        dx, dy = (np.polynomial.polynomial.polyder(c[:, axis]) for axis in (0, 1))
        ddx, ddy = (np.polynomial.polynomial.polyder(v) for v in (dx, dy))
        mul, add, sub = (np.polynomial.polynomial.polymul,
                         np.polynomial.polynomial.polyadd, np.polynomial.polynomial.polysub)
        numerator = sub(mul(dx, ddy), mul(dy, ddx))
        speed_sq = add(mul(dx, dx), mul(dy, dy))
        # d(kappa^2)/dt=0 away from zero curvature iff 2*n'*s-3*n*s'=0.
        critical = sub(2*mul(np.polynomial.polynomial.polyder(numerator), speed_sq),
                       3*mul(numerator, np.polynomial.polynomial.polyder(speed_sq)))
        roots = np.polynomial.polynomial.polyroots(critical)
        times = [0., 1., *[float(v.real) for v in roots if abs(v.imag)<1e-8 and 0<v.real<1]]
        speed = np.polynomial.polynomial.polyval(times, speed_sq)
        if not np.isfinite(speed).all() or min(speed) <= 1e-12:
            return float('inf')
        values = np.abs(np.polynomial.polynomial.polyval(times, numerator)) / speed**1.5
        if not np.isfinite(values).all():
            return float('inf')
        peak = max(peak, float(max(values)))
    return peak


def _conditioned_corridor_map(tomogram):
    provenance = getattr(tomogram, 'provenance', {})
    return (provenance.get('processing_schema') == 'd1max.crossfloor_planning/v1'
            and provenance.get('geometry_operation') ==
                'independent_floor_conditioning_visibility_filter_and_measured_return_restore'
            and provenance.get('frame_id') == 'd1max_multifloor_planning')


def _corridor_quality(before, after, *, conditioned=False, curve_peak=None):
    """Compare already-validated geometry, never collision/dynamic authority.

    Distributed native waviness can have a lower *local* curvature peak than
    one regular C2 corner, despite being longer and turning much more overall.
    Do not retain that waviness solely to win a peak statistic. The strict
    comparator remains unchanged for its other callers (source-identity paths).
    """
    failures = _quality_not_worse(before, after)
    tradeoff = None
    if conditioned and failures == ['curvature_max_per_m']:
        peak = [before['curvature_max_per_m'], after['curvature_max_per_m']]
        progress = all(after[key] < before[key] - 1e-6 * max(1., abs(before[key]))
                       for key in ('xy_length_m', 'total_abs_turn_rad', 'curvature_p95_per_m'))
        bounded = (curve_peak is not None and np.isfinite(curve_peak)
                   and 0 <= curve_peak <= MAX_CONDITIONED_CORNER_CURVATURE_PER_M)
        if (progress and bounded and all(value is not None and np.isfinite(value) for value in peak)):
            # All other metrics, including XYZ length and p95 curvature, have
            # already passed the same non-worsening comparison above. The
            # caller has checked the entire C2 curve against the original map.
            failures = []
            tradeoff = dict(metric='curvature_max_per_m', before=peak[0], after=peak[1],
                            reason='shorter_route_and_less_total_turn_with_nonworsening_p95',
                            polynomial_peak_per_m=curve_peak,
                            maximum_polynomial_peak_per_m=MAX_CONDITIONED_CORNER_CURVATURE_PER_M,
                            dynamic_feasibility_certified=False)
    return failures, tradeoff


def _cost_audit(tomogram, path, layer):
    touched = set()
    for a, b in zip(path[:-1], path[1:]):
        # Both call sites have already validated finite float64 XYZ arrays.
        for unused_time, cells in tomogram._segment_samples_f64(a, b):
            touched.update((layer, *cell) for cell in cells)
    values = np.array([tomogram.cost[cell] for cell in touched], dtype=float)
    return {'max_cost': float(values.max()) if len(values) else None,
            'mean_touched_cell_cost': float(values.mean()) if len(values) else None,
            'touched_layer_cells': len(touched),
            'mean_semantics': 'Equal weight per unique touched layer/cell, including both sides of boundaries; not an arc-length average',
            'hard_cost_threshold': float(tomogram.COST_THRESHOLD)}


def refine_corridor(tomogram, path_xyz, layer_ids, corner_cut_m=1.5, *, quality_policy='floor'):
    """Return a validated explicit candidate, or ``applied=False`` with reason.

    Original route validity is a precondition verified here, never assumed.
    Greedy visibility uses approximately 0.5 m *original* anchors, and falls
    back to adjacent original vertices rather than dropping blocked corners.
    Every proposed shortcut follows measured ground and uses full boundary
    checks. Up to three cut scales (1, .7, .4) are tested; no sharp-polyline
    fallback is silently published if rounded candidates fail.
    """
    if quality_policy not in ('floor', 'stair_xy'):
        raise ValueError('Unsupported explicit refinement quality policy')
    try:
        path = np.asarray(path_xyz, dtype=float)
        layers = np.asarray(layer_ids)
        if (path.ndim != 2 or path.shape[1] != 3 or len(path) < 2
                or not np.isfinite(path).all() or layers.shape != (len(path),)):
            return _rejected('invalid_input_shape_or_values')
        if len(path) > MAX_INPUT_POINTS:
            return _rejected('input_point_limit')
        # Bound work before full cell-boundary validation, not after walking an
        # arbitrarily long, repeatedly zigzagging input through the grid.
        with np.errstate(over='ignore', invalid='ignore'):
            delta = np.diff(path[:, :2], axis=0)
            input_length = float(np.hypot(delta[:, 0], delta[:, 1]).sum())
        if not np.isfinite(input_length) or input_length > MAX_XY_LENGTH_M:
            return _rejected('input_length_limit')
        if isinstance(corner_cut_m, (bool, np.bool_)) or not np.isfinite(corner_cut_m) or corner_cut_m <= 0:
            return _rejected('invalid_corner_cut')
        layers = np.array([tomogram.layer(value) for value in layers], dtype=int)
        tomogram.validate_path(path, layers)
        before = path_quality(path)
    except (TypeError, ValueError, TomogramError) as error:
        return _rejected('invalid_input_path', error_code=getattr(error, 'code', type(error).__name__),
                         validation_error=str(error))
    if np.any(layers != layers[0]):
        return _rejected('multi_layer_path_not_supported', before)
    if before['xy_length_m'] > MAX_XY_LENGTH_M:
        return _rejected('input_length_limit', before)
    if before['xy_length_m'] <= 1e-8 or before['closed_xy']:
        return _rejected('degenerate_or_closed_xy_route', before)
    layer = int(layers[0])
    before_cost = _cost_audit(tomogram, path, layer)
    kept = np.r_[True, np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 1e-10]
    reduced = path[kept].copy()
    reduced[0], reduced[-1] = path[0], path[-1]
    if len(reduced) < 2:
        return _rejected('degenerate_xy_route', before)
    visibility_checks, cache = 0, {}
    direct_geometry = None

    def visible(first, last):
        nonlocal visibility_checks, direct_geometry
        key = (int(first), int(last))
        if key in cache:
            return cache[key]
        visibility_checks += 1
        if visibility_checks > MAX_VISIBILITY_CHECKS:
            raise TomogramError('refinement_visibility_limit', 'Visibility search exceeded its bounded budget')
        try:
            expanded = _expand_segment(tomogram, _line(reduced[first, :2], reduced[last, :2]), layer,
                                       reduced[first, 2], reduced[last, 2])
            if first == 0 and last == len(reduced)-1:
                # Retain only one bounded geometry array, not every visibility
                # candidate. Acceptance is NOT cached: final validation below
                # still re-reads the map before publishing this same geometry.
                direct_geometry = expanded
            cache[key] = True
        except TomogramError as error:
            if error.code.endswith('_limit'):
                raise
            cache[key] = False
        return cache[key]

    try:
        direct = visible(0, len(reduced)-1)
        if direct:
            polyline = reduced[[0, -1], :2]
        else:
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reduced[:, :2], axis=0), axis=1))]
            coarse = [0]
            for i in range(1, len(reduced)-1):
                if arc[i]-arc[coarse[-1]] >= VISIBILITY_ANCHOR_SPACING_M:
                    coarse.append(i)
            coarse.append(len(reduced)-1)
            selected, current = [0], 0
            while current < len(reduced)-1:
                chosen = next((index for index in reversed(coarse) if index > current and visible(current, index)), None)
                if chosen is None:
                    chosen = current + 1
                    if not visible(current, chosen):
                        return _rejected('no_checked_visibility_successor', before, visibility_checks=visibility_checks)
                selected.append(chosen)
                current = chosen
                if len(selected) > MAX_POLYLINE_VERTICES:
                    return _rejected('visibility_polyline_limit', before, visibility_checks=visibility_checks)
            polyline = reduced[selected, :2]
    except TomogramError as error:
        return _rejected(error.code, before, visibility_checks=visibility_checks)
    failures = []

    def evaluate(polyline_candidate, scale, candidate_index):
        try:
            segments = ([_line(polyline_candidate[0], polyline_candidate[-1])] if direct
                        else _rounded_segments(polyline_candidate, float(corner_cut_m)*scale))
            if direct:
                candidate, partition_points = direct_geometry
                # Horner's a+(b-a) at t=1 can differ from b by one ULP.
                # Keep exact selected endpoints, as _expanded_candidate does
                # for multi-segment curves, then recheck both connectors.
                candidate = candidate.copy()
                candidate[0], candidate[-1] = path[0], path[-1]
                checks = tomogram.validate_path(candidate, np.full(len(candidate), layer))
            else:
                candidate, partition_points, checks = _expanded_candidate(tomogram, segments, layer, path[0], path[-1])
            after = path_quality(candidate)
            conditioned = _conditioned_corridor_map(tomogram)
            if quality_policy == 'stair_xy':
                # Stair Z is sampled from the original measured treads. A
                # denser, straighter XY curve can accumulate more stair-height
                # variation; that is not a reason to retain lateral waviness.
                # All ground/step/headroom/polynomial checks above are unchanged.
                worse = [key for key in _quality_not_worse(before, after) if key != 'xyz_length_m']
                peak_tradeoff = None
            else:
                worse, peak_tradeoff = _corridor_quality(before, after, conditioned=conditioned,
                    curve_peak=_curve_peak_curvature(segments) if conditioned else None)
            if worse:
                failures.append({'candidate_index': candidate_index, 'cut_scale': scale,
                                 'reason': 'quality_not_improved', 'worse_metrics': worse,
                                 'after_quality': after})
                return None
            audit = [{key: value.tolist() if isinstance(value, np.ndarray) else value
                      for key, value in segment.items()} for segment in segments]
            return {'applied': True, 'reason': 'checked_straight_line' if direct else 'checked_c2_corners',
                    'path': candidate.tolist(), 'layer_ids': [layer]*len(candidate),
                    'algorithm': 'validated visibility straight line' if direct else 'validated visibility + quintic Bezier XY rounding',
                    'is_native_gpmp': False, 'before_quality': before, 'after_quality': after,
                    'before_cost_audit': before_cost,
                    'after_cost_audit': _cost_audit(tomogram, candidate, layer),
                    'soft_cost_policy': 'prefer_geometric_simplicity_within_unchanged_hard_pct_mask',
                    'physical_safety_certified': False,
                    'quality_acceptance': {
                        'policy': ('nonworsening_xy_geometry_with_original_measured_stair_z'
                                   if quality_policy == 'stair_xy' else
                                   'nonworsening_primary_geometry_with_explicit_local_peak_tradeoff'),
                        'local_curvature_peak_tradeoff': peak_tradeoff},
                    'visibility_candidate_index': candidate_index,
                    'visibility_candidate_policy': ('original_greedy' if candidate_index == 0 else
                        'bounded_adjacent_corner_intersections_ranked_by_checked_map_cost'),
                    'segments': audit, 'visibility_checks': visibility_checks,
                    'visibility_polyline_xy': polyline_candidate.tolist(), 'corner_cut_scale': scale,
                    'curve_partition_points': partition_points,
                    'curve_validation': 'polynomial_cell_boundary_roots_and_interval_interiors_plus_validate_path',
                    'continuity': 'XY geometric C2 under arc-length parameterization; measured-ground Z, no 3D or dynamic C2 guarantee',
                    'height_semantics': 'measured_ground_with_exact_selected_endpoints',
                    'prior_candidate_failures': list(failures), **checks}
        except TomogramError as error:
            failures.append({'candidate_index': candidate_index, 'cut_scale': scale,
                             'reason': error.code, 'details': error.details})
            return None

    for scale in ([1.] if direct else [1., .7, .4]):
        result = evaluate(polyline, scale, 0)
        if result is not None:
            return result
    # Ordinary successful requests keep their exact output and work budget.
    # Only a failed greedy corner gets up to four local alternatives / twelve
    # additional full-curve checks. No new visibility graph, map or inflation.
    accepted = []
    alternatives = [] if direct else _corner_intersection_candidates(polyline, float(corner_cut_m))
    for index, alternative in enumerate(alternatives, 1):
        for scale in (1., .7, .4):
            result = evaluate(alternative, scale, index)
            if result is not None:
                accepted.append(result)
                break
    if accepted:
        # Do not pick the shortest corner at the expense of all map cost. All
        # choices passed full map geometry and the original quality policy;
        # only a peak-curvature tradeoff imposes the analytic bound above.
        # Cost units are not physical wall clearance.
        result = min(accepted, key=lambda item: (
            item['after_cost_audit']['max_cost'],
            item['after_cost_audit']['mean_touched_cell_cost'],
            item['after_quality']['xy_length_m'],
            item['after_quality']['total_abs_turn_rad']))
        result['alternative_polylines_checked'] = len(alternatives)
        result['prior_candidate_failures'] = failures
        return result
    return _rejected('no_valid_nonworsening_smooth_candidate', before,
                     visibility_checks=visibility_checks, candidate_failures=failures,
                     visibility_polyline_xy=polyline.tolist(),
                     alternative_polylines_checked=len(alternatives))
