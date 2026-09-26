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
OUTPUT_SPACING_M = 0.10
VISIBILITY_ANCHOR_SPACING_M = 0.50


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


def refine_corridor(tomogram, path_xyz, layer_ids, corner_cut_m=1.5):
    """Return a validated explicit candidate, or ``applied=False`` with reason.

    Original route validity is a precondition verified here, never assumed.
    Greedy visibility uses approximately 0.5 m *original* anchors, and falls
    back to adjacent original vertices rather than dropping blocked corners.
    Every proposed shortcut follows measured ground and uses full boundary
    checks. Up to three cut scales (1, .7, .4) are tested; no sharp-polyline
    fallback is silently published if rounded candidates fail.
    """
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
    for scale in ([1.] if direct else [1., .7, .4]):
        try:
            segments = ([_line(polyline[0], polyline[-1])] if direct
                        else _rounded_segments(polyline, float(corner_cut_m)*scale))
            if direct:
                candidate, partition_points = direct_geometry
                checks = tomogram.validate_path(candidate, np.full(len(candidate), layer))
            else:
                candidate, partition_points, checks = _expanded_candidate(tomogram, segments, layer, path[0], path[-1])
            after = path_quality(candidate)
            worse = _quality_not_worse(before, after)
            if worse:
                failures.append({'cut_scale': scale, 'reason': 'quality_not_improved', 'worse_metrics': worse,
                                 'after_quality': after})
                continue
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
                    'segments': audit, 'visibility_checks': visibility_checks,
                    'visibility_polyline_xy': polyline.tolist(), 'corner_cut_scale': scale,
                    'curve_partition_points': partition_points,
                    'curve_validation': 'polynomial_cell_boundary_roots_and_interval_interiors_plus_validate_path',
                    'continuity': 'XY geometric C2 under arc-length parameterization; measured-ground Z, no 3D or dynamic C2 guarantee',
                    'height_semantics': 'measured_ground_with_exact_selected_endpoints',
                    'prior_candidate_failures': failures, **checks}
        except TomogramError as error:
            failures.append({'cut_scale': scale, 'reason': error.code, 'details': error.details})
    return _rejected('no_valid_nonworsening_smooth_candidate', before,
                     visibility_checks=visibility_checks, candidate_failures=failures,
                     visibility_polyline_xy=polyline.tolist())
