"""Offline, measured-structure centering of an already verified stair polyline.

This is route-quality preprocessing, not another planner or a runtime motion
gate. It never edits a map, Z, layer ownership, platform or non-flight geometry.
Walls/rails come from the retained planning cloud, not an inflated cost contour.
The returned polyline has no polynomial execution certificate. Existing full
map checks and the caller's original-source support proof remain mandatory.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from numpy.polynomial import Polynomial
from scipy.spatial import cKDTree

from .path_quality import path_quality


@dataclass(frozen=True)
class CenteringPolicy:
    """Offline fitting scales; none of these relax map validity or clearance."""

    structure_bin_m: float = 0.20
    minimum_points_per_bin: int = 5
    minimum_side_bins: int = 5
    minimum_side_coverage: float = 0.65
    normal_neighbors: int = 12
    maximum_normal_z: float = 0.30
    maximum_plane_rms_m: float = 0.04
    maximum_neighborhood_radius_m: float = 0.22
    structure_height_min_m: float = 0.25
    structure_height_max_m: float = 1.25
    structure_lateral_min_m: float = 0.20
    structure_lateral_max_m: float = 1.20
    flight_height_association_m: float = 0.20
    maximum_lateral_change_m: float = 0.30
    smoothing_length_m: float = 0.50
    evaluation_spacing_m: float = 0.025
    minimum_center_error_reduction: float = 0.10


def _xyz(value, name, minimum=1):
    result = np.asarray(value, dtype=np.float64)
    if (result.ndim != 2 or result.shape[1] != 3 or len(result) < minimum
            or not np.isfinite(result).all()):
        raise ValueError(name + '_must_be_finite_xyz')
    return result


def _policy(value):
    value = CenteringPolicy() if value is None else value
    if not isinstance(value, CenteringPolicy):
        raise ValueError('centering_policy_type')
    for name, number in asdict(value).items():
        if isinstance(number, bool) or not np.isfinite(number) or number <= 0:
            raise ValueError('centering_policy_invalid:' + name)
    if (not 0 < value.minimum_side_coverage <= 1 or value.maximum_normal_z > 1
            or not 0 < value.minimum_center_error_reduction < 1
            or value.structure_height_min_m >= value.structure_height_max_m
            or value.structure_lateral_min_m >= value.structure_lateral_max_m):
        raise ValueError('centering_policy_bounds')
    for name in ('minimum_points_per_bin', 'minimum_side_bins', 'normal_neighbors'):
        if type(getattr(value, name)) is not int:
            raise ValueError('centering_policy_integer:' + name)
    return value


def _frame(flight):
    center = np.asarray(flight['center_xy'], dtype=float)
    axis = np.asarray(flight['uphill_axis_xy'], dtype=float)
    bounds = np.asarray(flight['along_range_m'], dtype=float)
    if (center.shape != (2,) or axis.shape != (2,) or bounds.shape != (2,)
            or not np.isfinite(np.r_[center, axis, bounds, flight['grade'], flight['intercept_m']]).all()
            or abs(np.linalg.norm(axis) - 1.) > 1e-6 or bounds[1] <= bounds[0]
            or flight['leg'] not in ('stair_lower', 'stair_upper')):
        raise ValueError('measured_flight_frame_invalid')
    return center, axis, np.array([-axis[1], axis[0]]), bounds


def extract_measured_flight_structure(planning_xyz, models, *, policy=None):
    """Extract current retained vertical returns with source row provenance.

    ``models`` are existing measured-flight models, each explicitly naming its
    ``leg``. Neighbourhood geometry is derived within a bounded expanded flight
    ROI; source point coordinates are not modified. A missing side is an error,
    not permission to use the PCT contour as an invented wall.
    """
    p = _policy(policy)
    cloud = _xyz(planning_xyz, 'planning_cloud', p.normal_neighbors)
    flights = []
    selected_models = [dict(m) for m in models
                       if m.get('kind') == 'flight_support_envelope' or 'leg' in m]
    if any('leg' not in m for m in selected_models):
        if len(selected_models) != 2 or any('leg' in m for m in selected_models):
            raise ValueError('measured_flight_explicit_ownership_required')
        selected_models.sort(key=lambda m: m['intercept_m'])
        for model, leg in zip(selected_models, ('stair_lower', 'stair_upper')):
            model['leg'] = leg
    for model in selected_models:
        center, axis, lateral, bounds = _frame(model)
        s = (cloud[:, :2] - center) @ axis
        t = (cloud[:, :2] - center) @ lateral
        elevation = cloud[:, 2] - (model['grade'] * s + model['intercept_m'])
        pad = p.maximum_neighborhood_radius_m * 2
        roi = ((s >= bounds[0] - pad) & (s <= bounds[1] + pad)
            & (abs(t) <= p.structure_lateral_max_m + pad)
            & (elevation >= p.structure_height_min_m - pad)
            & (elevation <= p.structure_height_max_m + pad))
        rows = np.flatnonzero(roi)
        if len(rows) < p.normal_neighbors:
            raise ValueError('measured_structure_insufficient_points')
        xyz = cloud[rows]
        distances, neighbors = cKDTree(xyz).query(xyz, k=p.normal_neighbors, workers=1)
        near = xyz[neighbors]
        near = near - near.mean(axis=1)[:, None, :]
        eigenvalues, eigenvectors = np.linalg.eigh(
            np.einsum('nki,nkj->nij', near, near) / p.normal_neighbors)
        keep = ((s[rows] >= bounds[0]) & (s[rows] <= bounds[1])
            & (abs(t[rows]) >= p.structure_lateral_min_m)
            & (abs(t[rows]) <= p.structure_lateral_max_m)
            & (elevation[rows] >= p.structure_height_min_m)
            & (elevation[rows] <= p.structure_height_max_m)
            & (abs(eigenvectors[:, 2, 0]) <= p.maximum_normal_z)
            & (np.sqrt(np.maximum(eigenvalues[:, 0], 0)) <= p.maximum_plane_rms_m)
            & (distances[:, -1] <= p.maximum_neighborhood_radius_m))
        flight = dict(model, structure_xyz=xyz[keep].copy(),
                      structure_source_indices=rows[keep].copy())
        # Fail before a candidate is generated if either measured side lacks
        # longitudinal coverage. The fitted middle is not a free-space proof.
        fit_measured_sides(flight, policy=p)
        flights.append(flight)
    return flights


def _robust_line(s, t):
    keep = np.ones(len(s), dtype=bool)
    for _ in range(5):
        line = np.polyfit(s[keep], t[keep], 1)
        residual = t - np.polyval(line, s)
        mad = np.median(abs(residual[keep] - np.median(residual[keep])))
        selected = abs(residual) <= max(0.03, 3 * 1.4826 * mad)
        if selected.sum() < 4 or np.array_equal(selected, keep):
            break
        keep = selected
    return line, keep


def fit_measured_sides(flight, *, policy=None):
    p = _policy(policy)
    center, axis, lateral, bounds = _frame(flight)
    xyz = _xyz(flight['structure_xyz'], 'measured_structure')
    s, t = (xyz[:, :2] - center) @ axis, (xyz[:, :2] - center) @ lateral
    fitted, details = {}, {}
    for name, sign in (('negative', -1), ('positive', 1)):
        bins = []
        for start in np.arange(bounds[0], bounds[1] - p.structure_bin_m / 2, p.structure_bin_m):
            selected = ((s >= start) & (s < min(bounds[1], start + p.structure_bin_m))
                        & (sign * t > p.structure_lateral_min_m))
            if selected.sum() >= p.minimum_points_per_bin:
                bins.append([float(np.median(s[selected])), float(np.median(t[selected])),
                             int(selected.sum())])
        if len(bins) < p.minimum_side_bins:
            raise ValueError('measured_structure_side_missing:' + name)
        b = np.asarray(bins)
        line, keep = _robust_line(b[:, 0], b[:, 1])
        if (keep.sum() < p.minimum_side_bins or
                np.ptp(b[keep, 0]) / np.ptp(bounds) < p.minimum_side_coverage):
            raise ValueError('measured_structure_side_coverage:' + name)
        fitted[name] = line
        details[name] = dict(bins=bins, accepted_bins=np.flatnonzero(keep).tolist(),
                             lateral_slope_intercept=line.tolist())
    width = np.polyval(fitted['positive'] - fitted['negative'], bounds)
    if np.min(width) <= 0:
        raise ValueError('measured_structure_sides_cross')
    return dict(sides=details,
        center_slope_intercept=((fitted['negative'] + fitted['positive']) / 2).tolist(),
        width_at_endpoints_m=width.tolist())


def _center_transition(start, end, start_slope, end_slope, length, *, middle_at_end):
    """Minimum spatial jerk with literal outer tangent and C2 affine join.

    The old route is a polyline: there is no measured outer acceleration to
    impose. With its second derivative free, minimizing integral(y'''**2)
    yields the natural condition y'''=0 at the outer end. At the affine end,
    value, slope and second derivative are fixed. This avoids the spurious
    extra curvature caused by forcing BOTH ends to zero second derivative.
    Derivatives are spatial, not a robot time/acceleration certificate.
    """
    if not middle_at_end:
        reverse = _center_transition(end, start, -end_slope, -start_slope,
                                     length, middle_at_end=True)
        return reverse(Polynomial([1., -1.]))
    delta = end - start
    a, b = length * start_slope, length * end_slope
    return Polynomial([start, a, (10 * delta - 6 * a - 4 * b) / 3., 0.,
                       -5 * delta + 2 * a + 3 * b, (8 * delta - 3 * a - 5 * b) / 3.])


def _centered_lateral(s, t, center_line, smoothing_length, structure_st, minimum_distance,
                      minimum_reduction, *, endpoint_slopes=None, return_profile=False):
    """Choose finite entry/exit blends, leaving an EXACT measured middle line.

    This is an offline geometric fit, not another planner or runtime gate.
    A bounded length grid selects the lowest-curvature feasible connection;
    the old side-biased endpoints no longer bend the entire flight. Each
    quintic has zero second derivative at the middle join. External input is
    a polyline, so only its literal endpoint chord (C1), not a fictitious old
    continuous acceleration, is preserved. Actual output chords retain the
    original distance to every supplied wall/railing return.
    """
    s, t = np.asarray(s, float), np.asarray(t, float)
    center_line, structure_st = np.asarray(center_line, float), np.asarray(structure_st, float)
    length = float(s[-1] - s[0])
    if (length <= 1e-6 or np.any(np.diff(s) < -1e-9) or s[1] <= s[0] or s[-1] <= s[-2]
            or structure_st.ndim != 2 or structure_st.shape[1] != 2 or not len(structure_st)):
        raise ValueError('structured_center_invalid_geometry')
    slope0, slope1 = endpoint_slopes if endpoint_slopes is not None else (
        (t[1] - t[0]) / (s[1] - s[0]), (t[-1] - t[-2]) / (s[-1] - s[-2]))
    # At least a quarter of the flight remains affine; short flights scale the
    # existing smoothing length instead of hiding an overlapping blend.
    middle_length = max(min(smoothing_length, length / 3), length / 4)
    minimum_blend = min(smoothing_length, length / 4)
    maximum_blend = length - middle_length - minimum_blend
    lengths = np.linspace(minimum_blend, maximum_blend, 17)
    station = np.unique(np.r_[np.linspace(s[0], s[-1], 257), s])
    desired = np.polyval(center_line, station)
    old_error = float(np.mean(abs(np.interp(station, s, t) - desired)))
    structure_xyz = np.c_[structure_st, np.zeros(len(structure_st))]
    candidates = []
    for entry_length in lengths:
        for exit_length in lengths:
            if entry_length + exit_length > length - middle_length + 1e-9:
                continue
            begin, end = s[0] + entry_length, s[-1] - exit_length
            first = _center_transition(t[0], np.polyval(center_line, begin),
                                       slope0, center_line[0], entry_length, middle_at_end=True)
            last = _center_transition(np.polyval(center_line, end), t[-1],
                                      center_line[0], slope1, exit_length, middle_at_end=False)
            def evaluate(at, derivative=0):
                values = np.polyval(center_line, at) if derivative == 0 else np.full(
                    np.shape(at), center_line[0] if derivative == 1 else 0.)
                left, right = at < begin, at > end
                values[left] = first.deriv(derivative)((at[left] - s[0]) / entry_length) / entry_length ** derivative
                values[right] = last.deriv(derivative)((at[right] - end) / exit_length) / exit_length ** derivative
                return values
            sampled = evaluate(station)
            error = float(np.mean(abs(sampled - desired)))
            if error > (1 - minimum_reduction) * old_error + 1e-12:
                continue
            derivative, second = evaluate(station, 1), evaluate(station, 2)
            peak = float(np.max(abs(second) / (1 + derivative ** 2) ** 1.5))
            total_turn = float(np.sum(abs(np.diff(np.arctan(derivative)))))
            values = evaluate(s)
            values[[0, -1]] = t[[0, -1]]
            candidates.append(((peak, total_turn, error), values, first, last, begin, end))
    # Expensive exact segment-to-return checks run in quality order, not in
    # the control lane or on an unbounded set of new optimization attempts.
    for score, values, first, last, begin, end in sorted(candidates, key=lambda item: item[0]):
        measured_distance = _minimum_structure_distance(np.c_[s, values, np.zeros(len(s))], structure_xyz)
        if measured_distance + 1e-6 < minimum_distance:
            continue
        transitions = []
        for polynomial, a, b, d0, d1 in ((first, s[0], begin, slope0, center_line[0]),
                                        (last, end, s[-1], center_line[0], slope1)):
            transitions.append(dict(along_range_m=[float(a), float(b)], length_m=float(b-a),
                normalized_coefficients=polynomial.coef.tolist(), start_value_m=float(polynomial(0.)),
                end_value_m=float(polynomial(1.)), start_slope=float(d0), end_slope=float(d1)))
        profile = dict(kind='c2_entry_exact_middle_c2_exit',
            center_along_range_m=[float(begin), float(end)],
            center_line=list(map(float, center_line)), transitions=transitions,
            sampled_curve_curvature_max_per_m=score[0], sampled_curve_total_turn_deg=float(np.rad2deg(score[1])),
            actual_structure_distance_minimum_m=float(measured_distance),
            evaluated_length_pairs=len(candidates), center_error_mean_absolute_m=score[2],
            endpoint_contract='literal_values_and_outer_polyline_chord_slopes; C2 at affine joins')
        return (values, profile) if return_profile else values
    raise ValueError('structured_center_no_feasible_transition')


def _dense_xyz(points, spacing):
    distance = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    arc = np.r_[0., np.cumsum(distance)]
    keep = np.r_[True, distance > 1e-12]
    stations = np.r_[np.arange(0, arc[-1], spacing), arc[-1]]
    return np.column_stack([np.interp(stations, arc[keep], points[keep, axis]) for axis in range(3)])


def _minimum_structure_distance(points, structure):
    """Exact XY segment-to-return distance, including between output vertices."""
    a, delta = points[:-1, :2], np.diff(points[:, :2], axis=0)
    length2 = np.einsum('ij,ij->i', delta, delta)
    minimum = np.inf
    # Bounded temporary storage even for high-density retained clouds.
    for begin in range(0, len(structure), 256):
        relative = structure[begin:begin + 256, None, :2] - a[None, :, :]
        dot = np.einsum('kij,ij->ki', relative, delta)
        ratio = np.divide(dot, length2[None, :], out=np.zeros_like(dot), where=length2[None, :] > 0)
        residual = relative - np.clip(ratio, 0., 1.)[:, :, None] * delta[None, :, :]
        minimum = min(minimum, float(np.sqrt(np.min(np.einsum('kij,kij->ki', residual, residual)))))
    return minimum


def flight_quality(points, flight, fit, *, policy=None):
    p = _policy(policy)
    center, axis, lateral, _ = _frame(flight)
    dense = _dense_xyz(_xyz(points, 'flight_path', 2), p.evaluation_spacing_m)
    s, t = (dense[:, :2] - center) @ axis, (dense[:, :2] - center) @ lateral
    error = t - np.polyval(fit['center_slope_intercept'], s)
    structure = _xyz(flight['structure_xyz'], 'structure')
    distance = cKDTree(structure[:, :2]).query(
        dense[:, :2], workers=1)[0]
    return dict(center_error_mean_absolute_m=float(np.mean(abs(error))),
        center_error_maximum_m=float(np.max(abs(error))),
        actual_retained_structure_distance_minimum_m=_minimum_structure_distance(points, structure),
        actual_retained_structure_distance_p10_m=float(np.quantile(distance, .1)),
        actual_retained_structure_distance_median_m=float(np.median(distance)),
        quality_10cm=path_quality(points),
        note='Centerline XY distance to finite retained returns; not body clearance or physical acceptance.')


def center_measured_stair_path(tomogram, points, layer_ids, edge_legs, *,
        settings, flights, validate_candidate, policy=None):
    """Return a bounded offline candidate only after original geometry proofs.

    ``validate_candidate(xyz, layers)`` must run the package's source-route
    builder. Raising or returning False rejects the candidate atomically. There
    is no weakened-map fallback and no change to execution authorization.
    """
    p = _policy(policy)
    original = _xyz(points, 'path', 3).copy()
    layers = np.asarray(layer_ids)
    owners = np.asarray(edge_legs)
    if (layers.shape != (len(original),) or not np.issubdtype(layers.dtype, np.integer)
            or owners.shape != (len(original) - 1,) or not callable(validate_candidate)):
        raise ValueError('stair_centering_route_contract')
    if not flights or len(flights) > 2 or len({f['leg'] for f in flights}) != len(flights):
        raise ValueError('stair_centering_flight_contract')
    candidate = original.copy()
    reports, spans, claimed = [], [], set()
    for flight in flights:
        center, axis, lateral, bounds = _frame(flight)
        fit = fit_measured_sides(flight, policy=p)
        s = (original[:, :2] - center) @ axis
        z = flight['grade'] * s + flight['intercept_m']
        owner = np.r_[False, owners == flight['leg']] & np.r_[owners == flight['leg'], False]
        selected = np.flatnonzero(owner & (s >= bounds[0]) & (s <= bounds[1])
            & (abs(original[:, 2] - z) <= p.flight_height_association_m))
        if len(selected) < 8 or np.any(np.diff(selected) != 1):
            raise ValueError('stair_centering_flight_not_contiguous')
        first, last = int(selected[0]), int(selected[-1])
        if claimed.intersection(selected):
            raise ValueError('stair_centering_flight_overlap')
        claimed.update(selected)
        segment = original[first:last + 1]
        along = (segment[:, :2] - center) @ axis
        transverse = (segment[:, :2] - center) @ lateral
        ds = np.diff(along)
        if (np.any(ds < -1e-8) or np.any((abs(ds) <= 1e-8) &
                (np.linalg.norm(np.diff(segment[:, :2], axis=0), axis=1) > 1e-8))
                or np.min(ds[[1, -2]]) <= 1e-8):
            raise ValueError('stair_centering_flight_not_monotone')
        old_quality = flight_quality(segment, flight, fit, policy=p)
        structure = _xyz(flight['structure_xyz'], 'structure')
        structure_st = np.column_stack(((structure[:, :2] - center) @ axis,
                                         (structure[:, :2] - center) @ lateral))
        new_t = transverse.copy()
        new_t[1:-1], profile = _centered_lateral(along[1:-1], transverse[1:-1],
            fit['center_slope_intercept'], p.smoothing_length_m, structure_st,
            old_quality['actual_retained_structure_distance_minimum_m'], p.minimum_center_error_reduction,
            endpoint_slopes=((transverse[1]-transverse[0])/ds[0],
                             (transverse[-1]-transverse[-2])/ds[-1]), return_profile=True)
        displacement = new_t - transverse
        if max(abs(displacement)) > p.maximum_lateral_change_m:
            raise ValueError('stair_centering_change_exceeds_bound')
        candidate[first:last + 1, :2] += displacement[:, None] * lateral
        # Preserve exact unchanged records; reconstruction of the whole XY
        # through an orthogonal frame would otherwise introduce roundoff.
        candidate[[first, first + 1, last - 1, last]] = original[[first, first + 1, last - 1, last]]
        new_quality = flight_quality(candidate[first:last + 1], flight, fit, policy=p)
        reports.append(dict(flight_id=int(flight['id']), leg=flight['leg'], fit=fit,
            before=old_quality, after=new_quality,
            maximum_xy_change_m=float(max(abs(displacement))), structured_profile=profile,
            structure_source_indices=np.asarray(flight.get('structure_source_indices', []), int).tolist()))
        spans.append(dict(leg=flight['leg'], first_index=first, last_index=last))
    reason = None
    if any(r['after']['actual_retained_structure_distance_minimum_m'] + 1e-6 <
            r['before']['actual_retained_structure_distance_minimum_m'] for r in reports):
        reason = 'measured_structure_distance_regressed'
    if any(r['after']['center_error_mean_absolute_m'] >=
            r['before']['center_error_mean_absolute_m'] for r in reports):
        reason = reason or 'centering_not_improved'
    # A straight but side-biased old line needs two genuine transitions to
    # reach the middle. Requiring its total turn to decrease is contradictory;
    # summing both flights also hid a bad flight behind the other one's gain.
    # Each flight now has a measured affine middle, a bounded pair of blends,
    # improved centre error and nondecreasing actual structure distance. Its
    # individual curvature/turn before and after remain explicit diagnostics.
    checks = {}
    if reason is None:
        from .crossfloor_route import _masked_tomogram, _check_joined
        try:
            checks['original'] = tomogram.validate_path(candidate, layers,
                max_ground_step_m=settings['limits']['max_ground_step_m'])
            checks['stair_mask'] = _masked_tomogram(tomogram, 'stair_lower', settings).validate_path(
                candidate, layers, max_ground_step_m=settings['limits']['max_ground_step_m'])
            checks['joined'] = _check_joined(candidate, layers, owners.tolist(), tomogram, settings)
            source_result = validate_candidate(candidate.copy(), layers.copy())
            if source_result is False:
                raise ValueError('source_route_rejected')
            checks['source'] = source_result
        except (ValueError, TypeError, IndexError) as error:
            reason = 'geometry_or_source_rejected:' + str(getattr(error, 'code', str(error)))
    accepted = reason is None
    result = candidate if accepted else original
    return dict(path=result, layer_ids=layers.copy(), applied=accepted, flight_spans=spans,
        diagnostics=dict(schema='d1max.measured_stair_centering/v1',
            policy=asdict(p), flights=reports, flight_spans=spans,
            candidate_rejection=reason, checks=checks,
            changed_vertices=np.flatnonzero(np.any(result != original, axis=1)).tolist(),
            z_unchanged=bool(np.array_equal(result[:, 2], original[:, 2])),
            whole_before=path_quality(original), whole_after=path_quality(result),
            execution_authorized=False))
