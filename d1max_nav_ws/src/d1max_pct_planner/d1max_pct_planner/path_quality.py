"""Sampling-normalized geometric path diagnostics, not collision/safety checks.

Lengths and lateral extrema describe the entire input XY polyline. Heading and
curvature are computed after fixed-XY-arc-length resampling, retaining the exact
endpoints. Curvature is a finite-difference estimate at this stated resolution,
not a bound on the curvature of a hidden spline. Full-path metrics are always
returned alongside optional metrics excluding the first/last metre.
"""
from __future__ import annotations

import numpy as np


def _lateral(points, start, chord):
    length = float(np.hypot(*chord))
    if length == 0:
        return {'lateral_deviation_max_m': None, 'lateral_deviation_range_m': None,
                'lateral_deviation_min_signed_m': None, 'lateral_deviation_max_signed_m': None}
    relative = points - start
    unit = chord / length
    signed = unit[0] * relative[:, 1] - unit[1] * relative[:, 0]
    return {'lateral_deviation_max_m': float(np.max(np.abs(signed))),
            'lateral_deviation_range_m': float(np.ptp(signed)),
            'lateral_deviation_min_signed_m': float(np.min(signed)),
            'lateral_deviation_max_signed_m': float(np.max(signed))}


def _turn_statistics(turns, curvature):
    return {'total_abs_turn_rad': float(np.sum(np.abs(turns))),
            'total_abs_turn_deg': float(np.degrees(np.sum(np.abs(turns)))),
            'curvature_p95_per_m': float(np.percentile(np.abs(curvature), 95)) if len(curvature) else None,
            'curvature_max_per_m': float(np.max(np.abs(curvature))) if len(curvature) else None,
            'curvature_samples': int(len(curvature))}


def path_quality(path_xyz, spacing_m=0.1):
    """Return a JSON-serializable dict of full and endpoint-trimmed XY metrics.

    ``path_xyz`` is a nonempty finite Nx3 array. Repeated XY vertices (including
    pure-Z movement) do not invent XY headings. Zero chord makes length/chord and
    chord-relative deviation undefined (``None``). A singleton/pure-Z path has
    no XY curvature, also represented as ``None``. Closed XY paths include the
    heading change across the closure seam in full-path turn/curvature metrics.

    At most one million resampled vertices are permitted. No input mutation,
    smoothing, curve optimization, snapping, ROS, or obstacle checking occurs.
    """
    try:
        points = np.asarray(path_xyz, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError('path_xyz must be a numeric Nx3 array') from error
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError('path_xyz must be a nonempty Nx3 array')
    if not np.isfinite(points).all():
        raise ValueError('path_xyz must contain only finite coordinates')
    if isinstance(spacing_m, (bool, np.bool_)):
        raise ValueError('spacing_m must be finite and positive')
    try:
        spacing = float(spacing_m)
    except (TypeError, ValueError) as error:
        raise ValueError('spacing_m must be finite and positive') from error
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError('spacing_m must be finite and positive')
    with np.errstate(over='ignore', invalid='ignore'):
        differences = np.diff(points, axis=0)
        distance_xy = np.hypot(differences[:, 0], differences[:, 1])
        length_xy = float(distance_xy.sum())
        length_3d = float(np.hypot(distance_xy, differences[:, 2]).sum())
        chord = points[-1, :2] - points[0, :2]
        chord_length = float(np.hypot(*chord))
        z_range = float(np.ptp(points[:, 2]))
    if not np.isfinite([length_xy, length_3d, chord_length, z_range]).all():
        raise ValueError('Path coordinate range overflows finite distance arithmetic')
    epsilon = max(np.spacing(max(length_xy, spacing)) * 16, spacing * 1e-10)
    chord_tolerance = max(np.spacing(length_xy) * 16, length_xy * 1e-12)
    closed = length_xy > 0 and chord_length <= chord_tolerance
    chord_for_lateral = np.zeros(2) if closed else chord
    result = {'schema': 'd1max.path_quality/v1', 'status': 'ok',
              'input_points': int(len(points)), 'spacing_m': spacing,
              'xy_length_m': length_xy, 'xyz_length_m': length_3d,
              'xy_chord_m': chord_length,
              'length_chord_ratio': length_xy / chord_length if chord_length > chord_tolerance else None,
              'closed_xy': bool(closed), 'z_range_m': z_range,
              'xy_vertices': int(1 + np.count_nonzero(distance_xy > 0)),
              'safety_checked': False, 'warning_codes': [],
              **_lateral(points[:, :2], points[0, :2], chord_for_lateral)}
    interior = {'available': False, 'excluded_start_m': 1.0, 'excluded_end_m': 1.0,
                'retained_arclength_m': max(0.0, length_xy - 2.0),
                'total_abs_turn_rad': None, 'total_abs_turn_deg': None,
                'curvature_p95_per_m': None, 'curvature_max_per_m': None,
                'curvature_samples': 0,
                'lateral_deviation_max_m': None, 'lateral_deviation_range_m': None,
                'lateral_deviation_min_signed_m': None, 'lateral_deviation_max_signed_m': None}
    result['interior_excluding_1m'] = interior
    if length_xy == 0:
        result.update(status='no_xy_motion', resampled_points=1, resampled_xy_length_m=0.0,
                      total_abs_turn_rad=None, total_abs_turn_deg=None,
                      curvature_p95_per_m=None, curvature_max_per_m=None, curvature_samples=0)
        result['warning_codes'].append('no_xy_motion')
        return result
    with np.errstate(over='ignore'):
        sample_budget = length_xy / spacing
    if not np.isfinite(sample_budget) or sample_budget > 999_998:
        raise ValueError('Requested spacing exceeds one million resampled vertices')
    retained = np.r_[True, distance_xy > 0]
    xy = points[retained, :2]
    arc = np.r_[0., np.cumsum(distance_xy[distance_xy > 0])]
    samples = np.arange(0., length_xy, spacing)
    if len(samples) > 1 and length_xy - samples[-1] <= epsilon:
        samples[-1] = length_xy
    else:
        samples = np.r_[samples, length_xy]
    resampled = np.column_stack([np.interp(samples, arc, xy[:, axis]) for axis in range(2)])
    displacement = np.diff(resampled, axis=0)
    segment_lengths = np.hypot(displacement[:, 0], displacement[:, 1])
    usable = segment_lengths > 0
    headings = np.arctan2(displacement[usable, 1], displacement[usable, 0])
    heading_s = ((samples[:-1] + samples[1:]) / 2)[usable]
    angle = np.diff(headings)
    turns = np.arctan2(np.sin(angle), np.cos(angle))
    ds = np.diff(heading_s)
    curvature = turns / ds
    turn_s = (heading_s[:-1] + heading_s[1:]) / 2
    if closed and len(headings) > 1:
        seam_angle = headings[0] - headings[-1]
        seam_turn = float(np.arctan2(np.sin(seam_angle), np.cos(seam_angle)))
        seam_ds = length_xy - heading_s[-1] + heading_s[0]
        turns = np.r_[turns, seam_turn]
        curvature = np.r_[curvature, seam_turn / seam_ds]
        turn_s = np.r_[turn_s, 0.0]
    result.update(resampled_points=int(len(samples)), resampled_xy_length_m=float(segment_lengths.sum()),
                  **_turn_statistics(turns, curvature))
    if len(headings) < 2:
        result['status'] = 'insufficient_xy_heading_samples'
        result['warning_codes'].append('insufficient_xy_heading_samples')
        if len(headings) == 0:
            result['total_abs_turn_rad'] = result['total_abs_turn_deg'] = None
    if closed:
        result['warning_codes'].append('zero_xy_chord')
    if length_xy > 2.0:
        trim = (turn_s >= 1.0) & (turn_s <= length_xy - 1.0)
        sample_arc = np.r_[1.0, arc[(arc > 1.0) & (arc < length_xy - 1.0)], length_xy - 1.0]
        interior_xy = np.column_stack([np.interp(sample_arc, arc, xy[:, axis]) for axis in range(2)])
        interior.update(available=True, **_turn_statistics(turns[trim], curvature[trim]),
                        **_lateral(interior_xy, points[0, :2], chord_for_lateral))
    return result
