"""Pure presentation of native SCAN splines; no admission or geometry repair.

Port of ROS 2 PlanningVisualization::displayOptimalTraj at d0b921c9:
evaluate the original spline and its derivative every ~0.1 s, then map the
trajectory's speed range from yellow to red. The caller owns identity pairing,
time leases, Marker publication and deletion. This module never grants motion.
"""
import math

import numpy as np
from scipy.interpolate import BSpline


def official_spline_style(*, order, knots, points):
    """Return unchanged native geometry and per-point upstream speed colours.

    ``points`` are B-spline control points, not sampled line points. ``speeds``
    are m/s when native points/timestamps use metres/seconds; colours are
    normalized within this trajectory, exactly as the official renderer does.
    Sample count, line width and sphere diameter match official ROS 2.
    """
    pts = np.asarray(points, dtype=float)
    knots = np.asarray(knots, dtype=float)
    if (type(order) is not int or order != 3 or pts.ndim != 2 or pts.shape[1] != 3
            or not 4 <= len(pts) <= 10000 or knots.ndim != 1
            or len(knots) != len(pts)+order+1 or not np.isfinite(pts).all()
            or not np.isfinite(knots).all() or not np.all(np.diff(knots) > 1e-9)):
        raise ValueError('invalid_native_cubic_spline')
    begin, end = float(knots[order]), float(knots[len(pts)])
    duration = end-begin
    if not .01 < duration <= 120.:
        raise ValueError('invalid_native_spline_duration')
    spline = BSpline(knots, pts, order, extrapolate=False)
    count = max(2, math.ceil(duration/.1)+1)
    times = np.linspace(begin, end, count)
    sampled = np.asarray(spline(times))
    speeds = np.linalg.norm(spline.derivative()(times), axis=1)
    if not np.isfinite(sampled).all() or not np.isfinite(speeds).all():
        raise ValueError('nonfinite_native_spline_samples')
    speed_range = max(1e-6, float(speeds.max()-speeds.min()))
    ratio = np.clip((speeds-speeds.min())/speed_range, 0., 1.)
    colors = np.column_stack((np.ones(count), 1.-ratio,
                              np.zeros(count), np.ones(count)))
    return {'points': sampled, 'colors': colors, 'speeds': speeds,
            'times': times-begin, 'line_width': .08, 'point_diameter': .08}
