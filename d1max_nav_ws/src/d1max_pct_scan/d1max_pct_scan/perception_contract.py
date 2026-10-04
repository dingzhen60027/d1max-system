"""Production acquisition admission: bounded measured returns, not LIO points.

The cap matches both native decoders. No-return/NaN observations have no
free-space meaning in the current wire protocol; no consumer may invent one.
"""
import math

MAX_ACQUISITION_POINTS = 100000
CONTRACT = 'measured_returns_64b_v1'


def acquisition_budget(adapter, projector_limit):
    nested = adapter.get('perception_rays', {})
    def option(name, default):
        return adapter.get('perception_rays.' + name, nested.get(name, default))
    count = option('max_input_points', MAX_ACQUISITION_POINTS)
    if (type(count) is not int or type(projector_limit) is not int
            or not 1 <= count <= MAX_ACQUISITION_POINTS or count != projector_limit):
        raise ValueError('per_sensor_acquisition_budget_mismatch')
    near, far = option('min_range', 0.), option('max_range', 1000.)
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (near, far))
            or not 0 <= near < far <= 1000):
        raise ValueError('invalid_independent_safety_range')
    return dict(contract=CONTRACT, max_input_points=count, min_range=near,
                max_range=far, return_semantics='finite_measured_hits_only_invalid_unknown')
