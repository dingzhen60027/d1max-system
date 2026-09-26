"""Native ray-integration receipts, never a collision/robot-motion permit."""
import math


def integrated_ray_stamp(value, context, *, now, timeout, barrier):
    if not isinstance(value, dict) or not isinstance(context, dict):
        raise ValueError('ray_context_unavailable')
    if type(value.get('schema')) is not int or value['schema'] != 1:
        raise ValueError('invalid_ray_receipt_schema')
    for key in ('session_id', 'epoch', 'seed_id', 'sequence', 'barrier_ns'):
        if type(value.get(key)) is not type(context.get(key)) or value.get(key) != context.get(key):
            raise ValueError('ray_receipt_context_mismatch')
    received = value.get('received_at_unix')
    stamp_ns = value.get('source_stamp_ns')
    if (type(received) not in (int, float) or not math.isfinite(received)
            or not -.1 <= now-received <= timeout
            or type(stamp_ns) is not int or stamp_ns <= 0):
        raise ValueError('ray_receipt_stale')
    stamp = stamp_ns * 1e-9
    if not -.1 <= now-stamp <= timeout or stamp <= barrier:
        raise ValueError('ray_measurement_stale_or_before_barrier')
    if value.get('valid') is not True:
        raise ValueError('ray_integration_unavailable:' + str(value.get('reason', ''))[:100])
    return stamp
