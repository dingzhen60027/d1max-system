"""Fresh continuous-pose evidence, independent of slow matching/UI status.

This is a snapshot of an ACTUAL navigation output, never a pose prediction or
permission to move. Its original source stamp cannot be extended by a status
heartbeat. The receiver must also bound its own monotonic receipt age.
"""

import math


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def continuous_pose_status(value, *, now, mono, received_mono, epoch, seed,
                           confirmed_seed, confirmations, local_fault,
                           map_frame, body_frame, status_timeout=.10):
    """Return (valid, state, reason), without reinterpreting matching health."""
    if local_fault:
        return False, 'fault', 'local_fault'
    if not isinstance(value, dict) or value.get('schema') != 1:
        return False, 'waiting', 'missing_pose_status'
    if (not _number(now) or not _number(mono) or not _number(received_mono)
            or not 0 <= mono-received_mono <= status_timeout):
        return False, 'paused', 'pose_status_receipt_stale'
    envelope = value.get('received_at_unix')
    if not _number(envelope) or envelope <= 0 or not -.01 <= now-envelope <= status_timeout:
        return False, 'paused', 'pose_status_source_stale'
    if (type(epoch) is not int or epoch < 1 or type(value.get('epoch')) is not int
            or value['epoch'] != epoch):
        return False, 'paused', 'pose_epoch_mismatch'
    if (not isinstance(seed, str) or not seed or seed != confirmed_seed
            or value.get('seed_id') != seed
            or type(confirmations) is not int or confirmations < 3):
        return False, 'paused', 'pose_seed_unconfirmed'
    if value.get('frame_id') != map_frame or value.get('body_frame') != body_frame:
        return False, 'fault', 'pose_frame_mismatch'
    if value.get('fault'):
        return False, 'fault', 'navigation_fault'
    if value.get('reset_pending') is not False:
        return False, 'initializing', 'filter_reset_pending'
    if value.get('valid') is not True or value.get('pose_valid') is not True:
        return False, 'paused', str(value.get('reason') or 'pose_output_unavailable')
    stamp, ttl = value.get('output_stamp_sec'), value.get('pose_timeout_sec')
    if (not _number(ttl) or not 0 < ttl <= .10 or not _number(stamp)
            or stamp <= 0 or not -.01 <= now-stamp <= ttl
            or stamp > envelope+.01):
        return False, 'paused', 'pose_output_source_stale'
    state = 'coasting' if value.get('quality') == 'coasting' else 'tracking'
    return True, state, 'fresh_continuous_pose'
