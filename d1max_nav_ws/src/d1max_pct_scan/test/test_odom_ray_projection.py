"""Production ray projection geometry, not a substitute occupancy map."""
from dataclasses import replace

import numpy as np
import pytest

from d1max_pct_scan.ray_projection import RayProjectorCore, ProjectionError
from test_ray_projection import CTX, EPOCH, IDENTITY, decode, feed, planar, project, raw_points


def odom_core():
    value = RayProjectorCore(projection_frame='odom')
    value.reset(CTX)
    value.set_extrinsics(body_to_tracking=IDENTITY, ray_to_tracking=IDENTITY)
    return value


def test_cumulative_map_correction_does_not_move_or_disable_local_geometry():
    baseline, corrected = odom_core(), odom_core()
    feed(baseline)
    # 0.16 m of normal accumulated correction exceeded the old map threshold.
    feed(corrected, correction=lambda t: planar(t, yaw=t/2))
    a, b = project(baseline, decode(raw_points())), project(corrected, decode(raw_points()))
    assert corrected.fault is None
    assert corrected.alignment_displacement['translation_m'] > .10
    assert a.points.tobytes() == b.points.tobytes()
    assert a.start_ns == b.start_ns == EPOCH
    assert b.projection_frame == 'odom'


@pytest.mark.parametrize('sensor,origin', [(0, (.40, 0., 0.)), (1, (-.40, 0., 0.))])
def test_odom_motion_compensation_preserves_each_sensor_origin_and_time(sensor, origin):
    value = odom_core()
    feed(value, local=lambda t: planar(t), correction=lambda t: planar(10., 8., 1.))
    raw = decode(raw_points(sensor=sensor, origin=origin))
    output = project(value, raw)
    t = raw.points['offset_time'] * 1e-9
    np.testing.assert_allclose(output.points['x'], 2.+t, atol=2e-7)
    np.testing.assert_allclose(output.points['origin_x'], origin[0]+t, atol=2e-7)
    for field in ('timestamp', 'source_timestamp', 'raw_timestamp', 'offset_time', 'sensor_id'):
        np.testing.assert_array_equal(output.points[field], raw.points[field])


def test_frame_switch_cannot_commit_inflight_map_geometry():
    value = odom_core()
    feed(value)
    snapshot = value.projection_snapshot()
    projected = project(snapshot, decode(raw_points()))
    with pytest.raises(ProjectionError):
        value.commit_projection(replace(projected, projection_frame='map'))
    assert value.sequence == 0


def test_local_context_reset_cannot_reuse_old_pose_history():
    value = odom_core()
    feed(value)
    old = value.projection_snapshot()
    projected = project(old, decode(raw_points()))
    value.reset(replace(CTX, epoch=2, sequence=2, barrier_ns=EPOCH+170000000))
    with pytest.raises(ProjectionError):
        value.commit_projection(projected)
    assert not value.local


def test_odom_mode_does_not_hide_abrupt_global_identity_geometry_jump():
    value = odom_core()
    feed(value, count=2)
    with pytest.raises(ProjectionError, match='map_alignment_jump'):
        feed(value, count=1, start=EPOCH+40000000, correction=lambda t: planar(1.))


def test_odom_mode_still_requires_real_authorized_watermark():
    value = odom_core()
    feed(value)
    with pytest.raises(ProjectionError):
        project(value, decode(raw_points()), authorized_pose_ns=EPOCH+90000000)
