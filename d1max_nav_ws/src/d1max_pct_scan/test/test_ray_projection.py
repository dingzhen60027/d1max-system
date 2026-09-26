"""Offline, pure geometry/contract tests. Never initializes ROS or a robot SDK."""
from dataclasses import replace
import math

import numpy as np
import pytest

from d1max_localization.math_utils import Pose3, compose, inverse, interpolate_pose
from d1max_pct_scan.ray_projection import (
    AwaitingCoverage, FIELDS, Limits, MapContext, ProjectionError, RAY_DTYPE,
    RayProjectorCore, RawRays, checked_pose, decode_rays, interpolate_many,
    session_settings, transform_many)

EPOCH = 1790412195000000000
CTX = MapContext('session', 1, 'seed', 1, EPOCH-1)
IDENTITY = Pose3((0., 0., 0.), (0., 0., 0., 1.))


def planar(x=0., y=0., yaw=0.):
    return Pose3((x, y, 0.), (0., 0., math.sin(yaw/2), math.cos(yaw/2)))


def raw_points(n=31, sensor=0, origin=(0., 0., 0.), start=EPOCH, duration=100000000):
    points = np.zeros(n, dtype=RAY_DTYPE)
    offsets = np.linspace(0, duration, n).astype(np.uint32)
    points['x'] = 2.
    points['intensity'] = np.arange(n)
    for name, value in zip(('origin_x', 'origin_y', 'origin_z'), origin):
        points[name] = value
    points['ring'] = np.arange(n) % 96
    points['sensor_id'] = sensor
    points['offset_time'] = offsets
    points['source_index'] = np.arange(n)*4
    points['timestamp'] = start*1e-9 + offsets.astype(float)*1e-9
    points['source_timestamp'] = points['timestamp'] - 23.
    points['raw_timestamp'] = points['source_timestamp'] * 1e6
    return points


def decode(points, **changes):
    args = dict(data=points.tobytes(), fields=FIELDS, point_step=64,
                row_step=len(points)*64, width=len(points), height=1, bigendian=False,
                header_ns=EPOCH, frame_id='lidar', expected_frame='lidar')
    args.update(changes)
    return decode_rays(**args)


def core(body=IDENTITY, ray=IDENTITY, limits=Limits()):
    value = RayProjectorCore(limits)
    value.reset(CTX)
    value.set_extrinsics(body_to_tracking=body, ray_to_tracking=ray)
    return value


def feed(value, *, local=lambda t: planar(t, yaw=t), correction=lambda t: IDENTITY,
         reverse_arrival=False, count=9, start=EPOCH, step=20000000):
    for i in range(count):
        stamp = start+i*step
        tracking = local(i*step*1e-9)
        body = compose(tracking, inverse(value.body_to_tracking))
        global_tracking = compose(correction(i*step*1e-9), tracking)
        if reverse_arrival:
            value.add_global_tracking(stamp, global_tracking, value.context)
            value.add_local_body(stamp, body, value.context)
        else:
            value.add_local_body(stamp, body, value.context)
            value.add_global_tracking(stamp, global_tracking, value.context)


def project(value, raw, **changes):
    args = dict(now_ns=EPOCH+180000000, authorized_pose_ns=EPOCH+160000000)
    args.update(changes)
    return value.project(raw, value.context, **args)


def test_decode_preserves_all_fields_and_original_acquisition_order():
    points = raw_points()
    points['offset_time'][[1, 2]] = points['offset_time'][[2, 1]]
    for name in ('timestamp', 'source_timestamp', 'raw_timestamp'):
        points[name][[1, 2]] = points[name][[2, 1]]
    raw = decode(points)
    assert raw.start_ns == EPOCH and raw.end_ns == EPOCH+100000000 and raw.sensor_id == 0
    assert raw.points.tobytes() == points.tobytes()


@pytest.mark.parametrize('changes', [dict(bigendian=True), dict(point_step=72),
    dict(width=250001), dict(height=2), dict(frame_id='map'), dict(row_step=1),
    dict(fields=FIELDS[:-1]), dict(fields=(FIELDS[0],)*len(FIELDS)), dict(header_ns=0)])
def test_bad_schema_fails_closed(changes):
    with pytest.raises(ProjectionError):
        decode(raw_points(), **changes)


@pytest.mark.parametrize('field,value', [('x', np.nan), ('timestamp', np.inf),
    ('sensor_id', 2), ('offset_time', 300000000), ('source_timestamp', 0.),
    ('raw_timestamp', np.nan), ('origin_x', .1), ('source_index', 0)])
def test_malformed_point_fails_closed_instead_of_creating_a_clearing_ray(field, value):
    points = raw_points()
    points[field][4] = value
    with pytest.raises(ProjectionError):
        decode(points)


@pytest.mark.parametrize('sensor,origin', [(0, (0., 0., 0.)), (1, (-.73, .02, 0.))])
def test_translating_rotating_scan_uses_each_acquisition_pose_and_its_real_origin(sensor, origin):
    body_to_tracking = planar(.4043, yaw=-1.56646)
    ray_to_tracking = Pose3((.1, -.05, .02), (0., 0., .1, math.sqrt(.99)))
    fixed_map_alignment = planar(3., -2., .3)
    value = core(body=body_to_tracking, ray=ray_to_tracking)
    feed(value, correction=lambda t: fixed_map_alignment)
    points = raw_points(sensor=sensor, origin=origin)
    wall = np.array([[10., 3., 1.]])
    expected_origins = []
    for i, ns in enumerate(points['offset_time']):
        tracking_pose = planar(float(ns)*1e-9, yaw=float(ns)*1e-9)
        transform = compose(fixed_map_alignment, compose(tracking_pose, ray_to_tracking))
        source_point = transform_many(inverse(transform), wall)[0]
        for name, component in zip(('x', 'y', 'z'), source_point):
            points[name][i] = component
        expected_origins.append(transform_many(transform, np.asarray([origin]))[0])
    result = project(value, decode(points))
    actual = np.column_stack([result.points[n] for n in ('x', 'y', 'z')])
    actual_origins = np.column_stack([result.points[n] for n in ('origin_x', 'origin_y', 'origin_z')])
    np.testing.assert_allclose(actual, np.broadcast_to(wall, actual.shape), atol=1e-6)
    np.testing.assert_allclose(actual_origins, expected_origins, atol=2e-7)
    assert np.linalg.norm(actual_origins[-1]-actual_origins[0]) > .02
    for name in ('intensity', 'sensor_id', 'ring', 'offset_time', 'source_index',
                 'timestamp', 'source_timestamp', 'raw_timestamp'):
        np.testing.assert_array_equal(result.points[name], points[name])
    assert result.alignment_ns == EPOCH+100000000


def test_scan_uses_one_alignment_not_different_global_corrections_per_point():
    value = core()
    feed(value, local=lambda t: planar(t), correction=lambda t: planar(t/2))
    points = raw_points()
    points['x'] = 10. - points['offset_time'].astype(float)*1e-9
    result = project(value, decode(points))
    np.testing.assert_allclose(result.points['x'], 10.05, atol=1e-6)
    np.testing.assert_allclose(result.points['origin_x'],
        points['offset_time'].astype(float)*1e-9+.05, atol=1e-7)


def test_global_first_and_local_first_pair_identically_without_body_tracking_confusion():
    a, b = core(body=planar(.4, .1, .8)), core(body=planar(.4, .1, .8))
    feed(a)
    feed(b, reverse_arrival=True)
    np.testing.assert_array_equal(project(a, decode(raw_points())).points,
                                  project(b, decode(raw_points())).points)


def test_missing_pose_end_waits_then_recovers_without_extrapolation():
    value = core()
    feed(value, count=4)
    raw = decode(raw_points())
    with pytest.raises(AwaitingCoverage, match='coverage'):
        project(value, raw)
    for ns in (80000000, 100000000, 120000000):
        tracking = planar(ns*1e-9, yaw=ns*1e-9)
        value.add_local_body(EPOCH+ns, tracking, CTX)
        value.add_global_tracking(EPOCH+ns, tracking, CTX)
    assert project(value, raw).end_ns == EPOCH+100000000


def test_large_pose_gap_is_not_interpolated_or_covered_by_nearest_pose():
    value = core()
    feed(value, count=3, step=80000000)
    with pytest.raises(ProjectionError, match='bracketing_gap'):
        project(value, decode(raw_points()))


def test_no_same_stamp_map_pair_waits_not_use_latest_tf_or_independent_global_interpolation():
    value = core()
    for i in range(9):
        value.add_local_body(EPOCH+i*20000000, planar(i*.02), CTX)
        value.add_global_tracking(EPOCH+i*20000000+1000, planar(i*.02), CTX)
    with pytest.raises(AwaitingCoverage, match='same_stamp'):
        project(value, decode(raw_points()))


def test_unconfirmed_future_output_is_not_used_as_map_alignment():
    value = core()
    feed(value)
    with pytest.raises(AwaitingCoverage, match='same_stamp'):
        project(value, decode(raw_points()), authorized_pose_ns=EPOCH+90000000)


def test_context_reset_clears_history_pending_pose_pairs_and_requires_extrinsics_again():
    value = core()
    feed(value)
    value.reset(replace(CTX, epoch=2, sequence=2, barrier_ns=EPOCH+90000000))
    assert not value.local and not value.alignments and not value.global_pending
    assert value.body_to_tracking is None
    with pytest.raises(ProjectionError, match='context_mismatch'):
        value.project(decode(raw_points()), CTX, now_ns=EPOCH+180000000,
                      authorized_pose_ns=EPOCH+160000000)


def test_same_context_heartbeat_does_not_reset_sequence_or_authorize_duplicate_measurements():
    value = core()
    feed(value)
    assert project(value, decode(raw_points())).sequence == 1
    value.reset(CTX)
    with pytest.raises(ProjectionError, match='duplicate'):
        project(value, decode(raw_points()))
    assert project(value, decode(raw_points(sensor=1))).sequence == 2


def test_stale_and_prebarrier_clouds_never_become_fresh_from_processing_time():
    value = core()
    feed(value)
    with pytest.raises(ProjectionError, match='stale'):
        project(value, decode(raw_points()), now_ns=EPOCH+600000000)
    value.context = replace(CTX, barrier_ns=EPOCH)
    with pytest.raises(ProjectionError, match='before_context'):
        project(value, decode(raw_points()))


def test_map_correction_jump_latches_until_new_map_context():
    value = core()
    feed(value, count=5)
    value.add_local_body(EPOCH+100000000, planar(.1, yaw=.1), CTX)
    with pytest.raises(ProjectionError, match='alignment_jump'):
        value.add_global_tracking(EPOCH+100000000, planar(3., yaw=.1), CTX)
    with pytest.raises(ProjectionError, match='alignment_jump'):
        project(value, decode(raw_points()))
    value.reset(CTX)
    assert value.fault  # heartbeat cannot clear it
    value.reset(replace(CTX, sequence=2, seed_id='new'))
    assert not value.fault


def test_changed_static_extrinsic_cannot_silently_reinterpret_same_epoch_history():
    value = core()
    feed(value)
    with pytest.raises(ProjectionError, match='extrinsic_changed'):
        value.set_extrinsics(body_to_tracking=planar(.5), ray_to_tracking=IDENTITY)
    with pytest.raises(ProjectionError, match='extrinsic_changed'):
        project(value, decode(raw_points()))


def test_worker_snapshot_is_independent_and_commit_requires_original_context():
    value = core()
    feed(value)
    snapshot = value.projection_snapshot()
    result = project(snapshot, decode(raw_points()))
    assert snapshot.sequence == 1 and value.sequence == 0
    assert value.last_input == [-1, -1]
    committed = value.commit_projection(result)
    assert committed.sequence == 1 and value.sequence == 1
    with pytest.raises(ProjectionError, match='already_committed'):
        value.commit_projection(result)
    value.reset(replace(CTX, sequence=2))
    assert snapshot.context == CTX and snapshot.local
    with pytest.raises(ProjectionError, match='context_changed'):
        value.commit_projection(result)


def test_history_and_unpaired_global_storage_are_bounded_and_reordered_samples_not_renewed():
    value = core()
    feed(value, count=500)
    assert len(value.local) <= 102
    assert len(value.alignments) <= 256
    assert not value.add_local_body(EPOCH, IDENTITY, CTX)
    for i in range(100):
        value.add_global_tracking(EPOCH+20000000000+i*20000000, IDENTITY, CTX)
    assert len(value.global_pending) == 16


def test_vectorized_slerp_matches_existing_localization_history_convention():
    samples = [(EPOCH, planar(0, yaw=3.0)), (EPOCH+40000000, planar(1, yaw=-3.0))]
    times = EPOCH+np.arange(41, dtype=np.int64)*1000000
    positions, rotations = interpolate_many(samples, times, 60000000)
    for index, t in enumerate(times):
        expected = interpolate_pose(samples, int(t), 60000000)
        np.testing.assert_allclose(positions[index], expected.position, atol=1e-12)
        assert abs(np.dot(rotations[index], expected.orientation)) > 1-1e-12


def session_config():
    session = dict(id='session', frame_id='map', perception_backend='per_sensor_rays')
    localization = dict(lio_localizer={'ros__parameters': dict(map_frame='map', odom_frame='odom',
        tracking_frame='tracking')}, navigation_estimation={'ros__parameters': dict(enabled=True, body_frame='body')},
        dual_lidar_adapter={'ros__parameters': dict(target_frame='lidar', **{'perception_rays.enabled': True})})
    return session, localization


def test_session_entrypoint_is_default_off_and_requires_explicit_matching_profile():
    assert session_settings({}, {}) == {'enabled': False}
    session, localization = session_config()
    settings = session_settings(session, localization)
    assert settings['enabled'] and settings['session_id'] == 'session'
    assert settings['tracking_frame'] == 'tracking' and settings['body_frame'] == 'body'
    localization['dual_lidar_adapter']['ros__parameters']['perception_rays.enabled'] = False
    with pytest.raises(ProjectionError, match='explicit_rays'):
        session_settings(session, localization)


@pytest.mark.parametrize('key,value', [('schema', True), ('epoch', True), ('epoch', 0),
    ('seed_id', ''), ('sequence', -1), ('barrier_ns', -1), ('session_id', 'a'*129)])
def test_map_context_requires_exact_bounded_identity(key, value):
    context = dict(schema=1, session_id='session', epoch=1, seed_id='seed', sequence=1, barrier_ns=0)
    context[key] = value
    with pytest.raises(ProjectionError):
        MapContext.parse(context)


def test_pose_rejects_nonunit_nonfinite_geometry():
    with pytest.raises(ProjectionError):
        checked_pose((0, 0, 0), (0, 0, 0, 0))
    with pytest.raises(ProjectionError):
        checked_pose((0, np.nan, 0), (0, 0, 0, 1))
