from array import array
from collections import deque

import numpy as np
import pytest

from d1max_pct_scan.live_scan_contract import (ReferenceGate, admissible_tagged_spline,
                                              checked_pose_status_envelope, validate_continuous_pose,
                                              cloud_source_issue,
                                              decode_xyz, fresh, localization_context,
                                              pack_xyz, sample_shadow_spline,
                                              take_latest_exact, transform_xyz)


def gate():
    result = ReferenceGate()
    assert not result.observe(True, 100., (1, 'seed'))
    return result


def admit(subject, stamp=100.1, now=100.1, **kwargs):
    return subject.accept([[0, 0, 0], [.5, 0, 0], [1, 0, 0]],
                          stamp=stamp, now=now, frame_id=kwargs.get('frame_id', 'd1max_loc_map'),
                          body_xyz=kwargs.get('body_xyz', [0, 0, .55]))


def test_recovery_needs_new_stamped_target_not_previous_route():
    g = gate()
    assert admit(g) and g.generation == 1
    assert g.observe(False, 100.2, (1, 'seed'))
    assert not g.active and g.generation == 2
    assert not g.observe(False, 100.3, (1, 'seed'))
    assert not g.observe(True, 100.4, (1, 'seed'))
    assert not g.active
    with pytest.raises(ValueError, match='obsolete'):
        admit(g, stamp=100.1, now=100.5)
    assert admit(g, stamp=100.6, now=100.6)
    assert g.generation == 3


def test_seed_change_cancels_even_when_status_valid():
    g = gate()
    admit(g)
    assert g.observe(True, 100.2, (1, 'different_seed'))
    assert not g.ready and not g.active


def test_missing_status_revokes_but_does_not_claim_epoch_or_seed_changed():
    g = gate()
    admit(g)
    previous = g.context
    assert g.observe(False, 100.2, None)
    assert g.reason == 'input_stale_or_invalid'
    assert g.context == previous
    assert not g.ready and not g.active and g.generation == 2
    assert not g.observe(True, 100.3, previous)
    assert g.reason == 'ready_requires_new_target'
    assert not g.active and g.generation == 2
    with pytest.raises(ValueError, match='obsolete'):
        admit(g, stamp=100.1, now=100.4)
    assert g.observe(True, 100.5, (2, 'new_seed'))
    assert g.reason == 'localization_context_changed'


def cloud_issue(**changes):
    args = dict(frame_id='d1max_loc_tracking', tracking_frame='d1max_loc_tracking',
                stamp=100., now=100.1, timeout=.5, context=('session', 1, 'seed'),
                sensor_barrier=99., last_input_stamp=99., data_size=1200,
                point_count=100, max_input_points=250000)
    args.update(changes)
    return cloud_source_issue(**args)


@pytest.mark.parametrize('changes,reason', [
    (dict(frame_id='wrong'), 'wrong_frame'),
    (dict(stamp=99.), 'invalid_stale_or_future_stamp'),
    (dict(stamp=100.3), 'invalid_stale_or_future_stamp'),
    (dict(context=None), 'localization_context_unavailable'),
    (dict(sensor_barrier=100.), 'before_sensor_barrier'),
    (dict(last_input_stamp=100.), 'nonmonotonic_input_stamp'),
    (dict(data_size=32*1024*1024+1), 'source_size_limit'),
    (dict(point_count=250001), 'source_size_limit'),
])
def test_cloud_source_rejection_details_preserve_original_admission(changes, reason):
    assert cloud_issue(**changes) == reason
    assert cloud_issue() is None


def test_explicit_cancel_keeps_sensor_ready_and_barrier():
    g = gate()
    admit(g)
    original = g.barrier
    g.revoke(100.2, 'explicit_reference_cancel', advance_barrier=False)
    assert g.ready and not g.active and g.barrier == original
    assert admit(g, stamp=100.3, now=100.3)
    assert g.generation == 3


def test_same_geometry_heartbeat_does_not_retrigger_optimizer():
    g = gate()
    admit(g)
    assert not admit(g, stamp=100.2, now=100.2)
    assert g.generation == 1


@pytest.mark.parametrize('kwargs', [dict(frame_id='conditioned_map'), dict(body_xyz=[5, 0, .55])])
def test_no_frame_relabel_or_far_start(kwargs):
    with pytest.raises(ValueError):
        admit(gate(), **kwargs)


@pytest.mark.parametrize('points', [[], [[0, 0, 0]], [[0, 0, 0], [2, 0, 0]],
                                    [[0, 0, 0], [float('nan'), 0, 0]]])
def test_bad_references(points):
    with pytest.raises(ValueError):
        gate().accept(points, frame_id='d1max_loc_map', stamp=100.1, now=100.1, body_xyz=[0, 0, .55])


def test_freshness_rejects_future_old_nonfinite_or_zero():
    for stamp in [0., 99., 100.2, float('nan'), None]:
        assert not fresh(stamp, 100., .5)
    assert fresh(99.8, 100., .5)


def test_exact_rigid_transform_rotates_then_translates():
    angle = np.pi/2
    result = transform_xyz([[1, 0, 0]], [2, 3, 4], [0, 0, np.sin(angle/2), np.cos(angle/2)])
    np.testing.assert_allclose(result, [[2, 4, 4]], atol=1e-6)
    with pytest.raises(ValueError):
        transform_xyz([[1, 0, 0]], [0, 0, 0], [0, 0, 0, 0])


def test_decode_organized_padded_big_endian_cloud():
    data = bytearray(64)
    view = np.ndarray((2, 2, 3), dtype='>f4', buffer=data, strides=(32, 12, 4))
    view[:] = [[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]]
    points = decode_xyz(data=data, fields=[(n, i*4, 7, 1) for i, n in enumerate('xyz')],
                        point_step=12, row_step=32, width=2, height=2, bigendian=True)
    np.testing.assert_array_equal(points, np.arange(1, 13).reshape(4, 3))


def test_cloud_budget_and_finite_filter():
    xyz = np.ones((20, 3), dtype='<f4')
    xyz[0, 1] = np.nan
    arguments = dict(data=xyz.tobytes(), fields=[(n, i*4, 7, 1) for i, n in enumerate('xyz')],
                     point_step=12, row_step=240, width=20, height=1, bigendian=False)
    output = decode_xyz(**arguments, max_output_points=4)
    assert len(output) <= 4 and np.isfinite(output).all()
    with pytest.raises(ValueError, match='oversize'):
        decode_xyz(**arguments, max_input_points=10)


def statuses():
    navigation = dict(schema=1, epoch=4, seed_id='seed', received_at_unix=100., valid=True, fault='')
    localizer = dict(session_id='localization-A', wall_time=100., localized=True,
                     local_epoch=4, active_seed_ns='seed', confirmed_seed_ns='seed',
                     verified_confirmations=3, local_fault='', navigation=dict(navigation),
                     frames=dict(map='d1max_loc_map', tracking='d1max_loc_tracking'))
    return localizer, navigation


def pose_lease(**changes):
    value = dict(schema=1, epoch=4, seed_id='seed', received_at_unix=100.09,
                 output_stamp_sec=100.08, pose_timeout_sec=.08, valid=True, pose_valid=True,
                 fault='', reset_pending=False, motion_control_enabled=False,
                 frame_id='d1max_loc_map', body_frame='d1max_loc_base_link')
    value.update(changes)
    return value


def test_explicit_localization_session_and_two_statuses_match():
    loc, nav = statuses()
    assert localization_context(loc, nav, pose_status=pose_lease(), session_id='localization-A', now=100.1) == (
        'localization-A', 4, 'seed')


@pytest.mark.parametrize('source,key,value', [
    ('localizer', 'session_id', 'old-session'), ('localizer', 'local_epoch', 3),
    ('localizer', 'active_seed_ns', 'another'), ('localizer', 'confirmed_seed_ns', 'old'),
    ('localizer', 'verified_confirmations', 2),
    ('localizer', 'wall_time', 98.), ('localizer', 'local_fault', 'clock_jump'),
    ('navigation', 'epoch', 5), ('navigation', 'seed_id', 'other'),
    ('navigation', 'received_at_unix', 98.), ('navigation', 'fault', 'reset'),
    ('embedded', 'epoch', 5), ('embedded', 'seed_id', 'other'),
    ('embedded', 'received_at_unix', 98.), ('embedded', 'fault', 'imu_gap'),
    ('navigation', 'reset_pending', True), ('embedded', 'reset_pending', True),
])
def test_different_session_epoch_seed_status_cannot_enable_shadow(source, key, value):
    loc, nav = statuses()
    selected = {'localizer': loc, 'navigation': nav, 'embedded': loc['navigation']}[source]
    selected[key] = value
    with pytest.raises(ValueError):
        localization_context(loc, nav, pose_status=pose_lease(), session_id='localization-A', now=100.1)


def test_matcher_and_slow_ui_status_do_not_revoke_fresh_confirmed_pose():
    loc, nav = statuses()
    loc['localized'] = False
    nav['valid'] = loc['navigation']['valid'] = False
    assert localization_context(loc, nav, pose_status=pose_lease(), session_id='localization-A',
                                now=100.1) == ('localization-A', 4, 'seed')


@pytest.mark.parametrize('changes', [dict(pose_valid=False), dict(output_stamp_sec=99.99),
    dict(received_at_unix=99.99), dict(output_stamp_sec=100.2), dict(reset_pending=True),
    dict(epoch=5), dict(seed_id='old'), dict(fault='invalid'), dict(pose_timeout_sec=1.),
    dict(body_frame='sensor'), dict(frame_id='conditioned'), dict(motion_control_enabled=True)])
def test_real_pose_lease_must_be_current_same_epoch_without_reset(changes):
    loc, nav = statuses()
    with pytest.raises(ValueError, match='continuous_pose'):
        localization_context(loc, nav, pose_status=pose_lease(**changes),
                             session_id='localization-A', now=100.1)


def test_pose_lease_is_required_no_legacy_status_fallback():
    loc, nav = statuses()
    with pytest.raises(ValueError, match='continuous_pose_status_missing'):
        localization_context(loc, nav, session_id='localization-A', now=100.1)


@pytest.mark.parametrize('age', [-.001, .081, .4, float('nan')])
def test_pose_receipt_lease_expires_even_when_ros_clock_is_paused(age):
    loc, nav = statuses()
    with pytest.raises(ValueError, match='continuous_pose'):
        localization_context(loc, nav, pose_status=pose_lease(), pose_receipt_age=age,
            session_id='localization-A', now=100.1)
    assert localization_context(loc, nav, pose_status=pose_lease(), pose_receipt_age=.079,
        session_id='localization-A', now=100.1)


@pytest.mark.parametrize('changes', [dict(received_at_unix=1000.), dict(schema=True),
    dict(pose_timeout_sec=.4), dict(output_stamp_sec=100.105), dict(reset_pending=None),
    dict(pose_valid='true'), dict(frame_id=None)])
def test_invalid_pose_envelope_cannot_become_packet_ordering_authority(changes):
    with pytest.raises(ValueError, match='continuous_pose'):
        checked_pose_status_envelope(pose_lease(**changes), now=100.1)


def test_real_fault_and_startup_envelopes_are_received_to_revoke_old_authority():
    assert checked_pose_status_envelope(pose_lease(valid=False, pose_valid=False,
        reset_pending=True, output_stamp_sec=None, epoch=0, seed_id=None), now=100.1) == 100.09
    with pytest.raises(ValueError, match='continuous_pose'):
        validate_continuous_pose(pose_lease(valid=False, pose_valid=False, fault='reset'),
            epoch=4, seed='seed', now=100.1, receipt_age=.01)


def test_five_hz_ui_does_not_create_periodic_holes_in_fifty_hz_pose():
    loc, nav = statuses()
    loc['localized'] = False
    loc['navigation']['valid'] = nav['valid'] = False
    for index in range(100):
        now = 100. + index*.02
        ui_stamp = 100. + (index//10)*.2
        loc['wall_time'] = nav['received_at_unix'] = ui_stamp
        loc['navigation']['received_at_unix'] = ui_stamp
        current = pose_lease(received_at_unix=now, output_stamp_sec=now-.025)
        assert localization_context(loc, nav, pose_status=current,
            session_id='localization-A', now=now) == ('localization-A', 4, 'seed')
    with pytest.raises(ValueError, match='continuous_pose'):
        localization_context(loc, nav, pose_status=current,
            session_id='localization-A', now=now+.081)


def tagged(g, **changes):
    arguments = dict(session_id='shadow', generation=g.generation, frame_id=g.frame_id,
                     trajectory_id=1, start_time=100.2, expected_session='shadow',
                     gate=g, last_id=0, now=100.3)
    arguments.update(changes)
    return admissible_tagged_spline(**arguments)


def test_old_spline_cannot_paint_new_goal_or_cancelled_goal():
    g = gate()
    admit(g)
    assert tagged(g)
    previous = g.generation
    g.revoke(100.3, 'cancel', advance_barrier=False)
    assert not tagged(g, generation=previous)
    assert admit(g, stamp=100.4, now=100.4)
    assert not tagged(g, generation=previous, start_time=100.5, now=100.6)
    assert tagged(g, start_time=100.5, now=100.6)


@pytest.mark.parametrize('changes', [dict(session_id='other'), dict(frame_id='other'),
                                     dict(trajectory_id=0), dict(start_time=99.9),
                                     dict(now=103.), dict(generation=99)])
def test_spline_metadata_rejection(changes):
    g = gate()
    admit(g)
    assert not tagged(g, **changes)


def spline():
    return dict(order=3, knots=np.arange(-3, 7, dtype=float),
                points=[[i*.5, 0, .55] for i in range(6)])


def test_shadow_spline_is_actual_deboor_curve_not_control_polygon():
    points = sample_shadow_spline(**spline())
    np.testing.assert_allclose(points[0], [.5, 0, .55])
    np.testing.assert_allclose(points[-1], [2., 0, .55])
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).max() <= .05


@pytest.mark.parametrize('change', ['order', 'knots', 'points', 'budget'])
def test_bad_native_spline_not_rendered(change):
    args = spline()
    if change == 'order': args['order'] = 2
    if change == 'knots': args['knots'][3] = args['knots'][2]
    if change == 'points': args['points'][3][0] = float('nan')
    if change == 'budget': args['points'][3][0] = 1e7
    with pytest.raises(ValueError):
        sample_shadow_spline(**args)


@pytest.mark.parametrize('layout', ['xyz', 'padded_point', 'padded_row', 'reordered', 'mixed64'])
@pytest.mark.parametrize('bigendian', [False, True])
@pytest.mark.parametrize('invalid', [False, True])
def test_decode_optimized_matches_finite_first_stride_for_all_layouts(layout, bigendian, invalid):
    width, height, point_step = 17, 3, 40
    row_step = point_step*width + (24 if layout == 'padded_row' else 0)
    endian = '>' if bigendian else '<'
    offsets = [16, 0, 8] if layout == 'reordered' else [0, 4, 8]
    kinds = [7, 7, 7]
    if layout == 'mixed64':
        offsets, kinds = [0, 8, 16], [8, 7, 8]
    if layout == 'xyz':
        point_step = 12
        row_step = width*12
    data = bytearray(row_step*height)
    expected = np.arange(width*height*3, dtype='<f4').reshape(-1, 3)/7.
    if invalid:
        expected[::5, 1] = np.nan
        expected[3::7, 2] = np.inf
    fields = []
    for axis, (offset, kind) in enumerate(zip(offsets, kinds)):
        fields.append(('xyz'[axis], offset, kind, 1))
        values = np.ndarray((height, width), dtype=endian+('f4' if kind == 7 else 'f8'),
                            buffer=data, offset=offset, strides=(row_step, point_step))
        values[:] = expected[:, axis].reshape(height, width)
    output = decode_xyz(data=data, fields=fields, point_step=point_step, row_step=row_step,
                        width=width, height=height, bigendian=bigendian, max_output_points=11)
    valid = expected[np.isfinite(expected).all(axis=1)]
    expected_selected = valid[::int(np.ceil(len(valid)/11))]
    np.testing.assert_array_equal(output, expected_selected)
    assert output.dtype == np.dtype('<f4')


def test_common_decode_borrows_cloud_storage_without_changing_input():
    raw = np.arange(200, dtype='<f4').reshape(25, 8)
    before = raw.copy()
    decoded = decode_xyz(data=memoryview(raw).cast('B'), fields=[(n, i*4, 7, 1) for i, n in enumerate('xyz')],
                         point_step=32, row_step=800, width=25, height=1, bigendian=False,
                         max_output_points=9)
    assert np.shares_memory(raw, decoded)
    transform_xyz(decoded, [1, 2, 3], [0, 0, 0, 1])
    np.testing.assert_array_equal(raw, before)


@pytest.mark.parametrize('count', [0, 1, 33444])
def test_pack_xyz_one_owned_payload_byte_equivalent(count):
    points = np.arange(count*3, dtype='<f4').reshape(count, 3)
    output = pack_xyz(points)
    assert isinstance(output, array) and output.typecode == 'B'
    assert output.tobytes() == points.tobytes()
    if count:
        points[:] = 999
        assert output.tobytes() != points.tobytes()


def test_pack_noncontiguous_float64_converts_same_wire_values():
    points = np.arange(300, dtype='>f8').reshape(100, 3)[::3]
    assert pack_xyz(points).tobytes() == np.asarray(points, dtype='<f4').tobytes()


def test_transform_optimization_keeps_original_float64_accumulation():
    from d1max_pct_scan.live_scan_contract import quaternion_matrix
    rng = np.random.default_rng(423)
    points = rng.uniform(-100., 100., size=(10000, 3)).astype('<f4')[::3]
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    translation = np.array([123.4, -0.034, 7.31])
    baseline = np.asarray(points @ quaternion_matrix(q).T+translation, dtype='<f4')
    np.testing.assert_array_equal(transform_xyz(points, translation, q), baseline)


def test_latest_exact_tf_does_not_block_behind_old_missing_transform():
    pending = deque([1, 2, 3], maxlen=3)
    looked_up = []
    def resolve(item):
        looked_up.append(item)
        return {'stamp': item} if item != 1 else None
    chosen, tf, dropped, waits = take_latest_exact(pending, rejection=lambda _: None, resolve=resolve)
    assert chosen == 3 and tf == {'stamp': 3}
    assert list(pending) == [] and dropped == {'superseded': 2}
    assert waits == 0 and looked_up == [3]


def test_newest_missing_tf_retained_without_relabeling_available_older_scan():
    pending = deque([1, 2, 3], maxlen=3)
    chosen, tf, dropped, waits = take_latest_exact(pending, rejection=lambda _: None,
                                                  resolve=lambda item: ('exact', item) if item == 2 else None)
    assert chosen == 2 and tf == ('exact', 2)
    assert list(pending) == [3] and dropped == {'superseded': 1} and waits == 1
    chosen, tf, _, _ = take_latest_exact(pending, rejection=lambda _: None,
                                        resolve=lambda item: ('exact', item))
    assert chosen == 3 and tf == ('exact', 3) and not pending


@pytest.mark.parametrize('reason', ['deadline_or_age', 'context_or_barrier', 'nonmonotonic_output'])
def test_queue_never_looks_up_or_recovers_invalid_scan(reason):
    pending = deque([1, 2, 3], maxlen=3)
    def forbidden_lookup(_):
        raise AssertionError('invalid scans must never request TF')
    chosen, tf, dropped, waits = take_latest_exact(pending, rejection=lambda _: reason,
                                                  resolve=forbidden_lookup)
    assert chosen is tf is None and not pending and dropped == {reason: 3} and waits == 0


def test_unavailable_exact_tf_queue_remains_bounded_and_original_ordered():
    pending = deque([1, 2, 3], maxlen=3)
    for item in range(4, 25):
        pending.append(item)
        selected, tf, dropped, waits = take_latest_exact(pending, rejection=lambda _: None,
                                                        resolve=lambda _: None)
        assert selected is tf is None and dropped == {} and waits == 3
        assert list(pending) == [item-2, item-1, item]
