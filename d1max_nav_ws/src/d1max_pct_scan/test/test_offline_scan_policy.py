"""Policy-specific offline harness checks; no ROS context, child or socket."""
import pytest
from copy import deepcopy
import numpy as np

from offline_native_scan_scenarios import (expected_acceptance, resolve_collision_policy,
    resolve_perception_backend, projected_ray_records, projected_ray_message,
    projected_status_matches, first_return_cloud, prepare, FRAME, RAY_SCAN_DURATION_NS)
from offline_live_chain_smoke import validate_support_policy


def test_live_default_and_explicit_override_are_distinct():
    normal = {'scan_collision_policy': 'observed_free'}
    assert resolve_collision_policy(normal) == 'observed_free'
    assert resolve_collision_policy(normal, 'official') == 'official'
    assert normal == {'scan_collision_policy': 'observed_free'}
    with pytest.raises(ValueError, match='Unknown SCAN collision policy'):
        resolve_collision_policy({'scan_collision_policy': 'offical'})


def test_unknown_observation_is_diagnostic_only_in_official_policy():
    assert expected_acceptance('box_occluded', 'official') is None
    assert expected_acceptance('box_occluded', 'observed_free') is False
    # Observed physical blockage is still a negative in BOTH policies.
    for policy in ('official', 'observed_free'):
        assert expected_acceptance('closed_corridor', policy) is False
        assert expected_acceptance('box_detour_multiview', policy) is True


def test_official_evidence_requires_disabled_extra_veto_without_claiming_validity():
    disabled = dict(enabled=False, valid=None, reason='not_part_of_official_scan_policy',
                    generation=3, plan_id=8)
    assert validate_support_policy('official', [disabled], 'source').startswith('official_')
    for mutation in ({'enabled': True}, {'valid': True}, {'reason': 'different_checker'}):
        changed = dict(disabled, **mutation)
        with pytest.raises(AssertionError, match='PCT support veto'):
            validate_support_policy('official', [changed], 'source')
    with pytest.raises(AssertionError, match='no native spline'):
        validate_support_policy('official', [], 'source')


def test_strict_support_provenance_and_motion_limits_remain_required():
    valid = dict(valid=True, support_index_sha256='source', motion_authorized=False,
                 foot_placement_or_swept_volume_certified=False)
    assert validate_support_policy('observed_free', [valid], 'source').startswith('native_splines_')
    for mutation in ({'support_index_sha256': 'foreign'}, {'motion_authorized': True},
                     {'foot_placement_or_swept_volume_certified': True}, {'valid': False}):
        with pytest.raises(AssertionError, match='provenance/boundary'):
            validate_support_policy('observed_free', [dict(valid, **mutation)], 'source')
    with pytest.raises(AssertionError, match='provenance/boundary'):
        validate_support_policy('observed_free',
            [dict(enabled=False, valid=None, reason='not_part_of_official_scan_policy')], 'source')


def test_backend_default_is_preserved_and_invalid_values_fail():
    assert resolve_perception_backend() == 'deskewed_cloud'
    assert resolve_perception_backend('per_sensor_rays') == 'per_sensor_rays'
    with pytest.raises(ValueError, match='Unknown offline perception backend'):
        resolve_perception_backend('merged_points')


@pytest.mark.parametrize('sensor', [0, 1])
def test_projected_wire_roundtrip_preserves_known_origins_endpoints_and_source_clocks(sensor):
    from d1max_pct_scan.ray_projection import FIELDS, decode_rays
    from rclpy.serialization import serialize_message, deserialize_message
    from d1max_planning_interfaces.msg import ProjectedRays
    origin = np.array([.2 if sensor == 0 else -.2, 0., .55])
    points = np.array([[1.5, 0., .55], [2., 1., .2], [0., -2., .55]])
    source_ns = 1_790_000_000_123_456_789
    records = projected_ray_records(points, origin, sensor_id=sensor,
        source_ns=source_ns, source_indices=[1, 3, 5])
    context = dict(schema=1, session_id='TEST_ONLY_wire', epoch=3, seed_id='known_map',
                   sequence=8, barrier_ns=source_ns-100_000_000)
    message = projected_ray_message(records, context, source_ns=source_ns, projection_sequence=20+sensor)
    # CDR serialization uses no node, router, graph or ROS context.
    message = deserialize_message(serialize_message(message), ProjectedRays)
    cloud = message.rays
    actual = decode_rays(data=cloud.data, fields=[(f.name, f.offset, f.datatype, f.count) for f in cloud.fields],
        point_step=cloud.point_step, row_step=cloud.row_step, width=cloud.width,
        height=cloud.height, bigendian=cloud.is_bigendian, header_ns=source_ns,
        frame_id=cloud.header.frame_id, expected_frame=FRAME)
    assert actual.sensor_id == sensor and actual.start_ns == source_ns
    assert actual.end_ns == source_ns+RAY_SCAN_DURATION_NS
    assert set((f.name, f.offset, f.datatype, f.count) for f in cloud.fields) == set(FIELDS)
    for axis, index in zip('xyz', range(3)):
        np.testing.assert_allclose(actual.points[axis], points[:, index])
        np.testing.assert_allclose(actual.points['origin_'+axis], origin[index])
    np.testing.assert_array_equal(actual.points['source_index'], [1, 3, 5])
    for key in ('session_id', 'epoch', 'seed_id', 'barrier_ns'):
        assert getattr(message, key) == context[key]
    assert message.context_sequence == context['sequence'] and message.projection_sequence == 20+sensor
    assert message.alignment_stamp == cloud.header.stamp
    assert message.acquisition_end.sec*10**9+message.acquisition_end.nanosec == actual.end_ns
    assert cloud.point_step == 64 and cloud.row_step == cloud.width*64


def test_bad_source_identity_or_barrier_cannot_create_a_projected_fixture_packet():
    points, origin = [[1., 0., .5], [2., 0., .5]], [0., 0., .5]
    with pytest.raises(ValueError, match='invalid_synthetic_projected_rays'):
        projected_ray_records(points, origin, sensor_id=2, source_ns=200)
    with pytest.raises(ValueError, match='invalid_synthetic_source_indices'):
        projected_ray_records(points, origin, sensor_id=0, source_ns=200, source_indices=[1, 1])
    records = projected_ray_records(points, origin, sensor_id=0, source_ns=200)
    with pytest.raises(ValueError, match='invalid_synthetic_projected_context'):
        projected_ray_message(records, {'barrier_ns': 200}, source_ns=200, projection_sequence=1)


def test_both_source_readiness_cannot_be_satisfied_by_one_source_or_foreign_context():
    context = dict(schema=1, session_id='TEST_ONLY_ready', epoch=1, seed_id='seed', sequence=1, barrier_ns=100)
    status = dict(context, valid=True, reason='integrated_both_sources', source_stamp_ns=110,
        sources=[dict(sensor_id=0, integrated_count=1, integrated_stamp_ns=120),
                 dict(sensor_id=1, integrated_count=1, integrated_stamp_ns=110)])
    assert projected_status_matches(status, context)
    for key, value in [('session_id', 'other'), ('epoch', 2), ('seed_id', 'other'),
                       ('sequence', 2), ('barrier_ns', 99), ('valid', False), ('source_stamp_ns', 120)]:
        assert not projected_status_matches(dict(status, **{key: value}), context)
    for fault in ('missing_source', 'duplicate_source', 'unintegrated', 'old_source'):
        changed = deepcopy(status)
        if fault == 'missing_source': changed['sources'].pop()
        if fault == 'duplicate_source': changed['sources'][1]['sensor_id'] = 0
        if fault == 'unintegrated': changed['sources'][1]['integrated_count'] = 0
        if fault == 'old_source': changed['sources'][1]['integrated_stamp_ns'] = 100
        assert not projected_status_matches(changed, context)


def test_per_source_interleaved_angles_keep_full_circle_and_recompute_true_first_hits():
    obstacle = np.array([[1.5, -.4, 0.], [2.2, .4, 2.2]])
    for sensor, shift in enumerate((.2, -.2)):
        origin = np.array([shift, 0., .55])
        cloud = first_return_cloud(origin, obstacle, azimuth_stride=2, azimuth_offset=sensor)
        assert 1000 < len(cloud) < 100000  # real native decoder budget, per sensor
        directions = cloud-origin
        assert directions[:, 0].min() < 0 < directions[:, 0].max()
        assert directions[:, 1].min() < 0 < directions[:, 1].max()
        assert np.any(np.isclose(cloud[:, 0], 1.5) & (np.abs(cloud[:, 1]) < .4))
        assert not np.any(((cloud > obstacle[0]+1e-5) & (cloud < obstacle[1]-1e-5)).all(axis=1))


def test_prepare_records_backend_and_nonphysical_dual_sensor_assumptions(tmp_path):
    legacy_dir, projected_dir = tmp_path/'legacy', tmp_path/'projected'
    legacy_dir.mkdir(); projected_dir.mkdir()
    legacy = prepare(legacy_dir, selected='straight_corridor')
    dual = prepare(projected_dir, selected='straight_corridor', perception_backend='per_sensor_rays')
    assert legacy['perception_backend'] == 'deskewed_cloud'
    assert legacy['fixture_assumptions']['sensor_count'] == 1
    assert dual['fixture_assumptions']['sensor_count'] == 2
    assert dual['fixture_assumptions']['is_physical_d1_calibration'] is False
    assert dual['physical_safety_validated'] is False and dual['motion_control_enabled'] is False
    assert dual['fixture_assumptions']['projection_scope'].endswith('not a TF/projector test')
    item = dual['cases']['straight_corridor']
    assert item['resolved_native_parameters']['grid_map.use_projected_rays'] is True
    assert item['resolved_native_parameters']['grid_map.require_localization_context'] is True
    assert item['resolved_native_parameters']['grid_map.localization_session_id'] == item['session_id']
    sources = item['views'][0]['sources']
    assert [s['sensor_id'] for s in sources] == [0, 1]
    assert sources[0]['origin'][0] - sources[1]['origin'][0] == pytest.approx(.4)
    for source in sources:
        with np.load(projected_dir/source['file']) as saved:
            assert len(saved['points']) == source['point_count']
            assert len(saved['source_indices']) == source['point_count']
