"""Entity suppression through the real ray core; no robot assets, ROS or SDK."""
from array import array
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from d1max_pct_scan.ray_projection import (
    ProjectionError, PreviewExcludedScan, PROJECTED_CENSORED_FIELDS)
from d1max_pct_scan.robot_self_filter import RobotSelfFilter
from d1max_pct_scan.robot_self_pose import JointPoseHistory
from test_ray_projection import CTX, EPOCH, IDENTITY, core, decode, feed, project, raw_points
from test_perception_ray_projector_boundary import (
    context, message, node, raw_cloud, ready, stamp)


class Model:
    """Only the geometry boundary is stubbed; source admission stays real."""
    def __init__(self, *, articulated=False, identifier='fixture', on_call=None):
        self.joint_types = {'hip': 'revolute'} if articulated else {}
        self.identifier, self.on_call = identifier, on_call
        self.calls = []
        self.configs = []
        self.timed_out = self.candidate_limit_reached = False

    def to_dict(self):
        return {'model_id': self.identifier, 'joint_types': self.joint_types}

    def classify(self, endpoints, origin, joint_positions=None, *, include_static=True,
                 include_dynamic=True, config):
        points = np.asarray(endpoints)
        joints = dict(joint_positions or {})
        self.calls.append((points.copy(), np.asarray(origin).copy(), joints,
                           include_static, include_dynamic))
        self.configs.append(config)
        if self.on_call:
            self.on_call()
        static = include_static & np.isclose(points[:, 0], .2)
        moving = include_dynamic & bool(joints.get('hip', -1.) >= .5) & np.isclose(points[:, 0], .4)
        mask = static | moving
        counts = {}
        if np.any(static):
            counts['body_collision'] = int(static.sum())
        if np.any(moving):
            counts['leg_collision'] = int(moving.sum())
        return NS(matched_mask=mask, per_link_counts=counts,
                  missing_links=('leg_link',) if self.joint_types and not joints else (),
                  origin_inside_links=(), timed_out=self.timed_out,
                  candidate_limit_reached=self.candidate_limit_reached)

    def classify_articulated(self, endpoints, origin, joint_arrays, *, include_static=False, config):
        mask = np.zeros(len(endpoints), dtype=bool)
        counts, missing = {}, set()
        for index, endpoint in enumerate(endpoints):
            joints = {name: float(values[index]) for name, values in joint_arrays.items()
                      if np.isfinite(values[index])}
            result = self.classify(np.asarray([endpoint]), origin, joints,
                include_static=include_static, include_dynamic=True, config=config)
            mask[index] = result.matched_mask[0]
            missing.update(result.missing_links)
            for name, count in result.per_link_counts.items():
                counts[name] = counts.get(name, 0)+count
        return NS(matched_mask=mask, per_link_counts=counts, missing_links=tuple(missing),
                  origin_inside_links=(), timed_out=self.timed_out,
                  candidate_limit_reached=self.candidate_limit_reached)


def filtering(model=None, **changes):
    settings = dict(mode='filter', measurement_padding_m=.005,
                    max_sensor_origin_error_m=.02, max_near_points=4096,
                    classification_budget_ms=50., body_roi_half_extent_m=[.7, .5, .85],
                    sensor_origins_body_m={'0': [0., 0., 0.], '1': [0., 0., 0.]})
    settings.update(changes)
    return RobotSelfFilter(model or Model(), settings)


def test_fixed_stance_does_not_gate_actual_rear_origin_on_missing_sdk_joints():
    model = Model()
    entity = filtering(model, pose_source='fixed_standing_cad',
                       joint_source='not_available_not_required')
    endpoints = np.asarray([[.2, .02, -.03], [1., .02, -.03]])
    result = entity.classify(endpoints, np.asarray([.076, 0., 0.]),
                            np.asarray([100, 101]), sensor_id=1)
    assert result.mask.tolist() == [True, False]
    assert result.report['requires_joint_stream'] is False
    assert result.report['sensor_origin_matches_cad'] is False
    assert result.report['origin_error_m'] == pytest.approx(.076)
    assert result.report['physical_calibration_verified'] is False
    assert model.calls[0][1].tolist() == [.076, 0., 0.]


def test_fixed_projector_creates_no_joint_subscription(node, monkeypatch):
    from d1max_pct_scan.perception_ray_projector import create_node
    entity = filtering(pose_source='fixed_standing_cad',
                       joint_source='not_available_not_required')
    monkeypatch.setattr(RobotSelfFilter, 'load', lambda _: entity)
    settings = dict(node.p, entity_filter_settings=entity.settings)
    fixed = create_node(settings)
    try:
        assert fixed.joint_history is None
        assert not any('joint' in item[1].lower() for item in fixed.subscriptions)
        ready(fixed)
        fixed.publish_status(fixed.current_context(), fixed.now_ns(), 0.)
        import json
        status = json.loads(fixed.status_pub.messages[-1].data)['entity_self_filter']
        assert status['pose_source'] == 'fixed_standing_cad'
        assert status['requires_joint_stream'] is False
        assert status['joint_source'] == 'not_available_not_required'
    finally:
        fixed.destroy_node()


def configured(entity, *, poses=None, moving=False):
    value = core()
    value.entity_filter, value.joint_poses = entity, poses
    if moving:
        feed(value)
    else:
        feed(value, local=lambda _: IDENTITY)
    return value


def points_with_x(values, **kwargs):
    points = raw_points(n=len(values), **kwargs)
    points['x'] = values
    points['y'] = .02
    points['z'] = -.03
    return points


def original_wire_bytes(points):
    return points.view(np.uint8).reshape(-1, points.dtype.itemsize)[:, :64].tobytes()


def test_self_return_endpoints_are_censored_but_actual_prefix_and_metadata_survive():
    value = configured(filtering())
    original = points_with_x([.2, 2., .2])
    result = project(value, decode(original))
    assert len(result.points) == 3
    assert original_wire_bytes(result.points) == original.tobytes()
    assert result.points['ray_kind'].tolist() == [1, 0, 1]
    assert result.wire_fields == PROJECTED_CENSORED_FIELDS
    assert np.all(result.points.view(np.uint8).reshape(-1, 68)[:, 65:] == 0)
    assert result.entity_filter_report['matched'] == 2
    assert result.entity_filter_report['suppressed'] == 2
    assert result.entity_filter_report['missing_links'] == []
    assert result.entity_filter_report['per_link_counts'] == {'body_collision': 2}
    assert result.start_ns == EPOCH and result.end_ns == EPOCH+100_000_000
    assert result.alignment_ns == EPOCH+100_000_000
    assert result.sequence == 1 and value.last_input == [EPOCH, -1]
    # No replacement endpoint/extension. Native DDA excludes each censored
    # endpoint's cell; only its measured prefix may clear prior occupancy.
    assert result.points['source_index'].tolist() == [0, 4, 8]
    np.testing.assert_array_equal(result.points['x'], original['x'])


def test_moving_projection_keeps_metadata_bytes_and_original_full_scan_time_after_censoring():
    value = configured(filtering(), moving=True)
    original = points_with_x([.2, 2., 3., .2])
    result = project(value, decode(original))
    actual_bytes = result.points.view(np.uint8).reshape(-1, 68)
    source_bytes = original.view(np.uint8).reshape(-1, 64)
    np.testing.assert_array_equal(actual_bytes[:, 12:16], source_bytes[:, 12:16])
    np.testing.assert_array_equal(actual_bytes[:, 28:64], source_bytes[:, 28:64])
    assert result.start_ns == EPOCH and result.end_ns == EPOCH+100_000_000
    assert result.alignment_ns == EPOCH+100_000_000
    assert result.points['offset_time'].min() == 0
    assert result.points['offset_time'].max() == 100_000_000
    assert np.any(result.points['origin_x'] != original['origin_x'])
    assert result.points['ray_kind'].tolist() == [1, 0, 0, 1]


def test_all_self_scan_preserves_only_measured_prefixes_with_original_source_watermark():
    value = configured(filtering())
    original = points_with_x([.2, .2, .2])
    result = project(value, decode(original))
    assert result.entity_filter_report['suppressed'] == 3
    assert original_wire_bytes(result.points) == original.tobytes()
    assert result.points['ray_kind'].tolist() == [1, 1, 1]
    assert result.sequence == 1
    assert value.last_input == [EPOCH, -1]
    with pytest.raises(ProjectionError, match='duplicate'):
        project(value, decode(original))


@pytest.mark.parametrize('field,value', [('x', np.nan), ('z', np.inf),
                                        ('source_timestamp', np.nan)])
def test_invalid_original_self_sample_cannot_become_a_clearing_prefix(field, value):
    core_value = configured(filtering())
    points = points_with_x([.2, .2, .2]);points[field][1] = value
    with pytest.raises(ProjectionError):
        project(core_value, decode(points))
    assert core_value.sequence == 0 and core_value.last_input == [-1, -1]


def test_preview_exclusion_remains_whole_ray_drop_even_for_censored_self_hits():
    value = configured(filtering())
    value.preview_exclusion = NS(mode='preview_drop_rays', digest='explicit-offline',
        matches=lambda endpoints, sensor: np.ones(len(endpoints), dtype=bool))
    with pytest.raises(PreviewExcludedScan):
        project(value, decode(points_with_x([.2, .2, .2])))
    assert value.sequence == 0 and value.last_input == [-1, -1]


def test_maximum_acquisition_extension_stays_inside_native_eight_mib_budget():
    from d1max_pct_scan.perception_contract import MAX_ACQUISITION_POINTS
    value = configured(filtering())
    points = raw_points(n=MAX_ACQUISITION_POINTS);points['x'][0] = .2
    result = project(value, decode(points))
    assert len(result.points) == MAX_ACQUISITION_POINTS
    assert result.points.dtype.itemsize == 68
    assert len(result.wire_data) == MAX_ACQUISITION_POINTS*68 < 8*1024*1024
    assert result.points['ray_kind'].sum() == 1
    assert original_wire_bytes(result.points) == points.tobytes()


def test_audit_reports_matches_while_preserving_every_real_ray():
    value = configured(filtering(mode='audit'))
    original = points_with_x([.2, 2., .2])
    result = project(value, decode(original))
    assert result.points.tobytes() == original.tobytes()
    assert result.entity_filter_report['matched'] == 2
    assert result.entity_filter_report['suppressed'] == 0


def test_dynamic_hit_uses_source_stamped_joint_at_each_exact_beam_time():
    model = Model(articulated=True)
    poses = JointPoseHistory(model.joint_types, identity=CTX)
    for offset, hip in ((0, 0.), (50_000_000, .5), (100_000_000, 1.)):
        poses.add_joint_state(EPOCH+offset, ['hip'], [hip])
    value = configured(filtering(model), poses=poses.snapshot())
    original = points_with_x([.4, .4, .4])
    result = project(value, decode(original))
    assert result.points['source_index'].tolist() == [0, 4, 8]
    assert result.points['ray_kind'].tolist() == [0, 1, 1]
    assert result.entity_filter_report['suppressed'] == 2
    dynamic_calls = [call for call in model.calls if call[4]]
    assert [call[2] for call in dynamic_calls] == [{'hip': 0.}, {'hip': .5}, {'hip': 1.}]


def test_missing_articulation_keeps_dynamic_rays_and_can_still_remove_static_self():
    model = Model(articulated=True)
    value = configured(filtering(model), poses=JointPoseHistory(model.joint_types).snapshot())
    original = points_with_x([.2, .4, 2.])
    result = project(value, decode(original))
    assert result.points['source_index'].tolist() == [0, 4, 8]
    assert result.points['ray_kind'].tolist() == [1, 0, 0]
    assert result.entity_filter_report['missing_links'] == ['leg_link']
    assert result.entity_filter_report['suppressed'] == 1


def test_pose_snapshot_retains_original_source_evidence_after_live_history_reset():
    model = Model(articulated=True)
    poses = JointPoseHistory(model.joint_types, identity=CTX)
    for offset in (0, 50_000_000, 100_000_000):
        poses.add_joint_state(EPOCH+offset, ['hip'], [0.])
    value = configured(filtering(model), poses=poses.snapshot())
    worker = value.projection_snapshot()
    poses.reset(replace(CTX, sequence=2))
    for offset in (0, 50_000_000, 100_000_000):
        poses.add_joint_state(EPOCH+offset, ['hip'], [1.])
    original = points_with_x([.4, .4, .4])
    projected = project(worker, decode(original))
    assert projected.points.tobytes() == original.tobytes()
    assert value.sequence == 0
    committed = value.commit_projection(projected)
    assert committed.sequence == 1
    assert all(not call[2] or call[2]['hip'] == 0. for call in model.calls)


def test_foreign_joint_pose_identity_skips_articulation_not_lidar_or_static_filter():
    model = Model(articulated=True)
    poses = JointPoseHistory(model.joint_types, identity=replace(CTX, seed_id='other'))
    for offset in (0, 50_000_000, 100_000_000):
        poses.add_joint_state(EPOCH+offset, ['hip'], [1.])
    value = configured(filtering(model), poses=poses.snapshot())
    result = project(value, decode(points_with_x([.2, .4, 2.])))
    assert result.points['source_index'].tolist() == [0, 4, 8]
    assert result.points['ray_kind'].tolist() == [1, 0, 0]
    assert result.entity_filter_report['missing_links'] == ['leg_link']
    assert not any(call[4] for call in model.calls)


def test_old_uncovered_joint_sample_cannot_disable_static_entity_suppression():
    model = Model(articulated=True)
    poses = JointPoseHistory(model.joint_types, identity=CTX)
    poses.add_joint_state(EPOCH-1, ['hip'], [1.])
    value = configured(filtering(model), poses=poses.snapshot())
    original = points_with_x([.2]+[.3]*1999)
    result = project(value, decode(original))
    assert result.entity_filter_report['suppressed'] == 1
    assert result.entity_filter_report['reason'] == ''
    assert result.entity_filter_report['missing_links'] == ['leg_link']
    assert len(model.calls) == 1


def test_completed_filtered_snapshot_cannot_commit_after_context_identity_changes():
    value = configured(filtering())
    worker = value.projection_snapshot()
    projected = project(worker, decode(points_with_x([.2, 2., .2])))
    value.reset(replace(CTX, epoch=2, seed_id='replacement', sequence=2))
    with pytest.raises(ProjectionError, match='context_changed'):
        value.commit_projection(projected)
    assert value.sequence == 0 and value.last_input == [-1, -1]


def test_completed_snapshot_cannot_commit_after_entity_model_policy_changes():
    value = configured(filtering(Model(identifier='one')))
    projected = project(value.projection_snapshot(), decode(points_with_x([.2, 2., .2])))
    value.entity_filter = filtering(Model(identifier='two'))
    with pytest.raises(ProjectionError, match='context_changed'):
        value.commit_projection(projected)
    assert value.sequence == 0 and value.last_input == [-1, -1]


def test_rear_origin_model_mismatch_preserves_real_origin_and_points_without_latching_fault():
    entity = filtering(sensor_origins_body_m={
        '0': [0., 0., 0.], '1': [-.4043, 0., -.0377]})
    value = configured(entity)
    original = points_with_x([.2, .2, .2], sensor=1,
                            origin=(-.327986246, 0., -.033212367))
    result = project(value, decode(original))
    assert result.points.tobytes() == original.tobytes()
    assert result.entity_filter_report['reason'] == 'sensor_origin_model_mismatch_rays_preserved'
    assert result.entity_filter_report['origin_error_m'] > .076
    assert result.entity_filter_report['suppressed'] == 0
    assert not entity.model.calls
    assert value.fault is None and result.sequence == 1


def test_dense_non_entity_roi_points_do_not_exhaust_geometry_candidate_budget():
    entity = filtering(max_near_points=2)
    value = configured(entity)
    original = points_with_x([.3, .3, .3])
    result = project(value, decode(original))
    assert result.points.tobytes() == original.tobytes()
    assert result.entity_filter_report['reason'] == ''
    assert len(entity.model.calls) == 1
    assert entity.model.configs[0].max_candidate_points == 2


def test_real_collision_geometry_dense_ground_adjacent_box_and_wire_bytes(tmp_path):
    from test_robot_self_geometry import model_at
    model = model_at(tmp_path, '''<link name="BASE"><collision>
      <origin xyz=".2 .02 -.03"/><geometry><box size=".04 .04 .04"/></geometry>
      </collision></link>''')
    value = configured(filtering(model, max_near_points=2))
    original = points_with_x([.2, .235]+[.4]*1998)
    original['z'][2:] = -.5  # Dense visible ground inside the broad-phase ROI.
    result = project(value, decode(original))
    assert result.entity_filter_report['suppressed'] == 1
    assert result.entity_filter_report['raw_link_match_count'] == 1
    assert result.entity_filter_report['reason'] == ''
    assert original_wire_bytes(result.points) == original.tobytes()
    assert result.points['ray_kind'][0] == 1
    assert not result.points['ray_kind'][1:].any()


def test_real_articulated_geometry_uses_each_beam_pose_through_projection(tmp_path):
    from test_robot_self_geometry import model_at
    model = model_at(tmp_path, '''<link name="BASE"/><link name="WHEEL"><collision>
      <origin xyz=".2 0 0"/><geometry><box size=".04 .04 .04"/></geometry>
      </collision></link><joint name="hip" type="revolute"><parent link="BASE"/>
      <child link="WHEEL"/><origin xyz=".4 0 -.03"/><axis xyz="0 1 0"/></joint>''')
    poses = JointPoseHistory(model.joint_types, identity=CTX)
    for offset, angle in ((0, 0.), (50_000_000, np.pi/2), (100_000_000, np.pi)):
        poses.add_joint_state(EPOCH+offset, ['hip'], [angle])
    snapshot = poses.snapshot()
    original = points_with_x([0., 0., 0., 2.])
    for index in range(3):
        joints = snapshot.positions_at(EPOCH+int(original['offset_time'][index]))
        transforms, _ = model.forward_kinematics(joints)
        wheel = transforms['WHEEL'] @ np.array([.2, 0., 0., 1.])
        for axis, name in enumerate(('x', 'y', 'z')):
            original[name][index] = wheel[axis]
    result = project(configured(filtering(model), poses=snapshot), decode(original))
    assert original_wire_bytes(result.points) == original.tobytes()
    assert result.points['ray_kind'].tolist() == [1, 1, 1, 0]
    assert result.entity_filter_report['per_link_counts'] == {'WHEEL': 3}
    assert result.entity_filter_report['missing_links'] == []


def test_measured_dynamic_occluder_is_considered_for_fixed_entity_hit(tmp_path):
    from test_robot_self_geometry import model_at
    model = model_at(tmp_path, '''<link name="BASE"><collision><origin xyz=".2 0 0"/>
      <geometry><box size=".2 .2 .2"/></geometry></collision></link>
      <link name="LEG"><collision><origin xyz="-.4 0 0"/>
      <geometry><box size=".2 .2 .2"/></geometry></collision></link>
      <joint name="hip" type="revolute"><parent link="BASE"/><child link="LEG"/>
      <axis xyz="0 1 0"/></joint>''')
    poses = JointPoseHistory(model.joint_types, identity=CTX)
    poses.add_joint_state(EPOCH, ['hip'], [0.])
    result = filtering(model, sensor_origins_body_m={'0': [-1., 0., 0.], '1': [-1., 0., 0.]}).classify(
        np.array([[.1, 0., 0.]]), np.array([-1., 0., 0.]), np.array([EPOCH]),
        sensor_id=0, poses=poses.snapshot())
    assert not result.mask.any()
    assert result.report['missing_links'] == []


def test_overrun_discards_mask_but_cannot_preempt_one_geometry_boundary_call(monkeypatch):
    # Synthetic elapsed time proves the outer budget is checked after classify
    # returns, without adding a wall-clock sleep or a flaky duration assertion.
    import d1max_pct_scan.robot_self_filter as implementation
    elapsed = [0.]

    def simulate_long_single_call():
        elapsed[0] += 10.

    model = Model(on_call=simulate_long_single_call)
    entity = filtering(model, classification_budget_ms=5.)
    monkeypatch.setattr(implementation.time, 'perf_counter', lambda: elapsed[0])
    result = entity.classify(np.array([[.2, 0., 0.]]), np.zeros(3),
                             np.array([EPOCH]), sensor_id=0)
    assert len(model.calls) == 1  # Static boundary already ran for 10 simulated seconds.
    assert elapsed[0] == 10.
    assert not result.mask.any()
    assert result.report['reason'] == 'entity_query_budget_rays_preserved'


@pytest.mark.parametrize('flag', ['timed_out', 'candidate_limit_reached'])
def test_incomplete_geometry_report_preserves_all_rays_even_if_elapsed_outer_time_is_small(flag):
    model = Model()
    setattr(model, flag, True)
    value = configured(filtering(model))
    original = points_with_x([.2, .2, .2])
    result = project(value, decode(original))
    assert result.points.tobytes() == original.tobytes()
    assert result.entity_filter_report['reason'] == 'entity_query_budget_rays_preserved'
    assert result.entity_filter_report['suppressed'] == 0


def attach_entity_to_node(value, *, articulated=False):
    entity = filtering(Model(articulated=articulated))
    value.entity_filter, value.core.entity_filter = entity, entity
    value.joint_history = JointPoseHistory(entity.model.joint_types, identity=value.current_context())
    return entity


def test_real_callback_all_self_outcome_publishes_explicit_measured_prefixes(node):
    ready(node)
    attach_entity_to_node(node)
    cloud = raw_cloud()
    points = raw_points()
    points['x'] = .2
    cloud.data = array('B', points.tobytes())
    node.on_rays(cloud)
    node.tick()
    node.inflight[0][0].result(timeout=2.)
    node.tick()
    assert len(node.publisher.messages) == 1
    output = node.publisher.messages[-1].rays
    assert output.point_step == 68 and output.row_step == len(points)*68
    assert output.data[64::68] == array('B', [1])*len(points)
    assert [(f.name, f.offset, f.datatype, f.count) for f in output.fields] == list(PROJECTED_CENSORED_FIELDS)
    assert node.core.sequence == 1 and node.core.last_input == [EPOCH, -1]
    assert node.last_published == [EPOCH, 0]
    assert node.entity_counts['suppressed'] == [len(points), 0]


def test_real_callback_context_reset_clears_joint_history_after_exact_ack(node):
    ready(node)
    attach_entity_to_node(node, articulated=True)
    node.on_joints(NS(header=NS(stamp=stamp(EPOCH), frame_id='body'), name=['hip'], position=[.5]))
    captured = node.joint_history.snapshot()
    assert captured.positions_at(EPOCH) == {'hip': .5}
    node.on_context(message(context(sequence=2)))
    assert node.current_context() is None
    node.on_ack(message(context(sequence=2)))
    assert node.current_context().sequence == 2
    assert node.joint_history.snapshot().positions_at(EPOCH) == {}
    assert captured.positions_at(EPOCH) == {'hip': .5}


@pytest.mark.parametrize('packet', [
    NS(data='{"receipt_stamp": 1790412195, "joint_angles": [90]}'),
    NS(header=NS(stamp=NS(sec=float('nan'), nanosec=0), frame_id='body'), name=['hip'], position=[.5]),
    NS(header=NS(stamp=NS(sec=float('inf'), nanosec=0), frame_id='body'), name=['hip'], position=[.5]),
    NS(header=NS(stamp=NS(sec=EPOCH//10**9, nanosec=-1), frame_id='body'), name=['hip'], position=[.5]),
    NS(header=NS(stamp=NS(sec=EPOCH//10**9, nanosec=10**9), frame_id='body'), name=['hip'], position=[.5]),
    NS(header=NS(stamp=stamp(EPOCH), frame_id='body'), name=['hip'], position=None),
    NS(header=NS(stamp=stamp(EPOCH), frame_id='body'), name=['hip', 'hip'], position=[0., 1.])])
def test_malformed_joint_packet_or_sdk_receipt_cannot_escape_ros_callback(node, packet):
    ready(node)
    attach_entity_to_node(node, articulated=True)
    node.on_joints(packet)
    assert node.joint_history.snapshot().sample_count == 0
    assert node.core.fault is None


def test_receipt_new_but_source_joint_old_or_foreign_frame_is_not_pose_evidence(node):
    ready(node)
    attach_entity_to_node(node, articulated=True)
    for source, frame in ((EPOCH-1, 'body'), (EPOCH, 'sdk_degrees'),
                          (node._now+20_000_000, 'body')):
        node.on_joints(NS(header=NS(stamp=stamp(source), frame_id=frame), name=['hip'], position=[.5]))
    assert node.joint_history.snapshot().sample_count == 0
