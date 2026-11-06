"""Production owner + production projection, bounded independent sources.

All transport is the existing in-memory ROS facade; ROS init is forbidden.
"""
from array import array
import itertools
import threading

import numpy as np
import pytest

from d1max_pct_scan import perception_ray_projector as implementation
from d1max_pct_scan.ray_projection import AwaitingCoverage, FIELDS, decode_rays
from test_perception_ray_projector_boundary import (
    node, ready, raw_cloud, raw_points, context, message, stamp, EPOCH)
from test_projector_completion_delivery import controlled


def test_slow_front_does_not_block_two_rear_commits_or_owner_sequence(node, monkeypatch):
    from d1max_pct_scan import ray_projection
    entered, release = threading.Event(), threading.Event()
    original = ray_projection.validate_ray_samples
    def validate(raw):
        if raw.sensor_id == 0:
            entered.set()
            assert release.wait(2.)
        return original(raw)
    monkeypatch.setattr(ray_projection, 'validate_ray_samples', validate)
    ready(node)
    node.on_rays(raw_cloud(0))
    node.on_rays(raw_cloud(1))
    node.tick()
    try:
        assert entered.wait(1.)
        front = node.inflight[0]
        for source in (EPOCH, EPOCH+1_000_000):
            if source != EPOCH:
                node.on_rays(raw_cloud(1, source))
                node.completion_guard.dispatch()
            node.inflight[1][0].result(timeout=2.)
            node.completion_guard.dispatch()
            assert node.inflight[0] is front and not front[0].done()
        assert [row.projection_sequence for row in node.publisher.messages] == [1, 2]
        assert node.last_published == [0, EPOCH+1_000_000]
    finally:
        release.set()
    node.inflight[0][0].result(timeout=2.)
    node.completion_guard.dispatch()
    assert [row.projection_sequence for row in node.publisher.messages] == [1, 2, 3]
    assert [row.rays.header.stamp for row in node.publisher.messages] == [
        stamp(EPOCH), stamp(EPOCH+1_000_000), stamp(EPOCH)]
    assert node.inflight == [None, None] and node.core.last_input == [EPOCH, EPOCH+1_000_000]


def test_context_churn_keeps_old_jobs_counted_and_never_builds_executor_backlog(node):
    ready(node)
    worker = controlled(node)
    for sensor in (0, 1):
        node.on_rays(raw_cloud(sensor))
    node.tick()
    old_jobs = list(node.inflight)
    assert len(worker.jobs) == 2
    for sequence in range(2, 12):
        node.on_context(message(context(sequence=sequence)))
        node.on_ack(message(context(sequence=sequence)))
        for sensor in (0, 1):
            node.on_rays(raw_cloud(sensor, EPOCH+sequence*1_000_000))
        node.tick()
        assert len(worker.jobs) == 2 and node.inflight == old_jobs
    for sensor in (1, 0):
        worker.complete(sensor=sensor)
        node.completion_guard.dispatch()
    assert not node.publisher.messages and node.last_published == [0, 0]
    assert node.core.sequence == 0 and len(worker.jobs) == 4
    assert sum(not job[0].done() for job in worker.jobs) == 2
    assert all(work[1][1].start_ns == EPOCH+11_000_000 for work in node.inflight)
    assert all(work[1][2].sequence == 11 for work in node.inflight)


def test_two_missing_coverage_jobs_do_not_self_wake_or_renew_receipts(node):
    ready(node)
    worker = controlled(node)
    for sensor in (0, 1):
        node.on_rays(raw_cloud(sensor))
    receipts = [item[3] for item in node.pending]
    node.tick()
    wakes = node.completion_guard.triggers
    for sensor in (0, 1):
        worker.complete(AwaitingCoverage('waiting_local_pose_coverage'), sensor=sensor)
    assert node.completion_guard.triggers == wakes and len(worker.jobs) == 2
    # The existing bounded fallback timer can retry; completions themselves
    # create no self-sustaining guard/worker loop against unchanged history.
    node.tick()
    assert len(worker.jobs) == 4
    assert [work[1][3] for work in node.inflight] == receipts
    for sensor in (0, 1):
        worker.complete(AwaitingCoverage('waiting_local_pose_coverage'), sensor=sensor)
    assert node.completion_guard.triggers == wakes
    assert not node.publisher.messages and node.last_published == [0, 0]


def test_shutdown_fences_both_late_completions(node):
    ready(node)
    worker = controlled(node)
    for sensor in (0, 1):
        node.on_rays(raw_cloud(sensor))
    node.tick()
    wakes = node.completion_guard.triggers
    node.destroy_node()
    for sensor in (1, 0):
        worker.complete(sensor=sensor)
    node.tick()
    assert node.completion_guard.destroyed
    assert node.completion_guard.triggers == wakes and node.completion_wake_failures == 0
    assert not node.publisher.messages


def test_each_completed_source_rechecks_original_receipt_after_other_publication(node, monkeypatch):
    ready(node)
    worker = controlled(node)
    clock = [implementation.time.monotonic()]
    monkeypatch.setattr(implementation.time, 'monotonic', lambda: clock[0])
    for sensor in (0, 1):
        node.on_rays(raw_cloud(sensor))
    node.tick()
    for sensor in (0, 1):
        worker.complete(sensor=sensor)
    publish = node.publisher.publish
    def slow_publish(message):
        publish(message)
        clock[0] += .26
    monkeypatch.setattr(node.publisher, 'publish', slow_publish)
    node.tick()
    assert len(node.publisher.messages) == 1
    assert node.last_published == [EPOCH, 0]
    assert node.last_drop['sensor'] == 1
    assert node.last_drop['reason'] == 'ray_projection_wait_expired_or_context_changed'


@pytest.mark.parametrize('rear_fails', [False, True])
def test_fixed_size_per_source_cost_timing_never_changes_acquisition_or_failure(node, monkeypatch, rear_fails):
    ready(node)
    worker = controlled(node)
    wall, cpu = itertools.count(0, 1_000_000), itertools.count(0, 100_000)
    monkeypatch.setattr(implementation.time, 'perf_counter_ns', lambda: next(wall))
    monkeypatch.setattr(implementation.time, 'thread_time_ns', lambda: next(cpu))
    original = implementation.project_validated_rays
    def project(snapshot, raw, *args, **kwargs):
        if rear_fails and raw.sensor_id == 1:
            raise ValueError('explicit_geometry_failure')
        return original(snapshot, raw, *args, **kwargs)
    monkeypatch.setattr(implementation, 'project_validated_rays', project)
    for sensor in (0, 1):
        node.on_rays(raw_cloud(sensor))
    node.tick()
    for sensor in (0, 1):
        worker.complete(sensor=sensor)
    node.tick()
    front, rear = node.projection_timing
    assert {key: value for key, value in front.items() if key != 'stages'} == dict(source_ns=EPOCH, worker_wall_ms=1., worker_cpu_ms=pytest.approx(.1), entity={},
                         owner_harvest_ms=3., publish_ms=1., published=True)
    assert {key: value for key, value in rear.items() if key != 'stages'} == dict(source_ns=EPOCH, worker_wall_ms=1., worker_cpu_ms=pytest.approx(.1), entity={},
                        owner_harvest_ms=4., publish_ms=0. if rear_fails else 1.,
                        published=not rear_fails)
    for stage in (front, rear):
        absolute = stage['stages']
        assert absolute['received_ns'] == node._now
        times = [absolute[key] for key in (
            'received_steady_ns', 'decoded_steady_ns', 'submitted_steady_ns',
            'worker_started_steady_ns', 'worker_ended_steady_ns', 'harvested_steady_ns')]
        assert times == sorted(times)
        if stage['published']:
            assert absolute['publish_started_ns'] == absolute['publish_ended_ns'] == node._now
            assert absolute['publish_started_steady_ns'] >= times[-1]
            assert absolute['publish_ended_steady_ns']-absolute['publish_started_steady_ns'] == 1_000_000
        else:
            assert all(absolute[key] == 0 for key in (
                'publish_started_ns', 'publish_ended_ns',
                'publish_started_steady_ns', 'publish_ended_steady_ns'))
    assert node.last_published == [EPOCH, 0 if rear_fails else EPOCH]
    assert all(output.rays.header.stamp == stamp(EPOCH) for output in node.publisher.messages)
    if rear_fails:
        assert node.last_drop['reason'] == 'explicit_geometry_failure'
    node.publish_status(node.current_context(), node._now, 10.)
    assert 'projection_timing=' in node.pipeline_logs[-1]
    assert len(node.projection_timing) == 2 and all(len(item) == 8 for item in node.projection_timing)
    assert all(len(item['stages']) == 11 for item in node.projection_timing)


def test_real_parallel_workers_preserve_fixed_cad_hits_concavity_and_occlusion(node, tmp_path, monkeypatch):
    from d1max_pct_scan.robot_fixed_stance import FixedStanceModel
    from d1max_pct_scan.robot_self_filter import RobotSelfFilter
    from d1max_pct_scan.robot_self_geometry import _MESH_CACHE
    from test_robot_fixed_stance import leg_model, _box_triangles, _write_triangles
    triangles = (_box_triangles([-.09, -.09, -.09], [-.03, .09, .09])+
                 _box_triangles([-.09, -.09, -.09], [.09, -.03, .09]))
    path = tmp_path/'concave_wheel.stl'
    _write_triangles(path, triangles)
    model = FixedStanceModel.from_base_model(leg_model(tmp_path, path))
    model.warmup()
    pose = np.asarray(model.to_dict()['nominal_link_transforms_root']['RBL_FOOT_LINK'])
    collision = next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK')
    origin = pose[:3, :3] @ np.array([-.5, .0573, 0])+pose[:3, 3]
    local = np.array([[-.09, .05, 0], [.05, .05, 0], [-.03, .05, 0],
                      [.3, .05, 0], [-.094, .05, 0], [-.096, .05, 0]])
    endpoints = (local+np.asarray(collision.xyz)) @ pose[:3, :3].T+pose[:3, 3]
    entity = RobotSelfFilter(model, dict(mode='filter', pose_source='fixed_standing_cad',
        measurement_padding_m=.005, max_sensor_origin_error_m=.02, max_near_points=4096,
        classification_budget_ms=50., body_roi_half_extent_m=[1., 1., 1.],
        sensor_origins_body_m={'0': origin.tolist(), '1': origin.tolist()}))
    ready(node)
    node.entity_filter = node.core.entity_filter = entity
    expected, clouds = [], []
    for sensor in (0, 1):
        cloud = raw_cloud(sensor)
        points = raw_points(n=6, sensor=sensor, origin=origin)
        for axis, name in enumerate(('x', 'y', 'z')):
            points[name] = endpoints[:, axis]
        cloud.width, cloud.row_step = 6, 6*64
        cloud.data = array('B', points.tobytes())
        raw = decode_rays(data=cloud.data, fields=FIELDS,
            point_step=64, row_step=6*64, width=6, height=1, bigendian=False,
            header_ns=EPOCH, frame_id='lidar', expected_frame='lidar')
        result = node.core.projection_snapshot().project(raw, node.core.context,
            now_ns=node.now_ns(), authorized_pose_ns=node.authorized_pose_ns)
        assert result.entity_filter_report['suppressed'] == 2
        assert result.points['source_index'].tolist() == [0, 4, 8, 12, 16, 20]
        assert result.points['ray_kind'].tolist() == [1, 0, 0, 0, 1, 0]
        expected.append(result.points.tobytes())
        clouds.append(cloud)
    cache_before = (_MESH_CACHE.bytes, tuple(_MESH_CACHE.entries))
    monkeypatch.setattr(_MESH_CACHE, 'get', lambda *args: pytest.fail('runtime mesh loading'))
    barrier, threads = threading.Barrier(2), set()
    original = implementation.project_validated_rays
    def concurrent(*args, **kwargs):
        threads.add(threading.get_ident())
        barrier.wait(timeout=2.)
        return original(*args, **kwargs)
    monkeypatch.setattr(implementation, 'project_validated_rays', concurrent)
    for cloud in clouds:
        node.on_rays(cloud)
    node.tick()
    for work in node.inflight:
        work[0].result(timeout=2.)
    node.completion_guard.dispatch()
    assert len(threads) == 2
    assert [row.rays.data.tobytes() for row in node.publisher.messages] == expected
    assert [row.projection_sequence for row in node.publisher.messages] == [1, 2]
    assert node.entity_counts['suppressed'] == [2, 2]
    assert node.last_published == [EPOCH, EPOCH]
    assert (_MESH_CACHE.bytes, tuple(_MESH_CACHE.entries)) == cache_before
    for sensor in (0, 1):
        stage = node.projection_timing[sensor]
        assert stage['source_ns'] == EPOCH
        assert stage['entity'] == implementation.entity_stage_summary(node.entity_reports[sensor])
        assert len(stage['entity']) == 5
        assert stage['entity']['suppressed'] == 2
        assert stage['entity']['classification_ms'] >= 0.


def test_entity_stage_summary_is_fixed_size_copy_and_absent_when_not_run():
    assert implementation.entity_stage_summary(None) == {}
    report = dict(classification_ms=17.25, near_point_count=123, matched=4,
                  suppressed=4, reason='entity_query_budget_rays_preserved',
                  per_link_counts={'not_copied': 999}, missing_links=['not_copied'])
    summary = implementation.entity_stage_summary(report)
    assert len(summary) == 5 and summary['classification_ms'] == 17.25
    report['suppressed'] = 0
    assert summary['suppressed'] == 4


def test_entity_cost_on_filtered_scan_is_not_reused_for_later_geometry_failure(node):
    from d1max_pct_scan.ray_projection import EntityFilteredScan
    ready(node)
    worker = controlled(node)
    report = dict(sensor_id=1, classification_ms=4.5, near_point_count=8,
                  matched=8, suppressed=8, reason='')
    node.on_rays(raw_cloud(1))
    node.tick()
    worker.complete(EntityFilteredScan(report), sensor=1)
    node.tick()
    first = node.projection_timing[1]
    assert first['source_ns'] == EPOCH and not first['published']
    assert first['entity'] == implementation.entity_stage_summary(report)
    node.on_rays(raw_cloud(1, EPOCH+1_000_000))
    node.tick()
    worker.complete(ValueError('invalid_geometry'), sensor=1)
    node.tick()
    second = node.projection_timing[1]
    assert second['source_ns'] == EPOCH+1_000_000 and not second['published']
    assert second['entity'] == {} and node.entity_reports[1] == report
    assert node.last_published == [0, 0] and not node.publisher.messages
