"""Real projector callbacks with an in-memory ROS facade; no ROS graph/SDK."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from test_perception_ray_projector_boundary import (node, ready, raw_cloud, message,
    context, stamp, pose, EPOCH)


def pose_packet(node, *, valid=False, output=None, **changes):
    value = deepcopy(node.statuses['pose'])
    value.update(received_at_unix=node._now*1e-9, valid=valid, pose_valid=valid,
                 output_stamp_sec=output)
    value.update(changes)
    return value


def finish(node):
    node.tick()
    assert node.inflight is not None
    node.inflight[0].result(timeout=2.)
    node.tick()


@pytest.mark.parametrize('phase', ['pending', 'inflight', 'no_new_status'])
def test_current_pose_gap_preserves_certified_historical_scan_not_control_lease(node, phase):
    ready(node)
    assert node.authorized_pose_ns == EPOCH+160000000
    node.on_rays(raw_cloud())
    if phase == 'inflight':
        node.tick()
        node.inflight[0].result(timeout=2.)
    generation = node.work_generation
    node._now = EPOCH+300000000  # The former 80 ms pose lease has expired.
    if phase != 'no_new_status':
        node.on_status('pose', message(pose_packet(node)))
    assert node.current_context() is not None
    assert node.authorized_pose_ns == EPOCH+160000000
    assert node.work_generation == generation
    if phase == 'inflight':
        node.tick()
    else:
        finish(node)
    assert len(node.publisher.messages) == 1
    result = node.publisher.messages[0]
    assert result.rays.header.stamp == stamp(EPOCH)
    assert result.acquisition_end == stamp(EPOCH+100000000)
    assert result.alignment_stamp == stamp(EPOCH+100000000)
    assert node.last_published == [EPOCH, 0]
    if phase != 'no_new_status':
        assert node.statuses['pose']['valid'] is False  # Never faked valid/current.


def test_old_authorization_cannot_cover_new_scan_even_with_new_uncertified_pose_history(node):
    ready(node)
    for offset in (180000000, 200000000, 220000000, 240000000):
        node._now = EPOCH+260000000
        node.on_local(NS(header=NS(frame_id='odom', stamp=stamp(EPOCH+offset)),
            child_frame_id='body', pose=NS(pose=pose(offset*1e-9))))
        node.on_global(NS(header=NS(frame_id='map', stamp=stamp(EPOCH+offset)),
            pose=pose(offset*1e-9)))
    # A false status carrying a newer output time is NOT an authorization.
    node.on_status('pose', message(pose_packet(node, output=(EPOCH+240000000)*1e-9)))
    node.on_rays(raw_cloud(start=EPOCH+80000000))  # Scan ends at .18; cert ends .16.
    node.tick()
    assert node.inflight is None and not node.publisher.messages
    assert node.error == 'waiting_authorized_pose_history'
    assert node.authorized_pose_ns == EPOCH+160000000
    received = node.pending[0][3]
    node._now = EPOCH+280000000
    node.on_status('pose', message(pose_packet(node, valid=True, output=(EPOCH+220000000)*1e-9)))
    assert node.authorized_pose_ns == EPOCH+220000000
    assert node.pending[0][3] == received
    finish(node)
    assert node.publisher.messages[0].rays.header.stamp == stamp(EPOCH+80000000)


@pytest.mark.parametrize('change', [dict(schema=2), dict(pose_timeout_sec=1.),
    dict(motion_control_enabled=True), dict(frame_id='foreign'), dict(body_frame='other'),
    dict(reset_pending=True), dict(fault='imu_fault'), dict(epoch=2), dict(seed_id='new'),
    dict(valid=True, pose_valid=False, output_stamp_sec=None)])
def test_invalid_fault_reset_or_identity_change_revokes_pending_and_completed_worker(node, change):
    ready(node)
    node.on_rays(raw_cloud())
    node.tick()
    node.inflight[0].result(timeout=2.)
    generation = node.work_generation
    node._now = EPOCH+300000000
    node.on_status('pose', message(pose_packet(node, **change)))
    assert node.work_generation > generation
    assert node.authorized_pose_ns == 0 and node.authorized_pose_context is None
    assert node.pending == [None, None] and node.current_context() is None
    node.tick()
    assert not node.publisher.messages


def test_current_pose_gap_does_not_extend_raw_measurement_age(node):
    from d1max_pct_scan.ray_projection import ProjectionError
    ready(node)
    node.on_rays(raw_cloud())
    node._now = EPOCH+510000000  # Still a fresh identity (<.5), but raw scan is stale.
    node.on_status('pose', message(pose_packet(node)))
    node.tick()
    with pytest.raises(ProjectionError, match='stale'):
        node.inflight[0].result(timeout=2.)
    node.tick()
    assert not node.publisher.messages
    assert node.last_published == [0, 0]
    assert 'stale' in node.error


def test_new_context_cannot_reuse_an_expired_pose_certificate(node):
    ready(node)
    node.on_rays(raw_cloud())
    node.tick()
    node.inflight[0].result(timeout=2.)
    node._now = EPOCH+300000000
    node.on_context(message(context(sequence=2)))
    node.on_ack(message(context(sequence=2)))
    assert node.current_context().sequence == 2
    assert node.authorized_pose_ns == 0 and node.authorized_pose_context is None
    node.tick()
    assert not node.publisher.messages


def test_valid_flag_with_expired_output_cannot_create_a_history_certificate(node):
    ready(node)
    node.on_rays(raw_cloud())
    node._now = EPOCH+300000000
    node.on_status('pose', message(pose_packet(node, valid=True, output=(EPOCH+160000000)*1e-9)))
    assert node.authorized_pose_ns == 0 and node.pending == [None, None]
    assert node.current_context() is None
