"""Same source-timed native receipt; pose gap is not a map evidence failure."""
from copy import deepcopy
from types import MethodType

import pytest

from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from test_live_scan_soft_loss import harness, receipt, body_message


def preview(monkeypatch):
    node, clock, paths, clears, _ = harness(monkeypatch)
    node.p.update(collision_policy='official', execution_mode='preview',
        localization_session_id='session', tracking_frame='d1max_loc_tracking')
    node.navigation = dict(schema=1, epoch=1, seed_id='seed', valid=True,
        received_at_unix=100.20, reset_pending=False, fault='')
    node.localizer = dict(session_id='session', wall_time=100.20, local_epoch=1,
        active_seed_ns='seed', confirmed_seed_ns='seed', verified_confirmations=3,
        local_fault='', navigation=deepcopy(node.navigation),
        frames=dict(map=node.p['map_frame'], tracking=node.p['tracking_frame']))
    node.pose_status = dict(schema=1, epoch=1, seed_id='seed', valid=True,
        pose_valid=True, fault='', reset_pending=False, motion_control_enabled=False,
        frame_id=node.p['map_frame'], body_frame=node.p['body_frame'],
        pose_timeout_sec=.08, received_at_unix=100.15, output_stamp_sec=100.15)
    node.pose_received = 10.15
    node.nav_received = node.localizer_received = node.freeze_received = 10.20
    node.execution_frozen = True
    node.body_received = 10.16
    for name in ('current_context', 'nav_body_ready'):
        setattr(node, name, MethodType(getattr(LiveScanBridge, name), node))
    return node, clock, paths, clears


def test_fresh_native_map_receipt_survives_pose_gap_but_does_not_admit_pose(monkeypatch):
    node, clock, paths, clears = preview(monkeypatch)
    assert node.current_context(clock.wall) is None  # last 50 Hz pose is 100 ms old
    node.update_gate()
    generation = node.gate.generation
    assert not node.gate.ready and node.gate.preview_paused
    assert node.cloud_stamp == 100.10
    receipt(node, clock, source=100.21)
    assert node.cloud_stamp == node.last_output_stamp == pytest.approx(100.21)
    assert node.ray_status_stamp == 100.25
    assert node.cloud_received == 10.25  # actual callback time, not forged pose time
    assert node.pose_received == 10.15
    assert not node.gate.ready and not node.sensor_ready
    assert not node.nav_body_ready(clock.wall, clock.mono)
    assert node.gate.active and node.gate.preview_paused and not paths
    assert not node.body_ready_checks['localization_context']
    assert 'cloud_source' not in node.ready_failure_reasons
    assert ('admission', 'preview_sensor_paused') in clears
    # Only a genuinely newer pose/body source and receipt can restore readiness.
    clock.wall, clock.mono = 100.28, 10.28
    node.pose_status.update(received_at_unix=100.27, output_stamp_sec=100.27)
    node.pose_received = 10.27
    node.body, node.body_received = body_message(100.27), 10.27
    node.update_gate()
    assert node.gate.ready and node.gate.active and not node.gate.preview_paused
    assert node.gate.generation == generation
    assert node.cloud_stamp == pytest.approx(100.21)
    assert node.gate.trajectory_barrier == 100.28  # new trajectory proof still required


@pytest.mark.parametrize('soft_state', ['pose_invalid', 'pose_geometry_invalid', 'navigation_invalid',
    'embedded_invalid', 'pose_missing'])
def test_soft_pose_invalidity_does_not_invalidate_completed_map_observations(monkeypatch, soft_state):
    node, clock, _, _ = preview(monkeypatch)
    if soft_state == 'pose_invalid': node.pose_status['valid'] = False
    if soft_state == 'pose_geometry_invalid': node.pose_status['pose_valid'] = False
    if soft_state == 'navigation_invalid': node.navigation['valid'] = False
    if soft_state == 'embedded_invalid': node.localizer['navigation']['valid'] = False
    if soft_state == 'pose_missing': node.pose_status = {}
    node.update_gate()
    receipt(node, clock, source=100.21)
    assert node.cloud_stamp == pytest.approx(100.21) and node.cloud_received == 10.25
    assert not node.gate.ready and not node.sensor_ready
    assert node.gate.active and node.gate.preview_paused
    assert not node.nav_body_ready(clock.wall, clock.mono)


@pytest.mark.parametrize('failure', ['pose_fault', 'pose_reset', 'pose_epoch', 'pose_seed',
    'pose_frame', 'pose_body_frame', 'pose_motion', 'identity_fault', 'local_fault',
    'nav_fault', 'nav_reset', 'nav_epoch', 'nav_seed', 'status_stale', 'status_receipt_stale',
    'freeze_false', 'freeze_stale', 'map_fault', 'map_unacked', 'foreign_map_identity',
    'invalid_ray', 'old_ray', 'wrong_ray_context', 'ray_before_barrier'])
def test_hard_or_source_failures_cannot_use_retained_identity(monkeypatch, failure):
    node, clock, _, _ = preview(monkeypatch)
    changes = {}
    if failure == 'pose_fault': node.pose_status['fault'] = 'lost'
    if failure == 'pose_reset': node.pose_status['reset_pending'] = True
    if failure == 'pose_epoch': node.pose_status['epoch'] = 2
    if failure == 'pose_seed': node.pose_status['seed_id'] = 'new'
    if failure == 'pose_frame': node.pose_status['frame_id'] = 'other'
    if failure == 'pose_body_frame': node.pose_status['body_frame'] = 'other'
    if failure == 'pose_motion': node.pose_status['motion_control_enabled'] = True
    if failure == 'identity_fault': node.pose_identity_fault = 'continuous_pose_frame_mismatch'
    if failure == 'local_fault': node.localizer['local_fault'] = 'lost'
    if failure == 'nav_fault': node.navigation['fault'] = 'lost'
    if failure == 'nav_reset': node.navigation['reset_pending'] = True
    if failure == 'nav_epoch': node.navigation['epoch'] = 2
    if failure == 'nav_seed': node.navigation['seed_id'] = 'new'
    if failure == 'status_stale': node.localizer['wall_time'] = 99.
    if failure == 'status_receipt_stale': node.nav_received = 9.
    if failure == 'freeze_false': node.execution_frozen = False
    if failure == 'freeze_stale': node.freeze_received = 9.
    if failure == 'map_fault': node.map_context_fault = 'static_extrinsic_changed_requires_context_reset'
    if failure == 'map_unacked': node.map_context_ready = False
    if failure == 'foreign_map_identity': node.map_context_identity = ('other', 1, 'seed')
    if failure == 'invalid_ray': changes['valid'] = False
    if failure == 'old_ray': changes['source_stamp_ns'] = 99_000_000_000
    if failure == 'wrong_ray_context': changes['epoch'] = 2
    if failure == 'ray_before_barrier': node.sensor_barrier = 100.22
    # Invalid packets must not alter perception receipt/source state at all;
    # separately exercised update_gate can revoke other state as required.
    node.update_gate = lambda: None
    original = (node.cloud_stamp, node.cloud_received, node.ray_status_stamp)
    receipt(node, clock, source=100.21, **changes)
    assert (node.cloud_stamp, node.cloud_received, node.ray_status_stamp) == original


@pytest.mark.parametrize('policy,mode,backend', [('observed_free', 'preview', 'per_sensor_rays'),
    ('official', 'execution', 'per_sensor_rays'), ('official', 'preview', 'deskewed_cloud')])
def test_other_backends_or_motion_do_not_borrow_retained_identity(monkeypatch, policy, mode, backend):
    node, clock, _, _ = preview(monkeypatch)
    node.p.update(collision_policy=policy, execution_mode=mode, perception_backend=backend)
    node.update_gate = lambda: None
    receipt(node, clock, source=100.21)
    assert node.cloud_stamp == 100.10 and node.cloud_received == 10.15


def test_new_native_status_cannot_renew_stopped_sensor_through_pose_gap(monkeypatch):
    node, clock, _, _ = preview(monkeypatch)
    node.update_gate = lambda: None
    receipt(node, clock, source=100.21)
    original = node.cloud_received
    clock.wall, clock.mono = 100.30, 10.30
    # A repeated completed status keeps its original receipt watermark.
    receipt(node, clock, source=100.21, received_at_unix=100.25)
    assert node.cloud_received == original
    # A newer publication cannot freshen an expired physical source.
    receipt(node, clock, source=99.)
    assert node.cloud_received == original and node.cloud_stamp == pytest.approx(100.21)
