"""Official preview task continuity; pure callbacks, no ROS init or robot IO."""
import pytest
from visualization_msgs.msg import Marker

from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from d1max_pct_scan.live_scan_contract import admissible_tagged_spline
from test_live_scan_soft_loss import harness, receipt
from test_live_scan_reference_order import (
    harness as reference_harness, path, spline_harness, spline, accepted_debug,
    lease_harness)


def preview(monkeypatch):
    node, clock, paths, clears, contexts = harness(monkeypatch)
    node.p.update(collision_policy='official', execution_mode='preview')
    node.clear_marker = lambda **kwargs: clears.append(('marker', kwargs))
    return node, clock, paths, clears, contexts


def lose_ray_lease(node, clock):
    clock.wall, clock.mono = 100.62, 10.62
    node.update_gate()


def test_ray_pause_preserves_native_task_generation_but_withdraws_visuals_and_authority(monkeypatch):
    node, clock, paths, clears, _ = preview(monkeypatch)
    generation, issue, digest = node.gate.generation, node.gate.issued_at, node.gate.digest
    lose_ray_lease(node, clock)
    assert node.gate.active and node.gate.preview_paused and not node.gate.ready
    assert (node.gate.generation, node.gate.issued_at, node.gate.digest) == (generation, issue, digest)
    assert node.gate.trajectory_barrier == clock.wall
    assert 'cloud_source' in node.ready_failure_reasons
    assert node.reference_refresh is None and not paths
    assert any(isinstance(item, tuple) and item[0] == 'marker' for item in clears)
    assert ('admission', 'preview_sensor_paused') in clears
    assert node.cloud_stamp == 100.10 and node.sensor_barrier == 99.
    count = len(clears)
    node.update_gate(); node.update_gate()
    assert len(clears) == count and node.gate.generation == generation


def test_same_context_recovery_keeps_reference_but_demands_new_native_curve(monkeypatch):
    node, clock, paths, _, _ = preview(monkeypatch)
    generation = node.gate.generation
    lose_ray_lease(node, clock)
    clock.wall, clock.mono = 100.68, 10.68
    receipt(node, clock, source=100.61)
    assert node.gate.ready and node.gate.active and not node.gate.preview_paused
    assert node.gate.generation == generation and not paths and node.reference_refresh is None
    assert node.gate.trajectory_barrier == 100.68
    assert node.cloud_stamp == 100.61  # never refresh the original ray source clock
    common = dict(session_id='session', generation=generation, frame_id=node.gate.frame_id,
                  trajectory_id=5, expected_session='session', gate=node.gate, last_id=4, now=100.70)
    assert not admissible_tagged_spline(start_time=100.65, **common)
    assert admissible_tagged_spline(start_time=100.69, **common)


@pytest.mark.parametrize('failure', ['context', 'map_fault', 'body_context'])
def test_hard_or_non_sensor_loss_still_cancels_a_paused_reference(monkeypatch, failure):
    node, clock, paths, _, _ = preview(monkeypatch)
    lose_ray_lease(node, clock)
    generation = node.gate.generation
    if failure == 'context':
        clock.context = ('session', 2, 'changed-seed')
    elif failure == 'map_fault':
        node.map_context_fault = 'static_extrinsic_changed_requires_context_reset'
    else:
        clock.context = None
    node.update_gate()
    assert not node.gate.active and not node.gate.preview_paused
    assert node.gate.generation > generation
    assert paths and not paths[-1].path.poses


def test_explicit_cancel_while_paused_never_resumes_on_new_sensor_evidence(monkeypatch):
    node, clock, _, _, _ = preview(monkeypatch)
    lose_ray_lease(node, clock)
    node.gate.revoke(clock.wall, 'explicit_reference_cancel', advance_barrier=False)
    generation = node.gate.generation
    clock.wall, clock.mono = 100.68, 10.68
    receipt(node, clock, source=100.61)
    assert node.gate.ready and not node.gate.active and not node.gate.preview_paused
    assert node.gate.generation == generation


@pytest.mark.parametrize('policy,mode,backend', [
    ('observed_free', 'preview', 'per_sensor_rays'),
    ('official', 'execution', 'per_sensor_rays'),
    ('official', 'preview', 'deskewed_cloud')])
def test_execution_strict_and_legacy_paths_keep_existing_revocation(monkeypatch, policy, mode, backend):
    node, clock, paths, _, _ = preview(monkeypatch)
    node.p.update(collision_policy=policy, execution_mode=mode, perception_backend=backend)
    generation = node.gate.generation
    lose_ray_lease(node, clock)
    assert not node.gate.active and not node.gate.preview_paused
    assert node.gate.generation == generation+1 and paths


def test_already_inactive_unready_reference_does_not_generate_repeated_cancels():
    bridge, published, cleared = reference_harness()
    bridge.p.update(collision_policy='official', execution_mode='preview')
    bridge.gate.active = bridge.gate.ready = False
    bridge.revoke_execution = lambda reason: None
    generation = bridge.gate.generation
    for stamp in (100.2, 100.3):
        LiveScanBridge.on_reference(bridge, path(stamp))
    assert bridge.counts['rejected_references'] == 2
    assert bridge.gate.generation == generation and not cleared and not published


def test_same_owner_geometry_during_sensor_pause_is_not_a_new_task():
    bridge, published, cleared = reference_harness()
    bridge.p.update(collision_policy='official', execution_mode='preview')
    bridge.gate.pause_preview_reference(100.2)
    generation = bridge.gate.generation
    LiveScanBridge.on_reference(bridge, path(100.3))
    assert bridge.gate.active and bridge.gate.preview_paused
    assert bridge.gate.generation == generation and bridge.gate.last_path_stamp == 100.3
    assert not cleared and not published and bridge.reference_refresh is None


def test_pre_recovery_spline_and_debug_cannot_reappear_after_resume(monkeypatch):
    node, _, emitted = spline_harness(monkeypatch)
    node.p.update(collision_policy='official', perception_backend='per_sensor_rays')
    node.gate.pause_preview_reference(100.3)
    assert node.gate.resume_preview_reference(100.4, node.gate.context)
    LiveScanBridge.on_spline(node, spline(1))  # original start 100.2, before recovery
    LiveScanBridge.on_local_debug(node, accepted_debug(1))
    assert node.pending_spline_marker is None and node.visible_spline_id == -1
    assert not [m for m in emitted if isinstance(m, Marker) and m.action == Marker.ADD]
    node.now_s = lambda: 100.6
    newer = spline(2)
    newer.trajectory.start_time.nanosec = 450_000_000
    LiveScanBridge.on_spline(node, newer)
    LiveScanBridge.on_local_debug(node, accepted_debug(2, stamp=100.55))
    assert node.visible_spline_id == 2
    assert len([m for m in emitted if isinstance(m, Marker) and m.action == Marker.ADD]) == 2
    assert not node.validated and not node.admissions[-1]['valid']


def test_status_identifies_pose_lease_failure_without_relaxing_it():
    node, _ = lease_harness()
    assert LiveScanBridge.current_context(node, 100.1, mono=50.079) is not None
    assert node.context_readiness_reason == 'valid_same_context'
    assert LiveScanBridge.current_context(node, 100.1, mono=50.081) is None
    assert node.context_readiness_reason != 'valid_same_context'
    assert LiveScanBridge.current_context(node, 100.1, mono=50.6) is None
    assert node.context_readiness_reason == 'localization_status_receipt_stale'


def test_status_identifies_freeze_failure_independently_from_valid_body():
    node, _ = lease_harness()
    node.current_context = lambda now, **kw: ('test', 4, 'seed')
    node.body = path(100.05)  # only its stamped header is inspected here
    node.body_context, node.body_received = ('test', 4, 'seed'), 50.
    node.sensor_barrier, node.freeze_received = 100.04, 50.
    node.execution_frozen = True
    assert LiveScanBridge.nav_body_ready(node, 100.1, 50.02)
    node.execution_frozen = False
    assert not LiveScanBridge.nav_body_ready(node, 100.1, 50.02)
    assert [key for key, valid in node.body_ready_checks.items() if not valid] == ['freeze_state']
