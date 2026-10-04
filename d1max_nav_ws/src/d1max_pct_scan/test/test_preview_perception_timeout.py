"""A bounded preview-only ray lease is independent of pose and source time."""
import pytest

from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from d1max_pct_scan.live_scan_contract import perception_timeout
from test_preview_sensor_pause import preview
from test_native_ray_receipt_callback import harness, deliver
from test_live_scan_reference_order import lease_harness


def params(**changes):
    value = dict(input_timeout=.5, perception_backend='per_sensor_rays',
                 collision_policy='official', execution_mode='preview',
                 perception_timeout=.75)
    return dict(value, **changes)


def test_default_remains_half_second_and_legacy_input_timeout_unchanged():
    value = params()
    del value['perception_timeout']
    assert perception_timeout(value) == .5
    assert perception_timeout(dict(input_timeout=.3)) == .3
    assert perception_timeout(params()) == .75


@pytest.mark.parametrize('changes', [
    dict(execution_mode='execution'), dict(collision_policy='observed_free'),
    dict(perception_backend='deskewed_cloud'), dict(execution_mode='unknown')])
def test_extended_lease_rejected_outside_official_ray_preview(changes):
    with pytest.raises(ValueError, match='requires_official_ray_preview'):
        perception_timeout(params(**changes))


@pytest.mark.parametrize('value', [.751, .099, float('nan'), float('inf'), True, '.75'])
def test_extended_lease_is_finite_typed_and_bounded(value):
    with pytest.raises(ValueError, match='invalid_bounded_perception_timeout'):
        perception_timeout(params(perception_timeout=value))


def test_completed_native_source_at_600ms_keeps_original_stamp_and_expires():
    node, value = harness()
    node.p = params()
    node.now_s = lambda: 100.7
    value['received_at_unix'] = 100.69
    deliver(node, value)
    assert node.error == ''
    assert node.cloud_stamp == pytest.approx(100.1)
    original_receipt = node.cloud_received
    node.now_s = lambda: 100.86
    deliver(node, dict(value, received_at_unix=100.85))
    assert node.error == 'ray_measurement_stale_or_before_barrier'
    assert node.cloud_received == original_receipt
    assert node.cloud_stamp == pytest.approx(100.1)


def test_native_receipt_itself_cannot_exceed_preview_timeout():
    node, value = harness()
    node.p = params()
    node.now_s = lambda: 100.96
    value['source_stamp_ns'] = 100_900_000_000
    deliver(node, value)
    assert node.error == 'ray_receipt_stale'
    assert node.cloud_stamp == 0.


def test_preview_gate_source_lease_and_receipt_each_expire(monkeypatch):
    node, clock, _, _, _ = preview(monkeypatch)
    node.p.update(perception_timeout=.75)
    # Body/context/freeze readiness is held fresh by the pure harness; only
    # perception clocks are moved. Source time is never rewritten.
    clock.wall, clock.mono = 100.72, 10.72
    node.update_gate()
    assert node.gate.ready and not node.gate.preview_paused
    assert node.cloud_stamp == 100.10
    clock.wall, clock.mono = 100.86, 10.86
    node.update_gate()
    assert node.gate.preview_paused
    assert 'cloud_source' in node.ready_failure_reasons
    node.cloud_stamp = 100.8
    clock.mono = 10.91
    node.update_gate()
    assert 'cloud_receipt' in node.ready_failure_reasons


def test_extended_ray_lease_does_not_extend_pose_lease_or_status_receipts():
    node, _ = lease_harness()
    node.p.update(params())
    assert LiveScanBridge.current_context(node, 100.1, mono=50.079) is not None
    assert LiveScanBridge.current_context(node, 100.1, mono=50.081) is None
    assert LiveScanBridge.current_context(node, 100.1, mono=50.6) is None
    assert node.context_readiness_reason == 'localization_status_receipt_stale'
