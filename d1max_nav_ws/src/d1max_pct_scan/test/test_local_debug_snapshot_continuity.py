"""Production bridge marker snapshots replace stable objects; no ROS graph."""
from copy import deepcopy

from visualization_msgs.msg import Marker, MarkerArray

from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from test_preview_trajectory_revalidation import bridge


def test_repeated_same_task_snapshot_overwrites_without_deleteall(monkeypatch):
    node, _, emitted = bridge(monkeypatch)
    emitted.clear()
    LiveScanBridge.publish_preview_trajectory(node)
    LiveScanBridge.publish_preview_trajectory(node)
    arrays = [value for value in emitted if isinstance(value, MarkerArray)]
    assert len(arrays) == 2
    assert all(m.action == Marker.ADD for packet in arrays for m in packet.markers)
    assert [(m.ns, m.id) for m in arrays[0].markers] == [(m.ns, m.id) for m in arrays[1].markers]
    assert arrays[0] == arrays[1]


def test_missing_marker_kind_is_deleted_without_erasing_surviving_kinds(monkeypatch):
    node, _, emitted = bridge(monkeypatch)
    original_keys = set(node.debug_marker_keys)
    real_specs = __import__('d1max_pct_scan.live_scan_bridge', fromlist=['local_debug_specs']).local_debug_specs
    monkeypatch.setattr('d1max_pct_scan.live_scan_bridge.local_debug_specs',
                        lambda **kw: real_specs(**kw)[:1])
    emitted.clear()
    LiveScanBridge.publish_preview_trajectory(node)
    packet = [value for value in emitted if isinstance(value, MarkerArray)][-1]
    additions = {(m.ns, m.id) for m in packet.markers if m.action == Marker.ADD}
    deletions = {(m.ns, m.id) for m in packet.markers if m.action == Marker.DELETE}
    assert len(additions) == 1 and deletions == original_keys-additions
    assert all(m.action != Marker.DELETEALL for m in packet.markers)


def test_explicit_revoke_still_clears_all_objects_and_marker_key_state(monkeypatch):
    node, _, emitted = bridge(monkeypatch)
    assert node.debug_marker_keys
    emitted.clear()
    LiveScanBridge.publish_debug_delete(node)
    assert not node.debug_marker_keys
    packet = [value for value in emitted if isinstance(value, MarkerArray)][-1]
    assert len(packet.markers) == 1 and packet.markers[0].action == Marker.DELETEALL
