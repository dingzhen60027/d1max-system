"""Actual bridge diagnostic callbacks, no ROS init, node, graph, or robot IO."""
import math
from types import SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from d1max_pct_scan.live_scan_contract import ReferenceGate


def snapshot(kinds=('reference','blocked'), *, stamp_ns=100_200_000_000, generation=3):
    markers=[]
    for kind in ('clear',*kinds):
        marker=Marker()
        marker.header.frame_id='map'
        marker.header.stamp=Time(sec=stamp_ns//10**9,nanosec=stamp_ns%10**9)
        marker.ns=f'local_attempt/session/{generation}/{kind}'
        marker.id=0
        marker.action=Marker.DELETEALL if kind=='clear' else Marker.ADD
        marker.type=Marker.LINE_STRIP
        marker.pose.orientation.w=1.
        marker.scale.x=marker.scale.y=marker.scale.z=.03
        marker.color.r=1.;marker.color.b=.8;marker.color.a=.9
        marker.lifetime.sec=2
        if kind!='clear':marker.points=[Point(x=1.,y=2.,z=.55),Point(x=2.,y=2.,z=.55)]
        markers.append(marker)
    return MarkerArray(markers=markers)


def harness():
    sent=[]
    node=NS(p=dict(session_id='session',map_frame='map'),
        gate=ReferenceGate(generation=3,ready=True,active=True,issued_at=100.),
        counts=dict(attempt_debug_rejected=0,attempt_debug_published=0),
        attempt_marker_keys=set(),attempt_marker_count=0,last_attempt_stamp_ns=0,
        attempt_received=-math.inf,attempt_visible_until=-math.inf,error='',now=100.3,
        attempt_debug_pub=NS(publish=sent.append),update_gate=lambda:None)
    node.now_s=lambda:node.now
    node.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=int(node.now))))
    return node,sent


def test_same_generation_updates_do_not_clear_and_recreate_existing_markers():
    node,sent=harness()
    first=snapshot()
    LiveScanBridge.on_attempt_debug(node,first)
    assert len(sent)==1 and all(m.action==Marker.ADD for m in sent[-1].markers)
    original_keys={(m.ns,m.id) for m in sent[-1].markers}
    node.now=100.8
    second=snapshot(stamp_ns=100_700_000_000)
    LiveScanBridge.on_attempt_debug(node,second)
    assert len(sent)==2 and all(m.action==Marker.ADD for m in sent[-1].markers)
    assert {(m.ns,m.id) for m in sent[-1].markers}==original_keys
    assert node.attempt_marker_count==2
    for marker in sent[-1].markers:
        assert marker.header.stamp==Time(sec=100,nanosec=700_000_000)
        assert marker.lifetime.sec+marker.lifetime.nanosec*1e-9==pytest.approx(1.9)
        assert marker.color.g==0.  # failure diagnostic never styled accepted


def test_missing_kind_is_deleted_without_erasing_surviving_reference():
    node,sent=harness();LiveScanBridge.on_attempt_debug(node,snapshot())
    node.now=100.8
    LiveScanBridge.on_attempt_debug(node,snapshot(('reference',),stamp_ns=100_700_000_000))
    actions={m.ns.rsplit('/',1)[-1]:m.action for m in sent[-1].markers}
    assert actions=={'blocked':Marker.DELETE,'reference':Marker.ADD}
    assert node.attempt_marker_keys=={('local_attempt/session/3/reference',0)}


def test_native_clear_only_snapshot_removes_all_keys_without_republishing_old_geometry():
    node,sent=harness();LiveScanBridge.on_attempt_debug(node,snapshot())
    node.now=100.8
    LiveScanBridge.on_attempt_debug(node,snapshot((),stamp_ns=100_700_000_000))
    assert len(sent[-1].markers)==2 and all(m.action==Marker.DELETE for m in sent[-1].markers)
    assert not node.attempt_marker_keys and node.attempt_marker_count==0


@pytest.mark.parametrize('generation,stamp', [(2,100_700_000_000),(3,100_200_000_000),(3,98_700_000_000)])
def test_wrong_task_old_or_stale_packet_cannot_clear_current_snapshot(generation,stamp):
    node,sent=harness();LiveScanBridge.on_attempt_debug(node,snapshot())
    keys=set(node.attempt_marker_keys);deadline=node.attempt_visible_until
    node.now=100.8
    LiveScanBridge.on_attempt_debug(node,snapshot((),generation=generation,stamp_ns=stamp))
    assert len(sent)==1 and node.attempt_marker_keys==keys
    assert node.attempt_visible_until==deadline and node.counts['attempt_debug_rejected']==1


def test_invalid_gate_rejects_new_diagnostics_and_real_clear_removes_everything():
    node,sent=harness();LiveScanBridge.on_attempt_debug(node,snapshot())
    node.gate.ready=node.gate.active=False
    LiveScanBridge.clear_attempt_debug(node)
    assert sent[-1].markers[0].action==Marker.DELETEALL
    assert not node.attempt_marker_keys and node.attempt_marker_count==0
    assert node.attempt_visible_until==-math.inf
    node.now=100.8
    LiveScanBridge.on_attempt_debug(node,snapshot(stamp_ns=100_700_000_000))
    assert len(sent)==2  # no old diagnostic resurrection while sensor/task invalid
