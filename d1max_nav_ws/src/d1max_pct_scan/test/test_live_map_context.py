"""Bridge context protocol exercised without ROS init or a transport graph."""
import json
import math
from collections import deque
from types import SimpleNamespace

import pytest
from std_msgs.msg import String
from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from d1max_pct_scan.live_scan_contract import ReferenceGate, cloud_source_issue
from builtin_interfaces.msg import Time


def harness():
    sent, cleared, updates = [], [], []
    node = SimpleNamespace(map_context_identity=None, map_context_record=None,
        map_context_sequence=0, map_context_ready=False, map_context_last_publish=-math.inf,
        reference_refresh=None, sensor_barrier=0., gate=ReferenceGate(),
        map_context_pub=SimpleNamespace(publish=sent.append), now_s=lambda:100.,
        current_context=lambda now:('session',1,'seed'), update_gate=lambda:updates.append(True))
    def clear(reason):
        cleared.append(reason)
        node.sensor_barrier=100.
    node.clear_outputs=clear
    return node,sent,cleared,updates


def test_bridge_waits_for_exact_reset_ack_then_retains_context_across_short_loss():
    node,sent,cleared,updates=harness()
    identity=('session',1,'seed')
    LiveScanBridge.sync_map_context(node,identity,100.,50.)
    assert len(sent)==len(cleared)==1 and not node.map_context_ready
    assert json.loads(sent[0].data)==dict(schema=1,session_id='session',epoch=1,
        seed_id='seed',sequence=1,barrier_ns=100_000_000_000)
    # An acknowledgement of a different identity cannot release the input gate.
    wrong=dict(node.map_context_record,seed_id='old')
    LiveScanBridge.on_map_context_ack(node,String(data=json.dumps(wrong)))
    assert not node.map_context_ready
    LiveScanBridge.on_map_context_ack(node,sent[0])
    assert node.map_context_ready and updates
    LiveScanBridge.sync_map_context(node,None,100.2,50.2)
    LiveScanBridge.sync_map_context(node,identity,100.3,50.3)
    assert node.map_context_ready and len(sent)==len(cleared)==1


def test_context_retry_is_bounded_and_goal_generation_never_resets_map():
    node,sent,cleared,_=harness()
    identity=('session',1,'seed')
    for i in range(20):
        LiveScanBridge.sync_map_context(node,identity,100+i*.01,50+i*.01)
    assert len(sent)==1 and len(cleared)==1
    LiveScanBridge.sync_map_context(node,identity,100.3,50.3)
    assert len(sent)==2 and sent[0].data==sent[1].data
    LiveScanBridge.on_map_context_ack(node,sent[-1])
    node.gate.generation+=10
    LiveScanBridge.sync_map_context(node,identity,100.6,50.6)
    assert node.map_context_sequence==1 and len(cleared)==1


def test_seed_or_epoch_change_requires_a_new_ack_and_rejects_delayed_old_ack():
    for replacement in [('session',1,'new-seed'),('session',2,'seed')]:
        node,sent,cleared,_=harness()
        LiveScanBridge.sync_map_context(node,('session',1,'seed'),100.,50.)
        first=sent[0]
        LiveScanBridge.on_map_context_ack(node,first)
        node.current_context=lambda now:replacement
        LiveScanBridge.sync_map_context(node,replacement,100.1,50.1)
        assert len(cleared)==2 and not node.map_context_ready
        LiveScanBridge.on_map_context_ack(node,first)
        assert not node.map_context_ready
        LiveScanBridge.on_map_context_ack(node,sent[-1])
        assert node.map_context_ready and node.map_context_sequence==2


@pytest.mark.parametrize('published, received', [(102.1,102.1),(102.05,102.1),(102.1,102.05)])
def test_reset_barrier_covers_future_old_clouds_and_context_uses_that_same_boundary(published, received):
    node,sent,_,_=harness()
    node.p=dict(session_id='session',map_frame='map')
    node.now_s=lambda:102.
    node.sensor_barrier=101.
    node.last_output_stamp=published;node.last_input_stamp=received
    node.pending=deque([object()])
    node.ground_support_check={'valid':True}
    node.cloud_stamp=published;node.cloud_received=51.
    node.drop_clouds=lambda *args:None
    node.reference_pub=SimpleNamespace(publish=lambda _:None)
    node.cloud_pub=SimpleNamespace(publish=lambda _:None)
    node.cloud_message=lambda points,stamp:None
    node.clear_marker=node.clear_attempt_debug=lambda:None
    node.revoke_execution=lambda _:None
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=102)))
    node.get_logger=lambda:SimpleNamespace(info=lambda _:None)
    node.clear_outputs=lambda reason:LiveScanBridge.clear_outputs(node,reason)
    node.map_context_identity=('session',1,'old-seed')
    LiveScanBridge.sync_map_context(node,('session',2,'new-seed'),102.,52.)
    record=json.loads(sent[-1].data)
    assert not node.pending and node.cloud_stamp==0.
    assert node.sensor_barrier>=max(published,received)
    assert record['barrier_ns']==math.ceil(node.sensor_barrier*1e9)
    assert record['barrier_ns']>=102_100_000_000
    # Timestamp is still within normal freshness after a few milliseconds;
    # rejection specifically comes from the new context boundary, not timeout.
    assert cloud_source_issue(frame_id='tracking',tracking_frame='tracking',
        stamp=102.1,now=102.01,timeout=.5,context=('session',2,'new-seed'),
        sensor_barrier=node.sensor_barrier,last_input_stamp=0.,data_size=12,
        point_count=1,max_input_points=100)=='before_sensor_barrier'


def projector_fault(node, **changes):
    value = dict(schema=1, session_id='session', received_at_unix=100.,
                 context={k:v for k,v in node.map_context_record.items() if k != 'schema'},
                 valid=False, fault='map_alignment_accumulation_requires_context_reset')
    value.update(changes)
    return String(data=json.dumps(value))


def fault_harness():
    node,sent,cleared,updates=harness()
    node.p=dict(perception_backend='per_sensor_rays',input_timeout=.5)
    node.map_context_fault=None
    LiveScanBridge.sync_map_context(node,('session',1,'seed'),100.,50.)
    LiveScanBridge.on_map_context_ack(node,sent[-1])
    return node,sent,cleared,updates


def test_projector_fault_revokes_without_waiting_for_native_ray_source_lease():
    node,sent,cleared,_=fault_harness()
    node.gate.ready=True
    LiveScanBridge.on_projector_status(node,projector_fault(node))
    assert node.map_context_fault=='map_alignment_accumulation_requires_context_reset'
    assert not node.map_context_ready and not node.gate.ready
    assert cleared[-1]==node.map_context_fault
    count=len(cleared)
    LiveScanBridge.on_projector_status(node,projector_fault(node))
    assert len(cleared)==count  # idempotent fault heartbeat
    LiveScanBridge.on_map_context_ack(node,sent[-1])
    LiveScanBridge.sync_map_context(node,('session',1,'seed'),100.1,50.1)
    assert not node.map_context_ready and len(sent)==1
    # A healthy status or delayed native receipt does not undo a latched fault.
    LiveScanBridge.on_projector_status(node,projector_fault(node,valid=True,fault=None))
    node.cloud_stamp=0.
    LiveScanBridge.on_native_rays(node,String(data=json.dumps({'received_at_unix':100.1})))
    assert node.cloud_stamp==0. and node.map_context_fault is not None


@pytest.mark.parametrize('changes', [dict(session_id='foreign'),dict(received_at_unix=99.),
    dict(schema=True),
    dict(context={}),dict(context=None),dict(valid=True),dict(fault=None),
    dict(fault='waiting_local_pose_history'),
    dict(context=dict(session_id='session',epoch=1,seed_id='seed',sequence=2,barrier_ns=100000000000))])
def test_foreign_stale_nonfault_status_cannot_revoke_current_context(changes):
    node,_,cleared,_=fault_harness()
    LiveScanBridge.on_projector_status(node,projector_fault(node,**changes))
    assert node.map_context_ready and node.map_context_fault is None and len(cleared)==1


def test_recovery_requires_owner_replacement_and_its_exact_ack():
    node,sent,_,_=fault_harness()
    rejected=projector_fault(node)
    LiveScanBridge.on_projector_status(node,rejected)
    old_ack=sent[-1]
    replacement=('session',2,'seed')
    node.current_context=lambda _:replacement
    LiveScanBridge.sync_map_context(node,replacement,100.1,50.1)
    assert node.map_context_fault is None and not node.map_context_ready
    LiveScanBridge.on_map_context_ack(node,old_ack)
    assert not node.map_context_ready
    LiveScanBridge.on_map_context_ack(node,sent[-1])
    assert node.map_context_ready
    LiveScanBridge.on_projector_status(node,rejected)
    assert node.map_context_ready and node.map_context_fault is None
