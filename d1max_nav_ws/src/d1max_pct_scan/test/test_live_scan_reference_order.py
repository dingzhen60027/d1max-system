"""Reference/cancel message ordering without creating a ROS node or robot IO."""
from types import SimpleNamespace
import json
import math

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
from d1max_planning_interfaces.msg import LocalPlanDebug, TaggedBspline

from d1max_pct_scan import live_scan_bridge
from d1max_pct_scan.live_scan_bridge import LiveScanBridge, new_reference_message
from d1max_pct_scan.live_scan_contract import ReferenceGate
from d1max_pct_scan.local_debug import LocalDebugGate
from d1max_pct_scan.motion_execution import ExecutionLease, MotionConfig
from d1max_pct_scan.pct_ground_support import GroundSupportError


def lease_value(**changes):
    value = dict(schema=1, epoch=4, seed_id='seed', received_at_unix=100.09,
        output_stamp_sec=100.08, pose_timeout_sec=.08, valid=True, pose_valid=True,
        fault='', reset_pending=False, motion_control_enabled=False,
        frame_id='d1max_loc_map', body_frame='d1max_loc_base_link')
    value.update(changes)
    return value


def lease_harness():
    nav = dict(schema=1, epoch=4, seed_id='seed', received_at_unix=100., valid=True,
               fault='', reset_pending=False)
    localizer = dict(session_id='test', wall_time=100., local_epoch=4, active_seed_ns='seed',
        confirmed_seed_ns='seed', verified_confirmations=3, local_fault='',
        navigation=dict(nav), frames=dict(map='d1max_loc_map', tracking='d1max_loc_tracking'))
    calls = []
    node = SimpleNamespace(p=dict(input_timeout=.5, map_frame='d1max_loc_map',
        tracking_frame='d1max_loc_tracking', body_frame='d1max_loc_base_link',
        localization_session_id='test'), navigation=nav, localizer=localizer,
        nav_received=50., localizer_received=50., pose_received=50., pose_status=lease_value(),
        pose_status_stamp=100.09, navigation_status_stamp=100., localizer_status_stamp=100.,
        highest_local_epoch=4, now_s=lambda:100.1, error='', update_gate=lambda:calls.append('gate'))
    node.map_context_ready = True
    node.map_context_identity = ('test',4,'seed')
    return node, calls


def test_bridge_pose_receipt_uses_advertised_ttl_not_half_second_context_timeout():
    node, _ = lease_harness()
    assert LiveScanBridge.current_context(node,100.1,mono=50.079) == ('test',4,'seed')
    assert LiveScanBridge.current_context(node,100.1,mono=50.081) is None
    assert LiveScanBridge.current_context(node,100.1,mono=49.999) is None


def test_invalid_future_pose_revokes_authority_without_poisoning_watermark():
    node, calls = lease_harness()
    LiveScanBridge.on_pose_status(node,String(data=json.dumps(lease_value(received_at_unix=1000.))))
    assert node.pose_status == {} and node.pose_status_stamp == 100.09 and calls
    # An out-of-order but fresh packet cannot revive authorization after the error.
    LiveScanBridge.on_pose_status(node,String(data=json.dumps(lease_value(received_at_unix=100.085))))
    assert node.pose_status == {} and node.pose_status_stamp == 100.09
    LiveScanBridge.on_pose_status(node,String(data=json.dumps(lease_value(received_at_unix=100.095))))
    assert node.pose_status['received_at_unix'] == 100.095
    assert node.pose_status_stamp == 100.095


def test_wrong_frame_pose_revokes_without_advancing_packet_watermark():
    node, _ = lease_harness()
    LiveScanBridge.on_pose_status(node,String(data=json.dumps(
        lease_value(received_at_unix=100.095,frame_id='old_map'))))
    assert node.pose_status == {} and node.pose_status_stamp == 100.09


def test_invalid_slow_status_timestamps_do_not_poison_recovery():
    node, _ = lease_harness()
    bad_nav = dict(node.navigation,received_at_unix=1000.)
    LiveScanBridge.on_navigation(node,String(data=json.dumps(bad_nav)))
    assert node.navigation == {} and node.navigation_status_stamp == 100.
    good_nav = dict(bad_nav,received_at_unix=100.05)
    LiveScanBridge.on_navigation(node,String(data=json.dumps(good_nav)))
    assert node.navigation['received_at_unix'] == 100.05
    bad_loc = dict(node.localizer,wall_time=1000.)
    LiveScanBridge.on_localizer(node,String(data=json.dumps(bad_loc)))
    assert node.localizer == {} and node.localizer_status_stamp == 100.
    LiveScanBridge.on_localizer(node,String(data=json.dumps(dict(bad_loc,wall_time=100.05))))
    assert node.localizer['wall_time'] == 100.05


def test_body_cannot_be_relabelled_across_sensor_barrier_or_regress_in_time():
    node, _ = lease_harness()
    published = []
    node.sensor_barrier, node.last_body_stamp = 100.07, 100.02
    node.body = None
    node.body_received = -math.inf
    node.body_context = None
    node.current_context = lambda now:('test',4,'seed')
    node.nav_body_ready = lambda now,mono:True
    node.body_pub = SimpleNamespace(publish=published.append)
    node.counts = dict(body_published=0)
    def body(stamp):
        message = Odometry()
        message.header.frame_id = 'd1max_loc_map'
        message.child_frame_id = 'd1max_loc_base_link'
        message.header.stamp.sec = int(stamp)
        message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
        message.pose.pose.orientation.w = 1.
        return message
    LiveScanBridge.on_body(node,body(100.06))
    assert node.body is None and not published and node.last_body_stamp == 100.02
    LiveScanBridge.on_body(node,body(100.09))
    assert len(published)==1 and node.last_body_stamp == 100.09
    accepted = node.body
    LiveScanBridge.on_body(node,body(100.08))
    assert node.body is accepted and len(published)==1 and node.last_body_stamp == 100.09


def test_cached_body_is_not_authority_after_sensor_barrier_advances():
    node, _ = lease_harness()
    node.current_context = lambda now,**kwargs:LiveScanBridge.current_context(node,now,**kwargs)
    node.body = Odometry()
    node.body.header.stamp.sec = 100
    node.body.header.stamp.nanosec = 50_000_000
    node.body_context = ('test',4,'seed')
    node.body_received = node.freeze_received = 50.
    node.execution_frozen = True
    node.sensor_barrier = 100.04
    assert LiveScanBridge.nav_body_ready(node,100.1,50.02)
    node.sensor_barrier = 100.06
    assert not LiveScanBridge.nav_body_ready(node,100.1,50.02)


def path(stamp, *, empty=False):
    message = Path()
    message.header.frame_id = 'd1max_loc_map'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    if not empty:
        for x in (0., .5, 1.):
            pose = PoseStamped()
            pose.header.frame_id = 'd1max_loc_map'
            pose.pose.position.x = x
            message.poses.append(pose)
    return message


def harness():
    gate = ReferenceGate()
    gate.observe(True, 100., ('test', 1, 'seed'))
    assert gate.accept([[0, 0, 0], [.5, 0, 0], [1, 0, 0]],
                       frame_id='d1max_loc_map', stamp=100.1, now=100.1,
                       body_xyz=[0, 0, .55])
    output = SimpleNamespace(
        gate=gate, p={'map_frame': 'd1max_loc_map', 'session_id': 'test'},
        reference_seen_stamp=100.1, reference_refresh={'current': True},
        body=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0., z=.55)))),
        counts={'references': 0, 'rejected_references': 0}, error='',
        reference_pub=SimpleNamespace(publish=lambda msg: published.append(msg)),
        now_s=lambda: 100.4, update_gate=lambda: None,
        clear_outputs=lambda reason, clear_sensors: cleared.append(reason),
        clear_marker=lambda: None, clear_attempt_debug=lambda: None,
        arm_reference_refresh=lambda *args: None)
    published, cleared = [], []
    return output, published, cleared


def test_stale_and_equal_cancel_cannot_clear_new_active_path():
    bridge, published, cleared = harness()
    for stamp in (100.05, 100.1):
        LiveScanBridge.on_reference(bridge, path(stamp, empty=True))
    assert bridge.gate.active and bridge.gate.generation == 1
    assert not cleared and not published and bridge.reference_refresh


def test_new_cancel_then_strictly_new_path_preserves_handover():
    bridge, published, cleared = harness()
    LiveScanBridge.on_reference(bridge, path(100.2, empty=True))
    assert not bridge.gate.active and bridge.gate.generation == 2
    assert cleared == ['explicit_reference_cancel']
    LiveScanBridge.on_reference(bridge, path(100.2))
    LiveScanBridge.on_reference(bridge, path(100.15, empty=True))
    assert not bridge.gate.active and bridge.gate.generation == 2
    assert len(cleared) == 1 and not published
    LiveScanBridge.on_reference(bridge, path(100.3))
    assert bridge.gate.active and bridge.gate.generation == 3
    assert bridge.reference_seen_stamp == 100.3
    assert len(published) == 1 and published[0].generation == 3


def test_wrong_frame_future_and_old_cancel_are_not_ordering_authority():
    assert not new_reference_message(100.2, 100.4, 100.1, 100.1,
                                     'wrong_frame', 'd1max_loc_map')
    assert not new_reference_message(100.6, 100.4, 100.1, 100.1,
                                     'd1max_loc_map', 'd1max_loc_map')
    assert not new_reference_message(98., 100.4, 100.1, 100.1,
                                     'd1max_loc_map', 'd1max_loc_map')


def spline(identifier, generation=1):
    message = TaggedBspline()
    message.session_id, message.generation = 'test', generation
    message.frame_id = 'd1max_loc_map'
    message.trajectory.traj_id = identifier
    message.trajectory.start_time.sec = 100
    message.trajectory.start_time.nanosec = 200000000
    return message


def spline_harness(monkeypatch):
    monkeypatch.setattr(live_scan_bridge, 'sample_shadow_spline',
                        lambda **_kwargs: np.array([[0., 0., .55], [.05, 0., .55]]))
    gate = ReferenceGate()
    gate.observe(True, 100., ('test', 1, 'seed'))
    gate.accept([[0, 0, 0], [.5, 0, 0], [1, 0, 0]], frame_id='d1max_loc_map',
                stamp=100.1, now=100.1, body_xyz=[0, 0, .55])
    support = SimpleNamespace(fail=False)

    def check(_samples, **_kwargs):
        if support.fail:
            raise GroundSupportError('pct_support_hole_or_blocked_cell')
        return {'checked_support_cells': 2, 'motion_authorized': False}

    support.validate_body_samples = check
    emitted = []
    bridge = SimpleNamespace(
        gate=gate, p={'session_id': 'test', 'map_frame': 'd1max_loc_map', 'body_height': .55,
                      'execution_mode': 'preview',
                      'ground_support_height_tolerance_m': .2,
                      'ground_support_max_step_m': .17},
        ground_support=support, ground_support_check={}, debug_gate=LocalDebugGate(),
        last_spline_id=-1, last_spline_stamp=0., pending_spline_marker=None,
        pending_spline_message=None, admitted_spline_key=None, admitted_record=None,
        admission_reason='startup', admission_sequence=0,
        spline_received=float('-inf'), visible_spline_id=-1,
        counts={'rejected_splines': 0, 'marker_published': 0,
                'debug_published': 0, 'debug_rejected': 0, 'debug_cleared': 0}, error='',
        now_s=lambda: 100.4, update_gate=lambda: None,
        marker_pub=SimpleNamespace(publish=lambda msg: emitted.append(msg)),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: path(100.4).header.stamp)),
        publish_debug_delete=lambda: None)
    bridge.admissions, bridge.validated = [], []
    bridge.admission_pub = SimpleNamespace(publish=lambda msg: bridge.admissions.append(json.loads(msg.data)))
    bridge.validated_spline_pub = SimpleNamespace(publish=bridge.validated.append)
    for method in ('revoke_execution', 'publish_execution_admission', 'admit_execution',
                   'execution_pair_valid', 'execution_admission_valid', 'publish_debug'):
        setattr(bridge, method, lambda *args, _method=method, **kwargs:
                getattr(LiveScanBridge, _method)(bridge, *args, **kwargs))
    bridge.delete_spline_marker = lambda: LiveScanBridge.delete_spline_marker(bridge)
    bridge.clear_debug = lambda phase='inactive': LiveScanBridge.clear_debug(bridge, phase)
    bridge.clear_marker = lambda **kwargs: LiveScanBridge.clear_marker(bridge, **kwargs)
    bridge.debug_pub = SimpleNamespace(publish=lambda msg: emitted.append(msg))
    return bridge, support, emitted


def test_support_reject_consumes_bad_id_but_new_valid_native_candidate_recovers(monkeypatch):
    bridge, support, emitted = spline_harness(monkeypatch)
    LiveScanBridge.on_spline(bridge, spline(1))
    assert bridge.ground_support_check['valid'] is True
    assert bridge.pending_spline_marker is not None and bridge.last_spline_id == 1
    support.fail = True
    LiveScanBridge.on_spline(bridge, spline(2))
    assert bridge.ground_support_check['reason'] == 'pct_support_hole_or_blocked_cell'
    assert bridge.pending_spline_marker is None and bridge.last_spline_id == 2
    assert bridge.visible_spline_id == -1 and bridge.counts['rejected_splines'] == 1
    LiveScanBridge.on_spline(bridge, spline(2))
    assert bridge.counts['rejected_splines'] == 2 and bridge.last_spline_id == 2
    support.fail = False
    LiveScanBridge.on_spline(bridge, spline(3))
    assert bridge.ground_support_check['valid'] is True
    assert bridge.ground_support_check['plan_id'] == 3
    assert bridge.pending_spline_marker is not None and bridge.last_spline_id == 3
    assert emitted and all(msg.action != msg.ADD for msg in emitted if hasattr(msg, 'action'))


def accepted_debug(identifier, *, generation=1, valid=True, stamp=100.3):
    message = LocalPlanDebug()
    message.header = path(stamp).header
    message.session_id, message.generation, message.plan_id = 'test', generation, identifier
    message.valid, message.phase = valid, 'accepted' if valid else 'failed_optimization'
    message.selected_reference = path(stamp)
    for pose in message.selected_reference.poses:
        pose.header = message.header
        pose.pose.position.z = .55
    message.projection.z = message.local_target.z = .55
    message.local_target.x, message.target_arc_m = 1., 1.
    return message


def test_preview_never_forwards_execution_spline_even_when_geometry_is_accepted(monkeypatch):
    bridge, _, _ = spline_harness(monkeypatch)
    LiveScanBridge.on_spline(bridge, spline(1))
    LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
    assert not bridge.validated
    assert bridge.admissions and not bridge.admissions[-1]['valid']


def test_execution_pair_handles_both_delivery_orders_and_forwards_unchanged_once(monkeypatch):
    for debug_first in (False, True):
        bridge, _, _ = spline_harness(monkeypatch)
        bridge.p['execution_mode'] = 'execution'
        candidate = spline(1)
        if debug_first:
            LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
        LiveScanBridge.on_spline(bridge, candidate)
        if not debug_first:
            assert not bridge.validated
            LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
        assert bridge.validated == [candidate]
        assert bridge.validated[0] is candidate
        status = bridge.admissions[-1]
        assert status['valid'] and status['generation'] == 1 and status['trajectory_id'] == 1
        assert status['source_stamp'] == 100.2 and status['issued_at'] == 100.1
        assert status['lease_timeout_sec'] == .35 and status['motion_authorized'] is False
        LiveScanBridge.on_spline(bridge, candidate)
        LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
        bridge.publish_debug()
        bridge.publish_execution_admission()
        assert bridge.validated == [candidate]


def test_execution_rejection_revokes_without_waiting_for_marker_or_task_timeout(monkeypatch):
    bridge, support, _ = spline_harness(monkeypatch)
    bridge.p['execution_mode'] = 'execution'
    LiveScanBridge.on_spline(bridge, spline(1))
    LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
    assert bridge.admissions[-1]['valid']
    support.fail = True
    LiveScanBridge.on_spline(bridge, spline(2))
    assert not bridge.admissions[-1]['valid']
    assert bridge.pending_spline_message is None
    assert len(bridge.validated) == 1
    LiveScanBridge.on_local_debug(bridge, accepted_debug(2, stamp=100.35))
    assert not bridge.admissions[-1]['valid'] and len(bridge.validated) == 1


def test_execution_debug_failure_context_change_and_source_expiry_revoke(monkeypatch):
    for fault in ('debug', 'context', 'source_time', 'receipt_time'):
        bridge, _, _ = spline_harness(monkeypatch)
        bridge.p['execution_mode'] = 'execution'
        LiveScanBridge.on_spline(bridge, spline(1))
        LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
        assert bridge.admissions[-1]['valid']
        if fault == 'debug':
            LiveScanBridge.on_local_debug(bridge, accepted_debug(1, valid=False, stamp=100.35))
        elif fault == 'context':
            bridge.gate.revoke(100.5, 'cancelled')
        elif fault == 'source_time':
            bridge.now_s = lambda: 103.
        else:
            bridge.admitted_record['spline_received'] -= 3.
        bridge.publish_execution_admission()
        assert not bridge.admissions[-1]['valid']
        assert bridge.admitted_spline_key is None


def test_execution_requires_one_named_fresh_freeze_publisher_but_allows_false():
    node, _ = lease_harness()
    node.p.update(execution_mode='execution', execution_tracker_node='/motion/coordinator')
    node.current_context = lambda now, **kw: ('test', 4, 'seed')
    node.body = Odometry()
    node.body.header.stamp = path(100.05).header.stamp
    node.body_context, node.body_received = ('test', 4, 'seed'), 50.
    node.sensor_barrier, node.freeze_received = 100.04, 50.
    node.execution_frozen = False
    publisher = SimpleNamespace(node_namespace='/motion', node_name='coordinator')
    node.get_publishers_info_by_topic = lambda _: [publisher]
    assert LiveScanBridge.nav_body_ready(node, 100.1, 50.02)
    node.get_publishers_info_by_topic = lambda _: [publisher, publisher]
    assert not LiveScanBridge.nav_body_ready(node, 100.1, 50.02)
    node.get_publishers_info_by_topic = lambda _: [SimpleNamespace(node_namespace='/', node_name='preview')]
    assert not LiveScanBridge.nav_body_ready(node, 100.1, 50.02)
    node.get_publishers_info_by_topic = lambda _: [publisher]
    assert not LiveScanBridge.nav_body_ready(node, 100.1, 50.6)
    node.p['execution_mode'] = 'preview'
    assert not LiveScanBridge.nav_body_ready(node, 100.1, 50.02)
    node.execution_frozen = True
    assert LiveScanBridge.nav_body_ready(node, 100.1, 50.02)


def test_normal_replanning_keeps_previous_admission_until_new_pair_arrives(monkeypatch):
    for debug_first in (False, True):
        bridge, _, _ = spline_harness(monkeypatch)
        bridge.p['execution_mode'] = 'execution'
        LiveScanBridge.on_spline(bridge, spline(1))
        LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
        split = len(bridge.admissions)
        candidate = spline(2)
        candidate.trajectory.start_time.nanosec = 250_000_000
        if debug_first:
            LiveScanBridge.on_local_debug(bridge, accepted_debug(2, stamp=100.35))
        else:
            LiveScanBridge.on_spline(bridge, candidate)
        assert bridge.admissions[-1]['valid']
        assert bridge.admissions[-1]['trajectory_id'] == 1
        assert bridge.admissions[-1]['handover_pending']
        assert bridge.admissions[-1]['pending_candidate_id'] == 2
        if debug_first:
            LiveScanBridge.on_spline(bridge, candidate)
        else:
            LiveScanBridge.on_local_debug(bridge, accepted_debug(2, stamp=100.35))
        assert bridge.admissions[-1]['valid']
        assert bridge.admissions[-1]['trajectory_id'] == 2
        assert not bridge.admissions[-1]['handover_pending']
        assert all(s['valid'] for s in bridge.admissions[split:])
        assert [m.trajectory.traj_id for m in bridge.validated] == [1, 2]


def test_failure_of_still_executing_plan_revokes_during_new_debug_only_handover(monkeypatch):
    bridge, _, _ = spline_harness(monkeypatch)
    bridge.p['execution_mode'] = 'execution'
    LiveScanBridge.on_spline(bridge, spline(1))
    LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
    LiveScanBridge.on_local_debug(bridge, accepted_debug(2, stamp=100.32))
    assert bridge.admissions[-1]['valid'] and bridge.admissions[-1]['trajectory_id'] == 1
    assert bridge.admissions[-1]['handover_pending']
    LiveScanBridge.on_local_debug(bridge, accepted_debug(1, valid=False, stamp=100.35))
    assert not bridge.admissions[-1]['valid']
    assert bridge.admissions[-1]['reason'] == 'native_failed_optimization'
    assert not bridge.admissions[-1]['handover_pending']


def test_predecessor_proof_is_bound_to_new_admission_and_stamp_is_not_refreshed(monkeypatch):
    bridge, _, _ = spline_harness(monkeypatch)
    bridge.p['execution_mode'] = 'execution'
    LiveScanBridge.on_spline(bridge, spline(1))
    LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
    assert not bridge.admissions[-1]['predecessor_safe']
    new_debug=accepted_debug(2,stamp=100.35)
    new_debug.predecessor_id=1
    new_debug.predecessor_safe=True
    new_debug.predecessor_check_stamp=path(100.34).header.stamp
    LiveScanBridge.on_local_debug(bridge,new_debug)
    # Until the exact new spline/ground support pairs, no new authority exists.
    assert bridge.admissions[-1]['trajectory_id']==1
    assert bridge.admissions[-1]['handover_pending']
    LiveScanBridge.on_spline(bridge,spline(2))
    first=bridge.admissions[-1]
    assert first['trajectory_id']==2 and first['predecessor_id']==1
    assert not first['handover_pending']
    assert first['predecessor_safe'] and first['predecessor_check_stamp']==100.34
    bridge.now_s=lambda:100.7
    bridge.publish_execution_admission()
    assert bridge.admissions[-1]['predecessor_check_stamp']==100.34
    assert bridge.admitted_record['debug'].predecessor_check_stamp==100.34


@pytest.mark.parametrize('predecessor_id, stamp', [(2,100.34),(3,100.34),(1,100.09),(1,100.36)])
def test_predecessor_proof_rejects_wrong_id_or_time_without_rejecting_new_plan(
        monkeypatch, predecessor_id, stamp):
    bridge, _, _ = spline_harness(monkeypatch)
    bridge.p['execution_mode']='execution'
    message=accepted_debug(2,stamp=100.35)
    message.predecessor_id=predecessor_id;message.predecessor_safe=True
    message.predecessor_check_stamp=path(stamp).header.stamp
    LiveScanBridge.on_spline(bridge,spline(2))
    LiveScanBridge.on_local_debug(bridge,message)
    assert bridge.admissions[-1]['valid'] and not bridge.admissions[-1]['predecessor_safe']


@pytest.mark.parametrize('phase, reason', [
    ('failed_optimization', 'native_failed_optimization'),
    ('cancelled', 'native_cancelled'), ('unknown_phase', 'invalid_debug')])
def test_native_failure_reason_survives_visual_clear_and_heartbeat(monkeypatch, phase, reason):
    bridge, _, _ = spline_harness(monkeypatch)
    bridge.p['execution_mode'] = 'execution'
    LiveScanBridge.on_spline(bridge, spline(1))
    LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
    message = accepted_debug(1, valid=False, stamp=100.35)
    message.phase = phase
    split = len(bridge.admissions)
    LiveScanBridge.on_local_debug(bridge, message)
    bridge.publish_execution_admission()
    assert bridge.visible_spline_id == -1 and bridge.pending_spline_message is None
    assert all(not item['valid'] and item['reason'] == reason for item in bridge.admissions[split:])


@pytest.mark.parametrize('measured_arrival', [True, False])
def test_actual_completed_callback_final_admission_drives_measured_completion_grace(
        monkeypatch, measured_arrival):
    """Use the bridge's emitted contract, not a hand-written completed payload."""
    clock = [100.4]
    monkeypatch.setattr(live_scan_bridge.time, 'time', lambda: clock[0])
    monkeypatch.setattr(live_scan_bridge.time, 'monotonic', lambda: clock[0])
    bridge, _, emitted = spline_harness(monkeypatch)
    bridge.now_s = lambda: clock[0]
    bridge.p['execution_mode'] = 'execution'
    LiveScanBridge.on_spline(bridge, spline(1))
    LiveScanBridge.on_local_debug(bridge, accepted_debug(1))
    core = ExecutionLease(MotionConfig('test', 'map', body_height_calibrated=True,
        collision_envelope_validated=True, vertical_envelope_validated=True,
        sdk_speed_mapping_validated=True))
    assert core.reference(session='test', generation=1, frame='d1max_loc_map',
        stamp=100.1, points=[(0., 0., 0.), (1., 0., 0.)], now=clock[0])
    assert core.observe('admission', bridge.admissions[-1], clock[0], clock[0])
    assert core.begin(clock[0], clock[0])[0]

    def gate():
        assert core.observe('gate', dict(navigation_session_id='test', map_version_id='map',
            wall_time=clock[0], armed=True), clock[0], clock[0])

    def tracker(**changes):
        value = dict(session_id='test', generation=1, trajectory_id=2,
            frame_id='d1max_loc_map', stamp=clock[0], active=True, finished=False,
            reason='tracking', command=dict(x=.2, y=0., yaw=.1), execution_frozen=False)
        assert core.observe('tracker', dict(value, **changes), clock[0], clock[0])

    clock[0] = 100.45
    gate()
    assert core.step(clock[0], clock[0]) == ((0., 0., 0.), True)
    clock[0] = 100.55
    candidate = spline(2)
    candidate.trajectory.start_time.nanosec = 500_000_000
    LiveScanBridge.on_spline(bridge, candidate)
    LiveScanBridge.on_local_debug(bridge, accepted_debug(2, stamp=100.52))
    assert core.observe('admission', bridge.admissions[-1], clock[0], clock[0])
    gate(); tracker()
    assert core.step(clock[0], clock[0]) == ((.2, 0., .1), False)

    clock[0] = 100.6
    completed = accepted_debug(2, valid=False, stamp=clock[0])
    completed.phase = 'completed'
    split = len(bridge.admissions)
    LiveScanBridge.on_local_debug(bridge, completed)
    assert bridge.visible_spline_id == -1 and bridge.pending_spline_message is None
    assert emitted[-1].action == emitted[-1].DELETEALL
    # Heartbeats retain the final semantic outcome even if depth-one delivery
    # drops every publication made during the native callback.
    clock[0] = 100.61
    bridge.publish_execution_admission()
    assert all(not item['valid'] and item['reason'] == 'native_completed'
               for item in bridge.admissions[split:])
    assert core.observe('admission', bridge.admissions[-1], clock[0], clock[0])
    assert core.step(clock[0], clock[0]) == ((0., 0., 0.), True)
    assert core.phase == 'verifying_completion' and core.task()['active']
    assert core.disarm_requested and not core.permit(clock[0], clock[0])['allow']
    if measured_arrival:
        clock[0] = 100.65
        tracker(active=False, finished=True, reason='goal_reached')
        core.step(clock[0], clock[0])
        assert core.phase == 'finished' and core.reason == 'goal_reached'
    else:
        clock[0] = 100.87
        tracker()
        core.step(clock[0], clock[0])
        assert core.phase == 'locked'
        assert core.reason == 'native_completed_without_measured_goal_confirmation'
    assert not core.task()['active'] and not core.permit(clock[0], clock[0])['allow']


def test_old_generation_spline_never_revives_after_task_handover(monkeypatch):
    bridge, _, _ = spline_harness(monkeypatch)
    LiveScanBridge.on_spline(bridge, spline(1))
    bridge.gate.revoke(100.25, 'new_goal', advance_barrier=False)
    bridge.gate.accept([[0, 0, 0], [.5, 0, 0], [1, 0, 0]],
        frame_id='d1max_loc_map', stamp=100.3, now=100.3,
        body_xyz=[0, 0, .55])
    before = bridge.ground_support_check.copy()
    LiveScanBridge.on_spline(bridge, spline(2, generation=1))
    assert bridge.counts['rejected_splines'] == 1
    assert bridge.ground_support_check == before
