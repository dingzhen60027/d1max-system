from copy import deepcopy
import pytest
from d1max_navigation_bt_interfaces.msg import RouteReference
from d1max_planning_interfaces.msg import ReferencePath, TaggedBspline
from d1max_pct_scan.route_ingress import (RouteIngress, annotate_native_reference,
                                        spline_matches_route)
from test_bt_action_adapters import candidate


def delivery(sequence=1, active=True):
    wire = candidate()
    wire.task_id, wire.route_id = 't', 'r'
    message = RouteReference()
    message.schema_version, message.session_id = 2, wire.session_id
    message.task_id, message.route_id, message.route_hash = 't', 'r', wire.route_hash
    message.delivery_sequence, message.active = sequence, active
    message.snapshot = wire
    message.source_stamp.sec, message.source_stamp.nanosec = 10, sequence*10000000
    message.snapshot.path.header.stamp = deepcopy(message.source_stamp)
    return message


def ingress():
    m = delivery()
    return RouteIngress(m.session_id, m.snapshot.map_version_id)


def accept(gate, m):
    return gate.accept(m, now=10.1,
        context=(m.session_id, m.snapshot.localization_epoch, m.snapshot.localization_seed_id))


def test_repeated_source_route_keeps_hash_duplicate_packet_ignored():
    gate, m = ingress(), delivery()
    assert accept(gate, m).route_hash == m.route_hash
    assert accept(gate, m) is None
    assert accept(gate, delivery(2)).route_hash == m.route_hash


def test_cancel_requires_exact_active_owner_no_old_task_resurrection():
    gate = ingress()
    accept(gate, delivery())
    bad = delivery(2, False); bad.task_id = bad.snapshot.task_id = 'old'
    assert accept(gate, bad) is None
    assert gate.active_identity[0] == 't'
    accept(gate, delivery(3, False))
    assert gate.active_identity is None
    with pytest.raises(ValueError, match='retired'):
        accept(gate, delivery(4))


@pytest.mark.parametrize('field,value', [('schema_version',1), ('session_id','wrong'),
    ('route_hash','bad'), ('task_id','foreign')])
def test_foreign_or_v1_envelope_cannot_feed_native_path(field,value):
    m = delivery(); setattr(m,field,value)
    with pytest.raises(ValueError):
        accept(ingress(),m)


def test_epoch_change_and_unretired_replacement_rejected():
    gate, m = ingress(), delivery()
    with pytest.raises(ValueError,match='context'):
        gate.accept(m,now=10.1,context=(m.session_id,999,'new'))
    accept(gate,m)
    new = delivery(2); new.task_id = new.snapshot.task_id = 'new'
    with pytest.raises(ValueError,match='not_retired'):
        accept(gate,new)


def test_native_reference_carries_segments_and_cancel_carries_identity():
    source = delivery().snapshot
    message = ReferencePath(); message.path = source.path
    annotate_native_reference(message,source,context_sequence=8)
    assert len(message.point_segment_ids) == len(message.path.poses)
    assert message.schema_version == 2 and message.route_hash == source.route_hash
    assert message.anchor_id == ''  # map preview never claims an odom anchor
    assert message.point_reference == 'ground'
    message.path.poses = []
    annotate_native_reference(message,source,context_sequence=8)
    assert message.route_hash == source.route_hash


def test_foreign_native_curve_cannot_be_committed():
    source = delivery().snapshot
    m = matching_spline(source)
    assert spline_matches_route(m,source,context_sequence=8)
    m.route_hash = 'other'
    assert not spline_matches_route(m,source,context_sequence=8)


def matching_spline(source):
    m = TaggedBspline()
    m.schema_version, m.context_sequence, m.map_geometry_revision = 2, 8, 8
    m.frame_id, m.point_reference = 'd1max_loc_map', 'body_center'
    for name in ('session_id','task_id','route_id','route_hash','map_version_id',
                 'localization_epoch','localization_seed_id'):
        setattr(m,name,getattr(source,name))
    part = source.segments[0]
    m.segment_id, m.segment_kind, m.required_mode = part.segment_id, part.kind, part.required_mode
    return m


@pytest.mark.parametrize('field,value', [('point_reference','ground'), ('point_reference',''),
    ('segment_kind','stair_down'), ('required_mode','unverified'), ('session_id','foreign'),
    ('frame_id','d1max_loc_odom'), ('anchor_id','invented'), ('anchor_revision',1),
    ('map_geometry_revision',9)])
def test_wrong_body_reference_segment_mode_or_fake_anchor_is_not_a_matching_curve(field, value):
    source = delivery().snapshot
    m = matching_spline(source)
    setattr(m,field,value)
    assert not spline_matches_route(m,source,context_sequence=8)


def test_odom_profile_requires_exact_supplied_admitted_anchor_not_a_nonempty_string():
    from d1max_pct_scan.control_frame_contract import create_anchor
    from test_continuous_reference import observation
    source = delivery().snapshot
    anchor = create_anchor(observation(frame='d1max_loc_map').body,observation().body,2)
    m = matching_spline(source)
    m.frame_id, m.anchor_id = 'd1max_loc_odom', anchor.anchor_id
    m.anchor_revision = m.map_geometry_revision = anchor.revision
    assert not spline_matches_route(m,source,context_sequence=8)
    assert spline_matches_route(m,source,context_sequence=8,anchor=anchor)
    m.anchor_revision += 1
    assert not spline_matches_route(m,source,context_sequence=8,anchor=anchor)


@pytest.mark.parametrize('delta_ns', [0, 1, 50])
def test_higher_sequence_cancel_survives_paused_clock_and_unix_float_collision(delta_ns):
    gate, first = ingress(), delivery()
    first.source_stamp.sec, first.source_stamp.nanosec = 1800000000, 123400000
    first.snapshot.path.header.stamp = deepcopy(first.source_stamp)
    now = 1800000000.1234
    assert gate.accept(first, now=now, context=('session',1,'seed'))
    cancel = deepcopy(first)
    cancel.active, cancel.delivery_sequence = False, 2
    cancel.source_stamp.nanosec += delta_ns
    cancel.snapshot.path.header.stamp = deepcopy(cancel.source_stamp)
    assert gate.accept(cancel, now=now, context=None)
    assert gate.active_identity is None and gate.last_stamp_ns == 1800000000123400000+delta_ns


def test_typed_bridge_cancel_and_new_owner_at_identical_tick_keep_old_cancel_fenced():
    from test_live_scan_reference_order import harness
    from d1max_pct_scan.live_scan_bridge import LiveScanBridge
    bridge, published, cleared = harness()
    bridge.p['session_id'] = 'session'
    bridge.route_ingress = ingress()
    bridge.map_context_sequence = 1
    bridge.current_context = lambda now: ('session',1,'seed')
    bridge.now_s = lambda: 1800000000.1234
    bridge.on_reference = lambda message, **kwargs: LiveScanBridge.on_reference(bridge,message,**kwargs)
    def typed(sequence, active, task='t'):
        message = delivery(sequence, active)
        message.task_id = message.snapshot.task_id = task
        message.source_stamp.sec, message.source_stamp.nanosec = 1800000000, 123400000
        message.snapshot.path.header.stamp = deepcopy(message.source_stamp)
        return message
    LiveScanBridge.on_committed_route(bridge,typed(1,True))
    assert bridge.gate.active and len(published) == 1
    stamp = deepcopy(published[-1].path.header.stamp)
    LiveScanBridge.on_committed_route(bridge,typed(2,False))
    assert not bridge.gate.active and cleared == ['explicit_reference_cancel']
    LiveScanBridge.on_committed_route(bridge,typed(3,True,'new'))
    assert bridge.gate.active and len(published) == 2
    assert published[-1].path.header.stamp == stamp  # no fabricated nextafter timestamp
    generation = bridge.gate.generation
    LiveScanBridge.on_committed_route(bridge,typed(4,False,'t'))
    LiveScanBridge.on_committed_route(bridge,typed(2,False,'new'))
    assert bridge.gate.active and bridge.gate.generation == generation
    assert bridge.route_ingress.active_identity[0] == 'new'
    assert cleared == ['explicit_reference_cancel']


def test_typed_sequence_never_renews_stale_delivery_source_time():
    gate, first = ingress(), delivery()
    assert accept(gate, first)
    cancel = delivery(2,False)
    with pytest.raises(ValueError, match='delivery_invalid'):
        gate.accept(cancel,now=13.,context=None)
    assert gate.active_identity is not None
