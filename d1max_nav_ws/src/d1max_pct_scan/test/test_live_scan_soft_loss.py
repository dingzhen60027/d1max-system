"""Real bridge callbacks, ROS message types only: no node, graph, or robot IO.

Transient pose loss revokes the path lease, not the unchanged native map epoch.
These regressions exercise the same update/clear/receipt methods as production.
"""
from collections import deque
import json
import math
from types import MethodType, SimpleNamespace as NS

import pytest
import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from d1max_pct_scan import live_scan_bridge
from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from d1max_pct_scan.live_scan_contract import ReferenceGate
from d1max_pct_scan.pct_ground_support import PCTGroundSupport


IDENTITY = ('session', 1, 'seed')


def support_index(ground_z=0., *, missing=(), levels=None):
    points = [(x*.1, y*.1, z) for x in range(3) for y in range(3)
              if (x, y) not in missing for z in (levels or [ground_z])]
    return PCTGroundSupport.from_arrays(np.asarray(points),
        dict(frame_id='d1max_loc_map', resolution=.1,
             source_pcd_sha256='a'*64, source_tomogram_sha256='b'*64))


def body_message(stamp=100.16, z=.55):
    message = Odometry()
    message.header.frame_id = 'd1max_loc_map'
    message.child_frame_id = 'd1max_loc_base_link'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    message.pose.pose.position.x = message.pose.pose.position.y = .1
    message.pose.pose.position.z = z
    message.pose.pose.orientation.w = 1.
    return message


def harness(monkeypatch, backend='per_sensor_rays'):
    clock = NS(wall=100.25, mono=10.25, context=IDENTITY)
    monkeypatch.setattr(live_scan_bridge.time, 'monotonic', lambda: clock.mono)
    paths, clears, contexts = [], [], []
    gate = ReferenceGate()
    gate.observe(True, 100., IDENTITY)
    gate.accept([[0, 0, 0], [1, 0, 0]], frame_id=gate.frame_id,
                stamp=100.12, now=100.12, body_xyz=[0, 0, .55])
    node = NS(p=dict(session_id='session', map_frame=gate.frame_id,
                     body_frame='d1max_loc_base_link', body_height=.55,
                     ground_support_height_tolerance_m=.2, ground_support_max_step_m=.17,
                     input_timeout=.5, perception_backend=backend),
              gate=gate, now_s=lambda: clock.wall,
              current_context=lambda *args, **kw: clock.context,
              map_context_identity=IDENTITY, map_context_sequence=1,
              map_context_ready=True, map_context_fault=None,
              map_context_record=dict(schema=1, session_id='session', epoch=1,
                                      seed_id='seed', sequence=1, barrier_ns=99_000_000_000),
              map_context_last_publish=10.,
              map_context_pub=NS(publish=contexts.append),
              reference_refresh=None, reference_refresh_last=-math.inf,
              sensor_barrier=99., cloud_stamp=100.10, cloud_received=10.15,
              last_input_stamp=100.16, last_output_stamp=100.10,
              ray_status_stamp=100.15, ray_integration={}, error='',
              pending=deque([object()]), ground_support_check={'valid': True},
              body=body_message(), body_context=IDENTITY, last_body_stamp=100.16,
              ground_support=support_index(),
              reference_pub=NS(publish=paths.append), cloud_pub=NS(publish=lambda _: None),
              cloud_message=lambda *args: None, drop_clouds=lambda *args: None,
              clear_marker=lambda **kwargs: clears.append('marker'),
              clear_attempt_debug=lambda: clears.append('attempt'),
              revoke_execution=lambda reason: clears.append(('admission', reason)),
              get_logger=lambda: NS(info=lambda _: None),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(
                  sec=int(clock.wall), nanosec=round((clock.wall % 1)*1e9)))))
    node.nav_body_ready = lambda now, mono: (
        clock.context is not None and node.map_context_ready and node.map_context_fault is None)
    for name in ('update_gate', 'current_body_ground_support_ready',
                 'sync_map_context', 'clear_outputs'):
        setattr(node, name, MethodType(getattr(LiveScanBridge, name), node))
    return node, clock, paths, clears, contexts


def lose_pose(node, clock):
    clock.context = None
    node.update_gate()


def receipt(node, clock, source=100.20, **changes):
    value = dict(node.map_context_record, received_at_unix=clock.wall,
                 valid=True, source_stamp_ns=round(source*1e9))
    value.update(changes)
    LiveScanBridge.on_native_rays(node, String(data=json.dumps(value)))


def test_short_pose_loss_revokes_path_but_not_same_context_ray_evidence(monkeypatch):
    node, clock, paths, clears, _ = harness(monkeypatch)
    old_generation = node.gate.generation
    lose_pose(node, clock)
    assert not node.gate.active and not node.gate.ready
    assert node.gate.generation == old_generation+1
    assert len(paths) == 1 and not paths[0].path.poses
    assert ('admission', 'input_stale_or_invalid') in clears
    assert node.sensor_barrier == 99.
    assert node.cloud_stamp == 100.10 and node.cloud_received == 10.15
    assert node.map_context_sequence == 1 and node.map_context_ready
    assert node.reference_refresh['source_stamp'] == 100.12
    # Repeated invalid ticks do not create new cancellation generations.
    node.update_gate()
    assert len(paths) == 1 and node.gate.generation == old_generation+1


def test_fresh_pre_pause_observation_can_recover_but_old_path_cannot(monkeypatch):
    node, clock, _, _, _ = harness(monkeypatch)
    lose_pose(node, clock)
    clock.wall, clock.mono, clock.context = 100.32, 10.32, IDENTITY
    # This is a real completed integration acquired before the 100.25 pause.
    # Its original source age is 120ms, not a receipt-time freshness substitute.
    receipt(node, clock)
    assert node.cloud_stamp == 100.20
    assert node.gate.ready and not node.gate.active
    assert node.gate.barrier == clock.wall  # task barrier still advances!
    with pytest.raises(ValueError, match='obsolete_or_stale_reference'):
        node.gate.accept([[0, 0, 0], [1, 0, 0]], frame_id=node.gate.frame_id,
                         stamp=100.12, now=clock.wall, body_xyz=[0, 0, .55])
    assert node.gate.accept([[0, 0, 0], [1, 0, 0]], frame_id=node.gate.frame_id,
                            stamp=100.33, now=100.33, body_xyz=[0, 0, .55])


def test_expired_rays_and_heartbeats_still_cannot_renew_perception(monkeypatch):
    node, clock, _, _, _ = harness(monkeypatch)
    lose_pose(node, clock)
    clock.wall, clock.mono, clock.context = 100.8, 10.8, IDENTITY
    receipt(node, clock, source=100.20)
    assert node.error == 'ray_measurement_stale_or_before_barrier'
    assert node.cloud_stamp == 100.10 and node.cloud_received == 10.15
    assert not node.gate.ready and not node.gate.active


@pytest.mark.parametrize('changes', [dict(valid=False), dict(seed_id='old'),
                                    dict(sequence=0), dict(epoch=0)])
def test_invalid_or_foreign_receipt_does_not_restore_perception(monkeypatch, changes):
    node, clock, _, _, _ = harness(monkeypatch)
    lose_pose(node, clock)
    clock.wall, clock.mono, clock.context = 100.7, 10.7, IDENTITY
    receipt(node, clock, source=100.6, **changes)
    assert node.cloud_stamp == 100.10 and node.cloud_received == 10.15
    assert not node.gate.ready and not node.gate.active


def test_actual_epoch_change_still_advances_sensor_barrier_and_requires_ack(monkeypatch):
    node, clock, _, _, contexts = harness(monkeypatch)
    lose_pose(node, clock)
    clock.wall, clock.mono, clock.context = 100.32, 10.32, ('session', 2, 'new-seed')
    node.update_gate()
    assert node.sensor_barrier >= clock.wall
    assert node.cloud_stamp == 0. and not node.pending
    assert not node.map_context_ready and node.map_context_sequence == 2
    assert len(contexts) == 1
    assert json.loads(contexts[0].data)['epoch'] == 2
    receipt(node, clock, source=100.20)
    assert node.cloud_stamp == 0. and not node.gate.active


def test_legacy_cloud_backend_retains_existing_invalidation(monkeypatch):
    node, clock, _, _, _ = harness(monkeypatch, 'deskewed_cloud')
    lose_pose(node, clock)
    assert node.sensor_barrier >= clock.wall
    assert node.cloud_stamp == 0. and not node.pending


def test_latched_geometry_fault_is_not_a_soft_recovery(monkeypatch):
    node, clock, _, _, _ = harness(monkeypatch)
    node.map_context_fault = 'map_alignment_accumulation_requires_context_reset'
    lose_pose(node, clock)
    assert node.sensor_barrier >= clock.wall and node.cloud_stamp == 0.
    clock.context = IDENTITY
    receipt(node, clock)
    assert not node.gate.ready and not node.gate.active and node.cloud_stamp == 0.


@pytest.mark.parametrize('backend', ['per_sensor_rays', 'deskewed_cloud'])
def test_lowered_body_revokes_height_contract_without_invalidating_sensors(monkeypatch, backend):
    node, clock, paths, clears, _ = harness(monkeypatch, backend)
    node.ground_support = support_index(-.525)
    node.body = body_message(z=.025)
    node.update_gate()
    assert node.gate.ready and node.current_body_ground_support_check['valid']
    old_generation = node.gate.generation
    published = []
    node.body_pub = NS(publish=published.append)
    node.counts = dict(body_published=0)
    lowered = body_message(100.20, -.360)
    # The real body callback continues publishing the unchanged measurement.
    LiveScanBridge.on_body(node, lowered)
    assert published == [lowered] and node.counts['body_published'] == 1
    assert not node.gate.ready and not node.gate.active and node.sensor_ready
    assert node.gate.generation == old_generation+1
    assert len(paths) == 1 and not paths[0].path.poses
    reason = 'pct_support_wrong_height_or_floor'
    assert node.current_body_ground_support_check['reason'] == reason
    assert ('admission', reason) in clears
    assert node.sensor_barrier == 99. and node.cloud_stamp == 100.10 and node.pending
    assert node.reference_refresh is None
    node.update_gate()
    assert len(paths) == 1 and node.gate.generation == old_generation+1
    if backend == 'per_sensor_rays':
        receipt(node, clock, source=100.21)
        assert node.cloud_stamp == pytest.approx(100.21) and not node.gate.ready
    # Height recovery renews readiness, never the old trajectory/generation.
    clock.wall, clock.mono = 100.32, 10.32
    LiveScanBridge.on_body(node, body_message(100.30, .025))
    assert node.gate.ready and not node.gate.active
    assert node.gate.generation == old_generation+1
    assert node.gate.barrier == clock.wall
    with pytest.raises(ValueError, match='obsolete_or_stale_reference'):
        node.gate.accept([[0, 0, -.525], [1, 0, -.525]], frame_id=node.gate.frame_id,
                         stamp=100.12, now=clock.wall, body_xyz=[.1, .1, .025])


@pytest.mark.parametrize('kind,reason', [
    ('hole', 'pct_support_hole_or_blocked_cell'),
    ('outside', 'pct_support_outside_map'),
    ('ambiguous', 'pct_support_ambiguous_floor'),
    ('missing_index', 'pct_support_unavailable'),
])
def test_current_body_support_failures_keep_their_specific_reason(monkeypatch, kind, reason):
    node, _, paths, _, _ = harness(monkeypatch)
    if kind == 'hole':
        node.ground_support = support_index(missing={(1, 1)})
    elif kind == 'outside':
        node.body.pose.pose.position.x = .5
    elif kind == 'ambiguous':
        node.ground_support = support_index(levels=[-.1, .1])
    else:
        del node.ground_support
    node.update_gate()
    assert not node.gate.ready and not node.gate.active
    assert node.current_body_ground_support_check['reason'] == reason
    assert node.gate.reason == reason
    assert len(paths) == 1 and node.sensor_barrier == 99.


def test_current_body_support_is_cached_by_source_stamp_and_context(monkeypatch):
    node, clock, _, _, _ = harness(monkeypatch)
    checked, original = [], node.ground_support.validate_body_samples
    def validate(points, **parameters):
        checked.append(points)
        return original(points, **parameters)
    monkeypatch.setattr(node.ground_support, 'validate_body_samples', validate)
    node.update_gate()
    node.update_gate()
    assert len(checked) == 1 and checked[0][0] == checked[0][1]
    node.body = body_message(100.20)
    node.update_gate()
    assert len(checked) == 2
    node.map_context_sequence += 1
    node.update_gate()
    assert len(checked) == 3
    # An old body cannot borrow the support result of a replacement context.
    clock.context = ('session', 2, 'new-seed')
    node.update_gate()
    assert not node.gate.ready and node.current_body_ground_support_check == {}
    assert len(checked) == 3
