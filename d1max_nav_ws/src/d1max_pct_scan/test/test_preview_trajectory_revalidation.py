"""Real preview bridge callbacks and immutable native-proof contract; no ROS IO."""
from copy import deepcopy
from collections import deque
from dataclasses import replace
import json
from types import SimpleNamespace as NS

import pytest
from visualization_msgs.msg import Marker

from d1max_pct_scan.live_scan_bridge import LiveScanBridge, preview_remaining
from d1max_pct_scan.live_scan_contract import ReferenceGate
from d1max_pct_scan.preview_trajectory import PreviewTrajectoryLease, preview_revalidation_enabled
from test_local_debug import snapshot
from test_live_scan_reference_order import spline_harness, spline, accepted_debug
from d1max_pct_scan.live_diagnostics import diagnostics
from test_live_diagnostics import states


def prepared():
    gate = ReferenceGate(generation=3, active=True, ready=True, issued_at=100.,
                         context=('test', 1, 'seed'))
    lease = PreviewTrajectoryLease()
    assert lease.accept(snapshot(), gate=gate, sequence=1, spline_stamp=100.1,
                        now=100.3, mono=10.3)
    return lease, gate


def proof(**changes):
    values = dict(phase='revalidated', stamp=103.2, stamp_ns=103_200_000_000,
                  checked_map_source_stamp_ns=103_100_000_000,
                  checked_body_source_stamp_ns=103_150_000_000,
                  checked_map_revision=4, checked_context_sequence=1)
    values.update(changes)
    return snapshot(**values)


def recheck(lease, gate, event=None, **kwargs):
    arguments = dict(gate=gate, sequence=1, now=103.3, mono=13.3,
                     perception_timeout=.5, body_timeout=.5, sensor_barrier=99.)
    arguments.update(kwargs)
    return lease.revalidate(event or proof(), **arguments)


def test_old_curve_only_becomes_current_after_actual_new_native_proof():
    lease, gate = prepared()
    assert lease.remaining(gate=gate, sequence=1, now=103.3, mono=13.3) == 0
    gate.pause_preview_reference(103.)
    lease.suspend()
    gate.resume_preview_reference(103.1, gate.context)
    assert lease.remaining(gate=gate, sequence=1, now=103.3, mono=13.3) == 0
    assert recheck(lease, gate)
    assert lease.remaining(gate=gate, sequence=1, now=103.3, mono=13.3) > 0
    assert lease.spline_source_stamp == 100.1
    assert lease.snapshot.stamp == 100.2  # neither acceptance nor geometry rewritten
    assert lease.proof_stamp == 103.2 and lease.proof_kind == 'revalidated'


@pytest.mark.parametrize('changes', [dict(plan_id=8), dict(session_id='foreign'),
    dict(generation=2), dict(frame_id='odom'), dict(valid=False), dict(phase='accepted'),
    dict(matching_path_headers=False), dict(stamp=100.2, stamp_ns=100_200_000_000),
    dict(stamp=104., stamp_ns=104_000_000_000),
    dict(checked_map_source_stamp_ns=100_000_000_000),
    dict(checked_body_source_stamp_ns=100_000_000_000),
    dict(checked_context_sequence=2), dict(checked_map_revision=0),
    dict(checked_map_source_stamp_ns=0), dict(selected_reference=[[0., 0., 0.]]),
    dict(local_target=[8., 2., 3.]), dict(projection=[3., 2., 1.]),
    dict(target_arc_m=8.)])
def test_wrong_stale_missing_or_geometry_changed_proof_never_restores(changes):
    lease, gate = prepared()
    lease.suspend()
    assert not recheck(lease, gate, proof(**changes))
    assert lease.remaining(gate=gate, sequence=1, now=103.3, mono=13.3) == 0


@pytest.mark.parametrize('change', ['cancel', 'generation', 'context', 'sequence', 'paused', 'not_ready'])
def test_hard_context_changes_and_pause_cannot_be_revived(change):
    lease, gate = prepared()
    kwargs = {}
    if change == 'cancel': gate.active = False
    if change == 'generation': gate.generation += 1
    if change == 'context': gate.context = ('test', 2, 'seed2')
    if change == 'sequence': kwargs['sequence'] = 2
    if change == 'paused': gate.preview_paused = True
    if change == 'not_ready': gate.ready = False
    assert not recheck(lease, gate, **kwargs)


def test_repeated_or_out_of_order_proof_never_renews_lease():
    lease, gate = prepared()
    assert recheck(lease, gate)
    assert not recheck(lease, gate, now=103.4, mono=13.4)
    assert lease.proof_received == 13.3
    assert not recheck(lease, gate, proof(stamp=103.25, stamp_ns=103_250_000_000,
                                         checked_map_revision=3))
    assert lease.proof_received == 13.3


def test_unknown_curve_and_collision_revoked_curve_cannot_be_rechecked():
    lease, gate = prepared()
    lease.reset()  # real collision, hard cancel, or startup without accepted pair
    assert not recheck(lease, gate)


@pytest.mark.parametrize('parameters', [dict(), dict(execution_mode='execution',
    collision_policy='official', perception_backend='per_sensor_rays'),
    dict(execution_mode='preview', collision_policy='observed_free', perception_backend='per_sensor_rays'),
    dict(execution_mode='preview', collision_policy='official', perception_backend='deskewed_cloud')])
def test_no_motion_or_strict_or_legacy_contract_can_opt_in(parameters):
    assert not preview_revalidation_enabled(parameters)


def bridge(monkeypatch, *, initially_accepted=True):
    node, _, emitted = spline_harness(monkeypatch)
    node.p.update(collision_policy='official', perception_backend='per_sensor_rays')
    clock = NS(wall=100.4, mono=10.4)
    monkeypatch.setattr('d1max_pct_scan.live_scan_bridge.time.monotonic', lambda: clock.mono)
    node.now_s = lambda: clock.wall
    if initially_accepted:
        LiveScanBridge.on_spline(node, spline(1))
        LiveScanBridge.on_local_debug(node, accepted_debug(1))
        assert node.visible_spline_id == 1 and preview_remaining(node) > 0
    return node, clock, emitted


def revalidation_message(node, stamp=103.2, **changes):
    original = accepted_debug(1)
    message = NS(**{name: deepcopy(getattr(original, name))
                    for name in original.get_fields_and_field_types()})
    message.phase = 'revalidated'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    message.selected_reference.header.stamp = message.header.stamp
    for pose in message.selected_reference.poses:
        pose.header.stamp = message.header.stamp
    message.checked_map_source_stamp_ns = round((stamp-.1)*1e9)
    message.checked_body_source_stamp_ns = round((stamp-.05)*1e9)
    message.checked_map_revision = 4
    message.checked_context_sequence = 1
    for key, value in changes.items(): setattr(message, key, value)
    return message


def test_bridge_soft_loss_requires_new_proof_then_renders_original_curve(monkeypatch):
    node, clock, emitted = bridge(monkeypatch)
    node.gate.pause_preview_reference(103.)
    node.clear_marker(reset_order=False, reason='preview_sensor_paused', retain_preview=True)
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    node.gate.resume_preview_reference(103.1, node.gate.context)
    clock.wall, clock.mono = 103.3, 13.3
    assert preview_remaining(node) == 0
    LiveScanBridge.on_local_debug(node, revalidation_message(node))
    assert node.visible_spline_id == 1 and preview_remaining(node) > 0
    drawn = [value for value in emitted if isinstance(value, Marker) and value.action == Marker.ADD]
    assert drawn[-1].header.stamp == spline(1).trajectory.start_time
    assert not node.validated and not node.admissions[-1]['valid']


@pytest.mark.parametrize('order', ['spline_first', 'debug_first'])
def test_new_candidate_handover_is_atomic_without_erasing_current_proven_curve(monkeypatch, order):
    node, clock, emitted = bridge(monkeypatch)
    emitted.clear()
    if order == 'spline_first':
        LiveScanBridge.on_spline(node, spline(2))
    else:
        LiveScanBridge.on_local_debug(node, accepted_debug(2, stamp=100.35))
    assert preview_remaining(node) > 0 and node.visible_spline_id == 1
    assert not [m for m in emitted if isinstance(m, Marker) and m.action == Marker.DELETEALL]
    if order == 'spline_first':
        LiveScanBridge.on_local_debug(node, accepted_debug(2, stamp=100.35))
    else:
        LiveScanBridge.on_spline(node, spline(2))
    assert node.visible_spline_id == 2 and preview_remaining(node) > 0
    assert node.preview_trajectory_lease.snapshot.plan_id == 2


def test_native_real_collision_revokes_and_later_same_plan_proof_cannot_revive(monkeypatch):
    node, clock, _ = bridge(monkeypatch)
    event = accepted_debug(1, valid=False, stamp=100.35)
    event.phase = 'failed_current_validation'
    LiveScanBridge.on_local_debug(node, event)
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    clock.wall, clock.mono = 103.3, 13.3
    LiveScanBridge.on_local_debug(node, revalidation_message(node))
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    assert node.debug_gate.phase == 'failed_current_validation'


@pytest.mark.parametrize('phase', ['waiting_sensor_map', 'waiting_recheck'])
def test_incomplete_native_proof_suspends_until_new_complete_proof(monkeypatch, phase):
    node, clock, _ = bridge(monkeypatch)
    event = accepted_debug(1, valid=False, stamp=100.35)
    event.phase = phase
    LiveScanBridge.on_local_debug(node, event)
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    assert node.preview_trajectory_lease.snapshot is not None
    clock.wall, clock.mono = 103.3, 13.3
    LiveScanBridge.on_local_debug(node, revalidation_message(node))
    assert node.visible_spline_id == 1 and preview_remaining(node) > 0
    assert not node.validated and not node.admissions[-1]['valid']


def published_tick_status(node, clock):
    """Exercise the production tick serializer, not a hand-built local status."""
    statuses = []
    node.p['localization_session_id'] = 'test'
    node.publish_pending = lambda *_: None
    node.request_reference_refresh = lambda *_: None
    node.attempt_marker_count = 0
    node.last_status = float('-inf')
    node.map_context_ready, node.map_context_fault = True, None
    node.navigation = dict(epoch=1, seed_id='seed', navigation_ready=False)
    node.reference_refresh, node.reference_refresh_id = None, 0
    node.execution_frozen, node.freeze_received = True, clock.mono
    node.cloud_stamp, node.body = clock.wall-.1, None
    node.pending = deque(maxlen=4)
    node.cloud_timing_ms, node.cloud_processing_samples = 0., deque()
    node.last_queue_wait_ms = 0.
    node.drop_reasons, node.invalid_source_reasons = {}, {}
    node.status_pub = NS(publish=lambda message: statuses.append(json.loads(message.data)))
    LiveScanBridge.tick(node)
    assert len(statuses) == 1
    return statuses[0]


def rendered_diagnostics(status, now):
    localization, global_state, _ = states()
    localization.update(session_id='test', wall_time=now-.1, local_epoch=1)
    localization['navigation'].update(received_at_unix=now-.1, epoch=1)
    global_state.update(session_id='test', localization_epoch=1,
                        last_route=dict(path_stamp=status['owner_reference_stamp']))
    return diagnostics(localization, global_state, status, session_id='test', now=now)


def test_actual_tick_and_ui_use_revalidated_display_identity_after_candidate_was_cleared(monkeypatch):
    node, clock, _ = bridge(monkeypatch)
    original_stamp = node.last_spline_stamp
    node.gate.pause_preview_reference(109.)
    node.clear_marker(reset_order=False, reason='preview_sensor_paused', retain_preview=True)
    assert node.last_spline_stamp == 0.
    node.gate.resume_preview_reference(109.1, node.gate.context)
    clock.wall, clock.mono = 109.3, 19.3
    LiveScanBridge.on_local_debug(node, revalidation_message(node, stamp=109.2))
    status = published_tick_status(node, clock)
    assert status['last_spline_id'] == status['local_debug_plan_id'] == 1
    assert status['last_spline_stamp'] == original_stamp == 100.2
    assert status['trajectory_revalidation']['original_spline_stamp'] == original_stamp
    assert status['latest_candidate_spline_stamp'] == node.last_spline_stamp == 0.
    assert rendered_diagnostics(status, clock.wall)['stages']['local']['label'] == '轨迹有效'
    assert not node.validated and not node.admissions[-1]['valid']


@pytest.mark.parametrize('order', ['spline_first', 'debug_first'])
def test_actual_tick_and_ui_keep_current_identity_while_successor_is_unpaired(monkeypatch, order):
    node, clock, _ = bridge(monkeypatch)
    if order == 'spline_first':
        LiveScanBridge.on_spline(node, spline(2))
    else:
        LiveScanBridge.on_local_debug(node, accepted_debug(2, stamp=100.35))
    status = published_tick_status(node, clock)
    assert status['last_spline_id'] == status['local_debug_plan_id'] == 1
    assert status['last_spline_stamp'] == 100.2
    assert status['latest_candidate_spline_id'] == node.last_spline_id
    assert node.last_spline_id == (2 if order == 'spline_first' else 1)
    assert rendered_diagnostics(status, clock.wall)['stages']['local']['label'] == '轨迹有效'


@pytest.mark.parametrize('order', ['spline_first', 'debug_first'])
@pytest.mark.parametrize('gap_position', ['before_pair', 'between_pair', 'after_pair'])
def test_first_delivery_across_sensor_gap_requires_new_native_proof(monkeypatch, order, gap_position):
    node, clock, emitted = bridge(monkeypatch, initially_accepted=False)
    callbacks = [(LiveScanBridge.on_spline, spline(1)),
                 (LiveScanBridge.on_local_debug, accepted_debug(1))]
    if order == 'debug_first':
        callbacks.reverse()
    def pause():
        node.gate.pause_preview_reference(100.4)
        node.clear_marker(reset_order=False, reason='preview_sensor_paused', retain_preview=True)
    if gap_position == 'before_pair': pause()
    callbacks[0][0](node, callbacks[0][1])
    if gap_position == 'between_pair': pause()
    callbacks[1][0](node, callbacks[1][1])
    if gap_position == 'after_pair': pause()
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    assert node.preview_trajectory_delivery.candidate is not None
    node.gate.resume_preview_reference(103.1, node.gate.context)
    clock.wall, clock.mono = 103.3, 13.3
    assert preview_remaining(node) == 0  # fresh callbacks/ready are not proofs
    LiveScanBridge.on_local_debug(node, revalidation_message(node))
    assert node.visible_spline_id == 1 and preview_remaining(node) > 0
    record = node.preview_trajectory_record
    assert record['message'].trajectory.start_time == spline(1).trajectory.start_time
    assert record['snapshot'].stamp == 100.3
    assert node.preview_trajectory_lease.spline_source_stamp == 100.2
    assert not node.validated and not node.admissions[-1]['valid']


@pytest.mark.parametrize('defect', ['no_spline', 'no_acceptance', 'wrong_id', 'stale_map',
    'stale_body', 'before_recovery', 'future_proof', 'changed_geometry', 'wrong_context',
    'wrong_generation', 'sequence_changed', 'hard_cancel', 'collision', 'late_duplicate',
    'map_barrier'])
def test_unproven_first_delivery_is_never_recovered(monkeypatch, defect):
    node, clock, _ = bridge(monkeypatch, initially_accepted=False)
    node.gate.pause_preview_reference(100.4)
    node.clear_marker(reset_order=False, reason='preview_sensor_paused', retain_preview=True)
    if defect != 'no_spline': LiveScanBridge.on_spline(node, spline(1))
    if defect != 'no_acceptance': LiveScanBridge.on_local_debug(node, accepted_debug(1))
    node.gate.resume_preview_reference(103.1, node.gate.context)
    clock.wall, clock.mono = 103.3, 13.3
    event = revalidation_message(node)
    if defect == 'wrong_id': event.plan_id = 2
    if defect == 'stale_map': event.checked_map_source_stamp_ns = 100_100_000_000
    if defect == 'stale_body': event.checked_body_source_stamp_ns = 100_100_000_000
    if defect == 'before_recovery': event = revalidation_message(node, stamp=103.)
    if defect == 'future_proof': event = revalidation_message(node, stamp=105.)
    if defect == 'changed_geometry': event.local_target.x += 1.
    if defect == 'wrong_context': node.gate.context = ('test', 2, 'changed')
    if defect == 'wrong_generation': node.gate.generation += 1
    if defect == 'sequence_changed': node.map_context_sequence += 1
    if defect == 'hard_cancel':
        node.gate.revoke(103.2, 'explicit_reference_cancel')
        node.clear_marker()
    if defect in ('collision', 'late_duplicate'):
        invalid = accepted_debug(1, valid=False, stamp=103.15)
        invalid.phase = 'failed_current_validation'
        LiveScanBridge.on_local_debug(node, invalid)
        if defect == 'late_duplicate':
            # Still-fresh duplicate geometry cannot rebuild an invalidated
            # plan, even when callbacks are maliciously re-timestamped.
            delayed = spline(1)
            delayed.trajectory.start_time = invalid.header.stamp
            LiveScanBridge.on_spline(node, delayed)
            LiveScanBridge.on_local_debug(node, accepted_debug(1, stamp=103.17))
    if defect == 'map_barrier': node.sensor_barrier = 103.15
    LiveScanBridge.on_local_debug(node, event)
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    assert not node.validated


def test_duplicate_first_delivery_cannot_change_original_cached_geometry(monkeypatch):
    node, clock, _ = bridge(monkeypatch, initially_accepted=False)
    node.gate.pause_preview_reference(100.4)
    LiveScanBridge.on_spline(node, spline(1))
    LiveScanBridge.on_local_debug(node, accepted_debug(1))
    duplicate = spline(1)
    duplicate.trajectory.pos_pts[0].x = 50.
    LiveScanBridge.on_spline(node, duplicate)
    node.gate.resume_preview_reference(103.1, node.gate.context)
    clock.wall, clock.mono = 103.3, 13.3
    LiveScanBridge.on_local_debug(node, revalidation_message(node))
    assert node.visible_spline_id == 1
    assert node.preview_trajectory_record['message'].trajectory.pos_pts[0].x == 0.


@pytest.mark.parametrize('cache_enabled', [False, True])
def test_identical_input_reproduces_delivery_black_hole_without_cache(monkeypatch, cache_enabled):
    """Same production callbacks; ablate only the new non-authoritative cache."""
    if not cache_enabled:
        monkeypatch.setattr('d1max_pct_scan.live_scan_bridge.preview_delivery', lambda _node: None)
    node, clock, _ = bridge(monkeypatch, initially_accepted=False)
    node.gate.pause_preview_reference(100.4)
    node.clear_marker(reset_order=False, reason='preview_sensor_paused', retain_preview=True)
    LiveScanBridge.on_spline(node, spline(1))
    LiveScanBridge.on_local_debug(node, accepted_debug(1))
    assert node.visible_spline_id == -1
    node.gate.resume_preview_reference(103.1, node.gate.context)
    clock.wall, clock.mono = 103.3, 13.3
    LiveScanBridge.on_local_debug(node, revalidation_message(node))
    assert (node.visible_spline_id == 1) is cache_enabled
    assert (preview_remaining(node) > 0) is cache_enabled
    assert not node.validated
