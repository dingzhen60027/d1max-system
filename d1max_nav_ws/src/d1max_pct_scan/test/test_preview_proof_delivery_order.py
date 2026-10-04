"""Source-timed, same-input native proof/pair delivery through actual callbacks."""
from copy import deepcopy
from itertools import permutations

import pytest

from d1max_pct_scan.live_scan_bridge import LiveScanBridge, preview_remaining
from d1max_pct_scan.preview_trajectory import PreviewTrajectoryDelivery
from test_live_scan_reference_order import spline, accepted_debug
from test_preview_trajectory_revalidation import bridge, revalidation_message, prepared, recheck, proof


def recovery_bridge(monkeypatch):
    node, clock, emitted = bridge(monkeypatch, initially_accepted=False)
    node.gate.pause_preview_reference(100.31)
    node.clear_marker(reset_order=False, reason='preview_sensor_paused', retain_preview=True)
    node.gate.resume_preview_reference(100.32, node.gate.context)
    return node, clock, emitted


@pytest.mark.parametrize('order', list(permutations(('spline', 'accepted', 'proof'))))
def test_all_cross_topic_orders_recover_only_when_original_pair_and_new_proof_exist(monkeypatch, order):
    node, clock, _ = recovery_bridge(monkeypatch)
    events = dict(spline=(LiveScanBridge.on_spline, spline(1)),
        accepted=(LiveScanBridge.on_local_debug, accepted_debug(1)),
        proof=(LiveScanBridge.on_local_debug, revalidation_message(node, stamp=100.35)))
    proof_received = None
    for index, name in enumerate(order):
        clock.wall, clock.mono = 100.4+.01*index, 10.4+.01*index
        if name == 'proof': proof_received = clock.mono
        callback, message = events[name]
        callback(node, message)
        if index < 2:
            assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    assert node.visible_spline_id == 1 and preview_remaining(node) > 0
    lease = node.preview_trajectory_lease
    assert lease.proof_kind == 'revalidated'
    assert lease.proof_received == proof_received
    assert lease.proof_stamp == 100.35 and lease.spline_source_stamp == 100.2
    assert node.preview_trajectory_record['message'].trajectory.start_time == spline(1).trajectory.start_time
    assert not node.validated and not node.admissions[-1]['valid']


@pytest.mark.parametrize('fault', ['map_stale_on_delivery', 'body_stale_on_delivery',
    'proof_stale_on_delivery', 'collision', 'cancel', 'new_generation', 'new_context',
    'map_sequence', 'wrong_geometry', 'foreign_session', 'wrong_plan', 'no_original_spline',
    'no_original_acceptance'])
def test_stored_proof_never_bypasses_source_identity_collision_or_original_pair(monkeypatch, fault):
    node, clock, _ = recovery_bridge(monkeypatch)
    event = revalidation_message(node, stamp=100.35)
    if fault == 'wrong_geometry': event.local_target.x += .1
    if fault == 'foreign_session': event.session_id = 'foreign'
    if fault == 'wrong_plan': event.plan_id += 1
    LiveScanBridge.on_local_debug(node, event)
    if fault == 'map_stale_on_delivery':
        clock.wall, clock.mono = 100.8, 10.8
    if fault == 'body_stale_on_delivery':
        node.preview_trajectory_delivery.pending_proof[0].checked_body_source_stamp_ns = 99_000_000_000
    if fault == 'proof_stale_on_delivery':
        clock.wall, clock.mono = 102.5, 12.5
    if fault == 'collision':
        invalid = accepted_debug(1, valid=False, stamp=100.36)
        invalid.phase = 'failed_current_validation'
        LiveScanBridge.on_local_debug(node, invalid)
    if fault == 'cancel':
        node.gate.revoke(100.36, 'explicit_reference_cancel')
        node.clear_marker()
    if fault == 'new_generation': node.gate.generation += 1
    if fault == 'new_context': node.gate.context = ('test', 2, 'new-seed')
    if fault == 'map_sequence': node.map_context_sequence += 1
    if fault != 'no_original_acceptance': LiveScanBridge.on_local_debug(node, accepted_debug(1))
    if fault != 'no_original_spline': LiveScanBridge.on_spline(node, spline(1))
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    assert not node.validated


def test_pending_proof_is_not_renewed_by_duplicate_receipts(monkeypatch):
    node, clock, _ = recovery_bridge(monkeypatch)
    event = revalidation_message(node, stamp=100.35)
    LiveScanBridge.on_local_debug(node, event)
    received = node.preview_trajectory_delivery.pending_proof[1]
    clock.wall, clock.mono = 100.45, 10.45
    LiveScanBridge.on_local_debug(node, deepcopy(event))
    assert node.preview_trajectory_delivery.pending_proof[1] == received


@pytest.mark.parametrize('limiting_source', ['map', 'body'])
def test_revalidated_curve_cannot_outlive_its_actual_source_evidence(limiting_source):
    lease, gate = prepared()
    event = proof()
    if limiting_source == 'map':
        event.checked_map_source_stamp_ns = 103_000_000_000
    else:
        event.checked_body_source_stamp_ns = 103_000_000_000
    assert recheck(lease, gate, event)
    # Debug itself is 0.3 s old and current gate remains ready. That is not
    # permission to treat the map/body source checked by this proof as fresh.
    assert lease.remaining(gate=gate, sequence=1, now=103.49, mono=13.49) > 0
    assert lease.remaining(gate=gate, sequence=1, now=103.51, mono=13.51) == 0
    assert lease.proof_stamp == 103.2 and lease.proof_received == 13.3


@pytest.mark.parametrize('phase', ['waiting_recheck', 'waiting_sensor_map'])
@pytest.mark.parametrize('pair_already_admitted', [False, True])
def test_positive_proof_older_than_native_suspension_cannot_restore_curve(monkeypatch, phase,
                                                                        pair_already_admitted):
    if pair_already_admitted:
        node, clock, _ = bridge(monkeypatch)
    else:
        node, clock, _ = recovery_bridge(monkeypatch)
        LiveScanBridge.on_local_debug(node, revalidation_message(node, stamp=100.35))
    waiting = accepted_debug(1, valid=False, stamp=100.37)
    waiting.phase = phase
    LiveScanBridge.on_local_debug(node, waiting)
    if not pair_already_admitted:
        LiveScanBridge.on_spline(node, spline(1))
        LiveScanBridge.on_local_debug(node, accepted_debug(1))
    LiveScanBridge.on_local_debug(node, revalidation_message(node, stamp=100.35))
    assert node.visible_spline_id == -1 and preview_remaining(node) == 0
    # Only a genuinely later complete native check may resolve that suspension.
    LiveScanBridge.on_local_debug(node, revalidation_message(node, stamp=100.39))
    assert node.visible_spline_id == 1 and preview_remaining(node) > 0


@pytest.mark.parametrize('recover_pending', [False, True])
def test_same_delivery_order_reproduces_previous_black_hole_without_pending_proof(monkeypatch,
                                                                               recover_pending):
    if not recover_pending:
        monkeypatch.setattr(PreviewTrajectoryDelivery, 'recover_pending', lambda *args, **kwargs: None)
    node, clock, _ = recovery_bridge(monkeypatch)
    LiveScanBridge.on_local_debug(node, revalidation_message(node, stamp=100.35))
    LiveScanBridge.on_spline(node, spline(1))
    LiveScanBridge.on_local_debug(node, accepted_debug(1))
    assert (preview_remaining(node) > 0) is recover_pending
    assert (node.visible_spline_id == 1) is recover_pending
