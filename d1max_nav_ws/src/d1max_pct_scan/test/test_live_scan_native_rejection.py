"""Native reference rejection -> owner revalidation, never silent WAIT_TARGET.

Real bridge/debug callbacks with bounded in-memory publishers; no ROS init.
"""
from collections import deque
import json
from types import MethodType, SimpleNamespace as NS

import pytest

from d1max_pct_scan import live_scan_bridge
from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from test_live_scan_reference_order import accepted_debug, spline_harness, spline


def harness(monkeypatch):
    node, _, _ = spline_harness(monkeypatch)
    clock = NS(wall=100.4, mono=10.)
    monkeypatch.setattr(live_scan_bridge.time, 'monotonic', lambda: clock.mono)
    node.now_s = lambda: clock.wall
    node.reference_refresh = None
    node.reference_refresh_id = 0
    node.native_reference_recovery = None
    node.sensor_barrier = 99.
    node.cloud_stamp, node.cloud_received = 100.2, 9.9
    node.pending = deque()
    node.map_context_ready, node.map_context_fault = True, None
    node.current_context = lambda now: ('test', 1, 'seed')
    node.get_logger = lambda: NS(info=lambda _: None)
    node.clear_attempt_debug = lambda: None
    node.references, node.refresh_requests = [], []
    node.reference_pub = NS(publish=node.references.append)
    node.reference_refresh_pub = NS(publish=lambda msg: node.refresh_requests.append(json.loads(msg.data)))
    node.execution_frozen = True
    node.localizer = dict(session_id='test', local_epoch=1, active_seed_ns='seed',
                         confirmed_seed_ns='seed', local_fault='')
    node.navigation = dict(epoch=1, seed_id='seed', fault='')
    node.counts.update(reference_refresh_requests=0, reference_refresh_expired=0)
    for name in ('clear_outputs', 'reject_native_reference', 'request_reference_refresh'):
        setattr(node, name, MethodType(getattr(LiveScanBridge, name), node))
    return node, clock


def rejected(phase='reference_rejected_odometry', **kwargs):
    value = accepted_debug(0, valid=False, **kwargs)
    value.phase = phase
    value.selected_reference.poses = []
    return value


def test_native_odometry_rejection_revokes_and_asks_owner_for_new_reference(monkeypatch):
    node, clock = harness(monkeypatch)
    original_generation = node.gate.generation
    LiveScanBridge.on_local_debug(node, rejected())
    assert not node.gate.active and node.gate.generation == original_generation+1
    assert node.error == 'reference_rejected_odometry'
    assert node.sensor_barrier == 99. and node.cloud_stamp == 100.2
    assert node.references and not node.references[-1].path.poses
    assert node.admissions and not node.admissions[-1]['valid']
    assert node.reference_refresh['source_stamp'] == 100.1
    node.request_reference_refresh(clock.wall, clock.mono)
    assert len(node.refresh_requests) == 1
    request = node.refresh_requests[0]
    assert request['rejected_reference_stamp'] == 100.1
    assert request['motion_enabled'] is False
    # Sensor recovery alone cannot make the consumed native generation valid.
    LiveScanBridge.on_spline(node, spline(1, generation=original_generation))
    assert not node.validated and not node.gate.active
    assert node.pending_spline_message is None


@pytest.mark.parametrize('phase', ['reference_rejected_frame', 'reference_rejected_geometry'])
def test_native_geometry_rejection_is_terminal_not_an_automatic_retry(monkeypatch, phase):
    node, _ = harness(monkeypatch)
    LiveScanBridge.on_local_debug(node, rejected(phase))
    assert not node.gate.active and node.reference_refresh is None
    assert not node.admissions[-1]['valid'] and node.error == phase


def test_execution_mode_does_not_automatically_retry_rejected_task(monkeypatch):
    node, _ = harness(monkeypatch)
    node.p['execution_mode'] = 'execution'
    LiveScanBridge.on_local_debug(node, rejected())
    assert not node.gate.active and node.reference_refresh is None
    assert not node.admissions[-1]['valid']


@pytest.mark.parametrize('field,value', [('session_id', 'other'), ('generation', 0)])
def test_foreign_rejection_cannot_cancel_current_task(monkeypatch, field, value):
    node, _ = harness(monkeypatch)
    message = rejected()
    setattr(message, field, value)
    LiveScanBridge.on_local_debug(node, message)
    assert node.gate.active and node.gate.generation == 1
    assert not node.references and node.reference_refresh is None


def test_obsolete_rejection_cannot_erase_later_accepted_native_result(monkeypatch):
    node, _ = harness(monkeypatch)
    LiveScanBridge.on_spline(node, spline(1))
    LiveScanBridge.on_local_debug(node, accepted_debug(1, stamp=100.35))
    LiveScanBridge.on_local_debug(node, rejected(stamp=100.3))
    assert node.gate.active and node.reference_refresh is None and not node.references
    assert node.debug_gate.active.plan_id == 1


def test_duplicate_rejection_cannot_extend_revalidation_deadline(monkeypatch):
    node, clock = harness(monkeypatch)
    message = rejected()
    LiveScanBridge.on_local_debug(node, message)
    recovery = dict(node.reference_refresh)
    clock.mono += .1
    LiveScanBridge.on_local_debug(node, message)
    assert node.reference_refresh == recovery and len(node.references) == 1


@pytest.mark.parametrize('changed_geometry', [False, True])
def test_same_context_revalidation_is_bounded_across_new_generations(monkeypatch, changed_geometry):
    node, clock = harness(monkeypatch)
    for i in range(4):
        if i:
            clock.wall += .1
            clock.mono += .1
            # The same owner goal can be replanned from a slightly changed
            # body pose. A different path digest must not renew the budget.
            lateral = .01*i if changed_geometry else 0.
            node.gate.accept([[0, 0, 0], [.5, lateral, 0], [1, 0, 0]],
                frame_id=node.gate.frame_id, stamp=clock.wall, now=clock.wall,
                body_xyz=[0, 0, .55])
        node.reject_native_reference('reference_rejected_odometry')
        assert not node.gate.active
        if i < 3:
            assert node.reference_refresh is not None
            assert node.reference_refresh['deadline'] == 20.
    assert node.reference_refresh is None
    assert node.error.endswith('revalidation_budget_exhausted')


def test_context_change_or_expired_handshake_cannot_request_old_goal(monkeypatch):
    for fault in ('context', 'expiry', 'cancel'):
        node, clock = harness(monkeypatch)
        LiveScanBridge.on_local_debug(node, rejected())
        if fault == 'context':
            node.localizer['local_epoch'] = 2
        elif fault == 'expiry':
            clock.mono = 21.
        else:
            node.reference_refresh = None  # explicit owner cancellation
        node.request_reference_refresh(clock.wall, clock.mono)
        assert not node.refresh_requests and node.reference_refresh is None
