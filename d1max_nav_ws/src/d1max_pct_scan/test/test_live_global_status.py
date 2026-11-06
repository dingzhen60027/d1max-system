"""Actual planner callbacks/status publication; no ROS initialization or worker.

Status observes the already accepted exact state pair. Publishing a diagnostic
must neither renew its original source nor change the retained global route.
"""
from copy import deepcopy
import json
from types import MethodType, SimpleNamespace as NS

import pytest

from d1max_pct_scan.atomic_navigation_inbox import AtomicNavigationInbox
from d1max_pct_scan.live_global_planner import LiveGlobalPlanner
from test_continuous_reference_transport import state
from test_global_mission_lifecycle import lifecycle


def status_planner(monkeypatch, *, contract='single_floor_v3'):
    node, clock, actions, paths, visuals, attempts = lifecycle(monkeypatch)
    clock.wall, clock.mono = 10., 100.
    node.p.update(session_id='session', map_version_id='map',
                  pipeline_contract=contract, freshness_s=.5, output_directory='')
    node.pause.clear()
    node.pause_wall = None
    node.atomic_inbox = AtomicNavigationInbox()
    node.atomic_state = None
    node.navigation = node.pose_status = {}
    node.body = node.body_context = None
    node.scan_received, node.scan = 100., {}
    node.retired_children = NS(pending=[])
    node.child = node.worker_generation = None
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(clock.wall*1e9)))
    messages = []
    node.status_pub = NS(publish=lambda message: messages.append(json.loads(message.data)))
    for method in ('on_atomic_state', 'confirmed_identity', 'publish_status'):
        setattr(node, method, MethodType(getattr(LiveGlobalPlanner, method), node))
    return node, clock, messages, paths, visuals, attempts


@pytest.mark.parametrize('contract', ['single_floor_v3', 'atomic_navigation_v3'])
def test_atomic_callback_status_uses_same_current_inbox_not_legacy_json(monkeypatch, contract):
    node, _, messages, paths, visuals, attempts = status_planner(monkeypatch, contract=contract)
    original_route = deepcopy(node.last_route)
    sample = state()
    node.on_atomic_state(sample)
    accepted = node.atomic_inbox.latest
    node.publish_status()
    assert node.navigation['valid'] is True
    assert 'navigation_ready' not in node.navigation  # Not a legacy JSON event.
    assert messages[-1]['navigation_ready'] is True
    assert accepted.source_ns == 10_000_000_000
    assert accepted.state.source_ns == accepted.source_ns
    assert sample.source_stamp == sample.global_odometry.header.stamp
    assert node.last_route == original_route and node.route_committed
    assert not paths and not visuals and not attempts


def test_atomic_status_before_first_pair_cannot_borrow_legacy_ready(monkeypatch):
    node, _, messages, _, _, _ = status_planner(monkeypatch)
    node.navigation = dict(valid=True, navigation_ready=True)
    node.publish_status()
    assert node.atomic_inbox.latest is None
    assert messages[-1]['navigation_ready'] is False


def test_status_rechecks_original_source_lease_without_restamping_history(monkeypatch):
    node, clock, messages, _, _, _ = status_planner(monkeypatch)
    node.on_atomic_state(state())
    accepted, body_stamp = node.atomic_inbox.latest, node.body_stamp
    clock.wall += .101; clock.mono += .101
    node.publish_status()
    # The historical valid pair is retained, but no longer usable now. Receipt
    # is still within .5 s; the original IMU's .1 s lease is not refreshed.
    assert node.navigation['valid'] is True
    assert messages[-1]['navigation_ready'] is False
    assert node.atomic_inbox.latest is accepted and node.body_stamp == body_stamp
    assert accepted.received == 100. and accepted.source_ns == 10_000_000_000


def test_status_receipt_expiry_cannot_borrow_fresh_source_clock(monkeypatch):
    node, clock, messages, _, _, _ = status_planner(monkeypatch)
    node.on_atomic_state(state())
    accepted = node.atomic_inbox.latest
    clock.mono += .501
    node.publish_status()
    assert messages[-1]['navigation_ready'] is False
    assert node.atomic_inbox.latest is accepted and accepted.received == 100.


def test_atomic_callback_expiring_during_receive_is_soft_wait_not_node_exception(monkeypatch):
    node, clock, messages, paths, visuals, attempts = status_planner(monkeypatch)
    samples = iter((10.099, 10.101))
    def clock_now():
        clock.wall = next(samples, 10.101)
        return NS(nanoseconds=round(clock.wall*1e9))
    node.get_clock = lambda: NS(now=clock_now)
    # Valid on intake, expired before the callback's former second identity
    # lookup. Crossing a measurement lease must not escape a ROS callback and
    # terminate the global planner; it must still not renew that lease.
    node.on_atomic_state(state())
    clock.wall = 10.101
    node.publish_status()
    assert node.body_context.epoch == 1
    assert node.body_context.confirmed_seed == 'seed'
    assert node.atomic_inbox.latest.source_ns == 10_000_000_000
    assert messages[-1]['navigation_ready'] is False
    with pytest.raises(ValueError, match='fresh_atomic_navigation_state_required'):
        node.confirmed_identity()
    assert not paths and not visuals and not attempts


@pytest.mark.parametrize('bad', ['duplicate', 'old_source', 'foreign_session',
                                'foreign_map', 'invalid_pair'])
def test_ignored_packets_cannot_make_expired_navigation_status_ready(monkeypatch, bad):
    node, clock, messages, _, _, _ = status_planner(monkeypatch)
    node.on_atomic_state(state())
    accepted = node.atomic_inbox.latest
    clock.wall += .11; clock.mono += .11
    sample = state(t=.11)
    if bad == 'duplicate': sample = state()
    elif bad == 'old_source': sample = state(t=-.01)
    elif bad == 'foreign_session': sample.session_id = 'other'
    elif bad == 'foreign_map': sample.map_version_id = 'other'
    else: sample.global_odometry.header.stamp.nanosec += 1
    node.on_atomic_state(sample)
    node.publish_status()
    assert node.atomic_inbox.latest is accepted
    assert messages[-1]['navigation_ready'] is False
    assert accepted.received == 100. and accepted.source_ns == 10_000_000_000


@pytest.mark.parametrize('reason', ['global_alignment_waiting', 'lio_reset'])
def test_explicit_unavailability_is_not_ready_and_new_real_state_recovers(monkeypatch, reason):
    node, clock, messages, paths, visuals, attempts = status_planner(monkeypatch)
    node.on_atomic_state(state())
    clock.wall += .02; clock.mono += .02
    unavailable = state(t=.02, usable=False)
    unavailable.reason = reason
    node.on_atomic_state(unavailable)
    node.publish_status()
    assert node.atomic_inbox.latest.state is None and node.body is None
    assert messages[-1]['navigation_ready'] is False
    assert messages[-1]['localization_valid'] is False
    clock.wall += .02; clock.mono += .02
    node.on_atomic_state(state(t=.04))
    node.publish_status()
    assert messages[-1]['navigation_ready'] is True
    assert node.atomic_inbox.latest.source_ns == 10_040_000_000
    assert not paths and not visuals and not attempts  # Status has no task ownership.


@pytest.mark.parametrize('ready', [False, True])
def test_legacy_status_keeps_existing_navigation_ready_field(monkeypatch, ready):
    node, _, messages, _, _, _ = status_planner(monkeypatch, contract='legacy')
    node.navigation = dict(valid=True, navigation_ready=ready)
    node.publish_status()
    assert messages[-1]['navigation_ready'] is ready
    assert node.atomic_inbox.latest is None
