"""Independent review cases for source leases and terminal task ownership."""
from copy import deepcopy
import json

import pytest
from rclpy.action import GoalResponse
from std_msgs.msg import String

from test_bt_action_adapters import (harness, commit, native_cancel, worker_retired)


@pytest.mark.parametrize('channel,timeout', [('scan', .5), ('worker', 1.)])
def test_callback_delay_cannot_renew_status_source_lease(monkeypatch, channel, timeout):
    node, clock = harness(monkeypatch)
    source = 100.11
    clock.wall, clock.mono = source+timeout*.8, 10.+timeout*.8
    packet = dict(session_id='session', received_at_unix=source, sensor_ready=True,
        native_map_context_ready=True, native_map_context_fault='', localization_epoch=1,
        localization_seed_id='seed')
    callback = node.on_scan_status if channel == 'scan' else node.on_worker_status
    callback(String(data=json.dumps(packet)))
    assert node.fresh_status(channel)
    clock.wall += timeout*.4; clock.mono += timeout*.4
    # Receipt is young, but the producer's original status is older than budget.
    assert not node.fresh_status(channel)
    if channel == 'scan':
        assert not node.local_ready((1, 'seed', 'map'))


def test_retired_follow_request_cannot_resurrect_canceled_route(monkeypatch):
    node, clock = harness(monkeypatch)
    _, handle = commit(node, clock)
    retired_request = deepcopy(handle.request)
    node.cancel_follow(handle)
    clock.wall += .1; clock.mono += .1
    node.on_scan_status(String(data=json.dumps(dict(session_id='session', received_at_unix=clock.wall,
        active_reference=False, generation=4))))
    native_cancel(node, clock)
    worker_retired(node, clock)
    node.follow_tick()
    assert handle.terminal == 'canceled' and node.follow is None
    assert node.follow_goal(retired_request) == GoalResponse.REJECT


def test_follow_not_delivered_waits_only_real_private_worker_retirement(monkeypatch):
    node, clock = harness(monkeypatch)
    node.route_reference_pub.subscribers = 0
    _, handle = commit(node, clock)
    assert node.follow is not None and not node.follow['committed']
    node.cancel_follow(handle)
    assert not node.reference_pub.messages
    clock.wall += .1; clock.mono += .1
    worker_retired(node, clock)
    node.follow_tick()
    assert node.native_cancel_ack is None  # no fabricated native operation
    assert handle.terminal == 'canceled' and not node.quarantine
