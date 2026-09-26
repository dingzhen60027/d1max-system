"""Completed native integrations, not message arrivals, renew perception."""
import json
from types import SimpleNamespace

from std_msgs.msg import String
from d1max_pct_scan.live_scan_bridge import LiveScanBridge


def harness():
    context = dict(session_id='session', epoch=1, seed_id='seed', sequence=2,
                   barrier_ns=99_000_000_000)
    node = SimpleNamespace(now_s=lambda:100.2, map_context_ready=True,
        map_context_identity=('session', 1, 'seed'),
        current_context=lambda now:('session', 1, 'seed'),
        map_context_record=context, p={'input_timeout':.5}, sensor_barrier=99.,
        ray_status_stamp=0., cloud_stamp=0., last_output_stamp=0.,
        cloud_received=0., ray_integration={}, error='', update_gate=lambda:None)
    value = dict(context, schema=1, received_at_unix=100.19, valid=True,
                 source_stamp_ns=100_100_000_000)
    return node, value


def deliver(node, value):
    LiveScanBridge.on_native_rays(node, String(data=json.dumps(value)))


def test_receipt_uses_measurement_not_publication_time():
    node, value = harness()
    deliver(node, value)
    assert abs(node.cloud_stamp-100.1)<1e-9
    assert node.last_output_stamp == node.cloud_stamp
    assert node.ray_status_stamp == 100.19


def test_foreign_invalid_or_regressed_receipt_cannot_renew():
    node, value = harness()
    deliver(node, value)
    original = node.cloud_received
    for changes in ({'seed_id':'old'}, {'valid':False},
                    {'source_stamp_ns':100_000_000_000},
                    {'source_stamp_ns':99_000_000_000},
                    {'received_at_unix':1000.}):
        deliver(node, dict(value, **changes))
        assert node.cloud_received == original
        assert abs(node.cloud_stamp-100.1)<1e-9


def test_periodic_receipt_cannot_keep_stopped_sensors_alive():
    node, value = harness()
    deliver(node, value)
    original = node.cloud_received
    node.now_s = lambda:101.
    deliver(node, dict(value, received_at_unix=101.))
    assert node.cloud_received == original
    assert node.error == 'ray_measurement_stale_or_before_barrier'
