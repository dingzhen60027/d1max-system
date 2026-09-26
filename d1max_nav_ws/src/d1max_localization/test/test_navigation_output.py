"""ROS serialization and lifecycle boundary tests without constructing ROS nodes."""

from unittest.mock import Mock
from collections import deque
from dataclasses import replace
import json
import time
import numpy as np
import pytest
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from d1max_localization.navigation_output import NavigationOutput
from d1max_localization.estimation.navigation import NavigationState
from d1max_localization.estimation_ros import stamp_time
from test_navigation_estimation import alignment, local, diagonal, locked, pending_local


@pytest.fixture
def node():
    node = NavigationOutput.__new__(NavigationOutput)
    node.p = {"map_frame": "d1max_loc_map", "tracking_frame": "d1max_loc_tracking"}
    node.core = locked()
    node.core.filtered.clear()
    node.now_s = Mock(return_value=10.22)
    node.last_error = ""
    node.future = None
    node.reset_client = Mock()
    node.map_pub = Mock()
    node.last_map_sent = 10.0
    node.request_key = None
    node.request_stamp = 0.0
    node.request_at = 0.0
    return node


def test_ros_numpy_covariance_is_converted_before_strict_contract(node):
    message = Odometry()
    message.header.stamp = stamp_time(10.22)
    message.header.frame_id = "d1max_loc_map"
    message.child_frame_id = "d1max_loc_tracking"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance = np.array(diagonal(), dtype=np.float64)
    node.on_filtered(message)
    assert not node.last_error and len(node.core.filtered) == 1
    assert all(type(x) is float for x in node.core.filtered[-1][2])


def test_reset_timeout_never_retries_or_accepts_a_late_ack_as_recovery(node):
    node.core.accept_map(alignment(10.22, seed="new-seed"), 10.22)
    future = Mock()
    future.done.return_value = False
    node.reset_client.call_async.return_value = future
    node.filter_lifecycle(10.22)
    assert node.reset_client.call_async.call_count == 1
    node.request_at = time.monotonic() - 3.0
    node.filter_lifecycle(10.23)
    assert node.core.filter_fault == "filter_reset_unknown"
    future.done.return_value = True
    future.result.return_value = object()
    node.filter_lifecycle(10.24)
    node.core.push_local(local(10.25), 10.25)
    node.core.accept_map(alignment(10.25, seed="yet-another-seed"), 10.25)
    node.filter_lifecycle(10.25)
    assert node.reset_client.call_async.call_count == 1
    assert node.core.output(10.25) is None


def test_old_reset_ack_cannot_authorize_new_local_epoch(node):
    node.core.accept_map(alignment(10.22, seed="new-seed"), 10.22)
    future = Mock()
    future.done.return_value = False
    node.reset_client.call_async.return_value = future
    node.filter_lifecycle(10.22)
    node.core.push_local(local(10.24, epoch=2), 10.24)
    future.done.return_value = True
    future.result.return_value = object()
    node.filter_lifecycle(10.24)
    assert not node.core.reset_ack_at and not node.core.filtered


def test_observed_frequency_expires_instead_of_showing_configured_rate():
    from collections import deque

    samples = deque(10.0 + i * 0.02 for i in range(51))
    assert NavigationOutput.rate_of(samples, 11.0) == pytest.approx(50.0)
    assert NavigationOutput.rate_of(samples, 11.2) == 0.0


def streaming_boundary(node):
    """Only Python callbacks/ROS message objects; never constructs a ROS node."""
    node.publish_local = Mock(return_value=False)
    node.publish_global = Mock()
    node.motion_pub = Mock()
    node.clear_path = Mock()
    node.local_times, node.global_times = deque(), deque()
    node.local_arrivals, node.global_arrivals = deque(), deque()
    node.last_local_sent = 0.0
    node.predictor_status = {}
    return node


def test_predictor_arrival_not_second_timer_delivers_each_real_local_sample(node):
    node = streaming_boundary(node)
    node.core = NavigationState()
    stamps = [10.0 + i * .02 for i in range(100)]
    # Ordered transport, fixed 50Hz source, 1..19ms arrival jitter. A 50Hz
    # latest-only poll at +10ms loses 25% despite no source loss or reordering.
    arrivals = [(stamp + (.019 if i % 4 == 0 else .001), stamp)
                for i, stamp in enumerate(stamps)]
    old_polled = []
    for tick in [10.01 + i * .02 for i in range(100)]:
        available = [stamp for arrival, stamp in arrivals if arrival <= tick]
        if available and (not old_polled or available[-1] > old_polled[-1]):
            old_polled.append(available[-1])
    assert len(old_polled) == 75
    for now, stamp in arrivals:
        node.now_s.return_value = now
        node.on_local(String(data=json.dumps(local(stamp))))
    delivered = [call.args[0].stamp for call in node.publish_local.call_args_list]
    assert delivered == pytest.approx(stamps, abs=1e-9)
    assert node.motion_pub.publish.call_count == 100
    assert node.publish_global.call_count == 0  # no seed/map was admitted
    assert not node.last_error


def test_global_sample_not_blocked_by_already_published_newer_local(node):
    node = streaming_boundary(node)
    node.now_s.return_value = 10.22
    node.on_local(String(data=json.dumps(local(10.22))))
    node.publish_global.reset_mock()
    node.core.filtered.clear()
    for stamp in (10.19, 10.21):
        message = Odometry()
        message.header.stamp = stamp_time(stamp)
        message.header.frame_id = 'd1max_loc_map'
        message.child_frame_id = 'd1max_loc_tracking'
        message.pose.pose.orientation.w = 1.
        message.pose.covariance = diagonal()
        node.on_filtered(message)
    assert node.publish_local.call_count == 1
    assert node.publish_global.call_count == 1
    assert node.publish_global.call_args.args[0].stamp == pytest.approx(10.22)
    assert node.core.alignment_target[0] == pytest.approx(10.21)
    assert not node.last_error


def test_duplicate_or_invalid_local_never_published_again(node):
    node = streaming_boundary(node)
    node.core = NavigationState()
    node.now_s.return_value = 10.22
    good = String(data=json.dumps(local(10.22)))
    node.on_local(good)
    node.on_local(good)
    node.on_local(String(data=json.dumps(dict(
        schema=1, epoch=1, valid=False, fault=False, received_at_unix=10.22))))
    assert node.publish_local.call_count == node.motion_pub.publish.call_count == 1
    assert node.publish_global.call_count == 0


def test_predictor_gap_diagnostics_reach_navigation_status_without_reset(node):
    node = streaming_boundary(node)
    node.now_s.return_value = 10.22
    value = local(10.22)
    value.update(degraded=True, reason="predicting_degraded_imu_gap",
                 imu_gap=dict(count=1, max_sec=.08, max_rotation_rad=.002,
                              integrated_sec=.06, rejected_reason=""))
    node.on_local(String(data=json.dumps(value)))
    assert node.predictor_status["degraded"] is True
    assert node.predictor_status["imu_gap"] == value["imu_gap"]
    assert node.core.epoch == 1
    node.clear_path.assert_not_called()
    assert node.publish_local.call_count == 1
    # A new normal sample removes the current warning; it is not latched by
    # the historical maximum gap or by the previous predictor status.
    node.now_s.return_value = 10.24
    value = local(10.24)
    value.update(degraded=False, reason="predicting", imu_gap=dict(count=0, max_sec=0.))
    node.on_local(String(data=json.dumps(value)))
    assert node.predictor_status["degraded"] is False
    assert node.predictor_status["imu_gap"]["count"] == 0
    assert node.publish_local.call_count == 2
    assert not node.last_error


def test_pending_imu_does_not_refeed_actual_twist_or_republish_local(node):
    node = streaming_boundary(node)
    node.core = locked()
    assert node.core.output(10.2)
    node.now_s.return_value = 10.23
    node.on_local(String(data=json.dumps(pending_local(10.23))))
    assert node.predictor_status["reason"] == "imu_stale"
    assert node.core.local_ready(10.23) and node.core.output_ready(10.23)
    node.motion_pub.publish.assert_not_called()
    node.publish_local.assert_not_called()
    node.publish_global.assert_not_called()
    node.now_s.return_value = 10.251
    node.on_local(String(data=json.dumps(pending_local(10.251))))
    assert not node.core.output_ready(10.251)
    assert not node.last_error


def status_boundary(node):
    node = streaming_boundary(node)
    node.core = locked()
    assert node.core.output(10.2)
    node.global_times.append(10.2)  # previous actual published global sample
    node.filter_lifecycle = Mock()
    node.status_pub = Mock()
    node.pose_status_pub = Mock()
    node.last_status = 0.
    node.rate = 50.
    node.p.update(body_frame="d1max_loc_base_link", extrinsics_verified=False,
                  time_alignment_verified=False)
    return node


def test_status_uses_actual_output_validity_not_previous_attempt_reason(node):
    node = status_boundary(node)
    node.now_s.return_value = 10.23
    assert node.core.push_local(pending_local(10.23), 10.23)
    node.core.reason = "aligned_history_stale"
    # Do not let this test pass just because output() happens to change reason.
    node.publish_aligned = Mock()
    node.tick()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status["valid"] is True
    assert status["navigation_ready"] is False and status["motion_control_enabled"] is False
    assert status["output_age_sec"] == pytest.approx(.03)
    assert list(node.global_times) == [10.2]
    node.last_status = 0.
    node.now_s.return_value = 10.251
    node.tick()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status["valid"] is False and status["local_fresh"] is False


def test_held_map_correction_is_not_misreported_as_missing_imu_coast(node):
    node = status_boundary(node)
    assert node.core.push_local(local(10.32), 10.32)
    assert node.core.output(10.32) is not None  # a new real local source stamp
    node.global_times.append(10.32)
    node.predictor_status = {"prediction_mode": "imu_propagation"}
    node.now_s.return_value = 10.32
    node.tick()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status["valid"] is True
    assert status["alignment_held"] is True
    assert status["alignment_mode"] == "held_correction"
    assert status["alignment_stamp_sec"] == pytest.approx(10.2)
    assert status["quality"] == "tracking"
    unavailable = alignment(10.33)
    unavailable["valid"] = False
    assert node.core.accept_map(unavailable, 10.33)
    assert node.core.push_local(local(10.34), 10.34)
    assert node.core.output(10.34) is not None
    node.global_times.append(10.34)
    node.now_s.return_value = 10.34
    node.last_status = 0.
    node.tick()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status["valid"] is True  # bounded estimate, not motion authorization
    assert status["map_alignment_soft_unavailable"] is True
    assert status["navigation_ready"] is False


def test_pending_status_and_duplicate_candidates_never_refresh_output_age(node):
    node = status_boundary(node)
    node.now_s.return_value = 10.23
    node.on_local(String(data=json.dumps(pending_local(10.23))))
    node.tick()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status["valid"] is True and status["state"] == "tracking"
    assert status["output_attempt_reason"] == "waiting_local_sample"
    assert status["prediction"]["reason"] == "imu_stale"
    node.motion_pub.publish.assert_not_called()
    node.publish_global.assert_not_called()
    assert list(node.global_times) == [10.2]


def test_pending_lio_keeps_actual_coast_provenance_without_republishing(node):
    node = status_boundary(node)
    node.core.limits = replace(node.core.limits, max_imu_age=.1, max_coast=.1)
    node.core.local[-1] = replace(node.core.local[-1], imu_stamp=10.17, extrapolation=.03)
    node.now_s.return_value = 10.23
    notice = pending_local(10.23, "lio_stale")
    notice.update(prediction_mode="unavailable", timing={"posterior_age_sec": .267})
    node.on_local(String(data=json.dumps(notice)))
    node.tick()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status["valid"] is True and status["quality"] == "coasting"
    assert status["prediction"]["reason"] == "lio_stale"
    assert status["prediction"]["timing"]["posterior_age_sec"] == .267
    assert status["local_source_timing"]["target_age_sec"] == pytest.approx(.03)
    assert status["output_stamp_sec"] == pytest.approx(10.2)
    node.motion_pub.publish.assert_not_called()
    node.publish_local.assert_not_called()
    node.publish_global.assert_not_called()
    assert list(node.global_times) == [10.2]


def test_compact_pose_status_is_published_at_output_cadence_not_diagnostic_cadence(node):
    node = status_boundary(node)
    node.publish_aligned = Mock()
    node.now_s.return_value = 10.22
    node.tick()
    first = json.loads(node.pose_status_pub.publish.call_args.args[0].data)
    assert first['valid'] is first['pose_valid'] is True
    assert first['output_stamp_sec'] == pytest.approx(10.2)
    assert first['pose_timeout_sec'] == pytest.approx(.08)
    assert first['frame_id'] == 'd1max_loc_map'
    assert first['body_frame'] == 'd1max_loc_base_link'
    assert first['motion_control_enabled'] is False
    node.now_s.return_value = 10.24
    node.tick()
    second = json.loads(node.pose_status_pub.publish.call_args.args[0].data)
    assert node.status_pub.publish.call_count == 1
    assert node.pose_status_pub.publish.call_count == 2
    assert second['output_stamp_sec'] == first['output_stamp_sec']
    assert second['received_at_unix'] > first['received_at_unix']
    node.now_s.return_value = 10.281
    node.tick()
    expired = json.loads(node.pose_status_pub.publish.call_args.args[0].data)
    assert expired['pose_valid'] is False
    assert expired['output_stamp_sec'] == first['output_stamp_sec']
    assert node.status_pub.publish.call_count == 1  # revoked before next UI tick


@pytest.mark.parametrize('fault,reset', [('local_clock_reset', False), ('', True)])
def test_compact_pose_contract_revokes_fault_and_ambiguous_reset(node, fault, reset):
    node = status_boundary(node)
    node.core.fault = fault
    node.future = Mock() if reset else None
    value = node.pose_status(10.22)
    assert value['valid'] is value['pose_valid'] is False
    assert value['fault'] == fault and value['reset_pending'] is reset
