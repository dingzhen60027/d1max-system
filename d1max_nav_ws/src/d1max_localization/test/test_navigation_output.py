"""ROS serialization and lifecycle boundary tests without constructing ROS nodes."""

from unittest.mock import Mock
import time
import numpy as np
import pytest
from nav_msgs.msg import Odometry
from d1max_localization.navigation_output import NavigationOutput
from d1max_localization.estimation.navigation import NavigationState
from d1max_localization.estimation_ros import stamp_time
from test_navigation_estimation import alignment, local, diagonal, locked


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
