"""Independent continuity-contract audit; message objects only, no ROS graph."""

from collections import deque
from copy import deepcopy
import json
import math
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import String

from d1max_localization.navigation_output import NavigationOutput
from d1max_localization.math_utils import Pose3, compose, pose_innovation, rotate_vector
from d1max_localization.estimation.navigation import NavigationState
from d1max_localization.estimation.prediction import rotation_step
from test_navigation_estimation import alignment, diagonal, local, locked, pending_local, push_filter


IDENTITY = Pose3((0., 0., 0.), (0., 0., 0., 1.))


def yaw_pose(xyz=(0., 0., 0.), yaw=0.):
    return Pose3(xyz, rotation_step((0., 0., 1.), yaw))


def assert_same_pose(a, b):
    error = pose_innovation(a, b)
    assert math.hypot(error.translation_xy, error.translation_z) < 1e-9
    assert error.rotation < 1e-7


def filter_pose(core, stamp, pose, pc=None, now=None):
    return core.push_filtered(stamp, pose, diagonal() if pc is None else pc,
                              (0., 0., 0.), (0., 0., 0.), stamp if now is None else now)


def message_node():
    # Bypass Node.__init__; there are no publishers, services or middleware.
    node = NavigationOutput.__new__(NavigationOutput)
    node.core = NavigationState()
    node.p = dict(map_frame="d1max_loc_map", odom_frame="d1max_loc_odom",
                  tracking_frame="d1max_loc_tracking", body_frame="d1max_loc_base_link",
                  trajectory_rate_hz=5.)
    node.extrinsic = IDENTITY
    node.now_s = Mock(return_value=10.)
    node.clear_path = Mock()
    node.motion_pub, node.local_pub, node.global_pub = Mock(), Mock(), Mock()
    node.pose_pub, node.path_pub, node.tf = Mock(), Mock(), Mock()
    node.path, node.local_times, node.global_times = deque(), deque(), deque()
    node.local_arrivals, node.global_arrivals = deque(), deque()
    node.last_local_sent = node.last_map_sent = node.last_path = 0.
    node.last_error = ""
    node.predictor_status = {}
    return node


def test_attitude_smoothing_never_rotates_physical_ekf_or_public_body_twist():
    node = message_node()
    for stamp, yaw in ((10., math.pi/2-.10), (10.02, math.pi/2)):
        value = local(stamp)
        q = rotation_step((0., 0., 1.), yaw)
        value["orientation"] = q
        value["world_velocity"] = rotate_vector(q, (1., 0., 0.))
        value["position"] = (0., stamp-10., 0.)
        node.now_s.return_value = stamp
        node.on_local(String(data=json.dumps(value)))
    assert not node.last_error
    raw, smoothed = node.core.raw_local, node.core.local[-1]
    assert pose_innovation(raw.pose, smoothed.pose).rotation > .09
    assert abs(smoothed.linear[1]) > .09  # fixture distinguishes old wrong path
    twist = node.motion_pub.publish.call_args.args[0].twist.twist
    assert (twist.linear.x, twist.linear.y, twist.linear.z) == pytest.approx((1., 0., 0.))
    public = node.local_pub.publish.call_args.args[0].twist.twist
    assert (public.linear.x, public.linear.y, public.linear.z) == pytest.approx((1., 0., 0.))
    node.publish_global(smoothed, yaw_pose((3., 2., 1.), -.6), diagonal())
    public_global = node.global_pub.publish.call_args.args[0].twist.twist
    assert (public_global.linear.x, public_global.linear.y, public_global.linear.z) == pytest.approx((1., 0., 0.))
    assert node.motion_pub.publish.call_count == 2


def test_filter_reset_applies_raw_lio_anchor_to_raw_not_smoothed_pose():
    core = NavigationState()
    assert core.push_local(local(10.), 10.)
    assert core.push_local(local(10.02, x=.05), 10.02)
    assert core.raw_local.pose.position[0] == pytest.approx(.05)
    assert core.local[-1].pose.position[0] < .004
    anchor = yaw_pose((2., -1., .3), math.pi/2)
    contract = alignment(10.02)
    contract["anchor"] = dict(position=anchor.position, orientation=anchor.orientation)
    assert core.accept_map(contract, 10.02)
    _, stamp, reset = core.begin_reset(10.02)
    assert stamp == pytest.approx(10.02)
    assert_same_pose(reset, compose(anchor, core.raw_local.pose))
    assert pose_innovation(reset, compose(anchor, core.local[-1].pose)).translation_xy > .04


def test_map_innovation_uses_raw_history_but_target_uses_public_history():
    core = locked()
    anchor = yaw_pose((2., -1., .3), math.pi/2)
    contract = alignment(10.21)
    contract["anchor"] = dict(position=anchor.position, orientation=anchor.orientation)
    assert core.accept_map(contract, 10.21)
    assert core.push_local(local(10.22, x=.05), 10.22)
    raw = core.local_at(10.22, raw=True)
    smooth = core.local_at(10.22)
    expected = compose(anchor, raw.pose)
    # 0.48m is inside the existing raw-map gate. Using the smoothed position
    # would incorrectly add 0.047m of smoothing lag and reject it as >0.5m.
    measured = Pose3((expected.position[0], expected.position[1]+.48, expected.position[2]), expected.orientation)
    assert pose_innovation(measured, compose(anchor, smooth.pose)).translation_xy > .5
    assert filter_pose(core, 10.22, measured)
    result = core.output(10.22)
    assert result is not None and not core.filter_fault
    assert_same_pose(compose(core.alignment_target[1], smooth.pose), measured)
    raw_mid = core.local_at(10.21, raw=True)
    smooth_mid = core.local_at(10.21)
    assert raw_mid.pose.position[0] == pytest.approx(.025)
    assert smooth_mid.pose.position[0] < .002


@pytest.mark.parametrize("future_sample", [False, True])
def test_current_local_and_future_ekf_cannot_extend_real_target_deadline(future_sample):
    core = locked()
    assert core.output(10.2)
    target = core.alignment_target
    assert core.push_local(local(10.24), 10.24)
    assert core.output(10.24) is not None  # same target, NEW local estimate
    assert core.alignment_target[0] == pytest.approx(10.2)
    # The verified correction is held for 0.60s, not one 80ms EKF tick.
    # Genuine map contracts remain fresh; they cannot re-stamp the EKF target.
    for stamp in np.arange(10.28, 10.79, .04):
        if stamp >= 10.5:
            assert core.accept_map(alignment(float(stamp)), float(stamp))
        assert core.push_local(local(float(stamp)), float(stamp))
        assert core.output(float(stamp)) is not None
    last_output = core.last_output
    assert core.push_local(local(10.801), 10.801)
    if future_sample:
        assert filter_pose(core, 10.81, IDENTITY, now=10.801)
    assert core.output(10.801) is None
    assert not core.output_ready(10.801)
    assert core.last_output == last_output
    assert core.alignment_target == target


def test_repeated_callbacks_do_not_spend_smoothing_budget_or_invent_output():
    core = locked()
    assert core.output(10.2)
    assert core.push_local(local(10.22), 10.22)
    assert push_filter(core, 10.22, x=.08)
    assert core.output(10.22)
    previous = (core.last_output, core.last_global, core.last_local, core.map_residual)
    for i in range(15):
        now = 10.22 + .0005*i
        assert not filter_pose(core, 10.22, yaw_pose((.08, 0., 0.)), now=now)
        assert core.output(now) is None
        assert (core.last_output, core.last_global, core.last_local, core.map_residual) == previous
    # A genuinely new correction also cannot manufacture a new local instant.
    assert filter_pose(core, 10.225, yaw_pose((.08, 0., 0.)), now=10.225)
    assert core.output(10.225) is None
    assert core.last_global == previous[1]
    assert core.push_local(local(10.24), 10.24)
    result = core.output(10.24)
    assert result is not None
    assert 0 < result[1].position[0]-previous[1].position[0] <= .003+1e-9


def test_covariance_preserves_filter_marginal_and_rotated_local_envelope():
    core = locked()
    contract = alignment(10.21)
    anchor = yaw_pose(yaw=math.pi/2)
    contract["anchor"] = dict(position=anchor.position, orientation=anchor.orientation)
    assert core.accept_map(contract, 10.21)
    value = local(10.22)
    local_cov = np.eye(6)*.005
    local_cov[0, 0], local_cov[1, 1] = .04, .01
    local_cov[0, 1] = local_cov[1, 0] = .02
    value["pose_covariance"] = local_cov.flatten().tolist()
    assert core.push_local(value, 10.22)
    filter_cov = np.diag([.2, .3, .4, .1, .2, .3])
    assert filter_pose(core, 10.22, anchor, pc=filter_cov.flatten().tolist())
    untouched_target_pc = deepcopy(core.filtered[-1][2])
    first = core.output(10.22)
    output_cov = np.asarray(first[2]).reshape(6, 6)
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    axes = np.zeros((6, 6))
    axes[:3, :3] = axes[3:, 3:] = rotation
    # Checks the advertised additional local envelope, not a claim that the
    # full correlated SE(3) composition has exact statistical covariance.
    assert np.linalg.eigvalsh(output_cov-filter_cov-axes@local_cov@axes.T)[0] >= -1e-9
    value["stamp_ns"] = value["imu_stamp_ns"] = "10240000000"
    value["received_at_unix"] = 10.24
    assert core.push_local(value, 10.24)
    second = core.output(10.24)
    assert second is not None and core.alignment_target[0] == pytest.approx(10.22)
    assert np.all(np.diag(np.asarray(second[2]).reshape(6, 6)) > np.diag(output_cov))
    assert core.alignment_target[2] == core.filtered[-1][2] == untouched_target_pc


def test_local_uncertainty_growth_reaches_global_without_new_ekf_sample():
    core = locked()
    first = core.output(10.2)
    value = local(10.22)
    value["pose_covariance"] = diagonal(.2)
    assert core.push_local(value, 10.22)
    second = core.output(10.22)
    assert second is not None
    assert all(second[2][7*i] > first[2][7*i]+.18 for i in range(6))


def test_epoch_and_reseed_remove_targets_and_old_ack_authority():
    core = locked()
    assert core.output(10.2)
    old_key, old_stamp = core.filter_key, core.reset_stamp
    assert core.push_local(local(10.22, epoch=2), 10.22)
    assert core.alignment_target is None and core.last_output_key is None
    assert not core.filtered and len(core.raw_history) == len(core.local) == 1
    assert not core.reset_ack(old_key, old_stamp, 10.22)
    assert not filter_pose(core, 10.22, IDENTITY)
    assert not core.output_ready(10.22)
    core = locked()
    assert core.output(10.2)
    assert core.accept_map(alignment(10.21, seed="new-seed"), 10.21)
    assert core.alignment_target is None and not core.output_ready(10.21)
    assert not core.reset_ack((1, "seed-a"), 10., 10.21)
    assert core.begin_reset(10.21)
    assert not core.filtered and not core.reset_ack_at


def test_soft_invalid_local_pauses_but_new_real_local_can_use_unexpired_alignment():
    core = locked()
    assert core.output(10.2)
    assert core.push_local(pending_local(10.21, reason="waiting_lio"), 10.21)
    target = core.alignment_target
    assert target is not None and target[0] == pytest.approx(10.2)
    assert core.output(10.21) is None and not core.output_ready(10.21)
    assert core.push_local(local(10.22), 10.22)
    assert core.output(10.22) is not None and core.output_ready(10.22)
    assert core.alignment_target == target
