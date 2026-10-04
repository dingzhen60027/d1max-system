"""No ROS, SDK, network, services, or physical acceptance changes."""
from dataclasses import replace
import math
from pathlib import Path
import pytest
from d1max_pct_scan.control_frame_contract import (
    Context, Rigid, BodySample, MapSpline, create_anchor, anchor_control_bundle,
    correction_review, control_frame_blockers)
from d1max_pct_scan import motion_coordinator, motion_stack


CTX = Context('session', 1, 'seed', 'pcd-content-sha')
NS = 1790462000123456789
IDENTITY = Rigid((0., 0., 0.), (0., 0., 0., 1.))


def yaw_pose(x=0., y=0., z=0., yaw=0.):
    return Rigid((x, y, z), (0., 0., math.sin(yaw/2), math.cos(yaw/2)))


def anchor(correction=IDENTITY, *, local=None, stamp=NS, revision=1, context=CTX):
    local = local or yaw_pose(2., 3., .5, .3)
    a = BodySample(context, 'd1max_loc_odom', 'd1max_loc_base_link', stamp, local)
    b = BodySample(context, 'd1max_loc_map', 'd1max_loc_base_link', stamp, correction.compose(local))
    return create_anchor(b, a, revision)


def spline(**changes):
    data = dict(context=CTX, generation=7, trajectory_id=91, source_ns=NS,
                points=((1., 2., .5), (2., 2.2, .5), (3., 3., .5), (4., 4., .5)),
                knots=(-.6, -.4, -.2, 0., .2, .4, .6, .8))
    return MapSpline(**dict(data, **changes))


def test_full_rigid_round_trip_not_just_yaw_or_frame_rename():
    q = tuple(v/math.sqrt(30) for v in (1., 2., 3., 4.))
    pose = Rigid((12., -4., 3.), q)
    point = (1., 2., -.5)
    assert pose.inverse().point(pose.point(point)) == pytest.approx(point)
    assert pose.compose(pose.inverse()).point(point) == pytest.approx(point)


def test_exact_body_source_time_and_identity_required():
    local = BodySample(CTX, 'd1max_loc_odom', 'd1max_loc_base_link', NS, IDENTITY)
    global_ = replace(local, frame='d1max_loc_map')
    for bad in (replace(global_, source_ns=NS+1),
                replace(global_, context=replace(CTX, epoch=2)),
                replace(global_, context=replace(CTX, seed='new')),
                replace(global_, child='lidar'), replace(global_, frame='map')):
        with pytest.raises(ValueError, match='same-source-time'):
            create_anchor(bad, local, 1)


def test_curve_goal_and_state_are_converted_atomically_with_original_identity():
    a = anchor(yaw_pose(10., -3., 1., math.pi/2))
    original = spline()
    bundle = anchor_control_bundle(a, original, (4., 4., .5))
    assert bundle.original is original
    assert bundle.original.source_ns == NS  # integer nanoseconds, no double rounding
    assert bundle.original.trajectory_id == 91 and bundle.original.generation == 7
    assert bundle.original.knots == original.knots
    assert bundle.points_odom[-1] == pytest.approx(bundle.goal_odom)
    assert bundle.goal_odom[2] == pytest.approx(-.5)  # no duplicate body-height offset
    assert a.body_in_odom.xyz == pytest.approx((2., 3., .5))
    for old, transformed in zip(original.points, bundle.points_odom):
        assert a.map_from_odom.point(transformed) == pytest.approx(old)


def test_affine_curve_and_derivatives_commute_with_rigid_conversion():
    a = anchor(yaw_pose(10., -3., 1., .7))
    b = anchor_control_bundle(a, spline(), (4., 4., .5))
    # Any B-spline point is an affine combination of its controls. Derivative
    # coefficients sum to zero; translations must not leak into velocity.
    weights, derivatives = (.1, .2, .3, .4), (-.3, -.2, .1, .4)
    combine = lambda points, ws: tuple(sum(w*p[i] for w, p in zip(ws, points)) for i in range(3))
    inverse = a.map_from_odom.inverse()
    assert combine(b.points_odom, weights) == pytest.approx(inverse.point(combine(b.original.points, weights)))
    assert combine(b.points_odom, derivatives) == pytest.approx(inverse.rotate(combine(b.original.points, derivatives)))


def test_latest_tf_cannot_relabel_old_curve_and_reseed_cannot_resurrect_it():
    for a in (anchor(stamp=NS+1), anchor(context=replace(CTX, epoch=2)),
              anchor(context=replace(CTX, map_version='other-pcd'))):
        with pytest.raises(ValueError, match='exact original-time anchor'):
            anchor_control_bundle(a, spline(), (4., 4., .5))


def test_map_correction_never_mutates_pinned_local_curve_or_state():
    b = anchor_control_bundle(anchor(), spline(), (4., 4., .5))
    before = (b.points_odom, b.goal_odom, b.anchor.body_in_odom, b.anchor.anchor_id)
    updated = anchor(yaw_pose(.01, 0, 0, .001), stamp=NS+20000000, revision=2)
    result = correction_review(b, updated, reserved_clearance_m=.1, body_radius_m=.7)
    assert result['reason'] == 'native_recheck_required' and not result['requires_stop']
    assert result['requires_native_recheck'] is True
    assert before == (b.points_odom, b.goal_odom, b.anchor.body_in_odom, b.anchor.anchor_id)


@pytest.mark.parametrize('correction', [yaw_pose(.25), yaw_pose(yaw=.1)])
def test_large_correction_or_rotation_is_not_a_new_physical_motion(correction):
    b = anchor_control_bundle(anchor(), spline(), (4., 4., .5))
    result = correction_review(b, anchor(correction, stamp=NS+1, revision=2),
                               reserved_clearance_m=.1, body_radius_m=.7)
    assert result['reason'] == 'correction_budget_exceeded' and result['requires_stop']


@pytest.mark.parametrize('change', [dict(context=replace(CTX, seed='new')),
    dict(context=replace(CTX, map_version='new')), dict(stamp=NS-1, revision=2),
    dict(stamp=NS+1, revision=1)])
def test_old_or_wrong_correction_identity_fails_closed(change):
    b = anchor_control_bundle(anchor(), spline(), (4., 4., .5))
    result = correction_review(b, anchor(yaw_pose(.001), **change),
                               reserved_clearance_m=.1, body_radius_m=.7)
    assert result['requires_stop']


def test_physical_flags_or_topic_rename_cannot_invent_missing_integration():
    forged = {'all_calibrated': True, 'control_anchor_validated': True,
              'odom_topic': '/d1max/localization/odometry/local'}
    assert set(control_frame_blockers()) <= set(motion_stack.motion_architecture_blockers(forged))
    with pytest.raises(ValueError, match='control_anchor_transport_missing'):
        motion_stack.execution_config(forged)
    with pytest.raises(ValueError, match='control_anchor_transport_missing'):
        motion_stack.parameters(forged, '/unused')
    # This calls the actual entrypoint and must fail before importing ROS or
    # reading an old session, not just test a disconnected validation helper.
    with pytest.raises(ValueError, match='control_anchor_transport_missing'):
        motion_coordinator.main(['--session', '/old/session/does-not-exist'])


def test_native_standalone_tracker_preflight_is_before_command_endpoints():
    path = Path(__file__).resolve().parents[2]/'d1max_trajectory_tracker/src/trajectory_tracker.cpp'
    text = path.read_text()
    # The old permanently blocked staging executable has been replaced by the
    # real schema-3 transaction gate. Config validation constructs that gate
    # before any endpoint; the actual gate is exercised by native C++ tests.
    assert text.index('std::make_unique<ExecutionContract>') < text.index('command_pub_ =')
    assert text.index('c.require_support_reference=true;') < text.index('std::make_unique<ExecutionContract>')
    assert 'const auto demand=execution_->step(ros_now,receipt);' in text
    assert 'create_subscription<wire::ExecutionPermit>' in text
    assert 'create_subscription<wire::TrajectoryValidation>' in text
    assert 'create_subscription<wire::SupportReference>' in text
    assert 'topic("command_topic", "/d1max/live_planning/execution/command_debug")' in text
    assert 'create_subscription<std_msgs::msg::String>' not in text  # no legacy JSON task authority
