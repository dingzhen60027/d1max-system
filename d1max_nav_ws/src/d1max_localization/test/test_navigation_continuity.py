"""Continuous moving estimates, not just a stationary/green UI regression."""
from dataclasses import replace
import math
from pathlib import Path
import pytest
import yaml

from d1max_localization.math_utils import Pose3, pose_innovation
from d1max_localization.estimation.continuity import continuous_local, correct_pose
from d1max_localization.estimation.navigation import NavigationLimits, NavigationState
from d1max_localization.estimation.prediction import InertialPredictor, PredictionLimits, rotation_step
from d1max_localization.estimation.contracts import MotionState
from test_navigation_estimation import diagonal, local, alignment, locked, push_filter, snapshot


def motion(t, x, *, speed=1.5, yaw=0., omega=0.):
    return MotionState(t, Pose3((x, 0., 0.), rotation_step((0., 0., omega), yaw/omega if omega else 0.)),
                       (speed, 0., 0.), (0., 0., omega), diagonal(), diagonal(), t-.1, t)


def test_local_correction_does_not_rate_limit_actual_fast_motion():
    limits = NavigationLimits()
    previous = motion(10., 0.)
    measured = motion(10.02, .03)
    output, residual = continuous_local(previous, measured, limits)
    assert output.pose.position == pytest.approx(measured.pose.position)
    assert output.world_velocity == (1.5, 0., 0.)
    assert residual == pytest.approx((0., 0.))


def test_lio_posterior_step_and_catch_up_are_corrected_without_pose_snap():
    limits = NavigationLimits()
    previous = motion(10., 0., speed=0.)
    measured = motion(10.02, .05, speed=0.)
    output, residual = continuous_local(previous, measured, limits)
    assert 0 < output.pose.position[0] <= limits.local_correction_speed*.02 + 1e-9
    assert residual[0] > .04
    assert output.pose_covariance[0] > measured.pose_covariance[0]
    # Converges to the same coordinate frame, rather than permanently shifting
    # odom or feeding the correction into the physical velocity observation.
    for i in range(1, 121):
        measured = replace(measured, stamp=10.02+i*.02, source_stamp=10.02+i*.02, imu_stamp=10.02+i*.02)
        output, residual = continuous_local(output, measured, limits)
    assert output.pose.position[0] == pytest.approx(.05, abs=1e-4)
    assert output.world_velocity == (0., 0., 0.)


def test_rotation_correction_uses_shortest_arc_and_real_dt():
    before = Pose3((0., 0., 0.), rotation_step((0., 0., 1.), math.pi-.01))
    after = Pose3((0., 0., 0.), rotation_step((0., 0., 1.), -math.pi+.03))
    output, remaining = correct_pose(before, after, .02, .15, .1, .3)
    assert pose_innovation(output, before).rotation <= .002 + 1e-9
    assert .037 < remaining[1] < .041  # no long-way quaternion interpolation


def test_smoothing_never_hides_divergence_or_reanchors_long_outage():
    limits = NavigationLimits()
    with pytest.raises(ValueError, match='diverged'):
        continuous_local(motion(10., 0.), motion(10.02, 1.), limits)
    with pytest.raises(ValueError, match='continuity_gap'):
        continuous_local(motion(10., 0.), motion(11., 1.5), limits)


def test_global_correction_has_rate_bound_and_residual_covariance():
    core = locked()
    first = core.output(10.2)
    core.push_local(local(10.22), 10.22)
    assert push_filter(core, 10.22, x=.08)
    output = core.output(10.22)
    assert output and not core.filter_fault
    assert 0 < output[1].position[0]-first[1].position[0] <= .003 + 1e-9
    assert core.map_residual[0] > .07
    assert output[2][0] > first[2][0]
    # Raw accepted steps are small, while lag may exceed one raw-step bound.
    # That lag must not be misclassified as a new discontinuous correction.
    for i in range(1, 121):
        t = 10.22 + i*.02
        x = min(.2, .08+i*.01)
        core.push_local(local(t), t)
        core.accept_map(alignment(t), t)
        assert push_filter(core, t, x=x)
        previous = output
        output = core.output(t)
        assert output is not None and not core.filter_fault
        assert abs(output[1].position[0]-previous[1].position[0]) <= .003+1e-9
    assert output[1].position[0] == pytest.approx(.2, abs=.003)


def test_extended_extrapolation_requires_explicit_coast_contract():
    core = NavigationState(replace(NavigationLimits(), max_imu_age=.1, max_coast=.1))
    value = local(10.)
    value.update(imu_stamp_ns='9920000000', source_stamp_ns='9900000000', extrapolation_sec=.08)
    assert not core.push_local(value, 10.)
    value.update(prediction_mode='coasting', degraded=True, reason='predicting_coast')
    assert core.push_local(value, 10.)
    assert core.local[-1].imu_stamp == pytest.approx(9.92)
    assert core.local_ready(10.)
    assert not core.local_ready(10.021)  # original source deadline, no freshening
    value.update(received_at_unix=10.02, stamp_ns='10020000000', extrapolation_sec=.101)
    assert not core.push_local(value, 10.02)


@pytest.mark.parametrize('blackout', [.025, .05, .075, .1])
def test_production_pipeline_50hz_moving_with_batched_imu_arrival(blackout):
    """Arrival-time simulation: 200Hz IMU, 10Hz delayed LIO, 50Hz private EKF.

    This is a deterministic timing/model test, not a live-network benchmark.
    A 100ms blackout is released on its boundary, not after its safety limit.
    """
    config = yaml.safe_load((Path(__file__).parents[1]/'config/localization.yaml').read_text())
    cfg = config['navigation_estimation']['ros__parameters']
    prediction = InertialPredictor(PredictionLimits(**{k[11:]:v for k,v in cfg.items() if k.startswith('prediction.')}))
    nav = NavigationState(NavigationLimits(**{k[7:]:v for k,v in cfg.items() if k.startswith('limits.')}))
    outputs, coasts, invalid = [], [], []
    next_imu = 0
    last_snapshot = None
    def x(t): return 1.5*(t-100.)
    for tick in range(151):
        now = 100.+tick*.02
        # Hold delivery only; after the window release every genuine sample.
        phase = (now-100.) % .5
        delivering = not (.20-1e-8 <= phase < .20+blackout-1e-8)
        if delivering:
            while 99.8+next_imu*.005 <= now+1e-9:
                stamp = 99.8+next_imu*.005
                prediction.push_imu(stamp, (0.,0.,9.81), (0.,0.,.2), now)
                next_imu += 1
        anchor = 99.9 + (tick//5)*.1
        if anchor != last_snapshot:
            value = snapshot(anchor)
            value.update(received_at_unix=now, position=[x(anchor),0.,0.],
                         orientation=list(rotation_step((0.,0.,.2), anchor-100.)))
            value['inertial']['world_velocity']=[1.5,0.,0.]
            assert prediction.accept(value, now)
            last_snapshot = anchor
        state = prediction.evaluate(now)
        assert state is not None, (blackout, now, prediction.reason)
        coasts.append(prediction.prediction_mode == 'coasting')
        value = local(now, x=state.pose.position[0])
        value.update(position=state.pose.position, orientation=state.pose.orientation,
                     world_velocity=state.world_velocity, angular=state.angular,
                     pose_covariance=state.pose_covariance, twist_covariance=state.twist_covariance,
                     source_stamp_ns=str(round(state.source_stamp*1e9)),
                     imu_stamp_ns=str(round(state.imu_stamp*1e9)),
                     extrapolation_sec=state.extrapolation, degraded=prediction.degraded,
                     prediction_mode=prediction.prediction_mode, reason=prediction.reason)
        assert nav.push_local(value, now), (now, nav.fault)
        c = alignment(now)
        c.update(position=[x(now),0.,0.], orientation=list(rotation_step((0.,0.,.2), now-100.)))
        assert nav.accept_map(c, now)
        if tick == 0:
            key, stamp, _ = nav.begin_reset(now)
            assert nav.reset_ack(key, stamp, now)
        output = nav.output(now)
        if output:
            outputs.append((now, output))
        if tick >= 10 and not nav.output_ready(now):
            invalid.append((now, nav.reason))
        filtered_stamp=now+.005
        assert nav.push_filtered(filtered_stamp,
            Pose3((x(filtered_stamp),0.,0.),rotation_step((0.,0.,.2), filtered_stamp-100.)),
            diagonal(),(1.5,0.,0.),(0.,0.,.2),filtered_stamp)
    assert not invalid
    assert any(coasts)
    steady=[item for item in outputs if item[0]>=100.2-1e-9]
    assert len(steady)==141
    for (arrival_a,a),(arrival_b,b) in zip(steady,steady[1:]):
        assert arrival_b-arrival_a == pytest.approx(.02)
        assert b[0].stamp-a[0].stamp == pytest.approx(.02)
        assert b[1].position[0]-a[1].position[0] == pytest.approx(.03, abs=1e-5)
    assert not nav.fault and not nav.filter_fault
    # Nothing in smoothing makes a lost source current or publishes a held TF.
    stopped_at=nav.last_output
    assert prediction.evaluate(103.2) is None
    assert nav.output(103.2) is None and not nav.output_ready(103.2)
    assert nav.last_output==stopped_at
