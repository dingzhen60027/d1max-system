"""One high-rate clock, two pure cores, asynchronous global EKF correction.

The private global EKF is unchanged. Prediction and public navigation delivery
share a serialized node to remove a ROS/JSON/timer phase hop; each numerical
component keeps its own responsibilities and original measurement provenance.
No SDK, controls, independent second LIO, fake observation or held-pose tick.
"""
from collections import deque
from dataclasses import fields
import json
import math
import signal
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .navigation_output import NavigationOutput, PREFIX
from .estimation.prediction import PredictionLimits
from .estimation.causal_prediction import CausalInertialPredictor


def causal_output_time(clock_ns):
    """Choose the closest past, stable time supported by the float core.

    The sealed numerical/wire core encodes round(t*1e9) and decodes ns*1e-9.
    At current Unix epochs this round trip can advance by hundreds of ns,
    making an estimate falsely future-dated on strict integer consumers.
    Select the numerical integration target BEFORE predicting, not restamp
    an existing pose. Measurements are untouched. No future tolerance changes.
    """
    if type(clock_ns) is not int or not 0<clock_ns<2**31*10**9:
        raise ValueError('invalid_output_clock_ns')
    target=clock_ns*1e-9
    for _ in range(32):
        wire=round(target*1e9)
        if wire<=clock_ns and wire*1e-9==target:
            return target
        target=math.nextafter(target,-math.inf)
    raise ValueError('output_clock_not_causally_representable')


def predictor_parameters(prediction):
    """Resolve the existing single YAML into the co-scheduled ROS boundary."""
    parameters = {'imu_frame': prediction['imu_frame']}
    for field in fields(PredictionLimits):
        if field.name in prediction:
            parameters['prediction.' + field.name] = prediction[field.name]
    return parameters


def prediction_envelope(core, state, now, sequence):
    value = dict(schema=1, epoch=core.epoch, valid=state is not None,
        fault=bool(core.fault), reason=core.reason, received_at_unix=now,
        sequence=sequence, imu_received=core.imu_received, imu_rejected=core.rejected,
        degraded=core.degraded, prediction_mode=core.prediction_mode,
        coast=dict(core.coast_stats), imu_gap=dict(core.gap_stats), timing=core.timing(now))
    value['lio_source_notice'] = dict(core.last_source_notice)
    value['lio_pending_notices'] = core.pending_notices
    if state is not None:
        value.update(stamp_ns=str(round(state.stamp*1e9)),
            source_stamp_ns=str(round(state.source_stamp*1e9)),
            imu_stamp_ns=str(round(state.imu_stamp*1e9)),
            frame=core.odom, child_frame=core.tracking,
            position=state.pose.position, orientation=state.pose.orientation,
            world_velocity=state.world_velocity, linear=state.linear, angular=state.angular,
            pose_covariance=state.pose_covariance, twist_covariance=state.twist_covariance,
            propagation_age_sec=state.stamp-state.source_stamp,
            extrapolation_sec=state.extrapolation,
            covariance_model='conservative_posterior_propagation_not_independent_sensor')
    return value


def quantiles_ms(values):
    if not values:
        return {}
    ordered = sorted(values)
    def percentile(fraction):
        position = (len(ordered)-1)*fraction
        left = int(position)
        return 1000*(ordered[left] + (ordered[min(left+1,len(ordered)-1)]-ordered[left])*(position-left))
    return dict(p50=percentile(.5), p95=percentile(.95), p99=percentile(.99),
                maximum=1000*ordered[-1])


class RealtimeNavigationOutput(NavigationOutput):
    def now_s(self):
        clock_ns=self.get_clock().now().nanoseconds
        # Uninitialized use_sim_time has no source epoch yet. Preserve the
        # normal ROS zero-time startup; no estimate is emitted at that time.
        target=causal_output_time(clock_ns) if clock_ns>0 else 0.
        if clock_ns>0:
            self.clock_target_quantization_ns_max=max(self.clock_target_quantization_ns_max,
                clock_ns-round(target*1e9))
        return target

    def __init__(self):
        self.clock_target_quantization_ns_max=0
        super().__init__()
        # Construct before the executor spins. There must be exactly one
        # predictor and one public TF writer; do not also launch lio_predictor.
        self.timer.cancel()
        self.destroy_subscription(self.local_sub)
        self.imu_frame = self.declare_parameter('imu_frame', 'd1max_loc_lidar').value
        limits = PredictionLimits(**{f.name:self.declare_parameter(
            'prediction.'+f.name, getattr(PredictionLimits(),f.name)).value
            for f in fields(PredictionLimits)})
        if (limits.max_horizon > self.core.limits.max_prediction_horizon
                or limits.max_coast > self.core.limits.max_coast
                or limits.max_coast > self.core.limits.max_imu_age):
            raise ValueError('predictor_bounds_exceed_navigation_contract')
        self.predictor = CausalInertialPredictor(limits,self.p['odom_frame'],self.p['tracking_frame'])
        self.prediction_pub = self.create_publisher(String, PREFIX+'prediction/local_sample', 5)
        self.rt_pub = self.create_publisher(String, PREFIX+'navigation/realtime', 2)
        self.posterior_sub = self.create_subscription(String,PREFIX+'lio/local_sample',self.on_posterior,20)
        self.imu_sub = self.create_subscription(Imu,PREFIX+'imu',self.on_imu,
            QoSProfile(depth=512,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.prediction_sequence = 0
        self.rt_durations = deque(maxlen=3000)
        self.rt_intervals = deque(maxlen=3000)
        self.rt_last_tick = self.rt_last_status = 0.
        self.rt_overruns = self.rt_ticks = 0
        self.timer = self.create_timer(1./self.rate,self.tick)

    def on_posterior(self, message):
        try:
            self.predictor.accept(json.loads(message.data),self.now_s())
        except (ValueError,KeyError,TypeError,OverflowError) as error:
            self.last_error = str(error)

    def on_imu(self, message):
        if message.header.frame_id != self.imu_frame:
            return
        a,w = message.linear_acceleration,message.angular_velocity
        try:
            self.predictor.push_imu(message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                (a.x,a.y,a.z),(w.x,w.y,w.z),self.now_s())
        except (ValueError,TypeError,OverflowError) as error:
            self.last_error = str(error)

    def tick(self):
        started = time.perf_counter()
        now = self.now_s()
        self.prediction_sequence += 1
        state = self.predictor.evaluate(now)
        if self.predictor.epoch:
            value = prediction_envelope(self.predictor,state,now,self.prediction_sequence)
            message = String(data=json.dumps(value,allow_nan=False))
            # Direct delivery to the public-estimate core. The diagnostic
            # wire below is observable but is never looped back as input.
            self.on_local(message)
            self.prediction_pub.publish(message)
        super().tick()
        duration = time.perf_counter()-started
        self.rt_ticks += 1
        self.rt_overruns += int(duration>1./self.rate)
        self.rt_durations.append(duration)
        if self.rt_last_tick:
            self.rt_intervals.append(now-self.rt_last_tick)
        self.rt_last_tick = now
        if now-self.rt_last_status >= 1.:
            self.rt_last_status = now
            self.rt_pub.publish(String(data=json.dumps(dict(
                schema=1,target_period_ms=1000/self.rate,ticks=self.rt_ticks,
                clock_target_quantization_ns_max=self.clock_target_quantization_ns_max,
                compute_overruns=self.rt_overruns,
                compute_ms=quantiles_ms(self.rt_durations),
                ros_tick_interval_ms=quantiles_ms(self.rt_intervals),
                predictor_reason=self.predictor.reason,
                lio_source_notice=self.predictor.last_source_notice,
                lio_pending_notices=self.predictor.pending_notices,
                timing=self.predictor.timing(now),motion_control_enabled=False),allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeNavigationOutput()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError as error:
            if rclpy.ok() or 'rcl_shutdown already called' not in str(error):
                raise
