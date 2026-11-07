"""ROS boundary for robot_localization and the only public navigation TF writer.

PCD coordinator and IMU predictor communicate through versioned, epoch-tagged
contracts. robot_localization's unguarded output is PRIVATE. Its SetPose service
is serialized and never retried on an ambiguous timeout. All public outputs are
admitted against source ages, seed/epoch, common-time history and jump limits.
This node never connects to the robot or publishes a motion command.
"""

from collections import deque
from dataclasses import fields
import json
import math
import signal
import time
from pathlib import Path as FilePath
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from robot_localization.srv import SetPose
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .math_utils import compose, inverse, rotate_vector
from .initial_pose import body_to_tracking_transform
from .estimation.contracts import MotionState, body_state, fresh
from .estimation.navigation import NavigationState, NavigationLimits
from .estimation_ros import (
    fill_pose,
    odometry,
    pose_message,
    read_pose,
    seconds,
    stamp_time,
    transform,
)

PREFIX = "/d1max/localization/"


class NavigationOutput(Node):
    def __init__(self):
        super().__init__("navigation_output")
        self.rate = self.declare_parameter("output_rate_hz", 50.0).value
        if not 20.0 <= self.rate <= 100.0:
            raise ValueError("navigation output rate must be 20..100 Hz")
        defaults = {
            "map_frame": "d1max_loc_map",
            "odom_frame": "d1max_loc_odom",
            "tracking_frame": "d1max_loc_tracking",
            "body_frame": "d1max_loc_base_link",
            "tracking_offset_body": [0.4043, 0.0, -0.0377],
            "sdk_to_tracking_yaw": 1.56646,
            "extrinsics_verified": False,
            "time_alignment_verified": False,
            "trajectory_rate_hz": 5.0,
            "trajectory_max_points": 1800,
        }
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        session_dir = self.declare_parameter('session_dir', '').value
        self.wire_session = {}
        if session_dir:
            self.wire_session = json.loads((FilePath(session_dir)/'session.json').read_text())
        if (
            not math.isfinite(self.p["trajectory_rate_hz"])
            or not 0.1 <= self.p["trajectory_rate_hz"] <= self.rate
            or type(self.p["trajectory_max_points"]) is not int
            or not 1 <= self.p["trajectory_max_points"] <= 10000
        ):
            raise ValueError("invalid bounded trajectory settings")
        if (
            len(set(self.p[k] for k in ("map_frame", "odom_frame", "tracking_frame", "body_frame")))
            != 4
        ):
            raise ValueError("navigation frames must be distinct")
        limits = {
            f.name: self.declare_parameter(
                "limits." + f.name, getattr(NavigationLimits(), f.name)
            ).value
            for f in fields(NavigationLimits)
        }
        self.core = NavigationState(
            NavigationLimits(**limits),
            self.p["odom_frame"],
            self.p["tracking_frame"],
            self.p["map_frame"],
        )
        self.extrinsic = body_to_tracking_transform(
            self.p["tracking_offset_body"], self.p["sdk_to_tracking_yaw"]
        )
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(
            transform(self.p["body_frame"], self.p["tracking_frame"], self.extrinsic, self.now_s())
        )
        self.local_pub = self.create_publisher(Odometry, PREFIX + "odometry/local", 20)
        self.global_pub = self.create_publisher(Odometry, PREFIX + "odometry/global", 20)
        from d1max_planning_interfaces.msg import NavigationState as NavigationStateMessage
        self.state_pair_pub = self.create_publisher(NavigationStateMessage, PREFIX+'navigation/state', 5)
        from d1max_planning_interfaces.msg import LocalNavigationState
        self.local_state_pub = self.create_publisher(LocalNavigationState, PREFIX+'navigation/local_state', 5)
        self.last_local_state_identity = None
        self.last_local_state_event = 0.
        self.last_local_state_lease = None
        self.last_local_unavailable = None
        self.pose_continuity = None
        self.pose_continuity_alignment = None
        self.pose_continuity_sequence = 0
        self.pose_continuity_pub = self.create_publisher(String,PREFIX+'navigation/pose_continuity',5)
        self.pose_pub = self.create_publisher(PoseStamped, PREFIX + "pose", 20)
        self.path_pub = self.create_publisher(Path, PREFIX + "trajectory", 2)
        self.status_pub = self.create_publisher(String, PREFIX + "navigation/status", 10)
        # A 5 Hz diagnostic snapshot cannot lease an 80 ms pose. Keep this
        # compact, source-stamped contract on the actual 50 Hz output cadence.
        self.pose_status_pub = self.create_publisher(String, PREFIX + "navigation/pose_status", 5)
        self.motion_pub = self.create_publisher(
            TwistWithCovarianceStamped, PREFIX + "estimator/motion", 20
        )
        self.map_pub = self.create_publisher(
            PoseWithCovarianceStamped, PREFIX + "estimator/map_pose", 10
        )
        self.reset_client = self.create_client(SetPose, PREFIX + "estimator/set_pose")
        self.local_sub = self.create_subscription(
            String, PREFIX + "prediction/local_sample", self.on_local, 20
        )
        self.map_sub = self.create_subscription(String, PREFIX + "map_alignment", self.on_map, 10)
        self.filter_sub = self.create_subscription(
            Odometry, PREFIX + "estimator/odometry_raw", self.on_filtered, 20
        )
        self.future = None
        self.request_key = None
        self.request_stamp = 0.0
        self.request_at = 0.0
        self.last_map_sent = 0.0
        self.last_local_sent = 0.0
        self.last_path = 0.0
        self.last_status = 0.0
        self.last_error = ""
        self.local_times = deque(maxlen=256)
        self.global_times = deque(maxlen=256)
        self.local_arrivals = deque(maxlen=256)
        self.global_arrivals = deque(maxlen=256)
        self.path = deque(maxlen=int(self.p["trajectory_max_points"]))
        self.predictor_status = {}
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def clear_path(self):
        self.path.clear()
        self.last_path = 0.0
        m = Path()
        m.header.frame_id = self.p["map_frame"]
        m.header.stamp = stamp_time(self.now_s())
        self.path_pub.publish(m)

    def on_local(self, message):
        try:
            value = json.loads(message.data)
            old_epoch = self.core.epoch
            old_sequence = self.core.local_sequence
            old_transition = self.core.local_transition_sequence
            accepted = self.core.push_local(value, self.now_s())
            if self.core.local_transition_sequence != old_transition:
                self.get_logger().info("LocalStateTransition " + json.dumps(
                    self.core.local_transition, allow_nan=False))
            if not accepted:
                return
            self.predictor_status = {
                k: value.get(k)
                for k in (
                    "reason",
                    "propagation_age_sec",
                    "extrapolation_sec",
                    "imu_received",
                    "imu_rejected",
                    "degraded",
                    "prediction_mode",
                    "coast",
                    "imu_gap",
                    "timing",
                )
            }
            if old_epoch != self.core.epoch:
                self.clear_path()
                self.local_times.clear()
                self.global_times.clear()
                self.local_arrivals.clear()
                self.global_arrivals.clear()
                self.last_local_sent = 0.0
                self.last_map_sent = 0.0
            if self.core.local_sequence == old_sequence:
                # A pending status may retain the last actual sample only until
                # its original TTL. It is not a new motion observation for EKF.
                self.publish_aligned(self.now_s())
                return
            if not self.core.local_ready(self.now_s()):
                return
            state = self.core.local[-1]
            raw_motion = self.core.raw_local
            # Only twist from the LIO/IMU predictor enters the global EKF. Do not
            # also fuse LIO pose, raw IMU or MC as independent copies of motion.
            m = TwistWithCovarianceStamped()
            m.header.frame_id = self.p["tracking_frame"]
            m.header.stamp = stamp_time(state.stamp)
            m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = raw_motion.linear
            m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = (
                raw_motion.angular
            )
            m.twist.covariance = raw_motion.twist_covariance
            self.motion_pub.publish(m)
            # The predictor already owns the requested cadence. Polling its
            # latest value with a second equal-rate timer drops real samples
            # whenever callback arrival crosses that timer's phase.
            self.publish_local(state)
            self.publish_aligned(self.now_s())
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.last_error = str(error)

    def on_map(self, message):
        try:
            old_key = self.core.key()
            if (
                self.core.accept_map(json.loads(message.data), self.now_s())
                and old_key != self.core.key()
            ):
                self.clear_path()
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.last_error = str(error)

    def on_filtered(self, message):
        if (
            message.header.frame_id != self.p["map_frame"]
            or message.child_frame_id != self.p["tracking_frame"]
        ):
            return
        try:
            v = message.twist.twist.linear
            w = message.twist.twist.angular
            accepted = self.core.push_filtered(
                seconds(message),
                read_pose(message.pose.pose),
                [float(x) for x in message.pose.covariance],
                (v.x, v.y, v.z),
                (w.x, w.y, w.z),
                self.now_s(),
            )
            if accepted:
                self.publish_aligned(self.now_s())
        except (ValueError, TypeError, OverflowError) as error:
            self.last_error = str(error)

    def filter_lifecycle(self, now):
        if self.future is not None:
            if self.future.done():
                try:
                    if self.future.result() is None:
                        raise RuntimeError("empty filter reset result")
                    self.core.reset_ack(self.request_key, self.request_stamp, now)
                except Exception as error:
                    self.core.filter_fault = "filter_reset_unknown"
                    self.last_error = str(error)
                self.future = None
            elif time.monotonic() - self.request_at > self.core.limits.reset_timeout:
                self.core.filter_fault = "filter_reset_unknown"
                self.last_error = "滤波器重置回执超时，禁止自动重试；停止并重启定位"
            return
        if not self.reset_client.service_is_ready():
            return
        reset = self.core.begin_reset(now)
        if reset:
            key, stamp, pose = reset
            request = SetPose.Request()
            request.pose = pose_message(
                self.p["map_frame"], pose, stamp, self.core.map_contract["covariance"]
            )
            self.request_key = key
            self.request_stamp = stamp
            self.request_at = time.monotonic()
            self.last_map_sent = stamp
            try:
                self.future = self.reset_client.call_async(request)
            except Exception as error:
                self.core.filter_fault = "filter_reset_unknown"
                self.last_error = str(error)
        if (
            self.core.reset_ack_at
            and self.core.filter_key == self.core.key()
            and self.core.map_ready(now)
            and not self.core.map_soft_unavailable
        ):
            c = self.core.map_contract
            if c["stamp"] > self.last_map_sent:
                self.map_pub.publish(
                    pose_message(self.p["map_frame"], c["pose"], c["stamp"], c["covariance"])
                )
                self.last_map_sent = c["stamp"]

    def publish_local(self, state):
        if state.stamp <= self.last_local_sent:
            return False
        pose, linear, angular, pc, tc = body_state(state, self.extrinsic)
        # Pose correction is not physical body motion. Keep twist sourced from
        # the original posterior/IMU estimate, not from its smoothed attitude.
        _, linear, angular, _, tc = body_state(self.core.raw_local, self.extrinsic)
        local_message = odometry(
                self.p["odom_frame"],
                self.p["body_frame"],
                pose,
                state.stamp,
                linear,
                angular,
                pc,
                tc,
            )
        self.local_pub.publish(local_message)
        key = self.core.key()
        if key and self.wire_session.get('id') and self.wire_session.get('version_id'):
            from .navigation_wire import local_body_message
            typed = local_body_message(local=local_message,
                session_id=self.wire_session['id'],map_version_id=self.wire_session['version_id'],
                epoch=self.core.epoch,seed_id=str(key[1]),posterior_stamp=state.source_stamp,
                imu_stamp=state.imu_stamp,extrapolation_sec=state.extrapolation)
            self.local_state_pub.publish(typed)
            # Cache only a successfully published contract, including its
            # ORIGINAL deadline. A callback/timer can never extend this lease.
            self.last_local_state_lease = (
                typed.session_id, typed.map_version_id,
                typed.source_stamp.sec*10**9+typed.source_stamp.nanosec,
                typed.valid_until.sec*10**9+typed.valid_until.nanosec)
            self.last_local_state_identity = (self.core.epoch,str(key[1]))
            self.last_local_state_event = state.stamp
            self.last_local_unavailable = None
        self.tf.sendTransform(
            transform(self.p["odom_frame"], self.p["body_frame"], pose, state.stamp)
        )
        self.last_local_sent = state.stamp
        self.local_times.append(state.stamp)
        self.local_arrivals.append(self.now_s())
        return True

    def publish_global(self, local, pose, pc):
        state = MotionState(
            local.stamp,
            pose,
            rotate_vector(pose.orientation, self.core.raw_local.linear),
            self.core.raw_local.angular,
            pc,
            self.core.raw_local.twist_covariance,
            local.source_stamp,
            local.imu_stamp,
            local.extrapolation,
        )
        body, linear, angular, body_cov, tc = body_state(state, self.extrinsic)
        global_message = odometry(
                self.p["map_frame"],
                self.p["body_frame"],
                body,
                local.stamp,
                linear,
                angular,
                body_cov,
                tc,
            )
        self.global_pub.publish(global_message)
        # Build both members from the SAME output sample, not two latest
        # subscribers. Epoch/seed are estimator-owned, never supplied by tasks.
        key = self.core.key()
        raw = self.core.local_at(local.stamp, raw=True)
        if key and raw is not None and self.core.last_output_key==key:
            # Freeze the offset at this ACTUAL same-time pair. A later raw
            # local sample or a soft control-unavailable notice cannot change
            # the coordinates of a position-only prediction's old anchor.
            self.pose_continuity_alignment=(key,compose(pose,inverse(local.pose)),
                                           compose(local.pose,inverse(raw.pose)))
        if key and self.wire_session.get('id') and self.wire_session.get('version_id'):
            if raw is not None:
                local_pose, _, _, local_cov, _ = body_state(local, self.extrinsic)
                _, local_linear, local_angular, _, local_tc = body_state(raw, self.extrinsic)
                local_message = odometry(self.p['odom_frame'], self.p['body_frame'], local_pose,
                    local.stamp, local_linear, local_angular, local_cov, local_tc)
                from .navigation_wire import body_pair_message
                self.state_pair_pub.publish(body_pair_message(local=local_message, global_=global_message,
                    session_id=self.wire_session['id'], map_version_id=self.wire_session['version_id'],
                    epoch=self.core.epoch, seed_id=str(key[1]), posterior_stamp=local.source_stamp,
                    imu_stamp=local.imu_stamp, extrapolation_sec=local.extrapolation))
        self.tf.sendTransform(
            transform(
                self.p["map_frame"],
                self.p["odom_frame"],
                compose(pose, inverse(local.pose)),
                local.stamp,
            )
        )
        m = pose_message(self.p["map_frame"], pose, local.stamp)
        self.pose_pub.publish(m)
        self.global_times.append(local.stamp)
        self.global_arrivals.append(self.now_s())
        if local.stamp - self.last_path >= 1.0 / self.p["trajectory_rate_hz"]:
            self.path.append(m)
            path = Path()
            path.header = m.header
            path.poses = list(self.path)
            self.path_pub.publish(path)
            self.last_path = local.stamp

    @staticmethod
    def rate_of(values, now):
        while values and now - values[0] > 2.0:
            values.popleft()
        if len(values) < 2 or not fresh(values[-1], now, 0.12):
            return 0.0
        span = values[-1] - values[0]
        return (len(values) - 1) / span if span >= 0.5 else 0.0

    def publish_aligned(self, now):
        output = self.core.output(now)
        if output:
            # The correction is derived in aligned history; the output itself
            # is a new local-time composition. The core owns deduplication.
            local, pose, pc = output
            self.publish_global(local, pose, pc)

    def pose_status(self, now):
        stamp = self.global_times[-1] if self.global_times else None
        timeout = min(self.core.limits.local_timeout, self.core.limits.filter_timeout)
        fault = self.core.fault or self.core.filter_fault
        valid = bool(stamp is not None and fresh(stamp, now, timeout)
                     and self.core.output_ready(now) and not fault and self.future is None)
        # A later pending predictor notice must not relabel the provenance of
        # the last actual output. It can neither advance nor refresh that pose.
        output_ns = round(stamp * 1e9) if stamp is not None else None
        output_source = next((state for state in reversed(self.core.local)
                              if round(state.stamp * 1e9) == output_ns), None)
        coasting = bool(output_source and output_source.extrapolation > .025 + 1e-6)
        model=self.pose_continuity
        model_valid=bool(model and model['available'] and not valid and not fault and self.future is None
            and self.core.key()==(model['localization_epoch'],model['localization_seed_id'])
            and fresh(model['estimate_stamp_ns']*1e-9,now,timeout)
            and round(now*1e9)<=model['valid_until_ns'])
        if model_valid:stamp=model['estimate_stamp_ns']*1e-9
        return {
            'schema': 1, 'epoch': self.core.epoch,
            'seed_id': self.core.key()[1] if self.core.key() else None,
            'frame_id': self.p['map_frame'], 'body_frame': self.p['body_frame'],
            'received_at_unix': now, 'output_stamp_sec': stamp,
            'pose_timeout_sec': timeout, 'valid': valid or model_valid, 'pose_valid': valid or model_valid,
            'fault': fault, 'reset_pending': self.future is not None,
            'motion_control_enabled': False, 'output_clock': 'local_motion_samples',
            'quality': ('fault' if fault else 'imu_gap_coasting_pose_only' if model_valid else 'coasting' if valid and coasting else
                        'tracking' if valid else 'paused'),
            'reason': 'bounded_missing_imu_model_not_motion_usable' if model_valid else self.core.output_attempt_reason,
            'motion_usable': valid,
            'posterior_stamp_ns': model['posterior_stamp_ns'] if model_valid else None,
            'imu_stamp_ns': model['imu_stamp_ns'] if model_valid else None,
        }

    def publish_pose_continuity(self, state=None, *, horizon=.75, imu_gap=.5):
        """One public TF writer; model geometry never enters EKF/typed control.

        Map/odom smoothing offset is the last REAL paired output. Retaining it
        for a finite prediction is not a new map correction or measurement.
        """
        now=self.now_s();key=self.core.key()
        alignment=self.pose_continuity_alignment
        available=bool(state is not None and key and alignment and alignment[0]==key
            and not self.core.fault and not self.core.filter_fault and self.future is None
            and state.stamp>self.core.last_output
            and fresh(state.stamp,now,self.core.limits.local_timeout,future=0.)
            and state.source_stamp<=state.stamp and state.imu_stamp<=state.stamp
            and fresh(state.source_stamp,now,horizon,future=0.)
            and fresh(state.imu_stamp,now,imu_gap,future=0.))
        self.pose_continuity_sequence+=1
        value=dict(schema=1,sequence=self.pose_continuity_sequence,estimate_stamp_ns=round(now*1e9),
            session_id=self.wire_session.get('id',''),
            map_version_id=self.wire_session.get('version_id',''),localization_epoch=self.core.epoch,
            localization_seed_id=str(key[1]) if key else '',frame_id=self.p['map_frame'],body_frame=self.p['body_frame'],
            available=available,motion_usable=False,quality='imu_gap_coasting_pose_only' if available else 'unavailable',
            reason='bounded_missing_imu_model_not_motion_usable' if available else 'no_bounded_position_model')
        if available:
            _,anchor,offset=alignment
            local_pose=compose(offset,state.pose)
            global_pose=compose(anchor,local_pose)
            from .lio_fusion import rotate_pose_covariance
            global_covariance=rotate_pose_covariance(state.pose_covariance,compose(anchor,offset).orientation)
            local_state=MotionState(state.stamp,local_pose,state.world_velocity,state.angular,
                state.pose_covariance,state.twist_covariance,state.source_stamp,state.imu_stamp,state.extrapolation)
            local_body=body_state(local_state,self.extrinsic)[0]
            global_state=MotionState(state.stamp,global_pose,rotate_vector(anchor.orientation,state.world_velocity),
                state.angular,global_covariance,state.twist_covariance,state.source_stamp,state.imu_stamp,state.extrapolation)
            body,_,_,pc,_=body_state(global_state,self.extrinsic)
            value.update(estimate_stamp_ns=round(state.stamp*1e9),posterior_stamp_ns=round(state.source_stamp*1e9),
                imu_stamp_ns=round(state.imu_stamp*1e9),
                valid_until_ns=min(round(state.source_stamp*1e9)+round(horizon*1e9),
                                   round(state.imu_stamp*1e9)+round(imu_gap*1e9)),
                position=list(body.position),orientation=list(body.orientation),pose_covariance=pc)
            self.tf.sendTransform(transform(self.p['odom_frame'],self.p['body_frame'],local_body,state.stamp))
            self.tf.sendTransform(transform(self.p['map_frame'],self.p['odom_frame'],anchor,state.stamp))
        self.pose_continuity=value
        if key is None or (alignment and alignment[0]!=key) or self.core.fault or self.core.filter_fault:
            self.pose_continuity_alignment=None
        self.pose_continuity_pub.publish(String(data=json.dumps(value,allow_nan=False)))

    def publish_local_unavailable(self, now):
        """One explicit revocation per episode, never a pose heartbeat.

        Soft expiry uses the last measurement watermark + 1 ns, so it cannot
        suppress a newer causal sample merely because transport was delayed.
        Epoch/seed loss is hard. Map/EKF freshness alone does not revoke odom.
        A pending next prediction cannot retract the consumption lease already
        issued for a valid source. This does not generate or republish a pose.
        """
        identity=self.last_local_state_identity
        if identity is None:
            return
        key=self.core.key()
        lease=self.last_local_state_lease
        wire_changed=bool(lease and lease[:2] != (
            self.wire_session.get('id'), self.wire_session.get('version_id')))
        changed=(wire_changed or self.core.epoch!=identity[0] or not key or str(key[1])!=identity[1])
        if not changed and self.core.local_ready(now):
            return
        # local_ready checks whether a NEW output can be made at this clock
        # (including the current IMU age). Typed consumers instead consume a
        # previously admitted source until its signed valid_until. Mixing these
        # two checks revoked valid states for a 0.204 ms IMU boundary crossing.
        # Only missing-input/age cases retain the old lease; a fault, invalid
        # measurement, reset or wire identity change still revokes immediately.
        now_ns=round(now*1e9)
        if (not changed and not self.core.fault and self.core.local_valid and lease
                and lease[2] <= now_ns < lease[3]):
            return
        reason='localization_reset' if changed else ('hard_localization_lost'
            if self.core.fault else 'local_navigation_unavailable')
        token=(identity,lease[:2] if lease else None,reason)
        if self.last_local_unavailable==token:
            return
        from d1max_planning_interfaces.msg import LocalNavigationState
        # Revoke the old contract's recipients if the wire identity changed;
        # publishing the reset only under the new map/session would be ignored.
        event=LocalNavigationState(schema_version=2,session_id=lease[0] if lease else self.wire_session['id'],
            map_version_id=lease[1] if lease else self.wire_session['version_id'],localization_epoch=max(identity[0],self.core.epoch),
            localization_seed_id=identity[1],usable=False,reason=reason)
        watermark=max(round(self.last_local_state_event*1e9)+1,1)
        # The envelope is an unavailable event; no geometry or renewed sensor
        # timestamp is carried. Current time is used only for a hard reset.
        if changed: watermark=max(watermark,round(now*1e9))
        event.source_stamp.sec,event.source_stamp.nanosec=divmod(watermark,10**9)
        self.local_state_pub.publish(event)
        # This is the existing once-per-episode revocation, not a new gate or
        # a heartbeat. Preserve the cause and original source ages before the
        # wire's intentionally empty unavailable payload reaches consumers.
        self.get_logger().info("LocalStateUnavailable " + json.dumps({
            "at_unix": now, "reason": reason, "event_source_ns": watermark,
            "epoch": event.localization_epoch, "seed": event.localization_seed_id,
            "local_valid": self.core.local_valid, "fault": self.core.fault,
            "issued_source_ns": lease[2] if lease else None,
            "issued_valid_until_ns": lease[3] if lease else None,
            "unavailable_trigger": ('localization_identity_changed' if changed else
                'localization_fault' if self.core.fault else
                'local_measurement_invalid' if not self.core.local_valid else
                'issued_local_state_lease_expired' if lease and now_ns >= lease[3] else
                'local_source_unavailable'),
            "prediction_reason": self.predictor_status.get("reason"),
            "source_timing": self.core.local_timing(now),
            "local_transition": self.core.local_transition,
            "local_rejected_samples": self.core.local_rejected_samples,
        }, allow_nan=False))
        self.last_local_unavailable=token

    def tick(self):
        now = self.now_s()
        self.filter_lifecycle(now)
        self.publish_local_unavailable(now)
        # Watchdog/reset/status only; input callbacks own actual sample delivery.
        # A reset acknowledgement can make an already received pair admissible.
        self.publish_aligned(now)
        pose_status = self.pose_status(now)
        pose_message = String()
        pose_message.data = json.dumps(pose_status, allow_nan=False)
        self.pose_status_pub.publish(pose_message)
        if now - self.last_status < 0.2:
            return
        self.last_status = now
        local_hz = self.rate_of(self.local_arrivals, now)
        global_hz = self.rate_of(self.global_arrivals, now)
        output_fresh = bool(
            self.global_times and fresh(self.global_times[-1], now, self.core.limits.local_timeout)
        )
        valid = output_fresh and pose_status['pose_valid']
        ready = (
            valid
            and not self.core.map_soft_unavailable
            and self.p["extrinsics_verified"]
            and self.p["time_alignment_verified"]
            and 0.8 * self.rate <= global_hz <= 1.2 * self.rate
        )
        fault = self.core.fault or self.core.filter_fault
        alignment_age = (
            now - self.core.alignment_target[0]
            if self.core.alignment_target else None
        )
        alignment_held = bool(
            valid and alignment_age is not None
            and alignment_age > self.core.limits.filter_timeout
        )
        quality = ("fault" if fault else pose_status['quality'] if pose_status['pose_valid'] else
                   "initializing" if not self.core.reset_ack_at else "paused")
        value = {
            "schema": 1,
            "epoch": self.core.epoch,
            "seed_id": self.core.key()[1] if self.core.key() else None,
            "received_at_unix": now,
            "valid": valid,
            "pose_valid": pose_status['pose_valid'],
            "position_output_available": pose_status['pose_valid'],
            "control_output_available": valid,
            "output_stamp_sec": pose_status['output_stamp_sec'],
            "pose_timeout_sec": pose_status['pose_timeout_sec'],
            "frame_id": self.p['map_frame'],
            "navigation_ready": bool(ready),
            "motion_control_enabled": False,
            "state": self.core.reason,
            "quality": quality,
            "output_attempt_reason": self.core.output_attempt_reason,
            "fault": fault,
            "last_error": self.last_error,
            "backend": "faster_lio_imu_prediction_robot_localization",
            "target_hz": self.rate,
            "local_observed_hz": local_hz,
            "global_observed_hz": global_hz,
            "local_timestamp_hz": self.rate_of(self.local_times, now),
            "global_timestamp_hz": self.rate_of(self.global_times, now),
            "global_max_publish_gap_sec": max(
                (b-a for a,b in zip(self.global_arrivals, list(self.global_arrivals)[1:])),
                default=None),
            "local_fresh": self.core.local_ready(now),
            "output_age_sec": now - self.global_times[-1] if self.global_times else None,
            "alignment_age_sec": alignment_age,
            "alignment_stamp_sec": self.core.alignment_target[0] if self.core.alignment_target else None,
            "alignment_held": alignment_held,
            "alignment_mode": "held_correction" if alignment_held else (
                "fresh_correction" if valid else "unavailable"
            ),
            "map_alignment_soft_unavailable": self.core.map_soft_unavailable,
            "output_clock": "local_motion_samples",
            "output_resume_count": self.core.output_resume_count,
            "prediction": self.predictor_status,
            "local_source_timing": self.core.local_timing(now),
            "local_admission": {
                "rejected_samples": self.core.local_rejected_samples,
                "pending_reason": self.core.local_rejection_reason,
                "last_transition": self.core.local_transition,
            },
            "correction_smoothing": {
                "local_translation_residual_m": self.core.local_residual[0],
                "local_rotation_residual_rad": self.core.local_residual[1],
                "translation_residual_m": self.core.map_residual[0],
                "rotation_residual_rad": self.core.map_residual[1],
                "active": max(*self.core.local_residual, *self.core.map_residual) > 0.001,
            },
            "reset_pending": self.future is not None,
            "body_frame": self.p["body_frame"],
            "display_pose_reference": self.p["tracking_frame"],
            "calibration": {
                "extrinsics_verified": self.p["extrinsics_verified"],
                "time_alignment_verified": self.p["time_alignment_verified"],
            },
        }
        message = String()
        message.data = json.dumps(value, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationOutput()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError as error:
            # Humble's SIGINT shutdown can win the race after try_shutdown's
            # context check. Suppress only this verified already-closed case.
            if rclpy.ok() or "rcl_shutdown already called" not in str(error):
                raise
