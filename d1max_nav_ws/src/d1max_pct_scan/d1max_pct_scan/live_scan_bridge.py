"""Live sensor adapter with explicit preview/execution admission, never a controller.

The legacy deskewed-cloud adapter and the explicit per-sensor-ray backend are
mutually exclusive. In ray mode only the native mapper's completed, context-
bound integration receipts renew perception freshness. Neither backend is a
hardware-validated motion permit.
"""
from collections import deque
import json
import math
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from d1max_planning_interfaces.msg import LocalPlanDebug, ReferencePath, TaggedBspline

from .live_scan_contract import (ReferenceGate, admissible_tagged_spline, checked_pose_status_envelope,
                                 cloud_source_issue, decode_xyz,
                                 fresh, localization_context, quaternion_matrix,
                                 pack_xyz, sample_shadow_spline, take_latest_exact, transform_xyz)
from .local_debug import (DEBUG_TIMEOUT, INVALID_PHASES, MAX_DEBUG_POINTS, DebugSnapshot,
                          LocalDebugGate, attempt_marker_contract, local_debug_specs)
from .pct_ground_support import PCTGroundSupport, GroundSupportError
from .perception_status import integrated_ray_stamp

PREFIX = '/d1max/live_planning/'


def seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def xyz(position):
    return [position.x, position.y, position.z]


def quat(rotation):
    return [rotation.x, rotation.y, rotation.z, rotation.w]


def new_reference_message(stamp, now, seen_stamp, accepted_stamp, frame_id, map_frame):
    """Order both path and cancel events from the same authoritative owner.

    An old/duplicate empty Path must not revoke a newer reference. Equal-stamp
    cancel/path events are ambiguous and fail closed rather than depending on
    cross-topic/callback arrival order.
    """
    return (frame_id == map_frame and fresh(stamp, now, 2.)
            and stamp > max(seen_stamp, accepted_stamp))


class LiveScanBridge(Node):
    def __init__(self):
        super().__init__('live_scan_bridge')
        if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
            raise ValueError('live shadow adapter requires unchanged rmw_zenoh_cpp')
        defaults = dict(session_id='', localization_session_id='', map_frame='d1max_loc_map',
                        perception_backend='deskewed_cloud',
                        execution_mode='preview',
                        execution_tracker_node='/d1max/live_planning/motion_coordinator',
                        tracking_frame='d1max_loc_tracking', body_frame='d1max_loc_base_link',
                        body_height=.55, input_timeout=.5, tf_wait_timeout=.25,
                        cloud_rate_hz=10., max_input_points=250000, max_output_points=100000,
                        ground_support_index='', ground_support_sha256='',
                        ground_support_source_pcd_sha256='', ground_support_tomogram_sha256='',
                        ground_support_height_tolerance_m=.20, ground_support_max_step_m=.17)
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.p['localization_session_id'] = self.p['localization_session_id'] or self.p['session_id']
        if (not self.p['session_id'] or len(self.p['session_id']) > 128
                or self.p['perception_backend'] not in ('deskewed_cloud', 'per_sensor_rays')
                or self.p['execution_mode'] not in ('preview', 'execution')
                or not self.p['execution_tracker_node'].startswith('/')
                or not .1 <= self.p['input_timeout'] <= .75
                or not .02 <= self.p['tf_wait_timeout'] <= .4
                or not 1 <= self.p['cloud_rate_hz'] <= 10
                or not .1 <= self.p['body_height'] <= 1
                or not .01 <= self.p['ground_support_height_tolerance_m'] <= .3
                or not .01 <= self.p['ground_support_max_step_m'] <= .25
                or not 100 <= self.p['max_output_points'] <= self.p['max_input_points'] <= 250000
                or len({self.p['map_frame'], self.p['tracking_frame'], self.p['body_frame']}) != 3):
            raise ValueError('invalid_bounded_live_shadow_parameters')
        # No optional bypass: live PCT preview must bind its exact support map.
        self.ground_support = PCTGroundSupport.from_npz(self.p['ground_support_index'],
            expected_sha256=self.p['ground_support_sha256'],
            expected_source_pcd_sha256=self.p['ground_support_source_pcd_sha256'],
            expected_tomogram_sha256=self.p['ground_support_tomogram_sha256'],
            expected_frame=self.p['map_frame'])
        self.ground_support_check = {}
        self.gate = ReferenceGate(self.p['map_frame'], self.p['body_height'])
        self.debug_gate = LocalDebugGate()
        self.navigation, self.localizer, self.pose_status = {}, {}, {}
        self.pose_received = -math.inf
        self.pose_status_stamp = self.navigation_status_stamp = self.localizer_status_stamp = 0.
        self.last_body_stamp = 0.
        self.highest_local_epoch = 0
        self.nav_received = self.localizer_received = self.body_received = self.cloud_received = -math.inf
        self.freeze_received = -math.inf
        self.execution_frozen = False
        self.last_spline_id = -1
        self.last_spline_stamp = 0.
        self.pending_spline_marker = None
        self.pending_spline_message = None
        self.admitted_spline_key = None
        self.admitted_record = None
        self.admission_reason = 'startup_requires_new_target'
        self.admission_sequence = 0
        self.spline_received = -math.inf
        self.visible_spline_id = -1
        self.last_attempt_stamp_ns = 0
        self.attempt_received = -math.inf
        self.attempt_visible_until = -math.inf
        self.attempt_marker_count = 0
        self.attempt_marker_keys = set()
        self.body = None
        self.body_context = None
        self.cloud_stamp = self.last_input_stamp = self.last_output_stamp = 0.
        self.sensor_barrier = 0.
        self.map_context_identity = None
        self.map_context_record = None
        self.map_context_sequence = 0
        self.map_context_ready = False
        self.map_context_last_publish = -math.inf
        # Do not cache/re-stamp a rejected Path. Ask its owning global planner
        # to revalidate the CURRENT goal and publish a new reference instead.
        self.reference_seen_stamp = self.now_s()
        self.reference_refresh = None
        self.reference_refresh_id = 0
        self.reference_refresh_last = -math.inf
        self.last_publish = self.last_status = -math.inf
        self.pending = deque(maxlen=3)
        self.counts = dict(cloud_received=0, cloud_published=0, cloud_dropped=0,
                           cloud_points=0, tf_waits=0, references=0, rejected_references=0,
                           body_published=0, marker_published=0, rejected_splines=0,
                           debug_published=0, debug_cleared=0, debug_rejected=0,
                           reference_refresh_requests=0, reference_refresh_expired=0,
                           attempt_debug_published=0, attempt_debug_rejected=0)
        self.drop_reasons = dict(invalid_source=0, queue_capacity=0, inputs_not_ready=0,
                                 deadline_or_age=0, context_or_barrier=0,
                                 nonmonotonic_output=0, superseded=0, invalid_geometry=0,
                                 sensor_invalidation=0)
        self.invalid_source_reasons = dict(wrong_frame=0, invalid_stale_or_future_stamp=0,
                                           localization_context_unavailable=0,
                                           before_sensor_barrier=0, nonmonotonic_input_stamp=0,
                                           source_size_limit=0)
        # Bounded summaries only: never serialize cloud data in diagnostics.
        self.cloud_timing_ms = dict(decode=0., transform=0., pack=0., publish=0., total=0.)
        self.cloud_processing_samples = deque(maxlen=128)
        self.last_queue_wait_ms = 0.
        self.ray_integration = {}
        self.ray_status_stamp = 0.
        self.error = ''
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Volatile subscriptions never pick up a pre-launch latched path/marker.
        self.cloud_pub = self.create_publisher(PointCloud2, PREFIX+'cloud_map', qos_profile_sensor_data)
        self.sensor_pub = self.create_publisher(Odometry, PREFIX+'sensor_pose', qos_profile_sensor_data)
        self.body_pub = self.create_publisher(Odometry, PREFIX+'body_pose', qos_profile_sensor_data)
        self.reference_pub = self.create_publisher(ReferencePath, PREFIX+'scan_reference', 1)
        self.reference_refresh_pub = self.create_publisher(
            String, PREFIX+'reference_refresh_request', 1)
        self.status_pub = self.create_publisher(String, PREFIX+'scan_bridge_status', 1)
        context_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_context_pub = self.create_publisher(String, PREFIX+'scan_map_context', context_qos)
        self.create_subscription(String, PREFIX+'scan_map_context_ack', self.on_map_context_ack, context_qos)
        self.validated_spline_pub = self.create_publisher(
            TaggedBspline, PREFIX+'validated_tagged_bspline', 1)
        self.admission_pub = self.create_publisher(String, PREFIX+'execution_admission', 1)
        marker_qos = QoSProfile(depth=2, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, PREFIX+'scan_optimal', marker_qos)
        self.debug_pub = self.create_publisher(MarkerArray, PREFIX+'local_debug', marker_qos)
        self.attempt_debug_pub = self.create_publisher(MarkerArray, PREFIX+'local_attempt_debug', marker_qos)
        self.create_subscription(String, '/d1max/localization/navigation/status', self.on_navigation, 5)
        self.create_subscription(String, '/d1max/localization/navigation/pose_status', self.on_pose_status, 5)
        self.create_subscription(String, '/d1max/localization/status', self.on_localizer, 5)
        self.create_subscription(Bool, PREFIX+'execution_frozen', self.on_freeze, 1)
        self.create_subscription(Odometry, '/d1max/localization/odometry/global', self.on_body, qos_profile_sensor_data)
        if self.p['perception_backend'] == 'per_sensor_rays':
            self.create_subscription(String, PREFIX+'rays_status', self.on_native_rays, 5)
        else:
            self.create_subscription(PointCloud2, '/d1max/localization/lio/deskewed', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Path, PREFIX+'reference_path', self.on_reference, 1)
        self.create_subscription(TaggedBspline, PREFIX+'scan_tagged_bspline', self.on_spline, 1)
        self.create_subscription(LocalPlanDebug, PREFIX+'native_local_debug', self.on_local_debug, 2)
        self.create_subscription(MarkerArray, PREFIX+'native_local_attempt_debug', self.on_attempt_debug, 2)
        self.create_timer(.02, self.tick)
        self.clear_outputs('startup_requires_new_target')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def nav_body_ready(self, now, mono):
        context = self.current_context(now, mono=mono)
        execution = self.p.get('execution_mode', 'preview') == 'execution'
        freeze_ready = self.execution_frozen
        if execution:
            # The coordinator owns this feedback while disarmed as well as
            # armed. Duplicate preview/tracker writers invalidate the lease.
            try:
                publishers = self.get_publishers_info_by_topic(PREFIX+'execution_frozen')
                names = [info.node_namespace.rstrip('/')+'/'+info.node_name for info in publishers]
                freeze_ready = names == [self.p['execution_tracker_node']]
            except (RCLError, RuntimeError):
                freeze_ready = False
        return (context is not None and self.map_context_ready
                and self.map_context_identity == context and self.body_context == context
                and 0 <= mono-self.nav_received <= self.p['input_timeout']
                and 0 <= mono-self.localizer_received <= self.p['input_timeout']
                and freeze_ready and 0 <= mono-self.freeze_received <= self.p['input_timeout']
                and self.body is not None and fresh(seconds(self.body.header.stamp), now, self.p['input_timeout'])
                and seconds(self.body.header.stamp) > self.sensor_barrier
                and 0 <= mono-self.body_received <= self.p['input_timeout'])

    def current_context(self, now, *, mono=None):
        try:
            mono = time.monotonic() if mono is None else mono
            if not (0 <= mono-self.nav_received <= self.p['input_timeout']
                    and 0 <= mono-self.localizer_received <= self.p['input_timeout']):
                raise ValueError('localization_status_receipt_stale')
            return localization_context(self.localizer, self.navigation,
                                        pose_status=self.pose_status,
                                        session_id=self.p['localization_session_id'], now=now,
                                        timeout=self.p['input_timeout'], map_frame=self.p['map_frame'],
                                        tracking_frame=self.p['tracking_frame'],
                                        body_frame=self.p['body_frame'],
                                        pose_receipt_age=mono-self.pose_received)
        except ValueError:
            return None

    def update_gate(self):
        now, mono = self.now_s(), time.monotonic()
        context = self.current_context(now)
        self.sync_map_context(context, now, mono)
        ready = (self.nav_body_ready(now, mono)
                 and fresh(self.cloud_stamp, now, self.p['input_timeout'])
                 and 0 <= mono-self.cloud_received <= self.p['input_timeout'])
        # Duplicate geometry can acknowledge a newer owner-issued Path without
        # starting a native generation. Refresh binds to that latest owner
        # stamp; issued_at must remain the actual native generation boundary.
        was_active, old_context, old_stamp = self.gate.active, self.gate.context, self.gate.last_path_stamp
        if self.gate.observe(ready, now, context):
            if (was_active and old_context is not None
                    and self.gate.reason == 'input_stale_or_invalid'):
                # The user goal belongs to the global coordinator. Losing a
                # cloud immediately revokes the local generation, but recovery
                # may ask that owner to revalidate its still-current goal.
                # Explicit cancellation cannot enter here: active is then false.
                self.reference_refresh = dict(context=old_context, source_stamp=old_stamp,
                                              deadline=mono+10., attempts=0)
                self.reference_refresh_last = -math.inf
            elif self.gate.reason == 'localization_context_changed':
                self.reference_refresh = None
            self.clear_outputs(self.gate.reason)
        return self.gate.ready

    def sync_map_context(self, context, now, mono):
        """Require a native map reset acknowledgement before new-context data.

        Loss of a lease alone retains the last identity and historical map.
        Localization session/epoch/seed changes clear it; goal generations do not.
        """
        if context is None:
            return
        if context != self.map_context_identity:
            self.map_context_identity = context
            self.map_context_ready = False
            self.map_context_sequence += 1
            self.reference_refresh = None
            self.gate.revoke(now, 'localization_context_changed')
            self.gate.ready = False
            self.gate.context = context
            self.clear_outputs('localization_context_changed')
            self.map_context_record = dict(schema=1, session_id=context[0], epoch=context[1],
                seed_id=context[2], sequence=self.map_context_sequence,
                barrier_ns=int(math.ceil(self.sensor_barrier*1e9)))
            self.map_context_last_publish = -math.inf
        if not self.map_context_ready and mono-self.map_context_last_publish >= .25:
            self.map_context_pub.publish(String(data=json.dumps(self.map_context_record, sort_keys=True)))
            self.map_context_last_publish = mono

    def on_map_context_ack(self, message):
        try:
            if len(message.data) > 4096:
                return
            value = json.loads(message.data)
            if (self.map_context_record is not None and value == self.map_context_record
                    and self.current_context(self.now_s()) == self.map_context_identity):
                self.map_context_ready = True
                self.update_gate()
        except (ValueError, TypeError):
            return

    def on_freeze(self, message):
        self.execution_frozen, self.freeze_received = message.data is True, time.monotonic()
        self.update_gate()

    def on_localizer(self, message):
        try:
            if len(message.data) > 262144:
                raise ValueError('oversize_localizer_status')
            value = json.loads(message.data)
            if (not isinstance(value, dict)
                    or value.get('session_id') != self.p['localization_session_id']):
                raise ValueError('localizer_session_mismatch')
            stamp = value.get('wall_time')
            if not fresh(stamp, self.now_s(), self.p['input_timeout']):
                raise ValueError('localizer_source_timestamp_invalid')
            epoch = value.get('local_epoch')
            if type(epoch) is not int or epoch < self.highest_local_epoch:
                raise ValueError('localizer_epoch_regressed_within_session')
            # Session is explicit in this source, never invented from a node name.
            if stamp <= self.localizer_status_stamp:
                return
            self.highest_local_epoch = epoch
            self.localizer_status_stamp = stamp
            self.localizer, self.localizer_received = value, time.monotonic()
        except (ValueError, TypeError) as exc:
            self.localizer = {}
            self.error = str(exc)
        self.update_gate()

    def on_navigation(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('oversize_navigation_status')
            value = json.loads(message.data)
            if (not isinstance(value, dict) or value.get('schema') != 1 or type(value.get('valid')) is not bool
                    or type(value.get('epoch')) is not int or value['epoch'] < 1
                    or (value['valid'] and not value.get('seed_id'))):
                raise ValueError('invalid_navigation_status_contract')
            stamp = value.get('received_at_unix')
            if not fresh(stamp, self.now_s(), self.p['input_timeout']):
                raise ValueError('navigation_source_timestamp_invalid')
            if stamp <= self.navigation_status_stamp:
                return
            self.navigation_status_stamp = stamp
            self.navigation = value
            self.nav_received = time.monotonic()
        except (ValueError, TypeError) as exc:
            self.navigation = {}
            self.error = str(exc)
        self.update_gate()

    def on_pose_status(self, message):
        try:
            if len(message.data) > 4096:
                raise ValueError('oversize_continuous_pose_status')
            value = json.loads(message.data)
            stamp = checked_pose_status_envelope(value, now=self.now_s(), timeout=self.p['input_timeout'])
            if (value['frame_id'] != self.p['map_frame']
                    or value['body_frame'] != self.p['body_frame']):
                raise ValueError('continuous_pose_frame_mismatch')
            if stamp <= self.pose_status_stamp:
                return
            self.pose_status_stamp = stamp
            self.pose_status, self.pose_received = value, time.monotonic()
        except (ValueError, TypeError) as exc:
            self.pose_status = {}
            self.error = str(exc)
        self.update_gate()

    def on_body(self, message):
        try:
            stamp = seconds(message.header.stamp)
            if (message.header.frame_id != self.p['map_frame']
                    or message.child_frame_id != self.p['body_frame']
                    or not fresh(stamp, self.now_s(), self.p['input_timeout'])
                    or not np.isfinite(xyz(message.pose.pose.position)).all()
                    or not np.isfinite(xyz(message.twist.twist.linear)+xyz(message.twist.twist.angular)).all()):
                raise ValueError('invalid_body_odometry')
            quaternion_matrix(quat(message.pose.pose.orientation))
            # Odometry has no epoch field. Never relabel a queued pre-reset body
            # sample with the new context, or let an out-of-order sample replace
            # an already accepted newer pose. Ignoring it cannot renew any lease.
            if stamp <= self.sensor_barrier or stamp <= self.last_body_stamp:
                self.error = 'body_before_barrier_or_nonmonotonic'
                return
            self.last_body_stamp = stamp
            self.body, self.body_received = message, time.monotonic()
            self.body_context = self.current_context(self.now_s())
            # Cancel the old generation before exposing a new-epoch body pose.
            self.update_gate()
            if self.nav_body_ready(self.now_s(), self.body_received):
                # Preserve measured child-frame twist and source timestamp verbatim.
                self.body_pub.publish(message)
                self.counts['body_published'] += 1
        except (ValueError, TypeError) as exc:
            self.body = None
            self.error = str(exc)
        self.update_gate()

    def on_native_rays(self, message):
        """Only native completed integrations renew the perception lease."""
        try:
            if len(message.data) > 16384:
                return
            value = json.loads(message.data)
            now, mono = self.now_s(), time.monotonic()
            if not self.map_context_ready or self.current_context(now) != self.map_context_identity:
                return
            stamp = integrated_ray_stamp(value, self.map_context_record, now=now,
                timeout=self.p['input_timeout'], barrier=self.sensor_barrier)
            if value['received_at_unix'] <= self.ray_status_stamp or stamp < self.cloud_stamp:
                return
            self.ray_status_stamp = value['received_at_unix']
            self.ray_integration = value
            self.cloud_stamp = self.last_output_stamp = stamp
            self.cloud_received = mono
            self.error = ''
        except (ValueError, TypeError) as error:
            # A bad/foreign packet cannot replace current geometry or renew a
            # lease. Genuine missing data expires on its original source clock.
            self.error = str(error)[:180]
        self.update_gate()

    def on_cloud(self, message):
        self.counts['cloud_received'] += 1
        stamp = seconds(message.header.stamp)
        now = self.now_s()
        context = self.current_context(now)
        issue = cloud_source_issue(frame_id=message.header.frame_id,
            tracking_frame=self.p['tracking_frame'], stamp=stamp, now=now,
            timeout=self.p['input_timeout'], context=context,
            sensor_barrier=self.sensor_barrier, last_input_stamp=self.last_input_stamp,
            data_size=len(message.data), point_count=int(message.width)*int(message.height),
            max_input_points=self.p['max_input_points'])
        if issue is not None:
            self.drop_clouds('invalid_source')
            self.invalid_source_reasons[issue] += 1
            self.error = 'invalid_cloud_source:' + issue
            return
        self.last_input_stamp = stamp
        if len(self.pending) == self.pending.maxlen:
            self.drop_clouds('queue_capacity')
        self.pending.append((message, time.monotonic(), context))

    def drop_clouds(self, reason, count=1):
        self.counts['cloud_dropped'] += count
        self.drop_reasons[reason] += count

    def cloud_message(self, points, stamp):
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = self.p['map_frame'], stamp
        cloud.height, cloud.width, cloud.point_step = 1, len(points), 12
        cloud.row_step, cloud.is_dense = 12*len(points), True
        cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.data = pack_xyz(points)
        return cloud

    def publish_pending(self, now, mono):
        if not self.nav_body_ready(now, mono):
            self.drop_clouds('inputs_not_ready', len(self.pending))
            self.pending.clear()
            return
        if mono-self.last_publish < 1./self.p['cloud_rate_hz']:
            return
        current_context = self.current_context(now)

        def rejection(item):
            message, received, context = item
            stamp = seconds(message.header.stamp)
            if (mono-received > self.p['tf_wait_timeout']
                    or not fresh(stamp, now, self.p['input_timeout'])):
                return 'deadline_or_age'
            if context != current_context or stamp <= self.sensor_barrier:
                return 'context_or_barrier'
            if stamp <= self.last_output_stamp:
                return 'nonmonotonic_output'
            return None

        def exact_transform(item):
            message = item[0]
            try:
                # Deliberately no Time(0), latest-TF fallback, or timestamp relabeling.
                return self.tf_buffer.lookup_transform(self.p['map_frame'], self.p['tracking_frame'],
                                                       Time.from_msg(message.header.stamp),
                                                       timeout=Duration(seconds=0.))
            except TransformException:
                return None

        selected, tf, dropped, waits = take_latest_exact(
            self.pending, rejection=rejection, resolve=exact_transform)
        self.counts['tf_waits'] += waits
        for reason, count in dropped.items():
            self.drop_clouds(reason, count)
        if dropped.get('deadline_or_age'):
            self.error = 'exact_timestamp_tf_deadline_or_cloud_age'
        if selected is not None:
            message, received, _ = selected
            stamp = seconds(message.header.stamp)
            self.last_queue_wait_ms = max(0., (mono-received)*1000.)
            try:
                started = time.perf_counter()
                points = decode_xyz(data=message.data,
                                    fields=[(f.name, f.offset, f.datatype, f.count) for f in message.fields],
                                    point_step=message.point_step, row_step=message.row_step,
                                    width=message.width, height=message.height, bigendian=message.is_bigendian,
                                    max_input_points=self.p['max_input_points'],
                                    max_output_points=self.p['max_output_points'])
                decoded = time.perf_counter()
                points = transform_xyz(points, xyz(tf.transform.translation), quat(tf.transform.rotation))
                transformed = time.perf_counter()
                cloud = self.cloud_message(points, message.header.stamp)
                packed = time.perf_counter()
                sensor = Odometry()
                sensor.header.frame_id, sensor.header.stamp = self.p['map_frame'], message.header.stamp
                sensor.child_frame_id = self.p['tracking_frame']
                sensor.pose.pose.position.x, sensor.pose.pose.position.y, sensor.pose.pose.position.z = xyz(tf.transform.translation)
                sensor.pose.pose.orientation = tf.transform.rotation
                self.sensor_pub.publish(sensor)
                self.cloud_pub.publish(cloud)
                published = time.perf_counter()
                self.cloud_timing_ms = dict(decode=(decoded-started)*1000.,
                    transform=(transformed-decoded)*1000., pack=(packed-transformed)*1000.,
                    publish=(published-packed)*1000., total=(published-started)*1000.)
                self.cloud_processing_samples.append(self.cloud_timing_ms['total'])
                self.cloud_stamp = self.last_output_stamp = stamp
                self.cloud_received, self.last_publish = mono, mono
                self.counts['cloud_published'] += 1
                self.counts['cloud_points'] = len(points)
                self.error = ''
            except (ValueError, TypeError) as exc:
                self.drop_clouds('invalid_geometry')
                self.error = str(exc)
            return

    def on_reference(self, message):
        was_ready = self.gate.ready
        self.update_gate()
        try:
            stamp, now = seconds(message.header.stamp), self.now_s()
            if not new_reference_message(stamp, now, self.reference_seen_stamp,
                                         self.gate.last_path_stamp,
                                         message.header.frame_id, self.p['map_frame']):
                # Stale/duplicate cancels are ignored, never allowed to clear
                # a newer active target or refresh handshake.
                return
            if not message.poses:
                self.reference_refresh = None
                self.reference_seen_stamp = stamp
                # New-goal handover cancels only the old route. Sensor readiness
                # and its epoch barrier must not oscillate while PCT computes.
                self.gate.revoke(self.now_s(), 'explicit_reference_cancel', advance_barrier=False)
                self.clear_outputs(self.gate.reason, clear_sensors=False)
                return
            if len(message.poses) > 20000 or any(p.header.frame_id not in ('', self.p['map_frame']) for p in message.poses):
                raise ValueError('reference_pose_frame_or_size_invalid')
            points = [xyz(p.pose.position) for p in message.poses]
            # A path published while the sensor gate was down can race the
            # ready edge, which raises the reference barrier. In either case
            # request a fresh owner-issued path, never relax that barrier.
            if (not self.gate.ready or not was_ready or stamp <= self.gate.barrier):
                self.arm_reference_refresh(points, message.header.frame_id, stamp, now)
            if not self.gate.accept(points,
                                    frame_id=message.header.frame_id,
                                    stamp=stamp, now=now,
                                    body_xyz=xyz(self.body.pose.pose.position) if self.body else []):
                self.reference_seen_stamp = stamp
                self.reference_refresh = None
                return
            self.reference_seen_stamp = max(self.reference_seen_stamp, stamp)
            self.reference_refresh = None
            typed = ReferencePath()
            typed.session_id, typed.generation, typed.path = self.p['session_id'], self.gate.generation, message
            self.clear_marker()
            self.clear_attempt_debug()
            self.reference_pub.publish(typed)
            self.counts['references'] += 1
        except (ValueError, TypeError) as exc:
            self.counts['rejected_references'] += 1
            self.error = str(exc)
            # A delayed/duplicated old publication cannot cancel a newer task,
            # and cannot resume an inactive one. Invalid NEW geometry does revoke.
            if self.error == 'obsolete_or_stale_reference':
                return
            if self.error != 'inputs_not_ready_requires_new_target':
                self.reference_refresh = None
            # Any rejected reference revokes; never silently continue an older one.
            self.gate.revoke(self.now_s(), self.error, advance_barrier=False)
            self.clear_outputs(self.error, clear_sensors=False)

    def arm_reference_refresh(self, points, frame_id, stamp, now):
        """Remember only an unaccepted request identity, not reusable geometry."""
        context = self.current_context(now)
        if (context is None or not fresh(stamp, now, 2.)
                or stamp <= max(self.reference_seen_stamp, self.gate.last_path_stamp)
                or frame_id != self.p['map_frame']):
            return
        values = np.asarray(points, dtype=float)
        if (values.ndim != 2 or values.shape[1] != 3
                or not 2 <= len(values) <= 20000 or not np.isfinite(values).all()):
            return
        steps = np.linalg.norm(np.diff(values, axis=0), axis=1)
        if steps.max() > 1. or not .05 <= steps.sum() <= 2000.:
            return
        self.reference_seen_stamp = stamp
        self.reference_refresh = dict(context=context, source_stamp=stamp,
                                      deadline=time.monotonic()+10., attempts=0)
        self.reference_refresh_last = -math.inf

    def request_reference_refresh(self, now, mono):
        pending = self.reference_refresh
        if pending is None:
            return
        expected = pending['context']
        # A hard identity change/fault always drops the handshake. Merely
        # lacking a fresh sample can wait within the fixed 10-second deadline.
        if (self.localizer.get('session_id') != expected[0]
                or self.localizer.get('local_epoch') != expected[1]
                or self.localizer.get('active_seed_ns') != expected[2]
                or self.localizer.get('confirmed_seed_ns') != expected[2]
                or self.localizer.get('local_fault') or self.navigation.get('fault')
                or (type(self.navigation.get('epoch')) is int
                    and self.navigation['epoch'] > 0 and self.navigation['epoch'] != expected[1])
                or (isinstance(self.navigation.get('seed_id'), str)
                    and self.navigation['seed_id'] and self.navigation['seed_id'] != expected[2])
                or not self.execution_frozen):
            self.reference_refresh = None
            return
        if mono >= pending['deadline'] or pending['attempts'] >= 20:
            self.reference_refresh = None
            self.counts['reference_refresh_expired'] += 1
            return
        if (not self.gate.ready or self.gate.active or self.current_context(now) != expected
                or mono-self.reference_refresh_last < .5):
            return
        self.reference_refresh_id += 1
        request = dict(schema=1, session_id=self.p['session_id'],
            localization_epoch=expected[1], localization_seed_id=expected[2],
            received_at_unix=now, request_id=self.reference_refresh_id,
            rejected_reference_stamp=pending['source_stamp'], motion_enabled=False)
        self.reference_refresh_pub.publish(String(data=json.dumps(request, allow_nan=False)))
        self.reference_refresh_last = mono
        pending['attempts'] += 1
        self.counts['reference_refresh_requests'] += 1

    def on_spline(self, message):
        self.update_gate()
        raw, stamp = message.trajectory, seconds(message.trajectory.start_time)
        if not admissible_tagged_spline(session_id=message.session_id, generation=message.generation,
                                       frame_id=message.frame_id, trajectory_id=raw.traj_id,
                                       start_time=stamp, expected_session=self.p['session_id'],
                                       gate=self.gate, last_id=self.last_spline_id, now=self.now_s()):
            self.counts['rejected_splines'] += 1
            self.publish_execution_admission()
            return
        try:
            if len(raw.pos_pts) > 10000 or len(raw.knots) > 10004:
                raise ValueError('native_spline_message_budget_exceeded')
            points = sample_shadow_spline(order=raw.order, knots=raw.knots,
                                          points=[xyz(point) for point in raw.pos_pts])
            started = time.perf_counter()
            try:
                support = self.ground_support.validate_body_samples(points,
                    body_height_m=self.p['body_height'],
                    height_tolerance_m=self.p['ground_support_height_tolerance_m'],
                    max_ground_step_m=self.p['ground_support_max_step_m'])
            except GroundSupportError as exc:
                self.ground_support_check = dict(valid=False, reason=str(exc),
                    generation=int(message.generation), plan_id=int(raw.traj_id),
                    source_stamp=stamp, elapsed_ms=(time.perf_counter()-started)*1000.)
                raise
            self.ground_support_check = dict(support, valid=True,
                generation=int(message.generation), plan_id=int(raw.traj_id),
                source_stamp=stamp, elapsed_ms=(time.perf_counter()-started)*1000.)
        except (ValueError, TypeError) as exc:
            self.counts['rejected_splines'] += 1
            self.error = str(exc)
            # An invalid geometry with a current tag consumes its identity;
            # duplicate packets cannot keep re-running expensive map admission.
            self.last_spline_id = max(self.last_spline_id, raw.traj_id)
            self.clear_marker(reset_order=False, clear_debug=False)
            self.clear_debug('invalid_spline')
            return
        # Map admission does not extend the sensor/identity lifetime consumed
        # during its computation, even if a large curve took longer to check.
        self.update_gate()
        if not admissible_tagged_spline(session_id=message.session_id, generation=message.generation,
                frame_id=message.frame_id, trajectory_id=raw.traj_id, start_time=stamp,
                expected_session=self.p['session_id'], gate=self.gate,
                last_id=self.last_spline_id, now=self.now_s()):
            self.counts['rejected_splines'] += 1
            return
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = message.frame_id, raw.start_time
        marker.ns, marker.id = 'scan_shadow_not_execution_authority', 0
        marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x = .035
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .12, .95, .25, 1.
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points]
        # TaggedBspline also carries emergency-stop/hover splines. A tag alone
        # is therefore not evidence of a successful native optimization. Keep
        # one bounded candidate until its exact accepted debug arrives.
        self.delete_spline_marker()
        self.pending_spline_marker = marker
        self.pending_spline_message = message
        self.spline_received = time.monotonic()
        self.last_spline_id, self.last_spline_stamp = raw.traj_id, stamp
        action = self.debug_gate.pair(spline_id=self.last_spline_id,
            spline_stamp=self.last_spline_stamp, now=self.now_s(), mono=time.monotonic())
        if action == 'draw':
            self.publish_debug()
        elif self.debug_gate.active is None:
            self.publish_debug_delete()
        self.publish_execution_admission()

    def on_local_debug(self, message):
        self.update_gate()
        path, stamp = message.selected_reference, message.header.stamp
        bounded = len(path.poses) <= MAX_DEBUG_POINTS
        headers_match = (bounded and path.header.frame_id == message.header.frame_id
                         and path.header.stamp == stamp and all(
                             p.header.frame_id == message.header.frame_id and p.header.stamp == stamp
                             for p in path.poses))
        snapshot = DebugSnapshot(session_id=message.session_id, generation=message.generation,
            plan_id=message.plan_id, frame_id=message.header.frame_id, stamp=seconds(stamp),
            stamp_ns=stamp.sec*1000000000+stamp.nanosec, valid=message.valid, phase=message.phase,
            selected_reference=[xyz(p.pose.position) for p in path.poses] if bounded else [],
            projection=xyz(message.projection), local_target=xyz(message.local_target),
            progress_arc_m=message.progress_arc_m, target_arc_m=message.target_arc_m,
            matching_path_headers=headers_match,
            predecessor_id=message.predecessor_id, predecessor_safe=message.predecessor_safe,
            predecessor_check_stamp=seconds(message.predecessor_check_stamp))
        record = self.admitted_record
        revoke_reason = None
        if (record is not None and not message.valid and self.gate.ready and self.gate.active
                and message.session_id == self.p['session_id']
                and message.generation == self.gate.generation
                and message.header.frame_id == self.p['map_frame']
                and message.plan_id >= record['message'].trajectory.traj_id
                and snapshot.stamp_ns > record['debug'].stamp_ns
                and fresh(snapshot.stamp, self.now_s(), DEBUG_TIMEOUT)):
            # A failure of the still-executing old plan matters even if a newer
            # accepted diagnostic is waiting for its own spline on another topic.
            revoke_reason = ('native_'+message.phase if message.phase in INVALID_PHASES
                             else 'invalid_debug')
        action = self.debug_gate.receive(snapshot, expected_session=self.p['session_id'],
            gate=self.gate, now=self.now_s(), mono=time.monotonic(),
            spline_id=self.last_spline_id, spline_stamp=self.last_spline_stamp)
        if action == 'ignore':
            self.counts['debug_rejected'] += 1
        elif action == 'draw':
            self.publish_debug()
        else:
            # A fresh accepted diagnostic awaiting its spline is not failure.
            # Keep the previous admitted record within its original lease.
            self.publish_debug_delete()
            self.delete_spline_marker()
            if not message.valid or self.debug_gate.phase == 'invalid_debug':
                # Clear display/execution admission, while retaining the
                # reference so native planning can report a later valid replan.
                # Publish only the semantic terminal reason, after clearing
                # graphics; depth-one consumers must not observe a generic
                # marker-clear reason instead of native completion/failure.
                phase = self.debug_gate.phase
                revoke_reason = revoke_reason or ('native_'+phase if phase in INVALID_PHASES else phase)
                self.last_spline_id = max(self.last_spline_id, self.debug_gate.highest_plan_id)
                self.clear_marker(reset_order=False, clear_debug=False, reason=revoke_reason)
                revoke_reason = None
        if revoke_reason is not None:
            # A rejected old executing plan can be ignored by the diagnostic
            # high-water mark while a newer accepted debug awaits its spline.
            self.revoke_execution(revoke_reason, clear_candidate=False)
        self.publish_execution_admission()

    def on_attempt_debug(self, message):
        self.update_gate()
        if not self.gate.ready or not self.gate.active:
            self.counts['attempt_debug_rejected'] += 1
            return
        try:
            stamp_ns, lifetime, count = attempt_marker_contract(message.markers,
                session_id=self.p['session_id'], generation=self.gate.generation,
                frame_id=self.p['map_frame'], issued_at=self.gate.issued_at, now=self.now_s())
            if stamp_ns <= self.last_attempt_stamp_ns:
                raise ValueError('obsolete_attempt_diagnostic')
            # Native diagnostics are a complete snapshot prefixed by DELETEALL.
            # RViz need not destroy/recreate every object for a same-task refresh:
            # overwrite stable IDs and explicitly delete only absent kinds. This
            # does NOT retain an old trajectory, extend source time or its TTL.
            additions = [marker for marker in message.markers if marker.action == Marker.ADD]
            keys = {(marker.ns, marker.id) for marker in additions}
            removals = []
            for namespace, identifier in sorted(self.attempt_marker_keys-keys):
                marker = Marker()
                marker.header = message.markers[0].header
                marker.ns, marker.id, marker.action = namespace, identifier, Marker.DELETE
                removals.append(marker)
            for marker in additions:
                marker.lifetime = Duration(seconds=lifetime).to_msg()
            self.attempt_debug_pub.publish(MarkerArray(markers=removals+additions))
            self.attempt_marker_keys = keys
            self.last_attempt_stamp_ns = stamp_ns
            self.attempt_received = time.monotonic()
            self.attempt_visible_until = self.attempt_received+lifetime
            self.attempt_marker_count = count
            self.counts['attempt_debug_published'] += 1
        except (ValueError, TypeError, AttributeError) as exc:
            self.counts['attempt_debug_rejected'] += 1
            self.error = str(exc)

    def clear_attempt_debug(self):
        marker = Marker()
        marker.header.frame_id = self.p['map_frame']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        self.attempt_debug_pub.publish(MarkerArray(markers=[marker]))
        self.attempt_marker_count = 0
        self.attempt_marker_keys.clear()
        self.attempt_visible_until = -math.inf

    def publish_debug(self):
        snapshot = self.debug_gate.active
        if snapshot is None:
            return
        support = self.ground_support_check
        if (support.get('valid') is not True or support.get('plan_id') != snapshot.plan_id
                or support.get('generation') != self.gate.generation
                or support.get('source_stamp') != self.last_spline_stamp):
            self.revoke_execution('ground_support_pair_invalid')
            self.delete_spline_marker()
            self.publish_debug_delete()
            return
        remaining = min(DEBUG_TIMEOUT, DEBUG_TIMEOUT-max(0., self.now_s()-snapshot.stamp),
                        DEBUG_TIMEOUT-max(0., self.now_s()-self.last_spline_stamp),
                        DEBUG_TIMEOUT-max(0., time.monotonic()-self.spline_received))
        if (remaining <= 0 or snapshot.plan_id != self.last_spline_id
                or self.pending_spline_marker is None or not self.gate.ready or not self.gate.active):
            self.delete_spline_marker()
            self.clear_debug('expired_or_unpaired')
            return
        # Draw green and native reference/target together only after pairing.
        # Preserve the spline's real start_time; never freshen an old trajectory.
        self.admit_execution()
        marker = self.pending_spline_marker
        marker.lifetime = Duration(seconds=remaining).to_msg()
        self.marker_pub.publish(marker)
        self.visible_spline_id = snapshot.plan_id
        self.counts['marker_published'] += 1
        stamp = Time(nanoseconds=snapshot.stamp_ns).to_msg()
        clear = Marker()
        clear.header.frame_id, clear.header.stamp, clear.action = snapshot.frame_id, stamp, Marker.DELETEALL
        markers = [clear]
        for spec in local_debug_specs(selected_reference=snapshot.selected_reference,
                projection=snapshot.projection, local_target=snapshot.local_target):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = snapshot.frame_id, stamp
            marker.ns, marker.id = spec['namespace'], 0
            marker.type, marker.action = getattr(Marker, spec['kind']), Marker.ADD
            marker.pose.orientation.w = 1.
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec['color']
            marker.scale.x = spec['width']
            if spec['kind'] in ('POINTS', 'SPHERE'):
                marker.scale.y = spec['width']
            if spec['kind'] == 'SPHERE':
                marker.scale.z = spec['width']
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec['position']
            else:
                marker.points = [Point(x=float(x), y=float(y), z=float(z))
                                 for x, y, z in spec['points']]
            marker.lifetime = Duration(seconds=remaining).to_msg()
            markers.append(marker)
        self.debug_pub.publish(MarkerArray(markers=markers))
        self.counts['debug_published'] += 1

    def publish_debug_delete(self):
        marker = Marker()
        marker.header.frame_id = self.p['map_frame']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        self.debug_pub.publish(MarkerArray(markers=[marker]))
        self.counts['debug_cleared'] += 1

    def clear_debug(self, phase='inactive'):
        self.revoke_execution(phase)
        if self.debug_gate.generation != self.gate.generation:
            self.debug_gate.reset(self.gate.generation)
        self.debug_gate.clear(phase)
        self.publish_debug_delete()

    def delete_spline_marker(self):
        marker = Marker()
        marker.header.frame_id = self.p['map_frame']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        self.visible_spline_id = -1

    def clear_marker(self, *, reset_order=True, clear_debug=True, reason='trajectory_cleared'):
        self.delete_spline_marker()
        if reset_order:
            self.last_spline_id = -1
        self.last_spline_stamp = 0.
        self.pending_spline_marker = None
        self.spline_received = -math.inf
        if clear_debug:
            self.clear_debug()
        self.revoke_execution(reason)

    def clear_outputs(self, reason, *, clear_sensors=True):
        self.ground_support_check = {}
        if clear_sensors:
            self.drop_clouds('sensor_invalidation', len(self.pending))
            self.pending.clear()
            # Cover every old-context sample already received or published,
            # including the permitted small future timestamp skew. Otherwise
            # a delayed old cloud/pose can arrive after the context-reset ACK
            # with a stamp beyond a merely "now" barrier and refill the map.
            # Round outward by one float ULP before converting to ROS ns, so
            # Unix-time float rounding cannot leave a sub-microsecond gap.
            self.sensor_barrier = math.nextafter(max(self.sensor_barrier, self.now_s(),
                self.last_output_stamp, self.last_input_stamp), math.inf)
            self.cloud_stamp = 0.
            self.cloud_received = -math.inf
        typed = ReferencePath()
        typed.session_id, typed.generation = self.p['session_id'], self.gate.generation
        typed.path.header.frame_id = self.p['map_frame']
        typed.path.header.stamp = self.get_clock().now().to_msg()
        self.reference_pub.publish(typed)
        self.clear_marker()
        self.clear_attempt_debug()
        self.revoke_execution(reason)
        if clear_sensors and self.p.get('perception_backend', 'deskewed_cloud') == 'deskewed_cloud':
            self.cloud_pub.publish(self.cloud_message(np.empty((0, 3)), typed.path.header.stamp))
        self.get_logger().info('SCAN reference/admission cleared: '+reason)

    def execution_pair_valid(self):
        """Admission is a bounded evidence lease, never an SDK motion permit."""
        message = self.pending_spline_message
        debug = self.debug_gate.active
        support = self.ground_support_check
        if message is None or debug is None:
            return False
        raw = message.trajectory
        return (self.p.get('execution_mode', 'preview') == 'execution'
                and self.gate.ready and self.gate.active
                and message.session_id == self.p['session_id']
                and message.generation == self.gate.generation
                and message.frame_id == self.p['map_frame']
                and raw.traj_id == self.last_spline_id == debug.plan_id
                and debug.session_id == message.session_id and debug.generation == message.generation
                and debug.valid and debug.phase == 'accepted'
                and support.get('valid') is True
                and support.get('generation') == message.generation
                and support.get('plan_id') == raw.traj_id
                and support.get('source_stamp') == seconds(raw.start_time) == self.last_spline_stamp
                and seconds(raw.start_time) >= self.gate.issued_at
                and fresh(self.last_spline_stamp, self.now_s(), DEBUG_TIMEOUT)
                and fresh(debug.stamp, self.now_s(), DEBUG_TIMEOUT)
                and 0 <= time.monotonic()-self.spline_received <= DEBUG_TIMEOUT
                and 0 <= time.monotonic()-self.debug_gate.received_mono <= DEBUG_TIMEOUT)

    def revoke_execution(self, reason, *, clear_candidate=True):
        self.admitted_spline_key = None
        self.admitted_record = None
        self.admission_reason = reason
        if clear_candidate:
            self.pending_spline_message = None
        self.publish_execution_admission()

    def admit_execution(self):
        if not self.execution_pair_valid():
            return
        message = self.pending_spline_message
        key = (message.generation, message.trajectory.traj_id)
        if key == self.admitted_spline_key:
            return  # Never re-stamp or re-publish a previously accepted curve.
        self.admitted_spline_key = key
        self.admitted_record = dict(message=message, debug=self.debug_gate.active,
            spline_received=self.spline_received, debug_received=self.debug_gate.received_mono,
            context=self.gate.context)
        self.admission_reason = 'accepted'
        self.validated_spline_pub.publish(message)
        self.publish_execution_admission()

    def execution_admission_valid(self):
        record = self.admitted_record
        if record is None:
            return False
        message, debug = record['message'], record['debug']
        raw = message.trajectory
        now, mono = self.now_s(), time.monotonic()
        return (self.p.get('execution_mode', 'preview') == 'execution'
                and self.gate.ready and self.gate.active and record['context'] == self.gate.context
                and message.session_id == self.p['session_id']
                and message.generation == self.gate.generation
                and message.frame_id == self.p['map_frame']
                and self.admitted_spline_key == (message.generation, raw.traj_id)
                and seconds(raw.start_time) >= self.gate.issued_at
                and fresh(seconds(raw.start_time), now, DEBUG_TIMEOUT)
                and fresh(debug.stamp, now, DEBUG_TIMEOUT)
                and 0 <= mono-record['spline_received'] <= DEBUG_TIMEOUT
                and 0 <= mono-record['debug_received'] <= DEBUG_TIMEOUT)

    def publish_execution_admission(self):
        valid = self.execution_admission_valid()
        if not valid and self.admitted_spline_key is not None:
            self.admitted_spline_key = None
            self.admitted_record = None
            self.admission_reason = 'admission_expired_or_context_changed'
        self.admission_sequence += 1
        record = self.admitted_record
        debug = record['debug'] if valid else self.debug_gate.active
        raw = record['message'].trajectory if valid else None
        # A newer native candidate may have been produced by a safety replan.
        # Retaining the old lease does not itself prove that old curve is safe.
        # Tell the coordinator to output zero while the exact replacement pair
        # is incomplete; only its eventual accepted record can carry a recheck.
        pending_candidate_id = 0
        if valid:
            candidate = self.pending_spline_message
            pending_debug = self.debug_gate.pending
            if (candidate is not None and candidate.generation == self.gate.generation
                    and candidate.trajectory.traj_id > raw.traj_id):
                pending_candidate_id = candidate.trajectory.traj_id
            if (pending_debug is not None and pending_debug.generation == self.gate.generation
                    and pending_debug.plan_id > raw.traj_id):
                pending_candidate_id = max(pending_candidate_id, pending_debug.plan_id)
        payload = dict(schema=1, session_id=self.p['session_id'],
            execution_mode=self.p.get('execution_mode', 'preview'),
            generation=self.gate.generation,
            trajectory_id=raw.traj_id if raw is not None else self.last_spline_id,
            valid=valid, reason=self.admission_reason,
            handover_pending=pending_candidate_id > 0,
            pending_candidate_id=pending_candidate_id,
            sequence=self.admission_sequence, frame_id=self.p['map_frame'],
            issued_at=self.gate.issued_at,
            source_stamp=seconds(raw.start_time) if raw is not None else self.last_spline_stamp,
            debug_stamp=debug.stamp if debug is not None else None,
            predecessor_id=debug.predecessor_id if valid else 0,
            predecessor_safe=bool(valid and debug.predecessor_safe
                and 0 < debug.predecessor_id < raw.traj_id
                and self.gate.issued_at <= debug.predecessor_check_stamp <= debug.stamp),
            predecessor_check_stamp=debug.predecessor_check_stamp if valid else 0.,
            stamp=self.now_s(), received_at_unix=time.time(), lease_timeout_sec=.35,
            localization_context=list(self.gate.context) if self.gate.context else None,
            motion_authorized=False)
        self.admission_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def tick(self):
        now, mono = self.now_s(), time.monotonic()
        self.publish_pending(now, mono)
        self.update_gate()
        self.request_reference_refresh(now, mono)
        if self.last_spline_stamp > 0 and not fresh(self.last_spline_stamp, now, DEBUG_TIMEOUT):
            self.clear_marker(reset_order=False, clear_debug=False)
            self.clear_debug('expired')
        if self.debug_gate.expire(now=now, mono=mono, gate=self.gate):
            self.revoke_execution('native_acceptance_expired')
            self.publish_debug_delete()
            self.delete_spline_marker()
        if self.attempt_marker_count and (
                not self.gate.ready or not self.gate.active or mono >= self.attempt_visible_until
                or not fresh(self.last_attempt_stamp_ns*1e-9, now, DEBUG_TIMEOUT)):
            self.clear_attempt_debug()
        if mono-self.last_status < .2:
            return
        self.last_status = mono
        self.publish_execution_admission()
        age = lambda stamp: now-stamp if stamp > 0 and math.isfinite(stamp) else None
        debug = self.debug_gate.active
        support = self.ground_support_check
        support_failed = (support.get('valid') is False
            and support.get('generation') == self.gate.generation
            and support.get('plan_id', -1) >= self.debug_gate.highest_plan_id
            and fresh(support.get('source_stamp', 0.), now, DEBUG_TIMEOUT))
        status = dict(schema=1, mode='LIVE_SHADOW_NO_MOTION', session_id=self.p['session_id'],
                      execution_mode=self.p['execution_mode'],
                      received_at_unix=now, frame_id=self.p['map_frame'], ready=self.gate.ready,
                      sensor_ready=self.gate.ready,
                      native_map_context_ready=self.map_context_ready,
                      native_map_context_sequence=self.map_context_sequence,
                      localization_session_id=self.p['localization_session_id'],
                      localization_epoch=self.navigation.get('epoch'),
                      localization_seed_id=self.navigation.get('seed_id'),
                      active_reference=self.gate.active, generation=self.gate.generation,
                      reference_stamp=self.gate.issued_at,
                      owner_reference_stamp=self.gate.last_path_stamp if self.gate.active else None,
                      reference_digest=self.gate.digest, reason=self.gate.reason, last_error=self.error,
                      reference_refresh_pending=self.reference_refresh is not None,
                      reference_refresh_request_id=self.reference_refresh_id,
                      motion_enabled=False, sdk_connected=False, hardware_validated=False,
                      execution_frozen=self.execution_frozen,
                      execution_freeze_fresh=0 <= mono-self.freeze_received <= self.p['input_timeout'],
                      spline_geometry_source='TaggedBspline_session_generation_verified_visualization_only',
                      last_spline_id=self.last_spline_id, last_spline_stamp=self.last_spline_stamp,
                      spline_visual_valid=(self.gate.ready and self.gate.active
                                           and self.visible_spline_id == self.last_spline_id
                                           and debug is not None and debug.plan_id == self.last_spline_id
                                           and fresh(self.last_spline_stamp, now, DEBUG_TIMEOUT)),
                      spline_waiting_for_native_acceptance=(self.pending_spline_marker is not None
                                                            and self.visible_spline_id != self.last_spline_id),
                      local_debug_valid=debug is not None,
                      local_debug_phase='failed_ground_support' if support_failed else self.debug_gate.phase,
                      ground_support_check=support,
                      local_debug_plan_id=debug.plan_id if debug else self.debug_gate.highest_plan_id,
                      local_debug_target=debug.local_target if debug else None,
                      local_debug_progress_arc_m=debug.progress_arc_m if debug else None,
                      local_debug_target_arc_m=debug.target_arc_m if debug else None,
                      local_debug_reference_points=len(debug.selected_reference) if debug else 0,
                      local_attempt_diagnostic_visible=self.attempt_marker_count > 0,
                      local_attempt_diagnostic_markers=self.attempt_marker_count,
                      local_attempt_diagnostic_stamp=self.last_attempt_stamp_ns*1e-9
                          if self.attempt_marker_count else None,
                      perception_backend=self.p.get('perception_backend', 'deskewed_cloud'),
                      ray_integration=getattr(self, 'ray_integration', {}),
                      ray_origin=('per_point_time_and_per_sensor_origin'
                          if self.p.get('perception_backend') == 'per_sensor_rays'
                          else 'tracking_origin_approximation_for_merged_dual_lidar'),
                      exact_cloud_stamp_tf=True, cloud_age=age(self.cloud_stamp),
                      body_age=age(seconds(self.body.header.stamp)) if self.body else None,
                      localization_navigation_ready=self.navigation.get('navigation_ready', False),
                      pending_clouds=len(self.pending), pending_capacity=self.pending.maxlen,
                      cloud_queue_policy=('per_sensor_latest_completed_integration'
                          if self.p.get('perception_backend') == 'per_sensor_rays'
                          else 'latest_available_exact_timestamp_tf'),
                      cloud_processing_ms=self.cloud_timing_ms,
                      cloud_processing_p50_ms=float(np.percentile(self.cloud_processing_samples, 50))
                          if self.cloud_processing_samples else None,
                      cloud_processing_p95_ms=float(np.percentile(self.cloud_processing_samples, 95))
                          if self.cloud_processing_samples else None,
                      cloud_queue_wait_ms=self.last_queue_wait_ms,
                      cloud_drop_reasons=self.drop_reasons,
                      invalid_source_reasons=self.invalid_source_reasons,
                      counters=self.counts)
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = LiveScanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.gate.revoke(node.now_s(), 'shadow_adapter_shutdown')
            node.clear_outputs(node.gate.reason)
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError:
            # Humble SIGINT may close the context between try_shutdown's check
            # and rcl_shutdown. Other errors in a live context still propagate.
            if rclpy.ok():
                raise


if __name__ == '__main__':
    main()
