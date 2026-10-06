"""Optional per-sensor metadata -> motion-compensated map rays boundary.

No SDK, command publishers, planner calls, ground clearing or startup authority.
The live session supervisor is the only owner of this independent process.
"""
import argparse
from array import array
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time

from .ray_projection import (AwaitingCoverage, MapContext, ProjectionError, PreviewExcludedScan,
                             RayProjectorCore, checked_pose, decode_rays, session_settings)

PREFIX = '/d1max/live_planning/'


def create_node(settings):
    # Importing configuration or the pure geometry module never creates a ROS context.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from d1max_planning_interfaces.msg import ProjectedRays, NavigationState, LocalNavigationState
    from .live_scan_contract import (localization_identity_context, fresh,
                                    checked_pose_status_envelope, validate_continuous_pose)

    def stamp_ns(stamp):
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    def pose(value):
        return checked_pose((value.position.x, value.position.y, value.position.z),
                            (value.orientation.x, value.orientation.y, value.orientation.z,
                             value.orientation.w))

    def assign_stamp(target, value):
        target.sec, target.nanosec = divmod(int(value), 1000000000)

    class PerceptionRayProjector(Node):
        def __init__(self):
            super().__init__('d1max_perception_ray_projector')
            self.p = settings
            self.atomic_state = None
            self.atomic_received = -math.inf
            from .local_navigation_state import LocalNavigationInbox
            self.local_inbox = LocalNavigationInbox()
            self.core = RayProjectorCore(settings['limits'], settings.get('preview_exclusion'),
                projection_frame=settings.get('projection_frame', 'map'),
                allow_simulation_snapshot=settings.get('allow_simulation_snapshot', False))
            self.exclusion_counts = {'matched': [0, 0], 'dropped': [0, 0]}
            self.request = self.ack = None
            self.highest_context_sequence = 0
            self.statuses = {'localizer': {}, 'navigation': {}, 'pose': {}}
            self.status_received = {key: -math.inf for key in self.statuses}
            self.status_stamps = {key: 0. for key in self.statuses}
            self.pose_status_fault = False
            self.authorized_pose_context = None
            self.authorized_pose_ns = 0
            self.static_edges = {}
            self.static_retry_count = 0
            self.last_static_retry = time.monotonic()
            self.static_retry_delay = 1.
            self.pending = [None, None]
            self.last_raw_stamp = [0, 0]
            self.last_published = [0, 0]
            self.last_status = -math.inf
            self.last_fault_status = None
            self.next_sensor = 0
            self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ray_geometry')
            self.inflight = None
            self.work_generation = 0
            self.error = 'waiting_localization_context'
            self.counts = dict(received=0, published=0, dropped=0, overwritten=0,
                               pose_rejected=0, context_resets=0)
            self.processing_ms = 0.
            transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(ProjectedRays, self.p['output_topic'],
                                                   qos_profile_sensor_data)
            self.status_pub = self.create_publisher(String, PREFIX+'ray_projector_status', 1)
            self.create_subscription(String, PREFIX+'scan_map_context', self.on_context, transient)
            self.create_subscription(String, PREFIX+'scan_map_context_ack', self.on_ack, transient)
            # Multiple independent static publishers can arrive together at
            # startup. A context's depth-one queue is not sufficient for TF.
            self.static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.static_subscription = self.create_subscription(
                TFMessage, '/tf_static', self.on_static, self.static_qos)
            self.create_subscription(PointCloud2, self.p['raw_topic'], self.on_rays,
                                     qos_profile_sensor_data)
            if self.p.get('typed_navigation_state'):
                kind,topic,callback=(LocalNavigationState,'/d1max/localization/navigation/local_state',self.on_local_navigation) \
                    if self.p.get('independent_local_state') else (NavigationState,'/d1max/localization/navigation/state',self.on_navigation)
                self.create_subscription(kind,topic,callback,10)
            else:
                self.create_subscription(Odometry, '/d1max/localization/odometry/local', self.on_local, 50)
                self.create_subscription(PoseStamped, '/d1max/localization/pose', self.on_global, 50)
                for key, topic in (('localizer', '/d1max/localization/status'),
                                   ('navigation', '/d1max/localization/navigation/status'),
                                   ('pose', '/d1max/localization/navigation/pose_status')):
                    self.create_subscription(String, topic,
                                             lambda message, k=key: self.on_status(k, message), 10)
            self.create_timer(.01, self.tick)

        def destroy_node(self):
            self.invalidate_pending()
            self.worker.shutdown(wait=True, cancel_futures=True)
            return super().destroy_node()

        def invalidate_pending(self):
            self.work_generation += 1
            self.pending = [None, None]
            self.authorized_pose_context = None
            self.authorized_pose_ns = 0

        def publish_latched_fault(self):
            """Revoke owner leases at the fault callback, not the periodic status tick."""
            if self.core.fault:
                self.invalidate_pending()
                self.error = self.core.fault
                context = self.current_context()
                key = (context, self.core.fault)
                if key != self.last_fault_status:
                    self.publish_status(context, self.now_ns(), time.monotonic())
                    self.last_fault_status = key

        def now_ns(self):
            return self.get_clock().now().nanoseconds

        def identity(self):
            now, mono = self.now_ns()*1e-9, time.monotonic()
            if self.p.get('typed_navigation_state'):
                if self.p.get('independent_local_state'):
                    from .local_navigation_state import history_identity
                    # Context is a history identity, not a motion lease. Old
                    # rays still require real pose coverage and bounded age.
                    return history_identity(self.local_inbox)
                state = self.atomic_state
                if (state is None or not 0 <= mono-self.atomic_received <= .4
                        or not 0 <= self.now_ns()-state.source_ns <= 400_000_000
                        or not 0 <= self.now_ns()-state.posterior_ns <= 400_000_000
                        or not 0 <= self.now_ns()-state.imu_ns <= 100_000_000):
                    return None
                return state.identity
            if self.pose_status_fault or any(not 0 <= mono-self.status_received[key] <= .5
                   for key in ('localizer', 'navigation')):
                return None
            try:
                identity = localization_identity_context(
                    self.statuses['localizer'], self.statuses['navigation'],
                    session_id=self.p['session_id'], now=now, timeout=.5,
                    map_frame=self.p['map_frame'], tracking_frame=self.p['tracking_frame'])
                pose_status = self.statuses['pose']
                if (not pose_status or pose_status.get('epoch') != identity[1]
                        or pose_status.get('seed_id') != identity[2]
                        or pose_status.get('fault') or pose_status.get('reset_pending') is not False
                        or pose_status.get('frame_id') != self.p['map_frame']
                        or pose_status.get('body_frame') != self.p['body_frame']):
                    return None
                return identity
            except (ValueError, TypeError, KeyError):
                return None

        def authorize_pose_history(self, context):
            """Pin a real validated output watermark, never renew its source time.

            Expiry of the current control lease cannot invalidate already
            measured history. It also cannot authorize a later scan/alignment.
            """
            if self.p.get('typed_navigation_state'):
                state = self.atomic_state
                if state is not None and state.identity == (context.session_id, context.epoch, context.seed_id):
                    self.authorized_pose_context = context
                    self.authorized_pose_ns = max(self.authorized_pose_ns, state.source_ns)
                return
            try:
                validate_continuous_pose(self.statuses['pose'], epoch=context.epoch,
                    seed=context.seed_id, now=self.now_ns()*1e-9, timeout=.5,
                    map_frame=self.p['map_frame'], body_frame=self.p['body_frame'],
                    receipt_age=time.monotonic()-self.status_received['pose'])
            except (ValueError, TypeError, KeyError):
                return
            source = round(self.statuses['pose']['output_stamp_sec']*1e9)
            if source <= context.barrier_ns:
                return
            if self.authorized_pose_context != context:
                self.authorized_pose_context, self.authorized_pose_ns = context, source
            else:
                self.authorized_pose_ns = max(self.authorized_pose_ns, source)

        def current_context(self):
            identity = self.identity()
            if (self.request is None or self.ack != self.request or identity is None
                    or identity != (self.request.session_id, self.request.epoch, self.request.seed_id)):
                return None
            if self.core.context != self.request:
                self.core.reset(self.request)
                self.invalidate_pending()
                self.last_raw_stamp = [0, 0]
                self.last_published = [0, 0]
                self.counts['context_resets'] += 1
                self.install_extrinsics()
                self.authorize_pose_history(self.request)
            return self.request

        def on_navigation(self, message):
            from .atomic_projection_state import validate
            try:
                state = validate(message, session_id=self.p['session_id'],
                    map_version_id=self.p['map_version_id'], now_ns=self.now_ns(),
                    map_frame=self.p['map_frame'], odom_frame=self.p['odom_frame'],
                    body_frame=self.p['body_frame'])
                old = self.atomic_state
                if old is not None and old.identity == state.identity and state.source_ns <= old.source_ns:
                    return  # A repeated delivery never renews the receipt lease.
                self.atomic_state, self.atomic_received = state, time.monotonic()
                context = self.current_context()
                if context is None:
                    self.invalidate_pending()
                    return
                # In odom, map correction is deliberately NOT integrated into
                # point geometry. Hard identity changes require a new context.
                self.core.add_local_body(state.source_ns, state.local_body, context)
                self.authorize_pose_history(context)
            except (ValueError, TypeError, AttributeError) as error:
                self.counts['pose_rejected'] += 1
                self.error = str(error)
                self.atomic_state = None
                self.invalidate_pending()
                self.publish_latched_fault()

        def on_local_navigation(self,message):
            event=self.local_inbox.accept(message,session_id=self.p['session_id'],
                map_version_id=self.p['map_version_id'],now_ns=self.now_ns(),monotonic=time.monotonic())
            if event is None:
                self.counts['pose_rejected']+=1
                return  # malformed/late packets do not erase measured history
            if event.hard_failure:
                self.atomic_state=None
                self.core.reset(None)
                self.invalidate_pending()
                self.error=event.reason
                return
            if event.state is None:
                return  # soft control expiry cannot discard acquisition history
            self.atomic_state,self.atomic_received=event.state,event.received
            context=self.current_context()
            if context is None:
                self.invalidate_pending()
                return
            try:
                self.core.add_local_body(event.source_ns,event.state.local_body,context)
                self.authorize_pose_history(context)
            except (ValueError,TypeError) as error:
                self.counts['pose_rejected']+=1
                self.error=str(error)
                self.publish_latched_fault()

        def on_status(self, key, message):
            try:
                if len(message.data) > (262144 if key == 'localizer' else 65536):
                    raise ProjectionError('oversize_pose_status')
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    raise ProjectionError('invalid_pose_status_object')
                stamp = value.get('wall_time' if key == 'localizer' else 'received_at_unix')
                if not fresh(stamp, self.now_ns()*1e-9, .5, future=.01):
                    raise ProjectionError('stale_pose_status')
                if stamp <= self.status_stamps[key]:
                    return
                if key == 'pose':
                    checked_pose_status_envelope(value, now=self.now_ns()*1e-9, timeout=.5)
                    if (value['frame_id'] != self.p['map_frame']
                            or value['body_frame'] != self.p['body_frame']
                            or value.get('fault') or value['reset_pending']
                            or value['epoch'] < 1 or not isinstance(value.get('seed_id'), str)
                            or not value['seed_id']):
                        raise ProjectionError('invalid_pose_history_identity_or_fault')
                    if value['valid']:
                        validate_continuous_pose(value, epoch=value['epoch'], seed=value['seed_id'],
                            now=self.now_ns()*1e-9, timeout=.5, receipt_age=0.,
                            map_frame=self.p['map_frame'], body_frame=self.p['body_frame'])
                    self.pose_status_fault = False
                self.status_stamps[key] = stamp
                self.statuses[key] = value
                self.status_received[key] = time.monotonic()
                context = self.current_context()
                if context is None:
                    self.invalidate_pending()
                else:
                    self.authorize_pose_history(context)
            except (ValueError, TypeError, KeyError) as error:
                self.statuses[key] = {}
                if key == 'pose':
                    self.pose_status_fault = True
                self.invalidate_pending()
                self.error = str(error)

        def read_context(self, message):
            if len(message.data) > 4096:
                raise ProjectionError('oversize_map_context')
            value = MapContext.parse(json.loads(message.data))
            if value.session_id != self.p['session_id']:
                raise ProjectionError('foreign_projector_context')
            return value

        def on_context(self, message):
            try:
                value = self.read_context(message)
                if value.sequence < self.highest_context_sequence:
                    return
                if value.sequence == self.highest_context_sequence and self.request != value:
                    raise ProjectionError('conflicting_map_context_sequence')
                if value != self.request:
                    self.core.reset(None)
                    self.invalidate_pending()
                    self.last_raw_stamp = [0, 0]
                    self.last_published = [0, 0]
                    self.request = value
                    self.highest_context_sequence = value.sequence
                self.current_context()
            except (ValueError, TypeError, KeyError) as error:
                self.request = None
                self.core.reset(None)
                self.invalidate_pending()
                self.error = str(error)

        def on_ack(self, message):
            try:
                value = self.read_context(message)
                if self.ack is not None and value.sequence < self.ack.sequence:
                    return
                self.ack = value
                self.current_context()
            except (ValueError, TypeError, KeyError) as error:
                self.ack = None
                self.invalidate_pending()
                self.error = str(error)

        def on_static(self, message):
            required = ((self.p['body_frame'], self.p['tracking_frame']),
                        (self.p['tracking_frame'], self.p['ray_frame']))
            try:
                for item in message.transforms:
                    key = (item.header.frame_id, item.child_frame_id)
                    if key not in required:
                        continue
                    v, q = item.transform.translation, item.transform.rotation
                    value = checked_pose((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))
                    self.static_edges[key] = value
                # Accept the identity only when the two named frames are literally identical.
                if self.p['ray_frame'] == self.p['tracking_frame']:
                    self.static_edges[required[1]] = checked_pose((0., 0., 0.), (0., 0., 0., 1.))
                self.install_extrinsics()
            except (ValueError, TypeError) as error:
                self.invalidate_pending()
                self.error = str(error)
                self.publish_latched_fault()

        def install_extrinsics(self):
            required = ((self.p['body_frame'], self.p['tracking_frame']),
                        (self.p['tracking_frame'], self.p['ray_frame']))
            if all(key in self.static_edges for key in required):
                self.core.set_extrinsics(body_to_tracking=self.static_edges[required[0]],
                                         ray_to_tracking=self.static_edges[required[1]])

        def missing_static_edges(self):
            required = ((self.p['body_frame'], self.p['tracking_frame']),
                        (self.p['tracking_frame'], self.p['ray_frame']))
            return [key for key in required if key not in self.static_edges]

        def retry_static_extrinsics(self, mono):
            """Request retained TF again, without inventing an absent transform."""
            if (not self.missing_static_edges() or self.core.fault
                    or mono-self.last_static_retry < self.static_retry_delay):
                return
            # Destroy first: never leave duplicate subscriptions or an unbounded
            # retry backlog. A new transient reader requests retained samples.
            self.destroy_subscription(self.static_subscription)
            self.static_subscription = self.create_subscription(
                TFMessage, '/tf_static', self.on_static, self.static_qos)
            self.last_static_retry = mono
            self.static_retry_count += 1
            self.static_retry_delay = min(8., self.static_retry_delay*2.)

        def on_local(self, message):
            context = self.current_context()
            if context is None:
                return
            try:
                stamp = stamp_ns(message.header.stamp)
                if (message.header.frame_id != self.p['odom_frame']
                        or message.child_frame_id != self.p['body_frame']
                        or not -.01 <= (self.now_ns()-stamp)*1e-9 <= .5):
                    raise ProjectionError('invalid_local_pose_frame_or_time')
                self.core.add_local_body(stamp, pose(message.pose.pose), context)
            except (ValueError, TypeError) as error:
                self.counts['pose_rejected'] += 1
                self.error = str(error)
                self.publish_latched_fault()

        def on_global(self, message):
            context = self.current_context()
            if context is None:
                return
            try:
                stamp = stamp_ns(message.header.stamp)
                if (message.header.frame_id != self.p['map_frame']
                        or not -.01 <= (self.now_ns()-stamp)*1e-9 <= .5):
                    raise ProjectionError('invalid_tracking_pose_frame_or_time')
                self.core.add_global_tracking(stamp, pose(message.pose), context)
            except (ValueError, TypeError) as error:
                self.counts['pose_rejected'] += 1
                self.error = str(error)
                self.publish_latched_fault()

        def on_rays(self, message):
            context = self.current_context()
            self.counts['received'] += 1
            if context is None or self.core.fault:
                self.counts['dropped'] += 1
                return
            try:
                raw = decode_rays(data=message.data,
                    fields=[(f.name, f.offset, f.datatype, f.count) for f in message.fields],
                    point_step=message.point_step, row_step=message.row_step,
                    width=message.width, height=message.height, bigendian=message.is_bigendian,
                    header_ns=stamp_ns(message.header.stamp), frame_id=message.header.frame_id,
                    expected_frame=self.p['ray_frame'], max_points=self.core.limits.max_input_points)
                if raw.sensor_id not in self.p['sensor_ids']:
                    raise ProjectionError('unconfigured_sensor_source')
                if raw.start_ns <= max(context.barrier_ns, self.last_raw_stamp[raw.sensor_id]):
                    raise ProjectionError('duplicate_or_precontext_raw_rays')
                self.last_raw_stamp[raw.sensor_id] = raw.start_ns
                if self.pending[raw.sensor_id] is not None:
                    self.counts['overwritten'] += 1
                self.pending[raw.sensor_id] = (message, raw, context, time.monotonic())
            except (ValueError, TypeError) as error:
                self.counts['dropped'] += 1
                self.error = str(error)

        def tick(self):
            now, mono = self.now_ns(), time.monotonic()
            self.retry_static_extrinsics(mono)
            context = self.current_context()
            if context is None or self.core.fault:
                self.invalidate_pending()
                self.error = self.core.fault or 'waiting_localization_context'
            # Geometry uses only a private snapshot; it never calls ROS or
            # touches live history. Pose/status callbacks keep their 80 ms lease.
            if self.inflight is not None and self.inflight[0].done():
                future, item, token, started = self.inflight
                self.inflight = None
                message, raw, captured_context, received = item
                sensor = raw.sensor_id
                try:
                    if (token != self.work_generation or context != captured_context
                            or self.core.fault or mono-received > .25):
                        raise ProjectionError('ray_projection_wait_expired_or_context_changed')
                    projected = future.result()
                    if (self.now_ns()-projected.start_ns > round(
                            self.core.limits.input_timeout_sec*1e9)
                            or self.current_context() != captured_context):
                        raise ProjectionError('projection_expired_during_conversion')
                    projected = self.core.commit_projection(projected)
                    output = ProjectedRays()
                    output.session_id, output.epoch, output.seed_id = (
                        context.session_id, context.epoch, context.seed_id)
                    output.context_sequence, output.barrier_ns = context.sequence, context.barrier_ns
                    output.projection_sequence = projected.sequence
                    assign_stamp(output.acquisition_end, projected.end_ns)
                    assign_stamp(output.alignment_stamp, projected.alignment_ns)
                    cloud = PointCloud2()
                    cloud.header.frame_id = self.p.get('output_frame', self.p['map_frame'])
                    assign_stamp(cloud.header.stamp, projected.start_ns)
                    cloud.height, cloud.width, cloud.point_step = 1, len(projected.points), 64
                    cloud.row_step, cloud.is_bigendian, cloud.is_dense = cloud.width*64, False, True
                    cloud.fields = message.fields
                    payload = array('B')
                    payload.frombytes(memoryview(projected.points).cast('B'))
                    cloud.data = payload
                    output.rays = cloud
                    self.publisher.publish(output)
                    self.exclusion_counts['matched'][sensor] += projected.exclusion_matched
                    self.exclusion_counts['dropped'][sensor] += projected.exclusion_dropped
                    self.last_published[sensor] = projected.start_ns
                    self.counts['published'] += 1
                    self.error = ''
                except PreviewExcludedScan as error:
                    self.counts['dropped'] += 1
                    self.error = str(error)
                    exclusion = self.core.preview_exclusion
                    if exclusion is not None and error.digest == exclusion.digest:
                        self.exclusion_counts['matched'][sensor] += error.count
                        self.exclusion_counts['dropped'][sensor] += error.count
                    # Intentionally no output, last_published or map update.
                except AwaitingCoverage as error:
                    # Preserve original receipt deadline; never replace a newer
                    # pending cloud with this older incomplete observation.
                    if self.pending[sensor] is None:
                        self.pending[sensor] = item
                    self.error = str(error)
                except (ValueError, TypeError, KeyError, OverflowError, RuntimeError) as error:
                    self.counts['dropped'] += 1
                    self.error = str(error)
                self.processing_ms = (time.perf_counter()-started)*1000.
            if context is not None and not self.core.fault and self.inflight is None:
                for sensor in (self.next_sensor, 1-self.next_sensor):
                    item = self.pending[sensor]
                    if item is None:
                        continue
                    message, raw, captured_context, received = item
                    if captured_context != context or mono-received > .25:
                        self.pending[sensor] = None
                        self.counts['dropped'] += 1
                        self.error = 'ray_projection_wait_expired_or_context_changed'
                        continue
                    authorized = self.authorized_pose_ns
                    if self.authorized_pose_context != context or raw.end_ns > authorized:
                        self.error = 'waiting_authorized_pose_history'
                        continue  # Original receive/source deadlines still apply.
                    snapshot = self.core.projection_snapshot()
                    self.pending[sensor] = None
                    future = self.worker.submit(snapshot.project, raw, captured_context,
                                                now_ns=now, authorized_pose_ns=authorized)
                    self.inflight = (future, item, self.work_generation, time.perf_counter())
                    self.next_sensor = 1-sensor
                    break  # exactly one in flight; no hidden executor backlog
            if mono-self.last_status >= .2:
                self.publish_status(context, now, mono)

        def publish_status(self, context, now, mono):
            context_ready = context is not None and self.core.fault is None
            projection_ready = bool(context_ready and self.core.ray_to_tracking is not None
                                    and len(self.core.local) >= 2 and
                                    (self.core.projection_frame=='odom' or self.core.alignments))
            source_fresh = [bool(stamp and 0 <= now-stamp <= round(
                self.core.limits.input_timeout_sec*1e9)) for stamp in self.last_published]
            state = dict(schema=1, session_id=self.p['session_id'], enabled=True,
                received_at_unix=now*1e-9,callback_wall_time=time.time(),context=asdict(context) if context else None,
                valid=projection_ready and all(source_fresh[i] for i in self.p['sensor_ids']),
                context_ready=context_ready, projection_ready=projection_ready,
                source_fresh=source_fresh,
                fault=self.core.fault, reason=self.error, motion_control_enabled=False,
                source='per_sensor_acquisition_time_one_map_alignment',
                last_published_source_ns=self.last_published,
                pending=[item is not None for item in self.pending],
                processing=self.inflight is not None,
                local_history_samples=len(self.core.local), alignment_pairs=len(self.core.alignments),
                alignment_displacement=self.core.alignment_displacement,
                alignment_rebuild_limits=dict(
                    translation_m=self.core.limits.max_alignment_translation_from_anchor_m,
                    rotation_rad=self.core.limits.max_alignment_rotation_from_anchor_rad),
                processing_ms=self.processing_ms, counts=self.counts)
            state['static_extrinsics'] = dict(
                ready=not self.missing_static_edges(),
                missing=[list(key) for key in self.missing_static_edges()],
                received=[list(key) for key in sorted(self.static_edges)],
                resubscriptions=self.static_retry_count,
                retry_interval_s=self.static_retry_delay)
            exclusion = self.core.preview_exclusion
            state['preview_ray_exclusion'] = dict(
                enabled=exclusion is not None, mode=exclusion.mode if exclusion else 'off',
                config_sha256=exclusion.digest if exclusion else '',
                calibration_verified=False, counts=self.exclusion_counts,
                warning='experimental_endpoint_exclusion_not_self_classification' if exclusion else '')
            self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
            self.last_status = mono

    return PerceptionRayProjector()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path)
    args, ros_args = parser.parse_known_args()
    if args.session is None:
        return  # standalone/default is disabled, no ROS graph side effects
    import yaml
    from .live_view_reload import read_owned_json
    session = read_owned_json(args.session/'session.json')
    path = args.session/'localization.yaml'
    if path.stat().st_size > 1024*1024:
        raise ValueError('oversize_session_localization_configuration')
    settings = session_settings(session, yaml.safe_load(path.read_text()))
    if not settings['enabled']:
        return
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise ValueError('Per-sensor projector requires unchanged rmw_zenoh_cpp')
    import rclpy
    rclpy.init(args=ros_args)
    node = create_node(settings)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
