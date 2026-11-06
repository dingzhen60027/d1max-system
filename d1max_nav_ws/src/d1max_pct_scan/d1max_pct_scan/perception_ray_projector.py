"""Optional per-sensor metadata -> motion-compensated map rays boundary.

No SDK, command publishers, planner calls, ground clearing or startup authority.
The live session supervisor is the only owner of this independent process.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
from threading import Lock
import time

from .ray_projection import (AwaitingCoverage, MapContext, ProjectionError, PreviewExcludedScan, EntityFilteredScan,
                             RayProjectorCore, checked_pose, decode_ray_envelope,
                             project_validated_rays, session_settings)

PREFIX = '/d1max/live_planning/'


def entity_stage_summary(report):
    """Bounded counters already measured by this source's geometry worker.

    Do not copy link lists, point data or the latest report from another scan.
    This is cost evidence only; it cannot alter the filter or its budget.
    """
    if report is None:
        return {}
    return {key: report[key] for key in (
        'classification_ms', 'near_point_count', 'matched', 'suppressed', 'reason')}


def timed_projection(snapshot, raw, context, *, now_ns, authorized_pose_ns, timing):
    """Worker-private timing; no source clock or projection result changes.

    The owner reads this fixed-size record only after Future.done(), including
    failed geometry. thread_time excludes time when this worker is not running.
    """
    started, cpu_started = time.perf_counter_ns(), time.thread_time_ns()
    try:
        return project_validated_rays(snapshot, raw, context,
            now_ns=now_ns, authorized_pose_ns=authorized_pose_ns)
    finally:
        ended, cpu_ended = time.perf_counter_ns(), time.thread_time_ns()
        timing.update(started_ns=started, ended_ns=ended,
                      worker_wall_ms=(ended-started)*1e-6,
                      worker_cpu_ms=(cpu_ended-cpu_started)*1e-6)


def create_node(settings):
    # Importing configuration or the pure geometry module never creates a ROS context.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
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
            from .local_navigation_state import LocalNavigationHistoryInbox
            self.local_inbox = LocalNavigationHistoryInbox()
            self.entity_filter = self.joint_history = None
            self.entity_reports = [None, None]
            self.entity_counts = {'matched': [0, 0], 'suppressed': [0, 0]}
            if settings.get('entity_filter_settings'):
                from .robot_self_filter import RobotSelfFilter
                self.entity_filter = RobotSelfFilter.load(settings['entity_filter_settings'])
                if self.entity_filter.requires_joint_stream:
                    from .robot_self_pose import JointPoseHistory
                    self.joint_history = JointPoseHistory(self.entity_filter.model.joint_types)
            self.core = RayProjectorCore(settings['limits'], settings.get('preview_exclusion'),
                projection_frame=settings.get('projection_frame', 'map'), entity_filter=self.entity_filter)
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
            self.last_raw_receipt_ns = [0, 0]
            self.last_raw_gap = [{}, {}]
            self.last_published = [0, 0]
            self.last_status = -math.inf
            self.last_pipeline_trace = -math.inf
            self.last_fault_status = None
            self.next_sensor = 0
            # One running job and one latest pending acquisition per source.
            # A slow front scan must not hold the rear source's original lease
            # hostage (or vice versa). The owner alone commits and publishes.
            self.worker = ThreadPoolExecutor(max_workers=2, thread_name_prefix='ray_geometry')
            self.inflight = [None, None]
            self.completion_lock = Lock()
            self.projection_stopping = False
            self.completion_wake_failures = 0
            self.work_generation = 0
            self.error = 'waiting_localization_context'
            self.counts = dict(received=0, published=0, dropped=0, overwritten=0,
                               pose_rejected=0, context_resets=0)
            # Passive, fixed-size evidence for pose-history holes. Ray geometry
            # cannot tell whether the producer omitted a sample, transport lost
            # it, or this consumer rejected it. Keep those observations separate
            # without renewing a sample's source time or control deadline.
            self.local_state_diagnostics = dict(
                counts=dict(received=0, accepted=0, rejected=0, soft_unavailable=0,
                            hard_unavailable=0, context_unavailable=0,
                            history_added=0, history_not_added=0, history_rejected=0,
                            history_after_control_deadline=0),
                last_received={}, last_rejected={}, last_history={},
                last_history_gap={}, max_history_gap_ns=0)
            # Fixed-cardinality counters: a later success must not erase the
            # cause of a discarded scan before the 1 Hz trace samples it.
            self.drop_stages = dict(context=0, envelope=0, wait=0, source=0,
                                    geometry=0, entity=0, preview=0)
            self.last_drop = {}
            self.processing_ms = 0.
            self.projection_timing = [{}, {}]
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
            # Shared dual-source topic: depth one can discard one source in a
            # front/rear burst. Bound middleware history to the source count;
            # the application separately keeps only the latest cloud per source.
            self.raw_qos = QoSProfile(depth=len(self.p['sensor_ids']),
                                     reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(PointCloud2, self.p['raw_topic'], self.on_rays, self.raw_qos)
            if self.entity_filter is not None and self.entity_filter.requires_joint_stream:
                from sensor_msgs.msg import JointState
                self.create_subscription(JointState,
                    settings['entity_filter_settings']['joint_state_topic'], self.on_joints,
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
            # Geometry completion only wakes the owner; it never publishes or
            # mutates its histories from the worker. Sharing the default
            # mutually-exclusive group also serializes this callback with
            # context/pose input and the fallback timer under any executor.
            self.completion_guard = self.create_guard_condition(
                self.tick, callback_group=self.default_callback_group)
            self.create_timer(.01, self.tick, callback_group=self.default_callback_group)

        def destroy_node(self):
            # Serialize trigger vs destruction. Join both workers before
            # Node destroys its guard, including an already completed Future.
            with self.completion_lock:
                self.projection_stopping = True
            self.invalidate_pending()
            self.worker.shutdown(wait=True, cancel_futures=True)
            return super().destroy_node()

        def projection_completed(self, future):
            # Missing historical pose coverage is retried by the existing
            # timer. Waking on that same result would immediately resubmit it
            # against unchanged history, creating a worker/owner busy loop.
            if future.cancelled() or isinstance(future.exception(), AwaitingCoverage):
                return
            self.wake_projection_owner()

        def wake_projection_owner(self):
            with self.completion_lock:
                if self.projection_stopping:
                    return
                try:
                    self.completion_guard.trigger()
                except RuntimeError:
                    # A signal may invalidate RCL before destroy_node runs.
                    # The normal timer remains the fallback for a live context;
                    # failed wakeups never publish or renew the original scan.
                    self.completion_wake_failures += 1

        def wake_ready_input(self):
            # Input delivery should not wait for the next 10 ms fallback
            # timer. One in-flight + one latest-only slot per source remain the
            # queue; waking the owner neither publishes nor authorizes data.
            if any(self.inflight[sensor] is None and item is not None
                    and item[2] == self.authorized_pose_context
                    and item[1].end_ns <= self.authorized_pose_ns
                    for sensor, item in enumerate(self.pending)):
                self.wake_projection_owner()

        def record_drop(self, stage, reason, raw=None):
            self.counts['dropped'] += 1
            self.drop_stages[stage] += 1
            self.last_drop = dict(stage=stage, reason=str(reason)[:256],
                observed_ns=self.now_ns(), sensor=raw.sensor_id if raw else -1,
                source_ns=raw.start_ns if raw else 0)
            # Preserve the actual missing bracket across later successful
            # scans. No retry, interpolation limit or source timestamp changes.
            if hasattr(reason, 'pose_gap'):
                self.last_drop['pose_gap'] = dict(reason.pose_gap)

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
                    self.wake_ready_input()
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
            self.wake_ready_input()

        def current_context(self):
            identity = self.identity()
            if (self.request is None or self.ack != self.request or identity is None
                    or identity != (self.request.session_id, self.request.epoch, self.request.seed_id)):
                return None
            if self.core.context != self.request:
                self.core.reset(self.request)
                if self.joint_history is not None:
                    self.joint_history.reset(self.request)
                self.invalidate_pending()
                self.last_raw_stamp = [0, 0]
                self.last_raw_receipt_ns = [0, 0]
                self.last_raw_gap = [{}, {}]
                self.last_published = [0, 0]
                self.counts['context_resets'] += 1
                self.install_extrinsics()
                self.authorize_pose_history(self.request)
            return self.request

        def on_joints(self, message):
            # This input is a named, radian-valued JointState with a real
            # acquisition stamp. SDK receipt-only JSON deliberately is NOT
            # mapped into it: that would invent time/unit/URDF evidence.
            context = self.current_context()
            if context is None or self.joint_history is None:
                return
            try:
                if not 0 <= int(message.header.stamp.nanosec) < 1_000_000_000:
                    return
                source = stamp_ns(message.header.stamp)
                if (source <= context.barrier_ns or not -10_000_000 <= self.now_ns()-source <= 2_000_000_000
                        or message.header.frame_id not in ('', self.p['body_frame'])):
                    return
                self.joint_history.add_joint_state(source, message.name, message.position, unit='radian')
            except (ValueError, TypeError, AttributeError, OverflowError):
                # Malformed articulation cannot stop independent LiDAR input.
                return

        def record_entity_report(self, report):
            if report is None:
                return
            sensor = report['sensor_id']
            self.entity_reports[sensor] = report
            for key in ('matched', 'suppressed'):
                self.entity_counts[key][sensor] += report[key]

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

        def local_state_receipt(self, message, received_ns):
            # Malformed envelopes remain the inbox's responsibility. Unknown
            # stamps are diagnostics only, not an invented rejection reason.
            from .atomic_projection_state import stamp_ns as checked_stamp_ns
            result = dict(received_ns=received_ns, source_ns=None, valid_until_ns=None)
            for name, field in (('source_ns', 'source_stamp'), ('valid_until_ns', 'valid_until')):
                try:
                    result[name] = checked_stamp_ns(getattr(message, field))
                except (ValueError, TypeError, AttributeError, OverflowError):
                    pass
            if result['source_ns'] is not None:
                result['source_age_ns'] = received_ns-result['source_ns']
            if result['valid_until_ns'] is not None:
                result['past_deadline'] = received_ns > result['valid_until_ns']
            return result

        def on_local_navigation(self,message):
            received_ns = self.now_ns()
            diagnostics = self.local_state_diagnostics
            counts = diagnostics['counts']
            counts['received'] += 1
            receipt = self.local_state_receipt(message, received_ns)
            diagnostics['last_received'] = receipt
            event=self.local_inbox.accept(message,session_id=self.p['session_id'],
                map_version_id=self.p['map_version_id'],now_ns=received_ns,monotonic=time.monotonic())
            if event is None:
                counts['rejected'] += 1
                diagnostics['last_rejected'] = dict(receipt, result='rejected_unknown_reason')
                self.counts['pose_rejected']+=1
                return  # malformed/late packets do not erase measured history
            if event.hard_failure:
                counts['hard_unavailable'] += 1
                self.atomic_state=None
                self.core.reset(None)
                self.invalidate_pending()
                self.error=event.reason
                return
            if event.state is None:
                counts['soft_unavailable'] += 1
                return  # soft control expiry cannot discard acquisition history
            counts['accepted'] += 1
            self.atomic_state,self.atomic_received=event.state,event.received
            context=self.current_context()
            if context is None:
                counts['context_unavailable'] += 1
                self.invalidate_pending()
                return
            try:
                previous_ns = self.core.last_local_ns
                added = self.core.add_local_body(event.source_ns,event.state.local_body,context)
                if added:
                    counts['history_added'] += 1
                    if received_ns > event.state.valid_until_ns:
                        counts['history_after_control_deadline'] += 1
                    gap_ns = event.source_ns-previous_ns if previous_ns else 0
                    history = dict(receipt, previous_source_ns=previous_ns, gap_ns=gap_ns)
                    diagnostics['last_history'] = history
                    diagnostics['max_history_gap_ns'] = max(
                        diagnostics['max_history_gap_ns'], gap_ns)
                    if gap_ns > round(self.core.limits.max_pose_gap_sec*1e9):
                        diagnostics['last_history_gap'] = history
                else:
                    counts['history_not_added'] += 1
                self.authorize_pose_history(context)
            except (ValueError,TypeError) as error:
                counts['history_rejected'] += 1
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
                    self.last_raw_receipt_ns = [0, 0]
                    self.last_raw_gap = [{}, {}]
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
            received = time.monotonic()
            # Passive fixed-size stage evidence travels with this acquisition,
            # including coverage retries. It is never used for freshness.
            ingress = dict(received_ns=self.now_ns(),
                           received_steady_ns=time.perf_counter_ns())
            context = self.current_context()
            self.counts['received'] += 1
            if context is None or self.core.fault:
                self.record_drop('context', self.core.fault or 'waiting_localization_context')
                return
            try:
                raw = decode_ray_envelope(data=message.data,
                    fields=[(f.name, f.offset, f.datatype, f.count) for f in message.fields],
                    point_step=message.point_step, row_step=message.row_step,
                    width=message.width, height=message.height, bigendian=message.is_bigendian,
                    header_ns=stamp_ns(message.header.stamp), frame_id=message.header.frame_id,
                    expected_frame=self.p['ray_frame'], max_points=self.core.limits.max_input_points)
                if raw.sensor_id not in self.p['sensor_ids']:
                    raise ProjectionError('unconfigured_sensor_source')
                if raw.start_ns <= max(context.barrier_ns, self.last_raw_stamp[raw.sensor_id]):
                    raise ProjectionError('duplicate_or_precontext_raw_rays')
                sensor, receipt_ns = raw.sensor_id, self.now_ns()
                previous, previous_receipt = self.last_raw_stamp[sensor], self.last_raw_receipt_ns[sensor]
                # Passive evidence only. At 10 Hz a delayed/missing acquisition
                # can disappear between 1 Hz log samples. Retain one event per
                # sensor, rather than inferring a network fault from map expiry.
                if previous and (raw.start_ns-previous > 150_000_000 or
                                 receipt_ns-previous_receipt > 150_000_000):
                    self.last_raw_gap[sensor] = dict(previous_source_ns=previous,
                        source_ns=raw.start_ns, previous_received_ns=previous_receipt,
                        received_ns=receipt_ns)
                self.last_raw_receipt_ns[sensor] = receipt_ns
                self.last_raw_stamp[raw.sensor_id] = raw.start_ns
                if self.pending[raw.sensor_id] is not None:
                    self.counts['overwritten'] += 1
                ingress['decoded_steady_ns'] = time.perf_counter_ns()
                self.pending[raw.sensor_id] = (message, raw, context, received, ingress)
                self.wake_ready_input()
            except (ValueError, TypeError) as error:
                self.record_drop('envelope', error)
                self.error = str(error)

        def tick(self):
            if self.projection_stopping:
                return
            now, mono = self.now_ns(), time.monotonic()
            self.retry_static_extrinsics(mono)
            context = self.current_context()
            if context is None or self.core.fault:
                self.invalidate_pending()
                self.error = self.core.fault or 'waiting_localization_context'
            # Geometry uses only a private snapshot; it never calls ROS or
            # touches live history. Pose/status callbacks keep their 80 ms lease.
            for sensor in self.p['sensor_ids']:
                work = self.inflight[sensor]
                if work is None or not work[0].done():
                    continue
                future, item, token, started, timing = work
                self.inflight[sensor] = None
                message, raw, captured_context, received, ingress = item
                # Completion ordering is owner-local; these monotonic times
                # describe cost only and never become observation timestamps.
                harvested = time.perf_counter_ns()
                stage = dict(source_ns=raw.start_ns,
                    worker_wall_ms=timing.get('worker_wall_ms', 0.),
                    worker_cpu_ms=timing.get('worker_cpu_ms', 0.),
                    owner_harvest_ms=max(0., (harvested-timing.get('ended_ns', harvested))*1e-6),
                    publish_ms=0., published=False, entity={},
                    stages=dict(ingress,
                        submitted_steady_ns=timing['submitted_steady_ns'],
                        worker_started_steady_ns=timing.get('started_ns', 0),
                        worker_ended_steady_ns=timing.get('ended_ns', 0),
                        harvested_steady_ns=harvested,
                        publish_started_ns=0, publish_ended_ns=0,
                        publish_started_steady_ns=0, publish_ended_steady_ns=0))
                drop_stage = 'wait'
                try:
                    if (token != self.work_generation or context != captured_context
                            or self.core.fault or time.monotonic()-received > .25):
                        raise ProjectionError('ray_projection_wait_expired_or_context_changed')
                    drop_stage = 'geometry'
                    projected = future.result()
                    stage['entity'] = entity_stage_summary(projected.entity_filter_report)
                    drop_stage = 'source'
                    if (self.now_ns()-projected.start_ns > round(
                            self.core.limits.input_timeout_sec*1e9)
                            or self.current_context() != captured_context):
                        raise ProjectionError('projection_expired_during_conversion')
                    drop_stage = 'geometry'
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
                    cloud.height, cloud.width = 1, len(projected.points)
                    cloud.point_step = projected.points.dtype.itemsize
                    cloud.row_step, cloud.is_bigendian, cloud.is_dense = cloud.width*cloud.point_step, False, True
                    cloud.fields = [PointField(name=n, offset=o, datatype=t, count=c)
                                    for n, o, t, c in projected.wire_fields]
                    # Worker already owns the final array('B'). Humble's
                    # PointCloud2 setter accepts it without another full-frame
                    # Python copy on this pose/status callback thread.
                    cloud.data = projected.wire_data
                    output.rays = cloud
                    stage['stages']['publish_started_ns'] = self.now_ns()
                    publication_started = time.perf_counter_ns()
                    stage['stages']['publish_started_steady_ns'] = publication_started
                    try:
                        self.publisher.publish(output)
                    finally:
                        publication_ended = time.perf_counter_ns()
                        stage['publish_ms'] = (publication_ended-publication_started)*1e-6
                        stage['stages']['publish_ended_steady_ns'] = publication_ended
                        stage['stages']['publish_ended_ns'] = self.now_ns()
                    stage['published'] = True
                    self.exclusion_counts['matched'][sensor] += projected.exclusion_matched
                    self.exclusion_counts['dropped'][sensor] += projected.exclusion_dropped
                    self.record_entity_report(projected.entity_filter_report)
                    self.last_published[sensor] = projected.start_ns
                    self.counts['published'] += 1
                    self.error = ''
                except EntityFilteredScan as error:
                    stage['entity'] = entity_stage_summary(error.report)
                    self.record_drop('entity', error, raw)
                    self.error = str(error)
                    self.record_entity_report(error.report)
                    # No output/map clearing or new observed-free lease.
                except PreviewExcludedScan as error:
                    self.record_drop('preview', error, raw)
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
                    self.record_drop(drop_stage, error, raw)
                    self.error = str(error)
                self.processing_ms = (time.perf_counter()-started)*1000.
                self.projection_timing[sensor] = stage
            if context is not None and not self.core.fault:
                for sensor in (self.next_sensor, 1-self.next_sensor):
                    if self.inflight[sensor] is not None:
                        continue
                    item = self.pending[sensor]
                    if item is None:
                        continue
                    message, raw, captured_context, received, ingress = item
                    if captured_context != context or time.monotonic()-received > .25:
                        self.pending[sensor] = None
                        self.record_drop('wait', 'ray_projection_wait_expired_or_context_changed', raw)
                        self.error = 'ray_projection_wait_expired_or_context_changed'
                        continue
                    authorized = self.authorized_pose_ns
                    if self.authorized_pose_context != context or raw.end_ns > authorized:
                        self.error = 'waiting_authorized_pose_history'
                        continue  # Original receive/source deadlines still apply.
                    snapshot = self.core.projection_snapshot()
                    if self.joint_history is not None:
                        snapshot.joint_poses = self.joint_history.snapshot()
                    self.pending[sensor] = None
                    timing, started = dict(submitted_steady_ns=time.perf_counter_ns()), time.perf_counter()
                    future = self.worker.submit(timed_projection, snapshot, raw, captured_context,
                        now_ns=self.now_ns(), authorized_pose_ns=authorized, timing=timing)
                    self.inflight[sensor] = (future, item, self.work_generation, started, timing)
                    self.next_sensor = 1-sensor
                    # Register only after the owner ledger is complete: an
                    # already-finished Future invokes this callback inline.
                    # Each source can start after its own predecessor, without
                    # a polling gap or waiting for the other source's geometry.
                    future.add_done_callback(self.projection_completed)
                    # Both slots include old-generation running jobs until the
                    # owner retires them: context churn cannot queue extra work.
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
                processing=any(work is not None for work in self.inflight),
                processing_sources=[work is not None for work in self.inflight],
                local_history_samples=len(self.core.local), alignment_pairs=len(self.core.alignments),
                alignment_displacement=self.core.alignment_displacement,
                alignment_rebuild_limits=dict(
                    translation_m=self.core.limits.max_alignment_translation_from_anchor_m,
                    rotation_rad=self.core.limits.max_alignment_rotation_from_anchor_rad),
                processing_ms=self.processing_ms,
                projection_timing=self.projection_timing,
                completion_wake_failures=self.completion_wake_failures, counts=self.counts,
                drop_stages=self.drop_stages, last_drop=self.last_drop,
                raw_ingress_gap=self.last_raw_gap,
                local_state_diagnostics=self.local_state_diagnostics)
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
            state['entity_self_filter'] = dict(
                enabled=self.entity_filter is not None,
                model_digest=self.entity_filter.digest if self.entity_filter else '',
                applicability=dict(self.entity_filter.applicability) if self.entity_filter else {},
                counts=self.entity_counts, latest=self.entity_reports,
                pose_source=self.entity_filter.pose_source if self.entity_filter else '',
                joint_source=self.entity_filter.settings.get('joint_source', '') if self.entity_filter else '',
                requires_joint_stream=self.entity_filter.requires_joint_stream if self.entity_filter else False,
                joint_samples=self.joint_history.snapshot().sample_count if self.joint_history else 0,
                joint_diagnostics=dict(self.joint_history.diagnostics) if self.joint_history else {},
                physical_calibration_verified=False,
                policy='suppress_model_surface_hits_only_no_near_box_clearing')
            self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
            # Preserve per-source timing in the existing process journal. A
            # healthy LIO stream does not establish that both raw-ray sources
            # reached this independent pipeline. This is passive, at most 1 Hz;
            # it changes no source time, map evidence, queue or motion lease.
            if mono-self.last_pipeline_trace >= 1.:
                age_ms = lambda stamp: (now-stamp)*1e-6 if stamp else float('nan')
                active = [work for work in self.inflight if work is not None]
                oldest = min(active, key=lambda work: work[3]) if active else None
                inflight_source = oldest[1][1].start_ns if oldest else 0
                inflight_ms = (time.perf_counter()-oldest[3])*1000. if oldest else 0.
                self.get_logger().info(
                    'RayProjectionTrace raw_front_ns=%d raw_rear_ns=%d '
                    'published_front_ns=%d published_rear_ns=%d '
                    'raw_front_age_ms=%.3f raw_rear_age_ms=%.3f '
                    'published_front_age_ms=%.3f published_rear_age_ms=%.3f '
                    'pending_front=%d pending_rear=%d inflight_source_ns=%d '
                    'inflight_ms=%.3f processing_ms=%.3f received=%d published=%d '
                    'dropped=%d overwritten=%d reason=%s drop_stages=%s last_drop=%s raw_ingress_gap=%s '
                    'local_state_diagnostics=%s projection_timing=%s' % (
                        *self.last_raw_stamp, *self.last_published,
                        *(age_ms(s) for s in self.last_raw_stamp),
                        *(age_ms(s) for s in self.last_published),
                        int(self.pending[0] is not None), int(self.pending[1] is not None),
                        inflight_source, inflight_ms, self.processing_ms,
                        self.counts['received'], self.counts['published'],
                        self.counts['dropped'], self.counts['overwritten'], self.error,
                        json.dumps(self.drop_stages, separators=(',', ':')),
                        json.dumps(self.last_drop, separators=(',', ':')),
                        json.dumps(self.last_raw_gap, separators=(',', ':')),
                        json.dumps(self.local_state_diagnostics, separators=(',', ':')),
                        json.dumps(self.projection_timing, separators=(',', ':'))))
                self.last_pipeline_trace = mono
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
    from rclpy._rclpy_pybind11 import RCLError
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_node(settings)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
            pass
        except RCLError:
            # SIGINT can invalidate the context between the executor's check
            # and WaitSet construction. Do not mask an actual live RCL fault.
            if rclpy.ok():
                raise
    finally:
        try:
            if node is not None:
                node.destroy_node()  # joins/cancels the bounded geometry worker
        finally:
            rclpy.try_shutdown()  # also safe after the signal already shut down


if __name__ == '__main__':
    main()
