"""Exercise the real node callbacks against an in-memory ROS facade, no ROS init."""
from array import array
import json
import sys
from threading import Event
from types import ModuleType, SimpleNamespace as NS

import pytest

from d1max_pct_scan.ray_projection import FIELDS, MapContext, session_settings
from test_ray_projection import EPOCH, raw_points, session_config


def stamp(ns):
    return NS(sec=ns//1000000000, nanosec=ns%1000000000)


def pose(x=0.):
    return NS(position=NS(x=x, y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class Cloud:
    def __init__(self):
        self.header = NS(frame_id='', stamp=stamp(0))


class ProjectedRays:
    def __init__(self):
        self.acquisition_end = stamp(0)
        self.alignment_stamp = stamp(0)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Guard:
    """Coalescing executor wake, never an inline worker-thread callback."""
    def __init__(self, callback, callback_group):
        self.callback, self.callback_group = callback, callback_group
        self.ready = Event()
        self.triggers = 0
        self.destroyed = False

    def trigger(self):
        if self.destroyed:
            raise RuntimeError('guard destroyed')
        self.triggers += 1
        self.ready.set()

    def dispatch(self):
        assert self.ready.wait(2.)
        self.ready.clear()
        self.callback()


class FakeNode:
    def __init__(self, name):
        self._now = EPOCH+180000000
        self.publishers = {}
        self.subscriptions = []
        self.destroyed_subscriptions = []
        self.pipeline_logs = []
        self.default_callback_group = object()
        self.guards, self.timers = [], []

    def create_publisher(self, kind, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, *args):
        self.subscriptions.append(args)
        return args

    def destroy_subscription(self, subscription):
        self.destroyed_subscriptions.append(subscription)
        self.subscriptions.remove(subscription)
        return True

    def create_timer(self, *args, **kwargs):
        self.timers.append((args, kwargs))
        return None

    def create_guard_condition(self, callback, *, callback_group):
        guard = Guard(callback, callback_group)
        self.guards.append(guard)
        return guard

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self._now))

    def get_logger(self):
        return NS(info=self.pipeline_logs.append)

    def destroy_node(self):
        for guard in self.guards:
            guard.destroyed = True


@pytest.fixture
def node(monkeypatch):
    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
        'rclpy.qos': {'QoSProfile': lambda **kw: kw,
            'DurabilityPolicy': NS(TRANSIENT_LOCAL=1, VOLATILE=2),
            'ReliabilityPolicy': NS(BEST_EFFORT=1), 'qos_profile_sensor_data': object()},
        'sensor_msgs.msg': {'PointCloud2': Cloud, 'PointField': lambda **kw: NS(**kw)}, 'nav_msgs.msg': {'Odometry': object},
        'geometry_msgs.msg': {'PoseStamped': object}, 'std_msgs.msg': {'String': lambda **kw: NS(**kw)},
        'tf2_msgs.msg': {'TFMessage': object},
        'd1max_planning_interfaces.msg': {'ProjectedRays': ProjectedRays,
            'NavigationState':object,'LocalNavigationState':object},
    }
    for name, values in modules.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    from d1max_pct_scan.perception_ray_projector import create_node
    session, config = session_config()
    value = create_node(session_settings(session, config))
    yield value
    value.destroy_node()


def message(value):
    return NS(data=json.dumps(value))


def context(sequence=1, seed='seed', barrier=EPOCH-1):
    return dict(schema=1, session_id='session', epoch=1, seed_id=seed,
                sequence=sequence, barrier_ns=barrier)


def statuses(node):
    now = node._now*1e-9
    nav = dict(schema=1, epoch=1, seed_id='seed', fault=False, reset_pending=False,
               received_at_unix=now)
    local = dict(session_id='session', wall_time=now, local_fault=False, local_epoch=1,
        active_seed_ns='seed', confirmed_seed_ns='seed', verified_confirmations=3,
        navigation=nav, frames=dict(map='map', tracking='tracking'))
    public = dict(schema=1, epoch=1, seed_id='seed', received_at_unix=now,
        output_stamp_sec=(EPOCH+160000000)*1e-9, pose_timeout_sec=.08,
        valid=True, pose_valid=True, reset_pending=False, fault=False,
        motion_control_enabled=False, frame_id='map', body_frame='body')
    for key, value in (('localizer', local), ('navigation', nav), ('pose', public)):
        node.on_status(key, message(value))


def static(node):
    edges = []
    for parent, child in (('body', 'tracking'), ('tracking', 'lidar')):
        edges.append(NS(header=NS(frame_id=parent), child_frame_id=child,
            transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))))
    node.on_static(NS(transforms=edges))


def ready(node):
    static(node)
    statuses(node)
    node.on_context(message(context()))
    node.on_ack(message(context()))
    for i in range(9):
        header = NS(stamp=stamp(EPOCH+i*20000000), frame_id='odom')
        node.on_local(NS(header=header, child_frame_id='body', pose=NS(pose=pose(i*.02))))
        header = NS(stamp=stamp(EPOCH+i*20000000), frame_id='map')
        node.on_global(NS(header=header, pose=pose(i*.02)))


def raw_cloud(sensor=0, start=EPOCH):
    points = raw_points(sensor=sensor, start=start)
    return NS(header=NS(frame_id='lidar', stamp=stamp(start)), width=len(points), height=1,
        point_step=64, row_step=len(points)*64, is_bigendian=False,
        fields=[NS(name=n, offset=o, datatype=t, count=c) for n, o, t, c in FIELDS],
        data=array('B', points.tobytes()))


def local_state_packet(source_ns, *, usable=True, reason='local_navigation_unavailable'):
    """The real inbox contract, represented without creating ROS objects."""
    velocity = lambda: NS(x=0., y=0., z=0.)
    return NS(schema_version=2, usable=usable, session_id='session', map_version_id='map',
        localization_epoch=1, localization_seed_id='seed', reason=reason,
        source_stamp=stamp(source_ns), posterior_stamp=stamp(source_ns-40_000_000),
        imu_stamp=stamp(source_ns-20_000_000), valid_until=stamp(source_ns+100_000_000),
        extrapolation_sec=.02,
        local_odometry=NS(header=NS(stamp=stamp(source_ns), frame_id='d1max_loc_odom'),
            child_frame_id='d1max_loc_base_link', pose=NS(pose=pose()),
            twist=NS(twist=NS(linear=velocity(), angular=velocity()))))


def typed_local_ready(node):
    node.p.update(typed_navigation_state=True, independent_local_state=True, map_version_id='map')
    static(node)
    node.on_context(message(context()))
    node.on_ack(message(context()))


def test_local_history_diagnostics_distinguish_receipts_rejection_and_real_gap(node):
    typed_local_ready(node)
    first = EPOCH+180_000_000
    node._now = first
    node.on_local_navigation(local_state_packet(first))
    second = first+60_238_592  # Observed log interval; the 60 ms limit is unchanged.
    node._now = second
    node.on_local_navigation(local_state_packet(second))
    diagnostics = node.local_state_diagnostics
    assert diagnostics['counts']['received'] == diagnostics['counts']['history_added'] == 2
    assert diagnostics['last_history_gap'] == dict(
        received_ns=second, source_ns=second, valid_until_ns=second+100_000_000,
        source_age_ns=0, past_deadline=False, previous_source_ns=first, gap_ns=60_238_592)
    assert diagnostics['max_history_gap_ns'] == 60_238_592
    assert node.core.limits.max_pose_gap_sec == .06
    gap_evidence = dict(diagnostics['last_history_gap'])
    third = second+20_000_000
    node._now = third
    node.on_local_navigation(local_state_packet(third))
    assert diagnostics['last_history']['gap_ns'] == 20_000_000
    assert diagnostics['last_history_gap'] == gap_evidence  # a later success cannot erase the hole
    history = list(node.core.local)
    authorized = node.authorized_pose_ns
    event = node.local_inbox.latest
    expired = third+20_000_000
    node._now = expired+400_000_001
    node.on_local_navigation(local_state_packet(expired))
    assert diagnostics['counts']['rejected'] == 1
    assert diagnostics['last_rejected'] == dict(
        received_ns=node._now, source_ns=expired, valid_until_ns=expired+100_000_000,
        source_age_ns=400_000_001, past_deadline=True, result='rejected_unknown_reason')
    assert node.local_inbox.latest is event
    assert list(node.core.local) == history and node.authorized_pose_ns == authorized
    # No fresh receipt means no new history; reporting does not manufacture one.
    count_before = dict(diagnostics['counts'])
    for mono in (10., 10.5, 11.):
        node.publish_status(node.current_context(), node._now, mono)
    assert diagnostics['counts'] == count_before
    assert len(node.pipeline_logs) == 2
    assert 'local_state_diagnostics=' in node.pipeline_logs[-1]
    assert list(node.core.local) == history and node.authorized_pose_ns == authorized


def test_local_history_diagnostics_are_bounded_and_do_not_change_revocation(node):
    typed_local_ready(node)
    source = node._now
    node.on_local_navigation(local_state_packet(source))
    authorized = node.authorized_pose_ns
    history = list(node.core.local)
    for _ in range(100):
        node.on_local_navigation(NS())  # Missing fields stay unknown, not zero source stamps.
    diagnostics = node.local_state_diagnostics
    assert diagnostics['counts']['received'] == 101
    assert diagnostics['counts']['rejected'] == 100
    assert diagnostics['last_rejected'] == dict(received_ns=node._now,
        source_ns=None, valid_until_ns=None, result='rejected_unknown_reason')
    assert set(diagnostics) == {'counts', 'last_received', 'last_rejected',
        'last_history', 'last_history_gap', 'max_history_gap_ns'}
    assert len(diagnostics['counts']) == 10
    assert list(node.core.local) == history and node.authorized_pose_ns == authorized
    node._now = source+1
    node.on_local_navigation(local_state_packet(node._now, usable=False))
    assert diagnostics['counts']['soft_unavailable'] == 1
    assert list(node.core.local) == history and node.authorized_pose_ns == authorized
    node._now += 1
    node.on_local_navigation(local_state_packet(node._now, usable=False, reason='lio_reset'))
    assert diagnostics['counts']['hard_unavailable'] == 1
    assert not node.core.local and node.atomic_state is None


@pytest.mark.parametrize('historical, published', [(False, 0), (True, 1)])
def test_identical_delayed_pose_callbacks_no_longer_create_a_false_scan_history_hole(
        node, historical, published):
    """Same input against old control admission and new historical admission.

    This is a production-callback counterexample, not a claim that the recorded
    60.238592 ms hole had the same cause (the old log lacks receipt evidence).
    """
    from d1max_pct_scan.local_navigation_state import LocalNavigationInbox, LocalNavigationHistoryInbox
    node.local_inbox = LocalNavigationHistoryInbox() if historical else LocalNavigationInbox()
    typed_local_ready(node)
    node.core.projection_frame = 'odom'
    node.p['output_frame'] = 'odom'
    node._now = EPOCH+20_000_000
    node.on_local_navigation(local_state_packet(EPOCH))
    node._now = EPOCH+200_000_000
    for i in range(1, 11):
        node.on_local_navigation(local_state_packet(EPOCH+i*20_000_000))
    history = list(node.core.local)
    assert node.authorized_pose_ns == EPOCH+200_000_000
    assert len(history) == (11 if historical else 7)
    cloud = raw_cloud()
    original = cloud.data.tobytes()
    node.on_rays(cloud)
    node.tick()
    try:
        node.inflight[0][0].result(timeout=2.)
    except ValueError:
        assert not historical
    node.tick()
    assert len(node.publisher.messages) == published
    assert cloud.data.tobytes() == original
    assert node.core.limits.max_pose_gap_sec == .06
    assert node.local_state_diagnostics['counts']['history_after_control_deadline'] == (4 if historical else 0)
    if historical:
        output = node.publisher.messages[0]
        assert output.rays.header.stamp == cloud.header.stamp
        assert output.acquisition_end == stamp(EPOCH+100_000_000)
        assert node.counts['dropped'] == 0
        assert not hasattr(node.local_inbox, 'usable')
    else:
        assert node.last_drop['reason'] == 'local_pose_bracketing_gap'
        assert node.last_drop['pose_gap']['right_ns']-node.last_drop['pose_gap']['left_ns'] == 100_000_000


def test_pipeline_journal_is_rate_limited_and_does_not_renew_pending_or_source(node):
    ready(node)
    source=raw_cloud()
    node.on_rays(source)
    pending=node.pending[0]
    raw=list(node.last_raw_stamp)
    published=list(node.last_published)
    context=node.current_context()
    node.publish_status(context,node._now,10.)
    node.publish_status(context,node._now,10.5)
    node.publish_status(context,node._now,11.)
    assert len(node.pipeline_logs)==2
    assert 'raw_front_ns=%d' % raw[0] in node.pipeline_logs[-1]
    assert 'published_front_ns=0' in node.pipeline_logs[-1]
    assert node.last_raw_stamp==raw and node.last_published==published
    assert node.pending[0] is pending and node.pending[0][3]==pending[3]
    assert node.inflight == [None, None] and not node.publisher.messages


def test_real_callbacks_publish_typed_context_original_time_and_per_point_map_origin(node):
    ready(node)
    source = raw_cloud()
    original = source.data.tobytes()
    node.on_rays(source)
    assert node.last_published == [0, 0]  # receipt must not masquerade as a published frame
    node.tick()
    projected = node.inflight[0][0].result(timeout=2.)  # pure worker, no ROS context or hardware
    node.tick()
    assert len(node.publisher.messages) == 1
    result = node.publisher.messages[0]
    # Final storage is prepared off the callback thread, including its owning
    # lifetime. Publishing must neither repack it nor alias the input message.
    assert result.rays.data is projected.wire_data
    assert result.rays.data is not source.data
    assert result.rays.data.tobytes() == projected.points.tobytes()
    assert source.data.tobytes() == original
    assert result.session_id == 'session' and result.context_sequence == 1
    assert result.epoch == 1 and result.seed_id == 'seed' and result.barrier_ns == EPOCH-1
    assert result.rays.header.frame_id == 'map'
    assert result.rays.header.stamp.sec == EPOCH//1000000000
    assert result.acquisition_end.nanosec == 100000000
    assert result.alignment_stamp.nanosec == 100000000
    assert result.projection_sequence == 1
    assert node.last_published == [EPOCH, 0]
    status = json.loads(node.status_pub.messages[-1].data)
    assert status['projection_ready'] is True and status['valid'] is False  # rear not observed yet


def test_context_requires_both_owner_request_and_exact_native_ack(node):
    static(node)
    statuses(node)
    node.on_ack(message(context()))
    assert node.current_context() is None
    node.on_context(message(context()))
    assert node.current_context() == MapContext.parse(context())
    node.on_context(message(context(sequence=2)))
    assert node.current_context() is None
    assert node.core.context is None
    node.on_ack(message(context(sequence=1)))
    assert node.current_context() is None
    node.on_ack(message(context(sequence=2)))
    assert node.current_context().sequence == 2


def test_new_context_clears_old_pending_rays_before_ack_and_does_not_forward_them(node):
    ready(node)
    node.on_rays(raw_cloud())
    assert node.pending[0]
    node.on_context(message(context(sequence=2, barrier=EPOCH+90000000)))
    assert node.pending == [None, None] and not node.core.local
    node.tick()
    assert not node.publisher.messages


def test_missing_static_edge_cannot_default_to_body_equals_tracking(node):
    statuses(node)
    node.on_context(message(context()))
    node.on_ack(message(context()))
    node.on_local(NS(header=NS(frame_id='odom', stamp=stamp(EPOCH)),
                     child_frame_id='body', pose=NS(pose=pose())))
    assert not node.core.local and node.counts['pose_rejected'] == 1


def test_static_reader_uses_tf_depth_and_retries_only_while_missing(node):
    assert node.static_qos['depth'] == 100
    original = node.last_static_retry
    node.retry_static_extrinsics(original+.99)
    assert not node.destroyed_subscriptions
    node.retry_static_extrinsics(original+1.)
    assert node.static_retry_count == 1
    assert len([item for item in node.subscriptions if item[1] == '/tf_static']) == 1
    assert len(node.destroyed_subscriptions) == 1
    assert node.core.body_to_tracking is None  # no guessed fallback
    static(node)
    node.retry_static_extrinsics(original+5.)
    assert node.static_retry_count == 1
    assert not node.missing_static_edges()


def test_late_separate_static_publishers_restore_pose_history_without_reset(node):
    statuses(node)
    node.on_context(message(context()))
    node.on_ack(message(context()))
    body = NS(header=NS(frame_id='odom', stamp=stamp(EPOCH)),
              child_frame_id='body', pose=NS(pose=pose()))
    node.on_local(body)
    assert not node.core.local
    for parent, child in (('tracking', 'lidar'), ('body', 'tracking')):
        node.on_static(NS(transforms=[NS(header=NS(frame_id=parent), child_frame_id=child,
            transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))]))
    node.on_local(body)
    assert len(node.core.local) == 1 and node.counts['context_resets'] == 1
    node.publish_status(node.current_context(), node.now_ns(), node.last_static_retry)
    status = json.loads(node.status_pub.messages[-1].data)['static_extrinsics']
    assert status['ready'] and not status['missing']


def test_static_changed_fault_is_not_hidden_by_resubscription(node):
    ready(node)
    node.core.fault = 'static_extrinsic_changed_requires_context_reset'
    node.static_edges.clear()
    node.retry_static_extrinsics(node.last_static_retry+2.)
    assert not node.destroyed_subscriptions
    assert node.core.fault == 'static_extrinsic_changed_requires_context_reset'


def test_missing_static_retries_back_off_without_duplicating_readers(node):
    started = node.last_static_retry
    node.retry_static_extrinsics(started+1.)
    assert node.static_retry_count == 1 and node.static_retry_delay == 2.
    node.retry_static_extrinsics(started+2.)
    assert node.static_retry_count == 1
    for elapsed in (3., 7., 15., 23.):
        node.retry_static_extrinsics(started+elapsed)
    assert node.static_retry_count == 5 and node.static_retry_delay == 8.
    assert len([item for item in node.subscriptions if item[1] == '/tf_static']) == 1
    node.publish_status(None, node.now_ns(), started+23.)
    static_status = json.loads(node.status_pub.messages[-1].data)['static_extrinsics']
    assert static_status['ready'] is False
    assert static_status['missing'] == [['body', 'tracking'], ['tracking', 'lidar']]


def test_expired_pending_frame_cannot_have_its_wait_budget_renewed_by_duplicate(node):
    ready(node)
    node.core.local.clear()
    node.on_rays(raw_cloud())
    original = node.pending[0]
    node.on_rays(raw_cloud())
    assert node.pending[0] is original
    node.pending[0] = (*original[:3], original[3]-.3, original[4])
    node.tick()
    assert node.pending[0] is None and not node.publisher.messages


def test_invalid_status_immediately_blocks_pending_projection(node):
    ready(node)
    node.on_rays(raw_cloud())
    node.on_status('pose', message(['not', 'an', 'object']))
    assert node.pending == [None, None]
    node.tick()
    assert not node.publisher.messages


def test_out_of_order_cloud_and_old_ack_cannot_replace_new_context(node):
    ready(node)
    node.on_context(message(context(sequence=2)))
    node.on_ack(message(context(sequence=2)))
    node.on_context(message(context(sequence=1)))
    node.on_ack(message(context(sequence=1)))
    assert node.current_context().sequence == 2


def test_rear_pending_not_overwritten_by_new_front_cloud(node):
    ready(node)
    node.on_rays(raw_cloud(1))
    rear = node.pending[1]
    node.on_rays(raw_cloud(0))
    node.on_rays(raw_cloud(0, EPOCH+1000000))
    assert node.pending[1] is rear
    assert node.counts['overwritten'] == 1


def test_completed_old_worker_snapshot_cannot_publish_after_context_transition(node):
    ready(node)
    node.on_rays(raw_cloud())
    node.tick()
    node.inflight[0][0].result(timeout=2.)
    node.on_context(message(context(sequence=2, barrier=EPOCH+90000000)))
    node.on_ack(message(context(sequence=2, barrier=EPOCH+90000000)))
    node.tick()
    assert not node.publisher.messages


def test_alignment_fault_immediately_publishes_context_bound_revoke_and_drops_worker(node):
    ready(node)
    node.on_rays(raw_cloud())
    node.tick()
    node.inflight[0][0].result(timeout=2.)
    # Single correction remains below the old 0.20 m step limit, but exceeds
    # the independent 0.10 m context-history bound.
    node.on_local(NS(header=NS(frame_id='odom', stamp=stamp(EPOCH+180000000)),
                     child_frame_id='body', pose=NS(pose=pose())))
    node.on_global(NS(header=NS(frame_id='map', stamp=stamp(EPOCH+180000000)),
                      pose=pose(.108)))
    status = json.loads(node.status_pub.messages[-1].data)
    assert status['fault'] == 'map_alignment_accumulation_requires_context_reset'
    assert status['context']['sequence'] == 1 and status['valid'] is False
    assert status['alignment_displacement']['translation_m'] == pytest.approx(.108)
    assert status['alignment_rebuild_limits'] == dict(translation_m=.10, rotation_rad=.05)
    fault_status_count = len(node.status_pub.messages)
    node.on_global(NS(header=NS(frame_id='map', stamp=stamp(EPOCH+180000000)),
                      pose=pose(.108)))
    assert len(node.status_pub.messages) == fault_status_count  # no pose-rate fault flood
    node.tick()
    assert not node.publisher.messages
    node.on_context(message(context()))
    node.on_ack(message(context()))
    assert node.core.fault is not None
    node.on_context(message(context(sequence=2, barrier=EPOCH+180000000)))
    assert node.current_context() is None
    node.on_ack(message(context(sequence=2, barrier=EPOCH+180000000)))
    assert node.current_context().sequence == 2 and node.core.fault is None
    assert node.core.sequence == 0


def test_worker_geometry_never_holds_pose_status_callbacks_or_refreshes_invalid_lease(node, monkeypatch):
    import threading
    from d1max_pct_scan.ray_projection import RayProjectorCore
    entered, release = threading.Event(), threading.Event()
    original = RayProjectorCore.project

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(2.)
        return original(*args, **kwargs)

    monkeypatch.setattr(RayProjectorCore, 'project', blocked)
    ready(node)
    node.on_rays(raw_cloud())
    node.tick()
    try:
        assert entered.wait(1.)
        # This callback runs while geometry is in flight; invalidating it cannot
        # be undone by the worker's older still-valid private status snapshot.
        node.on_status('pose', message(['invalid']))
        assert node.current_context() is None and node.pending == [None, None]
    finally:
        release.set()
    node.inflight[0][0].result(timeout=2.)
    node.tick()
    assert not node.publisher.messages


@pytest.mark.parametrize('sensors', [(0,), (1,), (0, 1)])
def test_raw_qos_is_bounded_sensor_compatible_not_reliable_backlog(node, sensors):
    from d1max_pct_scan.perception_ray_projector import create_node
    value = create_node(dict(node.p, sensor_ids=sensors))
    try:
        assert value.raw_qos == dict(depth=len(sensors), reliability=1, durability=2)
        assert [item[3] for item in value.subscriptions if item[1] == value.p['raw_topic']] == [value.raw_qos]
    finally:
        value.destroy_node()


def test_full_sample_validation_is_worker_only_and_keeps_identity_callbacks_responsive(node, monkeypatch):
    import threading
    from d1max_pct_scan import ray_projection
    original = ray_projection.validate_ray_samples
    entered, release = threading.Event(), threading.Event()
    callback_thread = threading.get_ident()
    def blocked(raw):
        assert threading.get_ident() != callback_thread
        entered.set()
        assert release.wait(2.)
        return original(raw)
    monkeypatch.setattr(ray_projection, 'validate_ray_samples', blocked)
    ready(node)
    node.on_rays(raw_cloud())
    assert not entered.is_set()  # no full sample scans in the input callback
    node.tick()
    try:
        assert entered.wait(1.)
        node.on_status('pose', message(['invalid']))
        assert node.current_context() is None
    finally:
        release.set()
    node.inflight[0][0].result(timeout=2.)
    node.tick()
    assert not node.publisher.messages


@pytest.mark.parametrize('field,value', [('x', float('nan')),
    ('source_timestamp', float('inf')), ('source_index', 0), ('origin_x', .1)])
def test_queued_invalid_samples_never_project_publish_or_renew_source(node, field, value):
    from d1max_pct_scan.ray_projection import ProjectionError
    ready(node)
    cloud = raw_cloud()
    points = raw_points()
    points[field][4] = value
    cloud.data = array('B', points.tobytes())
    node.on_rays(cloud)
    assert node.pending[0]  # only the envelope is admitted to a bounded queue
    node.tick()
    with pytest.raises(ProjectionError):
        node.inflight[0][0].result(timeout=2.)
    node.tick()
    assert not node.publisher.messages and node.last_published == [0, 0]
    assert node.core.sequence == 0 and node.core.last_input == [-1, -1]


def test_receipt_deadline_starts_before_envelope_work_and_is_not_refreshed(node, monkeypatch):
    import time
    from d1max_pct_scan import perception_ray_projector
    original = perception_ray_projector.decode_ray_envelope
    decode_started = []
    def envelope(**arguments):
        decode_started.append(time.monotonic())
        return original(**arguments)
    monkeypatch.setattr(perception_ray_projector, 'decode_ray_envelope', envelope)
    ready(node)
    node.on_rays(raw_cloud())
    item = node.pending[0]
    assert item[3] <= decode_started[0]
    node.on_rays(raw_cloud())
    assert node.pending[0] is item


def test_overload_keeps_latest_per_sensor_and_two_jobs_without_hidden_backlog(node, monkeypatch):
    import threading
    from d1max_pct_scan import ray_projection
    entered, release = threading.Event(), threading.Event()
    original = ray_projection.validate_ray_samples
    submissions = []
    submit = node.worker.submit
    def tracked_submit(*args, **kwargs):
        submissions.append(args)
        return submit(*args, **kwargs)
    def blocked(raw):
        entered.set()
        assert release.wait(2.)
        return original(raw)
    monkeypatch.setattr(node.worker, 'submit', tracked_submit)
    monkeypatch.setattr(ray_projection, 'validate_ray_samples', blocked)
    ready(node)
    node.on_rays(raw_cloud())
    node.tick()
    try:
        assert entered.wait(1.)
        node.on_rays(raw_cloud(1))
        for index in range(1, 20):
            node.on_rays(raw_cloud(0, EPOCH+index*100000))
            node.tick()
        assert len(submissions) == 2
        assert node.pending[0][1].start_ns == EPOCH+1900000
        assert node.pending[1] is None and node.inflight[1][1][1].sensor_id == 1
        assert node.counts['overwritten'] == 18
    finally:
        release.set()
    for work in node.inflight:
        work[0].result(timeout=2.)
    node.tick()
    assert len(submissions) == 3
    assert node.inflight[0][1][1].start_ns == EPOCH+1900000
    assert node.inflight[1] is None and node.last_published[1] == EPOCH


@pytest.fixture
def main_runtime(node, monkeypatch, tmp_path):
    """Run the entry point only against the existing in-memory ROS facade."""
    import rclpy
    from d1max_pct_scan import live_view_reload, perception_ray_projector
    class RCLError(RuntimeError):
        pass
    class ExternalShutdownException(Exception):
        pass
    implementation = ModuleType('rclpy._rclpy_pybind11')
    implementation.RCLError = RCLError
    monkeypatch.setitem(sys.modules, 'rclpy._rclpy_pybind11', implementation)
    calls = []
    state = NS(context_ok=True, spin_error=None, create_error=None, destroy_error=None)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    monkeypatch.setattr(sys, 'argv', ['perception_ray_projector', '--session', str(tmp_path)])
    (tmp_path/'localization.yaml').write_text('{}\n')
    monkeypatch.setattr(live_view_reload, 'read_owned_json', lambda path: {})
    monkeypatch.setattr(perception_ray_projector, 'session_settings', lambda *args: node.p)
    def initialize(**kwargs):
        calls.append('init')  # fake only: never invokes rclpy.init or a ROS graph
    def create(settings):
        calls.append('create')
        if state.create_error:
            raise state.create_error
        return node
    def spin(value):
        assert value is node
        calls.append('spin')
        if state.spin_error:
            raise state.spin_error
    destroy, shutdown = node.destroy_node, node.worker.shutdown
    def destroy_node():
        calls.append('destroy')
        result = destroy()
        if state.destroy_error:
            raise state.destroy_error
        return result
    def stop_worker(**kwargs):
        assert kwargs == dict(wait=True, cancel_futures=True)
        calls.append('worker_shutdown')
        return shutdown(**kwargs)
    monkeypatch.setattr(rclpy, 'init', initialize, raising=False)
    monkeypatch.setattr(rclpy, 'spin', spin, raising=False)
    monkeypatch.setattr(rclpy, 'ok', lambda: state.context_ok, raising=False)
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda: calls.append('try_shutdown'), raising=False)
    monkeypatch.setattr(rclpy, 'executors', NS(ExternalShutdownException=ExternalShutdownException), raising=False)
    monkeypatch.setattr(perception_ray_projector, 'create_node', create)
    monkeypatch.setattr(node, 'destroy_node', destroy_node)
    monkeypatch.setattr(node.worker, 'shutdown', stop_worker)
    try:
        yield NS(main=perception_ray_projector.main, state=state, calls=calls,
                 RCLError=RCLError, ExternalShutdownException=ExternalShutdownException)
    finally:
        state.destroy_error = None  # the enclosing node fixture also tears down


@pytest.mark.parametrize('error,context_ok,propagates', [
    ('rcl', False, False), ('rcl', True, True),
    ('runtime', False, True), ('runtime', True, True),
    ('interrupt', True, False), ('external_shutdown', False, False),
    ('none', True, False)])
def test_main_only_suppresses_rcl_error_after_shutdown_and_always_joins_worker(
        main_runtime, error, context_ok, propagates):
    runtime = main_runtime
    errors = dict(rcl=runtime.RCLError('WaitSet context invalid'),
                  runtime=RuntimeError('actual callback failure'),
                  interrupt=KeyboardInterrupt(),
                  external_shutdown=runtime.ExternalShutdownException(), none=None)
    runtime.state.context_ok = context_ok
    runtime.state.spin_error = errors[error]
    if propagates:
        with pytest.raises(type(errors[error])) as raised:
            runtime.main()
        assert raised.value is errors[error]
    else:
        runtime.main()
    assert runtime.calls == ['init', 'create', 'spin', 'destroy', 'worker_shutdown', 'try_shutdown']


def test_main_try_shutdown_even_when_node_creation_fails(main_runtime):
    runtime = main_runtime
    runtime.state.create_error = RuntimeError('node setup failure')
    with pytest.raises(RuntimeError, match='node setup failure'):
        runtime.main()
    assert runtime.calls == ['init', 'create', 'try_shutdown']


def test_main_try_shutdown_even_when_destroy_fails_after_worker_join(main_runtime):
    runtime = main_runtime
    runtime.state.destroy_error = RuntimeError('node destruction failure')
    with pytest.raises(RuntimeError, match='node destruction failure'):
        runtime.main()
    assert runtime.calls == ['init', 'create', 'spin', 'destroy', 'worker_shutdown', 'try_shutdown']
