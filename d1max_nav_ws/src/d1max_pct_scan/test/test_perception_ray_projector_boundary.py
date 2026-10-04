"""Exercise the real node callbacks against an in-memory ROS facade, no ROS init."""
from array import array
import json
import sys
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


class FakeNode:
    def __init__(self, name):
        self._now = EPOCH+180000000
        self.publishers = {}
        self.subscriptions = []
        self.destroyed_subscriptions = []

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

    def create_timer(self, *args):
        return None

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self._now))

    def destroy_node(self):
        pass


@pytest.fixture
def node(monkeypatch):
    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
        'rclpy.qos': {'QoSProfile': lambda **kw: kw,
            'DurabilityPolicy': NS(TRANSIENT_LOCAL=1), 'qos_profile_sensor_data': object()},
        'sensor_msgs.msg': {'PointCloud2': Cloud}, 'nav_msgs.msg': {'Odometry': object},
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


def test_real_callbacks_publish_typed_context_original_time_and_per_point_map_origin(node):
    ready(node)
    node.on_rays(raw_cloud())
    assert node.last_published == [0, 0]  # receipt must not masquerade as a published frame
    node.tick()
    node.inflight[0].result(timeout=2.)  # pure worker, no ROS context or hardware
    node.tick()
    assert len(node.publisher.messages) == 1
    result = node.publisher.messages[0]
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
    node.pending[0] = (*original[:3], original[3]-.3)
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
    node.inflight[0].result(timeout=2.)
    node.on_context(message(context(sequence=2, barrier=EPOCH+90000000)))
    node.on_ack(message(context(sequence=2, barrier=EPOCH+90000000)))
    node.tick()
    assert not node.publisher.messages


def test_alignment_fault_immediately_publishes_context_bound_revoke_and_drops_worker(node):
    ready(node)
    node.on_rays(raw_cloud())
    node.tick()
    node.inflight[0].result(timeout=2.)
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
    node.inflight[0].result(timeout=2.)
    node.tick()
    assert not node.publisher.messages
