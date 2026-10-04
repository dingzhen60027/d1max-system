"""Exercise production callbacks without nodes, graph, SDK or restamped data."""
from copy import deepcopy
from dataclasses import replace
from types import MethodType, SimpleNamespace

import pytest
from rclpy.time import Time

from d1max_pct_scan.atomic_navigation_inbox import AtomicNavigationInbox
from d1max_pct_scan.live_global_planner import LiveGlobalPlanner
from test_bt_action_adapters import harness, commit
from test_continuous_reference_transport import state
from test_global_mission_lifecycle import lifecycle


def fixture(monkeypatch, kind):
    if kind == 'adapter':
        node, clock = harness(monkeypatch)
        node.p.update(pipeline_contract='single_floor_v3', map_version_id='map')
        ready = node.ready_identity
    else:
        node, clock, *_ = lifecycle(monkeypatch)
        node.pause.clear()
        node.p.update(pipeline_contract='single_floor_v3', map_version_id='map',
                      session_id='session', freshness_s=.5)
        node.atomic_state = None
        node.atomic_inbox = AtomicNavigationInbox()
        node.get_clock = lambda: SimpleNamespace(now=lambda:Time(nanoseconds=round(clock.wall*1e9)))
        node.on_atomic_state = MethodType(LiveGlobalPlanner.on_atomic_state, node)
        node.confirmed_identity = MethodType(LiveGlobalPlanner.confirmed_identity, node)
        ready = node.confirmed_identity
    def send(**kwargs):
        message = state(t=clock.wall-10., **kwargs)
        node.on_atomic_state(message)
        return message
    return node, clock, ready, send


@pytest.mark.parametrize('kind', ['adapter','global'])
@pytest.mark.parametrize('bad', ['old_epoch','duplicate','bad_pose','bad_frame','foreign_map','future','bad_unavailable'])
def test_invalid_packets_preserve_last_measurement_without_renewal(monkeypatch, kind, bad):
    node, clock, ready, send = fixture(monkeypatch, kind)
    message = send(epoch=2)
    prior = node.atomic_state
    receipt = node.body_received
    clock.wall += .02; clock.mono += .02
    if bad == 'duplicate': message.global_odometry.pose.pose.position.x = 123.
    else: message = state(t=clock.wall-10., epoch=2)
    if bad == 'old_epoch': message.localization_epoch = 1
    elif bad == 'bad_pose': message.global_odometry.pose.pose.position.x = float('nan')
    elif bad == 'bad_frame': message.local_odometry.header.frame_id = 'map'
    elif bad == 'foreign_map': message.map_version_id = 'other'
    elif bad == 'future': message.source_stamp.sec += 1
    elif bad == 'bad_unavailable': message.usable=False; message.schema_version=1
    node.on_atomic_state(message)
    assert node.atomic_state is prior and node.body_received == receipt
    assert ready() is not None
    clock.wall += .081; clock.mono += .081  # original IMU TTL, not callback TTL
    with pytest.raises(ValueError): ready()
    assert node.body_received == receipt


@pytest.mark.parametrize('kind', ['adapter','global'])
def test_newest_unavailable_is_not_bad_geometry_and_old_usable_cannot_revive(monkeypatch, kind):
    node, clock, ready, send = fixture(monkeypatch, kind)
    old = send()
    clock.wall += .01; clock.mono += .01
    unavailable = state(t=clock.wall-10., usable=False)
    unavailable.reason='explicit_localizer_loss'
    unavailable.global_odometry.pose.pose.orientation.w = 0.
    unavailable.imu_stamp.sec = 0
    node.on_atomic_state(unavailable)
    assert node.atomic_state is None and node.body is None
    with pytest.raises(ValueError): ready()
    receipt = node.body_received
    node.on_atomic_state(old)
    assert node.atomic_state is None and node.body_received == receipt
    clock.wall += .01; clock.mono += .01
    send()
    assert ready() is not None


@pytest.mark.parametrize('kind', ['adapter','global'])
def test_source_ages_and_receipt_age_independently_expire_without_new_data(monkeypatch,kind):
    node, clock, ready, send = fixture(monkeypatch,kind)
    send(); clock.wall += .101
    with pytest.raises(ValueError): ready()
    send(); clock.mono += .501  # paused source clock cannot renew a lease
    with pytest.raises(ValueError): ready()


def test_adapter_hard_loss_cancels_once_even_if_recovered_before_timer(monkeypatch):
    node, clock = harness(monkeypatch); commit(node,clock)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map')
    node.on_atomic_state(state(t=clock.wall-10.))
    node.execution_permit=object(); node.stop_report=object()
    clock.wall += .01; clock.mono += .01
    unavailable=state(t=clock.wall-10.,usable=False);unavailable.reason='hard_localization_lost'
    node.on_atomic_state(unavailable)
    assert node.follow['cancel']['reason']=='atomic_navigation_hard_loss'
    assert node.execution_permit is None and node.stop_report is None
    count=len(node.worker_cancel_pub.messages)
    clock.wall += .01; clock.mono += .01
    node.on_atomic_state(state(t=clock.wall-10.))
    assert node.ready_identity()==(1,'seed','map')
    assert node.follow['cancel']['reason']=='atomic_navigation_hard_loss'
    assert len(node.worker_cancel_pub.messages)==count


@pytest.mark.parametrize('unavailable',[False,True])
def test_global_new_epoch_or_explicit_loss_retires_frozen_route(monkeypatch,unavailable):
    node, clock, _, send=fixture(monkeypatch,'global')
    send()
    from test_live_goal_pause import INTENT
    node.pause.install(replace(INTENT,identity=node.confirmed_identity()))
    frozen=deepcopy(node.cached_result['path_message'])
    clock.wall+=.01;clock.mono+=.01
    message=state(t=clock.wall-10.,epoch=1 if unavailable else 2,usable=not unavailable)
    if unavailable:message.reason='hard_localization_lost'
    node.on_atomic_state(message)
    assert node.pause.intent is None and node.cached_result is None
    assert not node.active_reference and not node.visual_path_available
    assert len(frozen.poses)==2  # invalidation, never silent route recomputation


def test_global_bad_packet_does_not_revoke_committed_route(monkeypatch):
    node,clock,_,send=fixture(monkeypatch,'global');send()
    from test_live_goal_pause import INTENT
    node.pause.install(replace(INTENT,identity=node.confirmed_identity()))
    intent=node.pause.intent;cached=node.cached_result;generation=node.generation
    clock.wall+=.01;clock.mono+=.01
    malformed=state(t=clock.wall-10.);malformed.global_odometry.header.frame_id='foreign'
    node.on_atomic_state(malformed)
    assert node.pause.intent is intent and node.cached_result is cached and node.generation==generation


def test_adapter_soft_unavailable_preserves_frozen_follow_without_replay(monkeypatch):
    node,clock=harness(monkeypatch);commit(node,clock)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map')
    node.on_atomic_state(state(t=clock.wall-10.))
    snapshot=node.snapshot;count=len(node.route_reference_pub.messages)
    clock.wall+=.01;clock.mono+=.01
    message=state(t=clock.wall-10.,usable=False);message.reason='imu_temporary_gap'
    node.on_atomic_state(message)
    assert node.body is None and not node.hard_reason(snapshot['context'])
    assert node.snapshot is snapshot and node.follow['cancel'] is None
    assert len(node.route_reference_pub.messages)==count


def test_global_soft_unavailable_preserves_committed_geometry_and_intent(monkeypatch):
    node,clock,_,send=fixture(monkeypatch,'global');send()
    from test_live_goal_pause import INTENT
    node.pause.install(replace(INTENT,identity=node.confirmed_identity()))
    # Exercise production readiness so missing body is a soft pause.
    node.planning_context=MethodType(LiveGlobalPlanner.planning_context,node)
    intent=node.pause.intent;cached=node.cached_result;generation=node.generation
    clock.wall+=.01;clock.mono+=.01
    send(usable=False)
    assert node.body is None and node.pause.paused_at is not None
    assert node.pause.intent is intent and node.cached_result is cached and node.generation==generation


@pytest.mark.parametrize('reason,hard',[('imu_temporary_gap',False),('lio_reset',True)])
def test_unavailable_health_keeps_identity_but_never_ready(monkeypatch,reason,hard):
    node,clock,_,send=fixture(monkeypatch,'adapter');send()
    clock.wall+=.01;clock.mono+=.01
    message=state(t=clock.wall-10.,usable=False);message.reason=reason
    node.on_atomic_state(message);node.tick()
    health=node.health_pub.messages[-1]
    assert (health.map_version_id,health.localization_epoch,health.localization_seed_id)==('map',1,'seed')
    assert not health.ready and health.hard_fault is hard
