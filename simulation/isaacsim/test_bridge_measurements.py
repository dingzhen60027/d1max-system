"""Exercise the actual bridge callbacks without a ROS context or plant."""
from copy import deepcopy
import json
from types import SimpleNamespace

import numpy as np
import pytest

import bridge
from protocol import decode, imu_packet, state_packet, ray_packets, NATIVE_RAY_PHASE


ANCHOR = 1791124691902856036


@pytest.fixture
def measured_bridge(monkeypatch, tmp_path):
    # Import only message/runtime definitions. The replaced Node/socket/tf and
    # run_bridge below never initialise ROS, bind UDP or create an executor.
    import rclpy.node
    import rclpy.clock
    import tf2_ros
    from d1max_pct_scan import isolated_zenoh

    publishers, transforms, sockets = {}, [], []

    class Socket:
        def __init__(self, *args):
            self.sent = []
            sockets.append(self)
        def setsockopt(self, *args): pass
        def bind(self, *args): pass
        def setblocking(self, *args): pass
        def sendto(self, data, target): self.sent.append((decode(data), target))
        def recvfrom(self, *args): raise BlockingIOError()
        def close(self): pass

    class Node:
        def __init__(self, *args, **kwargs): pass
        def create_publisher(self, msg_type, topic, qos):
            result = SimpleNamespace(messages=[], qos=qos)
            result.publish = lambda msg: result.messages.append(deepcopy(msg))
            publishers[topic] = result
            return result
        def create_subscription(self, *args): return None
        def create_timer(self, *args, **kwargs): return None
        def get_logger(self): return SimpleNamespace(error=lambda msg: None, warning=lambda msg: None)
        def destroy_node(self): pass

    class Transform:
        def __init__(self, *args): pass
        def sendTransform(self, value): transforms.append(deepcopy(value))

    monkeypatch.setattr(rclpy.node, 'Node', Node)
    monkeypatch.setattr(rclpy.clock, 'Clock', lambda **kwargs: object())
    monkeypatch.setattr(tf2_ros, 'TransformBroadcaster', Transform)
    monkeypatch.setattr(tf2_ros, 'StaticTransformBroadcaster', Transform)
    monkeypatch.setattr(isolated_zenoh, 'validate_environment', lambda: None)
    monkeypatch.setattr(bridge.socket, 'socket', Socket)
    now = SimpleNamespace(wall=0.)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: now.wall)
    session = dict(id='s', version_id='map', transport_mode='isolated_mock',
        simulation_backend='isaacsim_physx', max_speed_mps=.23, max_yaw_radps=.30,
        navigation_contract=dict(frames=dict(body_frame='d1max_loc_base_link',
            tracking_frame='d1max_loc_base_link', map_frame='d1max_loc_map', odom_frame='d1max_loc_odom')),
        isaac_bridge_contract=dict(clock_anchor_ns=ANCHOR, lidar_origins_body=[[.2,0.,.1],[-.2,0.,.1]],
            mc_measurement_topic=bridge.MC_MEASUREMENT_TOPIC,
            mc_measurement_contract=bridge.MC_MEASUREMENT_CONTRACT),
        static_collision_prior_contract=dict(static_prior_geometry_sha256='a'*64,
            max_body_tilt_rad=.35, expected_body_world_z=.5, max_body_height_error_m=.12,
            body_envelope_attestation_required=True, body_envelope_registry_sha256='b'*64),
        perception_native_phase_contract=dict(schema=1, phase=NATIVE_RAY_PHASE, physics_dt_ns=2_000_000))
    (tmp_path/'session.json').write_text(json.dumps(session))
    monkeypatch.setattr(bridge, 'run_bridge', lambda factory: captured.append(factory(context=None)))
    monkeypatch.setattr('sys.argv', ['bridge.py', '--session', str(tmp_path)])
    captured = []
    bridge.main()
    return SimpleNamespace(node=captured[0], publishers=publishers, transforms=transforms,
        sockets=sockets, now=now, session=session, directory=tmp_path)


def feed(value, seq, ns, *, geometry=True, velocity=(.01,-.02,.03), angular=(.01,.02,.03)):
    value.node.imu(decode(imu_packet('physical-epoch', seq, ns-1_000_000,
        [0,0,0,1], angular, [0,0,9.8067])))
    packet = decode(state_packet('physical-epoch', seq, ns,
        [1,2,.5,0,0,0,1], velocity, angular,
        metadata=dict(static_prior_geometry_valid=geometry,
            static_prior_geometry_sha256='a'*64, static_prior_geometry_checked_sim_time_ns=ns,
            body_envelope_valid=geometry, body_envelope_checked_sim_time_ns=ns,
            body_envelope_registry_sha256='b'*64)))
    value.node.state(packet)
    return packet


def time_ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


def test_geometry_fault_keeps_only_real_clock_and_sdk_mc_stop_measurements(measured_bridge):
    value = measured_bridge
    feed(value, 1, 20_000_000, velocity=(.11,-.12,.13), angular=(.14,.15,.16))
    feed(value, 2, 40_000_000, geometry=False)
    fault = value.node.fault
    assert fault == 'isaac_static_collision_geometry_revoked'
    before = {topic: len(pub.messages) for topic,pub in value.publishers.items()}
    before_tf = len(value.transforms)
    # The world certificate can return true; the terminal navigation fault
    # must not return to usable or resume any perception/TF publication.
    feed(value, 3, 60_000_000, geometry=True, velocity=(.001,.002,-.003), angular=(.004,-.005,.006))
    assert value.node.fault == fault and value.node.geometry_stop_measurements
    clock = value.publishers['/clock']
    assert time_ns(clock.messages[-1].clock) == ANCHOR+60_000_000
    from rclpy.qos import DurabilityPolicy
    assert clock.qos.depth == 1 and clock.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    mc = value.publishers[bridge.MC_MEASUREMENT_TOPIC].messages[-1]
    assert mc.usable and mc.localization_epoch == 1 and mc.localization_seed_id == 'isaac-physical-epoch'
    assert mc.reason == 'isaac_physx_measured_mc_only_after_geometry_revocation'
    assert time_ns(mc.source_stamp) == time_ns(mc.posterior_stamp) == ANCHOR+60_000_000
    assert time_ns(mc.imu_stamp) == ANCHOR+59_000_000
    assert mc.extrapolation_sec == pytest.approx(.001)
    assert [mc.local_odometry.pose.pose.position.x, mc.local_odometry.pose.pose.position.y] == [1.,2.]
    twist = mc.local_odometry.twist.twist
    assert [twist.linear.x,twist.linear.y,twist.linear.z] == [.001,.002,-.003]
    assert [twist.angular.x,twist.angular.y,twist.angular.z] == [.004,-.005,.006]
    assert np.linalg.norm([twist.linear.x,twist.linear.y,twist.linear.z]) > 0.
    for topic in before:
        if topic not in ('/clock', bridge.MC_MEASUREMENT_TOPIC):
            assert len(value.publishers[topic].messages) == before[topic]
    assert len(value.transforms) == before_tf
    for topic in ('/d1max/localization/navigation/state', '/d1max/localization/navigation/local_state'):
        terminal = value.publishers[topic].messages[-1]
        assert terminal.usable is False and terminal.localization_epoch == 2
        assert time_ns(terminal.source_stamp) == ANCHOR+20_000_000
    # SDK applied messages cannot bypass the latched geometry guard. It sends
    # an actual zero wire command regardless of a previously positive demand.
    value.node.applied(SimpleNamespace())
    assert value.sockets[1].sent[-1][0]['vx'] == value.sockets[1].sent[-1][0]['wz'] == 0.
    ray = decode(next(iter(ray_packets('physical-epoch', 1, 60_000_000, 0, [0,0,0], [[1,0,0]],
        phase=NATIVE_RAY_PHASE, native_frame_time_ns=62_000_000, native_frame_time_s=.062))))
    value.node.rays(ray)
    assert len(value.publishers['/d1max/localization/perception/rays_raw'].messages) == before['/d1max/localization/perception/rays_raw']
    assert value.node.state_count == 1 and value.node.mc_after_geometry_fault_count == 2


@pytest.mark.parametrize('bad', ['epoch', 'regression', 'pause'])
def test_post_geometry_measurements_cannot_cross_epoch_rollback_or_pause(measured_bridge, bad):
    value = measured_bridge
    feed(value, 1, 20_000_000)
    packet = feed(value, 2, 40_000_000, geometry=False)
    counts = [len(value.publishers[t].messages) for t in ('/clock', bridge.MC_MEASUREMENT_TOPIC)]
    if bad == 'pause':
        value.node.fail('isaac_simulation_paused')
    else:
        packet.update(sequence=3, sim_time_ns=30_000_000 if bad == 'regression' else 60_000_000)
        if bad == 'epoch': packet['epoch'] = 'foreign-epoch'
        value.node.state(packet)
    feed(value, 4, 80_000_000)
    assert not value.node.geometry_stop_measurements and value.node.measurement_fault
    assert value.node.fault == 'isaac_static_collision_geometry_revoked'
    assert counts == [len(value.publishers[t].messages) for t in ('/clock', bridge.MC_MEASUREMENT_TOPIC)]


def test_bad_duplicate_or_imu_missing_measurements_do_not_invent_mc_freshness(measured_bridge):
    value = measured_bridge
    feed(value, 1, 20_000_000)
    packet = feed(value, 2, 40_000_000, geometry=False)
    counts = [len(value.publishers[t].messages) for t in ('/clock', bridge.MC_MEASUREMENT_TOPIC)]
    value.node.state(packet)
    assert counts == [len(value.publishers[t].messages) for t in ('/clock', bridge.MC_MEASUREMENT_TOPIC)]
    packet.update(sequence=3, sim_time_ns=60_000_000, linear_velocity_world=[float('nan'),0,0])
    with pytest.raises(ValueError, match='nonfinite_vector'): value.node.state(packet)
    assert counts == [len(value.publishers[t].messages) for t in ('/clock', bridge.MC_MEASUREMENT_TOPIC)]
    packet.update(sequence=4, sim_time_ns=200_000_000, linear_velocity_world=[.001,.002,.003])
    value.node.state(packet)
    mc = value.publishers[bridge.MC_MEASUREMENT_TOPIC].messages[-1]
    assert not mc.usable
    assert time_ns(mc.source_stamp) == ANCHOR+200_000_000
    assert time_ns(mc.imu_stamp) == ANCHOR+39_000_000  # Actual last sensor reading, not body time.


@pytest.mark.parametrize('field,value,reason', [
    ('transport_mode','live','isaac_isolated_session_required'),
    ('mc_measurement_topic','/d1max/localization/navigation/local_state','invalid_isaac_mc_measurement_contract'),
    ('mc_measurement_contract','unbound','invalid_isaac_mc_measurement_contract')])
def test_stop_measurement_topic_cannot_be_bound_to_navigation_or_live_mode(measured_bridge, field, value, reason):
    fixture = measured_bridge
    if field == 'transport_mode': fixture.session[field] = value
    else: fixture.session['isaac_bridge_contract'][field] = value
    (fixture.directory/'session.json').write_text(json.dumps(fixture.session))
    with pytest.raises(ValueError, match=reason): bridge.main()
