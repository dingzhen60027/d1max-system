#!/usr/bin/env python3
"""Isaac measurements to the existing navigation contracts, isolated Zenoh only.

This is a simulated plant/ground-truth localization fixture. It never computes
a route, tracking command, safety permission, task success, or stopped evidence.
The existing SDK-free writer consumes measured velocities and owns that evidence.
"""
import argparse
from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
import signal
import socket
import time

import numpy as np

from protocol import (STATE_PORT, COMMAND_PORT, MAX_DATAGRAM, RayAssembler,
                      decode, finite_vector, command_packet)


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def validate_static_prior_state(packet, contract, rotation):
    """A closed-world prior is valid only while the actual world stays certified."""
    if not contract:
        return
    checked = packet.get('static_prior_geometry_checked_sim_time_ns')
    if (packet.get('static_prior_geometry_valid') is not True
            or packet.get('static_prior_geometry_sha256') != contract['static_prior_geometry_sha256']
            or type(checked) is not int or not 0 <= packet['sim_time_ns']-checked <= 100000000):
        raise ValueError('isaac_static_collision_geometry_revoked')
    # Native body envelopes use yaw; certify that omitted roll/pitch remain
    # inside this fixture's explicit bounded tilt contract.
    tilt = math.acos(float(np.clip(rotation[2,2],-1.,1.)))
    if tilt > contract['max_body_tilt_rad']:
        raise ValueError('isaac_static_prior_body_tilt_outside_contract')
    if abs(float(packet['pose'][2])-contract['expected_body_world_z'])>contract['max_body_height_error_m']:
        raise ValueError('isaac_static_prior_body_height_outside_flat_floor_contract')


def run_bridge(node_factory):
    """Finish the current callback before invalidating its ROS context.

    Humble's default SIGTERM handler shuts down the context asynchronously,
    including during a measurement publisher's callback. Own these signals so
    shutdown first stops the single executor, then sends the existing plant
    zero and destroys the node, and only then closes ROS. Real callback errors
    still propagate; none are reclassified as an expected shutdown.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    context = Context()
    executor = node = None
    stopping = False
    previous = {}

    def stop(_signum, _frame):
        nonlocal stopping
        if stopping:
            return
        stopping = True
        if executor is not None:
            executor.wake()

    try:
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        executor = SingleThreadedExecutor(context=context)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, stop)
        node = node_factory(context=context)
        executor.add_node(node)
        while not stopping and context.ok():
            executor.spin_once(timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopping = True
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=.1)
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                try:
                    context.try_shutdown()
                finally:
                    for sig, handler in previous.items():
                        signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--state-port', type=int, default=STATE_PORT)
    parser.add_argument('--command-port', type=int, default=COMMAND_PORT)
    parser.add_argument('--state-timeout', type=float, default=.5)
    args = parser.parse_args()
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from d1max_planning_interfaces.msg import NavigationState, LocalNavigationState, MotionDemand
    from d1max_pct_scan.ray_projection import FIELDS, RAY_DTYPE
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock as ClockMessage
    from sensor_msgs.msg import PointCloud2, PointField, Imu
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    directory = args.session.resolve(strict=True)
    session = json.loads((directory/'session.json').read_text())
    if session.get('transport_mode') != 'isolated_mock' or session.get('simulation_backend') != 'isaacsim_physx':
        raise ValueError('isaac_isolated_session_required')
    contract = session['isaac_bridge_contract']
    anchor_ns = int(contract['clock_anchor_ns'])
    frames = session['navigation_contract']['frames']

    def stamp(message, nanoseconds):
        message.sec, message.nanosec = divmod(int(nanoseconds), 10**9)

    class Bridge(Node):
        def __init__(self, *, context):
            # Publisher of /clock uses a wall timer; every graph consumer uses
            # ROS simulation time. Original samples are never republished fresh.
            super().__init__('d1max_isaac_measurement_bridge', context=context)
            self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
            self.receiver.bind(('127.0.0.1', args.state_port))
            self.receiver.setblocking(False)
            self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.target = ('127.0.0.1', args.command_port)
            self.assembler = RayAssembler()
            self.epoch = None
            self.last_state_sequence = -1
            self.last_state_sim_ns = -1
            self.last_state_wall = None
            self.last_sim_advance_wall = None
            self.command_sequence = 0
            self.last_status = 0.
            self.fault = ''
            self.state_count = 0
            self.ray_count = [0, 0]
            self.imu_count = 0
            self.last_imu_sequence = -1
            self.imu_history = deque(maxlen=64)
            self.last_local_state = None
            self.last_global_state = None
            self.clock_pub = self.create_publisher(ClockMessage, '/clock', 10)
            self.global_pub = self.create_publisher(NavigationState, '/d1max/localization/navigation/state', 10)
            self.local_pub = self.create_publisher(LocalNavigationState, '/d1max/localization/navigation/local_state', 10)
            self.odom_global_pub = self.create_publisher(Odometry, '/d1max/localization/odometry/global', qos_profile_sensor_data)
            self.odom_local_pub = self.create_publisher(Odometry, '/d1max/localization/odometry/local', qos_profile_sensor_data)
            self.ray_pub = self.create_publisher(PointCloud2, '/d1max/localization/perception/rays_raw', qos_profile_sensor_data)
            self.raw_cloud_pub = [self.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
                for topic in ('/front_lidar', '/rear_lidar')]
            imu_qos=QoSProfile(depth=512, reliability=qos_profile_sensor_data.reliability,
                durability=qos_profile_sensor_data.durability)
            self.imu_pub = [self.create_publisher(Imu, topic, imu_qos)
                for topic in ('/d1max/localization/imu', '/front_lidar/imu', '/imu_driver/imu_central',
                              '/d1max/localization/body_imu')]
            self.status_pub = self.create_publisher(String, '/d1max/isaacsim/status', 1)
            self.tf = TransformBroadcaster(self)
            self.static_tf = StaticTransformBroadcaster(self)
            transforms = []
            for parent, child in [(frames['body_frame'], frames['tracking_frame']),
                                  (frames['tracking_frame'], 'd1max_loc_lidar'),
                                  (frames['map_frame'], frames['odom_frame'])]:
                transform = TransformStamped()
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.
                transforms.append(transform)
            for child, origin in zip(('rslidar_head','rslidar_tail'), contract['lidar_origins_body']):
                transform = TransformStamped()
                transform.header.frame_id = frames['body_frame']
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = origin
                transforms.append(transform)
            self.static_tf.sendTransform(transforms)
            self.create_subscription(MotionDemand, '/d1max/live_planning/execution/applied_motion', self.applied, 10)
            self.create_timer(.002, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def send_zero(self):
            if self.epoch is not None:
                self.command_sequence += 1
                self.sender.sendto(command_packet(self.epoch, self.command_sequence,
                    max(0, self.last_state_sim_ns), 0., 0.), self.target)

        def fail(self, reason):
            if self.fault:
                return
            self.fault = reason
            self.send_zero()
            # Revoke the original measured sample without inventing a newer
            # IMU timestamp, pose, posterior, or stopped measurement.
            revoked = deepcopy(self.last_local_state) if self.last_local_state is not None else LocalNavigationState(
                schema_version=1, session_id=session['id'], map_version_id=session['version_id'],
                localization_epoch=1, localization_seed_id='isaac-'+str(self.epoch))
            revoked.usable, revoked.reason = False, reason
            # Ordered inboxes reject same-epoch duplicate source samples. The
            # terminal revocation is a new fixture epoch, with no new measured
            # source/IMU/pose; it never publishes a usable sample in that epoch.
            revoked.localization_epoch += 1
            self.local_pub.publish(revoked)
            if self.last_global_state is not None:
                global_revoked = deepcopy(self.last_global_state)
                global_revoked.usable, global_revoked.reason = False, reason
                global_revoked.localization_epoch = revoked.localization_epoch
                self.global_pub.publish(global_revoked)
            self.get_logger().error(reason+'; create a new session before restarting the simulator')

        def source(self, packet):
            if self.epoch is None:
                self.epoch = packet['epoch']
            if packet['epoch'] != self.epoch:
                self.fail('isaac_clock_epoch_changed')
                return None
            return anchor_ns+packet['sim_time_ns']

        def applied(self, message):
            if self.epoch is None or self.fault:
                self.send_zero()
                return
            now_wall = time.monotonic()
            if self.last_state_wall is None or now_wall-self.last_state_wall > args.state_timeout:
                self.fail('isaac_measured_state_timeout')
                return
            if (message.transport_mode != 'isolated_mock'
                    or message.version.session_id != session['id']
                    or message.version.map_version_id != session['version_id']):
                self.fail('foreign_applied_motion')
                return
            source_ns = message.source_stamp.sec*10**9+message.source_stamp.nanosec
            until_ns = message.valid_until.sec*10**9+message.valid_until.nanosec
            now_ns = anchor_ns+self.last_state_sim_ns
            if not 0 <= now_ns-source_ns <= 250000000 or until_ns < now_ns:
                self.send_zero()
                return
            velocity = message.velocity
            values = finite_vector([velocity.linear.x, velocity.linear.y, velocity.linear.z,
                velocity.angular.x, velocity.angular.y, velocity.angular.z], 6)
            if (any(abs(x) > 1e-6 for x in values[1:5])
                    or abs(values[0]) > session['max_speed_mps']+1e-6
                    or abs(values[5]) > session['max_yaw_radps']+1e-6):
                self.fail('unsupported_wheeled_applied_motion')
                return
            self.command_sequence += 1
            self.sender.sendto(command_packet(self.epoch, self.command_sequence,
                source_ns-anchor_ns, values[0], values[5],
                max(.001, min(.15, (until_ns-source_ns)*1e-9))), self.target)

        def state(self, packet):
            if self.fault:
                return
            source_ns = self.source(packet)
            if source_ns is None:
                return
            sequence = packet['sequence']
            if type(sequence) is not int or sequence < 0:
                raise ValueError('invalid_state_sequence')
            # Reordered UDP packets are ignored; a later sequence with a
            # backwards source time is a reset and irreversibly revokes motion.
            if sequence <= self.last_state_sequence:
                return
            if packet['sim_time_ns'] < self.last_state_sim_ns:
                self.fail('isaac_source_clock_regressed')
                return
            if packet['sim_time_ns'] == self.last_state_sim_ns:
                return
            pose = finite_vector(packet['pose'], 7)
            linear = finite_vector(packet['linear_velocity_world'], 3)
            angular = finite_vector(packet['angular_velocity_world'], 3)
            quaternion = np.asarray(pose[3:])
            if abs(np.linalg.norm(quaternion)-1.) > .01:
                raise ValueError('invalid_physics_quaternion')
            rotation = quaternion_matrix(quaternion/np.linalg.norm(quaternion))
            try:
                validate_static_prior_state(packet,session.get('static_collision_prior_contract'),rotation)
            except ValueError as error:
                self.fail(str(error))
                return
            # nav_msgs twist is body-frame; Isaac articulation velocities are
            # world-frame measurements, never substituted with command values.
            body_linear, body_angular = rotation.T@linear, rotation.T@angular
            self.last_state_sequence = sequence
            self.last_state_sim_ns = packet['sim_time_ns']
            self.last_state_wall = self.last_sim_advance_wall = time.monotonic()
            clock = ClockMessage()
            stamp(clock.clock, source_ns)
            self.clock_pub.publish(clock)
            odometry = Odometry()
            stamp(odometry.header.stamp, source_ns)
            odometry.header.frame_id = frames['odom_frame']
            odometry.child_frame_id = frames['body_frame']
            p, q = odometry.pose.pose.position, odometry.pose.pose.orientation
            p.x, p.y, p.z = pose[:3]
            q.x, q.y, q.z, q.w = pose[3:]
            v, w = odometry.twist.twist.linear, odometry.twist.twist.angular
            v.x, v.y, v.z = body_linear.tolist()
            w.x, w.y, w.z = body_angular.tolist()
            global_odometry = deepcopy(odometry)
            global_odometry.header.frame_id = frames['map_frame']
            seed = 'isaac-'+self.epoch
            actual_imu = next((measurement for measurement in reversed(self.imu_history)
                if measurement <= packet['sim_time_ns']), None)
            imu_usable = actual_imu is not None and 0 <= packet['sim_time_ns']-actual_imu <= 100000000
            imu_ns = anchor_ns+actual_imu if actual_imu is not None else 0
            local = LocalNavigationState(schema_version=1, session_id=session['id'],
                map_version_id=session['version_id'], localization_epoch=1,
                localization_seed_id=seed, local_odometry=odometry,
                usable=imu_usable, reason='isaac_physx_measured_groundtruth_fixture_not_estimator'
                    if imu_usable else 'isaac_actual_imu_missing_or_stale')
            global_state = NavigationState(schema_version=2, session_id=session['id'],
                map_version_id=session['version_id'], localization_epoch=1,
                localization_seed_id=seed, local_odometry=odometry,
                global_odometry=global_odometry, usable=imu_usable, reason=local.reason)
            for value in (local, global_state):
                for field in (value.source_stamp, value.posterior_stamp):
                    stamp(field, source_ns)
                stamp(value.imu_stamp, imu_ns)
                value.extrapolation_sec = (source_ns-imu_ns)*1e-9 if imu_usable else 0.
            self.local_pub.publish(local)
            self.last_local_state = deepcopy(local)
            self.global_pub.publish(global_state)
            self.last_global_state = deepcopy(global_state)
            self.odom_local_pub.publish(odometry)
            self.odom_global_pub.publish(global_odometry)
            transform = TransformStamped()
            transform.header = odometry.header
            transform.child_frame_id = frames['body_frame']
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = pose[:3]
            transform.transform.rotation = odometry.pose.pose.orientation
            self.tf.sendTransform(transform)
            self.state_count += 1

        def imu(self, packet):
            if self.fault or self.source(packet) is None:
                return
            sequence = packet['sequence']
            if type(sequence) is not int or sequence <= self.last_imu_sequence:
                return
            if packet.get('sensor_frame') != 'body':
                raise ValueError('unsupported_imu_axis_frame')
            source = packet['sim_time_ns']
            if self.imu_history and source <= self.imu_history[-1]:
                self.fail('isaac_imu_source_clock_regressed')
                return
            orientation = finite_vector(packet['orientation'], 4)
            angular = finite_vector(packet['angular_velocity'], 3)
            acceleration = finite_vector(packet['linear_acceleration'], 3)
            if abs(np.linalg.norm(orientation)-1.) > .01:
                raise ValueError('invalid_native_imu_orientation')
            message = Imu()
            message.header.frame_id = frames['body_frame']
            stamp(message.header.stamp, anchor_ns+source)
            message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = orientation
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = angular
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = acceleration
            # Zero covariance means unknown covariance in sensor_msgs/Imu;
            # measured deterministic values are not claimed as calibrated noise.
            for publisher in self.imu_pub:
                publisher.publish(message)
            self.imu_history.append(source)
            self.last_imu_sequence = sequence
            self.imu_count += 1

        def rays(self, packet):
            if self.fault or self.source(packet) is None:
                return
            scan = self.assembler.add(packet)
            if scan is None:
                return
            source_ns = anchor_ns+scan['sim_time_ns']
            # Original scan stamps may be slightly older than the latest body
            # sample. The existing projector waits for matching body history.
            if self.last_state_sim_ns-scan['sim_time_ns'] > 500000000:
                return
            xyz = np.asarray(scan['xyz'], dtype=np.float32)
            points = np.zeros(len(xyz), dtype=RAY_DTYPE)
            for name, values in zip(('x', 'y', 'z'), xyz.T):
                points[name] = values
            for name, value in zip(('origin_x', 'origin_y', 'origin_z'), scan['origin']):
                points[name] = value
            points['intensity'] = 1.
            points['sensor_id'] = scan['sensor_id']
            points['source_index'] = np.arange(len(xyz))
            if scan['rings'] is not None:
                points['ring'] = np.asarray(scan['rings'],dtype=np.uint16)
            # Snapshot rays are simultaneous; they do not invent per-point
            # acquisition offsets or Airy96 rings that Isaac did not measure.
            points['timestamp'] = points['source_timestamp'] = source_ns*1e-9
            points['raw_timestamp'] = scan['sim_time_ns']*1e-9
            cloud = PointCloud2(height=1, width=len(xyz),
                fields=[PointField(name=n, offset=o, datatype=d, count=c) for n,o,d,c in FIELDS],
                is_bigendian=False, point_step=64, row_step=64*len(xyz),
                data=points.tobytes(), is_dense=True)
            stamp(cloud.header.stamp, source_ns)
            cloud.header.frame_id = 'd1max_loc_lidar'
            self.ray_pub.publish(cloud)
            # Original measured XYZ in each LiDAR's actual sensor frame, plus
            # native acquisition time. No synthetic Livox CustomMsg is emitted.
            raw_dtype = np.dtype({'names':['x','y','z','ring','timestamp'],
                'formats':['<f4','<f4','<f4','<u2','<f8'], 'offsets':[0,4,8,12,16], 'itemsize':24})
            raw = np.zeros(len(xyz), dtype=raw_dtype)
            for name, values in zip(('x','y','z'), (xyz-np.asarray(scan['origin'])).T):
                raw[name] = values
            raw['timestamp'] = source_ns*1e-9
            if scan['rings'] is not None:
                raw['ring'] = np.asarray(scan['rings'],dtype=np.uint16)
            raw_fields=[('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),('z',8,PointField.FLOAT32),
                        ('timestamp',16,PointField.FLOAT64)]
            if scan['rings'] is not None:
                raw_fields.insert(3,('ring',12,PointField.UINT16))
            original = PointCloud2(height=1, width=len(raw), is_bigendian=False,
                point_step=24, row_step=24*len(raw), data=raw.tobytes(), is_dense=True,
                fields=[PointField(name=n, offset=o, datatype=d, count=1) for n,o,d in raw_fields])
            original.header.stamp = cloud.header.stamp
            original.header.frame_id = ('rslidar_head','rslidar_tail')[scan['sensor_id']]
            self.raw_cloud_pub[scan['sensor_id']].publish(original)
            self.ray_count[scan['sensor_id']] += 1

        def poll(self):
            for _ in range(512):
                try:
                    data, address = self.receiver.recvfrom(MAX_DATAGRAM+1)
                except BlockingIOError:
                    break
                if address[0] != '127.0.0.1':
                    continue
                try:
                    packet = decode(data)
                    if packet['type'] == 'state':
                        self.state(packet)
                    elif packet['type'] == 'imu':
                        self.imu(packet)
                    elif packet['type'] == 'rays':
                        self.rays(packet)
                    elif packet['type'] == 'status' and packet.get('playing') is False:
                        self.fail('isaac_simulation_paused')
                except (ValueError, TypeError, KeyError) as error:
                    self.get_logger().warning('Dropped invalid Isaac packet: '+str(error))
            now = time.monotonic()
            if self.last_state_wall is not None and now-self.last_state_wall > args.state_timeout:
                self.fail('isaac_measured_state_timeout_or_pause')
            if now-self.last_status > .5:
                self.last_status = now
                status = dict(schema=1, simulation=True, physical_acceptance=False,
                    session_id=session['id'], epoch=self.epoch,
                    last_sim_time_ns=self.last_state_sim_ns, measured_state_samples=self.state_count,
                    ray_scans=self.ray_count, incomplete_scans_dropped=self.assembler.dropped,
                    native_imu_samples=self.imu_count,
                    latest_native_imu_sim_time_ns=self.imu_history[-1] if self.imu_history else None,
                    fault=self.fault, localization='groundtruth_fixture', transport='isolated_mock',
                    clock='simulation_time_with_fixed_session_anchor')
                message = String(data=json.dumps(status))
                self.status_pub.publish(message)
                (directory/'isaac_bridge_status.json').write_text(json.dumps(status, indent=2)+'\n')

        def destroy_node(self):
            self.send_zero()
            self.receiver.close()
            self.sender.close()
            return super().destroy_node()

    run_bridge(Bridge)


if __name__ == '__main__':
    main()
