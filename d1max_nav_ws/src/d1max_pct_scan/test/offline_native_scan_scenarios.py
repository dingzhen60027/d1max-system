#!/usr/bin/env python3
"""Native SCAN only, analytic first-return synthetic lidar, NO HARDWARE/MOTION.

Private loopback Zenoh 17468 / domain 225. Production scan_parameters and robot
envelope are unchanged. Starts only a router, native SCAN and this test probe;
no PCT worker, SDK, localization, tracker, RViz or velocity publisher. The test
independently samples each accepted native spline against its known geometry.
The multi-view fixture moves only its analytic ray origin while its synthetic
body remains stationary: map acquisition, NOT a robot motion/dynamics loop.
It is not a physical robot, perception, terrain-support or dynamics validation.
"""
from array import array
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

import numpy as np
from scipy.interpolate import BSpline
import yaml


WS = Path(__file__).resolve().parents[3]
PORT, DOMAIN = 17468, '225'
FRAME = 'd1max_loc_map'
PREFIX = '/TEST_ONLY_native_scan/'
ROOM = np.array([[-2., -2., 0.], [6., 2., 2.2]])
CASES = {
    'straight_corridor': None,
    'box_occluded': np.array([[1.5, -.4, 0.], [2.2, .4, 2.2]]),
    'box_detour_multiview': np.array([[1.5, -.4, 0.], [2.2, .4, 2.2]]),
    'low_box_multiview': np.array([[1.5, -.4, 0.], [2.2, .4, .20]]),
    'closed_corridor': np.array([[1.5, -2., 0.], [1.7, 2., 2.2]]),
}


def source_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def first_return_cloud(origin, obstacle):
    """Visible first intersections with room surfaces and one solid AABB.

    Dense ideal rays are deliberate: feeding all room points would send rays
    through the test box and manufacture free-space evidence behind it.
    """
    azimuth, elevation = np.meshgrid(np.linspace(-math.pi, math.pi, 720, endpoint=False),
                                   np.deg2rad(np.linspace(-88., 88., 201)))
    directions = np.c_[np.cos(elevation).ravel()*np.cos(azimuth).ravel(),
                       np.cos(elevation).ravel()*np.sin(azimuth).ravel(),
                       np.sin(elevation).ravel()]
    safe = np.where(np.abs(directions) < 1e-12, 1e-12, directions)
    t1, t2 = (ROOM[0]-origin)/safe, (ROOM[1]-origin)/safe
    distance = np.min(np.maximum(t1, t2), axis=1)  # origin is inside room
    if obstacle is not None:
        t1, t2 = (obstacle[0]-origin)/safe, (obstacle[1]-origin)/safe
        near, far = np.max(np.minimum(t1, t2), axis=1), np.min(np.maximum(t1, t2), axis=1)
        hit = (near > 0) & (far >= near)
        distance[hit] = np.minimum(distance[hit], near[hit])
    cloud = origin + directions*distance[:, None]
    cloud = cloud[(distance > .05) & (distance <= 7.99)]
    return np.asarray(cloud, dtype='<f4')


def check_native_curve(message, debug, params, obstacle):
    """Independent SciPy evaluation over the entire native cubic domain.

    Geometry checks use the production two-circle proxy plus vertical extent,
    not a point robot. Dense sampling is still a discrete regression check,
    not a continuous-time collision proof or a validated D1 Max body envelope.
    """
    points = np.array([[p.x, p.y, p.z] for p in message.trajectory.pos_pts])
    knots = np.array(message.trajectory.knots)
    order = int(message.trajectory.order)
    assert order == 3 and len(knots) == len(points)+order+1
    assert np.isfinite(points).all() and np.isfinite(knots).all()
    assert np.all(np.diff(knots) > 0)
    curve = BSpline(knots, points, order, extrapolate=False)
    begin, end = knots[order], knots[len(points)]
    assert 0 < end-begin <= 120., 'invalid native duration'
    speed_bound = np.linalg.norm(curve.derivative().c, axis=1).max()
    dt = min(.01, .005/max(speed_bound, 1e-9))
    count = math.ceil((end-begin)/dt)+1
    assert count <= 50000, 'bounded checker sample budget exceeded'
    sample_time = np.linspace(begin, end, count)
    xyz = curve(sample_time)
    velocity = curve.derivative()(sample_time)
    acceleration = curve.derivative(2)(sample_time)
    assert np.isfinite(xyz).all()
    max_v, max_a = np.linalg.norm(velocity, axis=1).max(), np.linalg.norm(acceleration, axis=1).max()
    assert max_v <= params['manager.max_vel']+1e-6, f'speed violation {max_v}'
    assert max_a <= params['manager.max_acc']+1e-6, f'acceleration violation {max_a}'
    yaw = np.unwrap(np.arctan2(velocity[:, 1], velocity[:, 0]))
    moving = np.hypot(velocity[:, 0], velocity[:, 1]) > 1e-5
    if moving.any():
        # A zero-speed endpoint retains the nearest actual tangent, not atan2(0,0).
        yaw = np.interp(sample_time, sample_time[moving], yaw[moving])
    else:
        raise AssertionError('accepted native curve is stationary hover')
    radius = params['grid_map.double_cylinder_radius']
    offset = params['grid_map.double_cylinder_offset']
    up = params['grid_map.obstacles_inflation_z_up']
    down = params['grid_map.obstacles_inflation_z_down']
    inside = (xyz[:, 2]-up >= ROOM[0, 2]) & (xyz[:, 2]+down <= ROOM[1, 2])
    clearance = []
    for sign in (-1., 1.):
        centers = xyz[:, :2] + sign*offset*np.c_[np.cos(yaw), np.sin(yaw)]
        wall_clearance = np.minimum(centers-ROOM[0, :2], ROOM[1, :2]-centers).min(axis=1)-radius
        inside &= wall_clearance >= -1e-6
        clearance.extend(wall_clearance.tolist())
        if obstacle is not None:
            delta = np.maximum(np.maximum(obstacle[0, :2]-centers, centers-obstacle[1, :2]), 0.)
            xy_clearance = np.linalg.norm(delta, axis=1)-radius
            overlap_z = (xyz[:, 2]+down >= obstacle[0, 2]) & (xyz[:, 2]-up <= obstacle[1, 2])
            if np.any((xy_clearance <= 0.) & overlap_z):
                index = np.flatnonzero(overlap_z)[np.argmin(xy_clearance[overlap_z])]
                raise AssertionError('accepted curve collides with analytic box: '+json.dumps(dict(
                    minimum_clearance_m=float(xy_clearance[index]), sample_index=int(index),
                    time_sec=float(sample_time[index]), body=xyz[index].tolist(),
                    circle_center_xy=centers[index].tolist(), circle_sign=sign,
                    yaw_rad=float(yaw[index]), samples=count)))
            clearance.extend(xy_clearance[overlap_z].tolist())
    assert inside.all(), 'accepted curve exits corridor through wall/floor/ceiling'
    target = np.array([debug.local_target.x, debug.local_target.y, debug.local_target.z])
    assert np.linalg.norm(xyz[-1]-target) < .05, 'native curve does not reach its selected target'
    required_advance = (float(obstacle[1, 0])+radius+offset if obstacle is not None
                        else min(1., params['fsm.planning_horizon']*.5))
    assert xyz[-1, 0] > required_advance, 'accepted curve is only a short pre-obstacle stop'
    return dict(plan_id=int(debug.plan_id), generation=int(debug.generation),
                sample_count=count, max_sample_spacing_m=float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).max()),
                duration_sec=float(end-begin), max_speed_mps=float(max_v), max_acceleration_mps2=float(max_a),
                minimum_analytic_clearance_m=float(min(clearance)),
                start=xyz[0].tolist(), end=xyz[-1].tolist(), max_abs_y_m=float(np.abs(xyz[:, 1]).max()),
                z_range_m=[float(xyz[:, 2].min()), float(xyz[:, 2].max())])


def process_record(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
        fields = stat[stat.rfind(')')+2:].split()
        return dict(pid=pid, start_ticks=int(fields[19]), state=fields[0], pgid=int(fields[2]))
    except (OSError, ValueError, IndexError):
        return None


def prepare(directory, selected=None):
    sys.path.insert(0, str(WS/'src/d1max_pct_scan'))
    from d1max_pct_scan.live_session import DEFAULT_CONFIG, scan_parameters
    cfg = yaml.safe_load(DEFAULT_CONFIG.read_text())
    # This synthetic harness supplies a single calibrated sensor, not the
    # physical dual-source ray interface (covered by offline_projected_rays_bag).
    cfg['perception_backend'] = 'deskewed_cloud'
    fixture = dict(kind='SYNTHETIC_NATIVE_SCAN_NO_HARDWARE', frame_id=FRAME,
                   perception_backend=cfg['perception_backend'],
                   room=ROOM.tolist(), production_config=str(DEFAULT_CONFIG),
                   source_hashes={str(p): source_digest(p) for p in
                                  (DEFAULT_CONFIG, Path(cfg['scan_config']), Path(cfg['robot_profile']))},
                   motion_control_enabled=False, cases={})
    for name, obstacle in CASES.items():
        if selected is not None and name != selected:
            continue
        sid = 'TEST_ONLY_'+name+'_'+uuid.uuid4().hex[:8]
        params = scan_parameters(cfg, sid)
        assert params['grid_map.double_cylinder_radius'] >= .29
        assert params['grid_map.double_cylinder_offset'] >= .20
        assert params.get('grid_map.collision_check_enabled', True) is not False
        assert params['fsm.strict_input_frames'] and params['grid_map.strict_input_frames']
        assert params['fsm.require_tagged_reference']
        params_path = directory/(name+'.yaml')
        params_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        # A remote blockage can legitimately produce a short safe approach
        # segment. To test refusal to proceed, begin near the obstacle where
        # less than reference_target_min_advance remains for the full body.
        body_x = .85 if name in ('box_occluded', 'closed_corridor') else 0.
        origin = np.array([body_x, 0., cfg['body_height']])
        origins = [origin]
        if name.endswith('_multiview'):
            origins += [np.array([x, y, cfg['body_height']]) for x, y in
                        ((1., 1.2), (2., 1.2), (3.2, 1.2), (3.2, 0.))]
        views = []
        for index, ray_origin in enumerate(origins):
            cloud = first_return_cloud(ray_origin, obstacle)
            assert 1000 < len(cloud) <= 250000
            filename = name+('_scan_'+str(index) if index else '')+'.npy'
            np.save(directory/filename, cloud)
            views.append(dict(origin=ray_origin.tolist(), point_count=len(cloud), file=filename))
        fixture['cases'][name] = dict(session_id=sid, origin=origin.tolist(),
            point_count=views[0]['point_count'], views=views,
            fixture_semantics='stationary synthetic body; independently positioned analytic lidar views',
            obstacle=None if obstacle is None else obstacle.tolist(),
            expected_accepted=name in ('straight_corridor', 'box_detour_multiview', 'low_box_multiview'),
            params_file=str(params_path))
    scouting = {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}
    common = dict(scouting=scouting, timestamping={'enabled': True, 'drop_future_timestamp': False})
    (directory/'router.json5').write_text(json.dumps(dict(common, mode='router',
        listen={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True}, connect={'endpoints': []})))
    (directory/'client.json5').write_text(json.dumps(dict(common, mode='client',
        connect={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True})))
    (directory/'fixture.json').write_text(json.dumps(fixture, indent=2))
    return fixture


def run(directory, fixture):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', PORT))  # never reuse or stop someone else's router
    env = dict(os.environ)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG', 'ROS_LOCALHOST_ONLY'):
        env.pop(key, None)
        os.environ.pop(key, None)
    env.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID=DOMAIN,
        ZENOH_SESSION_CONFIG_URI=str(directory/'client.json5'),
        ZENOH_ROUTER_CONFIG_URI=str(directory/'router.json5'),
        ROS_LOG_DIR=str(directory/'ros_logs'), OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    os.environ.update(env)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Bool
    from d1max_planning_interfaces.msg import ReferencePath, TaggedBspline, LocalPlanDebug

    report = dict(kind=fixture['kind'], passed=False, isolation={'domain': DOMAIN, 'port': PORT,
        'router_listen': '127.0.0.1', 'upstream_endpoints': [], 'discovery': False},
        no_robot_or_motion=True, scenarios={}, owned_processes=[], cleanup={})
    processes, streams, node = [], [], None
    began = time.monotonic()
    deadline = began+18.*len(fixture['cases'])+14.

    def spawn(name, command):
        stream = (directory/(name+'.log')).open('w'); streams.append(stream)
        child = subprocess.Popen(command, env=env, cwd=WS, stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        record = process_record(child.pid)
        processes.append((name, child, record))
        report['owned_processes'].append(dict(name=name, command=command, **(record or {'pid': child.pid})))
        return child

    def stop(child):
        if child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=1.)

    class Probe(Node):
        def __init__(self, name):
            super().__init__('TEST_ONLY_'+name, enable_rosout=False, start_parameter_services=False)
            self.name, self.item = name, fixture['cases'][name]
            self.params = yaml.safe_load(Path(self.item['params_file']).read_text())['/**']['ros__parameters']
            self.prefix = PREFIX+name+'/'
            self.cloud_bytes = [array('B', np.load(directory/view['file']).tobytes())
                                for view in self.item['views']]
            self.body_pub = self.create_publisher(Odometry, self.prefix+'body', qos_profile_sensor_data)
            self.sensor_pub = self.create_publisher(Odometry, self.prefix+'sensor', qos_profile_sensor_data)
            self.cloud_pub = self.create_publisher(PointCloud2, self.prefix+'cloud', qos_profile_sensor_data)
            self.freeze_pub = self.create_publisher(Bool, self.prefix+'frozen', 5)
            self.ref_pub = self.create_publisher(ReferencePath, self.prefix+'reference', 1)
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, DurabilityPolicy
            context_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.context_record = dict(schema=1, session_id=self.item['session_id'], epoch=1,
                seed_id='TEST_ONLY_seed', sequence=1, barrier_ns=self.get_clock().now().nanoseconds)
            self.context_ack = False
            self.context_pub = self.create_publisher(String, self.prefix+'map_context', context_qos)
            self.create_subscription(String, self.prefix+'map_context_ack', self.on_context_ack, context_qos)
            self.tags, self.accepted, self.phases, self.errors, self.checked = {}, {}, Counter(), [], {}
            self.occupancy_count, self.occupancy_received = 0, 0
            self.ticks, self.published_clouds, self.issued_ns = 0, 0, 0
            self.create_subscription(TaggedBspline, self.prefix+'tagged', self.tagged, 20)
            self.create_subscription(LocalPlanDebug, self.prefix+'debug', self.debug, 20)
            self.create_subscription(PointCloud2, self.prefix+'scan/grid_map/occupancy', self.occupancy,
                                     qos_profile_sensor_data)
            self.create_timer(.02, self.publish_inputs)

        def on_context_ack(self, message):
            self.context_ack = json.loads(message.data) == self.context_record

        def occupancy(self, msg):
            self.occupancy_count = max(self.occupancy_count, msg.width*msg.height)
            self.occupancy_received += 1
            if self.occupancy_received % 5 == 0:
                from d1max_pct_scan.live_scan_contract import decode_xyz
                occupied = decode_xyz(data=msg.data,
                    fields=[(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                    point_step=msg.point_step, row_step=msg.row_step,
                    width=msg.width, height=msg.height, bigendian=msg.is_bigendian,
                    max_input_points=3000000, max_output_points=3000000)
                np.save(directory/(self.name+'_native_occupancy.npy'), occupied)

        def publish_inputs(self):
            if not self.context_ack:
                from std_msgs.msg import String
                self.context_pub.publish(String(data=json.dumps(self.context_record)))
                return
            stamp = self.get_clock().now().to_msg()
            pose = Odometry(); pose.header.frame_id = FRAME; pose.header.stamp = stamp
            pose.child_frame_id = 'TEST_ONLY_body'
            pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = self.item['origin']
            pose.pose.pose.orientation.w = 1.
            self.body_pub.publish(pose)
            self.freeze_pub.publish(Bool(data=True))
            if self.ticks % 5 == 0:
                view_index = self.published_clouds % len(self.item['views'])
                view = self.item['views'][view_index]
                sensor_pose = Odometry(); sensor_pose.header = pose.header
                sensor_pose.child_frame_id = 'TEST_ONLY_analytic_sensor'
                sensor_pose.pose.pose.position.x, sensor_pose.pose.pose.position.y, sensor_pose.pose.pose.position.z = view['origin']
                sensor_pose.pose.pose.orientation.w = 1.
                cloud = PointCloud2(); cloud.header.frame_id = FRAME; cloud.header.stamp = stamp
                cloud.height, cloud.width = 1, view['point_count']
                cloud.point_step, cloud.row_step = 12, cloud.width*12
                cloud.is_dense = True; cloud.data = self.cloud_bytes[view_index]
                cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate('xyz')]
                # Alternate topic delivery order. Original stamps remain identical;
                # a latest-pose implementation must not be relied on by this test.
                publishers = ((self.sensor_pub, sensor_pose), (self.cloud_pub, cloud))
                for publisher, message in publishers[::1 if self.published_clouds % 2 else -1]:
                    publisher.publish(message)
                self.published_clouds += 1
            self.ticks += 1

        def send_reference(self, cancel=False):
            msg = ReferencePath(); msg.session_id = self.item['session_id']
            msg.generation = 2 if cancel else 1
            msg.path.header.frame_id = FRAME; msg.path.header.stamp = self.get_clock().now().to_msg()
            if not cancel:
                self.issued_ns = msg.path.header.stamp.sec*10**9+msg.path.header.stamp.nanosec
                for x in np.linspace(self.item['origin'][0], 4.5, 91):
                    pose = PoseStamped(); pose.header = msg.path.header
                    pose.pose.position.x = float(x); pose.pose.orientation.w = 1.
                    msg.path.poses.append(pose)
            self.ref_pub.publish(msg)

        def tagged(self, msg):
            if msg.session_id == self.item['session_id'] and msg.generation == 1:
                self.tags[int(msg.trajectory.traj_id)] = msg
                self.check_pairs()

        def debug(self, msg):
            if msg.session_id != self.item['session_id'] or msg.generation != 1:
                return
            self.phases[msg.phase] += 1
            if msg.valid:
                if msg.phase != 'accepted':
                    self.errors.append('valid debug was not accepted')
                else:
                    self.accepted[int(msg.plan_id)] = msg
                self.check_pairs()

        def check_pairs(self):
            for plan_id in self.tags.keys() & self.accepted.keys() - self.checked.keys():
                tag, debug = self.tags[plan_id], self.accepted[plan_id]
                try:
                    assert tag.frame_id == debug.header.frame_id == FRAME
                    start_ns = tag.trajectory.start_time.sec*10**9+tag.trajectory.start_time.nanosec
                    assert start_ns >= self.issued_ns > 0
                    assert debug.selected_reference.poses, 'accepted diagnostic has no actual reference'
                    if not self.item['expected_accepted']:
                        raise AssertionError('blocked or unobserved corridor produced an accepted trajectory')
                    self.checked[plan_id] = check_native_curve(tag, debug, self.params, CASES[self.name])
                except (AssertionError, ValueError) as error:
                    self.checked[plan_id] = {'error': str(error)}
                    self.errors.append(str(error))

        def ready(self):
            return self.context_ack and all(pub.get_subscription_count() > 0 for pub in
                       (self.body_pub, self.sensor_pub, self.cloud_pub, self.freeze_pub, self.ref_pub))

    try:
        router = spawn('router', [str(Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd')])
        ready_by = time.monotonic()+3.
        while True:
            try:
                with socket.create_connection(('127.0.0.1', PORT), timeout=.1):
                    break
            except OSError:
                if time.monotonic() >= ready_by:
                    raise TimeoutError('isolated router startup timed out')
                time.sleep(.05)
        rclpy.init(args=[])
        executable = Path(get_package_prefix('scan_planner'))/'lib/scan_planner/scan_planner_node'
        report['native_executable'] = str(executable)
        report['native_executable_sha256'] = source_digest(executable)
        for name, item in fixture['cases'].items():
            node = Probe(name)
            command = [str(executable), '--ros-args', '-r', '__ns:='+node.prefix+'scan',
                       '-r', '__node:=scan_planner_node', '--params-file', item['params_file']]
            for source, target in {'body_pose': 'body', 'sensor_pose': 'sensor', 'cloud': 'cloud',
                'typed_initial_path': 'reference', 'planning/go2_execution_frozen': 'frozen',
                'grid_map/localization_context': 'map_context',
                'grid_map/localization_context_ack': 'map_context_ack',
                'planning/tagged_bspline': 'tagged', 'planning/local_plan_debug': 'debug'}.items():
                command += ['-r', source+':='+node.prefix+target]
            child = spawn(name, command)
            started = time.monotonic()
            issued = None
            while time.monotonic()-started < 18.:
                if time.monotonic() > deadline:
                    raise TimeoutError('total native smoke budget exceeded')
                if child.poll() is not None or router.poll() is not None:
                    raise RuntimeError('owned native process exited early')
                rclpy.spin_once(node, timeout_sec=.005)
                elapsed = time.monotonic()-started
                if issued is None and node.ready() and elapsed >= 2. and node.occupancy_count > 100:
                    node.send_reference(); issued = time.monotonic()
                if issued is not None and time.monotonic()-issued >= 5.:
                    if node.checked or (not item['expected_accepted'] and node.phases):
                        break
            assert issued is not None, f'{name}: input subscriptions/occupancy did not become ready'
            result = dict(phases=dict(node.phases), accepted_count=len(node.accepted),
                tagged_count=len(node.tags), checked=list(node.checked.values()), errors=node.errors,
                published_clouds=node.published_clouds, occupancy_points=node.occupancy_count,
                elapsed_sec=time.monotonic()-started, reference_observed_sec=time.monotonic()-issued)
            if item['expected_accepted']:
                result['passed'] = bool(node.checked) and not node.errors
            else:
                result['passed'] = not node.accepted and any(k.startswith('failed_') or k == 'waiting_observed_space'
                                                           for k in node.phases)
            forbidden = ('/cmd_vel', '/d1max/navigation/cmd_vel', '/d1max/pct_scan/cmd_vel_safe')
            result['no_motion_publishers'] = not any(node.get_publishers_info_by_topic(t) for t in forbidden)
            result['passed'] &= result['no_motion_publishers']
            report['scenarios'][name] = result
            native_outputs = []
            for plan_id in node.tags.keys() & node.accepted.keys():
                tag, debug = node.tags[plan_id], node.accepted[plan_id]
                native_outputs.append(dict(plan_id=plan_id, order=int(tag.trajectory.order),
                    knots=list(tag.trajectory.knots),
                    control_points=[[p.x, p.y, p.z] for p in tag.trajectory.pos_pts],
                    selected_reference=[[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                        for p in debug.selected_reference.poses],
                    local_target=[debug.local_target.x, debug.local_target.y, debug.local_target.z]))
            (directory/(name+'_native_outputs.json')).write_text(json.dumps(native_outputs, indent=2))
            print(name+': '+json.dumps(result, separators=(',', ':')), flush=True)
            node.send_reference(cancel=True)
            rclpy.spin_once(node, timeout_sec=.05)
            stop(child)
            node.destroy_node(); node = None
        report['passed'] = all(v['passed'] for v in report['scenarios'].values())
    except BaseException as error:
        report['error'] = type(error).__name__+': '+str(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.try_shutdown()
            except rclpy._rclpy_pybind11.RCLError:
                if rclpy.ok():
                    raise
        for _, child, _ in reversed(processes):
            stop(child)
        for stream in streams:
            stream.close()
        survivors = []
        for name, child, previous in processes:
            current = process_record(child.pid)
            if current and previous and current['start_ticks'] == previous['start_ticks'] and current['state'] != 'Z':
                survivors.append({'name': name, **current})
        report['cleanup'] = dict(survivors=survivors, child_exit_codes={n: c.returncode for n, c, _ in processes})
        report['passed'] &= not survivors
        report['elapsed_sec'] = time.monotonic()-began
        (directory/'report.json').write_text(json.dumps(report, indent=2))
    return report


def render_report(directory):
    """Render stored, checked native output; never launch or synthesize a route."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon
    report = json.loads((directory/'report.json').read_text())
    fixture = json.loads((directory/'fixture.json').read_text())
    overview_cases = {'straight_corridor', 'box_occluded', 'box_detour_multiview', 'closed_corridor'}
    if report.get('passed') is not True or not overview_cases.issubset(report['scenarios']):
        raise ValueError('Four passing native scenarios are required for the comparison image')
    titles = {'straight_corridor': 'Straight corridor', 'box_occluded': 'Occluded box: wait',
              'box_detour_multiview': 'Observed box: detour', 'closed_corridor': 'Closed corridor: reject'}
    order = ['straight_corridor', 'box_detour_multiview', 'box_occluded', 'closed_corridor']
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True, sharey=True)
    for ax, name in zip(axes.ravel(), order):
        item, result = fixture['cases'][name], report['scenarios'][name]
        obstacle = item['obstacle']
        ax.add_patch(Rectangle(ROOM[0, :2], *(ROOM[1, :2]-ROOM[0, :2]), fill=False,
                               edgecolor='#334155', linewidth=1.4))
        if obstacle is not None:
            lower, upper = np.asarray(obstacle)
            ax.add_patch(Rectangle(lower[:2], *(upper[:2]-lower[:2]),
                                   facecolor='#475569', edgecolor='#1e293b', linewidth=.8))
        if name == 'box_occluded':
            ax.add_patch(Polygon([[1.5, -.4], [6., -1.6], [6., 1.6], [1.5, .4]],
                                  facecolor='#cbd5e1', alpha=.5, hatch='///', edgecolor='#94a3b8', zorder=0))
        ax.plot([0., 4.5], [0., 0.], '--', color='#2563eb', linewidth=1., label='Requested reference')
        outputs = json.loads((directory/(name+'_native_outputs.json')).read_text())
        if result['checked']:
            checked = next(row for row in result['checked'] if 'error' not in row)
            raw = next(row for row in outputs if row['plan_id'] == checked['plan_id'])
            curve = BSpline(raw['knots'], raw['control_points'], raw['order'], extrapolate=False)
            points = curve(np.linspace(raw['knots'][3], raw['knots'][len(raw['control_points'])], 2000))
            ax.plot(points[:, 0], points[:, 1], color='#16a34a', linewidth=2.2, label='Checked native B-spline')
            note = f"Native plan {raw['plan_id']} · min clearance {checked['minimum_analytic_clearance_m']:.2f} m"
        else:
            assert not outputs and not result['accepted_count'], 'Rejected scene unexpectedly contains an accepted curve'
            note = '0 accepted trajectories · '+next(k for k in result['phases'] if k != 'reference_replaced')
        ax.scatter([0.], [0.], s=50, color='#2563eb', marker='o', zorder=4)
        ax.scatter([4.5], [0.], s=55, color='#7c3aed', marker='x', zorder=4)
        if len(item['views']) > 1:
            views = np.array([view['origin'] for view in item['views']])
            ax.scatter(views[:, 0], views[:, 1], marker='^', s=32, color='#0891b2', label='Synthetic ray origins', zorder=4)
        ax.set_title(titles[name], loc='left', fontsize=13, pad=10)
        ax.text(.03, .91, note, transform=ax.transAxes, va='top', fontsize=8.5, color='#334155',
                bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .95, 'pad': 2.})
        ax.set(xlim=(-.6, 5.2), ylim=(-2.25, 2.25), xlabel='X (m)', ylabel='Y (m)', aspect='equal')
        ax.grid(alpha=.18)
        ax.spines[['top', 'right']].set_visible(False)
    handles, labels = [], []
    for ax in axes.ravel():
        for handle, label in zip(*ax.get_legend_handles_labels()):
            if label not in labels:
                handles.append(handle); labels.append(label)
    fig.legend(handles, labels, loc='lower center', ncol=3, frameon=False, fontsize=10)
    fig.suptitle('Native SCAN · isolated synthetic regression', fontsize=17, x=.08, ha='left')
    fig.text(.08, .922, 'Actual returned splines; unchanged production limits. No robot, SDK or motion controller.',
             fontsize=10, color='#475569')
    fig.subplots_adjust(left=.08, right=.97, top=.87, bottom=.1, hspace=.35, wspace=.2)
    output = directory/'native_scan_scenarios.png'
    fig.savefig(output, dpi=170, facecolor='white')
    plt.close(fig)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true', help='write fixture/config, do not import/init ROS')
    parser.add_argument('--case', choices=list(CASES), help='run one bounded diagnostic case only')
    parser.add_argument('--render-report', type=Path, help='draw existing passing raw output only; no ROS')
    args = parser.parse_args()
    if args.render_report is not None:
        print(render_report(args.render_report.resolve()), flush=True)
        return 0
    directory = WS/'log/offline_native_scan_scenarios'/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    print('ARTIFACTS='+str(directory), flush=True)
    fixture = prepare(directory, args.case)
    if args.prepare_only:
        print(json.dumps({k: v['point_count'] for k, v in fixture['cases'].items()}), flush=True)
        return 0
    report = run(directory, fixture)
    print('PASSED='+str(report['passed']), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
