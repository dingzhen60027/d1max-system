#!/usr/bin/env python3
"""OFFLINE_SYNTHETIC smoke of live-topic wiring, not localization or navigation QA.

Run after sourcing d1max_ros2_env.sh and this workspace's install/setup.bash.
Owns loopback Zenoh 17467 / domain 224; never launches a localizer, SDK,
controller, tracker or robot-facing router. --prepare-only starts no processes.
All generated inputs are TEST_ONLY, stationary, idealized observations. Select
--sensor-fixture analytic_room for first-return wiring checks. The legacy
pcd_crop fixture is retained, but a stored PCD crop is not first-return sensing
and may legitimately fail strict observed-free admission. Neither certifies
the live robot's dual-lidar observation model or real-world collision clearance.
"""
from array import array
import argparse
from copy import deepcopy
from datetime import datetime
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
import yaml


WS = Path(__file__).resolve().parents[3]
ARTIFACTS = WS / 'maps/processed/sc_pgo_20260923_crossfloor_complete_001'
PREFIX = '/d1max/live_planning/'
PORT, DOMAIN = 17467, '224'
KIND = 'OFFLINE_SYNTHETIC_TEST_ONLY'
FRAME, BODY, TRACKING = 'd1max_loc_map', 'd1max_loc_base_link', 'd1max_loc_tracking'


def validate_reference_selection(debug, surface_path, start_body, body_height, horizon):
    """Independent check of native discrete-reference selection, not its optimizer.

    Mirrors only the documented path coordinate/arc convention. The actual
    selected points and local target must arrive from the native diagnostic;
    this helper never generates a replacement debug message or ROS geometry.
    """
    reference = [np.asarray(start_body, dtype=float)]
    for ground in surface_path:
        body = np.asarray(ground, dtype=float) + [0., 0., body_height]
        if np.linalg.norm(body-reference[-1]) > 1e-3:
            reference.append(body)
    reference = np.asarray(reference)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
    assert len(reference) >= 2 and np.isfinite(reference).all(), 'invalid fixture reference'
    progress, target_arc = debug['progress_arc'], debug['target_arc']
    assert 0 <= progress <= target_arc <= arc[-1]+1e-6, 'native reference arcs outside path'
    if 'reference_length' in debug:
        assert abs(debug['reference_length']-arc[-1]) < 1e-5, 'native reference length mismatch'
    assert abs(target_arc-min(arc[-1], progress+horizon)) < 1e-5, 'wrong local horizon selection'
    sample = lambda distance: np.array([np.interp(distance, arc, reference[:, axis]) for axis in range(3)])
    projection, target = sample(progress), sample(target_arc)
    np.testing.assert_allclose(debug['projection'], projection, atol=1e-5, rtol=0)
    np.testing.assert_allclose(debug['local_target'], target, atol=1e-5, rtol=0)
    expected = [np.asarray(debug['planning_start'])]
    for point in [projection, *reference[(arc > progress+1e-6) & (arc < target_arc-1e-6)], target]:
        if np.linalg.norm(point-expected[-1]) > 1e-6:
            expected.append(point)
    selected = np.asarray(debug['selected_reference'])
    np.testing.assert_allclose(selected, expected, atol=1e-5, rtol=0)
    np.testing.assert_allclose(selected[-1], debug['local_target'], atol=1e-6, rtol=0)
    return dict(semantics='discrete_reference.slice(actual_start, projected_progress, target_arc)',
                reference_length_m=float(arc[-1]), progress_arc_m=progress,
                target_arc_m=target_arc, horizon_m=horizon,
                selected_points=len(selected),
                selected_last_to_local_target_m=float(np.linalg.norm(selected[-1]-target)),
                target_is_final=bool(abs(target_arc-arc[-1]) < 1e-5))


def process_sample(pid):
    """Low-overhead Linux observation, with start ticks to reject PID reuse."""
    try:
        record = Path(f'/proc/{pid}/stat').read_text()
        fields = record[record.rfind(')')+2:].split()
        status = dict(line.split(':', 1) for line in
                      Path(f'/proc/{pid}/status').read_text().splitlines() if ':' in line)
        return dict(pid=pid, state=fields[0], ppid=int(fields[1]), pgid=int(fields[2]),
                    start_ticks=int(fields[19]),
                    rss_kib=int(status.get('VmRSS', '0').split()[0]),
                    threads=int(status.get('Threads', '0').split()[0]),
                    command=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')[:1000],
                    children=[int(p) for p in Path(f'/proc/{pid}/task/{pid}/children').read_text().split()])
    except (OSError, ValueError, IndexError):
        return None


def digest(path):
    with Path(path).open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(source.read()).hexdigest()


def prepare(directory, sensor_fixture='pcd_crop'):
    """Construct a source-backed, static test fixture; no network/ROS operations."""
    from tools.pointcloud_preprocessing.ground_path_bridge import GroundPathBridge
    from d1max_pct_scan.simulation import read_xyz
    manifest = ARTIFACTS / 'manifest.json'
    record = json.loads(manifest.read_text())
    route = json.loads((ARTIFACTS / 'route_001/audit.json').read_text())
    bridge = GroundPathBridge.from_artifacts(manifest)
    path = np.asarray(route['path_xyz'], dtype=float)
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    end = int(np.searchsorted(distances, 3.0))
    assert end < route['segments'][0]['last_index'], 'fixture must stay on lower floor'
    selected = path[[0, end]]
    originals = bridge.to_localization_ground(selected, ['floor1', 'floor1']).xyz
    start_body, goal_body = originals + [0., 0., .55]
    yaw = math.atan2(*(goal_body[:2] - start_body[:2])[::-1])
    # Synthetic sensor reference is explicitly known, not an inferred robot extrinsic.
    rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                         [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
    sensor_origin = start_body + rotation @ [.30, 0., 0.]
    source_path = Path(record['source_path'])
    if digest(source_path) != record['source_sha256']:
        raise ValueError('Original-coordinate PCD source hash changed')
    if sensor_fixture == 'pcd_crop':
        cloud = read_xyz(source_path)
        keep = ((np.linalg.norm(cloud[:, :2] - start_body[:2], axis=1) <= 6.)
                & (cloud[:, 2] >= start_body[2]-1.1) & (cloud[:, 2] <= start_body[2]+2.5))
        cloud = cloud[keep]
        cloud = cloud[::max(1, math.ceil(len(cloud)/60000))]
        if len(cloud) < 100:
            raise ValueError('Too few source measurements around the selected test start')
        local_points = np.asarray((cloud - sensor_origin) @ rotation, dtype='<f4')
        sensor_semantics = 'stored original-PCD crop; NOT ray-accurate free-space evidence'
    elif sensor_fixture == 'analytic_room':
        # This deliberately different fixture isolates message/state wiring
        # from saved-map visibility. The PCT map/reference/support remain real;
        # local perception is an explicitly synthetic empty room, not fabricated
        # extra returns added to the PCD crop to make a failed test pass.
        from offline_native_scan_scenarios import first_return_cloud
        local_origin = np.array([.30, 0., .55])
        local_points = np.asarray(first_return_cloud(local_origin, None)-local_origin, dtype='<f4')
        sensor_semantics = 'analytic first returns in empty room [-2,-2,0]..[6,2,2.2], transformed to fixture start/yaw; NOT the real PCD environment'
    else:
        raise ValueError('Unknown explicitly selected sensor fixture')
    np.save(directory / 'synthetic_tracking_cloud.npy', local_points)
    sid = 'TEST_ONLY_' + uuid.uuid4().hex[:12]
    fixture = dict(kind=KIND, session_id=sid, localization_session_id=sid,
        map_version_id='TEST_ONLY_map_' + record['source_sha256'][:16],
        frame_id=FRAME, body_frame=BODY, tracking_frame=TRACKING, body_height=.55,
        start_body=start_body.tolist(), goal_body=goal_body.tolist(), yaw=yaw,
        sensor_origin=sensor_origin.tolist(), planning_endpoints=selected.tolist(),
        source_pcd=str(source_path), source_pcd_sha256=record['source_sha256'],
        synthetic_cloud_points=len(local_points), sensor_fixture=sensor_fixture,
        sensor_semantics=sensor_semantics, bridge='GroundPathBridge exact artifact-bound floor1',
        planning_manifest=str(manifest), tomogram_npz=str(ARTIFACTS / 'pct/tomogram.npz'),
        crossfloor_route_config=str(ARTIFACTS / 'route.yaml'),
        ground_support_height_tolerance_m=.20, ground_support_max_step_m=.17,
        robot_connected=False, motion_enabled=False, real_localization=False)
    from d1max_pct_scan.live_session import prepare_ground_support
    fixture.update(prepare_ground_support(fixture, directory))
    (directory / 'fixture.json').write_text(json.dumps(fixture, indent=2))
    scout = {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}
    common = dict(scouting=scout, timestamping={'enabled': True, 'drop_future_timestamp': False})
    client = dict(common, mode='client', connect={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True})
    router = dict(common, mode='router', listen={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True}, connect={'endpoints': []})
    for name, value in [('client', client), ('router', router)]:
        (directory / (name + '.json5')).write_text(json.dumps(value, indent=2))
    return fixture, local_points


def isolated_environment(directory):
    env = dict(os.environ)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        env.pop(key, None)
    env.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID=DOMAIN,
               ZENOH_SESSION_CONFIG_URI=str(directory/'client.json5'),
               ZENOH_ROUTER_CONFIG_URI=str(directory/'router.json5'),
               OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
               ROS_LOG_DIR=str(directory/'ros_logs'))
    sources = [str(WS), str(WS/'src/d1max_pct_scan'), str(WS/'src/d1max_pct_planner')]
    env['PYTHONPATH'] = os.pathsep.join(sources + [env.get('PYTHONPATH', '')])
    return env


def run(directory, fixture, points, started):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Bool, Empty, String
    from visualization_msgs.msg import Marker, MarkerArray
    from d1max_planning_interfaces.msg import TaggedBspline, ReferencePath, LocalPlanDebug
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import TransformBroadcaster

    # Fail before any process or ROS node if this isolated router endpoint exists.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', PORT))
    env = isolated_environment(directory)
    os.environ.update(env)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        os.environ.pop(key, None)
    processes, streams = [], []
    deadline = started + 92.  # reserve up to eight seconds for owned-group cleanup
    report = dict(kind=KIND, passed=False, fixture=fixture, tests={}, processes=[], latencies={},
        isolation={'domain': DOMAIN, 'port': PORT, 'robot_uplink': False},
        limitations=[fixture['sensor_semantics'], 'Ideal stationary TF/odom only.',
                     'No robot, SDK, localization algorithm, dynamics or controller tested.',
                     'SCAN marker is gated shadow visualization, not execution authorization.'])
    node = None
    observed_processes, process_maxima = {}, {}
    sampled_at = -math.inf
    total_rss_max = 0
    fast_monitor_until = -math.inf
    native_worker_max, native_worker_samples = 0, 0
    def sample_process_tree():
        nonlocal sampled_at, total_rss_max, native_worker_max, native_worker_samples
        mono = time.monotonic()
        if mono-sampled_at < (.02 if mono < fast_monitor_until else .25):
            return
        sampled_at = mono
        queue = [(p.pid, name) for name, p in processes]
        seen, total_rss, native_workers = set(), 0, 0
        global_pid = next((p.pid for name, p in processes if name == 'global'), None)
        while queue:
            pid, owner = queue.pop()
            if pid in seen:
                continue
            seen.add(pid)
            current = process_sample(pid)
            if current is None:
                continue
            key = (pid, current['start_ticks'])
            observed_processes[key] = {**current, 'owner': owner}
            total_rss += current['rss_kib']
            if (current['ppid'] == global_pid and 'spawn_main' in current['command']
                    and current['state'] != 'Z'):
                native_workers += 1
            maxima = process_maxima.setdefault(key, dict(pid=pid, owner=owner, rss_kib=0, threads=0))
            maxima['rss_kib'] = max(maxima['rss_kib'], current['rss_kib'])
            maxima['threads'] = max(maxima['threads'], current['threads'])
            queue.extend((child, owner) for child in current['children'])
        total_rss_max = max(total_rss_max, total_rss)
        native_worker_samples += 1
        native_worker_max = max(native_worker_max, native_workers)
        if native_workers > 1:
            raise RuntimeError('More than one native PCT worker simultaneously alive')
    def spawn(name, command):
        stream = (directory/(name+'.log')).open('w')
        streams.append(stream)
        child = subprocess.Popen(command, env=env, cwd=WS, stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, child))
        report['processes'].append({'name': name, 'pid': child.pid, 'command': command})
        return child
    def params(name, value):
        path = directory/(name+'.yaml')
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': value}}))
        return ['--ros-args', '--params-file', str(path)]
    def children_alive():
        sample_process_tree()
        ended = [(name, p.returncode) for name, p in processes if p.poll() is not None]
        if ended:
            raise RuntimeError('Owned test child exited: ' + repr(ended))
        if time.monotonic() >= deadline:
            raise TimeoutError('92 second test budget exhausted; cleaning up before 100 seconds')

    class SyntheticProbe(Node):
        def __init__(self):
            super().__init__('TEST_ONLY_offline_live_chain_probe', enable_rosout=False,
                             start_parameter_services=False)
            self.statuses, self.events = {}, []
            self.support_evidence = {}
            self.valid, self.cloud_enabled, self.ticks = True, True, 0
            self.soft_invalid = False
            self.epoch = 1
            self.paths, self.tags, self.markers = [], [], []
            self.visual_paths = []
            self.visual_clears = 0
            self.references, self.native_debug, self.debug_markers = {}, [], []
            self.debug_clears = 0
            self.debug_clear_events = []
            self.last_native_debug = None
            self.debug_replay = None
            self.injected_debug_stamps = set()
            self.empty_paths = self.empty_references = self.deletes = 0
            self.odom = self.create_publisher(Odometry, '/d1max/localization/odometry/global', qos_profile_sensor_data)
            self.cloud = self.create_publisher(PointCloud2, '/d1max/localization/lio/deskewed', qos_profile_sensor_data)
            self.nav = self.create_publisher(String, '/d1max/localization/navigation/status', 5)
            self.pose_lease = self.create_publisher(String, '/d1max/localization/navigation/pose_status', 5)
            self.loc = self.create_publisher(String, '/d1max/localization/status', 5)
            self.frozen = self.create_publisher(Bool, PREFIX+'execution_frozen', 5)
            self.goal = self.create_publisher(PoseStamped, PREFIX+'goal', 1)
            self.cancel = self.create_publisher(Empty, PREFIX+'cancel', 1)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(RosPath, PREFIX+'reference_path', self.on_path, 5)
            self.create_subscription(RosPath, PREFIX+'global_path_visual', self.on_visual_path, 5)
            self.create_subscription(ReferencePath, PREFIX+'scan_reference', self.on_reference, 5)
            self.create_subscription(TaggedBspline, PREFIX+'scan_tagged_bspline', self.on_tag, 5)
            self.create_subscription(Marker, PREFIX+'scan_optimal', self.on_marker, 5)
            self.create_subscription(LocalPlanDebug, PREFIX+'native_local_debug', self.on_debug, 5)
            self.create_subscription(MarkerArray, PREFIX+'local_debug', self.on_debug_markers, 5)
            for name in ('global_status', 'scan_bridge_status'):
                self.create_subscription(String, PREFIX+name,
                    lambda m, k=name: self.on_status(k, m), 5)
            self.create_timer(.02, self.publish_test_inputs)

        def on_status(self, name, message):
            value = json.loads(message.data)
            self.statuses[name] = value
            support = value.get('ground_support_check')
            if name == 'scan_bridge_status' and isinstance(support, dict) and support.get('valid') is True:
                self.support_evidence[(support['generation'], support['plan_id'])] = support

        def on_visual_path(self, msg):
            if msg.poses:
                self.visual_paths.append(dict(stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                                              count=len(msg.poses)))
            else:
                self.visual_clears += 1

        def on_path(self, msg):
            if msg.poses:
                self.paths.append({'at_s': time.monotonic()-started,
                    'frame': msg.header.frame_id,
                    'stamp_ns': msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                    'count': len(msg.poses),
                    'first': [getattr(msg.poses[0].pose.position, k) for k in 'xyz'],
                    'last': [getattr(msg.poses[-1].pose.position, k) for k in 'xyz']})
            else:
                self.empty_paths += 1

        def on_reference(self, msg):
            if not msg.path.poses:
                self.empty_references += 1
            else:
                self.references[int(msg.generation)] = dict(session_id=msg.session_id,
                    frame=msg.path.header.frame_id,
                    stamp_ns=msg.path.header.stamp.sec*1000000000+msg.path.header.stamp.nanosec,
                    points=[[getattr(p.pose.position, k) for k in 'xyz'] for p in msg.path.poses])

        def on_tag(self, msg):
            self.tags.append({'at_s': time.monotonic()-started,
                              'session_id': msg.session_id, 'generation': msg.generation,
                              'frame': msg.frame_id, 'trajectory_id': msg.trajectory.traj_id,
                              'start_stamp_ns': msg.trajectory.start_time.sec*1000000000+msg.trajectory.start_time.nanosec,
                              'control_points': len(msg.trajectory.pos_pts)})

        def on_debug(self, msg):
            if not msg.valid:
                return
            stamp_ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            injected = stamp_ns in self.injected_debug_stamps
            if not injected:
                self.last_native_debug = deepcopy(msg)
            selected = [[getattr(p.pose.position, k) for k in 'xyz'] for p in msg.selected_reference.poses]
            self.native_debug.append(dict(at_s=time.monotonic()-started,
                stamp_ns=stamp_ns, origin='TEST_ONLY_delayed_replay' if injected else 'native_transport',
                session_id=msg.session_id, generation=int(msg.generation), plan_id=int(msg.plan_id),
                frame=msg.header.frame_id, phase=msg.phase,
                selected_frame=msg.selected_reference.header.frame_id,
                projection=[getattr(msg.projection, k) for k in 'xyz'],
                local_target=[getattr(msg.local_target, k) for k in 'xyz'],
                progress_arc=float(msg.progress_arc_m), target_arc=float(msg.target_arc_m),
                planning_start=selected[0] if selected else [], selected_reference=selected))

        def on_debug_markers(self, message):
            adds = []
            cleared = False
            for marker in message.markers:
                if marker.action == Marker.DELETEALL:
                    cleared = True
                elif marker.action == Marker.ADD:
                    adds.append(dict(namespace=marker.ns, marker_id=marker.id,
                        frame=marker.header.frame_id,
                        stamp_ns=marker.header.stamp.sec*1000000000+marker.header.stamp.nanosec,
                        lifetime_s=marker.lifetime.sec+marker.lifetime.nanosec*1e-9,
                        points=[[getattr(p, k) for k in 'xyz'] for p in marker.points],
                        position=[getattr(marker.pose.position, k) for k in 'xyz']))
            if adds:
                self.debug_markers.append(dict(at_s=time.monotonic()-started, markers=adds))
            elif cleared:
                # Redraws also begin with DELETEALL; only a clear-only batch
                # is evidence that an inactive task's debug was withdrawn.
                self.debug_clears += 1
                self.debug_clear_events.append(time.monotonic()-started)

        def output_counts(self):
            return len(self.paths), len(self.markers), len(self.debug_markers)

        def replay_old_debug(self, message):
            # Created only after the first real native evidence was captured.
            if self.debug_replay is None:
                self.debug_replay = self.create_publisher(LocalPlanDebug, PREFIX+'native_local_debug', 5)
                self.wait(lambda: self.debug_replay.get_subscription_count() >= 2,
                          2., 'test_replay_publisher_discovered_by_probe_and_bridge')
            self.debug_replay.publish(message)

        def assert_replayed_debug_rejected(self, message):
            replay = deepcopy(message)
            stamp = self.get_clock().now().to_msg()
            replay.header.stamp = stamp
            replay.selected_reference.header.stamp = stamp
            for pose in replay.selected_reference.poses:
                pose.header.stamp = stamp
            before = len(self.debug_markers)
            replay_ns = stamp.sec*1000000000+stamp.nanosec
            self.injected_debug_stamps.add(replay_ns)
            self.replay_old_debug(replay)
            self.wait(lambda: any(item['stamp_ns'] == replay_ns for item in self.native_debug),
                      1., 'delayed_debug_replay_observed_on_transport')
            self.observe(.35)
            assert not any(mark['stamp_ns'] == replay_ns for batch in self.debug_markers[before:]
                           for mark in batch['markers']), 'old native generation revived debug'

        def on_marker(self, msg):
            if msg.action == Marker.DELETEALL:
                self.deletes += 1
            elif msg.action == Marker.ADD and len(msg.points) > 1:
                self.markers.append({'at_s': time.monotonic()-started,
                                     'frame': msg.header.frame_id, 'points': len(msg.points),
                                     'stamp_ns': msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec})

        def publish_test_inputs(self):
            stamp = self.get_clock().now().to_msg()
            wall = stamp.sec + stamp.nanosec*1e-9
            q = [0., 0., math.sin(fixture['yaw']/2), math.cos(fixture['yaw']/2)]
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, FRAME, BODY
            for k, value in zip('xyz', fixture['start_body']):
                setattr(msg.pose.pose.position, k, value)
            for k, value in zip('xyzw', q):
                setattr(msg.pose.pose.orientation, k, value)
            self.odom.publish(msg)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, FRAME, TRACKING
            for k, value in zip('xyz', fixture['sensor_origin']):
                setattr(tf.transform.translation, k, value)
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(tf)
            self.pose_lease.publish(String(data=json.dumps(dict(
                schema=1, epoch=self.epoch, seed_id='TEST_ONLY_seed_'+str(self.epoch),
                frame_id=FRAME, body_frame=BODY, received_at_unix=wall,
                output_stamp_sec=wall, pose_timeout_sec=.08,
                valid=self.valid and not self.soft_invalid, pose_valid=self.valid and not self.soft_invalid,
                fault='' if self.valid else 'TEST_ONLY_injected_localization_invalid',
                reset_pending=False, motion_control_enabled=False))))
            self.ticks += 1
            if self.ticks % 5:
                return
            seed_id = 'TEST_ONLY_seed_'+str(self.epoch)
            navigation = dict(schema=1, session_id=fixture['session_id'], epoch=self.epoch,
                seed_id=seed_id, valid=self.valid and not self.soft_invalid, navigation_ready=False,
                motion_control_enabled=False, body_frame=BODY, received_at_unix=wall,
                fault='' if self.valid else 'TEST_ONLY_injected_localization_invalid')
            localizer = dict(kind=KIND, session_id=fixture['session_id'],
                map_version_id=fixture['map_version_id'], wall_time=wall,
                localized=self.valid and not self.soft_invalid, local_epoch=self.epoch, active_seed_ns=seed_id,
                confirmed_seed_ns=seed_id, verified_confirmations=3,
                local_fault='', frames={'map': FRAME, 'tracking': TRACKING}, navigation=navigation)
            self.nav.publish(String(data=json.dumps(navigation)))
            self.loc.publish(String(data=json.dumps(localizer)))
            self.frozen.publish(Bool(data=True))
            if not self.cloud_enabled:
                return
            cloud = PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp, TRACKING
            cloud.height, cloud.width, cloud.point_step = 1, len(points), 12
            cloud.row_step, cloud.is_dense = 12*len(points), True
            cloud.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i, k in enumerate('xyz')]
            cloud.data = array('B', points.tobytes())
            self.cloud.publish(cloud)

        def send_goal(self):
            goal = PoseStamped()
            goal.header.frame_id, goal.header.stamp = FRAME, self.get_clock().now().to_msg()
            for k, value in zip('xyz', fixture['goal_body']):
                setattr(goal.pose.position, k, value)
            goal.pose.orientation.w = 1.
            self.goal.publish(goal)
            return goal

        def wait(self, check, timeout, label):
            wait_started = time.monotonic()
            until = min(deadline, wait_started+timeout)
            while time.monotonic() < until:
                children_alive()
                rclpy.spin_once(self, timeout_sec=.02)
                if check():
                    self.events.append({'test': label, 'elapsed_s': time.monotonic()-started,
                                        'wait_s': time.monotonic()-wait_started})
                    return
            raise TimeoutError(label + ': ' + json.dumps(self.statuses, ensure_ascii=False))

        def observe(self, seconds):
            until = min(deadline, time.monotonic()+seconds)
            while time.monotonic() < until:
                children_alive()
                rclpy.spin_once(self, timeout_sec=.02)

    try:
        router = Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd'
        spawn('router', [str(router)])
        until = min(deadline, time.monotonic()+4.)
        while time.monotonic() < until:
            children_alive()
            try:
                with socket.create_connection(('127.0.0.1', PORT), timeout=.1):
                    break
            except OSError:
                time.sleep(.05)
        else:
            raise TimeoutError('Private loopback router not ready')
        rclpy.init(args=[])
        node = SyntheticProbe()
        global_params = {k: fixture[k] for k in ('session_id',
                          'planning_manifest', 'tomogram_npz', 'crossfloor_route_config')}
        global_params.update(current_floor='floor1', goal_floor='floor1', result_timeout_s=25.)
        spawn('global', [sys.executable, '-m', 'd1max_pct_scan.live_global_planner'] + params('global', global_params))
        from d1max_pct_scan.live_session import bridge_parameters
        bridge_params = bridge_parameters(fixture, fixture['session_id'])
        spawn('bridge', [sys.executable, '-m', 'd1max_pct_scan.live_scan_bridge'] + params('bridge', bridge_params))
        # Exercise the production SCAN settings, not a more permissive test variant.
        from d1max_pct_scan.live_session import scan_parameters
        scan_params = scan_parameters(dict(
            scan_config=str(WS/'src/d1max_scan_planner/config/d1max_scan_planner.yaml'),
            robot_profile=str(WS/'src/d1max_scan_planner/config/d1max_robot.yaml'),
            body_height=.55, frame_id=FRAME, scan_preview_speed_mps=.30,
            scan_preview_acc_mps2=.35, scan_local_horizon_m=2.), fixture['session_id'])
        scan_exe = Path(get_package_prefix('scan_planner'))/'lib/scan_planner/scan_planner_node'
        remaps = {'__ns': PREFIX+'scan', 'body_pose': PREFIX+'body_pose',
            'sensor_pose': PREFIX+'sensor_pose', 'cloud': PREFIX+'cloud_map',
            'typed_initial_path': PREFIX+'scan_reference',
            'planning/go2_execution_frozen': PREFIX+'execution_frozen',
            'grid_map/localization_context': PREFIX+'scan_map_context',
            'grid_map/localization_context_ack': PREFIX+'scan_map_context_ack',
            'planning/tagged_bspline': PREFIX+'scan_tagged_bspline',
            'planning/local_plan_debug': PREFIX+'native_local_debug'}
        command = [str(scan_exe)] + params('scan', scan_params)
        for source, target in remaps.items():
            command += ['-r', source+':='+target]
        spawn('scan', command)
        node.wait(lambda: node.statuses.get('scan_bridge_status', {}).get('ready')
                  and node.statuses.get('global_status', {}).get('localization_valid')
                  and node.statuses.get('global_status', {}).get('scan_sensor_ready'),
                  30., 'isolated_inputs_ready')
        report['tests']['isolated_inputs_ready'] = True
        report['latencies']['startup_to_ready_s'] = time.monotonic()-started
        def debug_evidence_since(first, previous_generation):
            for displayed in node.debug_markers[first:]:
                marks = displayed['markers']
                stamps = {mark['stamp_ns'] for mark in marks}
                assert len(stamps) == 1, 'diagnostic geometry mixes native attempts'
                source = next((debug for debug in node.native_debug
                               if debug['stamp_ns'] == marks[0]['stamp_ns']), None)
                if source is None:
                    continue  # independent ROS callback ordering
                if source['generation'] <= previous_generation:
                    continue  # old native attempt delivered during goal handover
                tag = next((tag for tag in node.tags if tag['session_id'] == source['session_id']
                            and tag['generation'] == source['generation']
                            and tag['trajectory_id'] == source['plan_id']), None)
                if tag is None:
                    continue
                green = next((mark for mark in node.markers if mark['stamp_ns'] == tag['start_stamp_ns']), None)
                if green is None:
                    continue
                assert source['phase'] == 'accepted', 'displayed debug is not a successful native plan'
                assert source['origin'] == 'native_transport', 'test replay was displayed as current native geometry'
                assert source['session_id'] == fixture['session_id'], 'wrong diagnostic session'
                assert source['frame'] == source['selected_frame'] == FRAME, 'wrong diagnostic frame'
                reference = node.references.get(source['generation'])
                if reference is None:
                    continue
                assert reference['session_id'] == source['session_id'], 'reference/debug tag mismatch'
                assert source['stamp_ns'] >= reference['stamp_ns'], 'pre-reference debug displayed'
                np.testing.assert_allclose(source['planning_start'], fixture['start_body'], atol=1e-5, rtol=0)
                evidence = validate_reference_selection(source, reference['points'],
                    fixture['start_body'], fixture['body_height'], scan_params['fsm.planning_horizon'])
                by_name = {mark['namespace']: mark for mark in marks}
                required = {'local_reference_segment', 'local_reference_anchors', 'local_target', 'local_reference_projection'}
                assert required <= by_name.keys(), 'missing diagnostic marker geometries'
                assert all(mark['frame'] == FRAME and 0 < mark['lifetime_s'] <= 2. for mark in marks), 'unbounded diagnostic lifetime/frame'
                np.testing.assert_allclose(by_name['local_reference_segment']['points'], source['selected_reference'], atol=1e-6, rtol=0)
                np.testing.assert_allclose(by_name['local_target']['position'], source['local_target'], atol=1e-6, rtol=0)
                np.testing.assert_allclose(by_name['local_reference_projection']['position'], source['projection'], atol=1e-6, rtol=0)
                anchors = by_name['local_reference_anchors']['points']
                assert anchors, 'missing native reference anchors'
                selected = np.asarray(source['selected_reference'])
                assert all(np.linalg.norm(selected-point, axis=1).min() <= 1e-6 for point in anchors), 'fabricated reference anchor'
                return dict(**evidence, session_id=source['session_id'], generation=source['generation'],
                            native_plan_id=source['plan_id'], native_source_stamp_ns=source['stamp_ns'],
                            matching_tagged_spline_id=tag['trajectory_id'],
                            gated_marker_at_s=green['at_s'], gated_debug_at_s=displayed['at_s'],
                            marker_lifetime_max_s=max(m['lifetime_s'] for m in marks))
            return None
        def plan_once(label, timeout):
            paths_before, markers_before = len(node.paths), len(node.markers)
            debug_before = len(node.debug_markers)
            previous_generation = max(node.references, default=-1)
            issued = time.monotonic()-started
            goal = node.send_goal()
            node.wait(lambda: len(node.paths)>paths_before and len(node.markers)>markers_before
                      and debug_evidence_since(debug_before, previous_generation) is not None,
                      timeout, label)
            evidence = debug_evidence_since(debug_before, previous_generation)
            report['latencies'][label] = dict(
                global_path_s=node.paths[paths_before]['at_s']-issued,
                gated_scan_marker_s=evidence['gated_marker_at_s']-issued,
                gated_native_debug_s=evidence['gated_debug_at_s']-issued)
            report.setdefault('native_debug_evidence', []).append(evidence)
            return goal
        first_goal = plan_once('native_global_scan_and_gated_marker', 25.)
        debug_publishers = node.get_publishers_info_by_topic(PREFIX+'native_local_debug')
        assert len(debug_publishers) == 1, 'initial diagnostic has more than one source'
        report['native_debug_publishers_before_test_replay'] = [dict(node_name=p.node_name, namespace=p.node_namespace) for p in debug_publishers]
        first_debug = deepcopy(node.last_native_debug)
        assert first_debug is not None, 'no actual native diagnostic received'
        report['tests']['native_debug_geometry_source_and_tags'] = True
        report['tests']['debug_markers_have_bounded_lifetime'] = True
        clears_before, replacement_issued = node.debug_clears, time.monotonic()-started
        plan_once('replacement_explicit_goal', 15.)
        assert node.debug_clears > clears_before, 'replacing active goal left diagnostic markers'
        replacement_clear = next(t for t in node.debug_clear_events if t >= replacement_issued)
        assert replacement_clear-replacement_issued < 1., 'goal replacement diagnostic clear was delayed'
        report['latencies']['replacement_debug_clear_s'] = replacement_clear-replacement_issued
        node.assert_replayed_debug_rejected(first_debug)
        report['tests']['goal_replacement_clears_and_rejects_old_debug'] = True
        if any(t['session_id'] != fixture['session_id'] or t['frame'] != FRAME for t in node.tags):
            raise RuntimeError('SCAN output carries wrong session or frame')
        report['tests']['native_global_scan_and_gated_marker'] = True
        empty, clear, canceled, debug_clear = node.empty_paths, node.deletes, node.empty_references, node.debug_clears
        issued = time.monotonic()
        node.cancel.publish(Empty())
        node.wait(lambda: node.empty_paths > empty and node.deletes > clear and node.empty_references > canceled and node.debug_clears > debug_clear,
                  3., 'cancel_clears_global_reference_and_marker')
        report['latencies']['cancel_all_outputs_clear_s'] = time.monotonic()-issued
        report['tests']['cancel_clears'] = True
        node.observe(.7)
        output_count = node.output_counts()
        node.assert_replayed_debug_rejected(first_debug)
        node.observe(1.5)
        assert node.output_counts() == output_count, 'cancel auto-resumed'
        report['tests']['cancel_clears_debug_and_rejects_delayed_output'] = True
        plan_once('second_explicit_goal', 15.)
        empty, clear, canceled, debug_clear = node.empty_paths, node.deletes, node.empty_references, node.debug_clears
        issued = time.monotonic()
        node.valid = False
        node.wait(lambda: node.empty_paths > empty and node.deletes > clear and node.empty_references > canceled and node.debug_clears > debug_clear,
                  3., 'invalid_localization_revokes_outputs')
        report['latencies']['invalid_status_all_outputs_clear_s'] = time.monotonic()-issued
        report['tests']['invalid_localization_revokes'] = True
        node.observe(.7)
        node.valid = True
        node.wait(lambda: node.statuses.get('scan_bridge_status', {}).get('ready'), 4., 'sensor_recovery')
        output_count = node.output_counts()
        node.observe(2.)
        assert node.output_counts() == output_count, 'recovery auto-resumed old task'
        assert not node.statuses['scan_bridge_status'].get('active_reference'), 'recovery kept active reference'
        report['tests']['recovery_requires_new_goal'] = True
        # A third sequential request measures native-worker reuse/reaping and
        # exercises a new epoch while a previously accepted path is active.
        third_goal = plan_once('third_explicit_goal', 15.)
        report['tests']['three_sequential_explicit_goals'] = True
        prior_path = node.paths[-1]
        node.wait(lambda: abs(node.statuses.get('global_status', {}).get('last_route', {})
                              .get('path_stamp', 0.)-prior_path['stamp_ns']*1e-9) < 1e-6,
                  2., 'pre_dropout_route_status_matches_path')
        prior_route = dict(node.statuses['global_status']['last_route'])
        prior_global_generation = node.statuses['global_status']['generation']
        prior_reference_generation = max(node.references)
        prior_path_count = len(node.paths)
        prior_debug_count = len(node.debug_markers)
        cloud_debug = deepcopy(node.last_native_debug)
        empty, clear, canceled, debug_clear = node.empty_paths, node.deletes, node.empty_references, node.debug_clears
        issued = time.monotonic()
        node.cloud_enabled = False
        visual_before = (len(node.visual_paths), node.visual_clears)
        node.wait(lambda: node.deletes > clear and node.empty_references > canceled and node.debug_clears > debug_clear,
                  3., 'cloud_dropout_revokes_local_execution_only')
        report['latencies']['cloud_dropout_local_outputs_clear_s'] = time.monotonic()-issued
        output_count = node.output_counts()
        node.observe(.3)
        assert node.output_counts() == output_count, 'cloud dropout revived old output before recovery'
        assert node.empty_paths == empty, 'cloud loss incorrectly revoked static global reference'
        assert (len(node.visual_paths), node.visual_clears) == visual_before, 'cloud loss erased/restamped blue global context'
        assert node.statuses['global_status']['goal_retained'], 'cloud loss erased unchanged goal'
        node.cloud_enabled = True
        node.wait(lambda: node.statuses.get('scan_bridge_status', {}).get('ready'), 4., 'cloud_recovery')
        node.wait(lambda: len(node.paths) > prior_path_count
                  and max(node.references, default=-1) > prior_reference_generation
                  and debug_evidence_since(prior_debug_count, prior_reference_generation) is not None,
                  15., 'short_cloud_recovery_revalidates_global_and_replans_local')
        resumed_path = node.paths[-1]
        node.wait(lambda: abs(node.statuses.get('global_status', {}).get('last_route', {})
                              .get('path_stamp', 0.)-resumed_path['stamp_ns']*1e-9) < 1e-6,
                  2., 'resumed_route_status_matches_new_path')
        resumed_route = node.statuses['global_status']['last_route']
        original_goal_stamp = third_goal.header.stamp.sec + third_goal.header.stamp.nanosec*1e-9
        assert resumed_path['stamp_ns'] > prior_path['stamp_ns'], 'old Path stamp reused on recovery'
        assert node.statuses['global_status']['generation'] == prior_global_generation, 'cloud dropout needlessly reran immutable global task'
        assert max(node.references) > prior_reference_generation, 'old SCAN reference generation reused'
        assert resumed_route['goal_stamp'] == original_goal_stamp, 'user goal identity changed during short pause'
        assert resumed_route['attempt_stamp'] == prior_route['attempt_stamp'], 'unchanged static result was needlessly recomputed'
        assert resumed_route['source_tomogram_sha256'] == digest(fixture['tomogram_npz']), 'resumed path source map changed'
        node.assert_replayed_debug_rejected(cloud_debug)
        report['tests']['short_cloud_dropout_retains_global_but_requires_new_local_trajectory'] = True
        report['latencies']['short_cloud_dropout_to_revalidated_path_s'] = resumed_path['at_s']-(issued-started)
        # A genuinely unavailable navigation output pauses global commitment.
        # The original blue context remains until the bounded 10-second commit
        # wait expires. This is distinct from cloud-only loss above.
        empty, clear, canceled, debug_clear = node.empty_paths, node.deletes, node.empty_references, node.debug_clears
        node.soft_invalid = True
        visual_before = (len(node.visual_paths), node.visual_clears)
        node.wait(lambda: node.empty_paths > empty and node.deletes > clear and node.empty_references > canceled and node.debug_clears > debug_clear,
                  3., 'navigation_soft_dropout_revokes_execution')
        node.wait(lambda: node.statuses.get('global_status', {}).get('paused_for_recovery'),
                  1., 'navigation_soft_pause_observed')
        node.observe(.5)
        assert (len(node.visual_paths), node.visual_clears) == visual_before, 'soft nav loss erased/restamped blue context'
        node.wait(lambda: node.statuses.get('global_status', {}).get('goal_retained') is False
                  and node.statuses.get('global_status', {}).get('reason') == 'global_result_commit_wait_expired',
                  11., 'navigation_commit_wait_expired')
        node.soft_invalid = False
        node.wait(lambda: node.statuses.get('scan_bridge_status', {}).get('ready'), 4., 'long_navigation_dropout_recovery')
        output_count = node.output_counts()
        node.observe(.8)
        assert node.output_counts() == output_count, 'expired goal auto-resumed after cloud recovery'
        assert not node.statuses['scan_bridge_status'].get('active_reference'), 'expired reference remained active'
        report['tests']['navigation_commit_timeout_requires_new_goal'] = True
        plan_once('new_goal_after_cloud_recovery', 15.)
        empty, clear, canceled, debug_clear = node.empty_paths, node.deletes, node.empty_references, node.debug_clears
        issued = time.monotonic()
        node.epoch = 2
        node.wait(lambda: node.empty_paths > empty and node.deletes > clear and node.empty_references > canceled and node.debug_clears > debug_clear,
                  3., 'new_epoch_revokes_previous_path')
        report['latencies']['epoch_change_all_outputs_clear_s'] = time.monotonic()-issued
        node.wait(lambda: node.statuses.get('scan_bridge_status', {}).get('ready')
                  and node.statuses.get('scan_bridge_status', {}).get('localization_epoch') == 2,
                  4., 'new_epoch_inputs_ready')
        node.observe(.5)
        output_count = node.output_counts()
        node.assert_replayed_debug_rejected(first_debug)
        # Delayed old-goal input and a regressing epoch cannot restore task 1.
        node.goal.publish(first_goal)
        node.epoch = 1
        node.observe(.3)
        node.epoch = 2
        node.wait(lambda: node.statuses.get('scan_bridge_status', {}).get('ready')
                  and node.statuses.get('scan_bridge_status', {}).get('localization_epoch') == 2,
                  4., 'restored_current_epoch_inputs')
        node.observe(1.)
        assert node.output_counts() == output_count, 'old epoch/goal revived outputs'
        assert not node.statuses['scan_bridge_status'].get('active_reference'), 'old epoch remained active'
        report['tests']['old_epoch_and_goal_cannot_resurrect'] = True
        # Burst replacements exercise the latest-pending request path, not just
        # serial goals. Source geometry remains the exact same three-metre line.
        fast_monitor_until = time.monotonic()+4.
        node.send_goal()
        node.observe(.08)
        node.send_goal()
        node.observe(.02)
        node.send_goal()
        node.observe(.02)
        empty, clear, canceled, debug_clear = node.empty_paths, node.deletes, node.empty_references, node.debug_clears
        issued = time.monotonic()
        node.cancel.publish(Empty())
        node.wait(lambda: node.empty_paths > empty and node.deletes > clear and node.empty_references > canceled and node.debug_clears > debug_clear,
                  3., 'rapid_replacement_cancel_clears')
        report['latencies']['rapid_replacement_cancel_s'] = time.monotonic()-issued
        node.wait(lambda: not node.statuses.get('global_status', {}).get('planning', True)
                  and not node.statuses.get('global_status', {}).get('pending_worker_start', True),
                  3., 'rapid_replacement_no_pending_start')
        node.observe(.5)
        output_count = node.output_counts()
        node.observe(1.)
        assert node.output_counts() == output_count, 'canceled pending goal revived'
        report['tests']['rapid_replacement_cancel_discards_pending'] = True
        fast_monitor_until = time.monotonic()+4.
        node.send_goal()
        node.observe(.08)
        node.send_goal()
        node.observe(.02)
        empty, clear, canceled, debug_clear = node.empty_paths, node.deletes, node.empty_references, node.debug_clears
        issued = time.monotonic()
        node.epoch = 3
        node.wait(lambda: node.empty_paths > empty and node.deletes > clear and node.empty_references > canceled and node.debug_clears > debug_clear,
                  3., 'rapid_replacement_new_epoch_clears')
        report['latencies']['pending_epoch_change_s'] = time.monotonic()-issued
        node.wait(lambda: node.statuses.get('scan_bridge_status', {}).get('ready')
                  and node.statuses.get('scan_bridge_status', {}).get('localization_epoch') == 3
                  and not node.statuses.get('global_status', {}).get('planning', True)
                  and not node.statuses.get('global_status', {}).get('pending_worker_start', True),
                  4., 'new_epoch_has_no_pending_worker')
        node.observe(.5)
        output_count = node.output_counts()
        node.observe(1.)
        assert node.output_counts() == output_count, 'epoch change revived pending goal'
        report['tests']['rapid_replacement_epoch_discards_pending'] = True
        assert native_worker_max == 1, 'native worker sampling did not establish single-worker execution'
        report['tests']['at_most_one_observed_native_worker'] = True
        forbidden = ('/cmd_vel', '/d1max/pct_scan/cmd_vel_safe', '/d1max/navigation/cmd_vel')
        assert not any(node.get_publishers_info_by_topic(t) for t in forbidden), 'Unexpected motion publisher'
        report['tests']['no_motion_publishers'] = True
        assert node.support_evidence, 'native local spline never passed actual PCT support validation'
        assert all(e['support_index_sha256'] == fixture['ground_support_sha256']
                   and e['motion_authorized'] is False
                   and e['foot_placement_or_swept_volume_certified'] is False
                   for e in node.support_evidence.values()), 'ground support provenance/boundary mismatch'
        report['tests']['native_splines_checked_against_exact_pct_support_index'] = True
        report['passed'] = True
    except BaseException as error:
        report['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        cleanup_started = time.monotonic()
        if node is not None:
            report.update(statuses=node.statuses, events=node.events, paths=node.paths,
                          ground_support_evidence=list(node.support_evidence.values()),
                          visual_paths=node.visual_paths, visual_path_clears=node.visual_clears,
                          tagged_splines=node.tags, gated_markers=node.markers,
                          native_debug=node.native_debug, gated_debug_markers=node.debug_markers,
                          debug_marker_clears=node.debug_clears, debug_clear_events=node.debug_clear_events,
                          empty_paths=node.empty_paths, empty_scan_references=node.empty_references,
                          marker_clears=node.deletes)
            if rclpy.ok():
                node.cancel.publish(Empty())
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        # Each child has its own process group, so native workers cannot orphan.
        for _, child in reversed(processes):
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        cleanup_end = time.monotonic()+4.
        for _, child in reversed(processes):
            try:
                child.wait(timeout=max(.01, cleanup_end-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=1.)
        for stream in streams:
            stream.close()
        report['shutdown_rclerrors'] = [name for name, _ in processes
            if 'RCLError' in (directory/(name+'.log')).read_text(errors='replace')]
        report['tests']['shutdown_without_rclerror'] = not report['shutdown_rclerrors']
        # Parent exit alone is insufficient: observe all descendants that were
        # actually seen, comparing kernel start time rather than PID alone.
        remaining = []
        for (pid, start_ticks), observed in observed_processes.items():
            current = process_sample(pid)
            if current and current['start_ticks'] == start_ticks and current['state'] != 'Z':
                remaining.append({**current, 'owner': observed['owner']})
        report['process_resources'] = dict(sampling_period_s=.25, burst_sampling_period_s=.02,
            peak_tree_rss_kib=total_rss_max, per_process_peaks=list(process_maxima.values()),
            native_worker_samples=native_worker_samples,
            max_simultaneous_observed_native_workers=native_worker_max,
            observed_descendants=[value for value in observed_processes.values()
                                  if value['pid'] not in {p.pid for _, p in processes}])
        report['remaining_observed_descendants'] = remaining
        report['all_observed_descendants_exited'] = not remaining
        report['latencies']['owned_process_cleanup_s'] = time.monotonic()-cleanup_started
        report['elapsed_s'] = time.monotonic()-started
        report['all_owned_children_exited'] = all(p.poll() is not None for _, p in processes)
        with socket.socket() as probe:
            probe.settimeout(.2)
            report['private_router_port_closed'] = probe.connect_ex(('127.0.0.1', PORT)) != 0
        report['tests']['private_router_port_released'] = report['private_router_port_closed']
        if remaining or not report['all_owned_children_exited']:
            report['passed'] = False
            report['error'] = 'Owned process/descendant cleanup not confirmed'
        elif report['shutdown_rclerrors']:
            report['passed'] = False
            report['error'] = 'ROS shutdown RCLError in: '+', '.join(report['shutdown_rclerrors'])
        elif not report['private_router_port_closed']:
            report['passed'] = False
            report['error'] = 'Private router port still accepts connections after cleanup'
        (directory/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({'kind': KIND, 'passed': report['passed'], 'report': str(directory/'report.json'),
                      'tests': report['tests'], 'error': report.get('error')}, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--sensor-fixture', choices=('pcd_crop', 'analytic_room'), default='pcd_crop')
    args = parser.parse_args()
    # Required modules are workspace source, not whatever previous install happens to contain.
    for path in (WS, WS/'src/d1max_pct_scan', WS/'src/d1max_pct_planner'):
        sys.path.insert(0, str(path))
    started = time.monotonic()
    directory = WS/'log/offline_live_chain_smoke'/(
        datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    fixture, points = prepare(directory, args.sensor_fixture)
    if args.prepare_only:
        print(json.dumps({'kind': KIND, 'prepared': str(directory), 'fixture': fixture}, indent=2))
        return 0
    return run(directory, fixture, points, started)


if __name__ == '__main__':
    raise SystemExit(main())
