#!/usr/bin/env python3
"""Actual bag -> actual C++ ray adapter -> pure projector -> actual native map.

No ROS init, graph, discovery, router, SDK, robot, or velocity publisher. CDR
serialization alone does not create a ROS context. Replay uses original source
and bag receipt times as a deterministic clock; processing latency is measured
separately, so success does NOT certify real-time scheduling or robot motion.
"""
from array import array
from bisect import bisect_right
from collections import Counter
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

WS = Path(__file__).resolve().parents[3]
for package in ('d1max_pct_scan', 'd1max_localization'):
    sys.path.insert(0, str(WS/'src'/package))
from d1max_localization.math_utils import Pose3, compose, normalize_quaternion, yaw_from_quaternion
from d1max_pct_scan.ray_projection import (MapContext, RayProjectorCore, checked_pose,
    decode_rays, AwaitingCoverage, ProjectionError, FIELDS)
from d1max_pct_scan.live_scan_contract import localization_context


def stamp_ns(stamp):
    return int(stamp.sec)*1000000000+int(stamp.nanosec)


def pose(value):
    return checked_pose((value.position.x, value.position.y, value.position.z),
                        (value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w))


def stats(values):
    return {} if not values else dict(count=len(values), minimum=float(np.min(values)),
        median=float(np.median(values)), p95=float(np.percentile(values, 95)), maximum=float(np.max(values)))


def run(args):
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import PointCloud2, PointField
    from d1max_planning_interfaces.msg import ProjectedRays

    database = args.bag.resolve()/'bag_0.db3'
    session = json.loads((args.session/'session.json').read_text())
    configuration = yaml.safe_load((args.session/'localization.yaml').read_text())
    adapter = configuration['dual_lidar_adapter']['ros__parameters']
    native_parameters = yaml.safe_load((args.session/'scan.yaml').read_text())['/**']['ros__parameters']
    connection = sqlite3.connect('file:'+str(database)+'?mode=ro', uri=True)
    topics = {int(identifier): (name, type_name) for identifier, name, type_name in
              connection.execute('SELECT id,name,type FROM topics')}
    required = ['/front_lidar', '/rear_lidar', '/d1max/localization/odometry/local',
                '/d1max/localization/pose', '/d1max/localization/odometry/global',
                '/d1max/localization/status', '/d1max/localization/navigation/status',
                '/d1max/localization/navigation/pose_status', '/tf_static',
                '/d1max/live_planning/scan_map_context_ack']
    missing = set(required)-{x[0] for x in topics.values()}
    if missing:
        raise ValueError('Bag missing required inputs: '+repr(missing))
    counts = {topics[i][0]: count for i, count in connection.execute(
        'SELECT topic_id,count(*) FROM messages GROUP BY topic_id')}
    events, raw_inputs, offsets, contexts, static = [], [], [], [], {}
    local_frames, global_frames = set(), set()
    report = dict(kind='REAL_BAG_PURE_OFFLINE_PROJECTOR_NATIVE_MAP', no_ros_initialized=True,
                  no_robot_or_motion=True, bag=str(database), session=str(args.session),
                  topic_counts=counts, input_sha256=hashlib.sha256(database.read_bytes()).hexdigest(),
                  limitations=['Deterministic bag receipt clock, not an executor/transport scheduling benchmark.',
                    'Recorded shared clock offset is approximate and rounded to six decimal places.',
                    'Robot geometry/extrinsic/time acceptance flags remain false; no motion approval.'])
    report['source_hashes'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in (
        WS/'src/d1max_pct_scan/d1max_pct_scan/ray_projection.py',
        WS/'src/scan_planner_vendor/plan_env/src/grid_map.cpp',
        WS/'src/d1max_localization/include/d1max_localization/perception_rays.hpp',
        args.session/'localization.yaml', args.session/'scan.yaml')}
    args.output.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryDirectory(prefix='d1max_offline_rays_') as temporary:
        stage = Path(temporary)
        for topic_id, timestamp, data in connection.execute(
                'SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
            name, type_name = topics[topic_id]
            if name not in required:
                continue
            if name in ('/front_lidar', '/rear_lidar'):
                source = 0 if name == '/front_lidar' else 1
                index = len(raw_inputs)
                filename = stage/f'input_{index}.cdr'
                filename.write_bytes(data)
                raw_inputs.append(dict(index=index, sensor=source, received=timestamp, input=filename))
                continue
            message = deserialize_message(data, get_message(type_name))
            if name == '/tf_static':
                for transform in message.transforms:
                    key = (transform.header.frame_id, transform.child_frame_id)
                    v, q = transform.transform.translation, transform.transform.rotation
                    value = checked_pose((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))
                    if key in static and static[key] != value:
                        raise ValueError('Static transform changed inside recorded bag: '+repr(key))
                    static[key] = value
            elif name.endswith('scan_map_context_ack'):
                contexts.append(json.loads(message.data))
            else:
                if name == '/d1max/localization/status':
                    status = json.loads(message.data)
                    clock = status['input_clock']
                    if clock.get('ready') != 'true':
                        raise ValueError('Recorded shared input clock was not ready')
                    offsets.append((timestamp, float(clock['offset_seconds']), clock.get('approximate')))
                if name.endswith('/odometry/local'):
                    local_frames.add((message.header.frame_id, message.child_frame_id))
                if name == '/d1max/localization/pose':
                    global_frames.add(message.header.frame_id)
                events.append((timestamp, name, message))
        if not contexts or any(value != contexts[0] for value in contexts):
            raise ValueError('This bounded harness requires one recorded localization context')
        context = MapContext.parse(contexts[0])
        if context.session_id != session['id']:
            raise ValueError('Recorded bag/session identity mismatch')
        if len(local_frames) != 1 or len(global_frames) != 1:
            raise ValueError('Recorded frame identity changed')
        odom_frame, body_frame = next(iter(local_frames)); map_frame = next(iter(global_frames))
        tracking_frame = configuration['lio_localizer']['ros__parameters']['tracking_frame']
        ray_frame = adapter['target_frame']
        body_tracking = static[(body_frame, tracking_frame)]
        tracking_ray = static[(tracking_frame, ray_frame)]
        if not offsets or len({value for _, value, _ in offsets}) != 1:
            raise ValueError('This audit requires recorded stable shared clock offset')
        shared_offset = offsets[0][1]
        report['clock'] = dict(offset_seconds=shared_offset, source='recorded localization/status.input_clock',
                               approximate=offsets[0][2], decimal_places=6)
        front = Pose3(tuple(adapter['front_translation']), normalize_quaternion(adapter['front_rotation']))
        rear = compose(front, Pose3(tuple(adapter['rear_to_front_translation']),
                                    normalize_quaternion(adapter['rear_to_front_rotation'])))
        requests = []
        for value in raw_inputs:
            source, index = value['sensor'], value['index']
            transform = front if source == 0 else rear
            output = stage/f'raw_{index}.cdr'; value['raw'] = output
            requests.append(dict(input_cdr=str(value['input']), output_cdr=str(output), sensor_id=source,
                source_frame=adapter['front_frame' if source == 0 else 'rear_frame'], target_frame=ray_frame,
                translation=transform.position, rotation=transform.orientation,
                clock_offset=shared_offset, ros_now=value['received']*1e-9,
                min_range=adapter['min_range'], max_range=adapter['max_range'],
                scan_period=adapter['scan_period_sec'], relative_timestamp_scale=adapter['relative_timestamp_scale']))
        request_file = stage/'conversion.jsonl'
        request_file.write_text(''.join(json.dumps(value)+'\n' for value in requests))
        began = time.perf_counter()
        conversion = subprocess.run([str(args.converter), str(request_file)], text=True,
                                    capture_output=True, timeout=120, check=True)
        report['conversion_wall_ms'] = (time.perf_counter()-began)*1000
        conversion_results = [json.loads(line) for line in conversion.stdout.splitlines()]
        if len(conversion_results) != len(requests):
            raise ValueError('Converter output count mismatch: '+conversion.stderr[-1000:])
        report['adapter'] = dict(ok=sum(bool(v.get('ok')) for v in conversion_results),
                                 results=conversion_results)
        for value, result in zip(raw_inputs, conversion_results):
            value['input'].unlink()
            if result.get('ok'):
                events.append((value['received'], 'raw', value))
        events.sort(key=lambda entry: entry[0])
        begin_ns, end_ns = events[0][0], events[-1][0]
        core = RayProjectorCore(); core.reset(context)
        core.set_extrinsics(body_to_tracking=body_tracking, ray_to_tracking=tracking_ray)
        statuses = {'localizer': {}, 'navigation': {}, 'pose': {}}
        receipts = {name: 0 for name in statuses}
        status_topics = {'/d1max/localization/status': 'localizer',
            '/d1max/localization/navigation/status': 'navigation',
            '/d1max/localization/navigation/pose_status': 'pose'}
        pending, latest_body = [None, None], None
        projector_times, native_times, native_accept_times, ticks = [], [], [], []
        drop, successes = Counter(), Counter()
        process = subprocess.Popen([str(args.probe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True, bufsize=1)

        def rpc(value):
            process.stdin.write(json.dumps(value)+'\n'); process.stdin.flush()
            result = json.loads(process.stdout.readline())
            if 'error' in result:
                raise RuntimeError(result['error'])
            return result

        def identity(now):
            if any(not 0 <= now-receipts[key] <= 500000000 for key in statuses):
                return None
            try:
                return localization_context(statuses['localizer'], statuses['navigation'],
                    pose_status=statuses['pose'], session_id=context.session_id, now=now*1e-9,
                    timeout=.5, map_frame=map_frame, tracking_frame=tracking_frame,
                    body_frame=body_frame, pose_receipt_age=(now-receipts['pose'])*1e-9)
            except (ValueError, TypeError, KeyError):
                return None

        def make_projected(projected, output):
            message = ProjectedRays()
            message.session_id, message.epoch, message.seed_id = context.session_id, context.epoch, context.seed_id
            message.context_sequence, message.barrier_ns = context.sequence, context.barrier_ns
            message.projection_sequence = projected.sequence
            for target, ns in ((message.acquisition_end, projected.end_ns),
                               (message.alignment_stamp, projected.alignment_ns),
                               (message.rays.header.stamp, projected.start_ns)):
                target.sec, target.nanosec = divmod(int(ns), 1000000000)
            cloud = message.rays; cloud.header.frame_id = map_frame
            cloud.width, cloud.height, cloud.point_step = len(projected.points), 1, 64
            cloud.row_step = cloud.width*64; cloud.is_dense = True
            cloud.fields = [PointField(name=n, offset=o, datatype=t, count=c) for n,o,t,c in FIELDS]
            payload = array('B'); payload.frombytes(memoryview(projected.points).cast('B')); cloud.data = payload
            output.write_bytes(serialize_message(message))

        try:
            native_parameters['grid_map.localization_session_id'] = context.session_id
            rpc(dict(type='configure', parameters=native_parameters, context=contexts[0]))
            index = 0
            for now in range(begin_ns, end_ns+600000001, 10000000):
                while index < len(events) and events[index][0] <= now:
                    received, topic, message = events[index]; index += 1
                    if topic in status_topics:
                        key = status_topics[topic]; statuses[key] = json.loads(message.data); receipts[key] = received
                    elif topic.endswith('/odometry/global'):
                        latest_body = pose(message.pose.pose)
                    elif topic == 'raw':
                        if identity(now) is None:
                            drop['raw_context_unavailable'] += 1; continue
                        cloud = deserialize_message(message['raw'].read_bytes(), PointCloud2)
                        message['raw'].unlink()
                        try:
                            raw = decode_rays(data=cloud.data, fields=[(f.name,f.offset,f.datatype,f.count) for f in cloud.fields],
                                point_step=cloud.point_step, row_step=cloud.row_step, width=cloud.width, height=cloud.height,
                                bigendian=cloud.is_bigendian, header_ns=stamp_ns(cloud.header.stamp),
                                frame_id=cloud.header.frame_id, expected_frame=ray_frame)
                            if pending[raw.sensor_id] is not None: drop['raw_overwritten'] += 1
                            pending[raw.sensor_id] = (raw, received)
                        except ProjectionError as error: drop[str(error)] += 1
                    elif topic in ('/d1max/localization/odometry/local', '/d1max/localization/pose'):
                        if identity(now) is None: continue
                        try:
                            if topic.endswith('/local'):
                                core.add_local_body(stamp_ns(message.header.stamp), pose(message.pose.pose), context)
                            else: core.add_global_tracking(stamp_ns(message.header.stamp), pose(message.pose), context)
                        except ProjectionError as error: drop['pose:'+str(error)] += 1
                if identity(now) is None:
                    pending = [None, None]
                else:
                    for sensor in (0,1):
                        item = pending[sensor]
                        if item is None: continue
                        raw, received = item
                        if now-received > 250000000:
                            drop['pending_expired'] += 1; pending[sensor] = None; continue
                        started = time.perf_counter()
                        try:
                            projected = core.project(raw, context, now_ns=now,
                                authorized_pose_ns=round(statuses['pose']['output_stamp_sec']*1e9))
                        except AwaitingCoverage: continue
                        except ProjectionError as error:
                            drop[str(error)] += 1; pending[sensor] = None; continue
                        projector_times.append((time.perf_counter()-started)*1000)
                        output = stage/'projected.cdr'; make_projected(projected, output)
                        accepted = rpc(dict(type='rays', file=str(output), now_ns=now)); output.unlink()
                        native_accept_times.append(accepted['processing_ms'])
                        successes[str(sensor)] += bool(accepted['accepted'])
                        if not accepted['accepted']: drop['native_rejected'] += 1
                        pending[sensor] = None
                if (now-begin_ns) % 50000000 == 0:
                    queries = []
                    if latest_body is not None:
                        yaw = yaw_from_quaternion(latest_body.orientation)
                        for label, distance in (('body',0.), ('ahead_2m',2.)):
                            p = list(latest_body.position); p[0] += distance*math.cos(yaw); p[1] += distance*math.sin(yaw)
                            queries.append(dict(label=label, position=p, yaw=yaw))
                    tick = rpc(dict(type='tick', now_ns=now, queries=queries)); tick['now_ns'] = now
                    native_times.append(tick['processing_ms']); ticks.append(tick)
            report['native_final'] = rpc(dict(type='summary', queries=queries))
        finally:
            process.stdin.close()
            process.wait(timeout=5)
            if process.returncode: raise RuntimeError(process.stderr.read()[-2000:])
        report['projector'] = dict(successfully_native_accepted=dict(successes), drops=dict(drop),
                                   processing_ms=stats(projector_times), fault=core.fault)
        report['native_timing_ms'] = dict(integrate_and_query=stats(native_times), accept=stats(native_accept_times))
        active_integration = []
        previous_counts = [0,0]
        for value in ticks:
            if value['integrated_counts'] != previous_counts:
                active_integration.append(value['processing_ms'])
            previous_counts = value['integrated_counts']
        report['native_timing_ms']['nonempty_integrate_and_query'] = stats(active_integration)
        report['native_timing_ms']['nonempty_ticks_exceeding_50ms'] = sum(value>50. for value in active_integration)
        active_ticks = [value for value in ticks if value['now_ns'] <= end_ns]
        report['integration_lease'] = dict(active_ticks=len(active_ticks),
            valid_ticks=sum(value['source_stamp_ns']>0 for value in active_ticks),
            first_valid_ns=next((value['now_ns'] for value in active_ticks if value['source_stamp_ns']>0), None),
            expired_after_end=ticks[-1]['source_stamp_ns']==0)
        for label in ('body','ahead_2m'):
            entries = [query for value in active_ticks if value['source_stamp_ns']>0
                       for query in value['queries'] if query['label']==label]
            report[label] = dict(count=len(entries), blocked=sum(e['collision']!=0 for e in entries),
                last=entries[-1] if entries else None)
        (args.output/'events.jsonl').write_text(''.join(json.dumps(value)+'\n' for value in ticks))
        (args.output/'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({key:value for key,value in report.items() if key!='adapter'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--converter', type=Path, default=WS/'build/d1max_localization/perception_rays_offline')
    parser.add_argument('--probe', type=Path, default=WS/'build/plan_env/offline_projected_rays_probe')
    run(parser.parse_args())
