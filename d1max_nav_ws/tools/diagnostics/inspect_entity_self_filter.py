#!/usr/bin/env python3
"""Read at most two real normalized scans; no ROS init, playback or SDK.

This checks the production entity classifier, not navigation success. Fixed
CAD stance needs no SDK joints; its posture remains an explicit approximation.
Recorded occupancy is never rewritten.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src/d1max_pct_scan'), str(ROOT/'src/d1max_localization')]


def main():
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from d1max_localization.math_utils import Pose3, compose
    from d1max_pct_scan.ray_projection import RAY_DTYPE, transform_many
    from d1max_pct_scan.robot_self_filter import RobotSelfFilter

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--extrinsics-report', type=Path, required=True)
    parser.add_argument('--config', type=Path,
        default=ROOT/'src/d1max_pct_scan/config/entity_self_filter.yaml')
    parser.add_argument('--output', type=Path, help='Save the generated comparison report')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    config['model_manifest'] = str(args.config.parent/config['model_manifest'])
    started = time.perf_counter()
    entity = RobotSelfFilter.load(config)
    load_ms = (time.perf_counter()-started)*1000.
    evidence = json.loads(args.extrinsics_report.read_text())['recorded_static_extrinsics']
    def pose(key):
        value = evidence[key]['pose']
        return Pose3(tuple(value['position_xyz_m']), tuple(value['quaternion_xyzw']))
    body_from_ray = compose(pose('d1max_loc_base_link -> d1max_loc_tracking'),
        pose('d1max_loc_tracking -> d1max_loc_lidar'))
    metadata_path = args.bag/'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
    samples, sensors, joint_topics = [], set(), set()
    for relative in metadata['relative_file_paths']:
        path = args.bag/relative
        with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            topics = list(db.execute('SELECT id,name,type FROM topics'))
            joint_topics.update(name for _, name, kind in topics
                                if 'joint' in name.lower() or 'JointState' in kind)
            topic = next((value for value in topics if value[1] ==
                          '/d1max/localization/perception/rays_raw'), None)
            if topic is None or len(sensors) == 2:
                continue
            # Interleaved source topic: bounded CDR reads, not a bag replay.
            for mid, receipt, cdr in db.execute(
                    'SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 6',
                    (topic[0],)):
                cloud = deserialize_message(cdr, PointCloud2)
                points = np.frombuffer(bytes(cloud.data), dtype=RAY_DTYPE)
                sensor = int(points['sensor_id'][0])
                if sensor in sensors:
                    continue
                sensors.add(sensor)
                start_ns = int(cloud.header.stamp.sec)*1_000_000_000+int(cloud.header.stamp.nanosec)
                endpoints = transform_many(body_from_ray,
                    np.column_stack([points[name] for name in ('x', 'y', 'z')]))
                origin = transform_many(body_from_ray,
                    np.asarray([[points[name][0] for name in ('origin_x', 'origin_y', 'origin_z')]]))[0]
                result = entity.classify(endpoints, origin,
                    start_ns+points['offset_time'].astype(np.int64), sensor_id=sensor)
                retained = points[~result.mask]
                samples.append(dict(database=str(path), message_id=int(mid), receipt_ns=int(receipt),
                    cdr_sha256=hashlib.sha256(cdr).hexdigest(), source_begin_ns=start_ns,
                    source_end_ns=start_ns+int(points['offset_time'].max()), sensor_id=sensor,
                    ray_origin_body_m=origin.tolist(), before_points=len(points), after_points=len(retained),
                    before_raw_bytes_sha256=hashlib.sha256(points.tobytes()).hexdigest(),
                    after_raw_bytes_sha256=hashlib.sha256(retained.tobytes()).hexdigest(),
                    requires_joint_stream=entity.requires_joint_stream, classification=result.report))
    nominal = entity.model.to_dict() if not entity.requires_joint_stream else None
    if nominal:
        nominal = {k: v for k, v in nominal.items() if k not in ('base_model', 'nominal_link_transforms_root')}
    report = dict(schema=1, diagnostic='bounded_actual_entity_filter_two_scan_comparison',
        bag=str(args.bag), metadata_sha256=hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
        model_digest=entity.digest, load_ms=load_ms, warmup=entity.warmup_report,
        recorded_joint_topics=sorted(joint_topics), samples=samples,
        scope=dict(ros_initialized=False, sdk_connected=False, replay=False,
                   maps_mutated=False, near_body_cleared=False, physical_acceptance=False),
        pose_source=entity.pose_source,
        nominal_stance=nominal,
        unresolved=['Fixed stance is nominal CAD geometry, not physical posture calibration',
                    'Runtime rear sensor origin differs from CAD by approximately 0.076 m',
                    'Sensor-frame origins are not certified per-beam optical origins'])
    encoded = json.dumps(report, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
        print(json.dumps(dict(output=str(args.output), samples=[{
            'sensor_id': sample['sensor_id'], 'before': sample['before_points'],
            'after': sample['after_points'], 'classification': sample['classification']}
            for sample in samples])))
    else:
        print(encoded)


if __name__ == '__main__':
    main()
