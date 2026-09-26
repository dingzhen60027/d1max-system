#!/usr/bin/env python3
"""Read four raw bag clouds; audit dual-LiDAR adapter without ROS initialization."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sqlite3
import struct

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

HERE = Path(__file__).resolve().parent
BAG = Path('/home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38')
CAL = HERE.parent/'lio_frontend_reliability_20260919/extrinsic_trials_20260919/calibrations/device_front_plus_runtime_rear.yaml'


def import_adapter():
    spec = importlib.util.spec_from_file_location('dual_adapter', HERE/'adapter.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_cloud(db, topic_id, header_ns, record_ns):
    candidates = db.execute('SELECT id, timestamp, substr(data,1,12) FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp LIMIT 24',
                            (topic_id, record_ns-500_000_000, record_ns+500_000_000)).fetchall()
    def stamp(row):
        sec, nsec = struct.unpack_from('<iI', row[2], 4)
        return sec*1_000_000_000+nsec
    row = min(candidates, key=lambda x: abs(stamp(x)-header_ns))
    assert abs(stamp(row)-header_ns) < 2_000_000
    payload = db.execute('SELECT data FROM messages WHERE id=?', (row[0],)).fetchone()[0]
    return deserialize_message(payload, PointCloud2), {'sqlite_id': row[0], 'record_ns': row[1], 'header_ns': stamp(row)}


def native_selected(msg, common_ns):
    fields = {f.name: f for f in msg.fields}
    names = ['x', 'y', 'z', 'intensity', 'ring', 'timestamp']
    dtype = np.dtype({'names': names, 'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                      'offsets': [fields[k].offset for k in names], 'itemsize': msg.point_step})
    raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width*msg.height)
    xyz = np.column_stack([raw[k] for k in ('x', 'y', 'z')])
    header = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
    own_rel = raw['timestamp']-header*1e-9
    rel = raw['timestamp']-common_ns*1e-9
    ranges = np.linalg.norm(xyz, axis=1)
    selected = (np.isfinite(xyz).all(axis=1) & np.isfinite(rel) & np.isfinite(raw['intensity']) &
                (raw['ring'] < 96) & (own_rel >= -1e-6) & (own_rel < .15) &
                (rel >= -1e-6) & (rel < .16) & (ranges > .2))
    order = np.argsort(rel[selected], kind='stable')
    return raw[selected][order], xyz[selected][order], ranges[selected][order]


def main():
    adapter = import_adapter()
    cfg = yaml.safe_load(CAL.read_text())
    rear_r, rear_t = adapter.rear_transform(cfg)
    snapshot = json.loads((BAG/'snapshots/localization_runtime_20260917.json').read_text())
    ota = np.array(snapshot['logged_loaded_calibration']['rear_lidar_to_front_lidar'])
    np.testing.assert_allclose(rear_r, adapter.R_N_L@ota[:3, :3], atol=1e-12)
    np.testing.assert_allclose(rear_t, adapter.R_N_L@ota[:3, 3], atol=1e-12)
    db_path = BAG/'slam_raw_20260917_171716_fe8f38_0.db3'
    db = sqlite3.connect(db_path.as_uri()+'?mode=ro', uri=True)
    db.execute('PRAGMA query_only=ON')
    topics = dict(db.execute('SELECT name,id FROM topics'))
    report = {'bag': str(BAG), 'read_only_bag': True, 'ros_initialized': False,
              'whole_bag_scan': False, 'full_payloads_read': 4,
              'adapter_sha256': hashlib.sha256((HERE/'adapter.py').read_bytes()).hexdigest(),
              'calibration': str(CAL), 'rear_transform_matches_saved_OTA': True,
              'R_N_rear': rear_r.tolist(), 't_N_rear': rear_t.tolist(), 'pairs': []}
    samples = [(1772316441199985981, 1789636638723983637),
               (1772316485699981213, 1789636683206135780)]
    for wanted, received in samples:
        front, fm = read_cloud(db, topics['/front_lidar'], wanted, received)
        rear, rm = read_cloud(db, topics['/rear_lidar'], wanted, received)
        merged, start, detail = adapter.merge_clouds(front, rear, rear_r, rear_t)
        assert start == min(fm['header_ns'], rm['header_ns'])
        assert np.all(np.diff(merged['time']) >= 0)
        assert np.all(np.isfinite(np.column_stack([merged[k] for k in ('x', 'y', 'z', 'time', 'sensor_range')])))
        assert 0 <= merged['time'][0] < 1e-4 and .09 < merged['time'][-1] < .11
        banks = []
        key_sets = []
        for source, msg, offset, rotation, translation in [('front', front, 0, adapter.R_N_L, np.zeros(3)),
                                                          ('rear', rear, 96, rear_r, rear_t)]:
            out = merged[(merged['ring'] >= offset) & (merged['ring'] < offset+96)]
            raw, native_xyz, native_ranges = native_selected(msg, start)
            assert len(out) == len(raw) > 1000
            np.testing.assert_array_equal(out['ring'], raw['ring']+offset)
            points = np.column_stack([out[k] for k in ('x', 'y', 'z')]).astype(float)
            restored = (points-translation)@rotation
            error = np.linalg.norm(restored-native_xyz.astype(float), axis=1)
            assert error.max() < 1e-4
            np.testing.assert_array_equal(out['sensor_range'], native_ranges)
            # Independent scalar double-precision formula; report adjacent-column
            # differences possible at float32 angular boundaries separately.
            expected_col = []
            for x, y, _ in native_xyz:
                b = (math.degrees(math.atan2(float(x), float(y)))-90.)/.4
                rounded = math.copysign(math.floor(abs(b)+.5), b)
                expected_col.append(int(-rounded+450)%900)
            expected_col = np.asarray(expected_col)
            col_error = np.abs(out['column'].astype(int)-expected_col)
            col_error = np.minimum(col_error, 900-col_error)
            assert col_error.max() <= 1
            # These samples must agree exactly, not merely within a column.
            assert np.count_nonzero(col_error) == 0
            absolute = start*1e-9+out['time'].astype(float)
            time_error_ns = np.abs(absolute-raw['timestamp'])*1e9
            assert time_error_ns.max() <= 500
            keys = set((out['ring'].astype(int)*900+out['column']).tolist())
            key_sets.append(keys)
            banks.append({'sensor': source, 'points': len(out),
                          'ring_min': int(out['ring'].min()), 'ring_max': int(out['ring'].max()),
                          'column_min': int(out['column'].min()), 'column_max': int(out['column'].max()),
                          'column_mismatch_count': int(np.count_nonzero(col_error)),
                          'native_range_exact_float32_match': True,
                          'inverse_transform_max_error_m': float(error.max()),
                          'max_absolute_point_time_error_ns': float(time_error_ns.max()),
                          'native_range_vs_virtual_front_range_mean_abs_diff_m': float(np.mean(np.abs(native_ranges-np.linalg.norm(points,axis=1)))),
                          'occupied_range_image_cells': len(keys), 'same_sensor_cell_collisions': len(out)-len(keys)})
        assert not key_sets[0].intersection(key_sets[1])
        assert len(merged) == sum(x['points'] for x in banks)
        report['pairs'].append({'front': fm, 'rear': rm, 'start_ns': start,
                                'rear_minus_front_header_ns': rm['header_ns']-fm['header_ns'],
                                'merged_time_sorted': True, 'cross_sensor_cell_collisions': 0,
                                'detail': detail, 'banks': banks})
    db.close()
    report['result'] = 'PASS'
    report['limitations'] = ['Two paired scans only; not full-session timing or pairing certification.',
                              'Tests adapter numerical conventions, not physical calibration accuracy or SLAM performance.',
                              'Up to one float64-epoch ulp near scan zero is clamped by adapter; no fabricated sensor timing.',
                              'Range-image same-sensor collisions retained in this report; cross-sensor banks remain separate.']
    output = HERE/'audit_dual_sample.json'
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'result': report['result'], 'output': str(output), 'pairs': report['pairs']}, indent=2))


if __name__ == '__main__':
    main()
