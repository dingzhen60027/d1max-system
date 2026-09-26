#!/usr/bin/env python3
"""Bounded read-only bag geometry audit; no ROS initialization or replay.

Requires the ROS Humble Python environment for message deserialization only.
All derived outputs remain beside this script; production extrinsics unchanged.
"""
import argparse
import json
import sqlite3
import struct
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def pc(points):
    obj = o3d.geometry.PointCloud()
    obj.points = o3d.utility.Vector3dVector(points)
    return obj


def read_cloud(db, topic_id, target_ns, target_header_ns=None):
    if target_header_ns is None:
        row = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? ORDER BY timestamp LIMIT 1', (topic_id, target_ns)).fetchone()
    else:
        candidates = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp LIMIT 10', (topic_id, target_ns-300_000_000, target_ns+300_000_000)).fetchall()
        def header_distance(row):
            sec,nsec = struct.unpack_from('<iI',row[1],4)
            return abs(sec*10**9+nsec-target_header_ns)
        row = min(candidates,key=header_distance)
    msg = deserialize_message(row[1], PointCloud2)
    fields = {f.name: f for f in msg.fields}
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                      'offsets': [fields[n].offset for n in ('x', 'y', 'z')], 'itemsize': msg.point_step})
    arr = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step)).reshape(-1)
    points = np.column_stack([arr[n] for n in ('x', 'y', 'z')]).astype(float)
    valid = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > .7) & (np.linalg.norm(points, axis=1) < 15.)
    return points[valid], {'record_ns': row[0], 'header_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                           'frame': msg.header.frame_id, 'raw_count': len(points), 'selected_count': int(valid.sum())}


def planes(points, max_planes=10):
    remain = pc(points).voxel_down_sample(.04)
    found = []
    for _ in range(max_planes):
        if len(remain.points) < 300:
            break
        model, idx = remain.segment_plane(distance_threshold=.018, ransac_n=3, num_iterations=1200)
        if len(idx) < 200:
            break
        selected = np.asarray(remain.points)[idx]
        center = np.mean(selected, axis=0)
        _, _, vh = np.linalg.svd(selected-center, full_matrices=False)
        n = vh[-1]
        if n[np.argmax(np.abs(n))] < 0:
            n = -n
        d = -float(n @ center)
        found.append({'normal': n.tolist(), 'd': d, 'center': center.tolist(),
                      'count': len(idx), 'rmse': float(np.sqrt(np.mean((selected@n+d)**2))),
                      'bounds': np.quantile(selected, [.05, .95], axis=0).tolist()})
        remain = remain.select_by_index(idx, invert=True)
    return found


def compare_planes(front, rear):
    matches = []
    for i, a in enumerate(front):
        for j, b in enumerate(rear):
            na, nb = np.array(a['normal']), np.array(b['normal'])
            s = 1 if na@nb >= 0 else -1
            angle = float(np.degrees(np.arccos(np.clip(s*na@nb, -1, 1))))
            if angle > 8 or abs(a['d']-s*b['d']) > .3:
                continue
            matches.append({'front_plane': i, 'rear_plane': j, 'normal_angle_deg': angle,
                            'd_difference_m': float(a['d']-s*b['d']),
                            'rear_center_to_front_plane_m': float(na@b['center']+a['d']),
                            'front_center_to_rear_plane_m': float(nb@a['center']+b['d'])})
    return matches


def icp_check(front, rear):
    target = pc(front).voxel_down_sample(.07)
    source = pc(rear).voxel_down_sample(.07)
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.25,max_nn=30))
    target.orient_normals_towards_camera_location(np.zeros(3))
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.25,max_nn=30))
    distances = cKDTree(np.asarray(target.points)).query(np.asarray(source.points),k=1)[0]
    reg = o3d.pipelines.registration.registration_icp(source,target,.20,np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
    info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(source,target,.15,reg.transformation)
    return {'diagnostic_only_not_calibration': True, 'max_correspondence_distance_m': .20,
            'fitness': reg.fitness, 'inlier_rmse_m': reg.inlier_rmse,
            'before_icp_rear_to_front_nn_distance_m': dict(zip(['min','p05','p50','p95'],np.quantile(distances,[0,.05,.5,.95]).tolist())),
            'before_icp_correspondences_under_20cm': int(np.sum(distances < .2)),
            'before_icp_correspondences_under_50cm': int(np.sum(distances < .5)),
            'rear_correction_T_in_normalized_front': reg.transformation.tolist(),
            'rotation_vector_deg': np.degrees(Rotation.from_matrix(reg.transformation[:3,:3].copy()).as_rotvec()).tolist(),
            'translation_m': reg.transformation[:3,3].tolist(),
            'point_to_point_information_eigenvalues': np.linalg.eigvalsh(info).tolist()}


def floor_roi(points, xlimit, yrange):
    selected = points[(np.abs(points[:,0]) < xlimit) & (points[:,1] > yrange[0]) & (points[:,1] < yrange[1]) & (points[:,2] > -.65) & (points[:,2] < -.30)]
    original_count = len(selected)
    for _ in range(5):
        center = np.mean(selected,axis=0)
        _,_,vh = np.linalg.svd(selected-center,full_matrices=False)
        n = vh[-1]
        if n[2] < 0:
            n = -n
        d = -float(n@center)
        selected = selected[np.abs(selected@n+d) < .015]
    return {'x_abs_max_m':xlimit,'y_range_m':yrange,'initial_points':original_count,'retained_points':len(selected),
            'normal':n.tolist(),'d':d,'rmse_m':float(np.sqrt(np.mean((selected@n+d)**2)))}


def floor_comparison(front,rear):
    result=[]
    for xlim,fy,ry in [(1.,[.7,2.5],[-2.8,-1.1]),(1.4,[.5,5.],[-3.2,-1.]),(.6,[.7,2.5],[-2.8,-1.1])]:
        f,r=floor_roi(front,xlim,fy),floor_roi(rear,xlim,ry)
        result.append({'front':f,'rear':r,'angle_deg':float(np.degrees(np.arccos(np.clip(np.dot(f['normal'],r['normal']),-1,1))))})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path('/home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38'))
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('dual_geometry_strict_pair_report.json'))
    parser.add_argument('--times', type=float, nargs='+', default=[.65,1.5,2.0])
    args = parser.parse_args()
    meta = yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start_ns = meta['starting_time']['nanoseconds_since_epoch']
    db_path = args.bag/meta['relative_file_paths'][0]
    db = sqlite3.connect(db_path.as_uri()+'?mode=ro',uri=True)
    topics = dict(db.execute('SELECT name,id FROM topics'))
    R_nf = Rotation.from_quat([-.499867275,.503186620,.497953310,.498977388]).as_matrix()
    R_fr = np.diag([1.,-1.,-1.])
    t_fr = np.array([0.,0.,-.7323])
    report = {'bag': str(args.bag), 'database': str(db_path), 'read_only': True,
              'transform_definition': 'p_N=R_NF p_F; p_N(rear)=R_NF (R_FR p_R+t_FR)',
              'R_NF': R_nf.tolist(), 'R_FR':R_fr.tolist(),'t_FR':t_fr.tolist(),
              'cautions': ['Planes are unlabelled RANSAC patches; candidate pairings require spatial-overlap review.',
                           'ICP is a diagnostic candidate, not an approved full extrinsic calibration.',
                           'Independent front/rear fields of view and weak directions can bias registration.',
                           'Initial 0-2 s was previously judged low motion, but no external stationary ground truth.'],
              'frames': []}
    for target_s in args.times:
        front, mf = read_cloud(db,topics['/front_lidar'],start_ns+int(target_s*1e9))
        rear, mr = read_cloud(db,topics['/rear_lidar'],mf['record_ns'],mf['header_ns'])
        if abs(mf['header_ns']-mr['header_ns']) > 5_000_000:
            raise ValueError('No rear cloud synchronized to front within 5 ms')
        f = front @ R_nf.T
        r = (rear @ R_fr.T+t_fr) @ R_nf.T
        fp,rp = planes(f),planes(r)
        result = {'requested_record_s': target_s, 'front':mf, 'rear':mr,
                  'front_minus_rear_header_ms': (mf['header_ns']-mr['header_ns'])*1e-6,
                  'front_planes':fp, 'rear_planes':rp,'plane_pairs':compare_planes(fp,rp),
                  'floor_roi_comparisons':floor_comparison(f,r),
                  'icp':icp_check(f,r)}
        report['frames'].append(result)
        print(json.dumps({'time':target_s,'pair_delta_ms':result['front_minus_rear_header_ms'],
                          'floor_roi_angles_deg':[v['angle_deg'] for v in result['floor_roi_comparisons']],
                          'icp':result['icp']},indent=2),flush=True)
    db.close()
    with args.output.open('x') as out:
        json.dump(report,out,indent=2)
    print(args.output)


if __name__ == '__main__':
    main()
