#!/usr/bin/env python3
"""Independent adjacent raw-LiDAR registration, bounded SQL reads, no ROS node.

Central gyro selects windows only. It never initializes, deskews, or otherwise
constrains registration. All registration transforms begin at identity.
"""
import argparse
import hashlib
import json
import sqlite3
import struct
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import yaml

ROOT=Path('/home/dndx/d1max_nav_ws')
BAG=ROOT/'bags/slam_raw_20260917_171716_fe8f38'
CACHE=ROOT/'experiments/central_imu_quality_20260918'
HERE=Path(__file__).resolve().parent
MAIN=[(.30,.75,30),(.15,.40,25),(.08,.20,20)]
ALTERNATIVE=[(.25,.65,30),(.12,.32,25),(.06,.16,20)]


def rotation_angle(T):
    return float(np.linalg.norm(Rotation.from_matrix(T[:3,:3]).as_rotvec())*180/np.pi)


def choose_windows(central):
    t=central['stamp_ns'].astype(float)*1e-9
    g=central['gyro']; bins=((t-t[0])*2).astype(int)
    candidates=[]
    for k in range(5,bins[-1]-5):
        v=g[bins==k]
        if len(v)>20:
            candidates.append((float(t[0]+(k+.5)/2),np.sqrt(np.mean(v*v,axis=0))))
    chosen=[]
    for axis in [0,1,2]:
        for centre,rms in sorted(candidates,key=lambda x:x[1][axis],reverse=True):
            if all(abs(centre-x['centre_sensor_sec'])>10 for x in chosen):
                chosen.append(dict(selection=f'central_axis_{axis}_rms',centre_sensor_sec=centre,
                                   centre_elapsed_sec=centre-float(t[0]),central_gyro_rms_xyz_rad_s=rms.tolist()))
                break
    centre,rms=max([x for x in candidates if x[0]-t[0]<180],key=lambda x:np.linalg.norm(x[1]))
    chosen.append(dict(selection='first_180s_max_gyro_rms',centre_sensor_sec=centre,
                       centre_elapsed_sec=centre-float(t[0]),central_gyro_rms_xyz_rad_s=rms.tolist()))
    return sorted(chosen,key=lambda x:x['centre_sensor_sec'])


def header_ns(row):
    sec,nsec=struct.unpack_from('<iI',row[2],4)
    return sec*10**9+nsec


def get_window_headers(db,topic,front,centre,count):
    i=int(np.argmin(abs(front['stamp_ns'].astype(float)*1e-9-centre)))
    record=int(front['record_ns'][i])
    rows=db.execute('SELECT id,timestamp,substr(data,1,12) FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp LIMIT 50',
                    (topic,record-1_200_000_000,record+1_200_000_000)).fetchall()
    rows=sorted(rows,key=header_ns)
    wanted=centre-count*.05
    start=int(np.argmin(abs(np.array([header_ns(x)*1e-9 for x in rows])-wanted)))
    selected=rows[start:start+count+1]
    assert len(selected)==count+1
    return selected,len(rows)


def load_cloud(db,row):
    blob=db.execute('SELECT data FROM messages WHERE id=?',(row[0],)).fetchone()[0]
    msg=deserialize_message(blob,PointCloud2)
    fs={f.name:f for f in msg.fields}
    assert not msg.is_bigendian and fs['timestamp'].datatype==8
    dtype=np.dtype(dict(names=['x','y','z','timestamp'],formats=['<f4','<f4','<f4','<f8'],
                        offsets=[fs[k].offset for k in ['x','y','z','timestamp']],itemsize=msg.point_step))
    values=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                      strides=(msg.row_step,msg.point_step)).reshape(-1)
    xyz=np.column_stack([values[k] for k in ['x','y','z']]).astype(float)
    radius=np.linalg.norm(xyz,axis=1)
    mask=np.isfinite(xyz).all(axis=1)&(radius>=.75)&(radius<=30)
    pcd=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[mask]))
    valid_t=values['timestamp'][mask];point_min=float(np.nanmin(values['timestamp']));point_max=float(np.nanmax(values['timestamp']))
    return pcd,dict(message_id=row[0],record_ns=row[1],header_ns=header_ns(row),frame_id=msg.header.frame_id,
                    raw_points=len(xyz),range_selected_points=int(mask.sum()),point_time_min_sec=point_min,
                    point_time_max_sec=point_max,scan_midpoint_estimate_sec=(point_min+point_max)*.5,
                    selected_point_time_mean_sec=float(np.mean(valid_t)),selected_point_time_median_sec=float(np.median(valid_t)))


def pyramids(pcd,scales):
    result=[]
    for voxel,_,_ in scales:
        down=pcd.voxel_down_sample(voxel)
        down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(.25,voxel*3),max_nn=30))
        result.append(down)
    return result


def register(source,target,scales):
    T=np.eye(4);history=[]
    for s,t,(_,distance,iterations) in zip(source,target,scales):
        result=o3d.pipelines.registration.registration_icp(
            s,t,distance,T,o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,relative_rmse=1e-6,max_iteration=iterations))
        T=np.array(result.transformation,copy=True)
        history.append(dict(fitness=float(result.fitness),euclidean_inlier_rmse_m=float(result.inlier_rmse),
                            source_points=len(s.points),target_points=len(t.points)))
    return T,history


def information_spectrum(source,target,T,threshold=.2):
    transformed=np.asarray(source.points)@T[:3,:3].T+T[:3,3]
    points=np.asarray(target.points);normals=np.asarray(target.normals)
    distance,index=cKDTree(points).query(transformed)
    ok=distance<threshold
    p=transformed[ok];n=normals[index[ok]]
    # Actual point-to-plane local Jacobian (left perturbation). Rotation and
    # translation have different units. Report both raw and scale-normalized
    # matrices, plus rotation information with translation marginalized.
    J=np.column_stack([np.cross(p,n),n]);H=J.T@J/max(len(J),1)
    scale=float(np.sqrt(np.mean(np.sum(p*p,axis=1))))
    Jn=np.column_stack([np.cross(p/scale,n),n]);Hn=Jn.T@Jn/max(len(Jn),1)
    rotation_schur=H[:3,:3]-H[:3,3:]@np.linalg.pinv(H[3:,3:])@H[3:,:3]
    eig=np.linalg.eigvalsh(Hn);rot=np.linalg.eigvalsh(rotation_schur)
    point_plane=np.sum((p-points[index[ok]])*n,axis=1)
    return dict(correspondences=int(ok.sum()),point_to_plane_rmse_m=float(np.sqrt(np.mean(point_plane**2))),
                rms_geometry_radius_m=scale,raw_information_eigenvalues=np.linalg.eigvalsh(H).tolist(),
                dimensionless_information_eigenvalues=eig.tolist(),dimensionless_min_max_ratio=float(eig[0]/eig[-1]),
                rotation_schur_eigenvalues=rot.tolist(),rotation_schur_min_max_ratio=float(rot[0]/rot[-1]),
                note='Local fixed-correspondence geometry proxy; not a calibrated covariance or proof of correct registration.')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--pairs-per-window',type=int,default=9)
    parser.add_argument('--limit',type=int,default=36);parser.add_argument('--output',type=Path,default=HERE/'lidar_rotations.json')
    args=parser.parse_args();assert 1<=args.pairs_per_window<=9 and 1<=args.limit<=36
    if args.output.exists():raise FileExistsError('Refusing overwrite: '+str(args.output))
    started=time.monotonic()
    central=np.load(CACHE/'central.npz');front=np.load(CACHE/'front.npz')
    windows=choose_windows(central)
    metadata=yaml.safe_load((BAG/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    report=dict(bag=str(BAG),topic='/front_lidar',frame='raw rslidar_head',registration_initial_guess='identity for every direction and voxel variant',
                no_lio_or_imu_prior=True,no_imu_deskew=True,central_gyro_usage='Window selection only, never registration input',
                transform_convention='p_prev = R_prev_from_next * p_next + t_prev_from_next. Source=next raw scan, target=previous raw scan; this is next LiDAR pose expressed in previous LiDAR axes.',
                physical_time_warning='Raw scans span about 100 ms and are not deskewed. Header=start of scan, midpoint is only an approximate effective pose time. Geometry visibility and motion distortions can shift effective time; do not claim precise physical delay from these observations alone.',
                filtering=dict(min_range_m=.75,max_range_m=30),main_multiscale=MAIN,alternate_multiscale=ALTERNATIVE,
                software_versions=dict(open3d=o3d.__version__,numpy=np.__version__),
                acceptance_thresholds=dict(min_forward_reverse_fitness=.55,max_euclidean_rmse_m=.10,max_forward_reverse_rotation_deg=.20,
                     max_forward_reverse_translation_m=.03,max_voxel_rotation_difference_deg=.20,max_voxel_translation_difference_m=.03,
                     min_dimensionless_information_ratio=1e-7,min_rotation_schur_ratio=.002,min_motion_rotation_deg=.08),
                windows=windows,pairs=[],full_cloud_payload_reads=0,read_only=True,ros_initialized=False)
    for window_index,window in enumerate(windows):
        sensor_i=int(np.argmin(abs(front['stamp_ns'].astype(float)*1e-9-window['centre_sensor_sec'])))
        record_ns=int(front['record_ns'][sensor_i])
        candidates=[entry for entry in metadata['files'] if
                    entry['starting_time']['nanoseconds_since_epoch']<=record_ns<=
                    entry['starting_time']['nanoseconds_since_epoch']+entry['duration']['nanoseconds']]
        assert len(candidates)==1
        db_path=BAG/candidates[0]['path']
        db=sqlite3.connect(db_path.as_uri()+'?mode=ro',uri=True)
        topic=db.execute('SELECT id FROM topics WHERE name=?',('/front_lidar',)).fetchone()[0]
        window['sqlite_file']=str(db_path)
        rows,headers_examined=get_window_headers(db,topic,front,window['centre_sensor_sec'],args.pairs_per_window)
        window['small_headers_examined']=headers_examined
        clouds=[]
        for row in rows:
            cloud,meta=load_cloud(db,row);report['full_cloud_payload_reads']+=1
            clouds.append((meta,pyramids(cloud,MAIN),pyramids(cloud,ALTERNATIVE)))
        db.close()
        for pair_index in range(len(clouds)-1):
            prev,pm,pa=clouds[pair_index];nxt,nm,na=clouds[pair_index+1]
            T,forward=register(nm,pm,MAIN);Tr,reverse=register(pm,nm,MAIN);Tv,alternate=register(na,pa,ALTERNATIVE)
            cycle=T@Tr;variant=np.linalg.inv(T)@Tv;info=information_spectrum(nm[-1],pm[-1],T)
            delta=(nxt['header_ns']-prev['header_ns'])*1e-9
            consistency=dict(forward_reverse_rotation_deg=rotation_angle(cycle),forward_reverse_translation_m=float(np.linalg.norm(cycle[:3,3])),
                             voxel_rotation_difference_deg=rotation_angle(variant),voxel_translation_difference_m=float(np.linalg.norm(variant[:3,3])))
            gates=dict(adjacent_header_interval=bool(.08<delta<.12),finite_transform=bool(np.isfinite(T).all()),
                       overlap=min(forward[-1]['fitness'],reverse[-1]['fitness'])>=.55,
                       euclidean_residual=max(forward[-1]['euclidean_inlier_rmse_m'],reverse[-1]['euclidean_inlier_rmse_m'])<=.10,
                       forward_reverse_rotation=consistency['forward_reverse_rotation_deg']<=.20,
                       forward_reverse_translation=consistency['forward_reverse_translation_m']<=.03,
                       voxel_rotation=consistency['voxel_rotation_difference_deg']<=.20,
                       voxel_translation=consistency['voxel_translation_difference_m']<=.03,
                       full_geometry_information=info['dimensionless_min_max_ratio']>=1e-7,
                       rotation_geometry_information=info['rotation_schur_min_max_ratio']>=.002,
                       informative_rotation=rotation_angle(T)>=.08)
            record=dict(window_index=window_index,pair_index=pair_index,previous=prev,next=nxt,header_delta_sec=delta,
                        midpoint_delta_sec=nxt['scan_midpoint_estimate_sec']-prev['scan_midpoint_estimate_sec'],
                        R_prev_from_next=T[:3,:3].tolist(),t_prev_from_next_m=T[:3,3].tolist(),T_prev_from_next=T.tolist(),
                        rotation_vector_prev_axes_rad=Rotation.from_matrix(T[:3,:3]).as_rotvec().tolist(),rotation_angle_deg=rotation_angle(T),
                        reverse_T_next_from_prev=Tr.tolist(),alternate_T_prev_from_next=Tv.tolist(),
                        forward_multiscale=forward,reverse_multiscale=reverse,alternate_multiscale=alternate,
                        consistency=consistency,information=info,gates=gates,accepted=all(gates.values()),
                        rejected_reasons=[key for key,value in gates.items() if not value])
            report['pairs'].append(record)
            print(json.dumps(dict(pair=len(report['pairs']),window=window_index,accepted=record['accepted'],
                  angle_deg=record['rotation_angle_deg'],cycle_deg=consistency['forward_reverse_rotation_deg'],
                  voxel_deg=consistency['voxel_rotation_difference_deg'],failed=record['rejected_reasons'],elapsed_sec=round(time.monotonic()-started,1))),flush=True)
            if len(report['pairs'])>=args.limit:break
        if len(report['pairs'])>=args.limit:break
    report['elapsed_sec']=time.monotonic()-started
    report['accepted_count']=sum(p['accepted'] for p in report['pairs'])
    report['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('WROTE '+str(args.output),flush=True)


if __name__=='__main__':main()
