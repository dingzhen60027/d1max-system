#!/usr/bin/env python3
"""Print minimal front-LiDAR/internal-IMU trial files; no writes or ROS calls."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

WS=Path('/home/dndx/d1max_nav_ws')
SNAP=WS/'bags/slam_raw_20260917_171716_fe8f38/snapshots/extrinsics_20260917.json'
CACHE=WS/'experiments/central_imu_quality_20260918/front.npz'
FIELDS=WS/'experiments/central_imu_quality_20260918/fields.json'


def validate_so3(R):
    R=np.asarray(R,float)
    det=float(np.linalg.det(R)); error=float(np.linalg.norm(R.T@R-np.eye(3)))
    assert abs(det-1)<1e-8 and error<1e-8
    return {'determinant':det,'orthogonality_frobenius':error}


def build():
    source=json.loads(SNAP.read_text())
    front=next(d for d in source['factory_devices'] if d['sn_hex']=='3009bede1530')
    q=np.asarray(front['quaternion_xyzw']); qnorm=float(np.linalg.norm(q))
    assert abs(qnorm-1)<1e-6
    R_F_L=Rotation.from_quat(q/qnorm).as_matrix()
    R_N_L=Rotation.from_quat(source['current_project']['point_rotation_xyzw']).as_matrix()
    A=np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
    R_F_N_device=R_F_L@R_N_L.T
    R_F_N_historical=A.T
    t_device=[float(v) for v in front['translation_m']]
    raw=np.load(CACHE,allow_pickle=False)
    elapsed=(raw['stamp_ns']-raw['stamp_ns'][0])*1e-9
    dt=np.diff(raw['stamp_ns'])*1e-6
    mask=elapsed<2; mask180=elapsed[1:]<=180
    a=raw['accel'];g=raw['gyro']
    sanity={
        'source_topic':'/front_lidar/imu','message_count':len(elapsed),
        'frame_id_counts':{str(k):int(v) for k,v in zip(*np.unique(raw['frame_id'],return_counts=True))},
        'first_stamp_ns':int(raw['stamp_ns'][0]),'last_stamp_ns':int(raw['stamp_ns'][-1]),
        'first_2s':{'count':int(mask.sum()),'acceleration_mean_raw':a[mask].mean(0).tolist(),
            'acceleration_std_raw':a[mask].std(0).tolist(),'acceleration_norm_median_raw':float(np.median(np.linalg.norm(a[mask],axis=1))),
            'gyro_mean_raw':g[mask].mean(0).tolist(),'gyro_std_raw':g[mask].std(0).tolist()},
        'first180_gaps_gt50ms':int((dt[mask180]>50).sum()),'first180_maxgap_ms':float(dt[mask180].max()),
        'full_acceleration_norm_median_raw':float(np.median(np.linalg.norm(a,axis=1))),
        'full_gap_ms_min_median_p95_p99_max':np.percentile(dt,[0,50,95,99,100]).tolist(),
        'header_nonpositive_deltas':int((dt<=0).sum()),
        'nonfinite_accel_or_gyro':int((~np.isfinite(a).all(1)|~np.isfinite(g).all(1)).sum()),
        'orientation_unique':np.unique(raw['orientation'],axis=0).tolist(),
        'notes':['Raw acceleration near 1 is consistent with g scale, not 1 m/s² gravity.',
                 'Raw angular velocity retained at scale 1; no degree-to-radian conversion added.',
                 'First two seconds are a fixed low-motion sample, not externally proven absolute rest.',
                 'Header frame rslidar_head is shared with points, but physical IMU vector basis differs.',
                 'Identity orientation is unavailable, not a measured robot attitude.',
                 'Front internal IMU has considerably more long gaps than central; do not silently loosen gap protection.']}
    specs=[
        ('historical_rotation_device_lever',R_F_N_historical,t_device,True,
         'Historical implied LiDAR-to-front-IMU rotation; device lever arm fixed equal to device_front_native, isolating rotation.'),
        ('device_front_native',R_F_N_device,t_device,True,
         'Device front LiDAR-to-front-internal-IMU R/t composed with unchanged normalized cloud basis.'),
        ('historical_exact_zero_lever',R_F_N_historical,[0.,0.,0.],False,
         'Reference only: equivalent physical historical chain of raw IMU rotated by A, LIO identity and zero lever; not a default extra replay.'),
    ]
    files={}; trials=[]
    for name,R,t,recommended,description in specs:
        cfg={
            'input_topic':'/front_lidar/imu','output_topic':'/d1max/slam/imu','frame_id':'front_imu_raw',
            'acceleration_scale':9.80665,'gyro_scale':1.0,'timestamp_offset_sec':0.0,
            'duplicate_policy':'preserve_and_count',
            'lio_extrinsic':{'rotation':[float(v) for v in R.ravel()],'translation':t},
            'lidar_extrinsics':{'rear_to_front':{'parent_frame':'rslidar_head','child_frame':'rslidar_tail',
                'rotation':[1.,0.,0.,0.,-1.,0.,0.,0.,-1.], 'translation':[0.,0.,-0.7323]}},
            'trial':{'name':name,'imu_source':'front_internal','cloud_selection':'front',
                'recommended_minimal_pair':recommended,'description':description,
                'raw_vectors_not_rotated':True,'central_imu_chain_removed':True,
                'point_normalization':'Keep current R_N_L and zero cloud translation. Do not apply DIFOP again in point adapter.',
                'loop_closure':False,'scene_constraints':False,'middleware':'rmw_zenoh_cpp',
                'classification':'Offline diagnostic only; not production approved'},
            'provenance':{'device_sn':'3009bede1530','device_record':str(SNAP),
                'rotation_formula':('R_F_N=R_F_L_device*R_N_L_current.T' if name=='device_front_native' else 'R_F_N=A.T; A=Rx(+90deg) historical front IMU normalization'),
                'translation_formula':('t_F_N=t_F_L_device because N and L share the front LiDAR origin' if name!='historical_exact_zero_lever' else 'Historical assumed zero LiDAR/internal-IMU lever arm'),
                'timing_policy':'No central-to-front 13ms correction; retain original front IMU header. Same hardware does not itself prove perfect LiDAR/IMU timing.',
                'units_policy':'Acceleration g-to-m/s² 9.80665 based on recorded magnitude and existing AIRY convention; gyro retained. Original SDK units not used as ROS topic proof.',
                'driver_basis_caveat':'Exact bag-time driver transformation not fully established; device q interpreted as raw front LiDAR to raw front internal IMU.',
                'manifest':'../manifest.json'},
        }
        filename='calibrations/'+name+'.yaml'
        files[filename]='# ISOLATED FRONT-ONLY EXPERIMENT. Do not install as default.\n'+yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False)
        trials.append({'name':name,'calibration':filename,'recommended_minimal_pair':recommended,
                       'rotation':R.tolist(),'translation_m':t,'SO3_check':validate_so3(R)})
    manifest={
        'schema_version':1,'purpose':'Remove central-IMU fit/lag/nominal-base chain; use only front cloud and its native raw-axis IMU.',
        'recommended_order':['historical_rotation_device_lever','device_front_native'],
        'optional_reference_only':'historical_exact_zero_lever',
        'controlled_difference_in_recommended_pair':'Only R_F_N changes; translation, sensor stream, timestamp and scales are identical.',
        'R_N_L_retained':R_N_L.tolist(),'R_F_L_device':R_F_L.tolist(),
        'device_quaternion_xyzw_raw':front['quaternion_xyzw'],'device_quaternion_raw_norm':qnorm,
        'device_q_normalization':'Explicit float32 norm correction only; no arbitrary matrix orthogonalization.',
        'rotation_pair_difference_deg':float(np.degrees(Rotation.from_matrix(R_F_N_device@R_F_N_historical.T).magnitude())),
        'device_lever_norm_m':float(np.linalg.norm(t_device)),
        'excluded':['central raw stream','two-IMU gyro fit','13ms central relative phase correction','nominal central/base lever arm','rear cloud points'],
        'retained_uncertainties':['raw point/IMU basis and driver transform conventions','device q physical direction convention',
                                  'LiDAR point-time vs front IMU sampling time','front IMU gaps/noise/accelerometer bias'],
        'runtime_requirements':['Explicit runner --imu-source front_internal','Explicit front-only cloud selector',
                                'Adapter must preserve raw axes; do not execute legacy A rotation as well as these R_F_N matrices.',
                                'LIO time_sync_en false; do not introduce central -13ms compensation.',
                                'Keep gap protection policy and record any rejected intervals or latched failure.',
                                'No online extrinsic estimation, loop closure or scene constraints. Zenoh unchanged.'],
        'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (SNAP,CACHE,FIELDS)},
        'trials':trials,
    }
    files['manifest.json']=json.dumps(manifest,ensure_ascii=False,indent=2)+'\n'
    files['front_imu_sanity.json']=json.dumps(sanity,ensure_ascii=False,indent=2)+'\n'
    return files


if __name__=='__main__':
    print(json.dumps(build(),ensure_ascii=False))
