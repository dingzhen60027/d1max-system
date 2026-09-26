#!/usr/bin/env python3
"""Print auditable offline extrinsic-trial files; does not write or launch ROS.

Output is a JSON map of relative filenames to file contents. Applying those
files is an explicit caller operation; this generator never edits defaults.
"""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

WS = Path('/home/dndx/d1max_nav_ws')
BASE = WS / 'maps/runs/20260919_125722_855277_central_imu_faster_lio_sc_pgo_zenoh/config/calibration.yaml'
FIT = WS / 'experiments/central_imu_fasterlio_20260918/calibration/gyro_rotation_candidate.json'
SNAPDIR = WS / 'bags/slam_raw_20260917_171716_fe8f38/snapshots'
DEVICE = SNAPDIR / 'extrinsics_20260917.json'
RUNTIME = SNAPDIR / 'localization_runtime_20260917.json'


def matrix_checks(R):
    R = np.asarray(R, dtype=float)
    return {'determinant':float(np.linalg.det(R)),
            'orthogonality_frobenius':float(np.linalg.norm(R.T @ R-np.eye(3)))}


def validate_rotation(R, name):
    R = np.asarray(R,dtype=float)
    if R.shape != (3,3) or not np.isfinite(R).all():
        raise ValueError(name + ': invalid matrix')
    check=matrix_checks(R)
    if abs(check['determinant']-1) > 1e-8 or check['orthogonality_frobenius'] > 1e-8:
        raise ValueError(name + ': not SO(3); refusing silent orthogonalization')
    return check


def build():
    baseline = yaml.safe_load(BASE.read_text())
    fit = json.loads(FIT.read_text())
    device = json.loads(DEVICE.read_text())
    runtime = json.loads(RUNTIME.read_text())
    front = next(d for d in device['factory_devices'] if d['sn_hex']=='3009bede1530')
    q = np.asarray(front['quaternion_xyzw'],float)
    qnorm = float(np.linalg.norm(q))
    if abs(qnorm-1) > 1e-6:
        raise ValueError('Device quaternion outside float32 unit-norm tolerance')
    # Explicit float32 quaternion normalization is NOT an invalid matrix repair.
    # Norm difference is preserved in the manifest (approximately 2e-9).
    R_F_L = Rotation.from_quat(q/qnorm).as_matrix()
    R_C_F = np.asarray(fit['R_c_from_f_raw'])
    R_N_L = Rotation.from_quat(device['current_project']['point_rotation_xyzw']).as_matrix()
    R_C_N_base = np.asarray(baseline['lio_extrinsic']['rotation']).reshape(3,3)
    R_C_N_device = R_C_F @ R_F_L @ R_N_L.T
    T_L_R = np.asarray(runtime['logged_loaded_calibration']['rear_lidar_to_front_lidar'],float)
    if not np.allclose(T_L_R[3],[0,0,0,1],atol=1e-12,rtol=0):
        raise ValueError('Runtime rear transform has invalid last row')
    R_rear_base = Rotation.from_quat(device['current_project']['rear_to_front_rotation_xyzw']).as_matrix()
    t_rear_base = device['current_project']['rear_to_front_translation_m']
    checks={name:validate_rotation(R,name) for name,R in {
        'baseline_lio_R':R_C_N_base,'device_front_q_R':R_F_L,
        'gyro_fit_R_C_F':R_C_F,'device_composed_lio_R':R_C_N_device,
        'runtime_rear_R':T_L_R[:3,:3]}.items()}
    rejected_R = np.asarray(runtime['logged_loaded_calibration']['front_lidar_to_front_imu'])[:3,:3]
    rejected = matrix_checks(rejected_R)
    if rejected['orthogonality_frobenius'] < 1e-3:
        raise ValueError('Recorded invalid OTA source changed; reassess rather than silently proceed')
    files={}
    trials=[]
    groups=[
        ('baseline',R_C_N_base,R_rear_base,t_rear_base,
         'None: reproduces current central-IMU extrinsics; rear transform made explicit.'),
        ('device_front_rotation',R_C_N_device,R_rear_base,t_rear_base,
         'Only LiDAR-to-central rotation changes via this front unit DIFOP. Point normalization, central lever arm and rear transform stay fixed.'),
        ('device_front_plus_runtime_rear',R_C_N_device,T_L_R[:3,:3],T_L_R[:3,3],
         'Compared with device_front_rotation, only rear-to-front R/t changes to the robot startup-log-confirmed loaded OTA transform.'),
    ]
    for name,Rc,Rr,tr,change in groups:
        cfg=copy.deepcopy(baseline)
        cfg['lio_extrinsic']['rotation']=[float(x) for x in Rc.ravel()]
        cfg['lidar_extrinsics']={'rear_to_front':{
            'parent_frame':'rslidar_head','child_frame':'rslidar_tail',
            'rotation':[float(x) for x in Rr.ravel()],
            'translation':[float(x) for x in tr]}}
        cfg['trial']={
            'name':name,'classification':'isolated offline diagnostic, not approved production calibration',
            'changed_variable':change,
            'rotation_convention':'column vectors; p_target=R_target_source*p_source+t_target_source; flat rotations row-major',
            'point_normalization':'Retain the existing R_N_L and zero translation; do not apply DIFOP twice.',
            'central_translation_policy':'Keep baseline nominal t_C_N exactly; not claiming a complete central-IMU calibration.',
            'driver_basis_status':'Unverified for the exact recorded robot driver build; candidate test only.',
            'loop_closure':False,'scene_constraints':False,'middleware':'rmw_zenoh_cpp',
        }
        # Retain original provenance as immutable historical detail but add an
        # explicit current formula so consumers cannot mistake it for this R.
        cfg['provenance']['trial_rotation_method']=('Original recorded R_C_N' if name=='baseline' else
            'R_C_N = R_C_F_gyrofit * R_F_L_device_SN3009bede1530 * R_N_L_existing.T')
        cfg['provenance']['trial_manifest']='../manifest.json'
        filename='calibrations/'+name+'.yaml'
        files[filename]='# OFFLINE DIAGNOSTIC ONLY. Do not install as default calibration.\n'+yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False)
        trials.append({'name':name,'calibration':filename,'change':change,
                       'R_C_N':Rc.tolist(),'t_C_N_m':cfg['lio_extrinsic']['translation'],
                       'R_front_from_rear':Rr.tolist(),'t_front_from_rear_m':[float(x) for x in tr]})
    angle=lambda a,b:float(np.degrees(Rotation.from_matrix(a@b.T).magnitude()))
    manifest={
        'schema_version':1,'status':'Offline controlled candidates; no deployment or successful result asserted',
        'purpose':'Test device-read calibration relations without scene priors or loop closure.',
        'minimal_order':[g[0] for g in groups],
        'preserved':['central IMU topic and raw axes','acceleration/gyro scales','timestamp_offset_sec=-0.013',
                     'duplicate handling','central nominal t_C_N','historical point normalization R_N_L',
                     'same bag and scan selection','Zenoh','no online extrinsic estimation'],
        'active_rear_source':'logged_loaded_calibration.rear_lidar_to_front_lidar, NOT exposed default ROS parameters',
        'front_device':{'sn_hex':front['sn_hex'],'src':front['src'],
                        'quaternion_xyzw_raw':front['quaternion_xyzw'],'quaternion_norm_raw':qnorm,
                        'quaternion_action':'explicit normalization only to remove float32 roundoff',
                        'original_device_translation_m_not_applied':front['translation_m'],
                        'translation_reason':'This is internal front LiDAR/IMU translation, not central IMU lever arm; keep central t fixed in this rotation-only trial.'},
        'SO3_checks':checks,
        'differences':{'front_rotation_change_deg':angle(R_C_N_device,R_C_N_base),
                       'rear_rotation_change_deg':angle(T_L_R[:3,:3],R_rear_base),
                       'rear_translation_change_norm_m':float(np.linalg.norm(T_L_R[:3,3]-t_rear_base))},
        'rejected_candidates':[{'name':'OTA front_lidar_to_front_imu','checks':rejected,
            'reason':'Not SO(3) as recorded. Not silently orthogonalized; not converted to a quaternion; not tested.'}],
        'unresolved_assumptions':[
            'DIFOP q interpreted as raw LiDAR-to-internal-IMU; exact bag-time driver transforms still unverified.',
            'Robot internal rear transform coordinate convention interpreted as raw rear-to-front named in startup log.',
            'A valid SO(3) matrix and evidence of loading do not establish physical accuracy.',
            'Gyro fit is IMU-to-IMU and nominal central origin remains unmeasured.',
        ],
        'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (BASE,FIT,DEVICE,RUNTIME)},
        'trials':trials,
    }
    files['manifest.json']=json.dumps(manifest,ensure_ascii=False,indent=2)+'\n'
    return files


if __name__=='__main__':
    print(json.dumps(build(),ensure_ascii=False))
