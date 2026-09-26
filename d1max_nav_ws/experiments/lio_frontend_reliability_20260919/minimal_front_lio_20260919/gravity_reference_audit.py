#!/usr/bin/env python3
"""Read completed short runs and print fixed-gravity-reference diagnostics.

No map/trajectory rewriting, per-frame leveling, fitting to ground, ROS, or new
replay. Each run's estimated final gravity is a hypothesis, not ground truth.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path('/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/runs')
NAMES=('extrinsic_baseline_180s_20260919','extrinsic_device_front_180s_20260919',
       'extrinsic_device_front_rear_180s_20260919','central_device_paired_front_180s_20260919')


def trend(height, distance):
    design=np.column_stack([np.ones(len(distance)),distance])
    coef=np.linalg.lstsq(design,height,rcond=None)[0]
    residual=height-design@coef
    return {'endpoint_relative_height_m':float(height[-1]),'min_max_m':[float(height.min()),float(height.max())],
            'height_span_m':float(np.ptp(height)),'OLS_slope_per_3d_path_m':float(coef[1]),
            'OLS_intercept_m':float(coef[0]),'OLS_residual_rmse_m':float(np.sqrt(np.mean(residual**2))),
            'OLS_residual_span_m':float(np.ptp(residual))}


def audit_run(name):
    root=ROOT/name
    result=json.loads((root/'result.json').read_text())
    assert result['complete'] is True and not result['loop_closure_enabled'] and not result['height_lock_enabled']
    cfg=yaml.safe_load((root/'config/calibration.yaml').read_text())
    state=np.genfromtxt(root/'frontend_state.csv',delimiter=',',names=True)
    p=np.column_stack([state[k] for k in ('x','y','z')])
    q=np.column_stack([state[k] for k in ('qx','qy','qz','qw')])
    g=np.column_stack([state[k] for k in ('gx','gy','gz')])
    assert np.isfinite(p).all() and np.isfinite(q).all() and np.isfinite(g).all()
    assert np.max(np.abs(np.linalg.norm(q,axis=1)-1))<1e-6
    t_I_N=np.asarray(cfg['lio_extrinsic']['translation'],float)
    # N cloud origin coincides with the physical front LiDAR origin. Use IMU
    # state orientation only for this rigid lever-arm change of reporting point.
    p_lidar=p+Rotation.from_quat(q).apply(t_I_N)
    dp=p_lidar-p_lidar[0]
    distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p_lidar,axis=0),axis=1))]
    elapsed=state['t']-state['t'][0]
    u_first=-g[0]/np.linalg.norm(g[0]);u_last=-g[-1]/np.linalg.norm(g[-1])
    up_world=np.array([0.,0.,1.])
    heights={'world_Z':dp@up_world,'initial_gravity_fixed_up':dp@u_first,
             'final_gravity_fixed_up':dp@u_last}
    metrics={key:trend(h,distance) for key,h in heights.items()}
    segments=[]
    bounds=[0.,30.,60.,90.,120.,150.,float(elapsed[-1])]
    for low,high in zip(bounds[:-1],bounds[1:]):
        start=int(np.searchsorted(elapsed,low,'left'))
        end=min(int(np.searchsorted(elapsed,high,'right'))-1,len(state)-1)
        segments.append({'elapsed_interval_s':[float(elapsed[start]),float(elapsed[end])],
            'distance_3d_m':float(distance[end]-distance[start]),
            **{key+'_delta_height_m':float(h[end]-h[start]) for key,h in heights.items()}})
    # A secondary fixed axis tests whether a single terminal sample is stable;
    # still estimated by this same filter, not independent validation.
    last10=g[elapsed>=elapsed[-1]-10].mean(0)
    up10=-last10/np.linalg.norm(last10)
    dz=float(dp[-1,2]);h_final=float(heights['final_gravity_fixed_up'][-1])
    return {'name':name,'directory':str(root),'complete':True,'rows':len(state),
       'first_last_header_s':[float(state['t'][0]),float(state['t'][-1])],
       'comparison_header_sequence_s':state['t'].tolist(),
       'reported_point':'physical front LiDAR origin inferred from configured IMU-to-front lever arm',
       't_I_N_m':t_I_N.tolist(),'input_imu':cfg['input_topic'],'imu_state_delta_z_m':float(p[-1,2]-p[0,2]),
       'front_lidar_delta_xyz_m':dp[-1].tolist(),'front_lidar_path_3d_m':float(distance[-1]),
       'initial_up':u_first.tolist(),'final_up':u_last.tolist(),
       'final_gravity_tilt_from_initial_deg':float(np.degrees(np.arccos(np.clip(u_first@u_last,-1,1)))),
       'reference_metrics':metrics,'time_segments':segments,
       'reference_change_decomposition':{
          'final_axis_minus_world_Z_endpoint_m':h_final-dz,
          'horizontal_component_m':float(dp[-1,:2]@u_last[:2]),
          'vertical_component_m':float(dz*(u_last[2]-1)),
          'endpoint_abs_height_magnitude_reduction_fraction':float(1-abs(h_final)/abs(dz)),
          'interpretation':'Algebraic change under a fixed axis; not estimated causal error removed or accuracy improvement.'},
       'terminal_axis_sensitivity':{'fixed_axis_definition':'minus mean estimated gravity in last 10 seconds, normalized',
           'axis':up10.tolist(),'angle_to_final_sample_deg':float(np.degrees(np.arccos(np.clip(up10@u_last,-1,1)))),
           'endpoint_height_m':float(dp[-1]@up10)},
       'input_sha256':{str(path):hashlib.sha256(path.read_bytes()).hexdigest()
           for path in (root/'frontend_state.csv',root/'config/calibration.yaml',root/'result.json')}}


def build():
    runs=[audit_run(name) for name in NAMES]
    stamps=[np.asarray(run.pop('comparison_header_sequence_s')) for run in runs]
    max_differences=[float(np.max(np.abs(t-stamps[0]))) if len(t)==len(stamps[0]) else float('inf') for t in stamps]
    matched=all(delta<1e-4 for delta in max_differences)
    assert matched
    return {'schema_version':1,'status':'Read-only fixed-reference hypothesis check; not map repair or accuracy validation',
      'formulas':{'front_lidar_origin':'p_W_L(t)=p_W_I(t)+R_W_I(t)*t_I_N; N shares front LiDAR origin',
         'fixed_final_up':'u=-g_last/norm(g_last), one constant vector per run',
         'height':'h(t)=u dot (p_W_L(t)-p_W_L(0))'},
      'state_headers_correspond_in_sequence_within_100us':matched,
      'max_header_difference_from_baseline_s':dict(zip(NAMES,max_differences)),
      'ground_used':False,'per_frame_gravity_rotation_used':False,'map_or_state_modified':False,
      'runs':runs,
      'limitations':[
        'Final gravity belongs to the same imperfect estimator, not an external measured vertical reference.',
        'Each run has its own final axis; heights are not all measured against one externally shared true axis.',
        'A near-straight route permits a small reference tilt to explain substantial endpoint Z change; this does not isolate the cause.',
        'Remaining height can include real robot-body motion, inaccurate lever arm, calibration/model errors and drift.',
        'The configured central lever arm is nominal; converting reporting origin removes a comparison confound but does not certify this lever arm.',
        'Local map deformation, wall alignment, yaw/XY accuracy, or long-loop consistency cannot be judged from this projection alone.',
        'No results from running or incomplete front-internal-IMU trials were read.',
      ]}


if __name__=='__main__':
    print(json.dumps(build(),ensure_ascii=False,indent=2))
