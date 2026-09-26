#!/usr/bin/env python3
"""Cached-only fixed-rotation front/central comparison; no ROS, no calibration writes."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
sys.dont_write_bytecode = True

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

DIR = Path(__file__).resolve().parent
FOCUS = DIR.parent
EXPERIMENTS = FOCUS.parents[1]
ROOT = EXPERIMENTS.parent
SPEC = importlib.util.spec_from_file_location('central_fit', FOCUS/'fit_lidar_central_rotation.py')
FIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FIT)


def fixed_profile(x, ys, lags, mask, rotation):
    records=[]
    for lag,y in zip(lags,ys):
        _,intercept,fit_detail=FIT.fit_huber(x[mask],y[mask],fixed_rotation=rotation)
        residual=FIT.helpers.residual_summary(x[mask],y[mask],rotation,intercept)
        records.append({'lag_ms':int(lag),'huber_cost':FIT.robust_score(x[mask],y[mask],rotation,intercept),
                        'vector_rmse_rad_s':residual['vector_rmse'],
                        'median_rad_s':residual['norm_median'],'p95_rad_s':residual['norm_p95'],
                        'intercept_rad_s':intercept.tolist(),'fit_detail':fit_detail})
    best=min(records,key=lambda z:z['huber_cost'])
    return {'best':best,'profile':records,
            'best_at_search_boundary':abs(best['lag_ms'])==80,
            'rotation_fixed':rotation.tolist()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=DIR/'front_control_v2.json')
    args=parser.parse_args()
    paths={'motion':FOCUS/'lidar_motion/lidar_rotations.json',
           'manifest':EXPERIMENTS/'lio_frontend_reliability_20260919/extrinsic_trials_20260919/manifest.json',
           'gyro_fit':EXPERIMENTS/'central_imu_fasterlio_20260918/calibration/gyro_rotation_candidate.json',
           'historical_config':EXPERIMENTS/'central_imu_fasterlio_20260918/calibration.yaml',
           'front_cache':EXPERIMENTS/'central_imu_quality_20260918/front.npz',
           'central_cache':EXPERIMENTS/'central_imu_quality_20260918/central.npz',
           'reused_helpers':FOCUS/'fit_lidar_central_rotation.py'}
    motion=json.loads(paths['motion'].read_text())
    pairs=[p for p in motion['pairs'] if p['accepted']]
    manifest=json.loads(paths['manifest'].read_text())
    assert manifest['front_device']['sn_hex']=='3009bede1530'
    rcf=np.array(json.loads(paths['gyro_fit'].read_text())['R_c_from_f_raw'])
    rnl=Rotation.from_quat([-.499867275,.503186620,.497953310,.498977388]).as_matrix()
    historical_rcl=np.array(yaml.safe_load(paths['historical_config'].read_text())['lio_extrinsic']['rotation']).reshape(3,3)@rnl
    front_rotations={'device':Rotation.from_quat(manifest['front_device']['quaternion_xyzw_raw']).as_matrix(),
                     'historical':rcf.T@historical_rcl}
    central_rotations={name:rcf@r for name,r in front_rotations.items()}
    arrays={}
    with np.load(paths['central_cache'],allow_pickle=False) as data:
        origin=int(data['stamp_ns'][0])
    for sensor in ('front','central'):
        with np.load(paths[sensor+'_cache'],allow_pickle=False) as data:
            arrays[sensor]=FIT.prepare((data['stamp_ns']-origin)*1e-9,data['gyro'])
    def midpoint(scan):
        return (int(scan['header_ns'])-origin)*1e-9+(scan['point_time_max_sec']-scan['point_time_min_sec'])*.5
    intervals=np.array([[midpoint(p['previous']),midpoint(p['next'])] for p in pairs])
    dt=intervals[:,1]-intervals[:,0]
    groups=np.array([p['window_index'] for p in pairs])
    lags=np.arange(-80,81,2)
    output={'status':'diagnostic fixed-rotation control only; no calibrated delay or extrinsic claim',
            'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths.values()},
            'selected_pairs':[{'window':int(g),'pair':p['pair_index'],'interval_s':interval.tolist()} for p,g,interval in zip(pairs,groups,intervals)],
            'source_time_origin_ns':origin,'lag_search_ms':[-80,80],'lag_step_ms':2,
            'lag_sign':'mean gyro_sensor(t+lag)=R_sensor_from_L*omega_L(t)+constant intercept. Both sensor profiles use original raw header stamps; no -13 ms shift pre-applied.',
            'front_device_sn':'3009bede1530','R_C_F':rcf.tolist(),
            'objective':'All fixed-rotation intercepts minimize the same radial Huber delta=.02 rad/s using root fit_huber IRLS; lag selected by that same objective.',
            'fixed_R_F_L':{k:v.tolist() for k,v in front_rotations.items()},
            'fixed_R_C_L':{k:v.tolist() for k,v in central_rotations.items()},'cases':[],
            'limitations':['LiDAR ICP on undeskewed raw scans is not truth; midpoint only approximates effective rotation epoch.',
                          'All 81 lags and both sensors share each threshold support mask; 20 ms and 35 ms supports may differ.',
                          'Only constant gyro intercept is fit; no free rotation, scale, accelerometer or translation fit.',
                          'Profile selected and evaluated on the same small sample set; results are descriptive and not held-out accuracy.',
                          'Adjacent LiDAR pairs are correlated and window selection originally used central gyro.',
                          'Similar residuals cannot establish LiDAR distortion as the cause; they only weaken a central-only residual explanation.',
                          'Conditional device quaternion interpretation and historical IMU-to-IMU rotation provenance remain unvalidated physically.']}
    for gap in (.020,.035):
        ys={}
        for sensor,(t,v) in arrays.items():
            # Restrict each interval to enough support for all lags before calling unchanged helpers.
            support=[]
            for a,b in intervals:
                lo=max(0,np.searchsorted(t,a-.081,side='right')-1)
                hi=min(len(t),np.searchsorted(t,b+.081,side='left')+1)
                support.append((t[lo:hi],v[lo:hi]))
            ys[sensor]=np.array([[FIT.interval_mean(ti,vi,a+lag*.001,b+lag*.001,gap)
                                 for (a,b),(ti,vi) in zip(intervals,support)] for lag in lags])
        common=np.isfinite(ys['front']).all(axis=(0,2))&np.isfinite(ys['central']).all(axis=(0,2))
        for variant in ('primary','alternate','reverse'):
            item={'max_source_gap_ms':int(gap*1000),'variant':variant,'common_pair_count':int(common.sum()),
                  'common_mask_both_sensors_all_lags':common.tolist(),
                  'common_pairs_per_window':{str(int(g)):int(np.sum(common&(groups==g))) for g in np.unique(groups)}}
            if common.sum()<6:
                item['status']='insufficient common support';output['cases'].append(item);continue
            x=Rotation.from_matrix(FIT.rotations_for(pairs,variant)).as_rotvec()/dt[:,None]
            item['excitation']=FIT.helpers.excitation(x[common])
            item['models']={}
            for sensor,rotations in [('front',front_rotations),('central',central_rotations)]:
                for name,rotation in rotations.items():
                    item['models'][sensor+'_'+name]=fixed_profile(x,ys[sensor],lags,common,rotation)
            item['central_to_front_comparison']={}
            for name in front_rotations:
                cb=item['models']['central_'+name]['best'];fb=item['models']['front_'+name]['best']
                item['central_to_front_comparison'][name]={'rmse_ratio':cb['vector_rmse_rad_s']/fb['vector_rmse_rad_s'],
                    'best_lag_difference_ms':cb['lag_ms']-fb['lag_ms'],
                    'central_rmse_rad_s':cb['vector_rmse_rad_s'],'front_rmse_rad_s':fb['vector_rmse_rad_s']}
            output['cases'].append(item)
    with args.output.open('x') as file:
        json.dump(output,file,indent=2,allow_nan=False);file.write('\n')
    print(json.dumps({'output':str(args.output),'cases':[
        {k:v for k,v in x.items() if k not in ('models','excitation','common_mask_both_sensors_all_lags')}
        | {'best_models':{name:model['best'] for name,model in x.get('models',{}).items()}} for x in output['cases']]},indent=2))


if __name__=='__main__':main()
