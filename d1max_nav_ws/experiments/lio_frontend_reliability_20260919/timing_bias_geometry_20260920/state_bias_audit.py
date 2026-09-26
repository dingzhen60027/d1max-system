#!/usr/bin/env python3
"""Bounded read-only audit of three completed LIO states and cached raw IMUs.

Does not initialize ROS, read complete bags, estimate a new calibration, alter
states/configuration, or use estimated speed as a stationary truth label.
Prints JSON only. Sensor-origin/motion ground truth is unavailable.
"""
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

WS=Path('/home/dndx/d1max_nav_ws')
ROOT=WS/'experiments/lio_frontend_reliability_20260919/runs'
CACHE=WS/'experiments/central_imu_quality_20260918'
NAMES=('central_device_paired_front_180s_20260919',
       'internal_device_paired_front_180s_20260919',
       'internal_historical_paired_front_180s_20260919')
G_SCALE=9.81
G_S2=98090/10000


def summary(x):
    a=np.asarray(x,float)
    return {'count':len(a),'mean':np.mean(a,axis=0).tolist(),
            'median':np.median(a,axis=0).tolist(),'p95':np.percentile(a,95,axis=0).tolist(),
            'max':np.max(a,axis=0).tolist()}


def vector_angle(a,b):
    a=a/np.linalg.norm(a);b=b/np.linalg.norm(b)
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)),np.clip(a@b,-1,1))))


def window_stats(t,a,w,gyro_bias):
    d=np.diff(t)
    return {'count':len(t),'duration_s':float(t[-1]-t[0]),
            'maximum_header_gap_ms':float(d.max()*1000) if len(d) else 0.,
            'accel_mean_m_s2':a.mean(0).tolist(),'accel_std_vector_norm_m_s2':float(np.linalg.norm(a.std(0))),
            'gyro_mean_rad_s':w.mean(0).tolist(),'gyro_std_vector_norm_rad_s':float(np.linalg.norm(w.std(0))),
            'gyro_mean_norm_rad_s':float(np.linalg.norm(w.mean(0))),
            'gyro_mean_minus_initial_bias_norm_rad_s':float(np.linalg.norm(w.mean(0)-gyro_bias))}


def audit(name):
    root=ROOT/name
    result=json.loads((root/'result.json').read_text());assert result['complete']
    assert not result['loop_closure_enabled'] and not result['height_lock_enabled']
    cfg=yaml.safe_load((root/'config/calibration.yaml').read_text())
    raw_name='central' if cfg['input_topic']=='/imu_driver/imu_central' else 'front'
    raw=np.load(CACHE/(raw_name+'.npz'),allow_pickle=False)
    t_raw=(raw['stamp_ns']-raw['stamp_ns'][0])*1e-9
    # Bound all arithmetic to first 181 s; cached arrays do not require bag I/O.
    bound=t_raw<=181.;raw_stamp=raw['stamp_ns'][bound];raw_t=t_raw[bound]
    a_raw=raw['accel'][bound];w_raw=raw['gyro'][bound]
    accel_scale=float(cfg['acceleration_scale']);gyro_scale=float(cfg['gyro_scale'])
    a=a_raw*accel_scale;w=w_raw*gyro_scale
    log=(root/'frontend.log').read_text()
    match=re.search(r'Stationary initialization: (\d+) samples, gravity-aligned world=1, mean acc=(.*), gyro bias=(.*)',log)
    assert match
    n_init=int(match[1]);mean_log=np.fromstring(match[2],sep=' ');bg_log=np.fromstring(match[3],sep=' ')
    # Only identify the recorded initial window among first 100 possible starts;
    # not selecting a quieter window or fitting a new bias from desired output.
    matches=[]
    for start in range(100):
        ma=a[start:start+n_init].mean(0);mw=w[start:start+n_init].mean(0)
        discrepancy=np.linalg.norm(ma-mean_log)+np.linalg.norm(mw-bg_log)
        matches.append((float(discrepancy),start,ma,mw))
    matches.sort(key=lambda item:item[0]);score,start,mean,bias=matches[0]
    # Log prints a limited number of significant digits; do not claim exact
    # receiver-history reconstruction from a matching rounded log.
    assert np.max(np.abs(mean-mean_log))<6e-6 and np.max(np.abs(bias-bg_log))<5e-9
    init=window_stats(raw_t[start:start+n_init],a[start:start+n_init],w[start:start+n_init],bias)
    init.update({'log_line':match[0],'cache_start_index':start,'cache_start_elapsed_s':float(raw_t[start]),
       'cache_end_elapsed_s':float(raw_t[start+n_init-1]),'log_mean_accel_m_s2':mean_log.tolist(),
       'log_gyro_bias_rad_s':bg_log.tolist(),'candidate_mean_accel_m_s2':mean.tolist(),
       'candidate_gyro_bias_rad_s':bias.tolist(),'best_mean_match_score':score,'second_match_score':matches[1][0],
       'agreement':'Matches rounded runtime initialization means; not a proof of every algorithm-received sample.'})
    lio_scale=G_SCALE/np.linalg.norm(mean)
    # Mimic gate moments only; independent nonoverlapping windows, not exact
    # ready() event replication. A low-variance IMU cannot prove zero velocity.
    windows=[]
    for low in np.arange(0.,178.1,2.):
        i=int(np.searchsorted(raw_t,low));j=int(np.searchsorted(raw_t,low+2.))+1
        if j>len(raw_t):break
        stat=window_stats(raw_t[i:j],a[i:j],w[i:j],bias)
        stat.update({'start_elapsed_s':float(raw_t[i]),'end_elapsed_s':float(raw_t[j-1])})
        stat['startup_gate_moment_conditions']=bool(stat['count']>=300 and stat['duration_s']>=2
          and stat['maximum_header_gap_ms']<=50 and 8<np.linalg.norm(a[i:j].mean(0))<12
          and stat['accel_std_vector_norm_m_s2']<=.2 and stat['gyro_std_vector_norm_rad_s']<=.015
          and stat['gyro_mean_norm_rad_s']<=.05)
        stat['additional_low_angular_motion_indicator']=bool(stat['startup_gate_moment_conditions']
          and stat['gyro_mean_minus_initial_bias_norm_rad_s']<=.005)
        windows.append(stat)
    s=np.genfromtxt(root/'frontend_state.csv',delimiter=',',names=True)
    t=s['t']-s['t'][0]
    q=np.column_stack([s[k] for k in ('qx','qy','qz','qw')]);rot=Rotation.from_quat(q);R=rot.as_matrix()
    g=np.column_stack([s[k] for k in ('gx','gy','gz')]);ba=np.column_stack([s[k] for k in ('bax','bay','baz')]);bg=np.column_stack([s[k] for k in ('bgx','bgy','bgz')])
    predicted_static_force=ba-np.einsum('nji,nj->ni',R,g)
    # Diagnostic instantaneous interpolation only across short source gaps;
    # this is NOT reconstruction of endpoint-averaged ESKF integration input.
    source_absolute=raw_stamp*1e-9+float(cfg['timestamp_offset_sec'])
    idx=np.searchsorted(source_absolute,s['t'])
    valid=(idx>0)&(idx<len(source_absolute));idx=np.clip(idx,1,len(source_absolute)-1)
    source_gaps=source_absolute[idx]-source_absolute[idx-1];valid &= source_gaps<=.020
    interpolated=np.column_stack([np.interp(s['t'],source_absolute,a[:,k]*lio_scale) for k in range(3)])
    implied_a=np.einsum('nij,nj->ni',R,interpolated-ba)+g
    def event(low,high):
        i=int(np.argmin(np.abs(t-low)));j=int(np.argmin(np.abs(t-high)))
        db=ba[j]-ba[i]
        dg=-R[j].T@(g[j]-g[i])
        dr=-(R[j].T-R[i].T)@g[i]
        total=db+dg+dr
        assert np.linalg.norm(total-(predicted_static_force[j]-predicted_static_force[i]))<1e-10
        denom=np.linalg.norm(db)+np.linalg.norm(dg)+np.linalg.norm(dr)
        near=lambda k:np.mean(a[(source_absolute>=s['t'][k]-.05)&(source_absolute<=s['t'][k]+.05)]*lio_scale,axis=0)
        measured_change=near(j)-near(i)
        def cosine(x,y):
            norm=np.linalg.norm(x)*np.linalg.norm(y)
            return float(x@y/norm) if norm>1e-12 else None
        return {'requested_elapsed_s':[low,high],'actual_elapsed_s':[float(t[i]),float(t[j])],
           'g_direction_change_deg':vector_angle(g[i],g[j]),
           'body_relative_rotation_deg':float(np.degrees((rot[i].inv()*rot[j]).magnitude())),
           'ba_first_last_m_s2':[ba[i].tolist(),ba[j].tolist()],
           'bg_first_last_rad_s':[bg[i].tolist(),bg[j].tolist()],
           'bias_delta_norm_m_s2':float(np.linalg.norm(db)),'gyro_bias_delta_norm_rad_s':float(np.linalg.norm(bg[j]-bg[i])),
           'predicted_static_force_delta_terms_m_s2':{'ba':db.tolist(),'gravity_at_endpoint_orientation':dg.tolist(),
               'orientation_with_start_gravity':dr.tolist(),'sum':total.tolist()},
           'delta_term_norms_m_s2':{'ba':float(np.linalg.norm(db)),'g':float(np.linalg.norm(dg)),
               'orientation':float(np.linalg.norm(dr)),'sum':float(np.linalg.norm(total))},
           'cosine_ba_vs_gravity_term':cosine(db,dg),
           'algebraic_cancellation_fraction':float(1-np.linalg.norm(total)/denom) if denom else 0.,
           'measured_force_local_100ms_mean_change_m_s2':measured_change.tolist(),
           'boundary_force_change_minus_static_model_change_m_s2':(measured_change-total).tolist(),
           'caution':'Static-force model only: a real linear acceleration need not be zero. Terms are same-estimator states, not independent evidence of correct bias.'}
    dg=np.diff(g,axis=0);dba=np.diff(ba,axis=0);dtheta=(rot[:-1].inv()*rot[1:]).magnitude()
    changes={'g_delta_norm':np.linalg.norm(dg,axis=1),'ba_delta_norm':np.linalg.norm(dba,axis=1),'orientation_delta_rad':dtheta}
    pairs={}
    for lo in (0.,10.):
        keep=t[1:]>=lo
        pairs[str(lo)]={f'{a_name}__{b_name}':float(np.corrcoef(changes[a_name][keep],changes[b_name][keep])[0,1])
            for a_name,b_name in [('g_delta_norm','ba_delta_norm'),('g_delta_norm','orientation_delta_rad')]}
    source_files=[root/'frontend_state.csv',root/'frontend.log',root/'config/calibration.yaml',CACHE/(raw_name+'.npz')]
    return {'name':name,'input_topic':cfg['input_topic'],'frame_id':cfg['frame_id'],'rows':len(s),
       'initialization':init,'acceleration_normalization':{'adapter_scale':accel_scale,'lio_scale':float(lio_scale),
          'combined_raw_accel_scale':float(accel_scale*lio_scale),'mean_norm_before_lio_scale_m_s2':float(np.linalg.norm(mean)),
          'target_accel_norm_m_s2':G_SCALE,'state_gravity_norm_first_last_m_s2':[float(np.linalg.norm(g[0])),float(np.linalg.norm(g[-1]))]},
       'raw_low_dynamic_screen':{'window_seconds':2.,'step_seconds':2.,
          'first5_last5_window_details':windows[:5]+windows[-5:],
          'startup_gate_candidate_intervals_s':[[v['start_elapsed_s'],v['end_elapsed_s']] for v in windows if v['startup_gate_moment_conditions']],
          'additional_low_angular_motion_candidate_intervals_s':[[v['start_elapsed_s'],v['end_elapsed_s']] for v in windows if v['additional_low_angular_motion_indicator']],
          'all_window_accel_std_vector_norm_summary_m_s2':summary([v['accel_std_vector_norm_m_s2'] for v in windows]),
          'all_window_gyro_std_vector_norm_summary_rad_s':summary([v['gyro_std_vector_norm_rad_s'] for v in windows]),
          'count_total':len(windows),'count_startup_gate_moment_conditions':sum(w['startup_gate_moment_conditions'] for w in windows),
          'count_additional_low_angular_motion_indicator':sum(w['additional_low_angular_motion_indicator'] for w in windows),
          'not_rest_truth':'No MC/contact/independent velocity label; constant velocity, constant acceleration and correlated repeated samples can confound IMU-only screening.'},
       'state_bias_summary':{'ba_norm_m_s2':summary(np.linalg.norm(ba,axis=1)),'bg_change_from_initial_norm_rad_s':summary(np.linalg.norm(bg-bg[0],axis=1)),
                            'g_direction_change_max_deg':max(vector_angle(g[0],value) for value in g)},
       'joint_delta_correlations_descriptive_only':pairs,
       'joint_change_events':[event(0,1.5),event(.5,1.5),event(30,150),event(float(t[-1]-10),float(t[-1]))],
       'implied_acceleration_diagnostic':{'formula':'R_W_I*(f_interpolated_scaled-ba)+g',
          'valid_samples':int(valid.sum()),'excluded_due_to_source_gap_or_coverage':int((~valid).sum()),
          'norm_m_s2':summary(np.linalg.norm(implied_a[valid],axis=1)),
          'meaning':'This is implied physical acceleration/self-consistency only, not estimation error; no true acceleration supplied. Interpolated input is not exact ESKF integration replay.'},
       'input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}}


def full_cache_low_dynamic_screen(sensor):
    raw=np.load(CACHE/(sensor+'.npz'),allow_pickle=False)
    t=(raw['stamp_ns']-raw['stamp_ns'][0])*1e-9
    a=raw['accel']*9.80665;w=raw['gyro']
    start_bias=w[t<2.].mean(0)
    passed=[];total=0
    for low in np.arange(0.,t[-1]-2.,2.):
        i=int(np.searchsorted(t,low));j=int(np.searchsorted(t,low+2.))+1
        if j>len(t):break
        stat=window_stats(t[i:j],a[i:j],w[i:j],start_bias);total+=1
        if (stat['count']>=300 and stat['duration_s']>=2
          and stat['maximum_header_gap_ms']<=50 and 8<np.linalg.norm(a[i:j].mean(0))<12
          and stat['accel_std_vector_norm_m_s2']<=.2 and stat['gyro_std_vector_norm_rad_s']<=.015
          and stat['gyro_mean_norm_rad_s']<=.05):
            mean=a[i:j].mean(0)
            stat.update({'start_elapsed_s':float(t[i]),'end_elapsed_s':float(t[j-1]),
                         'mean_specific_force_unit':(mean/np.linalg.norm(mean)).tolist()})
            passed.append(stat)
    directions=np.asarray([v['mean_specific_force_unit'] for v in passed])
    pair_angles=np.degrees(np.arccos(np.clip(directions@directions.T,-1.,1.))) if len(passed) else np.empty((0,0))
    return {'sensor':sensor,'cache_path':str(CACHE/(sensor+'.npz')),'duration_s':float(t[-1]),
       'sampling':'All available cache time covered by nonoverlapping 2s windows; same moment/gap thresholds as short audit. Not every possible sliding-window alignment.',
       'total_windows':total,'passing_windows':len(passed),'windows':passed,
       'mean_specific_force_direction_pairwise_angles_deg':pair_angles.tolist(),
       'maximum_pairwise_direction_angle_deg':float(pair_angles.max()) if len(passed) else None,
       'angle_to_first_passing_direction_deg':pair_angles[0].tolist() if len(passed) else [],
       'interpretation':'IMU-only low-dynamic candidates, not independently labelled rest. Nearly identical force directions do not supply well-conditioned multi-attitude accelerometer calibration.',
       'source_sha256':hashlib.sha256((CACHE/(sensor+'.npz')).read_bytes()).hexdigest()}


def common_window_relative_gyro_shift():
    front=np.load(CACHE/'front.npz',allow_pickle=False)
    central=np.load(CACHE/'central.npz',allow_pickle=False)
    fit_path=WS/'experiments/central_imu_fasterlio_20260918/calibration/gyro_rotation_candidate.json'
    fit=json.loads(fit_path.read_text());R_C_F=np.asarray(fit['R_c_from_f_raw'])
    offset_ns=-13_000_000
    F=front['stamp_ns'];C=central['stamp_ns']+offset_ns
    windows=[]
    for low in (0.,982.):
        # Intersect the original passing window in each sensor, after applying
        # only the already-declared central-to-front timestamp convention.
        bounds=[]
        for raw,shift in ((front,0),(central,offset_ns)):
            relative=(raw['stamp_ns']-raw['stamp_ns'][0])*1e-9
            i=int(np.searchsorted(relative,low));j=int(np.searchsorted(relative,low+2.))
            bounds.append((int(raw['stamp_ns'][i])+shift,int(raw['stamp_ns'][j])+shift))
        start=max(v[0] for v in bounds);end=min(v[1] for v in bounds)
        mf=(F>=start)&(F<=end);mc=(C>=start)&(C<=end)
        mean_f=front['gyro'][mf].mean(0);mean_c=central['gyro'][mc].mean(0)
        delta=R_C_F@mean_f-mean_c
        windows.append({'named_window_start_s':low,
           'common_interval_front_reference_absolute_ns':[str(start),str(end)],
           'common_interval_relative_to_first_front_header_s':[(start-int(F[0]))*1e-9,(end-int(F[0]))*1e-9],
           'counts':{'front':int(mf.sum()),'central':int(mc.sum())},
           'maximum_source_gap_ms':{'front':float(np.diff(F[mf]).max()*1e-6),'central':float(np.diff(C[mc]).max()*1e-6)},
           'gyro_mean_front_raw_rad_s':mean_f.tolist(),'gyro_mean_central_raw_rad_s':mean_c.tolist(),
           'gyro_mean_front_rotated_to_C_rad_s':(R_C_F@mean_f).tolist(),
           'R_C_F_mean_front_minus_mean_central_rad_s':delta.tolist(),
           'within_window_gyro_std_vector_norm_rad_s':{'front':float(np.linalg.norm(front['gyro'][mf].std(0))),
                                                      'central':float(np.linalg.norm(central['gyro'][mc].std(0)))}})
    change=np.asarray(windows[1]['R_C_F_mean_front_minus_mean_central_rad_s'])-np.asarray(windows[0]['R_C_F_mean_front_minus_mean_central_rad_s'])
    return {'method':'Original sample means in common absolute-time intersection; no resampling, deduplication, gyro-bias subtraction or new calibration.',
       'R_C_F':R_C_F.tolist(),'rotation_source':str(fit_path),'rotation_source_sha256':hashlib.sha256(fit_path.read_bytes()).hexdigest(),
       'central_header_offset_ns':offset_ns,'windows':windows,
       'end_minus_start_relative_gyro_offset_change_C_rad_s':change.tolist(),
       'relative_change_norm_rad_s':float(np.linalg.norm(change)),
       'relative_change_norm_deg_s':float(np.degrees(np.linalg.norm(change))),
       'interpretation':'Under fixed rigid relative rotation and adequate time alignment, common true angular motion cancels to first order. The changed two-stream offset is a raw-data consistency concern, not a measured thermal drift or attribution to either sensor.',
       'limitations':['Rotation/time shift remain experimental prior estimates; residual gain, filtering, changing latency, mounting flex or axis errors may contribute.',
          'Irregular samples and unequal sample counts can weight changing motion differently; mean difference is not a synchronized pointwise hardware truth.',
          'No independence assumption or confidence interval claimed for correlated/repeated samples.',
          'No compensation or parameter update applied.']}


def build():
    return {'schema_version':1,'status':'bounded read-only state/IMU self-consistency audit; not a new calibration or SLAM run',
       'runs':[audit(name) for name in NAMES],
       'whole_cached_bag_low_dynamic_screen':[full_cache_low_dynamic_screen(name) for name in ('central','front')],
       'common_window_relative_gyro_shift':common_window_relative_gyro_shift(),
       'model':{'propagation':'a_world=R_world_imu*(specific_force_scaled-b_a)+g_world',
          'static_force_prediction':'f_static=b_a-R_world_imu.T*g_world',
          'finite_delta_identity':'Delta f_static=Delta b_a - R_j.T*Delta g -(R_j.T-R_i.T)*g_i',
          'identifiability':'One static orientation provides three acceleration equations; unknown b_a(3), attitude(3), gravity direction(2) are not uniquely separated. Even known attitude leaves five unknown b_a+g-direction components and at most three equations.'},
       'gravity_magnitude_consistency':{'accel_normalization_target_m_s2':G_SCALE,'S2_radius_m_s2':G_S2,
          'difference_m_s2':G_SCALE-G_S2,'relative_difference_ppm':1e6*(G_SCALE/G_S2-1),
          'ideal_stationary_initial_world_acceleration_m_s2':[0.,0.,G_SCALE-G_S2],
          'equivalent_0p8deg_horizontal_accel_m_s2':float(G_S2*np.sin(np.radians(.8))),
          'caution':'The 0.001 m/s² radial mismatch is real but does not itself produce the roughly 0.8-degree horizontal direction discrepancy. LiDAR updates prevent interpreting its open-loop double integral as observed error.'},
       'limitations':['No independent ground-truth velocity, acceleration, attitude, or calibrated central sensor origin.',
          'Correlations and finite-state decompositions describe coupled estimator states, not causal proof of sensor bias or successful compensation.',
          'Two-second sample variance cannot be used as an Allan-noise/bias-stability characterization.',
          'Source cache and rounded initialization log agree, but do not record per-message algorithm-receipt hashes.',
          'No source bag was scanned; no additional SLAM run, production edit, ground prior, or ROS process was used.']}


if __name__=='__main__':
    print(json.dumps(build(),ensure_ascii=False,indent=2,allow_nan=False))
