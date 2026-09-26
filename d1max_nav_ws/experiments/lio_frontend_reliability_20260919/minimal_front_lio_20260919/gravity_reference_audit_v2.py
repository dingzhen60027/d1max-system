#!/usr/bin/env python3
"""Extend the immutable four-run fixed-reference audit with a completed fifth run.

Only reads the named completed device/internal trial; never historical trial.
"""
import json
from pathlib import Path
import runpy

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

HERE=Path(__file__).resolve().parent
BASE=runpy.run_path(str(HERE/'gravity_reference_audit.py'))
EXTRA='internal_device_paired_front_180s_20260919'


def angle_deg(a,b):
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)),np.clip(a@b,-1,1))))


def updates(name):
    root=BASE['ROOT']/name
    state=np.genfromtxt(root/'frontend_state.csv',delimiter=',',names=True)
    cfg=yaml.safe_load((root/'config/calibration.yaml').read_text())
    q=np.column_stack([state[k] for k in ('qx','qy','qz','qw')]);rot=Rotation.from_quat(q)
    p=np.column_stack([state[k] for k in ('x','y','z')])+rot.apply(cfg['lio_extrinsic']['translation'])
    v=np.column_stack([state[k] for k in ('vx','vy','vz')])
    ba=np.column_stack([state[k] for k in ('bax','bay','baz')])
    g=np.column_stack([state[k] for k in ('gx','gy','gz')]);u=-g/np.linalg.norm(g,axis=1)[:,None]
    t=state['t']-state['t'][0]
    def event(i,j):
        return {'start_elapsed_s':float(t[i]),'end_elapsed_s':float(t[j]),'dt_s':float(t[j]-t[i]),
            'g_direction_change_deg':angle_deg(u[i],u[j]),
            'estimated_body_relative_rotation_angle_deg':float(np.degrees((rot[i].inv()*rot[j]).magnitude())),
            'estimated_front_position_displacement_m':float(np.linalg.norm(p[j]-p[i])),
            'estimated_velocity_norm_first_last_m_s':[float(np.linalg.norm(v[i])),float(np.linalg.norm(v[j]))],
            'estimated_accel_bias_norm_first_last_m_s2':[float(np.linalg.norm(ba[i])),float(np.linalg.norm(ba[j]))]}
    frame_events=[event(i,i+1) for i in range(len(t)-1)]
    one_second=[]
    for j in range(1,len(t)):
        i=int(np.searchsorted(t,t[j]-1.,side='right'))-1
        if i>=0:
            one_second.append(event(i,j))
    ordered=sorted(frame_events,key=lambda e:e['g_direction_change_deg'],reverse=True)
    ordered1=sorted(one_second,key=lambda e:e['g_direction_change_deg'],reverse=True)
    after10=[e for e in ordered if e['start_elapsed_s']>=10]
    after10_one=[e for e in ordered1 if e['start_elapsed_s']>=10]
    final30=int(np.searchsorted(t,t[-1]-30))
    final10=int(np.searchsorted(t,t[-1]-10))
    return {'definition':'Angles between estimated gravity states; no external true-turn or stationary labels.',
        'max_per_frame':ordered[0],'top5_per_frame':ordered[:5],
        'max_approximately_one_second':ordered1[0],'top5_approximately_one_second':ordered1[:5],
        'max_per_frame_after_first10s':after10[0],
        'max_approximately_one_second_after_first10s':after10_one[0],
        'one_second_window_policy':'For each endpoint choose latest earlier sample at or before t_end-1s; actual interval reported, not interpolated.',
        'final10s':event(final10,len(t)-1),
        'final30s':event(final30,len(t)-1),
        'entire_run_final_state':event(0,len(t)-1),
        'interpretation':'Coincidence with estimated attitude/motion changes is descriptive; all quantities are produced by the same estimator.'}


def build():
    report=BASE['build']()
    report['schema_version']=2
    report['extends_without_overwriting']='gravity_reference_audit.json (four completed central-IMU runs)'
    extra=BASE['audit_run'](EXTRA)
    extra.pop('comparison_header_sequence_s')
    report['runs'].append(extra)
    names=[r['name'] for r in report['runs']]
    ref=np.genfromtxt(BASE['ROOT']/names[0]/'frontend_state.csv',delimiter=',',names=True)['t']
    maxdelta={}
    for run in report['runs']:
        t=np.genfromtxt(Path(run['directory'])/'frontend_state.csv',delimiter=',',names=True)['t']
        maxdelta[run['name']]=float(np.max(np.abs(t-ref))) if len(t)==len(ref) else None
        run['gravity_update_events']=updates(run['name'])
    report['max_header_difference_from_baseline_s']=maxdelta
    report['state_headers_correspond_in_sequence_within_100us']=all(v is not None and v<1e-4 for v in maxdelta.values())
    report['limitations']=[x for x in report['limitations'] if x!='No results from running or incomplete front-internal-IMU trials were read.']
    report['limitations'] += [
        'Only completed internal_device_paired_front_180s_20260919 was appended. No internal_historical trial was read.',
        'A single terminal gravity state is sensitive: last-10-second mean direction yields materially different heights in several runs.',
        'Gravity update events are correlated with estimated attitude/position changes, not independently measured physical turns/stops.']
    return report


if __name__=='__main__':
    print(json.dumps(build(),ensure_ascii=False,indent=2))
