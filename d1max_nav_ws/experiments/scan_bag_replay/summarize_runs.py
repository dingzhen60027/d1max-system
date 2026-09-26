#!/usr/bin/env python3
"""Read completed same-input native diagnostics and render a side-by-side report."""
import argparse
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from capture_frontend import save_json, sha256
from replay_native import WS, prepare_data, spline_measurement


def boundary_audit(curves, body, clock_offset_ns):
    """Compare native initial p/v against a real preceding pose, not publication order.

    Native computation can finish after another odometry sample is received. The
    exact source pose identifies the solve boundary; assuming the latest sample
    at publication would incorrectly classify normal computation delay as a
    boundary change. No matching source pose is explicitly inconclusive.
    """
    from scipy.spatial.transform import Rotation
    stamps=np.array([item['stamp_ns'] for item in body],dtype=np.int64)
    positions=np.array([item['position'] for item in body])
    velocity=np.array([Rotation.from_quat(item['orientation']).apply(item['linear']) for item in body])
    matches=[];changed=[];unmatched=[]
    for curve in curves:
        _,measurement=spline_measurement(curve)
        original_start=curve['native_start_stamp_ns']-clock_offset_ns
        candidates=np.flatnonzero((stamps<=original_start)&(stamps>=original_start-1000000000))
        if not len(candidates):
            unmatched.append(curve['id']);continue
        errors=np.linalg.norm(positions[candidates]-measurement['start_position'],axis=1)
        candidate=candidates[np.argmin(errors)]
        if errors.min()>1e-7:
            unmatched.append(curve['id']);continue
        error=float(np.linalg.norm(velocity[candidate]-measurement['start_velocity_world']))
        item={'id':curve['id'],'source_stamp_ns':int(stamps[candidate]),
              'source_age_at_native_start_s':float((original_start-stamps[candidate])*1e-9),
              'start_velocity_error_mps':error,
              'real_velocity_world':velocity[candidate].tolist(),
              'native_velocity_world':measurement['start_velocity_world']}
        matches.append(item)
        if error>1e-7:changed.append(item)
    return {'matched_position_count':len(matches),'unmatched_curve_ids':unmatched,
            'changed_velocity_curves':changed,
            'max_velocity_error_mps':max((item['start_velocity_error_mps'] for item in matches),default=None),
            'max_boundary_source_age_s':max((item['source_age_at_native_start_s'] for item in matches),default=None),
            'scope':'Source identified by exact recorded position preceding native start within 1 s; not a controller continuity proof.'}


def log_failure_counts(path):
    data=path.read_text(errors='replace')
    patterns={
        'moving_boundary_uniform_retime_rejected':'moving initial boundary forbids uniform retiming',
        'input_motion_above_limit':'Initial motion exceeds strict planning limits',
        'constrained_timing_exhausted':'Constrained moving timing rejected: constrained_timing_exhausted',
        'interior_refinement_failed':'Constrained moving timing rejected: interior_refinement_failed',
    }
    return {key:len(re.findall(re.escape(pattern),data)) for key,pattern in patterns.items()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    if out.exists() or out.parent!=WS/'log/scan_bag_replay':
        raise ValueError('New unique log/scan_bag_replay directory required')
    reports=[json.loads((p/'report.json').read_text()) for p in args.runs]
    first=reports[0]
    for value in reports:
        if (not value['test_completed'] or value['owned_processes_surviving'] or not value['private_port_released']
                or value['input_hashes']!=first['input_hashes']
                or value['native_binary_sha256']!=first['native_binary_sha256']
                or value['seconds_per_profile']!=first['seconds_per_profile']):
            raise ValueError('Not clean, completed, same-input/same-binary differential runs')
    source={}
    for folder,report in zip(args.runs,reports):
        for name,values in report['profiles'].items():
            if name in source:raise ValueError('Duplicate profile')
            source[name]=(folder,values)
    samples,body=prepare_data(Path(first['capture']))
    reference=np.array([b['position'] for b in body])
    first_stamp=max(samples[0]['stamp_ns'],body[0]['stamp_ns'])
    measured=np.array([b['position'] for b in body if b['stamp_ns']<=first_stamp+first['seconds_per_profile']*1e9])
    cloud=np.concatenate([s['world_cloud'][::8] for s in samples])
    xmin,xmax=reference[:,0].min()-2.,reference[:,0].max()+2.
    ymin,ymax=reference[:,1].min()-1.5,reference[:,1].max()+1.5
    low,high=np.quantile(reference[:,2],[.01,.99])
    show=cloud[(cloud[:,2]>low-.7)&(cloud[:,2]<high+.8)&(cloud[:,0]>xmin)&(cloud[:,0]<xmax)&(cloud[:,1]>ymin)&(cloud[:,1]<ymax)]
    figure,axes=plt.subplots(1,len(source),figsize=(max(6.,4.0*len(source)),9.),squeeze=False)
    for ax,(name,(folder,values)) in zip(axes[0],source.items()):
        data=json.loads((folder/(name+'_outputs.json')).read_text())
        ax.set_facecolor('#161c25')
        ax.scatter(show[:,0],show[:,1],s=.5,color='#668496',alpha=.25,rasterized=True)
        ax.plot(reference[:,0],reference[:,1],color='#4b9dff',lw=2.0,label='recorded future route (oracle)')
        ax.plot(measured[:,0],measured[:,1],color='white',lw=.9,ls='--',label='35 s observed motion')
        for i,curve in enumerate(data['curves']):
            xyz,_=spline_measurement(curve)
            ax.plot(xyz[:,0],xyz[:,1],color='#3eff9e',lw=1.6,alpha=.9,label='accepted native trajectory' if i==0 else None)
        ax.scatter(reference[0,0],reference[0,1],s=30,color='white',edgecolor='#203040',zorder=8)
        ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax);ax.set_aspect('equal')
        dynamic=values['phase_seconds'].get('failed_dynamics',0.)
        accepted=values['phase_seconds'].get('accepted',0.)
        ax.set_title(f"{name}\n{values['tagged_count']} native curves\ndynamics blocked {dynamic:.1f}s / accepted state {accepted:.1f}s",fontsize=10)
        ax.set_xlabel('local odom X [m]');ax.set_ylabel('local odom Y [m]')
        ax.legend(loc='upper right',fontsize=6.7,framealpha=.95)
    values=next(iter(source.values()))[1]
    counts=f"{values['body_records']} odometry + {values['cloud_records']} cloud inputs per profile"
    heading=('Actual D1 rosbag / recorded-motion diagnostic\n'+counts+
             '\nFuture recorded route is an oracle, not PCT\nNot collision-free execution validation') if len(source)==1 else (
             'Actual D1 rosbag / identical '+counts+
             '\nOffline recorded-motion diagnostic: not PCT, not collision-free execution validation')
    figure.suptitle(heading,fontsize=10 if len(source)==1 else 12)
    figure.tight_layout(rect=(0,0,1,.94));out.mkdir(parents=True)
    figure.savefig(out/'actual_bag_comparison.png',dpi=180);plt.close(figure)
    rows={}
    for name,(folder,values) in source.items():
        outputs=json.loads((folder/(name+'_outputs.json')).read_text())
        rows[name]={'report':str(folder/'report.json'),'report_sha256':sha256(folder/'report.json'),**values,
                    'native_log_failure_counts':log_failure_counts(folder/(name+'.log')),
                    'initial_boundary_audit':boundary_audit(outputs['curves'],body,values['constant_clock_offset_ns'])
                        if 'constant_clock_offset_ns' in values else {'unavailable':'Earlier harness did not preserve constant offset.'}}
    save_json(out/'comparison.json',{'capture':first['capture'],'profiles':rows,
        'input_hashes':first['input_hashes'],'native_binary_sha256':first['native_binary_sha256'],
        'scope':'Repeated real cloud and recorded motion with future-route oracle. No PCT, no controller, no global localization proof.',
        'unknown_fraction_scope':'diagnostic waiting time only, not voxel unknown coverage'})
    print(str(out/'actual_bag_comparison.png'))


if __name__=='__main__':main()
