"""Render recorded offline acceptance evidence; never participates in control."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('report',type=Path);args=parser.parse_args()
    report=json.loads(args.report.read_text());session=json.loads((args.report.parent/'session.json').read_text())
    cfg=yaml.safe_load(Path(session['config']).read_text());grid=np.load(cfg['planning_grid'])
    free=grid['free'];origin=grid['origin'];resolution=float(grid['resolution'])
    path=np.asarray(report['global_path']);travel=np.asarray([s['pose'] for s in report['samples']])
    if path.ndim!=2 or len(path)<2 or not len(travel):raise ValueError('Report lacks route/travel evidence')
    fig,ax=plt.subplots(figsize=(8,10),layout='constrained');fig.set_facecolor('#101820');ax.set_facecolor('#101820')
    extent=[origin[0],origin[0]+free.shape[0]*resolution,origin[1],origin[1]+free.shape[1]*resolution]
    ax.imshow(free.T.astype(float),origin='lower',extent=extent,cmap='gray',vmin=-.3,vmax=2.,interpolation='nearest')
    ax.plot(path[:,0],path[:,1],color='#59dbea',lw=3,label='Native PCT global route')
    ax.plot(travel[:,0],travel[:,1],color='#ff9c52',lw=1.5,label='Actual simulated travel')
    ax.scatter(*travel[0,:2],color='#85edb3',marker='o',s=50,zorder=5,label='Start')
    goal=report['goal_xy'];ax.scatter(*goal,color='white',marker='*',s=100,zorder=5,label='Goal')
    all_xy=np.vstack((path[:,:2],travel[:,:2],goal));lo=all_xy.min(0)-1.5;hi=all_xy.max(0)+1.5
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='Map X (m)',ylabel='Map Y (m)')
    for side in ax.spines.values():side.set_color('#8399a8')
    ax.tick_params(colors='#cbd6e0');ax.xaxis.label.set_color('#cbd6e0');ax.yaxis.label.set_color('#cbd6e0')
    label='PASS' if report['passed'] else 'NOT PASSED'
    distance=report['last_status']['sim']['distance_m'];error=report['tests'].get('arrive',{}).get('error_m')
    title=f'PCT + SCAN | OFFLINE SOFTWARE LOOP | {label}\nTravel {distance:.2f} m'
    if error is not None:title+=f' | final XY error {error:.3f} m'
    ax.set_title(title,color='white',fontsize=12);legend=ax.legend(facecolor='#17232d',edgecolor='#435968',fontsize=9)
    for text in legend.get_texts():text.set_color('white')
    output=args.report.with_suffix('.png');fig.savefig(output,dpi=180,facecolor=fig.get_facecolor());print(output)


if __name__=='__main__':main()
