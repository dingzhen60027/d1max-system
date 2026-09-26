"""Offline, grid-validated elastic-band smoothing. Never sends robot commands."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d, distance_transform_edt


def segment_clear(a,b,free,origin,res):
    """Check every crossed cell and both sides of exact boundary/corner touches."""
    a=(np.asarray(a)-origin)/res;b=(np.asarray(b)-origin)/res;delta=b-a
    ts=[0.,1.]
    for k in range(2):
        if abs(delta[k])>1e-12:
            boundaries=np.arange(np.ceil(min(a[k],b[k])),np.floor(max(a[k],b[k]))+1)
            ts.extend(((boundaries-a[k])/delta[k]).tolist())
    ts=np.unique(np.clip(ts,0,1));samples=np.r_[ts,(ts[:-1]+ts[1:])/2]
    points=a+samples[:,None]*delta
    # Supercover: exact grid-line contacts must have support on both sides.
    for shift in ((1e-8,1e-8),(-1e-8,1e-8),(1e-8,-1e-8),(-1e-8,-1e-8)):
        cells=np.floor(points+shift).astype(int)
        if (cells<0).any() or (cells>=free.shape).any() or not free[tuple(cells.T)].all():return False
    return True


def valid_path(p,free,origin,res):
    return all(segment_clear(a,b,free,origin,res) for a,b in zip(p[:-1],p[1:]))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,required=True);args=parser.parse_args()
    cfg=yaml.safe_load(args.config.read_text());src=Path(cfg['input']);out=Path(cfg['output'])
    if out.exists():raise ValueError('Use a new output folder; prior paths are preserved')
    raw=np.loadtxt(src/'path.csv',delimiter=',',skiprows=1);grid=np.load(src/'planning_grid.npz')
    free=grid['free'];origin=grid['origin'];res=float(grid['resolution']);z=grid['height']
    xy=raw[:,:2];assert valid_path(xy,free,origin,res)
    # Longest visible shortcut, without cutting corners or crossing unknown cells.
    simplified=[xy[0]];i=0
    while i<len(xy)-1:
        j=len(xy)-1
        while j>i+1 and not segment_clear(xy[i],xy[j],free,origin,res):j-=1
        simplified.append(xy[j]);i=j
    points=[]
    for a,b in zip(simplified[:-1],simplified[1:]):
        n=max(1,int(np.ceil(np.linalg.norm(b-a)/cfg['control_spacing_m'])))
        points.extend(np.linspace(a,b,n,endpoint=False))
    points=np.asarray(points+[simplified[-1]])
    # Each update preserves both adjacent straight segments in the safe grid.
    for iteration in range(cfg['iterations']):
        maximum=0.
        order=range(1,len(points)-1) if iteration%2==0 else range(len(points)-2,0,-1)
        for i in order:
            target=(points[i-1]+points[i+1])/2
            for blend in (.5,.25,.125):
                candidate=points[i]+blend*(target-points[i])
                if segment_clear(points[i-1],candidate,free,origin,res) and segment_clear(candidate,points[i+1],free,origin,res):
                    maximum=max(maximum,float(np.linalg.norm(candidate-points[i])));points[i]=candidate;break
        if maximum<cfg['convergence_m']:break
    # Produce sampled cubic curve only if its exported segments also pass supercover.
    for refinement in range(5):
        t=np.r_[0,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
        sample=np.linspace(0,t[-1],int(np.ceil(t[-1]/cfg['sample_spacing_m']))+1)
        spline=CubicSpline(t,points,axis=0,bc_type='natural');smooth=spline(sample)
        if valid_path(smooth,free,origin,res):break
        points=np.vstack([v for pair in zip(points[:-1],(points[:-1]+points[1:])/2) for v in pair]+[points[-1]])
    else:raise ValueError('Cubic path failed collision checks; original remains active')
    cells=np.floor((smooth-origin)/res).astype(int);measured_z=z[tuple(cells.T)]
    # Gentle Z noise filtering, checked against measured cell height; no map edit.
    smooth_z=gaussian_filter1d(measured_z,cfg['z_filter_sigma_samples'])
    smooth_z[0],smooth_z[-1]=raw[0,2],raw[-1,2]
    deviation=float(np.max(abs(smooth_z-measured_z)))
    if deviation>cfg['max_ground_height_deviation_m']:raise ValueError('Smoothed height no longer follows measured surface')
    path=np.column_stack((smooth,smooth_z))
    assert np.allclose(path[[0,-1]],raw[[0,-1]],atol=1e-7)
    clearance=distance_transform_edt(np.pad(grid['support']&~grid['obstacles'],1))[1:-1,1:-1]*res
    d=spline(sample,1);dd=spline(sample,2);curvature=abs(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/np.maximum(np.linalg.norm(d,axis=1)**3,1e-12)
    report={'success':True,'algorithm':'Collision-checked shortcuts + constrained elastic band + sampled cubic spline',
        'path_points':len(path),'length_m':float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),
        'original_length_m':float(np.linalg.norm(np.diff(raw,axis=0),axis=1).sum()),'shortcut_vertices':len(simplified),
        'max_curvature_per_m':float(curvature.max()),'max_ground_height_deviation_m':deviation,
        'minimum_grid_center_clearance_m':float(clearance[tuple(cells.T)].min()),
        'every_exported_segment_supercover_checked':True,'start_xyz':path[0].tolist(),'goal_xyz':path[-1].tolist(),
        'robot_commands_sent':False,'limitations':['Spatial path only; no timed velocity/acceleration profile or real robot clearance validation.',
        'Collision checks apply to exported sampled polyline and existing static grid, not an arbitrary downstream spline reconstruction.']}
    out.mkdir(parents=True);(out/'planner.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    np.savetxt(out/'path.csv',path,delimiter=',',header='x,y,z',comments='');path.astype('<f4').tofile(out/'planned_path.bin')
    (out/'path_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
