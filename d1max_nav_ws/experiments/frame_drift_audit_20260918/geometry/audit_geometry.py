"""Read source poses/clouds, write derived diagnostic results only."""
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path('/home/dndx/d1max_nav_ws/maps/runs/20260917_190115_bag_faster_lio_sc_pgo_loop_fix_zenoh')
OUT=Path(__file__).resolve().parent
raw=np.loadtxt(ROOT/'sc_pgo_final/odom_poses.txt').reshape(-1,3,4)
opt=np.loadtxt(ROOT/'sc_pgo_final/optimized_poses.txt').reshape(-1,3,4)
t=np.loadtxt(ROOT/'sc_pgo_final/times.txt'); t=t-t[0]
tum=np.loadtxt(ROOT/'frontend_odometry.tum')
dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(raw[:,:3,3],axis=0),axis=1))]
report={'keyframes':len(raw),'duration_sec':float(t[-1]),'path_length_m':float(dist[-1]),'frontend_tum_count':len(tum),'method':'20 early-dense/then uniformly indexed keyframes; body points radius 1-8m and z -2.5..-0.25m; voxel .10m; iterative RANSAC rejects normals >20deg; >=200 inliers, then SVD; ground plane center under sensor mapped by saved poses. No physical level/height ground truth.'}

def plane_stats(p):
    A=np.c_[p[:,:2],np.ones(len(p))]
    coef=np.linalg.lstsq(A,p[:,2],rcond=None)[0]
    res=p[:,2]-A@coef
    return {'plane_z_ax_by_c':coef.tolist(),'tilt_deg':float(np.degrees(np.arctan(np.linalg.norm(coef[:2])))),'residual_rmse_m':float(np.sqrt(np.mean(res**2))),'residual_peak_to_peak_m':float(np.ptp(res)),'z_peak_to_peak_m':float(np.ptp(p[:,2])),'residuals_m':res.tolist()}

for name,mats in [('frontend',raw),('optimized',opt)]:
    p=mats[:,:,3]; eul=Rotation.from_matrix(mats[:,:,:3]).as_euler('xyz',degrees=True)
    report[name]={'endpoint_delta_xyz_m':(p[-1]-p[0]).tolist(),'xyz_min_m':p.min(axis=0).tolist(),'xyz_max_m':p.max(axis=0).tolist(),'trajectory_best_plane':plane_stats(p),'roll_deg_percentiles':np.percentile(eul[:,0],[0,5,50,95,100]).tolist(),'pitch_deg_percentiles':np.percentile(eul[:,1],[0,5,50,95,100]).tolist()}
    # Quantified height threshold onset; report positions, time and distances.
    report[name]['first_z_thresholds']={}
    for level in [.1,.25,.5,1,2,3,4]:
        hit=np.flatnonzero(np.abs(p[:,2]-p[0,2])>=level)
        if len(hit):
            i=int(hit[0]); report[name]['first_z_thresholds'][str(level)]={'keyframe':i,'time_sec':float(t[i]),'path_m':float(dist[i]),'xyz_m':p[i].tolist()}
    report[name]['time_blocks']=[]
    for start in range(0,1000,100):
        ix=np.flatnonzero((t>=start)&(t<start+100))
        if len(ix): report[name]['time_blocks'].append({'start_sec':start,'z_median_m':float(np.median(p[ix,2])),'z_range_m':[float(p[ix,2].min()),float(p[ix,2].max())],'roll_median_deg':float(np.median(eul[ix,0])),'pitch_median_deg':float(np.median(eul[ix,1])),'first_xyz_m':p[ix[0]].tolist()})

ids=np.unique(np.r_[0,5,10,20,30,40,np.linspace(70,len(raw)-1,14,dtype=int)])
floors=[]
for i in ids:
    cloud=o3d.io.read_point_cloud(str(ROOT/f'sc_pgo/Scans/{i:06d}.pcd'))
    xyz=np.asarray(cloud.points); radius=np.linalg.norm(xyz[:,:2],axis=1)
    mask=np.isfinite(xyz).all(axis=1)&(radius>1)&(radius<8)&(xyz[:,2]>-2.5)&(xyz[:,2]<-.25)
    part=cloud.select_by_index(np.flatnonzero(mask)).voxel_down_sample(.1)
    candidate_count=len(part.points); accepted=None
    for trial in range(6):
        if len(part.points)<200: break
        model,ix=part.segment_plane(.025,3,800)
        n=np.asarray(model[:3]); n=n if n[2]>0 else -n
        if n[2]>np.cos(np.deg2rad(20)) and len(ix)>200:
            points=np.asarray(part.points)[ix]
            mu=points.mean(axis=0)
            _,_,vh=np.linalg.svd(points-mu,full_matrices=False)
            n=vh[-1]; n=n if n[2]>0 else -n; d=-n@mu
            residual=(points-mu)@n
            xy_eig=np.linalg.eigvalsh(np.cov(points[:,:2].T))
            under=np.array([0.,0.,-d/n[2]])
            accepted={'keyframe':int(i),'time_sec':float(t[i]),'distance_m':float(dist[i]),'candidate_points':candidate_count,'inliers':len(ix),'inlier_fraction':len(ix)/candidate_count,'local_normal':n.tolist(),'sensor_to_floor_z_m':float(d/n[2]),'plane_rmse_m':float(np.sqrt(np.mean(residual**2))),'xy_std_principal_m':np.sqrt(xy_eig).tolist(),'local_plane_center':mu.tolist()}
            # Block jackknife by spatial quadrant gives a more honest stability
            # measure than treating thousands of adjacent lidar points as iid.
            normals=[]
            for a,b in [(1,1),(1,-1),(-1,1),(-1,-1)]:
                q=points[~(((points[:,0]-mu[0])*a>=0)&((points[:,1]-mu[1])*b>=0))]
                if len(q)>100:
                    qmu=q.mean(axis=0); nn=np.linalg.svd(q-qmu,full_matrices=False)[2][-1]; nn=nn if nn[2]>0 else -nn
                    normals.append(float(np.degrees(np.arccos(np.clip(nn@n,-1,1)))))
            accepted['quadrant_omission_max_normal_change_deg']=max(normals) if normals else None
            for name,mats in [('frontend',raw),('optimized',opt)]:
                world_n=mats[i,:,:3]@n; world_p=mats[i,:,:3]@under+mats[i,:,3]
                accepted[name+'_floor_tilt_deg']=float(np.degrees(np.arccos(np.clip(world_n[2],-1,1))))
                accepted[name+'_floor_normal']=world_n.tolist()
                accepted[name+'_floor_xyz_m']=world_p.tolist()
            break
        part=part.select_by_index(ix,invert=True)
    floors.append(accepted or {'keyframe':int(i),'error':'No qualifying floor candidate'})
    print('frame',int(i), 'floor',None if not accepted else round(accepted['sensor_to_floor_z_m'],3),'tilt',None if not accepted else round(accepted['optimized_floor_tilt_deg'],3),flush=True)
report['sampled_floors']=floors
valid=[f for f in floors if 'error' not in f]
for name in ['frontend','optimized']:
    p=np.array([f[name+'_floor_xyz_m'] for f in valid]); nn=np.array([f[name+'_floor_normal'] for f in valid])
    common=nn.mean(axis=0); common/=np.linalg.norm(common)
    deviations=np.degrees(np.arccos(np.clip(nn@common,-1,1)))
    ps=plane_stats(p)
    plane_normal=np.r_[-np.array(ps['plane_z_ax_by_c'][:2]),1.]; plane_normal/=np.linalg.norm(plane_normal)
    angular=np.degrees(np.arccos(np.clip(nn@plane_normal,-1,1)))
    report[name]['floor_geometry']={**ps,'normal_mean':common.tolist(),'normal_deviation_from_mean_deg':deviations.tolist(),'normal_deviation_from_best_plane_deg':angular.tolist(),'tilt_median_deg':float(np.median([f[name+'_floor_tilt_deg'] for f in valid])),'tilt_max_deg':float(max(f[name+'_floor_tilt_deg'] for f in valid))}

(OUT/'geometry_report.json').write_text(json.dumps(report,indent=2)+'\n')
fig,ax=plt.subplots(3,1,figsize=(11,10),sharex=True)
colors={'frontend':'#C76328','optimized':'#156A9C'}
for name,mats in [('frontend',raw),('optimized',opt)]:
    c=colors[name]; eul=Rotation.from_matrix(mats[:,:,:3]).as_euler('xyz',degrees=True)
    ax[0].plot(t,mats[:,2,3],color=c,label=name+' sensor trajectory')
    ft=[f['time_sec'] for f in valid]
    ax[0].plot(ft,[f[name+'_floor_xyz_m'][2] for f in valid],color=c,marker='o',ls='--',label=name+' sampled floor')
    ax[1].plot(ft,[f[name+'_floor_tilt_deg'] for f in valid],color=c,marker='o',label=name+' local floor normal tilt')
    ax[2].plot(ft,report[name]['floor_geometry']['residuals_m'],color=c,marker='o',label=name+' floor after best-plane removal')
for a in ax: a.grid(alpha=.25); a.legend(fontsize=8)
ax[0].set_ylabel('World Z (m)'); ax[1].set_ylabel('Floor tilt (degrees)'); ax[2].set_ylabel('Floor plane residual (m)'); ax[2].set_xlabel('Seconds from first saved keyframe')
fig.suptitle('Corridor geometry audit: frontend vs final PGO\nLocal floor planes sampled below sensor; no surveyed level ground truth')
fig.tight_layout(); fig.savefig(OUT/'trajectory_and_floor_audit.png',dpi=150)
print('REPORT',OUT/'geometry_report.json',flush=True)
