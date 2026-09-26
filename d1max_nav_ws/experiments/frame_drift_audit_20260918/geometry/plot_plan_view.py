"""Plot positions of diagnosed vertical variation; read-only source access."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
OUT=Path(__file__).resolve().parent
ROOT=Path('/home/dndx/d1max_nav_ws/maps/runs/20260917_190115_bag_faster_lio_sc_pgo_loop_fix_zenoh')
d=json.loads((OUT/'geometry_report.json').read_text())
m=np.loadtxt(ROOT/'sc_pgo_final/optimized_poses.txt').reshape(-1,3,4)
p=m[:,:,3]
fig,axes=plt.subplots(1,2,figsize=(13,5.8),sharex=True,sharey=True)
segs=np.stack([p[:-1,:2],p[1:,:2]],axis=1)
lc=LineCollection(segs,cmap='viridis',norm=plt.Normalize(p[:,2].min(),p[:,2].max()),linewidth=4)
lc.set_array((p[:-1,2]+p[1:,2])/2); axes[0].add_collection(lc)
fig.colorbar(lc,ax=axes[0],shrink=.75,label='Sensor trajectory Z (m)')
floors=d['sampled_floors']; f=np.array([x['optimized_floor_xyz_m'] for x in floors]); res=np.array(d['optimized']['floor_geometry']['residuals_m'])
axes[1].plot(p[:,0],p[:,1],c='0.7',lw=1.5)
sc=axes[1].scatter(f[:,0],f[:,1],c=res,cmap='coolwarm',vmin=-.45,vmax=.45,s=65,zorder=3,edgecolor='white',linewidth=.5)
fig.colorbar(sc,ax=axes[1],shrink=.75,label='Floor residual after best plane removal (m)')
for a in axes:
    a.set_aspect('equal'); a.set_xlim(-74,23); a.set_ylim(-12,50); a.grid(alpha=.25); a.set_xlabel('World X (m)'); a.set_ylabel('World Y (m)')
    a.annotate('Start',p[0,:2],xytext=(7,9),textcoords='offset points',fontsize=9)
    for i in [165,343,468]:
        a.annotate(f'#{i}',p[i,:2],xytext=(5,7),textcoords='offset points',fontsize=8)
axes[0].set_title('Final PGO trajectory: Z span 1.309 m')
axes[1].set_title('20 local floor fits: remaining peak-to-peak 0.835 m')
fig.suptitle('Height variation is position-dependent after final PGO')
fig.tight_layout(); fig.savefig(OUT/'optimized_plan_view.png',dpi=160)
