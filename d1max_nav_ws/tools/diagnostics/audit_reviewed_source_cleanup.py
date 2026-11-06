#!/usr/bin/env python3
"""Same-coordinate, file-only PCT and retained-record cleanup comparison."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'src/d1max_pct_planner'), str(ROOT/'src/d1max_pct_scan')]


def surfaces(directory):
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.tomogram_display import surface_clouds
    from d1max_pct_scan.source_identity import SourceIdentityBridge
    t = TomogramMap(directory/'tomogram.npz', unknown_ceiling_policy='reject', max_ground_step_m=.17)
    b = SourceIdentityBridge.from_artifacts(directory/'manifest.json')
    xyz, costs, red = surface_clouds(t, range(t.layers))
    low, high = b.input_z_band
    xyz, costs = xyz[(xyz[:,2]>=low)&(xyz[:,2]<=high)], costs[(xyz[:,2]>=low)&(xyz[:,2]<=high)]
    red = red[(red[:,2]>=low)&(red[:,2]<=high)]
    forbidden = lambda p: np.all((p[:,:2]>=b.forbidden[0])&(p[:,:2]<=b.forbidden[1]), axis=1)
    kept = ~forbidden(xyz)
    xyz, costs, red = xyz[kept], costs[kept], red[~forbidden(red)]
    supported_red = cKDTree(b.support).query(red, workers=1)[0] <= .12
    green_xy = {tuple(v) for v in np.round(xyz[:,:2],5)}
    overlap = np.array([tuple(v) in green_xy for v in np.round(red[:,:2],5)])
    stat = dict(green_xyz=len(xyz), red_xyz=len(red),
        green_unique_xy=len(green_xy), red_unique_xy=len(np.unique(np.round(red[:,:2],5),axis=0)),
        red_without_measured_floor_ownership=int((~supported_red).sum()),
        red_with_measured_floor_ownership=int(supported_red.sum()),
        red_overlapping_valid_xy=int(overlap.sum()),
        measured_floor_blocked_unique_xy=len(np.unique(np.round(red[supported_red,:2],5),axis=0)))
    return t, b, xyz, costs, red, supported_red, stat


def run(before, after, output):
    from d1max_pct_planner.cpu_tomography import SENTINEL, height_gradients, traversal_cost
    from tools.pointcloud_preprocessing.pcd_io import read_pcd
    from tools.pointcloud_preprocessing.runner import sha256
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output.mkdir(parents=True,exist_ok=True)
    first, second = surfaces(before), surfaces(after)
    t0,b0,g0,c0,r0,s0,stats0 = first
    t1,b1,g1,c1,r1,s1,stats1 = second
    if t0.shape != t1.shape or not np.array_equal(t0.center,t1.center) or t0.resolution != t1.resolution:
        raise ValueError('same-input coordinate grid changed; compare by world cells explicitly')
    raw=read_pcd(b1.manifest['source_path'])
    out=read_pcd(after/'processed_map.pcd')
    with np.load(after/'source_indices.npz',allow_pickle=False) as archive:
        selected=archive['selected_source_indices']
    with np.load(after/'audit/source_record_cleanup.npz',allow_pickle=False) as archive:
        removed=archive['removed_source_indices']
    unchanged=np.array_equal(raw.records[selected],out.records)
    if not unchanged or not np.array_equal(b0.support,b1.support):
        raise ValueError('original records or existing observed floor support changed')
    protected_above=(raw.xyz[:,2]>.1)&np.isin(np.arange(len(raw.xyz)),
        np.load(before/'source_indices.npz')['selected_source_indices'])
    high_removed=np.intersect1d(np.flatnonzero(protected_above),removed).size
    report=dict(source_sha256=sha256(b1.manifest['source_path']),
        before_tomogram_sha256=t0.sha256,after_tomogram_sha256=t1.sha256,
        same_grid=True,source_records_byte_identical=bool(unchanged),
        observed_floor_support_preserved=len(b1.support),
        removed_original_records=len(removed),removed_record_xyz_bounds=[raw.xyz[removed].min(axis=0).tolist(),raw.xyz[removed].max(axis=0).tolist()],
        above_absolute_z_010_removed=high_removed,before=stats0,after=stats1,
        newly_allowed_cells=int((t1.allowed&~t0.allowed).sum()),
        no_longer_allowed_cells=int((t0.allowed&~t1.allowed).sum()),
        original_xyz_unchanged=True,unknown_policy_unchanged=True,
        physical_acceptance=False,execution_authorized=False,
        limitation='Static PCD filtering uses a reviewed scene prior; it does not prove semantic human identity or live collision safety')
    lost=np.argwhere(t0.allowed&~t1.allowed)
    cfg=b1.manifest['configuration']['pct']['traversability']
    raw_geometry=[]
    for tomogram in (t0,t1):
        ground=np.nan_to_num(tomogram.ground,nan=-float(SENTINEL))
        ceiling=np.nan_to_num(tomogram.ceiling,nan=float(SENTINEL))
        raw_cost=traversal_cost(ground,ceiling,tomogram.resolution,cfg)
        _,maximum_step_sq=height_gradients(ground)
        raw_geometry.append((ground,raw_cost,maximum_step_sq))
    report['no_longer_allowed_cell_details']=[]
    for layer,ix,iy in lost:
        xy=t0.world([ix,iy])
        record=dict(layer=int(layer),xy=xy.tolist(),
            before_ground_z=float(t0.ground[layer,ix,iy]),after_ground_z=float(t1.ground[layer,ix,iy]),
            before_cost=float(t0.cost[layer,ix,iy]),after_cost=float(t1.cost[layer,ix,iy]),
            before_ceiling_z=None if not t0.ceiling_known[layer,ix,iy] else float(t0.ceiling[layer,ix,iy]),
            after_ceiling_z=None if not t1.ceiling_known[layer,ix,iy] else float(t1.ceiling[layer,ix,iy]),
            after_reason=t1._reason(int(layer),(int(ix),int(iy))))
        record['own_raw_cost_before_after']=[float(values[1][layer,ix,iy]) for values in raw_geometry]
        contributors=[]
        radius=int((float(cfg['safe_margin'])+float(cfg['inflation']))/t1.resolution)
        for dx in range(-radius,radius+1):
            for dy in range(-radius,radius+1):
                j,k=int(ix+dx),int(iy+dy)
                if not (0<=j<t1.nx and 0<=k<t1.ny):continue
                weight=float(np.clip(1-(t1.resolution*np.hypot(dx,dy)-float(cfg['inflation']))/
                    (float(cfg['safe_margin'])+t1.resolution),0,1))
                contribution=raw_geometry[1][1][layer,j,k]*weight
                if abs(contribution-record['after_cost'])>1e-4:continue
                contributors.append(dict(offset_xy=[dx,dy],weight=weight,
                    raw_cost_before_after=[float(values[1][layer,j,k]) for values in raw_geometry],
                    ground_z_before_after=[float(values[0][layer,j,k]) for values in raw_geometry],
                    maximum_axis_step_before_after_m=[float(np.sqrt(values[2][layer,j,k])) for values in raw_geometry],
                    configured_step_max_m=float(cfg['step_max'])))
        record['dominant_inflation_contributors']=contributors
        record['cause']='Neighbor ground maximum changed after record removal; exposed step exceeds the unchanged 0.17 m limit and its raw barrier is radially inflated into this unchanged support cell.'
        report['no_longer_allowed_cell_details'].append(record)
    historical_path=Path(b1.manifest['source_record_cleanup']['manifest_path'])
    historical=json.loads(historical_path.read_text())
    report['historical_classification_summary']={
        'manifest_sha256':sha256(historical_path),
        'all_floor_removed':historical['removed_points'],
        'source_floor_statements':historical['floor_statistics'],
        'actual_transferred_indices':len(removed),
        'human_semantics_confirmed':False,
        'pointwise_old_module_reason_cached':False,
        'limitation':'The prior stores the final union removal index, not per-record semantic or module labels; do not call all 42945 points people.'}
    fig,axes=plt.subplots(1,3,figsize=(16,7),sharex=True,sharey=True)
    for ax,title,green,cost,red in [(axes[0],'Before: all slice surfaces',g0,c0,r0),
        (axes[1],'After: original-record cleanup',g1,c1,r1),
        (axes[2],'Removed source records (orange)',g1,c1,raw.xyz[removed])]:
        ax.scatter(green[:,0],green[:,1],s=2,c='#42cf55',rasterized=True)
        ax.scatter(red[:,0],red[:,1],s=2,c='#ed5454' if ax is not axes[2] else '#f2a030',rasterized=True)
        ax.set_title(title); ax.set_aspect('equal'); ax.set_xlabel('map X (m)'); ax.grid(alpha=.15)
    axes[0].set_ylabel('map Y (m)')
    fig.tight_layout();fig.savefig(output/'before_after.png',dpi=150);plt.close(fig)
    # An enlarged long corridor makes the formerly restored low-return speckle
    # visible. It is not an arbitrary masked-free ROI or a planning operation.
    fig,axes=plt.subplots(1,2,figsize=(10,10),sharex=True,sharey=True)
    for ax,title,green,red in [(axes[0],'Before',g0,r0),(axes[1],'After same-source selection',g1,r1)]:
        ax.scatter(green[:,0],green[:,1],s=12,c='#42cf55',marker='s',linewidths=0)
        ax.scatter(red[:,0],red[:,1],s=12,c='#ed5454',marker='s',linewidths=0)
        ax.set_xlim(2,12);ax.set_ylim(-1,44);ax.set_aspect('equal');ax.set_title(title)
    fig.tight_layout();fig.savefig(output/'east_corridor_before_after.png',dpi=150);plt.close(fig)
    (output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('before','after','output'):p.add_argument('--'+name,type=Path,required=True)
    run(**vars(p.parse_args()))
