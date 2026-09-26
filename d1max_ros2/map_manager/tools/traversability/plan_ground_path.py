"""Offline 2.5D A*: measured support + source-cloud obstacles; no ROS/control."""
import argparse
import heapq
import json
import math
import time
from pathlib import Path
import numpy as np
import yaml
from scipy import ndimage
from scipy.spatial import cKDTree
from extract_ground import read_source


def astar(free, z, start, goal, resolution, max_step):
    def edges(u):
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            v=(u[0]+dx,u[1]+dy)
            if not (0<=v[0]<free.shape[0] and 0<=v[1]<free.shape[1] and free[v]):continue
            if dx and dy and (not free[u[0]+dx,u[1]] or not free[u[0],u[1]+dy]):continue
            if abs(z[v]-z[u])>max_step:continue
            yield v,math.sqrt(resolution**2*(dx*dx+dy*dy)+(z[v]-z[u])**2)
    q=[(0.,0.,start)]; dist={start:0.}; parent={}
    while q:
        _,g,u=heapq.heappop(q)
        if g>dist[u]:continue
        if u==goal:
            path=[u]
            while u!=start:u=parent[u];path.append(u)
            return path[::-1],len(dist)
        for v,cost in edges(u):
            d=g+cost
            if d<dist.get(v,float('inf')):
                dist[v]=d;parent[v]=u
                heapq.heappush(q,(d+resolution*math.hypot(v[0]-goal[0],v[1]-goal[1]),d,v))
    raise ValueError('No connected path under configured support/obstacle constraints')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,required=True);args=parser.parse_args()
    cfg=yaml.safe_load(args.config.read_text());out=Path(cfg['output']);out.mkdir(parents=True,exist_ok=False)
    (out/'planner.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    p=read_source(Path(cfg['ground']))[:,:3];source=read_source(Path(cfg['source']))[:,:3]
    res=cfg['resolution_m'];origin=np.floor(p[:,:2].min(0)/res)*res-res
    cell=np.floor((p[:,:2]-origin)/res).astype(int);shape=tuple(cell.max(0)+2)
    count=np.zeros(shape,int);np.add.at(count,(cell[:,0],cell[:,1]),1)
    z=np.full(shape,np.nan);flat=np.ravel_multi_index(cell.T,shape);order=np.argsort(flat)
    unique,first,counts=np.unique(flat[order],return_index=True,return_counts=True)
    for key,a,n in zip(unique,first,counts):z.flat[key]=np.median(p[order[a:a+n],2])
    support=count>=cfg['minimum_points_per_cell']
    # Height is only looked up near an actually measured support cell.
    support_cells=np.argwhere(support);centers=origin+(support_cells+.5)*res
    d,near=cKDTree(centers).query(source[:,:2]);floor=z[tuple(support_cells[near].T)]
    dz=source[:,2]-floor
    occupied=np.zeros(shape,bool)
    select=(d<=cfg['obstacle_association_m'])&(dz>=cfg['obstacle_min_height_m'])&(dz<=cfg['obstacle_max_height_m'])
    obstacle_cell=np.floor((source[select,:2]-origin)/res).astype(int)
    inside=(obstacle_cell>=0).all(1)&(obstacle_cell<shape).all(1);obstacle_cell=obstacle_cell[inside]
    occupied[tuple(obstacle_cell.T)]=True
    valid=support&~occupied
    clearance=ndimage.distance_transform_edt(np.pad(valid,1))[1:-1,1:-1]*res
    free=valid&(clearance>=cfg['clearance_m']+res/math.sqrt(2))
    candidates=np.argwhere(free)
    if not len(candidates):raise ValueError('No supported cells after clearance filtering')
    xy=origin+(candidates+.5)*res
    start=tuple(candidates[np.argmin(np.linalg.norm(xy-np.asarray(cfg['start_xy']),axis=1))])
    goal=tuple(candidates[np.argmin(np.linalg.norm(xy-np.asarray(cfg['goal_xy']),axis=1))])
    for node,requested in [(start,cfg['start_xy']),(goal,cfg['goal_xy'])]:
        if np.linalg.norm(origin+(np.asarray(node)+.5)*res-requested)>cfg['max_endpoint_snap_m']:raise ValueError('Endpoint snap too far')
    np.savez_compressed(out/'planning_grid.npz',free=free,support=support,obstacles=occupied,height=z,origin=origin,resolution=res)
    labels,n=ndimage.label(free)
    sizes=np.bincount(labels.ravel());sizes[0]=0
    print('Connectivity',json.dumps({'components':n,'largest_cells':int(sizes.max()),'start_cells':int(sizes[labels[start]]),'goal_cells':int(sizes[labels[goal]])}),flush=True)
    t=time.perf_counter()
    try:
        route,expanded=astar(free,z,start,goal,res,cfg['max_adjacent_height_m'])
    except ValueError as exc:
        (out/'path_report.json').write_text(json.dumps({'success':False,'reason':str(exc),'config':cfg}))
        raise
    elapsed=time.perf_counter()-t
    cells=np.asarray(route);path=np.column_stack((origin+(cells+.5)*res,z[tuple(cells.T)]))
    assert np.all(free[tuple(cells.T)])
    for a,b in zip(cells[:-1],cells[1:]):
        delta=abs(b-a);assert delta.max()==1
        if delta.min()==1:assert free[a[0],b[1]] and free[b[0],a[1]]
        assert abs(z[tuple(a)]-z[tuple(b)])<=cfg['max_adjacent_height_m']
    np.savetxt(out/'path.csv',path,delimiter=',',header='x,y,z',comments='')
    path.astype('<f4').tofile(out/'planned_path.bin')
    np.savez_compressed(out/'planning_grid.npz',free=free,support=support,obstacles=occupied,height=z,origin=origin,resolution=res)
    report={'success':True,'algorithm':'A* on measured-ground 2.5D grid (not PCT/Nav2)', 'path_points':len(path),
        'length_m':float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),'search_seconds':elapsed,'expanded_nodes':expanded,
        'start_xyz':path[0].tolist(),'goal_xyz':path[-1].tolist(),'minimum_grid_clearance_m':float(clearance[tuple(cells.T)].min()),
        'supported_cells':int(support.sum()),'free_cells':int(free.sum()),'obstacle_cells':int(occupied.sum()),
        'all_path_cells_supported':True,'diagonal_corner_cutting':False,'robot_commands_sent':False,
        'limitations':['Single-floor static geometry test, not executable robot navigation.', 'No dynamic obstacle, localization, gait or braking validation.'],
        'config':cfg}
    (out/'path_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
