"""Extract measured floor/tread points. No traversability or robot model.

Exported XYZ/intensity records are an exact subset of the source PCD. Normals,
local planar bands and component cleanup classify points, never move or fill them.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from scipy import ndimage
from scipy.spatial import cKDTree


def read_source(path):
    with path.open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Missing PCD DATA header')
            words = line.decode('ascii').strip().split()
            if words and not words[0].startswith('#'):
                header[words[0]] = words[1:]
            if words[:1] == ['DATA']:
                break
        if header.get('FIELDS') != ['x','y','z','intensity'] or header.get('DATA') != ['binary']:
            raise ValueError('This source reader requires binary XYZI PCD')
        if header.get('SIZE') != ['4']*4 or header.get('TYPE') != ['F']*4 or header.get('COUNT') != ['1']*4:
            raise ValueError('Unsupported source field representation')
        points = int(header['POINTS'][0])
        data = np.frombuffer(stream.read(), dtype='<f4')
        if data.size != points*4:
            raise ValueError('PCD point count / file length mismatch')
    return data.reshape(-1,4)


def write_xyzi(path, records):
    header = (f'# .PCD v0.7 - measured source subset\nVERSION 0.7\nFIELDS x y z intensity\n'
              f'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(records)}\nHEIGHT 1\n'
              f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(records)}\nDATA binary\n')
    with path.open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(np.ascontiguousarray(records,dtype='<f4').tobytes())


def height(plane, x, y):
    a,b,c,d = plane
    return -(a*x+b*y+d)/c


def clean_indices(xyz, selected, cfg, stairs=False):
    ids = np.flatnonzero(selected)
    if not len(ids):
        return ids
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[ids]))
    _, good = cloud.remove_radius_outlier(nb_points=cfg['minimum_neighbours'], radius=cfg['radius_m'])
    ids = ids[np.asarray(good,dtype=int)]
    if not len(ids): return ids
    res = cfg['component_grid_m']
    cell = np.floor((xyz[ids,:2]-xyz[ids,:2].min(0))/res).astype(int)
    shape = tuple(cell.max(0)+1)
    occupied = np.zeros(shape, bool);occupied[cell[:,0],cell[:,1]]=True
    labels,_=ndimage.label(occupied,np.ones((3,3)))
    size=np.bincount(labels.ravel())
    minimum=cfg['minimum_stair_component_cells' if stairs else 'minimum_floor_component_cells']
    keep=size[labels[cell[:,0],cell[:,1]]] >= minimum
    return ids[keep]


def select_ground(xyz, normals, poses, cfg):
    region=np.full(len(xyz),-1,np.int8)
    for k,floor in enumerate(cfg['floors']):
        plane=np.asarray(floor['plane'],float)
        normal=plane[:3]/np.linalg.norm(plane[:3])
        distance=np.abs(xyz@normal+plane[3]/np.linalg.norm(plane[:3]))
        alignment=np.abs(normals@normal)
        selected=(distance<=floor['plane_band_m']) & (alignment>=floor['minimum_normal_alignment'])
        ids=clean_indices(xyz,selected,cfg['cleanup'])
        region[ids]=k
    # Estimate height offset from measured main-floor planes and recorded poses.
    # This is not clearance, body height, or any traversability threshold.
    diff=np.stack([poses[:,2]-height(f['plane'],poses[:,0],poses[:,1]) for f in cfg['floors']],axis=1)
    valid=(diff>0)&(diff<1.5)
    closest=np.min(np.where(valid,diff,np.inf),axis=1)
    offset=float(np.median(closest[np.isfinite(closest)]))
    if not np.isfinite(offset):
        raise ValueError('Cannot associate scan trajectory with the floor planes')
    trace=poses.copy();trace[:,2]-=offset
    st=cfg['stairs']
    roi=(xyz[:,:2]>=st['xy_min']).all(1)&(xyz[:,:2]<=st['xy_max']).all(1)
    lower=height(cfg['floors'][0]['plane'],xyz[:,0],xyz[:,1])
    upper=height(cfg['floors'][1]['plane'],xyz[:,0],xyz[:,1])
    candidate=roi&(xyz[:,2]>=lower-.1)&(xyz[:,2]<=upper+.1)&(np.abs(normals[:,2])>=st['minimum_abs_normal_z'])
    ids=np.flatnonzero(candidate)
    tree=cKDTree(trace)
    _,near=tree.query(xyz[ids],k=list(range(1,min(8,len(trace))+1)))
    delta=xyz[ids,None,:]-trace[near]
    associated=(np.abs(delta[:,:,2])<=st['trajectory_vertical_tolerance_m']) & (np.linalg.norm(delta[:,:,:2],axis=2)<=st['trajectory_lateral_tolerance_m'])
    selected=np.zeros(len(xyz),bool);selected[ids[associated.any(1)]]=True
    stairs_ids=clean_indices(xyz,selected,cfg['cleanup'],stairs=True)
    # End platforms already part of a main floor remain in that floor; staircase
    # detail rendering also shows nearby floor endpoints without duplicating data.
    region[stairs_ids[region[stairs_ids]<0]]=2
    return region,offset


def write_color_ply(path,xyz,colors):
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    cloud.colors=o3d.utility.Vector3dVector(np.asarray(colors,float)/255)
    if not o3d.io.write_point_cloud(str(path),cloud,write_ascii=False):
        raise RuntimeError(f'Could not write {path}')


def overview(out,xyz,regions,colors):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,11),facecolor='#101722')
    for k,ax in enumerate(axes):
        p=xyz[regions==k]
        ax.set_facecolor('#101722')
        ax.scatter(p[:,0],p[:,1],s=.35,color=colors[k]/255,linewidths=0)
        ax.set_aspect('equal');ax.set_title(['LOWER FLOOR — GROUND ONLY','UPPER FLOOR — GROUND ONLY'][k],color='white',fontsize=13)
        ax.tick_params(colors='#8798ac');ax.set_xlabel('X / m',color='#8798ac');ax.set_ylabel('Y / m',color='#8798ac')
        for spine in ax.spines.values():spine.set_color('#283347')
    fig.suptitle('SC-PGO 09-04 14:34 | measured floor points | no robot model',color='white',fontsize=15)
    fig.tight_layout();fig.savefig(out/'two_floors_overview.png',dpi=170,facecolor=fig.get_facecolor());plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path)
    args=parser.parse_args()
    started=time.perf_counter();config=args.config.resolve(strict=True)
    cfg=yaml.safe_load(config.read_text())
    if cfg.get('kind')!='ground_only' or 'robot' in cfg or cfg['cleanup']['hole_filling']:
        raise ValueError('Ground-only configuration required; no robot model or invented points')
    source=Path(cfg['source']['pcd']).resolve(strict=True)
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    records=read_source(source)
    if len(records)!=cfg['source']['expected_points']:raise ValueError('Wrong source map')
    b=np.asarray(cfg['preprocess']['bounds']);all_xyz=records[:,:3]
    roi=np.isfinite(all_xyz).all(1)&(all_xyz>=b[0]).all(1)&(all_xyz<=b[1]).all(1)
    original_index=np.flatnonzero(roi);xyz=all_xyz[roi]
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=cfg['preprocess']['normal_radius_m'],max_nn=cfg['preprocess']['normal_max_neighbours']))
    poses=np.loadtxt(cfg['source']['poses']).reshape(-1,3,4)[:,:,3]
    regions,offset=select_ground(xyz,np.asarray(cloud.normals),poses,cfg)
    selected=regions>=0;original_index=original_index[selected];regions=regions[selected];xyz=xyz[selected]
    colors=np.array([f['color'] for f in cfg['floors']]+[cfg['stairs']['color']],np.uint8)
    out=(config.parent/cfg['output']['directory']).resolve();out.mkdir(parents=True,exist_ok=True)
    shutil.copy2(config,out/'pipeline.yaml')
    write_xyzi(out/'ground.pcd',records[original_index])
    write_color_ply(out/'ground_colored.ply',xyz,colors[regions])
    np.savez_compressed(out/'ground_points.npz',xyz=xyz,intensity=records[original_index,3],source_indices=original_index,region=regions,frame=np.array(cfg['source']['frame']))
    layers=[]
    for k,name in enumerate(['lower','upper','stairs']):
        ids=regions==k;p=xyz[ids]
        write_xyzi(out/f'{name}_ground.pcd',records[original_index[ids]])
        write_color_ply(out/f'{name}_ground.ply',p,np.tile(colors[k],(len(p),1)))
        np.column_stack((p,np.tile(colors[k]/255,(len(p),1)))).astype('<f4').tofile(out/f'{name}.bin')
        layers.append({'id':name,'label':['下层地面','上层地面','楼梯踏面'][k],'points':int(ids.sum()),'file':f'{name}.bin',
                       'bounds':[p.min(0).tolist(),p.max(0).tolist()] if len(p) else None,'color':colors[k].tolist()})
    bg=cloud.voxel_down_sample(cfg['output']['source_preview_voxel_m']);bg_xyz=np.asarray(bg.points)
    np.column_stack((bg_xyz,np.full(bg_xyz.shape,.39))).astype('<f4').tofile(out/'source.bin')
    poses.astype('<f4').tofile(out/'recorded_trajectory.bin')
    report={'kind':'ground_only','name':cfg['name'],'source':cfg['source'],'source_sha256':digest,'input_points':len(records),
            'ground_points':len(xyz),'layers':layers,'estimated_trajectory_to_ground_offset_m':offset,
            'floor_planes':[f['plane'] for f in cfg['floors']],
            'stairs_xy_bounds':[cfg['stairs']['xy_min'],cfg['stairs']['xy_max']],
            'display_point_size_px':cfg['output']['display_point_size_px'],'source_preview_points':len(bg_xyz),
            'coordinate_change':False,'synthetic_points':0,'robot_model':False,'costmap':False,'source_unchanged':True,
            'generated_at':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'elapsed_seconds':round(time.perf_counter()-started,2),
            'notes':['仅筛选地面与楼梯踏面，PCD 的 XYZ 和 intensity 均来自原文件。','颜色只区分上下层和楼梯，不表示通行风险。',
                     '未插值补洞；大块空白可能是缺测或真实缺口。','没有机器人尺寸、障碍膨胀、通行成本或路径搜索。']}
    overview(out,xyz,regions,colors)
    if digest!=hashlib.sha256(source.read_bytes()).hexdigest():raise RuntimeError('Source changed during extraction')
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
