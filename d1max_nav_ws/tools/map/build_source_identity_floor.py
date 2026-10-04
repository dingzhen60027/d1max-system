#!/usr/bin/env python3
"""Build a bounded source-coordinate floor PCT without modifying any XYZ.

The existing floor evidence selects *original point indices*, never conditioned
coordinates. All source returns in the XY ROI (including ceilings/obstacles)
participate in tomography; only admissible ground cells are floor-owned.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'src/d1max_pct_scan'), str(ROOT/'src/d1max_pct_planner')]
from d1max_pct_scan.source_route import digest_file
from d1max_pct_planner.cpu_tomography import tomogram_from_points
from tools.pointcloud_preprocessing.pcd_io import read_pcd, write_subset


def build(source, evidence, config, output, floor_id):
    if not floor_id.isidentifier():
        raise ValueError('floor_id_must_be_an_identifier')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    inherited = yaml.safe_load(Path(config).read_text())
    cfg = {k:inherited[k] for k in ('preprocessing','pct')}
    cloud = read_pcd(source)
    lo, hi = np.asarray(cfg['preprocessing']['roi']['min']), np.asarray(cfg['preprocessing']['roi']['max'])
    selected = np.flatnonzero(cloud.finite_xyz_mask & np.all((cloud.xyz >= lo) & (cloud.xyz <= hi), axis=1))
    write_subset(output/'processed_map.pcd', cloud, selected)
    with np.load(evidence, allow_pickle=False) as audit:
        ids = audit[floor_id+'_source_indices'][audit[floor_id+'_support']]
    if ids.dtype.kind not in 'iu' or np.any(ids < 0) or np.any(ids >= len(cloud.xyz)):
        raise ValueError('invalid_original_floor_evidence_indices')
    # No planar prediction or synthesized returns: source points only.
    support = cloud.xyz[ids]
    np.savez_compressed(output/'source_indices.npz', selected_source_indices=selected,
                        **{floor_id+'_support_source_indices': ids, floor_id+'_support_xyz': support})
    payload, stats = tomogram_from_points(cloud.xyz[selected], cfg['pct'])
    data = payload['data']; ground = data[3]
    x = payload['center'][0] + (np.arange(ground.shape[1])-ground.shape[1]//2)*payload['resolution']
    y = payload['center'][1] + (np.arange(ground.shape[2])-ground.shape[2]//2)*payload['resolution']
    support_tree = cKDTree(support)
    # Union of both previously declared stair domains. Exclude XY at every
    # slice (including landings); retain their physical returns as obstacles.
    stair_lo, stair_hi = np.array([-34.3,46.7]), np.array([-26.2,57.3])
    forbidden = ((x[:,None]>=stair_lo[0]) & (x[:,None]<=stair_hi[0]) &
                 (y[None,:]>=stair_lo[1]) & (y[None,:]<=stair_hi[1]))
    counts = []
    for layer in range(len(ground)):
        ix, iy = np.where(np.isfinite(ground[layer]))
        points = np.c_[x[ix],y[iy],ground[layer,ix,iy]]
        supported = support_tree.query(points, workers=1)[0] <= .12
        keep = np.zeros(ground.shape[1:], bool)
        keep[ix[supported],iy[supported]] = True
        keep &= ~forbidden
        data[0,layer,~keep] = 50.
        counts.append(dict(layer=layer, supported_cells=int(keep.sum()),
            allowed_cells=int(np.count_nonzero(keep & (data[0,layer]<=20)))))
    data[1:3] = 0.
    data[1,:,1:-1,:] = data[0,:,2:,:]-data[0,:,:-2,:]
    data[2,:,:,1:-1] = data[0,:,:,2:]-data[0,:,:,:-2]
    source_hash = digest_file(source)
    manifest = dict(schema='d1max.source_identity_floor/v1', status='complete',
        geometry_operation='source_identity', frame_id='d1max_loc_map', floor_id=floor_id,
        source_path=str(Path(source).resolve()), source_sha256=source_hash,
        output_file='processed_map.pcd', output_sha256=digest_file(output/'processed_map.pcd'),
        source_indices_file='source_indices.npz', source_indices_sha256=digest_file(output/'source_indices.npz'),
        floor_evidence_path=str(Path(evidence).resolve()), floor_evidence_sha256=digest_file(evidence),
        configuration=cfg,parent_configuration=str(Path(config).resolve()),
        parent_configuration_sha256=digest_file(config),
        forbidden_stair_xy=[stair_lo.tolist(),stair_hi.tolist()],
        source_points=len(cloud.xyz), retained_points=len(selected), support_points=len(ids),
        xyz_modified=False, source_coordinate_error_m=0.,
        physical_acceptance=False, support_radius_m=.12,
        notes=['Observed original records only; no ground filling or flattening.',
               'Unknown overhead remains unknown; local safety is still required.'])
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    payload.update(source_pcd=str(output/'processed_map.pcd'),
        source_sha256=manifest['output_sha256'],frame_id='d1max_loc_map',
        original_source_pcd=str(Path(source).resolve()),original_source_sha256=source_hash,
        source_processing_manifest=str(output/'manifest.json'),
        source_processing_manifest_sha256=digest_file(output/'manifest.json'),
        geometry_operation='source_identity',processing_schema=manifest['schema'],planning_only=False)
    np.savez_compressed(output/'tomogram.npz',**payload)
    stats.update(floor_mask=counts,source_identity=True,source_map_sha256=source_hash,
                 tomogram_sha256=digest_file(output/'tomogram.npz'),physical_acceptance=False)
    (output/'build_report.json').write_text(json.dumps(stats,indent=2)+'\n')
    route = dict(schema='d1max.source_identity_route/v1',frame_id='d1max_loc_map',
        floor_id=floor_id,stairs_enabled=False,source_pcd=str(output/'processed_map.pcd'),
        tomogram_path=str(output/'tomogram.npz'),vendor_root=str(ROOT/'src/pct_planner_vendor'),
        floor_z_ranges=dict(lower=[float(support[:,2].min())-.15,float(support[:,2].max())+.15]),
        unknown_ceiling_policy='reject',minimum_headroom_m=.55,
        limits=dict(max_ground_step_m=.17),planning=dict(max_heading_rate=1.,astar_cost_weight=1.,
            optimizer_cost_margin=8.,path_refinement='visibility_c2',refinement_corner_cut_m=1.5,
            optimizer_sample_interval=10))
    (output/'route.yaml').write_text(yaml.safe_dump(route,sort_keys=False))
    print(json.dumps({'output':str(output),'shape':list(data.shape),'floor_mask':counts}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for name in ('source','evidence','config','output'):
        parser.add_argument('--'+name,required=True,type=Path)
    parser.add_argument('--floor-id',required=True,help='floor name recorded in the map package')
    build(**vars(parser.parse_args()))
