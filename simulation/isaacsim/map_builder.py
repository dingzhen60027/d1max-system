#!/usr/bin/env python3
"""Create source-identity PCT evidence from the exact Isaac scene surfaces.

These are synthetic, known scene geometry samples, not a SLAM map. The normal
CPU tomography equations and native PCT route planner remain the repo's own.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from scipy.spatial import cKDTree
import yaml


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sampled_axis(lower, upper, spacing):
    return np.linspace(lower, upper, max(2, int(np.ceil((upper-lower)/spacing))+1))


def plane(xbounds, ybounds, z, spacing):
    x, y = np.meshgrid(sampled_axis(*xbounds, spacing), sampled_axis(*ybounds, spacing), indexing='ij')
    return np.c_[x.ravel(), y.ravel(), np.full(x.size, z)]


def box_surface(center, size, spacing):
    lower, upper = np.asarray(center)-np.asarray(size)/2, np.asarray(center)+np.asarray(size)/2
    parts = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        a, b = np.meshgrid(sampled_axis(lower[others[0]], upper[others[0]], spacing),
            sampled_axis(lower[others[1]], upper[others[1]], spacing), indexing='ij')
        for edge in (lower[axis], upper[axis]):
            part = np.empty((a.size, 3))
            part[:, axis] = edge
            part[:, others[0]], part[:, others[1]] = a.ravel(), b.ravel()
            parts.append(part)
    return np.vstack(parts)


def write_pcd(path, xyz):
    xyz = np.asarray(xyz, dtype='<f4')
    xyzi = np.c_[xyz, np.ones(len(xyz), dtype='<f4')].astype('<f4')
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
        'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH '+str(len(xyz))+'\nHEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(len(xyz))+'\nDATA binary\n')
    with Path(path).open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(xyzi.tobytes())


def build(scene_config, output, vendor_root):
    from d1max_pct_planner.cpu_tomography import tomogram_from_points
    from d1max_pct_scan.source_identity import validate_package
    scene_config = Path(scene_config).resolve(strict=True)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    spec = json.loads(scene_config.read_text())
    floor = spec['floor']
    x0, x1, y0, y1 = floor['bounds']
    spacing = .05
    ground = plane((x0, x1), (y0, y1), floor['z'], spacing)
    supported = np.ones(len(ground), bool)
    parts = [ground, plane((x0, x1), (y0, y1), spec['ceiling']['z'], spacing)]
    for box in spec['static_boxes']:
        parts.append(box_surface(box['center'], box['size'], spacing))
        lower = np.asarray(box['center'])-np.asarray(box['size'])/2
        upper = np.asarray(box['center'])+np.asarray(box['size'])/2
        supported &= ~np.all((ground[:, :2] >= lower[:2]) & (ground[:, :2] <= upper[:2]), axis=1)
    xyz = np.vstack(parts).astype('<f4')
    source = output/'source_scene.pcd'
    write_pcd(source, xyz)
    # processed_map is the same exact records; no conditioning, flattening or
    # coordinate relocation between global route and measured Isaac poses.
    shutil.copyfile(source, output/'processed_map.pcd')
    # The current global worker still admits floor1/floor2 names internally.
    # Use its existing floor1 profile for this generated one-floor fixture.
    floor_id = 'floor1'
    indices = np.arange(len(ground), dtype=np.int64)[supported]
    support = xyz[indices]
    np.savez_compressed(output/'source_indices.npz', selected_source_indices=np.arange(len(xyz)),
        **{floor_id+'_support_source_indices': indices, floor_id+'_support_xyz': support})
    pct = dict(resolution=.10, slice_dh=.50, ground_height=float(floor['z']),
        simplify_layers=False, limits=dict(max_total_cells=1000000, max_working_bytes=200000000),
        traversability=dict(kernel_size=7, interval_min=.55, interval_free=.70,
            slope_max_rad=.40, step_max=.17, standable_ratio=.20, cost_barrier=50.,
            safe_margin=.40, inflation=.20))
    payload, stats = tomogram_from_points(xyz, pct)
    data = payload['data']
    ground_layers = data[3]
    x = payload['center'][0]+(np.arange(ground_layers.shape[1])-ground_layers.shape[1]//2)*payload['resolution']
    y = payload['center'][1]+(np.arange(ground_layers.shape[2])-ground_layers.shape[2]//2)*payload['resolution']
    tree = cKDTree(support)
    masks = []
    for layer in range(len(ground_layers)):
        ix, iy = np.where(np.isfinite(ground_layers[layer]))
        points = np.c_[x[ix], y[iy], ground_layers[layer, ix, iy]]
        observed_floor = tree.query(points, workers=1)[0] <= .12
        keep = np.zeros(ground_layers.shape[1:], bool)
        keep[ix[observed_floor], iy[observed_floor]] = True
        # PCT may retain higher surfaces as layers, but this fixture only grants
        # source-supported floor cells; cabinet tops are not a second floor.
        data[0, layer, ~keep] = 50.
        masks.append(dict(layer=layer, supported_cells=int(keep.sum()),
            allowed_cells=int(np.count_nonzero(keep & (data[0, layer] <= 20.)))))
    data[1:3] = 0.
    data[1, :, 1:-1, :] = data[0, :, 2:, :]-data[0, :, :-2, :]
    data[2, :, :, 1:-1] = data[0, :, :, 2:]-data[0, :, :, :-2]
    # The required exclusion rectangle is explicitly outside this generated
    # single-floor room. The fixture has no stairs, and route.yaml forbids them.
    forbidden = [[x1+10, y1+10], [x1+11, y1+11]]
    manifest = dict(schema='d1max.source_identity_floor/v1', status='complete',
        geometry_operation='source_identity', frame_id='d1max_loc_map', floor_id=floor_id,
        source_path=str(source), source_sha256=sha(source),
        output_file='processed_map.pcd', output_sha256=sha(output/'processed_map.pcd'),
        source_indices_file='source_indices.npz', source_indices_sha256=sha(output/'source_indices.npz'),
        configuration=dict(pct=pct), parent_configuration=str(scene_config),
        parent_configuration_sha256=sha(scene_config), forbidden_stair_xy=forbidden,
        source_points=len(xyz), retained_points=len(xyz), support_points=len(support),
        xyz_modified=False, source_coordinate_error_m=0., physical_acceptance=False,
        support_radius_m=.12, simulation=True, provenance='sampled_exact_isaac_scene_surfaces',
        notes=['Synthetic scene map; not SLAM or real robot evidence.',
               'Floor ownership excludes XY under actual walls and obstacles.',
               'Actual configured ceiling is sampled; unknown space stays unknown.'])
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    payload.update(source_pcd=str(output/'processed_map.pcd'), source_sha256=manifest['output_sha256'],
        frame_id='d1max_loc_map', original_source_pcd=str(source), original_source_sha256=sha(source),
        source_processing_manifest=str(output/'manifest.json'),
        source_processing_manifest_sha256=sha(output/'manifest.json'), geometry_operation='source_identity',
        processing_schema=manifest['schema'], planning_only=False)
    np.savez_compressed(output/'tomogram.npz', **payload)
    route = dict(schema='d1max.source_identity_route/v1', frame_id='d1max_loc_map', floor_id=floor_id,
        stairs_enabled=False, source_pcd=str(output/'processed_map.pcd'),
        tomogram_path=str(output/'tomogram.npz'), vendor_root=str(Path(vendor_root).resolve(strict=True)),
        floor_z_ranges={'lower': [float(floor['z'])-.15, float(floor['z'])+.15]},
        unknown_ceiling_policy='reject', minimum_headroom_m=.55, limits=dict(max_ground_step_m=.17),
        planning=dict(max_heading_rate=1., astar_cost_weight=1., optimizer_cost_margin=8.,
            path_refinement='visibility_c2', refinement_corner_cut_m=1.5, optimizer_sample_interval=10))
    (output/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False))
    checked = validate_package(output)
    for name, point in [('initial', spec['robot']['initial_pose'][:3]), ('goal', spec['robot']['goal'])]:
        support_z, distance = checked['bridge'].query(np.asarray(point[:2]).reshape(1,2))
        stats[name+'_support'] = dict(z=float(support_z[0]), distance_m=float(distance[0]))
    stats.update(physical_acceptance=False, simulation=True, scene_sha256=sha(scene_config),
        source_sha256=sha(source), tomogram_sha256=sha(output/'tomogram.npz'), floor_mask=masks)
    (output/'build_report.json').write_text(json.dumps(stats, indent=2)+'\n')
    return dict(output=str(output), source_points=len(xyz), support_points=len(support),
        tomogram_shape=list(data.shape), physical_acceptance=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--vendor-root', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(**vars(args))))
