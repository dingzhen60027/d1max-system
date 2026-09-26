"""Recover missing lower-floor cells from independently observed scan returns.

XY is copied from an actual optimized keyframe return. The recorded floor1
conditioning field derives planning Z; raw sample XYZ and field/Z evidence are
persisted. This never clears a trajectory tube or changes PCT costs.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import open3d as o3d
from scipy.ndimage import convolve
import yaml

WS = Path('/home/dndx/d1max_nav_ws')
sys.path.insert(0, str(WS))
sys.path.insert(0, str(WS / 'src/d1max_pct_planner'))
VENDOR = str(WS / 'src/pct_planner_vendor')
if os.environ.get('SWEEP_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(VENDOR)
    env['SWEEP_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:]], env=env).returncode)

import native_sweep as ns
from d1max_pct_planner.crossfloor_route import plan_crossfloor, _masked_tomogram, validate_config
from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_planner.tomogram_route import TomogramRoute
from tools.pointcloud_preprocessing.flat_floor import _anchors, _config, _field
from tools.pointcloud_preprocessing.multifloor_runner import protected_mask
from tools.pointcloud_preprocessing.pcd_io import read_pcd

ROOT = Path(__file__).resolve().parent
BASE = WS / 'maps/processed/sc_pgo_20260923_crossfloor_complete'
CONDITIONED = WS / 'maps/processed/sc_pgo_20260923_multifloor_conditioned_v1'
RUN = WS / 'maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/sc_pgo'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    with np.load(ROOT / 'cache_composed.npz') as z:
        payload = {k: z[k] for k in z.files}
    payload.update(frame_id='d1max_loc_map')
    tomo = TomogramMap(payload, unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    manifest = json.loads((CONDITIONED / 'manifest.json').read_text())
    configuration = manifest['configuration']
    parameters = _config({**manifest['resolved_floor_defaults']['conditioning'],
                          **configuration.get('conditioning_overrides', {}), 'reference_z_m': -.65})
    original = read_pcd(ns.PCD).xyz
    poses = np.loadtxt(RUN / 'optimized_poses.txt').reshape(-1, 3, 4)
    positions = poses[:, :, 3]
    selected = positions[:756]
    selected = selected[~protected_mask(selected, configuration['protected_regions'])]
    with np.load(CONDITIONED / 'audit/floor_evidence.npz') as evidence:
        floor_indices = evidence['floor1_source_indices']
        expected_xy, expected_z = evidence['floor1_anchors_xy'], evidence['floor1_anchors_z']
        floor_field_existing = evidence['floor1_floor_z']
    anchors = _anchors(original[floor_indices], selected, parameters)
    assert np.allclose(anchors['xy'], expected_xy, atol=1e-8)
    assert np.allclose(anchors['z'], expected_z, atol=1e-8)
    # Check the reconstructed, slope-aware field against saved per-point values.
    check_ids = np.flatnonzero(np.isfinite(floor_field_existing))[::1000]
    check_field = _field(original[floor_indices[check_ids], :2], anchors, parameters)
    field_error = float(np.max(np.abs(check_field-floor_field_existing[check_ids])))
    assert field_error < 1e-8
    ground = tomo.ground[4]
    support = np.isfinite(ground) & (np.abs(ground+.65) <= .08)
    neighbors = convolve(support.astype(int), np.ones((5, 5), int), mode='constant')
    grid_indices = np.indices(ground.shape).reshape(2, -1).T
    cell_xy = tomo.center+(grid_indices-tomo.offset)*tomo.resolution
    field = _field(cell_xy, anchors, parameters).reshape(ground.shape)
    protected = protected_mask(np.column_stack([cell_xy, np.full(len(cell_xy), -.65)]),
                               configuration['protected_regions']).reshape(ground.shape)
    holes = ((~np.isfinite(ground) | (ground < -.9)) & (neighbors >= 8)
             & np.isfinite(field) & ~protected)
    hole_ids = np.flatnonzero(holes)
    records = {int(k): [] for k in hole_ids}
    sources = []
    print('missing cells with existing floor support', len(hole_ids), flush=True)
    for frame in range(736):
        path = RUN / 'Scans' / f'{frame:06d}.pcd'
        scan = np.asarray(o3d.io.read_point_cloud(str(path)).points)
        pose = poses[frame]
        world = scan @ pose[:, :3].T + pose[:, 3]
        # Broad raw lower-floor height; final membership is field-relative.
        selected_mask = ((world[:, 2] >= -1.1) & (world[:, 2] <= -.25))
        raw_indices = np.flatnonzero(selected_mask)
        world = world[selected_mask]
        idx = np.rint((world[:, :2]-tomo.center)/tomo.resolution).astype(int)+tomo.offset
        inside = ((idx>=0)&(idx<ground.shape)).all(axis=1)
        world, idx, raw_indices = world[inside], idx[inside], raw_indices[inside]
        ids = np.ravel_multi_index(idx.T, ground.shape)
        selected_mask = holes.ravel()[ids]
        world, ids, raw_indices = world[selected_mask], ids[selected_mask], raw_indices[selected_mask]
        if not len(world):
            continue
        local_field = _field(world[:, :2], anchors, parameters)
        residual = world[:, 2]-local_field
        selected_mask = np.isfinite(residual)&(np.abs(residual)<=.055)
        world, ids, raw_indices = world[selected_mask], ids[selected_mask], raw_indices[selected_mask]
        local_field, residual = local_field[selected_mask], residual[selected_mask]
        before = sum(len(rows) for rows in records.values())
        for k in np.unique(ids):
            group = np.flatnonzero(ids==k)
            chosen = group[np.argsort(residual[group])[len(group)//2]]
            records[int(k)].append({'frame': frame, 'scan_point_index': int(raw_indices[chosen]),
                                   'sample_xyz': world[chosen].tolist(), 'field_z': float(local_field[chosen]),
                                   'derived_z': float(-.65+residual[chosen])})
        after = sum(len(rows) for rows in records.values())
        if after > before:
            sources.append({'frame': frame, 'scan_path': str(path), 'scan_sha256': digest(path),
                            'cells_observed': after-before})
        if frame % 50 == 0:
            print('frame', frame, 'observations', after, flush=True)
    points, raw_points, records_out = [], [], []
    for k, rows in sorted(records.items()):
        if len(rows)<5:
            continue
        heights = np.asarray([r['derived_z'] for r in rows])
        if np.ptp(heights)>.08:
            continue
        row = rows[np.argsort(heights)[len(rows)//2]]
        sample = np.array(row['sample_xyz'])
        derived = sample.copy()
        derived[2] = row['derived_z']
        cell = np.unravel_index(k, ground.shape)
        points.append(derived)
        raw_points.append(sample)
        records_out.append({'cell': list(map(int, cell)), 'cell_xy': tomo.world(cell).tolist(),
                            'old_ground_z': float(ground[cell]) if np.isfinite(ground[cell]) else None,
                            'sample_xyz': sample.tolist(), 'derived_xyz': derived.tolist(),
                            'derived_z': row['derived_z'], 'field_z': row['field_z'],
                            'source_frame': row['frame'], 'source_point_index': row['scan_point_index'],
                            'support_frames': [r['frame'] for r in rows],
                            'derived_height_range': [float(heights.min()), float(heights.max())],
                            'independent_observations': rows})
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    np.savez_compressed(ROOT/'lower_floor_measured_restore.npz', points=points,
                        sample_xyz=np.asarray(raw_points), source_field_z=[r['field_z'] for r in records_out])
    (ROOT/'lower_floor_measured_restore.json').write_text(json.dumps(records_out, indent=2)+'\n')
    lineage = {'source_pcd': ns.PCD, 'source_sha256': digest(ns.PCD),
               'conditioned_manifest': str(CONDITIONED/'manifest.json'),
               'conditioned_manifest_sha256': digest(CONDITIONED/'manifest.json'),
               'floor_evidence': str(CONDITIONED/'audit/floor_evidence.npz'),
               'floor_evidence_sha256': digest(CONDITIONED/'audit/floor_evidence.npz'),
               'optimized_poses': str(RUN/'optimized_poses.txt'), 'optimized_poses_sha256': digest(RUN/'optimized_poses.txt'),
               'field_reconstruction_max_error_m': field_error, 'parameters': parameters,
               'candidate_cells': len(hole_ids), 'restored_cells': len(points),
               'rule': {'minimum_frames':5, 'max_height_range_m':.08, 'max_field_residual_m':.055,
                        'neighbor_support_cells':8, 'neighbor_window':5, 'layer_id':4,
                        'height_operation':'derived_z = raw_sample_z - same_floor1_field(xy) - 0.65',
                        'snap_to_plane':False, 'trajectory_clearing':False}, 'scans':sources}
    (ROOT/'lower_floor_measured_restore.sources.json').write_text(json.dumps(lineage,indent=2)+'\n')
    print('RESTORED',len(points),'of',len(hole_ids),flush=True)
    test = ns.build(np.concatenate([read_pcd(BASE/'processed_map.pcd').xyz, points]), .10, .50, ns.CROP,
                    {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10})
    np.savez_compressed(ROOT/'cache_composed_lower_restore.npz',data=test.data,resolution=test.resolution,
                        center=test.center,slice_h0=test.slice_h0,slice_dh=test.slice_dh,selected_source_layers=test.source_layers)
    config=yaml.safe_load(open(ns.ROUTE_CFG))
    for key,anchor in config['anchors'].items():
        chosen=ns.snap_anchor(test,anchor['xyz'][:2],ns.EXPECTED[key])
        config['anchors'][key]={k:chosen[k] for k in ('xyz','layer_id')}
    outcomes=[]
    start=ns.snap_anchor(test,positions[400,:2],-.65)
    config['anchors']['start']={k:start[k] for k in ('xyz','layer_id')}
    try:
        route=plan_crossfloor(config,test)
        (ROOT/'lower_restore_crossfloor_kf400.json').write_text(json.dumps(route,indent=2)+'\n')
        outcomes.append({'test':'crossfloor_kf400','ok':True,'length_m':route['length_m']})
    except Exception as exc:
        outcomes.append({'test':'crossfloor_kf400','ok':False,'code':getattr(exc,'code',''),
                         'message':str(exc),'cause':getattr(exc.__cause__,'details',None)})
    masked=_masked_tomogram(test,'lower_floor',validate_config(config))
    start=ns.snap_anchor(test,positions[350,:2],-.65)
    goal=ns.snap_anchor(test,positions[400,:2],-.65)
    try:
        planner=TomogramRoute(masked,VENDOR,max_heading_rate=1,astar_cost_weight=1,
                              optimizer_cost_margin=8,path_refinement='visibility_c2')
        route=planner.plan(start['xyz'],goal['xyz'],start['layer_id'],goal['layer_id'])
        (ROOT/'lower_restore_corridor_350_400.json').write_text(json.dumps(route,indent=2,
            default=lambda value:value.tolist() if isinstance(value,np.ndarray) else value.item())+'\n')
        outcomes.append({'test':'corridor_350_400','ok':True,'quality':route['path_quality']})
    except Exception as exc:
        outcomes.append({'test':'corridor_350_400','ok':False,'code':getattr(exc,'code',''),
                         'message':str(exc),'cause':getattr(exc,'details',None)})
    (ROOT/'lower_floor_recovery_validation.json').write_text(json.dumps(outcomes,indent=2)+'\n')
    print(json.dumps(outcomes,indent=2),flush=True)


if __name__=='__main__':
    main()
