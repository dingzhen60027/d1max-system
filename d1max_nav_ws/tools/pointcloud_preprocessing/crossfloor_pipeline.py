"""Auditable offline building-map repair, PCT build and cross-floor regression.

One YAML binds each evidence cache to its exact source. Original maps are never
overwritten. Floor conditioning is independent of visibility filtering; scan
returns restore sampling coverage, never a fabricated flat surface. All motion
execution remains disabled.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

from .multifloor_runner import FRAME, protected_mask
from .pcd_io import read_pcd, _write_records
from . import load_config
from .runner import sha256

SCHEMA = 'd1max.crossfloor_planning/v1'
OPERATION = 'independent_floor_conditioning_visibility_filter_and_measured_return_restore'


def assert_unchanged(inputs):
    for path, digest in inputs.items():
        if sha256(path) != digest:
            raise ValueError(f'Input changed during cross-floor processing: {path}')


def merge_hashes(*groups):
    result = {}
    for group in groups:
        for path, digest in group.items():
            canonical = str(Path(path).resolve(strict=True))
            if canonical in result and result[canonical] != digest:
                raise ValueError(f'Conflicting source provenance: {canonical}')
            result[canonical] = digest
    return result


def _integer(value, label, minimum=0):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < minimum:
        raise ValueError(f'{label} must be an integer >= {minimum}')
    return int(value)


def verified_file(item):
    path = Path(item['path']).resolve(strict=True)
    if sha256(path) != item['sha256']:
        raise ValueError(f'Evidence hash mismatch: {path}')
    return path


def visibility_keep(xyz, votes, settings):
    count = len(xyz)
    free, hit, roi = (np.asarray(votes[k]) for k in ('free', 'hit', 'in_roi'))
    if any(v.shape != (count,) for v in (free, hit, roi)) or roi.dtype.kind != 'b':
        raise ValueError('Visibility evidence must match every source PCD record')
    if any(v.dtype.kind not in 'iu' or not np.isfinite(v).all() or (v < 0).any() for v in (free, hit)):
        raise ValueError('Invalid visibility counts')
    ratio = float(settings['free_hit_ratio'])
    minimum = _integer(settings['minimum_free_frames'], 'minimum_free_frames', 20)
    if not np.isfinite(ratio) or ratio < 10:
        raise ValueError('This reviewed visibility contract requires ratio >=10 and >=20 frames')
    remove = roi & (free >= ratio * np.maximum(hit, 1)) & (free >= minimum)
    protected = protected_mask(xyz, settings['structure_protection_regions'])
    remove &= ~protected
    return ~remove


def restore_records(cloud, evidence_path, sources_path, config):
    """Re-read real source returns, keeping intensity and frame/point identity."""
    sources = json.loads(Path(sources_path).read_text())
    assert_unchanged(sources)
    run = Path(config['keyframe_directory']).resolve(strict=True)
    poses_path = run / 'optimized_poses.txt'
    if str(poses_path) not in sources:
        raise ValueError('Pose file is not part of measurement provenance')
    pose_rows = np.loadtxt(poses_path, ndmin=2)
    if pose_rows.shape[1] != 12 or not np.isfinite(pose_rows).all():
        raise ValueError('Expected finite optimized KITTI 3x4 poses')
    poses = pose_rows.reshape(-1, 3, 4)
    minimum_frames = _integer(config['minimum_independent_frames'], 'minimum_independent_frames', 5)
    spread = float(config['maximum_height_spread_m'])
    if not np.isfinite(spread) or not 0 < spread <= .08:
        raise ValueError('maximum_height_spread_m must be in (0, 0.08]')
    evidence = json.loads(Path(evidence_path).read_text())
    output, used, scans = [], set(), {}
    for item in evidence:
        frames = [_integer(value, 'support frame') for value in item['frames']]
        if (len(set(frames)) < minimum_frames or len(frames) != len(set(frames))
                or any(frame >= len(poses) for frame in frames)):
            raise ValueError('Insufficient independent measured support')
        lo, hi = item['height_range']
        if not np.isfinite([lo, hi]).all() or not 0 <= hi - lo <= spread:
            raise ValueError('Restored surface has inconsistent height evidence')
        frame = _integer(item['source_frame_id'], 'source_frame_id')
        index = _integer(item['source_point_index'], 'source_point_index')
        if frame not in frames or (frame, index) in used or not 0 <= frame < len(poses):
            raise ValueError('Invalid/duplicated measured return identity')
        for support_frame in frames:
            if str(run / 'Scans' / f'{support_frame:06d}.pcd') not in sources:
                raise ValueError('Supporting frame is missing from source provenance')
        if frame not in scans:
            scans[frame] = read_pcd(run / 'Scans' / f'{frame:06d}.pcd')
        scan = scans[frame]
        if scan.records.dtype != cloud.records.dtype or not 0 <= index < len(scan.xyz):
            raise ValueError('Source scan point fields/index do not match map')
        world = scan.xyz[index] @ poses[frame, :, :3].T + poses[frame, :, 3]
        if not np.allclose(world, item['sample_xyz'], rtol=0, atol=1e-6):
            raise ValueError('Restoration is not the declared measured scan return')
        if not lo - 1e-6 <= world[2] <= hi + 1e-6:
            raise ValueError('Measured return is outside its declared height evidence')
        record = scan.records[index:index + 1].copy()
        for axis, value in zip(('x', 'y', 'z'), world):
            record[axis] = value
        output.append(record)
        used.add((frame, index))
    if not output:
        raise ValueError('No measured returns to restore')
    assert_unchanged(sources)
    return np.concatenate(output), evidence


def validate_floor_records(cloud, floor, indices):
    indices = np.asarray(indices)
    if (indices.dtype.kind not in 'iu' or indices.shape != (len(floor.xyz),)
            or cloud.records.dtype != floor.records.dtype):
        raise ValueError('Incompatible floor stage records')
    if (indices < 0).any() or (indices >= len(cloud.xyz)).any() or len(np.unique(indices)) != len(indices):
        raise ValueError('Invalid floor source indices')
    if not cloud.finite_xyz_mask.all() or not floor.finite_xyz_mask.all():
        raise ValueError('Nonfinite source/floor coordinates')
    for field in cloud.fields:
        if field != 'z' and cloud.records[field][indices].tobytes() != floor.records[field].tobytes():
            raise ValueError('Floor stage changed a non-Z field')


def select_portal(tomogram, request, radius, height_tolerance):
    """Resolve configured approximate portals once; record the actual shift.

    This is NOT used for interactive RViz endpoints. Those require exact
    selected surfaces and remain fail-closed in the planning worker.
    """
    xyz = np.asarray(request['xyz'], dtype=float)
    cell = tomogram.index(xyz[:2])
    r = int(np.ceil(radius / tomogram.resolution))
    choices = []
    for x in range(cell[0] - r, cell[0] + r + 1):
        for y in range(cell[1] - r, cell[1] + r + 1):
            if not tomogram.contains((x, y)):
                continue
            xy = tomogram.world((x, y))
            distance = float(np.linalg.norm(xy - xyz[:2]))
            if distance > radius:
                continue
            for layer in np.flatnonzero(tomogram.valid[:, x, y]):
                z = float(tomogram.ground[layer, x, y])
                error = abs(z - xyz[2])
                if error <= height_tolerance:
                    choices.append((distance, error, int(layer), [*xy.tolist(), z]))
    if not choices:
        raise ValueError(f'No valid measured portal near {xyz.tolist()}')
    distance, dz, layer, point = min(choices)
    return {'xyz': point, 'layer_id': layer}, {'requested_xyz': xyz.tolist(),
                                            'xy_shift_m': distance, 'z_shift_m': dz}


def run(path):
    config_path = Path(path).resolve(strict=True)
    config_hash = sha256(config_path)
    cfg = load_config(config_path)
    if cfg.get('schema') != SCHEMA or cfg.get('planning_only') is not True:
        raise ValueError('An explicit offline cross-floor planning derivative is required')
    source = verified_file(cfg['source'])
    floor_config = verified_file(cfg['floor_conditioning_config'])
    floor_cfg = load_config(floor_config)
    floor_dir = Path(floor_cfg['output_directory'])
    if not floor_dir.exists():
        from .multifloor_runner import run as build_floors
        build_floors(floor_config)
    floor_manifest_path = floor_dir / 'manifest.json'
    floor_manifest_hash = sha256(floor_manifest_path)
    floor_manifest = json.loads(floor_manifest_path.read_text())
    if (floor_manifest['source_sha256'] != cfg['source']['sha256']
            or floor_manifest['configuration'] != floor_cfg):
        raise ValueError('Floor stage cache is not from this source/configuration')
    for name, digest in floor_manifest['input_hashes'].items():
        if sha256(name) != digest:
            raise ValueError(f'Stale floor conditioning evidence: {name}')
    floor_file = floor_dir / floor_manifest['output_file']
    if sha256(floor_file) != floor_manifest['output_sha256']:
        raise ValueError('Floor stage output has changed')
    votes_path = verified_file(cfg['visibility']['evidence'])
    evidence_path = verified_file(cfg['measured_restore']['evidence'])
    sources_path = verified_file(cfg['measured_restore']['sources'])
    route_template_path = verified_file(cfg['route_template'])
    cloud, floor = read_pcd(source), read_pcd(floor_file)
    floor_evidence = floor_dir / 'audit/floor_evidence.npz'
    floor_evidence_hash = sha256(floor_evidence)
    if floor_evidence_hash != cfg['floor_evidence_sha256']:
        raise ValueError('Floor stage record lineage has changed')
    with np.load(floor_evidence, allow_pickle=False) as audit:
        indices = audit['source_indices']
    validate_floor_records(cloud, floor, indices)
    with np.load(votes_path, allow_pickle=False) as votes:
        keep = visibility_keep(cloud.xyz, votes, cfg['visibility'])
    restored, evidence = restore_records(cloud, evidence_path, sources_path, cfg['measured_restore'])
    inputs = {str(config_path): config_hash, str(source): cfg['source']['sha256'],
              str(floor_config): cfg['floor_conditioning_config']['sha256'],
              str(floor_manifest_path): floor_manifest_hash,
              str(floor_file): floor_manifest['output_sha256'], str(floor_evidence): floor_evidence_hash,
              str(votes_path): cfg['visibility']['evidence']['sha256'],
              str(evidence_path): cfg['measured_restore']['evidence']['sha256'],
              str(sources_path): cfg['measured_restore']['sources']['sha256'],
              str(route_template_path): cfg['route_template']['sha256']}
    inputs = merge_hashes(inputs, floor_manifest['input_hashes'], json.loads(sources_path.read_text()))
    restored_upper_count = len(restored)
    if cfg.get('conditioned_measured_restore'):
        from .measured_floor_restore import restore_conditioned_records
        lower = cfg['conditioned_measured_restore']
        lower_evidence, lower_sources = verified_file(lower['evidence']), verified_file(lower['sources'])
        lower_records, lower_items, lower_hashes = restore_conditioned_records(
            cloud, lower_evidence, lower_sources, floor_manifest_path)
        inputs = merge_hashes(inputs, lower_hashes,
                              {str(lower_evidence): lower['evidence']['sha256'],
                               str(lower_sources): lower['sources']['sha256']})
        restored = np.concatenate([restored, lower_records])
        evidence += lower_items
    assert_unchanged(inputs)
    selected = indices[keep[indices]]
    records = np.concatenate([floor.records[keep[indices]], restored])
    from d1max_pct_planner.official_pipeline import unique_output, build
    out = unique_output(cfg['output_directory'])
    target = out / 'processed_map.pcd'
    _write_records(target, cloud, records, 'offline multi-floor conditioned map and measured scan returns',
                   reset_viewpoint=True)
    np.savez_compressed(out / 'record_lineage.npz', retained_source_indices=selected,
                        visibility_removed_indices=np.flatnonzero(~keep),
                        restored_frame_ids=[i['source_frame_id'] for i in evidence],
                        restored_point_indices=[i['source_point_index'] for i in evidence])
    manifest = {'schema': 1, 'processing_schema': SCHEMA, 'status': 'complete',
                'geometry_operation': OPERATION, 'planning_only': True, 'frame_id': FRAME,
                'source_path': str(source), 'source_sha256': cfg['source']['sha256'],
                'output_file': target.name, 'output_sha256': sha256(target),
                'input_points': len(cloud.xyz), 'output_points': len(records),
                'removed_unique_source_points': len(cloud.xyz) - len(selected),
                'visibility_removed_source_points': int((~keep).sum()),
                'restored_measured_points': len(restored), 'synthetic_points': 0,
                'restored_upper_floor_points': restored_upper_count,
                'restored_conditioned_floor_points': len(restored) - restored_upper_count,
                'floor_processing_manifest': str(floor_dir / 'manifest.json'),
                'floor_processing_manifest_sha256': sha256(floor_dir / 'manifest.json'),
                'configuration': cfg, 'source_unchanged': True, 'input_hashes': inputs,
                'limitations': ['Offline static global planning only; no robot execution authorized.',
                    'Floor conditioning is non-rigid; this derivative is not a localization map.',
                    'Visibility is approximate without per-return sensor ID/origin; protected areas are map-specific.',
                    'Missing ceiling returns are unobserved, not certified free space.']}
    (out / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    (out / 'pipeline.yaml').write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    pct = copy.deepcopy(cfg['pct_pipeline'])
    pct.update(schema_version=1, source_pcd=str(target), source_processing_manifest=str(out / 'manifest.json'),
               output_directory=str(out / 'pct'), frame_id=FRAME)
    pct_config = out / 'pct_pipeline.yaml'
    pct_config.write_text(yaml.safe_dump(pct, sort_keys=False))
    pct_dir, _ = build(pct_config)
    assert_unchanged(inputs)
    from d1max_pct_planner.tomogram_map import TomogramMap
    tomo = TomogramMap(pct_dir / 'tomogram.npz', unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    route = load_config(route_template_path)
    for name, xyz in cfg.get('anchor_overrides', {}).items():
        if name not in route['anchors']:
            raise ValueError(f'Unknown portal override: {name}')
        route['anchors'][name]['xyz'] = xyz
    portal_audit = {}
    for name, anchor in route['anchors'].items():
        route['anchors'][name], portal_audit[name] = select_portal(
            tomo, anchor, cfg['portal_search_radius_m'], cfg['portal_height_tolerance_m'])
    route.update(tomogram_path=str(pct_dir / 'tomogram.npz'), source_pcd=str(target),
                 frame_id=FRAME, output_directory=str(out / 'route'))
    route['planning'].update(cfg.get('planning_overrides', {}))
    route_path = out / 'route.yaml'
    route_path.write_text(yaml.safe_dump(route, sort_keys=False))
    (out / 'portal_selection.json').write_text(json.dumps(portal_audit, indent=2))
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(route['vendor_root'])
    env['D1MAX_PCT_CROSSFLOOR_CHILD'] = '1'
    with (out / 'route.log').open('x') as log:
        process = subprocess.run([sys.executable, '-m', 'd1max_pct_planner.crossfloor_route',
                                  '--config', str(route_path)], env=env, stdout=log, stderr=subprocess.STDOUT)
    if process.returncode:
        raise RuntimeError(f'PCT route failed; map retained, no valid path published. See {out / "route.log"}')
    assert_unchanged(inputs)
    print(json.dumps({'status': 'complete', 'output': str(out), 'route': str(out / 'route/audit.json'),
                      'execution_authorized': False}), flush=True)
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    run(parser.parse_args().config)
