"""Reuse the proven flat-floor stages independently on explicit building levels.

Levels own disjoint original Z bands. Protected stair volumes retain original
records. Only measured floor support can seed conditioning or clutter removal.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
import yaml

from . import load_config
from .flat_floor import condition_flat_floor
from .flat_floor_runner import structural_cleanup
from .pcd_io import read_pcd, write_derived_z, write_subset
from .runner import sha256

SCHEMA = 'd1max.multifloor_planning/v1'
FRAME = 'd1max_multifloor_planning'
OPERATION = 'independent_floor_z_conditioning_with_preserved_stairs'


def protected_mask(xyz, regions):
    mask = np.zeros(len(xyz), dtype=bool)
    for region in regions:
        low, high = np.asarray(region['min'], float), np.asarray(region['max'], float)
        if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(low >= high):
            raise ValueError('Protected volume requires finite increasing XYZ bounds')
        mask |= ((xyz >= low) & (xyz <= high)).all(axis=1)
    return mask


def condition_levels(xyz, poses, config, defaults):
    original = np.asarray(xyz, dtype=float)
    result = original.copy()
    keep = np.ones(len(original), dtype=bool)
    owner = np.full(len(original), -1, dtype=np.int16)
    protected = protected_mask(original, config['protected_regions'])
    stats, evidence = [], {}
    ranges = []
    for level, floor in enumerate(config['floors']):
        low, high = map(float, floor['input_z_band'])
        if not np.isfinite([low, high]).all() or low >= high:
            raise ValueError('Invalid floor Z ownership band')
        if any(max(low, a) < min(high, b) for a, b in ranges):
            raise ValueError('Floor Z ownership bands must not overlap')
        ranges.append((low, high))
        ids = np.flatnonzero((original[:, 2] >= low) & (original[:, 2] < high) & ~protected)
        begin, end = floor['trajectory_range_inclusive']
        if not 0 <= begin < end < len(poses):
            raise ValueError('Invalid floor trajectory range')
        selected_poses = poses[begin:end + 1]
        selected_poses = selected_poses[~protected_mask(selected_poses, config['protected_regions'])]
        parameters = {**defaults['conditioning'], **config.get('conditioning_overrides', {}),
                      'reference_z_m': float(floor['reference_z_m'])}
        local = condition_flat_floor(original[ids], selected_poses, parameters)
        structural = {**defaults['obstacle_cleanup'], **config.get('obstacle_cleanup_overrides', {}),
                      'floor_reference_z_m': float(floor['reference_z_m'])}
        structural_cleanup(local, structural)
        shift = local['xyz'][:, 2] - original[ids, 2]
        if abs(shift).max(initial=0) > config['maximum_z_shift_m']:
            raise ValueError(f"{floor['id']}: Z correction exceeds configured bound")
        result[ids] = local['xyz']
        keep[ids] = local['keep_mask']
        owner[ids[local['conditioning_mask']]] = level
        stats.append({'id': floor['id'], 'reference_z_m': floor['reference_z_m'],
                      'input_z_band': [low, high], 'statistics': local['statistics']})
        evidence[f'{floor["id"]}_source_indices'] = ids
        evidence[f'{floor["id"]}_floor_z'] = local['floor_z']
        evidence[f'{floor["id"]}_support'] = local['floor_support_mask']
        evidence[f'{floor["id"]}_anchors_xy'] = local['anchors']['xy']
        evidence[f'{floor["id"]}_anchors_z'] = local['anchors']['z']
    if not np.array_equal(result[protected], original[protected]) or not keep[protected].all():
        raise RuntimeError('Protected staircase changed')
    if not np.array_equal(result[:, :2], original[:, :2]):
        raise RuntimeError('Floor conditioning changed XY')
    return result, keep, owner, protected, stats, evidence


def run(config_path):
    config_path = Path(config_path).resolve(strict=True)
    cfg = load_config(config_path)
    if cfg.get('schema') != SCHEMA or cfg.get('planning_only') is not True:
        raise ValueError('Explicit multi-floor planning derivative required')
    source = Path(cfg['source_pcd']).resolve(strict=True)
    trajectory = Path(cfg['trajectory_path']).resolve(strict=True)
    defaults_path = Path(cfg['floor_stage_defaults']).resolve(strict=True)
    defaults = load_config(defaults_path)
    hashes = {str(p): sha256(p) for p in (source, trajectory, config_path, defaults_path)}
    cloud = read_pcd(source)
    if not cloud.finite_xyz_mask.all():
        raise ValueError('Clean nonfinite input records first')
    matrices = np.loadtxt(trajectory, ndmin=2)
    if matrices.shape[1] != 12 or not np.isfinite(matrices).all():
        raise ValueError('Expected KITTI 3x4 poses')
    print(json.dumps({'stage': 'independent_floor_conditioning', 'points': len(cloud.xyz)}), flush=True)
    xyz, keep, owner, protected, stats, evidence = condition_levels(
        cloud.xyz, matrices[:, [3, 7, 11]], cfg, defaults)
    if (~keep).mean() > cfg['maximum_removed_fraction']:
        raise ValueError('Removal fraction exceeds bound')
    out = Path(cfg['output_directory'])
    out.mkdir(parents=True, exist_ok=False)
    (out / 'audit').mkdir()
    selected = np.flatnonzero(keep)
    target = out / 'processed_map.pcd'
    write_derived_z(target, cloud, selected, xyz[selected, 2])
    write_subset(out / 'audit/removed_clutter.pcd', cloud, ~keep)
    np.savez_compressed(out / 'audit/floor_evidence.npz', **evidence,
                        source_indices=selected, conditioning_owner=owner,
                        protected_stairs=protected, removed_indices=np.flatnonzero(~keep))
    readback = read_pcd(target)
    for field in cloud.fields:
        if field != 'z' and readback.records[field].tobytes() != cloud.records[field][selected].tobytes():
            raise RuntimeError('A non-Z field changed')
    for path, digest in hashes.items():
        if sha256(path) != digest:
            raise RuntimeError('Source or configuration changed during processing')
    manifest = {'schema': 1, 'processing_schema': SCHEMA, 'status': 'complete',
                'geometry_operation': OPERATION, 'planning_only': True, 'frame_id': FRAME,
                'created_at': datetime.now(timezone.utc).isoformat(),
                'source_path': str(source), 'source_sha256': hashes[str(source)],
                'source_unchanged': True, 'trajectory_path': str(trajectory),
                'trajectory_sha256': hashes[str(trajectory)],
                'output_file': 'processed_map.pcd', 'output_sha256': sha256(target),
                'input_points': len(cloud.xyz), 'output_points': len(selected),
                'removed_points': int((~keep).sum()), 'synthetic_points': 0,
                'protected_stair_points_unchanged': int(protected.sum()),
                'floor_statistics': stats, 'configuration': cfg,
                'resolved_floor_defaults': defaults, 'input_hashes': hashes,
                'coordinate_relation': 'Original XY; each declared flat level has a bounded local Z correction. Stairs unchanged; no global rigid TF.'}
    (out / 'pipeline.yaml').write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    (out / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    pct = {**defaults['pct_pipeline'], **cfg['pct_pipeline'], 'schema_version': 1,
           'source_pcd': str(target), 'frame_id': FRAME,
           'source_processing_manifest': str(out / 'manifest.json')}
    pct_path = out / 'pct_pipeline.yaml'
    pct_path.write_text(yaml.safe_dump(pct, allow_unicode=True, sort_keys=False))
    from d1max_pct_planner.official_pipeline import build
    pct_dir, _ = build(pct_path)
    print(json.dumps({'output': str(out), 'pct_output': str(pct_dir),
                      'removed': int((~keep).sum()), 'floors': stats}, ensure_ascii=False), flush=True)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    run(parser.parse_args().config)


if __name__ == '__main__':
    main()
