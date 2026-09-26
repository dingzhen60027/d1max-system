"""Scene-specific flat-floor prior -> independent planning PCD -> official PCT.

This is not a SLAM/calibration fix and does not replace a localization map.
No synthetic ground points, ROS, controls or active-map changes are performed.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np
import yaml

from .pcd_io import read_pcd, write_subset, write_derived_z
from .runner import sha256


SCHEMA = 'd1max.flat_floor_planning/v1'
OPERATION = 'z_column_shift_from_trajectory_supported_floor'
FRAME = 'd1max_flat_floor_planning'


def structural_cleanup(result, config):
    """Apply an independent, auditable obstacle stage in conditioned coordinates."""
    from .structural_obstacles import filter_structural_obstacles
    indices = np.flatnonzero(result['planning_mask'])
    stage = filter_structural_obstacles(result['xyz'][indices], config)
    if stage['config']['floor_reference_z_m'] != result['config']['reference_z_m']:
        raise ValueError('Obstacle cleanup and floor conditioning must use the same floor reference')
    removed = indices[stage['removed_mask']]
    result['keep_mask'][removed] = False
    result['planning_mask'][removed] = False
    result['statistics']['structural_obstacle_cleanup'] = stage['statistics']
    result['statistics']['planning_points_before_structural_cleanup'] = len(indices)
    result['statistics']['planning_output_points'] = int(result['planning_mask'].sum())
    result['statistics']['total_removed_points'] = int((~result['keep_mask']).sum())
    return indices, stage


def validate(config):
    if not isinstance(config, dict) or config.get('schema') != SCHEMA:
        raise ValueError('Expected '+SCHEMA)
    if config.get('prior', {}).get('single_level_flat_floor_confirmed') is not True:
        raise ValueError('This method requires an explicit single-level flat-floor prior')
    if config['output'].get('frame_id') != FRAME or config['output'].get('planning_only') is not True:
        raise ValueError('Use the separate planning-only derived frame, never the localization frame')
    if config['input']['frame_id'] == FRAME:
        raise ValueError('Input must be the original geometry, not an already conditioned derivative')
    for value in (config['input']['path'], config['trajectory']['path'], config['output']['directory']):
        if not Path(value).is_absolute():
            raise ValueError('Input, trajectory and output paths must be absolute')
    for name in ('maximum_removed_fraction', 'maximum_unsupported_fraction'):
        value = config['output'][name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value < 1:
            raise ValueError('Invalid '+name)
    if config['trajectory'].get('format') != 'kitti_3x4':
        raise ValueError('Expected KITTI row-major 3x4 trajectory')
    return config


def run(config_path, *, pcd_only=False):
    from .flat_floor import condition_flat_floor
    started = time.monotonic()
    config_path = Path(config_path).resolve(strict=True)
    cfg = validate(yaml.safe_load(config_path.read_text()))
    source = Path(cfg['input']['path']).resolve(strict=True)
    trajectory_path = Path(cfg['trajectory']['path']).resolve(strict=True)
    hashes = {str(p): sha256(p) for p in (source, trajectory_path, config_path)}
    implementation_hashes = {str(p): sha256(p) for p in Path(__file__).parent.glob('*.py')}
    for key, path in (('input', source), ('trajectory', trajectory_path)):
        if cfg[key].get('sha256') and cfg[key]['sha256'] != hashes[str(path)]:
            raise ValueError(key+' hash differs; inspect the changed input before reuse')
    cloud = read_pcd(source)
    if not cloud.finite_xyz_mask.all():
        raise ValueError('Explicitly clean nonfinite XYZ before conditioning')
    if len(cloud.xyz) > cfg['runtime']['maximum_points']:
        raise ValueError('Point budget exceeded')
    matrices = np.loadtxt(trajectory_path, ndmin=2)
    if matrices.shape[1] != 12 or not np.isfinite(matrices).all():
        raise ValueError('Expected finite KITTI 3x4 trajectory rows')
    poses = matrices[:, [3, 7, 11]]
    print(json.dumps({'stage': 'trajectory_supported_floor', 'points': len(cloud.xyz),
                      'trajectory_poses': len(poses)}), flush=True)
    result = condition_flat_floor(cloud.xyz, poses, cfg['conditioning'])
    structural_stage = None
    if cfg.get('obstacle_cleanup') is not None:
        print(json.dumps({'stage': 'structural_obstacle_cleanup'}), flush=True)
        structural_indices, structural_stage = structural_cleanup(result, cfg['obstacle_cleanup'])
    selected = np.flatnonzero(result['planning_mask'])
    clutter = np.flatnonzero(~result['keep_mask'])
    unsupported = np.flatnonzero(result['keep_mask'] & ~result['conditioning_mask'])
    below_floor = np.flatnonzero(result['excluded_below_floor_mask'])
    if not len(selected) or len(clutter) / len(cloud.xyz) > cfg['output']['maximum_removed_fraction']:
        raise ValueError('Empty result or clutter-removal guard exceeded; nothing published')
    if len(unsupported) / len(cloud.xyz) > cfg['output']['maximum_unsupported_fraction']:
        raise ValueError('Unsupported-region guard exceeded; nothing published')
    if not np.array_equal(result['xyz'][:, :2], cloud.xyz[:, :2]):
        raise ValueError('Conditioner must not alter XY or synthesize records')
    for path, digest in {**hashes, **implementation_hashes}.items():
        if sha256(path) != digest:
            raise ValueError('Input/config changed while conditioning')
    directory = Path(cfg['output']['directory'])
    directory.mkdir(parents=True, exist_ok=False)
    audit = directory/'audit'
    audit.mkdir()
    (directory/'pipeline.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
    output = directory/'processed_map.pcd'
    write_derived_z(output, cloud, selected, result['xyz'][selected, 2])
    write_subset(audit/'removed_near_ground_points.pcd', cloud, clutter)
    write_subset(audit/'excluded_unsupported_points.pcd', cloud, unsupported)
    write_subset(audit/'excluded_below_floor_points.pcd', cloud, below_floor)
    if structural_stage is not None:
        write_subset(audit/'removed_structurally_unsupported_points.pcd', cloud,
                     structural_indices[structural_stage['removed_mask']])
        np.savez_compressed(audit/'structural_obstacle_classification.npz',
                            source_indices=structural_indices, **{
                                key: value for key, value in structural_stage.items()
                                if isinstance(value, np.ndarray)})
        (audit/'structural_obstacle_stage.json').write_text(json.dumps({
            'config': structural_stage['config'], 'statistics': structural_stage['statistics'],
            'coordinate_space': FRAME, 'original_records_archived': True}, indent=2)+'\n')
    np.savez_compressed(audit/'conditioning_evidence.npz', source_indices=selected,
                        removed_indices=clutter, unsupported_indices=unsupported,
                        excluded_below_floor_indices=below_floor,
                        original_floor_z=result['floor_z'],
                        floor_support_mask=result['floor_support_mask'],
                        conditioning_mask=result['conditioning_mask'],
                        planning_mask=result['planning_mask'])
    np.savez_compressed(audit/'near_ground_classification.npz',
                        near_ground_candidates=result['near_ground_candidate_mask'],
                        protected_points=result['near_ground_protected_mask'],
                        removed_fragment_points=result['removed_fragment_mask'])
    np.savez_compressed(audit/'floor_anchors.npz', **{
        k: v for k, v in result['anchors'].items() if isinstance(v, np.ndarray)})
    (audit/'anchor_failures.json').write_text(json.dumps(result['anchors'].get('failures', {})))
    readback = read_pcd(output)
    if len(readback.xyz) != len(selected):
        raise RuntimeError('Output point count mismatch')
    for name in cloud.fields:
        if name != 'z' and readback.records[name].tobytes() != cloud.records[name][selected].tobytes():
            raise RuntimeError('A non-Z field changed: '+name)
    if not np.array_equal(readback.records['z'], result['xyz'][selected, 2].astype(cloud.records.dtype['z'])):
        raise RuntimeError('Output Z differs from the audited derivative')
    for path, digest in {**hashes, **implementation_hashes}.items():
        if sha256(path) != digest:
            raise RuntimeError('Input changed; no complete manifest published')
    created = datetime.now().astimezone().isoformat(timespec='seconds')
    manifest = {
        'schema': 1, 'processing_schema': SCHEMA, 'status': 'complete',
        'name': cfg['name'], 'note': '单层平地先验规划副本；保留原图，不可替代定位匹配PCD。',
        'created_at': created, 'completed_at': created,
        'source_path': str(source), 'source_sha256': hashes[str(source)],
        'source_id': cfg['input'].get('source_id'), 'source_name': cfg['input'].get('source_name'),
        'source_unchanged': True, 'source_frame_id': cfg['input']['frame_id'],
        'trajectory_path': str(trajectory_path), 'trajectory_sha256': hashes[str(trajectory_path)],
        'config_id': cfg['id'], 'config_name': cfg['name'], 'config_sha256': hashes[str(config_path)],
        'pipeline_file': 'pipeline.yaml', 'pipeline': cfg, 'output_file': 'processed_map.pcd',
        'output_sha256': sha256(output), 'frame_id': FRAME, 'planning_only': True,
        'geometry_operation': OPERATION, 'synthetic_points': 0,
        'input_points': len(cloud.records), 'output_points': len(selected),
        'removed_points': len(clutter), 'excluded_unsupported_points': len(unsupported),
        'structural_cleanup_removed_points': (int(structural_stage['removed_mask'].sum())
                                              if structural_stage is not None else 0),
        'excluded_below_floor_points': len(below_floor),
        'archived_original_records': True,
        'record_preservation': 'original_xy_and_all_non_z_fields; selected_original_records_only',
        'coordinate_relation': 'XY unchanged; Z is spatially conditioned. No rigid TF exists.',
        'viewpoint_policy': 'identity: no single sensor viewpoint exists for a nonrigid derivative',
        'filter_statistics': result['statistics'],
        'output_fields': readback.fields, 'source_fields': cloud.fields,
        'elapsed_seconds': time.monotonic()-started,
        'limitations': [
            'Uses user-confirmed single-level flat-floor prior; not suitable unchanged for slopes or stairs.',
            'Does not correct SLAM poses, sensor calibration or time synchronization.',
            'Trajectory seeds floor estimation; proximity to trajectory alone never creates free ground.',
            'No hole filling or synthetic floor; unsupported regions excluded and archived, not free.',
            'Conservative static geometry filtering is not semantic human removal.',
            'The optional structural stage removes unsupported low returns over measured floor; '
            'sparse real low objects can be ambiguous and need live obstacle sensing before navigation.',
            'Do not publish a rigid TF to the original 3D localization frame or use for 3D scan matching.',
            'Offline planning artifact only, not a clearance/locomotion certification.',
        ],
        'implementation_sha256': {Path(p).name: digest for p, digest in implementation_hashes.items()},
    }
    (audit/'quality_audit.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temp = directory/'manifest.json.tmp'
    temp.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temp.replace(directory/'manifest.json')
    print(json.dumps({'stage': 'pcd_complete', 'directory': str(directory),
                      'output_points': len(selected), 'clutter_removed': len(clutter),
                      'unsupported_archived': len(unsupported), 'statistics': result['statistics']},
                     ensure_ascii=False, allow_nan=False), flush=True)
    if not pcd_only:
        from d1max_pct_planner.official_pipeline import build
        pct_cfg = {**cfg['pct_pipeline'], 'schema_version': 1,
                   'source_pcd': str(output), 'frame_id': FRAME,
                   'source_processing_manifest': str(directory/'manifest.json')}
        pct_path = directory/'pct_pipeline.yaml'
        pct_path.write_text(yaml.safe_dump(pct_cfg, sort_keys=False, allow_unicode=True))
        pct_output, _ = build(pct_path)
        (directory/'pct_result.json').write_text(json.dumps({'directory': str(pct_output)}, indent=2)+'\n')
    return directory, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--pcd-only', action='store_true')
    args = parser.parse_args()
    run(args.config, pcd_only=args.pcd_only)


if __name__ == '__main__':
    main()
