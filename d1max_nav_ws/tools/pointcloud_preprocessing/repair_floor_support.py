"""Offline, record-preserving repair of an existing conditioned map package.

Re-evaluate the embedded floor-stage configuration against its exact original
PCD/poses, retain the audited final record selection and measured-return tail,
then rebuild PCT with unchanged equations/settings. Never modify the input
package, localization map, active selector or physical eligibility flags.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil

import numpy as np
import yaml

from .multifloor_runner import condition_levels
from .pcd_io import read_pcd, _write_records
from .runner import sha256
from . import reviewed_flat_corridor


def _json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def _validate_evidence_repair(old_evidence, evidence):
    """Compare old evidence with the newly computed original-record evidence."""
    for key, before in old_evidence.items():
        if key.endswith(('_support', '_snapped_support')) and key in evidence:
            # The only new membership is metric confirmation from the same
            # existing observed support, after all obstacle protection.
            floor = key.split('_', 1)[0]
            allowed = evidence[floor + '_metric_support_added'].copy()
            if key.endswith('_snapped_support'):
                allowed |= evidence.get(floor + '_reviewed_floor_prior', False)
            after = evidence[key]
            if np.any(before & ~after) or np.any((after & ~before) & ~allowed):
                raise ValueError('Repair changed classification outside declared floor repair: ' + key)
        elif key.endswith('_reviewed_floor_prior'):
            # A later explicit review may add corrected records when replayed
            # against the same ORIGINAL points. It cannot erase the previous
            # review or turn those corrections into measured floor support.
            # `evidence` is the current condition_levels result, not a merged
            # mask read from a previous package or the user review document.
            after = evidence.get(key)
            snapped = evidence.get(key.removesuffix('_reviewed_floor_prior') + '_snapped_support')
            if (after is None or snapped is None or before.dtype.kind != 'b'
                    or after.dtype.kind != 'b' or snapped.dtype.kind != 'b'
                    or before.shape != after.shape or after.shape != snapped.shape
                    or np.any(before & ~after) or np.any(after & ~snapped)):
                raise ValueError('Repair changed prior floor evidence: ' + key)
        elif key in evidence and not np.array_equal(evidence[key], before, equal_nan=True):
            raise ValueError('Repair changed prior floor evidence: ' + key)


def repair(package, lineage, output, reviewed_flat_corridors=None):
    from d1max_pct_scan.navigation_map_package import validate_package
    from d1max_pct_planner.official_pipeline import build

    package, lineage = Path(package).resolve(strict=True), Path(lineage).resolve(strict=True)
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('Candidate output must not exist')
    checked = validate_package(package)
    manifest = copy.deepcopy(checked['manifest'])
    stage_path = Path(manifest['floor_processing_manifest'])
    stage = json.loads(stage_path.read_text())
    evidence_path = stage_path.parent / 'audit/floor_evidence.npz'
    stage_pcd = stage_path.parent / stage['output_file']
    if sha256(stage_pcd) != stage['output_sha256']:
        raise ValueError('Original floor-stage PCD changed')
    inputs = {**checked['dependency_files'], str(stage_pcd): sha256(stage_pcd),
              str(lineage): sha256(lineage)}
    if reviewed_flat_corridors is not None:
        review_path = Path(reviewed_flat_corridors).resolve(strict=True)
        review = reviewed_flat_corridor.validate(json.loads(review_path.read_text()), manifest['source_sha256'])
        inputs[str(review_path)] = sha256(review_path)
        route_config = yaml.safe_load((package / 'route.yaml').read_text())
        roi = route_config['stair_roi']
        low, high = roi['min'], roi['max']
        corners = np.array([[low[0], low[1]], [low[0], high[1]],
                            [high[0], low[1]], [high[0], high[1]]])
        if not any(reviewed_flat_corridor.polygon_mask(corners, polygon).all()
                   for polygon in review['stair_exclusion_xy']):
            raise ValueError('Reviewed stair exclusions must cover route stair ROI')
        stage['configuration']['reviewed_flat_corridors'] = review
    # Default-package paths are mutable across deployment. Freeze their exact
    # old bytes for replay/audit; retain original names only as captured labels.
    before_dir = output.parent / 'before'
    stable_copies = {}
    for name in ('manifest.json', 'processed_map.pcd', 'route.yaml', 'pct/tomogram.npz'):
        original, frozen = package / name, before_dir / name
        frozen.parent.mkdir(parents=True, exist_ok=True)
        if frozen.exists():
            if sha256(frozen) != sha256(original):
                raise ValueError('Frozen before evidence differs from this input: ' + name)
        else:
            shutil.copy2(original, frozen)
        stable_copies[str(original)] = {'path': str(frozen), 'sha256': sha256(frozen)}
    cloud = read_pcd(manifest['source_path'])
    old_stage = read_pcd(stage_pcd)
    old_final = read_pcd(package / manifest['output_file'])
    with np.load(evidence_path, allow_pickle=False) as archive:
        old_evidence = {key: archive[key] for key in archive.files}
    with np.load(lineage, allow_pickle=False) as archive:
        line = {key: archive[key] for key in archive.files}
    retained = line['retained_source_indices']
    selected = old_evidence['source_indices']
    if (retained.dtype.kind not in 'iu' or len(np.unique(retained)) != len(retained)
            or (retained < 0).any() or (retained >= len(cloud.xyz)).any()
            or len(old_final.xyz) - len(retained) != manifest['restored_measured_points']):
        raise ValueError('Final record lineage is inconsistent')
    for field in cloud.fields:
        if field != 'z' and not np.array_equal(old_final.records[field][:len(retained)], cloud.records[field][retained]):
            raise ValueError('Final retained records are not the declared original source')
    offsets = np.searchsorted(selected, retained)
    if (offsets >= len(selected)).any() or not np.array_equal(selected[offsets], retained):
        raise ValueError('Final retained record missing from floor stage')
    if not np.array_equal(old_final.records[:len(retained)], old_stage.records[offsets]):
        raise ValueError('Final retained records differ from audited floor stage')
    poses = np.loadtxt(stage['trajectory_path'], ndmin=2)
    print(json.dumps({'stage': 'reclassify_same_original_records', 'points': len(cloud.xyz)}), flush=True)
    xyz, keep, owner, protected, stats, evidence = condition_levels(
        cloud.xyz, poses[:, [3, 7, 11]], stage['configuration'], stage['resolved_floor_defaults'],
        source_sha256=manifest['source_sha256'])
    if not np.array_equal(np.flatnonzero(keep), selected):
        raise ValueError('Repair changed record retention; refusing to publish')
    _validate_evidence_repair(old_evidence, evidence)
    if not np.array_equal(owner, old_evidence['conditioning_owner']) or not np.array_equal(protected, old_evidence['protected_stairs']):
        raise ValueError('Repair changed floor ownership or protected stairs')
    # No new record classification, deletion, restoration or XY manipulation.
    records = old_final.records.copy()
    records['z'][:len(retained)] = xyz[retained, 2]
    old_z = old_final.records['z']
    changed = records['z'] != old_z
    if not changed.any():
        raise ValueError('No classified support skirt was repaired')
    if changed[len(retained):].any() or changed[:len(retained)][protected[retained]].any():
        raise ValueError('Repair touched measured restoration or protected stairs')
    classified_snaps = np.zeros(len(cloud.xyz), dtype=bool)
    for floor in stage['configuration']['floors']:
        name = floor['id']
        classified_snaps[evidence[name + '_source_indices']] = evidence[name + '_snapped_support']
    if (changed[:len(retained)] & ~classified_snaps[retained]).any():
        raise ValueError('Repair changed Z outside classified support')
    new_stage = old_stage.records.copy()
    new_stage['z'] = xyz[selected, 2]
    output.mkdir(parents=True)
    if reviewed_flat_corridors is not None:
        _json(output / 'reviewed_flat_corridors.json', review)
    floor_dir = output / 'floor_stage'
    (floor_dir / 'audit').mkdir(parents=True)
    _write_records(floor_dir / 'processed_map.pcd', cloud, new_stage, 'classified floor support consistency repair', reset_viewpoint=True)
    np.savez_compressed(floor_dir / 'audit/floor_evidence.npz', **evidence,
                        source_indices=selected, conditioning_owner=owner,
                        protected_stairs=protected, removed_indices=np.flatnonzero(~keep))
    stage.update(output_sha256=sha256(floor_dir / 'processed_map.pcd'), floor_statistics=stats,
                 support_consistency_repair={'previous_manifest': str(stage_path),
                     'previous_manifest_sha256': sha256(stage_path),
                     'policy': 'classified_observed_support_with_metric_gaps_and_explicit_reviewed_prior'
                         if reviewed_flat_corridors is not None else
                         'classified_observed_support_after_obstacle_protection_with_metric_gaps'})
    stage['configuration']['output_directory'] = str(floor_dir)
    _json(floor_dir / 'manifest.json', stage)
    _write_records(output / 'processed_map.pcd', cloud, records, 'record-preserving classified floor support repair', reset_viewpoint=True)
    shutil.copy2(lineage, output / 'record_lineage.npz')
    manifest.update(output_sha256=sha256(output / 'processed_map.pcd'),
                    floor_processing_manifest=str(floor_dir / 'manifest.json'),
                    floor_processing_manifest_sha256=sha256(floor_dir / 'manifest.json'),
                    support_consistency_repair={'previous_manifest': str(package / 'manifest.json'),
                        'previous_manifest_sha256': sha256(package / 'manifest.json'),
                        'changed_retained_z_records': int(changed.sum()),
                        'retention_xy_non_z_and_restored_records_unchanged': True,
                        'captured_at_build_inputs': inputs,
                        'stable_before_copies': stable_copies,
                        'history_semantics': 'Captured source paths are historical labels, not current dependencies; stable_before_copies preserve replaced package bytes.'})
    manifest['support_consistency_repair']['previous_manifest'] = str(before_dir / 'manifest.json')
    if reviewed_flat_corridors is not None:
        manifest['support_consistency_repair']['reviewed_flat_corridors'] = {
            'file': 'reviewed_flat_corridors.json',
            'sha256': sha256(output / 'reviewed_flat_corridors.json'),
            'scope': 'static_planning_map_only',
            'original_measured_support_membership_unchanged_by_prior': True}
    manifest['configuration']['floor_evidence_sha256'] = sha256(floor_dir / 'audit/floor_evidence.npz')
    manifest['configuration']['output_directory'] = str(output)
    _json(output / 'manifest.json', manifest)
    pct = copy.deepcopy(manifest['configuration']['pct_pipeline'])
    pct.update(schema_version=1, source_pcd=str(output / 'processed_map.pcd'),
               source_processing_manifest=str(output / 'manifest.json'),
               output_directory=str(output / 'pct'), frame_id=manifest['frame_id'])
    pct_path = output / 'pct_pipeline.yaml'
    pct_path.write_text(yaml.safe_dump(pct, sort_keys=False))
    build(pct_path)
    route = yaml.safe_load((package / 'route.yaml').read_text())
    route.update(source_pcd=str(output / 'processed_map.pcd'), tomogram_path=str(output / 'pct/tomogram.npz'),
                 output_directory=str(output / 'route'))
    (output / 'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False))
    for path, digest in inputs.items():
        if sha256(path) != digest:
            raise ValueError('Input changed during repair: ' + path)
    result = validate_package(output)
    report = {'candidate': str(output), 'changed_records': int(changed.sum()),
              'floors': stats, 'source_localization_unchanged': True,
              'record_count_unchanged': len(records), 'capabilities': result['capabilities'],
              'dependency_files': result['dependency_files']}
    _json(output / 'support_repair_audit.json', report)
    print(json.dumps(report, ensure_ascii=False), flush=True)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', required=True, type=Path)
    parser.add_argument('--lineage', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--reviewed-flat-corridors', type=Path)
    repair(**vars(parser.parse_args()))
