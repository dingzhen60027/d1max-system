"""One reproducible YAML -> independent, field-preserving processed PCD.

Usage: PYTHONPATH=tools python3 -m pointcloud_preprocessing.runner --config FILE
The Web result catalog reads the completed manifest, but its legacy processing
runner does not support this standalone schema. No active map is switched.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import time
import uuid

import numpy as np
import yaml

from .algorithms import filter_indices
from .pcd_io import read_pcd, write_subset


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def number(value, low, high, integer=False):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('Parameter must be a finite number')
    if not low <= value <= high or (integer and int(value) != value):
        raise ValueError(f'Parameter outside [{low}, {high}] or not integral')


def validate(config):
    if not isinstance(config, dict) or config.get('schema') != 'd1max.structure_cleaning/v1':
        raise ValueError('Expected d1max.structure_cleaning/v1')
    if not re.fullmatch(r'[a-z0-9][a-z0-9_-]{1,63}', config.get('id', '')):
        raise ValueError('Invalid config id')
    for section, key in [('input', 'path'), ('output', 'root')]:
        if not Path(config[section][key]).is_absolute():
            raise ValueError('Input/output paths must be absolute')
    if not config['input'].get('frame_id'):
        raise ValueError('An explicit preserved frame_id is required')
    if config['output'].get('basename') != 'processed_map.pcd':
        raise ValueError('Expected fixed catalog basename processed_map.pcd')
    for key in ('preserve_all_fields', 'save_removed'):
        if config['output'].get(key) is not True:
            raise ValueError('Field preservation and removed-point archive are required')
    number(config['output']['maximum_removed_fraction'], 0.0, .5)
    number(config['runtime']['workers'], 1, 8, True)
    number(config['runtime']['maximum_points'], 1, 10000000, True)
    specs = {
        'radius_outlier': {'radius_m': (.02, .5, False), 'minimum_other_neighbors': (1, 20, True)},
        'statistical_outlier': {'neighbors': (4, 128, True), 'std_ratio': (1., 10., False)},
        'structure_support': {
            'neighbors': (8, 128, True), 'minimum_neighbors': (4, 64, True),
            'support_radius_m': (.1, 1., False), 'trim_fraction': (.5, 1., False),
            'plane_fit_tolerance_m': (.001, .1, False), 'maximum_surface_variation': (0., .1, False),
            'minimum_plane_spread_m': (.01, .3, False), 'query_to_plane_m': (.001, .05, False),
            'minimum_linearity_fraction': (.9, 1., False), 'minimum_line_span_m': (.05, 1., False),
            'query_to_line_m': (.001, .05, False), 'edge_allowance_m': (0., .1, False)}}
    seen = set()
    for module in config['modules']:
        kind = module.get('type')
        if kind not in specs or kind in seen or not isinstance(module.get('enabled'), bool):
            raise ValueError('Unknown/duplicate module or invalid enabled flag')
        if set(module['parameters']) != set(specs[kind]):
            raise ValueError('Unknown/missing module parameters')
        for key, limits in specs[kind].items():
            number(module['parameters'][key], *limits)
        seen.add(kind)
    if seen != set(specs):
        raise ValueError('All three explicit modules must be present (may be disabled)')
    structure = next(m for m in config['modules'] if m['type'] == 'structure_support')['parameters']
    if structure['minimum_neighbors'] > structure['neighbors']:
        raise ValueError('minimum_neighbors exceeds queried neighbors')
    return config


def run(config_path):
    started = time.monotonic()
    config_path = Path(config_path).resolve()
    config = validate(yaml.safe_load(config_path.read_text()))
    source = Path(config['input']['path']).resolve(strict=True)
    original_hash, config_hash = sha256(source), sha256(config_path)
    if config['input'].get('sha256') and config['input']['sha256'] != original_hash:
        raise ValueError('Source PCD changed; inspect it before running this configuration')
    cloud = read_pcd(source)
    if len(cloud.records) > config['runtime']['maximum_points']:
        raise ValueError('Input exceeds configured point budget')
    result = filter_indices(cloud.xyz, config['modules'], workers=config['runtime']['workers'],
                            progress=lambda phase: print(json.dumps({'stage': phase}), flush=True))
    removed_fraction = len(result['removed_indices']) / len(cloud.records)
    if removed_fraction > config['output']['maximum_removed_fraction']:
        raise ValueError(f'Removal {removed_fraction:.1%} exceeds guard; no result published')
    if not len(result['kept_indices']):
        raise ValueError('No points remain; no result published')
    if sha256(source) != original_hash or sha256(config_path) != config_hash:
        raise ValueError('Input/config changed during filtering')
    created = datetime.now().astimezone().isoformat(timespec='seconds')
    directory = Path(config['output']['root']) / (
        datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+config['id']+'_'+uuid.uuid4().hex[:6])
    directory.mkdir(parents=True, exist_ok=False)
    audit = directory / 'audit'
    audit.mkdir()
    config_copy = {**config, 'resolved_output_directory': str(directory)}
    (directory/'pipeline.yaml').write_text(yaml.safe_dump(config_copy, allow_unicode=True, sort_keys=False))
    output = directory / 'processed_map.pcd'
    fields = write_subset(output, cloud, result['kept_indices'])
    write_subset(audit/'removed_points.pcd', cloud, result['removed_indices'])
    np.savez_compressed(audit/'selection_indices.npz', kept_indices=result['kept_indices'],
                        removed_indices=result['removed_indices'], removed_reasons=result['removed_reasons'],
                        protected_indices=result['protected_indices'])
    # Verify exact record partition; neither intensity nor coordinates are rewritten.
    readback = read_pcd(output)
    if readback.records.tobytes() != cloud.records[result['kept_indices']].tobytes():
        raise RuntimeError('Record preservation check failed; incomplete build not cataloged')
    if sha256(source) != original_hash:
        raise RuntimeError('Source changed; incomplete build not cataloged')
    stats = result['statistics']
    manifest = {
        'schema': 1, 'status': 'complete', 'name': config['name'],
        'note': '结构保留离群清理；密集人物残影未作语义删除，未自动切换导航地图。',
        'created_at': created, 'completed_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'source_id': config['input'].get('source_id'), 'source_name': config['input'].get('source_name'),
        'source_path': str(source), 'source_fields': cloud.fields, 'source_sha256': original_hash,
        'source_unchanged': True, 'frame_id': config['input']['frame_id'],
        'config_id': config['id'], 'config_name': config['name'], 'config_sha256': config_hash,
        'pipeline_file': 'pipeline.yaml', 'pipeline': config_copy, 'output_file': 'processed_map.pcd',
        'input_points': len(cloud.records), 'output_points': len(readback.records), 'output_fields': fields,
        'removed_points': len(result['removed_indices']), 'removed_fraction': removed_fraction,
        'stage_counts': stats['stage_counts'], 'filter_statistics': stats,
        'record_preservation': 'byte_exact_original_records_in_original_order',
        'removed_points_file': 'audit/removed_points.pcd',
        'selection_indices_file': 'audit/selection_indices.npz',
        'output_sha256': sha256(output), 'elapsed_seconds': time.monotonic()-started,
        'implementation_sha256': {p.name: sha256(p) for p in Path(__file__).parent.glob('*.py')},
        'limitations': ['Not a semantic person detector; dense ambiguous objects remain.',
                        'Only static geometry filtering; removed cells are not certified free space.',
                        'No hole filling, global flattening, resampling, or active map switch.'],
    }
    # Complete manifest is the publication boundary recognized by the Web catalog.
    temporary = directory/'manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(directory/'manifest.json')
    print(json.dumps({'directory': str(directory), 'input_points': len(cloud.records),
                      'output_points': len(readback.records), 'removed_points': len(result['removed_indices']),
                      'protected_candidates': stats['protected_candidate_union'],
                      'source_unchanged': True}, ensure_ascii=False), flush=True)
    return directory, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    options = parser.parse_args()
    run(options.config)


if __name__ == '__main__':
    main()
