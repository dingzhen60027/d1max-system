"""One YAML: source PCD -> official-equation CPU PCT tomogram + provenance.

Run: python3 -m d1max_pct_planner.official_pipeline --config CONFIG.yaml
Outputs are exclusively created; an existing build is never overwritten.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import pickle
import shutil
import subprocess
import time

import numpy as np
import yaml

from .cpu_tomography import BACKEND, export_preview, tomogram_from_points, validate_config


REFERENCE_FILES = ['tomography/scripts/tomography.py', 'tomography/scripts/tomogram.py',
                   'tomography/scripts/kernels.py', 'tomography/config/scene_building.py',
                   'tomography/config/scene_spiral.py', 'tomography/config/scene_plaza.py']
PROCESSING_SCHEMA = 'd1max.flat_floor_planning/v1'
PLANNING_FRAME = 'd1max_flat_floor_planning'
GEOMETRY_OPERATION = 'z_column_shift_from_trajectory_supported_floor'


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def unique_output(base):
    base = Path(base).resolve()
    base.parent.mkdir(parents=True, exist_ok=True)
    for sequence in range(10000):
        candidate = base if sequence == 0 else base.with_name(base.name + f'_{sequence:03d}')
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError('Could not allocate a fresh output directory')


def load_config(path):
    from .paths import expand_tree
    config = expand_tree(yaml.safe_load(Path(path).read_text()))
    allowed = {'schema_version', 'source_pcd', 'source_processing_manifest', 'output_directory', 'vendor_root', 'frame_id',
               'preprocessing', 'pct', 'export', 'notes'}
    if not isinstance(config, dict) or set(config) - allowed:
        raise ValueError('Invalid/unknown pipeline configuration keys')
    if config.get('schema_version') != 1:
        raise ValueError('Pipeline schema_version must be 1')
    for name in ('source_pcd', 'vendor_root'):
        target = Path(config[name])
        if not target.is_absolute() or not target.exists():
            raise ValueError(f'{name} must reference an existing absolute path')
    if not Path(config['source_pcd']).is_file():
        raise ValueError('source_pcd must be a file')
    if not Path(config['output_directory']).is_absolute():
        raise ValueError('output_directory must be absolute')
    if not config.get('frame_id'):
        raise ValueError('An explicit frame_id is required; coordinates are not transformed')
    if 'source_processing_manifest' in config:
        processing = Path(config['source_processing_manifest'])
        if not processing.is_absolute() or not processing.is_file():
            raise ValueError('source_processing_manifest must reference an existing absolute file')
    validate_config(config['pct'])
    return config


def source_processing_provenance(config):
    """Validate an explicit planning-only conditioned input; no geometric work."""
    if 'source_processing_manifest' not in config:
        return None
    manifest_path = Path(config['source_processing_manifest'])
    if not manifest_path.is_absolute() or not manifest_path.is_file():
        raise ValueError('source_processing_manifest must reference an existing absolute file')
    manifest_path = manifest_path.resolve()
    manifest_bytes = manifest_path.read_bytes()
    record = json.loads(manifest_bytes)
    if not isinstance(record, dict):
        raise ValueError('Source processing manifest must be a JSON object')
    contracts = {
        PROCESSING_SCHEMA: (GEOMETRY_OPERATION, PLANNING_FRAME),
        'd1max.multifloor_planning/v1': (
            'independent_floor_z_conditioning_with_preserved_stairs', 'd1max_multifloor_planning'),
        'd1max.crossfloor_planning/v1': (
            'independent_floor_conditioning_visibility_filter_and_measured_return_restore',
            'd1max_multifloor_planning'),
    }
    contract = contracts.get(record.get('processing_schema'))
    if (contract is None or record.get('status') != 'complete'
            or record.get('geometry_operation') != contract[0]):
        raise ValueError('Unsupported or incomplete source processing manifest')
    if type(record.get('planning_only')) is not bool or record['planning_only'] is not True:
        raise ValueError('Conditioned input planning_only must be boolean true')
    if record.get('frame_id') != contract[1] or config.get('frame_id') != contract[1]:
        raise ValueError('Conditioned input requires its matching dedicated planning frame')
    output_file = record.get('output_file')
    if not isinstance(output_file, str) or not output_file:
        raise ValueError('Processing manifest output_file must identify its output PCD')
    output = Path(output_file)
    if not output.is_absolute():
        if '..' in output.parts:
            raise ValueError('Processing output_file must not escape its result directory')
        output = manifest_path.parent / output
    source = Path(config['source_pcd']).resolve()
    if output.resolve() != source or not source.is_file():
        raise ValueError('Processing manifest output_file must exactly match source_pcd')
    if record.get('output_sha256') != sha256(source):
        raise ValueError('Conditioned source PCD output_sha256 mismatch')
    original = record.get('source_path')
    if not isinstance(original, str) or not Path(original).is_absolute() or not Path(original).is_file():
        raise ValueError('Processing source_path must reference an existing absolute original PCD')
    original = Path(original).resolve()
    if record.get('source_sha256') != sha256(original):
        raise ValueError('Processing original source_sha256 mismatch')
    return {'manifest_path': str(manifest_path),
            'manifest_sha256': hashlib.sha256(manifest_bytes).hexdigest(),
            'processing_schema': record['processing_schema'], 'planning_only': True,
            'geometry_operation': record['geometry_operation'], 'frame_id': record['frame_id'],
            'output_path': str(source), 'output_sha256': record['output_sha256'],
            'original_source_pcd': str(original), 'original_source_sha256': record['source_sha256']}


def verify_processing_unchanged(provenance):
    if provenance and (sha256(provenance['manifest_path']) != provenance['manifest_sha256']
                       or sha256(provenance['original_source_pcd']) != provenance['original_source_sha256']):
        raise RuntimeError('Source processing manifest/original source changed during build; result is not published')


def preprocess(points, config):
    """Conservative independent options; default removes only nonfinite XYZ."""
    import open3d as o3d
    allowed = {'roi', 'voxel_size_m', 'radius_outlier'}
    if set(config) - allowed:
        raise ValueError('Unknown preprocessing options')
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('Expected XYZ array')
    stages = [{'stage': 'loaded', 'points': len(points)}]
    points = points[np.isfinite(points).all(axis=1)]
    stages.append({'stage': 'finite_xyz', 'points': len(points)})
    roi = config.get('roi')
    if roi is not None:
        if set(roi) != {'min', 'max'}:
            raise ValueError('ROI must provide min/max XYZ')
        low, high = np.asarray(roi['min'], float), np.asarray(roi['max'], float)
        if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or not np.all(low < high):
            raise ValueError('ROI bounds must be finite increasing XYZ')
        points = points[((points >= low) & (points <= high)).all(axis=1)]
    stages.append({'stage': 'explicit_roi', 'enabled': roi is not None, 'points': len(points)})
    voxel = float(config.get('voxel_size_m', 0))
    if not np.isfinite(voxel) or voxel < 0 or voxel > .2:
        raise ValueError('Optional fine voxel_size_m must be in [0, 0.2]')
    if voxel > 0 and len(points):
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        points = np.asarray(cloud.voxel_down_sample(voxel).points, dtype=np.float32)
    stages.append({'stage': 'fine_voxel_average', 'enabled': voxel > 0, 'points': len(points)})
    outlier = config.get('radius_outlier', {'enabled': False})
    if set(outlier) - {'enabled', 'radius_m', 'minimum_neighbors'}:
        raise ValueError('Unknown radius_outlier keys')
    if not isinstance(outlier.get('enabled', False), bool):
        raise ValueError('radius_outlier.enabled must be boolean')
    if outlier.get('enabled', False):
        radius, neighbors = float(outlier['radius_m']), outlier['minimum_neighbors']
        if not np.isfinite(radius) or radius <= 0 or isinstance(neighbors, bool) or int(neighbors) != neighbors or neighbors < 2:
            raise ValueError('Invalid radius outlier parameters')
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        _, indices = cloud.remove_radius_outlier(nb_points=int(neighbors), radius=radius)
        points = points[np.asarray(indices, dtype=int)]
    stages.append({'stage': 'isolated_point_filter', 'enabled': outlier.get('enabled', False), 'points': len(points)})
    if not len(points):
        raise ValueError('No finite points remain; source/config require inspection')
    return points, stages


def hardware_probe():
    result = {'nvidia_smi_available': shutil.which('nvidia-smi') is not None,
              'cupy_installed': importlib.util.find_spec('cupy') is not None,
              'cuda_executed_for_build': False}
    if result['cupy_installed']:
        try:
            import cupy
            result['cupy_version'] = cupy.__version__
            result['cuda_device_count'] = int(cupy.cuda.runtime.getDeviceCount())
        except Exception as exc:
            result['cuda_probe_error'] = f'{type(exc).__name__}: {exc}'
    return result


def build(config_path):
    import open3d as o3d
    started = time.perf_counter()
    config_path = Path(config_path).resolve()
    config = load_config(config_path)
    source, vendor = Path(config['source_pcd']).resolve(), Path(config['vendor_root']).resolve()
    source_hash, config_hash = sha256(source), sha256(config_path)
    processing = source_processing_provenance(config)
    canonical = json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    references = {name: sha256(vendor / name) for name in REFERENCE_FILES}
    version = subprocess.run(['git', '-C', str(vendor), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, check=True).stdout.strip()
    cloud = o3d.io.read_point_cloud(str(source), remove_nan_points=False, remove_infinite_points=False)
    points, stages = preprocess(np.asarray(cloud.points), config.get('preprocessing', {}))
    del cloud
    print(json.dumps({'stage': 'preprocessed', 'points': len(points), 'backend': BACKEND}), flush=True)
    payload, statistics = tomogram_from_points(points, config['pct'])
    payload.update({'source_pcd': str(source), 'source_sha256': source_hash,
                    'config_sha256': config_hash, 'frame_id': config['frame_id'],
                    'schema_version': 1, 'upstream_commit': version})
    if processing:
        payload.update({'source_processing_manifest': processing['manifest_path'],
                        'source_processing_manifest_sha256': processing['manifest_sha256'],
                        'processing_schema': processing['processing_schema'],
                        'planning_only': True, 'geometry_operation': processing['geometry_operation'],
                        'original_source_pcd': processing['original_source_pcd'],
                        'original_source_sha256': processing['original_source_sha256'],
                        'ground_semantics': 'input_derived_surface',
                        'ceiling_semantics': 'input_derived_upper_surface_or_nan_unobserved'})
    if sha256(source) != source_hash or sha256(config_path) != config_hash:
        raise RuntimeError('Source/config changed while building; result is not published')
    verify_processing_unchanged(processing)
    output = unique_output(config['output_directory'])
    npz_path, pickle_path = output / 'tomogram.npz', output / 'tomogram.pickle'
    # Safe loader path contains arrays/scalars/Unicode, never object dtype.
    np.savez_compressed(npz_path, **payload)
    # Official five-layer layout/keys; retain float32 to avoid introducing
    # half-precision height/cost changes. Official native wrapper casts it.
    with pickle_path.open('wb') as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    shutil.copy2(config_path, output / 'pipeline.yaml')
    preview_count = 0
    if config.get('export', {}).get('traversable_pcd', True):
        preview_count = export_preview(payload, output / 'traversable.pcd', traversable_only=True)
    files = {path.name: {'sha256': sha256(path), 'bytes': path.stat().st_size}
             for path in output.iterdir() if path.is_file()}
    manifest = {
        'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'complete', 'backend': BACKEND, 'upstream_commit': version,
        'upstream_reference_sha256': references, 'source_pcd': str(source),
        'source_sha256': source_hash, 'source_unchanged': sha256(source) == source_hash,
        'config_sha256': config_hash, 'canonical_config_sha256': hashlib.sha256(canonical).hexdigest(),
        'source_points': stages[0]['points'], 'used_points': len(points),
        'used_bounds_min': points.min(axis=0).tolist(), 'used_bounds_max': points.max(axis=0).tolist(),
        'preprocessing': stages, 'config': config, 'output_directory': str(output),
        'data_shape': list(payload['data'].shape), 'data_dtype': str(payload['data'].dtype),
        'tensor_channels': ['inflated_cost', 'cost_difference_x', 'cost_difference_y',
                            'input_derived_ground_z' if processing else 'measured_ground_z',
                            'input_derived_ceiling_z' if processing else 'measured_ceiling_z'],
        'frame_id': config['frame_id'], 'minimum_headroom_m': payload['minimum_headroom_m'],
        'preview_traversable_points': preview_count, 'statistics': statistics,
        'hardware': hardware_probe(), 'files': files, 'elapsed_seconds': time.perf_counter() - started,
        'differences_from_upstream_runtime': [
            'NumPy/SciPy CPU implementation of the checked upstream equations; no ROS1/CuPy/CUDA build executed.',
            'Validated against independent scalar kernel tests, not a GPU numerical-equivalence run on this machine.',
            'Finite-value checks, optional explicit conservative preprocessing and bounded allocations added.',
            'NPZ and trusted-local pickle retain float32; upstream export defaults to float16.',
            'Explicit selected_source_layers, source/config hashes and per-stage diagnostics added.',
            'CUDA point rounding follows half-away-from-zero; upstream Python query rounding uses ties-to-even.',
            'A single flat-height input is supported as one slice instead of allocating zero upstream slices.',
        ],
        'limitations': [
            'Offline static geometry only; D1 body/gait/braking envelope has not been physically certified.',
            'No ceiling return is unobserved_above, not verified open sky; raw PCD does not prove free volume.',
            'Unmeasured ground remains NaN and cannot be treated as traversable even if its raw cost is low.',
            ('Input already has non-rigid Z conditioning in a dedicated planning frame; surface heights are input-derived, not original measured ground truth.'
             if processing else 'No hole filling, floor flattening, inferred surfaces or dynamic-object semantic removal.'),
            'Single-floor task may retain several geometric slices; roof/table surfaces require operator floor selection.',
        ],
    }
    if processing:
        manifest.update(source_processing=processing, planning_only=True,
                        processing_schema=processing['processing_schema'],
                        geometry_operation=processing['geometry_operation'],
                        ground_semantics=payload['ground_semantics'],
                        ceiling_semantics=payload['ceiling_semantics'])
        manifest['limitations'].append(
            'Conditioned geometry is planning-only, not a localization reference or original 3D geometry; a single rigid TF cannot undo a spatially varying Z shift.')
    verify_processing_unchanged(processing)
    if sha256(source) != source_hash or sha256(config_path) != config_hash:
        raise RuntimeError('Source/config changed before publication; result is not published')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(output), 'shape': manifest['data_shape'],
                      'selected_source_layers': statistics['selected_source_layers'],
                      'seconds': manifest['elapsed_seconds'], 'source_unchanged': manifest['source_unchanged']}), flush=True)
    return output, manifest


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    options = parser.parse_args(args)
    build(options.config)


if __name__ == '__main__':
    main()
