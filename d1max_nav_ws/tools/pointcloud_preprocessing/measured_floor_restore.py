"""Verify real multi-frame returns before adding conditioned planning-floor points.

The raw scan identity, optimized pose and original conditioning field are all
re-evaluated. Only Z is derived; XY and non-coordinate fields come from a real
source return. This module is read-only and does not alter planner costs.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .flat_floor import _anchors, _config, _field
from .multifloor_runner import protected_mask
from .pcd_io import read_pcd
from .runner import sha256


def _integer(value, label):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{label} must be a nonnegative integer')
    return value


def _xyz(value, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f'{label} must be finite XYZ')
    return result


def _bind(inputs, path, expected=None):
    canonical = str(Path(path).resolve(strict=True))
    digest = sha256(canonical)
    if expected is not None and digest != expected:
        raise ValueError(f'Measured floor input hash mismatch: {canonical}')
    if canonical in inputs and inputs[canonical] != digest:
        raise ValueError(f'Measured floor input changed: {canonical}')
    inputs[canonical] = digest
    return Path(canonical)


def _floor_field(cloud, manifest, audit, poses):
    config = manifest['configuration']
    floor = next(item for item in config['floors'] if item['id'] == 'floor1')
    parameters = _config({**manifest['resolved_floor_defaults']['conditioning'],
                          **config.get('conditioning_overrides', {}),
                          'reference_z_m': float(floor['reference_z_m'])})
    start, end = floor['trajectory_range_inclusive']
    if not 0 <= start < end < len(poses):
        raise ValueError('Floor evidence has an invalid trajectory range')
    positions = poses[start:end+1, :, 3]
    positions = positions[~protected_mask(positions, config['protected_regions'])]
    indices = np.asarray(audit['floor1_source_indices'])
    if (indices.dtype.kind not in 'iu' or indices.ndim != 1 or not len(indices)
            or np.any(indices >= len(cloud.xyz)) or np.any(indices < 0)
            or len(np.unique(indices)) != len(indices)):
        raise ValueError('Invalid floor1 source record identity')
    anchors = _anchors(cloud.xyz[indices], positions, parameters)
    for name, key in [('xy', 'floor1_anchors_xy'), ('z', 'floor1_anchors_z')]:
        if anchors[name].shape != audit[key].shape or not np.allclose(anchors[name], audit[key], rtol=0, atol=1e-8):
            raise ValueError('Reconstructed floor anchors differ from conditioned map')
    field = audit['floor1_floor_z']
    if field.shape != indices.shape:
        raise ValueError('Invalid saved floor field shape')
    # Anchors are fully recomputed. This additional check binds their convention
    # to the saved field; output points use the recomputed field, never this cache.
    check = np.flatnonzero(np.isfinite(field))[::1000]
    if not len(check) or not np.allclose(_field(cloud.xyz[indices[check], :2], anchors, parameters),
                                       field[check], rtol=0, atol=1e-8):
        raise ValueError('Reconstructed floor field differs from conditioned map')
    return anchors, parameters, config['protected_regions'], (start, end)


def restore_conditioned_records(cloud, evidence_path, sources_path, floor_manifest_path):
    """Return ``(records, normalized_evidence, input_hashes)`` after verification.

    All supporting frames are read once in batches. The selected source record's
    ancillary fields are preserved. Returned evidence retains raw/derived XYZ
    and uses ``source_frame_id`` / ``source_point_index`` for common lineage.
    """
    inputs = {}
    evidence_path = _bind(inputs, evidence_path)
    sources_path = _bind(inputs, sources_path)
    manifest_path = _bind(inputs, floor_manifest_path)
    sources = json.loads(sources_path.read_text())
    evidence = json.loads(evidence_path.read_text())
    manifest = json.loads(manifest_path.read_text())
    if not isinstance(evidence, list) or not 0 < len(evidence) <= 100_000:
        raise ValueError('Expected a bounded nonempty measured-floor evidence list')
    source = _bind(inputs, sources['source_pcd'], sources['source_sha256'])
    if (source != Path(cloud.metadata['source_path']).resolve()
            or manifest['source_sha256'] != inputs[str(source)]):
        raise ValueError('Measured floor evidence belongs to another source map')
    if manifest_path != Path(sources['conditioned_manifest']).resolve(strict=True):
        raise ValueError('Measured floor evidence belongs to another floor manifest')
    _bind(inputs, manifest_path, sources['conditioned_manifest_sha256'])
    audit_path = _bind(inputs, sources['floor_evidence'], sources['floor_evidence_sha256'])
    if audit_path != manifest_path.parent/'audit/floor_evidence.npz':
        raise ValueError('Floor field lineage is outside the declared conditioning output')
    poses_path = _bind(inputs, sources['optimized_poses'], sources['optimized_poses_sha256'])
    if poses_path != Path(manifest['trajectory_path']).resolve(strict=True):
        raise ValueError('Restoration pose graph differs from floor conditioning')
    for path, expected in manifest.get('input_hashes', {}).items():
        _bind(inputs, path, expected)
    poses = np.loadtxt(poses_path, ndmin=2)
    if poses.shape[1] != 12 or not np.isfinite(poses).all():
        raise ValueError('Expected finite KITTI 3x4 optimized poses')
    poses = poses.reshape(-1, 3, 4)
    with np.load(audit_path, allow_pickle=False) as audit:
        anchors, parameters, protected, frame_range = _floor_field(cloud, manifest, audit, poses)
    if sources['parameters'] != parameters:
        raise ValueError('Measured return conditioning parameters differ from map')
    rule = sources['rule']
    if (_integer(rule['minimum_frames'], 'minimum_frames') < 5
            or not 0 < float(rule['max_height_range_m']) <= .08
            or not 0 < float(rule['max_field_residual_m']) <= .055
            or rule.get('snap_to_plane') is not False or rule.get('trajectory_clearing') is not False):
        raise ValueError('Measured floor recovery must keep the reviewed multi-frame contract')
    if len(evidence) != _integer(sources['restored_cells'], 'restored_cells'):
        raise ValueError('Restored count differs from source provenance')
    scans = {}
    for scan in sources['scans']:
        frame = _integer(scan['frame'], 'scan frame')
        path = _bind(inputs, scan['scan_path'], scan['scan_sha256'])
        if path != poses_path.parent/'Scans'/f'{frame:06d}.pcd' or frame in scans:
            raise ValueError('Invalid or duplicate supporting scan identity')
        scans[frame] = path
    groups, normalized, selected_ids, cells = {}, [], set(), set()
    for item_index, item in enumerate(evidence):
        item = dict(item)
        frame = _integer(item['source_frame'], 'source_frame')
        index = _integer(item['source_point_index'], 'source_point_index')
        if (frame, index) in selected_ids:
            raise ValueError('Duplicated selected measured return')
        selected_ids.add((frame, index))
        cell = tuple(_integer(value, 'cell') for value in item['cell'])
        if len(cell) != 2 or cell in cells:
            raise ValueError('Duplicate/invalid restored floor cell')
        cells.add(cell)
        rows = item['independent_observations']
        frames = [_integer(row['frame'], 'support frame') for row in rows]
        if (len(set(frames)) != len(frames) or len(frames) < rule['minimum_frames']
                or frames != item['support_frames'] or frame not in frames):
            raise ValueError('Restoration lacks independent measured support')
        for row in rows:
            support_frame = row['frame']
            if support_frame not in scans or not frame_range[0] <= support_frame <= frame_range[1]:
                raise ValueError('Supporting return is outside the verified floor trajectory')
            point_index = _integer(row['scan_point_index'], 'scan_point_index')
            groups.setdefault(support_frame, []).append((item_index, point_index, row))
        item['source_frame_id'] = frame
        item['conditioning_operation'] = 'raw_world_z_minus_verified_floor_field_plus_reference'
        item['verified_support'] = []
        normalized.append(item)
    output = np.empty(len(evidence), dtype=cloud.records.dtype)
    selected_seen = np.zeros(len(evidence), bool)
    for frame, references in groups.items():
        scan = read_pcd(scans[frame])
        if scan.records.dtype != cloud.records.dtype:
            raise ValueError('Measured scan fields differ from source map')
        indices = np.array([ref[1] for ref in references], dtype=np.int64)
        if np.any(indices >= len(scan.xyz)):
            raise ValueError('Measured scan point index is outside scan')
        world = scan.xyz[indices] @ poses[frame, :, :3].T + poses[frame, :, 3]
        field = _field(world[:, :2], anchors, parameters)
        derived = world.copy()
        derived[:, 2] += parameters['reference_z_m']-field
        if (not np.isfinite(derived).all()
                or np.any(np.abs(world[:, 2]-field) > rule['max_field_residual_m']+1e-8)
                or protected_mask(world, protected).any() or protected_mask(derived, protected).any()):
            raise ValueError('Measured return lacks floor evidence or enters protected staircase')
        for j, (item_index, point_index, row) in enumerate(references):
            item = normalized[item_index]
            if (not np.allclose(world[j], _xyz(row['sample_xyz'], 'sample_xyz'), rtol=0, atol=1e-6)
                    or not np.isclose(field[j], row['field_z'], rtol=0, atol=1e-8)
                    or not np.isclose(derived[j, 2], row['derived_z'], rtol=0, atol=1e-8)):
                raise ValueError('Support observation does not match its scan/pose/floor field')
            center = np.asarray(item['cell_xy'], dtype=float)
            if center.shape != (2,) or not np.isfinite(center).all() or np.any(np.abs(world[j, :2]-center)>.050001):
                raise ValueError('Independent return is not in the declared 10 cm cell')
            item['verified_support'].append({'frame':frame, 'point_index':point_index,
                                             'derived_z':float(derived[j, 2])})
            if frame == item['source_frame_id']:
                if point_index != item['source_point_index']:
                    raise ValueError('Selected source identity differs from support observation')
                if (not np.allclose(world[j], _xyz(item['sample_xyz'], 'selected sample_xyz'), rtol=0, atol=1e-6)
                        or not np.allclose(derived[j], _xyz(item['derived_xyz'], 'derived_xyz'), rtol=0, atol=1e-8)
                        or not np.isclose(field[j], item['field_z'], rtol=0, atol=1e-8)
                        or not np.isclose(derived[j, 2], item['derived_z'], rtol=0, atol=1e-8)):
                    raise ValueError('Selected conditioned return differs from measured evidence')
                output[item_index] = scan.records[point_index]
                for axis, value in zip(('x', 'y', 'z'), derived[j]):
                    output[axis][item_index] = value
                selected_seen[item_index] = True
    if not selected_seen.all():
        raise ValueError('Missing selected source return')
    for item in normalized:
        heights = np.array([row['derived_z'] for row in item['verified_support']])
        if (np.ptp(heights) > rule['max_height_range_m']+1e-8
                or not np.allclose([heights.min(),heights.max()], item['derived_height_range'], rtol=0, atol=1e-8)):
            raise ValueError('Verified multi-frame height spread exceeds contract')
    for path, expected in inputs.items():
        if sha256(path) != expected:
            raise ValueError(f'Measured floor source changed during validation: {path}')
    return output, normalized, inputs
