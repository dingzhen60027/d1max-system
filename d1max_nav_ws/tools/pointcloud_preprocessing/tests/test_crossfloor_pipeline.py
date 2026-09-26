"""Provenance, record preservation and fail-closed cross-floor derivatives."""
import copy
import json
from pathlib import Path
import struct
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pointcloud_preprocessing import crossfloor_pipeline as pipeline
from pointcloud_preprocessing.pcd_io import read_pcd, _write_records
from pointcloud_preprocessing.runner import sha256
from test_pcd_io import pcd


@pytest.fixture
def measurement(tmp_path):
    source = pcd(tmp_path)
    cloud = read_pcd(source)
    run = tmp_path / 'keyframes'
    (run / 'Scans').mkdir(parents=True)
    matrices = np.tile(np.eye(4)[:3], (5, 1, 1))
    matrices[:, 0, 3] = 10
    poses = run / 'optimized_poses.txt'
    np.savetxt(poses, matrices.reshape(5, 12))
    sources = {str(poses): sha256(poses), str(source): sha256(source)}
    for frame in range(5):
        scan = run / 'Scans' / f'{frame:06d}.pcd'
        scan.write_bytes(source.read_bytes())
        sources[str(scan)] = sha256(scan)
    item = {'frames': list(range(5)), 'height_range': [2.99, 3.01],
            'source_frame_id': 2, 'source_point_index': 0, 'sample_xyz': [11., 2., 3.]}
    evidence = tmp_path / 'returns.json'
    evidence.write_text(json.dumps([item]))
    source_manifest = tmp_path / 'sources.json'
    source_manifest.write_text(json.dumps(sources))
    cfg = {'keyframe_directory': str(run), 'minimum_independent_frames': 5,
           'maximum_height_spread_m': .08}
    return source, cloud, run, item, evidence, source_manifest, cfg


def test_restoration_is_actual_source_return_preserving_non_xyz_fields(measurement):
    source, cloud, _, item, evidence, sources, cfg = measurement
    before = source.read_bytes()
    records, lineage = pipeline.restore_records(cloud, evidence, sources, cfg)
    assert records['intensity'].tobytes() == cloud.records['intensity'][:1].tobytes()
    np.testing.assert_array_equal([records[name][0] for name in ('x', 'y', 'z')], item['sample_xyz'])
    assert lineage == [item]
    assert source.read_bytes() == before


@pytest.mark.parametrize('field,value,match', [
    ('source_frame_id', 2.8, 'integer'), ('source_point_index', .5, 'integer'),
    ('source_point_index', True, 'integer'), ('source_point_index', 99, 'fields/index'),
    ('source_frame_id', 99, 'identity'), ('sample_xyz', [11., 2., 3.1], 'declared measured'),
    ('height_range', [2., 2.01], 'outside.*height'), ('height_range', [2., 3.], 'inconsistent'),
    ('frames', [0, 1, 2, 3], 'independent'), ('frames', [0, 1, 2, 3, 3], 'independent'),
    ('frames', [0, 1, 2, 3, 9], 'independent'), ('frames', [0, 1, 2, 3, 4.0], 'integer'),
])
def test_fabricated_or_weak_measurement_evidence_is_rejected(measurement, field, value, match):
    _, cloud, _, item, evidence, sources, cfg = measurement
    item[field] = value
    evidence.write_text(json.dumps([item]))
    with pytest.raises(ValueError, match=match):
        pipeline.restore_records(cloud, evidence, sources, cfg)


def test_changed_scan_is_rejected_before_restoration(measurement):
    _, cloud, run, _, evidence, sources, cfg = measurement
    (run / 'Scans/000002.pcd').write_bytes(b'changed')
    with pytest.raises(ValueError, match='Input changed'):
        pipeline.restore_records(cloud, evidence, sources, cfg)


def test_scan_change_after_read_is_rejected(measurement, monkeypatch):
    _, cloud, run, _, evidence, sources, cfg = measurement
    target = run / 'Scans/000002.pcd'
    original_read = pipeline.read_pcd
    def read_then_change(path):
        result = original_read(path)
        if Path(path) == target:
            target.write_bytes(target.read_bytes() + b'changed after read')
        return result
    monkeypatch.setattr(pipeline, 'read_pcd', read_then_change)
    with pytest.raises(ValueError, match='Input changed'):
        pipeline.restore_records(cloud, evidence, sources, cfg)


def test_nan_noncoordinate_payload_is_preserved_bitwise(tmp_path):
    words = np.asarray([0x3f800000, 0x40000000, 0x40400000, 0x7fc01234,
                        0x40800000, 0x40a00000, 0x40c00000, 0x41b00000,
                        0x40e00000, 0x41000000, 0x41100000, 0x42040000], dtype='<u4')
    source = read_pcd(pcd(tmp_path, words.tobytes()))
    records = source.records[[1, 0]].copy()
    records['z'] -= .02
    target = tmp_path / 'floor.pcd'
    _write_records(target, source, records, 'test derived floor')
    floor = read_pcd(target)
    pipeline.validate_floor_records(source, floor, np.array([1, 0]))
    assert floor.records['intensity'].tobytes() == source.records['intensity'][[1, 0]].tobytes()
    records['intensity'][0] = 10
    changed = tmp_path / 'changed.pcd'
    _write_records(changed, source, records, 'test changed intensity')
    with pytest.raises(ValueError, match='non-Z'):
        pipeline.validate_floor_records(source, read_pcd(changed), np.array([1, 0]))


@pytest.mark.parametrize('ids', [[0, 0, 1], [0, 1, 9], [0., 1., 2.]])
def test_record_lineage_must_be_unique_bounded_integer(measurement, ids):
    _, cloud, *_ = measurement
    with pytest.raises(ValueError, match='indices|records'):
        pipeline.validate_floor_records(cloud, cloud, np.asarray(ids))


def test_visibility_preserves_protected_structure_and_unobserved_points():
    xyz = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], float)
    votes = {'free': np.array([100, 100, 100, 0]), 'hit': np.array([1, 1, 50, 0]),
             'in_roi': np.array([True, True, True, False])}
    cfg = {'free_hit_ratio': 10, 'minimum_free_frames': 20,
           'structure_protection_regions': [{'min': [-.1, -.1, -.1], 'max': [.1, .1, .1]}]}
    np.testing.assert_array_equal(pipeline.visibility_keep(xyz, votes, cfg), [True, False, True, True])
    for change in [dict(free_hit_ratio=float('nan')), dict(minimum_free_frames=20.5)]:
        with pytest.raises(ValueError):
            pipeline.visibility_keep(xyz, votes, {**cfg, **change})
    with pytest.raises(ValueError, match='counts'):
        pipeline.visibility_keep(xyz, {**votes, 'hit': votes['hit'].astype(float)}, cfg)


def test_hash_conflict_and_stale_evidence_fail_closed(tmp_path):
    source = tmp_path / 'evidence'
    source.write_bytes(b'original')
    digest = sha256(source)
    assert pipeline.verified_file({'path': str(source), 'sha256': digest}) == source
    with pytest.raises(ValueError, match='Conflicting'):
        pipeline.merge_hashes({str(source): digest}, {str(source): 'stale'})
    source.write_bytes(b'new')
    with pytest.raises(ValueError, match='hash mismatch'):
        pipeline.verified_file({'path': str(source), 'sha256': digest})


def test_pipeline_detects_source_change_before_any_new_output(measurement, tmp_path, monkeypatch):
    source, cloud, _, _, evidence, sources, restoration = measurement
    floor = tmp_path / 'floor'
    (floor / 'audit').mkdir(parents=True)
    floor_cfg = {'output_directory': str(floor)}
    floor_config = tmp_path / 'floor.yaml'
    floor_config.write_text(yaml.safe_dump(floor_cfg))
    (floor / 'processed_map.pcd').write_bytes(source.read_bytes())
    audit = floor / 'audit/floor_evidence.npz'
    np.savez(audit, source_indices=np.arange(len(cloud.xyz)))
    (floor / 'manifest.json').write_text(json.dumps({
        'source_sha256': sha256(source), 'configuration': floor_cfg,
        'input_hashes': {str(source): sha256(source)}, 'output_file': 'processed_map.pcd',
        'output_sha256': sha256(floor / 'processed_map.pcd')}))
    votes = tmp_path / 'votes.npz'
    np.savez(votes, free=np.zeros(3, int), hit=np.ones(3, int), in_roi=np.ones(3, bool))
    route_template = tmp_path / 'route.yaml'
    route_template.write_text('schema_version: 1\n')
    def bound(path):
        return {'path': str(path), 'sha256': sha256(path)}
    cfg = {'schema': pipeline.SCHEMA, 'planning_only': True, 'source': bound(source),
           'floor_conditioning_config': bound(floor_config), 'floor_evidence_sha256': sha256(audit),
           'visibility': {'evidence': bound(votes), 'free_hit_ratio': 10, 'minimum_free_frames': 20,
                          'structure_protection_regions': []},
           'measured_restore': {**restoration, 'evidence': bound(evidence), 'sources': bound(sources)},
           'route_template': bound(route_template), 'output_directory': str(tmp_path / 'must_not_exist')}
    path = tmp_path / 'pipeline.yaml'
    path.write_text(yaml.safe_dump(cfg))
    original_restore = pipeline.restore_records
    def restore_then_change(*args):
        result = original_restore(*args)
        source.write_bytes(source.read_bytes() + b'changed after decoding')
        return result
    monkeypatch.setattr(pipeline, 'restore_records', restore_then_change)
    with pytest.raises(ValueError, match='Input changed'):
        pipeline.run(path)
    assert not Path(cfg['output_directory']).exists()
