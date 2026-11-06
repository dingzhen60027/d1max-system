import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from d1max_pct_planner import official_pipeline as pipeline
from d1max_pct_planner.tomogram_map import TomogramMap


def processing_fixture(tmp_path):
    original = tmp_path / 'original.pcd'
    source = tmp_path / 'processed_map.pcd'
    original.write_bytes(b'original cloud preserved')
    source.write_bytes(b'conditioned geometry')
    record = {'schema_version': 1, 'processing_schema': pipeline.PROCESSING_SCHEMA,
              'status': 'complete', 'output_file': source.name,
              'output_sha256': pipeline.sha256(source), 'frame_id': pipeline.PLANNING_FRAME,
              'source_path': str(original), 'source_sha256': pipeline.sha256(original),
              'planning_only': True, 'geometry_operation': pipeline.GEOMETRY_OPERATION}
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps(record))
    config = {'source_pcd': str(source), 'frame_id': pipeline.PLANNING_FRAME,
              'source_processing_manifest': str(manifest)}
    return config, record, manifest


def test_no_manifest_keeps_default_behavior():
    assert pipeline.source_processing_provenance({'source_pcd': '/unused.pcd'}) is None


def test_valid_processing_manifest_records_exact_provenance(tmp_path):
    config, record, manifest = processing_fixture(tmp_path)
    provenance = pipeline.source_processing_provenance(config)
    assert provenance['manifest_path'] == str(manifest)
    assert provenance['manifest_sha256'] == pipeline.sha256(manifest)
    assert provenance['original_source_sha256'] == record['source_sha256']
    assert provenance['output_path'] == config['source_pcd']
    assert provenance['planning_only'] is True
    pipeline.verify_processing_unchanged(provenance)


@pytest.mark.parametrize('field,value', [
    ('processing_schema', 'unknown/v1'), ('status', 'running'),
    ('geometry_operation', 'rigid_transform'), ('planning_only', False),
    ('planning_only', 'true'), ('planning_only', 1), ('planning_only', None),
    ('frame_id', 'd1max_loc_map'), ('output_file', 'another.pcd'),
    ('output_file', '../processed_map.pcd'), ('output_file', None),
    ('output_sha256', 'wrong'), ('source_path', 'original.pcd'),
    ('source_sha256', 'wrong'),
])
def test_invalid_manifest_rejected(tmp_path, field, value):
    config, record, manifest = processing_fixture(tmp_path)
    record[field] = value
    manifest.write_text(json.dumps(record))
    with pytest.raises(ValueError):
        pipeline.source_processing_provenance(config)


def test_wrong_config_frame_rejected(tmp_path):
    config, _, _ = processing_fixture(tmp_path)
    config['frame_id'] = 'd1max_loc_map'
    with pytest.raises(ValueError, match='dedicated planning frame'):
        pipeline.source_processing_provenance(config)


def test_relative_manifest_rejected(tmp_path):
    config, _, _ = processing_fixture(tmp_path)
    config['source_processing_manifest'] = 'manifest.json'
    with pytest.raises(ValueError, match='absolute'):
        pipeline.source_processing_provenance(config)


@pytest.mark.parametrize('target', ['manifest', 'original'])
def test_change_during_build_is_rejected(tmp_path, target):
    config, record, manifest = processing_fixture(tmp_path)
    provenance = pipeline.source_processing_provenance(config)
    (manifest if target == 'manifest' else Path(record['source_path'])).write_bytes(b'changed')
    with pytest.raises(RuntimeError, match='changed during build'):
        pipeline.verify_processing_unchanged(provenance)


def stair_processing_fixture(tmp_path):
    config, record, manifest = processing_fixture(tmp_path)
    record.update(processing_schema='d1max.crossfloor_planning/v1',
                  geometry_operation='independent_floor_conditioning_visibility_filter_and_measured_return_restore',
                  frame_id='d1max_multifloor_planning')
    config['frame_id'] = record['frame_id']
    baseline, mapping = tmp_path/'original_planning.pcd', tmp_path/'stair_mapping.npz'
    baseline.write_bytes(b'previous planning map, not the localization cloud')
    mapping.write_bytes(b'hash-only provenance fixture, geometry validated by the bridge')
    record['stair_regularization'] = dict(schema='d1max.stair_regularization.v1',
        source_sha256=record['source_sha256'], output_map_file=record['output_file'],
        output_map_sha256=record['output_sha256'], input_map_file=baseline.name,
        input_map_sha256=pipeline.sha256(baseline), mapping_file=mapping.name,
        mapping_sha256=pipeline.sha256(mapping))
    manifest.write_text(json.dumps(record))
    return config, record, manifest, baseline, mapping


def test_stair_processing_is_bound_to_original_and_both_planning_maps(tmp_path):
    config, record, _, baseline, mapping = stair_processing_fixture(tmp_path)
    result = pipeline.source_processing_provenance(config)
    assert result['stair_regularization_schema'] == 'd1max.stair_regularization.v1'
    assert result['stair_mapping_sha256'] == pipeline.sha256(mapping)
    assert result['processing_dependency_hashes'] == {
        str(baseline): pipeline.sha256(baseline), str(mapping): pipeline.sha256(mapping),
        config['source_pcd']: record['output_sha256']}
    pipeline.verify_processing_unchanged(result)
    mapping.write_bytes(b'changed during tomography')
    with pytest.raises(RuntimeError, match='Stair processing evidence changed'):
        pipeline.verify_processing_unchanged(result)


def test_structured_cost_is_explicit_and_hash_bound(tmp_path):
    config, record, manifest, _, _ = stair_processing_fixture(tmp_path)
    model, evidence = tmp_path/'structure.json', tmp_path/'evidence.npz'
    model.write_text('{}')
    evidence.write_bytes(b'provenance-only fixture; geometry checked by the model loader')
    record['stair_structured_support'] = dict(model_file=model.name, evidence_file=evidence.name,
        model_sha256=pipeline.sha256(model), evidence_sha256=pipeline.sha256(evidence))
    record['stair_structured_cost'] = dict(schema='d1max.stair_structured_cost/v1',
        policy='source_supported_terrain_seeds_with_retained_entities',
        model_sha256=pipeline.sha256(model), evidence_sha256=pipeline.sha256(evidence))
    manifest.write_text(json.dumps(record))
    provenance = pipeline.source_processing_provenance(config)
    assert provenance['stair_structured_cost'] == record['stair_structured_cost']
    assert provenance['processing_dependency_hashes'][str(model)] == pipeline.sha256(model)
    evidence.write_bytes(b'changed')
    with pytest.raises(RuntimeError, match='Stair processing evidence changed'):
        pipeline.verify_processing_unchanged(provenance)
    with pytest.raises(ValueError, match='Structured stair artifact hash mismatch'):
        pipeline.source_processing_provenance(config)


def test_no_explicit_structured_prior_returns_identical_payload(tmp_path):
    payload = {'data': np.ones((5, 1, 3, 3), dtype=np.float32)}
    for processing in (None, {'stair_regularization_schema': 'd1max.stair_regularization.v1'}):
        actual, diagnostics = pipeline.apply_structured_prior(payload, {}, processing, tmp_path, None)
        assert actual is payload and diagnostics is None
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('field,value', [
    ('schema', 'unrelated'), ('source_sha256', '0'*64),
    ('output_map_sha256', '0'*64), ('mapping_sha256', '0'*64),
    ('input_map_sha256', '0'*64), ('mapping_file', None),
    ('output_map_file', 'original_planning.pcd'),
])
def test_stair_provenance_mismatch_is_not_ignored(tmp_path, field, value):
    config, record, manifest, _, _ = stair_processing_fixture(tmp_path)
    record['stair_regularization'][field] = value
    manifest.write_text(json.dumps(record))
    with pytest.raises(ValueError):
        pipeline.source_processing_provenance(config)


def test_conditioned_build_labels_derived_surfaces_without_object_payload(tmp_path):
    import open3d as o3d
    config, record, processing_manifest = processing_fixture(tmp_path)
    x, y = np.meshgrid(np.arange(-.4, .5, .2), np.arange(-.4, .5, .2))
    points = np.vstack([np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)]),
                        np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 2.)])])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    assert o3d.io.write_point_cloud(config['source_pcd'], cloud)
    record['output_sha256'] = pipeline.sha256(config['source_pcd'])
    processing_manifest.write_text(json.dumps(record))
    original_hash = pipeline.sha256(record['source_path'])
    config.update(schema_version=1, output_directory=str(tmp_path / 'result'),
                  vendor_root='${D1MAX_NAV_ROOT}/src/pct_planner_vendor',
                  pct={'resolution': .2, 'slice_dh': .5, 'ground_height': -.5,
                       'traversability': {'kernel_size': 3, 'interval_min': .55, 'interval_free': .7,
                                          'slope_max_rad': .4, 'step_max': .17, 'standable_ratio': .2,
                                          'cost_barrier': 50., 'safe_margin': .2, 'inflation': .2}},
                  export={'traversable_pcd': False})
    path = tmp_path / 'pipeline.yaml'
    path.write_text(yaml.safe_dump(config))
    output, manifest = pipeline.build(path)
    assert pipeline.sha256(record['source_path']) == original_hash
    assert manifest['source_processing']['manifest_sha256'] == pipeline.sha256(processing_manifest)
    assert manifest['planning_only'] is True
    assert manifest['ground_semantics'] == 'input_derived_surface'
    assert manifest['tensor_channels'][3:] == ['input_derived_ground_z', 'input_derived_ceiling_z']
    assert not any('No hole filling, floor flattening' in text for text in manifest['limitations'])
    assert any('non-rigid Z conditioning' in text for text in manifest['limitations'])
    with np.load(output / 'tomogram.npz', allow_pickle=False) as payload:
        assert all(payload[key].dtype.kind != 'O' for key in payload.files)
        assert str(payload['processing_schema']) == pipeline.PROCESSING_SCHEMA
        assert bool(payload['planning_only']) is True
        assert str(payload['geometry_operation']) == pipeline.GEOMETRY_OPERATION
        assert str(payload['ground_semantics']) == 'input_derived_surface'
        assert str(payload['original_source_sha256']) == original_hash
        assert str(payload['source_processing_manifest_sha256']) == pipeline.sha256(processing_manifest)
    tomogram = TomogramMap(output / 'tomogram.npz')
    exposed = tomogram.verify_source(config['source_pcd'], pipeline.PLANNING_FRAME)
    assert exposed['planning_only'] is True
    assert exposed['geometry_operation'] == pipeline.GEOMETRY_OPERATION
    assert exposed['processing_schema'] == pipeline.PROCESSING_SCHEMA
    assert exposed['original_source_sha256'] == original_hash
