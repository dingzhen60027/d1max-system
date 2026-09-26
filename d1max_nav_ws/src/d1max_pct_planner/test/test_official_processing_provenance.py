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
                  vendor_root='/home/dndx/d1max_nav_ws/src/pct_planner_vendor',
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
