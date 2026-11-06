import json

import numpy as np
import pytest
import yaml

from tools.pointcloud_preprocessing.source_record_cleanup import reviewed_source_keep
from tools.pointcloud_preprocessing.runner import sha256


def fixture(tmp_path):
    source = tmp_path / 'original.pcd'
    source.write_bytes(b'identity fixture')
    directory = tmp_path / 'reviewed'
    (directory / 'audit').mkdir(parents=True)
    evidence = directory / 'audit/floor_evidence.npz'
    np.savez_compressed(evidence, removed_indices=np.array([1, 3, 8]),
        floor1_source_indices=np.arange(8), protected_stairs=np.zeros(100, dtype=bool))
    manifest = directory / 'manifest.json'
    manifest.write_text(json.dumps(dict(status='complete',
        processing_schema='d1max.multifloor_planning/v1', source_path=str(source),
        source_sha256=sha256(source), input_points=100, removed_points=3)))
    settings = dict(schema='d1max.reviewed_source_record_cleanup/v1',
        manifest_path=str(manifest), manifest_sha256=sha256(manifest),
        evidence_sha256=sha256(evidence), maximum_removed_fraction=.05)
    return source, evidence, settings


def test_preserves_observed_support_and_other_floor(tmp_path):
    source, evidence, settings = fixture(tmp_path)
    keep, audit = reviewed_source_keep(source, 100, evidence, 'floor1', settings, np.array([3]))
    assert np.flatnonzero(~keep).tolist() == [1]
    assert audit['protected_observed_support_points'] == 1
    assert audit['source_xyz_modified'] is False


def test_no_implicit_cleanup(tmp_path):
    source, evidence, settings = fixture(tmp_path)
    keep, audit = reviewed_source_keep(source, 100, evidence, 'floor1', None, np.array([3]))
    assert keep.all() and audit['enabled'] is False


@pytest.mark.parametrize('change', ['source', 'evidence', 'manifest', 'floor', 'budget'])
def test_rejects_wrong_evidence_or_budget(tmp_path, change):
    source, evidence, settings = fixture(tmp_path)
    floor = 'floor1'
    if change == 'source':
        source.write_bytes(b'different source')
    elif change == 'evidence':
        settings['evidence_sha256'] = '0' * 64
    elif change == 'manifest':
        settings['manifest_sha256'] = '0' * 64
    elif change == 'floor':
        floor = 'other_floor'
    elif change == 'budget':
        settings['maximum_removed_fraction'] = .005
    with pytest.raises((ValueError, KeyError)):
        reviewed_source_keep(source, 100, evidence, floor, settings, np.array([3]))


@pytest.mark.parametrize('hash_valid', [False, True])
def test_invalid_route_template_never_leaves_partial_complete_output(tmp_path, hash_valid):
    from tools.map.build_source_identity_floor import build
    template=tmp_path/'route.yaml'
    template.write_text(yaml.safe_dump(dict(schema='wrong-template',floor_id='floor1')))
    config=tmp_path/'config.yaml'
    config.write_text(yaml.safe_dump(dict(route_template=dict(path=str(template),
        sha256=sha256(template) if hash_valid else '0'*64))))
    output=tmp_path/'new_map'
    with pytest.raises(ValueError,match='source_identity_route_template'):
        build(tmp_path/'not_read.pcd',tmp_path/'not_read.npz',config,output,'floor1')
    assert not output.exists()
