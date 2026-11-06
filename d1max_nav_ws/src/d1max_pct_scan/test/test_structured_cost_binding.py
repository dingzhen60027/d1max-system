"""Static artifact binding checks; these do not mock successful navigation."""
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_scan.navigation_map_package import _structured_cost_dependencies


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tensor_hash(data):
    return hashlib.sha256(np.ascontiguousarray(data).tobytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value))


@pytest.fixture
def package(tmp_path):
    (tmp_path/'pct').mkdir()
    data = np.zeros((5, 1, 3, 3), np.float32)
    data[0] = 25.
    data[3, 0] = np.arange(-1, 2)[:, None]*.05
    data[4] = data[3]+1.2
    base = data.copy()
    data[0, 0, 1, 1] = 15.
    data[1, :, 1:-1] = data[0, :, 2:]-data[0, :, :-2]
    data[2, :, :, 1:-1] = data[0, :, :, 2:]-data[0, :, :, :-2]
    base_path = tmp_path/'pct/structured_cost_base.npz'
    np.savez_compressed(base_path, data=base, center=[0., 0.], resolution=.1, slice_h0=0., slice_dh=.2)
    eligible = np.zeros((1, 3, 3), bool); eligible[0, 1, 1] = True
    witness = np.full((1, 3, 3), -1, np.int64); witness[0, 1, 1] = 101
    evidence = dict(eligible_terrain=eligible, changed_cost=eligible, modified_raw_seed=eligible,
        literal_source_witness=witness, qualified_source_indices=np.array([101], np.int64),
        qualified_source_xyz=np.zeros((1, 3)), qualified_source_surface_id=np.ones(1, np.int32))
    evidence_path = tmp_path/'pct/structured_cost_evidence.npz'
    np.savez_compressed(evidence_path, **evidence)
    source_evidence = tmp_path/'source_evidence.npz'
    np.savez_compressed(source_evidence, source_indices=np.array([101]), surface_id=np.array([1]))
    model = dict(flights=[dict(surface_id=1,
        frame=dict(center_xy=[0., 0.], uphill_axis_xy=[1., 0.], along_range_m=[-1., 1.]),
        retained_structure=dict(sides=dict(negative=dict(lateral_slope_intercept=[0., -.5]),
                                           positive=dict(lateral_slope_intercept=[0., .5]))),
        wall_boundary_guard_m=.06, height_model=dict(grade=.5, intercept_m=0.))])
    write_json(tmp_path/'model.json', model)
    structure = dict(model_file='model.json', model_sha256=digest(tmp_path/'model.json'),
        evidence_file=source_evidence.name, evidence_sha256=digest(source_evidence))
    prior = dict(schema='d1max.stair_structured_cost/v1',
        policy='source_supported_terrain_seeds_with_retained_entities',
        model_sha256=structure['model_sha256'], evidence_sha256=structure['evidence_sha256'])
    manifest = dict(stair_structured_support=structure, stair_structured_cost=prior,
                    output_sha256='b'*64, source_sha256='a'*64)
    provenance = dict(stair_structured_cost_schema=prior['schema'],
        stair_structured_model_sha256=structure['model_sha256'], stair_structured_evidence_sha256=structure['evidence_sha256'],
        stair_structured_base_sha256=digest(base_path), stair_structured_cost_evidence_sha256=digest(evidence_path),
        stair_structured_data_sha256=tensor_hash(data))
    write_json(tmp_path/'pct/structured_cost_report.json', dict(input_tomogram_sha256=digest(base_path),
        current_planning_sha256='b'*64, original_source_sha256='a'*64,
        model_is_explicit_planning_prior_not_observation=True, original_ground_ceiling_unchanged=True))
    grid = SimpleNamespace(data=data, cost=data[0], gradient_x=data[1], gradient_y=data[2],
        ground=data[3], ground_known=np.ones((1, 3, 3), bool), ceiling_known=np.ones((1, 3, 3), bool),
        headroom=data[4]-data[3], minimum_headroom_m=.5, center=np.zeros(2), offset=np.ones(2),
        resolution=.1, slice_h0=0., slice_dh=.2, provenance=provenance)
    mapping = SimpleNamespace(source_indices=np.array([101]), source_xyz=np.zeros((1, 3)))
    checked_paths = {}
    def checked(path, expected=None):
        if expected is not None:
            assert digest(path) == expected
        checked_paths[str(path)] = digest(path)
        return path
    return SimpleNamespace(directory=tmp_path, manifest=manifest, grid=grid, mapping=mapping,
        source_points=np.zeros((1, 3)), checked=checked, evidence=evidence,
        evidence_path=evidence_path, checked_paths=checked_paths)


def load(p):
    return _structured_cost_dependencies(p.directory, p.manifest, p.mapping,
        p.grid, p.source_points, p.checked)


def refresh_evidence(p):
    np.savez_compressed(p.evidence_path, **p.evidence)
    p.grid.provenance['stair_structured_cost_evidence_sha256'] = digest(p.evidence_path)


def test_consistent_literal_package_returns_only_qualified_source(package):
    result = load(package)
    np.testing.assert_array_equal(result, package.source_points)
    assert not result.flags.writeable
    assert len(package.checked_paths) == 3


def test_gradient_cannot_change_while_cost_is_unchanged_even_with_new_tensor_hash(package):
    package.grid.data[1, 0, 1, 0] = 5.
    package.grid.provenance['stair_structured_data_sha256'] = tensor_hash(package.grid.data)
    with pytest.raises(ValueError, match='gradient_not_cost_difference'):
        load(package)


def test_qualified_owner_cannot_be_relabelled_under_rehashed_evidence(package):
    package.evidence['qualified_source_surface_id'][:] = 3
    refresh_evidence(package)
    with pytest.raises(ValueError, match='qualified_source_owner'):
        load(package)


def test_equal_xyz_foreign_original_index_is_not_the_qualified_observation(package):
    # Two actual raw rows may share coordinates; normal evidence for one is
    # not authorization to substitute the other source record.
    package.mapping.source_indices = np.array([101, 102])
    package.mapping.source_xyz = np.zeros((2, 3))
    package.evidence['qualified_source_indices'][:] = 102
    package.evidence['literal_source_witness'][0, 1, 1] = 102
    refresh_evidence(package)
    with pytest.raises(ValueError, match='qualified_source_owner'):
        load(package)


def test_no_prior_means_no_additional_scan_support(package):
    del package.manifest['stair_structured_cost']
    package.grid.provenance = {}
    assert load(package) is None


def test_offline_cost_stage_passes_full_retained_cloud_not_tomography_subset(tmp_path, monkeypatch):
    from d1max_pct_planner.official_pipeline import apply_structured_prior
    from d1max_pct_scan.pointcloud_helpers import stair_structured_cost
    def pcd(path, xyz):
        points = np.asarray(xyz, '<f4')
        header = ('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n')
        path.write_bytes(header.encode()+points.tobytes())
    original, retained = tmp_path/'original.pcd', tmp_path/'retained.pcd'
    pcd(original, [[0, 0, 0], [0, 0, .3], [0, 0, 1.]])
    pcd(retained, [[0, 0, 0], [0, 0, .3]])
    write_json(tmp_path/'model.json', {})
    np.savez_compressed(tmp_path/'evidence.npz', source_indices=np.array([0]))
    processing = dict(manifest_path=str(tmp_path/'manifest.json'), original_source_pcd=str(original),
        output_path=str(retained), output_sha256=digest(retained),
        stair_structured_support=dict(model_file='model.json', evidence_file='evidence.npz',
            model_sha256=digest(tmp_path/'model.json'), evidence_sha256=digest(tmp_path/'evidence.npz')),
        stair_structured_cost=dict(schema='d1max.stair_structured_cost/v1'))
    captured = {}
    def inspect_input(data, **kwargs):
        captured.update(kwargs)
        return dict(data=data.copy(), arrays=dict(probe=np.array([1])), diagnostics=dict(probe_only=True))
    # This spies on data wiring only, never claims a planner/collision success.
    monkeypatch.setattr(stair_structured_cost, 'apply_structured_stair_cost', inspect_input)
    payload = dict(data=np.zeros((5, 1, 1, 1), np.float32), center=np.zeros(2), resolution=.1)
    apply_structured_prior(payload, dict(pct=dict(traversability={})), processing, tmp_path,
                           np.array([[0., 0., 0.]]))
    np.testing.assert_array_equal(captured['planning_xyz'], np.array([[0., 0., 0.], [0., 0., np.float32(.3)]]))
    assert captured['original_source_xyz'].shape == (3, 3)
