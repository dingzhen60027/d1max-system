"""Ground-only correspondence: keep the source rows, never fabricate support."""
import hashlib

import numpy as np
import pytest

from tools.pointcloud_preprocessing.ground_path_bridge import (
    GroundBridgeError, GroundPathBridge, StairGroundMapping,
)
from tools.pointcloud_preprocessing.tests.test_ground_path_bridge import _floor


def write_pcd(path, points):
    points = np.asarray(points, dtype='<f8')
    header = ('VERSION .7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    path.write_bytes(header.encode('ascii')+points.tobytes())


def artifact(tmp_path):
    source = np.array([[.5, .5, .7], [.6, .5, .7], [.5, .5, 3.7], [.6, .5, 3.7],
                       [.5, .5, 1.5]], dtype=np.float64)
    ids = np.arange(4, dtype=np.int64)
    planning = source[ids].copy()
    planning[:, 2] += .08
    arrays = dict(source_indices=ids, source_xyz=source[ids], planning_xyz=planning,
                  surface_id=np.array([1, 1, 2, 2], dtype=np.int32))
    (tmp_path/'input.pcd').write_bytes(b'input planning fixture')
    write_pcd(tmp_path/'output.pcd', planning)
    np.savez(tmp_path/'mapping.npz', **arrays)
    digest = lambda name: hashlib.sha256((tmp_path/name).read_bytes()).hexdigest()
    descriptor = dict(schema=StairGroundMapping.SCHEMA, source_sha256='a'*64,
        input_map_file='input.pcd', input_map_sha256=digest('input.pcd'),
        output_map_file='output.pcd', output_map_sha256=digest('output.pcd'),
        mapping_file='mapping.npz', mapping_sha256=digest('mapping.npz'),
        surfaces=[dict(id=1, kind='flight_support_envelope'), dict(id=2, kind='landing_plane')],
        max_displacement_m=.08, association_max_distance_m=.12)
    return source, arrays, descriptor


def load(tmp_path, source, descriptor):
    return StairGroundMapping.from_artifact(descriptor, base_dir=tmp_path, source_xyz=source,
        source_sha256='a'*64, output_path=tmp_path/'output.pcd',
        output_sha256=hashlib.sha256((tmp_path/'output.pcd').read_bytes()).hexdigest())


def test_hash_bound_original_rows_ground_round_trip_and_floor_separation(tmp_path):
    source, arrays, desc = artifact(tmp_path)
    mapping = load(tmp_path, source, desc)
    assert len(mapping.dependency_files) == 3
    assert mapping.source_indices.tolist() == [0, 1, 2, 3]
    region = {'min': [.4, .4, -2.], 'max': [1.5, 1.5, 7.]}
    bridge = GroundPathBridge({'floor1': _floor()}, [region], stair_mapping=mapping)
    for indexes in ([0, 1], [3, 2]):
        planned = arrays['planning_xyz'][indexes]
        original = bridge.to_localization_ground(planned, ['stairs']*2)
        assert np.allclose(original.xyz, source[indexes], rtol=0., atol=1e-15)
        assert np.array_equal(original.xyz[:, :2], planned[:, :2])
        assert np.allclose(bridge.to_planning_ground(original.xyz, ['stairs']*2).xyz, planned)
        assert original.diagnostics['stair_regularization']['mapped_points'] == 2
        assert original.diagnostics['certified_geometric_error_bound_m'] is None
    with pytest.raises(GroundBridgeError, match='correspondence_missing'):
        bridge.to_localization_ground([[.55, .5, 1.9]], ['stairs'])
    # A regularized artifact must not change existing flat-floor behavior.
    plain = GroundPathBridge({'floor1': _floor()}, [region])
    flat = np.array([[.1, .1, -.65], [.2, .1, -.65]])
    assert np.array_equal(bridge.to_localization_ground(flat, ['floor1']*2).xyz,
                          plain.to_localization_ground(flat, ['floor1']*2).xyz)
    # The same protected entry point may be owned by the adjacent floor leg.
    # Its semantic segment label must not bypass the ground correspondence.
    assert np.array_equal(bridge.to_localization_ground(
        arrays['planning_xyz'][:2], ['floor1']*2).xyz,
        bridge.to_localization_ground(arrays['planning_xyz'][:2], ['stairs']*2).xyz)


@pytest.mark.parametrize('mutation,reason', [
    (lambda d: d.update(schema='unknown'), 'schema_invalid'),
    (lambda d: d.update(source_sha256='b'*64), 'identity_mismatch'),
    (lambda d: d.update(output_map_sha256='b'*64), 'identity_mismatch'),
    (lambda d: d.update(mapping_sha256='b'*64), 'file_hash_mismatch'),
    (lambda d: d.update(input_map_sha256='b'*64), 'file_hash_mismatch'),
    (lambda d: d.update(max_displacement_m=.01), 'displacement_invalid'),
    (lambda d: d.update(max_displacement_m=.51), 'limits_invalid'),
    (lambda d: d.update(association_max_distance_m=float('nan')), 'limits_invalid'),
    (lambda d: d.update(surfaces=[dict(id=1, kind='flight')]), 'ownership_invalid'),
])
def test_manifest_mismatch_or_unbounded_relation_is_rejected(tmp_path, mutation, reason):
    source, _, desc = artifact(tmp_path)
    mutation(desc)
    with pytest.raises(GroundBridgeError, match=reason):
        load(tmp_path, source, desc)


@pytest.mark.parametrize('mutation,reason', [
    (lambda a: a['source_indices'].__setitem__(0, 4), 'source_rows_mismatch'),
    (lambda a: a['source_indices'].__setitem__(1, 0), 'source_rows_mismatch'),
    (lambda a: a['source_xyz'].__setitem__((0, 2), .701), 'source_rows_mismatch'),
    (lambda a: a['planning_xyz'].__setitem__((0, 0), .501), 'displacement_invalid'),
    (lambda a: a['planning_xyz'].__setitem__((0, 2), np.nan), 'array_contract_invalid'),
    (lambda a: a.update(source_indices=a['source_indices'].astype(np.float64)), 'array_contract_invalid'),
])
def test_mapping_cannot_reassign_rows_or_move_xy(tmp_path, mutation, reason):
    source, arrays, desc = artifact(tmp_path)
    mutation(arrays)
    np.savez(tmp_path/'mapping.npz', **arrays)
    desc['mapping_sha256'] = hashlib.sha256((tmp_path/'mapping.npz').read_bytes()).hexdigest()
    with pytest.raises(GroundBridgeError, match=reason):
        load(tmp_path, source, desc)


def test_correspondence_never_classifies_extra_roi_wall_or_leaves_stair_domain(tmp_path):
    source, arrays, desc = artifact(tmp_path)
    mapping = load(tmp_path, source, desc)
    original, planned = mapping.association_pairs(source)
    assert np.array_equal(original, arrays['source_xyz'])
    assert np.array_equal(planned, arrays['planning_xyz'])
    assert np.array_equal(original, source[:4])
    assert tuple(source[4]) not in set(map(tuple, original))
    with pytest.raises(GroundBridgeError, match='source_ownership_mismatch'):
        mapping.association_pairs(source[[0, 1, 2]])
    with pytest.raises(GroundBridgeError, match='outside_protected_domain'):
        GroundPathBridge({'floor1': _floor()}, [], stair_mapping=mapping)


def test_archive_budget_is_checked_before_loading_arrays(tmp_path, monkeypatch):
    source, _, desc = artifact(tmp_path)
    monkeypatch.setattr(StairGroundMapping, 'MAX_ARCHIVE_BYTES', 20)
    with pytest.raises(GroundBridgeError, match='archive_budget'):
        load(tmp_path, source, desc)


@pytest.mark.parametrize('direction', ['forward', 'inverse'])
def test_collapsed_correspondence_cannot_pick_an_arbitrary_source_height(tmp_path, direction):
    source, arrays, desc = artifact(tmp_path)
    source[1] = source[0]
    arrays['source_xyz'][1] = source[1]
    arrays['planning_xyz'][1] = arrays['planning_xyz'][0]
    if direction == 'forward':
        arrays['planning_xyz'][1,2] -= .02
    else:
        source[1,2] += .02
        arrays['source_xyz'][1] = source[1]
    np.savez(tmp_path/'mapping.npz', **arrays)
    desc['mapping_sha256'] = hashlib.sha256((tmp_path/'mapping.npz').read_bytes()).hexdigest()
    with pytest.raises(GroundBridgeError, match='ambiguous_coordinates'):
        load(tmp_path, source, desc)


def test_ground_correction_does_not_jump_between_noisy_returns_on_same_flight(tmp_path):
    source, arrays, desc = artifact(tmp_path)
    source[0, 2], source[1, 2] = .63, .77
    arrays['source_xyz'] = source[:4].copy()
    arrays['planning_xyz'][:2, 2] = .7
    mapping = StairGroundMapping(desc, arrays, source_xyz=source)
    queries = np.array([[.549, .5, .7], [.551, .5, .7]])
    nearest = mapping._planning_tree.query(queries)[1]
    old_delta = mapping.planning_xyz[nearest, 2]-mapping.source_xyz[nearest, 2]
    assert abs(old_delta[1]-old_delta[0]) > .12  # Reproduces nearest-row failure.
    mapped, distance = mapping.map_ground(queries, inverse=True)
    assert np.max(distance) < .12
    assert abs(mapped[1, 2]-mapped[0, 2]) < .02
    assert np.array_equal(mapped[:, :2], queries[:, :2])
    # The bridge estimates ground paths; it never changes the support records.
    measured, planned = mapping.association_pairs(source)
    assert np.array_equal(measured, arrays['source_xyz'])
    assert np.array_equal(planned, arrays['planning_xyz'])
    assert np.array_equal(mapping.map_ground(planned, inverse=True)[0], measured)
    # Exact records are the interpolation limit, not an epsilon snap branch.
    near = np.repeat(planned[:1], 3, axis=0)
    near[:, 0] += [1e-5, 1e-7, 1e-9]
    error = np.abs(mapping.map_ground(near, inverse=True)[0][:, 2]-measured[0, 2])
    assert error[0] < 1e-7 and error[2] <= error[1] <= error[0]


def test_ground_correction_does_not_average_separate_surface_ownership(tmp_path):
    source, arrays, desc = artifact(tmp_path)
    source[0, 2], source[1, 2] = .63, .77
    arrays['source_xyz'] = source[:4].copy()
    arrays['planning_xyz'][:2, 2] = .7
    arrays['surface_id'][1] = 2
    mapping = StairGroundMapping(desc, arrays, source_xyz=source)
    query = np.array([[.549, .5, .7]])
    result, _ = mapping.map_ground(query, inverse=True)
    assert np.allclose(result, [[.549, .5, .63]], atol=1e-12, rtol=0.)


def test_hashes_alone_do_not_allow_mapping_to_unwritten_planning_points(tmp_path):
    source, arrays, desc = artifact(tmp_path)
    arrays['planning_xyz'][0, 2] -= .01
    np.savez(tmp_path/'mapping.npz', **arrays)
    desc['mapping_sha256'] = hashlib.sha256((tmp_path/'mapping.npz').read_bytes()).hexdigest()
    with pytest.raises(GroundBridgeError, match='changed_output_rows_missing'):
        load(tmp_path, source, desc)


def test_identity_source_rows_do_not_need_reinsertion_in_output_pcd(tmp_path):
    source, arrays, desc = artifact(tmp_path)
    arrays['planning_xyz'][0] = arrays['source_xyz'][0]
    np.savez(tmp_path/'mapping.npz', **arrays)
    write_pcd(tmp_path/'output.pcd', arrays['planning_xyz'][1:])
    desc['mapping_sha256'] = hashlib.sha256((tmp_path/'mapping.npz').read_bytes()).hexdigest()
    desc['output_map_sha256'] = hashlib.sha256((tmp_path/'output.pcd').read_bytes()).hexdigest()
    mapping = load(tmp_path, source, desc)
    assert np.array_equal(mapping.source_xyz, arrays['source_xyz'])
