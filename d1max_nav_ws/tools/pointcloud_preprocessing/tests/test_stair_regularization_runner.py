import numpy as np
import pytest

from tools.pointcloud_preprocessing.stair_regularization_runner import (
    preserve_observed_pct_support, classified_support_mask, rebind_internal_portal_heights,
    preserve_point_slice_membership,
    merge_support_extension, build_support_extension,
    compile_stair_connector,
    checked_compiled_origin_artifacts, extension_route_config,
    saved_landing_request,
)


@pytest.mark.parametrize('use_effective', [False, True])
def test_saved_platform_rebuild_accepts_only_literal_requested_or_effective(use_effective):
    saved = dict(xyz=[1., .1, .5], layer_id=3, requested_xyz=[1., 0., .5], requested_layer_id=3)
    original = dict(xyz=saved['requested_xyz'], layer_id=3)
    current = dict(xyz=saved['xyz'], layer_id=3) if use_effective else original
    assert saved_landing_request(current, saved) == original
    with pytest.raises(ValueError, match='requested_landing_changed'):
        saved_landing_request(dict(xyz=[1., .05, .5], layer_id=3), saved)
    with pytest.raises(ValueError, match='requested_landing_changed'):
        saved_landing_request(dict(xyz=saved['xyz'], layer_id=4), saved)


def _config():
    return dict(resolution=.1, slice_dh=.5, ground_height=-1., simplify_layers=False,
                traversability=dict(kernel_size=3, interval_min=.55,
                    interval_free=.7, slope_max_rad=.6, step_max=.17,
                    standable_ratio=.2, cost_barrier=50., safe_margin=.1, inflation=.1))


def test_highest_point_proposal_rollback_preserves_old_valid_cells(tmp_path):
    cpu = pytest.importorskip('d1max_pct_planner.cpu_tomography')
    x, y = np.meshgrid(np.arange(0., 3.01, .05), np.arange(0., 3.01, .05))
    points = np.c_[x.ravel(), y.ravel(), np.where(y.ravel() < 1.5, 0., .14)].astype(np.float32)
    original, _ = cpu.tomogram_from_points(points, _config())
    path = tmp_path/'before.npz'
    np.savez_compressed(path, **original)
    proposal = points[:, 2].copy()
    proposal[points[:, 1] >= 1.5] += .07
    bad, _ = cpu.tomogram_from_points(np.c_[points[:, :2], proposal].astype(np.float32), _config())
    old_valid = np.isfinite(original['data'][3]) & (original['data'][0] <= 20.)
    assert np.any(old_valid & (bad['data'][0] > 20.))
    accepted, report = preserve_observed_pct_support(path, points, proposal, _config(), {})
    result, _ = cpu.tomogram_from_points(np.c_[points[:, :2], accepted].astype(np.float32), _config())
    assert not np.any(old_valid & (result['data'][0] > 20.))
    assert report['passes'][0]['lost_slice_cells'] > 0
    assert report['passes'][-1]['lost_slice_cells'] == 0
    assert report['retained_changed_points'] < report['proposed_changed_points']
    assert np.all((accepted == points[:, 2]) | (accepted == proposal))


def test_unchanged_proposal_does_not_modify_source(tmp_path):
    points = np.array([[0., 0., 0.], [1., 0., .1]], np.float32)
    archive = tmp_path/'before.npz'
    np.savez_compressed(archive, data=np.zeros((5, 1, 2, 2), np.float32), center=np.zeros(2))
    accepted, report = preserve_observed_pct_support(archive, points, points[:, 2], _config(), {})
    assert np.array_equal(accepted, points[:, 2])
    assert report['retained_changed_points'] == 0


def test_point_slice_membership_preserved_at_float32_closed_boundary():
    original = np.array([.49, .50, .51, .74, 1.51], np.float32)
    proposal = np.array([.52, .49, .50, .78, 1.55], np.float32)
    accepted, report = preserve_point_slice_membership(original, proposal, .5, .5, 3)
    assert np.array_equal(accepted, np.array([.49, .49, .51, .78, 1.55], np.float32))
    assert report['record_indices'] == [0, 2]
    for boundary in np.array([.5, 1., 1.5], np.float32):
        assert np.array_equal(original <= boundary, accepted <= boundary)


def test_known_but_blocked_support_is_not_erased_by_proposal(tmp_path):
    cpu = pytest.importorskip('d1max_pct_planner.cpu_tomography')
    x, y = np.meshgrid(np.arange(0., 1.01, .05), np.arange(0., 1.01, .05))
    low = np.c_[x.ravel(), y.ravel(), np.full(x.size, .49)]
    high = low.copy()
    high[:, 2] = .52  # Measured low ceiling: known support but not traversable.
    points = np.r_[low, high].astype(np.float32)
    original, _ = cpu.tomogram_from_points(points, _config())
    archive = tmp_path/'before.npz'
    np.savez_compressed(archive, **original)
    proposal = points[:, 2].copy()
    proposal[:len(low)] = .51
    accepted, report = preserve_observed_pct_support(archive, points, proposal, _config(), {})
    assert report['passes'][0]['lost_known_slice_cells'] > 0
    result, _ = cpu.tomogram_from_points(np.c_[points[:, :2], accepted].astype(np.float32), _config())
    assert not np.any(np.isfinite(original['data'][3]) & ~np.isfinite(result['data'][3]))


def test_support_association_respects_height_layer_and_validity():
    class Map:
        resolution = .1
        center = np.zeros(2)
        offset = np.zeros(2, int)
        nx = ny = 2
        ground = np.zeros((2, 2, 2))
        ground[1] = 3.
        valid = np.ones((2, 2, 2), bool)
        ground_known = np.ones((2, 2, 2), bool)
        valid[:, 1, 1] = False
    points = np.array([[0., 0., .02], [0., 0., 3.02], [0., 0., 1.5], [.1, .1, 0.]])
    assert classified_support_mask(points, Map()).tolist() == [True, True, False, False]


def test_stale_internal_landing_z_rebound_without_xy_or_layer_changes():
    from copy import deepcopy
    from types import SimpleNamespace
    old = SimpleNamespace(center=np.zeros(2), offset=np.zeros(2, int), resolution=.1,
        nx=3, ny=3, ground=np.ones((2, 3, 3)), valid=np.ones((2, 3, 3), bool))
    new = deepcopy(old)
    new.ground[1, 1, 1] = .93
    route = dict(anchors={name: dict(xyz=[.1, .1, 1.], layer_id=layer)
        for name, layer in [('start', 1), ('entry', 0), ('landing', 1), ('exit', 0), ('goal', 1)]})
    original = deepcopy(route)
    records = rebind_internal_portal_heights(route, old, new, .08)
    assert len(records) == 1 and records[0]['name'] == 'landing'
    assert route['anchors']['landing'] == dict(xyz=[.1, .1, .93], layer_id=1)
    for name in ('start', 'entry', 'exit', 'goal'):
        assert route['anchors'][name] == original['anchors'][name]
    new.valid[1, 1, 1] = False
    with pytest.raises(ValueError, match='bounded_observed_ground'):
        rebind_internal_portal_heights(original, old, new, .08)


def test_extension_mapping_preserves_all_previous_identity_and_corrected_rows():
    from types import SimpleNamespace
    source = np.array([[0., 0., 0.], [1., 0., .5], [2., 0., 1.]])
    old = dict(source_indices=np.array([0, 2], np.int64), source_xyz=source[[0, 2]],
        planning_xyz=source[[0, 2]].copy(), surface_id=np.array([0, 1], np.int32))
    old['planning_xyz'][1, 2] -= .04
    addition = SimpleNamespace(source_indices=np.array([1], np.int64), source_xyz=source[[1]],
        planning_xyz=source[[1]].copy(), surface_id=np.array([1], np.int32))
    addition.planning_xyz[0, 2] -= .03
    merged = merge_support_extension(old, addition, source)
    assert merged['source_indices'].tolist() == [0, 1, 2]
    for key in old:
        np.testing.assert_array_equal(merged[key][[0, 2]], old[key])
    assert merged['planning_xyz'][1, 2] == .47


@pytest.mark.parametrize('change', ['duplicate', 'forged_source', 'moved_xy'])
def test_extension_rejects_old_mapping_overwrite_and_false_source(change):
    from types import SimpleNamespace
    source = np.array([[0., 0., 0.], [1., 0., .5]])
    old = dict(source_indices=np.array([0], np.int64), source_xyz=source[[0]],
        planning_xyz=source[[0]].copy(), surface_id=np.array([1], np.int32))
    addition = SimpleNamespace(source_indices=np.array([1], np.int64), source_xyz=source[[1]],
        planning_xyz=source[[1]].copy(), surface_id=np.array([1], np.int32))
    if change == 'duplicate':
        addition.source_indices[0] = 0
    elif change == 'forged_source':
        addition.source_xyz[0, 2] += .01
    else:
        addition.planning_xyz[0, 0] += .01
    with pytest.raises(ValueError, match='overwrite_or_invent'):
        merge_support_extension(old, addition, source)


def test_extension_build_never_overwrites_existing_directory(tmp_path):
    with pytest.raises(ValueError, match='new_separate_directory'):
        build_support_extension(tmp_path, tmp_path)


@pytest.mark.parametrize('surfaces', [None, [], [True], ['3']])
def test_extension_builder_requires_explicit_reviewed_surface_scope(tmp_path, surfaces):
    package = tmp_path/'input'
    package.mkdir()
    with pytest.raises(ValueError, match='explicit_reviewed_surfaces'):
        build_support_extension(package, tmp_path/'new', surface_ids=surfaces)


def test_compiled_connector_requires_full_pct_and_never_treats_skip_as_complete(tmp_path):
    package = tmp_path/'input'
    package.mkdir()
    with pytest.raises(ValueError, match='complete_production_pct'):
        build_support_extension(package, tmp_path/'new', surface_ids=[3], build_pct=False,
                                compiled_stair_route=tmp_path/'old_route.json')


@pytest.mark.parametrize('change', ['authorized', 'direction', 'status', 'floor_included'])
def test_compiler_rejects_non_stair_or_execution_origin_before_writing(tmp_path, change):
    import json
    package = tmp_path/'new'
    package.mkdir()
    origin = tmp_path/'route.json'
    value = dict(status='validated_offline_route', execution_authorized=False,
        direction='lower_to_upper', segments=[dict(name='stair_lower'), dict(name='stair_upper')])
    if change == 'authorized': value['execution_authorized'] = True
    if change == 'direction': value['direction'] = 'upper_to_lower'
    if change == 'status': value['status'] = 'not_validated'
    if change == 'floor_included': value['segments'].insert(0, dict(name='lower_floor'))
    origin.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='compiled_stair_requires'):
        compile_stair_connector(package, origin)
    assert list(package.iterdir()) == []


def test_compiler_never_overwrites_existing_compiled_artifact(tmp_path):
    artifact = tmp_path/'compiled_stair_connector.json'
    artifact.write_text('previous')
    origin = tmp_path/'origin.json'
    origin.write_text('{}')
    with pytest.raises(ValueError, match='already_exists'):
        compile_stair_connector(tmp_path, origin)
    assert artifact.read_text() == 'previous'


def test_explicit_relocated_origin_map_preserves_original_hash_contract(tmp_path):
    import hashlib
    origin = tmp_path/'sealed_before'
    (origin/'pct').mkdir(parents=True)
    pcd, grid = origin/'processed_map.pcd', origin/'pct/tomogram.npz'
    pcd.write_bytes(b'literal old PCD bytes')
    grid.write_bytes(b'literal old tomogram bytes')
    value = dict(source_pcd='/old/now-replaced/map.pcd', tomogram_path='/old/replaced.npz',
        source_pcd_sha256=hashlib.sha256(pcd.read_bytes()).hexdigest(),
        tomogram_sha256=hashlib.sha256(grid.read_bytes()).hexdigest())
    value['source_tomogram_sha256'] = value['tomogram_sha256']
    assert checked_compiled_origin_artifacts(value, origin) == (pcd, grid)
    assert value['source_pcd'] == '/old/now-replaced/map.pcd'
    pcd.write_bytes(b'new map at the same address')
    with pytest.raises(ValueError, match='origin_map_identity_invalid'):
        checked_compiled_origin_artifacts(value, origin)


def test_new_extension_route_removes_stale_compiled_descriptor_only(tmp_path):
    original = dict(source_pcd='/old/a.pcd', tomogram_path='/old/a.npz', output_directory='/old/route',
        compiled_stair_connector=dict(file='/old/compiled.json', sha256='a'*64),
        anchors={'landing': {'xyz': [1., 2., 3.], 'layer_id': 8}}, planning={'fixed': 'unchanged'})
    copied = extension_route_config(original, tmp_path)
    assert 'compiled_stair_connector' not in copied
    assert original['compiled_stair_connector']['file'] == '/old/compiled.json'
    assert copied['anchors'] == original['anchors'] and copied['planning'] == original['planning']
    assert copied['source_pcd'] == str(tmp_path/'processed_map.pcd')
