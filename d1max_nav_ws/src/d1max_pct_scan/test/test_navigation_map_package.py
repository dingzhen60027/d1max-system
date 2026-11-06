"""Passive package contracts using small literal source/derived artifacts.

The fixture is synthetic test data, not a map or physical-acceptance result.
It uses the production ground-bridge reconstruction and source-route loader;
no mocked successful geometry verdict or native planner is involved.
"""
import json
import shutil

import numpy as np
import pytest
import yaml

from d1max_pct_scan.navigation_map_package import CONDITIONED_GEOMETRY, validate_package
from d1max_pct_scan.navigation_map_package import _compiled_stair_origin_dependency
from d1max_pct_scan.source_route import digest_file


def write_json(path, value):
    path.write_text(json.dumps(value))


def write_pcd(path, xyz):
    xyz = np.asarray(xyz, dtype='<f4')
    header = ('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(xyz)}\nDATA binary\n')
    path.write_bytes(header.encode() + xyz.tobytes())


def compiled_origin_fixture(tmp_path):
    """Literal provenance-only fixture; no native geometry verdict is mocked."""
    from copy import deepcopy
    value = dict(status='validated_offline_route', execution_authorized=False,
        direction='lower_to_upper', tomogram_sha256='a'*64,
        path=[[0., 0., 0.], [1., 0., .5], [2., 0., 1.]], layer_ids=[0, 1, 2],
        segments=[dict(last_index=1), dict(first_index=1)])
    path = tmp_path/'compiled_stair_origin_route.json'
    write_json(path, value)
    digest = digest_file(path)
    loaded = dict(origin=dict(route_file=path.name, source_route_sha256=digest),
        legs=dict(stair_lower=dict(path=value['path'][:2], layer_ids=[0, 1]),
                  stair_upper=dict(path=value['path'][1:], layer_ids=[1, 2])),
        diagnostics=dict(compiled_stair_connector=dict(source_route_sha256=digest,
            origin_tomogram_sha256='a'*64)))
    return path, value, deepcopy(loaded)


def test_compiled_origin_dependency_is_literal_hash_bound_package_file(tmp_path):
    path, _, connector = compiled_origin_fixture(tmp_path)
    dependencies = {}
    def check(file, digest):
        assert digest_file(file) == digest
        dependencies[str(file)] = digest
    _compiled_stair_origin_dependency(tmp_path, connector, check)
    assert dependencies == {str(path): digest_file(path)}


@pytest.mark.parametrize('mutation', ['geometry', 'layers', 'authority', 'direction', 'old_map', 'foreign_hash'])
def test_compiled_origin_cannot_claim_foreign_curve_or_old_identity(tmp_path, mutation):
    path, value, connector = compiled_origin_fixture(tmp_path)
    if mutation == 'geometry': value['path'][1][1] = .1
    if mutation == 'layers': value['layer_ids'][1] = 0
    if mutation == 'authority': value['execution_authorized'] = True
    if mutation == 'direction': value['direction'] = 'upper_to_lower'
    if mutation == 'old_map': value['tomogram_sha256'] = 'b'*64
    write_json(path, value)
    digest = digest_file(path)
    connector['origin']['source_route_sha256'] = digest
    if mutation != 'foreign_hash':
        connector['diagnostics']['compiled_stair_connector']['source_route_sha256'] = digest
    else:
        connector['diagnostics']['compiled_stair_connector']['source_route_sha256'] = 'f'*64
    # Runtime geometry was already checked separately; this helper only links
    # its exported polyline to the exact immutable offline source document.
    def check(file, expected):
        assert digest_file(file) == expected
    with pytest.raises(ValueError, match='compiled_stair_origin'):
        _compiled_stair_origin_dependency(tmp_path, connector, check)


def test_compiled_origin_cannot_escape_package(tmp_path):
    _, _, connector = compiled_origin_fixture(tmp_path)
    connector['origin']['route_file'] = '../outside.json'
    with pytest.raises(ValueError, match='package_relative'):
        _compiled_stair_origin_dependency(tmp_path, connector, lambda *args: None)


@pytest.mark.parametrize('mutation', [None, 'z', 'platform', 'outside', 'layers', 'hash', 'scope', 'authority'])
@pytest.mark.parametrize('model_derivative', [False, True])
def test_centered_native_derivative_has_separate_bound_evidence(tmp_path, mutation, model_derivative):
    from copy import deepcopy
    from d1max_pct_planner.compiled_stair_connector import REFINEMENT_SCHEMA, polyline_sha256
    path, original, connector = compiled_origin_fixture(tmp_path)
    original['path'] = [[float(i), 0., float(i)*.1] for i in range(13)]
    original['layer_ids'] = list(range(13))
    original['segments'] = [dict(last_index=6), dict(first_index=6)]
    write_json(path, original)
    digest = digest_file(path)
    connector['origin']['source_route_sha256'] = digest
    connector['diagnostics']['compiled_stair_connector']['source_route_sha256'] = digest
    points, layers = deepcopy(original['path']), original['layer_ids'][:]
    points[3][1] = .1
    evidence = dict(schema=REFINEMENT_SCHEMA, execution_authorized=False,
        source_route_sha256=digest,
        input_geometry_sha256=polyline_sha256(original['path'], original['layer_ids']),
        changed_vertices=[3], flight_spans=[dict(leg='stair_lower', first_index=1, last_index=4)])
    if mutation == 'z': points[3][2] += .01
    if mutation == 'platform': points[6][1] += .01
    if mutation == 'outside': points[5][1] += .01
    if mutation == 'layers': layers[3] = 2
    if mutation == 'scope': evidence['flight_spans'][0]['last_index'] = 7
    if mutation == 'authority': evidence['execution_authorized'] = True
    evidence['changed_vertices'] = np.flatnonzero(np.any(np.asarray(points) != original['path'], axis=1)).tolist()
    evidence['output_geometry_sha256'] = polyline_sha256(points, layers)
    if mutation == 'hash': evidence['output_geometry_sha256'] = 'f'*64
    evidence_path = tmp_path/'centering.json'
    write_json(evidence_path, evidence)
    connector['origin']['centering_refinement'] = dict(file=evidence_path.name, sha256=digest_file(evidence_path))
    connector['legs'] = dict(stair_lower=dict(path=points[:7], layer_ids=layers[:7]),
                            stair_upper=dict(path=points[6:], layer_ids=layers[6:]))
    if model_derivative:
        # Provenance-only input from the separately tested production model
        # loader: current Z is different, but its base must still satisfy the
        # original native/centering evidence without changing that evidence.
        connector['origin']['parametric_geometry'] = dict(schema='d1max.compiled_stair_parametric_geometry/v1')
        connector['model_base_geometry'] = dict(path=deepcopy(points), layer_ids=layers[:])
        modeled = [[p[0], p[1], p[2]+.02] for p in points]
        connector['legs'] = dict(stair_lower=dict(path=modeled[:7], layer_ids=layers[:7]),
                                stair_upper=dict(path=modeled[6:], layer_ids=layers[6:]))
    dependencies = {}
    def check(file, expected):
        assert digest_file(file) == expected
        dependencies[str(file)] = expected
    if mutation is None:
        _compiled_stair_origin_dependency(tmp_path, connector, check)
        assert set(dependencies) == {str(path), str(evidence_path)}
    else:
        with pytest.raises(ValueError, match='compiled_stair_centering'):
            _compiled_stair_origin_dependency(tmp_path, connector, check)


@pytest.mark.parametrize('mutation', [None, 'xy', 'layers', 'missing_derivation', 'base_z'])
def test_model_derivation_cannot_relabel_native_origin(tmp_path, mutation):
    from copy import deepcopy
    path, original, connector = compiled_origin_fixture(tmp_path)
    connector['origin']['parametric_geometry'] = dict(schema='d1max.compiled_stair_parametric_geometry/v1')
    connector['model_base_geometry'] = dict(path=deepcopy(original['path']), layer_ids=original['layer_ids'][:])
    modeled = [[p[0], p[1], p[2]+.02] for p in original['path']]
    connector['legs'] = dict(stair_lower=dict(path=modeled[:2], layer_ids=[0, 1]),
                            stair_upper=dict(path=modeled[1:], layer_ids=[1, 2]))
    if mutation == 'xy': connector['legs']['stair_lower']['path'][0][0] += .01
    if mutation == 'layers': connector['model_base_geometry']['layer_ids'][0] = 1
    if mutation == 'missing_derivation': del connector['origin']['parametric_geometry']
    if mutation == 'base_z': connector['model_base_geometry']['path'][0][2] += .01
    def check(file, expected):
        assert digest_file(file) == expected
    if mutation is None:
        _compiled_stair_origin_dependency(tmp_path, connector, check)
    else:
        with pytest.raises(ValueError, match='compiled_stair_origin'):
            _compiled_stair_origin_dependency(tmp_path, connector, check)


@pytest.fixture
def conditioned_package(tmp_path):
    directory = tmp_path/'conditioned'
    (directory/'pct').mkdir(parents=True)
    original = tmp_path/'original'
    original.mkdir()
    source = original/'processed_map.pcd'
    source_manifest = original/'manifest.json'
    write_json(source_manifest, dict(frame_id='d1max_loc_map', physical_acceptance=False))
    grid = np.linspace(-.2, .2, 5)
    xy = np.array([[x, y] for x in grid for y in grid])
    xyz = np.concatenate([np.column_stack((xy, np.full(len(xy), z)))
                          for z in (-.75, 3.)])
    write_pcd(source, xyz)
    conditioned = xyz.copy()
    conditioned[:len(xy), 2] = -.65
    conditioned[len(xy):, 2] = 3.15
    output = directory/'processed_map.pcd'
    write_pcd(output, conditioned)

    stage_directory = tmp_path/'floor_stage'
    (stage_directory/'audit').mkdir(parents=True)
    trajectory = stage_directory/'optimized_poses.txt'
    anchor_xy = np.array([[-.05, -.05], [.05, 0.], [0., .05]])
    rows = []
    for ground_z in (-.75, 3.):
        for x, y in anchor_xy:
            rows.append([1., 0., 0., x, 0., 1., 0., y, 0., 0., 1., ground_z+.55])
    np.savetxt(trajectory, rows, fmt='%.17g')
    evidence = stage_directory/'audit/floor_evidence.npz'
    evidence_payload = {}
    floors = []
    for index, (floor, source_z, reference_z, bounds) in enumerate((
            ('floor1', -.75, -.65, [-1., 0.]),
            ('floor2', 3., 3.15, [2.8, 3.5]))):
        evidence_payload.update({
            floor+'_source_indices': np.arange(index*len(xy), (index+1)*len(xy)),
            floor+'_support': np.ones(len(xy), dtype=bool),
            floor+'_floor_z': np.full(len(xy), source_z),
            floor+'_anchors_xy': anchor_xy,
            floor+'_anchors_z': np.full(len(anchor_xy), source_z),
        })
        floors.append(dict(id=floor, input_z_band=bounds, reference_z_m=reference_z,
                           trajectory_range_inclusive=[3*index, 3*index+2]))
    np.savez_compressed(evidence, **evidence_payload)
    defaults = stage_directory/'defaults.yaml'
    defaults.write_text('conditioning: {}\n')
    stage_path = stage_directory/'manifest.json'
    stage = dict(status='complete', planning_only=True, frame_id='d1max_multifloor_planning',
        source_path=str(source), source_sha256=digest_file(source),
        trajectory_path=str(trajectory), trajectory_sha256=digest_file(trajectory),
        configuration=dict(source_pcd=str(source), trajectory_path=str(trajectory),
            floor_stage_defaults=str(defaults), floors=floors, protected_regions=[]),
        resolved_floor_defaults=dict(conditioning={}),
        input_hashes={str(defaults): digest_file(defaults)})
    write_json(stage_path, stage)

    tomogram_path = directory/'pct/tomogram.npz'
    route_path = directory/'route.yaml'
    route = dict(schema_version=1, frame_id='d1max_multifloor_planning',
        source_pcd=str(output), tomogram_path=str(tomogram_path),
        vendor_root=str(tmp_path/'unused_native'), output_directory=str(tmp_path/'unused_routes'),
        anchors={name: dict(xyz=point, layer_id=layer) for name, point, layer in (
            ('start', [-.1, 0., -.65], 0), ('entry', [0., 0., -.65], 0),
            ('landing', [0., 0., 1.25], 0), ('exit', [.1, 0., 3.15], 1),
            ('goal', [.2, 0., 3.15], 1))},
        stair_roi=dict(min=[-.15, -.15, -.8], max=[.15, .15, 3.3]),
        floor_z_ranges=dict(lower=[-1., 0.], upper=[2.8, 3.5]),
        unknown_ceiling_policy='reject', limits=dict(max_ground_step_m=.17))
    route_path.write_text(yaml.safe_dump(route))

    linked_paths = {}
    def linked(name):
        path = tmp_path/(name+'.json')
        write_json(path, dict(fixture=name))
        linked_paths[name] = path
        return dict(path=str(path), sha256=digest_file(path))

    configuration = dict(floor_evidence_sha256=digest_file(evidence),
        floor_conditioning_config=linked('floor_conditioning_config'),
        visibility=dict(evidence=linked('visibility')),
        measured_restore=dict(evidence=linked('upper_evidence'), sources=linked('upper_sources')),
        conditioned_measured_restore=dict(evidence=linked('lower_evidence'), sources=linked('lower_sources')),
        route_template=linked('route_template'))
    manifest_path = directory/'manifest.json'
    manifest = dict(schema=1, processing_schema='d1max.crossfloor_planning/v1',
        status='complete', planning_only=True, geometry_operation=CONDITIONED_GEOMETRY,
        frame_id='d1max_multifloor_planning', source_path=str(source),
        source_sha256=digest_file(source), output_file='processed_map.pcd',
        output_sha256=digest_file(output), floor_processing_manifest=str(stage_path),
        floor_processing_manifest_sha256=digest_file(stage_path), configuration=configuration,
        physical_acceptance=False)
    write_json(manifest_path, manifest)
    data = np.zeros((5, 2, 5, 5), dtype=np.float32)
    data[3, 0] = -.65
    data[3, 1] = 3.15
    data[4] = data[3]+2.
    np.savez_compressed(tomogram_path, data=data, resolution=.1, center=[0., 0.],
        slice_h0=-.65, slice_dh=3.8, source_pcd=str(output),
        source_sha256=manifest['output_sha256'], frame_id=manifest['frame_id'],
        source_processing_manifest=str(manifest_path),
        source_processing_manifest_sha256=digest_file(manifest_path),
        original_source_pcd=str(source), original_source_sha256=manifest['source_sha256'],
        geometry_operation=CONDITIONED_GEOMETRY, processing_schema=manifest['processing_schema'],
        planning_only=True)
    return dict(directory=directory, source=source, source_manifest=source_manifest,
        output=output, manifest=manifest_path, stage=stage_path, trajectory=trajectory,
        evidence=evidence, defaults=defaults, route=route_path, tomogram=tomogram_path,
        linked=linked_paths)


def rewrite_tomogram(path, **changes):
    with np.load(path, allow_pickle=False) as archive:
        payload = {key: archive[key].copy() for key in archive.files}
    payload.update(changes)
    np.savez_compressed(path, **payload)


def with_stair_mapping(fixture):
    """Add one literal, bounded stair correspondence, not a fake motion pass."""
    from d1max_pct_scan.pointcloud_helpers.pcd_io import read_pcd
    source_xyz = read_pcd(fixture['source']).xyz
    planning_xyz = read_pcd(fixture['output']).xyz
    # The new row is outside both floor ownership bands, so their measured
    # evidence/anchors remain byte-identical. Its ROI is explicitly separate.
    source_xyz = np.vstack((source_xyz, [[0., 0., 1.2]]))
    planning_xyz = np.vstack((planning_xyz, [[0., 0., 1.2]]))
    write_pcd(fixture['source'], source_xyz)
    original = fixture['directory']/'original_planning.pcd'
    write_pcd(original, planning_xyz)
    planning_xyz[-1, 2] += .01
    write_pcd(fixture['output'], planning_xyz)
    source_xyz = read_pcd(fixture['source']).xyz
    planning_xyz = read_pcd(fixture['output']).xyz
    mapping = fixture['directory']/'stair_ground_mapping.npz'
    np.savez_compressed(mapping, source_indices=np.array([len(source_xyz)-1], dtype=np.int64),
        source_xyz=source_xyz[-1:].astype(np.float64),
        planning_xyz=planning_xyz[-1:].astype(np.float64), surface_id=np.array([1], dtype=np.int32))
    stage = json.loads(fixture['stage'].read_text())
    stage['source_sha256'] = digest_file(fixture['source'])
    stage['configuration']['protected_regions'] = [dict(min=[-.3, -.3, 1.], max=[.3, .3, 1.4])]
    write_json(fixture['stage'], stage)
    manifest = json.loads(fixture['manifest'].read_text())
    manifest.update(source_sha256=stage['source_sha256'],
        output_sha256=digest_file(fixture['output']),
        floor_processing_manifest_sha256=digest_file(fixture['stage']))
    manifest['stair_regularization'] = dict(schema='d1max.stair_regularization.v1',
        source_sha256=manifest['source_sha256'], input_map_file=original.name,
        input_map_sha256=digest_file(original), output_map_file=fixture['output'].name,
        output_map_sha256=manifest['output_sha256'], mapping_file=mapping.name,
        mapping_sha256=digest_file(mapping), surfaces=[dict(id=1, kind='flight_support_envelope')],
        max_displacement_m=.08, association_max_distance_m=.12)
    write_json(fixture['manifest'], manifest)
    rewrite_tomogram(fixture['tomogram'], original_source_sha256=manifest['source_sha256'],
        source_sha256=manifest['output_sha256'],
        source_processing_manifest_sha256=digest_file(fixture['manifest']),
        stair_regularization_schema='d1max.stair_regularization.v1',
        stair_mapping_sha256=digest_file(mapping))
    return original, mapping


def test_stair_package_freezes_correspondence_and_original_planning_input(conditioned_package):
    original, mapping = with_stair_mapping(conditioned_package)
    package = validate_package(conditioned_package['directory'])
    assert package['dependency_files'][str(original)] == digest_file(original)
    assert package['dependency_files'][str(mapping)] == digest_file(mapping)
    assert package['bridge'].stair_mapping is not None
    assert package['capabilities']['execution_geometry_eligible'] is False
    mapping.write_bytes(b'stale support mapping')
    with pytest.raises(ValueError, match='hash_mismatch'):
        validate_package(conditioned_package['directory'])


def test_original_riser_lineage_is_a_runtime_dependency_not_only_build_history(conditioned_package):
    fixture = conditioned_package
    with_stair_mapping(fixture)
    lineage = fixture['directory']/'record_lineage.npz'
    np.savez_compressed(lineage, retained_source_indices=np.arange(51, dtype=np.int64))
    manifest = json.loads(fixture['manifest'].read_text())
    manifest['input_points'] = 51
    manifest['support_consistency_repair'] = dict(captured_at_build_inputs={
        '/historical/not-a-runtime-search-path/record_lineage.npz': digest_file(lineage)})
    write_json(fixture['manifest'], manifest)
    rewrite_tomogram(fixture['tomogram'], source_processing_manifest_sha256=digest_file(fixture['manifest']))
    package = validate_package(fixture['directory'])
    assert package['dependency_files'][str(lineage)] == digest_file(lineage)
    np.savez_compressed(lineage, retained_source_indices=np.arange(50, dtype=np.int64))
    with pytest.raises(ValueError, match='raw_stair_record_lineage_changed'):
        validate_package(fixture['directory'])


@pytest.mark.parametrize('field', ['stair_regularization_schema', 'stair_mapping_sha256'])
def test_stair_tomogram_cannot_use_old_correspondence(conditioned_package, field):
    with_stair_mapping(conditioned_package)
    rewrite_tomogram(conditioned_package['tomogram'], **{field: 'old'})
    with pytest.raises(ValueError, match='stair_tomogram_binding_invalid'):
        validate_package(conditioned_package['directory'])


def test_conditioned_package_has_exact_two_floor_planning_capabilities_and_literal_dependencies(
        conditioned_package, monkeypatch):
    import subprocess
    from d1max_pct_planner import crossfloor_preview
    def forbidden(*args, **kwargs):
        raise AssertionError('package validation must not launch processes or a native planner')
    monkeypatch.setattr(subprocess, 'Popen', forbidden)
    monkeypatch.setattr(subprocess, 'run', forbidden)
    monkeypatch.setattr(crossfloor_preview, 'CrossfloorPreviewRoute', forbidden)
    fixture = conditioned_package
    expected_paths = {fixture[key].resolve() for key in (
        'source', 'source_manifest', 'output', 'manifest', 'stage', 'trajectory',
        'evidence', 'route', 'tomogram')}
    expected_paths.update(path.resolve() for name, path in fixture['linked'].items()
                          if name not in ('floor_conditioning_config', 'route_template'))
    before = {str(path): digest_file(path) for path in expected_paths}
    bound = validate_package(fixture['directory'])
    assert bound['capabilities'] == dict(geometry_kind='conditioned_multifloor_ground',
        floor_ids=['floor1', 'floor2'], stairs_planning_enabled=True,
        execution_geometry_eligible=False, planning_frame='d1max_multifloor_planning')
    assert bound['dependency_files'] == before
    assert bound['rebuild_input_references'] == [
        dict(path=str(fixture['defaults']), sha256=digest_file(fixture['defaults'])),
        bound['manifest']['configuration']['floor_conditioning_config'],
        bound['manifest']['configuration']['route_template'],
    ]
    assert bound['tomogram_path'] == fixture['tomogram']
    assert bound['manifest']['physical_acceptance'] is False
    assert bound['tomogram'].unknown_ceiling_policy == 'reject'
    assert bound['tomogram'].max_ground_step_m == .17
    assert {path: digest_file(path) for path in before} == before


def test_conditioned_ground_bridge_changes_only_supported_ground_z_not_body_or_tf(conditioned_package):
    bound = validate_package(conditioned_package['directory'])
    bridge = bound['bridge']
    assert bridge.source_frame == 'd1max_loc_map'
    assert bridge.planning_frame == 'd1max_multifloor_planning'
    for floor, source_z, planning_z in (('floor1', -.75, -.65), ('floor2', 3., 3.15)):
        ground = np.array([[.05, .02, source_z]])
        projected = bridge.to_planning_ground(ground, [floor])
        np.testing.assert_array_equal(projected.xyz[:, :2], ground[:, :2])
        assert projected.xyz[0, 2] == pytest.approx(planning_z)
        recovered = bridge.to_localization_ground(projected.xyz, [floor])
        np.testing.assert_allclose(recovered.xyz, ground, atol=1e-10)
        assert recovered.diagnostics['exact_inverse_of_snapped_records'] is False
        body = ground[0]+[0., 0., .55]
        with pytest.raises(ValueError, match='source point is not observed ground'):
            bridge.to_planning_ground([body], [floor])
        queried = bridge.project_live_pose_to_ground(body, floor,
            body_height_interval_m=(.4, .7))
        assert queried.diagnostics['ground_only'] is True
        assert queried.diagnostics['certified_geometric_error_bound_m'] is None
        np.testing.assert_allclose(queried.diagnostics['source_ground_xyz'], ground[0])


def test_conditioned_geometry_never_becomes_execution_package(conditioned_package):
    loaded=validate_package(conditioned_package['directory'], purpose='execution')
    preview=validate_package(conditioned_package['directory'], purpose='planning_only')
    assert loaded['capabilities']==preview['capabilities']
    assert loaded['capabilities']['execution_geometry_eligible'] is False
    assert loaded['manifest']['planning_only'] is True
    assert loaded['dependency_files']==preview['dependency_files']


@pytest.mark.parametrize('field', ['source_pcd', 'tomogram_path'])
def test_equal_bytes_foreign_route_artifact_cannot_replace_local_binding(conditioned_package, field):
    fixture = conditioned_package
    route = yaml.safe_load(fixture['route'].read_text())
    source = fixture['output'] if field == 'source_pcd' else fixture['tomogram']
    foreign = fixture['directory'].parent/('foreign'+source.suffix)
    shutil.copyfile(source, foreign)
    assert digest_file(source) == digest_file(foreign)
    route[field] = str(foreign)
    fixture['route'].write_text(yaml.safe_dump(route))
    with pytest.raises(ValueError, match='conditioned_multifloor_foreign_route_artifact'):
        validate_package(fixture['directory'])


def test_conditioned_runtime_uses_canonical_pct_tomogram_not_equal_bytes_at_package_root(conditioned_package):
    fixture = conditioned_package
    root_copy = fixture['directory']/'tomogram.npz'
    shutil.copyfile(fixture['tomogram'], root_copy)
    route = yaml.safe_load(fixture['route'].read_text())
    route['tomogram_path'] = str(root_copy)
    fixture['route'].write_text(yaml.safe_dump(route))
    with pytest.raises(ValueError, match='conditioned_multifloor_foreign_route_artifact'):
        validate_package(fixture['directory'])


def test_declared_output_cannot_escape_package_even_with_equal_bytes(conditioned_package):
    fixture = conditioned_package
    outside = fixture['directory'].parent/'outside.pcd'
    shutil.copyfile(fixture['output'], outside)
    manifest = json.loads(fixture['manifest'].read_text())
    manifest['output_file'] = '../outside.pcd'
    write_json(fixture['manifest'], manifest)
    rewrite_tomogram(fixture['tomogram'], source_processing_manifest_sha256=digest_file(fixture['manifest']))
    with pytest.raises(ValueError, match='navigation_map_artifact_must_be_package_relative'):
        validate_package(fixture['directory'])


@pytest.mark.parametrize('field,value', [
    ('source_processing_manifest_sha256', 'a'*64),
    ('original_source_sha256', 'b'*64),
    ('geometry_operation', 'source_identity'),
    ('processing_schema', 'another_schema'),
    ('planning_only', False),
    ('source_sha256', 'c'*64),
    ('frame_id', 'd1max_loc_map'),
])
def test_tomogram_foreign_identity_or_authority_fails_closed(conditioned_package, field, value):
    rewrite_tomogram(conditioned_package['tomogram'], **{field: value})
    with pytest.raises(ValueError):
        validate_package(conditioned_package['directory'])


@pytest.mark.parametrize('field,key', [
    ('source_processing_manifest', 'manifest'), ('source_pcd', 'output'),
    ('original_source_pcd', 'source'),
])
def test_tomogram_equal_bytes_do_not_allow_foreign_artifact_paths(conditioned_package, field, key):
    fixture = conditioned_package
    foreign = fixture['directory'].parent/('other_'+fixture[key].name)
    shutil.copyfile(fixture[key], foreign)
    rewrite_tomogram(fixture['tomogram'], **{field: str(foreign)})
    with pytest.raises(ValueError, match='conditioned_multifloor_tomogram_binding_invalid'):
        validate_package(fixture['directory'])


@pytest.mark.parametrize('dependency', [
    'source', 'output', 'stage', 'trajectory', 'evidence',
    'visibility', 'upper_evidence', 'upper_sources', 'lower_evidence', 'lower_sources',
])
def test_runtime_dependency_tamper_is_not_hidden_by_valid_tomogram(conditioned_package, dependency):
    fixture = conditioned_package
    path = fixture.get(dependency) or fixture['linked'][dependency]
    tomogram_hash = digest_file(fixture['tomogram'])
    path.write_bytes(path.read_bytes()+b'changed')
    with pytest.raises(ValueError, match='navigation_map_dependency_changed'):
        validate_package(fixture['directory'])
    assert digest_file(fixture['tomogram']) == tomogram_hash


@pytest.mark.parametrize('dependency', ['defaults', 'floor_conditioning_config', 'route_template'])
def test_historical_rebuild_config_keeps_original_lineage_not_false_runtime_hash(conditioned_package, dependency):
    fixture = conditioned_package
    path = fixture.get(dependency) or fixture['linked'][dependency]
    original_hash = digest_file(path)
    path.write_bytes(path.read_bytes()+b'current rebuild config changed')
    bound = validate_package(fixture['directory'])
    reference = next(value for value in bound['rebuild_input_references'] if value['path'] == str(path))
    assert reference['sha256'] == original_hash
    assert reference['sha256'] != digest_file(path)
    assert str(path) not in bound['dependency_files']
    assert bound['capabilities']['execution_geometry_eligible'] is False


@pytest.mark.parametrize('dependency', ['defaults', 'floor_conditioning_config', 'route_template'])
def test_rebuild_lineage_must_still_have_exact_typed_original_hash(conditioned_package, dependency):
    fixture = conditioned_package
    manifest = json.loads(fixture['manifest'].read_text())
    if dependency == 'defaults':
        stage = json.loads(fixture['stage'].read_text())
        stage['input_hashes'][str(fixture['defaults'])] = 'not a sha256'
        write_json(fixture['stage'], stage)
        manifest['floor_processing_manifest_sha256'] = digest_file(fixture['stage'])
    else:
        manifest['configuration'][dependency]['sha256'] = 'not a sha256'
    write_json(fixture['manifest'], manifest)
    rewrite_tomogram(fixture['tomogram'], source_processing_manifest_sha256=digest_file(fixture['manifest']))
    with pytest.raises(ValueError, match='conditioned_multifloor_rebuild_lineage_invalid'):
        validate_package(fixture['directory'])


def test_rehashed_cached_height_cannot_replace_reconstructed_source_ground(conditioned_package):
    fixture = conditioned_package
    with np.load(fixture['evidence'], allow_pickle=False) as archive:
        payload = {key: archive[key].copy() for key in archive.files}
    payload['floor2_anchors_z'] += .02
    np.savez_compressed(fixture['evidence'], **payload)
    manifest = json.loads(fixture['manifest'].read_text())
    manifest['configuration']['floor_evidence_sha256'] = digest_file(fixture['evidence'])
    write_json(fixture['manifest'], manifest)
    rewrite_tomogram(fixture['tomogram'], source_processing_manifest_sha256=digest_file(fixture['manifest']))
    # Even mutually consistent packaging hashes cannot certify an invented
    # support field. The real bridge reconstructs it from literal source PCDs.
    with pytest.raises(ValueError, match='reconstructed anchors disagree'):
        validate_package(fixture['directory'])


def test_source_identity_retains_original_validator_and_execution_capability(tmp_path):
    from test_source_identity_package import package
    _, directory = package(tmp_path)
    bound = validate_package(directory, purpose='execution')
    assert bound['capabilities'] == dict(geometry_kind='source_identity', floor_ids=['floor1'],
        stairs_planning_enabled=False, execution_geometry_eligible=True,
        planning_frame='d1max_loc_map')
    assert bound['bridge'].manifest['physical_acceptance'] is False
    source = json.loads((directory/'manifest.json').read_text())['source_path']
    assert bound['dependency_files'][source] == digest_file(source)
    (directory/'processed_map.pcd').write_bytes(b'not original records')
    with pytest.raises(ValueError, match='source_identity_artifact_changed'):
        validate_package(directory, purpose='execution')
