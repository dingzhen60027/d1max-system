from copy import deepcopy
import json
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_scan.source_route import RouteSnapshot, SourceRouteBuilder, SourceSupport


class Bridge:
    source_frame = 'd1max_loc_map'
    planning_frame = 'd1max_multifloor_planning'

    def to_localization_ground(self, points, labels):
        points = np.array(points, dtype=float, copy=True)
        # Spatially varying displacement: deliberately NOT a rigid transform.
        points[:, 2] += .1*points[:, 0]
        return SimpleNamespace(xyz=points, diagnostics=dict(
            ground_path_only=True, exact_inverse_of_snapped_records=False,
            certified_geometric_error_bound_m=None))


def builder(support=None):
    return SourceRouteBuilder(bridge=Bridge(), source_map_sha256='a'*64,
        conditioning_sha256='b'*64, tomogram_sha256='c'*64, support=support)


def example(floor='floor1', support=None):
    z, layer = (0., 4) if floor == 'floor1' else (3.8, 12)
    points = [[0., 0., z], [.5, 0., z], [1., 0., z]]
    result = dict(route_type='same_floor', layer_ids=[layer]*3, source_layer_ids=[layer]*3)
    return builder(support).build(result, points, [floor]*3, map_version_id='original-map-1')


def test_same_xy_different_floors_preserve_source_layers_and_hash():
    low, high = example(), example('floor2')
    assert low.route_hash != high.route_hash
    assert low.payload()['point_floor_ids'] == ['floor1']*3
    assert high.payload()['point_floor_ids'] == ['floor2']*3
    assert high.payload()['source_layer_ids'] == [12]*3
    assert high.payload()['xyz'][1][2] == pytest.approx(3.85)


def test_nonrigid_projection_is_ground_only_not_tf_and_never_fake_certification():
    value = example().payload()
    assert [p[2] for p in value['xyz']] == [0., .05, .1]
    assert value['point_reference'] == 'ground'
    assert value['geometry_evidence']['projection_kind'] == 'non_rigid_ground_only_not_tf'
    assert value['geometry_evidence']['certified_geometric_error_bound_m'] is None
    assert value['geometry_evidence']['ground_to_body_height_applications'] == 0
    assert value['preview_ready'] and not value['execution_eligible']
    assert value['eligibility_reason'] == 'source_support_missing'


def test_snapshot_is_immutable_hash_stable_and_independent_of_callback_clocks():
    first = example()
    altered = first.payload()
    altered['xyz'][0][0] = 33.
    assert first.payload()['xyz'][0][0] == 0.
    assert example().route_hash == first.route_hash
    assert RouteSnapshot.create(first.payload()).route_hash == first.route_hash
    assert first.with_goal(has_goal_yaw=True, goal_yaw=.4).route_hash != first.route_hash


@pytest.mark.parametrize('schema', [0, 1, 3, '2'])
def test_old_or_unknown_schema_is_rejected(schema):
    data = example().payload()
    data['schema_version'] = schema
    with pytest.raises(ValueError, match='schema_2'):
        RouteSnapshot.create(data)


def test_measured_support_has_interior_coverage_and_cannot_be_claimed_as_clearance():
    # Endpoints alone must not hide a real unobserved 0.8 m hole in the middle.
    sparse = SourceSupport({'floor1': [[0., 0., 0.], [1., 0., .1]]}, 'a'*64)
    data = example(support=sparse).payload()
    audit = data['geometry_evidence']['measured_source_support']
    assert audit['unsupported_samples'] > 0 and not audit['all_samples_supported']
    assert not data['execution_eligible']
    dense = [[x, 0., .1*x] for x in np.linspace(0, 1, 101)]
    supported = example(support=SourceSupport({'floor1': dense}, 'a'*64)).payload()
    assert supported['geometry_evidence']['measured_source_support']['all_samples_supported']
    assert not supported['execution_eligible']  # no swept-volume/physical proof
    assert 'error_bound' in supported['eligibility_reason']


def test_other_floor_support_does_not_make_same_xy_ground_observed():
    support = SourceSupport({'floor2': [[x, 0., .1*x+3.8] for x in np.linspace(0, 1, 100)]}, 'a'*64)
    assert example(support=support).payload()['eligibility_reason'] == 'source_support_missing'


def test_forged_authority_or_changed_geometry_is_rejected():
    value = example().payload()
    value['execution_eligible'] = True
    with pytest.raises(ValueError, match='geometry_authority'):
        RouteSnapshot.create(value)
    value = example().payload()
    value['geometry_evidence']['certified_geometric_error_bound_m'] = .15
    with pytest.raises(ValueError, match='geometry_authority'):
        RouteSnapshot.create(value)


@pytest.mark.parametrize('direction', ['lower_to_upper', 'upper_to_lower'])
def test_crossfloor_segments_portals_and_direction_survive(direction):
    points = [[i*.1, 0., z] for i, z in enumerate([0., 0., .4, .8, .8])]
    names = ['lower_floor', 'stair_lower', 'stair_upper', 'upper_floor']
    labels = ['floor1', 'stair_lower', 'stair_upper', 'floor2', 'floor2']
    portals = ['start', 'entry', 'landing', 'exit', 'goal']
    layers = [4, 4, 8, 12, 12]
    if direction == 'upper_to_lower':
        points, labels, layers = points[::-1], labels[::-1], layers[::-1]
        names, portals = names[::-1], portals[::-1]
        ownership = {'lower_floor':'floor1','upper_floor':'floor2',
                     'stair_lower':'stair_lower','stair_upper':'stair_upper'}
        labels = [ownership[name] for name in names]+[ownership[names[-1]]]
    raw = dict(layer_ids=layers, source_layer_ids=layers, direction=direction,
        segments=[dict(name=name, first_index=i, last_index=i+1,
                       **{'from': portals[i], 'to': portals[i+1]}) for i, name in enumerate(names)])
    value = builder().build(raw, points, labels, map_version_id='map').payload()
    assert value['edge_segments'] == names
    assert value['segments'][1]['entry_portal_id'] == portals[1]
    assert value['segments'][1]['kind'] == ('stair_up' if direction == 'lower_to_upper' else 'stair_down')
    assert all(x['direction'] == direction for x in value['segments'])


def test_uncovered_route_edges_are_not_silently_promoted():
    value = example().payload()
    value['segments'][0]['end_index'] = 1
    with pytest.raises(ValueError, match='uncovered'):
        RouteSnapshot.create(value)


def test_source_support_belongs_to_exact_original_map():
    with pytest.raises(ValueError, match='hash_mismatch'):
        builder(SourceSupport({}, 'd'*64))


def test_artifact_builder_passes_mapping_and_keeps_measured_source_ownership(tmp_path):
    from d1max_pct_scan.pointcloud_helpers.ground_path_bridge import StairGroundMapping
    from d1max_pct_scan.source_route import digest_file
    source = np.array([[0.,0.,0.],[.1,0.,0.],[0.,0.,3.],[.1,0.,3.],
                       [.5,0.,1.],[.6,0.,1.],[.5,0.,1.6]],dtype=np.float64)
    path = tmp_path/'source.pcd'
    path.write_bytes(('VERSION .7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\n'
        'COUNT 1 1 1\nWIDTH 7\nHEIGHT 1\nPOINTS 7\nDATA binary\n').encode()+source.tobytes())
    source_hash = digest_file(path)
    stage = tmp_path/'stage.json'; stage.write_text('{}')
    (tmp_path/'audit').mkdir()
    evidence = tmp_path/'audit/floor_evidence.npz'
    np.savez(evidence, floor1_source_indices=np.array([0,1]),floor1_support=np.ones(2,dtype=bool),
        floor2_source_indices=np.array([2,3]),floor2_support=np.ones(2,dtype=bool))
    manifest = tmp_path/'manifest.json'
    manifest.write_text(json.dumps(dict(source_path=str(path),source_sha256=source_hash,
        floor_processing_manifest=str(stage),floor_processing_manifest_sha256=digest_file(stage),
        configuration=dict(floor_evidence_sha256=digest_file(evidence)))))
    original = source[[4,5]]
    mapping = StairGroundMapping(dict(source_sha256=source_hash,mapping_sha256='b'*64,
        surfaces=[dict(id=1,kind='landing_plane')],max_displacement_m=.08,
        association_max_distance_m=.12),dict(source_indices=np.array([4,5],dtype=np.int64),
        source_xyz=original,planning_xyz=original+[0.,0.,.08],surface_id=np.ones(2,dtype=np.int32)),
        source_xyz=source)
    bridge = SimpleNamespace(source_frame='d1max_loc_map',planning_frame='d1max_multifloor_planning',
        floors={'floor1':None,'floor2':None},protected_regions=[dict(min=[.4,-1.,.5],max=[.7,1.,2.])],
        stair_mapping=mapping)
    built = SourceRouteBuilder.from_artifacts(manifest,SimpleNamespace(sha256='c'*64),bridge)
    assert built.support.stair_mapping is mapping
    assert np.array_equal(built.support.trees['stairs'].data,original)
    assert np.array_equal(built.support.trees['floor1'].data,source[:2])
    assert set(map(tuple,built.support.trees['stair_upper'].data)) == set(map(tuple,source[[2,3,4,5]]))
    assert tuple(source[6]) not in set(map(tuple,built.support.trees['stairs'].data))


@pytest.mark.parametrize('layers', [[4.1, 4., 4.], [True, True, True], [4, 4], [-1, 4, 4]])
def test_original_layer_ids_are_never_guessed_by_numeric_truncation(layers):
    with pytest.raises(ValueError, match='layer_ownership'):
        builder().build(dict(route_type='same_floor', layer_ids=[4]*3, source_layer_ids=layers),
            [[0., 0., 0.], [.5, 0., 0.], [1., 0., 0.]], ['floor1']*3, map_version_id='map')
