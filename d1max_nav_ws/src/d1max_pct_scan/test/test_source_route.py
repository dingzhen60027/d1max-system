from copy import deepcopy
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


@pytest.mark.parametrize('layers', [[4.1, 4., 4.], [True, True, True], [4, 4], [-1, 4, 4]])
def test_original_layer_ids_are_never_guessed_by_numeric_truncation(layers):
    with pytest.raises(ValueError, match='layer_ownership'):
        builder().build(dict(route_type='same_floor', layer_ids=[4]*3, source_layer_ids=layers),
            [[0., 0., 0.], [.5, 0., 0.], [1., 0., 0.]], ['floor1']*3, map_version_id='map')
