import copy
import json

import numpy as np
import pytest

from tools.pointcloud_preprocessing.stair_structured_support import (
    StructuredSupportPolicy, build_structured_stair_support,
)


def fixture(*, gap=False, obstacle=False, second_floor=False):
    x, y = np.meshgrid(np.arange(-.70, .701, .04), np.arange(.04, 3.961, .04))
    floor = np.c_[x.ravel(), y.ravel(), .5*y.ravel()+.009*np.sin(y.ravel()*9+x.ravel()*5)]
    if gap:
        floor = floor[(floor[:, 1] < 1.3) | (floor[:, 1] > 1.8)]
    along, height = np.meshgrid(np.arange(.02, 3.99, .04), np.arange(.26, 1.24, .04))
    walls = [np.c_[np.full(along.size, side), along.ravel(), .5*along.ravel()+height.ravel()]
             for side in (-.75, .75)]
    parts = [floor, *walls]
    if obstacle:
        bx, by = np.meshgrid(np.arange(-.12, .13, .04), np.arange(1.9, 2.13, .04))
        parts.append(np.c_[bx.ravel(), by.ravel(), .5*by.ravel()+.25])
    if second_floor:
        parts.append(floor+[0, 0, 3.])
    source = np.concatenate(parts)
    ids = np.flatnonzero(abs(floor[:, 0]) < .24)
    planning = source.copy()
    # Deliberately retain a prior correction exactly rather than re-fit it.
    planning[ids[:5], 2] += .003
    mapping = dict(source_indices=ids, source_xyz=source[ids].copy(),
        planning_xyz=planning[ids].copy(), surface_id=np.ones(len(ids), np.int32))
    models = [dict(id=1, name='lower', kind='flight_support_envelope', leg='stair_lower',
        center_xy=[0., 2.], uphill_axis_xy=[0., 1.], grade=.5, intercept_m=1.,
        along_range_m=[-1.94, 1.94], interior_lateral_range_m=[-.24, .24],
        scope_minimum=[-.3, .04, 0.], scope_maximum=[.3, 3.96, 2.1])]
    return source, planning, mapping, models, len(floor)


def run(data, *, retained=None, policy=None, measured_flights=None):
    source, planning, mapping, models, _ = data
    return build_structured_stair_support(source, planning,
        np.arange(len(source), dtype=np.int64) if retained is None else retained,
        mapping, models, source_sha256='a'*64, planning_sha256='b'*64,
        policy=policy, measured_flights=measured_flights)


def test_real_source_support_expands_to_measured_sides_not_old_cost_footprint():
    data = fixture()
    before = data[0].copy(), data[1].copy()
    result = run(data)
    addition, model = result['addition'], result['model']
    assert len(addition.source_indices) > 1000
    assert abs(addition.source_xyz[:, 0]).max() > .60
    assert not np.intersect1d(addition.source_indices, data[2]['source_indices']).size
    assert np.array_equal(addition.source_xyz, data[0][addition.source_indices])
    assert np.array_equal(addition.source_xyz[:, :2], addition.planning_xyz[:, :2])
    assert np.max(abs(addition.planning_xyz[:, 2]-addition.source_xyz[:, 2])) <= .08
    assert model['flights'][0]['available_outside_previous_band'] > 1000
    assert min(model['flights'][0]['width_after_guard_at_endpoints_m']) > 1.3
    assert not model['model_samples_are_observations']
    stations = result['arrays']['grid_model_xyz'][:, 1]-2.
    assert stations.min() >= -1.94
    assert stations.max() <= 1.94
    assert json.loads(json.dumps(model)) == model
    np.testing.assert_array_equal(data[0], before[0])
    np.testing.assert_array_equal(data[1], before[1])


def test_existing_mapping_rows_and_previous_planning_z_are_immutable():
    data = fixture()
    arrays = run(data)['arrays']
    positions = {int(i): k for k, i in enumerate(arrays['source_indices'])}
    for i, value in zip(data[2]['source_indices'], data[2]['planning_xyz']):
        if int(i) in positions:
            np.testing.assert_array_equal(arrays['planning_xyz'][positions[int(i)]], value)
    assert arrays['immutable_existing_mapping'].any()


def test_walls_other_floor_and_actual_elevated_object_not_support_or_erased():
    data = fixture(obstacle=True, second_floor=True)
    result = run(data)
    assert result['addition'].source_indices.max() < data[4]
    arrays = result['arrays']
    middle = (abs(arrays['grid_model_xyz'][:, 0]) < .08) & (abs(arrays['grid_model_xyz'][:, 1]-2.) < .08)
    assert middle.any()
    assert arrays['grid_retained_entity_blocked'][middle].all()
    assert not arrays['grid_available'][middle].any()
    assert result['model']['flights'][0]['retained_entity_planning_indices']


def test_real_missing_support_gap_is_not_synthesized_by_wide_rectangle():
    result = run(fixture(gap=True))
    arrays = result['arrays']
    middle = (arrays['grid_model_xyz'][:, 1] > 1.44) & (arrays['grid_model_xyz'][:, 1] < 1.66)
    assert middle.any()
    assert not arrays['grid_source_covered'][middle].any()
    assert not arrays['grid_available'][middle].any()
    assert not np.any((result['addition'].source_xyz[:, 1] > 1.3) & (result['addition'].source_xyz[:, 1] < 1.8))


def test_removed_original_ghost_is_not_reintroduced_as_current_entity_or_source_row():
    data = fixture(obstacle=True)
    source, planning, mapping, models, floor_count = data
    # Retain floor and walls only. The old raw box cannot return as an entity.
    # Derive exact box count from the original source rather than fixture rounding.
    box = (abs(source[:, 0]) < .2) & (abs(source[:, 1]-2.) < .2) & (source[:, 2]-.5*source[:, 1] > .20)
    retained = np.flatnonzero(~box)
    altered = (source, planning[retained], mapping, models, floor_count)
    result = run(altered, retained=retained)
    assert not np.isin(result['addition'].source_indices, np.flatnonzero(box)).any()
    assert not result['arrays']['grid_retained_entity_blocked'].any()


def test_missing_measured_side_does_not_fallback_to_pct_contour():
    data = fixture()
    source, planning, mapping, models, floor_count = data
    retained = np.flatnonzero(source[:, 0] > -.74)
    with pytest.raises(ValueError, match='measured_structure_side_missing'):
        run((source, planning[retained], mapping, models, floor_count), retained=retained)


def test_foreign_source_identity_or_duplicate_lineage_is_rejected():
    data = fixture()
    altered = list(data)
    altered[2] = copy.deepcopy(data[2])
    altered[2]['source_xyz'][0, 2] += .01
    with pytest.raises(ValueError, match='mapping_identity'):
        run(altered)
    with pytest.raises(ValueError, match='lineage'):
        run(data, retained=np.zeros(len(data[0]), np.int64))


@pytest.mark.parametrize('values', [dict(maximum_support_residual_m=.09),
    dict(source_support_radius_m=.2), dict(support_neighbor_m=.3),
    dict(grid_spacing_m=0), dict(maximum_grid_points=True)])
def test_policy_cannot_hide_unsupported_expansion(values):
    with pytest.raises(ValueError, match='policy'):
        StructuredSupportPolicy(**values)


def test_grid_memory_limit_is_bounded():
    with pytest.raises(ValueError, match='grid_work_limit'):
        run(fixture(), policy=StructuredSupportPolicy(maximum_grid_points=10))


def test_supplied_boundary_cannot_claim_foreign_points_or_height_model():
    from d1max_pct_planner.stair_centering import extract_measured_flight_structure
    data = fixture()
    flights = extract_measured_flight_structure(data[1], data[3])
    flights[0]['structure_xyz'][0, 0] += .01
    with pytest.raises(ValueError, match='retained_structure_identity'):
        run(data, measured_flights=flights)
    flights = extract_measured_flight_structure(data[1], data[3])
    flights[0]['grade'] += .01
    with pytest.raises(ValueError, match='retained_structure_identity'):
        run(data, measured_flights=flights)
