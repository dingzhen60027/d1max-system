"""Artifact consistency, not motion approval; all test points are synthetic."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial import cKDTree

from d1max_pct_scan.structured_stair_evidence import validate_structured_evidence


def evidence():
    stations = np.r_[np.arange(0., 1., .05), 1.]
    lateral = np.linspace(-.25, .25, 11)
    grid = np.array([[s, t, .5*s] for s in stations for t in lateral])
    xyz = grid.copy()
    xyz[:, 2] += .003*np.sin(np.arange(len(xyz)))
    ids = np.arange(100, 100+len(xyz), dtype=np.int64)
    immutable = xyz[:, 0] < .1
    planning = xyz.copy()
    planning[~immutable, 2] = grid[~immutable, 2]
    arrays = dict(source_indices=ids, source_xyz=xyz, planning_xyz=planning,
        surface_id=np.ones(len(xyz), np.int32), source_normal_z=np.full(len(xyz), .89),
        source_plane_rms_m=np.full(len(xyz), .01), immutable_existing_mapping=immutable,
        source_is_new_mapping_row=~immutable, grid_model_xyz=grid,
        grid_surface_id=np.ones(len(grid), np.int32),
        grid_retained_entity_blocked=np.zeros(len(grid), bool))
    arrays['grid_retained_entity_blocked'][100] = True
    flight = dict(surface_id=1, leg='stair_lower',
        frame=dict(center_xy=[0., 0.], uphill_axis_xy=[1., 0.], along_range_m=[0., 1.]),
        height_model=dict(kind='planning_support_envelope_not_physical_treads', grade=.5, intercept_m=0.),
        retained_structure=dict(sides=dict(
            negative=dict(lateral_slope_intercept=[0., -.30]),
            positive=dict(lateral_slope_intercept=[0., .30]))), wall_boundary_guard_m=.05,
        connectivity=dict(policy='same_surface_measured_3d_component_with_existing_support_anchor',
                          neighbour_radius_m=.16))
    model = dict(schema='d1max.structured_stair_support.v1', source_sha256='a'*64,
        planning_sha256='b'*64, source_records_are_literal=True, model_samples_are_observations=False,
        original_source_modified=False, runtime_collision_authority_unchanged=True,
        policy=dict(grid_spacing_m=.05, source_support_radius_m=.12, wall_boundary_guard_m=.05,
            maximum_support_residual_m=.08, support_neighbor_m=.16, endpoint_blend_m=.15,
            retained_entity_height_m=.08, retained_entity_top_m=.70, retained_entity_radius_m=.05,
            maximum_grid_points=250000), flights=[flight])
    return refresh(model, arrays)


def refresh(model, arrays):
    """Recompute declarations for negative structural cases, not fake verdicts."""
    xyz, grid = arrays['source_xyz'], arrays['grid_model_xyz']
    distance, near = cKDTree(xyz).query(grid)
    arrays['grid_source_distance_m'] = distance
    arrays['grid_nearest_source_index'] = arrays['source_indices'][near]
    arrays['grid_source_covered'] = distance <= model['policy']['source_support_radius_m']
    arrays['grid_available'] = arrays['grid_source_covered'] & ~arrays['grid_retained_entity_blocked']
    f = model['flights'][0]
    f.update(connected_support_points=len(xyz), new_source_rows=int(arrays['source_is_new_mapping_row'].sum()),
        immutable_existing_mapping_rows=int(arrays['immutable_existing_mapping'].sum()), model_samples=len(grid),
        observed_supported_samples=int(arrays['grid_source_covered'].sum()),
        excluded_by_retained_entities=int((arrays['grid_source_covered'] & arrays['grid_retained_entity_blocked']).sum()),
        available_model_samples=int(arrays['grid_available'].sum()))
    f['connectivity'].update(connected_points=len(xyz), anchor_points=int(arrays['immutable_existing_mapping'].sum()))
    mapping = SimpleNamespace(source_hash=model['source_sha256'], source_indices=arrays['source_indices'].copy(),
        source_xyz=xyz.copy(), planning_xyz=arrays['planning_xyz'].copy(), surface_ids=arrays['surface_id'].copy())
    return model, arrays, mapping


def test_returns_literal_source_not_model_and_has_no_shared_mutable_storage():
    model, arrays, mapping = evidence()
    before = deepcopy(arrays)
    result = validate_structured_evidence(model, arrays, mapping)
    np.testing.assert_array_equal(result, arrays['source_xyz'])
    assert not np.array_equal(result, arrays['grid_model_xyz'])
    assert not np.shares_memory(result, arrays['source_xyz'])
    # This base validator deliberately does NOT turn entity indices into a
    # free-space proof. The caller's qualified cost/entity intersection remains.
    assert arrays['grid_retained_entity_blocked'].any()
    for key in arrays:
        np.testing.assert_array_equal(arrays[key], before[key])


@pytest.mark.parametrize('mutation', ['owner_zero', 'owner_foreign', 'grid_shift', 'nearest_foreign',
    'available_without_cover', 'source_nonliteral', 'normal_nan', 'normal_wall', 'rms_bad',
    'source_xy_changed', 'source_z_large', 'immutable_changed', 'flags', 'grid_owner',
    'distance', 'coverage', 'grid_shape', 'grid_bool', 'duplicate_source', 'model_authority',
    'source_hash', 'count', 'plane', 'boundary', 'grid_spacing_tiny', 'anchors'])
def test_rejects_internally_inconsistent_even_if_file_hashes_were_recomputed(mutation):
    model, a, mapping = evidence()
    if mutation == 'owner_zero': a['surface_id'][:] = 0
    elif mutation == 'owner_foreign': mapping.surface_ids[:] = 3
    elif mutation == 'grid_shift': a['grid_model_xyz'] += 100
    elif mutation == 'nearest_foreign': a['grid_nearest_source_index'][:] = 2**40
    elif mutation == 'available_without_cover': a['grid_source_covered'][:] = False; a['grid_available'][:] = True
    elif mutation == 'source_nonliteral': a['source_xyz'][4, 2] += .001
    elif mutation == 'normal_nan': a['source_normal_z'][0] = np.nan
    elif mutation == 'normal_wall': a['source_normal_z'][0] = .5
    elif mutation == 'rms_bad': a['source_plane_rms_m'][0] = .04
    elif mutation == 'source_xy_changed': a['planning_xyz'][0, 0] += .001
    elif mutation == 'source_z_large': a['planning_xyz'][0, 2] += .081
    elif mutation == 'immutable_changed': mapping.planning_xyz[0, 2] += .001
    elif mutation == 'flags': a['immutable_existing_mapping'][:] = False
    elif mutation == 'grid_owner': a['grid_surface_id'][:] = 2
    elif mutation == 'distance': a['grid_source_distance_m'][:] = 0.
    elif mutation == 'coverage': a['grid_source_covered'][:] = False; a['grid_available'][:] = False
    elif mutation == 'grid_shape': a['grid_nearest_source_index'] = a['grid_nearest_source_index'][:, None]
    elif mutation == 'grid_bool': a['grid_available'] = a['grid_available'].astype(np.uint8)
    elif mutation == 'duplicate_source': a['source_indices'][1] = a['source_indices'][0]
    elif mutation == 'model_authority': model['model_samples_are_observations'] = True
    elif mutation == 'source_hash': mapping.source_hash = 'c'*64
    elif mutation == 'count': model['flights'][0]['model_samples'] += 1
    elif mutation == 'plane': model['flights'][0]['height_model']['intercept_m'] += .5
    elif mutation == 'boundary': model['flights'][0]['retained_structure']['sides']['positive']['lateral_slope_intercept'][1] = -.5
    elif mutation == 'grid_spacing_tiny': model['policy']['grid_spacing_m'] = 1e-100
    elif mutation == 'anchors': mapping.surface_ids[:] = 0
    with pytest.raises(ValueError, match='structured_stair_evidence_'):
        validate_structured_evidence(model, a, mapping)


def test_unmodified_owner_zero_rows_are_allowed_but_not_anchors():
    model, a, mapping = evidence()
    mapping.surface_ids[~a['immutable_existing_mapping']] = 0
    np.testing.assert_array_equal(validate_structured_evidence(model, a, mapping), a['source_xyz'])


def test_new_proposal_is_not_confused_with_slice_preserving_final_mapping():
    model, a, mapping = evidence()
    mapping.planning_xyz[~a['immutable_existing_mapping']] = mapping.source_xyz[~a['immutable_existing_mapping']]
    np.testing.assert_array_equal(validate_structured_evidence(model, a, mapping), a['source_xyz'])


def test_disconnected_measured_island_cannot_use_grid_as_bridge():
    model, a, _ = evidence()
    keep = (a['source_xyz'][:, 0] < .35) | (a['source_xyz'][:, 0] > .65)
    for name in ('source_indices', 'source_xyz', 'planning_xyz', 'surface_id', 'source_normal_z',
                 'source_plane_rms_m', 'immutable_existing_mapping', 'source_is_new_mapping_row'):
        a[name] = a[name][keep]
    model, a, mapping = refresh(model, a)
    with pytest.raises(ValueError, match='unanchored_source_component'):
        validate_structured_evidence(model, a, mapping)


def test_source_above_plane_is_not_ground_despite_mapping_membership():
    model, a, _ = evidence()
    a['source_xyz'][50, 2] += .15
    a['planning_xyz'][50, 2] += .15
    model, a, mapping = refresh(model, a)
    with pytest.raises(ValueError, match='source_outside_flight'):
        validate_structured_evidence(model, a, mapping)
