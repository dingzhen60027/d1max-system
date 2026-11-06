"""Document-backed facts versus explicit unvalidated preview assumptions.

Pure configuration tests only; no ROS, SDK, subprocess or hardware access.
"""
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from d1max_pct_scan.robot_profile import load_robot_profile, scan_robot_parameters, safety_scan_parameters


PROFILE = Path(__file__).resolve().parents[2] / 'd1max_scan_planner/config/d1max_robot.yaml'


@pytest.fixture
def profile():
    return load_robot_profile(PROFILE)


def test_official_dimensions_and_engineering_assumptions_keep_separate_provenance(profile):
    official, engineering = profile['official'], profile['engineering']
    assert official['standing_size_m'] == {'length': .930, 'width': .480, 'height': .585}
    assert engineering['body_reference_height_m'] == .50
    assert engineering['body_reference_height_m'] != official['standing_size_m']['height']
    assert engineering['mode'] == 'standing_preview_only'
    for flag in ('motion_authorized', 'body_height_calibrated', 'collision_envelope_validated',
                 'vertical_envelope_validated', 'sdk_speed_mapping_validated',
                 'planning_geometry_validated'):
        assert engineering[flag] is False
    assert official['sdk_speed_document_conflict'] is True
    assert Path(profile['sources']['specification']['path']).is_absolute()
    assert engineering['project_speed_limit_mps'] == 1.5
    assert engineering['preview_speed_mps'] == .3
    assert official['product_max_speed_mps'] > engineering['project_speed_limit_mps']


def test_native_footprint_is_derived_from_dimensions_and_explicit_margins(profile):
    profile['engineering']['collision_model'] = 'native_double_cylinder_approximation'
    value = scan_robot_parameters(profile)
    assert value['grid_map.double_cylinder_radius'] == pytest.approx(.29)
    assert value['grid_map.double_cylinder_offset'] == pytest.approx(.20)
    assert value['grid_map.body_height'] == .50
    assert value['grid_map.obstacles_inflation_z_up'] == .40
    assert value['grid_map.obstacles_inflation_z_down'] == .45
    assert value['grid_map.planning_obstacles_inflation_z_up'] == .12
    assert value['grid_map.planning_obstacles_inflation_z_down'] == .45
    assert not any('extrinsic' in key or 'sdk' in key for key in value)
    larger = deepcopy(profile)
    larger['official']['standing_size_m']['width'] = .50
    changed = scan_robot_parameters(larger)
    assert changed['grid_map.double_cylinder_radius'] == pytest.approx(.30)
    assert changed['grid_map.double_cylinder_offset'] == pytest.approx(.19)
    assert changed['grid_map.body_height'] == value['grid_map.body_height']


def test_legacy_single_layer_keeps_original_kernel_and_no_new_authority(profile):
    profile['engineering']['collision_model'] = 'native_double_cylinder_approximation'
    formal = scan_robot_parameters(profile)
    legacy = scan_robot_parameters(profile, strict_layer_available=False)
    assert legacy['grid_map.planning_obstacles_inflation_z_up'] == .40
    assert legacy['grid_map.planning_obstacles_inflation_z_down'] == .45
    for key, value in formal.items():
        if 'planning_obstacles_inflation' not in key:
            assert legacy[key] == value
    assert profile['engineering']['motion_authorized'] is False
    assert safety_scan_parameters(profile) == {'min_height': -.40, 'max_height': .80}


@pytest.mark.parametrize('capability', [None, 'true', 1, 0])
def test_collision_layer_capability_cannot_be_inferred_from_truthiness(profile, capability):
    with pytest.raises(ValueError, match='capability must be explicit'):
        scan_robot_parameters(profile, strict_layer_available=capability)


def test_low_obstacle_envelope_is_shared_with_final_safety_scan(profile):
    scan = scan_robot_parameters(profile)
    safety = safety_scan_parameters(profile)
    e = profile['engineering']
    assert e['ground_exclusion_height_m'] == .10
    assert e['body_reference_height_m'] + safety['min_height'] == pytest.approx(e['ground_exclusion_height_m'])
    assert safety['min_height'] <= -scan['grid_map.body_below']
    assert safety['max_height'] >= scan['grid_map.body_above']
    assert e['vertical_envelope_validated'] is False
    assert safety['min_height'] <= -scan['grid_map.planning_body_below']


def test_oriented_box_shared_by_native_candidate_and_raw_query(profile):
    e = profile['engineering']
    scan = scan_robot_parameters(profile)
    assert e['collision_model'] == 'native_oriented_box_approximation'
    assert scan['grid_map.collision_model'] == 'oriented_box'
    assert scan['grid_map.box_length'] == .93
    assert scan['grid_map.box_width'] == .30
    assert scan['grid_map.body_below'] + scan['grid_map.body_above'] == .50
    for side in ('below', 'above'):
        assert scan['grid_map.body_'+side] == scan['grid_map.planning_body_'+side]
    assert profile['official']['standing_size_m']['width'] == .48
    assert e['collision_envelope_validated'] is False


@pytest.mark.parametrize('clearance', [.025, .05])
def test_planar_clearance_expands_shared_box_without_changing_nominal_or_self_geometry(profile, clearance):
    original = deepcopy(profile)
    nominal = scan_robot_parameters(profile, strict_layer_available=False)
    padded = scan_robot_parameters(profile, strict_layer_available=False, planar_clearance_m=clearance)
    assert padded['grid_map.box_length'] == pytest.approx(.93 + 2. * clearance)
    assert padded['grid_map.box_width'] == pytest.approx(.30 + 2. * clearance)
    assert padded['grid_map.box_length'] / 2. - nominal['grid_map.box_length'] / 2. == pytest.approx(clearance)
    assert padded['grid_map.box_width'] / 2. - nominal['grid_map.box_width'] / 2. == pytest.approx(clearance)
    for key, value in nominal.items():
        if key not in ('grid_map.box_length', 'grid_map.box_width', 'grid_map.double_cylinder_radius'):
            assert padded[key] == value
    for side in ('below', 'above'):
        assert padded['grid_map.body_' + side] == padded['grid_map.planning_body_' + side] == .25
    assert profile == original
    assert scan_robot_parameters(profile, planar_clearance_m=0.) == scan_robot_parameters(profile)
    assert not any('clearance' in key for key in padded)


def test_planar_clearance_preserves_legacy_cylinder_centres_and_expands_radius(profile):
    profile['engineering']['collision_model'] = 'native_double_cylinder_approximation'
    nominal = scan_robot_parameters(profile, strict_layer_available=False)
    padded = scan_robot_parameters(profile, strict_layer_available=False, planar_clearance_m=.025)
    assert padded['grid_map.double_cylinder_radius'] == pytest.approx(.315)
    assert padded['grid_map.double_cylinder_offset'] == nominal['grid_map.double_cylinder_offset']
    for key, value in nominal.items():
        if key != 'grid_map.double_cylinder_radius':
            assert padded[key] == value


@pytest.mark.parametrize('clearance', [-.001, float('nan'), float('inf'), float('-inf'),
                                       1e308, True, False, None, '.025'])
def test_planar_clearance_rejects_invalid_config_not_by_truthiness(profile, clearance):
    with pytest.raises(ValueError, match='planar clearance'):
        scan_robot_parameters(profile, planar_clearance_m=clearance)


def test_planar_clearance_has_no_unrelated_small_cap(profile):
    padded = scan_robot_parameters(profile, planar_clearance_m=1.)
    assert padded['grid_map.box_length'] == pytest.approx(2.93)
    assert padded['grid_map.box_width'] == pytest.approx(2.30)


@pytest.mark.parametrize('clearance', [0., .025, .05])
def test_stair_clearance_preserves_every_flat_parameter_and_nominal_geometry(profile, clearance):
    original = deepcopy(profile)
    flat = scan_robot_parameters(profile, planar_clearance_m=.05)
    scoped = scan_robot_parameters(profile, planar_clearance_m=.05,
                                   stair_planar_clearance_m=clearance)
    assert {key: scoped[key] for key in flat} == flat
    assert set(scoped) - set(flat) == {
        'grid_map.box_nominal_length', 'grid_map.box_nominal_width',
        'grid_map.stair_planar_clearance'}
    assert scoped['grid_map.box_nominal_length'] == .93
    assert scoped['grid_map.box_nominal_width'] == .30
    assert scoped['grid_map.stair_planar_clearance'] == clearance
    assert profile == original
    assert scan_robot_parameters(profile, planar_clearance_m=.05,
                                 stair_planar_clearance_m=None) == flat


@pytest.mark.parametrize('clearance', [-.001, .05001, float('nan'), float('inf'),
                                      float('-inf'), True, False, '.025'])
def test_stair_clearance_rejects_invalid_or_larger_than_flat_margin(profile, clearance):
    with pytest.raises(ValueError, match='stair planar clearance'):
        scan_robot_parameters(profile, planar_clearance_m=.05,
                              stair_planar_clearance_m=clearance)


def test_stair_clearance_does_not_silently_change_legacy_cylinders(profile):
    profile['engineering']['collision_model'] = 'native_double_cylinder_approximation'
    with pytest.raises(ValueError, match='oriented box'):
        scan_robot_parameters(profile, planar_clearance_m=.05,
                              stair_planar_clearance_m=.025)


def test_support_height_does_not_resize_collision_kernels(tmp_path, profile):
    old_scan, old_safety = scan_robot_parameters(profile), safety_scan_parameters(profile)
    profile['engineering']['body_reference_height_m'] = .54
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    changed = load_robot_profile(path)
    new_scan = scan_robot_parameters(changed)
    assert new_scan['grid_map.body_height'] == .54
    assert {k: v for k, v in new_scan.items() if k != 'grid_map.body_height'} == {
        k: v for k, v in old_scan.items() if k != 'grid_map.body_height'}
    # Native body kernels stay immutable; the auxiliary flat-floor scan's
    # lower height follows the reference point so low obstacles stay visible.
    new_safety=safety_scan_parameters(changed)
    assert new_safety['max_height']==old_safety['max_height']
    assert changed['engineering']['body_reference_height_m']+new_safety['min_height']==pytest.approx(
        changed['engineering']['ground_exclusion_height_m'])


def test_cad_torso_proxy_and_operator_wheel_clarification_do_not_become_self_filter(profile):
    cad, e = profile['urdf_reference_only'], profile['engineering']
    assert cad['torso_bounds_base_m']['min'][2] == pytest.approx(-.08930)
    assert e['planning_body_below_m'] == pytest.approx(
        -cad['torso_bounds_base_m']['min'][2] + e['planning_torso_lower_margin_m'])
    assert cad['foot_mesh_outer_diameter_m'] == pytest.approx(.19191625714302063)
    assert cad['operator_wheel_dimensions_approx_m']['diameter'] == .19
    assert cad['operator_previous_wheel_diameter_description_m'] == .09
    assert cad['operator_wheel_dimensions_confirmed_at'] == '2026-10-10'
    assert cad['wheel_diameter_provenance_conflict'] is False
    assert cad['entity_self_filter_calibrated'] is False
    assert e['planning_geometry_provenance'] == 'cad_torso_with_unvalidated_margin'
    assert e['planning_geometry_validated'] is False


@pytest.mark.parametrize('key,value', [
    ('motion_authorized', True), ('mode', 'motion_control'),
    ('body_reference_height_m', float('nan')), ('footprint_lateral_margin_m', -.05),
    ('footprint_longitudinal_margin_m', True), ('footprint_lateral_margin_m', 1.),
    ('preview_speed_mps', .31), ('preview_acceleration_mps2', .36),
    ('project_speed_limit_mps', 1.51), ('collision_model', 'unimplemented_capsule'),
    ('ground_exclusion_height_m', .35), ('ground_exclusion_height_m', .08),
    ('obstacle_dilation_up_m', .10), ('safety_scan_above_base_m', .10),
    ('planning_body_below_m', .41), ('planning_body_below_m', .10),
    ('planning_body_below_m', float('nan')), ('planning_body_above_m', .46),
    ('planning_body_above_m', .05), ('planning_torso_lower_margin_m', -.01),
    ('planning_geometry_provenance', 'accepted_robot_envelope'),
    ('planning_geometry_validated', True),
])
def test_invalid_or_motion_enabled_profile_is_rejected(tmp_path, profile, key, value):
    profile['engineering'][key] = value
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):
        load_robot_profile(path)
