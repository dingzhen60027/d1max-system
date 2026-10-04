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
                 'vertical_envelope_validated', 'sdk_speed_mapping_validated'):
        assert engineering[flag] is False
    assert official['sdk_speed_document_conflict'] is True
    assert Path(profile['sources']['specification']['path']).is_absolute()
    assert engineering['project_speed_limit_mps'] == 1.5
    assert engineering['preview_speed_mps'] == .3
    assert official['product_max_speed_mps'] > engineering['project_speed_limit_mps']


def test_native_footprint_is_derived_from_dimensions_and_explicit_margins(profile):
    value = scan_robot_parameters(profile)
    assert value['grid_map.double_cylinder_radius'] == pytest.approx(.29)
    assert value['grid_map.double_cylinder_offset'] == pytest.approx(.20)
    assert value['grid_map.body_height'] == .50
    assert value['grid_map.obstacles_inflation_z_up'] == .40
    assert value['grid_map.obstacles_inflation_z_down'] == .45
    assert not any('extrinsic' in key or 'sdk' in key for key in value)
    larger = deepcopy(profile)
    larger['official']['standing_size_m']['width'] = .50
    changed = scan_robot_parameters(larger)
    assert changed['grid_map.double_cylinder_radius'] == pytest.approx(.30)
    assert changed['grid_map.double_cylinder_offset'] == pytest.approx(.19)
    assert changed['grid_map.body_height'] == value['grid_map.body_height']


def test_low_obstacle_envelope_is_shared_with_final_safety_scan(profile):
    scan = scan_robot_parameters(profile)
    safety = safety_scan_parameters(profile)
    e = profile['engineering']
    assert e['ground_exclusion_height_m'] == .10
    assert e['body_reference_height_m'] + safety['min_height'] == pytest.approx(.10)
    assert safety['min_height'] == -scan['grid_map.obstacles_inflation_z_up']
    assert safety['max_height'] >= scan['grid_map.obstacles_inflation_z_down']
    assert e['vertical_envelope_validated'] is False


@pytest.mark.parametrize('key,value', [
    ('motion_authorized', True), ('mode', 'motion_control'),
    ('body_reference_height_m', float('nan')), ('footprint_lateral_margin_m', -.05),
    ('footprint_longitudinal_margin_m', True), ('footprint_lateral_margin_m', 1.),
    ('preview_speed_mps', .31), ('preview_acceleration_mps2', .36),
    ('project_speed_limit_mps', 1.51), ('collision_model', 'unimplemented_capsule'),
    ('ground_exclusion_height_m', .35), ('ground_exclusion_height_m', .08),
    ('obstacle_dilation_up_m', .15), ('safety_scan_above_base_m', .10),
])
def test_invalid_or_motion_enabled_profile_is_rejected(tmp_path, profile, key, value):
    profile['engineering'][key] = value
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):
        load_robot_profile(path)
