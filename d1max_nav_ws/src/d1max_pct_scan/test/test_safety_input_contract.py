"""Effective production wiring, not a replacement safety map or ROS service."""
from pathlib import Path

import pytest

from d1max_pct_scan.motion_stack import describe_parameters
from d1max_pct_scan.robot_profile import load_robot_profile
from d1max_pct_scan.safety_input_contract import (
    FILTERED_SOURCE, PREFIX, inspect_safety_input, safety_input_blockers)


WORKSPACE = Path(__file__).resolve().parents[3]


def effective_nodes():
    profile = load_robot_profile(WORKSPACE/'src/d1max_scan_planner/config/d1max_robot.yaml')
    session = dict(id='test', frame_id='d1max_loc_map', version_id='map-test',
                   robot_profile_snapshot=profile,
                   motion=dict(body_height=profile['engineering']['body_reference_height_m'], max_speed=.3, max_yaw=.5,
                               single_floor_height_span=.25))
    return describe_parameters(session, WORKSPACE)


def test_current_production_wiring_has_explicit_unresolved_safety_provenance():
    result = inspect_safety_input(effective_nodes())
    assert result['input_topic'] == FILTERED_SOURCE
    assert result['source_kind'] == 'lio_filtered_endpoints'
    assert result['blockers'] == list(safety_input_blockers())
    assert result['motion_architecture_ready'] is result['coverage_verified'] is False
    assert not any(result['acquisition_contract'].values())
    assert result['no_return_semantics'] == 'unknown_nan_not_clear'


def test_yaml_calibration_booleans_do_not_implement_missing_scan_contract():
    spoof = dict(safety_scan_verified=True, raw_safety_scan=True,
                 motion_control_enabled=True, time_alignment_verified=True)
    assert safety_input_blockers(spoof) == safety_input_blockers()


def test_changing_topic_to_raw_without_a_new_producer_never_lifts_blocker():
    nodes = effective_nodes()
    nodes[PREFIX+'navigation_cloud_to_scan']['ros__parameters']['input_topic'] = '/front_lidar'
    result = inspect_safety_input(nodes)
    assert result['source_kind'] == 'unverified_endpoints'
    assert 'unverified_safety_endpoint_source' in result['blockers']
    assert set(safety_input_blockers()) <= set(result['blockers'])


@pytest.mark.parametrize('name,field,value,issue', [
    ('navigation_cloud_to_scan', 'target_frame', 'map', 'projection_frame'),
    ('navigation_cloud_to_scan', 'output_topic', '/wrong', 'projection_output'),
    ('navigation_command_gate', 'scan_topic', '/wrong', 'gate_scan'),
    ('navigation_command_gate', 'base_frame', 'lidar', 'gate_body'),
    ('navigation_command_gate', 'odom_frame', 'map', 'gate_odom'),
    ('collision_monitor', 'base_frame_id', 'lidar', 'monitor_body'),
    ('collision_monitor', 'odom_frame_id', 'map', 'monitor_odom'),
    ('collision_monitor', 'scan', {'topic': '/wrong'}, 'monitor_scan'),
])
def test_effective_safety_consumer_drift_is_visible(name, field, value, issue):
    nodes = effective_nodes()
    nodes[PREFIX+name]['ros__parameters'][field] = value
    result = inspect_safety_input(nodes)
    assert 'safety_input_wiring_mismatch:'+issue in result['blockers']


@pytest.mark.parametrize('timeout', [.51, float('nan'), None, True, 0.])
def test_age_budget_is_not_silently_extended(timeout):
    nodes = effective_nodes()
    nodes[PREFIX+'navigation_cloud_to_scan']['ros__parameters']['max_cloud_age'] = timeout
    assert 'invalid_safety_cloud_source_age_bound' in inspect_safety_input(nodes)['blockers']


@pytest.mark.parametrize('nodes', [{}, None])
def test_missing_effective_nodes_are_not_substituted_with_good_defaults(nodes):
    result = inspect_safety_input(nodes)
    assert sum(issue.startswith('missing_effective_node:') for issue in result['blockers']) == 3
    assert result['motion_architecture_ready'] is False
