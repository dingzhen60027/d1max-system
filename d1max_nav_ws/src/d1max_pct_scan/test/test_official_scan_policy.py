"""Official planning policy, exercised without ROS init, SDK or services."""
from unittest.mock import patch

import pytest
import yaml
from d1max_pct_planner.paths import expand_tree

from d1max_pct_scan import live_session
from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from test_live_scan_soft_loss import harness, body_message
from test_live_scan_reference_order import spline_harness, spline, accepted_debug


def test_default_session_selects_official_policy_without_changing_robot_or_rays():
    cfg = expand_tree(yaml.safe_load(live_session.DEFAULT_CONFIG.read_text()))
    # Compare policy alone; the default no-motion mask/budget is a separate
    # preview capability and cannot be inherited by a strict baseline.
    cfg.pop('preview_ray_exclusion', None)
    cfg['perception_timeout_s'] = .5
    official = live_session.scan_parameters(cfg, 'policy-test')
    strict = live_session.scan_parameters(dict(cfg, scan_collision_policy='observed_free'), 'policy-test')
    assert cfg['scan_collision_policy'] == 'official'
    assert official['grid_map.require_observed_free'] is False
    assert strict['grid_map.require_observed_free'] is True
    assert official['grid_map.use_projected_rays'] is True
    assert official['grid_map.require_localization_context'] is True
    assert official['optimization.lambda_reference'] == 0.
    assert official['optimization.lambda_collision'] == 1.
    assert official['optimization.lambda_feasibility'] == .1
    assert official['grid_map.resolution'] == .05
    assert [official['grid_map.sliding_map_size_'+axis] for axis in 'xyz'] == [10., 10., 5.]
    # Higher map precision, never smaller D1 collision geometry or less clearance.
    assert official['grid_map.double_cylinder_radius'] == .29
    assert official['grid_map.double_cylinder_offset'] == pytest.approx(.20)
    from d1max_pct_scan.robot_profile import load_robot_profile
    profile=load_robot_profile(live_session.paths.nav_root()/'src/d1max_scan_planner/config/d1max_robot.yaml')
    assert official['grid_map.obstacles_inflation_z_up'] == profile['engineering']['obstacle_dilation_up_m']
    assert official['grid_map.obstacles_inflation_z_down'] == profile['engineering']['obstacle_dilation_down_m']
    # The no-motion heading/revalidation lifecycle is explicitly scoped to the
    # official projected-ray preview; it must never leak into strict execution.
    assert {key for key in official if official[key] != strict[key]} == {
        'grid_map.require_observed_free', 'grid_map.preview_only',
        'manager.preview_body_heading_contract'}
    assert official['grid_map.preview_only'] is True and strict['grid_map.preview_only'] is False
    assert official['manager.preview_body_heading_contract'] is True
    assert strict['manager.preview_body_heading_contract'] is False
    assert cfg['motion_control_enabled'] is False
    assert 'preview_ray_exclusion' not in cfg


def test_official_policy_does_not_add_current_body_height_veto(monkeypatch):
    node, _, paths, _, _ = harness(monkeypatch)
    node.p['collision_policy'] = 'official'
    node.body = body_message(z=-.36)
    # Neither a failed custom support model nor a lowered pose rewrites body Z.
    with patch.object(node.ground_support, 'validate_body_samples', side_effect=AssertionError('extra checker')):
        node.update_gate()
    assert node.gate.active and node.gate.ready and not paths
    assert node.body.pose.pose.position.z == -.36
    assert node.current_body_ground_support_check['enabled'] is False
    # Source-time validation remains independent of the planning policy.
    node.cloud_stamp = 90.
    node.update_gate()
    assert node.gate.active and node.gate.preview_paused and not node.gate.ready
    assert 'cloud_source' in node.ready_failure_reasons


def test_official_native_spline_not_revetoed_by_custom_ground_checker(monkeypatch):
    node, support, emitted = spline_harness(monkeypatch)
    node.p['collision_policy'] = 'official'
    with patch.object(support, 'validate_body_samples', side_effect=AssertionError('extra checker')):
        LiveScanBridge.on_spline(node, spline(1))
    assert node.counts['rejected_splines'] == 0
    assert node.pending_spline_marker is not None
    assert node.ground_support_check['enabled'] is False
    assert not [m for m in emitted if getattr(m, 'action', None) == 0]
    LiveScanBridge.on_local_debug(node, accepted_debug(1))
    assert node.visible_spline_id == 1
    assert not node.validated and not node.admissions[-1]['valid']
    from visualization_msgs.msg import Marker
    drawn = [m for m in emitted if isinstance(m, Marker) and m.action == Marker.ADD]
    assert {m.type for m in drawn} == {Marker.LINE_STRIP, Marker.SPHERE_LIST}
    assert len(drawn) == 2 and len({m.id for m in drawn}) == 2
    for m in drawn:
        assert len(m.colors) == len(m.points) > 2
        assert all(c.r == 1. and 0. <= c.g <= 1. and c.b == 0. and c.a == 1. for c in m.colors)
        assert m.scale.x == .08 and m.header.stamp == spline(1).trajectory.start_time
    node.delete_spline_marker()
    assert emitted[-1].action == Marker.DELETEALL  # both official graphics clear together


def test_official_policy_cannot_gain_execution_authority(monkeypatch):
    node, _, _ = spline_harness(monkeypatch)
    node.p.update(collision_policy='official', execution_mode='execution')
    LiveScanBridge.on_spline(node, spline(1))
    LiveScanBridge.on_local_debug(node, accepted_debug(1))
    assert not node.validated and not node.admissions[-1]['valid']


def test_official_policy_rejected_before_motion_preparation(tmp_path):
    cfg = expand_tree(yaml.safe_load(live_session.DEFAULT_CONFIG.read_text()))
    cfg.pop('preview_ray_exclusion', None)
    cfg['perception_timeout_s'] = .5
    cfg['perception_backend'] = 'deskewed_cloud'
    target = tmp_path/'official.yaml'
    target.write_text(yaml.safe_dump(cfg))
    with patch.object(live_session, 'prepare') as prepare:
        with pytest.raises(ValueError, match='does not authorize a motion session'):
            live_session.prepare_motion(target)
        prepare.assert_not_called()


def test_unknown_policy_name_fails_explicitly(tmp_path):
    cfg = expand_tree(yaml.safe_load(live_session.DEFAULT_CONFIG.read_text()))
    cfg['scan_collision_policy'] = 'offical_typo'
    target = tmp_path/'bad.yaml'
    target.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match='Unknown SCAN collision policy'):
        live_session.load_config(target)
