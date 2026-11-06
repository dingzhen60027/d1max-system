"""File-only contract: session control parameters reach the formal follower.

No ROS node or SDK session is created by these checks.
"""
import ast
from pathlib import Path
import re

import pytest


ROOT = Path(__file__).resolve().parents[3]
NODE = ROOT / 'src/d1max_trajectory_tracker/src/local_follower_node.cpp'
SESSION = ROOT / 'src/d1max_pct_scan/d1max_pct_scan/single_floor_session.py'


def tracker_projection_keys():
    tree = ast.parse(SESSION.read_text())
    prepare = next(node for node in tree.body
                   if isinstance(node, ast.FunctionDef) and node.name == 'prepare')
    keys = set()
    for node in ast.walk(prepare):
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == 'tracker' for target in node.targets):
            assert isinstance(node.value, ast.Call)
            keys.update(keyword.arg for keyword in node.value.keywords if keyword.arg)
    # Stationary evidence is added only when the measured/runtime record
    # supplies it, but those same keys must still be wired at the node.
    for node in ast.walk(prepare):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name) and node.func.value.id == 'tracker'
                and node.func.attr == 'update'):
            keys.update(keyword.arg for keyword in node.keywords if keyword.arg)
    return keys


@pytest.mark.parametrize('key', [
    'task_timeout', 'odom_timeout', 'trajectory_timeout',
    'goal_tolerance', 'goal_height_tolerance', 'heading_threshold',
    'steering_enter_angle_rad','steering_exit_angle_rad','steering_error_scale_rad',
    'velocity_damping_gain', 'yaw_damping_gain',
    'max_jerk', 'max_yaw_jerk',
    'stationary_linear_threshold_mps', 'stationary_angular_threshold_radps',
    'stationary_reentry_duration_s',
])
def test_existing_session_parameter_is_consumed_before_validation(key):
    assert key in tracker_projection_keys()
    source = NODE.read_text()
    assignment = re.search(
        rf'c\.{key}\s*=\s*declare_parameter<double>\(\s*"{key}"\s*,\s*c\.{key}\s*\)',
        source)
    assert assignment, f'formal local_follower ignores projected {key}'
    assert assignment.end() < source.index('c.validate()')


def test_projected_stationary_sample_count_is_bounded_before_unsigned_conversion():
    assert 'stationary_minimum_samples' in tracker_projection_keys()
    source = NODE.read_text()
    declaration = source.index('declare_parameter<int>("stationary_minimum_samples",3)')
    guard = source.index('if(stationary_samples<3||stationary_samples>512)')
    conversion = source.index('c.stationary_minimum_samples=static_cast<unsigned>(stationary_samples)')
    assert declaration < guard < conversion < source.index('c.validate()')


def test_formal_follower_requires_original_native_heading_seed_before_validation():
    assert 'require_body_heading_seed' in tracker_projection_keys()
    source = NODE.read_text()
    declaration = source.index('c.require_body_heading_seed=declare_parameter<bool>("require_body_heading_seed",true)')
    assert declaration < source.index('c.validate()')


@pytest.mark.parametrize('key', ['velocity_damping_gain', 'yaw_damping_gain','max_jerk','max_yaw_jerk',
                               'steering_enter_angle_rad','steering_exit_angle_rad','steering_error_scale_rad'])
@pytest.mark.parametrize('filename', ['local_follower_node.cpp', 'trajectory_tracker.cpp'])
def test_both_controller_nodes_consume_measured_damping(key, filename):
    source = (NODE.parent / filename).read_text()
    assignment = re.search(
        rf'c\.{key}\s*=\s*declare_parameter<double>\(\s*"{key}"\s*,\s*c\.{key}\s*\)',
        source)
    assert assignment, f'{filename} ignores measured damping {key}'
    boundary = ('c.validate()' if filename == 'local_follower_node.cpp'
                else 'std::make_unique<ExecutionContract>(c,')
    # The legacy contract constructs TrackerCore, which validates Config.
    assert assignment.end() < source.index(boundary)
