"""D1 Max document-backed physical facts, separate from preview assumptions."""
import math
from pathlib import Path

import yaml


def load_robot_profile(path):
    profile = yaml.safe_load(Path(path).read_text())
    if not isinstance(profile, dict) or profile.get('schema') != 1 or profile.get('model') != 'D1 Max':
        raise ValueError('invalid D1 Max robot profile')
    official, engineering = profile['official'], profile['engineering']
    if (engineering.get('mode') != 'standing_preview_only'
            or engineering.get('motion_authorized') is not False):
        raise ValueError('robot profile does not authorize a motion-enabled entry')
    size = official['standing_size_m']
    fields = [*size.values(), *[engineering[k] for k in (
        'body_reference_height_m', 'footprint_lateral_margin_m', 'footprint_longitudinal_margin_m',
        'obstacle_dilation_up_m', 'obstacle_dilation_down_m', 'ground_exclusion_height_m',
        'safety_scan_above_base_m', 'preview_speed_mps',
        'preview_acceleration_mps2', 'project_speed_limit_mps')]]
    if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= 2. for v in fields):
        raise ValueError('invalid physical or preview limit in robot profile')
    if engineering['collision_model'] != 'native_double_cylinder_approximation':
        raise ValueError('robot profile requires an implemented collision model')
    below = engineering['body_reference_height_m'] - engineering['ground_exclusion_height_m']
    if (not .08 < engineering['ground_exclusion_height_m'] <= .15
            or below <= 0.
            or abs(engineering['obstacle_dilation_up_m'] - below) > 1e-9
            or engineering['safety_scan_above_base_m'] < engineering['obstacle_dilation_down_m']):
        raise ValueError('inconsistent ground exclusion, body envelope or safety scan')
    # Extent check is necessary, not proof of full shape/swept-volume coverage.
    radius = size['width'] / 2. + engineering['footprint_lateral_margin_m']
    offset = size['length'] / 2. + engineering['footprint_longitudinal_margin_m'] - radius
    if offset <= 0.:
        raise ValueError('standing dimensions cannot use the selected two-cylinder approximation')
    if 2*radius < size['width'] or 2*(radius+offset) < size['length']:
        raise ValueError('preview collision extents smaller than documented standing dimensions')
    if (engineering['preview_speed_mps'] > min(.30, engineering['project_speed_limit_mps'])
            or engineering['project_speed_limit_mps'] > 1.5
            or engineering['preview_acceleration_mps2'] > .35):
        raise ValueError('robot profile exceeds no-motion preview limits')
    return profile


def scan_robot_parameters(profile):
    e = profile['engineering']
    size = profile['official']['standing_size_m']
    radius = size['width'] / 2. + e['footprint_lateral_margin_m']
    offset = size['length'] / 2. + e['footprint_longitudinal_margin_m'] - radius
    return {
        'grid_map.double_cylinder_radius': radius,
        'grid_map.double_cylinder_offset': offset,
        'grid_map.body_height': e['body_reference_height_m'],
        'grid_map.obstacles_inflation_z_up': e['obstacle_dilation_up_m'],
        'grid_map.obstacles_inflation_z_down': e['obstacle_dilation_down_m'],
    }


def safety_scan_parameters(profile):
    """Same below-body envelope as SCAN; never silently reuse a torso-only slice."""
    e = profile['engineering']
    return {'min_height': -e['obstacle_dilation_up_m'],
            'max_height': e['safety_scan_above_base_m']}
