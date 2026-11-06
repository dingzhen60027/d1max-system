"""D1 support height and configurable collision geometry, without authority.

SCAN searches at the body reference. Support height locates that reference over
the ground; it is not the vertical thickness of the robot. Candidate geometry is
a rigid proxy requested by the operator. Candidate and raw queries use the
same shape, but keep their different occupancy evidence policies. Neither a
box nor legacy cylinders certify the full moving wheel/leg swept volume.
"""
import math
from pathlib import Path

import yaml


def load_robot_profile(path):
    from d1max_pct_planner.paths import expand_tree
    profile = expand_tree(yaml.safe_load(Path(path).read_text()))
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
        'planning_body_below_m', 'planning_body_above_m', 'planning_torso_lower_margin_m',
        'safety_scan_above_base_m', 'preview_speed_mps',
        'preview_acceleration_mps2', 'project_speed_limit_mps')]]
    if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= 2. for v in fields):
        raise ValueError('invalid physical or preview limit in robot profile')
    shape = engineering['collision_model']
    if shape not in ('native_double_cylinder_approximation', 'native_oriented_box_approximation'):
        raise ValueError('robot profile requires an implemented collision model')
    # Do not derive body thickness from support height or a presumed ground
    # clearance. A planner proxy may be smaller than the physical safety
    # envelope only because it is NOT execution collision evidence.
    if (not .08 < engineering['ground_exclusion_height_m'] <= .15
            or engineering['planning_body_below_m'] > engineering['obstacle_dilation_up_m']
            or engineering['planning_body_above_m'] > engineering['obstacle_dilation_down_m']
            or engineering['safety_scan_above_base_m'] < engineering['obstacle_dilation_down_m']):
        raise ValueError('inconsistent planning proxy, execution envelope or safety scan')
    if (engineering.get('planning_geometry_provenance') != 'cad_torso_with_unvalidated_margin'
            or engineering.get('planning_geometry_validated') is not False):
        raise ValueError('candidate torso proxy is not a physical geometry acceptance')
    torso = profile['urdf_reference_only']['torso_bounds_base_m']
    lower, upper = torso['min'], torso['max']
    if (not isinstance(lower, list) or not isinstance(upper, list)
            or len(lower) != 3 or len(upper) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v)
                   for v in lower + upper)
            or any(a >= b for a, b in zip(lower, upper))
            or not lower[2] < 0. < upper[2]
            or abs(engineering['planning_body_below_m'] -
                   (-lower[2] + engineering['planning_torso_lower_margin_m'])) > 1e-9
            or engineering['planning_body_above_m'] < upper[2]):
        raise ValueError('candidate proxy does not contain its declared CAD torso extent')
    if shape == 'native_oriented_box_approximation':
        box_fields = [engineering[k] for k in ('box_length_m', 'box_width_m',
            'box_height_m', 'box_body_below_m', 'box_body_above_m')]
        if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= 2.
               for v in box_fields):
            raise ValueError('invalid oriented box geometry')
        length, width, height, below, above = box_fields
        if (abs(height-below-above) > 1e-9
                or length/2. < max(abs(lower[0]), abs(upper[0]))
                or width/2. < max(abs(lower[1]), abs(upper[1]))
                or below < -lower[2] or above < upper[2]
                or engineering.get('box_geometry_provenance') !=
                    'operator_rigid_proxy_centered_base_link_not_measured'):
            raise ValueError('oriented box does not contain CAD torso or declared height')
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


def scan_robot_parameters(profile, *, strict_layer_available=True, planar_clearance_m=0.,
                          stair_planar_clearance_m=None):
    """Candidate proxy only when an independent strict layer is available.

    The formal SCAN-only collision chain passes False: both planning and
    running-trajectory checks share the full body kernel. True is retained for
    explicit historical strict-layer fixtures, not a second default policy.

    Planar clearance pads that shared kernel, never the nominal robot profile,
    raw sensor self-filter or vertical/support geometry. It is per side, not a
    change to measured body dimensions or the optimizer's soft rebound cost.
    """
    if type(strict_layer_available) is not bool:
        raise ValueError('strict collision layer capability must be explicit')
    if (type(planar_clearance_m) not in (int, float)
            or not math.isfinite(planar_clearance_m) or planar_clearance_m < 0.):
        raise ValueError('planar clearance must be finite and nonnegative')
    clearance = float(planar_clearance_m)
    if stair_planar_clearance_m is not None and (
            type(stair_planar_clearance_m) not in (int, float)
            or not math.isfinite(stair_planar_clearance_m)
            or not 0. <= stair_planar_clearance_m <= clearance):
        raise ValueError('stair planar clearance must be finite and within the normal clearance')
    e = profile['engineering']
    size = profile['official']['standing_size_m']
    radius = size['width'] / 2. + e['footprint_lateral_margin_m']
    offset = size['length'] / 2. + e['footprint_longitudinal_margin_m'] - radius
    result = {
        # Keep both cylinder centres fixed: recomputing their offset from the
        # padded radius would silently remove the longitudinal clearance.
        'grid_map.double_cylinder_radius': radius + clearance,
        'grid_map.double_cylinder_offset': offset,
        'grid_map.body_height': e['body_reference_height_m'],
        'grid_map.obstacles_inflation_z_up': e['obstacle_dilation_up_m'],
        'grid_map.obstacles_inflation_z_down': e['obstacle_dilation_down_m'],
        # Only the official candidate planning kernel consumes these values.
        # Existing keys above retain the raw-proof / motion-sweep envelope.
        'grid_map.planning_obstacles_inflation_z_up': (
            e['planning_body_below_m'] if strict_layer_available else e['obstacle_dilation_up_m']),
        'grid_map.planning_obstacles_inflation_z_down': (
            e['planning_body_above_m'] if strict_layer_available else e['obstacle_dilation_down_m']),
    }
    if e['collision_model'] == 'native_oriented_box_approximation':
        # Both native paths consume one shape; do not leave old proof/sweep
        # cylinders active while only shrinking the displayed planner proxy.
        result.update({
            'grid_map.collision_model': 'oriented_box',
            'grid_map.box_length': e['box_length_m'] + 2. * clearance,
            'grid_map.box_width': e['box_width_m'] + 2. * clearance,
            'grid_map.body_below': e['box_body_below_m'],
            'grid_map.body_above': e['box_body_above_m'],
            'grid_map.planning_body_below': e['box_body_below_m'],
            'grid_map.planning_body_above': e['box_body_above_m'],
            # Retained declared aliases must agree too; diagnostics should
            # never advertise the old cylinder vertical envelope as active.
            'grid_map.obstacles_inflation_z_up': e['box_body_below_m'],
            'grid_map.obstacles_inflation_z_down': e['box_body_above_m'],
            'grid_map.planning_obstacles_inflation_z_up': e['box_body_below_m'],
            'grid_map.planning_obstacles_inflation_z_down': e['box_body_above_m'],
        })
        if stair_planar_clearance_m is not None:
            # Native readers apply this smaller EXTRA margin only at points
            # associated with a stair/landing support region. Flat points in a
            # mixed window still use box_length/width above. Nominal geometry,
            # sensor self-filter, body Z and SDK profile remain untouched.
            result.update({
                'grid_map.box_nominal_length': e['box_length_m'],
                'grid_map.box_nominal_width': e['box_width_m'],
                'grid_map.stair_planar_clearance': float(stair_planar_clearance_m),
            })
    else:
        if stair_planar_clearance_m is not None:
            raise ValueError('stair planar clearance requires the oriented box model')
        result['grid_map.collision_model'] = 'double_cylinder'
    # No arbitrary clearance cap; reject only unrepresentable geometry.
    if any(not math.isfinite(value) for key, value in result.items()
           if key in ('grid_map.box_length', 'grid_map.box_width', 'grid_map.double_cylinder_radius')):
        raise ValueError('planar clearance produces nonfinite geometry')
    return result


def safety_scan_parameters(profile):
    """Flat-floor auxiliary scan; never replace Native 3D collision checks.

    A torso box's lower face is not a ground exclusion height. Keep low boxes
    visible to the legacy 2D monitor even if the torso approximation is smaller.
    """
    e = profile['engineering']
    below = e['box_body_below_m'] if e['collision_model'] == 'native_oriented_box_approximation' else e['obstacle_dilation_up_m']
    return {'min_height': min(-below,e['ground_exclusion_height_m']-e['body_reference_height_m']),
            'max_height': e['safety_scan_above_base_m']}
