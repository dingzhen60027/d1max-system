"""Pure admission rules for a fresh, shadow-only live PCT global request."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


class GlobalPlanError(ValueError):
    pass


FLOOR_NAMES = {'floor1': 'lower', 'floor2': 'upper'}
LEG_FLOORS = {'lower_floor': 'floor1', 'upper_floor': 'floor2',
              'stair_lower': 'stair_lower', 'stair_upper': 'stair_upper'}


def finite_xyz(value, name='xyz'):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise GlobalPlanError(f'{name}: expected finite XYZ')
    return point


def supported_start_floor(bridge, body_xyz, *, height_interval_m):
    """Resolve only a unique measured floor-height hypothesis, never nearest XY.

    The fixed startup floor is a seed/UI hint, not a permanent assumption after
    crossing floors. Stair-transition, absent and ambiguous support all reject.
    The body pose is queried unchanged, never warped into the planning frame.
    """
    body = finite_xyz(body_xyz, 'body')
    candidates = []
    for floor in FLOOR_NAMES:
        if floor not in bridge.floors:
            continue
        try:
            projected = bridge.project_live_pose_to_ground(
                body, floor, body_height_interval_m=height_interval_m)
        except ValueError:
            continue
        candidates.append((floor, projected))
    if len(candidates) != 1:
        raise GlobalPlanError('current_floor_requires_unique_measured_ground_support')
    return candidates[0]


def fresh_stamp(stamp, now, maximum_age_s=.5, future_tolerance_s=.1):
    return (type(stamp) in (int, float) and math.isfinite(stamp) and stamp > 0
            and math.isfinite(now) and -future_tolerance_s <= now-stamp <= maximum_age_s)


def new_goal_stamp(stamp, *, now, startup_stamp, previous_stamp, maximum_age_s=2.):
    if (not fresh_stamp(stamp, now, maximum_age_s) or stamp <= startup_stamp
            or stamp <= previous_stamp):
        raise GlobalPlanError('goal_stamp_not_new_fresh_or_after_startup')
    return stamp


def healthy_context(navigation, scan, *, session_id, now, navigation_age_s=.5,
                    scan_age_s=.5, cloud_age_s=.5):
    """Return (epoch, seed), or reject; readiness never resurrects an old goal."""
    if not isinstance(navigation, dict) or not isinstance(scan, dict):
        raise GlobalPlanError('navigation_or_scan_status_missing')
    epoch, seed = navigation.get('epoch'), navigation.get('seed_id')
    if (navigation.get('valid') is not True
            or navigation.get('fault') or type(epoch) is not int or epoch < 1 or not seed
            or not fresh_stamp(navigation.get('received_at_unix'), now, navigation_age_s)):
        raise GlobalPlanError('localization_output_invalid_or_stale')
    if (scan.get('session_id') != session_id
            or scan.get('localization_session_id') != session_id
            or scan.get('ready') is not True or scan.get('sensor_ready') is not True
            or scan.get('motion_enabled') is not False
            or scan.get('localization_epoch') != epoch
            or scan.get('localization_seed_id') != seed
            or not fresh_stamp(scan.get('received_at_unix'), now, scan_age_s)):
        raise GlobalPlanError('scan_bridge_not_ready_same_session_and_seed')
    cloud_age = scan.get('cloud_age')
    if (type(cloud_age) not in (int, float) or not math.isfinite(cloud_age)
            or not -.1 <= cloud_age <= cloud_age_s):
        raise GlobalPlanError('scan_cloud_not_fresh')
    return epoch, seed


def healthy_global_context(navigation, *, now, navigation_age_s=.5):
    """Global PCT only needs a fresh localization output, never SCAN/cloud.

    Local collision/sensor admission belongs to the downstream SCAN bridge.
    This function deliberately does not inspect its ready/active_reference bits.
    """
    if not isinstance(navigation, dict):
        raise GlobalPlanError('navigation_status_missing')
    epoch, seed = navigation.get('epoch'), navigation.get('seed_id')
    if (navigation.get('valid') is not True or navigation.get('fault')
            or type(epoch) is not int or epoch < 1 or not isinstance(seed, str)
            or not seed or not fresh_stamp(navigation.get('received_at_unix'),
                                           now, navigation_age_s)):
        raise GlobalPlanError('localization_output_invalid_or_stale')
    return epoch, seed


def choose_floor_surface(tomogram, xy, floor_id, floor_z_ranges,
                         *, hint_z=None, maximum_hint_error_m=.15):
    """Select a measured PCT surface at *the exact XY* and explicit floor.

    Returns a TomogramMap surface record, including the exact XY and layer ID.
    Multiple distinct eligible heights mean ambiguity, never nearest-XY repair.
    """
    if floor_id not in FLOOR_NAMES:
        raise GlobalPlanError('floor_id_must_be_explicit_floor1_or_floor2')
    xy = np.asarray(xy, dtype=float)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        raise GlobalPlanError('goal_xy_invalid')
    low, high = floor_z_ranges[FLOOR_NAMES[floor_id]]
    if not math.isfinite(low) or not math.isfinite(high) or low >= high:
        raise GlobalPlanError('floor_z_range_invalid')
    candidates = [surface for surface in tomogram.sample_surfaces(xy, deduplicate=True)
                  if low <= surface['ground_z'] <= high]
    if hint_z is not None:
        if not math.isfinite(hint_z):
            raise GlobalPlanError('goal_height_invalid')
        candidates = [surface for surface in candidates
                      if abs(surface['ground_z']-hint_z) <= maximum_hint_error_m]
    if not candidates:
        raise GlobalPlanError('no_traversable_measured_surface_at_exact_xy_and_floor')
    heights = {round(item['ground_z'], 4) for item in candidates}
    if len(heights) != 1:
        raise GlobalPlanError('multiple_distinct_surfaces_on_declared_floor')
    return candidates[0]


def floor_from_planning_z(z, floor_z_ranges):
    if not math.isfinite(z):
        raise GlobalPlanError('goal_height_invalid')
    matches = [floor for floor, name in FLOOR_NAMES.items()
               if floor_z_ranges[name][0] <= z <= floor_z_ranges[name][1]]
    if len(matches) != 1:
        raise GlobalPlanError('3d_goal_height_must_select_exactly_one_floor')
    return matches[0]


def floor_from_original_ground_z(bridge, xyz, *, tolerance_m=.15):
    """Resolve a source-frame *ground point* by Z, not by overlapping XY."""
    point = finite_xyz(xyz, 'goal3d')
    if any(np.all(point >= region['min']) and np.all(point <= region['max'])
           for region in bridge.protected_regions):
        raise GlobalPlanError('goal_in_stair_transition_requires_floor_target')
    matches = []
    for floor_id in FLOOR_NAMES:
        try:
            floor = bridge.floors[floor_id]
            field, _ = floor.query(point[:2].reshape(1, 2), bridge.limits)
            if abs(point[2]-field[0]) <= tolerance_m:
                matches.append(floor_id)
        except ValueError:
            pass
    if len(matches) != 1:
        raise GlobalPlanError('source_3d_goal_z_does_not_uniquely_match_observed_floor')
    return matches[0]


def labels_for_result(result, *, same_floor):
    points = np.asarray(result.get('path', result.get('path_xyz')), dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise GlobalPlanError('native_result_path_invalid')
    if same_floor:
        if same_floor not in FLOOR_NAMES:
            raise GlobalPlanError('same_floor_label_invalid')
        return points, [same_floor] * len(points)
    edge_legs = result.get('edge_legs')
    if not isinstance(edge_legs, (list, tuple)) or len(edge_legs) != len(points)-1:
        raise GlobalPlanError('crossfloor_result_edges_missing')
    try:
        labels = [LEG_FLOORS[edge_legs[min(i, len(edge_legs)-1)]] for i in range(len(points))]
    except KeyError as exc:
        raise GlobalPlanError(f'crossfloor_result_leg_unknown:{exc}') from exc
    return points, labels


@dataclass(frozen=True)
class RequestEvidence:
    generation: int
    epoch: int
    seed_id: str
    goal_stamp: float
    start_body_xyz: tuple[float, float, float]
    issued_monotonic: float


def result_admissible(evidence: RequestEvidence, *, generation, context, body_xyz,
                      now_monotonic, result_timeout_s, max_start_move_m=.3):
    body = finite_xyz(body_xyz, 'current_body')
    start = finite_xyz(evidence.start_body_xyz, 'request_start_body')
    if (generation != evidence.generation or context != (evidence.epoch, evidence.seed_id)
            or not math.isfinite(now_monotonic) or now_monotonic < evidence.issued_monotonic
            or now_monotonic-evidence.issued_monotonic > result_timeout_s):
        raise GlobalPlanError('obsolete_context_or_timed_out_result')
    moved = float(np.linalg.norm(body-start))
    if moved > max_start_move_m:
        raise GlobalPlanError(f'robot_moved_{moved:.3f}m_from_planning_start')
    return moved
