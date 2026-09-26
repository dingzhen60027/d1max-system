"""Explicit restoration of a verified curve after a monotonic cost reduction.

This is not replanning or a fallback: callers must explicitly supply both the
previous artifact and its saved route. No ROS, UI, native planner or writes.
"""
from collections.abc import Mapping
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .tomogram_map import TomogramError, TomogramMap


FULL_CURVE_VALIDATION = 'quintic_cell_boundary_roots_and_interval_interiors'
RESTORED_ORIGIN = 'previous_verified_native_curve_revalidated_not_replanned'


def _require(condition, code, message, **details):
    if not condition:
        raise TomogramError(code, message, **details)


def restore_verified_route(current_map, previous_tomogram_path, original_route_json):
    """Return an unchanged verified route with explicit current-map provenance.

    ``original_route_json`` is a path or mapping. All geometry, source identity,
    support/headroom evidence and source-layer identities must match exactly;
    no cell cost may increase. Old and new endpoint/path checks must both pass.
    Existing path points and layers are never snapped, smoothed or relocated.
    """
    started = time.monotonic()
    _require(isinstance(current_map, TomogramMap) and bool(current_map.sha256),
             'restore_missing_map_hash', 'Route restoration requires a file-backed current tomogram')
    old = TomogramMap(previous_tomogram_path,
                      minimum_headroom_m=current_map.minimum_headroom_m,
                      unknown_ceiling_policy=current_map.unknown_ceiling_policy,
                      max_ground_step_m=current_map.max_ground_step_m)
    source_route = None
    if isinstance(original_route_json, Mapping):
        original = deepcopy(dict(original_route_json))
    else:
        source_route = str(Path(original_route_json).resolve())
        original = json.loads(Path(source_route).read_text(encoding='utf-8'))
    _require(isinstance(original, dict), 'restore_invalid_route', 'Saved route must be a JSON object')
    for key in ('source_pcd', 'source_sha256', 'frame_id'):
        _require(bool(old.provenance.get(key)) and old.provenance.get(key) == current_map.provenance.get(key),
                 'restore_source_mismatch', 'Route maps must have identical source and frame', field=key)
    _require(original.get('source_tomogram_sha256') == old.sha256,
             'restore_map_hash_mismatch', 'Saved route does not match the previous tomogram hash')
    _require(original.get('curve_validation') == FULL_CURVE_VALIDATION
             and original.get('quintic_segments', 0) > 0
             and original.get('extra_erosion_cells') == 0
             and original.get('cost_threshold') == current_map.COST_THRESHOLD,
             'restore_unverified_curve', 'Saved route lacks the required complete quintic validation contract')
    if original.get('frame_id') is not None:
        _require(original['frame_id'] == current_map.provenance['frame_id'],
                 'restore_source_mismatch', 'Saved route frame differs from the map')
    equal = {
        'shape': old.data.shape == current_map.data.shape,
        'resolution': old.resolution == current_map.resolution,
        'center': np.array_equal(old.center, current_map.center),
        'slice_h0': old.slice_h0 == current_map.slice_h0,
        'slice_dh': old.slice_dh == current_map.slice_dh,
        'source_layers': np.array_equal(old.source_layers, current_map.source_layers),
        'ground': np.array_equal(old.ground, current_map.ground, equal_nan=True),
        'ceiling': np.array_equal(old.ceiling, current_map.ceiling, equal_nan=True),
        'open_sky_verified': np.array_equal(old.open_sky_verified, current_map.open_sky_verified),
        'cost_threshold': old.COST_THRESHOLD == current_map.COST_THRESHOLD,
    }
    _require(all(equal.values()), 'restore_geometry_mismatch',
             'Cannot restore a curve after geometric or evidence changes',
             changed_fields=[key for key, value in equal.items() if not value])
    finite = np.isfinite(old.cost)
    _require(np.array_equal(finite, np.isfinite(current_map.cost)), 'restore_cost_mask_changed',
             'Cost-known mask must be identical for monotonic restoration')
    delta = current_map.cost[finite].astype(float) - old.cost[finite].astype(float)
    maximum_delta = float(np.max(delta)) if delta.size else 0.0
    _require(maximum_delta <= 0.0, 'restore_cost_increased',
             'Cannot restore when any grid-cell cost increases', maximum_cost_increase=maximum_delta)
    _require(not np.any(old.valid & ~current_map.valid), 'restore_stricter_validity',
             'Current map must not reject any previously valid cell')
    try:
        path = np.asarray(original['path'], dtype=float)
        layers = np.asarray(original['layer_ids'])
        first, last = np.asarray(original['start_xyz']), np.asarray(original['goal_xyz'])
        start_layer, goal_layer = original['start_layer'], original['goal_layer']
    except (KeyError, TypeError, ValueError) as exc:
        raise TomogramError('restore_invalid_route', 'Saved route is missing valid path/endpoints') from exc
    _require(path.ndim == 2 and path.shape[1] == 3 and len(path) >= 2
             and layers.shape == (len(path),) and np.isfinite(path).all(),
             'restore_invalid_route', 'Saved route must contain finite XYZ points and matching layer IDs')
    _require(np.array_equal(first, path[0]) and np.array_equal(last, path[-1])
             and start_layer == layers[0] and goal_layer == layers[-1],
             'restore_endpoint_mismatch', 'Saved exact endpoints must match the curve endpoints')
    checks = {}
    for name, tomogram in (('previous', old), ('current', current_map)):
        for point, layer in ((first, start_layer), (last, goal_layer)):
            selected = tomogram.validate_endpoint(point, layer)
            _require(np.array_equal(selected['xyz'], point), 'restore_endpoint_mismatch',
                     'Validation must not relocate saved endpoints')
        checks[name] = tomogram.validate_path(path, layers)
    route_digest = hashlib.sha256(json.dumps(original, sort_keys=True, allow_nan=False).encode()).hexdigest()
    result = deepcopy(original)
    for key in ('kind', 'generation'):
        result.pop(key, None)
    previous_elapsed = result.pop('elapsed_s', None)
    result.update(source_tomogram_sha256=current_map.sha256,
                  source_layer_ids=current_map.source_layers[layers.astype(int)].tolist(),
                  route_origin=RESTORED_ORIGIN, previous_native_elapsed_s=previous_elapsed,
                  elapsed_s=0.0, restore_elapsed_s=time.monotonic() - started,
                  frame_id=current_map.provenance['frame_id'], **checks['current'])
    result['revalidation'] = {
        'route_origin': RESTORED_ORIGIN, 'native_replanning_executed': False,
        'previous_tomogram_path': old.source, 'planning_source_tomogram_sha256': old.sha256,
        'current_tomogram_path': current_map.source, 'current_tomogram_sha256': current_map.sha256,
        'original_route_json': source_route, 'original_route_canonical_sha256': route_digest,
        'geometry_equal': {key: bool(value) for key, value in equal.items()},
        'maximum_new_minus_old_cost': maximum_delta, 'validity_not_stricter': True,
        'validation_settings': {'minimum_headroom_m': current_map.minimum_headroom_m,
                                'max_ground_step_m': current_map.max_ground_step_m,
                                'unknown_ceiling_policy': current_map.unknown_ceiling_policy},
        'path_checks': checks,
        'continuous_curve_validity_basis': 'Previous complete quintic validation is preserved by identical geometry/evidence and non-increasing costs; the exact saved endpoints and path are independently checked on both artifacts.',
    }
    return result
