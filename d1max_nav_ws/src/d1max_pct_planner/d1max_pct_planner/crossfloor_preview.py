"""Offline RViz adapter for explicit building levels and their stair portals."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
import threading

import numpy as np
import yaml

from .crossfloor_route import (validate_config, plan_crossfloor, _masked_tomogram,
                              _native_route, _check_stair_profile, _check_leg,
                              checked_landing_adjustment, STAIR_REFINEMENT_PROOF, LEGS,
                              _stationary_floor_leg)
from .path_quality import path_quality
from .tomogram_map import TomogramMap
from .crossfloor_partial import endpoint_region, plan_partial_stair


def _signature(value):
    return json.dumps(value,sort_keys=True,allow_nan=False,
        default=lambda value:value.tolist() if isinstance(value,np.ndarray) else value.item())


def _map_settings_signature(settings):
    keys = ('planning','limits','vendor_root','floor_z_ranges','stair_roi','unknown_ceiling_policy')
    return _signature({**{key:settings[key] for key in keys},
        **({'compiled_stair_connector':settings['compiled_stair_connector']}
           if 'compiled_stair_connector' in settings else {})})


def _immutable_map_snapshot(original):
    """A planner instance owns one fixed map; caller arrays cannot change it."""
    result = TomogramMap(dict(data=original.data, resolution=original.resolution,
        center=original.center.copy(), slice_h0=original.slice_h0,
        slice_dh=original.slice_dh, selected_source_layers=original.source_layers.copy(),
        minimum_headroom_m=original.minimum_headroom_m,
        open_sky_verified=original.open_sky_verified.copy()),
        unknown_ceiling_policy=original.unknown_ceiling_policy,
        max_ground_step_m=original.max_ground_step_m)
    result.source, result.sha256 = original.source, original.sha256
    result.provenance = deepcopy(original.provenance)
    for value in vars(result).values():
        if isinstance(value, np.ndarray):
            value.flags.writeable = False
    return result


class _FixedStairCache:
    """Two validated curves owned by one immutable map/configuration snapshot.

    This is not a general route memoizer. Only the configured entry/landing/
    exit pairs qualify, with exact XYZ and layer identity. Copies isolate the
    retained result from the coordinator's reversal and external consumers.
    """
    def __init__(self, tomogram, settings):
        self.tomogram = tomogram
        self.signature = _map_settings_signature(settings)
        self.anchors = _signature({key: settings['anchors'][key]
                                   for key in ('entry', 'landing', 'exit')})
        self._results = {}
        self.stats = dict(hits=0, misses=0, max_entries=2)
        # Separate compiled-polyline proof path. Native put() below remains
        # quintic-only; no old polynomial/map certificate is relabelled.
        from .compiled_stair_connector import load_compiled_stair_connector
        self._compiled = load_compiled_stair_connector(tomogram, settings)
        self._compiled_results = {}
        if self._compiled is not None:
            for name, first, last in LEGS:
                if name not in self._compiled['legs']:
                    continue
                key = self._key(tomogram, name, settings['anchors'][first],
                                settings['anchors'][last], settings)
                self._compiled_results[key] = deepcopy(self._compiled['legs'][name])
            self._results = deepcopy(self._compiled_results)

    def compiled_bundle(self):
        return deepcopy(self._compiled)

    def _key(self, tomogram, name, begin, end, settings):
        if name not in ('stair_lower', 'stair_upper'):
            return None
        if (tomogram is not self.tomogram
                or _map_settings_signature(settings) != self.signature
                or _signature({key: settings['anchors'][key]
                               for key in ('entry', 'landing', 'exit')}) != self.anchors):
            raise ValueError('Fixed stair cache requires its original map/configuration snapshot')
        _, first, last = next(leg for leg in LEGS if leg[0] == name)
        if (_signature(begin) != _signature(settings['anchors'][first])
                or _signature(end) != _signature(settings['anchors'][last])):
            raise ValueError('Fixed stair cache requires exact configured portal endpoints')
        return (name, _signature(begin), _signature(end))

    def get(self, tomogram, name, begin, end, settings):
        key = self._key(tomogram, name, begin, end, settings)
        if key is None:
            return None
        raw = self._results.get(key)
        if raw is None and key in self._compiled_results:
            # An unrelated failed goal clears transient native results, not
            # this already-verified immutable map artifact. No disk reread or
            # fallback to a different curve occurs on the next fixed query.
            raw = deepcopy(self._compiled_results[key])
            self._results[key] = raw
        self.stats['hits' if raw is not None else 'misses'] += 1
        return deepcopy(raw) if raw is not None else None

    def put(self, tomogram, name, begin, end, settings, raw):
        key = self._key(tomogram, name, begin, end, settings)
        if key is None:
            return
        # Full polynomial validation is required in addition to the sampled
        # geometry check performed by plan_crossfloor before this call.
        if (raw.get('curve_validation') != 'quintic_cell_boundary_roots_and_interval_interiors'
                or raw.get('quintic_segments', 0) <= 0
                or raw.get('curve_partition_points', 0) < 2):
            return
        if key not in self._results and len(self._results) >= self.stats['max_entries']:
            raise RuntimeError('Fixed stair cache capacity exceeded')
        self._results[key] = deepcopy(raw)

    def clear(self):
        self._results.clear()


def _unique_array_bytes(maps):
    """Count owned NumPy storage once, excluding native C++ allocations."""
    buffers = {}
    for grid in maps:
        for value in vars(grid).values():
            if isinstance(value, np.ndarray):
                while isinstance(value.base, np.ndarray):
                    value = value.base
                buffers[id(value)] = value.nbytes
    return sum(buffers.values())


def load_config(path, tomogram):
    from .paths import expand_tree
    raw = expand_tree(yaml.safe_load(Path(path).read_text()))
    settings = validate_config(raw)
    if Path(settings['tomogram_path']).resolve() != Path(tomogram.source).resolve():
        raise ValueError('Cross-floor coordinator and editor must use the same tomogram')
    # A matching file hash does not establish matching runtime safety policy:
    # TomogramMap computes its admissible mask using constructor overrides.
    # Never reuse that mask under different coordinator settings, or mix the
    # map's default step limit with the route's explicit one between checks.
    if tomogram.unknown_ceiling_policy != settings['unknown_ceiling_policy']:
        raise ValueError('Cross-floor unknown_ceiling_policy mismatch: '
                         f'tomogram={tomogram.unknown_ceiling_policy}, '
                         f'route={settings["unknown_ceiling_policy"]}; '
                         'reload the tomogram with the matching route policy')
    if tomogram.max_ground_step_m != settings['limits']['max_ground_step_m']:
        raise ValueError('Cross-floor max_ground_step_m mismatch: '
                         f'tomogram={tomogram.max_ground_step_m}, '
                         f'route={settings["limits"]["max_ground_step_m"]}; '
                         'reload the tomogram with the matching route limit')
    # Route schema v1 has no separate headroom setting. Its single authority
    # remains the supplied TomogramMap; snapshots and masks preserve that exact
    # value, including an explicit caller override. Do not invent a 0.55 default
    # here or silently overwrite the already computed clearance mask.
    tomogram.verify_source(settings['source_pcd'], settings['frame_id'])
    return raw, settings


def visible_surfaces(points, settings):
    """Only display declared floors, plus intermediate surfaces in the stair.

    Does not change costs, validity or planning data. Blocked surfaces within
    these physical domains remain red.
    """
    points = np.asarray(points)
    keep = np.zeros(len(points), bool)
    for low, high in settings['floor_z_ranges'].values():
        keep |= (points[:, 2] >= low) & (points[:, 2] <= high)
    low, high = settings['stair_roi']
    keep |= np.all((points >= low) & (points <= high), axis=1)
    return keep


def _as_preview(result, tomogram, elapsed=0.):
    points = np.asarray(result['path_xyz'])
    layers = np.asarray(result['layer_ids'], dtype=int)
    checks = tomogram.validate_path(points, layers)
    checks['layer_transition_count'] = checks.pop('layer_transitions', 0)
    # The server excludes `path` from its 5 Hz status message. Do not retain a
    # second complete XYZ copy under the audit export's `path_xyz` key.
    return {**{key: value for key, value in result.items() if key != 'path_xyz'},
            **checks, 'path': points.tolist(),
            'start_xyz': points[0].tolist(), 'goal_xyz': points[-1].tolist(),
            'start_layer': int(layers[0]), 'goal_layer': int(layers[-1]),
            'source_tomogram_sha256': tomogram.sha256,
            'elapsed_s': elapsed, 'algorithm': 'PCT floor/stair coordinator + checked native routes',
            'segment_algorithms': [segment.get('native_algorithm') for segment in result.get('segments', [])],
            'height_semantics': 'planning_ground_support_not_body_or_timed_trajectory',
            'path_quality': path_quality(points), 'execution_authorized': False}


class CrossfloorPreviewRoute:
    def __init__(self, tomogram, config_path):
        self.raw, self.settings = load_config(config_path, tomogram)
        self.tomogram = _immutable_map_snapshot(tomogram)
        self._planning_lock = threading.Lock()
        self._native_maps = {}
        self._fixed_signature = _map_settings_signature(self.settings)
        self._config_signature = (_signature(self.raw), _signature(self.settings))
        self.cache_stats = dict(hits=0, misses=0, invalidations=0, max_entries=3)
        self._fixed_stairs = _FixedStairCache(self.tomogram, self.settings)
        # Verify ABI and parameters once; do not allocate a full native map for
        # this metadata-only probe. Individual masked legs own their native map.
        probe = _native_route(tomogram, 'lower_floor', self.settings, defer_map=True)
        self.native_runtime, self.native_parameters = probe.native_runtime, probe.native_parameters

    def _resources(self, name, settings):
        if name not in {leg[0] for leg in LEGS}:
            raise ValueError('Unknown fixed map resource')
        if _map_settings_signature(settings) != self._fixed_signature:
            raise ValueError('Map configuration changed; create a new planner snapshot')
        # Both stair legs use the same ROI mask and path_refinement='none'.
        # Native mutable query state is safe to share because plan() owns the
        # lock and runs legs sequentially. Failed queries invalidate everything.
        key = 'stairs' if name.startswith('stair_') else name
        if key not in self._native_maps:
            if len(self._native_maps) >= self.cache_stats['max_entries']:
                raise RuntimeError('Native map cache capacity exceeded')
            masked = _masked_tomogram(self.tomogram, name, settings)
            route = _native_route(masked, name, settings)
            for value in vars(masked).values():
                if isinstance(value,np.ndarray):
                    value.flags.writeable = False
            self._native_maps[key] = (masked,route)
            self.cache_stats['misses'] += 1
        else:
            self.cache_stats['hits'] += 1
        return self._native_maps[key]

    def _resource_stats(self):
        maps = [self.tomogram] + [item[0] for item in self._native_maps.values()]
        nodes = int(self.tomogram.layers * self.tomogram.nx * self.tomogram.ny)
        native_search = {}
        for key, (_, route) in self._native_maps.items():
            wrapper = getattr(route, 'planner', None)
            native = getattr(wrapper, 'planner', None)
            if native is not None:
                native_search[key] = dict(native.get_path_finder().get_memory_stats())
        return {**self.cache_stats, 'entries': len(self._native_maps),
                'resource_keys': sorted(self._native_maps),
                'shared_stair_resource': True,
                'grid_nodes_per_native_map': nodes,
                'native_grid_nodes_total': nodes * len(self._native_maps),
                'grid_nodes_semantics': 'mathematical_grid_cells_not_allocated_search_nodes',
                'native_search': native_search,
                'tomogram_numpy_bytes': _unique_array_bytes(maps),
                'memory_scope': 'unique_tomogram_numpy_buffers_excludes_native_allocations'}

    def _floor(self, point):
        return endpoint_region(point, self.settings)

    def warmup_resources(self):
        """Build three native map resources without computing a user route."""
        if not self._planning_lock.acquire(blocking=False):
            raise RuntimeError('Planner snapshot already has an active request')
        try:
            if (_signature(self.raw),_signature(self.settings))!=self._config_signature:
                raise ValueError('Map configuration changed; create a new planner snapshot')
            for name in ('lower_floor', 'stair_lower', 'upper_floor'):
                self._resources(name, self.settings)
            return {'native_map_cache': self._resource_stats(),
                    'fixed_stair_cache': {**self._fixed_stairs.stats,
                        'entries': len(self._fixed_stairs._results)},
                    'scope': 'three_native_maps_not_goal_or_cached_floor_route'}
        except BaseException:
            self._native_maps.clear()
            self._fixed_stairs.clear()
            self.cache_stats['invalidations'] += 1
            raise
        finally:
            self._planning_lock.release()

    def plan(self, start, goal, start_layer, goal_layer):
        # Upstream native objects hold per-query mutable search/optimizer data.
        # A* resets on success/failure; the outer coordinator nevertheless drops
        # all native optimizer and curve-cache state after any failed request.
        # Successful immutable map snapshots may be reused. No concurrent goals.
        if not self._planning_lock.acquire(blocking=False):
            raise RuntimeError('Planner snapshot already has an active request')
        try:
            if (_signature(self.raw), _signature(self.settings)) != self._config_signature:
                raise ValueError('Map configuration changed; create a new planner snapshot')
            result = self._plan(start,goal,start_layer,goal_layer)
            result['native_map_cache'] = self._resource_stats()
            result['fixed_stair_cache'] = {**self._fixed_stairs.stats,
                                          'entries': len(self._fixed_stairs._results)}
            return result
        except BaseException:
            self._native_maps.clear()
            self._fixed_stairs.clear()
            self.cache_stats['invalidations'] += 1
            raise
        finally:
            self._planning_lock.release()

    def _plan(self, start, goal, start_layer, goal_layer):
        began = time.monotonic()
        self.tomogram.validate_endpoint(start, start_layer)
        self.tomogram.validate_endpoint(goal, goal_layer)
        first, last = self._floor(start), self._floor(goal)
        if first not in ('lower', 'upper') or last not in ('lower', 'upper'):
            result = plan_partial_stair(self.tomogram, self.settings, start, goal,
                start_layer, goal_layer, resources=self._resources)
            return _as_preview(result, self.tomogram, time.monotonic()-began)
        if first == last:
            name = first + '_floor'
            masked, route = self._resources(name, self.settings)
            result = route.plan(start, goal, start_layer, goal_layer)
            checks = self.tomogram.validate_path(result['path'], result['layer_ids'])
            return {**result, **checks, 'source_tomogram_sha256': self.tomogram.sha256,
                    'execution_authorized': False, 'route_type': 'same_floor', 'floor': first}
        raw = deepcopy(self.raw)
        reverse = first == 'upper'
        a, b = (goal, start) if reverse else (start, goal)
        la, lb = (goal_layer, start_layer) if reverse else (start_layer, goal_layer)
        raw['anchors']['start'] = {'xyz': list(a), 'layer_id': int(la)}
        raw['anchors']['goal'] = {'xyz': list(b), 'layer_id': int(lb)}
        result = plan_crossfloor(raw, self.tomogram, resources=self._resources,
                                fixed_leg_cache=self._fixed_stairs)
        if reverse:
            count = len(result['path_xyz'])
            for key in ('path_xyz', 'layer_ids', 'source_layer_ids', 'edge_legs'):
                result[key] = result[key][::-1]
            result['segments'] = [{**segment,
                'first_index': count - 1 - segment['last_index'],
                'last_index': count - 1 - segment['first_index'],
                'from': segment['to'], 'to': segment['from']}
                for segment in result['segments'][::-1]]
            result['anchors']['start'], result['anchors']['goal'] = (
                result['anchors']['goal'], result['anchors']['start'])
            endpoint_names = {'start': 'goal', 'goal': 'start'}
            for segment in result['segments']:
                for key in ('from', 'to'):
                    segment[key] = endpoint_names.get(segment[key], segment[key])
            result['layer_transitions'] = [{**event, 'edge': count - 2 - event['edge'],
                'from_layer': event['to_layer'], 'to_layer': event['from_layer']}
                for event in result['layer_transitions'][::-1]]
            validation = validate_config(raw)
            # The selected landing, not the originally requested corner, owns
            # the shared boundary after a checked platform rounding.
            validation['anchors']['landing'] = {
                'xyz': np.asarray(result['anchors']['landing']['xyz'], dtype=float),
                'layer_id': result['anchors']['landing']['layer_id']}
            validation['anchors']['entry'], validation['anchors']['exit'] = (
                validation['anchors']['exit'], validation['anchors']['entry'])
            result['stair_profile'] = _check_stair_profile(
                np.asarray(result['path_xyz']), result['edge_legs'], validation)
            refinement = result.get('stair_refinement')
            if refinement and refinement.get('compiled_stair_connector') is not None:
                stair_records = [segment for segment in result['segments']
                                 if segment['name'].startswith('stair_')]
                stair_start, stair_end = stair_records[0]['first_index'], stair_records[-1]['last_index']
                refinement['landing_index'] = stair_end-stair_start-refinement['landing_index']
                refinement['output_direction'] = 'upper_to_lower'
                refinement['stair_profile'] = deepcopy(result['stair_profile'])
            if refinement and refinement.get('applied'):
                stair_records = [segment for segment in result['segments']
                                 if segment['name'].startswith('stair_')]
                stair_start, stair_end = stair_records[0]['first_index'], stair_records[-1]['last_index']
                refinement['landing_index'] = stair_end-stair_start-refinement['landing_index']
                # Span records refer to the canonical low-to-high polynomial
                # calculation, not to a second solve performed during reversal.
                refinement['span_record_direction'] = 'lower_to_upper'
                refinement['output_direction'] = 'upper_to_lower'
                if 'stair_profile' in refinement:
                    refinement['stair_profile'] = deepcopy(result['stair_profile'])
            result['direction'] = 'upper_to_lower'
        else:
            result['direction'] = 'lower_to_upper'
        return _as_preview(result, self.tomogram, time.monotonic() - began)


def restore_crossfloor(tomogram, config_path, audit_path):
    """Restore only the exact saved map/config; no old-map fallback or mutation."""
    _, settings = load_config(config_path, tomogram)
    record = json.loads(Path(audit_path).read_text())
    if (record.get('status') != 'validated_offline_route'
            or record.get('tomogram_sha256') != tomogram.sha256
            or record.get('source_pcd_sha256') != tomogram.provenance.get('source_sha256')
            or record.get('frame_id') != tomogram.provenance.get('frame_id')
            or record.get('config_sha256') != hashlib.sha256(Path(config_path).read_bytes()).hexdigest()):
        raise ValueError('Saved cross-floor route does not match this map/configuration')
    segments = record.get('segments', [])
    proofs = {'quintic_cell_boundary_roots_and_interval_interiors',
              'polynomial_cell_boundary_roots_and_interval_interiors_plus_validate_path',
              STAIR_REFINEMENT_PROOF}
    from .compiled_stair_connector import PROOF as COMPILED_PROOF, load_compiled_stair_connector
    compiled = load_compiled_stair_connector(tomogram, settings)
    if compiled is not None:
        proofs.add(COMPILED_PROOF)
    refined = record.get('stair_refinement', {})
    if any(segment.get('curve_validation') == COMPILED_PROOF for segment in segments):
        if (compiled is None or refined.get('curve_validation') != COMPILED_PROOF
                or any(segment.get('curve_validation') != COMPILED_PROOF
                       for segment in segments if segment.get('name', '').startswith('stair_'))):
            raise ValueError('Saved compiled stairs require both exact current-map connector legs')
    expected_legs = [leg for leg in LEGS if not _stationary_floor_leg(
        leg[0], settings['anchors'][leg[1]], settings['anchors'][leg[2]])]
    if len(segments) != len(expected_legs) or any(
            segment.get('curve_validation') not in proofs
            or segment.get('curve_partition_points', 0) < 2
            or (segment['name'].startswith('stair_')
                and segment.get('curve_validation') not in (STAIR_REFINEMENT_PROOF, COMPILED_PROOF)
                and segment.get('quintic_segments', 0) <= 0)
            or (segment.get('curve_validation') == COMPILED_PROOF
                and not segment['name'].startswith('stair_'))
            for segment in segments):
        raise ValueError('Saved cross-floor route lacks complete native-curve validation evidence')
    points, layers = np.asarray(record['path_xyz']), np.asarray(record['layer_ids'])
    stair_mask = _masked_tomogram(tomogram, 'stair_lower', settings)
    selected_settings = settings
    if refined.get('applied') is True:
        landing = record['anchors']['landing']
        if (landing.get('requested_xyz') != settings['anchors']['landing']['xyz'].tolist()
                or landing.get('requested_layer_id') != settings['anchors']['landing']['layer_id']):
            raise ValueError('Saved effective landing does not identify its configured portal')
        adjustment = checked_landing_adjustment(tomogram, stair_mask, settings, landing)
        if (refined.get('curve_validation') != STAIR_REFINEMENT_PROOF
                or abs(float(landing.get('displacement_m', -1))-adjustment['displacement_m']) > 1e-8
                or any(segment.get('curve_validation') != STAIR_REFINEMENT_PROOF
                       for segment in segments if segment['name'].startswith('stair_'))):
            raise ValueError('Saved refined stair proof does not match its selected geometry')
        selected_settings = {**settings, 'anchors': {**settings['anchors'],
            'landing': dict(xyz=np.asarray(landing['xyz']), layer_id=landing['layer_id'])}}
    elif any(segment.get('curve_validation') == STAIR_REFINEMENT_PROOF for segment in segments):
        raise ValueError('Saved stair refinement proof lacks its refinement provenance')
    cursor = 0
    for segment, (name, begin, end) in zip(segments, expected_legs):
        a, b = segment['first_index'], segment['last_index']
        if (segment['name'] != name or segment['from'] != begin or segment['to'] != end
                or a != cursor or not a < b < len(points)):
            raise ValueError('Saved cross-floor segment order/coverage is invalid')
        masked = stair_mask if name.startswith('stair_') else _masked_tomogram(tomogram, name, settings)
        if segment.get('curve_validation') == COMPILED_PROOF:
            expected = compiled['legs'][name]
            if (refined.get('applied') is not False
                    or refined.get('curve_validation') != COMPILED_PROOF
                    or segment.get('compiled_stair_connector') != expected['compiled_stair_connector']
                    or refined.get('compiled_stair_connector') != expected['compiled_stair_connector']
                    or not np.array_equal(points[a:b+1], expected['path'])
                    or not np.array_equal(layers[a:b+1], expected['layer_ids'])):
                raise ValueError('Saved compiled stair geometry does not match its current-map bundle')
        _check_leg({'path': points[a:b + 1], 'layer_ids': layers[a:b + 1]}, tomogram, masked,
                   name, selected_settings['anchors'][begin], selected_settings['anchors'][end],
                   selected_settings)
        if record['edge_legs'][a:b] != [name] * (b - a):
            raise ValueError('Saved cross-floor edge identities do not match the segments')
        cursor = b
    if cursor != len(points) - 1 or len(record['edge_legs']) != len(points) - 1:
        raise ValueError('Saved cross-floor route has uncovered edges')
    _check_stair_profile(points, record['edge_legs'], selected_settings)
    result = _as_preview(record, tomogram)
    result['route_origin'] = 'saved_crossfloor_route_exact_map_revalidated'
    return result
