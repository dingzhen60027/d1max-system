"""Native layered PCT planning with exact endpoints and checked quintic curves."""
import time

import numpy as np

from .planner_core import TomogramPlanner
from .path_quality import path_quality
from .tomogram_map import TomogramError, TomogramMap


def real_unit_roots(coefficients):
    coefficients = np.trim_zeros(coefficients, 'b')
    if len(coefficients) < 2:
        return []
    # Straight visibility segments cross hundreds of grid boundaries. A
    # degree-one polynomial has an exact scalar solution; constructing a
    # companion matrix/eigensolver for each boundary adds no information.
    if len(coefficients) == 2 and np.isfinite(coefficients).all():
        value = -coefficients[0] / coefficients[1]
        return [float(value)] if 0 < value < 1 else []
    values = np.polynomial.polynomial.polyroots(coefficients)
    return [float(value.real) for value in values
            if abs(value.imag) <= 1e-8 and 0 < value.real < 1]


def quintic_xy(first, last, dt, resolution, center, offset):
    """Exact WNOJ/Hermite segment from upstream position/velocity/acceleration.

    Upstream state order is native X/v/a/Y/v/a (native X is world Y).
    Coefficients use normalized time [0,1], then become world-metre XY.
    """
    if not np.isfinite(dt) or dt <= 0:
        raise TomogramError('invalid_native_timing', 'Native quintic timing is invalid')
    indices = np.array([[3, 0], [4, 1], [5, 2]])
    a, b = np.asarray(first)[indices], np.asarray(last)[indices]
    coefficients = np.zeros((6, 2))
    coefficients[0], coefficients[1], coefficients[2] = a[0], a[1] * dt, a[2] * dt * dt / 2
    dp = b[0] - coefficients[:3].sum(axis=0)
    dv = b[1] * dt - coefficients[1] - 2 * coefficients[2]
    da = b[2] * dt * dt - 2 * coefficients[2]
    coefficients[3] = 10 * dp - 4 * dv + 0.5 * da
    coefficients[4] = -15 * dp + 7 * dv - da
    coefficients[5] = 6 * dp - 3 * dv + 0.5 * da
    coefficients *= resolution
    coefficients[0] += np.asarray(center) - np.asarray(offset) * resolution
    return coefficients


def curve_partition(coefficients, tomogram):
    """Split at every exact XY cell-boundary crossing and polynomial extremum.

    Each open interval stays in the same grid cells. Validating boundaries and
    interval interiors therefore covers the curve, not just sparse chords.
    """
    times = {0.0, 1.0, 0.5}
    for axis in range(2):
        poly = coefficients[:, axis]
        extrema = real_unit_roots(np.polynomial.polynomial.polyder(poly))
        times.update(extrema)
        values = np.polynomial.polynomial.polyval([0., *extrema, 1.], poly)
        limits = (values - tomogram.center[axis]) / tomogram.resolution + tomogram.offset[axis]
        if limits.min() <= -0.5 or limits.max() >= tomogram.shape[axis] - 0.5:
            raise TomogramError('curve_outside_map', 'Native quintic leaves the measured tomogram',
                                axis=axis, minimum_index=float(limits.min()), maximum_index=float(limits.max()))
        for integer in range(int(np.ceil(limits.min() - 0.5)), int(np.floor(limits.max() - 0.5)) + 1):
            boundary = tomogram.center[axis] + (integer + 0.5 - tomogram.offset[axis]) * tomogram.resolution
            equation = poly.copy()
            equation[0] -= boundary
            times.update(real_unit_roots(equation))
    ordered = sorted(times)
    return sorted(times | {(a + b) / 2 for a, b in zip(ordered, ordered[1:])})


def expand_native_curve(native, tomogram):
    path, layers = np.asarray(native['path']), np.asarray(native['layer_ids'], dtype=int)
    states = native.get('native_states')
    if states is None:
        return path.copy(), layers.copy(), {'quintic_segments': 0, 'curve_partition_points': len(path)}
    states = np.asarray(states, dtype=float)
    if states.shape != (len(path), 6) or not np.isfinite(states).all():
        raise TomogramError('invalid_native_states', 'Layered route requires native quintic states')
    expanded, identities = [], []
    for index, (a, b) in enumerate(zip(states[:-1], states[1:])):
        polynomial = quintic_xy(a, b, native['native_sample_dt'], tomogram.resolution,
                               tomogram.center, tomogram.offset)
        times = curve_partition(polynomial, tomogram)
        positions = np.polynomial.polynomial.polyval(times, polynomial).T
        for t, xy in zip(times, positions):
            # These are the genuine native retained-slice identities, not a
            # default floor. A change remains subject to overlap/step checking.
            layer = int(layers[index] if t < 0.5 else layers[index + 1])
            ground = tomogram.surface_ground_z(xy, layer)
            point = [float(xy[0]), float(xy[1]), ground]
            if expanded and identities[-1] == layer and np.linalg.norm(np.asarray(point) - expanded[-1]) < 1e-9:
                continue
            expanded.append(point)
            identities.append(layer)
    return np.asarray(expanded), np.asarray(identities, dtype=int), {
        'quintic_segments': len(states) - 1, 'curve_partition_points': len(expanded),
        'curve_validation': 'quintic_cell_boundary_roots_and_interval_interiors'}


class TomogramRoute:
    def __init__(self, tomogram, vendor_root, max_heading_rate=10.0, height_tolerance_m=0.08,
                 astar_cost_weight=.2, optimizer_cost_margin=15.0,
                 path_refinement='none', refinement_corner_cut_m=1.5,
                 optimizer_sample_interval=10, defer_map=False):
        if path_refinement not in ('none', 'visibility_c2'):
            raise ValueError('path_refinement must be none or visibility_c2')
        if (isinstance(refinement_corner_cut_m, (bool, np.bool_))
                or not np.isfinite(refinement_corner_cut_m)
                or not 0 < refinement_corner_cut_m <= 5):
            raise ValueError('refinement_corner_cut_m must be in (0, 5] metres')
        self.path_refinement = path_refinement
        self.refinement_corner_cut_m = float(refinement_corner_cut_m)
        self.tomogram = tomogram if isinstance(tomogram, TomogramMap) else TomogramMap(tomogram)
        self.height_tolerance_m = float(height_tolerance_m)
        from .native_runtime import probe_native_libraries
        probe_native_libraries(vendor_root)
        self.planner = TomogramPlanner(vendor_root, use_quintic=True,
                                      max_heading_rate=max_heading_rate, ground_z=True,
                                      astar_cost_weight=astar_cost_weight,
                                      optimizer_cost_margin=optimizer_cost_margin,
                                      optimizer_sample_interval=optimizer_sample_interval)
        self.native_parameters = dict(self.planner.native_parameters)
        self.native_runtime = probe_native_libraries(vendor_root, require_loaded=True)
        self._map_loaded = False
        if not defer_map:
            self._load_native_map()

    def _load_native_map(self):
        if not self._map_loaded:
            self.planner.load_payload(self.tomogram.native_payload())
            self._map_loaded = True

    def plan(self, start_xyz, goal_xyz, start_layer, goal_layer):
        started = time.monotonic()
        start = self.tomogram.validate_endpoint(start_xyz, start_layer, self.height_tolerance_m)
        goal = self.tomogram.validate_endpoint(goal_xyz, goal_layer, self.height_tolerance_m)
        if start['layer_id'] == goal['layer_id'] and np.array_equal(
                self.tomogram.index(start['xyz'][:2]), self.tomogram.index(goal['xyz'][:2])):
            raise TomogramError('endpoints_too_close', 'Select distinct cells for native PCT planning')
        if getattr(self, '_map_loaded', True) is False:
            self._load_native_map()
        native = self.planner.plan(start['xyz'][:2], goal['xyz'][:2],
                                   start['layer_id'], goal['layer_id'], return_details=True)
        if native is None:
            raise TomogramError('native_planning_failed', 'Native PCT A* / GPMP did not find a route')
        try:
            path, layers, curve_checks = expand_native_curve(native, self.tomogram)
        except TomogramError as exc:
            raise TomogramError('invalid_curve', 'Native GPMP curve leaves the traversable surface',
                                cause_code=exc.code, **exc.details) from exc
        if len(path) < 2:
            raise TomogramError('empty_native_curve', 'Native PCT returned insufficient curve data')
        # The native search omits its start and uses grid centres. Add the
        # precise editor endpoints, then validate both connectors as well.
        path = np.vstack([start['xyz'], path, goal['xyz']])
        layers = np.r_[start['layer_id'], layers, goal['layer_id']]
        keep = np.r_[True, (np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-9)
                           | (np.diff(layers) != 0)]
        path, layers = path[keep], layers[keep]
        if len(path) < 2:
            path, layers = np.array([start['xyz'], goal['xyz']]), np.array([start_layer, goal_layer])
        path[0], path[-1] = start['xyz'], goal['xyz']
        layers[0], layers[-1] = start['layer_id'], goal['layer_id']
        checks = self.tomogram.validate_path(path, layers)
        # Refinement is an explicit, independently checked optional stage.
        # Never invoke it to rescue an invalid native curve or weaken a mask.
        refinement = {'applied': False, 'reason': 'disabled'}
        native_geometry = None
        algorithm = 'upstream PCT OfflineElePlanner + GPMP quintic'
        if getattr(self, 'path_refinement', 'none') == 'visibility_c2':
            from .corridor_refinement import refine_corridor
            refinement = refine_corridor(self.tomogram, path, layers,
                                         corner_cut_m=self.refinement_corner_cut_m)
            if refinement['applied']:
                native_geometry = {
                    **curve_checks, 'length_m': float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                    'path_quality': path_quality(path),
                    'collision_checks': checks,
                }
                path = np.asarray(refinement['path'], dtype=float)
                layers = np.asarray(refinement['layer_ids'], dtype=int)
                if not np.array_equal(path[0], start['xyz']) or not np.array_equal(path[-1], goal['xyz']):
                    raise TomogramError('refinement_endpoint_mismatch', 'Refinement changed exact endpoints')
                checks = self.tomogram.validate_path(path, layers)
                curve_checks = {
                    'curve_validation': refinement['curve_validation'],
                    'quintic_segments': sum(segment.get('kind') != 'line'
                                            for segment in refinement.get('segments', [])),
                    'curve_partition_points': len(path),
                }
                algorithm += ' + checked visibility/C2 corridor refinement'
        return {'path': path.tolist(), 'layer_ids': layers.tolist(),
                'source_layer_ids': self.tomogram.source_layers[layers].tolist(),
                'start_layer': int(start_layer), 'goal_layer': int(goal_layer),
                'start_xyz': start['xyz'], 'goal_xyz': goal['xyz'],
                'length_m': float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                'elapsed_s': time.monotonic() - started,
                'algorithm': algorithm,
                'map_backend': 'official_pct_cpu_layered_tomogram',
                'source_tomogram_sha256': self.tomogram.sha256,
                'native_runtime': getattr(self, 'native_runtime', None),
                'native_parameters': getattr(self, 'native_parameters', None),
                'path_refinement': {key: value for key, value in refinement.items()
                                    if key not in ('path', 'layer_ids')},
                'native_geometry': native_geometry,
                'height_semantics': 'measured_ground_with_exact_selected_endpoints',
                # Geometry diagnostics are not a replacement for collision checks.
                'path_quality': path_quality(path),
                **curve_checks, **checks}


def worker_main(settings, requests, results):
    """Native bindings stay in a separately owned worker process."""
    try:
        tomogram = TomogramMap(settings['tomogram_path'],
            minimum_headroom_m=settings.get('minimum_headroom_m'),
            unknown_ceiling_policy=settings.get('unknown_ceiling_policy', 'allow_unobserved'),
            max_ground_step_m=settings.get('max_ground_step_m', 0.15))
        if settings.get('crossfloor_route_config'):
            from .crossfloor_preview import CrossfloorPreviewRoute
            planner = CrossfloorPreviewRoute(tomogram, settings['crossfloor_route_config'])
        else:
            # The cross-floor coordinator builds its own masked native maps.
            # Do not allocate/load a full native map only to discard it here.
            planner = TomogramRoute(tomogram, settings['vendor_root'],
                max_heading_rate=settings.get('max_heading_rate', 10.0),
                height_tolerance_m=settings.get('max_selection_height_error_m', 0.08),
                astar_cost_weight=settings.get('astar_cost_weight', .2),
                optimizer_cost_margin=settings.get('optimizer_cost_margin', 15.0),
                optimizer_sample_interval=settings.get('optimizer_sample_interval', 10),
                path_refinement=settings.get('path_refinement', 'none'),
                refinement_corner_cut_m=settings.get('refinement_corner_cut_m', 1.5))
        results.put({'kind': 'ready', 'source_tomogram_sha256': tomogram.sha256,
                     'backend': 'native_pct_gpmp_layered_tomogram', 'layers': tomogram.available_layers(),
                     'native_runtime': planner.native_runtime,
                     'native_parameters': planner.native_parameters,
                     'traversable_layer_cells': int(tomogram.valid.sum())})
    except Exception as exc:
        results.put({'kind': 'initialization_failed', 'error': str(exc),
                     'error_code': getattr(exc, 'code', 'initialization_failed'),
                     'details': getattr(exc, 'details', {})})
        return
    while True:
        request = requests.get()
        if request is None:
            return
        generation = request['generation']
        try:
            result = planner.plan(request['start_xyz'], request['goal_xyz'],
                                  request['start_layer'], request['goal_layer'])
            results.put({'kind': 'planned', 'generation': generation, **result})
        except Exception as exc:
            results.put({'kind': 'failed', 'generation': generation, 'error': str(exc),
                         'error_code': getattr(exc, 'code', 'native_error'),
                         'details': getattr(exc, 'details', {})})
