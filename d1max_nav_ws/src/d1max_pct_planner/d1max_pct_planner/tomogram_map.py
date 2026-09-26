"""Measured layered PCT tomogram contract, selection and curve validation.

No XY snapping, synthetic ceiling, extra erosion or replacement planner lives
here. Native PCT uses cell centres / numpy rint; geometric validation additionally
checks both sides of a cell boundary, including its corners.
"""
import hashlib
import math
from pathlib import Path

import numpy as np


class TomogramError(ValueError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code, self.details = code, details

    def as_dict(self):
        return {'code': self.code, 'message': str(self), **self.details}


class TomogramMap:
    COST_THRESHOLD = 20.0

    def __init__(self, source, minimum_headroom_m=None,
                 unknown_ceiling_policy='allow_unobserved', max_ground_step_m=0.15):
        if isinstance(source, (str, Path)):
            self.source = str(Path(source).resolve())
            self.sha256 = hashlib.sha256(Path(source).read_bytes()).hexdigest()
            with np.load(source, allow_pickle=False) as archive:
                payload = {name: archive[name].copy() for name in archive.files}
        else:
            self.source, self.sha256 = None, None
            payload = dict(source)
        self.provenance = {name: str(np.asarray(payload[name]).item()) for name in
                           ('source_pcd', 'source_sha256', 'frame_id', 'upstream_commit',
                            'source_processing_manifest', 'source_processing_manifest_sha256',
                            'processing_schema', 'geometry_operation', 'original_source_pcd',
                            'original_source_sha256', 'ground_semantics', 'ceiling_semantics')
                           if name in payload}
        if 'planning_only' in payload:
            planning_only = np.asarray(payload['planning_only'])
            if planning_only.ndim != 0 or planning_only.dtype.kind != 'b':
                raise TomogramError('invalid_map_provenance', 'planning_only must be a boolean scalar')
            self.provenance['planning_only'] = bool(planning_only.item())
        self.data = np.asarray(payload['data'], dtype=np.float32).copy()
        if self.data.ndim != 4 or self.data.shape[0] != 5 or min(self.data.shape[1:]) < 1:
            raise TomogramError('invalid_tomogram', 'Expected data[5, layers, X, Y]')
        self.resolution = float(payload['resolution'])
        self.center = np.asarray(payload['center'], dtype=float)
        self.slice_h0, self.slice_dh = float(payload['slice_h0']), float(payload['slice_dh'])
        if (not np.isfinite(self.resolution) or self.resolution <= 0
                or self.center.shape != (2,) or not np.isfinite(self.center).all()
                or not np.isfinite([self.slice_h0, self.slice_dh]).all() or self.slice_dh <= 0):
            raise TomogramError('invalid_geometry', 'Invalid tomogram centre, resolution or slices')
        self.layers, self.nx, self.ny = self.data.shape[1:]
        self.shape = (self.nx, self.ny)
        self.offset = np.array([self.nx // 2, self.ny // 2], dtype=int)
        self.origin = self.center - (self.offset + 0.5) * self.resolution
        self.source_layers = np.asarray(payload.get('selected_source_layers', np.arange(self.layers)))
        if (self.source_layers.shape != (self.layers,) or not np.isfinite(self.source_layers).all()
                or not np.equal(self.source_layers, np.rint(self.source_layers)).all()
                or (np.diff(self.source_layers) <= 0).any() or (self.source_layers < 0).any()):
            raise TomogramError('invalid_layers', 'Source slice indices must be unique increasing integers')
        self.source_layers = self.source_layers.astype(int)
        self.minimum_headroom_m = float(payload.get('minimum_headroom_m', 0.55)
                                       if minimum_headroom_m is None else minimum_headroom_m)
        self.max_ground_step_m = float(max_ground_step_m)
        if (not np.isfinite([self.minimum_headroom_m, self.max_ground_step_m]).all()
                or self.minimum_headroom_m <= 0 or self.max_ground_step_m <= 0):
            raise TomogramError('invalid_clearance', 'Headroom and step limits must be finite and positive')
        if unknown_ceiling_policy not in ('allow_unobserved', 'reject'):
            raise TomogramError('invalid_ceiling_policy', 'Unknown ceiling policy must be explicit')
        self.unknown_ceiling_policy = unknown_ceiling_policy
        self.cost, self.gradient_x, self.gradient_y, self.ground, self.ceiling = self.data
        if np.isinf(self.data).any():
            raise TomogramError('invalid_tomogram', 'Use NaN for unobserved surfaces, not infinity')
        self.ground_known = np.isfinite(self.ground) & (self.ground > -99.0)
        self.ceiling_known = np.isfinite(self.ceiling) & (self.ceiling < 1e5)
        self.open_sky_verified = np.asarray(payload.get('open_sky_verified',
                                                       np.zeros_like(self.ground, dtype=bool)), dtype=bool)
        if self.open_sky_verified.shape != self.ground.shape:
            raise TomogramError('invalid_open_sky_mask', 'Open-sky mask must match layer/X/Y shape')
        self.headroom = np.where(self.ceiling_known, self.ceiling - self.ground, np.nan)
        ceiling_acceptable = (self.ceiling_known & (self.headroom >= self.minimum_headroom_m - 1e-6))
        ceiling_acceptable |= ~self.ceiling_known & (
            self.open_sky_verified | (unknown_ceiling_policy == 'allow_unobserved'))
        self.valid = (self.ground_known & np.isfinite(self.cost) & (self.cost >= 0)
                      & (self.cost <= self.COST_THRESHOLD) & ceiling_acceptable)
        self.allowed = self.valid
        self.traversable = self.valid

    def verify_source(self, pcd_path, frame_id):
        """Never overlay a different PCD or silently relabel map coordinates."""
        if not self.provenance.get('source_sha256') or not self.provenance.get('frame_id'):
            raise TomogramError('missing_map_provenance', 'Rebuild tomography with source hash and frame')
        if frame_id != self.provenance['frame_id']:
            raise TomogramError('map_frame_mismatch', 'PCD tomogram frame differs from preview frame',
                                expected=self.provenance['frame_id'], actual=frame_id)
        digest = hashlib.sha256()
        with Path(pcd_path).open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        if digest.hexdigest() != self.provenance['source_sha256']:
            raise TomogramError('map_source_mismatch', 'Displayed PCD is not the tomogram source',
                                expected=self.provenance['source_sha256'], actual=digest.hexdigest())
        return dict(self.provenance)

    def index(self, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise TomogramError('invalid_xy', 'XY must contain two finite values')
        relative = (xy - self.center) / self.resolution
        if np.max(np.abs(relative)) > 1e8:
            raise TomogramError('outside_map', 'XY is outside the tomogram')
        return np.rint(relative).astype(int) + self.offset

    def world(self, index_xy):
        index = np.asarray(index_xy, dtype=float)
        if index.shape != (2,) or not np.isfinite(index).all():
            raise TomogramError('invalid_index', 'Cell index must contain X/Y')
        return self.center + (index - self.offset) * self.resolution

    def contains(self, cell):
        return 0 <= cell[0] < self.nx and 0 <= cell[1] < self.ny

    def layer(self, value):
        if isinstance(value, (bool, np.bool_)) or not np.isfinite(value) or int(value) != value:
            raise TomogramError('invalid_layer', 'Layer must be an integer index')
        value = int(value)
        if not 0 <= value < self.layers:
            raise TomogramError('invalid_layer', 'Layer is outside the tomogram', layer_id=value)
        return value

    def point_cells(self, xy):
        # Public validation remains identical to index(); hot geometric loops
        # call the scalar helper only after both endpoints have passed it.
        self.index(xy)
        return self._point_cells_scalar(float(xy[0]), float(xy[1]))

    def _point_cells_scalar(self, x, y):
        """Exact rint/supercover convention without tiny NumPy allocations.

        Round the *relative* coordinate before adding offset. Rounding the
        absolute cell coordinate instead changes half ties with odd offsets.
        Python round and np.rint both use round-to-even for finite doubles.
        """
        relative_x = (x-float(self.center[0]))/self.resolution
        relative_y = (y-float(self.center[1]))/self.resolution
        continuous_x = relative_x+int(self.offset[0])
        continuous_y = relative_y+int(self.offset[1])
        lower_x, lower_y = math.floor(continuous_x), math.floor(continuous_y)
        xs = ((lower_x, lower_x+1) if abs(continuous_x-lower_x-.5)<1e-8
              else (round(relative_x)+int(self.offset[0]),))
        ys = ((lower_y, lower_y+1) if abs(continuous_y-lower_y-.5)<1e-8
              else (round(relative_y)+int(self.offset[1]),))
        return {(ix, iy) for ix in xs for iy in ys}

    def _reason(self, layer, cell):
        if not self.contains(cell):
            return 'outside_map'
        key = (layer, *cell)
        if not self.ground_known[key]:
            return 'unobserved_ground'
        cost = float(self.cost[key])
        if not math.isfinite(cost) or cost < 0:
            return 'unknown_cost'
        if cost > self.COST_THRESHOLD:
            return 'pct_cost_blocked'
        if self.ceiling_known[key] and self.headroom[key] < self.minimum_headroom_m - 1e-6:
            return 'insufficient_headroom'
        if (not self.ceiling_known[key] and not self.open_sky_verified[key]
                and self.unknown_ceiling_policy == 'reject'):
            return 'unobserved_ceiling'
        return None

    def _cell_info(self, layer, cell, xy):
        reason = self._reason(layer, cell)
        if not self.contains(cell):
            return {'layer_id': layer, 'valid': False, 'reason_code': reason}
        key = (layer, *cell)
        measured = bool(self.ceiling_known[key])
        ground = float(self.ground[key]) if self.ground_known[key] else None
        return {'layer_id': int(layer), 'source_layer': int(self.source_layers[layer]),
                'cell_xy': list(map(int, cell)), 'xyz': [float(xy[0]), float(xy[1]), ground],
                'ground_z': ground, 'ceiling_z': float(self.ceiling[key]) if measured else None,
                'ceiling_state': 'measured' if measured else (
                    'verified_open_sky' if self.open_sky_verified[key] else 'unobserved_above'),
                'headroom_m': float(self.headroom[key]) if measured and ground is not None else None,
                'cost': float(self.cost[key]) if np.isfinite(self.cost[key]) else None,
                'valid': reason is None, 'reason_code': reason}

    def _surface(self, xy, layer):
        layer = self.layer(layer)
        cell = tuple(self.index(xy))
        result = self._cell_info(layer, cell, xy)
        if not result['valid']:
            raise TomogramError(result['reason_code'], 'Selected surface is not traversable', **result)
        touched = self.point_cells(xy)
        for boundary_cell in touched:
            reason = self._reason(layer, boundary_cell)
            if reason:
                raise TomogramError(reason, 'Selected point touches invalid cell boundary',
                                    layer_id=layer, cell_xy=list(boundary_cell))
            if abs(self.ground[(layer, *boundary_cell)] - result['ground_z']) > self.max_ground_step_m:
                raise TomogramError('ground_step', 'Boundary crosses a ground-height discontinuity')
        result['boundary_cells'] = len(touched)
        return result

    def surface_ground_z(self, xy, layer):
        """Same support contract as _surface, without selector UI dictionaries.

        Expansion can ask for thousands of heights per route. The normal path
        needs only Z; on a failure use the full selector to preserve its exact
        reason/details. No validity result is cached across calls, so edited
        cost, ground or clearance arrays are always checked again.
        """
        layer = self.layer(layer)
        cell = tuple(self.index(xy))
        if self._reason(layer, cell):
            return self._surface(xy, layer)['ground_z']
        ground = float(self.ground[(layer, *cell)])
        for boundary_cell in self._point_cells_scalar(float(xy[0]), float(xy[1])):
            if boundary_cell == cell:
                continue  # centre cell already passed the identical predicate
            if (self._reason(layer, boundary_cell)
                    or abs(self.ground[(layer, *boundary_cell)] - ground) > self.max_ground_step_m):
                return self._surface(xy, layer)['ground_z']
        return ground

    def surface_ground_z_many(self, positions, layer):
        """Batch the identical support predicate, including all boundary cells.

        This consumes only current map arrays; it is not a validity cache.
        Temporary storage is bounded by 4096 points per chunk (four supercover
        cells each), not by map size. Scalar fallback preserves exact ordered
        error diagnostics for invalid input or a rejected support sample.
        """
        points = np.asarray(positions, dtype=float)
        layer = self.layer(layer)
        if points.ndim != 2 or points.shape[1] != 2:
            raise TomogramError('invalid_xy', 'XY batch must contain finite XY pairs')
        if len(points) > 4096:
            return np.concatenate([self.surface_ground_z_many(points[first:first+4096], layer)
                                   for first in range(0, len(points), 4096)])
        relative = (points-self.center)/self.resolution
        if (not np.isfinite(relative).all() or np.max(np.abs(relative), initial=0.) > 1e8):
            return np.array([self.surface_ground_z(point, layer) for point in points])
        indices = np.rint(relative).astype(int)+self.offset
        continuous = relative+self.offset
        lower = np.floor(continuous).astype(int)
        boundary = np.abs(continuous-lower-.5)<1e-8
        first = np.where(boundary, lower, indices)
        second = np.where(boundary, lower+1, indices)
        # Duplicating a non-boundary cell is harmless. Fixed four-column
        # arrays avoid Python objects for each of thousands of samples.
        xs = np.stack([first[:, 0], first[:, 0], second[:, 0], second[:, 0]], axis=1)
        ys = np.stack([first[:, 1], second[:, 1], first[:, 1], second[:, 1]], axis=1)
        inside = (xs>=0)&(xs<self.nx)&(ys>=0)&(ys<self.ny)
        center_inside = (indices[:, 0]>=0)&(indices[:, 0]<self.nx)&(indices[:, 1]>=0)&(indices[:, 1]<self.ny)
        safe_x, safe_y = np.clip(xs, 0, self.nx-1), np.clip(ys, 0, self.ny-1)
        keys = (layer, safe_x, safe_y)
        center_x = np.clip(indices[:, 0], 0, self.nx-1)
        center_y = np.clip(indices[:, 1], 0, self.ny-1)
        ground = self.ground[layer, center_x, center_y].astype(float)
        # Scalar numpy values promote to Python double in _reason/_surface;
        # use explicit float64 here too at threshold/step boundary values.
        costs = self.cost[keys].astype(float)
        headroom = self.headroom[keys].astype(float)
        known_ceiling = self.ceiling_known[keys]
        valid = (inside & self.ground_known[keys] & np.isfinite(costs) & (costs>=0)
                 & (costs<=self.COST_THRESHOLD)
                 & ~(known_ceiling & (headroom<self.minimum_headroom_m-1e-6)))
        if self.unknown_ceiling_policy == 'reject':
            valid &= known_ceiling | self.open_sky_verified[keys]
        valid &= ~(np.abs(self.ground[keys].astype(float)-ground[:, None])>self.max_ground_step_m)
        accepted = center_inside & valid.all(axis=1)
        for index in np.flatnonzero(~accepted):
            # A centre exactly on a tie can be either adjacent cell. The full
            # selector retains its original reason ordering/details on error.
            self._surface(points[index], layer)
        return ground

    def sample_surfaces(self, xy, traversable_only=True, deduplicate=True):
        cell = tuple(self.index(xy))
        if not self.contains(cell):
            return []
        results = []
        for layer in range(self.layers):
            if traversable_only:
                try:
                    info = self._surface(xy, layer)
                except TomogramError:
                    continue
            else:
                info = self._cell_info(layer, cell, xy)
            info['equivalent_layers'] = [layer]
            if deduplicate and info['valid']:
                same = next((item for item in results if item['valid']
                             and abs(item['ground_z'] - info['ground_z']) <= 1e-4), None)
                if same is not None:
                    same['equivalent_layers'].append(layer)
                    continue
            results.append(info)
        return sorted(results, key=lambda item: (item['ground_z'] is None,
                                                item['ground_z'] or 0., item['layer_id']))

    def select(self, xyz, mode='free_xyz', preferred_layer=None, layer_lock=None,
               height_tolerance_m=0.08, max_follow_height_change_m=0.3):
        point = np.asarray(xyz, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise TomogramError('invalid_xyz', 'Select a finite XYZ point')
        if mode not in ('free_xyz', 'ground_follow'):
            raise TomogramError('invalid_selection_mode', 'Use free_xyz or ground_follow')
        limit = float(height_tolerance_m if mode == 'free_xyz' else max_follow_height_change_m)
        if not np.isfinite(limit) or limit <= 0:
            raise TomogramError('invalid_height_tolerance', 'Height tolerance must be finite and positive')
        if layer_lock is not None:
            candidates = [self._surface(point[:2], layer_lock)]
        else:
            candidates = self.sample_surfaces(point[:2], deduplicate=False)
        eligible = (candidates if layer_lock is not None and mode == 'ground_follow' else
                    [info for info in candidates if abs(info['ground_z'] - point[2]) <= limit + 1e-8])
        if not eligible:
            cell = tuple(self.index(point[:2]))
            if not self.contains(cell):
                raise TomogramError('outside_map', 'Selected XY lies outside the tomogram')
            # Preserve the geometric reason at the requested surface height.
            # A roof elsewhere in Z must not turn a blocked floor into a mere
            # height-entry error. No fallback changes XY or the allowed mask.
            nearby = [info for info in self.sample_surfaces(
                point[:2], traversable_only=False, deduplicate=False)
                if info['ground_z'] is not None and abs(info['ground_z'] - point[2]) <= limit + 1e-8]
            nearby.sort(key=lambda info: (abs(info['ground_z'] - point[2]),
                                         info['layer_id'] != preferred_layer, info['layer_id']))
            for info in nearby:
                try:
                    self._surface(point[:2], info['layer_id'])
                except TomogramError as exc:
                    raise exc
            code = 'height_off_surface' if candidates else 'no_traversable_surface'
            raise TomogramError(code, 'No supported surface at selected XYZ; XY was not moved',
                                xyz=point.tolist(), candidate_layers=candidates, tolerance_m=limit)
        preferred = None
        if preferred_layer is not None:
            preferred_layer = self.layer(preferred_layer)
            preferred = next((info for info in eligible if info['layer_id'] == preferred_layer), None)
        if preferred is not None:
            chosen = preferred
        else:
            ranked = sorted(eligible, key=lambda info: (abs(info['ground_z'] - point[2]), info['layer_id']))
            chosen = ranked[0]
            for other in ranked[1:]:
                if (abs(abs(other['ground_z'] - point[2]) - abs(chosen['ground_z'] - point[2])) < 1e-7
                        and abs(other['ground_z'] - chosen['ground_z']) > 1e-4):
                    raise TomogramError('ambiguous_surface', 'Choose an explicit layer or surface height',
                                        candidate_layers=ranked)
            if mode == 'ground_follow' and preferred_layer is not None and layer_lock is None:
                # A different retained slice is allowed only for the same
                # continuous physical surface, never as a silent floor jump.
                cell = tuple(self.index(point[:2]))
                old_ground = self.ground[(preferred_layer, *cell)] if self.contains(cell) else np.nan
                previous_height = float(old_ground) if np.isfinite(old_ground) else float(point[2])
                if abs(previous_height - chosen['ground_z']) > self.max_ground_step_m:
                    raise TomogramError('surface_switch_requires_selection',
                                        'Previous surface ended; choose a layer explicitly')
        selected = point.copy()
        if mode == 'ground_follow':
            selected[2] = chosen['ground_z']
        return {**chosen, 'xyz': selected.tolist(), 'selection_mode': mode,
                'height_error_m': float(selected[2] - chosen['ground_z'])}

    def validate_endpoint(self, xyz, layer_id=None, height_tolerance_m=0.08):
        return self.select(xyz, mode='free_xyz', layer_lock=layer_id,
                           height_tolerance_m=height_tolerance_m)

    def initial_seed(self, anchor_xyz, layer_lock=None):
        anchor = np.asarray(anchor_xyz, dtype=float)
        if anchor.shape != (3,) or not np.isfinite(anchor).all():
            raise TomogramError('invalid_anchor', 'Initial placement requires XYZ height guidance')
        candidates = np.argwhere(self.valid)
        if layer_lock is not None:
            candidates = candidates[candidates[:, 0] == self.layer(layer_lock)]
        if not len(candidates):
            raise TomogramError('no_seed', 'No traversable surface for initial placement')
        heights = self.ground[tuple(candidates.T)]
        # Seed search may relocate initial XY, but it never discards floor Z.
        near_height = np.abs(heights - anchor[2]) <= max(self.slice_dh, self.max_ground_step_m)
        candidates, heights = candidates[near_height], heights[near_height]
        if not len(candidates):
            raise TomogramError('no_seed_at_height', 'No supported initial surface near requested height')
        xy = self.center + (candidates[:, 1:] - self.offset) * self.resolution
        distance = np.sum((xy - anchor[:2]) ** 2, axis=1) + (heights - anchor[2]) ** 2
        selected = int(np.argmin(distance))
        return self._surface(xy[selected], int(candidates[selected, 0]))

    def display_points(self, layer_id=None, deduplicate=True):
        indices = np.argwhere(self.valid)
        if layer_id is not None:
            indices = indices[indices[:, 0] == self.layer(layer_id)]
        xyz = np.column_stack((self.center + (indices[:, 1:] - self.offset) * self.resolution,
                               self.ground[tuple(indices.T)]))
        if deduplicate and len(xyz):
            _, keep = np.unique(np.round(xyz, 4), axis=0, return_index=True)
            keep.sort()
            xyz, indices = xyz[keep], indices[keep]
        return {'xyz': xyz, 'layer_ids': indices[:, 0], 'cost': self.cost[tuple(indices.T)],
                'ceiling_known': self.ceiling_known[tuple(indices.T)]}

    def available_layers(self):
        result = []
        for layer in range(self.layers):
            heights = self.ground[layer][self.valid[layer]]
            result.append({'layer_id': layer, 'source_layer': int(self.source_layers[layer]),
                           'slice_height_m': self.slice_h0 + int(self.source_layers[layer]) * self.slice_dh,
                           'traversable_cells': int(self.valid[layer].sum()),
                           'ground_min_m': float(heights.min()) if len(heights) else None,
                           'ground_max_m': float(heights.max()) if len(heights) else None,
                           'unobserved_ceiling_cells': int((self.valid[layer]
                                                           & ~self.ceiling_known[layer]).sum())})
        return result

    def segment_samples(self, a, b):
        for endpoint in (a, b):
            if not self.contains(tuple(self.index(endpoint))):
                raise TomogramError('curve_outside_map', 'Curve endpoint lies outside the tomogram')
        if np.asarray(a).dtype == np.float64 and np.asarray(b).dtype == np.float64:
            return self._segment_samples_f64(a, b, check_endpoints=False)
        start = (np.asarray(a) - self.center) / self.resolution + self.offset
        end = (np.asarray(b) - self.center) / self.resolution + self.offset
        delta = end - start
        boundaries = {0.0, 1.0}
        for axis in range(2):
            if abs(delta[axis]) < 1e-12:
                continue
            low, high = sorted((start[axis], end[axis]))
            for integer in range(int(np.ceil(low - 0.5)), int(np.floor(high - 0.5)) + 1):
                t = (integer + 0.5 - start[axis]) / delta[axis]
                if 0 < t < 1:
                    boundaries.add(float(t))
        boundaries = sorted(boundaries)
        times = sorted(boundaries + [(a + b) / 2 for a, b in zip(boundaries, boundaries[1:])])
        # Preserve public float32/integer interpolation semantics around ties.
        return [(t, self.point_cells(np.asarray(a)+t*(np.asarray(b)-np.asarray(a))))
                for t in times]

    def _segment_samples_f64(self, a, b, check_endpoints=True):
        """Scalar supercover for finite float64 XY already shape-validated.

        validate_path checks all XYZ values once. Reallocating 2-element NumPy
        arrays and validating their shape at every short segment/grid crossing
        is unnecessary; rounding and boundary arithmetic stay identical.
        """
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        cx, cy = float(self.center[0]), float(self.center[1])
        ox, oy = int(self.offset[0]), int(self.offset[1])
        start = ((ax-cx)/self.resolution, (ay-cy)/self.resolution)
        end = ((bx-cx)/self.resolution, (by-cy)/self.resolution)
        if check_endpoints:
            for relative in (start, end):
                if max(abs(relative[0]), abs(relative[1])) > 1e8:
                    raise TomogramError('outside_map', 'XY is outside the tomogram')
                if not self.contains((round(relative[0])+ox, round(relative[1])+oy)):
                    raise TomogramError('curve_outside_map', 'Curve endpoint lies outside the tomogram')
        start, end = (start[0]+ox, start[1]+oy), (end[0]+ox, end[1]+oy)
        boundaries = {0., 1.}
        for first, last in zip(start, end):
            delta = last-first
            if abs(delta) < 1e-12:
                continue
            low, high = sorted((first, last))
            for integer in range(math.ceil(low-.5), math.floor(high-.5)+1):
                t = (integer+.5-first)/delta
                if 0 < t < 1:
                    boundaries.add(t)
        boundaries = sorted(boundaries)
        times = sorted(boundaries+[(first+last)/2 for first, last in zip(boundaries, boundaries[1:])])
        dx, dy = bx-ax, by-ay
        return [(t, self._point_cells_scalar(ax+t*dx, ay+t*dy)) for t in times]

    def validate_path(self, path_xyz, layer_ids, max_ground_step_m=None, surface_tolerance_m=None):
        path = np.asarray(path_xyz, dtype=float)
        layers = np.asarray(layer_ids)
        if (path.ndim != 2 or path.shape[1] != 3 or len(path) < 2 or not np.isfinite(path).all()
                or layers.shape != (len(path),)):
            raise TomogramError('invalid_path', 'Path needs finite XYZ and one explicit layer per point')
        layers = np.array([self.layer(value) for value in layers])
        step = self.max_ground_step_m if max_ground_step_m is None else float(max_ground_step_m)
        tolerance = max(step, 0.08) if surface_tolerance_m is None else float(surface_tolerance_m)
        if not np.isfinite([step, tolerance]).all() or min(step, tolerance) <= 0:
            raise TomogramError('invalid_path_limits', 'Path step and surface tolerance must be positive')
        touched, boundary_count, changes, unknown_ceiling = set(), 0, 0, set()
        for segment, (a, b, first, last) in enumerate(zip(path[:-1], path[1:], layers[:-1], layers[1:])):
            if abs(int(last) - int(first)) > 1:
                raise TomogramError('nonadjacent_layer_transition', 'Native curve skips a retained layer',
                                    segment=segment, layers=[int(first), int(last)])
            if first == last:
                # The overwhelmingly common case has one reachable state.
                # Min/max bounds are exactly the Cartesian-product height
                # test below, without allocating per-sample state dictionaries.
                layer = int(first)
                previous_min = previous_max = float(a[2])
                for t, cells in self._segment_samples_f64(a, b):
                    z = float(a[2] + t * (b[2] - a[2]))
                    heights, reasons = [], []
                    for cell in cells:
                        reason = self._reason(layer, cell)
                        if reason:
                            reasons.append({'layer': layer, 'cell': list(cell), 'code': reason})
                            break
                        ground = float(self.ground[(layer, *cell)])
                        if abs(z - ground) > tolerance + 1e-6:
                            reasons.append({'layer': layer, 'cell': list(cell), 'code': 'curve_off_surface'})
                            break
                        heights.append(ground)
                    low, high = (min(heights), max(heights)) if heights else (0., 0.)
                    if (reasons or not high-low <= step+1e-6
                            or max(abs(high-previous_min), abs(previous_max-low)) > step+1e-6):
                        raise TomogramError('invalid_curve', 'Optimized curve crosses unsupported/blocked/discontinuous surface',
                                            segment=segment, fraction=t, reasons=reasons,
                                            layers=[int(first), int(last)], cells=[list(cell) for cell in sorted(cells)])
                    previous_min, previous_max = low, high
                    boundary_count += int(len(cells) > 1)
                    for cell in cells:
                        key = (layer, *cell)
                        touched.add(key)
                        if not self.ceiling_known[key] and not self.open_sky_verified[key]:
                            unknown_ceiling.add(key)
                continue
            states = {int(first)}
            previous_heights = {int(first): [float(a[2])]}
            for t, cells in self._segment_samples_f64(a, b):
                z = float(a[2] + t * (b[2] - a[2]))
                possible = {}
                reasons = []
                for layer in {int(first), int(last)}:
                    cell_heights = []
                    for cell in cells:
                        reason = self._reason(layer, cell)
                        if reason:
                            reasons.append({'layer': layer, 'cell': list(cell), 'code': reason})
                            break
                        ground = float(self.ground[(layer, *cell)])
                        if abs(z - ground) > tolerance + 1e-6:
                            reasons.append({'layer': layer, 'cell': list(cell), 'code': 'curve_off_surface'})
                            break
                        cell_heights.append(ground)
                    else:
                        if max(cell_heights) - min(cell_heights) <= step + 1e-6:
                            possible[layer] = cell_heights
                next_states = set()
                for layer, heights in possible.items():
                    if t == 0 and layer != first or t == 1 and layer != last:
                        continue
                    for previous in states:
                        if previous == last and layer != last:
                            continue
                        if max(abs(x - y) for x in heights for y in previous_heights[previous]) > step + 1e-6:
                            continue
                        if previous != layer:
                            if previous not in possible:
                                continue
                            if max(abs(x - y) for x in heights for y in possible[previous]) > step + 1e-6:
                                continue
                        next_states.add(layer)
                if not next_states:
                    raise TomogramError('invalid_curve', 'Optimized curve crosses unsupported/blocked/discontinuous surface',
                                        segment=segment, fraction=t, reasons=reasons,
                                        layers=[int(first), int(last)], cells=[list(cell) for cell in sorted(cells)])
                states, previous_heights = next_states, {layer: possible[layer] for layer in next_states}
                boundary_count += int(len(cells) > 1)
                for layer in next_states:
                    for cell in cells:
                        key = (layer, *cell)
                        touched.add(key)
                        if not self.ceiling_known[key] and not self.open_sky_verified[key]:
                            unknown_ceiling.add(key)
            changes += int(first != last)
        return {'checked_layer_cells': len(touched), 'checked_cells': len(touched),
                'boundary_samples': boundary_count, 'layer_transitions': changes,
                'unobserved_ceiling_cells': len(unknown_ceiling),
                'unknown_ceiling_policy': self.unknown_ceiling_policy,
                'minimum_measured_headroom_m': min((float(self.headroom[key]) for key in touched
                                                   if self.ceiling_known[key]), default=None),
                'cost_threshold': self.COST_THRESHOLD, 'extra_erosion_cells': 0}

    def native_payload(self):
        data = self.data.copy()
        # Apply the same invalid-support/clearance mask consumed by selectors
        # and displays. This is no spatial erosion and does not invent geometry.
        # Operate only on the owned copy. Avoid boolean gather/scatter and a
        # second full gradient copy for each masked floor/stair native map.
        np.fmax(data[0], 50., out=data[0], where=~self.valid)
        np.nan_to_num(data[1:3], copy=False, nan=0.)
        return {'data': data, 'resolution': self.resolution, 'center': self.center.copy(),
                'slice_h0': self.slice_h0, 'slice_dh': self.slice_dh,
                'selected_source_layers': self.source_layers.copy()}
