"""Shared ground-surface resolution for goal editing and route admission.

Only ground points are mapped. Stairs use source-owned correspondence or the
explicit model facade; model geometry remains distinct from observations. No
robot pose, sensor point, TF, map cost or clearance is changed by this module.
"""
from __future__ import annotations

import numpy as np

from d1max_pct_planner.crossfloor_partial import endpoint_region
from .live_global_contract import FLOOR_NAMES, finite_xyz, choose_floor_surface


class NavigationSurfaceError(ValueError):
    def __init__(self, code, detail=None):
        self.code = code
        super().__init__(detail or code)


class NavigationSurfaceResolver:
    def __init__(self, tomogram, bridge, route_settings):
        self.tomogram, self.bridge = tomogram, bridge
        self.settings = route_settings
        self.floor_ranges = dict(route_settings['floor_z_ranges'])
        self.height_tolerance_m = .15
        self.stair_roi = None
        self.landing = None
        if 'stair_roi' in route_settings and 'anchors' in route_settings:
            raw = route_settings['stair_roi']
            self.stair_roi = np.asarray([raw['min'], raw['max']] if isinstance(raw, dict) else raw, float)
            self.landing = finite_xyz(route_settings['anchors']['landing']['xyz'])
            if self.stair_roi.shape != (2, 3) or not np.isfinite(self.stair_roi).all() or np.any(self.stair_roi[0] >= self.stair_roi[1]):
                raise NavigationSurfaceError('invalid_stair_roi')
            # Native route classification consumes the normalized ROI. The
            # editor also accepts a sealed route YAML's min/max representation.
            self.settings = {**route_settings, 'stair_roi': self.stair_roi}

    @property
    def stairs_enabled(self):
        return self.stair_roi is not None

    def _protected(self, point):
        return any(np.all(point >= region['min']) and np.all(point <= region['max'])
                   for region in self.bridge.protected_regions)

    def _region(self, point):
        if self.stairs_enabled:
            native_region = endpoint_region(point, self.settings)
            return {'lower': 'floor1', 'upper': 'floor2'}.get(native_region, native_region)
        floor = [key for key, name in FLOOR_NAMES.items() if key in self.bridge.floors
                 and name in self.floor_ranges and self.floor_ranges[name][0] <= point[2] <= self.floor_ranges[name][1]]
        if len(floor) == 1:
            return floor[0]
        raise NavigationSurfaceError('surface_outside_declared_navigation_regions')

    def _matches(self, surface, requested):
        if requested in (None, 'auto'):
            return True
        if requested == 'stairs':
            # This is an editing-area constraint, not the native route token.
            # Entry/exit floor-level support retains its floor ownership;
            # first/last elevated treads retain stair ownership even where
            # the native floor-height search bands overlap them.
            point = np.asarray(surface['planning_xyz'])
            return (self.stairs_enabled and np.all(point >= self.stair_roi[0]-1e-6)
                    and np.all(point <= self.stair_roi[1]+1e-6))
        return surface['region_id'] == requested

    def _surface_record(self, info):
        planning = finite_xyz(info['xyz'])
        region = self._region(planning)
        model_surface = (self.bridge.model_surface_at(planning)
                         if hasattr(self.bridge, 'model_surface_at') else None)
        if self.bridge.source_frame == self.bridge.planning_frame:
            source = planning.copy()
        elif model_surface is not None:
            # The same bounded source-frame model is used by route restoration
            # and body-ground lookup. Never apply a nearest-return inverse or
            # flat-floor field to this explicitly modeled ground a second time.
            source = self.bridge.to_localization_ground(planning[None, :], [region]).xyz[0]
        elif self._protected(planning):
            if not self.stairs_enabled:
                raise NavigationSurfaceError('goal_in_stair_transition_requires_floor_target')
            # Older maps preserved stair XYZ verbatim. A regularized package
            # instead carries an explicit ground-only relation. The goal
            # editor and current-body support query must use the same inverse
            # as route construction and displayed traversability, not label a
            # conditioned planning height as an original observation.
            source = (self.bridge.to_localization_ground(planning[None, :], ['stairs']).xyz[0]
                      if getattr(self.bridge, 'stair_mapping', None) is not None
                      else planning.copy())
        else:
            if region == 'landing':
                raise NavigationSurfaceError('stair_source_mapping_unavailable')
            source = self.bridge.to_localization_ground(planning[None, :], [region]).xyz[0]
        floor = region if region in FLOOR_NAMES else None
        if floor is not None and not self._protected(planning) and model_surface is None:
            # A protected transition keeps measured XYZ even at floor-level
            # entry/exit support. The conditioned floor
            # height field is not authoritative inside this non-warped area.
            field, _ = self.bridge.floors[floor].query(source[None, :2], self.bridge.limits)
            if abs(source[2]-field[0]) > self.height_tolerance_m:
                raise NavigationSurfaceError('source_3d_goal_z_does_not_uniquely_match_observed_floor')
        record = {**info, 'source_xyz': source.tolist(), 'planning_xyz': planning.tolist(),
                'region_id': region, 'floor_id': floor, 'ground_z': float(source[2]),
                'planning_ground_z': float(planning[2]), 'frame_id': self.bridge.source_frame,
                'planning_frame_id': self.bridge.planning_frame}
        if model_surface is not None:
            record['parametric_stair_model_surface_id'] = model_surface['surface_id']
            record['model_points_are_observations'] = False
        return record

    def surfaces(self, xy):
        """All mapped exact-XY candidates, including native blocked surfaces."""
        result = []
        for info in self.tomogram.sample_surfaces(xy, traversable_only=False, deduplicate=True):
            if info.get('ground_z') is None:
                continue
            try:
                result.append(self._surface_record(info))
            except ValueError:
                continue
        return result

    def resolve_goal(self, source_xyz, *, snap=False, region=None, frame_id=None, layer_id=None):
        point = finite_xyz(source_xyz)
        frame = frame_id or self.bridge.source_frame
        if frame not in (self.bridge.source_frame, self.bridge.planning_frame):
            raise NavigationSurfaceError('goal_frame_not_supported')
        if region not in (None, 'auto', 'stairs', 'floor1', 'floor2', 'stair_lower', 'stair_upper', 'landing'):
            raise NavigationSurfaceError('unknown_navigation_region')
        if not self.tomogram.contains(tuple(self.tomogram.index(point[:2]))):
            raise NavigationSurfaceError('outside_map', 'Selected XY is outside the tomogram')
        candidates = [item for item in self.surfaces(point[:2]) if self._matches(item, region)]
        if layer_id is not None:
            layer_id = self.tomogram.layer(layer_id)
            candidates = [item for item in candidates if layer_id in item.get('equivalent_layers', [item['layer_id']])]
        coordinate = 'source_xyz' if frame == self.bridge.source_frame else 'planning_xyz'
        ranked = sorted(candidates, key=lambda item: (abs(item[coordinate][2]-point[2]), not item['valid'], item['layer_id']))
        eligible = ranked if snap else [item for item in ranked if abs(item[coordinate][2]-point[2]) <= self.height_tolerance_m+1e-8]
        if not eligible:
            code = 'height_off_surface' if ranked else 'no_traversable_surface'
            raise NavigationSurfaceError(code, 'No measured surface at the requested XYZ/region; XY was not moved')
        chosen = eligible[0]
        # Preserve source-level ambiguity rather than silently choosing a roof
        # or another branch. An explicit native layer is an actual choice.
        if layer_id is None:
            for other in eligible[1:]:
                same_distance = abs(abs(other[coordinate][2]-point[2])-abs(chosen[coordinate][2]-point[2])) < 1e-7
                if same_distance and abs(other[coordinate][2]-chosen[coordinate][2]) > 1e-4:
                    raise NavigationSurfaceError('ambiguous_surface')
        # sample_surfaces(False) is diagnostic only: revalidate exact boundary
        # cells with the production selector, including cost and headroom.
        planning = np.asarray(chosen['planning_xyz'])
        if (chosen['floor_id'] is not None and not self._protected(planning)
                and 'parametric_stair_model_surface_id' not in chosen):
            choose_floor_surface(self.tomogram, point[:2], chosen['floor_id'], self.floor_ranges,
                                 hint_z=float(planning[2]), maximum_hint_error_m=self.height_tolerance_m)
        actual_layer = chosen['layer_id'] if layer_id is None else layer_id
        checked = self.tomogram.validate_endpoint(planning, layer_id=actual_layer,
                                                  height_tolerance_m=self.height_tolerance_m)
        result = {**chosen, **{key: checked[key] for key in ('cost', 'headroom_m', 'ceiling_state')},
                  'valid': True, 'reason_code': 'traversable', 'layer_id': int(actual_layer),
                  'height_error_m': 0. if snap else float(point[2]-chosen[coordinate][2])}
        if not snap:
            # Preserve a valid free-XYZ edit's offset in both coordinate forms.
            offset = result['height_error_m']
            result['source_xyz'] = [*chosen['source_xyz'][:2], chosen['source_xyz'][2]+offset]
            result['planning_xyz'] = [*chosen['planning_xyz'][:2], chosen['planning_xyz'][2]+offset]
        return result

    def resolve_body(self, body_xyz, height_interval_m):
        body = finite_xyz(body_xyz)
        interval = np.asarray(height_interval_m, float)
        if interval.shape != (2,) or not np.isfinite(interval).all() or not 0 <= interval[0] < interval[1]:
            raise NavigationSurfaceError('invalid_body_height_interval')
        candidates = []
        for item in self.surfaces(body[:2]):
            height = float(body[2]-item['source_xyz'][2])
            if interval[0] <= height <= interval[1]:
                try:
                    selected = self.resolve_goal(item['source_xyz'], region=item['region_id'], layer_id=item['layer_id'])
                except ValueError:
                    continue
                if not any(abs(old['ground_z']-selected['ground_z']) <= 1e-4 for old in candidates):
                    candidates.append({**selected, 'body_height_m': height})
        if len(candidates) != 1:
            raise NavigationSurfaceError('current_support_requires_unique_measured_surface')
        return candidates[0]
