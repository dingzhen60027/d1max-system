"""Passive goal editing over the same measured surfaces as live PCT.

This module owns a draft, never a navigation task.  XY is never repaired by
nearest-cell search.  TomogramSelection supplies ground following and native
surface diagnostics; the live global admission functions remain the final
authority for floor identity and the 15 cm height contract.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

from d1max_pct_planner.paths import expand_tree
from d1max_pct_planner.preview_core import finite_xyz
from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_planner.tomogram_selection import TomogramSelection

from .live_global_contract import (
    FLOOR_NAMES, choose_floor_surface, floor_from_original_ground_z, floor_from_planning_z,
)
from .pointcloud_helpers.ground_path_bridge import GroundPathBridge, ParametricGroundPathBridge
from .source_identity import SourceIdentityBridge
from .source_route import digest_file
from .navigation_surfaces import NavigationSurfaceResolver, NavigationSurfaceError


HEIGHT_TOLERANCE_M = .15  # Identical to the live global goal admission.
REASON_LABELS = {
    'traversable': '目标可规划',
    'outside_map': '目标在地图外',
    'unobserved_ground': '此处缺少地面',
    'no_traversable_surface': '此处没有可通行地面',
    'no_ground_on_selected_floor': '当前楼层此处没有地面',
    'unknown_cost': '此处代价未知',
    'pct_cost_blocked': '此处不可通行',
    'insufficient_headroom': '此处净空不足',
    'unobserved_ceiling': '此处净空未确认',
    'height_off_surface': '目标未贴地',
    'ground_step': '此处地面不连续',
    'ambiguous_surface': '请选择楼层或地面高度',
    'surface_switch_requires_selection': '请选择下一段地面',
    'no_traversable_measured_surface_at_exact_xy_and_floor': '当前楼层此处不可规划',
    'multiple_distinct_surfaces_on_declared_floor': '此处存在多个地面高度',
    'source_3d_goal_z_does_not_uniquely_match_observed_floor': '目标未落在原图地面',
    'goal_in_stair_transition_requires_floor_target': '请选择楼层内的目标',
    'selected_floor_does_not_match_source_support': '目标不在所选楼层',
}


class LiveGoalSelection:
    """One editable ground goal with explicit floor / native slice controls."""

    def __init__(self, tomogram, bridge, floor_z_ranges, *, mode='ground', route_settings=None):
        self.conditioned = bridge.source_frame != bridge.planning_frame
        if self.conditioned and not isinstance(bridge, (GroundPathBridge, ParametricGroundPathBridge)):
            raise ValueError('goal_editor_requires_audited_ground_bridge')
        self.tomogram, self.bridge = tomogram, bridge
        # Only the editor's native surface query lives in planning coordinates.
        # Its public draft remains source-ground XYZ; no body pose/scan is warped.
        self._source_point, self._source_error = None, None
        self.floor_z_ranges = dict(floor_z_ranges)
        ranges = {floor: self.floor_z_ranges[name] for floor, name in FLOOR_NAMES.items()
                  if floor in bridge.floors and name in self.floor_z_ranges}
        if not ranges:
            raise ValueError('goal_editor_has_no_declared_source_floor')
        self.selection = TomogramSelection(
            tomogram, height_tolerance=HEIGHT_TOLERANCE_M, mode=mode,
            anchor_z=float(bridge.floors[next(iter(ranges))].reference_z_m),
            max_follow_height_change=tomogram.max_ground_step_m,
            floor_ranges=ranges)
        self._explicit_floor = False
        self.region_constraint = None
        self.surface_resolver = (NavigationSurfaceResolver(tomogram, bridge, route_settings)
                                 if route_settings and 'stair_roi' in route_settings else None)
        self.binding_status = 'provided_measured_map'

    @classmethod
    def from_session(cls, session, *, mode='ground'):
        """Load once from an already verified formal session.

        Session preparation/verification proves the PCD records and indices.
        The editor rechecks the small manifest, support archive, route and
        actual tomogram against that sealed identity. The source-identity path
        avoids large PCD copies; a conditioned map reconstructs its existing
        audited height field once and reuses it for goals and initial seeds.
        """
        hashes = session.get('input_hashes', {})

        def pinned(path, *, read=True):
            path = Path(path).resolve(strict=True)
            expected = hashes.get(str(path))
            if not isinstance(expected, str) or len(expected) != 64:
                raise ValueError('goal_editor_input_not_in_verified_session:' + path.name)
            if read and digest_file(path) != expected:
                raise ValueError('goal_editor_session_input_changed:' + path.name)
            return path, expected

        manifest_path, manifest_hash = pinned(session['planning_manifest'])
        route_path, _ = pinned(session['crossfloor_route_config'])
        tomogram_path, tomogram_hash = pinned(session['tomogram_npz'], read=False)
        source_path, source_hash = pinned(session['map_pcd'], read=False)
        manifest = json.loads(manifest_path.read_text())
        if manifest.get('geometry_operation') != 'source_identity':
            return cls._from_conditioned_session(session, manifest_path, manifest_hash,
                manifest, route_path, tomogram_path, tomogram_hash, source_path,
                source_hash, pinned, mode=mode)
        if (manifest.get('schema') != 'd1max.source_identity_floor/v1'
                or manifest.get('status') != 'complete'
                or manifest.get('geometry_operation') != 'source_identity'
                or manifest.get('xyz_modified') is not False
                or manifest.get('source_coordinate_error_m') != 0.
                or manifest.get('frame_id') != session['frame_id']
                or Path(manifest['source_path']).resolve() != source_path
                or manifest.get('source_sha256') != source_hash):
            raise ValueError('goal_editor_source_identity_binding_invalid')
        support_path, support_hash = pinned(manifest_path.parent / manifest['source_indices_file'])
        output_path, output_hash = pinned(manifest_path.parent / manifest['output_file'], read=False)
        if (support_hash != manifest['source_indices_sha256']
                or output_hash != manifest['output_sha256']):
            raise ValueError('goal_editor_support_or_output_identity_changed')
        floor = manifest['floor_id']
        with np.load(support_path, allow_pickle=False) as archive:
            support = archive[floor + '_support_xyz'].copy()
        bridge = SourceIdentityBridge(support, manifest['forbidden_stair_xy'], floor_id=floor)
        raw = expand_tree(yaml.safe_load(route_path.read_text()))
        tomogram = TomogramMap(tomogram_path,
            minimum_headroom_m=raw.get('minimum_headroom_m'),
            unknown_ceiling_policy=raw.get('unknown_ceiling_policy', 'allow_unobserved'),
            max_ground_step_m=raw['limits']['max_ground_step_m'])
        provenance = tomogram.provenance
        if (tomogram.sha256 != tomogram_hash
                or raw.get('schema') != 'd1max.source_identity_route/v1'
                or raw.get('floor_id') != floor or raw.get('stairs_enabled') is not False
                or raw.get('frame_id') != session['frame_id']
                or Path(raw['source_pcd']).resolve() != output_path
                or Path(raw['tomogram_path']).resolve() != tomogram_path
                or provenance.get('geometry_operation') != 'source_identity'
                or provenance.get('source_processing_manifest_sha256') != manifest_hash
                or provenance.get('original_source_sha256') != source_hash
                or provenance.get('source_sha256') != output_hash
                or provenance.get('frame_id') != session['frame_id']
                or Path(provenance.get('source_pcd', '')).resolve() != output_path
                or Path(provenance.get('source_processing_manifest', '')).resolve() != manifest_path):
            raise ValueError('goal_editor_tomogram_route_binding_invalid')
        result = cls(tomogram, bridge, raw['floor_z_ranges'], mode=mode, route_settings=raw)
        result.binding_status = 'verified_session_source_identity'
        return result

    @classmethod
    def _from_conditioned_session(cls, session, manifest_path, manifest_hash, manifest,
                                  route_path, tomogram_path, tomogram_hash, source_path,
                                  source_hash, pinned, *, mode):
        """Reuse the historical, hash-bound ground bridge, never a new TF."""
        operation = 'independent_floor_conditioning_visibility_filter_and_measured_return_restore'
        if (manifest.get('status') != 'complete' or manifest.get('planning_only') is not True
                or manifest.get('processing_schema') != 'd1max.crossfloor_planning/v1'
                or manifest.get('geometry_operation') != operation
                or manifest.get('frame_id') != 'd1max_multifloor_planning'
                or session['frame_id'] != 'd1max_loc_map'
                or Path(manifest['source_path']).resolve() != source_path
                or manifest.get('source_sha256') != source_hash):
            raise ValueError('goal_editor_conditioned_binding_invalid')
        # Exactly the same runtime dependency contract as formal preparation.
        # Historical raw scans/rebuild YAML hashes remain lineage references;
        # those files are not read by this ground bridge or the runtime editor.
        from .navigation_map_package import validate_package
        package = validate_package(manifest_path.parent)
        dependencies = package['dependency_files']
        if session.get('map_runtime_dependencies') != dependencies:
            raise ValueError('goal_editor_conditioned_runtime_dependencies_changed')
        for dependency, expected in dependencies.items():
            _, actual = pinned(dependency, read=False)
            if actual != expected:
                raise ValueError('goal_editor_conditioned_dependency_changed')
        bridge, tomogram, raw = package['bridge'], package['tomogram'], package['route']
        if (tomogram.sha256 != tomogram_hash
                or dependencies[str(manifest_path)] != manifest_hash
                or package['tomogram_path'] != tomogram_path
                or route_path != manifest_path.parent / 'route.yaml'):
            raise ValueError('goal_editor_tomogram_route_binding_invalid')
        result = cls(tomogram, bridge, raw['floor_z_ranges'], mode=mode, route_settings=raw)
        result.binding_status = 'verified_session_conditioned_ground_bridge'
        return result

    @property
    def mode(self):
        return self.selection.mode

    @property
    def available_floors(self):
        return tuple(self.selection.floor_ranges)

    @property
    def floor_choices(self):
        labels = {'floor1': '一楼', 'floor2': '二楼'}
        choices = [dict(floor_id=floor, label=labels.get(floor, floor),
                     z_range=list(self.selection.floor_ranges[floor]))
                for floor in self.available_floors]
        if self.conditioned:
            for choice in choices:
                choice['z_range_frame_id'] = self.bridge.planning_frame
        return choices

    @property
    def region_choices(self):
        """Goal-only choices; initial localization still uses floor_choices."""
        choices = [dict(choice, region_id=choice['floor_id']) for choice in self.floor_choices]
        if self.surface_resolver is not None:
            choices.append(dict(region_id='stairs', floor_id='stairs', label='楼梯 / 平台'))
        return choices

    @property
    def xyz(self):
        point = self._source_point if self.conditioned or self.surface_resolver is not None else self.selection.points['goal']
        return None if point is None else tuple(map(float, point))

    def set_mode(self, mode):
        self.selection.set_mode(mode)
        if mode == 'free' and not self._explicit_floor:
            # A free Z edit is an explicit height selection.  Do not carry an
            # automatically remembered ground floor into that free edit.
            self.selection.set_active_floor(None)
        return self.feedback() if self.xyz is None else self.edit(self.xyz)

    def set_floor(self, floor_id):
        if self.surface_resolver is not None:
            if floor_id not in (None, 'auto', *self.available_floors, 'stairs', 'stair_lower', 'stair_upper', 'landing'):
                raise ValueError('Unknown navigation region')
            self.region_constraint = None if floor_id in (None, 'auto') else floor_id
            self.selection.set_active_floor(floor_id if floor_id in self.available_floors else None)
            self._explicit_floor = self.region_constraint is not None
            if self.xyz is None:
                return self.feedback()
            return self.edit(self.xyz) if self.region_constraint is None else self.snap_ground()
        self.selection.set_active_floor(floor_id)
        self._explicit_floor = floor_id is not None
        # Selecting a physical floor is an explicit draft edit.  It may change
        # Z at this XY, even in free mode, but does not commit a new task.
        return self.feedback() if self.xyz is None or floor_id is None else self.snap_ground()

    def set_layer(self, layer_id):
        self.selection.set_active_layer(layer_id)
        self.region_constraint = None
        self._explicit_floor = False
        if self.surface_resolver is not None and self.xyz is not None and layer_id < 0:
            return self.edit(self.xyz)
        return self.feedback() if self.xyz is None or layer_id < 0 else self.snap_ground()

    def edit(self, xyz):
        requested = finite_xyz(xyz)
        if self.surface_resolver is not None:
            self._edit_navigation(requested)
        elif self.conditioned:
            self._edit_conditioned(requested)
        else:
            self.selection.set_point('goal', requested)
        result = self.feedback()
        # Ground dragging retains the physical floor, not one tomographic
        # slice.  Equivalent slices on the same floor remain usable.
        if (self.surface_resolver is None and result['valid'] and self.mode == 'ground'
                and self.selection.active_layer < 0
                and self.selection.active_floor is None):
            self.selection.set_active_floor(result['floor_id'])
            result = self.feedback()
        return result

    def snap_ground(self):
        if self.surface_resolver is not None:
            if self.xyz is None:
                raise ValueError('Select goal first')
            self._edit_navigation(finite_xyz(self.xyz), snap=True)
            return self.feedback()
        if self.conditioned:
            if self.xyz is None:
                raise ValueError('Select goal first')
            self._edit_conditioned(finite_xyz(self.xyz), snap=True)
            return self.feedback()
        try:
            self.selection.snap_ground('goal')
        except ValueError as exc:
            # Keep failure evidence with the draft until the next successful
            # edit.  A later status tick must not resurrect the old valid slice
            # after an explicit snap/layer selection has just failed.
            self.selection.edit_errors['goal'] = exc
            result = self.feedback()
            code = getattr(exc, 'code', str(exc))
            result.update(valid=False, reason=REASON_LABELS.get(code, '目标不可规划'),
                          reason_code=code, detail=str(exc))
            return result
        return self.feedback()

    def _edit_navigation(self, requested, *, snap=False):
        previous = self._source_point
        hint = requested.copy()
        if self.mode == 'ground' and previous is not None and not snap:
            hint[2] = previous[2]
        self._source_point, self._source_error = hint.copy(), None
        try:
            result = self.surface_resolver.resolve_goal(hint,
                snap=snap or self.mode == 'ground', region=self.region_constraint,
                layer_id=self.selection.active_layer if self.selection.active_layer >= 0 else None)
            if (self.mode == 'ground' and previous is not None and not snap
                    and abs(result['source_xyz'][2]-hint[2]) > self.tomogram.max_ground_step_m+1e-8):
                raise NavigationSurfaceError('surface_switch_requires_selection')
            self._source_point = np.asarray(result['source_xyz'], float)
            self.selection._store('goal', result['planning_xyz'], result['layer_id'])
        except ValueError as exc:
            self._source_error = exc

    def _navigation_feedback(self):
        point = self._source_point
        result = dict(valid=False, reason_code='invalid_endpoint', detail='',
            xyz=None if point is None else point.tolist(), ground_z=None, height_error_m=None,
            headroom_m=None, cost=None, ceiling_state=None, floor_id=None, region_id=None,
            layer_id=self.selection.layers['goal'], equivalent_layers=[],
            selection_mode='ground' if self.mode == 'ground' else 'free_xyz', mode=self.mode,
            frame_id=self.bridge.source_frame, planning_frame_id=self.bridge.planning_frame,
            floor_constraint=self.selection.active_floor, region_constraint=self.region_constraint,
            layer_constraint=self.selection.active_layer, binding_status=self.binding_status,
            projection_kind='non_rigid_ground_only_not_tf', motion_enabled=False)
        try:
            if self._source_error is not None:
                raise self._source_error
            if point is None:
                raise ValueError('Select goal first')
            result.update(self.surface_resolver.resolve_goal(point, region=self.region_constraint,
                layer_id=self.selection.active_layer if self.selection.active_layer >= 0 else None))
            result['xyz'] = point.tolist()
        except ValueError as exc:
            result.update(valid=False, reason_code=getattr(exc, 'code', str(exc)), detail=str(exc))
        result['layer'] = result['layer_id']
        result['reason'] = REASON_LABELS.get(result['reason_code'],
            '请设置目标' if point is None else '目标不可规划')
        return result

    def _edit_conditioned(self, requested, *, snap=False):
        """Query a labelled ground surface; retain invalid source XYZ for repair."""
        previous = self._source_point
        self._source_point, self._source_error = requested.copy(), None
        if self.mode == 'ground' and previous is not None:
            self._source_point[2] = previous[2]
        try:
            floor = self.selection.active_floor
            if self.selection.active_layer >= 0:
                surface = next(info for info in self.tomogram.sample_surfaces(
                    requested[:2], traversable_only=False, deduplicate=False)
                    if info['layer_id'] == self.selection.active_layer)
                floor = floor_from_planning_z(surface['ground_z'], self.floor_z_ranges)
            if floor is None:
                floor = floor_from_original_ground_z(self.bridge, requested)
            field, _ = self.bridge.floors[floor].query(requested[None, :2], self.bridge.limits)
            native = requested.copy()
            native[2] += self.bridge.floors[floor].reference_z_m - float(field[0])
            if snap or self.mode == 'ground' and self.selection.active_floor is not None:
                native[2] = self.bridge.floors[floor].reference_z_m
            if snap:
                surface = self.selection._select(native, 'goal', snap=True)
                self.selection._store('goal', surface['xyz'], surface['layer_id'])
            else:
                self.selection.set_point('goal', native)
            self.selection.validate('goal')
            point = self.selection.points['goal']
            source = self.bridge.to_localization_ground(point[None, :], [floor]).xyz[0]
            self._source_point = source if snap or self.mode == 'ground' else requested.copy()
        except (ValueError, StopIteration) as exc:
            self._source_error = exc if isinstance(exc, ValueError) else ValueError('no_ground_on_selected_floor')

    def _admit(self, xyz):
        floor = floor_from_original_ground_z(self.bridge, xyz)
        constraint = self.selection.active_floor
        if constraint is not None and floor != constraint:
            raise ValueError('selected_floor_does_not_match_source_support')
        surface = choose_floor_surface(self.tomogram, xyz[:2], floor,
            self.floor_z_ranges, hint_z=(self.bridge.floors[floor].reference_z_m
                if self.conditioned else float(xyz[2])),
            maximum_hint_error_m=HEIGHT_TOLERANCE_M)
        return floor, surface

    def feedback(self):
        """JSON-safe diagnostics.  A valid draft is not execution permission."""
        if self.surface_resolver is not None:
            return self._navigation_feedback()
        result = dict(self.selection.validation('goal'))
        point = self._source_point if self.conditioned else self.selection.points['goal']
        if self.conditioned:
            # Native cost/headroom remain explicitly planning-map evidence.
            # Never label its conditioned Z as original-map ground geometry.
            result.update(ground_z=None, height_error_m=None,
                projection_kind='non_rigid_ground_only_not_tf',
                planning_frame_id=self.bridge.planning_frame)
            if self._source_error is not None:
                code = getattr(self._source_error, 'code', str(self._source_error))
                result.update(valid=False, reason_code=code, detail=str(self._source_error))
        result.update(xyz=None if point is None else point.tolist(),
            selection_mode='ground' if self.mode == 'ground' else 'free_xyz',
            mode=self.mode, frame_id=self.bridge.source_frame,
            floor_constraint=self.selection.active_floor,
            layer_constraint=self.selection.active_layer,
            binding_status=self.binding_status, motion_enabled=False)
        for key in ('floor_id', 'ground_z', 'height_error_m', 'headroom_m', 'cost', 'ceiling_state'):
            result.setdefault(key, None)
        result.setdefault('detail', '')
        result['equivalent_layers'] = []
        if result['valid']:
            try:
                floor, surface = self._admit(point)
                ground_z = surface['ground_z']
                if self.conditioned:
                    ground_z = float(self.bridge.to_localization_ground(
                        np.asarray(surface['xyz'])[None, :], [floor]).xyz[0, 2])
                    result['planning_ground_z'] = surface['ground_z']
                # Use PCT's exact admitted surface evidence, not whichever
                # visually close raw slice an editor diagnostic preferred.
                result.update(floor_id=floor, ground_z=ground_z,
                    height_error_m=float(point[2] - ground_z),
                    headroom_m=surface.get('headroom_m'), cost=surface.get('cost'),
                    ceiling_state=surface.get('ceiling_state'),
                    equivalent_layers=surface.get('equivalent_layers', [surface['layer_id']]))
            except ValueError as exc:
                result.update(valid=False, reason=getattr(exc, 'code', str(exc)),
                    reason_code=getattr(exc, 'code', str(exc)), detail=str(exc))
        result['layer'] = result.get('layer_id')
        result['reason'] = REASON_LABELS.get(result['reason_code'],
            '请设置目标' if point is None else '目标不可规划')
        return result

    def surfaces(self, xy=None):
        """Measured surfaces under exact XY, including blocked choices."""
        if xy is None:
            if self.xyz is None:
                return []
            xy = self.xyz[:2]
        if self.surface_resolver is not None:
            return [dict(item, xyz=item['source_xyz'], layer=item['layer_id'])
                    for item in self.surface_resolver.surfaces(xy)]
        result = []
        for surface in self.tomogram.sample_surfaces(xy, traversable_only=False, deduplicate=True):
            if surface.get('ground_z') is None:
                continue
            entry = dict(surface)
            try:
                if self.conditioned:
                    floor = floor_from_planning_z(surface['ground_z'], self.floor_z_ranges)
                    source = self.bridge.to_localization_ground(
                        np.asarray(surface['xyz'])[None, :], [floor]).xyz[0]
                    entry.update(xyz=source.tolist(), ground_z=float(source[2]),
                        planning_ground_z=surface['ground_z'],
                        frame_id=self.bridge.source_frame,
                        projection_kind='non_rigid_ground_only_not_tf')
                else:
                    source = surface['xyz']
                entry['floor_id'] = floor_from_original_ground_z(self.bridge, source)
                if entry['valid']:
                    choose_floor_surface(self.tomogram, surface['xyz'][:2], entry['floor_id'],
                        self.floor_z_ranges, hint_z=surface['ground_z'])
            except ValueError as exc:
                if self.conditioned and 'projection_kind' not in entry:
                    continue  # No source-ground coordinate claim outside the bridge domain.
                entry.setdefault('floor_id', None)
                entry['valid'] = False
                entry['reason_code'] = getattr(exc, 'code', str(exc))
            entry['layer'] = entry['layer_id']
            result.append(entry)
        return result
