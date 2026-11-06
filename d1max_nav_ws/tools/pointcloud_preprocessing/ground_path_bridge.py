"""Audited *ground-path* bridge between the original and conditioned map frames.

This is not a TF and must not be used to transform an IMU, lidar scan, or robot
body pose.  The planning map has a spatially varying Z correction and snapped
floor returns; an arbitrary 3-D point has no exact inverse. Ground-path samples
use either the observed correspondence or a separately hash-bound, explicit
stair geometry prior; the latter is never reported as measured support. A live
body pose can be used to *query* the corresponding ground surfaces, with an
explicit caller-supplied body-height gate, but is never itself warped.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence
import zipfile

import numpy as np
from scipy.spatial import cKDTree

from .flat_floor import _anchors, _config, _field
from .pcd_io import read_pcd


class GroundBridgeError(ValueError):
    """A requested path point is outside the proven coordinate relation."""


def _digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def _xyz(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3 or len(array) == 0 or not np.isfinite(array).all():
        raise GroundBridgeError(f'{name} must be a nonempty finite Nx3 array')
    return array


@dataclass(frozen=True)
class ProjectionLimits:
    max_ground_residual_m: float = 0.15
    max_conditioning_shift_m: float = 0.50
    max_boundary_jump_m: float = 0.12
    adjacent_xy_m: float = 0.25

    def __post_init__(self):
        for name in ('max_ground_residual_m', 'max_conditioning_shift_m',
                     'max_boundary_jump_m', 'adjacent_xy_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise GroundBridgeError(f'{name} must be finite and positive')


@dataclass
class FloorField:
    floor_id: str
    reference_z_m: float
    input_z_band: tuple[float, float]
    anchors: Mapping[str, np.ndarray]
    support_xy: np.ndarray
    config: Mapping[str, object]

    def __post_init__(self):
        self.config = _config(self.config)
        self.support_xy = np.asarray(self.support_xy, dtype=np.float64)
        if (self.support_xy.ndim != 2 or self.support_xy.shape[1] != 2
                or len(self.support_xy) == 0 or not np.isfinite(self.support_xy).all()):
            raise GroundBridgeError(f'{self.floor_id}: invalid observed support XY')
        checked = {}
        for key, width in (('xy', 2), ('z', None), ('slopes', 2)):
            arr = np.asarray(self.anchors[key], dtype=np.float64)
            if key == 'z':
                valid_shape = arr.ndim == 1
            else:
                valid_shape = arr.ndim == 2 and arr.shape[1] == width
            if not valid_shape or len(arr) == 0 or not np.isfinite(arr).all():
                raise GroundBridgeError(f'{self.floor_id}: invalid anchor {key}')
            checked[key] = arr
        if len({len(v) for v in checked.values()}) != 1:
            raise GroundBridgeError(f'{self.floor_id}: mismatched anchor rows')
        self.anchors = checked
        low, high = map(float, self.input_z_band)
        if (not np.isfinite([low, high, self.reference_z_m]).all() or low >= high):
            raise GroundBridgeError(f'{self.floor_id}: invalid Z bounds')
        self.input_z_band = (low, high)
        self._support_tree = cKDTree(self.support_xy)

    def query(self, xy: np.ndarray, limits: ProjectionLimits) -> tuple[np.ndarray, np.ndarray]:
        field = _field(xy, self.anchors, self.config)
        distance, _ = self._support_tree.query(xy, workers=self.config['workers'])
        allowed = (np.isfinite(field) & np.isfinite(distance)
                   & (distance <= self.config['conditioning_support_radius_m']))
        allowed &= np.abs(self.reference_z_m - field) <= limits.max_conditioning_shift_m
        if not allowed.all():
            bad = int(np.flatnonzero(~allowed)[0])
            raise GroundBridgeError(
                f'{self.floor_id}: point {bad} has no bounded observed height field '
                f'(support distance {distance[bad]:.3f} m)')
        return field, distance


@dataclass(frozen=True)
class BridgeResult:
    xyz: np.ndarray
    diagnostics: dict


def load_parametric_stair_model(manifest_path, manifest, *, checked=None):
    """One hash-bound model for goal selection, ground restoration and references.

    The optional asset is explicit source-frame geometry, not a set of measured
    returns. This initializer is shared by the ground bridge and package loader.
    """
    binding = manifest.get('parametric_stair_geometry')
    if binding is None:
        return None, '', {}
    sha = binding.get('model_sha256') if isinstance(binding, dict) else None
    if (not isinstance(binding, dict)
            or binding.get('schema') != 'd1max.parametric_stair_geometry_binding/v1'
            or binding.get('source_map_sha256') != manifest.get('source_sha256')
            or binding.get('planning_map_sha256') != manifest.get('output_sha256')
            or not isinstance(sha, str) or len(sha) != 64
            or any(c not in '0123456789abcdef' for c in sha)):
        raise GroundBridgeError('parametric_stair_model_binding_invalid')
    directory = Path(manifest_path).resolve(strict=True).parent
    value = binding.get('model_file')
    if not isinstance(value, str) or not value:
        raise GroundBridgeError('parametric_stair_model_file_invalid')
    relative = Path(value)
    if relative.is_absolute() or '..' in relative.parts:
        raise GroundBridgeError('parametric_stair_model_file_escapes_package')
    path = (directory / relative).resolve(strict=True)
    if not path.is_relative_to(directory) or not path.is_file():
        raise GroundBridgeError('parametric_stair_model_file_escapes_package')
    if path.stat().st_size > 8*1024*1024:
        raise GroundBridgeError('parametric_stair_model_budget')
    raw = path.read_bytes()
    if len(raw) > 8*1024*1024:
        raise GroundBridgeError('parametric_stair_model_budget')
    actual = hashlib.sha256(raw).hexdigest()
    if actual != sha:
        raise GroundBridgeError('parametric_stair_model_changed')
    from d1max_pct_planner.stair_model_geometry import ParametricStairModel
    model = ParametricStairModel.from_dict(json.loads(raw))
    if (model.frame_id != 'd1max_loc_map'
            or model.record['source_map_sha256'] != binding['source_map_sha256']
            or model.record['planning_map_sha256'] != binding['planning_map_sha256']):
        raise GroundBridgeError('parametric_stair_model_identity_mismatch')
    if checked is not None:
        checked(path, actual)
    return model, actual, {str(path): actual}


class ParametricGroundPathBridge:
    """Ground-only model facade. Outside the explicit XYZ domain, delegate unchanged.

    Source and planning models describe the same ground coordinates. Do not
    send their points through the legacy nearest-return inverse a second time.
    Neither this facade nor its diagnostics promote model points to observations.
    """
    def __init__(self, original, model, model_sha256):
        if isinstance(original, ParametricGroundPathBridge):
            if original.stair_model_sha256 != model_sha256:
                raise GroundBridgeError('parametric_stair_bridge_identity_mismatch')
            original = original.original
        if (model.frame_id != original.source_frame or len(model_sha256) != 64
                or any(c not in '0123456789abcdef' for c in model_sha256)):
            raise GroundBridgeError('parametric_stair_bridge_identity_mismatch')
        self.original = original
        self.stair_model, self.stair_model_sha256 = model, model_sha256

    def __getattr__(self, name):
        return getattr(self.original, name)

    def model_surface_at(self, point):
        point = np.asarray(point, dtype=float)
        return self.stair_model.surface_at(point[:2], height_hint=float(point[2]))

    def to_localization_ground(self, planning_xyz, floor_ids):
        return self._map(_xyz(planning_xyz, 'planning_xyz'), floor_ids, inverse=True)

    def to_planning_ground(self, original_xyz, floor_ids):
        return self._map(_xyz(original_xyz, 'original_xyz'), floor_ids, inverse=False)

    def _map(self, points, labels, *, inverse):
        labels = tuple(labels)
        if len(labels) != len(points):
            raise GroundBridgeError('One explicit floor/transition label is required per point')
        method = self.original.to_localization_ground if inverse else self.original.to_planning_ground
        surfaces = [self.model_surface_at(p) for p in points]
        modeled = np.asarray([s is not None for s in surfaces])
        if not modeled.any():
            return method(points, labels)
        xyz = points.copy()
        for i in np.flatnonzero(modeled):
            xyz[i, 2] = self.stair_model.ground_at(surfaces[i], points[i, :2])
        original_diagnostics = []
        outside = np.flatnonzero(~modeled)
        # Preserve old adjacency checks on each actual contiguous floor run.
        for ids in np.split(outside, np.flatnonzero(np.diff(outside) > 1)+1):
            if len(ids):
                converted = method(points[ids], [labels[i] for i in ids])
                xyz[ids] = converted.xyz
                original_diagnostics.append(converted.diagnostics)
        return BridgeResult(xyz, dict(
            source_frame=self.planning_frame if inverse else self.source_frame,
            target_frame=self.source_frame if inverse else self.planning_frame,
            ground_path_only=True, certified_geometric_error_bound_m=None,
            parametric_stair_model=dict(schema=self.stair_model.record['schema'],
                model_sha256=self.stair_model_sha256, model_points_are_observations=False,
                modeled_indices=np.flatnonzero(modeled).tolist(),
                surface_ids=[s['surface_id'] if s is not None else '' for s in surfaces],
                ground_to_body_height_applications=0),
            original_ground_bridge=original_diagnostics))

    def project_live_pose_to_ground(self, original_body_xyz, floor_id, *, body_height_interval_m):
        body = _xyz(np.asarray(original_body_xyz, float).reshape(1, 3), 'original_body_xyz')[0]
        lo, hi = map(float, body_height_interval_m)
        if not np.isfinite([lo, hi]).all() or lo <= 0 or hi <= lo or hi > 2.:
            raise GroundBridgeError('Explicit finite body-height interval within (0, 2.0] m required')
        candidates = []
        for surface in self.stair_model.surfaces:
            if not self.stair_model.contains(surface, body[:2]):
                continue
            z = float(self.stair_model.ground_at(surface, body[:2]))
            if lo <= body[2]-z <= hi and not any(abs(z-old) <= 1e-6 for old in candidates):
                candidates.append(z)
        if not candidates:
            return self.original.project_live_pose_to_ground(original_body_xyz, floor_id,
                body_height_interval_m=body_height_interval_m)
        if len(candidates) != 1:
            raise GroundBridgeError('Live pose requires a unique model ground surface')
        ground = np.array([body[0], body[1], candidates[0]])
        return BridgeResult(ground, dict(source_ground_xyz=ground.tolist(),
            body_height_m=float(body[2]-ground[2]), floor_id=floor_id,
            source_frame=self.source_frame, planning_frame=self.planning_frame,
            ground_only=True, exact_inverse=False, model_points_are_observations=False,
            parametric_stair_model_sha256=self.stair_model_sha256,
            certified_geometric_error_bound_m=None))


class StairGroundMapping:
    """Hashed, per-return stair correspondence; not synthetic support or a TF.

    A regularized planning surface may differ from the measured surface by
    more than half a PCT cell. Keep those two coordinates together instead of
    widening the measured-surface classifier. All trees are built once here;
    Runtime ground projection interpolates a bounded local correspondence
    field in XYZ, without mixing flight/platform ownership. The measured
    support rows themselves are never interpolated.
    """
    SCHEMA = 'd1max.stair_regularization.v1'
    MAX_ROWS = 1_000_000
    MAX_ARCHIVE_BYTES = 128 * 1024 * 1024

    @classmethod
    def from_artifact(cls, descriptor, *, base_dir, source_xyz, source_sha256,
                      output_path, output_sha256):
        if not isinstance(descriptor, dict) or descriptor.get('schema') != cls.SCHEMA:
            raise GroundBridgeError('stair_mapping_schema_invalid')
        for name in ('source_sha256', 'input_map_sha256', 'output_map_sha256', 'mapping_sha256'):
            value = descriptor.get(name)
            if (not isinstance(value, str) or len(value) != 64
                    or any(c not in '0123456789abcdef' for c in value)):
                raise GroundBridgeError('stair_mapping_hash_invalid')
        if (descriptor['source_sha256'] != source_sha256
                or descriptor['output_map_sha256'] != output_sha256):
            raise GroundBridgeError('stair_mapping_map_identity_mismatch')
        paths = {}
        for name in ('input_map', 'output_map', 'mapping'):
            value = descriptor.get(name + '_file')
            if not isinstance(value, str) or not value:
                raise GroundBridgeError('stair_mapping_file_missing')
            path = (Path(base_dir) / value).resolve(strict=True)
            if _digest(path) != descriptor[name + '_sha256']:
                raise GroundBridgeError('stair_mapping_file_hash_mismatch')
            paths[name] = path
        if paths['output_map'] != Path(output_path).resolve(strict=True):
            raise GroundBridgeError('stair_mapping_output_file_mismatch')
        try:
            with zipfile.ZipFile(paths['mapping']) as archive:
                if (sum(info.file_size for info in archive.infolist()) > cls.MAX_ARCHIVE_BYTES
                        or len(archive.infolist()) != 4):
                    raise GroundBridgeError('stair_mapping_archive_budget_exceeded')
            with np.load(paths['mapping'], allow_pickle=False) as evidence:
                if set(evidence.files) != {'source_indices', 'source_xyz', 'planning_xyz', 'surface_id'}:
                    raise GroundBridgeError('stair_mapping_arrays_invalid')
                arrays = {key: evidence[key] for key in evidence.files}
        except (OSError, ValueError, zipfile.BadZipFile) as exc:
            if isinstance(exc, GroundBridgeError):
                raise
            raise GroundBridgeError('stair_mapping_archive_invalid') from exc
        result = cls(descriptor, arrays, source_xyz=source_xyz)
        changed = np.any(result.source_xyz != result.planning_xyz, axis=1)
        if np.any(changed):
            # A signed pair of file hashes does not prove that the mapped
            # planning coordinates were actually written into that PCD.
            # Check only modified rows; old observed identity support can be
            # retained in this correspondence without reinserting it in PCD.
            try:
                output_cloud = read_pcd(paths['output_map'])
                if not output_cloud.finite_xyz_mask.all():
                    raise GroundBridgeError('stair_mapping_output_geometry_invalid')
                distance = cKDTree(output_cloud.xyz).query(
                    result.planning_xyz[changed], workers=1)[0]
                if np.any(distance > 2e-6):
                    raise GroundBridgeError('stair_mapping_changed_output_rows_missing')
            except (OSError, ValueError) as exc:
                if isinstance(exc, GroundBridgeError):
                    raise
                raise GroundBridgeError('stair_mapping_output_geometry_invalid') from exc
        result.dependency_files = {str(path): descriptor[name + '_sha256']
                                   for name, path in paths.items()}
        return result

    def __init__(self, descriptor, arrays, *, source_xyz):
        self.source_hash = descriptor['source_sha256']
        self.mapping_hash = descriptor['mapping_sha256']
        self.dependency_files = {}
        ids, original, planning, surfaces = (arrays[key] for key in
            ('source_indices', 'source_xyz', 'planning_xyz', 'surface_id'))
        if (ids.dtype != np.dtype('int64') or ids.ndim != 1
                or not 1 <= len(ids) <= self.MAX_ROWS
                or surfaces.dtype != np.dtype('int32') or surfaces.shape != ids.shape
                or original.dtype != np.dtype('float64') or planning.dtype != np.dtype('float64')
                or original.shape != (len(ids), 3) or planning.shape != original.shape
                or not np.isfinite(original).all() or not np.isfinite(planning).all()):
            raise GroundBridgeError('stair_mapping_array_contract_invalid')
        source = np.asarray(source_xyz)
        if (source.ndim != 2 or source.shape[1] != 3 or np.any(ids < 0)
                or np.any(ids >= len(source)) or len(np.unique(ids)) != len(ids)
                or not np.array_equal(original, source[ids])):
            raise GroundBridgeError('stair_mapping_source_rows_mismatch')
        definitions = descriptor.get('surfaces')
        if (not isinstance(definitions, list) or not 1 <= len(definitions) <= 4096
                or any(not isinstance(s, dict) or type(s.get('id')) is not int
                       or not isinstance(s.get('kind'), str) or not s['kind'] for s in definitions)):
            raise GroundBridgeError('stair_mapping_surface_definitions_invalid')
        surface_ids = [s['id'] for s in definitions]
        if len(set(surface_ids)) != len(surface_ids) or set(np.unique(surfaces)) != set(surface_ids):
            raise GroundBridgeError('stair_mapping_surface_ownership_invalid')
        displacement = descriptor.get('max_displacement_m')
        radius = descriptor.get('association_max_distance_m')
        if (type(displacement) not in (int, float) or not np.isfinite(displacement)
                or not 0 <= displacement <= .5 or type(radius) not in (int, float)
                or not np.isfinite(radius) or not .01 <= radius <= .3):
            raise GroundBridgeError('stair_mapping_limits_invalid')
        if (not np.array_equal(original[:, :2], planning[:, :2])
                or np.max(np.abs(planning[:, 2] - original[:, 2])) > displacement + 1e-9):
            raise GroundBridgeError('stair_mapping_displacement_invalid')
        shifts = planning[:, 2] - original[:, 2]
        for coordinates in (original, planning):
            unique, inverse_ids = np.unique(coordinates, axis=0, return_inverse=True)
            if len(unique) != len(coordinates):
                lower = np.full(len(unique), np.inf)
                upper = np.full(len(unique), -np.inf)
                np.minimum.at(lower, inverse_ids, shifts)
                np.maximum.at(upper, inverse_ids, shifts)
                if np.any(upper-lower > 1e-9):
                    raise GroundBridgeError('stair_mapping_ambiguous_coordinates')
        self.source_indices = ids.copy()
        self.source_xyz, self.planning_xyz = original.copy(), planning.copy()
        self.surface_ids = surfaces.copy()
        self.radius = float(radius)
        for array in (self.source_indices, self.source_xyz, self.planning_xyz, self.surface_ids):
            array.flags.writeable = False
        self._source_tree, self._planning_tree = cKDTree(self.source_xyz), cKDTree(self.planning_xyz)

    def map_ground(self, points, *, inverse):
        points = _xyz(points, 'stair_ground')
        tree = self._planning_tree if inverse else self._source_tree
        distances, ids = tree.query(points, k=min(8, len(self.source_xyz)), workers=1)
        distances = np.asarray(distances).reshape((len(points), -1))
        ids = np.asarray(ids).reshape((len(points), -1))
        if np.any(distances[:, 0] > self.radius + 1e-9):
            raise GroundBridgeError('stair_mapping_measured_correspondence_missing')
        # A nearest-return delta jumps at Voronoi boundaries: neighbouring
        # observed returns on a noisy tread can have opposite corrections.
        # Interpolate only the ground-path correction, within the same labelled
        # surface and the original bounded observation neighbourhood. This is
        # neither a synthetic support cloud nor a transform for live body data.
        shifts = self.planning_xyz[ids, 2] - self.source_xyz[ids, 2]
        owned = self.surface_ids[ids] == self.surface_ids[ids[:, :1]]
        # Normalise the inverse-square kernel by the nearest distance. This
        # avoids overflow while keeping the exact-row result its continuous
        # limit, instead of snapping back inside a separate epsilon region.
        weights = np.zeros_like(distances)
        regular = distances[:, 0] > 0.
        if np.any(regular):
            reach = np.full((np.count_nonzero(regular), 1), self.radius)
            if distances.shape[1] == 8:
                # The last neighbour has zero compact-support weight, so a
                # rank-8/rank-9 exchange cannot jump the correction field.
                reach = np.minimum(reach, distances[regular, -1:])
            weights[regular] = (np.maximum(0., 1.-distances[regular]/reach)**2
                * (distances[regular, :1]/distances[regular])**2)
        weights *= owned
        denominator = weights.sum(axis=1)
        shift = shifts[:, 0].copy()
        valid = denominator > 0.
        shift[valid] = np.sum(weights[valid]*shifts[valid], axis=1)/denominator[valid]
        result = points.copy()
        result[:, 2] += -shift if inverse else shift
        return result, distances[:, 0]

    def association_pairs(self, literal_points):
        """Use only verified rows from the caller's source-owned stair set."""
        points = np.asarray(literal_points, dtype=float).reshape((-1, 3))
        if not len(points):
            return points.copy(), points.copy()
        distances, ids = cKDTree(points).query(self.source_xyz, workers=1)
        if np.any(distances > 1e-9) or not np.array_equal(points[ids], self.source_xyz):
            raise GroundBridgeError('stair_mapping_source_ownership_mismatch')
        return points[ids].copy(), self.planning_xyz


class GroundPathBridge:
    """A non-rigid, floor-labelled visualization bridge; never a control TF."""

    def __init__(self, floors: Mapping[str, FloorField], protected_regions: Sequence[dict],
                 *, source_frame: str = 'd1max_loc_map',
                 planning_frame: str = 'd1max_multifloor_planning',
                 limits: ProjectionLimits | None = None, stair_mapping=None):
        if not floors or not source_frame or not planning_frame or source_frame == planning_frame:
            raise GroundBridgeError('Distinct, explicit original/planning frames are required')
        self.floors = dict(floors)
        self.protected_regions = tuple(protected_regions)
        self.source_frame = source_frame
        self.planning_frame = planning_frame
        self.limits = limits or ProjectionLimits()
        self.stair_mapping = stair_mapping
        for region in self.protected_regions:
            low, high = np.asarray(region['min'], float), np.asarray(region['max'], float)
            if (low.shape != (3,) or high.shape != (3,)
                    or not np.isfinite([low, high]).all() or np.any(low >= high)):
                raise GroundBridgeError('Invalid protected stair volume')
        if stair_mapping is not None and (
                not _protected_mask(stair_mapping.source_xyz, self.protected_regions).all()
                or not _protected_mask(stair_mapping.planning_xyz, self.protected_regions).all()):
            raise GroundBridgeError('stair_mapping_outside_protected_domain')

    @classmethod
    def from_artifacts(cls, derived_manifest_path: str | Path, *, limits: ProjectionLimits | None = None):
        """Rebuild the *same* field from hashed source, poses, and cached evidence.

        This is intentionally an initialization-time operation, not an online
        callback.  No map file is modified and no process is launched.
        """
        final_path = Path(derived_manifest_path).resolve(strict=True)
        final = json.loads(final_path.read_text())
        if (final.get('status') != 'complete' or final.get('planning_only') is not True
                or final.get('frame_id') != 'd1max_multifloor_planning'):
            raise GroundBridgeError('Expected the completed non-rigid planning derivative')
        final_cloud_path = final_path.parent / final['output_file']
        if _digest(final_cloud_path) != final['output_sha256']:
            raise GroundBridgeError('Final PCT source map hash mismatch')
        floor_path = Path(final['floor_processing_manifest']).resolve(strict=True)
        if _digest(floor_path) != final['floor_processing_manifest_sha256']:
            raise GroundBridgeError('Floor-stage manifest hash mismatch')
        stage = json.loads(floor_path.read_text())
        if (stage.get('status') != 'complete' or stage.get('planning_only') is not True
                or stage.get('source_sha256') != final.get('source_sha256')
                or stage.get('frame_id') != final['frame_id']):
            raise GroundBridgeError('Floor-stage/final map provenance mismatch')
        source_path = Path(stage['source_path']).resolve(strict=True)
        trajectory_path = Path(stage['trajectory_path']).resolve(strict=True)
        if (_digest(source_path) != stage['source_sha256']
                or _digest(trajectory_path) != stage['trajectory_sha256']):
            raise GroundBridgeError('Original map or optimized trajectory hash mismatch')
        source_manifest_path = source_path.parent / 'manifest.json'
        source_manifest = json.loads(source_manifest_path.read_text())
        if source_manifest.get('frame_id') != 'd1max_loc_map':
            raise GroundBridgeError('Localization source is not d1max_loc_map')
        evidence_path = floor_path.parent / 'audit/floor_evidence.npz'
        evidence_hash = final.get('configuration', {}).get('floor_evidence_sha256')
        if not evidence_hash or _digest(evidence_path) != evidence_hash:
            raise GroundBridgeError('Floor evidence hash mismatch')
        cloud = read_pcd(source_path)
        if not cloud.finite_xyz_mask.all():
            raise GroundBridgeError('Original map contains nonfinite coordinates')
        matrices = np.loadtxt(trajectory_path, ndmin=2)
        if matrices.ndim != 2 or matrices.shape[1] != 12 or not np.isfinite(matrices).all():
            raise GroundBridgeError('Invalid optimized trajectory')
        poses = matrices[:, [3, 7, 11]]
        cfg = stage['configuration']
        defaults = stage['resolved_floor_defaults']
        protected = _protected_mask(cloud.xyz, cfg['protected_regions'])
        floors = {}
        with np.load(evidence_path, allow_pickle=False) as evidence:
            for floor in cfg['floors']:
                floor_id = floor['id']
                low, high = map(float, floor['input_z_band'])
                expected_ids = np.flatnonzero((cloud.xyz[:, 2] >= low)
                                          & (cloud.xyz[:, 2] < high) & ~protected)
                ids = evidence[f'{floor_id}_source_indices']
                if not np.array_equal(ids, expected_ids):
                    raise GroundBridgeError(f'{floor_id}: floor ownership changed')
                begin, end = floor['trajectory_range_inclusive']
                selected_poses = poses[begin:end + 1]
                selected_poses = selected_poses[
                    ~_protected_mask(selected_poses, cfg['protected_regions'])]
                params = {**defaults['conditioning'], **cfg.get('conditioning_overrides', {}),
                          'reference_z_m': float(floor['reference_z_m'])}
                normalized = _config(params)
                anchors = _anchors(cloud.xyz[ids], selected_poses, normalized)
                for field_name in ('xy', 'z'):
                    cached = evidence[f'{floor_id}_anchors_{field_name}']
                    if not np.allclose(anchors[field_name], cached, atol=1e-9, rtol=0):
                        raise GroundBridgeError(f'{floor_id}: reconstructed anchors disagree')
                support = evidence[f'{floor_id}_support']
                original_field = evidence[f'{floor_id}_floor_z']
                if (support.dtype != np.bool_ or support.shape != ids.shape
                        or original_field.shape != ids.shape or not support.any()
                        or not np.isfinite(original_field[support]).all()):
                    raise GroundBridgeError(f'{floor_id}: invalid cached support/domain')
                sample = np.flatnonzero(np.isfinite(original_field))
                sample = sample[np.linspace(0, len(sample) - 1, min(len(sample), 4096), dtype=int)]
                observed_field = _field(cloud.xyz[ids[sample], :2], anchors, normalized)
                if not np.allclose(observed_field, original_field[sample], atol=1e-8, rtol=0):
                    raise GroundBridgeError(f'{floor_id}: reconstructed field disagrees')
                floors[floor_id] = FloorField(
                    floor_id=floor_id, reference_z_m=float(floor['reference_z_m']),
                    input_z_band=(low, high), anchors=anchors,
                    support_xy=cloud.xyz[ids[support], :2], config=normalized)
        stair_mapping = None
        if 'stair_regularization' in final:
            stair_mapping = StairGroundMapping.from_artifact(final['stair_regularization'],
                base_dir=final_path.parent, source_xyz=cloud.xyz,
                source_sha256=final['source_sha256'], output_path=final_cloud_path,
                output_sha256=final['output_sha256'])
        bridge = cls(floors, cfg['protected_regions'], source_frame='d1max_loc_map',
                     planning_frame=final['frame_id'], limits=limits, stair_mapping=stair_mapping)
        model, model_hash, _ = load_parametric_stair_model(final_path, final)
        return ParametricGroundPathBridge(bridge, model, model_hash) if model is not None else bridge

    def to_localization_ground(self, planning_xyz, floor_ids: Sequence[str]) -> BridgeResult:
        """Estimate original-map ground geometry for a labelled PCT path.

        The estimate is not an exact inverse of snapped point records.  Results
        may be displayed in the original frame, but are not motion commands.
        """
        return self._map(_xyz(planning_xyz, 'planning_xyz'), floor_ids, inverse=True)

    def to_planning_ground(self, original_xyz, floor_ids: Sequence[str]) -> BridgeResult:
        """Map *known ground points*, never raw body poses or sensor returns."""
        return self._map(_xyz(original_xyz, 'original_xyz'), floor_ids, inverse=False)

    def project_live_pose_to_ground(self, original_body_xyz, floor_id: str,
                                    *, body_height_interval_m: tuple[float, float]) -> BridgeResult:
        """Use explicit level + calibrated body-height interval to query ground.

        The returned XYZ is on the planning *ground*, not a transformed body
        pose.  A stair/unknown-floor pose is deliberately rejected.
        """
        body = _xyz(np.asarray(original_body_xyz, float).reshape(1, 3), 'original_body_xyz')
        if floor_id not in ('floor1', 'floor2') or floor_id not in self.floors:
            raise GroundBridgeError('Live pose requires explicit floor1/floor2')
        lo, hi = map(float, body_height_interval_m)
        if not np.isfinite([lo, hi]).all() or lo <= 0 or hi <= lo or hi > 2.0:
            raise GroundBridgeError('Explicit finite body-height interval within (0, 2.0] m required')
        if _protected_mask(body, self.protected_regions)[0]:
            raise GroundBridgeError('Live pose is in the protected stair transition')
        floor = self.floors[floor_id]
        field, distance = floor.query(body[:, :2], self.limits)
        height = float(body[0, 2] - field[0])
        if not lo <= height <= hi:
            raise GroundBridgeError(f'{floor_id}: body height {height:.3f} m outside calibrated interval')
        ground = np.array([body[0, 0], body[0, 1], floor.reference_z_m])
        return BridgeResult(ground, {'source_ground_xyz': [float(body[0, 0]), float(body[0, 1]),
                                                          float(field[0])],
                                     'body_height_m': height, 'support_distance_m': float(distance[0]),
                                     'floor_id': floor_id, 'source_frame': self.source_frame,
                                     'planning_frame': self.planning_frame,
                                     'ground_only': True, 'exact_inverse': False,
                                     'certified_geometric_error_bound_m': None})

    def _map(self, points: np.ndarray, labels: Sequence[str], *, inverse: bool) -> BridgeResult:
        labels = tuple(labels)
        if len(labels) != len(points):
            raise GroundBridgeError('One explicit floor/transition label is required per point')
        output = points.copy()
        protected = _protected_mask(points, self.protected_regions)
        support_distances = np.zeros(len(points), dtype=float)
        correction = np.zeros(len(points), dtype=float)
        mapped_stairs = np.zeros(len(points), dtype=bool)
        for label in set(labels):
            ids = np.flatnonzero(np.asarray(labels) == label)
            active = ids[~protected[ids]]
            if (self.stair_mapping is not None
                    and (label in ('stairs', 'stair_lower', 'stair_upper') or label in self.floors)):
                # An entry/exit sample can retain a floor label while its
                # ground return belongs to the protected stair volume. Use
                # the same observed correspondence as selection/display;
                # the label must not create a second coordinate convention.
                stair_ids = ids[protected[ids]]
                if len(stair_ids):
                    output[stair_ids], support_distances[stair_ids] = self.stair_mapping.map_ground(
                        points[stair_ids], inverse=inverse)
                    correction[stair_ids] = output[stair_ids, 2] - points[stair_ids, 2]
                    mapped_stairs[stair_ids] = True
            if label == 'stairs':
                if len(active):
                    raise GroundBridgeError('Unprotected stair point requires stair_lower/stair_upper ownership')
                continue
            floor_id = {'stair_lower': 'floor1', 'stair_upper': 'floor2'}.get(label, label)
            if floor_id not in self.floors:
                raise GroundBridgeError(f'Unknown floor/transition label {label!r}')
            if not len(active):
                continue
            floor = self.floors[floor_id]
            field, distance = floor.query(points[active, :2], self.limits)
            shift = floor.reference_z_m - field
            if inverse:
                if label == floor_id and np.any(np.abs(points[active, 2] - floor.reference_z_m)
                                                > self.limits.max_ground_residual_m):
                    raise GroundBridgeError(f'{label}: planning path is not near its floor surface')
                output[active, 2] -= shift
                if np.any((output[active, 2] < floor.input_z_band[0])
                          | (output[active, 2] >= floor.input_z_band[1])):
                    raise GroundBridgeError(f'{label}: inverse leaves original floor ownership band')
            else:
                if label == floor_id and np.any(np.abs(points[active, 2] - field)
                                                > self.limits.max_ground_residual_m):
                    raise GroundBridgeError(f'{label}: source point is not observed ground')
                if np.any((points[active, 2] < floor.input_z_band[0])
                          | (points[active, 2] >= floor.input_z_band[1])):
                    raise GroundBridgeError(f'{label}: source point outside floor ownership band')
                output[active, 2] += shift
            correction[active] = output[active, 2] - points[active, 2]
            support_distances[active] = distance
        xy_step = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
        correction_step = np.abs(np.diff(correction))
        local = xy_step <= self.limits.adjacent_xy_m
        bad = np.flatnonzero(local & (correction_step > self.limits.max_boundary_jump_m))
        if len(bad):
            index = int(bad[0])
            raise GroundBridgeError(
                f'Non-rigid correction jumps {correction_step[index]:.3f} m '
                f'between adjacent path points {index}/{index + 1}')
        diagnostics = {
            'source_frame': self.planning_frame if inverse else self.source_frame,
            'target_frame': self.source_frame if inverse else self.planning_frame,
            'ground_path_only': True, 'exact_inverse_of_snapped_records': False,
            'certified_geometric_error_bound_m': None,
            'points': len(points), 'protected_stair_points': int(protected.sum()),
            'maximum_support_distance_m': float(support_distances.max()),
            'maximum_abs_height_correction_m': float(np.abs(correction).max()),
            'maximum_adjacent_correction_jump_m': float(correction_step[local].max(initial=0)),
            'limits': vars(self.limits).copy(),
        }
        if self.stair_mapping is not None:
            diagnostics['stair_regularization'] = dict(
                schema=self.stair_mapping.SCHEMA, mapping_sha256=self.stair_mapping.mapping_hash,
                source_sha256=self.stair_mapping.source_hash, mapped_points=int(mapped_stairs.sum()),
                support_semantics='source_returns_not_regularized_surface_observations')
        return BridgeResult(output, diagnostics)


def _protected_mask(points: np.ndarray, regions: Sequence[dict]) -> np.ndarray:
    result = np.zeros(len(points), dtype=bool)
    for region in regions:
        low, high = np.asarray(region['min'], float), np.asarray(region['max'], float)
        result |= ((points >= low) & (points <= high)).all(axis=1)
    return result
