"""Audited *ground-path* bridge between the original and conditioned map frames.

This is not a TF and must not be used to transform an IMU, lidar scan, or robot
body pose.  The planning map has a spatially varying Z correction and snapped
floor returns; an arbitrary 3-D point has no exact inverse.  Only explicitly
labelled, observed-support ground-path samples are admitted here.  A live body
pose can be used to *query* the corresponding two ground surfaces, with an
explicit caller-supplied body-height gate, but is never itself warped.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence

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


class GroundPathBridge:
    """A non-rigid, floor-labelled visualization bridge; never a control TF."""

    def __init__(self, floors: Mapping[str, FloorField], protected_regions: Sequence[dict],
                 *, source_frame: str = 'd1max_loc_map',
                 planning_frame: str = 'd1max_multifloor_planning',
                 limits: ProjectionLimits | None = None):
        if not floors or not source_frame or not planning_frame or source_frame == planning_frame:
            raise GroundBridgeError('Distinct, explicit original/planning frames are required')
        self.floors = dict(floors)
        self.protected_regions = tuple(protected_regions)
        self.source_frame = source_frame
        self.planning_frame = planning_frame
        self.limits = limits or ProjectionLimits()
        for region in self.protected_regions:
            low, high = np.asarray(region['min'], float), np.asarray(region['max'], float)
            if (low.shape != (3,) or high.shape != (3,)
                    or not np.isfinite([low, high]).all() or np.any(low >= high)):
                raise GroundBridgeError('Invalid protected stair volume')

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
        return cls(floors, cfg['protected_regions'], source_frame='d1max_loc_map',
                   planning_frame=final['frame_id'], limits=limits)

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
        for label in set(labels):
            ids = np.flatnonzero(np.asarray(labels) == label)
            active = ids[~protected[ids]]
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
        return BridgeResult(output, diagnostics)


def _protected_mask(points: np.ndarray, regions: Sequence[dict]) -> np.ndarray:
    result = np.zeros(len(points), dtype=bool)
    for region in regions:
        low, high = np.asarray(region['min'], float), np.asarray(region['max'], float)
        result |= ((points >= low) & (points <= high)).all(axis=1)
    return result
