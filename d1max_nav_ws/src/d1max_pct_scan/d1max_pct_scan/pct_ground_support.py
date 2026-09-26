"""Bounded PCT centreline-ground admission in the original localization frame.

This verifies that a displayed body spline stays above *valid PCT support
cells*. It is not a foot-placement, swept-body collision, or motion permit.
``source_pcd_sha256`` pins the original localization PCD manifest's source
hash; ``source_tomogram_sha256`` separately pins the conditioned PCT map.
The input points are the audited ground-only inverse of the PCT surfaces;
live scans/body poses must never be non-rigidly warped into the PCT frame.
"""
from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
import tempfile

import numpy as np


class GroundSupportError(ValueError):
    pass


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _metadata(metadata):
    if not isinstance(metadata, dict):
        raise GroundSupportError('invalid_support_metadata')
    for key in ('frame_id', 'source_pcd_sha256', 'source_tomogram_sha256'):
        value = metadata.get(key)
        if not isinstance(value, str) or not value or len(value) > 128:
            raise GroundSupportError('missing_' + key)
    if metadata['frame_id'] != 'd1max_loc_map':
        raise GroundSupportError('support_frame_mismatch')
    for key in ('source_pcd_sha256', 'source_tomogram_sha256'):
        value = metadata[key]
        if len(value) != 64 or any(char not in '0123456789abcdef' for char in value):
            raise GroundSupportError('invalid_' + key)
    resolution = metadata.get('resolution')
    if (type(resolution) not in (float, int) or not math.isfinite(resolution)
            or not .02 <= resolution <= .5):
        raise GroundSupportError('invalid_support_resolution')
    return float(resolution)


class PCTGroundSupport:
    """Sparse exact-cell lookup; different floors retain separate Z candidates."""

    def __init__(self, origin, resolution, shape, keys, offsets, ground_z,
                 metadata, *, index_sha256=None):
        self.resolution = _metadata(metadata)
        if abs(self.resolution - resolution) > 1e-9:
            raise GroundSupportError('support_resolution_mismatch')
        self.origin = np.asarray(origin, dtype=np.float64)
        self.shape = np.asarray(shape, dtype=np.int64)
        self.keys = np.asarray(keys, dtype=np.int64)
        self.offsets = np.asarray(offsets, dtype=np.int64)
        self.ground_z = np.asarray(ground_z, dtype=np.float64)
        if (self.origin.shape != (2,) or not np.isfinite(self.origin).all()
                or self.shape.shape != (2,) or np.any(self.shape < 1)
                or np.any(self.shape > 1000000) or int(np.prod(self.shape)) > 10**10
                or self.keys.ndim != 1 or len(self.keys) < 1 or len(self.keys) > 1000000
                or self.offsets.shape != (len(self.keys) + 1,)
                or self.offsets[0] != 0 or self.offsets[-1] != len(self.ground_z)
                or len(self.ground_z) > 1000000 or not np.isfinite(self.ground_z).all()
                or np.any(np.diff(self.keys) <= 0) or np.any(np.diff(self.offsets) <= 0)
                or np.any(self.keys < 0) or np.any(self.keys >= int(np.prod(self.shape)))):
            raise GroundSupportError('invalid_sparse_support_index')
        self.metadata = dict(metadata)
        self.index_sha256 = index_sha256

    @classmethod
    def from_arrays(cls, xyz_original, metadata):
        """Build only from PCT-valid original-frame ground centres, never raw PCD."""
        resolution = _metadata(metadata)
        points = np.asarray(xyz_original, dtype=np.float64)
        if (points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= 1000000
                or not np.isfinite(points).all()):
            raise GroundSupportError('invalid_valid_pct_ground_centres')
        origin = points[:, :2].min(axis=0)
        ij_float = (points[:, :2] - origin) / resolution
        ij = np.rint(ij_float).astype(np.int64)
        if (np.max(np.abs(ij_float - ij)) > 1e-3 or np.any(ij < 0)
                or np.any(ij.max(axis=0) > 999999)):
            raise GroundSupportError('pct_ground_is_not_one_regular_cell_lattice')
        shape = ij.max(axis=0) + 1
        keys = ij[:, 0] * shape[1] + ij[:, 1]
        order = np.lexsort((points[:, 2], keys))
        keys, heights = keys[order], points[order, 2]
        unique, first = np.unique(keys, return_index=True)
        offsets = np.r_[first, len(keys)]
        return cls(origin, resolution, shape, unique, offsets, heights, metadata)

    @classmethod
    def from_npz(cls, path, *, expected_sha256, expected_source_pcd_sha256,
                 expected_tomogram_sha256, expected_frame='d1max_loc_map'):
        path = Path(path)
        if _sha256(path) != expected_sha256:
            raise GroundSupportError('support_index_hash_mismatch')
        with np.load(path, allow_pickle=False) as archive:
            metadata = {key: str(archive[key].item()) for key in
                        ('frame_id', 'source_pcd_sha256', 'source_tomogram_sha256')}
            metadata['resolution'] = float(archive['resolution'])
            if (metadata['frame_id'] != expected_frame
                    or metadata['source_pcd_sha256'] != expected_source_pcd_sha256
                    or metadata['source_tomogram_sha256'] != expected_tomogram_sha256):
                raise GroundSupportError('support_index_provenance_mismatch')
            return cls(archive['origin'], metadata['resolution'], archive['shape'],
                       archive['keys'], archive['offsets'], archive['ground_z'], metadata,
                       index_sha256=expected_sha256)

    def write_npz(self, path):
        """Atomic offline/session-preparation artifact; never called per spline."""
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix='.pct-support-', suffix='.npz', dir=destination.parent)
        try:
            with os.fdopen(fd, 'wb') as stream:
                np.savez_compressed(stream, origin=self.origin, resolution=self.resolution,
                    shape=self.shape, keys=self.keys, offsets=self.offsets,
                    ground_z=self.ground_z, frame_id=self.metadata['frame_id'],
                    source_pcd_sha256=self.metadata['source_pcd_sha256'],
                    source_tomogram_sha256=self.metadata['source_tomogram_sha256'])
            os.replace(temporary, destination)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return _sha256(destination)

    def _cell_heights(self, cell):
        x, y = cell
        if not (0 <= x < self.shape[0] and 0 <= y < self.shape[1]):
            raise GroundSupportError('pct_support_outside_map')
        key = int(x * self.shape[1] + y)
        position = int(np.searchsorted(self.keys, key))
        if position >= len(self.keys) or self.keys[position] != key:
            raise GroundSupportError('pct_support_hole_or_blocked_cell')
        return self.ground_z[self.offsets[position]:self.offsets[position + 1]]

    def _ground(self, cell, desired, tolerance, max_step):
        heights = self._cell_heights(cell)
        eligible = heights[np.abs(heights - desired) <= tolerance + 1e-9]
        if not len(eligible):
            raise GroundSupportError('pct_support_wrong_height_or_floor')
        residual = np.abs(eligible-desired)
        selected = int(np.argmin(residual))
        # Two genuinely different surfaces may coexist at a stair landing.
        # An exact/clear best match is usable; nearly tied distinct heights
        # require an explicit layer and cannot be guessed from body Z.
        tied = eligible[np.abs(residual-residual[selected]) <= .01]
        if len(tied) > 1 and np.ptp(tied) > max_step + 1e-9:
            raise GroundSupportError('pct_support_ambiguous_floor')
        return float(eligible[selected])

    def _segment_cells(self, a, b):
        """Exact supercover; cell centres are integer coordinates."""
        a = (np.asarray(a, dtype=float) - self.origin) / self.resolution + .5
        b = (np.asarray(b, dtype=float) - self.origin) / self.resolution + .5
        delta = b - a
        ts = [0., 1.]
        for axis in range(2):
            if abs(delta[axis]) > 1e-12:
                low, high = sorted((a[axis], b[axis]))
                for boundary in range(math.ceil(low), math.floor(high) + 1):
                    t = (boundary - a[axis]) / delta[axis]
                    if 0 < t < 1:
                        ts.append(float(t))
        ts = sorted(set(ts))
        hits = set()
        for t in ts + [(x + y)/2 for x, y in zip(ts, ts[1:])]:
            point = a + t * delta
            axes = []
            for value in point:
                integer = round(float(value))
                axes.append((integer - 1, integer) if abs(value - integer) < 1e-8
                            else (math.floor(value),))
            hits.update((x, y) for x in axes[0] for y in axes[1])
        return hits

    def validate_body_samples(self, samples_xyz, *, body_height_m=.55,
                              height_tolerance_m=.20, max_ground_step_m=.17):
        """Admit only an unchanged, finely sampled body centreline above PCT cells."""
        samples = np.asarray(samples_xyz, dtype=np.float64)
        if (samples.ndim != 2 or samples.shape[1] != 3 or not 2 <= len(samples) <= 20001
                or not np.isfinite(samples).all()):
            raise GroundSupportError('invalid_body_spline_samples')
        for name, value, low, high in (
                ('body_height', body_height_m, .1, 1.),
                ('height_tolerance', height_tolerance_m, .01, .3),
                ('max_ground_step', max_ground_step_m, .01, .25)):
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise GroundSupportError('invalid_' + name)
        if np.max(np.linalg.norm(np.diff(samples, axis=0), axis=1)) > .08 + 1e-9:
            raise GroundSupportError('body_spline_sampling_too_sparse')
        checked = set()
        previous_segment = {}
        maximum_residual, maximum_step = 0., 0.
        for first, second in zip(samples[:-1], samples[1:]):
            delta = second[:2] - first[:2]
            denom = float(delta @ delta)
            segment_ground = {}
            for cell in self._segment_cells(first[:2], second[:2]):
                centre = self.origin + np.asarray(cell, dtype=float) * self.resolution
                t = min(1., max(0., float((centre-first[:2]) @ delta / denom))) if denom > 1e-12 else .5
                desired = float(first[2] + t * (second[2]-first[2]) - body_height_m)
                ground = self._ground(cell, desired, height_tolerance_m, max_ground_step_m)
                maximum_residual = max(maximum_residual, abs(ground-desired))
                # Compare this segment and the immediately preceding segment,
                # not every historical XY visit: stacked floors may legitimately
                # reuse XY much later in a route. Set order cannot skip checks.
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        neighbour = (cell[0]+dx, cell[1]+dy)
                        for nearby in (segment_ground, previous_segment):
                            if neighbour not in nearby:
                                continue
                            step = abs(ground-nearby[neighbour])
                            maximum_step = max(maximum_step, step)
                            if step > max_ground_step_m + 1e-9:
                                raise GroundSupportError('pct_support_ground_step')
                segment_ground[cell] = ground
                checked.add(cell)
            previous_segment = segment_ground
        return {'checked_support_cells': len(checked),
                'maximum_height_residual_m': maximum_residual,
                'maximum_adjacent_ground_step_m': maximum_step,
                'body_height_assumption_m': float(body_height_m),
                'support_index_sha256': self.index_sha256,
                'centreline_support_only': True,
                'foot_placement_or_swept_volume_certified': False,
                'motion_authorized': False}


def write_support_index(path, xyz_original, metadata):
    """Prepare a hash-pinned light artifact from valid PCT ground points once."""
    return PCTGroundSupport.from_arrays(xyz_original, metadata).write_npz(path)
