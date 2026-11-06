"""Immutable source-map domains for semantic portal correspondence.

This is neither collision clearance nor execution permission. Native SCAN still
owns the complete robot footprint and live obstacles. These finite domains only
prevent a bounded, off-centre crossing from being mistaken for another floor or
for an unrelated branch outside the already modelled stair structure.
"""
from dataclasses import dataclass

import numpy as np


def _array(value):
    source = np.asarray(value, dtype=float)
    return np.frombuffer(source.tobytes(), dtype=float).reshape(source.shape)


@dataclass(frozen=True)
class PortalSurface:
    owners: frozenset
    origin: np.ndarray
    basis: np.ndarray
    polygon: np.ndarray
    grade: float
    intercept: float


@dataclass(frozen=True)
class PortalBoundary:
    center: np.ndarray
    rotation: np.ndarray
    half: np.ndarray


@dataclass(frozen=True)
class ReferencePortalGeometry:
    source_hash: str
    frame_id: str
    surfaces: tuple
    boundaries: tuple

    @classmethod
    def from_model(cls, model):
        # The package loader has already validated ParametricStairModel.
        # Copy only immutable numeric geometry, not its mutable dicts/caches.
        if model.frame_id != 'd1max_loc_map':
            raise ValueError('reference_portal_geometry_frame_mismatch')
        return cls(model.record['source_map_sha256'], model.frame_id,
            tuple(PortalSurface(frozenset(s['segment_ids']), _array(s['_origin']),
                _array(s['_basis']), _array(s['_polygon']), float(s['support_grade']),
                float(s['support_intercept_m'])) for s in model.surfaces),
            tuple(PortalBoundary(_array(b['_center']), _array(b['_rotation']),
                _array(b['_half'])) for b in model.obstacles))

    def __deepcopy__(self, memo):
        return self  # All numeric buffers are bytes-owned and read-only.

    def contains(self, ground, owners, *, height, vertical_bound):
        point = np.asarray(ground, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            return False
        matched = False
        for surface in self.surfaces:
            if not surface.owners.intersection(owners):
                continue
            local = (point[:2]-surface.origin[:2]) @ surface.basis[:2,:2].T
            edge = np.roll(surface.polygon,-1,axis=0)-surface.polygon
            delta = local-surface.polygon
            if np.any(edge[:,0]*delta[:,1]-edge[:,1]*delta[:,0] < -1e-9):
                continue
            z = surface.origin[2]+surface.intercept+surface.grade*local[0]
            if abs(point[2]-z) <= vertical_bound:
                matched = True
                break
        if not matched:
            return False
        return self.clear_column(point,height=height)

    def clear_column(self, ground, *, height):
        point = np.asarray(ground,dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            return False
        # A source-map reference column cannot be relabelled through a wall
        # or handrail model. This is NOT a second robot collision envelope.
        for boundary in self.boundaries:
            start = (point-boundary.center) @ boundary.rotation
            delta = np.array((0.,0.,height)) @ boundary.rotation
            lo,hi = 0.,1.
            for axis in range(3):
                if abs(delta[axis]) <= 1e-12:
                    if abs(start[axis]) > boundary.half[axis]:
                        lo,hi = 1.,0.
                        break
                else:
                    ends = ((-boundary.half[axis]-start[axis])/delta[axis],
                            (boundary.half[axis]-start[axis])/delta[axis])
                    lo,hi = max(lo,min(ends)),min(hi,max(ends))
            if lo <= hi:
                return False
        return True
