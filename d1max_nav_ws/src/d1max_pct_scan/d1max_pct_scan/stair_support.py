"""Associate literal stair returns with independently evidenced support.

This is a static source/classification record, not a gait capability, live
occupancy clearance or permission to move. No point is removed from GridMap.
In particular, a protected stair ROI also contains walls and railings; its
membership alone must never turn all of those returns into support.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class StairSupportSelection:
    points: np.ndarray
    verified: bool
    reason: str
    samples: int = 0
    maximum_distance_m: float = math.inf


class StairSupportIndex:
    """Bounded, original-XYZ subset of measured stair surfaces.

    Association uses the exact production PCT cell convention. A hash-bound
    regularization mapping can provide the planning coordinate of a measured
    row; the returned support is still that row's unchanged original XYZ.
    Multiple surfaces at the same XY remain separated by their original Z.
    Optional independently classified literal tread evidence describes terrain,
    not whether an inflated robot footprint can traverse that point.
    """
    def __init__(self, literal_points, tomogram, *, support_radius_m=.12, stair_mapping=None,
                 structured_source_points=None):
        points = np.asarray(literal_points, dtype=float).reshape((-1, 3))
        if not np.isfinite(points).all() or not .01 <= support_radius_m <= .3:
            raise ValueError('invalid_stair_support_source')
        self.radius = float(support_radius_m)
        self.surface_tolerance_m = min(.06, float(tomogram.resolution)*.5)
        association = points
        if stair_mapping is not None:
            # Never classify a raw wall/railing merely because it lies in the
            # stair ROI. Only the mapping's verified source rows are eligible.
            points, association = stair_mapping.association_pairs(points)
        # A package loader may additionally supply independently classified
        # tread returns (literal source identity + measured normal/connected
        # patch + retained structural-obstacle exclusion). PCT's body-clearance
        # cost does not decide whether such a surface was observed. This is
        # NOT an ROI mask, synthesized model grid, or collision exemption.
        # Require exact mapping rows, then intersect with the caller's verified
        # original-source subset. Old packages without this evidence retain
        # exactly the previous PCT classification below.
        structured = np.zeros(len(points), dtype=bool)
        if structured_source_points is not None:
            evidence = np.asarray(structured_source_points, dtype=float)
            if (stair_mapping is None or evidence.ndim != 2 or evidence.shape[1:] != (3,)
                    or len(evidence) > 20000 or not np.isfinite(evidence).all()):
                raise ValueError('structured_stair_support_evidence_invalid')
            classified = set(map(tuple, evidence))
            if not classified <= set(map(tuple, stair_mapping.source_xyz)):
                raise ValueError('structured_stair_support_not_literal_mapping_rows')
            structured = np.asarray([tuple(p) in classified for p in points], dtype=bool)
        # A per-return association is made once at map loading. Never allocate
        # layers x entire PCD or run ground classification on the control lane.
        keep = np.zeros(len(points), dtype=bool)
        for begin in range(0, len(points), 8192):
            batch = association[begin:begin+8192]
            relative = (batch[:, :2]-tomogram.center)/tomogram.resolution
            cells = np.rint(relative).astype(np.int64)+tomogram.offset
            inside = ((cells[:, 0] >= 0) & (cells[:, 0] < tomogram.nx)
                      & (cells[:, 1] >= 0) & (cells[:, 1] < tomogram.ny))
            ids = np.flatnonzero(inside)
            x, y = cells[ids, 0], cells[ids, 1]
            if len(ids):
                ground = tomogram.ground[:, x, y]
                matches = (tomogram.valid[:, x, y] & tomogram.ground_known[:, x, y]
                           & (np.abs(ground-batch[ids, 2]) <= self.surface_tolerance_m+1e-9))
                keep[begin+ids] = np.any(matches, axis=0) | structured[begin+ids]
        self.points = points[keep].copy()
        self.points.flags.writeable = False
        self.tree = cKDTree(self.points) if len(self.points) else None

    def select(self, ground_reference, *, sample_spacing_m=.05, classified_floor_tree=None,
               context_radius_m=1.5):
        """Select source returns around this XYZ window, checking its coverage.

        This only proves nearby measured surface returns for a particular
        reference. It does not prove foot placement or that the body is clear.
        SCAN remains the sole live collision owner.
        """
        empty = np.empty((0, 3), dtype=float)
        path = np.asarray(ground_reference, dtype=float)
        if (path.ndim != 2 or path.shape[1:] != (3,) or not 2 <= len(path) <= 10000
                or not np.isfinite(path).all() or not .02 <= sample_spacing_m <= .1
                or not np.isfinite(context_radius_m) or not self.radius <= context_radius_m <= 2.):
            return StairSupportSelection(empty, False, 'stair_reference_geometry_missing')
        trees=[tree for tree in (self.tree,classified_floor_tree) if tree is not None]
        if not trees:
            return StairSupportSelection(empty, False, 'stair_measured_surface_returns_missing')
        samples = [path[0]]
        for start, end in zip(path[:-1], path[1:]):
            count = max(1, int(math.ceil(float(np.linalg.norm(end-start))/sample_spacing_m)))
            if len(samples)+count > 10000:
                return StairSupportSelection(empty, False, 'stair_reference_sampling_budget_exceeded')
            samples.extend(start+(end-start)*(i/count) for i in range(1, count+1))
        samples = np.asarray(samples)
        queries=[tree.query(samples,workers=1) for tree in trees]
        choices=np.argmin(np.array([q[0] for q in queries]),axis=0)
        distances=np.min(np.array([q[0] for q in queries]),axis=0)
        maximum = float(np.max(distances))
        if maximum > self.radius+1e-9:
            return StairSupportSelection(empty, False, 'stair_measured_surface_coverage_missing',
                                         len(samples), maximum)
        # Coverage and search context are different. A 20 cm centreline strip
        # excluded all of SCAN's 45--125 cm lateral candidate supports, even
        # when the actual measured tread continued across the stair width.
        # Keep nearby literal returns for native candidate queries; never
        # increase the 12 cm coverage tolerance or manufacture a flat surface.
        # XYZ neighbourhoods also avoid copying a whole other floor at this XY.
        # Required centreline rows always survive the bounded packet budget.
        required,optional=[],[]
        for number,tree in enumerate(trees):
            nearest_ids=np.unique(queries[number][1][choices==number])
            neighbors=tree.query_ball_point(samples,context_radius_m,workers=1)
            ids=np.unique(np.concatenate([nearest_ids,*[np.asarray(v,dtype=int) for v in neighbors]]))
            required.append(tree.data[nearest_ids])
            optional.append(tree.data[np.setdiff1d(ids,nearest_ids,assume_unique=True)])
        required=np.unique(np.concatenate(required),axis=0)
        extras=np.unique(np.concatenate(optional),axis=0)
        budget=20000-len(required)
        if len(extras)>budget:
            extras=extras[np.linspace(0,len(extras)-1,budget,dtype=int)] if budget else extras[:0]
        selected=np.unique(np.concatenate([required,extras]),axis=0)
        selected.flags.writeable = False
        return StairSupportSelection(selected, True, 'original_stair_surface_returns_associated',
                                     len(samples), maximum)
