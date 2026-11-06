"""Spatially rolling reference scheduling; not a trajectory or motion lease.

The original window extents survive a delayed geometry commit. Scheduling
never mutates an accepted window, regenerates a route, or cancels a pending
candidate. The caller still owns source freshness, cancellation and portals.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WindowProgress:
    start_arc_m: float
    end_arc_m: float
    issued_confirmed_arc_m: float
    confirmed_arc_m: float


@dataclass(frozen=True)
class RollingReferencePolicy:
    minimum_interval_s: float = .5
    advance_m: float = .25
    minimum_overlap_fraction: float = .75

    def reason(self, window, *, elapsed_s, correction_due=False, covers_segment_end=False):
        """Return a scheduling reason, or None; never an execution verdict.

        Motion renews a 4 m window after 25 cm, with at least 500 ms between
        ordinary proposals. Short windows scale the spatial step to retain
        >=75% overlap at the trigger (delayed delivery may consume more).
        A low remaining look-ahead is a catch-up trigger, not a commit-time
        reset of the measured progress or a demand to stop at a window seam.
        """
        if math.isnan(elapsed_s) or elapsed_s < self.minimum_interval_s:
            return None
        if window is None:
            return 'initial_window'
        if correction_due:
            return 'coordinate_anchor_update'
        if covers_segment_end:
            return None
        if not all(math.isfinite(v) for v in (
                window.start_arc_m, window.end_arc_m,
                window.issued_confirmed_arc_m, window.confirmed_arc_m)):
            return None
        span = window.end_arc_m-window.start_arc_m
        if span <= .01:
            return None
        # The geometry starts at the measured arc. Its monotonic issue water-
        # mark is separate: after a small reverse, rebuilding from that actual
        # position must not make an unmoving robot publish forever.
        advanced = window.confirmed_arc_m-window.issued_confirmed_arc_m
        if advanced <= 1e-6:
            return None
        remaining = window.end_arc_m-window.confirmed_arc_m
        if remaining <= .5*span:
            return 'recover_lookahead'
        step = min(self.advance_m, (1.-self.minimum_overlap_fraction)*span)
        if advanced+1e-9 >= step:
            return 'measured_progress'
        return None
