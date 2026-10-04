"""Pure, fixed-anchor route progress and semantic reference windows.

This is a geometry contract, not collision validation or motion permission.
Source-map ground geometry is already immutable before entering this module.
Normal map corrections create candidates only: they never drag an admitted
odom reference. Replacing an anchor requires a new, externally revalidated
bundle; this module deliberately exposes no in-place reanchor operation.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np

from .control_frame_contract import BodySample, Context, ControlAnchor, create_anchor
from .source_route import RouteSnapshot


@dataclass(frozen=True)
class Observation:
    body: BodySample
    received_monotonic: float


@dataclass(frozen=True)
class ModeObservation:
    context: Context
    task_id: str
    route_hash: str
    segment_id: str
    mode: str
    source_ns: int
    received_monotonic: float


@dataclass(frozen=True)
class Progress:
    task_id: str
    route_hash: str
    anchor_id: str
    segment_id: str
    edge_index: int
    measured_arc_m: float
    confirmed_arc_m: float
    cross_track_m: float
    source_ns: int
    received_monotonic: float
    body_source_xyz: tuple


@dataclass(frozen=True)
class ReferenceWindow:
    context: Context
    task_id: str
    route_hash: str
    anchor_id: str
    anchor_revision: int
    segment_id: str
    segment_kind: str
    required_mode: str
    source_ns: int
    received_monotonic: float
    ground_source_xyz: tuple
    body_odom_xyz: tuple
    edge_indices: tuple
    frame_id: str = 'd1max_loc_odom'
    point_reference: str = 'body_center'
    ground_to_body_height_applications: int = 1
    execution_eligible: bool = False
    # Route arc of the last window sample. Same route arc as Progress, so it
    # stays comparable across anchor revisions; geometry bookkeeping only.
    end_arc_m: float = 0.


class ContinuousReference:
    def __init__(self, snapshot, *, context, task_id, anchor, body_height_m,
                 body_height_calibration_id, freshness_s=.5, maximum_speed_mps=1.5,
                 projection_radius_m=.3, backward_window_m=.35, forward_window_m=1.,
                 portal_radius_m=.12, stationary_span_s=.2, lateral_radius_m=1.4):
        if (not isinstance(snapshot, RouteSnapshot)
                or RouteSnapshot.create(snapshot.payload()) != snapshot
                or not isinstance(context, Context) or not isinstance(anchor, ControlAnchor)
                or anchor.context != context or snapshot.payload()['map_version_id'] != context.map_version
                or not isinstance(task_id, str) or not 0 < len(task_id) <= 160
                or not isinstance(body_height_calibration_id, str) or not body_height_calibration_id):
            raise ValueError('complete_immutable_route_and_anchor_identity_required')
        limits = ((body_height_m, .05, 1.5), (freshness_s, .02, 2.),
            (maximum_speed_mps, .01, 1.5), (projection_radius_m, .01, .5),
            (backward_window_m, .01, .5), (forward_window_m, .05, 2.),
            (portal_radius_m, .01, .25), (stationary_span_s, .1, 2.),
            (lateral_radius_m, .01, 1.5))
        if any(type(v) not in (int, float) or not math.isfinite(v) or not a <= v <= b
               for v, a, b in limits):
            raise ValueError('bounded_reference_parameters_required')
        self.snapshot, self.context, self.task_id, self.anchor = snapshot, context, task_id, anchor
        self.body_height, self.height_calibration_id = float(body_height_m), body_height_calibration_id
        self.freshness, self.max_speed, self.radius = freshness_s, maximum_speed_mps, projection_radius_m
        self.backward, self.forward, self.portal_radius = backward_window_m, forward_window_m, portal_radius_m
        self.stationary_span = stationary_span_s
        # projection_radius_m bounds height and along-route correspondence;
        # lateral_radius_m only bounds a horizontal local detour (box bypass)
        # so progress keeps advancing off-centerline. Not a collision allowance.
        if lateral_radius_m < projection_radius_m:
            raise ValueError('bounded_reference_parameters_required')
        self.lateral = float(lateral_radius_m)
        value = snapshot.payload()
        self._points = np.asarray(value['xyz'], dtype=float)
        self._arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self._points, axis=0), axis=1))]
        self._segments = tuple(value['segments'])
        self._segment = 0
        self._measured = self._confirmed = 0.
        self._progress = None
        self._last_sample = None
        self._history = deque(maxlen=128)
        self.candidate_anchor = None

    def _fresh(self, source_ns, received, *, current_source_ns, now_monotonic):
        if (type(source_ns) is not int or type(current_source_ns) is not int
                or source_ns <= 0 or current_source_ns < source_ns
                or (current_source_ns-source_ns)*1e-9 > self.freshness
                or type(received) not in (int, float) or not math.isfinite(received)
                or type(now_monotonic) not in (int, float) or not math.isfinite(now_monotonic)
                or received < 0 or not 0 <= now_monotonic-received <= self.freshness):
            raise ValueError('source_and_monotonic_receipt_must_both_be_fresh')

    def observe_pair(self, global_observation, local_observation, *, revision,
                     current_source_ns, now_monotonic):
        """Save a correction candidate, leaving all incumbent geometry unchanged."""
        for observation in (global_observation, local_observation):
            self._fresh(observation.body.source_ns, observation.received_monotonic,
                        current_source_ns=current_source_ns, now_monotonic=now_monotonic)
        candidate = create_anchor(global_observation.body, local_observation.body, revision)
        previous = self.candidate_anchor or self.anchor
        if (candidate.context != self.context or candidate.revision <= previous.revision
                or candidate.source_ns <= previous.source_ns):
            raise ValueError('correction_context_or_version_changed')
        if (math.dist(candidate.map_from_odom.xyz, previous.map_from_odom.xyz) <= 1e-9
                and abs(sum(a*b for a, b in zip(candidate.map_from_odom.xyzw,
                                              previous.map_from_odom.xyzw))) >= 1.-1e-12):
            # Pure body motion is not a map correction. Avoid minting an
            # anchor replacement every odometry tick for an identical rigid.
            return previous
        self.candidate_anchor = candidate
        return candidate

    def prepare_reanchor(self, observation, *, current_source_ns, now_monotonic):
        """Build an independent candidate; installing it requires external commit.

        A map correction is not measured robot motion. Reproject the latest
        local observation within the SAME semantic segment and bounded progress
        neighbourhood, without comparing the correction against robot speed.
        Neither incumbent geometry nor confirmed task progress is rewound.
        """
        candidate = self.candidate_anchor
        if candidate is None or candidate.context != self.context:
            raise ValueError('same_context_anchor_candidate_required')
        self._require_progress(current_source_ns, now_monotonic)
        self._fresh(observation.body.source_ns, observation.received_monotonic,
                    current_source_ns=current_source_ns, now_monotonic=now_monotonic)
        staged = deepcopy(self)
        staged.anchor, staged.candidate_anchor = candidate, None
        staged._last_sample = staged._progress = None
        staged._history.clear()
        # This bound is a correspondence check, not a motion allowance. It
        # cannot jump to a distant self-intersection or neighbouring branch.
        original_forward, original_lateral = staged.forward, staged.lateral
        staged.forward = min(staged.forward, staged.backward)
        # A correction may not move the body sideways by more than the
        # correspondence radius, even though a detour may already be off-line.
        staged.lateral = min(staged.lateral, self._progress.cross_track_m+staged.radius)
        staged.project(observation, current_source_ns=current_source_ns,
                       now_monotonic=now_monotonic)
        staged.forward, staged.lateral = original_forward, original_lateral
        return staged

    def project(self, observation, *, current_source_ns, now_monotonic):
        body = observation.body
        self._fresh(body.source_ns, observation.received_monotonic,
                    current_source_ns=current_source_ns, now_monotonic=now_monotonic)
        if (body.context != self.context or body.frame != 'd1max_loc_odom'
                or body.child != 'd1max_loc_base_link' or body.source_ns < self.anchor.source_ns
                or self._last_sample is not None and (body.source_ns <= self._last_sample[0]
                    or observation.received_monotonic < self._last_sample[1])):
            raise ValueError('local_body_context_or_source_not_newer')
        source_body = np.asarray(self.anchor.map_from_odom.point(body.pose.xyz))
        ground = source_body-np.array((0., 0., self.body_height))
        segment = self._segments[self._segment]
        a, b = segment['begin_index'], segment['end_index']
        dt = 0. if self._last_sample is None else (body.source_ns-self._last_sample[0])*1e-9
        advance = self.forward if self._last_sample is None else min(self.forward, self.max_speed*dt+.05)
        lo, hi = max(self._arc[a], self._measured-self.backward), min(self._arc[b], self._measured+advance)
        if self._last_sample is not None:
            displacement = math.dist(source_body, self._last_sample[2])
            if displacement > self.max_speed*dt+.05:
                raise ValueError('measured_body_discontinuity')
        candidates = []
        first = max(a, int(np.searchsorted(self._arc, lo, side='right'))-1)
        last = min(b, int(np.searchsorted(self._arc, hi, side='right')))
        for i in range(first, last):
            length = self._arc[i+1]-self._arc[i]
            if length <= 1e-9 or self._arc[i+1] < lo or self._arc[i] > hi:
                continue
            left, right = max(0., (lo-self._arc[i])/length), min(1., (hi-self._arc[i])/length)
            delta = self._points[i+1]-self._points[i]
            free = float(np.clip(np.dot(ground-self._points[i], delta)/(length*length), 0., 1.))
            t = float(np.clip(free, left, right))
            p = self._points[i]+delta*t
            r = ground-p
            # overshoot: along-route residual caused by the motion/progress
            # window bound, not by the robot being beside the centerline.
            candidates.append((float(np.linalg.norm(r)), float(self._arc[i]+t*length), i,
                               float(np.linalg.norm(r[:2])), abs(float(r[2])), abs(free-t)*length))
        if not candidates:
            raise ValueError('current_semantic_segment_has_no_projection')
        candidates.sort()
        distance, measured, edge, lateral, vertical, overshoot = candidates[0]
        if vertical > self.radius or overshoot > self.radius or lateral > self.lateral:
            raise ValueError('measured_body_outside_current_semantic_segment')
        if any(abs(c[0]-distance) < .01 and abs(c[1]-measured) > .25 for c in candidates[1:]):
            raise ValueError('projection_ambiguous_at_same_segment_crossing')
        self._measured, self._confirmed = measured, max(self._confirmed, measured)
        self._progress = Progress(self.task_id, self.snapshot.route_hash, self.anchor.anchor_id,
            segment['segment_id'], edge, measured, self._confirmed, lateral,
            body.source_ns, observation.received_monotonic, tuple(float(v) for v in source_body))
        self._history.append((body.source_ns, tuple(source_body)))
        self._last_sample = (body.source_ns, observation.received_monotonic, tuple(source_body))
        while len(self._history) > 3 and (body.source_ns-self._history[1][0])*1e-9 >= self.stationary_span:
            self._history.popleft()
        return self._progress

    def _stationary(self):
        if len(self._history) < 3:
            return False
        start, end = self._history[0], self._history[-1]
        return ((end[0]-start[0])*1e-9 >= self.stationary_span
                and all(math.dist(p, end[1]) <= .02 for _, p in self._history))

    def advance_segment(self, *, current_source_ns, now_monotonic, mode=None):
        """Explicit adjacent-portal transition, never nearest-neighbour relabeling."""
        p = self._require_progress(current_source_ns, now_monotonic)
        if self._segment+1 == len(self._segments):
            raise ValueError('already_final_semantic_segment')
        old, new = self._segments[self._segment:self._segment+2]
        end = old['end_index']
        portal_body = self._points[end]+np.array((0., 0., self.body_height))
        if (new['begin_index'] != end or not old['exit_portal_id']
                or old['exit_portal_id'] != new['entry_portal_id']
                or self._arc[end]-p.measured_arc_m > self.portal_radius
                or math.dist(p.body_source_xyz, portal_body) > self.portal_radius):
            raise ValueError('adjacent_portal_not_physically_reached')
        mode_matches = False
        if mode is not None:
            self._fresh(mode.source_ns, mode.received_monotonic,
                        current_source_ns=current_source_ns, now_monotonic=now_monotonic)
            mode_matches = (mode.context == self.context and mode.task_id == self.task_id
                and mode.route_hash == self.snapshot.route_hash and mode.segment_id == new['segment_id']
                and mode.mode == new['required_mode'] and mode.source_ns >= p.source_ns)
        if not self._stationary() and not mode_matches:
            raise ValueError('portal_requires_measured_stop_or_bound_mode_evidence')
        self._segment += 1
        self._history.clear()
        # A new measured projection is required before a window from the new
        # segment can be issued. Old progress cannot pretend to own new mode.
        self._progress = None
        return new['segment_id']

    def _require_progress(self, current_source_ns, now_monotonic):
        if self._progress is None:
            raise ValueError('fresh_measured_projection_required')
        self._fresh(self._progress.source_ns, self._progress.received_monotonic,
                    current_source_ns=current_source_ns, now_monotonic=now_monotonic)
        return self._progress

    def window(self, *, current_source_ns, now_monotonic, horizon_m=4., spacing_m=.1):
        p = self._require_progress(current_source_ns, now_monotonic)
        if (type(horizon_m) not in (int, float) or not .05 <= horizon_m <= 10.
                or type(spacing_m) not in (int, float) or not .02 <= spacing_m <= .5):
            raise ValueError('bounded_reference_window_required')
        segment = self._segments[self._segment]
        end = min(self._arc[segment['end_index']], p.measured_arc_m+horizon_m)
        if end-p.measured_arc_m <= .01:
            raise ValueError('segment_endpoint_reached_requires_transition_or_arrival')
        count = max(2, int(math.ceil((end-p.measured_arc_m)/spacing_m))+1)
        samples = np.linspace(p.measured_arc_m, end, count)
        edges = np.clip(np.searchsorted(self._arc, samples, side='right')-1,
                        segment['begin_index'], segment['end_index']-1)
        points = []
        for s, i in zip(samples, edges):
            span = self._arc[i+1]-self._arc[i]
            t = (s-self._arc[i])/span if span > 1e-9 else 0.
            points.append(tuple(float(v) for v in self._points[i]+(self._points[i+1]-self._points[i])*t))
        inverse = self.anchor.map_from_odom.inverse()
        body = tuple(inverse.point((x, y, z+self.body_height)) for x, y, z in points)
        return ReferenceWindow(self.context, self.task_id, self.snapshot.route_hash,
            self.anchor.anchor_id, self.anchor.revision, segment['segment_id'], segment['kind'],
            segment['required_mode'], p.source_ns, p.received_monotonic, tuple(points), body,
            tuple(int(i) for i in edges), end_arc_m=float(end))
