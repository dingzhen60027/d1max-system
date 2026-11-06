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
from types import MappingProxyType

import numpy as np

from .control_frame_contract import BodySample, Context, ControlAnchor, create_anchor
from .reference_geometry_contract import PROJECTION_RADIUS_M, LATERAL_RADIUS_M
from .source_route import RouteSnapshot, canonical


EXECUTION_MODE_POLICIES = ('segment_modes', 'fixed_stair')


def validate_execution_mode_policy(value):
    """Session control policy, separate from immutable terrain semantics."""
    if value not in EXECUTION_MODE_POLICIES:
        raise ValueError('explicit_execution_mode_policy_required')
    return value


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
class ReceiptModeConfirmation:
    """A writer-confirmed SDK callback event; not a vendor sensor timestamp."""
    context: Context
    task_id: str
    route_hash: str
    segment_id: str
    mode: str
    callback_ns: int
    valid_until_ns: int
    received_monotonic: float
    callback_sequence: int
    sdk_session: str
    time_basis: str = 'sdk_callback_receipt_no_vendor_stamp'


@dataclass(frozen=True)
class PreviewSegmentTransition:
    """Geometry-only request accepted exclusively by a planning-only core."""
    context: Context
    task_id: str
    route_hash: str
    segment_id: str


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
    start_arc_m: float = 0.
    issued_confirmed_arc_m: float = 0.
    point_segment_ids: tuple = ()
    point_segment_kinds: tuple = ()
    point_required_modes: tuple = ()


def _immutable_array(values):
    """Bytes-owned storage cannot be made writable by a candidate reader."""
    array = np.asarray(values, dtype=float)
    return np.frombuffer(array.tobytes(), dtype=float).reshape(array.shape)


def _immutable_metadata(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _immutable_metadata(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_immutable_metadata(item) for item in value)
    return value


@dataclass(frozen=True)
class SupportEvidence:
    source_map_sha256: str
    execution_eligible: bool
    eligibility_reason: str
    observed_source_support_xyz: np.ndarray
    canonical_observed_support: str


class ContinuousReference:
    def __init__(self, snapshot, *, context, task_id, anchor, body_height_m,
                 body_height_calibration_id, freshness_s=.5, maximum_speed_mps=1.5,
                 projection_radius_m=PROJECTION_RADIUS_M, backward_window_m=.35, forward_window_m=1.,
                 portal_radius_m=.12, stationary_span_s=.2, lateral_radius_m=LATERAL_RADIUS_M,
                 preview_only=False, execution_mode_policy='segment_modes', portal_geometry=None):
        value = snapshot.payload() if isinstance(snapshot, RouteSnapshot) else None
        if (value is None or RouteSnapshot.create(value) != snapshot
                or not isinstance(context, Context) or not isinstance(anchor, ControlAnchor)
                or anchor.context != context or value['map_version_id'] != context.map_version
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
        if portal_geometry is not None:
            from .reference_portal_geometry import ReferencePortalGeometry
            if (not isinstance(portal_geometry,ReferencePortalGeometry)
                    or portal_geometry.source_hash!=value['source_map_sha256']
                    or portal_geometry.frame_id!='d1max_loc_map'):
                raise ValueError('reference_portal_geometry_identity_mismatch')
        self.portal_geometry=portal_geometry
        if type(preview_only) is not bool:
            raise ValueError('explicit_preview_capability_required')
        self.preview_only=preview_only
        # This is not mode evidence or motion permission. The single SDK
        # writer owns the session's actual operating mode. Segment modes below
        # continue to describe support geometry even under fixed STAIR control.
        self.execution_mode_policy=validate_execution_mode_policy(execution_mode_policy)
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
        self._points = _immutable_array(value['xyz'])
        self._arc = _immutable_array(np.r_[0., np.cumsum(np.linalg.norm(np.diff(self._points, axis=0), axis=1))])
        self._segments = tuple(_immutable_metadata(segment) for segment in value['segments'])
        self.goal_ground_xyz = tuple(value['xyz'][-1])
        self.goal_heading = (value['has_goal_yaw'], value['goal_yaw'], value['goal_yaw_tolerance_rad'])
        self.support_evidence = SupportEvidence(value['source_map_sha256'], value['execution_eligible'],
            value['eligibility_reason'], _immutable_array(
                value['geometry_evidence'].get('observed_source_support_xyz', [])).reshape((-1, 3)),
            canonical(value['geometry_evidence'].get('observed_source_support_xyz', [])))
        self._segment = 0
        self._measured = self._confirmed = 0.
        self._progress = None
        self._last_sample = None
        self._history = deque(maxlen=128)
        self.candidate_anchor = None
        # A portal is a measured event, not a timer state. A pending candidate
        # may occupy the publication slot while the body actually crosses it.
        self._portal_observation = None

    def __deepcopy__(self, memo):
        # Reanchor only stages dynamic progress. Geometry and literal source
        # evidence belong to the immutable route, not to an anchor version.
        staged = object.__new__(type(self))
        memo[id(self)] = staged
        shared = {'snapshot', '_points', '_arc', '_segments', 'goal_ground_xyz',
                  'goal_heading', 'support_evidence', 'portal_geometry'}
        for name, value in self.__dict__.items():
            setattr(staged, name, value if name in shared else deepcopy(value, memo))
        return staged

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
            # Pure body motion under the ACCEPTED rigid is not a correction.
            # An uncommitted correction, however, may need a retry after its
            # earlier projection/preparation failed. A stable transform does
            # not make this genuinely new same-time measured pair obsolete:
            # keeping the first candidate forever makes recovery impossible
            # once that pair ages out. Store the new candidate identity only;
            # admitted geometry and already published proposals stay literal.
            if self.candidate_anchor is None:
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
        self._fresh(observation.body.source_ns, observation.received_monotonic,
                    current_source_ns=current_source_ns, now_monotonic=now_monotonic)
        # Old-anchor correspondence can be precisely what a new localization
        # correction repairs. Requiring it to remain fresh makes recovery
        # circular; a successful native geometry commit may even have cleared
        # _progress after its old-anchor projection failed. Retain the last
        # established arc/cross-track only as a spatial search bound, never as
        # renewed measured progress. Both the correction and the new body are
        # still original, current observations, and the staged projection
        # below must independently pass the unchanged same-segment XYZ bounds.
        if (self._last_sample is None or observation.body.source_ns<self._last_sample[0]
                or not 0<=current_source_ns-candidate.source_ns<=self.freshness*1e9):
            raise ValueError('fresh_correction_and_established_correspondence_required')
        segment=self._segments[self._segment]
        edge=max(segment['begin_index'],min(segment['end_index']-1,
            int(np.searchsorted(self._arc,self._measured,side='right'))-1))
        length=self._arc[edge+1]-self._arc[edge]
        fraction=0. if length<=1e-9 else (self._measured-self._arc[edge])/length
        last_projection=self._points[edge]+fraction*(self._points[edge+1]-self._points[edge])
        last_cross_track=float(np.linalg.norm(np.asarray(self._last_sample[2])[:2]-last_projection[:2]))
        # An admitted finite local curve can keep moving while correspondence
        # to the OLD anchor is unavailable. Its last successful route arc is
        # then historical, not the body's current position. Recover the actual
        # odom displacement under that unchanged anchor before staging the new
        # one; a map correction itself must not count as physical travel.
        old_frame_body=np.asarray(self.anchor.map_from_odom.point(observation.body.pose.xyz))
        measured_delta=old_frame_body-np.asarray(self._last_sample[2])
        elapsed=(observation.body.source_ns-self._last_sample[0])*1e-9
        if ((elapsed==0. and np.linalg.norm(measured_delta)>1e-9)
                or np.linalg.norm(measured_delta)>self.max_speed*elapsed+.05):
            raise ValueError('measured_body_discontinuity')
        measured_forward=self._reanchor_forward_correspondence(
            old_frame_body,last_projection,edge,measured_delta)
        staged = deepcopy(self)
        staged.anchor, staged.candidate_anchor = candidate, None
        staged._last_sample = staged._progress = None
        staged._history.clear()
        staged._portal_observation = None
        # Keep the original correction radius around measured route motion,
        # not around an obsolete arc. Vertical drift on a flat corridor and
        # a pure lateral detour add no forward search allowance. The exact same
        # semantic segment, XYZ and ambiguity tests still select the candidate;
        # no old progress is renewed and admitted geometry remains unchanged.
        original_forward, original_lateral = staged.forward, staged.lateral
        staged.forward = min(staged.forward, staged.backward)+measured_forward
        # A correction may not move the body sideways by more than the
        # correspondence radius, even though a detour may already be off-line.
        staged.lateral = min(staged.lateral, last_cross_track+staged.radius)
        staged.project(observation, current_source_ns=current_source_ns,
                       now_monotonic=now_monotonic)
        staged.forward, staged.lateral = original_forward, original_lateral
        return staged

    def _reanchor_forward_correspondence(self, body, last_projection, edge, delta):
        """Find the body's ordered XY correspondence, not a velocity allowance.

        A single old edge tangent loses motion after a corner. Conversely,
        blindly using distance/elapsed time would count a lateral detour or Z
        drift as forward progress. Search only the current semantic segment's
        bounded forward prefix and compare old/new nearest XY points. Planar
        route length bounds this *search*, and the points supply their change
        in original 3-D arc (also on slopes). Comparing both bodies in the same
        XY correspondence avoids counting an old XYZ projection's slope
        residual as new physical travel. The new anchor still has to pass the
        ordinary full XYZ correspondence/ambiguity checks before any window
        can be proposed. This helper does not establish measured progress.
        """
        measured_planar=float(np.linalg.norm(delta[:2]))
        if measured_planar<=1e-9:
            return 0.
        remaining=measured_planar+min(self.forward,self.backward)
        previous=np.asarray(last_projection)
        previous_arc=float(self._measured)
        bodies=(body-delta,body)
        best_distances=[float(np.linalg.norm((p-previous)[:2])) for p in bodies]
        best_arcs=[previous_arc,previous_arc]
        end=self._segments[self._segment]['end_index']
        for i in range(edge,end):
            endpoint=self._points[i+1]
            vector=endpoint-previous
            planar=float(np.linalg.norm(vector[:2]))
            # Vertical-only edges cannot be justified by horizontal movement.
            # They retain the existing ordinary correspondence margin.
            if planar<=1e-9:
                if np.linalg.norm(vector)<=1e-9:
                    continue  # Duplicate source vertices add neither arc nor motion.
                break
            fraction=min(1.,remaining/planar)
            for j,current_body in enumerate(bodies):
                along=float(np.clip(np.dot((current_body-previous)[:2],vector[:2])/(planar*planar),0.,fraction))
                point=previous+along*vector
                distance=float(np.linalg.norm((current_body-point)[:2]))
                if distance<best_distances[j]-1e-9:
                    best_distances[j]=distance
                    best_arcs[j]=previous_arc+along*(self._arc[i+1]-previous_arc)
            remaining-=planar
            if remaining<=1e-9:
                break
            previous,previous_arc=endpoint,float(self._arc[i+1])
        return max(0.,best_arcs[1]-best_arcs[0])

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
        if self.execution_mode_policy == 'fixed_stair' and self._stair_boundary():
            end = segment['end_index']
            portal_body = self._points[end]+np.array((0., 0., self.body_height))
            if (self._adjacent_link(self._segment)
                    and ((self._arc[end]-measured <= self.portal_radius
                        and math.dist(source_body, portal_body) <= self.portal_radius)
                        or self._crossed_adjacent_corridor(source_body, measured))):
                self._portal_observation = self._progress
            elif (self._portal_observation is not None
                    and (measured < self._portal_observation.measured_arc_m-self.portal_radius
                        or self._crossed_adjacent_corridor(source_body, measured, reverse=True))):
                self._portal_observation = None  # A genuine retreat must cross again.
        self._history.append((body.source_ns, tuple(source_body)))
        self._last_sample = (body.source_ns, observation.received_monotonic, tuple(source_body))
        while len(self._history) > 3 and (body.source_ns-self._history[1][0])*1e-9 >= self.stationary_span:
            self._history.popleft()
        return self._progress

    def _crossed_adjacent_corridor(self, source_body, measured, *, reverse=False):
        """Recognize a rounded, measured crossing of one stair boundary.

        A portal is a semantic cross-section, not a compulsory centerline
        waypoint. SCAN can round a corner without entering the tiny portal
        sphere. Require the two real successive samples to cross the local
        incoming/outgoing bisector, with bounded XYZ correspondence on both
        sides. This never searches another segment/floor, renews a source
        time, or moves an admitted route. The ordinary flat/mode-transaction
        behavior remains unchanged.
        """
        if (self.execution_mode_policy!='fixed_stair' or not self._stair_boundary()
                or self._last_sample is None):
            return False
        old,new=self._segments[self._segment:self._segment+2]
        end=old['end_index']; arc=float(self._arc[end])
        if arc-measured>self.radius:
            return False
        portal=self._points[end]
        previous=np.asarray(self._last_sample[2])-np.array((0.,0.,self.body_height))
        current=np.asarray(source_body)-np.array((0.,0.,self.body_height))
        if reverse:previous,current=current,previous
        def point_at(distance):
            i=min(len(self._points)-2,max(0,int(np.searchsorted(self._arc,distance,side='right'))-1))
            t=(distance-self._arc[i])/max(1e-12,self._arc[i+1]-self._arc[i])
            return self._points[i]+t*(self._points[i+1]-self._points[i])

        before=point_at(max(float(self._arc[old['begin_index']]),arc-self.radius))
        after=point_at(min(float(self._arc[new['end_index']]),arc+self.radius))
        incoming,outgoing=(portal-before)[:2],(after-portal)[:2]
        # Vertical-only or folded-back endpoints have no unambiguous planar
        # section. They retain the original exact portal correspondence.
        lengths=np.linalg.norm(incoming),np.linalg.norm(outgoing)
        if min(lengths)<=1e-9:
            return False
        normal=incoming/lengths[0]+outgoing/lengths[1]
        if np.linalg.norm(normal)<=1e-6:
            return False
        if (np.dot((previous-portal)[:2],normal)>1e-9
                or np.dot((current-portal)[:2],normal)<=1e-9):
            return False

        if self.portal_geometry is not None and self._modeled_portal_crossing(
                previous,current,old,new,arc,portal,normal):
            return True
        # Keep the original literal-centre behaviour, including when a model
        # rejects the *additional* lateral correspondence. The model bounds
        # only that extension; this layer is not a replacement collision gate.
        # SCAN still owns walls/handrails and the complete robot envelope.
        if max(np.linalg.norm(previous-portal),np.linalg.norm(current-portal))>self.radius:
            return False

        # The forward sample must actually correspond to the literal adjacent
        # prefix, beyond its first point; crossing an infinite plane is not
        # enough. Limit the search to the same bounded neighbourhood.
        hi=min(float(self._arc[new['end_index']]),arc+self.radius)
        last=min(new['end_index'],int(np.searchsorted(self._arc,hi,side='right')))
        candidates=[]
        for i in range(new['begin_index'],last):
            length=self._arc[i+1]-self._arc[i]
            if length<=1e-9:continue
            delta=self._points[i+1]-self._points[i]
            t=float(np.clip(np.dot(current-self._points[i],delta)/(length*length),
                0.,min(1.,(hi-self._arc[i])/length)))
            p=self._points[i]+delta*t
            candidates.append((float(np.linalg.norm(current-p)),float(self._arc[i]+t*length)))
        if not candidates:
            return False
        distance,projected=min(candidates)
        return distance<=self.radius and projected>arc+1e-9

    def _modeled_portal_crossing(self, previous, current, old, new, arc, portal, normal):
        """Cross one finite adjacent section, not a compulsory centre point.

        Keep the existing vertical/along bounds and measured motion checks.
        Only the lateral detour already allowed by project() may be used here,
        and only inside the exact source model and adjacent semantic owners.
        """
        def correspondence(point,segment,lo,hi):
            candidates=[]
            first=max(segment['begin_index'],int(np.searchsorted(self._arc,lo,side='right'))-1)
            last=min(segment['end_index'],int(np.searchsorted(self._arc,hi,side='right')))
            for i in range(first,last):
                length=self._arc[i+1]-self._arc[i]
                if length<=1e-9:continue
                delta=self._points[i+1]-self._points[i]
                free=float(np.dot(point-self._points[i],delta)/(length*length))
                left=max(0.,(lo-self._arc[i])/length)
                right=min(1.,(hi-self._arc[i])/length)
                t=float(np.clip(free,left,right))
                residual=point-(self._points[i]+t*delta)
                candidates.append((float(np.linalg.norm(residual)),float(self._arc[i]+t*length),
                    float(np.linalg.norm(residual[:2])),abs(float(residual[2])),abs(free-t)*length))
            if not candidates:return None
            result=min(candidates)
            return result if (result[2]<=self.lateral and result[3]<=self.radius
                              and result[4]<=self.radius) else None

        before=correspondence(previous,old,max(float(self._arc[old['begin_index']]),arc-self.radius),arc)
        after=correspondence(current,new,arc,min(float(self._arc[new['end_index']]),arc+self.radius))
        if before is None or after is None or after[1]<=arc+1e-9:
            return False
        signed_before=float(np.dot((previous-portal)[:2],normal))
        signed_after=float(np.dot((current-portal)[:2],normal))
        fraction=-signed_before/(signed_after-signed_before)
        crossing=previous+fraction*(current-previous)
        geometry=self.portal_geometry
        if not geometry.contains(crossing,{old['segment_id'],new['segment_id']},
                height=self.body_height,vertical_bound=self.radius):
            return False
        # On a stair exit the next ordinary-floor prefix is deliberately not
        # extrapolated as an infinite stair model. Its original bounded XYZ
        # correspondence above, plus the finite portal section, owns the
        # semantic transition. SCAN still checks the actual floor connector.
        if new['required_mode']=='general' and new['kind'] in ('floor','landing'):
            return geometry.clear_column(current,height=self.body_height)
        return geometry.contains(current,{new['segment_id']},height=self.body_height,
            vertical_bound=self.radius)

    def _stationary(self):
        if len(self._history) < 3:
            return False
        start, end = self._history[0], self._history[-1]
        return ((end[0]-start[0])*1e-9 >= self.stationary_span
                and all(math.dist(p, end[1]) <= .02 for _, p in self._history))

    def adjacent_general_segment(self):
        """An ordinary same-floor boundary needs no stop or mode transaction.

        This is semantic classification only. The measured XYZ portal test in
        advance_segment still decides whether this particular source sample
        reached the boundary; a nearby XY on another floor is insufficient.
        """
        if self._segment+1 >= len(self._segments):
            return False
        old,new=self._segments[self._segment:self._segment+2]
        return (old['kind'] in ('floor','landing') and new['kind'] in ('floor','landing')
            and old['required_mode']==new['required_mode']=='general'
            and old['floor_id']==new['floor_id'] and bool(old['floor_id']))

    def reached_adjacent_portal(self, *, current_source_ns, now_monotonic):
        """Return literal adjacent semantics only after its measured XYZ portal."""
        p = self._require_progress(current_source_ns, now_monotonic)
        if self._segment+1 == len(self._segments):
            raise ValueError('already_final_semantic_segment')
        old, new = self._segments[self._segment:self._segment+2]
        end = old['end_index']
        portal_body = self._points[end]+np.array((0., 0., self.body_height))
        reached_now=math.dist(p.body_source_xyz, portal_body) <= self.portal_radius
        witnessed=self._portal_observation
        reached_before=(self.execution_mode_policy=='fixed_stair' and witnessed is not None
            and witnessed.segment_id==old['segment_id'] and witnessed.anchor_id==self.anchor.anchor_id
            and witnessed.source_ns<=p.source_ns
            and witnessed.received_monotonic<=p.received_monotonic
            and p.measured_arc_m>=witnessed.measured_arc_m
            and math.dist(p.body_source_xyz, portal_body)<=self.radius)
        if (not self._adjacent_link(self._segment)
                or (self._arc[end]-p.measured_arc_m > self.portal_radius and not reached_before)
                or not (reached_now or reached_before)):
            raise ValueError('adjacent_portal_not_physically_reached')
        return old,new,tuple(float(v) for v in portal_body)

    def requires_mode_transition(self):
        """Whether this boundary owns a mode transaction, not geometric reach."""
        return self.execution_mode_policy!='fixed_stair' and not self.adjacent_general_segment()

    def advance_segment(self, *, current_source_ns, now_monotonic, mode=None):
        """Explicit adjacent-portal transition, never nearest-neighbour relabeling."""
        old,new,_=self.reached_adjacent_portal(current_source_ns=current_source_ns,
            now_monotonic=now_monotonic)
        p=self._progress
        mode_matches = False
        if isinstance(mode,ReceiptModeConfirmation):
            mode_matches=(mode.context==self.context and mode.task_id==self.task_id
                and mode.route_hash==self.snapshot.route_hash and mode.segment_id==new['segment_id']
                and mode.mode==new['required_mode'] and bool(mode.sdk_session)
                and type(mode.callback_sequence) is int and mode.callback_sequence>0
                and mode.time_basis in ('sdk_callback_receipt_no_vendor_stamp','isolated_mock_callback_receipt')
                and 0<mode.callback_ns<=current_source_ns<mode.valid_until_ns
                and current_source_ns-mode.callback_ns<=2_500_000_000
                and 0<=now_monotonic-mode.received_monotonic<=2.5)
        elif isinstance(mode,PreviewSegmentTransition):
            mode_matches=(self.preview_only and mode.context==self.context and mode.task_id==self.task_id
                and mode.route_hash==self.snapshot.route_hash and mode.segment_id==new['segment_id'])
        elif mode is not None:
            self._fresh(mode.source_ns, mode.received_monotonic,
                        current_source_ns=current_source_ns, now_monotonic=now_monotonic)
            mode_matches = (mode.context == self.context and mode.task_id == self.task_id
                and mode.route_hash == self.snapshot.route_hash and mode.segment_id == new['segment_id']
                and mode.mode == new['required_mode'] and mode.source_ns >= p.source_ns)
        if self.requires_mode_transition() and not mode_matches:
            # Being stationary does not prove that the robot entered STAIR or
            # another required mode. Special boundaries remain explicit until
            # a real, target-segment-bound mode observation is available.
            raise ValueError('portal_requires_bound_mode_evidence')
        self._enter_adjacent_segment()
        return new['segment_id']

    def _enter_adjacent_segment(self):
        self._segment += 1
        self._history.clear()
        self._portal_observation = None
        # A new measured projection is required before a window from the new
        # segment can be issued. Old progress cannot pretend to own new mode.
        self._progress = None

    def _can_resume_witnessed_portal(self, observation):
        """A remembered crossing is not fresh progress or mode evidence.

        Used only when a delayed candidate outlived the incumbent segment's
        projection region. The latest sample must still obey the usual motion
        continuity bound; the successor then performs its own bounded XYZ
        projection. No old sample is re-stamped and no route segment is skipped.
        """
        witness=self._portal_observation
        if (self.execution_mode_policy!='fixed_stair' or witness is None
                or not self._adjacent_link(self._segment) or self._last_sample is None):
            return False
        body=observation.body
        current=self.anchor.map_from_odom.point(body.pose.xyz)
        old=self._segments[self._segment]
        last_ns,last_received,last_body=self._last_sample
        return (witness.task_id==self.task_id and witness.route_hash==self.snapshot.route_hash
            and witness.segment_id==old['segment_id'] and witness.anchor_id==self.anchor.anchor_id
            and body.source_ns>=last_ns>=witness.source_ns
            and observation.received_monotonic>=last_received>=witness.received_monotonic
            and (body.source_ns>last_ns or
                observation.received_monotonic==last_received and math.dist(current,last_body)<=1e-12)
            and math.dist(current,last_body)<=self.max_speed*(body.source_ns-last_ns)*1e-9+.05)

    def prepare_segment_transition(self, observation, *, current_source_ns,
                                   now_monotonic, mode=None):
        """Stage an adjacent segment without changing the admitted reference.

        Reproject the SAME literal measured sample on the adjacent segment.
        Its source/receipt are not renewed, and the incumbent remains intact
        until the local geometry owner commits this prepared snapshot.
        """
        self._fresh(observation.body.source_ns,observation.received_monotonic,
            current_source_ns=current_source_ns,now_monotonic=now_monotonic)
        source_xyz=self.anchor.map_from_odom.point(observation.body.pose.xyz)
        if (observation.body.context!=self.context or observation.body.frame!='d1max_loc_odom'
                or observation.body.child!='d1max_loc_base_link'):
            raise ValueError('segment_transition_requires_exact_measured_sample')
        progress=self._progress
        exact=(progress is not None and observation.body.source_ns==progress.source_ns
            and observation.received_monotonic==progress.received_monotonic
            and math.dist(source_xyz,progress.body_source_xyz)<=1e-12)
        witnessed=self._can_resume_witnessed_portal(observation)
        if not exact and not witnessed:
            raise ValueError('segment_transition_requires_exact_measured_sample')
        staged=deepcopy(self)
        if witnessed:
            # Candidate latency can outlast the small portal neighbourhood
            # even while projection on the former flat segment still works.
            # In either case use the same measured crossing and then require
            # a fresh bounded XYZ projection on the literal adjacent segment.
            staged._enter_adjacent_segment()
        else:
            staged.advance_segment(current_source_ns=current_source_ns,
                now_monotonic=now_monotonic,mode=mode)
        staged._last_sample=None
        staged.project(observation,current_source_ns=current_source_ns,now_monotonic=now_monotonic)
        return staged

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
        last_segment=self._window_last_segment(p.measured_arc_m+horizon_m)
        last_edge=self._segments[last_segment]['end_index']
        end = min(self._arc[last_edge], p.measured_arc_m+horizon_m)
        if end-p.measured_arc_m <= .01:
            raise ValueError('segment_endpoint_reached_requires_transition_or_arrival')
        count = max(2, int(math.ceil((end-p.measured_arc_m)/spacing_m))+1)
        samples = np.linspace(p.measured_arc_m, end, count)
        if last_segment != self._segment:
            # Preserve literal semantic boundaries in the future geometry.
            # This look-ahead is not a claim that the body crossed a portal.
            portals=[self._arc[s['end_index']] for s in self._segments[self._segment:last_segment]
                     if p.measured_arc_m < self._arc[s['end_index']] < end]
            samples=np.unique(np.r_[samples,portals])
        edges = np.clip(np.searchsorted(self._arc, samples, side='right')-1,
                        segment['begin_index'], last_edge-1)
        if last_segment!=self._segment and edges[0]>=segment['end_index']:
            # The first point is the shared literal portal. Keep its current
            # owner until the measured segment commits; subsequent points own
            # the adjacent terrain. This changes no coordinate or height.
            edges[0]=segment['end_index']-1
        points = []
        for s, i in zip(samples, edges):
            span = self._arc[i+1]-self._arc[i]
            t = (s-self._arc[i])/span if span > 1e-9 else 0.
            points.append(tuple(float(v) for v in self._points[i]+(self._points[i+1]-self._points[i])*t))
        inverse = self.anchor.map_from_odom.inverse()
        body = tuple(inverse.point((x, y, z+self.body_height)) for x, y, z in points)
        owners=tuple(next(s for s in self._segments[self._segment:last_segment+1]
                          if s['begin_index']<=int(edge)<s['end_index']) for edge in edges)
        return ReferenceWindow(self.context, self.task_id, self.snapshot.route_hash,
            self.anchor.anchor_id, self.anchor.revision, segment['segment_id'], segment['kind'],
            segment['required_mode'], p.source_ns, p.received_monotonic, tuple(points), body,
            tuple(int(i) for i in edges), end_arc_m=float(end),
            start_arc_m=float(p.measured_arc_m), issued_confirmed_arc_m=float(p.confirmed_arc_m),
            point_segment_ids=tuple(s['segment_id'] for s in owners),
            point_segment_kinds=tuple(s['kind'] for s in owners),
            point_required_modes=tuple(s['required_mode'] for s in owners))

    def _adjacent_link(self, index):
        if index+1 >= len(self._segments):
            return False
        old,new=self._segments[index:index+2]
        return (old['end_index']==new['begin_index'] and bool(old['exit_portal_id'])
                and old['exit_portal_id']==new['entry_portal_id'])

    def _stair_boundary(self):
        return (self._adjacent_link(self._segment)
            and any(s['required_mode']=='stair' for s in self._segments[self._segment:self._segment+2]))

    def _window_last_segment(self, desired_end):
        """Look through verified route adjacency, never relabel measured progress.

        Only fixed-mode windows which actually include stairs use this rule.
        Ordinary floor-only windows and the legacy mode transaction stay exact.
        """
        if self.execution_mode_policy!='fixed_stair':
            return self._segment
        index=self._segment
        if (self._segments[index]['kind'] not in ('floor','landing','stair_up','stair_down')
                or self._segments[index]['required_mode'] not in ('general','stair')):
            return index
        has_stair=self._segments[index]['required_mode']=='stair'
        while self._arc[self._segments[index]['end_index']] < desired_end-1e-9 and self._adjacent_link(index):
            following=self._segments[index+1]
            if (following['kind'] not in ('floor','landing','stair_up','stair_down')
                    or following['required_mode'] not in ('general','stair')):
                break
            index+=1
            has_stair=has_stair or following['required_mode']=='stair'
        return index if has_stair else self._segment
