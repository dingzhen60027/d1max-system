"""Wire-independent fixed-anchor continuous-odom reference transport.

No ROS node, SDK connection, executor, or publisher is created here. The formal
ReferenceNode uses the historical ``publish_staging`` callback to prepare a
typed ReferenceProposal; only the BT owner can commit that geometry. The old
staging topic remains available for isolated compatibility probes. A receipt
is neither trajectory collision acceptance nor physical execution authority.
"""
from copy import deepcopy
from dataclasses import dataclass
import math

from .control_frame_contract import BodySample, Context, Rigid, create_anchor
from .continuous_reference import ContinuousReference, Observation, validate_execution_mode_policy
from .route_ingress import RouteIngress
from .source_route_ros import from_message
from .atomic_projection_state import ATOMIC_OUTPUT_LEASE_NS, validate as validate_pair
from .rolling_reference_policy import WindowProgress

STAGING_REFERENCE_TOPIC = '/d1max/live_planning/bt/staging/reference_path'
STAGING_NATIVE_PARAMETERS = {'fsm.reference_path_z_offset': 0., 'grid_map.frame_id': 'd1max_loc_odom'}


def nanoseconds(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000 or stamp.sec < 0:
        raise ValueError('invalid_source_timestamp')
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def set_stamp(stamp, value):
    stamp.sec, stamp.nanosec = divmod(value, 1_000_000_000)


@dataclass(frozen=True)
class ReferenceReceipt:
    """An explicit native-reference receipt, never a trajectory safety permit."""
    session_id: str
    task_id: str
    route_id: str
    route_hash: str
    generation: int
    anchor_id: str
    context_sequence: int
    segment_id: str
    source_ns: int
    accepted: bool


class ContinuousReferenceTransport:
    def __init__(self, *, session_id, map_version_id, expected_hashes,
                 body_height_m, body_height_calibration_id, publish_staging,
                 freshness_s=.5, context_sequence=1, native_reference_path_z_offset=0.,
                 reference_horizon_m=4., preview_only=False, execution_mode_policy='segment_modes'):
        if (set(expected_hashes) != {'source_map_sha256', 'tomogram_sha256', 'conditioning_sha256'}
                or any(not isinstance(v, str) or len(v) != 64
                       or any(c not in '0123456789abcdef' for c in v) for v in expected_hashes.values())
                or type(context_sequence) is not int or context_sequence < 1
                or not callable(publish_staging) or not .02 <= freshness_s <= 2.
                or type(native_reference_path_z_offset) not in (int, float)
                or native_reference_path_z_offset != 0.
                or type(reference_horizon_m) not in (int,float)
                or not .5 <= reference_horizon_m <= 4.):
            raise ValueError('explicit_staging_provenance_and_transport_required')
        self.ingress = RouteIngress(session_id, map_version_id)
        self.expected_hashes = dict(expected_hashes)
        self.height, self.height_id, self.freshness = body_height_m, body_height_calibration_id, freshness_s
        self.publish = publish_staging
        self.reference_horizon_m = float(reference_horizon_m)
        if type(preview_only) is not bool:
            raise ValueError('explicit_preview_capability_required')
        self.preview_only=preview_only
        self.portal_geometry=None
        self.execution_mode_policy=validate_execution_mode_policy(execution_mode_policy)
        self.context_sequence = context_sequence
        self.core = self.route = self.snapshot = self.latest_pair = None
        self.latest_local = None
        self.pending = self.accepted = None
        self.pending_core = None
        self._pending_received_monotonic = None
        self._pending_end_arc_m = self._accepted_end_arc_m = None
        self._pending_start_arc_m = self._accepted_start_arc_m = None
        self._pending_issue_arc_m = self._accepted_issue_arc_m = None
        self.generation = 0
        self.quarantined = False
        self._reset_pair = None
        self._latest_pair_measurement_ns = self._reset_pair_measurement_ns = None
        self._retired_generation = 0
        self._retired_stamp_ns = 0
        self.status = dict(phase='waiting_route_and_navigation', execution_eligible=False,
                           physical_stop_confirmed=False)

    def _fresh(self, source, receipt, current, now):
        if (type(current) is not int or source <= 0 or current < source
                or (current-source)*1e-9 > self.freshness
                or not math.isfinite(receipt) or not math.isfinite(now)
                or receipt < 0 or not 0 <= now-receipt <= self.freshness):
            raise ValueError('staging_state_source_or_receipt_stale')

    def _fresh_pair_output(self, observation, current_source_ns, now_monotonic):
        self._fresh(observation.body.source_ns,observation.received_monotonic,
                    current_source_ns,now_monotonic)
        if current_source_ns-observation.body.source_ns > ATOMIC_OUTPUT_LEASE_NS:
            raise ValueError('staging_navigation_output_lease_expired')

    @property
    def empty_ownership(self):
        """No task geometry or outstanding retirement owned by this transport.

        Native publication is owned by the node's prepared ledger, not the
        staging callback. The recovery caller must check that separate fact.
        A generation counter is deliberately not an ownership proof.
        """
        return (self.ingress.active_identity is None
            and all(value is None for value in (self.route,self.snapshot,self.core,
                self.pending,self.pending_core,self.accepted))
            and self._retired_generation == 0 and self._retired_stamp_ns == 0)

    def _pair(self, message, *, received_monotonic, current_source_ns, now_monotonic):
        if (message.schema_version != 2 or not message.usable
                or message.session_id != self.ingress.session_id
                or message.map_version_id != self.ingress.map_version_id):
            raise ValueError('staging_navigation_identity_or_usability_invalid')
        # Use the same original output lease as the publisher/local inbox.
        # Raw posterior/IMU provenance is validated at output source, not aged
        # again here by delivery latency. Receipt and geometry remain literal.
        state=validate_pair(message,session_id=self.ingress.session_id,
            map_version_id=self.ingress.map_version_id,now_ns=current_source_ns)
        source=state.source_ns
        self._fresh(source,received_monotonic,current_source_ns,now_monotonic)
        context = Context(message.session_id, message.localization_epoch,
                          message.localization_seed_id, message.map_version_id)
        samples = []
        for odom, frame in ((message.global_odometry, 'd1max_loc_map'),
                            (message.local_odometry, 'd1max_loc_odom')):
            if (nanoseconds(odom.header.stamp) != source or odom.header.frame_id != frame
                    or odom.child_frame_id != 'd1max_loc_base_link'):
                raise ValueError('staging_navigation_pair_not_exact_same_source')
            p, q, t = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist
            if any(not math.isfinite(getattr(v, axis)) for v in (t.linear, t.angular) for axis in 'xyz'):
                raise ValueError('staging_navigation_twist_nonfinite')
            samples.append(Observation(BodySample(context, frame, odom.child_frame_id, source,
                Rigid((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))), received_monotonic))
        return tuple(samples)

    def on_route(self, message, *, current_source_ns, now_monotonic):
        if self.quarantined and message.active:
            raise ValueError('staging_context_quarantined')
        # Verify configured artifacts BEFORE RouteIngress mutates ownership.
        checked = from_message(message.snapshot)
        value = checked.payload()
        if any(value[k] != expected for k, expected in self.expected_hashes.items()):
            raise ValueError('staging_route_artifact_identity_mismatch')
        context = ((self.latest_pair[0].body.context.session, self.latest_pair[0].body.context.epoch,
                    self.latest_pair[0].body.context.seed) if self.latest_pair else
                   (message.session_id, message.snapshot.localization_epoch, message.snapshot.localization_seed_id))
        result = self.ingress.accept(message, now=current_source_ns*1e-9, context=context)
        if result is None:
            return None
        if not message.active:
            cancel = self._empty_reference(message)
            self.publish(deepcopy(cancel))
            if self.quarantined:
                self._retired_generation = cancel.generation
                self._retired_stamp_ns = nanoseconds(cancel.path.header.stamp)
            self.core = self.route = self.snapshot = self.pending = self.accepted = None
            self.pending_core = None
            self._pending_received_monotonic = None
            self._pending_end_arc_m = self._accepted_end_arc_m = None
            self._pending_start_arc_m = self._accepted_start_arc_m = None
            self._pending_issue_arc_m = self._accepted_issue_arc_m = None
            self.status.update(phase='reference_cancel_staged', physical_stop_confirmed=False)
            return cancel
        if self.route is not None:
            # A soft refresh is a new delivery, not a new route or anchor.
            self.status.update(last_delivery_sequence=message.delivery_sequence)
            return None
        self.route, self.snapshot = deepcopy(message), result
        self.status.update(phase='waiting_exact_navigation_pair', route_hash=result.route_hash)
        if self.latest_pair:
            return self._initialize(current_source_ns, now_monotonic)
        return None

    def on_navigation(self, message, *, received_monotonic, current_source_ns, now_monotonic):
        try:
            pair = self._pair(message, received_monotonic=received_monotonic,
                              current_source_ns=current_source_ns, now_monotonic=now_monotonic)
            measurement_ns = (nanoseconds(message.posterior_stamp),nanoseconds(message.imu_stamp))
            if self.latest_pair:
                if pair[0].body.context.epoch < self.latest_pair[0].body.context.epoch:
                    raise ValueError('staging_navigation_epoch_regressed')
                if (pair[0].body.context.epoch==self.latest_pair[0].body.context.epoch
                        and pair[0].body.source_ns <= self.latest_pair[0].body.source_ns):
                    return None  # late identities cannot resurrect an older frame context
            if self.quarantined:
                old = self.latest_pair[0].body.context if self.latest_pair else None
                previous = self._reset_pair
                if ((pair[0].body.context != old or self.empty_ownership)
                        and (previous is None or pair[0].body.context.epoch >= previous[0].body.context.epoch)
                        and (previous is None or pair[0].body.source_ns > previous[0].body.source_ns)):
                    self._reset_pair = pair
                    self._reset_pair_measurement_ns = measurement_ns
                raise ValueError('staging_context_quarantined')
            if self.core is not None and pair[0].body.context != self.core.context:
                self.quarantined = True
                self._reset_pair = pair
                self._reset_pair_measurement_ns = measurement_ns
                self.discard_pending()
                self.core._progress = None
                self.core.candidate_anchor = None
                raise ValueError('staging_localization_context_reset_requires_retirement')
            if (self.latest_pair and pair[0].body.context.epoch==self.latest_pair[0].body.context.epoch
                    and pair[0].body.source_ns <= self.latest_pair[0].body.source_ns):
                return None  # duplicate receipt must not renew source/receipt leases
            self.latest_pair = pair
            self._latest_pair_measurement_ns = measurement_ns
            # Global pairs establish candidate anchors. They must not move the
            # local progress backwards when the independent local stream leads.
            if self.latest_local is None or pair[1].body.source_ns>self.latest_local.body.source_ns:
                self.latest_local = pair[1]
            if self.route is None:
                self.status.update(phase='waiting_route')
                return None
            if self.core is None:
                return self._initialize(current_source_ns, now_monotonic)
            # Record an exact same-context correction before projecting with
            # the incumbent anchor. A failed old-anchor correspondence must
            # not prevent creation of the independent replacement candidate.
            # observe_pair does not move accepted geometry or task progress.
            revision = (self.core.candidate_anchor or self.core.anchor).revision+1
            self.core.observe_pair(*pair, revision=revision, current_source_ns=current_source_ns,
                                   now_monotonic=now_monotonic)
            self._project_pending_sample(pair[1],current_source_ns=current_source_ns,
                now_monotonic=now_monotonic)
            last=self.core._last_sample
            progress = (self.core.project(pair[1],current_source_ns=current_source_ns,
                now_monotonic=now_monotonic) if last is None or pair[1].body.source_ns>last[0]
                else self.core._progress)
            if progress is None:
                self.status.update(phase='committed_reference_waiting_measured_projection')
                return None
            self.status.update(phase='tracking_staged_reference' if self.accepted else 'waiting_reference_receipt',
                measured_arc_m=progress.measured_arc_m, confirmed_arc_m=progress.confirmed_arc_m,
                source_ns=progress.source_ns, anchor_id=progress.anchor_id,
                candidate_anchor_id=(self.core.candidate_anchor.anchor_id if self.core.candidate_anchor else ''))
            # No native reference publication on odometry or map correction.
            return progress
        except ValueError as error:
            self.status.update(phase='staging_input_unavailable', reason=str(error), execution_eligible=False)
            raise

    def on_local(self,message,*,received_monotonic,current_source_ns,now_monotonic):
        from .local_navigation_state import validate_local
        state=validate_local(message,session_id=self.ingress.session_id,
            map_version_id=self.ingress.map_version_id,now_ns=current_source_ns)
        context=Context(message.session_id,message.localization_epoch,message.localization_seed_id,message.map_version_id)
        p=state.local_body
        sample=Observation(BodySample(context,'d1max_loc_odom','d1max_loc_base_link',state.source_ns,
            Rigid(tuple(float(v) for v in p.position),tuple(float(v) for v in p.orientation))),received_monotonic)
        if self.latest_local and sample.body.source_ns<=self.latest_local.body.source_ns:
            return None
        if self.core and context!=self.core.context:
            self.quarantined=True
            self.discard_pending()
            raise ValueError('staging_localization_context_reset_requires_retirement')
        if self.quarantined:
            raise ValueError('staging_context_quarantined')
        # Gate the shared input before either geometry changes observation
        # history; stale receipt time is not a candidate projection failure.
        self._fresh(sample.body.source_ns,sample.received_monotonic,
            current_source_ns,now_monotonic)
        self.latest_local=sample
        if self.core is None:
            return None  # A local sample never fabricates a global anchor.
        self._project_pending_sample(sample,current_source_ns=current_source_ns,
            now_monotonic=now_monotonic)
        progress=self.core.project(sample,current_source_ns=current_source_ns,now_monotonic=now_monotonic)
        self.status.update(measured_arc_m=progress.measured_arc_m,confirmed_arc_m=progress.confirmed_arc_m,
            source_ns=progress.source_ns,anchor_id=progress.anchor_id)
        return progress

    def _project_pending_sample(self,sample,*,current_source_ns,now_monotonic):
        # Distinct incumbent/candidate geometry must each observe the same
        # literal sample. Losing old-segment correspondence must not hide a
        # real exit crossing from the pending segment. The incumbent still
        # projects independently afterwards and its exception is not masked.
        candidate=self.pending_core
        if candidate is not None and candidate is not self.core:
            last=candidate._last_sample
            if last is not None and sample.body.source_ns<=last[0]:
                return  # A delayed global pair cannot rewind/renew local evidence.
            try:
                candidate.project(sample,current_source_ns=current_source_ns,now_monotonic=now_monotonic)
            except ValueError as error:
                # A successor anchor is only a candidate. Its correspondence
                # failure cannot suppress the already-valid measured progress
                # of the immutable incumbent, nor retire the task/permit.
                rejected=(dict(generation=self.pending.generation,
                    task_id=self.pending.task_id,route_id=self.pending.route_id,
                    route_hash=self.pending.route_hash,anchor_id=self.pending.anchor_id,
                    anchor_revision=self.pending.anchor_revision,
                    source_ns=nanoseconds(self.pending.path.header.stamp))
                    if self.pending is not None else None)
                # A projection failure is not proof that the sole writer did
                # not select this prepared geometry. Keep its identity until
                # the owner lane retires it or commits the matching snapshot.
                candidate._progress = None
                self.status.update(phase='candidate_projection_rejected',
                    rejected_pending_generation=rejected['generation'] if rejected else 0,
                    rejected_pending_identity=rejected,pending_reason=str(error))

    def recover_empty_context(self, *, context_sequence, context, native_reference_unretired,
                              current_source_ns, now_monotonic):
        """Recover initialization after a hard event with no task to retire.

        Called only after the node matches the exact new context ACK and its
        prepared/publication ledger proves no native work remains. The saved
        pair retains all original acquisition and receipt times. This method
        neither republishes geometry nor resets delivery/retired-task fences.
        """
        if (not self.quarantined or not self.empty_ownership
                or native_reference_unretired is not False
                or type(context_sequence) is not int or context_sequence != self.context_sequence
                or self._reset_pair is None or self._reset_pair_measurement_ns is None):
            raise ValueError('empty_owner_and_new_context_ack_required')
        pair=self._reset_pair
        identity=pair[0].body.context
        if context != (identity.session,identity.epoch,identity.seed):
            raise ValueError('empty_context_ack_does_not_match_latest_navigation')
        for observation in pair:
            self._fresh_pair_output(observation,current_source_ns,now_monotonic)
        self.latest_pair=pair
        self._latest_pair_measurement_ns=self._reset_pair_measurement_ns
        self.latest_local=pair[1]
        self._reset_pair=self._reset_pair_measurement_ns=None
        self.quarantined=False
        self.status.update(phase='waiting_new_task_after_empty_context_reset',
                           execution_eligible=False,physical_stop_confirmed=False,reason='')
        return True

    def recover_retired_context(self, *, retired_generation, context_sequence, context,
                                current_source_ns, now_monotonic):
        """Clear quarantine after native retirement AND the new context ACK.

        This returns to an EMPTY task slot. It does not resume the retired
        route, restore a permit, or issue a reference/physical command.
        """
        if (not self.quarantined or self._reset_pair is None
                or self._retired_generation <= 0 or retired_generation != self._retired_generation
                or context_sequence != self.context_sequence or self.ingress.active_identity is not None
                or any(x is not None for x in (self.core,self.route,self.snapshot,self.pending,self.accepted))):
            raise ValueError('native_retirement_and_new_context_ack_required')
        identity = self._reset_pair[0].body.context
        if context != (identity.session,identity.epoch,identity.seed):
            raise ValueError('retired_context_ack_does_not_match_latest_navigation')
        for observation in self._reset_pair:
            self._fresh_pair_output(observation,current_source_ns,now_monotonic)
        self.latest_pair,self._reset_pair = self._reset_pair,None
        self._latest_pair_measurement_ns=self._reset_pair_measurement_ns
        self._reset_pair_measurement_ns=None
        self.latest_local=self.latest_pair[1]
        self.quarantined = False
        self._retired_generation = self._retired_stamp_ns = 0
        self.status.update(phase='waiting_new_task_after_context_reset',execution_eligible=False,
                           physical_stop_confirmed=False,reason='')
        return True

    def _initialize(self, current_source_ns, now_monotonic):
        global_, local = self.latest_pair
        context = global_.body.context
        wire = self.route.snapshot
        if (context.epoch != wire.localization_epoch or context.seed != wire.localization_seed_id
                or context.session != wire.session_id or context.map_version != wire.map_version_id):
            self.quarantined = True
            self._reset_pair = self.latest_pair
            self._reset_pair_measurement_ns=self._latest_pair_measurement_ns
            raise ValueError('staging_route_and_navigation_context_disagree')
        for sample in self.latest_pair:
            self._fresh_pair_output(sample,current_source_ns,now_monotonic)
        core = ContinuousReference(self.snapshot, context=context, task_id=self.route.task_id,
            anchor=create_anchor(global_.body, local.body, 1), body_height_m=self.height,
            body_height_calibration_id=self.height_id, freshness_s=self.freshness,
            preview_only=self.preview_only,execution_mode_policy=self.execution_mode_policy,
            portal_geometry=self.portal_geometry)
        core.project(local, current_source_ns=current_source_ns, now_monotonic=now_monotonic)
        self.core = core
        return self.propose_window(current_source_ns=current_source_ns, now_monotonic=now_monotonic)

    def _metadata(self, target):
        target.schema_version, target.session_id = 2, self.ingress.session_id
        target.point_reference = 'body_center'
        self.generation += 1
        target.generation = self.generation
        for name in ('task_id', 'route_id', 'route_hash', 'map_version_id',
                     'localization_epoch', 'localization_seed_id'):
            setattr(target, name, getattr(self.route.snapshot, name))
        target.context_sequence = self.context_sequence

    def request_segment_transition(self, *, current_source_ns, now_monotonic, mode=None):
        if self.quarantined or self.core is None or self.pending is not None or self.accepted is None:
            raise ValueError('staging_segment_transition_owner_not_ready')
        sample=self.latest_local or (self.latest_pair[1] if self.latest_pair else None)
        if sample is None:
            raise ValueError('segment_transition_requires_exact_measured_sample')
        staged=self.core.prepare_segment_transition(sample,current_source_ns=current_source_ns,
            now_monotonic=now_monotonic,mode=mode)
        output=self._publish_window(staged,current_source_ns=current_source_ns,now_monotonic=now_monotonic)
        self.status.update(phase='waiting_next_segment_geometry_commit',next_segment_id=output.segment_id)
        return output.segment_id

    def propose_window(self, *, current_source_ns, now_monotonic):
        if self.quarantined or self.core is None or self.pending is not None:
            raise ValueError('staging_window_owner_or_pending_candidate_invalid')
        # A spatial refresh must not turn every tiny global-pair correction
        # into an anchor transaction. Retain sub-threshold corrections in the
        # current core until the EXISTING correction_due batch is reached;
        # otherwise Native cannot recognize ordinary overlapping windows and
        # every moving curve races an unnecessary exact-predecessor replace.
        staged = (self.core.prepare_reanchor(self.latest_local or self.latest_pair[1],
            current_source_ns=current_source_ns, now_monotonic=now_monotonic)
            if self.correction_ready(current_source_ns=current_source_ns) else self.core)
        return self._publish_window(staged,current_source_ns=current_source_ns,now_monotonic=now_monotonic)

    def _publish_window(self,staged,*,current_source_ns,now_monotonic):
        from d1max_planning_interfaces.msg import ReferencePath
        from geometry_msgs.msg import PoseStamped
        window = staged.window(current_source_ns=current_source_ns, now_monotonic=now_monotonic,
                               horizon_m=self.reference_horizon_m)
        output = ReferencePath()
        self._metadata(output)
        output.anchor_id, output.anchor_revision = window.anchor_id, window.anchor_revision
        output.map_geometry_revision = window.anchor_revision
        output.segment_id, output.segment_kind, output.required_mode = (
            window.segment_id, window.segment_kind, window.required_mode)
        output.path.header.frame_id = window.frame_id
        set_stamp(output.path.header.stamp, window.source_ns)
        for xyz in window.body_odom_xyz:
            pose = PoseStamped()
            pose.header = deepcopy(output.path.header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            pose.pose.orientation.w = 1.
            output.path.poses.append(pose)
        output.point_segment_ids = list(window.point_segment_ids)
        output.point_segment_kinds = list(window.point_segment_kinds)
        output.point_required_modes = list(window.point_required_modes)
        self.pending = deepcopy(output)
        self.pending_core = staged
        self._pending_received_monotonic = window.received_monotonic
        self._pending_end_arc_m = window.end_arc_m
        self._pending_start_arc_m = window.start_arc_m
        self._pending_issue_arc_m = window.issued_confirmed_arc_m
        self.publish(deepcopy(output))
        self.status.update(phase='waiting_reference_receipt', pending_generation=output.generation,
            anchor_id=window.anchor_id, source_ns=window.source_ns)
        return output

    def correction_due(self):
        """Batch small corrections; never mutate an accepted anchor in place."""
        if self.core is None or self.core.candidate_anchor is None:
            return False
        a, b = self.core.anchor.map_from_odom, self.core.candidate_anchor.map_from_odom
        cosine = min(1., abs(sum(x*y for x, y in zip(a.xyzw, b.xyzw))))
        return math.dist(a.xyz, b.xyz) >= .05 or 2.*math.acos(cosine) >= .02

    def correction_ready(self, *, current_source_ns):
        """A correction transaction is separate from rolling the odom window.

        An expired optional global pair cannot be applied, but it must not
        block extending valid incumbent geometry from fresh local progress.
        Keep the candidate literal for diagnostics; only a genuinely new
        global/local pair can renew it. _publish_window still checks measured
        progress freshness, context and semantic bounds. A future candidate
        is deliberately NOT hidden here: prepare_reanchor must reject it.
        """
        if not self.correction_due():
            return False
        return current_source_ns-self.core.candidate_anchor.source_ns <= self.freshness*1e9

    def covers_current_segment_end(self):
        """Geometry-only coverage, not authorization or a collision lease."""
        if self.core is None or self.accepted is None or not self.accepted.path.poses:
            return False
        segment=self.core._segments[self.core._segment]
        if self.accepted.segment_id!=segment['segment_id']:
            return False
        end=self.core._points[segment['end_index']]
        expected=self.core.anchor.map_from_odom.inverse().point(tuple(float(v) for v in
            (end[0],end[1],end[2]+self.height)))
        p=self.accepted.path.poses[-1].pose.position
        return math.dist((p.x,p.y,p.z),expected)<=1e-6

    def remaining_window_arc_m(self):
        """Route arc left in the accepted window, measured from confirmed progress.

        Independent of when the owner committed the window, so a late commit
        cannot consume the look-ahead that triggers the next window.
        """
        if (self.core is None or self.accepted is None or self._accepted_end_arc_m is None
                or self.core._progress is None):
            return None
        return self._accepted_end_arc_m-self.core._progress.confirmed_arc_m

    def rolling_window_progress(self):
        """Original geometry bounds and live progress; no commit-time reset."""
        if (self.core is None or self.accepted is None or self.core._progress is None
                or any(v is None for v in (self._accepted_start_arc_m,
                    self._accepted_end_arc_m,self._accepted_issue_arc_m))):
            return None
        return WindowProgress(self._accepted_start_arc_m,self._accepted_end_arc_m,
            self._accepted_issue_arc_m,self.core._progress.confirmed_arc_m)

    def accepted_route_progress(self, *, current_source_ns, now_monotonic):
        """Return source-bound facts from the committed core, never pending.

        This is task supervision only. It supplies no control/collision lease
        and never refreshes the source time of a previous local observation.
        """
        if self.quarantined or self.core is None or self.accepted is None:
            return None
        core, reference = self.core, self.accepted
        progress = core._require_progress(current_source_ns, now_monotonic)
        local = self.latest_local
        if (local is None or local.body.source_ns != progress.source_ns
                or local.body.context != core.context
                or reference.anchor_id != core.anchor.anchor_id
                or reference.anchor_revision != core.anchor.revision
                or reference.segment_id != progress.segment_id
                or progress.anchor_id != core.anchor.anchor_id):
            return None
        self._fresh(local.body.source_ns, local.received_monotonic,
                    current_source_ns, now_monotonic)
        return reference, progress, local.body, core.anchor, core.body_height

    def discard_pending(self):
        self.pending = self.pending_core = None
        self._pending_received_monotonic = None
        self._pending_end_arc_m = None
        self._pending_start_arc_m = self._pending_issue_arc_m = None

    def commit_pending(self, *, current_source_ns, now_monotonic, owner_committed=False):
        """Called only after the formal owner commits this exact prepared version."""
        if self.pending is None or self.pending_core is None or self.quarantined:
            raise ValueError('no_prepared_reference_to_commit')
        accepted=self._install_committed_window(self.pending,self.pending_core,
            received_monotonic=self._pending_received_monotonic,
            end_arc_m=self._pending_end_arc_m,start_arc_m=self._pending_start_arc_m,
            issue_arc_m=self._pending_issue_arc_m,current_source_ns=current_source_ns,
            now_monotonic=now_monotonic,owner_committed=owner_committed)
        self.discard_pending()
        return accepted

    def _install_committed_window(self,reference,core,*,received_monotonic,
                                  end_arc_m,start_arc_m,issue_arc_m,
                                  current_source_ns,now_monotonic,owner_committed=False):
        """Adopt the caller-verified owner fact, independent of the pending slot.

        The node first joins the exact native receipt, original validation and
        successful commit. A delayed fact may describe an older prepared
        window while a successor is already preparing or awaiting its receipt.
        Installing accepted geometry must not retire that successor. This
        internal helper does not authorize motion or renew an input/proof.
        """
        if reference is None or core is None or self.quarantined:
            raise ValueError('no_prepared_reference_to_commit')
        progress_error = ''
        if owner_committed:
            # A reference is immutable geometry, not a collision lease. Once
            # native accepted its delivery, a fresh owner commit may arrive
            # after that delivery deadline. Require CURRENT source-timed body
            # evidence; do not renew/rewrite the original proposal stamp.
            local=self.latest_local or (self.latest_pair[1] if self.latest_pair else None)
            if local is not None and local.body.context != core.context:
                raise ValueError('committed_reference_current_context_missing')
            try:
                if local is None:
                    raise ValueError('committed_reference_current_body_missing')
                self._fresh(local.body.source_ns,local.received_monotonic,current_source_ns,now_monotonic)
                last = core._last_sample
                if last is None or local.body.source_ns > last[0]:
                    core.project(local,current_source_ns=current_source_ns,now_monotonic=now_monotonic)
                core._require_progress(current_source_ns, now_monotonic)
            except ValueError as error:
                # A BT geometry commit is already a fact. Failing current
                # measured projection cannot roll it back to an old anchor.
                # Adopt geometry but emit no RouteProgress until a new real
                # sample successfully projects; this supplies no motion lease.
                progress_error = str(error)
                core._progress = None
        else:
            self._fresh(nanoseconds(reference.path.header.stamp), received_monotonic,
                        current_source_ns, now_monotonic)
            core._require_progress(current_source_ns, now_monotonic)
        self.core, self.accepted = core, deepcopy(reference)
        self._accepted_end_arc_m = end_arc_m
        self._accepted_start_arc_m = start_arc_m
        self._accepted_issue_arc_m = issue_arc_m
        self.status.update(phase='reference_receipt_acknowledged',
            accepted_generation=self.accepted.generation, anchor_id=self.core.anchor.anchor_id,
            committed_progress_reason=progress_error)
        return self.accepted

    def pending_window_remaining(self):
        if self.pending_core is None or self.pending_core._progress is None or self._pending_end_arc_m is None:
            return None
        return self._pending_end_arc_m-self.pending_core._progress.confirmed_arc_m

    def on_reference_receipt(self, receipt, *, current_source_ns, now_monotonic):
        """Acknowledge reference ownership only; native collision proof is separate."""
        if self.pending is None or self.core is None or self.quarantined:
            return False
        pending = self.pending
        if (not isinstance(receipt, ReferenceReceipt) or type(receipt.accepted) is not bool
                or any(getattr(receipt, field) != getattr(pending, field) for field in (
                    'session_id', 'task_id', 'route_id', 'route_hash', 'generation', 'anchor_id',
                    'context_sequence', 'segment_id'))
                or receipt.source_ns != nanoseconds(pending.path.header.stamp)):
            return False
        try:
            self._fresh(receipt.source_ns, self._pending_received_monotonic,
                        current_source_ns, now_monotonic)
            self.core._require_progress(current_source_ns, now_monotonic)
        except ValueError:
            self.discard_pending()
            self.status.update(phase='candidate_reference_receipt_expired', execution_eligible=False)
            return False
        if receipt.accepted:
            self.commit_pending(current_source_ns=current_source_ns, now_monotonic=now_monotonic)
        else:
            self.status.update(phase='candidate_reference_rejected')
            self.discard_pending()
        return True

    def _empty_reference(self, message):
        from d1max_planning_interfaces.msg import ReferencePath
        output = ReferencePath()
        self._metadata(output)
        output.path.header.frame_id = 'd1max_loc_odom'
        output.path.header.stamp = deepcopy(message.source_stamp)
        if self.core is not None:
            output.anchor_id, output.anchor_revision = self.core.anchor.anchor_id, self.core.anchor.revision
            output.map_geometry_revision = self.core.anchor.revision
        return output
