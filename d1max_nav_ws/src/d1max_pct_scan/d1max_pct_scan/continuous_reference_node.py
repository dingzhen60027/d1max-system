"""Production odom reference transport; a single local owner commits geometry.

Callbacks are separately constructible without rclpy.init or a ROS graph.
First and successor windows use the same prepare/receipt/permit protocol.
"""
from copy import deepcopy
from collections import OrderedDict
from dataclasses import dataclass, field
import hashlib
import json
import math
import time
import numpy as np

from .atomic_projection_state import validate as validate_state
from .atomic_navigation_inbox import navigation_event_is_new
from .continuous_reference_transport import ContinuousReferenceTransport, ReferenceReceipt, nanoseconds, set_stamp
from .execution_safety import version as execution_version_identity
from .source_route import canonical
from .rolling_reference_policy import RollingReferencePolicy

PREFIX='/d1max/live_planning/'


@dataclass
class PreparedWindow:
    """Bounded immutable geometry facts, distinct from the current proposal slot."""
    proposal: object
    core: object
    received: float
    end_arc_m: float
    start_arc_m: float
    issued_confirmed_arc_m: float
    delivered: bool = False
    receipt: object = None
    support: object = None
    support_request: object = None
    support_preparing: bool = False
    last_support_preparation_monotonic: float = -math.inf
    support_sent: bool = False
    support_inflight: bool = False
    last_support_delivery_monotonic: float = -math.inf
    last_delivery_monotonic: float = -math.inf
    retirement_requested: bool = False
    # A claim is undecided until the sole writer publishes its exact CAS
    # outcome. Deadline/revocation alone never proves it was not applied.
    handoffs: dict = field(default_factory=dict)
    acknowledgements: dict = field(default_factory=dict)
    reconciled_handoffs: set = field(default_factory=set)
    closed_handoff_sequence: dict = field(default_factory=dict)
    installed: bool = False
    # Real observations in this exact candidate's frozen coordinate anchor.
    # Separate from immutable preparation geometry and never borrowed from a
    # different anchor or task when an owner acknowledgement arrives late.
    observed_core: object = None


def execution_binding(permit):
    return (permit.execution_id,permit.control_epoch,permit.sdk_session,
            permit.sdk_arm_generation,permit.transport_mode)


def version_of(reference):
    from d1max_planning_interfaces.msg import ExecutionVersion
    version=ExecutionVersion(schema_version=3)
    for name in ('session_id','task_id','route_id','route_hash','map_version_id','localization_epoch',
                 'localization_seed_id','segment_id','anchor_id','anchor_revision','context_sequence','map_geometry_revision'):
        setattr(version,name,getattr(reference,name))
    version.reference_generation=reference.generation
    return version


@dataclass(frozen=True)
class CommittedTrajectoryStatus:
    valid: bool
    plan_id: int
    phase: str


def candidate_preparation_status(*, reference, proposal, receipt, proof, proof_received,
                                 context_sequence, quarantined, now_ns,
                                 now_monotonic, freshness_s, transport_mode, stateful=False):
    """Diagnostic of native candidate preparation, NOT a commit or permission.

    The accepted receipt is an immutable geometry event at its original source
    time. Only a new native whole-curve proof can describe current readiness;
    neither this result nor JSON status participates in the owner's typed CAS.
    """
    waiting=CommittedTrajectoryStatus(False,-1,'waiting_candidate_validation')
    if (quarantined or reference is None or proposal is None or receipt is None
            or proposal.version!=version_of(reference) or receipt.version!=proposal.version
            or receipt.proposal_id!=proposal.proposal_id or not receipt.accepted
            or receipt.expected_version!=proposal.expected_version
            or receipt.expected_trajectory_id!=proposal.expected_trajectory_id
            or proposal.transport_mode!=transport_mode or receipt.transport_mode!=transport_mode
            or context_sequence<=0 or reference.context_sequence!=context_sequence
            or not 0<nanoseconds(proposal.source_stamp)<=nanoseconds(receipt.source_stamp)<=now_ns
            or nanoseconds(receipt.source_stamp)>nanoseconds(proposal.valid_until)):
        return waiting
    if proof is None:
        return waiting
    plan_id=proof.trajectory_id
    waiting=CommittedTrajectoryStatus(False,plan_id,'waiting_candidate_validation')
    if (proof.version!=proposal.version or proof.proposal_id!=proposal.proposal_id
            or plan_id<0 or proof.sequence<=0 or proof.frame_id!='d1max_loc_odom'
            or proof.transport_mode!=transport_mode
            or nanoseconds(proof.check_end)<nanoseconds(reference.path.header.stamp)):
        return waiting
    if not proof.valid:
        return CommittedTrajectoryStatus(False,plan_id,proof.reason or 'failed_candidate_validation')
    begin,end=nanoseconds(proof.check_begin),nanoseconds(proof.check_end)
    domain=(proof.checked_from_time,proof.valid_start_time,proof.checked_to_time,proof.curve_duration)
    if (not proof.whole_curve or proof.remaining_curve or not proof.support_reference_id
            or not proof.support_hash or not all(math.isfinite(value) for value in domain)
            or proof.checked_from_time!=0. or not 0.<=proof.valid_start_time<=proof.checked_to_time
            or proof.curve_duration<=0. or abs(proof.checked_to_time-proof.curve_duration)>1e-6
            or not 0<begin<=end<=now_ns
            or nanoseconds(proof.source_stamp)!=end
            or (nanoseconds(proof.valid_until)!=0 if stateful else
                not (now_ns<nanoseconds(proof.valid_until)
                    and now_ns-end<=round(freshness_s*1e9)
                    and 0<=now_monotonic-proof_received<=freshness_s))):
        return waiting
    return CommittedTrajectoryStatus(True,plan_id,'candidate_ready_waiting_owner_commit')


def committed_trajectory_status(*, reference, permit, proof, proof_received,
                                context_sequence, quarantined, now_ns,
                                now_monotonic, freshness_s, transport_mode):
    """Select the exact committed curve using native proof, never debug traffic.

    The proof inbox already orders positive/negative facts per full version and
    curve. This read-only policy consumes the latest fact with its ORIGINAL
    deadline/receipt; it neither refreshes evidence nor grants motion.
    """
    waiting=CommittedTrajectoryStatus(False,-1,'waiting_native_trajectory')
    if (quarantined or reference is None or permit is None
            or not permit.geometry_committed or permit.version!=version_of(reference)
            or permit.transport_mode!=transport_mode
            or permit.trajectory_id<0 or context_sequence<=0
            or reference.context_sequence!=context_sequence):
        return waiting
    plan_id=permit.trajectory_id
    if (permit.revoked or not 0<nanoseconds(permit.source_stamp)<=now_ns
            or now_ns>=nanoseconds(permit.valid_until)):
        return CommittedTrajectoryStatus(False,plan_id,'waiting_current_permission')
    return current_geometry_proof_status(reference=reference,version=permit.version,plan_id=plan_id,
        validation_floor=permit.validation_sequence,proof=proof,proof_received=proof_received,
        now_ns=now_ns,now_monotonic=now_monotonic,freshness_s=freshness_s,transport_mode=transport_mode)


def current_geometry_proof_status(*,reference,version,plan_id,validation_floor,proof,proof_received,
                                  now_ns,now_monotonic,freshness_s,transport_mode,stateful=False):
    if (proof is None or proof.version!=version or proof.trajectory_id!=plan_id
            or proof.frame_id!='d1max_loc_odom' or proof.transport_mode!=transport_mode
            or proof.sequence<=0 or proof.sequence<validation_floor
            or nanoseconds(proof.check_end)<nanoseconds(reference.path.header.stamp)):
        return CommittedTrajectoryStatus(False,plan_id,'waiting_current_validation')
    if not proof.valid:
        return CommittedTrajectoryStatus(False,plan_id,proof.reason or 'failed_current_validation')
    begin,end=nanoseconds(proof.check_begin),nanoseconds(proof.check_end)
    if (not 0<begin<=end<=now_ns
            or nanoseconds(proof.source_stamp)!=end
            or (nanoseconds(proof.valid_until)!=0 if stateful else
                not (now_ns<nanoseconds(proof.valid_until)
                    and now_ns-end<=round(freshness_s*1e9)
                    and 0<=now_monotonic-proof_received<=freshness_s))):
        return CommittedTrajectoryStatus(False,plan_id,'waiting_current_validation')
    return CommittedTrajectoryStatus(True,plan_id,'accepted')


def local_committed_trajectory_status(*,reference,commit,proof,proof_received,context_sequence,
                                     quarantined,now_ns,now_monotonic,freshness_s,transport_mode):
    """Display SCAN's ordered collision state, not a timed motion permission.

    The local executor owns actual collision and pipeline-failure transitions.
    An observer must not invent another transition when a map is being updated.
    """
    if (quarantined or reference is None or commit is None or not commit.accepted or commit.retired
            or commit.version!=version_of(reference) or commit.transport_mode!=transport_mode
            or context_sequence<=0 or reference.context_sequence!=context_sequence
            or commit.commit_sequence<=0 or commit.trajectory_id<0
            or not 0<nanoseconds(commit.source_stamp)<=now_ns):
        return CommittedTrajectoryStatus(False,-1,'waiting_native_trajectory')
    return current_geometry_proof_status(reference=reference,version=commit.version,plan_id=commit.trajectory_id,
        validation_floor=commit.native_validation_sequence,proof=proof,proof_received=proof_received,
        now_ns=now_ns,now_monotonic=now_monotonic,freshness_s=freshness_s,transport_mode=transport_mode,
        stateful=True)


class ReferenceCallbacks:
    def __init__(self,params,publish,now_ns,monotonic=time.monotonic,diagnostic=None,
                 preparation_ready=None):
        self.p,self.publish,self.now_ns,self.mono=params,publish,now_ns,monotonic
        self.diagnostic=diagnostic
        self._diagnostic_states={}
        if params['transport_mode'] not in ('live','isolated_mock'):
            raise ValueError('explicit_reference_transport_mode_required')
        self.geometry_owner=params.get('geometry_owner','bt_writer')
        self.execution_purpose=params.get('execution_purpose','execution')
        if self.execution_purpose not in ('execution','planning_only'):
            raise ValueError('explicit_reference_execution_purpose_required')
        from .continuous_reference import validate_execution_mode_policy
        self.execution_mode_policy=validate_execution_mode_policy(params.get('execution_mode_policy','segment_modes'))
        if self.geometry_owner not in ('bt_writer','local_executor'):
            raise ValueError('explicit_reference_geometry_owner_required')
        self.map_evidence_age_s=float(params.get('map_evidence_max_age_s',.4))
        # Match Native's official projected-ray ceiling. This is only the
        # configured map SOURCE budget, not the body/receipt/proof freshness.
        if not math.isfinite(self.map_evidence_age_s) or not 0<self.map_evidence_age_s<=.6:
            raise ValueError('map_evidence_age_exceeds_native_contract')
        self.map_evidence_age_ns=round(self.map_evidence_age_s*1e9)
        freshness=params.get('freshness_s',.4)
        # One consumer budget for body, delivery and diagnostic admission.
        # It may be stricter than the wire contract, never exceed its 400 ms
        # maximum or borrow the independent raw-map evidence allowance.
        if (type(freshness) not in (int,float) or not math.isfinite(freshness)
                or not .02<=freshness<=.4):
            raise ValueError('reference_freshness_exceeds_control_contract')
        self.transport=ContinuousReferenceTransport(session_id=params['session_id'],
            map_version_id=params['map_version_id'],expected_hashes={k:params['expected_'+k]
                for k in ('source_map_sha256','tomogram_sha256','conditioning_sha256')},
            body_height_m=params['body_height_m'],body_height_calibration_id=params['body_height_calibration_id'],
            publish_staging=self.prepare,freshness_s=freshness,
            reference_horizon_m=params.get('reference_horizon_m',2.),
            preview_only=self.execution_purpose=='planning_only',
            execution_mode_policy=self.execution_mode_policy)
        from .segment_transition import SegmentTransitionInbox
        self.mode_transition=SegmentTransitionInbox(params['transport_mode'])
        self.task_lease=None
        self.mode_request_sequence=0
        self.mode_request_last_sent=-math.inf
        self.freshness_ns=round(self.transport.freshness*1e9)
        self.context=None; self.context_ready=False; self.map_report=None; self.map_received=-math.inf
        self.state=None; self.state_received=-math.inf
        from .local_navigation_state import LocalNavigationInbox
        self.local_inbox=LocalNavigationInbox()
        self.proposal=None; self.receipt=None; self.permit=None
        self.prepared={}
        self.prepared_limit=8
        self.early_acks={}
        self.deferred_permit=None
        self.local_commit=None
        self.deferred_local_commit=None
        self.deferred_local_commits={}
        self.local_commit_fact_limit=16
        self._reconciling_local_commits=False
        self.debug=None; self.debug_received=-math.inf
        # Native sends committed proof and candidate outcomes on the same
        # topic. Preserve each curve's literal fact; receipt of a candidate
        # outcome must neither replace nor renew the committed curve proof.
        self.debug_facts={}
        self.validation_facts={}
        # A commit refers to the Native verdict valid at its ORIGINAL event
        # time. Later checks (including negatives) must still own live
        # readiness, without erasing that bounded software-admission history.
        self.local_commit_proofs={}
        self.rolling_policy=RollingReferencePolicy()
        self.context_last_sent=-math.inf
        self.last_proposal_monotonic=-math.inf
        self.error=''; self.cancel_generation=0
        self.native_retirement=None
        # Publication is a sticky fact. Clearing a prepared candidate during a
        # reset cannot prove a reliable native delivery was never in flight.
        self.native_route_exposures=set()
        self.pending_route_retirement=None
        self.route_dispositions=OrderedDict()
        self.context_reset_reason=''
        self.context_reset_source_ns=0
        self.support_map=None
        self.route_progress_sequence=0
        self.route_progress_key=None
        if params.get('planning_manifest'):
            from .reference_support import load_reference_support
            self.support_map=load_reference_support(params['planning_manifest'],
                **{name:params[name] for name in ('expected_source_map_sha256',
                    'expected_conditioning_sha256','expected_tomogram_sha256')})
            model=getattr(self.support_map,'stair_model',None)
            if model is not None:
                from .reference_portal_geometry import ReferencePortalGeometry
                self.transport.portal_geometry=ReferencePortalGeometry.from_model(model)
        self.preparation=None
        self.support_delivery=None
        if params.get('async_support_preparation',False):
            from .bounded_preparation import LatestPreparation
            from .bounded_support_delivery import BoundedSupportDelivery
            self.preparation=LatestPreparation(self.build_support,notify_ready=preparation_ready)
            try:
                self.support_delivery=BoundedSupportDelivery(
                    lambda support:self.publish('support',support),notify_ready=preparation_ready)
            except BaseException:
                self.preparation.close()
                raise

    def close(self):
        prepared=self.preparation is None or self.preparation.close()
        delivered=self.support_delivery is None or self.support_delivery.close()
        return prepared and delivered

    def invalidate_preparation(self):
        if self.preparation is not None:
            self.preparation.invalidate()
        if self.support_delivery is not None:
            self.support_delivery.invalidate()
        for record in self.prepared.values():
            record.support_request=None
            record.support_preparing=False

    def clear_prepared(self):
        # Called only on task cancellation or a hard localization identity
        # change, never on a candidate timeout or ordinary map correction.
        self.prepared.clear(); self.early_acks.clear(); self.deferred_permit=None
        self.local_commit=self.deferred_local_commit=None
        self.deferred_local_commits.clear()
        self.local_commit_proofs.clear()
        self.mode_transition.clear()
        self.task_lease=None

    def trace(self,event,**fields):
        """Bounded, change-triggered diagnostics; never a geometry/source fact."""
        if self.diagnostic is None:
            return
        now=self.mono()
        # Counters and identities can change at control rate. Report at most
        # twice per second per fixed event kind; retain the latest on next tick.
        previous=self._diagnostic_states.get(event)
        if previous is not None and (fields==previous[1] or now-previous[0]<.5):
            return
        self._diagnostic_states[event]=(now,deepcopy(fields))
        self.diagnostic('ReferenceFlow '+json.dumps(dict(event=event,**fields),sort_keys=True))

    def geometry_owner_fact(self):
        if self.geometry_owner=='local_executor':
            return self.local_commit if self.local_commit is not None and self.local_commit.accepted and not self.local_commit.retired else None
        return self.permit if self.permit is not None and self.permit.geometry_committed and not self.permit.revoked else None

    def clear_native_debug(self):
        self.debug=None; self.debug_received=-math.inf; self.debug_facts.clear()
        self.validation_facts.clear()
        self.local_commit_proofs.clear()

    def committed_debug_key(self):
        reference=self.transport.accepted
        owner=self.geometry_owner_fact()
        if reference is None or owner is None or owner.version!=version_of(reference):
            return None
        return reference.generation,owner.trajectory_id

    def native_debug_fresh(self,message,received):
        return bool(message and 0<=self.mono()-received<=self.transport.freshness
            and 0<=self.now_ns()-nanoseconds(message.header.stamp)<=self.freshness_ns)

    def committed_validation_key(self):
        owner=self.geometry_owner_fact()
        return ((execution_version_identity(owner.version),owner.trajectory_id)
            if self.committed_debug_key() is not None else None)

    def pending_validation_fact(self):
        pending=self.transport.pending
        if pending is None or self.proposal is None:
            return None,-math.inf
        # Select the literal latest curve, including its negative result. An
        # earlier positive curve must not make a newer rejected candidate ready.
        facts=[fact for fact in self.validation_facts.values()
            if fact[0].version==version_of(pending)
            and fact[0].proposal_id==self.proposal.proposal_id]
        return max(facts,key=lambda fact:(fact[0].trajectory_id,fact[0].sequence),
            default=(None,-math.inf))

    def on_validation(self,message):
        """Observe SCAN decisions; never perform or time out collision checks."""
        # A retired localization/task context has no live decision inbox.
        # Reject before caching: delayed SCAN facts cannot seed its successor.
        # This is identity retirement, not a map age/renewal requirement.
        if self.transport.quarantined:
            return False
        end=nanoseconds(message.check_end);begin=nanoseconds(message.check_begin)
        stateful=self.geometry_owner=='local_executor'
        if (message.version.schema_version!=3 or message.version.session_id!=self.p['session_id']
                or message.version.map_version_id!=self.p['map_version_id']
                or message.transport_mode!=self.p['transport_mode'] or message.frame_id!='d1max_loc_odom'
                or message.sequence<=0 or message.trajectory_id<0
                or not 0<begin<=end<=self.now_ns() or nanoseconds(message.source_stamp)!=end
                or (nanoseconds(message.valid_until)!=0 if stateful else
                    message.valid and nanoseconds(message.valid_until)<=end)):
            return False
        reference=self.transport.accepted
        if not (reference is not None and message.version==version_of(reference)
                or self.prepared_for(message.version) is not None):
            return False
        historical=self.remember_local_commit_proof(message)
        if not stateful and not 0<=self.now_ns()-end<=self.freshness_ns:
            self.reconcile_local_commit()
            return historical  # Historical facts never enter the live inbox.
        key=execution_version_identity(message.version),message.trajectory_id
        previous=self.validation_facts.get(key)
        if previous is not None:
            old=previous[0]
            if message.sequence<=old.sequence:
                self.reconcile_local_commit()
                return message==old or historical # no live receipt-time renewal
            if (message.map_snapshot_revision<old.map_snapshot_revision
                    or nanoseconds(message.body_source_stamp)<nanoseconds(old.body_source_stamp)):
                self.reconcile_local_commit()
                return historical # no revival of a newer negative live verdict
        self.validation_facts[key]=(deepcopy(message),self.mono())
        while len(self.validation_facts)>self.prepared_limit:
            retired=min((k for k in self.validation_facts if k!=self.committed_validation_key()),
                key=lambda k:self.validation_facts[k][1])
            self.validation_facts.pop(retired)
        self.reconcile_local_commit()
        return True

    def remember_local_commit_proof(self,message):
        if self.geometry_owner!='local_executor' or self.transport.quarantined:
            return False
        record=self.prepared_for(message.version)
        if record is None or not record.delivered or record.support is None:
            return False
        domain=(message.checked_from_time,message.valid_start_time,message.valid_start_arc_length,
                message.checked_to_time,message.curve_duration)
        end,until=nanoseconds(message.check_end),nanoseconds(message.valid_until)
        if until!=0:
            # Formal SCAN decisions are state, not the old 250 ms proof lease.
            # Reject mixed-version input rather than silently reinterpreting it.
            self.trace('commit_proof_rejected',reason='legacy_proof_lease_in_stateful_session',
                generation=message.version.reference_generation,curve=message.trajectory_id,
                sequence=message.sequence,check_end_ns=end,valid_until_ns=until,
                lease_ns=until-end)
            return False
        if (message.proposal_id!=record.proposal.proposal_id or not message.valid
                or not message.whole_curve or message.remaining_curve
                or message.support_reference_id!=record.support.support_reference_id
                or message.support_hash!=record.support.support_hash
                or message.map_snapshot_revision<=0 or until!=0
                or not 0<nanoseconds(message.body_source_stamp)<=end
                or not all(math.isfinite(value) for value in domain)
                or message.checked_from_time!=0. or message.valid_start_arc_length<0.
                or not 0.<=message.valid_start_time<=message.checked_to_time
                or not 0.<message.curve_duration<=120.
                or abs(message.checked_to_time-message.curve_duration)>1e-6):
            return False
        key=(execution_version_identity(message.version),message.trajectory_id,message.sequence)
        previous=self.local_commit_proofs.get(key)
        if previous is not None:
            return message==previous
        self.local_commit_proofs[key]=deepcopy(message)
        while len(self.local_commit_proofs)>16:
            self.local_commit_proofs.pop(next(iter(self.local_commit_proofs)))
        return True

    def reconcile_local_commit(self):
        if self._reconciling_local_commits or not self.deferred_local_commits:
            return False
        self._reconciling_local_commits=True
        changed=False
        try:
            # A small literal fact ledger joins independent receipt/proof/ACK
            # topics. A newer fully evidenced Native result is authoritative
            # even when an intermediate ACK was lost; this observer never
            # replays a CAS, invents its predecessor or renews a proof.
            for _ in range(self.local_commit_fact_limit):
                advanced=False
                for sequence in sorted(self.deferred_local_commits):
                    message=self.deferred_local_commits.get(sequence)
                    if message is not None and self.on_local_commit(message):
                        advanced=changed=True
                        break
                if not advanced:
                    break
        finally:
            self._reconciling_local_commits=False
        return changed

    def defer_local_commit(self,message,reason):
        sequence=message.commit_sequence
        old=self.deferred_local_commits.get(sequence)
        if old is not None and old!=message:
            self.trace('commit_rejected',reason='commit_sequence_content_changed',sequence=sequence)
            return False
        if old is None:
            if len(self.deferred_local_commits)>=self.local_commit_fact_limit:
                oldest=min(self.deferred_local_commits)
                if sequence<oldest:
                    return False  # Late historical facts cannot evict newer results.
                # Geometry is observed from Native's accepted snapshot, not
                # replayed by walking every predecessor. Keep the latest
                # bounded original facts so a missing old proof cannot block
                # a newer result whose matching proof arrives next.
                self.deferred_local_commits.pop(oldest)
                self.trace('commit_evicted',sequence=oldest,replaced_by=sequence,
                    reason='newer_native_result_waiting_original_proof')
            self.deferred_local_commits[sequence]=deepcopy(message)
        self.deferred_local_commit=self.deferred_local_commits[min(self.deferred_local_commits)]
        self.trace('commit_deferred',reason=reason,sequence=sequence,
            curve=message.trajectory_id,expected_curve=message.expected_trajectory_id,
            owner_curve=self.local_commit.trajectory_id if self.local_commit else -1,
            pending=len(self.deferred_local_commits))
        return True

    def pending_debug_fact(self):
        pending=self.transport.pending
        if pending is None or self.receipt is None or not self.receipt.accepted:
            return None,-math.inf
        facts=[fact for fact in self.debug_facts.values()
            if fact[0].generation==pending.generation
            and nanoseconds(fact[0].header.stamp)>=nanoseconds(pending.path.header.stamp)]
        return max(facts,key=lambda fact:nanoseconds(fact[0].header.stamp),default=(None,-math.inf))

    def prepared_for(self,version):
        value=self.prepared.get(version.reference_generation)
        return value if value is not None and value.proposal.version==version else None

    def writer_unresolved(self):
        if self.geometry_owner=='local_executor':
            return False  # SDK command outcomes do not own reference geometry.
        current_binding=execution_binding(self.permit) if self.permit is not None else None
        def owner_applied(ack):
            return bool(self.permit is not None and self.permit.geometry_committed
                and self.permit.version==ack.candidate_version
                and self.permit.trajectory_id==ack.candidate_trajectory_id
                and current_binding==(ack.execution_id,ack.control_epoch,ack.sdk_session,
                                      ack.sdk_arm_generation,ack.transport_mode))
        for record in self.prepared.values():
            for key,grant in record.handoffs.items():
                if (current_binding is not None and current_binding[0]
                        and key[0]!=current_binding):
                    continue  # Another execution incarnation cannot block this one.
                ack=record.acknowledgements.get(key)
                if ack is None or ack.applied and key not in record.reconciled_handoffs and not owner_applied(ack):
                    return True
        for key,ack in self.early_acks.items():
            if ack.applied and (current_binding is None or not current_binding[0] or key[0]==current_binding):
                record=self.prepared_for(ack.candidate_version)
                if record is not None and not owner_applied(ack):
                    return True
        return False

    def retire_proposal(self,reason):
        """Free only the planning slot; preserve any possible writer fact."""
        if self.proposal is None:
            return True
        record=self.prepared_for(self.proposal.version)
        if record is not None:
            self.preserve_prepared_observations(record)
            record.retirement_requested=True
        if self.writer_unresolved():
            self.error='waiting_writer_reference_outcome:'+reason
            return False
        if record is not None and record.delivered and record.receipt is None:
            # Native may have accepted this request while its receipt was lost
            # (including the first allowed=False geometry commit). A delivery
            # timeout is not proof that it was never admitted. Keep ONE pending
            # request and recover its exact admission fact; do not generate an
            # unbounded sequence of expired, potentially applied candidates.
            self.error='waiting_native_reference_receipt:'+reason
            return False
        self.invalidate_preparation()
        self.transport.discard_pending()
        self.proposal=self.receipt=None
        if record is not None and not record.delivered:
            self.prepared.pop(record.proposal.version.reference_generation,None)
        self.error=reason
        return True

    def prune_prepared(self):
        # The monotonically sequenced BT commit is the retirement fence for
        # earlier geometry, not the expiry of a proposal delivery lease.
        owner=self.geometry_owner_fact()
        if owner is None:
            return
        current=owner.version.reference_generation
        for generation,record in list(self.prepared.items()):
            unresolved=any(key not in record.acknowledgements for key in record.handoffs)
            if generation<current and not unresolved:
                self.prepared.pop(generation)
        if self.geometry_owner=='local_executor':
            return
        binding=execution_binding(self.permit)
        for key,ack in list(self.early_acks.items()):
            record=self.prepared_for(ack.candidate_version)
            if record is None or (record.installed and key[0]==binding
                    and ack.candidate_version==self.permit.version
                    and ack.candidate_trajectory_id==self.permit.trajectory_id):
                self.early_acks.pop(key)

    def retire_fenced_reanchors(self,admission):
        """Use Native's accepted hard replacement, not age, as a retirement fact.

        A changed anchor cannot be a rolling extension. Native checks its
        actual incumbent under the commit mutex, then clears both selected
        and prepared old candidates before acknowledging this admission.
        Ordinary rolling receipts are NOT this fence: Native may retain an
        older same-anchor candidate while computing the next window.
        """
        if (self.geometry_owner!='local_executor' or self.transport.quarantined
                or self.transport.route is None or admission.receipt is None
                or not admission.receipt.accepted):
            return
        proposal=admission.proposal
        incoming,expected=proposal.version,proposal.expected_version
        if (not expected.anchor_id or incoming.anchor_id==expected.anchor_id
                or incoming.task_id!=self.transport.route.task_id
                or incoming.route_hash!=self.transport.route.route_hash):
            return
        task_fields=('schema_version','session_id','task_id','route_id','route_hash',
            'map_version_id','localization_epoch','localization_seed_id','context_sequence')
        if any(getattr(incoming,k)!=getattr(expected,k) for k in task_fields):
            return
        owner=self.geometry_owner_fact()
        retired=[]
        for generation,record in list(self.prepared.items()):
            version=record.proposal.version
            if (generation>=incoming.reference_generation or record.installed
                    or not record.retirement_requested or not record.delivered
                    or record.receipt is None or not record.receipt.accepted or record.handoffs
                    or version==expected or owner is not None and version==owner.version
                    or version.anchor_id==incoming.anchor_id
                    or any(getattr(version,k)!=getattr(incoming,k) for k in task_fields)
                    or any(version in (event.version,event.expected_version)
                           for event in self.deferred_local_commits.values())):
                continue
            # Delayed owner results and the original predecessor stay retained
            # above. These remaining failed, uncommitted old-anchor records
            # cannot pass Native's post-replacement reservation/identity CAS.
            self.prepared.pop(generation)
            retired.append(generation)
        if retired:
            self.trace('reference_retirement',admission_generation=incoming.reference_generation,
                retired_generations=retired,reason='accepted_native_hard_reanchor')

    def reconcile_owner_facts(self):
        if self.permit is None or not self.permit.geometry_committed:
            return
        binding=execution_binding(self.permit)
        # A later exact writer commit and its corresponding BT geometry
        # permit supersede earlier commits in that same execution binding.
        # This handles an old ACK delivered after the next permit heartbeat.
        watermark=max((ack.commit_sequence for record in self.prepared.values()
            for key,ack in record.acknowledgements.items()
            if ack.applied and key[0]==binding and ack.candidate_version==self.permit.version
                and ack.candidate_trajectory_id==self.permit.trajectory_id),default=0)
        for record in self.prepared.values():
            for key,ack in record.acknowledgements.items():
                if ack.applied and key[0]==binding and ack.commit_sequence<=watermark:
                    record.reconciled_handoffs.add(key)
                    record.closed_handoff_sequence[binding]=max(ack.grant_sequence,
                        record.closed_handoff_sequence.get(binding,0))

    def propose_window(self):
        self.prune_prepared()
        if (self.geometry_owner=='local_executor' and self.local_commit is not None
                and self.local_commit.retired):
            self.error='local_geometry_owner_retired'
            return None
        if self.writer_unresolved():
            self.error='waiting_writer_reference_outcome'
            return None
        if len(self.prepared)>=self.prepared_limit:
            self.error='prepared_reference_ledger_full_waiting_owner_resolution'
            return None
        return self.transport.propose_window(current_source_ns=self.now_ns(),now_monotonic=self.mono())

    def on_navigation(self,message):
        state=validate_state(message,session_id=self.p['session_id'],map_version_id=self.p['map_version_id'],now_ns=self.now_ns())
        if not navigation_event_is_new(state.identity,state.source_ns,self.state):
            return False  # old epoch/seed callbacks cannot replace the current atomic frame context
        local=self.local_inbox.latest if self.p.get('local_state_enabled') else None
        if (local is not None and state.identity!=local.identity
                and not navigation_event_is_new(state.identity,state.source_ns,local)):
            # Independent topics can deliver the local reset before an older
            # global pair. Compare identities before changing native context,
            # otherwise that queued pair rolls back its acknowledged barrier.
            # A SAME-identity pair may lag local and still establish a genuine
            # map->odom anchor; no global pose is manufactured from local data.
            return False
        if self.context is None or state.identity!=(self.context['session_id'],self.context['epoch'],self.context['seed_id']):
            if self.context is not None:
                self.invalidate_preparation()
                self.clear_prepared()
                self.proposal=self.receipt=self.permit=None
                self.clear_native_debug()
                # A hard notice under the OLD localization identity may have
                # already canceled and retired its Native operation before
                # this new verified pair arrives. Keep that immutable ACK:
                # recovery still matches the exact cancel generation/source,
                # empty owner and new context. Clearing it here strands the
                # retired generation in quarantine; it is not a live curve or
                # a permission that must be revoked by this identity change.
            self.context=dict(schema=1,session_id=state.identity[0],epoch=state.identity[1],seed_id=state.identity[2],
                sequence=1 if self.context is None else self.context['sequence']+1,barrier_ns=state.source_ns)
            self.context_ready=False; self.map_report=None; self.context_last_sent=-math.inf
            self.transport.context_sequence=self.context['sequence']
        self.state=state
        if not self.p.get('local_state_enabled'):
            self.state_received=self.mono()
            self.publish('body',deepcopy(message.local_odometry))
        self.sync_context()
        try:
            self.transport.on_navigation(message,received_monotonic=self.mono(),
                current_source_ns=self.now_ns(),now_monotonic=self.mono())
        except ValueError as error:
            self.error=str(error)
            if not self.transport.quarantined and self.continue_same_floor_segment():
                return True  # Latest literal pose can complete a witnessed portal after candidate latency.
            self.try_recover_context()
            # A new exact global/local pair may correct the incumbent anchor
            # even when projection under that old anchor failed. Preparing an
            # independent candidate does not replace/refresh old geometry or
            # grant movement; native still owns its validation and commit.
            self.refresh_window()
            return False
        self.try_recover_context()
        self.publish_route_progress()
        self.refresh_window()
        return True

    def on_local_navigation(self,message):
        event=self.local_inbox.accept(message,session_id=self.p['session_id'],
            map_version_id=self.p['map_version_id'],now_ns=self.now_ns(),monotonic=self.mono())
        if event is None: return False
        if event.hard_failure:
            self.invalidate_preparation()
            self.transport.quarantined=True
            self.transport.discard_pending()
            self.clear_prepared()
            self.proposal=self.receipt=self.permit=None
            self.clear_native_debug()
            # Even a same-identity loss needs a NEW acknowledged barrier;
            # an ACK from before the hard event cannot recover the empty slot.
            self.context=dict(schema=1,session_id=event.identity[0],epoch=event.identity[1],
                seed_id=event.identity[2],sequence=1 if self.context is None else self.context['sequence']+1,
                barrier_ns=event.source_ns)
            self.context_ready=False; self.map_report=None; self.context_last_sent=-math.inf
            self.transport.context_sequence=self.context['sequence']
            self.context_reset_reason=event.reason
            self.context_reset_source_ns=event.source_ns
            self.sync_context()
            self.error=event.reason
            return False
        if event.state is None: return False
        self.state_received=event.received
        self.publish('body',deepcopy(message.local_odometry))
        try:
            self.transport.on_local(message,received_monotonic=event.received,
                current_source_ns=self.now_ns(),now_monotonic=self.mono())
        except ValueError as error:
            if self.transport.quarantined:
                self.clear_native_debug()
            self.error=str(error)
            if not self.transport.quarantined and self.continue_same_floor_segment():
                return True
            if not self.try_recover_context():
                return False
        if (self.proposal is not None and self.transport.pending_core is not None
                and self.transport.pending_core._progress is None):
            self.retire_proposal(str(self.transport.status.get('pending_reason','candidate_projection_unavailable')))
        self.publish_route_progress()
        self.refresh_window()
        return True

    def publish_route_progress(self):
        try:
            facts=self.transport.accepted_route_progress(current_source_ns=self.now_ns(),now_monotonic=self.mono())
        except ValueError:
            return False  # Committed geometry does not invent measured progress.
        if facts is None:
            return False
        reference,progress,body,anchor,height=facts
        if self.p.get('local_state_enabled'):
            # An exact global pair can arrive ahead of its independent local
            # topic. It establishes an anchor, not a new task-progress source.
            # Do not let that cross-topic phase replace the last literal local
            # observation in the owner's bounded progress cache.
            event=self.local_inbox.latest
            state=None if event is None else event.state
            if (state is None or event.hard_failure
                    or not self.local_inbox.task_usable(now_ns=self.now_ns(),
                        monotonic=self.mono(),receipt_timeout_s=self.transport.freshness)
                    or state.source_ns!=progress.source_ns
                    or state.identity!=(body.context.session,body.context.epoch,body.context.seed)
                    or not np.allclose(body.pose.xyz,state.local_body.position,rtol=0.,atol=1e-12)
                    or not np.allclose(body.pose.xyzw,state.local_body.orientation,rtol=0.,atol=1e-12)):
                return False
        key=(reference.task_id,reference.route_hash,reference.generation,anchor.anchor_id,progress.source_ns)
        if key==self.route_progress_key:
            return False  # Timer/permit heartbeat is not a new source observation.
        from d1max_planning_interfaces.msg import RouteProgress
        message=RouteProgress(schema_version=1,version=version_of(reference),
            frame_id='d1max_loc_map',transport_mode=self.p['transport_mode'],
            edge_index=progress.edge_index,measured_arc_m=progress.measured_arc_m,
            confirmed_arc_m=progress.confirmed_arc_m,cross_track_m=progress.cross_track_m,
            body_reference_height_m=height)
        self.route_progress_sequence+=1
        message.sequence=self.route_progress_sequence
        set_stamp(message.body_source_stamp,progress.source_ns)
        message.odom_body_pose.header.frame_id=body.frame
        message.odom_body_pose.header.stamp=deepcopy(message.body_source_stamp)
        p,q=message.odom_body_pose.pose.position,message.odom_body_pose.pose.orientation
        p.x,p.y,p.z=body.pose.xyz;q.x,q.y,q.z,q.w=body.pose.xyzw
        set_stamp(message.anchor_source_stamp,anchor.source_ns)
        p,q=message.map_from_odom.position,message.map_from_odom.orientation
        p.x,p.y,p.z=anchor.map_from_odom.xyz;q.x,q.y,q.z,q.w=anchor.map_from_odom.xyzw
        message.source_map_body_xyz.x,message.source_map_body_xyz.y,message.source_map_body_xyz.z=progress.body_source_xyz
        self.route_progress_key=key
        self.publish('route_progress',message)
        return True

    def on_task_lease(self,message):
        """Read BT's task grant for a mode request; never create a motion grant."""
        from .segment_transition import same_task
        route=self.transport.route
        if (message.schema_version!=1 or route is None or message.transport_mode!=self.p['transport_mode']
                or not same_task(message.version,route.snapshot)
                or not message.execution_id or not message.sdk_session or message.control_epoch<=0
                or not 0<nanoseconds(message.source_stamp)<=self.now_ns()<nanoseconds(message.valid_until)):
            return False
        previous=self.task_lease
        if previous is not None and same_task(previous.version,message.version):
            if message.control_epoch<previous.control_epoch:
                return False
            if message.control_epoch==previous.control_epoch and (
                    previous.execution_id!=message.execution_id or previous.sdk_session!=message.sdk_session
                    or message.sdk_arm_generation<previous.sdk_arm_generation or message.sequence<=previous.sequence):
                return False
        self.task_lease=deepcopy(message)
        if message.revoked:
            self.mode_transition.clear()
        return True

    def on_mode_transition(self,message):
        # The fixed-mode session never owns a per-segment SDK transaction.
        # In particular, a late ACK from a previous policy cannot hold or
        # advance its geometry, and is not treated as fresh mode evidence.
        if self.execution_mode_policy=='fixed_stair':
            return False
        if not self.mode_transition.observe(message,now_ns=self.now_ns(),received_monotonic=self.mono()):
            return False
        self.error='' if message.confirmed and message.capability_verified else 'mode_transition:'+message.reason
        self.refresh_window()
        return True

    def continue_same_floor_segment(self):
        """Prepare an adjacent segment at its measured XYZ portal.

        With fixed STAIR, all boundaries use the same geometric handoff with
        no mode transaction. Terrain labels are unchanged. A live special
        boundary requests the sole SDK writer's mode transaction and consumes
        its bound callback evidence. Preview uses an explicit geometry-only
        capability; it never creates a synthetic SDK mode observation.
        """
        t=self.transport
        if (t.core is None or t.accepted is None or t.pending is not None
                or self.geometry_owner_fact() is None):
            return False
        self.prune_prepared()
        if self.writer_unresolved() or len(self.prepared)>=self.prepared_limit:
            return False
        try:
            if self.execution_mode_policy=='fixed_stair':
                # A pending solve may have delayed ownership beyond the
                # small portal region. The core keeps the literal crossing,
                # but projects the latest body on exactly the adjacent segment.
                t.request_segment_transition(current_source_ns=self.now_ns(),now_monotonic=self.mono())
                return True
            old,new,portal=t.core.reached_adjacent_portal(current_source_ns=self.now_ns(),now_monotonic=self.mono())
            mode=None
            if t.core.requires_mode_transition():
                if self.execution_purpose=='planning_only':
                    from .continuous_reference import PreviewSegmentTransition
                    mode=PreviewSegmentTransition(t.core.context,t.core.task_id,t.core.snapshot.route_hash,new['segment_id'])
                else:
                    mode=self.mode_transition.confirmation(core=t.core,reference=t.accepted,
                        lease=self.task_lease,now_ns=self.now_ns(),now_monotonic=self.mono())
                    if mode is None:
                        self.request_mode_transition(old,new,portal)
                        return True  # Keep the route and current window while the writer coordinates the mode.
            t.request_segment_transition(current_source_ns=self.now_ns(),now_monotonic=self.mono(),mode=mode)
        except ValueError as error:
            if str(error) not in ('adjacent_portal_not_physically_reached','already_final_semantic_segment'):
                self.error=str(error)
            return False
        return True

    def request_mode_transition(self,old,new,portal):
        if self.execution_mode_policy=='fixed_stair':
            return False
        from .segment_transition import same_task
        lease=self.task_lease;reference=self.transport.accepted;now=self.now_ns()
        if (lease is None or lease.revoked or not same_task(lease.version,reference)
                or lease.sdk_arm_generation<=0 or not 0<nanoseconds(lease.source_stamp)<=now<nanoseconds(lease.valid_until)):
            self.error='waiting_execution_confirmation_for_mode_transition'
            return False
        from d1max_planning_interfaces.msg import ModeTransitionRequest
        request=self.mode_transition.request
        binding=(lease.execution_id,lease.control_epoch,lease.sdk_session,lease.sdk_arm_generation)
        if (request is None or request.version!=version_of(reference)
                or request.target_segment_id!=new['segment_id']
                or binding!=(request.execution_id,request.control_epoch,request.sdk_session,request.sdk_arm_generation)):
            self.mode_request_sequence+=1
            request=ModeTransitionRequest(schema_version=1,version=version_of(reference),
                request_id=hashlib.sha256((reference.task_id+':'+reference.segment_id+':'+new['segment_id']+
                    ':'+str(self.mode_request_sequence)+':'+lease.execution_id).encode()).hexdigest(),
                sequence=self.mode_request_sequence,target_segment_id=new['segment_id'],
                current_mode=old['required_mode'],target_mode=new['required_mode'],
                transport_mode=self.p['transport_mode'],execution_id=lease.execution_id,
                control_epoch=lease.control_epoch,sdk_session=lease.sdk_session,
                sdk_arm_generation=lease.sdk_arm_generation,preview_only=False)
            set_stamp(request.source_stamp,now)
            set_stamp(request.valid_until,now+10_000_000_000)
            set_stamp(request.body_source_stamp,self.transport.core._progress.source_ns)
            request.portal_body_source_xyz.x,request.portal_body_source_xyz.y,request.portal_body_source_xyz.z=portal
            self.mode_transition.begin(request)
            self.mode_request_last_sent=-math.inf
        # Reliable delivery can be retried with the exact same transaction.
        # No timer expiry invents a new request or refreshes a source stamp.
        if self.mono()-self.mode_request_last_sent>=.5 and now<nanoseconds(request.valid_until):
            self.publish('mode_request',request)
            self.mode_request_last_sent=self.mono()
        state=self.mode_transition.state
        self.error='mode_transition:'+state.reason if state else 'waiting_mode_transition_writer'
        return True

    def refresh_window(self):
        t=self.transport
        if not t.core or t.quarantined or not self.reference_inputs_ready():
            return
        # A measured portal crossing is an event, not a periodic replan. Check
        # it before the ordinary 2 Hz refresh interval, otherwise a robot can
        # traverse the small physical portal while the timer is throttled.
        if self.continue_same_floor_segment():
            return
        elapsed=self.mono()-self.last_proposal_monotonic
        if elapsed<self.rolling_policy.minimum_interval_s:
            return
        remaining=t.pending_window_remaining()
        # A consumed candidate may release the planning slot, but its literal
        # geometry survives a cross-topic late writer/owner commit in ledger.
        if t.pending is not None and remaining is not None and remaining<=.2:
            self.retire_proposal('pending_window_consumed_rebuild_same_route')
        # A running candidate owns its literal window and identity. Ordinary
        # body updates never interrupt it; a late commit retains the original
        # publication arc so the next window can immediately catch up.
        if t.pending is not None:
            return
        window=t.rolling_window_progress()
        correction_due=t.correction_ready(current_source_ns=self.now_ns())
        if t.accepted is not None and window is None and not correction_due:
            return  # Missing progress cannot fabricate a new initial window.
        reason=self.rolling_policy.reason(window,elapsed_s=elapsed,
            covers_segment_end=t.covers_current_segment_end(),correction_due=correction_due)
        if reason is not None:
            try: self.propose_window()
            except ValueError as error: self.error=str(error)

    def sync_context(self):
        if self.context and not self.context_ready and self.mono()-self.context_last_sent>=.25:
            self.publish('context',dict(self.context)); self.context_last_sent=self.mono()

    def on_context_ack(self,value):
        if value==self.context:
            self.context_ready=True
            self.try_recover_context()
            return True
        return False

    def on_map(self,value):
        if (not self.context or value.get('session_id')!=self.context['session_id']
                or any(value.get(k)!=self.context[k] for k in ('epoch','seed_id','sequence','barrier_ns'))):
            return False
        stamp=value.get('source_stamp_ns')
        if type(stamp) is not int or stamp<=0 or self.map_report and stamp<self.map_report['source_stamp_ns']:
            return False
        self.map_report,self.map_received=value,self.mono()
        return True

    @staticmethod
    def route_identity(message):
        return message.task_id,message.route_id,message.route_hash

    def route_receipt(self,message,status,reason='',*,required=False,confirmed=False,generation=0):
        from d1max_planning_interfaces.msg import RouteDeliveryReceipt
        receipt=RouteDeliveryReceipt(schema_version=1,source_stamp=deepcopy(message.source_stamp),
            session_id=message.session_id,task_id=message.task_id,route_id=message.route_id,
            route_hash=message.route_hash,delivery_sequence=message.delivery_sequence,active=message.active,
            status=status,native_retirement_required=required,native_retirement_confirmed=confirmed,
            native_generation=generation,reason=reason)
        set_stamp(receipt.receipt_stamp,self.now_ns())
        identity=self.route_identity(message)
        self.route_dispositions.setdefault(identity,{})[message.active]=deepcopy(receipt)
        self.route_dispositions.move_to_end(identity)
        # Receipts are a bounded idempotence cache, not the retired-task fence.
        # Keep the exact current/pending owner retryable while retaining recent
        # completed replies; canonical retirement lives in RouteIngress.
        protected={self.transport.ingress.active_identity}
        if self.pending_route_retirement is not None:
            protected.add(self.route_identity(self.pending_route_retirement['command']))
        while len(self.route_dispositions)>129:
            victim=next(key for key in self.route_dispositions if key not in protected)
            del self.route_dispositions[victim]
        self.publish('route_receipt',receipt)
        return receipt

    def finish_route_retirement(self,*,native_confirmed=False):
        pending=self.pending_route_retirement
        if pending is None or pending['required'] and not native_confirmed:
            return False
        from d1max_planning_interfaces.msg import RouteDeliveryReceipt
        command=pending['command'];identity=self.route_identity(command)
        if pending['required']:
            self.native_route_exposures.discard(identity)
        else:
            # No native publish happened. The manager has already installed
            # the task tombstone and invalidated its asynchronous preparation.
            # There is no raw operation or ACK to fabricate or wait for.
            self.transport._retired_generation=self.transport._retired_stamp_ns=0
        self.pending_route_retirement=None
        self.route_receipt(command,RouteDeliveryReceipt.RETIRED,'reference_work_retired',
            required=pending['required'],confirmed=native_confirmed,
            generation=pending['generation'] if pending['required'] else 0)
        return True

    def on_route(self,message):
        # Accept the immutable BT task once, even if map ACK arrives later.
        # Actual native proposal delivery remains gated in prepare/tick.
        # RouteIngress retains sequence and cancellation ownership, so an old
        # queued active task cannot resurrect after its cancellation.
        from d1max_planning_interfaces.msg import RouteDeliveryReceipt
        from .source_route_ros import from_message
        identity=self.route_identity(message)
        # Check provenance before emitting even a negative acknowledgement.
        value=from_message(message.snapshot).payload()
        if (message.schema_version!=2 or message.session_id!=self.p['session_id']
                or message.snapshot.map_version_id!=self.p['map_version_id']
                or any(value[k]!=expected for k,expected in self.transport.expected_hashes.items())
                or any(getattr(message.snapshot,k)!=getattr(message,k)
                    for k in ('session_id','task_id','route_id','route_hash'))
                or message.snapshot.path.header.stamp!=message.source_stamp
                or not all(identity) or message.delivery_sequence<=0):
            raise ValueError('route_delivery_receipt_identity_invalid')
        previous=self.route_dispositions.get(identity,{}).get(message.active)
        if (previous is not None and previous.delivery_sequence==message.delivery_sequence
                and previous.source_stamp==message.source_stamp
                and (not message.active or not self.transport.ingress.is_retired(identity))):
            # Reaffirm a durable software fact with the current manager's
            # receipt time. Its original delivery/source identity is unchanged;
            # this never renews a sensor, trajectory or motion lease.
            self.route_receipt(message,previous.status,previous.reason,
                required=previous.native_retirement_required,
                confirmed=previous.native_retirement_confirmed,generation=previous.native_generation)
            return None
        pending=self.pending_route_retirement
        if pending is not None:
            if not message.active and identity!=self.route_identity(pending['command']):
                if (identity not in self.native_route_exposures
                        and self.transport.ingress.active_identity!=identity
                        and self.transport.ingress.accept(message,now=self.now_ns()*1e-9,
                            context=None,retire_unadmitted=True) is not None):
                    self.route_receipt(message,RouteDeliveryReceipt.RETIRED,'unadmitted_route_retired')
                    return None
            elif (not message.active and message.delivery_sequence==pending['command'].delivery_sequence
                    and message.source_stamp==pending['command'].source_stamp):
                return None  # The one native barrier is still in flight.
            self.route_receipt(message,RouteDeliveryReceipt.REJECTED,'prior_route_retirement_pending')
            return None
        try:
            result=self.transport.on_route(message,current_source_ns=self.now_ns(),now_monotonic=self.mono())
        except ValueError as error:
            # Acceptance of ownership and preparation are different stages.
            # An initialization error AFTER ingress acceptance still owns the
            # task and must be canceled, never declared "never submitted".
            owned=self.transport.ingress.active_identity==identity and self.transport.route is not None
            self.route_receipt(message,RouteDeliveryReceipt.ACCEPTED if message.active and owned
                else RouteDeliveryReceipt.REJECTED,str(error))
            self.error=str(error)
            raise
        if message.active:
            owned=self.transport.ingress.active_identity==identity and self.transport.route is not None
            self.route_receipt(message,RouteDeliveryReceipt.ACCEPTED if owned
                else RouteDeliveryReceipt.REJECTED,'' if owned else 'route_delivery_not_accepted')
            return result
        if result is None:
            # Rejected/unseen active delivery: install its cancellation
            # tombstone first, so a queued reliable active cannot resurrect it.
            if identity in self.native_route_exposures:
                self.route_receipt(message,RouteDeliveryReceipt.REJECTED,'native_route_owner_not_retired')
                return None
            retired=self.transport.ingress.accept(message,now=self.now_ns()*1e-9,
                context=None,retire_unadmitted=True)
            if retired is None:
                self.route_receipt(message,RouteDeliveryReceipt.REJECTED,'route_cancel_not_owned')
                return None
            self.route_receipt(message,RouteDeliveryReceipt.RETIRED,'unadmitted_route_retired')
            self.try_recover_context()
            return None
        required=identity in self.native_route_exposures
        self.pending_route_retirement=dict(command=deepcopy(message),required=required,
            generation=self.cancel_generation if required else 0)
        if not required:
            self.finish_route_retirement()
            self.try_recover_context()
        return result

    def prepare(self,reference):
        if not reference.path.poses:
            self.invalidate_preparation()
            self.clear_prepared()
            self.clear_native_debug()
            self.cancel_generation=reference.generation
            self.proposal=self.receipt=None
            # A new task has no predecessor from the retired owner. Keep the
            # bridge generation fence, not the old BT execution version.
            self.permit=None
            identity=(reference.task_id,reference.route_id,reference.route_hash)
            if identity in self.native_route_exposures:
                self.publish('cancel_reference',reference)
            return
        if not self.reference_inputs_ready():
            self.transport.discard_pending()
            self.error=('reference_requires_acknowledged_context' if self.geometry_owner=='local_executor'
                else 'reference_requires_acknowledged_fresh_native_map')
            return
        from d1max_planning_interfaces.msg import ReferenceProposal, SupportReference
        from geometry_msgs.msg import Point
        proposal=ReferenceProposal()
        proposal.version=version_of(reference)
        proposal.reference=deepcopy(reference)
        proposal.expected_trajectory_id=-1
        owner=self.geometry_owner_fact()
        if owner is not None:
            proposal.expected_version=deepcopy(owner.version)
            proposal.expected_trajectory_id=owner.trajectory_id
        proposal.proposal_id=f'{reference.task_id}.{reference.generation}'
        proposal.source_stamp=deepcopy(reference.path.header.stamp)
        set_stamp(proposal.valid_until,nanoseconds(proposal.source_stamp)+self.freshness_ns)
        proposal.transport_mode=self.p['transport_mode']
        core=self.transport.pending_core or self.transport.core
        endpoint=core.goal_ground_xyz
        xyz=core.anchor.map_from_odom.inverse().point((endpoint[0],endpoint[1],endpoint[2]+core.body_height))
        proposal.goal_position=Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2]))
        proposal.has_goal_yaw,yaw,proposal.goal_yaw_tolerance_rad=core.goal_heading
        if proposal.has_goal_yaw:
            heading=core.anchor.map_from_odom.inverse().rotate((math.cos(yaw),math.sin(yaw),0.))
            if math.hypot(heading[0],heading[1])<.1:
                raise ValueError('goal_heading_projection_degenerate')
            proposal.goal_yaw=math.atan2(heading[1],heading[0])
        self.proposal,self.receipt=proposal,None
        if len(self.prepared)>=self.prepared_limit:
            # This also covers route initialization, which enters prepare
            # directly rather than through the timer's proposal wrapper.
            self.transport.discard_pending(); self.proposal=None
            self.error='prepared_reference_ledger_full_waiting_owner_resolution'
            return
        self.prepared[reference.generation]=PreparedWindow(deepcopy(proposal),deepcopy(core),
            self.transport._pending_received_monotonic,self.transport._pending_end_arc_m,
            self.transport._pending_start_arc_m,self.transport._pending_issue_arc_m)
        self.last_proposal_monotonic=self.mono()
        # Admit the small immutable window while its actual body source is
        # fresh. Static map support is NOT a sensor heartbeat: preparing or
        # publishing thousands of points must not consume this deadline.
        # Native retains the incumbent until the successor is actually proved
        # and committed; receiving this proposal does not grant movement.
        if not self.deliver_proposal(proposal):
            return
        # Freeze the candidate's exact anchor, original source time and support
        # identity. The worker reads immutable map data, never transport/core.
        segment=core._segments[core._segment]
        segment_context=(segment['segment_id'],segment['floor_id'],segment['kind'],segment['required_mode'])
        mixed=set(reference.point_segment_ids)!={reference.segment_id}
        if mixed or segment['required_mode']=='stair':
            represented=set(reference.point_segment_ids)
            # Read-only views share the immutable route; no full-route copy on
            # each rolling proposal. Ownership must also see adjacent segments
            # not yet represented, otherwise a 2->3 segment window relabels
            # the same original returns as it advances.
            ownership_contexts=tuple((s['segment_id'],s['floor_id'],s['kind'],s['required_mode'],
                s['begin_index'],s['end_index'],
                core._points[s['begin_index']:s['end_index']+1])
                for s in core._segments)
            ownership_identity=(reference.task_id,reference.route_hash)
            if mixed:
                segment_context += (tuple((*context,ownership_contexts,ownership_identity) for context in ownership_contexts
                    if context[0] in represented),)
            else:
                segment_context += ((),ownership_contexts,ownership_identity)
        request=(deepcopy(proposal),core.support_evidence,core.anchor.map_from_odom.inverse(),
                 tuple(core._progress.body_source_xyz[:2]),core.body_height,
                 segment_context)
        record=self.prepared_for(proposal.version)
        record.support_request=request
        self.prepare_support(record)

    def prepare_support(self,record):
        """Retry the original static work, not a new time/anchor/admission."""
        record.last_support_preparation_monotonic=self.mono()
        record.support_preparing=True
        try:
            if self.preparation is not None:
                self.preparation.submit(record.proposal.proposal_id,record.support_request)
            else:
                value=self.build_support(record.support_request)
                record.support_preparing=False
                self.deliver_support(*value)
        except Exception as error:
            record.support_preparing=False
            self.error='support_preparation_failed:'+type(error).__name__+':'+str(error)

    def build_support(self,request):
        from d1max_planning_interfaces.msg import SupportReference, SupportRegion
        from geometry_msgs.msg import Point
        proposal,evidence,inverse,center,body_height,segment=request
        segment_id,floor_id,segment_kind,required_mode=segment[:4]
        contexts=segment[4] if len(segment)>4 else ()
        ownership_contexts=segment[5] if len(segment)>5 else ()
        ownership_identity=segment[6] if len(segment)>6 else ()
        if (proposal.reference.segment_id!=segment_id
                or proposal.reference.segment_kind!=segment_kind
                or proposal.reference.required_mode!=required_mode):
            raise ValueError('support_reference_segment_identity_mismatch')
        support=SupportReference(version=deepcopy(proposal.version),
            support_reference_id=proposal.proposal_id, support_map_sha256=evidence.source_map_sha256,
            floor_id=floor_id,segment_kind=segment_kind,required_mode=required_mode,frame_id='d1max_loc_map',
            source_stamp=deepcopy(proposal.source_stamp),support_xy_radius_m=.2,
            body_reference_height_m=body_height,max_support_slope_rad=.15,max_support_step_m=.05,
            verified=bool(evidence.execution_eligible),reason=evidence.eligibility_reason)
        # Literal original-map returns are preserved in the snapshot evidence.
        # Reference interpolation is never relabelled as a measured return.
        source=evidence.observed_source_support_xyz
        encoded_points=evidence.canonical_observed_support
        if contexts:
            from .reference_support import transition_source_returns
            forward=inverse.inverse()
            ground=np.asarray([forward.point((p.pose.position.x,p.pose.position.y,p.pose.position.z))-
                np.array((0.,0.,body_height)) for p in proposal.reference.path.poses])
            source,regions,support.verified,support.reason=transition_source_returns(
                self.support_map,ground,proposal.reference,contexts,evidence.source_map_sha256)
            support.regions=[SupportRegion(**r) for r in regions]
            support.max_support_slope_rad=support.max_support_step_m=0.
            encoded_points=canonical(source.tolist())
        elif self.support_map is not None:
            from .reference_support import ConditionedReferenceSupport, nearby_source_returns
            conditioned_stair=isinstance(self.support_map,ConditionedReferenceSupport) and required_mode=='stair'
            # Stair packets select their own short XYZ window; avoid gathering
            # an unrelated six-metre floor/ROI cloud before discarding it.
            if not conditioned_stair:
                source=nearby_source_returns(self.support_map,center,6.,floor_id)
            if isinstance(self.support_map,ConditionedReferenceSupport):
                # The support packet certifies only the provenance/class of
                # these source returns. Whole-route execution eligibility is
                # a separate, unchanged fact in the route/BT contracts.
                support.verified=self.support_map.classified_ground(
                    source_map_sha256=evidence.source_map_sha256,floor_id=floor_id,
                    segment_kind=segment_kind,required_mode=required_mode)
                support.reason=('original_floor_support_classified' if support.verified else
                    'source_support_not_classified_for_segment')
                if required_mode=='stair':
                    # The reference is a body path in odom. Associate ORIGINAL
                    # source-map returns with its ground window; do not turn
                    # the protected stairs ROI (walls included) into support.
                    forward=inverse.inverse()
                    ground=np.asarray([forward.point((p.pose.position.x,p.pose.position.y,
                        p.pose.position.z))-np.array((0.,0.,body_height))
                        for p in proposal.reference.path.poses],dtype=float).reshape((-1,3))
                    selection=self.support_map.stair_surface(ground,
                        source_map_sha256=evidence.source_map_sha256,floor_id=floor_id,
                        segment_kind=segment_kind,required_mode=required_mode)
                    source=selection.points
                    if ownership_contexts:
                        from .reference_support import owned_stair_source_returns
                        source=owned_stair_source_returns(self.support_map,source,segment,ownership_contexts,ownership_identity)
                    support.verified=selection.verified
                    support.reason=selection.reason
                    # Historical flat-ground limits must not become a stair
                    # locomotion certificate. Native SCAN consumes provenance;
                    # the legacy flat connector deliberately cannot use this.
                    support.max_support_slope_rad=0.
                    support.max_support_step_m=0.
            if len(source)>20000:
                # Deterministic selection of real points, never voxel centres.
                _,unique=np.unique(np.floor(source/.05).astype(np.int64),axis=0,return_index=True)
                source=source[np.sort(unique)]
            if len(source)>20000:
                source=source[np.linspace(0,len(source)-1,20000,dtype=int)]
            encoded_points=canonical(source.tolist())
        # Only independent, literal package evidence may qualify a contact.
        # Resolve indices after all selection/downsampling and before the
        # bound map->odom transform; no coordinate tolerance can add a row.
        from .reference_support import ConditionedReferenceSupport
        riser_source=np.empty((0,3),dtype=float);riser_owners=[]
        if support.verified and isinstance(self.support_map,ConditionedReferenceSupport):
            support.stair_contact_indices=self.support_map.contact_indices(source,
                source_map_sha256=evidence.source_map_sha256,floor_id=floor_id,
                segment_kind=segment_kind,required_mode=required_mode,
                regions=regions if contexts else ())
            if contexts or required_mode=='stair':
                from .reference_support import original_stair_riser_candidates
                riser_source,riser_owners=original_stair_riser_candidates(self.support_map,ground,
                    source_hash=evidence.source_map_sha256,contexts=contexts,
                    context=segment,domain=ownership_contexts,identity=ownership_identity)
        from .ray_projection import rotate_many
        transformed=rotate_many(np.asarray(inverse.xyzw),
            source)+np.asarray(inverse.xyz)
        for odom in transformed:
            # Tracker operates in odom; preserve the bound anchor explicitly.
            support.support_ground_xyz.append(Point(x=float(odom[0]),y=float(odom[1]),z=float(odom[2])))
        if len(riser_source):
            # Independent original-map observations retain their own channel;
            # never contaminate support_ground_xyz/terrain fitting. Both use
            # the exact same frozen source frame, anchor, version and stamp.
            riser_odom=rotate_many(np.asarray(inverse.xyzw),riser_source)+np.asarray(inverse.xyz)
            support.stair_riser_xyz=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in riser_odom]
            support.stair_riser_owner=riser_owners
        # The shared static model has its own typed channel. Sparse measured
        # returns do not erase it; it never makes those returns verified.
        from .reference_support import parametric_stair_packet
        model_hash, model_surfaces, model_obstacles, model_identity = parametric_stair_packet(
            self.support_map, source_hash=evidence.source_map_sha256,
            segment=segment, contexts=contexts, inverse=inverse)
        if model_hash:
            support.stair_model_hash=model_hash
            support.stair_model_surfaces=model_surfaces
            support.stair_model_obstacles=model_obstacles
        support.frame_id='d1max_loc_odom'
        if len(source)==0:
            support.verified=False
            if required_mode!='stair' and not contexts:
                support.reason='original_support_returns_missing'
        # Same canonical {anchor,map,points} bytes as before. Cache the literal
        # evidence encoding, preserving int/float and signed-zero identity;
        # numeric conversion for geometry must not silently change its hash.
        encoded='{"anchor":'+canonical(proposal.version.anchor_id)+',"map":'+canonical(
            evidence.source_map_sha256)+',"points":'+encoded_points+'}'
        if contexts:
            # The same points with different terrain ownership are not the
            # same support packet. Preserve legacy hashes for nonmixed input.
            encoded=encoded[:-1]+',"regions":'+canonical(regions)+'}'
        if support.stair_contact_indices:
            # Empty qualification preserves the existing flat/legacy hashes.
            # The nonempty subset changes geometry semantics and therefore
            # must never share an identity with the same unqualified rows.
            encoded=encoded[:-1]+',"stair_contact_indices":'+canonical(
                list(support.stair_contact_indices))+'}'
        if len(riser_source):
            from .reference_support import canonical_riser_points
            encoded=encoded[:-1]+',"stair_riser_xyz":'+canonical_riser_points(self.support_map,riser_source)+\
                ',"stair_riser_owner":'+canonical(riser_owners)+'}'
        if model_identity is not None:
            encoded=encoded[:-1]+',"parametric_stair_model":'+canonical(model_identity)+'}'
        support.support_hash=hashlib.sha256(encoded.encode()).hexdigest()
        return proposal,support

    def deliver_proposal(self,proposal):
        """Fresh geometry admission, independent of static support delivery."""
        if (self.transport.quarantined or self.proposal is None
                or proposal!=self.proposal or not self.reference_inputs_ready()
                or self.transport.pending is None
                or proposal.version!=version_of(self.transport.pending)
                or not nanoseconds(proposal.source_stamp)<=self.now_ns()<=nanoseconds(proposal.valid_until)):
            return False
        record=self.prepared_for(proposal.version)
        if record is None:
            return False
        record.delivered=True
        record.last_delivery_monotonic=self.mono()
        self.native_route_exposures.add((proposal.version.task_id,proposal.version.route_id,
                                        proposal.version.route_hash))
        try:
            self.publish('proposal',proposal)
        except Exception as error:
            # A middleware exception is not proof that Native never received
            # the request. Keep its literal identity and recover its receipt.
            self.error='reference_proposal_delivery_failed:'+type(error).__name__+':'+str(error)
            return True
        accepted=self.transport.accepted
        owner=self.geometry_owner_fact()
        self.trace('proposal',generation=proposal.version.reference_generation,
            expected_curve=proposal.expected_trajectory_id,
            observed_owner_curve=owner.trajectory_id if owner is not None else -1,
            previous_generation=accepted.generation if accepted is not None else 0,
            same_anchor=bool(accepted is not None
                and accepted.anchor_id==proposal.version.anchor_id
                and accepted.anchor_revision==proposal.version.anchor_revision
                and accepted.map_geometry_revision==proposal.version.map_geometry_revision),
            source_age_ms=round((self.now_ns()-nanoseconds(proposal.source_stamp))*1e-6,3))
        # A prior map-ACK/freshness wait is no longer the reason once the
        # same immutable task has actually been delivered to native.
        self.error=''
        return True

    def deliver_support(self,proposal,support):
        """Transfer static evidence without restamping or re-admitting geometry.

        The original proposal's deadline bounds Native admission only. An
        accepted reference may still receive its exact static support later;
        current map/body/proof freshness remains Native's separate concern.
        """
        from .segment_transition import same_task
        record=self.prepared_for(proposal.version)
        route=self.transport.route
        if (self.transport.quarantined or route is None or record is None
                or record.proposal!=proposal or not record.delivered
                or record.retirement_requested
                or record.receipt is not None and not record.receipt.accepted
                or not same_task(proposal.version,route.snapshot)
                or support.version!=proposal.version or support.source_stamp!=proposal.source_stamp):
            return False
        # Move the immutable worker value. A deep copy here blocked every
        # navigation callback and made already-fresh proposals expire.
        record.support=support
        record.support_request=None
        if self.error.startswith('support_preparation_failed:'):
            self.error=''
        self.send_support(record)
        return True

    def send_support(self,record):
        record.last_support_delivery_monotonic=self.mono()
        if self.support_delivery is not None:
            record.support_inflight=True
            displaced=self.support_delivery.submit(record.proposal.proposal_id,record.support)
            if displaced is not None and displaced!=record.proposal.proposal_id:
                for previous in self.prepared.values():
                    if previous.proposal.proposal_id==displaced:
                        previous.support_inflight=False
        else:
            try:
                self.publish('support',record.support)
                record.support_sent=True
                if self.error.startswith('support_delivery_failed:'):
                    self.error=''
            except Exception as error:
                self.error='support_delivery_failed:'+type(error).__name__+':'+str(error)

    def harvest_support_delivery(self):
        if self.support_delivery is None:
            return False
        result,pending=self.support_delivery.take_state()
        for record in self.prepared.values():
            record.support_inflight=record.proposal.proposal_id in pending
        if result is None:
            return False
        record=next((r for r in self.prepared.values() if r.proposal.proposal_id==result.key),None)
        if record is not None:
            record.support_sent=not result.error
            if result.error and record.proposal==self.proposal:
                self.error='support_delivery_failed:'+result.error
            elif not result.error and record.proposal==self.proposal and self.error.startswith('support_delivery_failed:'):
                self.error=''
        return True

    def retry_unacknowledged_delivery(self):
        """Recover a missing admission fact, never renew input or motion leases.

        Native answers an exact repeated request with its original receipt.
        If it never admitted the request, its now-expired original deadline
        produces a rejection instead. Only that matched fact releases the
        preparation slot; no receipt, timer or publication proves retirement.
        """
        if self.transport.quarantined or self.proposal is None:
            return False
        record=self.prepared_for(self.proposal.version)
        if (record is None or record.proposal!=self.proposal or not record.delivered
                or record.receipt is not None
                or self.mono()-record.last_delivery_monotonic<.5):
            return False
        record.last_delivery_monotonic=self.mono()
        try:
            self.publish('proposal',deepcopy(record.proposal))
        except Exception as error:
            self.error='reference_proposal_delivery_failed:'+type(error).__name__+':'+str(error)
        return True

    def on_receipt(self,message):
        record=self.prepared_for(message.version)
        p=record.proposal if record is not None else None
        if (self.transport.quarantined or p is None or not record.delivered
                or message.proposal_id!=p.proposal_id or message.version!=p.version
                or message.expected_version!=p.expected_version or message.expected_trajectory_id!=p.expected_trajectory_id
                or message.transport_mode!=self.p['transport_mode']
                or nanoseconds(message.source_stamp)<nanoseconds(p.source_stamp)
                or nanoseconds(message.source_stamp)>self.now_ns()+20_000_000
                or nanoseconds(message.valid_until)<nanoseconds(message.source_stamp)
                or message.accepted and nanoseconds(message.source_stamp)>nanoseconds(p.valid_until)):
            return False
        # Receipt describes native admission at the original delivery time,
        # not current sensor validity. A delayed accepted receipt must survive
        # an already-issued writer CAS; it never renews a motion/sensor lease.
        if record.receipt is not None:
            return message==record.receipt and message.accepted
        record.receipt=deepcopy(message)
        self.trace('receipt',accepted=message.accepted,reason=message.reason,
            generation=message.version.reference_generation,
            expected_curve=message.expected_trajectory_id,
            owner_curve=self.local_commit.trajectory_id if self.local_commit else -1)
        if self.proposal is not None and self.proposal.version==p.version:
            self.receipt=deepcopy(message)
            if not message.accepted:
                self.retire_proposal('candidate_reference_rejected')
            elif self.error.startswith(('waiting_native_reference_receipt:',
                                        'reference_proposal_delivery_failed:')):
                self.error=''
        if not message.accepted and not record.handoffs and not record.installed:
            self.prepared.pop(message.version.reference_generation,None)
        self.reconcile_deferred_permit()
        self.reconcile_local_commit()
        self.retire_fenced_reanchors(record)
        return message.accepted

    def on_handoff(self,message):
        if self.geometry_owner=='local_executor':
            return False
        record=self.prepared_for(message.candidate.version)
        route=self.transport.route
        if (self.transport.quarantined or route is None or record is None
                or message.schema_version!=2 or not message.handoff_id or message.sequence<=0
                or message.candidate.transport_mode!=self.p['transport_mode']
                or message.candidate.version.task_id!=route.task_id
                or message.candidate.version.route_hash!=route.route_hash
                or not record.delivered):
            return False
        key=(execution_binding(message.candidate),message.handoff_id,message.sequence)
        previous=record.handoffs.get(key)
        if previous is not None:
            # Revocation is not a never-applied proof. Only an exact writer
            # outcome can release an undecided transaction.
            return previous==message or message.revoked
        if message.sequence<=record.closed_handoff_sequence.get(key[0],0):
            return False  # Retired reliable redelivery cannot recreate a claim.
        for old_key,ack in list(record.acknowledgements.items()):
            if not ack.applied or old_key in record.reconciled_handoffs:
                record.handoffs.pop(old_key,None);record.acknowledgements.pop(old_key,None)
                record.reconciled_handoffs.discard(old_key)
        if (message.revoked or not message.candidate.allowed or message.candidate.geometry_committed
                or not message.candidate.execution_id or message.candidate.control_epoch<=0
                or not message.candidate.sdk_session or message.candidate.sdk_arm_generation<=0
                or not 0<nanoseconds(message.source_stamp)<=self.now_ns()+20_000_000
                or nanoseconds(message.valid_until)<=nanoseconds(message.source_stamp)
                or nanoseconds(message.transition_deadline)>nanoseconds(message.valid_until)
                or nanoseconds(message.transition_deadline)<=nanoseconds(message.source_stamp)
                or len(record.handoffs)>=self.prepared_limit
                or key[0] not in record.closed_handoff_sequence
                    and len(record.closed_handoff_sequence)>=self.prepared_limit):
            return False
        # One writer per exact execution binding. Cross-topic arrival may be
        # after SDK Applied; the original fixed transaction still identifies
        # that outcome even if its deadline has elapsed in transit.
        record.handoffs[key]=deepcopy(message)
        early=self.early_acks.pop(key,None)
        if early is not None:
            self.on_commit_ack(early)
        return True

    def on_commit_ack(self,message):
        if self.geometry_owner=='local_executor':
            return False
        record=self.prepared_for(message.candidate_version)
        if (self.transport.quarantined or record is None or message.schema_version!=1
                or message.transport_mode!=self.p['transport_mode']
                or not message.handoff_id or message.grant_sequence<=0 or message.sequence<=0):
            return False
        binding=(message.execution_id,message.control_epoch,message.sdk_session,
                 message.sdk_arm_generation,message.transport_mode)
        key=(binding,message.handoff_id,message.grant_sequence)
        grant=record.handoffs.get(key)
        if grant is None:
            # ACK and grant are separate reliable topics. Keep a fixed small
            # buffer, never install geometry or grant execution from an ACK.
            if message.grant_sequence<=record.closed_handoff_sequence.get(binding,0):
                return False
            if key not in self.early_acks and len(self.early_acks)>=self.prepared_limit:
                self.error='reference_ack_buffer_full_waiting_owner_resolution'
                return False
            old=self.early_acks.get(key)
            if old is not None and old!=message:
                return False
            self.early_acks[key]=deepcopy(message)
            return True
        if (message.incumbent_version!=grant.incumbent.version
                or message.candidate_version!=grant.candidate.version
                or message.incumbent_trajectory_id!=grant.incumbent.trajectory_id
                or message.candidate_trajectory_id!=grant.candidate.trajectory_id
                or message.permit_sequence!=grant.candidate.sequence
                or message.previous_commit_sequence!=grant.expected_commit_sequence):
            return False
        if message.applied:
            if (message.commit_sequence!=message.previous_commit_sequence+1
                    or message.entry_admission_sequence<=0
                    or not nanoseconds(grant.source_stamp)<=nanoseconds(message.applied_at)
                        <=nanoseconds(grant.transition_deadline)):
                return False
        elif (message.commit_sequence!=message.previous_commit_sequence
                or message.write_submitted or message.write_acknowledged):
            return False
        old=record.acknowledgements.get(key)
        if old is not None:
            if old==message:
                return True
            # Vendor callback may acknowledge the very same irreversible
            # writer call later. It cannot change any selection/source fact.
            previous,updated=deepcopy(old),deepcopy(message)
            previous.sequence=updated.sequence=0
            previous.write_acknowledged=updated.write_acknowledged=False
            previous.reason=updated.reason=''
            if previous!=updated or message.sequence<=old.sequence:
                return False
        record.acknowledgements[key]=deepcopy(message)
        if not message.applied:
            record.closed_handoff_sequence[binding]=max(message.grant_sequence,
                record.closed_handoff_sequence.get(binding,0))
        if not message.applied and record.retirement_requested and self.proposal is not None:
            if self.proposal.version==record.proposal.version:
                self.retire_proposal('writer_rejected_retired_reference')
        self.reconcile_deferred_permit()
        self.reconcile_owner_facts()
        return True

    def reconcile_deferred_permit(self):
        pending=self.deferred_permit
        if pending is None:
            return False
        self.deferred_permit=None
        return self.on_permit(pending)

    def preserve_prepared_observations(self,record):
        """Keep measured portal history only for this exact pending geometry."""
        t=self.transport
        candidate=t.pending_core
        if (candidate is None or t.pending is None
                or candidate.execution_mode_policy!='fixed_stair'
                or not candidate._stair_boundary()
                or version_of(t.pending)!=record.proposal.version
                or candidate.context!=record.core.context
                or candidate.task_id!=record.core.task_id
                or candidate.snapshot.route_hash!=record.core.snapshot.route_hash
                or candidate.anchor!=record.core.anchor
                or candidate._segment!=record.core._segment
                or candidate._last_sample is None):
            return
        previous=record.observed_core or record.core
        if (previous._last_sample is None
                or candidate._last_sample[0]>previous._last_sample[0]):
            # This also preserves a genuine retreat that clears a previously
            # witnessed portal. Never retain just a sticky crossing flag.
            record.observed_core=deepcopy(candidate)

    def install_prepared_reference(self,record):
        """Install only the immutable geometry named by the sole owner's fact."""
        t=self.transport
        matching_pending=(t.pending is not None and t.pending==record.proposal.reference)
        # An immutable pending snapshot may predate an independently received
        # map correction. Committing that snapshot must not erase the newer
        # correction candidate. Preserve the literal candidate only; accepted
        # geometry keeps its original anchor until a later owner commit.
        same_route=(t.core is not None and t.core.context==record.core.context
            and t.core.task_id==record.core.task_id
            and t.core.snapshot.route_hash==record.core.snapshot.route_hash)
        correction=t.core.candidate_anchor if same_route else None
        self.preserve_prepared_observations(record)
        # Ordinary rolling windows share the exact immutable route, anchor
        # and semantic segment. Their owner ACK can arrive after >1 m of
        # *already observed* intermediate motion. Reuse that continuous real
        # projection instead of jumping once from the old preparation body
        # and failing the unchanged bounded-projection check. A reanchor or
        # segment transition is different geometry and cannot take this path.
        live_progress=(same_route and t.core.anchor==record.core.anchor
            and t.core._segment==record.core._segment and t.core._progress is not None
            and (record.core._progress is None
                or t.core._progress.source_ns>record.core._progress.source_ns))
        observed=record.observed_core or record.core
        if (live_progress and (observed._last_sample is None
                or t.core._last_sample[0]>=observed._last_sample[0])):
            observed=t.core
        try:
            t._install_committed_window(record.proposal.reference,deepcopy(observed),
                received_monotonic=record.received,end_arc_m=record.end_arc_m,
                start_arc_m=record.start_arc_m,issue_arc_m=record.issued_confirmed_arc_m,
                current_source_ns=self.now_ns(),now_monotonic=self.mono(),owner_committed=True)
        except ValueError as error:
            self.error=str(error)
            return False
        previous=t.core.candidate_anchor or t.core.anchor
        if (correction is not None and correction.context==t.core.context
                and correction.source_ns>previous.source_ns and correction.revision>previous.revision):
            t.core.candidate_anchor=correction
        record.installed=True
        if matching_pending:
            # Retire only the slot whose exact geometry was installed. An
            # older successful owner result is not a cancellation of a newer
            # support worker, reliable delivery or pending receipt.
            self.invalidate_preparation()
            t.discard_pending()
            if self.proposal==record.proposal:
                self.proposal=self.receipt=None
        return True

    def on_local_commit(self,message):
        """Consume Native's acknowledged local geometry, never an SDK write ACK.

        The event can be delayed; its source remains original. A current real
        body sample reprojects its immutable core, while only native proof
        keeps collision readiness fresh. No event grants movement.
        """
        if self.geometry_owner!='local_executor' or self.transport.quarantined:
            return False
        route=self.transport.route
        record=self.prepared_for(message.version)
        if route is None or record is None or not record.delivered:
            return False
        proposal=record.proposal
        try:
            source,body=nanoseconds(message.source_stamp),nanoseconds(message.body_source_stamp)
        except (ValueError,TypeError,AttributeError):
            return False
        if (message.schema_version!=1 or message.transport_mode!=self.p['transport_mode']
                or message.version.session_id!=self.p['session_id']
                or message.version.map_version_id!=self.p['map_version_id']
                or message.version.task_id!=route.task_id or message.version.route_id!=route.route_id
                or message.version.route_hash!=route.route_hash or message.proposal_id!=proposal.proposal_id
                or not nanoseconds(proposal.source_stamp)<=source<=self.now_ns()
                or not 0<body<=source or message.trajectory_id<0
                or not message.accepted or message.retired or message.disposition!='committed'):
            return False
        previous=self.local_commit
        if message.accepted:
            if message.commit_sequence<=0 or message.native_validation_sequence<=0:
                return False
            if previous is not None and message.commit_sequence<=previous.commit_sequence:
                return message==previous
            p=message.measured_pose;t=message.measured_twist
            values=tuple(getattr(p.position,k) for k in 'xyz')+tuple(getattr(p.orientation,k) for k in 'xyzw')
            values+=tuple(getattr(v,k) for v in (t.linear,t.angular) for k in 'xyz')
            if (not all(math.isfinite(v) for v in values+(message.curve_time,message.arc_length))
                    or message.curve_time<0. or message.arc_length<0.
                    or abs(sum(getattr(p.orientation,k)**2 for k in 'xyzw')-1.)>1e-3):
                return False
        queued=self.deferred_local_commits.get(message.commit_sequence)
        if queued is not None and queued!=message:
            self.trace('commit_rejected',reason='commit_sequence_content_changed',
                sequence=message.commit_sequence)
            return False
        same_reference=previous is not None and message.version==previous.version
        # Proposal predecessor is an immutable preparation hint. Ordinary
        # incumbent refreshes may advance its curve while this reference waits;
        # the executor's successful commit records the actual current CAS.
        expected_version=previous.version if previous is not None else proposal.expected_version
        expected_trajectory=previous.trajectory_id if previous is not None else proposal.expected_trajectory_id
        observed_gap=False
        if message.expected_version!=expected_version or message.expected_trajectory_id!=expected_trajectory:
            # This is a Native *successful CAS result*, not a request to do
            # another CAS here. Reliable topics are independent, and an ACK
            # can predate this observer's subscription. Requiring every
            # intermediate result permanently froze the expected-curve water-
            # mark. Recover only an already-known forward geometry domain,
            # and still require this exact commit's original whole proof and
            # accepted proposal receipt below. No motion authority is issued.
            predecessor=self.prepared_for(message.expected_version)
            forward=(predecessor is not None
                and 0<=message.expected_trajectory_id<message.trajectory_id
                and message.expected_version.task_id==route.task_id
                and message.expected_version.route_id==route.route_id
                and message.expected_version.route_hash==route.route_hash
                and message.expected_version.reference_generation<=message.version.reference_generation
                and (previous is None
                    or message.version.reference_generation>=previous.version.reference_generation
                        and nanoseconds(message.source_stamp)>=nanoseconds(previous.source_stamp)
                        and (message.expected_version.reference_generation>previous.version.reference_generation
                            or message.expected_version==previous.version
                                and message.expected_trajectory_id>previous.trajectory_id)))
            if not forward:
                return False
            if predecessor.receipt is None:
                self.defer_local_commit(message,'waiting_known_predecessor_receipt')
                return False
            if not predecessor.receipt.accepted:
                return False
            observed_gap=True
        proof=self.local_commit_proofs.get((execution_version_identity(message.version),
            message.trajectory_id,message.native_validation_sequence))
        if proof is not None and (message.map_geometry_revision!=proof.map_snapshot_revision
                or nanoseconds(proof.valid_until)!=0 or nanoseconds(proof.check_end)>source
                or nanoseconds(proof.body_source_stamp)>body):
            return False
        if record.receipt is None or proof is None:
            self.defer_local_commit(message,'waiting_original_receipt_or_proof')
            return False  # Native receipt/verdict and commit are separate reliable facts.
        if not record.receipt.accepted:
            return False
        if not same_reference and not self.install_prepared_reference(record):
            return False
        self.local_commit=deepcopy(message)
        self.deferred_local_commits={sequence:fact for sequence,fact in self.deferred_local_commits.items()
            if sequence>message.commit_sequence}
        self.deferred_local_commit=(self.deferred_local_commits[min(self.deferred_local_commits)]
            if self.deferred_local_commits else None)
        # Never retire the pending reference merely because the incumbent
        # refreshed. Its literal geometry stays staged for an actual-current CAS.
        self.prune_prepared()
        self.publish_route_progress()
        self.error=''
        self.trace('commit',sequence=message.commit_sequence,curve=message.trajectory_id,
            generation=message.version.reference_generation,
            deferred=len(self.deferred_local_commits),native_snapshot_recovery=observed_gap,
            previous_observed_curve=expected_trajectory,
            native_previous_curve=message.expected_trajectory_id)
        self.reconcile_local_commit()
        return True

    def on_permit(self,message):
        if self.geometry_owner=='local_executor':
            return False  # Task authorization is not per-curve geometry ownership.
        if (self.transport.quarantined or message.transport_mode!=self.p['transport_mode'] or message.version.schema_version!=3
                or message.version.session_id!=self.p['session_id']
                or message.version.map_version_id!=self.p['map_version_id']
                or not nanoseconds(message.source_stamp)<=self.now_ns()<=nanoseconds(message.valid_until)
                or self.permit is not None and message.sequence<=self.permit.sequence):
            return False
        route=self.transport.route
        if (route is None or message.version.task_id!=route.task_id or message.version.route_hash!=route.route_hash):
            return False
        if self.transport.accepted is None or message.version!=version_of(self.transport.accepted):
            record=self.prepared_for(message.version)
            if not message.geometry_committed or record is None or not record.delivered:
                return False
            if record.receipt is None:
                if self.deferred_permit is None or message.sequence>self.deferred_permit.sequence:
                    self.deferred_permit=deepcopy(message)
                return False
            if not record.receipt.accepted:
                return False
            # BT version commit, not receipt, installs the prepared reference.
            # Reconstruct its exact immutable snapshot even if the planning
            # slot has already moved on. Never transplant the latest anchor.
            if not self.install_prepared_reference(record):
                return False
        if self.permit is not None and execution_binding(self.permit)!=execution_binding(message):
            self.clear_native_debug()
        self.permit=deepcopy(message)
        self.deferred_permit=None
        self.reconcile_owner_facts()
        if self.proposal is not None and (self.proposal.expected_version!=message.version
                or self.proposal.expected_trajectory_id!=message.trajectory_id):
            self.retire_proposal('candidate_predecessor_advanced')
        self.prune_prepared()
        self.publish_route_progress()
        return True

    def on_debug(self,message):
        if (message.session_id!=self.p['session_id'] or message.header.frame_id!='d1max_loc_odom'
                or message.header.stamp.sec<0 or not 0<=message.header.stamp.nanosec<1_000_000_000):
            return False
        source_ns=nanoseconds(message.header.stamp)
        if message.phase=='cancelled':
            # Native cancellation is a one-shot software retirement fact,
            # not a renewable curve/environment check. Delivery latency must
            # not turn a proven retirement into permanent ownership uncertainty.
            t=self.transport; pending=self.pending_route_retirement
            pending_match=bool(pending is not None and pending['required']
                and pending['generation']==message.generation)
            retired_match=bool(t.quarantined and t._retired_generation==message.generation
                and t._retired_stamp_ns>0)
            if (message.valid or self.cancel_generation<=0 or message.generation!=self.cancel_generation
                    or not 0<source_ns<=self.now_ns() or not (pending_match or retired_match)
                    or source_ns<t._retired_stamp_ns
                    or pending_match and source_ns<nanoseconds(pending['command'].source_stamp)):
                return False
            previous=self.native_retirement
            if previous is not None and previous['generation']==message.generation:
                if source_ns<previous['source_ns']:
                    return False
                # Repeated delivery is not a new native event/receipt.
            else:
                self.native_retirement=dict(generation=message.generation,
                    source_ns=source_ns,received=self.mono())
            if pending_match:
                self.finish_route_retirement(native_confirmed=True)
            self.try_recover_context()
            return True
        if not 0<=self.now_ns()-source_ns<=self.freshness_ns:
            return False
        key=message.generation,message.plan_id
        previous=self.debug_facts.get(key)
        if previous is not None and nanoseconds(message.header.stamp)<=nanoseconds(previous[0].header.stamp):
            # Duplicate delivery is not a new native check. An older positive
            # report cannot revive a newer negative report of this same curve.
            return message==previous[0]
        self.debug,self.debug_received=deepcopy(message),self.mono()
        self.debug_facts[key]=(self.debug,self.debug_received)
        while len(self.debug_facts)>self.prepared_limit:
            retired=min((k for k in self.debug_facts if k!=self.committed_debug_key()),
                key=lambda k:self.debug_facts[k][1])
            self.debug_facts.pop(retired)
        return True

    def try_recover_context(self):
        t=self.transport; ack=self.native_retirement
        if not t.quarantined or not self.context_ready:
            return False
        context=(self.context['session_id'],self.context['epoch'],self.context['seed_id'])
        if self.p.get('local_state_enabled'):
            latest=self.local_inbox.latest
            if (latest is None or latest.state is None or latest.identity!=context
                    or not self.local_inbox.usable(now_ns=self.now_ns(),monotonic=self.mono(),
                        receipt_timeout_s=self.transport.freshness)):
                return False
        try:
            t.recover_empty_context(context_sequence=self.context['sequence'],context=context,
                current_source_ns=self.now_ns(),now_monotonic=self.mono(),
                native_reference_unretired=bool(self.native_route_exposures
                    or self.pending_route_retirement is not None or self.writer_unresolved()))
        except ValueError:
            pass
        else:
            self.proposal=self.receipt=self.permit=None
            self.clear_native_debug(); self.error=''
            return True
        if not ack:
            return False
        # The exact old operation stays retired while a NEW context/body is
        # acquired. Only that new state is leased; the immutable retirement
        # fact must match the current barrier and cannot reopen another owner.
        if (ack['generation']!=self.cancel_generation or ack['generation']!=t._retired_generation
                or not 0<ack['source_ns']<=self.now_ns() or ack['source_ns']<t._retired_stamp_ns
                or self.mono()<ack['received'] or self.native_route_exposures
                or self.pending_route_retirement is not None or self.writer_unresolved()):
            return False
        try:
            t.recover_retired_context(retired_generation=ack['generation'],
                context_sequence=self.context['sequence'],
                context=(self.context['session_id'],self.context['epoch'],self.context['seed_id']),
                current_source_ns=self.now_ns(),now_monotonic=self.mono())
        except ValueError:
            return False
        self.proposal=self.receipt=self.permit=self.native_retirement=None
        self.clear_native_debug()
        self.error=''
        return True

    def reference_inputs_ready(self):
        """Reference geometry depends on coordinates, not a second map lease.

        Native SCAN owns map readiness, collision and real input-loss handling.
        A delayed map status message must not freeze the measured route window.
        Pose and route-source checks remain at their actual consumers.
        """
        if self.geometry_owner=='local_executor':
            return self.context is not None and self.context_ready and not self.transport.quarantined
        return self.map_fresh()

    def map_fresh(self):
        m=self.map_report
        return bool(self.context_ready and m and m.get('valid') is True
            and 0<=self.mono()-self.map_received<=self.transport.freshness
            and 0<=self.now_ns()-m['source_stamp_ns']<=self.map_evidence_age_ns
            and len(m.get('sources',[]))==2
            and all(type(v.get('sensor_id')) is int for v in m['sources'])
            and {v.get('sensor_id') for v in m['sources']}=={0,1}
            and all(type(v.get('integrated_stamp_ns')) is int
                and 0<=self.now_ns()-v['integrated_stamp_ns']<=self.map_evidence_age_ns for v in m['sources'])
            and m['source_stamp_ns']==min(v['integrated_stamp_ns'] for v in m['sources']))

    def harvest_preparation(self):
        """Consume a completed immutable result in the serialized owner lane.

        The production worker wakes this callback through a guard condition;
        tick remains a fallback. Neither entry point renews the proposal's
        original source, deadline or map evidence. A stale wake is a no-op.
        """
        self.harvest_support_delivery()
        if self.preparation is not None:
            result=self.preparation.take()
            if result is not None and self.proposal is not None and result.key==self.proposal.proposal_id:
                record=self.prepared_for(self.proposal.version)
                if record is not None:
                    record.support_preparing=False
                if result.error:
                    self.error='support_preparation_failed:'+result.error
                else:
                    return self.deliver_support(*result.value)
        return False

    def tick(self):
        self.harvest_preparation()
        self.sync_context()
        self.try_recover_context()
        if self.p.get('local_state_enabled') and self.local_inbox.usable(now_ns=self.now_ns(),
                monotonic=self.mono(),receipt_timeout_s=self.transport.freshness):
            self.refresh_window()
        if (self.proposal and self.now_ns()>nanoseconds(self.proposal.valid_until)
                and not (self.receipt is not None and self.receipt.accepted)):
            # This deadline bounds delivery to native. Its accepted receipt is
            # a durable geometry event, not a renewable sensor/command lease.
            # Keep accepted pending geometry until the fresh BT commit arrives.
            record=self.prepared_for(self.proposal.version)
            if record is not None and record.delivered and record.receipt is None:
                # Expiry asks for the original native admission fact, not for
                # geometry retirement. If the original accepted receipt arrives
                # later, it remains usable for a fresh owner geometry commit.
                self.error='waiting_native_reference_receipt:reference_delivery_expired_waiting_native_fact'
            else:
                self.retire_proposal('reference_delivery_expired_waiting_native_fact')
        self.retry_unacknowledged_delivery()
        if self.proposal is not None:
            from .segment_transition import same_task
            record=self.prepared_for(self.proposal.version)
            retryable=(record is not None and not record.retirement_requested
                and not self.transport.quarantined and self.transport.route is not None
                and same_task(record.proposal.version,self.transport.route.snapshot)
                and (record.receipt is None or record.receipt.accepted))
            if (retryable and record.support is None and record.support_request is not None
                    and not record.support_preparing
                    and self.mono()-record.last_support_preparation_monotonic>=.5):
                self.prepare_support(record)
            if (retryable and record.support is not None
                    and not record.support_sent and not record.support_inflight
                    and self.mono()-record.last_support_delivery_monotonic>=.5):
                self.send_support(record)
        if (self.transport.core and self.transport.pending is None and self.transport.accepted is None
                and self.reference_inputs_ready() and 0<=self.mono()-self.state_received<self.transport.freshness
                and self.mono()-self.last_proposal_monotonic>=self.rolling_policy.minimum_interval_s):
            try:
                self.propose_window()
            except ValueError as error: self.error=str(error)
        t=self.transport; now=self.now_ns(); core=t.core
        window=t.rolling_window_progress()
        if window is not None:
            self.trace('window',generation=t.accepted.generation if t.accepted else 0,
                pending_generation=t.pending.generation if t.pending else 0,
                start_arc_m=round(window.start_arc_m,3),end_arc_m=round(window.end_arc_m,3),
                issued_arc_m=round(window.issued_confirmed_arc_m,3),
                confirmed_arc_m=round(window.confirmed_arc_m,3),reason=self.error,
                correction_source_age_ms=(round((now-core.candidate_anchor.source_ns)*1e-6,3)
                    if core and core.candidate_anchor else None),
                local_source_age_ms=(round((now-t.latest_local.body.source_ns)*1e-6,3)
                    if t.latest_local else None),
                correspondence_source_age_ms=(round((now-core._last_sample[0])*1e-6,3)
                    if core and core._last_sample else None),
                owner_curve=self.local_commit.trajectory_id if self.local_commit else -1)
        pending_debug,pending_received=self.pending_debug_fact()
        if t.accepted is not None:
            d,debug_received=self.debug_facts.get(self.committed_debug_key(),(None,-math.inf))
        else:
            d,debug_received=(pending_debug,pending_received) if pending_debug else (self.debug,self.debug_received)
        debug_fresh=self.native_debug_fresh(d,debug_received)
        proof,proof_received=self.validation_facts.get(self.committed_validation_key(),(None,-math.inf))
        # Debug has no proof sequence/deadline: a delayed invalid heartbeat may
        # describe an earlier source outage, and a missing heartbeat may simply
        # lag an actual positive check. Only the exact committed native fact is
        # authority. Map relay freshness admits NEW reference inputs, not this
        # already issued proof; all original negative/source/permit fences stay.
        status_inputs=dict(reference=t.accepted,proof=proof,proof_received=proof_received,
            context_sequence=(self.context or {}).get('sequence',0),quarantined=t.quarantined,
            now_ns=now,now_monotonic=self.mono(),freshness_s=self.transport.freshness,
            transport_mode=self.p['transport_mode'])
        committed_status=(local_committed_trajectory_status(commit=self.local_commit,**status_inputs)
            if self.geometry_owner=='local_executor' else committed_trajectory_status(permit=self.permit,**status_inputs))
        candidate_proof,candidate_received=self.pending_validation_fact()
        candidate_status=candidate_preparation_status(reference=t.pending,proposal=self.proposal,
            receipt=self.receipt,proof=candidate_proof,proof_received=candidate_received,
            context_sequence=(self.context or {}).get('sequence',0),quarantined=t.quarantined,
            now_ns=now,now_monotonic=self.mono(),freshness_s=self.transport.freshness,
            transport_mode=self.p['transport_mode'],stateful=self.geometry_owner=='local_executor')
        # A no-solution notification is a versioned attempt outcome, not a
        # collision lease. Native can wait without repeating that event. Keep
        # its cause until this pending version changes; never refresh its
        # timestamp or use this diagnostic as trajectory/control validity.
        pending_phase=pending_debug.phase if pending_debug and not pending_debug.valid else ''
        valid=committed_status.valid
        debug_phase=committed_status.phase if t.accepted is not None else (
            d.phase if d else 'waiting_native_trajectory')
        context=self.context or {}
        record=dict(schema=1,session_id=self.p['session_id'],received_at_unix=now*1e-9,callback_wall_time=time.time(),
            localization_session_id=self.p['session_id'],localization_epoch=context.get('epoch',0),
            localization_seed_id=context.get('seed_id',''),native_map_context_ready=self.context_ready,
            native_map_context_fault='',sensor_ready=self.map_fresh(),ready=self.map_fresh(),motion_enabled=False,
            active_reference=t.accepted is not None,generation=t.generation,
            owner_reference_stamp=nanoseconds(t.route.source_stamp)*1e-9 if t.route else 0.,
            owner_reference_stamp_ns=t.ingress.last_stamp_ns if t.route else 0,
            owner_delivery_sequence=t.ingress.last_sequence if t.route else 0,
            owner_task_id=t.route.task_id if t.route else '',
            owner_route_id=t.route.route_id if t.route else '',
            owner_route_hash=t.route.route_hash if t.route else '',
            pending_reference_generation=t.pending.generation if t.pending else 0,
            prepared_reference_count=len(self.prepared),writer_reference_unresolved=self.writer_unresolved(),
            geometry_owner=self.geometry_owner,execution_mode_policy=self.execution_mode_policy,
            local_commit_sequence=self.local_commit.commit_sequence if self.local_commit else 0,
            deferred_local_commit_count=len(self.deferred_local_commits),
            retired_reference_candidate=t.status.get('rejected_pending_identity'),
            native_reference_received=bool(self.receipt and self.receipt.accepted),
            local_debug_generation=d.generation if d else 0,local_debug_fresh=debug_fresh,
            native_debug_phase=d.phase if d else '',native_debug_valid=bool(d and d.valid),
            native_pending_phase=pending_phase,
            native_pending_debug_generation=pending_debug.generation if pending_debug else 0,
            native_pending_debug_fresh=self.native_debug_fresh(pending_debug,pending_received),
            local_validation_sequence=proof.sequence if proof else 0,
            local_validation_valid_until_ns=nanoseconds(proof.valid_until) if proof else 0,
            candidate_admission_ready=candidate_status.valid,candidate_plan_id=candidate_status.plan_id,
            candidate_phase=candidate_status.phase,
            candidate_validation_sequence=candidate_proof.sequence if candidate_proof else 0,
            candidate_validation_valid_until_ns=nanoseconds(candidate_proof.valid_until) if candidate_proof else 0,
            spline_visual_valid=valid,last_spline_id=committed_status.plan_id,
            local_debug_phase=debug_phase,reason=self.error,
            context_reset_reason=self.context_reset_reason,context_reset_source_ns=self.context_reset_source_ns,
            route_native_work_unretired=bool(self.native_route_exposures),
            route_retirement_pending=self.pending_route_retirement is not None,
            local_debug_progress_arc_m=core._progress.measured_arc_m if core and core._progress else 0.)
        self.publish('status',record)
        return record


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from d1max_navigation_bt_interfaces.msg import RouteReference
    from d1max_planning_interfaces.msg import (NavigationState,LocalNavigationState,ReferenceProposal,ReferenceReceipt,
        ExecutionPermit,ExecutionHandoffGrant,ExecutionCommitAck,SupportReference,ReferencePath,LocalPlanDebug,RouteProgress,TrajectoryValidation,
        RouteDeliveryReceipt,LocalTrajectoryCommit,TaskExecutionLease,ModeTransitionRequest,ModeTransitionState)
    class ReferenceNode(Node):
        def __init__(self):
            super().__init__('d1max_continuous_reference')
            defaults=dict(session_id='',map_version_id='',expected_source_map_sha256='',
                expected_tomogram_sha256='',expected_conditioning_sha256='',body_height_m=.55,
                body_height_calibration_id='',freshness_s=.4,map_evidence_max_age_s=.4,transport_mode='live',planning_manifest='',
                reference_horizon_m=2.,floor_id='',local_state_enabled=False,async_support_preparation=True,
                geometry_owner='local_executor',execution_purpose='execution',execution_mode_policy='segment_modes')
            p={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
            durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            delivery_qos=QoSProfile(depth=8,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            pubs={key:self.create_publisher(kind,PREFIX+topic,delivery_qos if key=='route_receipt'
                else durable if key in ('context','support','proposal') else 5)
                for key,kind,topic in (('body',Odometry,'body_pose'),('context',String,'scan_map_context'),
                    ('status',String,'scan_bridge_status'),('proposal',ReferenceProposal,'execution/reference_proposal'),
                    ('route_receipt',RouteDeliveryReceipt,'execution/route_delivery_receipt'),
                    ('route_progress',RouteProgress,'execution/route_progress'),
                    ('mode_request',ModeTransitionRequest,'execution/mode_transition_request'),
                    ('support',SupportReference,'execution/support'),('cancel_reference',ReferencePath,'scan_reference'))}
            # Guard callbacks share the default mutually-exclusive owner group
            # with subscriptions and tick. Preparation only triggers this
            # wake. The separate bounded delivery worker publishes immutable
            # static support only, never proposals or reference ownership.
            self.preparation_guard=self.create_guard_condition(self.on_preparation_ready)
            self.callbacks=ReferenceCallbacks(p,lambda key,value:pubs[key].publish(String(data=json.dumps(value))
                if key in ('context','status') else value),lambda:self.get_clock().now().nanoseconds,
                diagnostic=lambda text:self.get_logger().info(text),
                preparation_ready=self.preparation_guard.trigger)
            c=self.callbacks
            subscriptions=[(RouteReference,PREFIX+'committed_route',c.on_route,durable),
                    (NavigationState,'/d1max/localization/navigation/state',c.on_navigation,qos_profile_sensor_data),
                    (ReferenceReceipt,PREFIX+'execution/reference_receipt',c.on_receipt,5),
                    (LocalPlanDebug,PREFIX+'native_local_debug',c.on_debug,5),
                    (TrajectoryValidation,PREFIX+'execution/validation',c.on_validation,8),
                    (TaskExecutionLease,PREFIX+'execution/task_lease',c.on_task_lease,8),
                    (ModeTransitionState,PREFIX+'execution/mode_transition_state',c.on_mode_transition,8),
                    (String,PREFIX+'scan_map_context_ack',lambda m:c.on_context_ack(json.loads(m.data)),durable),
                    (String,PREFIX+'rays_status',lambda m:c.on_map(json.loads(m.data)),5)]
            if p['geometry_owner']=='local_executor':
                subscriptions.append((LocalTrajectoryCommit,PREFIX+'execution/local_commit_result',c.on_local_commit,delivery_qos))
            else:
                subscriptions.extend(((ExecutionPermit,PREFIX+'execution/permit',c.on_permit,5),
                    (ExecutionHandoffGrant,PREFIX+'execution/handoff_grant',c.on_handoff,8),
                    (ExecutionCommitAck,PREFIX+'execution/commit_ack',c.on_commit_ack,8)))
            for kind,topic,callback,qos in subscriptions:
                self.create_subscription(kind,topic,lambda m,fn=callback:self.checked(fn,m),qos)
            if p['local_state_enabled']:
                self.create_subscription(LocalNavigationState,'/d1max/localization/navigation/local_state',
                    lambda m:self.checked(c.on_local_navigation,m),
                    QoSProfile(depth=8,reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(.05,c.tick)
        def on_preparation_ready(self):
            try: self.callbacks.harvest_preparation()
            except (ValueError,TypeError,AttributeError,KeyError) as error: self.callbacks.error=str(error)
        def checked(self,callback,message):
            try: callback(message)
            except (ValueError,TypeError,AttributeError,KeyError) as error: self.callbacks.error=str(error)
    rclpy.init(args=args); node=ReferenceNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException): pass
    finally:
        if not node.callbacks.close():
            # Never destroy a publisher/node while its middleware call is
            # still entered on another thread. This is a failed process exit,
            # not a clean retirement ACK; the session supervisor owns the
            # bounded process shutdown. Workers are daemons and own no motion.
            raise RuntimeError('reference_workers_not_joined_before_node_shutdown')
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
