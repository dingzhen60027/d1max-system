"""Bounded candidate admission; only the sole writer ACK changes active identity.

The conditional grant is never injected into the ordinary motion gate. Both
slots retain original evidence deadlines. Rejection of a candidate cannot
clear a safe incumbent, and an irreversible write fact cannot roll back.
"""
from collections import OrderedDict, deque
from copy import deepcopy
import math

from .atomic_projection_state import stamp_ns
from .command_admission import CommandAdmission
from .execution_safety import Decision, ExecutionSafety, binding, fresh, version


def task_binding(message):
    v=message.version
    return (v.schema_version,v.session_id,v.task_id,v.route_id,v.route_hash,
            v.map_version_id,v.localization_epoch,v.localization_seed_id,
            message.execution_id,message.control_epoch,message.sdk_session,
            message.sdk_arm_generation,message.transport_mode)


def vector(obj):
    return tuple(getattr(obj,a) for a in 'xyz')


def distance(a,b):
    values=tuple(x-y for x,y in zip(vector(a),vector(b)))
    return math.sqrt(sum(x*x for x in values)) if all(math.isfinite(x) for x in values) else math.inf


class CandidateSafety(ExecutionSafety):
    def _admit(self,demand,now_ns,**kwargs):
        # Zero proposed at a new geometry entry is not an obstacle clearance.
        return super()._admit(demand,now_ns,require_zero_sweep=True,**kwargs)


class ExecutionHandoffAdmission:
    def __init__(self,core,*,enabled=False):
        self.core=core
        self.ordinary=CommandAdmission(core)
        self.enabled=enabled
        self.commit_sequence=0
        self.applied=None
        self.grant=None
        self.candidate=None
        self.prepared=OrderedDict()
        self.finished=deque(maxlen=8)
        self.last_grant_sequence=0
        self.last_ack_sequence=0
        self.pending_revoked=False
        self.failed_write_binding=None
        self.stationary=deque(maxlen=8)

    def on_stationary(self,message):
        # These are original SDK-published witnesses, not local-odom velocity.
        if message.schema_version!=1 or message.sequence<=0:return False
        if self.stationary and message.sequence<=self.stationary[-1].sequence:return False
        self.stationary.append(deepcopy(message))
        if (not message.usable and self.grant is not None and self.candidate is not None
                and self.grant.transition_mode==1
                and task_binding(message)==task_binding(self.grant.incumbent)
                and version(message.version)==version(self.grant.incumbent.version)
                and message.applied_trajectory_id==self.grant.incumbent.trajectory_id
                and message.writer_commit_sequence==self.grant.expected_commit_sequence):
            self._retire_pending()
        return True

    def _stationary_matches(self,g,now_ns):
        policy=self.core.stationary_policy
        e=g.stationary_evidence;a=g.incumbent
        source=stamp_ns(e.source_stamp);received=stamp_ns(e.received_stamp)
        lower=stamp_ns(e.capture_lower_bound);upper=stamp_ns(e.capture_upper_bound)
        speed=e.measured_linear_mps;yaw=e.measured_angular_radps
        delay=e.mc_capture_delay_bound_sec
        latest=self.stationary[-1] if self.stationary else None
        return (latest is not None and latest.usable and task_binding(latest)==task_binding(e)
            and latest.writer_commit_sequence==e.writer_commit_sequence
            and latest.applied_trajectory_id==e.applied_trajectory_id
            and latest.zero_write_sequence==e.zero_write_sequence
            and any(e==published for published in self.stationary)
            and e.schema_version==1 and e.usable and e.nonzero_blocked
            and version(e.version)==version(a.version) and e.applied_trajectory_id==a.trajectory_id
            and task_binding(e)==task_binding(a) and e.writer_commit_sequence==g.expected_commit_sequence
            and e.zero_write_sequence>0 and e.mc_raw_stamp_ns>0 and bool(e.mc_clock_epoch)
            and e.time_basis==('source_delta_host_anchor_approximate' if self.core.mode=='live'
                              else 'isolated_simulated_source_clock')
            and (self.core.mode!='live' or e.physical_acceptance_verified)
            and fresh(e.source_stamp,now_ns,250_000_000)
            and fresh(e.received_stamp,now_ns,250_000_000)
            and now_ns<stamp_ns(e.valid_until)<=source+250_000_000
            and stamp_ns(g.transition_deadline)<=stamp_ns(e.valid_until)
            and 0<stamp_ns(e.zero_ack_at)<lower<=upper<=received
            and lower-stamp_ns(e.zero_ack_at)>=round(policy['reentry_duration_s']*1e9)
            and math.isfinite(delay) and 0<=delay<=.25
            and math.isfinite(e.stationary_duration_sec) and e.stationary_duration_sec>=policy['reentry_duration_s']
            and e.stationary_samples>=policy['minimum_new_samples']
            and math.isfinite(speed) and 0<=speed<=policy['linear_threshold_mps']
            and math.isfinite(yaw) and 0<=yaw<=policy['angular_threshold_radps'])

    def on_permit(self,message):
        if (self.failed_write_binding is not None and message.allowed and not message.revoked
                and task_binding(message)==self.failed_write_binding):return False
        if (self.enabled and self.applied is not None and not message.revoked and message.allowed
                and task_binding(message)==task_binding(self.applied)
                and (binding(message)!=binding(self.applied) or message.trajectory_id!=self.applied.trajectory_id)):
            # A delayed incumbent heartbeat (or an ordinary new geometry
            # grant) cannot replace an identity already chosen by the writer.
            return False
        accepted=self.core.on_permit(message)
        stopped_reentry=(self.grant is not None and self.candidate is not None
            and self.grant.transition_mode==1 and not message.revoked and not message.allowed
            and message.phase=='holding' and binding(message)==binding(self.grant.incumbent)
            and message.trajectory_id==self.grant.incumbent.trajectory_id
            and task_binding(message)==task_binding(self.grant.incumbent))
        if accepted and not stopped_reentry and (message.revoked or not message.allowed or message.phase!='tracking' or
                (self.applied is not None and task_binding(message)!=task_binding(self.applied))):
            self._retire_pending()
            if self.applied is not None and task_binding(message)!=task_binding(self.applied):
                self.applied=None;self.commit_sequence=0
                self.failed_write_binding=None
                self.stationary.clear()
        return accepted

    def _retire_pending(self,*,revoked=True):
        if self.grant is not None and self.grant.handoff_id not in self.finished:
            self.finished.append(self.grant.handoff_id)
        self.prepared.clear();self.candidate=None
        self.pending_revoked=self.pending_revoked or revoked
        # Retain the one tombstone: ACK may describe a write made before a
        # revocation. It updates history, never revives motion authorization.

    def on_grant(self,message,now_ns):
        try:
            if not self.enabled or message.schema_version!=2 or not message.handoff_id:return False
            if self.grant is not None and message.handoff_id==self.grant.handoff_id:
                if message.revoked:
                    self._retire_pending();return True
                return message==self.grant and self.candidate is not None
            if message.handoff_id in self.finished or message.sequence<=self.last_grant_sequence:return False
            if self.grant is not None and self.candidate is not None:return False
            a,b=message.incumbent,message.candidate
            if (message.revoked or self.applied is None or message.expected_commit_sequence!=self.commit_sequence
                    or self.commit_sequence<=0 or task_binding(a)!=task_binding(b)
                    or task_binding(a)!=task_binding(self.applied)
                    or binding(a)!=binding(self.applied) or a.trajectory_id!=self.applied.trajectory_id
                    or binding(a)==binding(b) and a.trajectory_id==b.trajectory_id
                    or a.revoked or not b.allowed or b.revoked
                    or b.geometry_committed or b.phase!='tracking'
                    or not fresh(message.source_stamp,now_ns,350_000_000)
                    or not now_ns<stamp_ns(message.transition_deadline)<=stamp_ns(message.source_stamp)+250_000_000
                    or not stamp_ns(message.transition_deadline)<=stamp_ns(message.valid_until)):
                return False
            if message.transition_mode==0:
                if (not a.allowed or a.phase!='tracking' or not
                    now_ns<stamp_ns(message.retain_incumbent_until)<=stamp_ns(a.valid_until)):return False
            elif message.transition_mode==1:
                if (a.allowed or a.phase!='holding' or
                    stamp_ns(message.retain_incumbent_until)!=stamp_ns(message.source_stamp)
                    or not self._stationary_matches(message,now_ns)):return False
            else:return False
            # The incumbent must be an actual signed ordinary lease, not a
            # plausible wrapper assembled by a downstream candidate producer.
            if not any(p==a for p in self.core.permits):return False
            self.grant=deepcopy(message);self.last_grant_sequence=message.sequence
            self.pending_revoked=False
            c=CandidateSafety(self.core.session,self.core.mode,max_speed=self.core.max_speed,
                max_yaw=self.core.max_yaw,braking_model_sha256=self.core.braking_model_sha256)
            c.raw=self.core.raw;c.proofs=self.core.proofs;c.proof_history=self.core.proof_history
            c.invalidated_through=self.core.invalidated_through;c.motion_proofs=self.core.motion_proofs
            if not c.on_permit(b):return False
            self.candidate=c;self.prepared.clear()
            return True
        except (ValueError,TypeError,AttributeError,OverflowError):return False

    def on_ack(self,message,now_ns):
        try:
            if (not self.enabled or message.schema_version!=1 or message.sequence<=self.last_ack_sequence
                    or not fresh(message.applied_at,now_ns,350_000_000)):
                return False
            g=self.grant
            if (message.applied and self.applied is not None
                    and message.commit_sequence==self.commit_sequence
                    and version(message.candidate_version)==version(self.applied.version)
                    and message.candidate_trajectory_id==self.applied.trajectory_id
                    and message.permit_sequence==self.applied.sequence
                    and message.execution_id==self.applied.execution_id
                    and message.sdk_session==self.applied.sdk_session
                    and message.control_epoch==self.applied.control_epoch
                    and message.sdk_arm_generation==self.applied.sdk_arm_generation
                    and message.transport_mode==self.core.mode):
                # Async vendor callback/retransmit reports the same software
                # write fact. It must not commit again, clear active permission,
                # or renew any lease/evidence.
                self.last_ack_sequence=message.sequence
                return True
            if not message.handoff_id:
                p=next((p for p in self.core.permits if p.sequence==message.permit_sequence),None)
                if (self.commit_sequence!=0 or p is None or not message.applied
                        or message.previous_commit_sequence!=0 or message.commit_sequence!=1
                        or not p.allowed or p.revoked or not p.geometry_committed
                        or not stamp_ns(p.source_stamp)<=stamp_ns(message.applied_at)<=stamp_ns(p.valid_until)):
                    return False
            else:
                if g is None or message.handoff_id!=g.handoff_id or message.grant_sequence!=g.sequence:return False
                p=g.candidate
                if (message.previous_commit_sequence!=g.expected_commit_sequence
                        or message.incumbent_trajectory_id!=g.incumbent.trajectory_id
                        or version(message.incumbent_version)!=version(g.incumbent.version)):
                    return False
                if not message.applied:
                    if message.commit_sequence!=self.commit_sequence or not self._ack_binding(message,p):return False
                    self.last_ack_sequence=message.sequence;self._retire_pending();return True
                if stamp_ns(message.applied_at)>stamp_ns(g.transition_deadline):return False
                if message.commit_sequence!=g.expected_commit_sequence+1:return False
            if (not self._ack_binding(message,p) or message.measured_pose.header.frame_id!='d1max_loc_odom'
                    or stamp_ns(message.measured_pose.header.stamp)!=stamp_ns(message.body_source_stamp)):
                return False
            self.last_ack_sequence=message.sequence;self.commit_sequence=message.commit_sequence
            self.applied=deepcopy(p)
            self.applied.geometry_committed=True
            # Promote only the original conditional lease. Revoke/HOLD which
            # overtook the ACK remains a fence; a historical fact is not a new
            # lease, renewed stamp, or physical confirmation.
            if not message.write_submitted:self.failed_write_binding=task_binding(p)
            stopped_reentry=(bool(message.handoff_id) and g is not None and g.transition_mode==1
                and self.core.permits and not self.core.permits[-1].revoked
                and self.core.permits[-1].phase=='holding'
                and binding(self.core.permits[-1])==binding(g.incumbent)
                and self.core.permits[-1].trajectory_id==g.incumbent.trajectory_id)
            revoked=bool(not message.write_submitted or (message.handoff_id and self.pending_revoked)
                or self.core.permits and (self.core.permits[-1].revoked or
                    not self.core.permits[-1].allowed and not stopped_reentry))
            if not revoked:
                self.core.permits.clear();self.core.permits.append(deepcopy(self.applied))
                self.core.last_original=None;self.ordinary.recent=None
            else:
                self.core.permits.clear();self.core.last_original=None;self.ordinary.recent=None
            self._retire_pending(revoked=False)
            return True
        except (ValueError,TypeError,AttributeError,OverflowError):return False

    def _ack_binding(self,message,p):
        return (version(message.candidate_version)==version(p.version)
            and message.candidate_trajectory_id==p.trajectory_id and message.permit_sequence==p.sequence
            and message.execution_id==p.execution_id and message.sdk_session==p.sdk_session
            and message.control_epoch==p.control_epoch and message.sdk_arm_generation==p.sdk_arm_generation
            and message.transport_mode==self.core.mode)

    def _entry_matches(self,m,proof,now_ns):
        a=m.entry_admission;d=m.demand
        g=self.grant
        return (g is not None and m.schema_version==1 and m.handoff_id==g.handoff_id and m.grant_sequence==g.sequence
            and m.expected_commit_sequence==g.expected_commit_sequence
            and binding(d)==binding(g.candidate) and d.trajectory_id==g.candidate.trajectory_id
            and d.permit_sequence==g.candidate.sequence
            and fresh(m.entry_source_stamp,now_ns,100_000_000)
            and stamp_ns(m.entry_source_stamp)==stamp_ns(d.body_source_stamp)==stamp_ns(m.measured_pose.header.stamp)
            and m.measured_pose.header.frame_id=='d1max_loc_odom'
            and a.accepted and a.sequence>0 and version(a.version)==version(d.version)
            and a.trajectory_id==d.trajectory_id and a.validation_sequence==d.validation_sequence
            and stamp_ns(a.body_source_stamp)==stamp_ns(m.entry_source_stamp)
            and fresh(a.checked_at,now_ns,100_000_000) and now_ns<stamp_ns(a.valid_until)
            and distance(m.measured_pose.pose.position,m.curve_entry_pose.position)<=.0125
            and distance(m.measured_twist.linear,m.curve_entry_twist.linear)<=.05
            and math.isfinite(m.curve_time) and m.curve_time>=0
            and proof.handoff_id==m.handoff_id and proof.entry_admission_sequence==a.sequence
            and proof.entry_curve_pose==m.curve_entry_pose and proof.entry_curve_twist==m.curve_entry_twist
            and proof.entry_curve_time==m.curve_time)

    def _attempt(self,message,now_ns):
        if (self.candidate is None or self.grant is None
                or now_ns>=stamp_ns(self.grant.transition_deadline)):
            return Decision(False,'prepared_grant_missing_or_expired')
        try:
            proof=self.core.motion_proofs.get(self.core.motion_key(message.demand))
            if proof is None:return Decision(False,'waiting_command_sweep')
            if not self._entry_matches(message,proof,now_ns):return Decision(False,'prepared_entry_binding_mismatch')
            result=self.candidate.admit(message.demand,now_ns)
            if not result.allowed and result.reason=='replayed_motion_demand':
                result=self.candidate.admit_evidence_upgrade(message.demand,now_ns)
            if not result.allowed:return result
            out=deepcopy(message);out.demand=result.output
            return Decision(True,'conditional_prepared_motion_admitted_not_applied',out)
        except (ValueError,TypeError,AttributeError,OverflowError):
            return Decision(False,'malformed_prepared_motion')

    def prepared_demand(self,message,now_ns):
        result=self._attempt(message,now_ns)
        if result.reason in ('waiting_command_sweep','waiting_native_curve_proof'):
            key=self.core.motion_key(message.demand);self.prepared[key]=deepcopy(message)
            self.prepared.move_to_end(key)
            while len(self.prepared)>8:self.prepared.popitem(last=False)
        elif result.allowed:
            for k,m in tuple(self.prepared.items()):
                if m.demand.sequence<=message.demand.sequence:del self.prepared[k]
        return result

    def motion_proof(self,message,now_ns):
        if not getattr(message,'handoff_id',''):
            return False,self.ordinary.proof(message,now_ns)
        if not self.core.on_motion_validation(message):return True,Decision(False,'old_prepared_proof')
        key=self.core.motion_key(message,proof=True);m=self.prepared.get(key)
        if m is None:return True,Decision(False,'prepared_proof_without_original_demand')
        return True,self.prepared_demand(m,now_ns)

    def validation(self,message,now_ns):
        ordinary=self.ordinary.validation(message,now_ns)
        prepared=[]
        for key,m in reversed(tuple(self.prepared.items())):
            if now_ns>=stamp_ns(m.demand.valid_until) or not fresh(m.demand.source_stamp,now_ns,100_000_000):
                del self.prepared[key];continue
            if key in self.core.motion_proofs:
                result=self.prepared_demand(m,now_ns)
                if result.allowed:prepared.append(result);break
        return ordinary,tuple(prepared)
