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
from .execution_timing import stationary_velocity_within
from .execution_safety import Decision, ExecutionSafety, binding, fresh, version


def task_binding(message, *, acknowledgement=False):
    v=message.candidate_version if acknowledgement else message.version
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


def retain_unresolved(records,key,value):
    """Fixed eight facts: retain the earliest unresolved and the latest seven."""
    if key in records:return
    records[key]=deepcopy(value)
    if len(records)>8:del records[next(iter(tuple(records)[1:]))]


class InitialCommitAdmission(CommandAdmission):
    def __init__(self,core,pin):
        super().__init__(core)
        self.pin=pin

    def demand(self,message,now_ns):
        result=super().demand(message,now_ns)
        if result.allowed and result.output is not None:self.pin(result.output)
        return result

    def proof(self,message,now_ns):
        result=super().proof(message,now_ns)
        if result.allowed and result.output is not None:self.pin(result.output)
        return result

    def validation(self,message,now_ns):
        results=super().validation(message,now_ns)
        for result in results:
            if result.allowed and result.output is not None:self.pin(result.output)
        return results


class ExecutionHandoffAdmission:
    def __init__(self,core,*,enabled=False):
        self.core=core
        self.ordinary=InitialCommitAdmission(core,self._pin_initial_command)
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
        self.initial_submissions=OrderedDict()
        self.prepared_submissions=OrderedDict()
        self.handoff_submissions=OrderedDict()
        self.last_applied_ack=None
        self.execution_revoked_at=None
        # A supplied core may already contain its owner lease. Adopt only
        # that signed context; no ACK or stationary callback can select it.
        self.execution_binding=task_binding(core.permits[-1]) if core.permits else None

    def _begin_execution(self,context):
        # The sole writer and BT owner restart their handoff/ACK counters at
        # each grant. These replay floors belong to that full execution, not
        # to the lifetime of this ROS node. The ordinary permit publisher's
        # session-wide sequence floor remains untouched in the core.
        self.execution_binding=context
        self.commit_sequence=0
        self.applied=self.grant=self.candidate=None
        self.prepared.clear();self.finished.clear();self.stationary.clear()
        self.initial_submissions.clear();self.prepared_submissions.clear()
        self.handoff_submissions.clear()
        self.last_applied_ack=None;self.execution_revoked_at=None
        self.last_grant_sequence=self.last_ack_sequence=0
        self.pending_revoked=False;self.failed_write_binding=None
        self.ordinary.pending.clear();self.ordinary.recent=None
        self.core.last_binding=None;self.core.last_demand=0
        self.core.last_original=None;self.core.last_admitted_proof=0

    def _pin_initial_command(self,demand):
        if self.commit_sequence or task_binding(demand)!=self.execution_binding:return
        p=next((p for p in self.core.permits if p.sequence==demand.permit_sequence
                and binding(p)==binding(demand) and p.trajectory_id==demand.trajectory_id
                and p.allowed and p.geometry_committed and not p.revoked),None)
        if p is None:return
        # This records actual admission output, never a permission heartbeat.
        # Multiple 50 Hz commands can share one 10 Hz signed permit.
        if p.sequence not in self.initial_submissions:
            retain_unresolved(self.initial_submissions,p.sequence,(p,OrderedDict()))
        commands=self.initial_submissions[p.sequence][1]
        retain_unresolved(commands,(demand.sequence,demand.motion_validation_sequence),demand)

    def _submitted_command(self,message,demand,p):
        at=stamp_ns(message.applied_at)
        return (message.demand_sequence==demand.sequence
            and message.motion_validation_sequence==demand.motion_validation_sequence
            and message.demand_source_stamp==demand.source_stamp
            and message.demand_body_source_stamp==demand.body_source_stamp
            and message.applied_velocity==demand.velocity
            and fresh(demand.source_stamp,at,self.core.command_pipeline_bound_ns)
            and fresh(demand.body_source_stamp,at,self.core.command_pipeline_bound_ns)
            and fresh(message.body_source_stamp,at,100_000_000)
            and stamp_ns(demand.source_stamp)<=at<=stamp_ns(demand.valid_until)
            and stamp_ns(p.source_stamp)<=at<=stamp_ns(p.valid_until)
            and at<=stamp_ns(message.valid_until)<=min(stamp_ns(p.valid_until),stamp_ns(demand.valid_until)))

    def on_stationary(self,message):
        # These are original SDK-published witnesses, not local-odom velocity.
        if (message.schema_version!=1 or message.sequence<=0
                or task_binding(message)!=self.execution_binding):return False
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
        # Physical acceptance is a reporting fact, not a substitute for (or
        # veto of) this original SDK stopping witness. CandidateSafety keeps
        # the same bound runtime-model SHA as the ordinary safety gate.
        policy=self.core.stationary_policy
        e=g.stationary_evidence;a=g.incumbent
        source=stamp_ns(e.source_stamp);received=stamp_ns(e.received_stamp)
        lower=stamp_ns(e.capture_lower_bound);upper=stamp_ns(e.capture_upper_bound)
        speed=e.measured_linear_mps;yaw=e.measured_angular_radps
        delay=e.mc_capture_delay_bound_sec
        anchor=min(source,received)
        # Match SDK/Owner/Tracker's original MC witness contract. Only the
        # explicitly approximate live clock may lead receipt within its
        # recorded capture bound. Freshness and the fixed lease use the
        # conservative original minimum; never restamp or borrow future age.
        if not math.isfinite(delay) or not 0<delay<=.25:return False
        capture_bound_ns=round(delay*1e9)
        latest=self.stationary[-1] if self.stationary else None
        return (latest is not None and latest.usable and task_binding(latest)==task_binding(e)
            and latest.writer_commit_sequence==e.writer_commit_sequence
            and latest.applied_trajectory_id==e.applied_trajectory_id
            and latest.zero_write_sequence==e.zero_write_sequence
            and latest.mc_clock_epoch==e.mc_clock_epoch
            and latest.mc_raw_stamp_ns>=e.mc_raw_stamp_ns
            and any(e==published for published in self.stationary)
            and e.schema_version==1 and e.usable and e.nonzero_blocked
            and version(e.version)==version(a.version) and e.applied_trajectory_id==a.trajectory_id
            and task_binding(e)==task_binding(a) and e.writer_commit_sequence==g.expected_commit_sequence
            and e.zero_write_sequence>0 and e.mc_raw_stamp_ns>0 and bool(e.mc_clock_epoch)
            and e.time_basis==('source_delta_host_anchor_approximate' if self.core.mode=='live'
                              else 'isolated_simulated_source_clock')
            and (source-received<=capture_bound_ns if self.core.mode=='live' else source<=received)
            and 0<anchor<=now_ns and now_ns-anchor<=250_000_000
            and fresh(e.received_stamp,now_ns,250_000_000)
            and now_ns<stamp_ns(e.valid_until)<=anchor+250_000_000
            and stamp_ns(g.transition_deadline)<=stamp_ns(e.valid_until)
            and stamp_ns(g.valid_until)<=stamp_ns(e.valid_until)
            and stamp_ns(g.candidate.valid_until)<=stamp_ns(e.valid_until)
            and stamp_ns(g.source_stamp)>=received
            and 0<stamp_ns(e.zero_ack_at)<lower<=anchor and upper==received
            and upper-lower<=capture_bound_ns+abs(source-received)+1_000
            and lower-stamp_ns(e.zero_ack_at)>=round(policy['reentry_duration_s']*1e9)
            and math.isfinite(e.stationary_duration_sec) and e.stationary_duration_sec>=policy['reentry_duration_s']
            and e.stationary_samples>=policy['minimum_new_samples']
            and stationary_velocity_within(e,policy))

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
        if accepted and task_binding(message)!=self.execution_binding:
            self._begin_execution(task_binding(message))
        if accepted and message.revoked:
            at=stamp_ns(message.source_stamp)
            self.execution_revoked_at=at if self.execution_revoked_at is None else min(at,self.execution_revoked_at)
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
        if self.grant is not None and revoked:
            transaction=self.handoff_submissions.get(self.grant.handoff_id)
            if transaction is not None:transaction['retired']=True
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
            if message.handoff_id in self.handoff_submissions:return False
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
            retain_unresolved(self.handoff_submissions,message.handoff_id,
                dict(grant=message,commands=OrderedDict(),retired=False))
            self.prepared_submissions=self.handoff_submissions[message.handoff_id]['commands']
            self.pending_revoked=False
            c=CandidateSafety(self.core.session,self.core.mode,max_speed=self.core.max_speed,
                max_yaw=self.core.max_yaw,braking_model_sha256=self.core.braking_model_sha256,
                sensor_source_age_s=self.core.sensor_max_age_ns/1e9,
                stationary_policy=self.core.stationary_policy,
                kinematic_profile=self.core.kinematic_profile,
                command_pipeline_bound_s=self.core.command_pipeline_bound_ns/1e9)
            c.proofs=self.core.proofs;c.proof_history=self.core.proof_history
            c.invalidated_through=self.core.invalidated_through;c.motion_proofs=self.core.motion_proofs
            if not c.on_permit(b):return False
            self.candidate=c;self.prepared.clear()
            return True
        except (ValueError,TypeError,AttributeError,OverflowError):return False

    def on_ack(self,message,now_ns):
        try:
            at=stamp_ns(message.applied_at)
            current=fresh(message.applied_at,now_ns,350_000_000)
            if (not self.enabled or message.schema_version!=1 or message.sequence<=self.last_ack_sequence
                    or task_binding(message,acknowledgement=True)!=self.execution_binding
                    or not 0<at<=now_ns
                    or message.applied and self.execution_revoked_at is not None and at>self.execution_revoked_at):
                return False
            g=self.grant
            transaction=None
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
                original=deepcopy(self.last_applied_ack);incoming=deepcopy(message)
                if original is None:return False
                if original.write_acknowledged and not incoming.write_acknowledged:return False
                original.sequence=incoming.sequence=0
                original.write_acknowledged=incoming.write_acknowledged=False
                original.reason=incoming.reason=''
                if original!=incoming:return False
                self.last_ack_sequence=message.sequence
                self.last_applied_ack=deepcopy(message)
                return True
            if not message.handoff_id:
                p=next((p for p in self.core.permits if p.sequence==message.permit_sequence),None)
                pin=self.initial_submissions.get(message.permit_sequence)
                if p is None and pin is not None:p=pin[0]
                if (self.commit_sequence!=0 or p is None or not message.applied
                        or message.previous_commit_sequence!=0 or message.commit_sequence!=1
                        or not p.allowed or p.revoked or not p.geometry_committed
                        or not stamp_ns(p.source_stamp)<=stamp_ns(message.applied_at)<=stamp_ns(p.valid_until)):
                    return False
                if not current and (pin is None or not any(self._submitted_command(message,d,p)
                        for d in pin[1].values())):return False
            else:
                transaction=self.handoff_submissions.get(message.handoff_id)
                if transaction is not None:g=transaction['grant']
                if g is None or message.handoff_id!=g.handoff_id or message.grant_sequence!=g.sequence:return False
                p=g.candidate
                if (message.previous_commit_sequence!=g.expected_commit_sequence
                        or message.incumbent_trajectory_id!=g.incumbent.trajectory_id
                        or version(message.incumbent_version)!=version(g.incumbent.version)):
                    return False
                if not message.applied:
                    # Exact no-write is a software retirement fact, not an
                    # expiring permission. Late delivery must release only
                    # its original transaction; it never renews the incumbent
                    # or candidate lease and cannot authorize any command.
                    if (self.grant is None or self.candidate is None or self.pending_revoked
                            or self.grant.handoff_id!=g.handoff_id
                            or message.write_submitted or message.write_acknowledged
                            or not stamp_ns(g.source_stamp)<=at
                            or message.previous_commit_sequence!=self.commit_sequence
                            or message.commit_sequence!=self.commit_sequence
                            or not self._ack_binding(message,p)):return False
                    self.last_ack_sequence=message.sequence
                    if transaction is not None:transaction['retired']=True
                    if self.grant is not None and self.grant.handoff_id==g.handoff_id:self._retire_pending()
                    return True
                if not max(stamp_ns(g.source_stamp),stamp_ns(p.source_stamp))<=at<=min(
                        stamp_ns(g.transition_deadline),stamp_ns(g.valid_until),stamp_ns(p.valid_until)):return False
                if (message.previous_commit_sequence!=self.commit_sequence
                        or message.commit_sequence!=g.expected_commit_sequence+1):return False
                submissions=transaction['commands'] if transaction is not None else self.prepared_submissions
                if not current and not any(self._submitted_command(message,m.demand,p)
                        and message.entry_admission_sequence==m.entry_admission.sequence
                        and message.entry_source_stamp==m.entry_source_stamp
                        and message.curve_time==m.curve_time
                        for m in submissions.values()):return False
            if (not self._ack_binding(message,p) or message.measured_pose.header.frame_id!='d1max_loc_odom'
                    or stamp_ns(message.measured_pose.header.stamp)!=stamp_ns(message.body_source_stamp)):
                return False
            self.last_ack_sequence=message.sequence;self.commit_sequence=message.commit_sequence
            self.last_applied_ack=deepcopy(message)
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
            retired_transaction=(transaction is not None and (transaction['retired']
                or self.grant is None or self.grant.handoff_id!=g.handoff_id))
            revoked=bool(not message.write_submitted or retired_transaction
                or (message.handoff_id and self.pending_revoked)
                or self.core.permits and (self.core.permits[-1].revoked or
                    not self.core.permits[-1].allowed and not stopped_reentry))
            live_original=(current and fresh(p.source_stamp,now_ns,350_000_000)
                and now_ns<min(stamp_ns(p.valid_until),stamp_ns(getattr(message,'valid_until',p.valid_until))))
            if not revoked and live_original:
                self.core.permits.clear();self.core.permits.append(deepcopy(self.applied))
                self.core.last_original=None;self.ordinary.recent=None
            elif revoked or not self.core.permits or binding(self.core.permits[-1])!=binding(self.applied):
                self.core.permits.clear();self.core.last_original=None;self.ordinary.recent=None
            if transaction is not None:transaction['retired']=True
            # A historical applied transaction overtakes any newer conditional
            # draft based on the old writer identity. Retire that draft, too.
            self._retire_pending(revoked=bool(message.handoff_id and self.grant is not None
                and self.grant.handoff_id!=g.handoff_id))
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
            and fresh(m.entry_source_stamp,now_ns,self.core.command_pipeline_bound_ns)
            and stamp_ns(m.entry_source_stamp)==stamp_ns(d.body_source_stamp)==stamp_ns(m.measured_pose.header.stamp)
            and m.measured_pose.header.frame_id=='d1max_loc_odom'
            and a.accepted and a.sequence>0 and version(a.version)==version(d.version)
            and a.trajectory_id==d.trajectory_id and a.validation_sequence==d.validation_sequence
            and stamp_ns(a.body_source_stamp)==stamp_ns(m.entry_source_stamp)
            and fresh(a.checked_at,now_ns,self.core.command_pipeline_bound_ns) and now_ns<stamp_ns(a.valid_until)
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
            retain_unresolved(self.prepared_submissions,
                (result.output.demand.sequence,result.output.demand.motion_validation_sequence),result.output)
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
            if now_ns>=stamp_ns(m.demand.valid_until) or not fresh(m.demand.source_stamp,now_ns,self.core.command_pipeline_bound_ns):
                del self.prepared[key];continue
            if key in self.core.motion_proofs:
                result=self.prepared_demand(m,now_ns)
                if result.allowed:prepared.append(result);break
        return ordinary,tuple(prepared)
