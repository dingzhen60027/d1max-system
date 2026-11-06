"""Bind SCAN's native decisions to the exact command and owner permission.

SCAN owns collision and perception-source admission. This module joins its
original decisions to the command identity, sequence and unchanged deadlines;
it neither observes a second ray stream nor recomputes free-space evidence.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math

from .atomic_projection_state import stamp_ns

VERSION_FIELDS = ('schema_version','session_id','task_id','route_id','route_hash',
    'map_version_id','localization_epoch','localization_seed_id','reference_generation',
    'segment_id','anchor_id','anchor_revision','context_sequence','map_geometry_revision')


def version(message):
    return tuple(getattr(message,k) for k in VERSION_FIELDS)


def binding(message):
    return (version(message.version),message.execution_id,message.control_epoch,
            message.sdk_session,message.sdk_arm_generation)


def fresh(stamp, now_ns, maximum_ns):
    value=stamp_ns(stamp)
    return value > 0 and 0 <= now_ns-value <= maximum_ns


@dataclass(frozen=True)
class Decision:
    allowed: bool
    reason: str
    output: object = None


class AdmissionDiagnostics:
    """Fixed-capacity observations; never authority or reconstructed evidence."""
    REASONS=('native_scan_execution_admitted','zero_stop_request_not_free_space_proof',
        'foreign_execution_context','motion_demand_expired','no_task_owner_permission',
        'task_owner_revoked','execution_version_not_committed','permission_expired_or_curve_mismatch',
        'replayed_motion_demand','single_floor_motion_limits_exceeded','waiting_command_sweep',
        'waiting_native_curve_proof','command_sweep_rejected','command_sweep_binding_mismatch',
        'command_sweep_velocity_mismatch','command_sweep_evidence_expired',
        'native_swept_collision_proof_missing_or_revoked','native_evidence_expired_or_support_missing',
        'old_or_foreign_command_proof','command_proof_without_pending_demand',
        'evidence_upgrade_not_current_immutable_demand','evidence_upgrade_requires_new_positive_sweep',
        'malformed_execution_evidence','malformed_execution_evidence_upgrade','other')

    def __init__(self):
        self.counts=dict.fromkeys(self.REASONS,0)
        self.last_nonzero_rejection=None

    def record(self,result,message=None):
        reason=result.reason.split(':',1)[0]
        bucket=reason if reason in self.counts else 'other'
        self.counts[bucket]=min(self.counts[bucket]+1,(1<<64)-1)
        if result.allowed or message is None or not hasattr(message,'velocity'):return
        values=[getattr(v,axis) for v in (message.velocity.linear,message.velocity.angular) for axis in 'xyz']
        if not any(x!=0. for x in values):return
        # One original typed input is retained, not a per-task dictionary or a
        # command queue. Text clipping applies ONLY to diagnostic presentation.
        def display(value):return value[:256] if isinstance(value,str) else value
        source=getattr(message,'source_stamp',getattr(message,'demand_source_stamp',None))
        body=getattr(message,'body_source_stamp',None)
        if hasattr(message,'demand_body_source_stamp'):body=message.demand_body_source_stamp
        until=getattr(message,'demand_valid_until',getattr(message,'valid_until',None))
        self.last_nonzero_rejection=dict(reason=result.reason[:256],
            input_kind='motion_validation' if hasattr(message,'demand_sequence') else 'motion_demand',
            version={name:display(getattr(message.version,name)) for name in VERSION_FIELDS},
            execution_id=display(message.execution_id),control_epoch=message.control_epoch,
            sdk_session=display(message.sdk_session),sdk_arm_generation=message.sdk_arm_generation,
            trajectory_id=message.trajectory_id,
            demand_sequence=getattr(message,'demand_sequence',message.sequence),
            permit_sequence=message.permit_sequence,
            source_stamp_ns=stamp_ns(source) if source is not None else 0,
            body_source_stamp_ns=stamp_ns(body) if body is not None else 0,
            valid_until_ns=stamp_ns(until) if until is not None else 0)

    def snapshot(self):
        return dict(reason_counts=self.counts.copy(),last_nonzero_rejection=deepcopy(self.last_nonzero_rejection))


class ExecutionSafety:
    def __init__(self, session_id, transport_mode, *, max_speed=.30, max_yaw=.50,
                 braking_model_sha256='',sensor_source_age_s=.5,stationary_policy=None,
                 kinematic_profile='forward_yaw',command_pipeline_bound_s=.1):
        if not session_id or transport_mode not in ('live','isolated_mock'):
            raise ValueError('explicit_execution_safety_context_required')
        if not 0 < max_speed <= .30 or not 0 < max_yaw <= .50:
            raise ValueError('invalid_first_acceptance_motion_limits')
        self.session,self.mode,self.max_speed,self.max_yaw=session_id,transport_mode,max_speed,max_yaw
        if kinematic_profile not in ('forward_yaw','holonomic_xyyaw'):
            raise ValueError('unknown_kinematic_profile')
        self.kinematic_profile=kinematic_profile
        if len(braking_model_sha256)!=64 or any(c not in '0123456789abcdef' for c in braking_model_sha256):
            raise ValueError('bound_braking_model_required')
        self.braking_model_sha256=braking_model_sha256
        if (type(sensor_source_age_s) not in (int,float) or not math.isfinite(sensor_source_age_s)
                or not .001<=sensor_source_age_s<=.6):
            raise ValueError('invalid_bound_sensor_source_age')
        # Keep compatibility with the bound timing record and candidate copy.
        # SCAN evaluates this source-age bound before publishing its decision;
        # downstream admission uses that decision's original validity only.
        self.sensor_max_age_ns=int(sensor_source_age_s*1e9)
        if (type(command_pipeline_bound_s) not in (int,float)
                or not math.isfinite(command_pipeline_bound_s)
                or not .1<=command_pipeline_bound_s<=.5):
            raise ValueError('invalid_bound_command_pipeline')
        # One hashed acquisition-to-writer budget, not another callback TTL.
        # Native safety still caps its proof by the latest actual body/rays.
        self.command_pipeline_bound_ns=round(command_pipeline_bound_s*1e9)
        if stationary_policy is None:
            self.stationary_policy=dict(linear_threshold_mps=.03,angular_threshold_radps=.05,
                reentry_duration_s=.6,minimum_new_samples=3)
        else:
            from .execution_timing import validate_stationary
            self.stationary_policy=validate_stationary({'stationary_evidence':stationary_policy})
        self.motion_proofs={}
        self.permits=deque(maxlen=8)
        self.proofs={}
        self.proof_history={}
        self.invalidated_through={}
        self.last_demand=0
        self.last_binding=None
        self.last_original=None
        self.last_admitted_proof=0
        self.last_permit_sequence=0
        self.reason='waiting_execution_evidence'

    def _context(self, message):
        return (message.version.schema_version==3 and message.version.session_id==self.session
                and message.transport_mode==self.mode)

    def on_permit(self, message):
        if not self._context(message) or message.sequence <= self.last_permit_sequence:
            return False
        self.last_permit_sequence=message.sequence
        # Revocation invalidates all previously received authorizations, even
        # if an old lease has not expired. No canceled generation is resurrected.
        if message.revoked or not message.allowed:
            self.permits.clear()
            self.last_original=None
        elif self.permits and (binding(message)!=binding(self.permits[-1])
                or message.trajectory_id!=self.permits[-1].trajectory_id
                or message.phase!=self.permits[-1].phase):
            self.permits.clear()
            self.last_original=None
        self.permits.append(deepcopy(message))
        return True

    def on_validation(self, message):
        if not self._context(message) or message.sequence<=0:
            return False
        key=(version(message.version),message.trajectory_id)
        old=self.proofs.get(key)
        if old is not None and message.sequence<=old.sequence:
            # Two evidence topics can overtake each other. An intervening
            # negative revision remains a fence even when its delivery follows
            # the newer positive snapshot; it must not replace that snapshot.
            if (not message.valid
                    and message.sequence>self.invalidated_through.get(key,0)):
                self.invalidated_through[key]=message.sequence
                return True
            return False
        self.proofs[key]=deepcopy(message)
        self.proof_history.setdefault(key,deque(maxlen=8)).append(deepcopy(message))
        if not message.valid:
            self.invalidated_through[key]=message.sequence
            if (self.last_original is not None and
                    (version(self.last_original.version),self.last_original.trajectory_id)==key):
                self.last_original=None
        if len(self.proofs)>8:
            retired=next(iter(self.proofs))
            del self.proofs[retired]
            self.proof_history.pop(retired,None)
            self.invalidated_through.pop(retired,None)
        return True

    @staticmethod
    def motion_key(message, *, proof=False):
        return (binding(message),message.trajectory_id,
                message.demand_sequence if proof else message.sequence)

    def on_motion_validation(self,message):
        if not self._context(message):return False
        key=self.motion_key(message,proof=True)
        old=self.motion_proofs.get(key)
        if old is not None and not old.valid:return False
        if old is not None and message.sequence<=old.sequence and message.valid:return False
        # A real negative check is an irreversible fence for this command ID,
        # even if cross-topic queueing delivers it after a higher positive.
        if message.sequence<=0:return False
        self.motion_proofs[key]=deepcopy(message)
        if not message.valid and self.last_original is not None and self.motion_key(self.last_original)==key:
            self.last_original=None
        if len(self.motion_proofs)>32:self.motion_proofs.pop(next(iter(self.motion_proofs)))
        return True

    def _motion_proof(self,demand,now_ns):
        proof=self.motion_proofs.get(self.motion_key(demand))
        if proof is None:return Decision(False,'waiting_command_sweep')
        if not proof.valid:return Decision(False,'command_sweep_rejected:'+proof.reason)
        pairs=((proof.demand_source_stamp,demand.source_stamp),
               (proof.demand_body_source_stamp,demand.body_source_stamp),
               (proof.demand_valid_until,demand.valid_until))
        if (proof.braking_model_sha256!=self.braking_model_sha256
                or proof.permit_sequence!=demand.permit_sequence
                or proof.trajectory_validation_sequence<demand.validation_sequence
                or any(stamp_ns(a)!=stamp_ns(b) for a,b in pairs)
                or proof.frame_id!='d1max_loc_odom'
                or proof.map_snapshot_revision<=0):
            return Decision(False,'command_sweep_binding_mismatch')
        for component in ('linear','angular'):
            if any(getattr(getattr(proof.velocity,component),axis)!=getattr(getattr(demand.velocity,component),axis)
                   for axis in 'xyz'):
                return Decision(False,'command_sweep_velocity_mismatch')
        if (not fresh(proof.check_end,now_ns,250_000_000)
                or not stamp_ns(proof.check_begin)<=stamp_ns(proof.check_end)
                or not now_ns<stamp_ns(proof.valid_until)<=stamp_ns(demand.valid_until)):
            return Decision(False,'command_sweep_evidence_expired')
        return Decision(True,'native_command_sweep_verified',proof)

    @staticmethod
    def _proof_domain(proof,permit):
        """Validate SCAN's declared check domain, not a second progress stream.

        The exact MotionValidation already names the native curve proof and
        binds the original command/body sources. Native selects and verifies
        its measured entry before it issues that verdict. Waiting for an
        independently delivered tracker progress message here would add a
        second, shorter clock and could reject that unchanged native decision.
        """
        values=(proof.checked_from_time,proof.checked_to_time,proof.curve_duration,
                proof.valid_start_time,proof.reverse_margin_m)
        if (not all(math.isfinite(x) for x in values) or proof.curve_duration<=0
                or not 0<=proof.checked_from_time<=proof.valid_start_time<=proof.checked_to_time
                or abs(proof.checked_to_time-proof.curve_duration)>1e-6):return False
        if proof.whole_curve:
            return not proof.remaining_curve and proof.checked_from_time<=1e-9
        return (proof.remaining_curve and proof.reverse_margin_m>=.15
                and permit.geometry_committed)

    def stop_for_rejection(self, demand, now_ns, reason):
        """Request zero with the original identity/lease; never extend evidence.

        A malformed, stale, revoked or foreign command cannot obtain a new
        authorization here. The SDK's independent watchdog handles that case.
        """
        try:
            out=deepcopy(demand)
            for component in (out.velocity.linear,out.velocity.angular):
                component.x=component.y=component.z=0.
            out.hold=True
            out.safety_checked=False
            out.reason='safety_hold:'+reason
            return self.admit(out,now_ns)
        except (ValueError,TypeError,AttributeError,OverflowError):
            return Decision(False,'malformed_stop_request')

    def admit(self, demand, now_ns):
        try:
            return self._admit(demand,now_ns)
        except (ValueError,TypeError,AttributeError,OverflowError):
            return Decision(False,'malformed_execution_evidence')

    def admit_evidence_upgrade(self,demand,now_ns):
        """Bind a newer real sweep to one unchanged, still-live command.

        This is not command regeneration: source, body source, velocity,
        identity and original lease must be byte-for-byte equal to the last
        admitted input. Revocation, a negative sweep or an intervening command
        destroys this opportunity. No timer calls this method.
        """
        try:
            if (self.last_original is None or demand!=self.last_original
                    or (binding(demand),demand.trajectory_id)!=self.last_binding
                    or demand.sequence!=self.last_demand):
                return Decision(False,'evidence_upgrade_not_current_immutable_demand')
            proof=self.motion_proofs.get(self.motion_key(demand))
            if proof is None or not proof.valid or proof.sequence<=self.last_admitted_proof:
                return Decision(False,'evidence_upgrade_requires_new_positive_sweep')
            return self._admit(demand,now_ns,evidence_upgrade=True)
        except (ValueError,TypeError,AttributeError,OverflowError):
            return Decision(False,'malformed_execution_evidence_upgrade')

    def _admit(self,demand,now_ns,*,evidence_upgrade=False,require_zero_sweep=False):
        if not self._context(demand):
            return Decision(False,'foreign_execution_context')
        if (not fresh(demand.source_stamp,now_ns,self.command_pipeline_bound_ns)
                or not fresh(demand.body_source_stamp,now_ns,self.command_pipeline_bound_ns)
                or not now_ns < stamp_ns(demand.valid_until) <= stamp_ns(demand.source_stamp)+250_000_000):
            return Decision(False,'motion_demand_expired')
        if not self.permits:
            return Decision(False,'no_task_owner_permission')
        latest=self.permits[-1]
        if latest.revoked or not latest.allowed:
            return Decision(False,'task_owner_revoked')
        if binding(latest)!=binding(demand) or latest.trajectory_id!=demand.trajectory_id:
            return Decision(False,'execution_version_not_committed')
        p=next((p for p in reversed(self.permits) if p.sequence==demand.permit_sequence),None)
        if (p is None or not p.allowed or p.revoked or binding(p)!=binding(demand)
                or p.trajectory_id!=demand.trajectory_id or p.phase!=latest.phase
                or not fresh(p.source_stamp,now_ns,350_000_000)
                or not now_ns < stamp_ns(p.valid_until) <= stamp_ns(p.source_stamp)+350_000_000):
            return Decision(False,'permission_expired_or_curve_mismatch')
        demand_binding=(binding(demand),demand.trajectory_id)
        if demand_binding==self.last_binding and demand.sequence<=self.last_demand and not evidence_upgrade:
            return Decision(False,'replayed_motion_demand')
        values=[getattr(v,axis) for v in (demand.velocity.linear,demand.velocity.angular) for axis in 'xyz']
        if (not all(math.isfinite(v) for v in values)
                or math.hypot(values[0],values[1])>self.max_speed
                or abs(values[5])>self.max_yaw or any(abs(values[i])>1e-9 for i in (2,3,4))
                or self.kinematic_profile=='forward_yaw' and (values[0]<0 or abs(values[1])>1e-9)):
            return Decision(False,'single_floor_motion_limits_exceeded')
        zero=all(abs(x)<1e-12 for x in values)
        if zero and not require_zero_sweep:
            out=deepcopy(demand)
            out.hold=True; out.safety_checked=False
            self.last_binding,self.last_demand=demand_binding,demand.sequence
            self.last_original=None
            self.last_admitted_proof=0
            return Decision(True,'zero_stop_request_not_free_space_proof',out)
        # The demand's revision is the real evidence at controller generation,
        # not a frozen expiry for the later native sweep. A fresh sweep names
        # the exact newer same-curve proof it used. No deadline is restamped.
        key=(version(demand.version),demand.trajectory_id)
        if demand.validation_sequence<=self.invalidated_through.get(key,0):
            return Decision(False,'native_swept_collision_proof_missing_or_revoked')
        command=self._motion_proof(demand,now_ns)
        # A callback-time expired floor is not a collision verdict. Native can
        # bind newer evidence for the exact original demand before its original
        # deadline. Wait for that actual sweep; do not prematurely consume the
        # sequence by publishing zero based on the obsolete floor snapshot.
        # Known negative revisions above still reject immediately, and this
        # waiting result contains NO output or permission to move.
        if not command.allowed:return command
        actual_sequence=command.output.trajectory_validation_sequence
        proof=next((x for x in reversed(self.proof_history.get(key,()))
                    if x.sequence==actual_sequence),None)
        # Native publishes trajectory and command proofs on different topics.
        # A verified sweep can arrive before its named geometry proof. Buffer
        # this bounded delivery gap; it is not free-space permission or a new
        # collision. Real negative evidence below still fences the old demand.
        if (proof is None and command.allowed
                and actual_sequence>getattr(self.proofs.get(key),'sequence',0)
                and demand.validation_sequence>self.invalidated_through.get(key,0)):
            return Decision(False,'waiting_native_curve_proof')
        # A geometry decision alone never authorizes a write: SCAN's exact
        # command decision must already match this unchanged demand. Its ray
        # sources and map admission have been evaluated inside SCAN.
        if (proof is None or not proof.valid or not self._proof_domain(proof,p)
                or demand.validation_sequence<=self.invalidated_through.get(key,0)
                # The owner grants authority for this immutable curve. Its
                # proof sequence is a floor, not a lock to an expired sample.
                # The command still names one exact fresh proof, and newer
                # negative evidence fences every older positive result above.
                or proof.sequence<demand.validation_sequence or proof.sequence<p.validation_sequence
                or proof.frame_id!='d1max_loc_odom'
                or proof.collision_policy not in ('observed_free','official_inflated_geometry')):
            return Decision(False,'native_swept_collision_proof_missing_or_revoked')
        if (not fresh(proof.check_end,now_ns,250_000_000)
                or not now_ns < stamp_ns(proof.valid_until)
                or stamp_ns(proof.check_begin)>stamp_ns(proof.check_end)
                or not proof.support_reference_id or len(proof.support_hash)!=64):
            return Decision(False,'native_evidence_expired_or_support_missing')
        out=deepcopy(demand)
        out.safety_checked=True
        out.motion_validation_sequence=command.output.sequence
        out.braking_model_sha256=self.braking_model_sha256
        oldest=min((proof.front_ray_source_stamp,proof.rear_ray_source_stamp),key=stamp_ns)
        out.safety_source_stamp=deepcopy(oldest)
        self.last_binding,self.last_demand=demand_binding,demand.sequence
        self.last_original=deepcopy(demand)
        self.last_admitted_proof=command.output.sequence
        return Decision(True,'native_scan_execution_admitted',out)
