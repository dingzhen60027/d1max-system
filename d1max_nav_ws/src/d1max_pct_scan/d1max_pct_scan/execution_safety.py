"""Typed command admission, independent of the planner and SDK transport.

Collision decisions come from the real native validator. This module does not
replace GridMap or manufacture free space: it binds that decision to the exact
task/curve and the independently received dual-LiDAR source evidence.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math

from .atomic_projection_state import stamp_ns
from .ray_projection import FIELDS, RAY_DTYPE
from .perception_contract import MAX_ACQUISITION_POINTS
import numpy as np

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


class ExecutionSafety:
    def __init__(self, session_id, transport_mode, *, max_speed=.30, max_yaw=.50,
                 braking_model_sha256='',sensor_source_age_s=.5,stationary_policy=None):
        if not session_id or transport_mode not in ('live','isolated_mock'):
            raise ValueError('explicit_execution_safety_context_required')
        if not 0 < max_speed <= .30 or not 0 < max_yaw <= .50:
            raise ValueError('invalid_first_acceptance_motion_limits')
        self.session,self.mode,self.max_speed,self.max_yaw=session_id,transport_mode,max_speed,max_yaw
        if len(braking_model_sha256)!=64 or any(c not in '0123456789abcdef' for c in braking_model_sha256):
            raise ValueError('bound_braking_model_required')
        self.braking_model_sha256=braking_model_sha256
        if (type(sensor_source_age_s) not in (int,float) or not math.isfinite(sensor_source_age_s)
                or not .001<=sensor_source_age_s<=.6):
            raise ValueError('invalid_bound_sensor_source_age')
        self.sensor_max_age_ns=int(sensor_source_age_s*1e9)
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
        self.raw=[deque(maxlen=32),deque(maxlen=32)]
        self.progress=None
        self.last_demand=0
        self.last_binding=None
        self.last_original=None
        self.last_admitted_proof=0
        self.last_permit_sequence=0
        self.last_raw_sequence=0
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

    def on_progress(self, message):
        if message.session_id!=self.session or message.schema_version!=2:return False
        if self.progress is not None and stamp_ns(message.header.stamp)<=stamp_ns(self.progress.header.stamp):return False
        self.progress=deepcopy(message)
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
                or not fresh(proof.body_source_stamp,now_ns,100_000_000)
                or not fresh(proof.front_ray_source_stamp,now_ns,self.sensor_max_age_ns)
                or not fresh(proof.rear_ray_source_stamp,now_ns,self.sensor_max_age_ns)
                or not stamp_ns(proof.check_begin)<=stamp_ns(proof.check_end)
                or not now_ns<stamp_ns(proof.valid_until)<=stamp_ns(demand.valid_until)
                or not self._source_seen(0,proof,proof.front_ray_source_stamp)
                or not self._source_seen(1,proof,proof.rear_ray_source_stamp)):
            return Decision(False,'command_sweep_evidence_expired_or_unbound')
        return Decision(True,'native_command_sweep_verified',proof)

    def _proof_domain(self,proof,permit,now_ns):
        values=(proof.checked_from_time,proof.checked_to_time,proof.curve_duration,
                proof.valid_start_time,proof.reverse_margin_m)
        if (not all(math.isfinite(x) for x in values) or proof.curve_duration<=0
                or not 0<=proof.checked_from_time<=proof.valid_start_time<=proof.checked_to_time
                or abs(proof.checked_to_time-proof.curve_duration)>1e-6):return False
        if proof.whole_curve:
            return not proof.remaining_curve and proof.checked_from_time<=1e-9
        p=self.progress;v=proof.version
        if (not proof.remaining_curve or proof.reverse_margin_m<.15 or not permit.geometry_committed
                or p is None or not p.valid or p.header.frame_id!='d1max_loc_odom'
                or not fresh(p.header.stamp,now_ns,100_000_000)
                or p.trajectory_id!=proof.trajectory_id
                or not math.isfinite(p.curve_time)
                or not proof.checked_from_time<=p.curve_time<=proof.checked_to_time):return False
        names=('session_id','task_id','route_id','route_hash','map_version_id','localization_epoch',
               'localization_seed_id','segment_id','anchor_id','anchor_revision','context_sequence')
        return p.generation==v.reference_generation and all(getattr(p,n)==getattr(v,n) for n in names)

    def on_rays(self, message, now_ns):
        cloud=message.rays
        fields=tuple((f.name,f.offset,f.datatype,f.count) for f in cloud.fields)
        if (message.session_id!=self.session or cloud.header.frame_id!='d1max_loc_odom'
                or message.projection_sequence<=self.last_raw_sequence
                or fields!=FIELDS or cloud.point_step!=64 or cloud.height!=1
                or cloud.width<1 or cloud.width>MAX_ACQUISITION_POINTS or cloud.is_bigendian
                or cloud.row_step!=cloud.width*64 or len(cloud.data)!=cloud.row_step
                or not fresh(message.acquisition_end,now_ns,self.sensor_max_age_ns)):
            return False
        points=np.frombuffer(cloud.data,dtype=RAY_DTYPE,count=cloud.width)
        sensor=int(points['sensor_id'][0])
        if sensor not in (0,1) or np.any(points['sensor_id']!=sensor):
            return False
        if any(not np.isfinite(points[k]).all() for k in
               ('x','y','z','origin_x','origin_y','origin_z','timestamp','source_timestamp','raw_timestamp')):
            return False
        first,end=stamp_ns(cloud.header.stamp),stamp_ns(message.acquisition_end)
        if not first<=end<=first+150_000_000:
            return False
        self.raw[sensor].append((message.epoch,message.seed_id,message.context_sequence,first,end))
        self.last_raw_sequence=message.projection_sequence
        return True

    def _source_seen(self, sensor, proof, raw_stamp):
        # GridMap's integrated watermark is scan BEGIN. An END timestamp is
        # not evidence that the corresponding rays have actually integrated.
        ns=stamp_ns(raw_stamp); v=proof.version
        return any(epoch==v.localization_epoch and seed==v.localization_seed_id
                   and seq==v.context_sequence and ns==begin
                   for epoch,seed,seq,begin,end in self.raw[sensor])

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
        if (not fresh(demand.source_stamp,now_ns,100_000_000)
                or not fresh(demand.body_source_stamp,now_ns,100_000_000)
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
        if (not all(math.isfinite(v) for v in values) or not 0<=values[0]<=self.max_speed
                or abs(values[5])>self.max_yaw or any(abs(values[i])>1e-9 for i in (1,2,3,4))):
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
        if (proof is None or not proof.valid or not self._proof_domain(proof,p,now_ns)
                or demand.validation_sequence<=self.invalidated_through.get(key,0)
                # The owner grants authority for this immutable curve. Its
                # proof sequence is a floor, not a lock to an expired sample.
                # The command still names one exact fresh proof, and newer
                # negative evidence fences every older positive result above.
                or proof.sequence<demand.validation_sequence or proof.sequence<p.validation_sequence
                or proof.frame_id!='d1max_loc_odom' or proof.collision_policy!='observed_free'):
            return Decision(False,'native_swept_collision_proof_missing_or_revoked')
        if (not fresh(proof.check_end,now_ns,250_000_000)
                or not now_ns < stamp_ns(proof.valid_until)
                or not fresh(proof.body_source_stamp,now_ns,400_000_000)
                or not fresh(proof.front_ray_source_stamp,now_ns,self.sensor_max_age_ns)
                or not fresh(proof.rear_ray_source_stamp,now_ns,self.sensor_max_age_ns)
                or stamp_ns(proof.check_begin)>stamp_ns(proof.check_end)
                or not proof.support_reference_id or len(proof.support_hash)!=64):
            return Decision(False,'native_evidence_expired_or_support_missing')
        if (not self._source_seen(0,proof,proof.front_ray_source_stamp)
                or not self._source_seen(1,proof,proof.rear_ray_source_stamp)):
            return Decision(False,'collision_proof_not_bound_to_received_raw_sources')
        if not command.allowed:return command
        out=deepcopy(demand)
        out.safety_checked=True
        out.motion_validation_sequence=command.output.sequence
        out.braking_model_sha256=self.braking_model_sha256
        oldest=min((proof.front_ray_source_stamp,proof.rear_ray_source_stamp),key=stamp_ns)
        out.safety_source_stamp=deepcopy(oldest)
        self.last_binding,self.last_demand=demand_binding,demand.sequence
        self.last_original=deepcopy(demand)
        self.last_admitted_proof=command.output.sequence
        return Decision(True,'native_raw_ray_execution_admitted',out)
