"""Bounded inbox for the sole writer's adjacent-segment mode transaction.

Mode confirmation uses original SDK callback receipt time, never a synthetic
50 Hz body source. The inbox is not an SDK client and cannot authorize motion.
"""
from copy import deepcopy
import math

from .continuous_reference import ReceiptModeConfirmation
from .continuous_reference_transport import nanoseconds


TASK_FIELDS=('session_id','task_id','route_id','route_hash','map_version_id',
             'localization_epoch','localization_seed_id')


def same_task(a,b):
    return all(getattr(a,key)==getattr(b,key) for key in TASK_FIELDS)


class SegmentTransitionInbox:
    def __init__(self, transport_mode):
        if transport_mode not in ('live','isolated_mock'):
            raise ValueError('explicit_transition_transport_required')
        self.transport_mode=transport_mode
        self.request=None
        self.state=None
        self.received=-math.inf
        self.sequence=0

    def clear(self):
        self.request=self.state=None
        self.received=-math.inf
        self.sequence=0

    def begin(self,request):
        self.request=deepcopy(request)
        self.state=None
        self.received=-math.inf
        self.sequence=0

    def observe(self,state,*,now_ns,received_monotonic):
        request=self.request
        if (request is None or state.schema_version!=1 or state.version!=request.version
                or state.request_id!=request.request_id or state.request_sequence!=request.sequence
                or state.target_segment_id!=request.target_segment_id or state.target_mode!=request.target_mode
                or state.transport_mode!=self.transport_mode or state.transport_mode!=request.transport_mode
                or any(getattr(state,key)!=getattr(request,key) for key in
                    ('execution_id','control_epoch','sdk_session','sdk_arm_generation'))
                or state.sequence<=self.sequence or not math.isfinite(received_monotonic)
                or received_monotonic<0 or not 0<nanoseconds(request.source_stamp)<=nanoseconds(state.source_stamp)<=now_ns
                or nanoseconds(state.valid_until)<=now_ns):
            return False
        self.sequence=state.sequence
        self.state=deepcopy(state)
        self.received=received_monotonic
        return True

    def confirmation(self,*,core,reference,lease,now_ns,now_monotonic):
        request,state=self.request,self.state
        if request is None or state is None:
            return None
        expected_basis=('sdk_callback_receipt_no_vendor_stamp' if self.transport_mode=='live'
                        else 'isolated_mock_callback_receipt')
        expected_mode={'general':(1,5),'stair':(3,7)}.get(state.target_mode)
        if (expected_mode is None or not state.confirmed or not state.capability_verified
                or not state.control_owned or not state.emergency_clear or state.speed_level!=1
                or (state.sdk_mode,state.motion_status)!=expected_mode
                or not state.sdk_session or state.sdk_arm_generation<=0
                or state.time_basis!=expected_basis or state.robot_callback_sequence<=0
                or lease is None or lease.revoked or not same_task(lease.version,state.version)
                or lease.transport_mode!=self.transport_mode
                or any(getattr(lease,key)!=getattr(request,key) for key in
                    ('execution_id','control_epoch','sdk_session','sdk_arm_generation'))
                or not 0<nanoseconds(lease.source_stamp)<=now_ns<nanoseconds(lease.valid_until)
                or reference.segment_id!=state.version.segment_id
                or not same_task(reference,state.version)
                or not 0<nanoseconds(state.robot_callback_stamp)<=nanoseconds(state.source_stamp)<=now_ns
                or now_ns>=nanoseconds(state.valid_until)
                or now_ns-nanoseconds(state.robot_callback_stamp)>2_500_000_000
                or not 0<=now_monotonic-self.received<=2.5):
            return None
        return ReceiptModeConfirmation(core.context,core.task_id,core.snapshot.route_hash,
            state.target_segment_id,state.target_mode,nanoseconds(state.robot_callback_stamp),
            nanoseconds(state.valid_until),self.received,state.robot_callback_sequence,
            state.sdk_session,state.time_basis)
