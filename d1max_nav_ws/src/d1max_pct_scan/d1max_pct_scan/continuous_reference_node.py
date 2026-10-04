"""Production odom reference transport; BT alone commits execution versions.

Callbacks are separately constructible without rclpy.init or a ROS graph.
First and successor windows use the same prepare/receipt/permit protocol.
"""
from copy import deepcopy
import hashlib
import json
import math
import time
import numpy as np

from .atomic_projection_state import validate as validate_state
from .atomic_navigation_inbox import navigation_event_is_new
from .continuous_reference_transport import ContinuousReferenceTransport, ReferenceReceipt, nanoseconds, set_stamp
from .source_route import canonical

PREFIX='/d1max/live_planning/'


def version_of(reference):
    from d1max_planning_interfaces.msg import ExecutionVersion
    version=ExecutionVersion(schema_version=3)
    for name in ('session_id','task_id','route_id','route_hash','map_version_id','localization_epoch',
                 'localization_seed_id','segment_id','anchor_id','anchor_revision','context_sequence','map_geometry_revision'):
        setattr(version,name,getattr(reference,name))
    version.reference_generation=reference.generation
    return version


class ReferenceCallbacks:
    def __init__(self,params,publish,now_ns,monotonic=time.monotonic):
        self.p,self.publish,self.now_ns,self.mono=params,publish,now_ns,monotonic
        if params['transport_mode'] not in ('live','isolated_mock'):
            raise ValueError('explicit_reference_transport_mode_required')
        self.map_evidence_age_s=float(params.get('map_evidence_max_age_s',.4))
        if not math.isfinite(self.map_evidence_age_s) or not 0<self.map_evidence_age_s<=.5:
            raise ValueError('map_evidence_age_exceeds_native_contract')
        self.map_evidence_age_ns=round(self.map_evidence_age_s*1e9)
        self.transport=ContinuousReferenceTransport(session_id=params['session_id'],
            map_version_id=params['map_version_id'],expected_hashes={k:params['expected_'+k]
                for k in ('source_map_sha256','tomogram_sha256','conditioning_sha256')},
            body_height_m=params['body_height_m'],body_height_calibration_id=params['body_height_calibration_id'],
            publish_staging=self.prepare,freshness_s=params.get('freshness_s',.4),
            reference_horizon_m=params.get('reference_horizon_m',2.))
        self.context=None; self.context_ready=False; self.map_report=None; self.map_received=-math.inf
        self.state=None; self.state_received=-math.inf
        from .local_navigation_state import LocalNavigationInbox
        self.local_inbox=LocalNavigationInbox()
        self.proposal=None; self.receipt=None; self.permit=None
        self.debug=None; self.debug_received=-math.inf
        # Half the 2 m horizon: same nominal cadence as before, commit-latency free.
        self.window_reissue_arc_m=.5*self.transport.reference_horizon_m
        self.context_last_sent=-math.inf
        self.last_proposal_monotonic=-math.inf
        self.error=''; self.cancel_generation=0
        self.native_retirement=None
        self.support_map=None
        self.route_progress_sequence=0
        self.route_progress_key=None
        if params.get('planning_manifest'):
            from .source_identity import SourceIdentityBridge
            self.support_map=SourceIdentityBridge.from_artifacts(params['planning_manifest'])
            if self.support_map.manifest['source_sha256']!=params['expected_source_map_sha256']:
                raise ValueError('reference_support_map_identity_mismatch')
        self.preparation=None
        if params.get('async_support_preparation',False):
            from .bounded_preparation import LatestPreparation
            self.preparation=LatestPreparation(self.build_support)

    def close(self):
        return self.preparation is None or self.preparation.close()

    def invalidate_preparation(self):
        if self.preparation is not None:
            self.preparation.invalidate()

    def on_navigation(self,message):
        state=validate_state(message,session_id=self.p['session_id'],map_version_id=self.p['map_version_id'],now_ns=self.now_ns())
        if not navigation_event_is_new(state.identity,state.source_ns,self.state):
            return False  # old epoch/seed callbacks cannot replace the current atomic frame context
        if self.context is None or state.identity!=(self.context['session_id'],self.context['epoch'],self.context['seed_id']):
            if self.context is not None:
                self.invalidate_preparation()
                self.proposal=self.receipt=self.permit=None
                self.debug=None; self.native_retirement=None
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
            self.try_recover_context()
            return False
        core=self.transport.core
        self.publish_route_progress()
        # Re-issue on look-ahead actually left in the accepted window, not on
        # progress since the owner commit: a late commit must not let native
        # exhaust the window (MEASURED_GOAL_REACHED) before the next one exists.
        remaining=self.transport.remaining_window_arc_m()
        if (core and self.transport.accepted is not None and self.transport.pending is None
                and core._progress and ((not self.transport.covers_current_segment_end()
                    and remaining is not None and remaining<=self.window_reissue_arc_m)
                    or self.transport.correction_due())
                and self.permit is not None and not self.permit.revoked
                and self.map_fresh() and self.mono()-self.last_proposal_monotonic>=.5):
            try:
                self.transport.propose_window(current_source_ns=self.now_ns(),now_monotonic=self.mono())
            except ValueError as error:
                self.error=str(error)
        return True

    def on_local_navigation(self,message):
        event=self.local_inbox.accept(message,session_id=self.p['session_id'],
            map_version_id=self.p['map_version_id'],now_ns=self.now_ns(),monotonic=self.mono())
        if event is None: return False
        if event.hard_failure:
            self.invalidate_preparation()
            self.transport.quarantined=True
            self.transport.discard_pending()
            self.proposal=self.receipt=self.permit=None
            self.error=event.reason
            return False
        if event.state is None: return False
        self.state_received=event.received
        self.publish('body',deepcopy(message.local_odometry))
        try:
            self.transport.on_local(message,received_monotonic=event.received,
                current_source_ns=self.now_ns(),now_monotonic=self.mono())
        except ValueError as error:
            self.error=str(error)
            return False
        self.publish_route_progress()
        self.refresh_window()
        return True

    def publish_route_progress(self):
        facts=self.transport.accepted_route_progress(current_source_ns=self.now_ns(),now_monotonic=self.mono())
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

    def refresh_window(self):
        t=self.transport
        if not t.core or t.quarantined or not self.map_fresh() or self.mono()-self.last_proposal_monotonic<.5:
            return
        remaining=t.pending_window_remaining()
        # A receipt only confirms delivery. If the robot has consumed that
        # pending window before a valid trajectory/owner commit, retire the
        # version and prepare from its actual current progress. The route and
        # any accepted incumbent remain untouched; generation fences old work.
        if t.pending is not None and remaining is not None and remaining<=.2:
            t.discard_pending(); self.proposal=self.receipt=None
            self.error='pending_window_consumed_rebuild_same_route'
        remaining=t.remaining_window_arc_m()
        if (t.pending is None and (t.accepted is None or remaining is not None
                and not t.covers_current_segment_end() and remaining<=self.window_reissue_arc_m
                or t.correction_due())):
            try: t.propose_window(current_source_ns=self.now_ns(),now_monotonic=self.mono())
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

    def on_route(self,message):
        # Accept the immutable BT task once, even if map ACK arrives later.
        # Actual native proposal delivery remains gated in prepare/tick.
        # RouteIngress retains sequence and cancellation ownership, so an old
        # queued active task cannot resurrect after its cancellation.
        return self.transport.on_route(message,current_source_ns=self.now_ns(),now_monotonic=self.mono())

    def prepare(self,reference):
        if not reference.path.poses:
            self.invalidate_preparation()
            self.cancel_generation=reference.generation
            self.proposal=self.receipt=None
            # A new task has no predecessor from the retired owner. Keep the
            # bridge generation fence, not the old BT execution version.
            self.permit=None
            self.publish('cancel_reference',reference)
            return
        if not self.map_fresh():
            self.transport.discard_pending()
            self.error='reference_requires_acknowledged_fresh_native_map'
            return
        from d1max_planning_interfaces.msg import ReferenceProposal, SupportReference
        from geometry_msgs.msg import Point
        proposal=ReferenceProposal()
        proposal.version=version_of(reference)
        proposal.reference=deepcopy(reference)
        proposal.expected_trajectory_id=-1
        if self.permit is not None:
            proposal.expected_version=deepcopy(self.permit.version)
            proposal.expected_trajectory_id=self.permit.trajectory_id
        proposal.proposal_id=f'{reference.task_id}.{reference.generation}'
        proposal.source_stamp=deepcopy(reference.path.header.stamp)
        set_stamp(proposal.valid_until,nanoseconds(proposal.source_stamp)+400_000_000)
        proposal.transport_mode=self.p['transport_mode']
        core=self.transport.pending_core or self.transport.core
        endpoint=core.snapshot.payload()['xyz'][-1]
        xyz=core.anchor.map_from_odom.inverse().point((endpoint[0],endpoint[1],endpoint[2]+core.body_height))
        proposal.goal_position=Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2]))
        frozen=core.snapshot.payload()
        proposal.has_goal_yaw=frozen['has_goal_yaw']
        proposal.goal_yaw_tolerance_rad=frozen['goal_yaw_tolerance_rad']
        if proposal.has_goal_yaw:
            yaw=frozen['goal_yaw']
            heading=core.anchor.map_from_odom.inverse().rotate((math.cos(yaw),math.sin(yaw),0.))
            if math.hypot(heading[0],heading[1])<.1:
                raise ValueError('goal_heading_projection_degenerate')
            proposal.goal_yaw=math.atan2(heading[1],heading[0])
        self.proposal,self.receipt=proposal,None
        self.last_proposal_monotonic=self.mono()
        value=core.snapshot.payload(); evidence=value['geometry_evidence']
        # Freeze the candidate's exact anchor, original source time and support
        # identity. The worker reads immutable map data, never transport/core.
        request=(deepcopy(proposal),deepcopy(value),core.anchor.map_from_odom.inverse(),
                 tuple(core._progress.body_source_xyz[:2]),core.body_height)
        if self.preparation is not None:
            self.preparation.submit(proposal.proposal_id,request)
            return
        self.deliver_support(*self.build_support(request))

    def build_support(self,request):
        from d1max_planning_interfaces.msg import SupportReference
        from geometry_msgs.msg import Point
        proposal,value,inverse,center,body_height=request
        evidence=value['geometry_evidence']
        support=SupportReference(version=deepcopy(proposal.version),
            support_reference_id=proposal.proposal_id, support_map_sha256=value['source_map_sha256'],
            floor_id=self.p['floor_id'],segment_kind='floor',required_mode='general',frame_id='d1max_loc_map',
            source_stamp=deepcopy(proposal.source_stamp),support_xy_radius_m=.2,
            body_reference_height_m=body_height,max_support_slope_rad=.15,max_support_step_m=.05,
            verified=bool(value['execution_eligible']),reason=value['eligibility_reason'])
        # Literal original-map returns are preserved in the snapshot evidence.
        # Reference interpolation is never relabelled as a measured return.
        points=evidence.get('observed_source_support_xyz',[])
        if self.support_map is not None:
            ids=self.support_map.tree.query_ball_point(center,6.)
            source=self.support_map.support[ids]
            if len(source)>20000:
                # Deterministic selection of real points, never voxel centres.
                _,unique=np.unique(np.floor(source/.05).astype(np.int64),axis=0,return_index=True)
                source=source[np.sort(unique)]
            if len(source)>20000:
                source=source[np.linspace(0,len(source)-1,20000,dtype=int)]
            points=source.tolist()
        from .ray_projection import rotate_many
        transformed=rotate_many(np.asarray(inverse.xyzw),
            np.asarray(points,dtype=float).reshape((-1,3)))+np.asarray(inverse.xyz)
        for odom in transformed:
            # Tracker operates in odom; preserve the bound anchor explicitly.
            support.support_ground_xyz.append(Point(x=float(odom[0]),y=float(odom[1]),z=float(odom[2])))
        support.frame_id='d1max_loc_odom'
        if not points: support.verified=False; support.reason='original_support_returns_missing'
        support.support_hash=hashlib.sha256(canonical(dict(map=value['source_map_sha256'],
            anchor=proposal.version.anchor_id,points=points)).encode()).hexdigest()
        return proposal,support

    def deliver_support(self,proposal,support):
        # Admission remains in the serialized owner lane. Slow/old results may
        # not overwrite a new task, clear an incumbent, or extend any lease.
        if (self.transport.quarantined or self.proposal is None
                or proposal!=self.proposal or not self.map_fresh()
                or self.transport.pending is None
                or proposal.version!=version_of(self.transport.pending)
                or support.version!=proposal.version or support.source_stamp!=proposal.source_stamp
                or not nanoseconds(proposal.source_stamp)<=self.now_ns()<=nanoseconds(proposal.valid_until)):
            return False
        self.publish('support',support)
        self.publish('proposal',proposal)
        # A prior map-ACK/freshness wait is no longer the reason once the
        # same immutable task has actually been delivered to native.
        self.error=''
        return True

    def on_receipt(self,message):
        p=self.proposal
        if (self.transport.quarantined or p is None or message.proposal_id!=p.proposal_id or message.version!=p.version
                or message.expected_version!=p.expected_version or message.expected_trajectory_id!=p.expected_trajectory_id
                or message.transport_mode!=self.p['transport_mode']
                or not nanoseconds(message.source_stamp)<=self.now_ns()<=nanoseconds(message.valid_until)):
            return False
        self.receipt=deepcopy(message)
        if not message.accepted:
            self.transport.discard_pending(); self.proposal=None
        return message.accepted

    def on_permit(self,message):
        if (self.transport.quarantined or message.transport_mode!=self.p['transport_mode'] or message.version.schema_version!=3
                or message.version.session_id!=self.p['session_id']
                or message.version.map_version_id!=self.p['map_version_id']
                or not nanoseconds(message.source_stamp)<=self.now_ns()<=nanoseconds(message.valid_until)
                or self.permit is not None and message.sequence<=self.permit.sequence):
            return False
        route=self.transport.route
        if (route is None or message.version.task_id!=route.task_id or message.version.route_hash!=route.route_hash):
            return False
        if self.proposal is not None and message.version==self.proposal.version:
            if not message.geometry_committed:
                return False
            if self.receipt is None or not self.receipt.accepted or self.transport.pending is None:
                return False
            # BT version commit, not receipt, installs the prepared reference.
            try:
                self.transport.commit_pending(current_source_ns=self.now_ns(),now_monotonic=self.mono(),
                                              owner_committed=True)
            except ValueError as error:
                self.error=str(error)
                return False
            self.proposal=None
        elif self.transport.accepted is None or message.version!=version_of(self.transport.accepted):
            return False
        self.permit=deepcopy(message)
        self.publish_route_progress()
        return True

    def on_debug(self,message):
        if (message.session_id!=self.p['session_id'] or message.header.frame_id!='d1max_loc_odom'
                or not 0<=self.now_ns()-nanoseconds(message.header.stamp)<=400_000_000): return False
        self.debug,self.debug_received=deepcopy(message),self.mono()
        if (message.phase=='cancelled' and not message.valid
                and self.cancel_generation>0 and message.generation==self.cancel_generation
                and nanoseconds(message.header.stamp)>=self.transport._retired_stamp_ns):
            self.native_retirement=dict(generation=message.generation,
                source_ns=nanoseconds(message.header.stamp),received=self.mono())
            self.try_recover_context()
        return True

    def try_recover_context(self):
        t=self.transport; ack=self.native_retirement
        if not t.quarantined or not self.context_ready or not ack:
            return False
        if (not 0<=self.now_ns()-ack['source_ns']<=2_500_000_000
                or not 0<=self.mono()-ack['received']<=2.5):
            return False
        try:
            t.recover_retired_context(retired_generation=ack['generation'],
                context_sequence=self.context['sequence'],
                context=(self.context['session_id'],self.context['epoch'],self.context['seed_id']),
                current_source_ns=self.now_ns(),now_monotonic=self.mono())
        except ValueError:
            return False
        self.proposal=self.receipt=self.permit=self.debug=self.native_retirement=None
        self.error=''
        return True

    def map_fresh(self):
        m=self.map_report
        return bool(self.context_ready and m and m.get('valid') is True
            and self.mono()-self.map_received<=.4
            and 0<=self.now_ns()-m['source_stamp_ns']<=self.map_evidence_age_ns
            and len(m.get('sources',[]))==2
            and all(type(v.get('sensor_id')) is int for v in m['sources'])
            and {v.get('sensor_id') for v in m['sources']}=={0,1}
            and all(type(v.get('integrated_stamp_ns')) is int
                and 0<=self.now_ns()-v['integrated_stamp_ns']<=self.map_evidence_age_ns for v in m['sources'])
            and m['source_stamp_ns']==min(v['integrated_stamp_ns'] for v in m['sources']))

    def tick(self):
        if self.preparation is not None:
            result=self.preparation.take()
            if result is not None and self.proposal is not None and result.key==self.proposal.proposal_id:
                if result.error:
                    self.error='support_preparation_failed:'+result.error
                else:
                    self.deliver_support(*result.value)
        self.sync_context()
        if self.p.get('local_state_enabled') and self.local_inbox.usable(now_ns=self.now_ns(),
                monotonic=self.mono(),receipt_timeout_s=.4):
            self.refresh_window()
        if (self.proposal and self.now_ns()>nanoseconds(self.proposal.valid_until)
                and not (self.receipt is not None and self.receipt.accepted)):
            # This deadline bounds delivery to native. Its accepted receipt is
            # a durable geometry event, not a renewable sensor/command lease.
            # Keep accepted pending geometry until the fresh BT commit arrives.
            self.transport.discard_pending(); self.proposal=self.receipt=None
        if (self.transport.core and self.transport.pending is None and self.transport.accepted is None
                and self.map_fresh() and self.mono()-self.state_received<.4
                and self.mono()-self.last_proposal_monotonic>=.5):
            try:
                self.transport.propose_window(current_source_ns=self.now_ns(),now_monotonic=self.mono())
            except ValueError as error: self.error=str(error)
        t=self.transport; d=self.debug; now=self.now_ns(); core=t.core
        debug_fresh=bool(d and self.mono()-self.debug_received<=.4
            and 0<=now-nanoseconds(d.header.stamp)<=400_000_000)
        # A no-solution notification is a versioned attempt outcome, not a
        # collision lease. Native can wait without repeating that event. Keep
        # its cause until this pending version changes; never refresh its
        # timestamp or use this diagnostic as trajectory/control validity.
        pending_phase=(d.phase if d and not d.valid and t.pending is not None
            and self.receipt is not None and self.receipt.accepted
            and d.generation==t.pending.generation
            and nanoseconds(d.header.stamp)>=nanoseconds(t.pending.path.header.stamp) else '')
        valid=bool(d and d.valid and self.mono()-self.debug_received<=.4
            and 0<=now-d.checked_map_source_stamp_ns<=self.map_evidence_age_ns
            and 0<=now-d.checked_body_source_stamp_ns<=400_000_000
            and not t.quarantined and t.accepted and d.generation==t.accepted.generation
            and d.checked_context_sequence==(self.context or {}).get('sequence') and self.map_fresh())
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
            native_reference_received=bool(self.receipt and self.receipt.accepted),
            local_debug_generation=d.generation if d else 0,local_debug_fresh=debug_fresh,
            native_pending_phase=pending_phase,
            spline_visual_valid=valid,last_spline_id=d.plan_id if d else -1,
            local_debug_phase=d.phase if d else 'waiting_native_trajectory',reason=self.error,
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
        ExecutionPermit,SupportReference,ReferencePath,LocalPlanDebug,RouteProgress)
    class ReferenceNode(Node):
        def __init__(self):
            super().__init__('d1max_continuous_reference')
            defaults=dict(session_id='',map_version_id='',expected_source_map_sha256='',
                expected_tomogram_sha256='',expected_conditioning_sha256='',body_height_m=.55,
                body_height_calibration_id='',freshness_s=.4,map_evidence_max_age_s=.4,transport_mode='live',planning_manifest='',
                reference_horizon_m=2.,floor_id='',local_state_enabled=False,async_support_preparation=True)
            p={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
            durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            pubs={key:self.create_publisher(kind,PREFIX+topic,durable if key in ('context','support') else 5)
                for key,kind,topic in (('body',Odometry,'body_pose'),('context',String,'scan_map_context'),
                    ('status',String,'scan_bridge_status'),('proposal',ReferenceProposal,'execution/reference_proposal'),
                    ('route_progress',RouteProgress,'execution/route_progress'),
                    ('support',SupportReference,'execution/support'),('cancel_reference',ReferencePath,'scan_reference'))}
            self.callbacks=ReferenceCallbacks(p,lambda key,value:pubs[key].publish(String(data=json.dumps(value))
                if key in ('context','status') else value),lambda:self.get_clock().now().nanoseconds)
            c=self.callbacks
            for kind,topic,callback,qos in ((RouteReference,PREFIX+'committed_route',c.on_route,durable),
                    (NavigationState,'/d1max/localization/navigation/state',c.on_navigation,qos_profile_sensor_data),
                    (ReferenceReceipt,PREFIX+'execution/reference_receipt',c.on_receipt,5),
                    (ExecutionPermit,PREFIX+'execution/permit',c.on_permit,5),
                    (LocalPlanDebug,PREFIX+'native_local_debug',c.on_debug,5),
                    (String,PREFIX+'scan_map_context_ack',lambda m:c.on_context_ack(json.loads(m.data)),durable),
                    (String,PREFIX+'rays_status',lambda m:c.on_map(json.loads(m.data)),5)):
                self.create_subscription(kind,topic,lambda m,fn=callback:self.checked(fn,m),qos)
            if p['local_state_enabled']:
                self.create_subscription(LocalNavigationState,'/d1max/localization/navigation/local_state',
                    lambda m:self.checked(c.on_local_navigation,m),qos_profile_sensor_data)
            self.create_timer(.05,c.tick)
        def checked(self,callback,message):
            try: callback(message)
            except (ValueError,TypeError,AttributeError,KeyError) as error: self.callbacks.error=str(error)
    rclpy.init(args=args); node=ReferenceNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException): pass
    finally:
        node.callbacks.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
