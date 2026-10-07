"""ROS boundary for identity-preserving independent command admission."""
import json
import os
import time


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from d1max_planning_interfaces.msg import (ExecutionPermit, MotionDemand, MotionValidation,
        TrajectoryValidation, ProjectedRays, TrackingProgress, ExecutionHandoffGrant,
        PreparedMotionDemand, ExecutionCommitAck, SDKStationaryEvidence)
    from .execution_safety import ExecutionSafety
    from .command_admission import CommandAdmission
    from .execution_handoff import ExecutionHandoffAdmission
    from .braking_model import load_model
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp':
        raise ValueError('unchanged_zenoh_required')
    class SafetyNode(Node):
        def __init__(self):
            super().__init__('d1max_execution_safety')
            sid=self.declare_parameter('session_id','').value
            mode=self.declare_parameter('transport_mode','live').value
            if mode=='isolated_mock':
                from .isolated_zenoh import validate_environment
                validate_environment()
            record=self.declare_parameter('execution_braking_model_record','').value
            digest=self.declare_parameter('execution_braking_model_sha256','').value
            load_model(record,digest,mode)
            from pathlib import Path
            from .execution_timing import validate_timing,validate_stationary
            bound_record=json.loads(Path(record).read_text())
            timing=validate_timing(bound_record)
            self.core=ExecutionSafety(sid,mode,braking_model_sha256=digest,
                stationary_policy=validate_stationary(bound_record),
                sensor_source_age_s=timing['sensor_source_age_bound_s'],
                maximum_isaac_actor_id=self.declare_parameter('maximum_isaac_actor_id',0).value,
                max_speed=self.declare_parameter('max_speed',.30).value,
                max_yaw=self.declare_parameter('max_yaw',.50).value)
            self.handoff=ExecutionHandoffAdmission(self.core,
                enabled=self.declare_parameter('writer_handoff_enabled',False).value)
            self.admission=self.handoff.ordinary
            self.publisher=self.create_publisher(MotionDemand,'/d1max/live_planning/execution/safe_demand',1)
            self.prepared_publisher=self.create_publisher(PreparedMotionDemand,'/d1max/live_planning/execution/safe_prepared_demand',1)
            self.status=self.create_publisher(String,'/d1max/live_planning/execution/safety_status',1)
            self.accepted=self.rejected=0
            self.reason='waiting_evidence'
            self.create_subscription(ExecutionPermit,'/d1max/live_planning/execution/permit',
                lambda m:self.handoff.on_permit(m,self.get_clock().now().nanoseconds),5)
            self.create_subscription(ExecutionHandoffGrant,'/d1max/live_planning/execution/handoff_grant',
                lambda m:self.handoff.on_grant(m,self.get_clock().now().nanoseconds),5)
            self.create_subscription(ExecutionCommitAck,'/d1max/live_planning/execution/commit_ack',
                lambda m:self.handoff.on_ack(m,self.get_clock().now().nanoseconds),5)
            self.create_subscription(SDKStationaryEvidence,'/d1max/live_planning/execution/stationary_evidence',
                self.handoff.on_stationary,5)
            self.create_subscription(PreparedMotionDemand,'/d1max/live_planning/execution/prepared_demand',
                lambda m:self.output(self.handoff.prepared_demand(m,self.get_clock().now().nanoseconds),prepared=True),2)
            self.create_subscription(TrajectoryValidation,'/d1max/live_planning/execution/validation',self.on_validation,5)
            self.create_subscription(TrackingProgress,'/d1max/live_planning/tracking_progress',self.core.on_progress,2)
            self.create_subscription(ProjectedRays,'/d1max/live_planning/rays_odom',
                lambda m:self.core.on_rays(m,self.get_clock().now().nanoseconds),qos_profile_sensor_data)
            self.create_subscription(MotionDemand,'/d1max/live_planning/execution/demand',self.on_demand,1)
            self.create_subscription(MotionValidation,'/d1max/live_planning/execution/motion_validation',self.on_motion_proof,5)
            from .component_health import create_functional_heartbeat_timer
            self.status_timer=create_functional_heartbeat_timer(self,self.on_status)
        def on_demand(self,message):
            now=self.get_clock().now().nanoseconds
            self.output(self.admission.demand(message,now))
        def on_motion_proof(self,message):
            prepared,result=self.handoff.motion_proof(message,self.get_clock().now().nanoseconds)
            self.output(result,prepared=prepared)
        def on_validation(self,message):
            ordinary,prepared=self.handoff.validation(message,self.get_clock().now().nanoseconds)
            for result in ordinary:
                self.output(result)
            for result in prepared:self.output(result,prepared=True)
        def output(self,result,*,prepared=False):
            self.reason=result.reason
            if result.allowed:
                self.accepted+=1
            else:
                self.rejected+=1
            if result.output is not None:
                (self.prepared_publisher if prepared else self.publisher).publish(result.output)
            # No periodic cached command publisher. A higher real native
            # proof may upgrade one unchanged command only inside its original
            # source/lease limits; it cannot renew control or sensor time.
        def on_status(self):
            self.status.publish(String(data=json.dumps(dict(schema=3,phase='command_admission',
                session_id=self.core.session,transport_mode=self.core.mode,reason=self.reason,
                accepted=self.accepted,rejected=self.rejected,callback_wall_time=time.time(),
                received_at_unix=self.get_clock().now().nanoseconds*1e-9))))
    rclpy.init(args=args)
    node=SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()
