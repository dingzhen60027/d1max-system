"""Resolve generated v2 Python and C typesupport before spawning any nodes."""


def validate_interfaces():
    from rclpy.serialization import serialize_message, deserialize_message
    from d1max_navigation_bt_interfaces.msg import RouteSnapshot, RouteReference, NavigationHealth
    from d1max_navigation_bt_interfaces.action import Navigate, ComputeRoute, FollowRoute
    from d1max_navigation_bt_interfaces.srv import ConfirmExecution, PrepareTransition
    from d1max_planning_interfaces.msg import NavigationState, ReferencePath, TaggedBspline, TrackingProgress, LocalNavigationState
    required = {
        RouteSnapshot: ('route_hash','point_floor_ids','segments','execution_eligible'),
        RouteReference: ('delivery_sequence','active','snapshot'),
        Navigate.Goal: ('has_goal_yaw','goal_yaw_tolerance_rad'),
        ComputeRoute.Goal: (), FollowRoute.Goal: ('snapshot',),
        ConfirmExecution.Request: ('route_hash','request_id'), PrepareTransition.Request: ('reason','session_id'),
        NavigationState: ('local_odometry','global_odometry','posterior_stamp','imu_stamp'),
        NavigationHealth: ('evidence_source_stamp','local_control_ready','global_planning_ready'),
        ReferencePath: ('anchor_id','route_hash','point_segment_ids','point_reference'),
        TaggedBspline: ('anchor_id','route_hash','segment_id','point_reference',
                       'valid_start_time','valid_start_arc_length','join_source_stamp',
                       'join_pose','join_twist','join_acceleration','join_acceleration_valid'),
    }
    for cls, names in required.items():
        message = cls()
        if not all(hasattr(message,name) for name in ('schema_version',)+names):
            raise ValueError('Navigation v2 interface missing: '+cls.__name__)
        message.schema_version = 2
        restored = deserialize_message(serialize_message(message), cls)
        if restored != message:
            raise ValueError('Navigation interface typesupport mismatch: '+cls.__name__)
    progress = TrackingProgress()
    for name in ('trajectory_id','anchor_id','localization_epoch','localization_seed_id','arc_length'):
        if not hasattr(progress,name):
            raise ValueError('Tracking progress v2 interface missing: '+name)
    deserialize_message(serialize_message(progress), TrackingProgress)
    local=LocalNavigationState(schema_version=1)
    for name in ('local_odometry','source_stamp','posterior_stamp','imu_stamp','usable','localization_epoch'):
        if not hasattr(local,name):raise ValueError('Continuous local interface missing:'+name)
    if deserialize_message(serialize_message(local),LocalNavigationState)!=local:
        raise ValueError('Continuous local typesupport mismatch')
    return {'schema_version':2,'local_state_contract':'continuous_odom_v1','message_types_verified':len(required)+2,
            'ros_context_started':False}


def validate_execution_interfaces():
    """Round-trip actual generated v3 types before starting any process."""
    from rclpy.serialization import serialize_message, deserialize_message
    from d1max_planning_interfaces import msg, srv
    result=validate_interfaces()
    from d1max_navigation_bt_interfaces.srv import PrepareInitialPose
    from d1max_navigation_bt_interfaces.msg import InitialPoseOutcome
    for cls,fields in ((PrepareInitialPose.Request,('operation','session_id','request_id','source_map_sha256','body_pose')),
                       (PrepareInitialPose.Response,('accepted','ready','application_authorized','physical_stop_confirmed','reason')),
                       (InitialPoseOutcome,('request_id','map_version_id','intent_source_stamp','source_stamp',
                           'localization_epoch','localization_seed_id','accepted','applied'))):
        value=cls()
        if not all(hasattr(value,name) for name in fields):
            raise ValueError('Initial pose transaction interface missing:'+cls.__name__)
        if deserialize_message(serialize_message(value),cls)!=value:
            raise ValueError('Initial pose transaction typesupport mismatch')
    required={
        'ExecutionVersion': ('task_id','route_hash','anchor_revision','context_sequence'),
        'ExecutionPermit': ('geometry_committed','execution_id','valid_until','validation_sequence'),
        'MotionDemand': ('permit_sequence','safety_checked','safety_source_stamp',
                         'motion_validation_sequence','braking_model_sha256'),
        'MotionValidation': ('demand_sequence','demand_source_stamp','demand_body_source_stamp',
            'body_source_stamp','demand_valid_until','velocity','map_snapshot_revision',
            'front_ray_source_stamp','rear_ray_source_stamp','braking_model_sha256',
            'handoff_id','entry_admission_sequence','entry_curve_pose','entry_curve_twist'),
        'TrajectoryValidation': ('front_ray_source_stamp','rear_ray_source_stamp','support_hash','valid_until',
                                'remaining_curve','checked_from_time','checked_to_time','curve_duration','reverse_margin_m'),
        'TrajectoryAdmission': ('accepted','sequence'),
        'TrackerGeometryReceipt': ('version','frame_id','transport_mode','trajectory_id','installation_sequence','permit_sequence',
            'admission_sequence','validation_sequence','installed_at','body_source_stamp','installed'),
        'RouteProgress': ('version','sequence','frame_id','transport_mode','body_source_stamp','odom_body_pose',
            'anchor_source_stamp','map_from_odom','source_map_body_xyz','body_reference_height_m',
            'edge_index','measured_arc_m','confirmed_arc_m','cross_track_m'),
        'SDKExecutionState': ('sdk_session','grant_ready','fault_latched','writer_commit_sequence','applied_trajectory_id'),
        'ExecutionHandoffGrant': ('handoff_id','expected_commit_sequence','incumbent','candidate','transition_deadline',
            'transition_mode','stationary_evidence'),
        'SDKStationaryEvidence': ('zero_write_sequence','zero_ack_at','mc_raw_stamp_ns','capture_lower_bound',
            'capture_upper_bound','writer_commit_sequence','stationary_samples','stationary_duration_sec'),
        'PreparedMotionDemand': ('handoff_id','demand','entry_admission','measured_pose','entry_source_stamp','curve_entry_pose'),
        'ExecutionCommitAck': ('handoff_id','commit_sequence','candidate_version','applied','write_submitted','write_acknowledged',
            'demand_body_source_stamp','entry_source_stamp'),
        'StopReport': ('measured_stop_confirmed','mc_raw_stamp_ns','time_basis'),
        'ReferenceProposal': ('reference',),
        'ReferenceReceipt': ('expected_version','accepted'),
        'SupportReference': ('support_ground_xyz','body_reference_height_m','verified'),
    }
    for name, fields in required.items():
        cls=getattr(msg,name,None)
        if cls is None:raise ValueError('Execution v3 interface missing: '+name)
        value=cls()
        if not all(hasattr(value,f) for f in fields):
            raise ValueError('Execution v3 interface missing: '+name)
        if deserialize_message(serialize_message(value),cls)!=value:
            raise ValueError('Execution v3 typesupport mismatch: '+name)
    for cls in (srv.ExecutionGrant.Request,srv.ExecutionGrant.Response):
        value=cls()
        if deserialize_message(serialize_message(value),cls)!=value:
            raise ValueError('ExecutionGrant typesupport mismatch')
    return dict(result,execution_schema_version=3,handoff_contract='writer_applied_cas_v3',
        route_progress_contract='source_bound_full_route_arc_v1',
        initial_pose_transaction='bt_initial_pose_transaction_v1',execution_types_verified=len(required)+5)
