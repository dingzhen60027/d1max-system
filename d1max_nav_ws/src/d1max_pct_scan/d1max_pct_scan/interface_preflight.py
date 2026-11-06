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
        Navigate.Goal: ('has_goal_yaw','goal_yaw_tolerance_rad','request_execution'),
        ComputeRoute.Goal: (), FollowRoute.Goal: ('snapshot',),
        ConfirmExecution.Request: ('route_hash','request_id'), PrepareTransition.Request: ('reason','session_id'),
        NavigationState: ('local_odometry','global_odometry','posterior_stamp','imu_stamp'),
        NavigationHealth: ('evidence_source_stamp','local_control_ready','global_planning_ready'),
        ReferencePath: ('anchor_id','route_hash','point_segment_ids','point_reference'),
        TaggedBspline: ('anchor_id','route_hash','segment_id','point_reference','splice',
                       'valid_start_time','valid_start_arc_length','join_source_stamp',
                       'join_pose','join_twist','join_acceleration','join_acceleration_valid',
                       'body_heading_seed_valid','body_heading_seed_yaw',
                       'body_heading_sampling_intervals'),
    }
    for cls, names in required.items():
        message = cls()
        if not all(hasattr(message,name) for name in ('schema_version',)+names):
            raise ValueError('Navigation v2 interface missing: '+cls.__name__)
        message.schema_version = 2
        restored = deserialize_message(serialize_message(message), cls)
        if restored != message:
            raise ValueError('Navigation interface typesupport mismatch: '+cls.__name__)
    # Non-default payload detects stale generated/native layouts; merely
    # round-tripping an all-zero message does not prove the new seed contract.
    seeded=TaggedBspline(body_heading_seed_valid=True,body_heading_seed_yaw=.73,
                        body_heading_sampling_intervals=217)
    if deserialize_message(serialize_message(seeded),TaggedBspline)!=seeded:
        raise ValueError('Native heading seed typesupport mismatch')
    progress = TrackingProgress()
    for name in ('trajectory_id','anchor_id','localization_epoch','localization_seed_id','arc_length'):
        if not hasattr(progress,name):
            raise ValueError('Tracking progress v2 interface missing: '+name)
    deserialize_message(serialize_message(progress), TrackingProgress)
    local=LocalNavigationState(schema_version=2)
    for name in ('local_odometry','source_stamp','posterior_stamp','imu_stamp','valid_until','usable','localization_epoch'):
        if not hasattr(local,name):raise ValueError('Continuous local interface missing:'+name)
    if deserialize_message(serialize_message(local),LocalNavigationState)!=local:
        raise ValueError('Continuous local typesupport mismatch')
    return {'schema_version':2,'local_state_contract':'continuous_odom_v2','message_types_verified':len(required)+2,
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
        'TaskExecutionLease': ('version','execution_id','sequence','allowed','revoked','valid_until'),
        'LocalMotionCommand': ('version','lease_sequence','body_source_stamp','body_valid_until','valid_until','velocity',
                               'acknowledged_discontinuity_write'),
        'TaskCommandFeedback': ('version','execution_id','write_sequence','source_stamp','write_submitted',
            'command_sequence','command_source_stamp','command_valid_until','velocity',
            'sdk_arguments_available','sdk_nonzero','sdk_forward','sdk_lateral','sdk_yaw',
            'discontinuity_write_sequence','discontinuity_stamp','interrupted_command_sequence'),
        'LocalExecutionStatus': ('version','execution_id','sequence','at_goal','software_retired'),
        'LocalTrajectoryCommit': ('version','expected_version','expected_trajectory_id','commit_sequence','body_source_stamp','native_validation_sequence'),
        'TrajectorySplice': ('incumbent_version','incumbent_trajectory_id','measured_progress_stamp','splice_curve_time','splice_arc_length'),
        'ExecutionVersion': ('task_id','route_hash','anchor_revision','context_sequence'),
        'ExecutionPermit': ('geometry_committed','execution_id','valid_until','validation_sequence'),
        'MotionDemand': ('permit_sequence','safety_checked','safety_source_stamp',
                         'motion_validation_sequence','braking_model_sha256','acknowledged_discontinuity_write'),
        'MotionValidation': ('demand_sequence','demand_source_stamp','demand_body_source_stamp',
            'body_source_stamp','demand_valid_until','velocity','map_snapshot_revision',
            'front_ray_source_stamp','rear_ray_source_stamp','braking_model_sha256',
            'handoff_id','entry_admission_sequence','entry_curve_pose','entry_curve_twist'),
        'TrajectoryValidation': ('splice','front_ray_source_stamp','rear_ray_source_stamp','support_hash','valid_until',
                                'remaining_curve','checked_from_time','checked_to_time','curve_duration','reverse_margin_m'),
        'TrajectoryAdmission': ('accepted','sequence'),
        'TrackerGeometryReceipt': ('version','frame_id','transport_mode','trajectory_id','installation_sequence','permit_sequence',
            'admission_sequence','validation_sequence','installed_at','body_source_stamp','installed'),
        'RouteProgress': ('version','sequence','frame_id','transport_mode','body_source_stamp','odom_body_pose',
            'anchor_source_stamp','map_from_odom','source_map_body_xyz','body_reference_height_m',
            'edge_index','measured_arc_m','confirmed_arc_m','cross_track_m'),
        'SDKExecutionState': ('sdk_session','grant_ready','fault_latched','writer_commit_sequence','applied_trajectory_id',
            'motion_mode','motion_mode_confirmed','mode_transition_active'),
        'ModeTransitionRequest': ('version','request_id','sequence','target_segment_id','target_mode',
            'execution_id','control_epoch','sdk_session','sdk_arm_generation','body_source_stamp','preview_only'),
        'ModeTransitionState': ('version','request_id','request_sequence','sequence','target_segment_id','target_mode',
            'execution_id','control_epoch','sdk_session','sdk_arm_generation','confirmed','capability_verified',
            'robot_callback_sequence','robot_callback_stamp','time_basis'),
        'SDKExecutionRetirement': ('schema_version','version','execution_id','control_epoch','sdk_session',
            'sdk_arm_generation','grant_request_id','sequence','source_stamp','transport_mode',
            'nonzero_blocked','software_retired','motion_never_submitted','pending_sdk_transactions'),
        'ExecutionHandoffGrant': ('handoff_id','expected_commit_sequence','incumbent','candidate','transition_deadline',
            'transition_mode','stationary_evidence'),
        'SDKStationaryEvidence': ('zero_write_sequence','zero_ack_at','mc_raw_stamp_ns','capture_lower_bound',
            'capture_upper_bound','writer_commit_sequence','stationary_samples','stationary_duration_sec',
            'velocity_metric','measured_planar_mps','measured_vertical_mps'),
        'PreparedMotionDemand': ('handoff_id','demand','entry_admission','measured_pose','entry_source_stamp','curve_entry_pose'),
        'ExecutionCommitAck': ('handoff_id','commit_sequence','candidate_version','applied','write_submitted','write_acknowledged',
            'demand_body_source_stamp','entry_source_stamp'),
        'StopReport': ('measured_stop_confirmed','mc_raw_stamp_ns','time_basis',
            'velocity_metric','measured_planar_mps','measured_vertical_mps'),
        'ReferenceProposal': ('reference',),
        'ReferenceReceipt': ('expected_version','accepted'),
        'RouteDeliveryReceipt': ('source_stamp','receipt_stamp','session_id','task_id','route_id',
            'route_hash','delivery_sequence','active','status','native_retirement_required',
            'native_retirement_confirmed','native_generation'),
        'SupportReference': ('support_ground_xyz','stair_contact_indices','stair_riser_xyz',
                             'stair_riser_owner','body_reference_height_m','verified','regions',
                             'stair_model_hash','stair_model_surfaces','stair_model_obstacles'),
        'SupportRegion': ('segment_id','floor_id','segment_kind','required_mode','begin_index','end_index'),
        'StairSurfaceModel': ('surface_id','segment_id','kind','pose','polygon',
            'support_grade','support_intercept_m','height_association_m','contact_tolerance_m',
            'tread_stations_m','tread_heights_m'),
        'StairBoundaryModel': ('structure_id','pose','half_extents'),
    }
    for name, fields in required.items():
        cls=getattr(msg,name,None)
        if cls is None:raise ValueError('Execution v3 interface missing: '+name)
        value=cls()
        if not all(hasattr(value,f) for f in fields):
            raise ValueError('Execution v3 interface missing: '+name)
        if deserialize_message(serialize_message(value),cls)!=value:
            raise ValueError('Execution v3 typesupport mismatch: '+name)
    # A sender gap is an actual writer event, not a fresh lease or measurement.
    # Non-default values catch new Python messages paired with stale C support.
    for value in (msg.MotionDemand(acknowledged_discontinuity_write=29),
                  msg.LocalMotionCommand(acknowledged_discontinuity_write=29),
                  msg.TaskCommandFeedback(schema_version=1,execution_id='preflight',
                      write_sequence=37,write_submitted=True,command_selected=True,
                      command_sequence=41,sdk_arguments_available=True,sdk_nonzero=True,
                      sdk_forward=.3125,sdk_lateral=0.,sdk_yaw=-.25,
                      discontinuity_write_sequence=29,interrupted_command_sequence=31)):
        if deserialize_message(serialize_message(value),type(value))!=value:
            raise ValueError('Actual writer feedback typesupport mismatch: '+type(value).__name__)
    # Exercise non-default fields: an all-zero round trip can miss a stale
    # native typesupport layout, including silently lost contact provenance.
    from geometry_msgs.msg import Point
    contact=msg.SupportReference(support_ground_xyz=[Point(x=.2,y=.3,z=.4),Point(x=.5,y=.6,z=.7)],
        stair_contact_indices=[1],stair_riser_xyz=[Point(x=.25,y=.35,z=.45)],
        stair_riser_owner=[0],regions=[msg.SupportRegion(segment_id='entry',floor_id='stairs',
            segment_kind='stair_up',required_mode='stair',begin_index=0,end_index=2)])
    if deserialize_message(serialize_message(contact),type(contact))!=contact:
        raise ValueError('Literal stair contact typesupport mismatch')
    from geometry_msgs.msg import Pose, Quaternion, Vector3
    model=msg.SupportReference(stair_model_hash='preflight-model-v1',verified=False,
        stair_model_surfaces=[msg.StairSurfaceModel(surface_id='flight',segment_id='up',
            kind='continuous_support_envelope',pose=Pose(position=Point(x=1.,y=2.,z=3.),
                orientation=Quaternion(w=1.)),polygon=[Point(x=-1.,y=-.6),
                Point(x=1.,y=-.6),Point(x=1.,y=.6),Point(x=-1.,y=.6)],
            support_grade=.5,support_intercept_m=.1,height_association_m=.25,contact_tolerance_m=.02)],
        stair_model_obstacles=[msg.StairBoundaryModel(structure_id='wall',
            pose=Pose(orientation=Quaternion(w=1.)),half_extents=Vector3(x=1.,y=.1,z=.8))])
    if deserialize_message(serialize_message(model),type(model))!=model:
        raise ValueError('Parametric stair model typesupport mismatch')
    for value in (msg.SupportReference(regions=[msg.SupportRegion(segment_id='entry',floor_id='floor1',
                      segment_kind='floor',required_mode='general',begin_index=2,end_index=19)]),
                  msg.SDKExecutionState(motion_mode='stair',motion_mode_confirmed=True,
                                      mode_transition_active=True),
                  msg.ModeTransitionRequest(schema_version=1,request_id='preflight',
                      target_mode='stair',sequence=17,sdk_arm_generation=3),
                  msg.ModeTransitionState(schema_version=1,request_id='preflight',
                      target_mode='general',confirmed=True,robot_callback_sequence=19,
                      time_basis='sdk_callback_receipt_no_vendor_stamp')):
        if deserialize_message(serialize_message(value),type(value))!=value:
            raise ValueError('Stair mode transaction typesupport mismatch')
    for cls in (srv.ExecutionGrant.Request,srv.ExecutionGrant.Response):
        value=cls()
        for field in ('version','execution_id','control_epoch','request_id','sdk_session','transport_mode'):
            if not hasattr(value,field):
                raise ValueError('ExecutionGrant transaction binding missing:'+field)
        if cls is srv.ExecutionGrant.Response:
            if not hasattr(value,'definitely_not_admitted') or value.definitely_not_admitted is not False:
                raise ValueError('ExecutionGrant explicit non-admission contract missing or unsafe default')
        if deserialize_message(serialize_message(value),cls)!=value:
            raise ValueError('ExecutionGrant typesupport mismatch')
    return dict(result,execution_schema_version=3,handoff_contract='local_executor_v1',execution_architecture='task_follower_v1',
        route_progress_contract='source_bound_full_route_arc_v1',
        command_feedback_contract='actual_writer_discontinuity_v1',
        grant_rejection_contract='writer_bound_never_admitted_v1',
        execution_retirement_contract='writer_bound_software_retirement_v1',
        stationary_velocity_contract='raw_3d_with_guarded_planar_v1',
        route_delivery_contract='manager_bound_delivery_retirement_v1',
        mode_transition_contract='writer_bound_adjacent_segment_v1',
        support_transition_contract='literal_adjacent_regions_v1',
        stair_contact_contract='qualified_literal_indices_and_source_risers_v2',
        stair_model_contract='parametric_stair_geometry_v1',
        initial_pose_transaction='bt_initial_pose_transaction_v1',execution_types_verified=len(required)+5)
