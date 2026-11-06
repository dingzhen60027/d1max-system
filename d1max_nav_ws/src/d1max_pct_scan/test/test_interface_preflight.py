from d1max_pct_scan.interface_preflight import validate_interfaces
import pytest


def test_actual_generated_cpp_python_contract_roundtrips_without_ros_context():
    result=validate_interfaces()
    assert result['schema_version'] == 2
    assert result['message_types_verified'] == 13
    assert result['local_state_contract']=='continuous_odom_v2'
    assert result['ros_context_started'] is False


def test_actual_writer_cas_interfaces_roundtrip_without_starting_ros():
    from d1max_pct_scan.interface_preflight import validate_execution_interfaces
    result=validate_execution_interfaces()
    assert result['execution_schema_version']==3
    assert result['handoff_contract']=='local_executor_v1'
    assert result['execution_architecture']=='task_follower_v1'
    assert result['execution_types_verified']==35
    assert result['command_feedback_contract']=='actual_writer_discontinuity_v1'
    assert result['mode_transition_contract']=='writer_bound_adjacent_segment_v1'
    assert result['route_delivery_contract']=='manager_bound_delivery_retirement_v1'
    assert result['execution_retirement_contract']=='writer_bound_software_retirement_v1'
    assert result['stationary_velocity_contract']=='raw_3d_with_guarded_planar_v1'
    assert result['initial_pose_transaction']=='bt_initial_pose_transaction_v1'
    assert result['route_progress_contract']=='source_bound_full_route_arc_v1'
    assert result['stair_contact_contract']=='qualified_literal_indices_and_source_risers_v2'
    assert result['stair_model_contract']=='parametric_stair_geometry_v1'


@pytest.mark.parametrize('field', ['stair_model_hash','stair_model_surfaces','stair_model_obstacles'])
def test_nonzero_model_detects_stale_typesupport(monkeypatch,field):
    from d1max_planning_interfaces.msg import SupportReference
    import rclpy.serialization
    from d1max_pct_scan.interface_preflight import validate_execution_interfaces
    original=rclpy.serialization.deserialize_message
    def stale_deserialize(data,cls):
        value=original(data,cls)
        if cls is SupportReference:
            setattr(value,field,'' if field=='stair_model_hash' else [])
        return value
    monkeypatch.setattr(rclpy.serialization,'deserialize_message',stale_deserialize)
    with pytest.raises(ValueError,match='Parametric stair model typesupport mismatch'):
        validate_execution_interfaces()


def test_old_support_interface_is_rejected_before_any_ros_context(monkeypatch):
    from types import SimpleNamespace
    import pytest
    from d1max_planning_interfaces import msg
    from d1max_pct_scan.interface_preflight import validate_execution_interfaces
    monkeypatch.setattr(msg,'SupportReference',lambda:SimpleNamespace(
        support_ground_xyz=[],body_reference_height_m=.55,verified=False,regions=[]))
    with pytest.raises(ValueError,match='Execution v3 interface missing: SupportReference'):
        validate_execution_interfaces()


@pytest.mark.parametrize('message,field', [
    ('MotionDemand','acknowledged_discontinuity_write'),
    ('LocalMotionCommand','acknowledged_discontinuity_write'),
    ('TaskCommandFeedback','discontinuity_write_sequence'),
    ('TaskCommandFeedback','sdk_forward'),
])
def test_sender_history_contract_rejects_stale_generated_support(monkeypatch,message,field):
    from d1max_planning_interfaces import msg
    import rclpy.serialization
    from d1max_pct_scan.interface_preflight import validate_execution_interfaces
    cls=getattr(msg,message)
    original=rclpy.serialization.deserialize_message
    def stale_deserialize(data,kind):
        value=original(data,kind)
        if kind is cls:
            setattr(value,field,0. if field=='sdk_forward' else 0)
        return value
    monkeypatch.setattr(rclpy.serialization,'deserialize_message',stale_deserialize)
    with pytest.raises(ValueError,match='Actual writer feedback typesupport mismatch'):
        validate_execution_interfaces()


@pytest.mark.parametrize('field', ['stair_contact_indices', 'stair_riser_xyz', 'stair_riser_owner'])
def test_nonzero_contact_indices_detect_silently_stale_typesupport(monkeypatch, field):
    from d1max_planning_interfaces.msg import SupportReference
    import rclpy.serialization
    from d1max_pct_scan.interface_preflight import validate_execution_interfaces
    original=rclpy.serialization.deserialize_message
    def stale_deserialize(data,cls):
        value=original(data,cls)
        if cls is SupportReference:
            setattr(value,field,[])
        return value
    monkeypatch.setattr(rclpy.serialization,'deserialize_message',stale_deserialize)
    with pytest.raises(ValueError,match='Literal stair contact typesupport mismatch'):
        validate_execution_interfaces()


def test_goal_execution_intent_defaults_to_preview_and_roundtrips_typed_authorization():
    from d1max_navigation_bt_interfaces.action import Navigate
    from rclpy.serialization import serialize_message, deserialize_message
    request=Navigate.Goal(schema_version=2)
    assert request.request_execution is False
    request.request_execution=True
    restored=deserialize_message(serialize_message(request),Navigate.Goal)
    assert restored==request and restored.request_execution is True


def test_native_heading_memory_roundtrips_without_relabeling_the_measured_join():
    from d1max_planning_interfaces.msg import TaggedBspline
    from rclpy.serialization import serialize_message, deserialize_message
    value=TaggedBspline(schema_version=2,body_heading_seed_valid=True,
        body_heading_seed_yaw=.73,body_heading_sampling_intervals=217)
    value.join_pose.orientation.w=1.
    restored=deserialize_message(serialize_message(value),TaggedBspline)
    assert restored==value
    assert restored.body_heading_seed_yaw==.73
    assert restored.body_heading_sampling_intervals==217
    assert restored.join_pose.orientation.w==1.


def test_original_mc_stop_metric_roundtrips_without_replacing_spatial_norm():
    import math
    from d1max_planning_interfaces.msg import StopReport, SDKStationaryEvidence
    from rclpy.serialization import serialize_message, deserialize_message
    for cls in (StopReport,SDKStationaryEvidence):
        value=cls(velocity_metric='planar_xy_with_vertical_guard_v1',
            measured_planar_mps=.0284,measured_vertical_mps=.0135,
            measured_linear_mps=math.hypot(.0284,.0135),measured_angular_radps=.005)
        assert value.measured_linear_mps>.03
        restored=deserialize_message(serialize_message(value),cls)
        assert restored==value and not restored.physical_acceptance_verified
