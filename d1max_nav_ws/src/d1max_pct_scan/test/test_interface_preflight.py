from d1max_pct_scan.interface_preflight import validate_interfaces


def test_actual_generated_cpp_python_contract_roundtrips_without_ros_context():
    result=validate_interfaces()
    assert result['schema_version'] == 2
    assert result['message_types_verified'] == 13
    assert result['local_state_contract']=='continuous_odom_v1'
    assert result['ros_context_started'] is False


def test_actual_writer_cas_interfaces_roundtrip_without_starting_ros():
    from d1max_pct_scan.interface_preflight import validate_execution_interfaces
    result=validate_execution_interfaces()
    assert result['execution_schema_version']==3
    assert result['handoff_contract']=='writer_applied_cas_v3'
    assert result['execution_types_verified']==22
    assert result['initial_pose_transaction']=='bt_initial_pose_transaction_v1'
    assert result['route_progress_contract']=='source_bound_full_route_arc_v1'
