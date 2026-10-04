"""Read-only safety-scan provenance and unresolved execution architecture.

The current motion producer is navigation_cloud_to_scan: an endpoint projection
of Faster-LIO's already filtered cloud.  Raw per-sensor metadata exists in the
preview branch but is NOT consumed by this safety producer.  Changing a YAML
topic, adding verification booleans, or receiving a scan with many finite bins
cannot implement the missing acquisition-time/source-health contract.

No ROS imports, SDK calls, calibration flags, or motion authority live here.
"""
import math


PREFIX = '/d1max/live_planning/'
FILTERED_SOURCE = '/d1max/localization/lio/deskewed'


def safety_input_blockers(session=None):
    """Implementation facts, not operator-overridable acceptance flags.

    Future implementation must provide independently time-bound front/rear ray
    origins/endpoints and explicit unknown/no-return semantics before removing
    these blockers.  Keep physical envelope/time/extrinsic acceptance separate.
    """
    return (
        'safety_scan_uses_lio_filtered_endpoints_not_validated_raw_rays',
        'safety_scan_missing_per_sensor_acquisition_and_health_contract',
        'safety_scan_unknown_coverage_not_bound_to_execution_swept_volume',
    )


def inspect_safety_input(parameters, *, body_frame='d1max_loc_base_link',
                         odom_frame='d1max_loc_odom',
                         safety_scan_topic=PREFIX+'safety_scan'):
    """Inspect effective motion node parameters without asserting deployment.

    Input is motion_stack.describe_parameters() or a parsed motion.yaml. The
    caller must separately verify installed producer/consumer binaries. Missing
    or drifted nodes are reported rather than silently filled with defaults.
    """
    issues = []

    def node(name):
        value = parameters.get(PREFIX+name) if isinstance(parameters, dict) else None
        result = value.get('ros__parameters') if isinstance(value, dict) else None
        if not isinstance(result, dict):
            issues.append('missing_effective_node:'+name)
            return {}
        return result

    projection = node('navigation_cloud_to_scan')
    gate = node('navigation_command_gate')
    monitor = node('collision_monitor')
    checks = (
        ('projection_output', projection.get('output_topic'), safety_scan_topic),
        ('projection_frame', projection.get('target_frame'), body_frame),
        ('gate_scan', gate.get('scan_topic'), safety_scan_topic),
        ('gate_body', gate.get('base_frame'), body_frame),
        ('gate_odom', gate.get('odom_frame'), odom_frame),
        ('monitor_body', monitor.get('base_frame_id'), body_frame),
        ('monitor_odom', monitor.get('odom_frame_id'), odom_frame),
        ('monitor_scan', monitor.get('scan', {}).get('topic')
         if isinstance(monitor.get('scan'), dict) else None, safety_scan_topic),
    )
    for label, actual, expected in checks:
        if actual != expected:
            issues.append('safety_input_wiring_mismatch:'+label)
    source = projection.get('input_topic')
    if source != FILTERED_SOURCE:
        issues.append('unverified_safety_endpoint_source')
    timeout = projection.get('max_cloud_age')
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= .5:
        issues.append('invalid_safety_cloud_source_age_bound')
    return dict(
        schema=1, producer='d1max_navigation.navigation_cloud_to_scan',
        input_topic=source, output_topic=projection.get('output_topic'),
        source_kind='lio_filtered_endpoints' if source == FILTERED_SOURCE else 'unverified_endpoints',
        target_frame=projection.get('target_frame'), source_age_limit_s=timeout,
        acquisition_contract=dict(per_sensor_ids=False, per_point_times=False,
            ray_origins=False, dual_source_health=False, fixed_clock_mapping_proof=False),
        no_return_semantics='unknown_nan_not_clear',
        usable_endpoint_scan_is_not_full_coverage=True,
        coverage_verified=False, motion_architecture_ready=False,
        blockers=list(safety_input_blockers())+issues)
