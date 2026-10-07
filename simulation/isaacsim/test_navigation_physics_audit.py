import copy
import hashlib
import json
import math

import pytest

from navigation_physics_audit import audit_navigation_physics, audit_navigation_physics_files


def encoded(value):
    return json.dumps(value, separators=(',', ':')).encode()


@pytest.fixture
def evidence():
    stationary = dict(profile='general_low_speed', linear_threshold_mps=.03,
        angular_threshold_radps=.05, stationary_duration_s=1., reentry_duration_s=.6,
        minimum_new_samples=3, mc_expected_hz=50., mc_min_hz=40., mc_max_hz=60.,
        measured_static_linear_bound_mps=.01, measured_static_angular_bound_radps=.02)
    scene = dict(robot=dict(kind='official_spot_physx', record_full_physics_history=True,
        max_linear_speed=.25, max_angular_speed=.3, policy_input_limits=dict(linear=.4, angular=.5)))
    model = dict(schema_version=3, model='reaction_braking_reachable_v1',
        session_id='fixture', transport_mode='isolated_mock', fixture_only=True,
        stationary_evidence=stationary,
        execution_timing=dict(sensor_source_age_bound_s=.5, command_pipeline_bound_s=.1,
            writer_period_s=.05, source_time_uncertainty_s=.02),
        measurements=dict(max_speed_mps=.6, max_yaw_radps=.8, reaction_bound_s=.67,
            stopping_distance_m=.5, stopping_yaw_rad=.4, stop_latency_bound_s=3.,
            tracking_error_bound_m=.1, heading_error_bound_rad=.1),
        isolated_platform_model=dict(schema=1, kind='official_spot_physx',
            source_scope='isolated_simulation_physx_measured_model',
            command_max_speed_mps=.25, command_max_yaw_radps=.3,
            reachable_max_speed_mps=.6, reachable_max_yaw_radps=.8),
        isolated_full_xyz_reference_model=dict(schema=1, kind='official_spot_physx',
            source_scope='isolated_simulation_physx_measured_model', reference_max_speed_mps=.6,
            measured_travel_max_speed_mps=.6, observed_max_full_xyz_speed_mps=.5759470588816599,
            evidence_sha256='a'*64))
    session = dict(id='fixture', transport_mode='isolated_mock', physical_acceptance=False,
        simulation_backend='isaacsim_physx', simulation_clock='isaac_fixed_anchor_v1',
        max_speed_mps=.25, max_yaw_radps=.3, stationary_evidence=stationary,
        execution_braking_model_record='/sealed/model.json', input_hashes={},
        isaac_bridge_contract=dict(clock_anchor_ns=1_000_000_000_000,
            source_clock='isaac_physics_time', scene_config='/sealed/scene.json'))
    rows = []
    for i in range(701):
        # Motion ends at tick99. Zero consumption starts at exactly200ms;
        # measured settling ends at300ms and confirms at1300ms.
        velocity = [.2, 0., 0.] if i < 150 else [0., 0., .01]
        rows.append(dict(schema=1, kind='actual_full_body_component_measurement',
            phase='source', source_tick=i, source_ns=i*2_000_000,
            native_physics_tick=1000+i, policy_counter=1000+i,
            acquisition_phase='actual_final_PhysX_END' if i == 700 else 'actual_pre_policy_PhysX_BEGIN',
            session_id='fixture', epoch='epoch', clock_anchor_ns=1_000_000_000_000,
            mode='navigation_wire', authority=dict(policy_input_override=None, replay_original_authority=None),
            highlevel_command=[.25, 0.] if i < 100 else [0., 0.],
            official_policy_input=[.4, 0., 0.] if i < 100 else [0., 0., 0.],
            position=[min(i, 150)*.0004, 0., .481+i*.000001],
            quaternion_wxyz=[1., 0., 0., 0.], linear_velocity_world=velocity,
            angular_velocity_world=[0., 0., 0.], full_linear_speed=math.hypot(*velocity), full_angular_speed=0.))
    manifest = dict(schema=1, kind='actual_500hz_pre_policy_history', physics_hz=500,
        policy_hz=50, source_tick_ns=2_000_000, command_limits=[.25, .3],
        actual_source_ticks=700, duration_ns=1_400_000_000, first_source_tick=0, last_source_tick=699,
        consecutive_tick_invocations_checked=True, component_measurement_count=701,
        component_last_source_ns=1_400_000_000, component_measurements_file='policy_component_state_500hz.jsonl',
        observation_count=170)
    return dict(session=session, scene=scene, model=model, manifest=manifest, rows=rows)


def seal(evidence):
    session, manifest = evidence['session'], evidence['manifest']
    scene_bytes, model_bytes = encoded(evidence['scene']), encoded(evidence['model'])
    raw_bytes = b'\n'.join(encoded(row) for row in evidence['rows'])+b'\n'
    scene_sha, model_sha = hashlib.sha256(scene_bytes).hexdigest(), hashlib.sha256(model_bytes).hexdigest()
    session['execution_braking_model_sha256'] = model_sha
    session['isaac_bridge_contract']['scene_sha256'] = scene_sha
    session['input_hashes'][session['execution_braking_model_record']] = model_sha
    session['input_hashes'][session['isaac_bridge_contract']['scene_config']] = scene_sha
    manifest['scene_sha256'] = scene_sha
    manifest['component_measurements_sha256'] = hashlib.sha256(raw_bytes).hexdigest()
    return session, scene_bytes, model_bytes, manifest, raw_bytes


def audit(evidence, **kwargs):
    return audit_navigation_physics(*seal(evidence), expected_epoch='epoch', **kwargs)


def test_complete_native_history_and_exact_original_stop_origin(evidence):
    before = copy.deepcopy(evidence['rows'])
    result = audit(evidence)
    assert result['verified'], result['errors']
    assert result['sample_count'] == 701 and result['complete_native_500hz']
    assert result['final_stop']['confirmation_latency_s'] == 1.1
    assert result['final_stop']['zero_consumption_source_ns'] == 1_000_200_000_000
    assert result['final_stop']['confirmation_source_ns'] == 1_001_300_000_000
    assert evidence['rows'] == before
    assert not result['hardware_acceptance'] and not result['continuous_collision_claim']


@pytest.mark.parametrize('axis', ['linear', 'angular'])
def test_early_stationary_window_does_not_hide_later_motion_at_zero_demand(evidence, axis):
    row = evidence['rows'][675]
    if axis == 'linear':
        row['linear_velocity_world'] = [.08, 0., 0.]
        row['full_linear_speed'] = .08
    else:
        row['angular_velocity_world'] = [0., 0., .06]
        row['full_angular_speed'] = .06
    result = audit(evidence)
    assert not result['verified']
    stop = result['final_stop']
    assert stop['initial_stationary_window_verified']
    assert stop['confirmation_latency_s'] == 1.1
    assert not stop['maintained_stationarity_to_end']
    assert stop['reason'] == 'recorded_stop_then_zero_tail_motion'
    assert stop['post_confirmation_motion_samples'] == 1
    assert stop['first_post_confirmation_motion_source_ns'] == 1_001_350_000_000
    assert row['highlevel_command'] == [0., 0.]


@pytest.mark.parametrize('bad', ['missing_interior', 'source_gap', 'native_counter', 'policy_counter',
    'epoch', 'end_identity', 'end_phase', 'nan', 'input_override', 'mode', 'derived_speed', 'zero_quaternion'])
def test_original_raw_invalid_measurement_is_not_reconstructed(evidence, bad):
    row = evidence['rows'][200]
    if bad == 'missing_interior':
        del evidence['rows'][200]
    elif bad == 'source_gap': row['source_ns'] += 1
    elif bad == 'native_counter': row['native_physics_tick'] += 1
    elif bad == 'policy_counter': row['policy_counter'] += 1
    elif bad == 'epoch': row['epoch'] = 'other'
    elif bad == 'end_identity': evidence['rows'][-1]['session_id'] = 'other'
    elif bad == 'end_phase': evidence['rows'][-1]['acquisition_phase'] = 'actual_pre_policy_PhysX_BEGIN'
    elif bad == 'nan': row['linear_velocity_world'][2] = math.nan
    elif bad == 'input_override': row['authority']['policy_input_override'] = [0., 0., 0.]
    elif bad == 'mode': row['mode'] = 'isolated_policy_calibration'
    elif bad == 'derived_speed': row['full_linear_speed'] = 0.
    elif bad == 'zero_quaternion': row['quaternion_wxyz'] = [0., 0., 0., 0.]
    result = audit(evidence)
    assert not result['verified'] and result['errors']


@pytest.mark.parametrize('bad', ['raw', 'model', 'scene', 'input_hash', 'live', 'marker', 'disabled', 'string_bool'])
def test_strict_seal_and_opt_in(evidence, bad):
    if bad == 'live': evidence['session']['transport_mode'] = 'live'
    elif bad == 'marker': del evidence['model']['isolated_full_xyz_reference_model']
    elif bad == 'disabled': evidence['scene']['robot']['record_full_physics_history'] = False
    elif bad == 'string_bool': evidence['scene']['robot']['record_full_physics_history'] = 'true'
    args = list(seal(evidence))
    if bad == 'raw': args[-1] += b'\n'
    elif bad == 'model': args[2] += b' '
    elif bad == 'scene': args[1] += b' '
    elif bad == 'input_hash': args[0]['input_hashes']['/sealed/model.json'] = 'b'*64
    result = audit_navigation_physics(*args, expected_epoch='epoch')
    assert not result['verified'] and result['errors']


def test_v39_raw_full_xyz_peak_rejects_point_six_without_clamping(evidence):
    row = evidence['rows'][90]
    row['linear_velocity_world'] = [.609435, 0., 0.]
    row['full_linear_speed'] = .609435
    result = audit(evidence)
    assert not result['verified']
    assert result['all_epoch']['max_full_xyz_speed_mps'] == .609435
    assert len(result['all_epoch']['exceeded_speed_domains']) == 3
    assert row['linear_velocity_world'] == [.609435, 0., 0.]


def test_full_xyz_and_yaw_are_separate_from_angular_diagnostic(evidence):
    row = evidence['rows'][90]
    row['linear_velocity_world'] = [.2, 0., .59]
    row['full_linear_speed'] = math.hypot(.2, 0., .59)
    row['angular_velocity_world'] = [1.1, 0., .1]
    row['full_angular_speed'] = math.hypot(1.1, 0., .1)
    result = audit(evidence)
    assert not result['verified'] and result['all_epoch']['exceeded_speed_domains']
    assert result['all_epoch']['max_full_angular_speed_radps'] > .8
    assert not result['all_epoch']['yaw_domain_exceeded']


def test_end_is_actual_measurement_but_not_new_command_consumption(evidence):
    evidence['rows'][-1]['highlevel_command'] = [.25, 0.]
    assert audit(evidence)['verified']


def test_zero_window_requires_full_duration_not_sample_count(evidence):
    # Last source1398ms; first eligible400ms =>998ms, not one second.
    for row in evidence['rows'][150:200]:
        row['linear_velocity_world'] = [.031, 0., 0.]
        row['full_linear_speed'] = .031
    evidence['rows'] = evidence['rows'][:-1]
    evidence['rows'][-1]['acquisition_phase'] = 'actual_final_PhysX_END'
    evidence['manifest'].update(actual_source_ticks=699, duration_ns=1_398_000_000,
        last_source_tick=698, component_measurement_count=700, component_last_source_ns=1_398_000_000)
    result = audit(evidence)
    assert not result['verified']
    assert result['final_stop']['reason'] == 'final_zero_tail_has_no_complete_stationary_window'


def test_action_uses_original_anchored_source_and_preserves_epoch_peak(evidence):
    row = evidence['rows'][90]
    row['linear_velocity_world'] = [.609435, 0., 0.]
    row['full_linear_speed'] = .609435
    result = audit(evidence, action_interval_ns=(1_000_400_000_000, 1_000_800_000_000))
    assert not result['verified']
    assert not result['action_interval']['exceeded_speed_domains']
    assert result['action_interval']['sample_count'] == 201


def test_files_wrapper_missing_history_rejects_instead_of_using_ros(tmp_path):
    path = tmp_path/'session.json'
    path.write_text('{}')
    assert not audit_navigation_physics_files(path)['verified']


def test_no_consumed_nonzero_is_not_a_stop_transition_proof(evidence):
    for row in evidence['rows']:
        row['highlevel_command'] = [0., 0.]
    result = audit(evidence)
    assert not result['verified'] and not result['final_stop']['transition_exercised']


def test_world_yaw_bound_is_checked_independently(evidence):
    row = evidence['rows'][50]
    row['angular_velocity_world'] = [0., 0., .800001]
    row['full_angular_speed'] = .800001
    result = audit(evidence)
    assert not result['verified'] and result['all_epoch']['yaw_domain_exceeded']
    assert not result['all_epoch']['exceeded_speed_domains']


def test_stop_confirmation_three_point_zero_zero_two_is_not_three_seconds(evidence):
    template = copy.deepcopy(evidence['rows'][-1])
    for i in range(701, 1702):
        row = copy.deepcopy(template)
        row.update(source_tick=i, source_ns=i*2_000_000,
            native_physics_tick=1000+i, policy_counter=1000+i)
        evidence['rows'].append(row)
    for i, row in enumerate(evidence['rows']):
        row['acquisition_phase'] = 'actual_final_PhysX_END' if i == 1701 else 'actual_pre_policy_PhysX_BEGIN'
        if 150 <= i < 1101:
            row['linear_velocity_world'] = [.031, 0., 0.]
            row['full_linear_speed'] = .031
    evidence['manifest'].update(actual_source_ticks=1701, duration_ns=3_402_000_000,
        last_source_tick=1700, component_measurement_count=1702, component_last_source_ns=3_402_000_000)
    result = audit(evidence)
    assert not result['verified']
    assert result['final_stop']['confirmation_latency_s'] == 3.002
    assert result['final_stop']['reason'] == 'recorded_stop_outside_original_bounds'


def test_stop_distance_uses_extra_actual_xy_path_not_velocity_clipping(evidence):
    evidence['rows'][300]['position'][0] += .3
    result = audit(evidence)
    assert not result['verified']
    assert result['final_stop']['extra_xy_path_m'] > .5


def test_exact_float32_policy_cap_representation_is_not_spurious_rejection(evidence):
    import struct
    evidence['rows'][50]['official_policy_input'][0] = struct.unpack('f', struct.pack('f', .4))[0]
    assert audit(evidence)['verified']
