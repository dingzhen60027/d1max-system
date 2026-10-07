import copy
import math

import pytest

from stall_recovery import MIN_FORWARD_MPS, StallRecovery, enabled_for


DT_NS = 2_000_000
IDENTITY = dict(epoch='physics-epoch', session_id='fixture-session',
                clock_anchor_ns=1_000_000_000_000)
AUTOMATIC_CONTEXT = object()


def actual_context(tick, *, counter_offset=1000, native_offset=1000):
    return dict(IDENTITY, phase='source', acquisition_phase='actual_pre_policy_PhysX_BEGIN',
                source_tick=tick, source_ns=tick * DT_NS,
                native_physics_tick=tick + native_offset, policy_counter=tick + counter_offset)


def observe(gate, tick, command=(.1, 0.), linear=(0., 0., .02), angular=(.04, 0., 0.),
            *, counter_offset=1000, native_offset=1000, tick_context=AUTOMATIC_CONTEXT):
    if tick_context is AUTOMATIC_CONTEXT:
        tick_context = actual_context(tick, counter_offset=counter_offset, native_offset=native_offset)
    return gate.observe(tick * DT_NS, command, linear, angular, tick + counter_offset,
                        tick_context=tick_context)


def samples(gate, first, count, **kwargs):
    return [observe(gate, tick, **kwargs) for tick in range(first, first + count)]


def test_default_off_and_explicit_isolated_spot_only():
    assert not enabled_for({'kind': 'official_spot_physx'})
    cfg = dict(schema=1, enabled=True, isolated_fixture=True)
    assert enabled_for(dict(kind='official_spot_physx', stall_recovery=cfg))
    for robot in [dict(kind='wheel', stall_recovery=cfg),
                  dict(kind='official_spot_physx', stall_recovery=dict(cfg, isolated_fixture=False)),
                  dict(kind='official_spot_physx', stall_recovery=dict(cfg, enabled=1))]:
        with pytest.raises(ValueError):
            enabled_for(robot)


def test_exact_forward_threshold_and_static_bounds_preserve_original_command_and_context():
    assert MIN_FORWARD_MPS == .05
    gate = StallRecovery()
    command, linear, angular = [.05, -.02], [.03, 0., 0.], [0., .05, 0.]
    before = copy.deepcopy((command, linear, angular))
    assert not any(samples(gate, 0, 500, command=command, linear=linear, angular=angular))
    context = actual_context(500)
    context_before = copy.deepcopy(context)
    event = observe(gate, 500, command, linear, angular, tick_context=context)
    assert event['qualification_min_forward_mps'] == .05
    assert event['command'] == command
    assert event['continuous_static_duration_ns'] == 1_000_000_000
    assert event['full_linear_norm'] == .03 and event['full_angular_norm'] == .05
    assert event['source_identity'] == IDENTITY
    assert event['source_tick'] == 500 and event['native_physics_tick'] == 1500
    assert event['source_ns'] == 1_000_000_000 and event['policy_counter'] == 1500
    assert (command, linear, angular) == before and context == context_before


@pytest.mark.parametrize('forward', [math.nextafter(.05, 0.), .001, 1e-12, 0., -.05])
def test_below_threshold_tiny_goal_zero_and_reverse_never_qualify(forward):
    gate = StallRecovery()
    command = [forward, .02]
    assert not any(samples(gate, 0, 600, command=command))
    assert command == [forward, .02]
    # Low demand cannot contribute time to a later qualified forward episode.
    assert not any(samples(gate, 600, 500, command=[.05, 0.]))
    assert observe(gate, 1100, [.05, 0.])['continuous_static_duration_ns'] == 1_000_000_000


@pytest.mark.parametrize('forward', [math.nextafter(.05, 0.), 0., -.1])
def test_low_demand_clears_an_accumulated_unconsumed_window(forward):
    gate = StallRecovery()
    assert not any(samples(gate, 0, 500))
    assert observe(gate, 500, [forward, 0.]) is None
    assert not any(samples(gate, 501, 509))
    event = observe(gate, 1010)
    assert event['episode'] == 2
    assert event['stationary_start_ns'] == 1_002_000_000
    assert event['continuous_static_duration_ns'] == 1_018_000_000


def test_real_counter_boundary_one_per_episode_and_zero_preserved():
    gate = StallRecovery()
    assert not any(samples(gate, 0, 500))
    assert observe(gate, 500)['episode'] == 1
    assert not any(samples(gate, 501, 600))
    assert observe(gate, 1101, [0., 0.]) is None
    assert not any(samples(gate, 1102, 508))
    assert observe(gate, 1610)['episode'] == 2


@pytest.mark.parametrize('linear,angular', [([0., 0., .031], [0., 0., 0.]),
    ([0., 0., 0.], [.051, 0., 0.]), ([.03, .03, 0.], [0., 0., 0.]),
    ([0., 0., 0.], [.04, .04, 0.])])
def test_complete_xyz_and_angular_norms(linear, angular):
    assert not any(samples(StallRecovery(), 0, 600, linear=linear, angular=angular))


def test_actual_motion_interrupts_the_continuous_static_window():
    gate = StallRecovery()
    assert not any(samples(gate, 0, 250))
    assert observe(gate, 250, linear=[0., 0., .031]) is None
    assert not any(samples(gate, 251, 509))
    event = observe(gate, 760)
    assert event['stationary_start_ns'] == 502_000_000
    assert event['continuous_static_duration_ns'] == 1_018_000_000
    assert event['episode'] == 1


def test_legal_native_gap_requires_a_new_full_source_window():
    gate = StallRecovery()
    assert not any(samples(gate, 0, 500))
    # Tick500 is missing. Source, native tick and actual counter all advance by two.
    assert observe(gate, 501) is None
    assert not any(samples(gate, 502, 508))
    event = observe(gate, 1010)
    assert event['stationary_start_ns'] == 1_002_000_000
    assert event['continuous_static_duration_ns'] == 1_018_000_000
    assert event['policy_counter'] == 2010


def test_legal_gap_cannot_rearm_an_already_consumed_episode():
    gate = StallRecovery()
    assert not any(samples(gate, 0, 500))
    assert observe(gate, 500)
    assert not any(samples(gate, 502, 608))
    assert observe(gate, 1110, [math.nextafter(.05, 0.), 0.]) is None
    assert not any(samples(gate, 1111, 509))
    assert observe(gate, 1620)['episode'] == 2


def test_waits_for_next_existing_inference_without_changing_counter():
    gate = StallRecovery()
    # This is a fixed startup offset, not an invented boundary counter.
    assert not any(samples(gate, 0, 500, counter_offset=1001))
    assert observe(gate, 500, counter_offset=1001) is None
    assert not any(samples(gate, 501, 8, counter_offset=1001))
    event = observe(gate, 509, counter_offset=1001)
    assert event['policy_counter'] == 1510
    assert event['continuous_static_duration_ns'] == 1_018_000_000
    assert event['source_tick'] == 509 and event['native_physics_tick'] == 1509


def test_first_actual_context_binds_identity_and_independent_native_counter_offsets():
    gate = StallRecovery()
    identity = dict(epoch='another-physics-epoch', session_id='another-session', clock_anchor_ns=42)
    event = None
    # The gate can begin later in an already-running source epoch.
    for tick in range(100, 610):
        context = actual_context(tick, counter_offset=41, native_offset=730_000)
        context.update(identity)
        event = observe(gate, tick, counter_offset=41, native_offset=730_000, tick_context=context)
        if tick < 609:
            assert event is None
    assert event['source_identity'] == identity
    assert event['policy_counter'] == 650
    assert event['source_tick'] == 609 and event['native_physics_tick'] == 730_609


@pytest.mark.parametrize('bad', ['absent', 'not_object', 'missing_phase', 'missing_acquisition', 'bootstrap', 'end',
    'missing_tick', 'negative_tick', 'bool_tick', 'float_tick', 'missing_source', 'source_mismatch',
    'tick_scale_mismatch',
    'missing_native', 'negative_native', 'bool_native', 'missing_counter', 'counter_mismatch',
    'bool_counter', 'missing_epoch', 'empty_epoch', 'wrong_epoch_type', 'missing_session',
    'empty_session', 'missing_anchor', 'zero_anchor', 'bool_anchor'])
def test_invalid_actual_context_permanently_faults_even_before_first_binding(bad):
    gate = StallRecovery()
    context = actual_context(0)
    if bad == 'absent': context = None
    elif bad == 'not_object': context = []
    elif bad.startswith('missing_'):
        field = {'phase': 'phase', 'acquisition': 'acquisition_phase', 'tick': 'source_tick', 'source': 'source_ns',
                 'native': 'native_physics_tick', 'counter': 'policy_counter', 'epoch': 'epoch',
                 'session': 'session_id', 'anchor': 'clock_anchor_ns'}[bad.removeprefix('missing_')]
        del context[field]
    else:
        field, value = {
            'bootstrap': ('phase', 'bootstrap'), 'end': ('acquisition_phase', 'actual_final_PhysX_END'),
            'negative_tick': ('source_tick', -1), 'bool_tick': ('source_tick', False),
            'float_tick': ('source_tick', 0.), 'source_mismatch': ('source_ns', 1),
            'tick_scale_mismatch': ('source_tick', 1),
            'negative_native': ('native_physics_tick', -1), 'bool_native': ('native_physics_tick', True),
            'counter_mismatch': ('policy_counter', 1001), 'bool_counter': ('policy_counter', True),
            'empty_epoch': ('epoch', ''), 'wrong_epoch_type': ('epoch', 3),
            'empty_session': ('session_id', ''), 'zero_anchor': ('clock_anchor_ns', 0),
            'bool_anchor': ('clock_anchor_ns', True)}[bad]
        context[field] = value
    with pytest.raises(ValueError):
        observe(gate, 0, tick_context=context)
    with pytest.raises(ValueError):
        observe(gate, 1)
    with pytest.raises(ValueError):
        observe(gate, 500, [0., 0.])


@pytest.mark.parametrize('bad', ['epoch_drift', 'session_drift', 'anchor_drift',
    'native_offset_drift', 'counter_offset_drift', 'native_rollback', 'counter_rollback',
    'context_counter_disagrees', 'source_rollback', 'gap_with_wrong_native', 'gap_with_wrong_counter'])
def test_bound_source_identity_and_counter_faults_are_latched(bad):
    gate = StallRecovery()
    assert observe(gate, 0) is None
    tick = 2 if bad.startswith('gap_') else 1
    context = actual_context(tick)
    counter_offset = 1000
    if bad == 'epoch_drift': context['epoch'] = 'other-epoch'
    elif bad == 'session_drift': context['session_id'] = 'other-session'
    elif bad == 'anchor_drift': context['clock_anchor_ns'] += 1
    elif bad == 'native_offset_drift': context['native_physics_tick'] += 1
    elif bad == 'counter_offset_drift':
        context['policy_counter'] += 1
        counter_offset += 1
    elif bad == 'native_rollback': context['native_physics_tick'] = 999
    elif bad == 'counter_rollback':
        context['policy_counter'] = 999
        counter_offset = 998
    elif bad == 'context_counter_disagrees': context['policy_counter'] += 1
    elif bad == 'source_rollback':
        tick = 0
        context = actual_context(tick)
    elif bad == 'gap_with_wrong_native': context['native_physics_tick'] -= 1
    elif bad == 'gap_with_wrong_counter':
        context['policy_counter'] -= 1
        counter_offset -= 1
    with pytest.raises(ValueError):
        observe(gate, tick, counter_offset=counter_offset, tick_context=context)
    with pytest.raises(ValueError):
        observe(gate, 3)
    with pytest.raises(ValueError):
        observe(gate, 503, [0., 0.])


@pytest.mark.parametrize('linear,angular', [([math.nan, 0., 0.], [0., 0., 0.]),
    ([0., 0., 0.], [0., math.inf, 0.])])
def test_nonfinite_measurements_are_rejected(linear, angular):
    with pytest.raises(ValueError):
        observe(StallRecovery(), 0, linear=linear, angular=angular)
