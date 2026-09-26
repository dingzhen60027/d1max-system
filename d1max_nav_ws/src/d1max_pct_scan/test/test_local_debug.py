import numpy as np
import pytest

from d1max_pct_scan.local_debug import (DEBUG_TIMEOUT, MAX_DEBUG_POINTS, DebugSnapshot,
                                      LocalDebugGate, local_debug_specs)
from d1max_pct_scan.live_scan_contract import ReferenceGate


def geometry():
    return dict(selected_reference=[[1., 2., 3.], [4., 5., 6.], [7., 8., 9.]],
                projection=[1.2, 2.3, 3.4], local_target=[7.1, 8.2, 9.3])


def test_debug_draws_actual_native_geometry_without_deriving_target():
    native = geometry()
    specs = {s['namespace']: s for s in local_debug_specs(**native)}
    assert specs['local_reference_segment']['points'] == native['selected_reference']
    assert specs['local_reference_anchors']['points'] == native['selected_reference']
    assert specs['local_target']['position'] == native['local_target']
    assert specs['local_reference_projection']['position'] == native['projection']
    assert specs['local_target']['position'] != native['selected_reference'][-1]
    assert specs['local_reference_segment']['color'][:3] == (1., .55, .08)
    assert specs['local_reference_segment']['width'] == .03
    assert specs['local_reference_anchors']['color'][2] == 1.
    assert specs['local_target']['color'][:3] == (1., .40, .02)
    assert all('TEXT' not in spec['kind'] for spec in specs.values())


@pytest.mark.parametrize('field,value', [
    ('selected_reference', []),
    ('selected_reference', [[1., 2., 3.]]*(MAX_DEBUG_POINTS+1)),
    ('selected_reference', [[1., 2.], [3., 4.]]),
    ('selected_reference', [[1., np.nan, 3.]]),
    ('projection', [np.inf, 0., 0.]),
    ('projection', [0., 0.]),
    ('local_target', [0., -np.inf, 0.]),
    ('local_target', [0., 0., 0., 0.]),
    ('selected_reference', [[0., 0., 10001.]]),
    ('local_target', [-10001., 0., 0.]),
    ('projection', [0., 1e100, 0.]),
])
def test_invalid_debug_geometry_is_not_displayable(field, value):
    args = geometry()
    args[field] = value
    with pytest.raises(ValueError):
        local_debug_specs(**args)


def test_one_native_anchor_allowed_and_not_replaced_by_invented_line():
    args = geometry()
    args['selected_reference'] = [[1., 2., 3.]]
    assert local_debug_specs(**args)[0]['points'] == [[1., 2., 3.]]


def reference_gate():
    return ReferenceGate(generation=3, ready=True, active=True, issued_at=100.)


def snapshot(**changes):
    values = dict(session_id='test', generation=3, plan_id=7, frame_id='d1max_loc_map',
                  stamp=100.2, valid=True, phase='accepted', progress_arc_m=1., target_arc_m=5.,
                  **geometry())
    values.update(changes)
    return DebugSnapshot(**values)


def receive(state, event=None, **changes):
    args = dict(expected_session='test', gate=reference_gate(), now=100.3, mono=10.,
                spline_id=7, spline_stamp=100.1)
    args.update(changes)
    return state.receive(event or snapshot(), **args)


def test_accepted_debug_requires_matching_valid_spline_not_just_native_success():
    state = LocalDebugGate()
    assert receive(state, spline_id=6) == 'pending'
    assert state.active is None and state.pending.plan_id == 7
    assert state.pair(spline_id=6, spline_stamp=100.1, now=100.4, mono=10.1) == 'pending'
    assert state.pair(spline_id=7, spline_stamp=100.1, now=100.4, mono=10.1) == 'draw'
    assert state.active.plan_id == 7 and state.pending is None


def test_success_after_valid_spline_draws_immediately_and_is_not_a_goal_inference():
    state = LocalDebugGate()
    assert receive(state) == 'draw'
    assert state.active.local_target == geometry()['local_target']


def test_fresh_debug_with_stale_spline_cannot_render():
    state = LocalDebugGate()
    assert receive(state, spline_stamp=98.) == 'pending'
    assert state.active is None


@pytest.mark.parametrize('changes', [dict(session_id='old'), dict(generation=2),
    dict(stamp=99.9), dict(stamp=98.), dict(stamp=100.5), dict(stamp=float('nan'))])
def test_foreign_stale_or_future_debug_cannot_replace_current_display(changes):
    state = LocalDebugGate()
    assert receive(state) == 'draw'
    current = state.active
    assert receive(state, snapshot(**changes)) == 'ignore'
    assert state.active is current


@pytest.mark.parametrize('change', [dict(frame_id='wrong'), dict(plan_id=-1), dict(valid=1),
    dict(phase='candidate'), dict(matching_path_headers=False), dict(progress_arc_m=-1.),
    dict(progress_arc_m=float('nan')), dict(target_arc_m=.5),
    dict(target_arc_m=10001.),
    dict(selected_reference=[]), dict(local_target=[0., float('nan'), 0.])])
def test_new_malformed_current_debug_clears_old_display(change):
    state = LocalDebugGate()
    assert receive(state) == 'draw'
    changed = dict(plan_id=8, stamp=100.25)
    changed.update(change)
    assert receive(state, snapshot(**changed)) == 'clear'
    assert state.active is state.pending is None and state.phase == 'invalid_debug'


@pytest.mark.parametrize('phase', ['reference_replaced', 'cancelled', 'failed', 'completed',
                                  'emergency_stop', 'debug_overflow', 'debug_invalid'])
def test_explicit_invalid_terminal_phase_clears_and_delayed_success_cannot_revive(phase):
    state = LocalDebugGate()
    assert receive(state) == 'draw'
    assert receive(state, snapshot(valid=False, phase=phase, stamp=100.25,
                                   selected_reference=[])) == 'clear'
    assert state.active is None and state.phase == phase
    assert receive(state) == 'ignore'
    # Even a malformed re-stamped repeat of the same accepted plan cannot revive.
    assert receive(state, snapshot(stamp=100.28)) == 'ignore'
    assert receive(state, snapshot(plan_id=8, stamp=100.29), spline_id=8) == 'draw'


def test_new_spline_never_leaves_previous_goal_debug_visible():
    state = LocalDebugGate()
    assert receive(state) == 'draw'
    assert state.pair(spline_id=8, spline_stamp=100.25, now=100.3, mono=10.1) == 'clear'
    assert state.active is None


def test_old_failure_cannot_erase_new_spline_before_its_debug_arrives():
    state = LocalDebugGate()
    assert receive(state) == 'draw'
    state.pair(spline_id=8, spline_stamp=100.25, now=100.3, mono=10.1)
    assert receive(state, snapshot(valid=False, phase='failed', stamp=100.28), spline_id=8) == 'ignore'
    assert receive(state, snapshot(plan_id=8, stamp=100.29), spline_id=8) == 'draw'


def test_old_failure_cannot_erase_newer_pending_native_debug():
    state = LocalDebugGate()
    assert receive(state, snapshot(plan_id=8), spline_id=7) == 'pending'
    assert receive(state, snapshot(valid=False, phase='failed', stamp=100.28), spline_id=7) == 'ignore'
    assert state.pending.plan_id == 8


def test_newest_pending_only_and_late_old_plan_cannot_reappear():
    state = LocalDebugGate()
    assert receive(state, snapshot(plan_id=8), spline_id=7) == 'pending'
    assert receive(state, snapshot(plan_id=9, stamp=100.25), spline_id=7) == 'pending'
    assert state.pending.plan_id == 9
    assert receive(state, snapshot(plan_id=8, stamp=100.27), spline_id=7) == 'ignore'
    assert state.pair(spline_id=8, spline_stamp=100.26, now=100.3, mono=10.1) == 'pending'
    assert state.active is None
    assert state.pair(spline_id=9, spline_stamp=100.28, now=100.3, mono=10.1) == 'draw'


@pytest.mark.parametrize('change', ['wall_expired', 'mono_expired', 'cancel', 'localization', 'epoch'])
def test_expiration_cancel_epoch_and_localization_loss_clear_current_and_pending(change):
    for paired in (False, True):
        state = LocalDebugGate()
        receive(state, spline_id=7 if paired else 6)
        gate, now, mono = reference_gate(), 100.4, 10.2
        if change == 'wall_expired': now += DEBUG_TIMEOUT
        if change == 'mono_expired': mono += DEBUG_TIMEOUT
        if change == 'cancel': gate.active = False
        if change == 'localization': gate.ready = False
        if change == 'epoch': gate.generation += 1
        assert state.expire(now=now, mono=mono, gate=gate)
        assert state.active is state.pending is None


def test_expired_pending_is_not_resurrected_by_late_spline():
    state = LocalDebugGate()
    receive(state, spline_id=6)
    assert state.pair(spline_id=7, spline_stamp=102.4, now=102.5, mono=12.2) == 'clear'
    assert state.active is None


def test_distinct_native_nanosecond_events_do_not_collapse_to_float_timestamp():
    state = LocalDebugGate()
    epoch_s = 1800000000.
    gate = reference_gate()
    gate.issued_at = epoch_s
    kwargs = dict(gate=gate, now=epoch_s+.1, spline_stamp=epoch_s)
    event = snapshot(stamp=epoch_s, stamp_ns=1800000000000000000)
    assert receive(state, event, **kwargs) == 'draw'
    failure = snapshot(stamp=epoch_s, stamp_ns=1800000000000000001,
                       valid=False, phase='failed', selected_reference=[])
    assert receive(state, failure, **kwargs) == 'clear'
    assert state.phase == 'failed' and state.active is None
    assert receive(state, event, **kwargs) == 'ignore'


@pytest.mark.parametrize('phase', [
    'failed_reference_geometry', 'failed_reference_search_budget',
    'failed_reference_target_occupied', 'failed_reference_search',
    'failed_reference_start_occupied', 'failed_reference_lattice_occupied',
    'failed_reference_outside_map',
    'failed_reference_search_collision', 'failed_rebound_search',
    'failed_optimization', 'failed_dynamics', 'failed_final_collision',
    'waiting_sensor_map', 'waiting_goal_reached', 'waiting_observed_space',
])
def test_native_failure_stage_is_preserved_without_fabricating_success(phase):
    state = LocalDebugGate()
    event = snapshot(valid=False, phase=phase, selected_reference=[])
    assert receive(state, event) == 'clear'
    assert state.phase == phase and state.active is None and state.pending is None


def attempt_marker(*, action=0, kind='reference', stamp_ns=100200000000):
    from types import SimpleNamespace as NS
    return NS(header=NS(frame_id='map', stamp=NS(sec=stamp_ns//10**9, nanosec=stamp_ns%10**9)),
              ns='local_attempt/session/3/'+kind, action=action, type=4, frame_locked=False,
              points=[NS(x=1., y=2., z=3.), NS(x=2., y=2., z=3.)] if action == 0 else [],
              pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)),
              color=NS(r=1., g=.85, b=0., a=.9), scale=NS(x=.025, y=.025, z=.025),
              lifetime=NS(sec=1, nanosec=900000000), mesh_resource='', text='')


def check_attempt(markers):
    from d1max_pct_scan.local_debug import attempt_marker_contract
    return attempt_marker_contract(markers, session_id='session', generation=3,
                                   frame_id='map', issued_at=100.1, now=100.3)


def test_attempt_is_only_bounded_actual_geometry_not_an_accepted_plan():
    markers = [attempt_marker(action=3), attempt_marker()]
    stamp, ttl, count = check_attempt(markers)
    assert stamp == 100200000000 and ttl == pytest.approx(1.9) and count == 1


@pytest.mark.parametrize('issue', ['session', 'generation', 'time', 'frame', 'oversize', 'green', 'infinite'])
def test_attempt_diagnostics_fail_closed(issue):
    marker = attempt_marker()
    if issue == 'session': marker.ns = 'local_attempt/other/3/reference'
    if issue == 'generation': marker.ns = 'local_attempt/session/2/reference'
    if issue == 'time': marker.header.stamp.sec = 98
    if issue == 'frame': marker.header.frame_id = 'odom'
    if issue == 'oversize': marker.points *= 4096
    if issue == 'green': marker.color.r, marker.color.g = .12, .95
    if issue == 'infinite': marker.points[0].z = float('inf')
    with pytest.raises(ValueError):
        check_attempt([marker])
