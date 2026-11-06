"""Production reference workflow consumes the native original evidence budget.

It never renews GridMap evidence, body freshness or a motion permission.
"""
from copy import deepcopy

import pytest

from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from test_continuous_reference_node import harness,admit
from test_continuous_reference_transport import state,envelope
from d1max_pct_scan.continuous_reference_transport import nanoseconds,set_stamp


def configured(age=.5):
    prior,clock,output=harness()
    callback=ReferenceCallbacks(dict(prior.p,map_evidence_max_age_s=age),
        lambda k,v:output.append((k,deepcopy(v))),lambda:clock.ns,lambda:clock.mono)
    callback.on_navigation(state());callback.on_context_ack(callback.context)
    clock.ns+=1_000_000_000;clock.mono+=1.
    return callback,clock


def evidence(callback,clock,age_ns):
    source=clock.ns-age_ns
    return dict(callback.context,valid=True,source_stamp_ns=source,
        sources=[dict(sensor_id=i,integrated_stamp_ns=source) for i in (0,1)])


def test_proven_native_window_is_not_confused_with_body_or_receipt_freshness():
    # Exact measured bag timing: rear age + production fusion + consumer phase.
    original_age=246_600_367+106_062_613+50_000_000
    legacy,clock,_=harness()
    clock.ns+=500_000_000;clock.mono+=.5
    original=evidence(legacy,clock,original_age)
    legacy.on_map(original)
    assert not legacy.map_fresh()
    callback,new_clock=configured()
    new_clock.ns,new_clock.mono=clock.ns,clock.mono
    callback.on_map(original)
    assert callback.map_fresh()
    assert callback.map_report==original
    assert callback.transport.freshness==.4


def test_a_new_receipt_cannot_extend_original_half_second_sensor_deadline():
    callback,clock=configured()
    original=evidence(callback,clock,500_000_000)
    assert callback.on_map(original) and callback.map_fresh()
    clock.ns+=1
    assert callback.on_map(original)
    assert not callback.map_fresh()
    assert callback.map_report['source_stamp_ns']==original['source_stamp_ns']


@pytest.mark.parametrize('failure',['native_invalid','one_old_sensor','duplicate_sensor','boolean_sensor','false_minimum','receipt_stale'])
def test_observed_native_rejections_and_provenance_are_preserved(failure):
    callback,clock=configured()
    original=evidence(callback,clock,402_662_980)
    if failure=='native_invalid':original['valid']=False
    elif failure=='one_old_sensor':
        original['sources'][1]['integrated_stamp_ns']=clock.ns-500_000_001
        original['source_stamp_ns']=original['sources'][1]['integrated_stamp_ns']
    elif failure=='duplicate_sensor':original['sources'][1]['sensor_id']=0
    elif failure=='boolean_sensor':original['sources'][1]['sensor_id']=True
    elif failure=='false_minimum':original['source_stamp_ns']=clock.ns
    callback.on_map(original)
    if failure=='receipt_stale':clock.mono+=.400001
    assert not callback.map_fresh()


@pytest.mark.parametrize('age',[0.,-.1,.500000001,float('nan'),float('inf')])
def test_reference_cannot_configure_more_than_native_evidence_contract(age):
    with pytest.raises(ValueError,match='map_evidence_age_exceeds_native_contract'):
        configured(age)


def strict_callback():
    prior,clock,output=harness()
    callback=ReferenceCallbacks(dict(prior.p,map_evidence_max_age_s=.5,freshness_s=.3),
        lambda k,v:output.append((k,deepcopy(v))),lambda:clock.ns,lambda:clock.mono)
    callback.on_navigation(state());callback.on_context_ack(callback.context)
    callback.on_map(evidence(callback,clock,0))
    return callback,clock


def test_stricter_receipt_budget_does_not_replace_raw_map_source_budget():
    callback,clock=strict_callback()
    clock.ns+=400_000_000;clock.mono+=.4
    assert not callback.map_fresh()  # Receipt, not the still-valid raw source.
    raw=evidence(callback,clock,400_000_000)
    assert callback.on_map(raw) and callback.map_fresh()
    assert callback.map_report==raw and callback.map_evidence_age_ns==500_000_000
    assert callback.transport.freshness==.3


def test_stricter_body_delivery_budget_is_bound_to_original_proposal():
    from d1max_planning_interfaces.msg import ReferenceReceipt
    callback,clock=strict_callback();callback.on_route(envelope())
    proposal=deepcopy(callback.proposal)
    assert nanoseconds(proposal.valid_until)-nanoseconds(proposal.source_stamp)==300_000_000
    receipt=ReferenceReceipt(version=proposal.version,proposal_id=proposal.proposal_id,
        expected_version=proposal.expected_version,expected_trajectory_id=proposal.expected_trajectory_id,
        source_stamp=proposal.source_stamp,valid_until=proposal.valid_until,
        accepted=True,transport_mode='isolated_mock')
    clock.ns+=300_000_001;clock.mono+=.300000001
    assert not callback.on_receipt(receipt)
    assert callback.transport.accepted is None
    assert callback.proposal==proposal  # Rejection never re-dates the candidate.
    with pytest.raises(ValueError,match='source_and_monotonic_receipt_must_both_be_fresh'):
        callback.transport.core._require_progress(clock.ns,clock.mono)


@pytest.mark.parametrize('expired',['header','receipt','body'])
def test_stricter_debug_admission_uses_same_budget_without_renewing_map_evidence(expired):
    from d1max_planning_interfaces.msg import LocalPlanDebug
    callback,clock=strict_callback();callback.on_route(envelope());admit(callback)
    debug=LocalPlanDebug(session_id='session',generation=callback.transport.accepted.generation,
        plan_id=1,phase='accepted',valid=True,checked_context_sequence=callback.context['sequence'],
        checked_map_source_stamp_ns=clock.ns,checked_body_source_stamp_ns=clock.ns)
    debug.header.frame_id='d1max_loc_odom';set_stamp(debug.header.stamp,clock.ns)
    assert callback.on_debug(debug)
    assert callback.tick()['spline_visual_valid']
    if expired=='header':
        clock.ns+=300_000_001
        assert not callback.on_debug(debug)
    elif expired=='receipt':
        clock.mono+=.300000001
    else:
        clock.ns+=300_000_001
        set_stamp(debug.header.stamp,clock.ns)
        assert callback.on_debug(debug)
    # Keep a new map receipt with literal <=500 ms source evidence, isolating
    # the shorter body/debug budget. No sensor stamp is renewed by this timer.
    original_map=deepcopy(callback.map_report)
    callback.on_map(original_map)
    assert callback.map_fresh() and callback.map_report==original_map
    status=callback.tick()
    assert not status['spline_visual_valid']
    assert status['local_debug_fresh'] is (expired=='body')
    assert callback.transport.accepted is not None  # Not task cancellation.


@pytest.mark.parametrize('freshness',[True,0.,.01,.400000001,.5,2.,float('nan'),float('inf')])
def test_reference_policy_cannot_expand_control_wire_maximum(freshness):
    prior,clock,_=harness()
    with pytest.raises(ValueError,match='reference_freshness_exceeds_control_contract'):
        ReferenceCallbacks(dict(prior.p,freshness_s=freshness),lambda *_:None,
            lambda:clock.ns,lambda:clock.mono)
