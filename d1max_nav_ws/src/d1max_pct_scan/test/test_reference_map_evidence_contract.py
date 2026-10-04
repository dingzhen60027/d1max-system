"""Production reference workflow consumes the native original evidence budget.

It never renews GridMap evidence, body freshness or a motion permission.
"""
from copy import deepcopy

import pytest

from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from test_continuous_reference_node import harness
from test_continuous_reference_transport import state


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
