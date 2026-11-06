"""Real ReferenceCallbacks with controlled middleware/work latency; no ROS graph."""
from copy import deepcopy
from threading import Event
import time
import pytest

from d1max_planning_interfaces.msg import ReferenceReceipt
from d1max_pct_scan.continuous_reference_transport import nanoseconds, set_stamp
from test_continuous_reference_node import (harness, native_received, local_state_from_pair,
                                           local_geometry_commit, long_envelope, reset_debug)
from test_continuous_reference_transport import state, envelope, BASE_NS


def test_same_input_350ms_preparation_100ms_support_send_does_not_expire_proposal():
    """Old code sent at 450 ms after checking at 350 ms: Native rejected it."""
    c,clock,out=harness(geometry_owner='local_executor')
    build=c.build_support
    publish=c.publish
    ages=[]
    def delayed_build(request):
        clock.ns+=350_000_000;clock.mono+=.35
        return build(request)
    def delayed_publish(kind,value):
        if kind=='support':
            clock.ns+=100_000_000;clock.mono+=.1
        if kind=='proposal':
            age=clock.ns-nanoseconds(value.source_stamp);ages.append(age)
            receipt=ReferenceReceipt(version=deepcopy(value.version),proposal_id=value.proposal_id,
                expected_version=deepcopy(value.expected_version),expected_trajectory_id=value.expected_trajectory_id,
                accepted=clock.ns<nanoseconds(value.valid_until),transport_mode='isolated_mock')
            set_stamp(receipt.source_stamp,clock.ns);set_stamp(receipt.valid_until,clock.ns+500_000_000)
            c.on_receipt(receipt)
        publish(kind,value)
    c.build_support=delayed_build;c.publish=delayed_publish
    c.on_route(long_envelope())
    assert ages==[0] and c.receipt.accepted
    assert nanoseconds(c.proposal.source_stamp)==BASE_NS
    assert nanoseconds(c.proposal.valid_until)==BASE_NS+400_000_000
    assert c.prepared_for(c.proposal.version).support.source_stamp==c.proposal.source_stamp
    c.tick()
    assert c.proposal is not None and c.transport.pending is not None


def test_blocked_static_send_does_not_block_local_state_or_task_cancellation():
    wake,entered,release=Event(),Event(),Event()
    c,clock,out=harness(geometry_owner='local_executor',local_state_enabled=True,
                        async_support_preparation=True,preparation_ready=wake.set)
    publish=c.publish
    def blocked(kind,value):
        if kind=='support':
            entered.set();assert release.wait(2.)
        publish(kind,value)
    c.publish=blocked
    try:
        route=long_envelope();c.on_route(route);proposal=native_received(c)
        assert wake.wait(1.) and c.harvest_preparation()
        assert entered.wait(1.)
        begin=time.monotonic()
        clock.ns+=20_000_000;clock.mono+=.02
        assert c.on_local_navigation(local_state_from_pair(state(t=.02,x=.005)))
        assert c.transport.latest_local.body.source_ns==clock.ns
        cancel=deepcopy(route);cancel.active=False;cancel.delivery_sequence=2
        c.on_route(cancel)
        assert time.monotonic()-begin<.2
        assert c.pending_route_retirement['required']
        assert [v for k,v in out if k=='cancel_reference']
        c.on_debug(reset_debug(c,clock,c.cancel_generation))
        assert c.pending_route_retirement is None and c.transport.route is None
        release.set()
        assert c.close()
        # A send already entered middleware may finish after cancel. It is
        # static old-version data, not a new proposal/grant or task resurrection.
        assert len([v for k,v in out if k=='proposal'])==1
        supports=[v for k,v in out if k=='support']
        assert len(supports)==1 and supports[0].version==proposal.version
        assert c.transport.route is None and c.local_commit is None
    finally:
        release.set();assert c.close()


def test_admitted_support_finishes_after_deadline_without_rewriting_source_or_anchor():
    wake,entered,release=Event(),Event(),Event()
    c,clock,out=harness(geometry_owner='local_executor',async_support_preparation=True,
                        preparation_ready=wake.set)
    build=c.preparation.prepare
    def blocked(request):
        entered.set();assert release.wait(2.)
        return build(request)
    c.preparation.prepare=blocked
    try:
        c.on_route(long_envelope());assert entered.wait(1.)
        proposal=native_received(c)
        clock.ns+=600_000_000;clock.mono+=.6
        c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
            sources=[dict(sensor_id=j,integrated_stamp_ns=clock.ns) for j in (0,1)]))
        c.on_navigation(state(t=.6,x=.03));c.tick()
        assert c.proposal==proposal and c.receipt.accepted
        release.set();assert wake.wait(1.);assert c.harvest_preparation()
        with c.support_delivery._cv:
            assert c.support_delivery._cv.wait_for(lambda:c.support_delivery._result is not None,timeout=1.)
        c.harvest_support_delivery()
        support=c.prepared_for(proposal.version).support
        assert support.version==proposal.version and support.source_stamp==proposal.source_stamp
        assert c.on_local_commit(local_geometry_commit(c,clock,proposal))
        assert c.transport.accepted==proposal.reference
        assert nanoseconds(c.prepared_for(proposal.version).proposal.valid_until)==BASE_NS+400_000_000
    finally:
        release.set();assert c.close()


def test_support_send_failure_retries_literal_packet_not_another_proposal():
    wake=Event();c,clock,out=harness(async_support_preparation=True,preparation_ready=wake.set)
    publish=c.publish;calls=[]
    def fail_once(kind,value):
        if kind=='support':
            calls.append(value)
            if len(calls)==1:raise RuntimeError('fixture middleware failure')
        publish(kind,value)
    c.publish=fail_once
    try:
        c.on_route(envelope());proposal=native_received(c)
        assert wake.wait(1.) and c.harvest_preparation()
        with c.support_delivery._cv:
            assert c.support_delivery._cv.wait_for(lambda:c.support_delivery._result is not None,timeout=1.)
        c.harvest_support_delivery()
        record=c.prepared_for(proposal.version)
        assert not record.support_sent and not record.support_inflight
        clock.ns+=600_000_000;clock.mono+=.6;c.tick()
        with c.support_delivery._cv:
            assert c.support_delivery._cv.wait_for(lambda:c.support_delivery._result is not None,timeout=1.)
        c.harvest_support_delivery()
        assert record.support_sent and calls[0] is calls[1]
        assert len([v for k,v in out if k=='proposal'])==1
        assert calls[1].source_stamp==proposal.source_stamp
    finally:assert c.close()


def test_static_writer_is_bounded_latest_only_and_cancel_drops_waiting():
    from d1max_pct_scan.bounded_support_delivery import BoundedSupportDelivery
    entered,release=Event(),Event();sent=[]
    def blocked(value):
        entered.set();assert release.wait(1.);sent.append(value)
    delivery=BoundedSupportDelivery(blocked)
    try:
        delivery.submit('one',1);assert entered.wait(1.)
        for index in range(2,100):delivery.submit(str(index),index)
        with delivery._cv:assert delivery._waiting==('99',99)
        delivery.invalidate()
        with delivery._cv:assert delivery._waiting is None
        release.set();assert delivery.close()
        assert sent==[1]
    finally:release.set();assert delivery.close()


def test_displaced_and_coalesced_completion_never_leave_false_inflight_records():
    from types import SimpleNamespace
    from d1max_pct_scan.bounded_support_delivery import BoundedSupportDelivery
    entered,release,completed=Event(),Event(),Event();sent=[]
    def blocked(value):
        if value=='A':
            entered.set();assert release.wait(1.)
        sent.append(value)
        if value=='C':completed.set()
    c,clock,out=harness()
    c.support_delivery=BoundedSupportDelivery(blocked)
    records={key:SimpleNamespace(proposal=SimpleNamespace(proposal_id=key),support=key,
        support_inflight=False,support_sent=False,last_support_delivery_monotonic=-1.)
        for key in ('A','B','C')}
    c.prepared=records
    try:
        c.send_support(records['A']);assert entered.wait(1.)
        c.send_support(records['B']);c.send_support(records['C'])
        assert not records['B'].support_inflight and records['C'].support_inflight
        release.set();assert completed.wait(1.)
        with c.support_delivery._cv:
            assert c.support_delivery._cv.wait_for(
                lambda:c.support_delivery._result is not None and
                c.support_delivery._result.key=='C',timeout=1.)
        assert c.harvest_support_delivery()
        assert sent==['A','C']
        assert all(not r.support_inflight for r in records.values())
        # Coalesced A is not falsely labelled sent; only actual C completion
        # is acknowledged. A is obsolete, but could be retried literally if
        # it were still the current pending version.
        assert not records['A'].support_sent and not records['B'].support_sent
        assert records['C'].support_sent
    finally:release.set();assert c.close()


def test_close_reports_blocked_publisher_before_resources_can_be_destroyed():
    from d1max_pct_scan.bounded_support_delivery import BoundedSupportDelivery
    entered,release=Event(),Event()
    def blocked(value):
        entered.set();assert release.wait(1.)
    delivery=BoundedSupportDelivery(blocked)
    try:
        delivery.submit('A',object());assert entered.wait(1.)
        assert not delivery.close(timeout=.01)
        assert delivery._thread.is_alive()
        release.set();assert delivery.close()
        assert not delivery._thread.is_alive()
    finally:release.set();assert delivery.close()


@pytest.mark.parametrize('asynchronous',[False,True])
@pytest.mark.parametrize('cancel',[False,True])
def test_failed_support_preparation_retries_same_admitted_request_unless_cancelled(asynchronous,cancel):
    c,clock,out=harness(geometry_owner='local_executor',async_support_preparation=asynchronous)
    build=c.build_support;calls=[]
    def fail_once(request):
        calls.append(request)
        if len(calls)==1:raise RuntimeError('fixture static build failure')
        return build(request)
    if asynchronous:c.preparation.prepare=fail_once
    else:c.build_support=fail_once
    try:
        route=long_envelope();c.on_route(route);proposal=native_received(c)
        if asynchronous:
            with c.preparation._cv:
                assert c.preparation._cv.wait_for(lambda:c.preparation._result is not None,timeout=1.)
            c.harvest_preparation()
        record=c.prepared_for(proposal.version)
        assert c.error.startswith('support_preparation_failed:')
        assert record.support_request is not None and not record.support_preparing
        if cancel:
            cancelled=deepcopy(route);cancelled.active=False;cancelled.delivery_sequence+=1
            c.on_route(cancelled)
            assert record.support_request is None
        clock.ns+=600_000_000;clock.mono+=.6;c.tick()
        if cancel:
            assert len(calls)==1 and c.proposal is None
            assert not [v for k,v in out if k=='support']
            return
        if asynchronous:
            with c.preparation._cv:
                assert c.preparation._cv.wait_for(lambda:c.preparation._result is not None,timeout=1.)
            assert c.harvest_preparation()
            with c.support_delivery._cv:
                assert c.support_delivery._cv.wait_for(lambda:c.support_delivery._result is not None,timeout=1.)
            c.harvest_support_delivery()
        assert record.support_sent and record.support_request is None
        assert len(calls)==2 and calls[0] is calls[1]
        assert record.support.source_stamp==proposal.source_stamp
        assert record.support.version==proposal.version
        assert c.proposal==proposal and c.receipt.accepted
        assert len([v for k,v in out if k=='proposal'])==1
        assert not c.error.startswith('support_preparation_failed:')
    finally:assert c.close()


def test_proposal_publish_error_recovers_original_admission_fact_not_new_timestamp():
    c,clock,out=harness();publish=c.publish;calls=[];receipt=[]
    def failed_after_delivery(kind,value):
        if kind=='proposal':
            calls.append(deepcopy(value))
            if len(calls)==1:
                fact=ReferenceReceipt(version=deepcopy(value.version),proposal_id=value.proposal_id,
                    expected_version=deepcopy(value.expected_version),expected_trajectory_id=value.expected_trajectory_id,
                    accepted=True,transport_mode='isolated_mock')
                set_stamp(fact.source_stamp,clock.ns);set_stamp(fact.valid_until,clock.ns+500_000_000)
                receipt.append(fact)
                raise RuntimeError('fixture ambiguous middleware outcome')
            assert c.on_receipt(receipt[0])
        publish(kind,value)
    c.publish=failed_after_delivery
    c.on_route(long_envelope());proposal=deepcopy(c.proposal)
    assert c.error.startswith('reference_proposal_delivery_failed:')
    assert c.prepared_for(proposal.version).support is not None
    clock.ns+=600_000_000;clock.mono+=.6;c.tick()
    assert c.receipt.accepted and c.proposal==proposal
    assert calls==[proposal,proposal]
    assert nanoseconds(proposal.valid_until)==BASE_NS+400_000_000
