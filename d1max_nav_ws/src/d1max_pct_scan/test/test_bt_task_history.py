"""Long-lived real callback contracts, without a ROS graph or robot session."""
from copy import deepcopy

import pytest
from rclpy.action import GoalResponse
from d1max_planning_interfaces.msg import RouteDeliveryReceipt
from d1max_pct_scan.route_ingress import RouteIngress
from test_route_ingress import delivery
from test_bt_action_adapters import harness as adapter_harness, Handle, request
from test_continuous_reference_node import harness, delivery_for, delivery_receipts


def task_delivery(ordinal, sequence, *, active=True, task=None, route=None):
    message=delivery()
    message.task_id=message.snapshot.task_id=task or f'session.{ordinal}'
    if route is not None:
        message.route_id=message.snapshot.route_id=route
    message.delivery_sequence=sequence
    message.active=active
    return message


def accept(gate,message,**kwargs):
    return gate.accept(message,now=10.1,context=('session',1,'seed'),**kwargs)


def test_formal_route_retirement_survives_more_than_128_targets_and_rejects_old_revival():
    first=task_delivery(1,1)
    gate=RouteIngress('session',first.snapshot.map_version_id)
    for ordinal in range(1,301):
        assert accept(gate,task_delivery(ordinal,2*ordinal-1))
        assert accept(gate,task_delivery(ordinal,2*ordinal,active=False))
        assert gate.active_identity is None
    # A fresh delivery is not a new task. Even evicted receipt history must not
    # permit either the original or a differently named route for task 1.
    for sequence,route in ((601,None),(602,'different-route')):
        with pytest.raises(ValueError,match='retired'):
            accept(gate,task_delivery(1,sequence,route=route))
    assert not gate.retired  # formal history does not accumulate per-task sets
    assert accept(gate,task_delivery(301,603))
    assert accept(gate,task_delivery(301,604,active=False))


def test_task_tombstone_is_not_bypassed_by_changing_route_id():
    gate=RouteIngress('session',delivery().snapshot.map_version_id)
    assert accept(gate,task_delivery(7,1,active=False),retire_unadmitted=True)
    with pytest.raises(ValueError,match='retired'):
        accept(gate,task_delivery(7,2,route='another-route'))
    assert gate.active_identity is None


def test_future_unadmitted_cancel_does_not_revoke_current_exact_owner_or_lower_fence():
    gate=RouteIngress('session',delivery().snapshot.map_version_id)
    assert accept(gate,task_delivery(5,1))
    owner=gate.active_identity
    assert accept(gate,task_delivery(9,2,active=False),retire_unadmitted=True)
    assert gate.active_identity==owner
    assert accept(gate,task_delivery(5,3))
    assert gate.active_identity==owner
    assert accept(gate,task_delivery(5,4,active=False))
    assert gate.active_identity is None
    for ordinal in (5,6,8,9):
        with pytest.raises(ValueError,match='retired'):
            accept(gate,task_delivery(ordinal,10+ordinal))
    assert accept(gate,task_delivery(10,30))
    newer=gate.active_identity
    assert accept(gate,task_delivery(5,31,active=False),retire_unadmitted=True)
    assert gate.active_identity==newer


@pytest.mark.parametrize('suffix',['0','01','+1','-1','1.0','1x','١','18446744073709551616',''])
def test_reserved_task_aliases_never_enter_legacy_domain_or_advance_watermark(suffix):
    gate=RouteIngress('session',delivery().snapshot.map_version_id)
    with pytest.raises(ValueError,match='task_identity_sequence_invalid'):
        accept(gate,task_delivery(1,1,task='session.'+suffix,active=False),retire_unadmitted=True)
    assert gate.last_sequence==0 and gate.active_identity is None
    assert accept(gate,task_delivery(1,2))


def test_rejected_context_and_unretired_replacement_do_not_poison_task_watermark():
    gate=RouteIngress('session',delivery().snapshot.map_version_id)
    with pytest.raises(ValueError,match='context'):
        gate.accept(task_delivery(99999,1),now=10.1,context=('session',2,'different'))
    assert accept(gate,task_delivery(1,2))
    with pytest.raises(ValueError,match='not_retired'):
        accept(gate,task_delivery(99999,3))
    assert accept(gate,task_delivery(1,4,active=False))
    assert accept(gate,task_delivery(2,5))


def test_legacy_route_budget_rejects_new_owner_before_accept_and_never_blocks_formal_tasks():
    gate=RouteIngress('session',delivery().snapshot.map_version_id)
    for ordinal in range(128):
        assert accept(gate,task_delivery(0,ordinal+1,task=f'old-{ordinal}',active=False),
            retire_unadmitted=True)
    before_sequence=gate.last_sequence
    with pytest.raises(ValueError,match='capacity'):
        accept(gate,task_delivery(0,129,task='would-get-stuck'))
    assert gate.active_identity is None and gate.last_sequence==before_sequence
    assert len(gate.retired)==128
    assert accept(gate,task_delivery(1,130))
    assert accept(gate,task_delivery(1,131,active=False))
    assert gate.active_identity is None and len(gate.retired)==128


def test_unadmitted_legacy_cancels_cannot_take_the_current_owners_retirement_slot():
    gate=RouteIngress('session',delivery().snapshot.map_version_id)
    assert accept(gate,task_delivery(0,1,task='current-legacy'))
    owner=gate.active_identity
    for ordinal in range(127):
        assert accept(gate,task_delivery(0,ordinal+2,task=f'other-{ordinal}',active=False),
            retire_unadmitted=True)
    with pytest.raises(ValueError,match='capacity'):
        accept(gate,task_delivery(0,129,task='overflow-cancel',active=False),retire_unadmitted=True)
    assert gate.active_identity==owner and gate.last_sequence==128
    assert accept(gate,task_delivery(0,130,task='current-legacy'))
    assert accept(gate,task_delivery(0,131,task='current-legacy',active=False))
    assert gate.active_identity is None and len(gate.retired)==128
    # The full compatibility set does not consume formal watermark storage.
    assert accept(gate,task_delivery(1,132))
    assert accept(gate,task_delivery(1,133,active=False))


def test_route_receipt_cache_remains_bounded_and_retries_recent_cancellation():
    c,clock,out=harness(geometry_owner='local_executor')
    c.map_report=None  # Exercise ownership before native work exists, not fake ACKs.
    for ordinal in range(1,151):
        c.on_route(delivery_for(f'session.{ordinal}',sequence=ordinal*2-1))
        assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.ACCEPTED
        c.on_route(delivery_for(f'session.{ordinal}',sequence=ordinal*2,active=False))
        assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.RETIRED
        assert c.pending_route_retirement is None and not c.native_route_exposures
    assert len(c.route_dispositions)<=129
    cancellation=delivery_for('session.150',sequence=300,active=False)
    c.on_route(cancellation)
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.RETIRED
    assert delivery_receipts(out)[-1].source_stamp==cancellation.source_stamp
    # An original ACTIVE receipt still cached must not masquerade as active
    # after retirement; neither a fresh higher-sequence delivery may revive it.
    for sequence in (299,301):
        try:c.on_route(delivery_for('session.150',sequence=sequence))
        except ValueError as error:assert 'retired' in str(error)
        assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.REJECTED
        assert c.transport.ingress.active_identity is None
    with pytest.raises(ValueError,match='retired'):
        c.on_route(delivery_for('session.1',sequence=302))
    c.on_route(delivery_for('session.151',sequence=303))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.ACCEPTED


def test_current_receipt_survives_many_future_unadmitted_cancels_and_then_retires():
    c,clock,out=harness(geometry_owner='local_executor');c.map_report=None
    active=delivery_for('session.1',sequence=1)
    c.on_route(active)
    owner=c.transport.ingress.active_identity
    for ordinal in range(2,151):
        c.on_route(delivery_for(f'session.{ordinal}',sequence=ordinal,active=False))
        assert c.transport.ingress.active_identity==owner
    assert len(c.route_dispositions)<=129
    c.on_route(deepcopy(active))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.ACCEPTED
    assert delivery_receipts(out)[-1].source_stamp==active.source_stamp
    c.on_route(delivery_for('session.1',sequence=151,active=False))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.RETIRED
    assert c.transport.ingress.active_identity is None
    with pytest.raises(ValueError,match='retired'):
        c.on_route(delivery_for('session.149',sequence=152))
    c.on_route(delivery_for('session.151',sequence=153))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.ACCEPTED


def test_pending_native_retirement_is_not_evicted_by_newer_unadmitted_cancels():
    from test_continuous_reference_node import reset_debug
    c,clock,out=harness(geometry_owner='local_executor')
    c.on_route(delivery_for('session.1',sequence=1))
    c.on_route(delivery_for('session.1',sequence=2,active=False))
    pending=deepcopy(c.pending_route_retirement)
    assert pending['required'] and c.native_route_exposures
    for ordinal in range(2,151):
        c.on_route(delivery_for(f'session.{ordinal}',sequence=ordinal+1,active=False))
        assert c.pending_route_retirement==pending
        assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.RETIRED
    assert len(c.route_dispositions)<=129
    assert c.route_identity(pending['command']) in c.route_dispositions
    c.on_debug(reset_debug(c,clock,pending['generation']-1))
    assert c.pending_route_retirement==pending
    c.on_debug(reset_debug(c,clock,pending['generation']))
    receipt=delivery_receipts(out)[-1]
    assert receipt.task_id=='session.1' and receipt.status==RouteDeliveryReceipt.RETIRED
    assert receipt.native_retirement_required and receipt.native_retirement_confirmed
    assert receipt.source_stamp==pending['command'].source_stamp
    assert c.pending_route_retirement is None and not c.native_route_exposures
    with pytest.raises(ValueError,match='retired'):
        c.on_route(delivery_for('session.149',sequence=200))
    c.on_route(delivery_for('session.151',sequence=201))
    owner=c.transport.ingress.active_identity
    c.on_debug(reset_debug(c,clock,pending['generation']))
    assert c.transport.ingress.active_identity==owner


def test_empty_localization_context_recovery_preserves_formal_retirement_watermark():
    from test_continuous_reference_transport import transport,on_state,empty_reset_pair,recover_empty,BASE_NS
    value,output=transport();on_state(value)
    for sequence,active in ((1,True),(2,False)):
        value.on_route(delivery_for('session.130',sequence=sequence,active=active),
            current_source_ns=BASE_NS,now_monotonic=100.)
    empty_reset_pair(value)
    assert recover_empty(value)
    with pytest.raises(ValueError,match='retired'):
        value.on_route(delivery_for('session.1',sequence=3,t=.1),
            current_source_ns=BASE_NS+100_000_000,now_monotonic=100.1)
    value.on_route(delivery_for('session.131',sequence=4,t=.1),
        current_source_ns=BASE_NS+100_000_000,now_monotonic=100.1)
    assert value.ingress.active_identity[0]=='session.131'


def admit_compute_without_starting_worker(node,task):
    handle=Handle(request(task))
    assert node.compute_goal(handle.request)==GoalResponse.ACCEPT
    node.accept_compute(handle)
    assert handle.executed==1 and node.compute['handle'] is handle
    # Simulate completed worker ownership only; these tests concern the real
    # admission callbacks, not algorithm results or a fabricated navigation.
    node.compute=None
    node.executions.clear()
    return handle


def test_formal_compute_admission_survives_4096_tasks_without_quarantine(monkeypatch):
    node,clock=adapter_harness(monkeypatch)
    node.compute_tick=lambda:None
    for ordinal in range(1,4101):
        admit_compute_without_starting_worker(node,f'session.{ordinal}')
        assert node.quarantine==''
    assert node.compute_goal(request('session.1'))==GoalResponse.REJECT
    assert node.compute_goal(request('session.4100'))==GoalResponse.REJECT
    assert not node.used_tasks.legacy
    assert node.compute_goal(request('session.4101'))==GoalResponse.ACCEPT


def test_legacy_compute_capacity_rejects_before_accept_not_quarantines_existing_work(monkeypatch):
    node,clock=adapter_harness(monkeypatch);node.compute_tick=lambda:None
    for ordinal in range(4096):
        admit_compute_without_starting_worker(node,f'legacy-{ordinal}')
    assert node.compute_goal(request('legacy-overflow'))==GoalResponse.REJECT
    assert node.quarantine=='' and not node.reserved_compute and node.compute is None
    assert node.compute_goal(request('legacy-0'))==GoalResponse.REJECT
    # A legacy compatibility budget cannot exhaust formal task watermarks.
    admit_compute_without_starting_worker(node,'session.1')
    assert node.quarantine==''


@pytest.mark.parametrize('task',['session.0','session.01','session.+1','session.18446744073709551616'])
def test_compute_alias_rejection_does_not_reserve_or_quarantine_session(monkeypatch,task):
    node,clock=adapter_harness(monkeypatch);node.compute_tick=lambda:None
    assert node.compute_goal(request(task))==GoalResponse.REJECT
    assert not node.reserved_compute and node.quarantine==''
    admit_compute_without_starting_worker(node,'session.1')


def test_history_parses_complete_dotted_session_and_never_evicts_legacy_fence():
    from d1max_pct_scan.bt_task_identity import BoundedTaskHistory, UINT64_MAX
    history=BoundedTaskHistory('run.session',legacy_limit=2)
    history.remember('run.session.2')
    assert history.contains('run.session.1') and history.contains('run.session.2')
    assert not history.contains('run.session.3')
    history.remember('legacy-a');history.remember('legacy-b')
    assert not history.can_remember('legacy-c')
    with pytest.raises(ValueError,match='capacity'):history.remember('legacy-c')
    assert history.contains('legacy-a') and history.contains('legacy-b')
    history.remember(f'run.session.{UINT64_MAX}')
    assert history.through==UINT64_MAX and len(history.legacy)==2
