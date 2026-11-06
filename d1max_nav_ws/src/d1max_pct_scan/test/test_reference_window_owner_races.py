"""Owner facts and pending windows are independent serialized state lanes."""
from copy import deepcopy
from threading import Event

import pytest

from d1max_pct_scan.bounded_preparation import LatestPreparation
from d1max_pct_scan.continuous_reference_transport import set_stamp
from d1max_planning_interfaces.msg import ReferenceReceipt
from test_continuous_reference_node import (
    native_received, local_geometry_commit, local_commit_proof,
    delivery_for, reset_debug,
)
from test_rolling_reference_policy import committed_harness, step
from test_continuous_reference_transport import BASE_NS


def old_result_with_successor(*, worker=False):
    c, clock, output, route = committed_harness()
    for i in range(1, 18):
        step(c, clock, i, .015*i)
    old = native_received(c)
    event = local_geometry_commit(c, clock, old, 2, 2, publish_proof=False)
    proof = local_commit_proof(c, clock, old, event)
    started, release = Event(), Event()
    if worker:
        def prepare(request):
            started.set()
            assert release.wait(3.)
            return c.build_support(request)
        c.preparation = LatestPreparation(prepare)
    for i in range(18, 272):
        step(c, clock, i, .015*i)
    if worker:
        assert started.wait(1.)
    newer = deepcopy(c.proposal)
    assert newer.version.reference_generation == old.version.reference_generation+1
    return c, clock, output, route, old, event, proof, newer, release


def install_old(c, event, proof):
    assert c.on_validation(proof)
    assert c.on_local_commit(event)
    assert c.transport.accepted.generation == event.version.reference_generation


@pytest.mark.parametrize('receipt', ['unacknowledged', 'accepted'])
def test_delayed_old_commit_keeps_successor_delivery_and_original_bounds(receipt):
    c, clock, output, route, old, event, proof, newer, _ = old_result_with_successor()
    if receipt == 'accepted':
        native_received(c)
    pending = c.transport.pending
    pending_core = c.transport.pending_core
    exact_receipt = deepcopy(c.receipt)
    bounds = (c.transport._pending_received_monotonic, c.transport._pending_start_arc_m,
              c.transport._pending_end_arc_m, c.transport._pending_issue_arc_m)
    install_old(c, event, proof)
    assert c.transport.pending is pending
    assert c.transport.pending_core is pending_core
    assert c.proposal == newer and c.receipt == exact_receipt
    assert bounds == (c.transport._pending_received_monotonic, c.transport._pending_start_arc_m,
                      c.transport._pending_end_arc_m, c.transport._pending_issue_arc_m)
    assert c.transport.accepted.path.header.stamp == old.source_stamp
    assert c.transport.route == route
    assert not [kind for kind, _ in output if kind in ('cancel_reference', 'sdk', 'motion')]
    clock.ns += 500_000_000
    clock.mono += .5
    assert c.retry_unacknowledged_delivery() == (receipt == 'unacknowledged')
    if receipt == 'unacknowledged':
        assert [value for kind, value in output if kind == 'proposal'][-1] == newer


def test_successor_support_worker_is_not_invalidated_by_old_owner_fact():
    c, clock, output, route, old, event, proof, newer, release = old_result_with_successor(worker=True)
    try:
        record = c.prepared_for(newer.version)
        assert record.delivered and record.support is None
        ticket = c.preparation._ticket
        install_old(c, event, proof)
        assert c.preparation._ticket == ticket
        assert c.proposal == newer and c.transport.pending == newer.reference
        release.set()
        with c.preparation._cv:
            assert c.preparation._cv.wait_for(lambda: c.preparation._result is not None, timeout=1.)
        assert c.harvest_preparation()
        assert record.delivered
        assert [value for kind, value in output if kind == 'proposal'][-1] == newer
        assert c.transport.accepted == old.reference
        assert c.transport.route == route
    finally:
        release.set()
        assert c.close()


def test_newer_receipt_and_commit_still_close_their_exact_slot_after_old_result():
    c, clock, _, _, _, event, proof, newer, _ = old_result_with_successor()
    install_old(c, event, proof)
    native_received(c)
    next_event = local_geometry_commit(c, clock, newer, 3, 3)
    assert c.on_local_commit(next_event)
    assert c.transport.accepted == newer.reference
    assert c.proposal is None and c.receipt is None and c.transport.pending is None
    accepted = deepcopy(c.transport.accepted)
    core = c.transport.core
    assert c.on_local_commit(next_event)  # exact duplicate is idempotent
    assert not c.on_local_commit(event)
    assert c.transport.accepted == accepted and c.transport.core is core


def test_late_successor_rejection_retires_only_its_own_pending_slot():
    c, clock, _, _, old, event, proof, newer, _ = old_result_with_successor()
    install_old(c, event, proof)
    negative = ReferenceReceipt(version=newer.version, proposal_id=newer.proposal_id,
        expected_version=newer.expected_version, expected_trajectory_id=newer.expected_trajectory_id,
        source_stamp=newer.source_stamp, valid_until=newer.valid_until,
        accepted=False, reason='rolling_reference_predecessor_changed_retry',
        transport_mode='isolated_mock')
    assert not c.on_receipt(negative)
    assert c.transport.pending is None and c.proposal is None
    assert c.transport.accepted == old.reference
    assert c.local_commit == event


@pytest.mark.parametrize('boundary', ['cancel', 'replacement', 'context_reset'])
def test_old_owner_fact_never_restores_retired_task_or_context(boundary):
    c, clock, _, route, _, event, proof, _, _ = old_result_with_successor()
    if boundary == 'context_reset':
        c.transport.quarantined = True
    else:
        cancel = deepcopy(route)
        cancel.active = False
        cancel.delivery_sequence += 1
        set_stamp(cancel.source_stamp, clock.ns)
        c.on_route(cancel)
        if boundary == 'replacement':
            c.on_debug(reset_debug(c, clock, c.cancel_generation))
            # A new route can own the input slot even before its first usable
            # source body. Its identity must never accept the prior result.
            newer = delivery_for('replacement', sequence=cancel.delivery_sequence+1,
                                 t=(clock.ns-BASE_NS)*1e-9)
            try:
                c.on_route(newer)
            except ValueError:
                pass
            assert c.transport.ingress.active_identity == (
                newer.task_id, newer.route_id, newer.route_hash)
    before = deepcopy(c.transport.accepted)
    proposal = deepcopy(c.proposal)
    facts = deepcopy(c.validation_facts)
    proofs = deepcopy(c.local_commit_proofs)
    deferred = deepcopy(c.deferred_local_commits)
    assert not c.on_validation(proof)
    assert not c.on_local_commit(event)
    assert c.transport.accepted == before and c.proposal == proposal
    assert c.validation_facts == facts
    assert c.local_commit_proofs == proofs and c.deferred_local_commits == deferred
