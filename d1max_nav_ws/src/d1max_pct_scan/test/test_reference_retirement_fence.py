"""Reference slots retire on native hard-reanchor facts, never a timeout."""
from copy import deepcopy

import pytest

from test_continuous_reference_node import local_geometry_commit, native_received
from test_reference_stable_correction import feed
from test_rolling_reference_policy import committed_harness


def failed_correction():
    c, clock, output, route = committed_harness()
    incumbent = deepcopy(c.transport.accepted)
    feed(c, clock, .6)
    old = deepcopy(native_received(c))
    feed(c, clock, .8, .70, global_z=None)
    assert c.proposal is None
    assert c.prepared_for(old.version).retirement_requested
    assert c.transport.accepted == incumbent
    feed(c, clock, 1.4)
    assert c.proposal is not None
    return c, clock, output, route, incumbent, old, deepcopy(c.proposal)


def test_repeated_failed_reanchors_do_not_fill_prepared_ledger_and_block_the_task():
    c, clock, output, route = committed_harness()
    incumbent = deepcopy(c.transport.accepted)
    counts = []
    for i in range(12):
        feed(c, clock, .6+i*.8)
        assert c.proposal is not None, (i, c.error)
        current = deepcopy(native_received(c))
        counts.append(len(c.prepared))
        assert current.version.task_id == route.task_id
        assert current.version.route_hash == route.route_hash
        assert c.transport.accepted == incumbent
        feed(c, clock, .8+i*.8, .70, global_z=None)
        assert c.proposal is None
    feed(c, clock, 10.4)
    recovered = deepcopy(native_received(c))
    assert c.on_local_commit(local_geometry_commit(c, clock, recovered, sequence=2, trajectory=2))
    assert c.transport.accepted == recovered.reference
    assert max(counts) <= 2 and len(c.prepared) == 1
    assert not [k for k, _ in output if k in ('sdk', 'motion', 'cancel_reference', 'handoff')]


@pytest.mark.parametrize('outcome', ['unknown', 'rejected', 'predecessor_mismatch'])
def test_no_accepted_hard_reanchor_means_no_old_record_retirement(outcome):
    from d1max_planning_interfaces.msg import ReferenceReceipt
    c, clock, _, _, _, old, current = failed_correction()
    if outcome != 'unknown':
        receipt = ReferenceReceipt(version=current.version, proposal_id=current.proposal_id,
            expected_version=current.expected_version, expected_trajectory_id=current.expected_trajectory_id,
            source_stamp=current.source_stamp, valid_until=current.valid_until,
            accepted=False, reason='reference_contract_rejected', transport_mode='isolated_mock')
        if outcome == 'predecessor_mismatch':
            receipt.accepted = True
            receipt.expected_trajectory_id += 1
        assert not c.on_receipt(receipt)
    clock.ns += 5_000_000_000
    clock.mono += 5.
    c.prune_prepared()
    assert c.prepared_for(old.version) is not None


def test_accepted_hard_reanchor_removes_only_proven_retired_old_anchor():
    c, clock, _, _, incumbent, old, current = failed_correction()
    late = local_geometry_commit(c, clock, old, sequence=2, trajectory=2, publish_proof=False)
    native_received(c)
    assert c.prepared_for(old.version) is None
    assert c.prepared_for(current.version) is not None
    assert c.prepared_for(current.expected_version) is not None
    assert c.transport.accepted == incumbent
    assert not c.on_local_commit(late)
    assert c.transport.accepted == incumbent


@pytest.mark.parametrize('protected', ['expected_version', 'installed', 'unknown_receipt', 'deferred_commit'])
def test_hard_reanchor_keeps_owner_or_unresolved_original_facts(protected):
    c, clock, _, _, _, old, current = failed_correction()
    record = c.prepared_for(old.version)
    if protected == 'expected_version':
        # Explicitly retain the predecessor named by native, including when
        # that owner's separate result has not reached this observer yet.
        c.proposal.expected_version = deepcopy(old.version)
        record_current = c.prepared_for(current.version)
        record_current.proposal.expected_version = deepcopy(old.version)
    elif protected == 'installed':
        record.installed = True
    elif protected == 'unknown_receipt':
        record.receipt = None
    else:
        event = local_geometry_commit(c, clock, old, sequence=2, trajectory=2, publish_proof=False)
        c.deferred_local_commits[event.commit_sequence] = event
    native_received(c)
    assert c.prepared_for(old.version) is record


def test_ordinary_rolling_receipt_cannot_retire_an_older_selected_candidate():
    from test_reference_window_owner_races import old_result_with_successor, install_old
    c, _, _, _, old, event, proof, newer, _ = old_result_with_successor()
    native_received(c)
    assert c.prepared_for(old.version) is not None
    assert c.prepared_for(newer.version) is not None
    install_old(c, event, proof)
    assert c.transport.accepted == old.reference
    assert c.proposal == newer


def test_late_receipt_does_not_retire_newer_pending_record():
    c, clock, _, _, _, old, current = failed_correction()
    old_record = c.prepared_for(old.version)
    duplicate = deepcopy(old_record.receipt)
    assert c.on_receipt(duplicate)
    assert c.prepared_for(current.version) is not None and c.proposal == current
    # Even an exactly matching unknown original receipt cannot make an old
    # generation into a fence for the current or a newer generation.
    old_record.receipt = None
    assert c.on_receipt(duplicate)
    assert c.prepared_for(current.version) is not None and c.proposal == current


def test_cancelled_task_does_not_reopen_on_late_hard_reanchor_receipt():
    from d1max_pct_scan.continuous_reference_transport import set_stamp
    c, clock, output, route, _, old, current = failed_correction()
    old_receipt = deepcopy(c.prepared_for(old.version).receipt)
    cancel = deepcopy(route)
    cancel.active = False
    cancel.delivery_sequence += 1
    set_stamp(cancel.source_stamp, clock.ns)
    c.on_route(cancel)
    assert not c.on_receipt(old_receipt)
    assert not c.prepared and c.proposal is None and c.transport.route is None
