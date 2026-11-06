"""Expired optional corrections must not starve an accepted odom window.

Passive real reference callbacks; no ROS graph or fabricated execution proof.
"""
from copy import deepcopy

import pytest

from test_continuous_reference_node import local_state_from_pair, native_received, local_geometry_commit
from test_continuous_reference_transport import BASE_NS, state
from test_rolling_reference_policy import committed_harness


def feed(c, clock, t, x, *, correction=None):
    clock.ns, clock.mono = BASE_NS+round(t*1e9), 100+t
    c.on_map(dict(c.context, valid=True, source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i, integrated_stamp_ns=clock.ns) for i in (0, 1)]))
    pair = state(t=t, x=x, correction=correction or 0.)
    if correction is None:
        return c.on_local_navigation(local_state_from_pair(pair))
    return c.on_navigation(pair)


def test_expired_correction_does_not_freeze_fresh_same_anchor_rolling_window():
    c, clock, output, route = committed_harness()
    owner = deepcopy(c.transport.accepted)
    # Correction arrives before the existing 0.5 s publication interval.
    feed(c, clock, .05, .015, correction=.1)
    candidate = c.transport.core.candidate_anchor
    assert candidate is not None and c.proposal is None
    # Independent local navigation continues, the optional global pair does not.
    for i in range(3, 18):
        feed(c, clock, i*.05, i*.015)
    assert c.proposal is not None, c.error
    proposal = native_received(c)
    assert proposal.version.anchor_id == owner.anchor_id
    assert proposal.version.route_hash == route.route_hash
    assert proposal.reference.path.poses[-1].pose.position.x > 4.
    assert proposal.source_stamp == state(t=.85, x=.255).source_stamp
    assert c.transport.core.candidate_anchor is candidate
    assert c.transport.accepted == owner
    assert c.on_local_commit(local_geometry_commit(c, clock, proposal, sequence=2, trajectory=2))
    # A genuinely new global/local pair may still stage the original correction.
    feed(c, clock, 1.4, .42, correction=.1)
    recovered = native_received(c)
    assert recovered.version.anchor_id != owner.anchor_id
    assert recovered.version.route_hash == route.route_hash
    assert not [k for k, _ in output if k in ('sdk', 'motion', 'cancel_reference', 'handoff')]


def test_stale_local_progress_cannot_be_renewed_by_deferring_a_correction():
    c, clock, _, _ = committed_harness()
    feed(c, clock, .1, .03, correction=.1)
    clock.ns, clock.mono = BASE_NS+2_000_000_000, 102.
    with pytest.raises(ValueError, match='fresh|stale'):
        c.transport.propose_window(current_source_ns=clock.ns, now_monotonic=clock.mono)
    assert c.transport.pending is None


def test_missing_established_progress_is_not_treated_as_an_optional_correction():
    c, clock, _, _ = committed_harness()
    feed(c, clock, .1, .03, correction=.1)
    c.transport.core._progress = None
    c.transport.core._last_sample = None
    with pytest.raises(ValueError, match='correspondence'):
        c.transport.propose_window(current_source_ns=clock.ns, now_monotonic=clock.mono)


def test_localization_reset_still_retires_instead_of_rolling_an_old_window():
    c, clock, _, _ = committed_harness()
    feed(c, clock, .1, .03, correction=.1)
    clock.ns, clock.mono = BASE_NS+600_000_000, 100.6
    c.on_navigation(state(t=.6, x=.18, epoch=2))
    assert c.transport.quarantined
    with pytest.raises(ValueError, match='owner_or_pending'):
        c.transport.propose_window(current_source_ns=clock.ns, now_monotonic=clock.mono)
