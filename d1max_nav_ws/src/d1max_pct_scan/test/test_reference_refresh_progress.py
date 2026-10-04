import math

import pytest

from d1max_pct_scan.live_scan_contract import ReferenceGate


def active_gate():
    gate = ReferenceGate()
    gate.observe(True, 100., ('session', 1, 'seed'))
    route = [[i*.5, 0., 0.] for i in range(12)]
    assert gate.accept(route, frame_id=gate.frame_id, stamp=100.1, now=100.1,
                       body_xyz=[0., 0., .55])
    return gate, route


def test_same_active_route_refresh_after_progress_keeps_native_identity():
    gate, route = active_gate()
    generation, digest, issued_at = gate.generation, gate.digest, gate.issued_at
    for step in range(1, 15):
        now = 100.1+step*.1
        assert not gate.accept(route, frame_id=gate.frame_id, stamp=now, now=now,
                               body_xyz=[step*.3, 0., .55])
        assert (gate.generation, gate.digest, gate.issued_at) == (generation, digest, issued_at)
        assert gate.last_path_stamp == now and gate.active


@pytest.mark.parametrize('inactive', [False, True])
def test_new_or_inactive_route_cannot_skip_start_admission(inactive):
    gate, route = active_gate()
    if inactive:
        gate.revoke(100.2, 'cancel')
    else:
        route[-1][1] = .1
    with pytest.raises(ValueError, match='reference_start_not_near_live_body'):
        gate.accept(route, frame_id=gate.frame_id, stamp=100.3, now=100.3,
                    body_xyz=[3., 0., .55])


def test_same_geometry_does_not_skip_freshness_frame_or_current_body_checks():
    for override in (dict(stamp=99.), dict(frame_id='other'),
                     dict(body_xyz=[math.nan, 0., .55])):
        gate, route = active_gate()
        args = dict(frame_id=gate.frame_id, stamp=100.3, now=100.3, body_xyz=[3., 0., .55])
        args.update(override)
        with pytest.raises(ValueError):
            gate.accept(route, **args)


def test_soft_pause_preserves_identity_but_not_local_permission():
    gate, _ = active_gate()
    generation, stamp, digest = gate.generation, gate.issued_at, gate.digest
    assert gate.pause_preview_reference(100.2)
    assert not gate.ready and gate.preview_paused and gate.active
    assert not gate.resume_preview_reference(100.3, ('session', 2, 'other'))
    assert gate.resume_preview_reference(100.4, gate.context)
    assert (gate.generation, gate.issued_at, gate.digest) == (generation, stamp, digest)
    assert gate.trajectory_barrier == 100.4
