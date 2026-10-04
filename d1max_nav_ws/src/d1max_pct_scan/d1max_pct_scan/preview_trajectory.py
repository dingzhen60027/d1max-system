"""Current-curve evidence for official no-motion preview, not motion admission.

Geometry is immutable and comes only from an already paired native acceptance.
A source gap suspends it. Only an explicit newer whole-curve native recheck can
make it current again; callback arrival or a new planning attempt cannot.
"""
from copy import deepcopy
import math

from .live_scan_contract import fresh
from .local_debug import DEBUG_TIMEOUT, LocalDebugGate


def preview_revalidation_enabled(parameters):
    return (parameters.get('execution_mode') == 'preview'
            and parameters.get('collision_policy') == 'official'
            and parameters.get('perception_backend') == 'per_sensor_rays')


def geometry_identity(snapshot):
    return (snapshot.session_id, snapshot.generation, snapshot.plan_id,
            snapshot.frame_id, snapshot.selected_reference, snapshot.projection,
            snapshot.local_target, snapshot.progress_arc_m, snapshot.target_arc_m)


class PreviewTrajectoryLease:
    def __init__(self):
        self.reset()

    def reset(self):
        self.snapshot = None
        self.context = None
        self.context_sequence = 0
        self.spline_source_stamp = 0.
        self.proof_stamp = 0.
        self.proof_stamp_ns = 0
        self.proof_barrier_ns = 0
        self.proof_received = -math.inf
        self.proof_kind = 'none'
        self.map_source_stamp_ns = self.body_source_stamp_ns = self.map_revision = 0
        self.source_valid_until = math.inf
        self.suspended = True

    def accept(self, snapshot, *, gate, sequence, spline_stamp, now, mono):
        """Called only after the existing exact debug/spline pairing succeeds."""
        if (not gate.ready or not gate.active or snapshot.phase != 'accepted'
                or not snapshot.valid or snapshot.generation != gate.generation
                or snapshot.frame_id != gate.frame_id
                or not fresh(snapshot.stamp, now, DEBUG_TIMEOUT)
                or not fresh(spline_stamp, now, DEBUG_TIMEOUT)
                or snapshot.stamp < gate.trajectory_barrier
                or spline_stamp < gate.trajectory_barrier):
            return False
        self.reset()
        self.snapshot = deepcopy(snapshot)
        self.context, self.context_sequence = gate.context, sequence
        self.spline_source_stamp = spline_stamp
        self.proof_stamp = snapshot.stamp
        self.proof_stamp_ns = snapshot.stamp_ns or round(snapshot.stamp * 1e9)
        self.proof_received, self.proof_kind = mono, 'accepted'
        self.suspended = False
        return True

    def suspend(self, *, proof_barrier_ns=0):
        self.suspended = True
        self.proof_barrier_ns = max(self.proof_barrier_ns, proof_barrier_ns)

    def revalidate(self, snapshot, *, gate, sequence, now, mono,
                   perception_timeout, body_timeout, sensor_barrier):
        """A new check proves old geometry; never changes its original time."""
        if self.snapshot is None:
            return False
        stamp_ns = snapshot.stamp_ns or round(snapshot.stamp * 1e9)
        source = getattr(snapshot, 'checked_map_source_stamp_ns', 0)
        body = getattr(snapshot, 'checked_body_source_stamp_ns', 0)
        revision = getattr(snapshot, 'checked_map_revision', 0)
        checked_sequence = getattr(snapshot, 'checked_context_sequence', 0)
        if (not gate.ready or not gate.active or gate.preview_paused
                or gate.context != self.context
                or sequence != self.context_sequence or checked_sequence != sequence
                or snapshot.generation != gate.generation
                or snapshot.phase != 'revalidated' or snapshot.valid is not True
                or snapshot.matching_path_headers is not True
                or geometry_identity(snapshot) != geometry_identity(self.snapshot)
                or not fresh(snapshot.stamp, now, DEBUG_TIMEOUT)
                or snapshot.stamp < gate.trajectory_barrier
                or stamp_ns <= max(self.proof_stamp_ns, self.proof_barrier_ns)
                or any(type(value) is not int or value <= 0
                       for value in (source, body, revision, checked_sequence))
                or source < self.map_source_stamp_ns or body < self.body_source_stamp_ns
                or revision < self.map_revision
                or source * 1e-9 <= sensor_barrier
                or not fresh(source * 1e-9, now, perception_timeout)
                or not fresh(body * 1e-9, now, body_timeout)
                or max(source, body) > stamp_ns + 100_000_000):
            return False
        self.proof_stamp, self.proof_stamp_ns = snapshot.stamp, stamp_ns
        self.proof_received, self.proof_kind = mono, 'revalidated'
        self.map_source_stamp_ns, self.body_source_stamp_ns = source, body
        self.map_revision = revision
        self.source_valid_until = min(source*1e-9+perception_timeout,
                                      body*1e-9+body_timeout)
        self.suspended = False
        return True

    def remaining(self, *, gate, sequence, now, mono):
        if (self.snapshot is None or self.suspended or not gate.ready or not gate.active
                or gate.preview_paused or gate.context != self.context
                or gate.generation != self.snapshot.generation
                or gate.frame_id != self.snapshot.frame_id or sequence != self.context_sequence
                or self.proof_stamp < gate.trajectory_barrier
                or not fresh(self.proof_stamp, now, DEBUG_TIMEOUT)
                or not 0 <= mono-self.proof_received <= DEBUG_TIMEOUT):
            return 0.
        return max(0., min(DEBUG_TIMEOUT - max(0., now-self.proof_stamp),
                           DEBUG_TIMEOUT - (mono-self.proof_received),
                           self.source_valid_until-now))


class PreviewTrajectoryDelivery:
    """Bounded, non-authoritative delivery of native geometry across soft gaps.

    A callback can receive the first spline/accepted pair while the map lease
    is suspended. Keep that pair without drawing it or admitting execution.
    Only a later whole-curve recheck can promote it to a preview lease. Source
    stamps and geometry are never regenerated; hard invalidation tombstones
    every received plan so delayed packets cannot rebuild a revoked pair.
    """
    def __init__(self):
        self.key = None
        self.floor = -1
        self.highest = -1
        self.debug = LocalDebugGate()
        self.spline = self.candidate = None
        self.pending_proof = None
        self.proof_barrier_ns = 0

    def synchronize(self, gate, sequence, session_id):
        key = (session_id, gate.generation, gate.frame_id, gate.context, sequence)
        if key != self.key:
            self.key, self.floor, self.highest = key, -1, -1
            self.debug.reset(gate.generation)
            self.spline = self.candidate = None
            self.pending_proof = None
            self.proof_barrier_ns = 0
        return gate.active and gate.context is not None and sequence > 0

    def clear(self):
        self.floor = max(self.floor, self.highest)
        self.debug.clear('delivery_invalidated')
        self.spline = self.candidate = None
        self.pending_proof = None

    def eligible_spline(self, *, session_id, generation, frame_id, plan_id,
                        stamp, gate, sequence, expected_session, now):
        return (self.synchronize(gate, sequence, expected_session)
                and session_id == expected_session and type(generation) is int
                and generation == gate.generation
                and frame_id == gate.frame_id and type(plan_id) is int
                and max(self.floor, -1) < plan_id < 2**63
                and plan_id >= self.highest
                and stamp >= gate.issued_at and fresh(stamp, now, DEBUG_TIMEOUT)
                and (self.spline is None or plan_id != self.spline['plan_id']))

    def capture_spline(self, record, *, session_id, generation, frame_id, plan_id,
                       stamp, gate, sequence, expected_session, now, mono):
        if not self.eligible_spline(session_id=session_id, generation=generation,
                frame_id=frame_id, plan_id=plan_id, stamp=stamp, gate=gate,
                sequence=sequence, expected_session=expected_session, now=now):
            return False
        self.highest = max(self.highest, plan_id)
        self.spline = dict(record=deepcopy(record), plan_id=plan_id, stamp=stamp, received=mono)
        self.debug.pair(spline_id=plan_id, spline_stamp=stamp, now=now, mono=mono)
        self._pair(gate, sequence, now, mono)
        return True

    def capture_debug(self, snapshot, *, gate, sequence, expected_session, now, mono):
        if (not self.synchronize(gate, sequence, expected_session)
                or snapshot.session_id != expected_session
                or snapshot.generation != gate.generation or snapshot.frame_id != gate.frame_id
                or not snapshot.matching_path_headers or snapshot.stamp < gate.issued_at
                or not fresh(snapshot.stamp, now, DEBUG_TIMEOUT)):
            return
        if not snapshot.valid:
            event_ns = snapshot.stamp_ns or round(snapshot.stamp * 1e9)
            if type(snapshot.plan_id) is int and snapshot.plan_id >= 0:
                # A later explicit inability to certify the curve fences all
                # earlier positive proofs, including cross-topic late arrivals.
                self.proof_barrier_ns = max(self.proof_barrier_ns, event_ns)
                if self.pending_proof is not None:
                    pending = self.pending_proof[0]
                    if (pending.stamp_ns or round(pending.stamp*1e9)) <= self.proof_barrier_ns:
                        self.pending_proof = None
            if (snapshot.phase not in ('waiting_sensor_map', 'waiting_recheck')
                    and type(snapshot.plan_id) is int and snapshot.plan_id >= self.highest
                    and event_ns > self.debug.last_event_stamp_ns):
                self.highest = max(self.highest, snapshot.plan_id)
                self.debug.last_event_stamp_ns = event_ns
                self.clear()
            return
        if snapshot.phase == 'revalidated':
            # The proof and the original spline travel on different topics.
            # Store one bounded original event if it beats the geometry pair;
            # storage is never admission and does not advance debug ordering.
            previous = self.pending_proof[0] if self.pending_proof is not None else None
            event_ns = snapshot.stamp_ns or round(snapshot.stamp * 1e9)
            if (type(snapshot.plan_id) is int and snapshot.plan_id > self.floor
                    and event_ns > self.proof_barrier_ns
                    and (previous is None or event_ns > (previous.stamp_ns
                         or round(previous.stamp * 1e9)))):
                self.pending_proof = (deepcopy(snapshot), mono)
            return
        if (snapshot.phase != 'accepted' or type(snapshot.plan_id) is not int
                or snapshot.plan_id <= self.floor):
            return
        # Reuse the production accepted-debug geometry/order validator. This
        # copy permits storage only; it is never used for output admission.
        storage_gate = deepcopy(gate)
        storage_gate.ready = True
        source = self.spline
        action = self.debug.receive(deepcopy(snapshot), expected_session=expected_session,
            gate=storage_gate, now=now, mono=mono,
            spline_id=source['plan_id'] if source else -1,
            spline_stamp=source['stamp'] if source else 0.)
        if action != 'ignore':
            self.highest = max(self.highest, self.debug.highest_plan_id)
        self._pair(gate, sequence, now, mono)

    def _pair(self, gate, sequence, now, mono):
        source, debug = self.spline, self.debug.active
        if (source is None or debug is None or source['plan_id'] != debug.plan_id
                or not 0 <= mono-source['received'] <= DEBUG_TIMEOUT):
            return
        storage_gate = deepcopy(gate)
        storage_gate.ready, storage_gate.preview_paused = True, False
        storage_gate.trajectory_barrier = storage_gate.issued_at
        lease = PreviewTrajectoryLease()
        if lease.accept(debug, gate=storage_gate, sequence=sequence,
                        spline_stamp=source['stamp'], now=now, mono=mono):
            lease.suspend()
            record = deepcopy(source['record'])
            record['snapshot'] = deepcopy(debug)
            self.candidate = (lease, record)

    def recover(self, snapshot, **proof_arguments):
        gate, sequence = proof_arguments['gate'], proof_arguments['sequence']
        if (self.key is None or self.key[1:] !=
                (gate.generation, gate.frame_id, gate.context, sequence)
                or self.candidate is None
                or (snapshot.stamp_ns or round(snapshot.stamp*1e9)) <= self.proof_barrier_ns):
            return None
        lease, record = self.candidate
        if lease.revalidate(snapshot, **proof_arguments):
            if (self.pending_proof is not None
                    and self.pending_proof[0].stamp_ns == snapshot.stamp_ns):
                self.pending_proof = None
            return lease, deepcopy(record)
        return None

    def recover_pending(self, **proof_arguments):
        if self.pending_proof is None:
            return None
        snapshot, received = self.pending_proof
        if (not 0 <= proof_arguments['mono']-received <= DEBUG_TIMEOUT
                or not fresh(snapshot.stamp, proof_arguments['now'], DEBUG_TIMEOUT)):
            self.pending_proof = None
            return None
        # Re-check all original source/body/map/context bounds at the current
        # time, but preserve actual proof receipt time when a late pair arrives.
        arguments = dict(proof_arguments, mono=received)
        return self.recover(snapshot, **arguments)
