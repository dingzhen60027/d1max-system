"""Bounded source-time joint history for articulated robot self geometry.

This module accepts named JointState positions with the ROS SI-unit contract:
angular joints are in radians, and prismatic joints are in metres. A receipt
time or an SDK JSON value with unspecified units is not a source measurement.
There is no ROS import, default joint angle, extrapolation, or last-pose hold.

Missing coverage is local to each joint. Callers must skip an articulated link
when any joint on its chain is missing; fixed geometry needs no joint sample.
"""
from bisect import bisect_left
from collections.abc import Mapping
from dataclasses import dataclass, fields, is_dataclass, replace
import math
from itertools import islice
from numbers import Integral, Real
from threading import Lock


_JOINT_TYPES = frozenset(('fixed', 'revolute', 'continuous', 'prismatic'))
_MAX_MODEL_JOINTS = 256


def _stamp(value):
    return isinstance(value, Integral) and not isinstance(value, bool) and 0 < value < 2**63


def _name(value):
    return isinstance(value, str) and 0 < len(value) <= 128


def _freeze_identity(value):
    """Copy pure identity data without keeping a mutable caller-owned object."""
    if value is None or isinstance(value, (str, bytes, bool, int)):
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    if isinstance(value, (tuple, list)):
        return tuple(_freeze_identity(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_freeze_identity(item) for item in value)
    if isinstance(value, Mapping):
        return frozenset((_freeze_identity(key), _freeze_identity(item))
                         for key, item in value.items())
    if is_dataclass(value) and not isinstance(value, type):
        copied = {field.name: _freeze_identity(getattr(value, field.name))
                  for field in fields(value)}
        if value.__dataclass_params__.frozen and all(field.init for field in fields(value)):
            return replace(value, **copied)
        return (type(value).__module__, type(value).__qualname__, tuple(copied.items()))
    raise ValueError('invalid_joint_pose_identity')


@dataclass(frozen=True)
class JointSample:
    source_ns: int
    positions: tuple


@dataclass(frozen=True)
class JointTrajectory:
    name: str
    joint_type: str
    source_ns: tuple
    positions: tuple

    def position_at(self, source_ns, max_gap_ns):
        index = bisect_left(self.source_ns, source_ns)
        if index < len(self.source_ns) and self.source_ns[index] == source_ns:
            return self.positions[index]
        if index == 0 or index == len(self.source_ns):
            return None
        left_ns, right_ns = self.source_ns[index-1:index+1]
        if right_ns-left_ns > max_gap_ns:
            return None
        left, right = self.positions[index-1:index+1]
        fraction = (source_ns-left_ns)/(right_ns-left_ns)
        if self.joint_type == 'continuous':
            # Reduce before subtraction so finite extreme readings cannot
            # overflow into an exception in a worker's shortest-arc query.
            difference = math.remainder(math.remainder(right, 2.*math.pi)
                - math.remainder(left, 2.*math.pi), 2.*math.pi)
            return left + fraction*difference
        # Revolute coordinates remain the measured coordinates, even across
        # +/-pi. Their physical bounded arc must not be replaced by wrapping.
        return (1.-fraction)*left + fraction*right


@dataclass(frozen=True)
class JointPoseSnapshot:
    """Immutable worker input; each query returns a fresh, caller-owned dict."""
    identity: object
    generation: int
    joint_types: tuple
    samples: tuple
    trajectories: tuple
    max_gap_ns: int
    diagnostic_counts: tuple

    @property
    def sample_count(self):
        return len(self.samples)

    @property
    def latest_source_ns(self):
        return self.samples[-1].source_ns if self.samples else 0

    @property
    def oldest_source_ns(self):
        return self.samples[0].source_ns if self.samples else 0

    @property
    def diagnostics(self):
        return dict(self.diagnostic_counts)

    def positions_at(self, source_ns, required_joints=None):
        """Only measured/interpolated joints at this actual acquisition time.

        A single exact sample covers its own instant. Otherwise both samples
        must bracket the query and their source-time gap must meet the bound.
        Unknown times, unknown joints, and missing samples yield absent keys.
        """
        if not _stamp(source_ns):
            return {}
        required = None if required_joints is None else frozenset(required_joints)
        output = {}
        for trajectory in self.trajectories:
            if required is not None and trajectory.name not in required:
                continue
            position = trajectory.position_at(int(source_ns), self.max_gap_ns)
            if position is not None:
                output[trajectory.name] = position
        return output

    def positions_at_many(self, times_ns, required_joints=None):
        """Preserve ray order and exact times; no time quantization or fallback."""
        required = None if required_joints is None else tuple(required_joints)
        return tuple(self.positions_at(value, required) for value in times_ns)

    joints_at_many = positions_at_many

    def positions_arrays_at(self, times_ns, required_joints=None):
        """Per-ray read-only float arrays, with NaN for absent joint evidence.

        Bracket lookup is vectorized independently for each measured joint.
        Source nanoseconds remain integers through subtraction, including for
        large Unix epochs. Unknown times have no coverage, and neither missing
        joints nor unbracketed rays acquire a default or held posture.
        """
        import numpy as np

        # Preserve explicit integer identity in mixed Python sequences; a
        # float timestamp must not become a valid, rounded source nanosecond.
        if isinstance(times_ns, np.ndarray):
            times = times_ns
        else:
            try:
                times = np.asarray(tuple(times_ns), dtype=object)
            except TypeError as error:
                raise ValueError('invalid_joint_pose_query_times') from error
        if times.ndim != 1:
            raise ValueError('invalid_joint_pose_query_times')
        if times.dtype.kind in 'iu':
            valid = times > 0
            if times.dtype.kind == 'u':
                valid &= times < np.uint64(2**63)
            queries = times.astype(np.int64, copy=False)
        elif times.dtype.kind == 'O':
            valid = np.fromiter((_stamp(value) for value in times), dtype=bool, count=len(times))
            queries = np.fromiter((int(value) if covered else 0
                for value, covered in zip(times, valid)), dtype=np.int64, count=len(times))
        else:
            valid = np.zeros(len(times), dtype=bool)
            queries = np.zeros(len(times), dtype=np.int64)
        required = None if required_joints is None else frozenset(required_joints)

        def remainder(values):
            # NumPy's modulo uses a different +/-pi tie convention from
            # math.remainder. Reproduce its nearest-even quotient, including
            # multi-turn readings, without subtracting huge float multiples.
            period = 2.*math.pi
            result = np.fmod(values, period)
            result = np.where(result > math.pi, result-period, result)
            result = np.where(result < -math.pi, result+period, result)
            ties = np.abs(result) == math.pi
            if np.any(ties):
                result[ties] = np.where(np.remainder(values[ties], 2.*period) > period,
                                        -math.pi, math.pi)
            return result

        output = {}
        for trajectory in self.trajectories:
            if required is not None and trajectory.name not in required:
                continue
            positions = np.full(len(times), np.nan, dtype=float)
            if trajectory.source_ns and np.any(valid):
                stamps = np.asarray(trajectory.source_ns, dtype=np.int64)
                values = np.asarray(trajectory.positions, dtype=float)
                right = np.searchsorted(stamps, queries, side='left')
                lookup = np.minimum(right, len(stamps)-1)
                exact = valid & (right < len(stamps)) & (stamps[lookup] == queries)
                positions[exact] = values[lookup[exact]]
                between = valid & ~exact & (right > 0) & (right < len(stamps))
                rows = np.flatnonzero(between)
                if len(rows):
                    upper = right[rows]
                    lower = upper-1
                    gaps = stamps[upper]-stamps[lower]
                    covered = gaps <= self.max_gap_ns
                    rows, lower, upper, gaps = (
                        rows[covered], lower[covered], upper[covered], gaps[covered])
                    fraction = (queries[rows]-stamps[lower])/gaps
                    left, right_values = values[lower], values[upper]
                    if trajectory.joint_type == 'continuous':
                        difference = remainder(remainder(right_values)-remainder(left))
                        positions[rows] = left + fraction*difference
                    else:
                        positions[rows] = (1.-fraction)*left + fraction*right_values
            positions.flags.writeable = False
            output[trajectory.name] = positions
        return output

    def positions_for_offsets(self, start_ns, offsets_ns, required_joints=None):
        """Evaluate start + each ray's integer nanosecond acquisition offset."""
        required = None if required_joints is None else tuple(required_joints)
        output = []
        for offset in offsets_ns:
            valid = (_stamp(start_ns) and isinstance(offset, Integral)
                     and not isinstance(offset, bool) and offset >= 0)
            source_ns = int(start_ns)+int(offset) if valid else 0
            output.append(self.positions_at(source_ns, required))
        return tuple(output)


class JointPoseHistory:
    """Thread-safe bounded collector restricted to the model's active joints.

    Out-of-order source samples within the retained window are inserted in
    source order. An identical duplicate is a no-op. A conflicting value at an
    existing source time never overwrites that value, including after a worker
    has acquired a snapshot. Disjoint named samples at the same time can merge.
    """
    def __init__(self, joint_types, *, history_ns=2_000_000_000, max_samples=256,
                 max_gap_ns=60_000_000, identity=None):
        if (not isinstance(joint_types, Mapping)
                or len(joint_types) > _MAX_MODEL_JOINTS
                or any(not _name(name) or not isinstance(kind, str) or kind not in _JOINT_TYPES
                       for name, kind in joint_types.items())):
            raise ValueError('invalid_robot_model_joint_types')
        if (not isinstance(history_ns, Integral) or isinstance(history_ns, bool)
                or not 1 <= history_ns <= 10_000_000_000
                or not isinstance(max_gap_ns, Integral) or isinstance(max_gap_ns, bool)
                or not 1 <= max_gap_ns <= history_ns
                or not isinstance(max_samples, Integral) or isinstance(max_samples, bool)
                or not 2 <= max_samples <= 4096):
            raise ValueError('invalid_joint_pose_history_limits')
        self.history_ns, self.max_samples, self.max_gap_ns = (
            int(history_ns), int(max_samples), int(max_gap_ns))
        self._joint_types = {name: kind for name, kind in joint_types.items() if kind != 'fixed'}
        self._fixed_joint_names = frozenset(name for name, kind in joint_types.items() if kind == 'fixed')
        self._samples = []
        self._identity = _freeze_identity(identity)
        self._generation = 0
        self._latest_source_ns = 0
        self._lock = Lock()
        self._diagnostics = dict(accepted_joint_updates=0, duplicate_joint_updates=0,
            conflicting_joint_updates=0, unknown_joint_names=0,
            invalid_joint_updates=0, invalid_joint_positions=0,
            expired_joint_updates=0, identity_resets=0)

    @property
    def diagnostics(self):
        with self._lock:
            return dict(self._diagnostics)

    def reset(self, identity=None):
        """Clear every retained joint on an identity/barrier reset."""
        identity = _freeze_identity(identity)
        with self._lock:
            self._samples.clear()
            self._latest_source_ns = 0
            self._identity = identity
            self._generation += 1
            self._diagnostics['identity_resets'] += 1

    def add_joint_state(self, source_ns, names, positions, *, unit='radian'):
        """Accept an explicit stamped named SI-unit JointState, never SDK JSON.

        ``unit`` states the angular contract; prismatic positions retain their
        JointState metre contract. Invalid individual positions and unrelated
        names cannot manufacture a zero angle for a model joint. Unknown source
        time or malformed parallel arrays reject this update without throwing.
        """
        try:
            if (not _stamp(source_ns) or unit != 'radian'
                    or isinstance(names, (str, bytes, Mapping))
                    or isinstance(positions, (str, bytes, Mapping))):
                raise ValueError
            # ROS arrays are finite, but the pure contract also bounds a caller
            # passing an oversized or unbounded iterable before copying it.
            names = tuple(islice(names, 2*_MAX_MODEL_JOINTS+1))
            positions = tuple(islice(positions, 2*_MAX_MODEL_JOINTS+1))
            if (len(names) != len(positions) or not 0 < len(names) <= 2*_MAX_MODEL_JOINTS
                    or any(not _name(name) for name in names) or len(set(names)) != len(names)):
                raise ValueError
        except (TypeError, ValueError):
            with self._lock:
                self._diagnostics['invalid_joint_updates'] += 1
            return False
        values, unknown, invalid = {}, 0, 0
        for name, position in zip(names, positions):
            if name in self._fixed_joint_names:
                continue
            if name not in self._joint_types:
                unknown += 1
            else:
                try:
                    if not isinstance(position, Real) or isinstance(position, bool):
                        raise ValueError
                    value = float(position)
                    if not math.isfinite(value):
                        raise ValueError
                except (TypeError, ValueError, OverflowError):
                    invalid += 1
                else:
                    values[name] = value
        with self._lock:
            self._diagnostics['unknown_joint_names'] += unknown
            self._diagnostics['invalid_joint_positions'] += invalid
            if not values:
                return False
            source_ns = int(source_ns)
            cutoff = self._latest_source_ns-self.history_ns
            if (source_ns < cutoff or (len(self._samples) >= self.max_samples
                    and source_ns < self._samples[0].source_ns)):
                self._diagnostics['expired_joint_updates'] += 1
                return False
            stamps = [sample.source_ns for sample in self._samples]
            index = bisect_left(stamps, source_ns)
            if index < len(stamps) and stamps[index] == source_ns:
                previous = dict(self._samples[index].positions)
                additions = {}
                for name, value in values.items():
                    if name not in previous:
                        additions[name] = value
                    elif previous[name] == value:
                        self._diagnostics['duplicate_joint_updates'] += 1
                    else:
                        self._diagnostics['conflicting_joint_updates'] += 1
                if not additions:
                    return False
                previous.update(additions)
                self._samples[index] = JointSample(source_ns, tuple(sorted(previous.items())))
            else:
                self._samples.insert(index, JointSample(source_ns, tuple(sorted(values.items()))))
            self._latest_source_ns = max(self._latest_source_ns, source_ns)
            cutoff = self._latest_source_ns-self.history_ns
            while self._samples and self._samples[0].source_ns < cutoff:
                self._samples.pop(0)
            excess = len(self._samples)-self.max_samples
            if excess > 0:
                del self._samples[:excess]
            self._diagnostics['accepted_joint_updates'] += 1
            return True

    def snapshot(self):
        """Capture one generation without sharing any mutable history storage."""
        with self._lock:
            samples = tuple(self._samples)
            trajectories = []
            for name, kind in sorted(self._joint_types.items()):
                measured = tuple((sample.source_ns, value)
                    for sample in samples for candidate, value in sample.positions
                    if candidate == name)
                trajectories.append(JointTrajectory(name, kind,
                    tuple(item[0] for item in measured), tuple(item[1] for item in measured)))
            return JointPoseSnapshot(self._identity, self._generation,
                tuple(sorted(self._joint_types.items())), samples, tuple(trajectories),
                self.max_gap_ns, tuple(sorted(self._diagnostics.items())))
