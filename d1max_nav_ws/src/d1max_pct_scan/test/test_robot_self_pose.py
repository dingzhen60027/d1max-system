"""Source-stamped articulated pose contract, independent of ROS and the SDK."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import FrozenInstanceError, dataclass
from itertools import count
import math
from threading import Event

import numpy as np
import pytest

from d1max_pct_scan.robot_self_pose import JointPoseHistory


T = 10_000_000_000
DT = 20_000_000


def history(**kwargs):
    return JointPoseHistory({'hip': 'revolute', 'head': 'continuous'}, **kwargs)


def add(model, stamp, hip, head=0.):
    return model.add_joint_state(stamp, ['hip', 'head'], [hip, head])


def test_same_scan_uses_each_exact_ray_acquisition_offset_and_keeps_order():
    model = history()
    add(model, T, 0.)
    add(model, T+DT, 1.)
    result = model.snapshot().positions_for_offsets(T, [DT, 0, DT//2, DT//2])
    assert [sample['hip'] for sample in result] == [1., 0., .5, .5]
    assert model.snapshot().joints_at_many([T, T+DT]) == (
        {'hip': 0., 'head': 0.}, {'hip': 1., 'head': 0.})


def test_out_of_order_duplicate_and_conflict_cannot_overwrite_source():
    model = history()
    assert add(model, T+DT, 1.)
    assert add(model, T, 0.)
    captured = model.snapshot()
    assert not add(model, T, 0.)
    assert not add(model, T, 99.)
    assert captured.positions_at(T+DT//2)['hip'] == .5
    assert model.snapshot().positions_at(T+DT//2)['hip'] == .5
    assert model.snapshot().oldest_source_ns == T
    assert model.diagnostics['duplicate_joint_updates'] == 3
    assert model.diagnostics['conflicting_joint_updates'] == 1


def test_disjoint_named_joint_messages_merge_without_mutating_existing_snapshot():
    model = history()
    assert model.add_joint_state(T, ['hip'], [.25])
    captured = model.snapshot()
    assert model.add_joint_state(T, ['head'], [.5])
    assert captured.positions_at(T) == {'hip': .25}
    assert model.snapshot().positions_at(T) == {'hip': .25, 'head': .5}


def test_no_extrapolation_hold_or_default_zero_but_exact_sample_is_real_coverage():
    model = history()
    assert model.snapshot().positions_at(T) == {}
    add(model, T, 2.)
    captured = model.snapshot()
    assert captured.positions_at(T) == {'hip': 2., 'head': 0.}
    assert captured.positions_at(T-1) == {}
    assert captured.positions_at(T+1) == {}
    add(model, T+DT, 3.)
    assert model.snapshot().positions_at(T+DT+1) == {}


def test_sample_gap_bound_applies_to_each_joint_actual_measurements():
    model = history(max_gap_ns=DT)
    add(model, T, 0.)
    model.add_joint_state(T+DT, ['hip'], [1.])
    add(model, T+2*DT, 2.)
    captured = model.snapshot()
    assert captured.positions_at(T+DT//2) == {'hip': .5}
    assert captured.positions_at(T+DT) == {'hip': 1.}
    assert captured.positions_at(T+DT+DT//2) == {'hip': 1.5}
    assert captured.positions_at(T+2*DT) == {'hip': 2., 'head': 0.}


def test_missing_joint_skips_only_its_affected_link_dependency():
    model = history()
    model.add_joint_state(T, ['hip'], [0.])
    model.add_joint_state(T+DT, ['hip'], [1.])
    poses = model.snapshot().positions_at(T+DT//2, ('hip', 'head', 'foreign'))
    assert poses == {'hip': .5}
    chains = {'fixed_body': (), 'leg': ('hip',), 'head_link': ('head',)}
    usable = {link for link, chain in chains.items() if all(joint in poses for joint in chain)}
    assert usable == {'fixed_body', 'leg'}


def test_continuous_shortest_arc_and_revolute_actual_measured_arc_differ():
    model = history()
    add(model, T, 3., 3.)
    add(model, T+DT, -3., -3.)
    values = model.snapshot().positions_at(T+DT//2)
    assert values['hip'] == 0.
    assert values['head'] == pytest.approx(math.pi)
    assert model.snapshot().positions_at(T+DT)['head'] == -3.


def test_continuous_direction_and_prismatic_real_coordinates():
    model = JointPoseHistory({'head': 'continuous', 'slide': 'prismatic'})
    model.add_joint_state(T, ['head', 'slide'], [-3., .1])
    model.add_joint_state(T+DT, ['head', 'slide'], [3., .3])
    values = model.snapshot().positions_at(T+DT//2)
    assert values['head'] == pytest.approx(-math.pi)
    assert values['slide'] == pytest.approx(.2)


def test_history_is_bounded_by_source_time_and_by_sample_count():
    model = history(history_ns=3*DT, max_samples=3, max_gap_ns=DT)
    for index in range(10):
        add(model, T+index*DT, float(index))
    captured = model.snapshot()
    assert captured.sample_count == 3
    assert captured.oldest_source_ns == T+7*DT
    assert captured.latest_source_ns == T+9*DT
    assert not add(model, T, -1.)
    assert not add(model, T+6*DT, -1.)
    assert model.diagnostics['expired_joint_updates'] == 2
    assert captured.positions_at(T) == {}
    # A source-time advance removes all earlier history, even if capacity is free.
    add(model, T+20*DT, 20.)
    assert model.snapshot().sample_count == 1
    assert model.snapshot().positions_at(T+9*DT) == {}


@dataclass(frozen=True)
class Context:
    session_id: str
    epoch: int


def test_identity_reset_clears_history_and_old_snapshot_keeps_original_generation():
    model = history(identity=Context('one', 1))
    add(model, T, 0.)
    add(model, T+DT, 1.)
    old = model.snapshot()
    model.reset(Context('one', 2))
    current = model.snapshot()
    assert current.positions_at(T) == {}
    assert current.identity == Context('one', 2)
    assert current.generation == old.generation+1
    assert old.identity == Context('one', 1)
    assert old.positions_at(T+DT//2)['hip'] == .5
    add(model, T, 5.)
    assert model.snapshot().positions_at(T)['hip'] == 5.


def test_snapshot_and_input_arrays_are_isolated_across_real_worker_thread():
    model = history()
    names, positions = ['hip', 'head'], [0., 0.]
    model.add_joint_state(T, names, positions)
    add(model, T+DT, 1.)
    captured = model.snapshot()
    ready, release = Event(), Event()

    def query_worker():
        ready.set()
        assert release.wait(timeout=2.)
        result = captured.positions_at(T+DT//2)
        result['hip'] = 900.
        diagnostics = captured.diagnostics
        diagnostics['accepted_joint_updates'] = 900
        return captured.positions_at(T+DT//2)

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(query_worker)
        assert ready.wait(timeout=2.)
        names[0], positions[0] = 'changed', 999.
        model.reset(('new', 2))
        add(model, T, 10.)
        add(model, T+DT, 20.)
        release.set()
        assert future.result(timeout=2.)['hip'] == .5
    assert model.snapshot().positions_at(T+DT//2)['hip'] == 15.
    assert captured.diagnostics['accepted_joint_updates'] == 2
    with pytest.raises(FrozenInstanceError):
        captured.max_gap_ns = 100
    with pytest.raises(FrozenInstanceError):
        captured.samples[0].source_ns = 100
    with pytest.raises(TypeError):
        captured.samples[0].positions[0] = ('hip', 123.)


def test_mutable_identity_is_copied_into_immutable_data():
    identity = {'session': ['one', 1]}
    model = history(identity=identity)
    identity['session'][0] = 'changed'
    assert model.snapshot().identity == frozenset((('session', ('one', 1)),))


@pytest.mark.parametrize('stamp', [None, 0, -1, 1.5, True, 2**63])
def test_unknown_source_time_never_uses_receipt_time(stamp):
    model = history()
    assert not model.add_joint_state(stamp, ['hip'], [0.])
    assert model.snapshot().sample_count == 0
    assert model.snapshot().positions_at(stamp) == {}


@pytest.mark.parametrize('names,positions,unit', [
    (['hip'], [0., 1.], 'radian'), (['hip', 'hip'], [0., 1.], 'radian'),
    (['hip'], [90.], 'degree'), ({'hip': 0.}, [0.], 'radian'),
    (['hip'], {'receipt_stamp': T, 'joint_angles': [0.]}, 'radian')])
def test_malformed_units_or_sdk_like_mapping_are_not_joint_state(names, positions, unit):
    model = history()
    assert not model.add_joint_state(T, names, positions, unit=unit)
    assert model.snapshot().positions_at(T) == {}
    assert model.diagnostics['invalid_joint_updates'] == 1


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), True, '0'])
def test_bad_individual_joint_does_not_remove_other_valid_joint(invalid):
    model = history()
    assert model.add_joint_state(T, ['hip', 'head'], [invalid, .5])
    assert model.snapshot().positions_at(T) == {'head': .5}
    assert model.diagnostics['invalid_joint_positions'] == 1


def test_names_are_matched_explicitly_and_foreign_names_are_never_added():
    model = history()
    assert model.add_joint_state(T, ['head', 'foreign', 'hip'], [.5, 100., .25])
    assert model.snapshot().positions_at(T) == {'hip': .25, 'head': .5}
    assert model.snapshot().positions_at(T, ['foreign']) == {}
    assert model.diagnostics['unknown_joint_names'] == 1


def test_invalid_ray_offset_is_local_to_that_ray():
    model = history()
    add(model, T, .25)
    result = model.snapshot().positions_for_offsets(T, [0, None, -1, .5, True, 2**63])
    assert result == ({'hip': .25, 'head': 0.}, {}, {}, {}, {}, {})


@pytest.mark.parametrize('kwargs', [dict(history_ns=0), dict(max_samples=1),
    dict(max_samples=4097), dict(max_gap_ns=0), dict(max_gap_ns=3_000_000_000)])
def test_invalid_history_configuration_fails_at_preparation(kwargs):
    with pytest.raises(ValueError, match='history_limits'):
        history(**kwargs)


def test_invalid_model_joint_type_fails_at_preparation():
    with pytest.raises(ValueError, match='model_joint_types'):
        JointPoseHistory({'hip': 'floating'})


def test_full_urdf_joint_mapping_can_include_fixed_joints_without_fake_angles():
    model = JointPoseHistory({'mount': 'fixed', 'hip': 'revolute'})
    assert not model.add_joint_state(T, ['mount'], [0.])
    assert model.snapshot().positions_at(T) == {}
    assert model.add_joint_state(T, ['mount', 'hip'], [0., .5])
    assert model.snapshot().positions_at(T) == {'hip': .5}
    assert model.diagnostics['unknown_joint_names'] == 0


def test_unbounded_input_arrays_are_rejected_after_bounded_copy():
    model = history()
    assert not model.add_joint_state(T, ('hip' for _ in count()), count())
    assert model.snapshot().sample_count == 0


def test_numeric_overflow_cannot_kill_pose_queries_or_other_valid_joints():
    model = history()
    assert model.add_joint_state(T-1, ['hip', 'head'], [10**1000, .5])
    assert model.snapshot().positions_at(T-1) == {'head': .5}
    add(model, T, 1e308, 1e308)
    add(model, T+DT, -1e308, -1e308)
    values = model.snapshot().positions_at(T+DT//2)
    assert values['hip'] == 0.
    assert math.isfinite(values['head'])


def assert_arrays_match_scalar(snapshot, times, required_joints=None):
    result = snapshot.positions_arrays_at(times, required_joints)
    scalar = snapshot.positions_at_many(times, required_joints)
    for name, actual in result.items():
        expected = np.array([position.get(name, np.nan) for position in scalar])
        np.testing.assert_allclose(actual, expected, rtol=1e-14, atol=1e-14, equal_nan=True)
        assert actual.shape == (len(times),) and actual.dtype == np.dtype(float)
        assert not actual.flags.writeable
    return result


def test_arrays_match_exact_between_missing_unordered_duplicate_ray_queries():
    model = history(max_gap_ns=DT)
    add(model, T, 0., 3.)
    model.add_joint_state(T+DT, ['hip'], [1.])
    add(model, T+2*DT, 2., -3.)
    times = np.array([T+DT, T, T+DT//2, T+DT//2, T-1,
                      T+3*DT, T+2*DT, T+DT+DT//2], dtype=np.int64)
    result = assert_arrays_match_scalar(model.snapshot(), times)
    np.testing.assert_allclose(result['hip'], [1., 0., .5, .5, np.nan, np.nan, 2., 1.5], equal_nan=True)
    assert np.isnan(result['head'][[0, 2, 3, 4, 5, 7]]).all()


def test_arrays_do_not_round_large_epoch_nanoseconds_or_quantize_intermediate_queries():
    start = 1_790_412_195_000_000_000
    model = history(max_gap_ns=DT)
    add(model, start, 0.)
    add(model, start+3, 3.)
    times = np.array([start+2, start+1, start+3, start, start+4], dtype=np.int64)
    result = assert_arrays_match_scalar(model.snapshot(), times)
    np.testing.assert_allclose(result['hip'], [2., 1., 3., 0., np.nan], equal_nan=True)


def test_arrays_missing_joint_and_uncovered_exact_single_sample_are_nan_without_default_angle():
    model = history()
    model.add_joint_state(T, ['hip'], [.5])
    times = np.array([T-1, T, T+1], dtype=np.int64)
    result = assert_arrays_match_scalar(model.snapshot(), times)
    np.testing.assert_allclose(result['hip'], [np.nan, .5, np.nan], equal_nan=True)
    assert np.isnan(result['head']).all()
    assert set(model.snapshot().positions_arrays_at(times, ['head', 'foreign'])) == {'head'}


@pytest.mark.parametrize('left,right', [(3., -3.), (-3., 3.), (0., math.pi),
    (0., -math.pi), (0., 3.*math.pi), (0., -3.*math.pi),
    (2.*math.pi, 3.*math.pi), (-2.*math.pi, -3.*math.pi), (1e308, -1e308)])
def test_arrays_continuous_shortest_arc_matches_scalar_including_pi_ties_and_extremes(left, right):
    model = history()
    add(model, T, left, left)
    add(model, T+DT, right, right)
    times = np.array([T+DT//4, T+DT//2, T+3*DT//4, T, T+DT], dtype=np.int64)
    assert_arrays_match_scalar(model.snapshot(), times)


def test_arrays_prismatic_coordinates_and_gap_boundary_match_scalar():
    model = JointPoseHistory({'slide': 'prismatic'}, max_gap_ns=DT)
    model.add_joint_state(T, ['slide'], [.1])
    model.add_joint_state(T+DT, ['slide'], [.3])
    model.add_joint_state(T+3*DT, ['slide'], [.7])
    times = np.array([T+DT//2, T+2*DT, T+3*DT], dtype=np.int64)
    result = assert_arrays_match_scalar(model.snapshot(), times)
    np.testing.assert_allclose(result['slide'], [.2, np.nan, .7], equal_nan=True)


@pytest.mark.parametrize('times', [
    [T, None, 0, -1, True, T+.5, 2**63, T+DT],
    np.array([T, 2**63, 2**64-1], dtype=np.uint64),
    np.array([float(T), float('nan'), float('inf')]),
    np.array([True, False])])
def test_arrays_invalid_source_times_are_local_missing_evidence(times):
    model = history()
    add(model, T, 0.)
    add(model, T+DT, 1.)
    assert_arrays_match_scalar(model.snapshot(), times)


def test_arrays_are_caller_owned_readonly_and_remain_isolated_from_later_history_changes():
    model = history()
    add(model, T, 0.)
    add(model, T+DT, 1.)
    snapshot = model.snapshot()
    times = np.array([T, T+DT//2, T+DT], dtype=np.int64)
    first = snapshot.positions_arrays_at(times)
    with pytest.raises(ValueError):
        first['hip'][1] = 900.
    # Even a caller deliberately re-enabling its own array cannot mutate any
    # retained sample or a separate query result.
    first['hip'].flags.writeable = True
    first['hip'][1] = 900.
    model.reset(('replacement', 2))
    add(model, T, 10.)
    add(model, T+DT, 20.)
    np.testing.assert_array_equal(snapshot.positions_arrays_at(times)['hip'], [0., .5, 1.])
    np.testing.assert_array_equal(model.snapshot().positions_arrays_at(times)['hip'], [10., 15., 20.])


def test_arrays_empty_times_and_empty_fixed_only_model_are_valid():
    result = history().snapshot().positions_arrays_at(np.array([], dtype=np.int64))
    assert set(result) == {'hip', 'head'}
    assert all(value.shape == (0,) and not value.flags.writeable for value in result.values())
    assert JointPoseHistory({'mount': 'fixed'}).snapshot().positions_arrays_at([T]) == {}


@pytest.mark.parametrize('times', [T, np.array([[T]]), [[T]], None])
def test_arrays_require_one_dimensional_ray_time_sequence(times):
    with pytest.raises(ValueError, match='query_times'):
        history().snapshot().positions_arrays_at(times)
