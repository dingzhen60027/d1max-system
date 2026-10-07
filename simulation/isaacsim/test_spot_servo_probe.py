from spot_servo_probe import phases, stop_completion


def rows(linear, angular, count=1501):
    return [dict(source_ns=(i + 1) * 2_000_000, linear_velocity=linear,
                 angular_velocity=angular) for i in range(count)]


def test_stop_requires_original_full_norms_not_only_xy_or_yaw():
    assert stop_completion(rows([0., 0., .031], [0., 0., 0.])) is None
    assert stop_completion(rows([0., 0., 0.], [.051, 0., 0.])) is None
    assert stop_completion(rows([0., 0., .02], [.04, 0., 0.])) == 1.


def test_a_bad_physics_sample_restarts_continuous_stop_window():
    values = rows([0., 0., 0.], [0., 0., 0.])
    values[499]['angular_velocity'] = [.06, 0., 0.]
    assert stop_completion(values) == 2.


def test_default_probe_retains_two_histories_and_original_command_cap():
    schedule = phases()
    names = [p[0] for p in schedule]
    assert names.index('park') < names.index('low_after_park')
    assert names.index('walk') < names.index('stop_walk') < names.index('low_after_walk')
    assert max(abs(demand) for _, _, demand in schedule) == .15
    assert all(count > 0 for _, count, _ in schedule)
