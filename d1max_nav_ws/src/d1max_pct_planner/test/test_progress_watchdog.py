import math

import pytest

from d1max_pct_planner.progress_watchdog import TranslationProgressWatchdog


def test_stationary_or_turn_in_place_times_out_at_twenty_seconds():
    watch = TranslationProgressWatchdog()
    watch.activate((1.0, 2.0), 100.0)
    assert not watch.observe((1.0, 2.0), 106.0)
    assert not watch.observe((1.0, 2.0), 119.9)
    assert watch.observe((1.0, 2.0), 120.0)
    assert watch.snapshot()['seconds_without_translation_progress'] == 20.0


def test_translation_in_small_increments_refreshes_the_anchor():
    watch = TranslationProgressWatchdog()
    watch.activate((0.0, 0.0), 0.0)
    assert not watch.observe((0.04, 0.0), 5.0)
    assert not watch.observe((0.08, 0.0), 10.0)
    assert not watch.observe((0.12, 0.0), 15.0)
    assert watch.snapshot()['seconds_without_translation_progress'] == 0.0
    assert not watch.observe((0.12, 0.0), 34.9)
    assert watch.observe((0.12, 0.0), 35.0)


def test_jitter_does_not_accumulate_into_false_progress():
    watch = TranslationProgressWatchdog()
    watch.activate((0.0, 0.0), 0.0)
    for tick in range(1, 200):
        assert not watch.observe((0.04 if tick % 2 else -0.04, 0.0), tick * 0.1)
    assert watch.observe((0.04, 0.0), 20.0)


def test_motion_away_from_goal_still_counts_as_translation():
    watch = TranslationProgressWatchdog()
    watch.activate((0.0, 0.0), 0.0)
    assert not watch.observe((-0.12, 0.0), 15.0)
    assert not watch.observe((-0.12, -0.12), 30.0)
    assert watch.snapshot()['seconds_without_translation_progress'] == 0.0


def test_inactive_time_does_not_count_and_new_task_resets():
    watch = TranslationProgressWatchdog()
    assert not watch.observe((0.0, 0.0), 1000.0)
    watch.activate((0.0, 0.0), 1000.0)
    assert watch.observe((0.0, 0.0), 1020.0)
    # No automatic reactivation from later movement after a blocked task.
    assert watch.observe((5.0, 5.0), 1021.0)
    watch.deactivate()
    assert not watch.observe((5.0, 5.0), 2000.0)
    assert watch.snapshot()['active'] is False
    watch.activate((5.0, 5.0), 2000.0)
    assert not watch.observe((5.0, 5.0), 2019.0)
    assert watch.observe((5.0, 5.0), 2020.0)


def test_explicit_replacement_activation_gets_fresh_budget():
    watch = TranslationProgressWatchdog()
    watch.activate((0.0, 0.0), 0.0)
    assert not watch.observe((0.0, 0.0), 19.0)
    watch.activate((0.0, 0.0), 19.0)
    assert not watch.observe((0.0, 0.0), 38.0)
    assert watch.observe((0.0, 0.0), 39.0)


@pytest.mark.parametrize('timeout,distance', [(0, 0.1), (-1, 0.1), (math.inf, 0.1),
                                            (20, 0), (20, -1), (20, math.nan)])
def test_invalid_settings_fail_closed(timeout, distance):
    with pytest.raises(ValueError):
        TranslationProgressWatchdog(timeout, distance)


def test_invalid_sample_and_steady_clock_rollback_rejected():
    watch = TranslationProgressWatchdog()
    watch.activate((0.0, 0.0), 10.0)
    with pytest.raises(ValueError, match='finite'):
        watch.observe((math.nan, 0.0), 11.0)
    with pytest.raises(ValueError, match='backwards'):
        watch.observe((0.0, 0.0), 9.0)
