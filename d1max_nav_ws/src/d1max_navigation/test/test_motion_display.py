from d1max_navigation.status_display import motion_label


def label(**kw):
    return motion_label({'mode': 'live', 'wall_time': 100., **kw}, 10., 10.1, 100.1)


def test_disabled_and_locked_are_different():
    assert label(motion_enabled=False) == 'MOTION DISABLED'
    assert label(motion_enabled=True, armed=False, reason='not_armed') == 'DRIVE LOCKED: not_armed'


def test_armed_is_visible_even_without_command():
    assert label(motion_enabled=True, armed=True, allowed=False) == 'DRIVE ARMED - IDLE'
    assert label(motion_enabled=True, armed=True, allowed=True) == 'DRIVE ARMED - COMMAND ACTIVE'


def test_missing_stale_wrong_mode_do_not_claim_disabled():
    assert 'UNKNOWN' in motion_label(None, None, 10., 100.)
    assert 'UNKNOWN' in motion_label({'wall_time': 90.}, 10., 10., 100.)
    assert 'UNKNOWN' in label(mode='simulation')
