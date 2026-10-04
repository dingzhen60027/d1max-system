from d1max_pct_scan.live_view_reload import presentation_exit_is_nonfatal


def session(**changes):
    return dict(ui_lifetime_policy='independent', motion_control_enabled=False,
                preview_freeze_owner='supervisor_child', **changes)


def test_display_exit_does_not_cancel_a_supervisor_owned_preview():
    for name in ('rviz', 'view'):
        assert presentation_exit_is_nonfatal(name, session())


def test_critical_child_failures_are_never_hidden_as_ui_detach():
    for name in ('scan', 'navigator', 'localization', 'bt_adapters', 'perception', 'motion_stack'):
        assert not presentation_exit_is_nonfatal(name, session())


def test_legacy_or_view_owned_freeze_does_not_gain_detach_by_accident():
    for value in ({}, {'ui_lifetime_policy': 'independent'},
                  {'ui_lifetime_policy': 'independent', 'motion_control_enabled': False,
                   'preview_freeze_owner': 'view'}):
        assert not presentation_exit_is_nonfatal('rviz', value)


def test_motion_owner_survives_display_but_is_not_implicitly_created():
    value = dict(ui_lifetime_policy='independent', motion_control_enabled=True,
                 preview_freeze_owner='motion_coordinator')
    assert presentation_exit_is_nonfatal('rviz', value)
    assert not presentation_exit_is_nonfatal('motion_coordinator', value)
