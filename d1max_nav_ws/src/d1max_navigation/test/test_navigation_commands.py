import pytest

from d1max_navigation.navigation_commands import (
    validate_context, validated_gate, goal_allowed, validate_goal, observe_action, motion_admission)
from d1max_navigation.navigation_session import OWNER
from d1max_navigation.simulation import OccupancyWorld


def context():
    session = {'navigation_session_id': 'a'*32, 'version_id': 'grid-a', 'mode': 'sim'}
    unit = {'ActiveState': 'active', 'Description': OWNER+'sim:grid-a:'+'a'*32}
    return unit, session


def test_only_current_owned_generation_can_mutate():
    unit, session = context()
    assert validate_context(unit, session, 'a'*32, 'grid-a') == ('a'*32, 'grid-a')
    for ident, version in [('b'*32, 'grid-a'), ('a'*32, 'grid-b')]:
        with pytest.raises(ValueError):
            validate_context(unit, session, ident, version)
    for key, value in [('ActiveState', 'inactive'), ('Description', OWNER+'sim:grid-a')]:
        with pytest.raises(ValueError):
            validate_context(dict(unit, **{key: value}), session)


def test_status_from_old_or_wrong_router_generation_is_not_authorization():
    _, session = context()
    gate = {**session, 'map_version_id': 'grid-a', 'mode': 'simulation', 'wall_time': 100.}
    assert validated_gate(gate, session, 100.) is gate
    for key, value in [('navigation_session_id', 'b'*32), ('map_version_id', 'other'),
                       ('mode', 'live'), ('wall_time', 90.), ('wall_time', 101.)]:
        assert validated_gate(dict(gate, **{key: value}), session, 100.) is None


def test_goal_requires_explicit_live_arm_and_capability():
    session = {'mode': 'live', 'enable_motion': True}
    gate = {'armed': True, 'motion_enabled': True, 'reason': 'command_missing_or_stale'}
    goal_allowed(gate, session)
    for key, value in [('armed', False), ('motion_enabled', False), ('reason', 'scan_missing_or_stale')]:
        with pytest.raises(ValueError):
            goal_allowed(dict(gate, **{key: value}), session)
    with pytest.raises(ValueError):
        goal_allowed(gate, dict(session, enable_motion=False))


def test_goal_cannot_be_outside_map_unknown_or_nonfinite():
    world = OccupancyWorld([0]*400, 20, 20, .1)
    assert validate_goal(1., 1., 0., world) == (1., 1., 0.)
    for target in [(0., 0., 0.), (100., 0., 0.), (1., 1., float('nan'))]:
        with pytest.raises(ValueError):
            validate_goal(*target, world)
    world.blocked[10, 10] = True
    with pytest.raises(ValueError):
        validate_goal(1., 1., 0., world)


def action_status(status, stamp, ident=1):
    from types import SimpleNamespace as NS
    return NS(status=status, goal_info=NS(stamp=NS(sec=stamp, nanosec=0),
                                         goal_id=NS(uuid=[ident]*16)))


@pytest.mark.parametrize('active', [1, 2, 3])
@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_newer_terminal_goal_never_hides_older_active_goal(active, terminal):
    actions, seen = {}, set()
    selected = observe_action(actions, seen, 'navigate_to_pose',
                              [action_status(active, 10), action_status(terminal, 20, 2)])
    assert selected['status'] == active
    assert selected['goal_id'] == '01'*16
    assert seen == {'navigate_to_pose'}


def test_empty_status_removes_old_action_but_is_not_unobserved():
    actions, seen = {}, set()
    observe_action(actions, seen, 'navigate_to_pose', [action_status(2, 10)])
    observe_action(actions, seen, 'follow_waypoints', [action_status(2, 11, 2)])
    selected = observe_action(actions, seen, 'follow_waypoints', [])
    assert 'follow_waypoints' not in actions
    assert 'follow_waypoints' in seen
    assert selected['action'] == 'navigate_to_pose'
    assert observe_action(actions, seen, 'navigate_to_pose', []) is None
    assert actions == {}
    assert seen == {'navigate_to_pose', 'follow_waypoints'}


@pytest.mark.parametrize('operation', ['arm', 'goal'])
@pytest.mark.parametrize('name', ['navigate_to_pose', 'navigate_through_poses', 'follow_waypoints'])
def test_action_refresh_during_discovery_revokes_pre_wait_admission(operation, name):
    _, session = context()
    gate = {'navigation_session_id': session['navigation_session_id'], 'map_version_id': 'grid-a',
            'mode': 'simulation', 'wall_time': 100., 'armed': True, 'reason': 'simulation_only'}
    actions, seen = {}, set()
    observe_action(actions, seen, name, [action_status(4, 10)])
    assert motion_admission(operation, gate, session, actions, 100.) is gate
    # Equivalent to callback dispatch while waiting for cancellation/discovery.
    observe_action(actions, seen, name, [action_status(2, 11)])
    with pytest.raises(ValueError, match='Cancel the existing goal'):
        motion_admission(operation, gate, session, actions, 100.1)
    observe_action(actions, seen, name, [])
    assert motion_admission(operation, gate, session, actions, 100.1) is gate


@pytest.mark.parametrize('operation', ['arm', 'goal'])
def test_final_motion_admission_revalidates_gate_session_and_time(operation):
    _, session = context()
    gate = {'navigation_session_id': session['navigation_session_id'], 'map_version_id': 'grid-a',
            'mode': 'simulation', 'wall_time': 100., 'armed': True, 'reason': 'simulation_only'}
    for update in ({'navigation_session_id': 'b'*32}, {'map_version_id': 'grid-b'},
                   {'wall_time': 98.}, {'mode': 'live'}):
        with pytest.raises(ValueError, match='No fresh command gate'):
            motion_admission(operation, dict(gate, **update), session, {}, 100.)
