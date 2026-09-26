import math

import numpy as np
import pytest

from d1max_navigation.simulation import OccupancyWorld, PlanarPose, SimulationPlant, validate_isolation


def world():
    grid = np.zeros((100, 120), dtype=int)
    grid[:, 0] = grid[:, -1] = grid[0, :] = grid[-1, :] = 100
    grid[25:75, 80] = 100
    return OccupancyWorld(grid, 120, 100, .1)


def test_map_unknown_outside_and_footprint_are_blocked():
    w = world()
    assert w.clear(3., 3.)
    assert not w.clear(-1., 3.)
    assert not w.clear(8., 3.)
    assert not w.clear(7.7, 3.)
    grid = np.zeros((20, 20), dtype=int)
    grid[10, 10] = -1
    assert not OccupancyWorld(grid, 20, 20, .1).clear(1.05, 1.05, .01)


def test_rotated_origin():
    w = OccupancyWorld(np.zeros((30, 30)), 30, 30, .1, (10., -5., math.pi/2))
    gx, gy = w.grid_coordinates(9., -4.)
    assert gx == pytest.approx(10.)
    assert gy == pytest.approx(10.)
    assert w.clear(9., -4.)


def test_synthetic_scan_hits_wall_at_expected_range():
    w = world()
    ranges = w.raycast(PlanarPose(3., 5., 0.), count=360, max_range=10.)
    assert ranges[180] == pytest.approx(5., abs=.1)
    assert ranges[0] == pytest.approx(2.9, abs=.1)


def test_dynamic_obstacle_appears_and_clears_without_mutating_static_map():
    w = world()
    static = w.blocked.copy()
    pose = PlanarPose(3., 5., 0.)
    baseline = w.raycast(pose, max_range=10.)
    w.set_dynamic_obstacle((4.5, 5., .2))
    assert w.raycast(pose, max_range=10.)[180] == pytest.approx(1.3, abs=.1)
    assert not w.clear(4.5, 5., .1)
    assert not w.segment_clear((3., 5.), (5., 5.), .2)
    assert np.array_equal(static, w.blocked)
    w.set_dynamic_obstacle(None)
    assert np.array_equal(w.raycast(pose, max_range=10.), baseline)
    assert w.clear(4.5, 5., .1)
    assert np.array_equal(static, w.blocked)


def test_dense_synthetic_scan_keeps_consistent_angular_geometry():
    w = world()
    ranges = w.raycast(PlanarPose(3., 5., 0.), count=720, max_range=10.)
    assert len(ranges) == 720
    assert ranges[360] == pytest.approx(5., abs=.1)
    assert ranges[0] == pytest.approx(2.9, abs=.1)


@pytest.mark.parametrize('obstacle', [(1, 1, 2), (1, 1, -1), (float('nan'), 1, .1),
                                     (100, 1, .1), (1, 1)])
def test_invalid_dynamic_obstacle_rejected(obstacle):
    with pytest.raises(ValueError):
        world().set_dynamic_obstacle(obstacle)


def test_command_timeout_and_invalid_command_fail_closed():
    plant = SimulationPlant(PlanarPose(3., 3., 0.))
    assert plant.set_command(.2, 0., 0., 1.)
    plant.step(world(), .1, 1.1)
    assert plant.pose.x == pytest.approx(3.02)
    plant.step(world(), .1, 1.5)
    assert plant.pose.x == pytest.approx(3.02)
    assert plant.velocity == (0., 0., 0.)
    assert not plant.set_command(float('nan'), 0., 0., 2.)
    plant.step(world(), .1, 2.1)
    assert plant.pose.x == pytest.approx(3.02)


def test_velocity_bounds_body_frame_and_turning():
    plant = SimulationPlant(PlanarPose(3., 3., math.pi/2))
    plant.set_command(10., 0., 0., 1.)
    plant.step(world(), .1, 1.1)
    assert plant.pose.x == pytest.approx(3.)
    assert plant.pose.y == pytest.approx(3.03)
    plant.set_command(0., 0., 10., 1.2)
    plant.step(world(), .1, 1.3)
    assert plant.pose.yaw == pytest.approx(math.pi/2+.05)


def test_collision_prevents_crossing_wall():
    plant = SimulationPlant(PlanarPose(7.5, 3., 0.), radius=.4)
    for i in range(100):
        plant.set_command(.4, 0., 0., i*.1)
        plant.step(world(), .1, i*.1)
    assert plant.pose.x < 7.6
    assert plant.collision_stops > 0


def test_pose_reset_requires_clear_footprint_and_clears_command():
    plant = SimulationPlant(PlanarPose(3., 3., 0.))
    plant.set_command(.2, 0., 0., 1.)
    assert not plant.reset(PlanarPose(8., 3., 0.), world())
    assert plant.reset(PlanarPose(4., 4., 0.), world())
    assert plant.command_time is None
    assert plant.command == plant.velocity == (0., 0., 0.)


def test_goal_suggestion_is_reachable_and_does_not_mutate_pose():
    w, pose = world(), PlanarPose(3., 3., 0.)
    goal = w.suggest_goal(pose)
    assert goal is not None
    assert 2.9 < math.hypot(goal['x']-pose.x, goal['y']-pose.y) <= 5.01
    assert w.segment_clear((pose.x, pose.y), (goal['x'], goal['y']), .5)
    assert pose.x == 3.


def config():
    return {'mode': 'client', 'connect': {'endpoints': ['tcp/127.0.0.1:7460']},
            'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}}


def test_isolated_zenoh_session_allowed():
    validate_isolation({'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp'}, config())


@pytest.mark.parametrize('change', [
    lambda c: c['connect'].update(endpoints=['tcp/127.0.0.1:7448']),
    lambda c: c['connect'].update(endpoints=['tcp/127.0.0.1:7460', 'tcp/192.168.1.1:7447']),
    lambda c: c['scouting']['multicast'].update(enabled=True),
    lambda c: c['scouting']['gossip'].update(enabled=True),
    lambda c: c.update(mode='peer'),
    lambda c: c.update(listen={'endpoints': ['tcp/0.0.0.0:9999']}),
])
def test_live_or_discoverable_session_refused(change):
    c = config()
    change(c)
    with pytest.raises(ValueError):
        validate_isolation({'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp'}, c)


@pytest.mark.parametrize('environment', [
    {}, {'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'},
    {'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp', 'ZENOH_CONFIG_OVERRIDE': 'connect/endpoints=live'},
])
def test_other_middleware_or_overrides_refused(environment):
    with pytest.raises(ValueError):
        validate_isolation(environment, config())


@pytest.mark.parametrize('dt', [-1., 0., .3, float('nan')])
def test_scheduling_gap_never_causes_pose_jump(dt):
    plant = SimulationPlant(PlanarPose(3., 3., 0.))
    plant.set_command(.4, 0., 0., 1.)
    plant.step(world(), dt, 1.1)
    assert plant.pose.x == 3.


@pytest.mark.parametrize('kwargs', [{'command_timeout': 1.}, {'max_linear': 10.}, {'radius': -1.}, {'max_angular': float('nan')}])
def test_invalid_plant_bounds_rejected(kwargs):
    with pytest.raises(ValueError):
        SimulationPlant(**kwargs)
