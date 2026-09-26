import numpy as np
import pytest

from d1max_pct_scan.live_global_contract import (
    GlobalPlanError, RequestEvidence, choose_floor_surface,
    floor_from_original_ground_z, floor_from_planning_z, fresh_stamp,
    healthy_context, labels_for_result,
    new_goal_stamp, result_admissible, supported_start_floor,
)


class TwoFloorTomogram:
    def sample_surfaces(self, xy, deduplicate=True):
        assert deduplicate
        if np.any(np.asarray(xy) != [2., 3.]):
            return []
        return [{'xyz': [2., 3., -.65], 'ground_z': -.65, 'layer_id': 4},
                {'xyz': [2., 3., 3.15], 'ground_z': 3.15, 'layer_id': 12}]


RANGES = {'lower': [-1., 0.], 'upper': [2.8, 3.5]}


def test_current_start_floor_follows_measured_height_not_the_startup_floor():
    class Bridge:
        floors = {'floor1': -.65, 'floor2': 3.15}

        def project_live_pose_to_ground(self, body, floor, *, body_height_interval_m):
            low, high = body_height_interval_m
            if not low <= body[2]-self.floors[floor] <= high:
                raise ValueError('unsupported height')
            return (float(body[0]), float(body[1]), self.floors[floor])

    bridge = Bridge()
    assert supported_start_floor(bridge, [2., 3., -.10], height_interval_m=(.25, .85))[0] == 'floor1'
    assert supported_start_floor(bridge, [2., 3., 3.70], height_interval_m=(.25, .85))[0] == 'floor2'
    with pytest.raises(GlobalPlanError, match='unique_measured'):
        supported_start_floor(bridge, [2., 3., 1.5], height_interval_m=(.25, .85))
    bridge.floors['floor2'] = -.60  # contradictory overlapping ownership
    with pytest.raises(GlobalPlanError, match='unique_measured'):
        supported_start_floor(bridge, [2., 3., -.10], height_interval_m=(.25, .85))


def _status(now):
    nav = dict(valid=True, navigation_ready=False, fault=None, epoch=4,
               seed_id='seed-a', received_at_unix=now)
    scan = dict(session_id='session-a', localization_session_id='session-a',
                ready=True, sensor_ready=True, motion_enabled=False,
                localization_epoch=4, localization_seed_id='seed-a',
                received_at_unix=now, cloud_age=.05)
    return nav, scan


def test_shadow_ready_allows_unverified_navigation_but_never_bad_session_or_cloud():
    nav, scan = _status(100.)
    assert healthy_context(nav, scan, session_id='session-a', now=100.1) == (4, 'seed-a')
    scan['localization_session_id'] = 'old-session'
    with pytest.raises(GlobalPlanError, match='same_session'):
        healthy_context(nav, scan, session_id='session-a', now=100.1)
    scan['localization_session_id'] = 'session-a'
    scan['cloud_age'] = 1.0
    with pytest.raises(GlobalPlanError, match='cloud_not_fresh'):
        healthy_context(nav, scan, session_id='session-a', now=100.1)


def test_goal_timestamp_must_be_newer_than_startup_and_prior_goal():
    assert new_goal_stamp(100.5, now=100.6, startup_stamp=100., previous_stamp=100.) == 100.5
    with pytest.raises(GlobalPlanError, match='goal_stamp'):
        new_goal_stamp(100., now=100.1, startup_stamp=100., previous_stamp=99.)
    with pytest.raises(GlobalPlanError, match='goal_stamp'):
        new_goal_stamp(99., now=102., startup_stamp=98., previous_stamp=98.)
    with pytest.raises(GlobalPlanError, match='goal_stamp'):
        new_goal_stamp(100.4, now=100.6, startup_stamp=100., previous_stamp=100.5)
    assert not fresh_stamp(0, 100.)


def test_same_xy_uses_z_and_explicit_floor_not_nearest_xy_or_wrong_layer():
    tomogram = TwoFloorTomogram()
    low = choose_floor_surface(tomogram, [2., 3.], 'floor1', RANGES, hint_z=-.65)
    upper = choose_floor_surface(tomogram, [2., 3.], 'floor2', RANGES, hint_z=3.15)
    assert low['layer_id'] == 4 and upper['layer_id'] == 12
    assert floor_from_planning_z(3.15, RANGES) == 'floor2'
    with pytest.raises(GlobalPlanError, match='exact_xy'):
        choose_floor_surface(tomogram, [2.1, 3.], 'floor1', RANGES, hint_z=-.65)
    with pytest.raises(GlobalPlanError, match='exactly_one_floor'):
        floor_from_planning_z(1.3, RANGES)


def test_original_3d_goal_resolves_by_measured_height_not_overlapping_xy():
    class Floor:
        def __init__(self, z):
            self.z = z

        def query(self, xy, limits):
            return np.asarray([self.z]), np.asarray([0.])

    class Bridge:
        protected_regions = ()
        floors = {'floor1': Floor(-.57), 'floor2': Floor(3.04)}
        limits = object()

    assert floor_from_original_ground_z(Bridge(), [2., 3., 3.04]) == 'floor2'
    assert floor_from_original_ground_z(Bridge(), [2., 3., -.57]) == 'floor1'
    with pytest.raises(GlobalPlanError, match='uniquely_match'):
        floor_from_original_ground_z(Bridge(), [2., 3., 1.3])


def test_result_labels_follow_edges_not_xy_or_layer_guess():
    route = {'path': [[0., 0., -.65], [1., 0., -.65], [2., 0., 1.],
                      [3., 0., 3.15]],
             'edge_legs': ['lower_floor', 'stair_lower', 'stair_upper']}
    _, labels = labels_for_result(route, same_floor=None)
    assert labels == ['floor1', 'stair_lower', 'stair_upper', 'stair_upper']
    _, same = labels_for_result(route, same_floor='floor1')
    assert same == ['floor1'] * 4
    with pytest.raises(GlobalPlanError, match='edges_missing'):
        labels_for_result({'path': route['path'], 'edge_legs': ['lower_floor']}, same_floor=None)


def test_late_or_moved_or_reseeded_results_are_rejected():
    evidence = RequestEvidence(8, 4, 'seed-a', 100., (1., 2., .5), 20.)
    assert result_admissible(evidence, generation=8, context=(4, 'seed-a'),
                             body_xyz=[1.2, 2., .5], now_monotonic=21.,
                             result_timeout_s=10.) == pytest.approx(.2)
    for kwargs in ({'generation': 9}, {'context': (5, 'seed-a')},
                   {'body_xyz': [1.4, 2., .5]}, {'now_monotonic': 31.}):
        values = dict(generation=8, context=(4, 'seed-a'), body_xyz=[1., 2., .5],
                      now_monotonic=21., result_timeout_s=10.)
        values.update(kwargs)
        with pytest.raises(GlobalPlanError):
            result_admissible(evidence, **values)
