"""Offline regressions for bounded, shadow-only goal pause and replacement."""
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_scan.live_goal_pause import (
    BoundedGoalPause, GoalIdentity, GoalIntent, confirmed_goal_identity,
    hard_identity_issue,
)
from d1max_pct_scan.live_global_contract import GlobalPlanError, RequestEvidence
from d1max_pct_scan.live_global_planner import LiveGlobalPlanner


IDENTITY = GoalIdentity('session-a', 'map-a', 4, 'seed-a', 'd1max_loc_map', 'hash-a')
INTENT = GoalIntent(100.1, '2d', 'd1max_loc_map', (1., 2., 0.), 'floor1', IDENTITY)


def _localizer(now=100.):
    return dict(session_id='session-a', map_version_id='map-a', wall_time=now,
                local_epoch=4, active_seed_ns='seed-a', confirmed_seed_ns='seed-a',
                verified_confirmations=3, frames={'map': 'd1max_loc_map'},
                localized=True, local_fault=None)


def _navigation(now=100.):
    return dict(valid=True, epoch=4, seed_id='seed-a', received_at_unix=now,
                fault=None)


def _scan(now=100.):
    return dict(session_id='session-a', localization_session_id='session-a',
                localization_epoch=4, localization_seed_id='seed-a',
                frame_id='d1max_loc_map', received_at_unix=now, cloud_age=.02,
                motion_enabled=False)


def test_goal_identity_requires_fresh_confirmed_localizer_not_nav_hint():
    assert confirmed_goal_identity(_localizer(), session_id='session-a',
                                   frame_id='d1max_loc_map', tomogram_sha256='hash-a',
                                   now=100.2, freshness_s=.5) == IDENTITY
    stale = _localizer()
    with pytest.raises(ValueError, match='fresh_localizer'):
        confirmed_goal_identity(stale, session_id='session-a',
                                frame_id='d1max_loc_map', tomogram_sha256='hash-a',
                                now=101., freshness_s=.5)
    unconfirmed = _localizer()
    unconfirmed['confirmed_seed_ns'] = None
    with pytest.raises(ValueError, match='confirmed_localizer'):
        confirmed_goal_identity(unconfirmed, session_id='session-a',
                                frame_id='d1max_loc_map', tomogram_sha256='hash-a',
                                now=100.2, freshness_s=.5)


def test_matching_temporarily_lost_does_not_erase_confirmed_identity():
    value = _localizer()
    value.update(localized=False, state='lost', consecutive_match_failures=4)
    assert confirmed_goal_identity(value, session_id='session-a',
        frame_id='d1max_loc_map', tomogram_sha256='hash-a', now=100.2,
        freshness_s=.5) == IDENTITY
    value['verified_confirmations'] = 2
    with pytest.raises(ValueError, match='confirmed_localizer'):
        confirmed_goal_identity(value, session_id='session-a',
            frame_id='d1max_loc_map', tomogram_sha256='hash-a', now=100.2,
            freshness_s=.5)


@pytest.mark.parametrize('source,field,value', [
    ('localizer', 'session_id', 'session-b'),
    ('localizer', 'map_version_id', 'map-b'),
    ('localizer', 'local_epoch', 5),
    ('localizer', 'confirmed_seed_ns', 'seed-b'),
    ('localizer', 'active_seed_ns', 'seed-b'),
    ('localizer', 'local_fault', 'fault'),
    ('navigation', 'epoch', 5),
    ('navigation', 'seed_id', 'seed-b'),
    ('navigation', 'fault', 'fault'),
    ('navigation', 'reset_pending', True),
    ('localizer', 'reset_pending', True),
    ('scan', 'session_id', 'session-b'),
    ('scan', 'localization_seed_id', 'seed-b'),
    ('scan', 'frame_id', 'other_map'),
])
def test_explicit_identity_change_or_fault_is_hard_cancel(source, field, value):
    values = dict(localizer=_localizer(), navigation=_navigation(), scan=_scan())
    values[source][field] = value
    assert hard_identity_issue(IDENTITY, **values)


def test_missing_navigation_seed_is_not_falsely_a_new_identity():
    navigation = _navigation()
    navigation['seed_id'] = None
    navigation['valid'] = False
    assert hard_identity_issue(IDENTITY, localizer=_localizer(),
                               navigation=navigation, scan=_scan()) is None


def test_pause_retains_only_immutable_goal_for_two_seconds_and_new_samples():
    pause = BoundedGoalPause()
    pause.install(INTENT)
    assert pause.pause(10.)
    assert pause.intent is INTENT
    assert not pause.pause(10.3)  # repeated faults cannot extend the deadline
    assert not pause.observe_good(10.4, 100.4)
    assert not pause.observe_good(10.5, 100.4)  # duplicate heartbeat
    assert not pause.observe_good(10.55, 100.55)
    assert pause.observe_good(10.61, 100.61)
    assert pause.intent is INTENT and pause.paused_at == 10.
    assert pause.expired(12.)
    pause.clear()  # explicit cancel/identity change/TTL discards the intent
    assert pause.intent is None
    assert not pause.observe_good(12.1, 100.7)


def test_bad_sample_resets_resume_evidence_without_extending_ttl():
    pause = BoundedGoalPause()
    pause.install(INTENT)
    pause.pause(10.)
    pause.observe_good(10.1, 100.1)
    pause.pause(10.2)
    assert pause.good_count == 0 and pause.paused_at == 10.
    assert not pause.observe_good(10.3, 100.3)
    assert not pause.observe_good(10.4, 100.4)
    assert pause.observe_good(10.6, 100.6)
    pause.resumed()
    assert pause.intent is INTENT and pause.paused_at is None
    pause.install(GoalIntent(101., '2d', 'd1max_loc_map', (2., 2., 0.),
                             'floor1', IDENTITY))
    assert pause.intent.user_stamp == 101. and pause.good_count == 0


def test_soft_pause_suspends_admission_without_cancelling_immutable_task():
    pause = BoundedGoalPause()
    pause.install(INTENT)
    actions = []
    fake = SimpleNamespace(p=dict(retain_preview_task_on_soft_loss=True),
                           pause=pause, generation=8, current=object(),
                           pending_goal=object(), active_context=(4, 'seed-a'),
                           now_s=lambda: 100.5, cached_result=None, commit_wait_started=None,
                           stop_child=lambda: actions.append('stop_child'),
                           publish_empty=lambda reason: actions.append(reason))
    LiveGlobalPlanner.pause_reference(fake, 'navigation_stale')
    assert fake.generation == 8
    assert fake.current is not None and fake.pending_goal is not None
    assert fake.active_context is None
    assert fake.pause_wall == 100.5 and fake.state == 'computing_while_navigation_unavailable'
    assert actions == []  # Empty Path is exclusively a hard cancellation.
    assert not fake.active_reference and fake.reason == 'paused:navigation_stale'
    assert fake.pause.intent is INTENT
    LiveGlobalPlanner.pause_reference(fake, 'still_stale')
    assert fake.generation == 8  # no repeated cancellation churn


def test_global_pause_barrier_requires_new_body_and_localizer_not_cloud():
    pause = BoundedGoalPause()
    pause.install(INTENT)
    pause.pause(10.)
    fake = SimpleNamespace(pause=pause, pause_wall=100.5, body_stamp=100.6,
                           body_received=10.1, localizer_received=10.1,
                           localizer=dict(wall_time=100.45))
    assert not LiveGlobalPlanner._pause_barrier_passed(fake)
    fake.localizer['wall_time'] = 100.6
    assert LiveGlobalPlanner._pause_barrier_passed(fake)
    fake.body_stamp = 100.49
    assert not LiveGlobalPlanner._pause_barrier_passed(fake)
    fake.body_stamp = 100.6
    fake.body_received = 9.9
    assert not LiveGlobalPlanner._pause_barrier_passed(fake)


def test_resume_keeps_unmoved_native_attempt_and_old_packet_is_ignored():
    pause = BoundedGoalPause()
    pause.install(INTENT)
    pause.pause(10.)
    actions = []
    fake = SimpleNamespace(pause=pause, pause_wall=100.5,
                           confirmed_identity=lambda: IDENTITY,
                           planning_context=lambda: (4, 'seed-a'),
                           now_s=lambda: 100.8, current=object(), cached_evidence=None,
                           cached_result=None, _start_moved=lambda evidence: False,
                           plan_intent=lambda intent, **kwargs: actions.append((intent, kwargs)),
                           revoke=lambda reason: pytest.fail(f'unexpected revoke: {reason}'))
    LiveGlobalPlanner.resume_intent(fake)
    assert actions == []
    assert fake.state == 'computing_native_pct'
    assert pause.intent is INTENT and pause.paused_at is None
    assert fake.pause_wall is None
    fake.current = RequestEvidence(12, 4, 'seed-a', 100.8, (0., 0., .5), 10.8)
    LiveGlobalPlanner._finish(fake, {'kind': 'planned', 'generation': 8,
                                     'result': {'path_xyz': [[0, 0, 0], [1, 0, 0]]}})
    assert fake.current.generation == 12  # retired worker result cannot publish


def test_resume_identity_change_rejects_without_old_route():
    pause = BoundedGoalPause()
    pause.install(INTENT)
    pause.pause(10.)
    actions = []
    fake = SimpleNamespace(pause=pause, confirmed_identity=lambda: GoalIdentity(
        'session-a', 'map-a', 5, 'seed-a', 'd1max_loc_map', 'hash-a'),
        planning_context=lambda: (4, 'seed-a'), now_s=lambda: 100.8,
        plan_intent=lambda *_args, **_kwargs: pytest.fail('old goal resumed'),
        revoke=lambda reason: actions.append(reason))
    LiveGlobalPlanner.resume_intent(fake)
    assert fake.state == 'goal_rejected'
    assert actions and 'resume_revalidation_failed' in actions[0]


def test_new_attempt_reprojects_current_body_without_reusing_old_start():
    class Map:
        def sample_surfaces(self, xy, deduplicate=True):
            return [dict(xyz=[float(xy[0]), float(xy[1]), -.5],
                         ground_z=-.5, layer_id=4)]

    class Bridge:
        source_frame = 'd1max_loc_map'
        planning_frame = 'd1max_multifloor_planning'
        floors = {'floor1': SimpleNamespace(reference_z_m=-.5)}

        def project_live_pose_to_ground(self, body, floor, **_kwargs):
            assert floor == 'floor1'
            return SimpleNamespace(xyz=np.array([body[0], body[1], -.5]))

    pause = BoundedGoalPause()
    pause.install(INTENT)
    fake = SimpleNamespace(pause=pause, bridge=Bridge(), tomogram=Map(),
                           route_settings={'floor_z_ranges': {'lower': [-1., 0.]}},
                           p={'current_floor': 'floor1', 'body_height_min_m': .25,
                              'body_height_max_m': .85},
                           body=np.array([1., 2., .5]), generation=10,
                           now_s=lambda: 100.9, goal_deadline_monotonic=float('inf'),
                           _maybe_launch_pending=lambda: None)
    LiveGlobalPlanner.plan_intent(fake, INTENT, context=(4, 'seed-a'),
                                  attempt_stamp=100.7)
    old = fake.current
    assert fake.pending_goal[0]['xyz'][:2] == [1., 2.]
    fake.body = np.array([1.5, 2.5, .5])
    LiveGlobalPlanner.plan_intent(fake, INTENT, context=(4, 'seed-a'),
                                  attempt_stamp=100.9)
    assert fake.current.generation > old.generation
    assert fake.current.start_body_xyz == (1.5, 2.5, .5)
    assert fake.pending_goal[0]['xyz'][:2] == [1.5, 2.5]
    assert fake.pending_goal[1]['xyz'][:2] == [1., 2.]


@pytest.mark.parametrize('same_frame',[True,False])
def test_source_identity_3d_goal_uses_local_height_not_floor_median(same_frame):
    """Keep the contract explicit even when source/planning frame aliases change.

    The production identity bridge currently aliases both frame names, so the
    old planning-frame branch also kept goal Z. The regression must not claim
    that production had taken its conditioned/non-rigid source-frame branch.
    """
    ground=-.7043251991271973
    class Floor:
        reference_z_m=-.5535517632961273
        def query(self,xy,limits):return np.array([ground]),np.array([.01])
    class Bridge:
        source_frame='d1max_loc_map'
        planning_frame='d1max_loc_map' if same_frame else 'identity_planning_alias'
        projection_kind='source_identity'
        floors={'floor1':Floor()};protected_regions=[];limits=None
        def project_live_pose_to_ground(self,body,floor,**kwargs):
            return SimpleNamespace(xyz=np.array([body[0],body[1],ground]))
    class Map:
        def sample_surfaces(self,xy,deduplicate=True):
            return [dict(xyz=[*xy,ground],ground_z=ground,layer_id=4)]
    intent=GoalIntent(100.1,'3d','d1max_loc_map',
        (-31.600778198242185,44.995704650878906,-.744481235742569),'floor1',IDENTITY)
    pause=BoundedGoalPause();pause.install(intent)
    fake=SimpleNamespace(pause=pause,bridge=Bridge(),tomogram=Map(),
        route_settings={'floor_z_ranges':{'lower':[-.95,-.10]}},
        p={'body_height_min_m':.25,'body_height_max_m':.85},body=np.array([1.,2.,-.2]),
        generation=0,now_s=lambda:100.9,goal_deadline_monotonic=float('inf'),
        _maybe_launch_pending=lambda:None)
    LiveGlobalPlanner.plan_intent(fake,intent,context=(4,'seed-a'),attempt_stamp=100.9)
    assert fake.pending_goal[1]['ground_z']==ground
    assert np.array_equal(fake.pending_goal[1]['xyz'][:2],intent.xyz[:2])
    from d1max_pct_scan.live_global_contract import choose_floor_surface
    with pytest.raises(GlobalPlanError,match='no_traversable_measured'):
        choose_floor_surface(fake.tomogram,intent.xyz[:2],'floor1',
            fake.route_settings['floor_z_ranges'],hint_z=Bridge.floors['floor1'].reference_z_m)
    wrong=GoalIntent(100.2,'3d','d1max_loc_map',(*intent.xyz[:2],ground-.151),'floor1',IDENTITY)
    pause.install(wrong)
    with pytest.raises(GlobalPlanError,match='uniquely_match_observed_floor'):
        LiveGlobalPlanner.plan_intent(fake,wrong,context=(4,'seed-a'),attempt_stamp=100.9)


def test_transient_invalid_pauses_then_requires_three_distinct_fresh_samples(monkeypatch):
    clock = [10.]
    monkeypatch.setattr('d1max_pct_scan.live_global_planner.time.monotonic',
                        lambda: clock[0])
    pause = BoundedGoalPause()
    pause.install(INTENT)
    actions = []
    fake = SimpleNamespace(
        p=dict(retain_preview_task_on_soft_loss=True),
        pause=pause, pause_wall=None, generation=3, current=object(),
        pending_goal=None, active_context=(4, 'seed-a'),
        tomogram=SimpleNamespace(sha256='hash-a'), localizer=_localizer(),
        navigation=_navigation(), scan=_scan(), body_stamp=100.,
        pose_status=_navigation(),
        body_received=9., scan_received=9., localizer_received=9., now_s=lambda: 100.5,
        cached_result=None, commit_wait_started=None,
        stop_child=lambda: actions.append('retire_worker'),
        publish_empty=lambda reason: actions.append('empty:' + reason),
        confirmed_identity=lambda: IDENTITY,
        resume_intent=lambda: actions.append('new_attempt'))

    def context():
        if not fake.navigation['valid']:
            raise GlobalPlanError('temporary_nav_invalid')
        return (4, 'seed-a')

    def revoke(reason):
        actions.append('hard:' + reason)
        pause.clear()

    fake.planning_context = context
    fake.revoke = revoke
    fake.pause_reference = lambda reason: LiveGlobalPlanner.pause_reference(fake, reason)
    fake._pause_barrier_passed = lambda: LiveGlobalPlanner._pause_barrier_passed(fake)
    fake.navigation.update(valid=False, seed_id=None)
    LiveGlobalPlanner._revoke_if_context_lost(fake)
    assert pause.intent is INTENT and pause.paused_at == 10.
    assert actions == []
    assert fake.reason == 'paused:temporary_nav_invalid' and not fake.active_reference
    fake.navigation.update(valid=True, seed_id='seed-a')
    fake.body_received = fake.localizer_received = 10.05
    fake.localizer['wall_time'] = 100.6
    fake.body_stamp = 100.6
    fake.scan.update(received_at_unix=100.65, cloud_age=.02)
    for when, nav_stamp in ((10.1, 100.6), (10.2, 100.7), (10.32, 100.8)):
        clock[0] = when
        fake.navigation['received_at_unix'] = nav_stamp
        fake.pose_status['received_at_unix'] = nav_stamp
        LiveGlobalPlanner._revoke_if_context_lost(fake)
    assert actions[-1] == 'new_attempt'
    assert 'hard:' not in ' '.join(actions)
    fake.localizer['map_version_id'] = 'different-map'
    LiveGlobalPlanner._revoke_if_context_lost(fake)
    assert any(action.startswith('hard:goal_identity_or_fault_changed') for action in actions)
    assert pause.intent is None
