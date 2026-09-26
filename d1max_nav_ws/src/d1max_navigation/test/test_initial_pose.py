import math
from io import BytesIO
from urllib.error import HTTPError

import pytest

from d1max_navigation.initial_pose import (
    LocalWebClient, PoseRejected, SubmissionUnknown, pose_payload,
    submit_checked_pose, validate_context, validate_pose_age,
)


def overview():
    return {'phase': 'running', 'id': 'session-a', 'version_id': 'grid-a',
            'health': {'session_id': 'session-a', 'map_version_id': 'grid-a',
                       'wall_time': 100.0, 'initial_pose_ready': True,
                       'head_direction': 1, 'frames': {'map': 'd1max_loc_map'}}}


def pose(**overrides):
    kwargs = dict(frame='d1max_loc_map', expected_frame='d1max_loc_map',
                  stamp=100.0, now=100.2, position=(1., 2., 0.),
                  quaternion=(0., 0., math.sqrt(.5), math.sqrt(.5)), body_z=.4)
    kwargs.update(overrides)
    return pose_payload(**kwargs)


def test_body_seed_uses_explicit_body_height_and_rviz_yaw():
    result = pose(position=(1., 2., -10.))
    assert result == {'x': 1., 'y': 2., 'z': .4, 'yaw': pytest.approx(math.pi/2),
                      'reference': 'body'}


@pytest.mark.parametrize('overrides', [
    {'frame': 'map'}, {'stamp': 0.}, {'stamp': 95.}, {'stamp': 101.},
    {'position': (float('nan'), 0., 0.)}, {'position': (0., 0., float('inf'))},
    {'position': (100001., 0., 0.)}, {'body_z': 101.}, {'body_z': True},
    {'quaternion': (0., 0., 0., 0.)}, {'quaternion': (0., 0., 0., 2.)},
    {'quaternion': (math.sin(.1), 0., 0., math.cos(.1))},
    {'quaternion': (0., math.sin(.1), 0., math.cos(.1))},
    {'quaternion': (0., 0., 0., float('nan'))},
])
def test_pose_rejections(overrides):
    with pytest.raises(PoseRejected):
        pose(**overrides)


def test_quaternion_sign_equivalence():
    assert pose(quaternion=(0., 0., -math.sqrt(.5), -math.sqrt(.5)))['yaw'] == pytest.approx(math.pi/2)


def test_context_valid_and_optional_session_pin():
    session, _ = validate_context(overview(), expected_version_id='grid-a', now=100.5)
    assert session == 'session-a'
    with pytest.raises(PoseRejected):
        validate_context(overview(), expected_version_id='grid-a', expected_session_id='old', now=100.5)


@pytest.mark.parametrize('change', [
    lambda s: s.update(phase='stopped'), lambda s: s.update(version_id='other'),
    lambda s: s.update(health={}), lambda s: s['health'].update(wall_time=95.),
    lambda s: s['health'].update(wall_time=101.),
    lambda s: s['health'].update(session_id='other'),
    lambda s: s['health'].update(map_version_id='other'),
    lambda s: s['health'].update(initial_pose_ready=False),
    lambda s: s['health'].update(head_direction=2),
    lambda s: s['health'].update(frames={'map': 'map'}),
])
def test_context_fail_closed(change):
    state = overview()
    change(state)
    with pytest.raises(PoseRejected):
        validate_context(state, expected_version_id='grid-a', now=100.5)


class FakeClient:
    def __init__(self, state=None, response=None):
        self.state = overview() if state is None else state
        self.response = response or {'status': 'queued', 'accepted': False, 'request_id': 'request-a'}
        self.calls = []

    def request(self, path, payload=None, headers=None):
        self.calls.append((path, payload, headers))
        return self.state if payload is None else self.response


def test_one_post_uses_atomic_server_context_guards():
    client = FakeClient()
    receipt = submit_checked_pose(client, pose(), expected_version_id='grid-a', wall_now=lambda: 100.5)
    assert receipt.request_id == 'request-a'
    assert receipt.session_id == 'session-a'
    assert len(client.calls) == 2
    assert client.calls[-1][2] == {'X-D1max-Session-Id': 'session-a', 'X-D1max-Map-Version': 'grid-a'}
    assert client.calls[-1][1]['reference'] == 'body'


def test_no_post_when_offline_or_map_changed():
    for state in ({'phase': 'stopped'}, {**overview(), 'version_id': 'grid-other'}):
        client = FakeClient(state)
        with pytest.raises(PoseRejected):
            submit_checked_pose(client, pose(), expected_version_id='grid-a', wall_now=lambda: 100.5)
        assert len(client.calls) == 1


def test_pose_expiring_during_http_is_not_posted():
    client = FakeClient()
    ages = iter((100.5, 105.0))
    with pytest.raises(PoseRejected):
        submit_checked_pose(client, pose(), expected_version_id='grid-a', wall_now=lambda: 100.5,
                            check_pose_fresh=lambda: validate_pose_age(100., next(ages)))
    assert len(client.calls) == 1


def test_queued_is_not_success_and_bad_reply_is_not_retried():
    client = FakeClient(response={'status': 'success', 'accepted': True})
    with pytest.raises(SubmissionUnknown):
        submit_checked_pose(client, pose(), expected_version_id='grid-a', wall_now=lambda: 100.5)
    assert len(client.calls) == 2


@pytest.mark.parametrize('url', ['https://127.0.0.1:8766', 'http://robot:8766',
                              'http://127.0.0.1:8766/api', 'http://user:password@localhost:8766',
                              'http://127.0.0.1:8766?redirect=evil'])
def test_client_only_accepts_loopback_root(url):
    with pytest.raises(ValueError):
        LocalWebClient(url)


def test_client_does_not_offer_lifecycle_operations():
    client = LocalWebClient()
    for endpoint in ('/api/localization/start', '/api/localization/stop', '/api/localization/connect'):
        with pytest.raises(ValueError):
            client.request(endpoint, {})


@pytest.mark.parametrize('error,exception', [
    (TimeoutError('timeout'), SubmissionUnknown),
    (HTTPError('http://127.0.0.1:8766', 500, 'Error', {}, BytesIO(b'{}')), SubmissionUnknown),
    (HTTPError('http://127.0.0.1:8766', 409, 'Conflict', {}, BytesIO(b'{"detail":"stale session"}')), PoseRejected),
])
def test_http_submission_error_is_never_retried(error, exception):
    client = LocalWebClient()

    class FailingOpener:
        calls = 0

        def open(self, *_args, **_kwargs):
            self.calls += 1
            raise error

    client.opener = FailingOpener()
    with pytest.raises(exception):
        client.request('/api/localization/initial-pose', pose())
    assert client.opener.calls == 1


def test_offline_ros_callback_never_calls_http_or_reads_pose():
    pytest.importorskip('rclpy')
    from d1max_navigation.rviz_initial_pose_bridge import RvizInitialPoseBridge

    class OfflineBridge:
        p = {'offline': True}
        events = []

        def emit(self, state, message):
            self.events.append((state, message))

    bridge = OfflineBridge()
    # No client, executor or valid pose object exists: the offline gate must return first.
    RvizInitialPoseBridge.on_pose(bridge, None)
    assert bridge.events[0][0] == 'rejected'
