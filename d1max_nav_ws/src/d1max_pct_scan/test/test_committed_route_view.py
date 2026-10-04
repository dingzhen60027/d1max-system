"""Real wire-message callbacks; no ROS graph or navigation owner is started."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from d1max_pct_scan.committed_route_view import CommittedRouteView
from test_route_ingress import delivery


def fixture():
    message = delivery()
    return CommittedRouteView(message.session_id, message.snapshot.map_version_id), message


def test_true_committed_route_is_copied_with_original_source_frame_and_stamp():
    view, message = fixture()
    path = view.receive(message)
    assert path == message.snapshot.path
    assert view.snapshot.route_hash == message.route_hash
    path.poses[0].pose.position.x += 20.
    message.snapshot.path.poses[0].pose.position.y += 20.
    assert view.displayed_path().poses[0] != path.poses[0]
    assert view.displayed_path().poses[0] != message.snapshot.path.poses[0]


def test_completion_or_cancellation_preserves_historical_path_not_active_reference():
    view, message = fixture()
    original = view.receive(message)
    cancel = deepcopy(message)
    cancel.active, cancel.delivery_sequence = False, 2
    assert view.receive(cancel) is None
    assert view.historical
    assert view.displayed_path() == original
    resurrect = deepcopy(message)
    resurrect.delivery_sequence = 3
    assert view.receive(resurrect) is None
    assert view.historical and view.displayed_path() == original


def test_new_committed_task_replaces_history_but_delayed_old_cancel_does_not_clear_it():
    view, old = fixture()
    view.receive(old)
    new = deepcopy(old)
    new.task_id = new.snapshot.task_id = 'next-task'
    new.delivery_sequence = 3
    assert view.receive(new) == new.snapshot.path
    old.active, old.delivery_sequence = False, 4
    assert view.receive(old) is None
    assert view.identity[0] == 'next-task' and not view.historical
    old.active, old.delivery_sequence = True, 5
    assert view.receive(old) is None
    assert view.identity[0] == 'next-task'


def test_duplicate_commit_does_not_restamp_or_republish():
    view, message = fixture()
    original = view.receive(message)
    assert view.receive(message) is None
    message.delivery_sequence += 1
    message.source_stamp.sec += 20
    message.snapshot.path.header.stamp = deepcopy(message.source_stamp)
    assert view.receive(message) is None
    assert view.displayed_path() == original


@pytest.mark.parametrize('field,value', [('schema_version', 1), ('session_id', 'other'),
    ('task_id', 'wrong'), ('route_hash', 'bad')])
def test_invalid_envelope_never_replaces_history(field, value):
    view, message = fixture()
    original = view.receive(message)
    bad = delivery(2)
    setattr(bad, field, value)
    assert view.receive(bad) is None
    assert view.last_sequence == 1 and view.displayed_path() == original


@pytest.mark.parametrize('mutation', ['map', 'path', 'hash', 'preview', 'stamp'])
def test_untrusted_candidate_geometry_or_mismatched_source_is_rejected(mutation):
    view, message = fixture()
    if mutation == 'map': message.snapshot.map_version_id = 'other-map'
    elif mutation == 'path': message.snapshot.path.poses = []
    elif mutation == 'hash': message.snapshot.path.poses[0].pose.position.z += .1
    elif mutation == 'preview': message.snapshot.preview_ready = False
    else: message.snapshot.path.header.stamp.sec += 1
    assert view.receive(message) is None
    assert view.displayed_path() is None and view.last_sequence == 0


@pytest.mark.parametrize('field', ['session', 'map'])
def test_only_explicit_context_change_clears_historical_geometry(field):
    view, message = fixture()
    view.receive(message)
    assert not view.select_context(view.session_id, view.map_version_id)
    session = 'new-session' if field == 'session' else view.session_id
    map_id = 'new-map' if field == 'map' else view.map_version_id
    assert view.select_context(session, map_id)
    assert view.displayed_path() is None and view.last_sequence == 0
    assert view.receive(message) is None


def test_initial_inactive_delivery_cannot_invent_a_prior_committed_display():
    view, message = fixture()
    message.active = False
    assert view.receive(message) is None
    assert view.displayed_path() is None


def test_production_display_callback_publishes_only_new_commit_not_retirement():
    from d1max_pct_scan.committed_route_view import publish_committed_route
    cache, first = fixture()
    published = []
    publisher = SimpleNamespace(publish=lambda message: published.append(deepcopy(message)))
    publish_committed_route(cache, publisher, first)
    cancel = deepcopy(first)
    cancel.delivery_sequence, cancel.active = 2, False
    publish_committed_route(cache, publisher, cancel)
    assert len(published) == 1 and published[0] == first.snapshot.path
    assert cache.historical and cache.displayed_path() == published[0]
