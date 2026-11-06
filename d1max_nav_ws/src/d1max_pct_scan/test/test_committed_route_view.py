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


def snapshot_fixture():
    from test_bt_action_adapters import candidate
    source = candidate()
    source.task_id, source.route_id = 'global-task', 'global-route'
    provenance = {key: getattr(source, key) for key in (
        'frame_id', 'source_map_sha256', 'tomogram_sha256', 'conditioning_sha256')}
    cache = CommittedRouteView(source.session_id, source.map_version_id,
                               expected_provenance=provenance)
    return cache, source


def test_global_route_snapshot_displays_original_typed_path_without_local_authority():
    from d1max_navigation_bt_interfaces.msg import RouteSnapshot
    cache, source = snapshot_fixture()
    original = deepcopy(source.path)
    path = cache.receive_snapshot(source)
    assert path == original
    assert isinstance(cache.snapshot, RouteSnapshot)
    assert cache.last_sequence == 0  # No fabricated RouteReference delivery.
    assert not cache.snapshot.execution_eligible
    path.poses[0].pose.position.x += 12.
    source.path.poses[0].pose.position.y += 12.
    assert cache.displayed_path() == original
    assert cache.displayed_path().header == original.header


def test_snapshot_duplicate_retired_and_out_of_order_results_do_not_replace_display():
    cache, old = snapshot_fixture()
    original = cache.receive_snapshot(old)
    restamped = deepcopy(old)
    restamped.path.header.stamp.sec += 20
    assert cache.receive_snapshot(restamped) is None
    assert cache.displayed_path() == original
    new = deepcopy(old)
    new.task_id, new.route_id = 'next-task', 'next-route'
    assert cache.receive_snapshot(new) is None  # Equal source time has no sequence fence.
    new.path.header.stamp.nanosec += 1
    assert cache.receive_snapshot(new) == new.path
    assert (old.task_id, old.route_id, old.route_hash) in cache.retired
    assert cache.receive_snapshot(restamped) is None
    delayed = deepcopy(new)
    delayed.task_id, delayed.route_id = 'older-third-task', 'older-third-route'
    delayed.path.header.stamp.nanosec -= 2
    assert cache.receive_snapshot(delayed) is None
    assert cache.identity == (new.task_id, new.route_id, new.route_hash)


@pytest.mark.parametrize('mutation', [
    'schema', 'session', 'map', 'identity', 'frame', 'hash', 'preview', 'stamp', 'evidence',
])
def test_snapshot_untrusted_wire_contract_does_not_advance_source_order(mutation):
    cache, source = snapshot_fixture()
    if mutation == 'schema': source.schema_version = 1
    elif mutation == 'session': source.session_id = 'foreign-session'
    elif mutation == 'map': source.map_version_id = 'foreign-map'
    elif mutation == 'identity': source.task_id = ''
    elif mutation == 'frame': source.path.header.frame_id = 'd1max_multifloor_planning'
    elif mutation == 'hash': source.path.poses[0].pose.position.z += .1
    elif mutation == 'preview': source.preview_ready = False
    elif mutation == 'stamp': source.path.header.stamp.sec = -1
    else: source.geometry_evidence_json = '{malformed JSON'
    assert cache.receive_snapshot(source) is None
    assert cache.displayed_path() is None
    assert cache.last_snapshot_stamp_ns == 0 and cache.last_sequence == 0


@pytest.mark.parametrize('field', ['source_map_sha256', 'tomogram_sha256', 'conditioning_sha256'])
def test_snapshot_valid_rehashed_foreign_provenance_cannot_reuse_configured_map_id(field):
    from d1max_pct_scan.source_route import RouteSnapshot
    from d1max_pct_scan.source_route_ros import from_message, to_message
    cache, source = snapshot_fixture()
    value = from_message(source).payload()
    value[field] = 'd'*64
    if field == 'source_map_sha256':
        value['geometry_evidence'][field] = value[field]
    foreign = to_message(RouteSnapshot.create(value), session_id=source.session_id,
        task_id=source.task_id, route_id=source.route_id, epoch=source.localization_epoch,
        seed_id=source.localization_seed_id, stamp=source.path.header.stamp)
    assert from_message(foreign)  # Valid typed geometry alone is not this configured map.
    assert cache.receive_snapshot(foreign) is None
    assert cache.displayed_path() is None


def test_snapshot_nanosecond_source_order_does_not_round_to_unix_float_time():
    cache, source = snapshot_fixture()
    source.path.header.stamp.sec = 1800000000
    source.path.header.stamp.nanosec = 123400000
    assert cache.receive_snapshot(source) == source.path
    new = deepcopy(source)
    new.task_id = 'one-nanosecond-later'
    new.path.header.stamp.nanosec += 1
    assert cache.receive_snapshot(new) == new.path
    assert cache.last_snapshot_stamp_ns == 1800000000123400001


def test_snapshot_production_callback_publishes_once_and_context_change_clears_order():
    from d1max_pct_scan.committed_route_view import publish_route_snapshot
    cache, source = snapshot_fixture()
    published = []
    publisher = SimpleNamespace(publish=lambda path: published.append(deepcopy(path)))
    publish_route_snapshot(cache, publisher, source)
    publish_route_snapshot(cache, publisher, source)
    assert published == [source.path]
    assert cache.select_context('new-session', 'new-map')
    assert cache.displayed_path() is None and cache.last_snapshot_stamp_ns == 0
    publish_route_snapshot(cache, publisher, source)
    assert published == [source.path]


def test_view_session_provenance_reuses_only_pinned_original_artifact_hashes(tmp_path):
    from d1max_pct_scan.committed_route_view import session_route_provenance
    paths = {name: str(tmp_path/name) for name in ('map_pcd', 'tomogram_npz', 'planning_manifest')}
    session = dict(paths, frame_id='d1max_loc_map', input_hashes={
        paths['map_pcd']: 'a'*64, paths['tomogram_npz']: 'b'*64,
        paths['planning_manifest']: 'c'*64})
    assert session_route_provenance(session) == dict(frame_id='d1max_loc_map',
        source_map_sha256='a'*64, tomogram_sha256='b'*64, conditioning_sha256='c'*64)
    assert session_route_provenance(dict(frame_id='d1max_loc_map')) == {'frame_id': 'd1max_loc_map'}


def test_snapshot_same_context_selection_keeps_provenance_until_explicit_reconfiguration():
    cache, source = snapshot_fixture()
    path = cache.receive_snapshot(source)
    assert not cache.select_context(cache.session_id, cache.map_version_id)
    assert cache.displayed_path() == path
    new_provenance = dict(cache.expected_provenance, source_map_sha256='e'*64)
    assert cache.select_context(cache.session_id, cache.map_version_id,
                                 expected_provenance=new_provenance)
    assert cache.displayed_path() is None
    assert cache.receive_snapshot(source) is None


@pytest.mark.parametrize('hash_value', ['', 'not-a-sha256', 'G'*64])
def test_view_does_not_invent_a_valid_configured_provenance_hash(hash_value):
    with pytest.raises(ValueError, match='committed_route_view_provenance'):
        CommittedRouteView('session', 'map', expected_provenance={'source_map_sha256': hash_value})
