"""Bounded experimental exclusion contracts; no ROS initialization or hardware."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
import json
import os
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest
import yaml

from d1max_localization.math_utils import compose, inverse
from d1max_pct_scan.preview_ray_exclusion import from_session
from d1max_pct_scan.ray_projection import (
    AwaitingCoverage, Limits, PreviewExcludedScan, ProjectionError,
    RayProjectorCore, session_settings, transform_many)
from test_ray_projection import (
    CTX, EPOCH, IDENTITY, decode, feed, planar, project, raw_points, session_config)


def preview_session(mode='preview_drop_rays', sensor=0, pad=0.):
    session, _ = session_config()
    session.update(mode='LIVE_VISUALIZATION_NO_MOTION', motion_control_enabled=False,
        preview_ray_exclusion=dict(schema=1, mode=mode, frame_id='body',
            calibration_verified=False, pad_m=pad, boxes=[dict(name='suspect-return',
                sensor_id=sensor, min_m=[.4, -.1, -.1], max_m=[.6, .1, .1])]))
    return session


def configured_core(exclusion=None, body=IDENTITY, ray=IDENTITY):
    value = RayProjectorCore(Limits(), preview_exclusion=exclusion)
    value.reset(CTX)
    value.set_extrinsics(body_to_tracking=body, ray_to_tracking=ray)
    return value


def mixed_points(sensor=0):
    points = raw_points(n=5, sensor=sensor, origin=(-.73, .02, 0.))
    # First/last points match, leaving no zero-offset or scan-end point after filtering.
    points['x'] = [.5, .7, .5, .8, .5]
    return points


def test_absent_configuration_is_byte_identical_and_audit_never_removes_a_ray():
    assert from_session({}, 'body') is None
    points = mixed_points()
    baseline = configured_core()
    explicit_none = configured_core(from_session({}, 'body'))
    audit = configured_core(from_session(preview_session('audit'), 'body'))
    for value in (baseline, explicit_none, audit):
        feed(value)
    expected = project(baseline, decode(points))
    unchanged = project(explicit_none, decode(points))
    inspected = project(audit, decode(points))
    assert unchanged.points.tobytes() == inspected.points.tobytes() == expected.points.tobytes()
    assert unchanged.exclusion_digest == ''
    assert inspected.exclusion_matched == 3 and inspected.exclusion_dropped == 0


@pytest.mark.parametrize('value', [None, {}, [], False, 'off'])
def test_explicit_ambiguous_configuration_is_rejected(value):
    session = preview_session()
    session['preview_ray_exclusion'] = value
    with pytest.raises(ValueError):
        from_session(session, 'body')


@pytest.mark.parametrize('key,value', [
    ('mode', 'LIVE_NAVIGATION'), ('mode', None),
    ('motion_control_enabled', True), ('motion_control_enabled', 0),
    ('motion_control_enabled', 'false'), ('motion_control_enabled', None),
    ('motion', {}), ('perception_backend', 'deskewed_cloud'), ('perception_backend', None),
])
def test_exclusion_requires_explicit_no_motion_ray_session(key, value):
    session = preview_session()
    session[key] = value
    with pytest.raises(ValueError):
        from_session(session, 'body')


@pytest.mark.parametrize('value', [None, {}, False])
def test_explicit_exclusion_cannot_hide_behind_disabled_projector_backend(value):
    session = dict(perception_backend='deskewed_cloud', preview_ray_exclusion=value)
    with pytest.raises(ProjectionError):
        session_settings(session, {})


@pytest.mark.parametrize('key,value', [
    ('schema', True), ('schema', 2), ('mode', 'self_filter'), ('frame_id', 'tracking'),
    ('calibration_verified', True), ('calibration_verified', 0),
    ('pad_m', True), ('pad_m', -.001), ('pad_m', .020001),
    ('pad_m', float('nan')), ('pad_m', float('inf')), ('boxes', []), ('boxes', {}),
    ('pad_m', [.02, .02]), ('pad_m', [.02, .02, -.001]),
    ('pad_m', [.02, True, 0.]), ('pad_m', [.02, .02, float('nan')]),
])
def test_configuration_rejects_unbounded_or_mislabelled_experiment(key, value):
    session = preview_session()
    session['preview_ray_exclusion'][key] = value
    with pytest.raises(ValueError):
        from_session(session, 'body')


@pytest.mark.parametrize('key,value', [
    ('sensor_id', True), ('sensor_id', 2), ('name', ''), ('name', 'a'*65),
    ('min_m', [.4, 0.]), ('min_m', [True, 0., 0.]), ('min_m', [float('nan'), 0., 0.]),
    ('max_m', [.4, .1, .1]), ('max_m', [.650001, .1, .1]),
    ('max_m', [.6, float('inf'), .1]),
])
def test_box_schema_and_extent_rejection(key, value):
    session = preview_session()
    session['preview_ray_exclusion']['boxes'][0][key] = value
    with pytest.raises(ValueError):
        from_session(session, 'body')


def test_unknown_keys_duplicate_names_and_excess_boxes_are_rejected():
    examples = []
    session = preview_session()
    session['preview_ray_exclusion']['invented_free_space'] = True
    examples.append(session)
    session = preview_session()
    session['preview_ray_exclusion']['boxes'][0]['clear_unknown'] = True
    examples.append(session)
    session = preview_session()
    session['preview_ray_exclusion']['boxes'] *= 2
    examples.append(session)
    session = preview_session()
    box = session['preview_ray_exclusion']['boxes'][0]
    session['preview_ray_exclusion']['boxes'] = [dict(box, name=str(i)) for i in range(5)]
    examples.append(session)
    for session in examples:
        with pytest.raises(ValueError):
            from_session(session, 'body')


@pytest.mark.parametrize('count', [3, 4])
def test_three_or_four_bounded_boxes_still_select_exact_sensor_and_union(count):
    session = preview_session(pad=[.02, .02, 0.])
    boxes = [dict(name=str(i), sensor_id=i % 2,
                  min_m=[-.8+.4*i, -.1, -.1], max_m=[-.6+.4*i, .1, .1])
             for i in range(count)]
    session['preview_ray_exclusion']['boxes'] = boxes
    exclusion = from_session(session, 'body')
    centres = np.array([(np.array(b['min_m'])+b['max_m'])/2 for b in boxes])
    for sensor in (0, 1):
        np.testing.assert_array_equal(exclusion.matches(centres, sensor),
                                      [b['sensor_id'] == sensor for b in boxes])
    # Four boxes do not change any individual extent restriction.
    session['preview_ray_exclusion']['boxes'][-1]['max_m'][1] = .151
    with pytest.raises(ValueError, match='not_local'):
        from_session(session, 'body')


def current_geometry_session():
    session = dict(mode='LIVE_VISUALIZATION_NO_MOTION', motion_control_enabled=False,
                   perception_backend='per_sensor_rays')
    fragment = Path(__file__).resolve().parents[1]/'config/near_body_preview_20260927.yaml'
    session.update(yaml.safe_load(fragment.read_text()))
    return session


def test_current_three_cluster_fragment_does_not_expand_z_or_remove_outside_returns():
    exclusion = from_session(current_geometry_session(), 'd1max_loc_base_link')
    assert len(exclusion.boxes) == 3 and exclusion.pad_m == (.02, .02, 0.)
    for box in exclusion.boxes:
        centre = (np.array(box.minimum)+box.maximum)/2
        ground = centre.copy()
        ground[2] = .005776817141662471*ground[0]+.01202982176734893*ground[1]-.531076144766088
        above, below, outside = centre.copy(), centre.copy(), centre.copy()
        above[2] = box.maximum[2] + 1e-6
        below[2] = box.minimum[2] - 1e-6
        outside[0] = box.maximum[0] + .020001
        points = np.array([centre, ground, above, below, outside])
        np.testing.assert_array_equal(exclusion.matches(points, box.sensor_id),
                                      [True, False, False, False, False])
        assert not exclusion.matches(points, 1-box.sensor_id).any()
        # This is an explicit dangerous counterexample, not a safety assertion:
        # an external obstacle endpoint at the same centre is ALSO hidden.
        assert exclusion.matches(np.array([centre]), box.sensor_id)[0]


@pytest.mark.parametrize('change', [dict(mode='LIVE_NAVIGATION'),
                                  dict(motion_control_enabled=True), dict(motion={})])
def test_current_three_cluster_fragment_cannot_be_used_by_execution(change):
    session = current_geometry_session()
    session.update(change)
    with pytest.raises(ValueError, match='explicit_no_motion'):
        from_session(session, 'd1max_loc_base_link')


def test_padded_box_beyond_per_axis_near_body_bound_is_rejected():
    session = preview_session(pad=.02)
    session['preview_ray_exclusion']['boxes'][0].update(
        min_m=[1.18, -.02, -.02], max_m=[1.2, .02, .02])
    with pytest.raises(ValueError):
        from_session(session, 'body')


def test_endpoint_boundary_padding_sensor_selection_and_overlaps_use_union():
    session = preview_session(pad=.02)
    first = session['preview_ray_exclusion']['boxes'][0]
    session['preview_ray_exclusion']['boxes'].append(dict(deepcopy(first), name='overlap'))
    exclusion = from_session(session, 'body')
    lower, upper = np.array(first['min_m'])-.02, np.array(first['max_m'])+.02
    points = np.array([lower, upper, [.5, 0., 0.], [upper[0]+1e-6, 0., 0.]])
    np.testing.assert_array_equal(exclusion.matches(points, 0), [True, True, True, False])
    assert not exclusion.matches(points, 1).any()
    value = configured_core(exclusion)
    feed(value)
    result = project(value, decode(mixed_points()))
    assert result.exclusion_matched == result.exclusion_dropped == 3


def test_xy_only_padding_does_not_expand_vertical_endpoint_exclusion():
    horizontal = from_session(preview_session(pad=[.02, .02, 0.]), 'body')
    isotropic = from_session(preview_session(pad=.02), 'body')
    points = np.array([[.39, 0., 0.], [.5, .11, 0.], [.5, 0., -.11], [.5, 0., .11]])
    np.testing.assert_array_equal(horizontal.matches(points, 0), [True, True, False, False])
    np.testing.assert_array_equal(isotropic.matches(points, 0), [True, True, True, True])
    assert horizontal.digest != isotropic.digest


def test_real_external_obstacle_inside_box_is_also_hidden_and_outside_obstacle_retained():
    # This limitation is deliberate and is why this feature cannot authorize motion:
    # geometric membership cannot distinguish a robot return from a real obstacle.
    points = mixed_points()
    original_bytes = points.tobytes()
    exclusion = from_session(preview_session(), 'body')
    value, baseline = configured_core(exclusion), configured_core()
    feed(value)
    feed(baseline)
    expected = project(baseline, decode(points))
    result = project(value, decode(points))
    assert result.points.tobytes() == expected.points[[1, 3]].tobytes()
    assert result.exclusion_matched == result.exclusion_dropped == 3
    assert result.start_ns == EPOCH and result.end_ns == EPOCH+100000000
    assert result.alignment_ns == expected.alignment_ns
    assert result.points['offset_time'].tolist() == [25000000, 75000000]
    for name in ('intensity', 'sensor_id', 'ring', 'offset_time', 'source_index',
                 'timestamp', 'source_timestamp', 'raw_timestamp'):
        assert result.points[name].tobytes() == points[name][[1, 3]].tobytes()
    assert points.tobytes() == original_bytes  # no mutation of raw evidence


def test_other_sensor_is_byte_preserved_including_its_actual_origin():
    value, baseline = configured_core(from_session(preview_session(sensor=0), 'body')), configured_core()
    feed(value)
    feed(baseline)
    points = mixed_points(sensor=1)
    result, expected = project(value, decode(points)), project(baseline, decode(points))
    assert result.points.tobytes() == expected.points.tobytes()
    assert result.exclusion_matched == result.exclusion_dropped == 0


def test_body_membership_uses_both_nonidentity_extrinsics_before_map_motion():
    body, ray = planar(.4043, .02, -1.56646), planar(.08, -.03, .4)
    raw = mixed_points()
    body_points = np.column_stack([raw[name] for name in ('x', 'y', 'z')])
    ray_points = transform_many(inverse(compose(body, ray)), body_points)
    for axis, name in enumerate(('x', 'y', 'z')):
        raw[name] = ray_points[:, axis]
    value = configured_core(from_session(preview_session(), 'body'), body, ray)
    baseline = configured_core(None, body, ray)
    for current in (value, baseline):
        feed(current, correction=lambda t: planar(3., -2., .3))
    actual, expected = project(value, decode(raw)), project(baseline, decode(raw))
    assert actual.points.tobytes() == expected.points[[1, 3]].tobytes()


def test_removed_final_points_still_require_full_original_pose_coverage():
    value = configured_core(from_session(preview_session(), 'body'))
    feed(value, count=4)  # ends at 60 ms; retained point at 25 ms is covered
    points = mixed_points()
    points['x'][3] = .5  # all endpoints after 25 ms would otherwise be removed
    with pytest.raises(AwaitingCoverage, match='coverage'):
        project(value, decode(points))
    assert value.sequence == 0 and value.last_input == [-1, -1]


def test_removed_first_point_still_requires_original_start_pose_coverage():
    value = configured_core(from_session(preview_session(), 'body'))
    feed(value, start=EPOCH+20000000)
    with pytest.raises(ProjectionError, match='before_pose_history'):
        project(value, decode(mixed_points()))


def test_all_filtered_scan_keeps_counts_but_creates_no_observation_or_sequence():
    exclusion = from_session(preview_session(), 'body')
    value = configured_core(exclusion)
    feed(value)
    points = raw_points(n=5)
    points['x'] = .5
    with pytest.raises(PreviewExcludedScan) as raised:
        project(value, decode(points))
    assert raised.value.count == 5 and raised.value.digest == exclusion.digest
    assert value.sequence == 0 and value.last_input == [-1, -1]


def test_configuration_is_deeply_immutable_and_worker_digest_rejects_stale_commit():
    session = preview_session()
    exclusion = from_session(session, 'body')
    session['preview_ray_exclusion']['boxes'][0]['min_m'][0] = -99
    assert exclusion.boxes[0].minimum[0] == .4
    with pytest.raises(FrozenInstanceError):
        exclusion.mode = 'audit'
    with pytest.raises(TypeError):
        exclusion.boxes[0].minimum[0] = 0.
    value = configured_core(exclusion)
    feed(value)
    snapshot = value.projection_snapshot()
    result = project(snapshot, decode(mixed_points()))
    assert snapshot.preview_exclusion is exclusion
    assert value.sequence == 0 and value.last_input == [-1, -1]
    value.preview_exclusion = from_session(preview_session(pad=.01), 'body')
    with pytest.raises(ProjectionError):
        value.commit_projection(result)
    assert value.sequence == 0 and value.last_input == [-1, -1]


@pytest.mark.parametrize('value', [None, {}, False, preview_session()['preview_ray_exclusion']])
def test_motion_preparation_rejects_any_explicit_exclusion_before_other_preparation(tmp_path, value):
    from d1max_pct_scan import live_session
    path = tmp_path/'excluded.yaml'
    path.write_text(yaml.safe_dump({'preview_ray_exclusion': value}))
    with patch.object(live_session, 'load_config') as load, \
         patch.object(live_session, 'prepare') as prepare, \
         patch.object(live_session.subprocess, 'Popen') as spawn:
        with pytest.raises(ValueError, match='exclusion'):
            live_session.prepare_motion(path)
        load.assert_not_called()
        prepare.assert_not_called()
        spawn.assert_not_called()


@pytest.mark.parametrize('value', [None, {}, False, preview_session()['preview_ray_exclusion']])
def test_owned_run_rejects_edited_motion_session_before_any_child_spawn(tmp_path, monkeypatch, value):
    from d1max_pct_scan import live_session
    identifier = 'preview-exclusion-test'
    (tmp_path/'session.json').write_text(json.dumps(dict(id=identifier, mode='LIVE_NAVIGATION',
        motion_control_enabled=True, preview_ray_exclusion=value)))
    monkeypatch.setenv('INVOCATION_ID', 'offline-unit-identity')
    own_unit = dict(MainPID=str(os.getpid()), Description=live_session.OWNER+identifier)
    with patch.object(live_session, 'unit', side_effect=[own_unit, {'ActiveState': 'inactive'}]), \
         patch.object(live_session.subprocess, 'Popen') as spawn:
        with pytest.raises(ValueError, match='exclusion'):
            live_session.run(tmp_path)
        spawn.assert_not_called()
