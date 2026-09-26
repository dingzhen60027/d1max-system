import numpy as np
import pytest
from d1max_pct_planner.measured_grid import MeasuredGrid
from d1max_pct_scan.trajectory_guard import (AdmissionContext, GuardStatus, RejectionAudit,
                                           sample_checked_spline, spline_audit_payload)


def grid(tmp_path, blocked=None, step=False):
    free = np.ones((30, 30), dtype=bool)
    free[[0, -1]] = False
    free[:, [0, -1]] = False
    if blocked:
        free[blocked] = False
    height = np.zeros_like(free, dtype=float)
    if step:
        height[12:] = 0.3
    target = tmp_path / 'grid.npz'
    np.savez(target, free=free, support=free, obstacles=~free, height=height,
             origin=np.array([0., 0.]), resolution=0.2)
    return MeasuredGrid(target, minimum_clearance_m=0.2, optimization_guard_cells=0)


def spline():
    return {'order': 3, 'knots': np.arange(-3, 7, dtype=float),
            'points': [[0.5 + i * 0.5, 2.1, .55] for i in range(6)]}


def test_matches_vendor_deboor_domain_and_keeps_height(tmp_path):
    report, samples = sample_checked_spline(grid(tmp_path), **spline())
    # Native cubic uniform DeBoor: linear controls P0..P5 yield P1 at t=0,
    # P4 at t=3. This is not interpolation through the first/last controls.
    np.testing.assert_allclose(samples[0], [1.0, 2.1, .55])
    np.testing.assert_allclose(samples[-1], [2.5, 2.1, .55])
    assert report['duration'] == 3
    assert report['max_height_error_m'] == pytest.approx(0, abs=1e-12)
    assert report['arc_length_sampling_bound_m'] <= .025000001
    assert report['checked_cells'] > 5


def test_blocked_between_control_points_rejected(tmp_path):
    with pytest.raises(ValueError, match='blocked/unsupported'):
        sample_checked_spline(grid(tmp_path, blocked=(8, 10)), **spline())


def test_ground_step_rejected(tmp_path):
    with pytest.raises(ValueError, match='ground step'):
        sample_checked_spline(grid(tmp_path, step=True), **spline())


def test_height_off_ground_rejected_without_flattening(tmp_path):
    data = spline()
    data['points'] = [[x, y, 2.] for x, y, z in data['points']]
    with pytest.raises(ValueError, match='height_outside'):
        sample_checked_spline(grid(tmp_path), **data)


@pytest.mark.parametrize('kind', ['nan', 'order', 'knot_count', 'repeated', 'empty', 'budget'])
def test_malformed_spline_rejected(tmp_path, kind):
    data = spline()
    if kind == 'nan': data['points'][2][0] = np.nan
    if kind == 'order': data['order'] = 2
    if kind == 'knot_count': data['knots'] = data['knots'][:-1]
    if kind == 'repeated': data['knots'][2] = data['knots'][1]
    if kind == 'empty': data['points'] = []
    if kind == 'budget': data['points'][2][0] = 1e7
    with pytest.raises(ValueError):
        sample_checked_spline(grid(tmp_path), **data)


def task(generation=1, active=True):
    return dict(session_id='abc', generation=generation, active=active,
                issued_at=100., frame_id='d1max_loc_map')


def test_context_only_accepts_matching_live_generation():
    context = AdmissionContext('abc')
    context.receive_task(task(), 100., 10.)
    context.check('abc', 1, 'd1max_loc_map', 1, 100., 100.1, 10.1)
    for session, generation, frame, ident, start in [
            ('other', 1, 'd1max_loc_map', 1, 100.),
            ('abc', 2, 'd1max_loc_map', 1, 100.),
            ('abc', 1, 'odom', 1, 100.),
            ('abc', 1, 'd1max_loc_map', 1, 99.)]:
        with pytest.raises(ValueError):
            context.check(session, generation, frame, ident, start, 100.1, 10.1)
    context.last_id = 1
    with pytest.raises(ValueError, match='nonmonotonic'):
        context.check('abc', 1, 'd1max_loc_map', 1, 100., 100.1, 10.1)


def test_cancel_latched_once_and_old_generation_cannot_resume():
    context = AdmissionContext('abc')
    context.receive_task(task(), 100., 10.)
    assert context.revoke()
    assert not context.revoke()
    assert not context.receive_task(task(), 100.1, 10.1)
    next_task = task(2); next_task['issued_at'] = 100.1
    assert context.receive_task(next_task, 100.1, 10.1)
    assert context.revoke()


def test_inactive_generation_then_new_activation_is_supported():
    context = AdmissionContext('abc')
    assert context.receive_task(task(1, False), 100., 10.)
    assert not context.receive_task(task(1), 100., 10.)
    assert context.receive_task(task(2), 100., 10.)


def test_initial_generation_zero_inactive_is_valid_but_cannot_drive():
    context = AdmissionContext('abc')
    assert context.receive_task(task(0, False), 100., 10.)
    assert not context.task.active
    with pytest.raises(ValueError, match='invalid_task_generation'):
        context.receive_task(task(0, True), 100., 10.)


def test_old_planner_output_cannot_revoke_new_active_generation():
    context = AdmissionContext('abc')
    assert context.receive_task(task(1, False), 100., 10.)
    assert context.receive_task(task(2), 100., 10.)
    assert context.ignore_reason('abc', 1, 100) == 'unrelated_generation'
    assert context.ignore_reason('other', 2, 100) == 'unrelated_session'
    assert context.ignore_reason('abc', 3, 100) == 'unrelated_generation'
    assert context.task.active
    assert context.task.generation == 2
    assert not context.cancel_latched
    assert context.ignore_reason('abc', 2, 1) == ''


def test_cancelled_tasks_and_duplicate_splines_are_ignored():
    context = AdmissionContext('abc')
    context.receive_task(task(), 100., 10.)
    context.last_id = 10
    assert context.ignore_reason('abc', 1, 9) == 'obsolete_trajectory_id'
    assert context.task.active
    context.receive_task(task(2, False), 100., 10.)
    assert context.ignore_reason('abc', 1, 11) == 'no_active_task'
    assert not context.task.active


def test_unrelated_or_delayed_task_heartbeat_does_not_cancel_current():
    context = AdmissionContext('abc')
    context.receive_task(task(3), 100., 10.)
    old = task(2, False)
    assert not context.receive_task(old, 100.1, 10.1)
    old['session_id'] = 'other'
    assert not context.receive_task(old, 100.1, 10.1)
    assert context.task.active and context.task.generation == 3


def test_current_invalid_geometry_revokes_and_requires_new_generation(tmp_path):
    context = AdmissionContext('abc')
    context.receive_task(task(), 100., 10.)
    assert not context.ignore_reason('abc', 1, 1)
    with pytest.raises(ValueError, match='blocked/unsupported'):
        sample_checked_spline(grid(tmp_path, blocked=(8, 10)), **spline())
    assert context.revoke()
    assert not context.task.active
    assert not context.receive_task(task(), 100.1, 10.1)


def test_stale_task_has_no_permission_to_forward():
    context = AdmissionContext('abc')
    context.receive_task(task(), 100., 10.)
    with pytest.raises(ValueError, match='inactive_or_stale'):
        context.check('abc', 1, 'd1max_loc_map', 1, 100., 101., 11.)


def test_sample_densification_does_not_modify_measured_free_mask(tmp_path):
    measured = grid(tmp_path)
    before = measured.free.copy()
    sample_checked_spline(measured, **spline())
    np.testing.assert_array_equal(measured.free, before)


def test_rejection_remains_visible_after_ignored_stop_and_new_accepted_status():
    status = GuardStatus()
    status.rejected({'error': 'blocked cell (8, 10)', 'generation': 2, 'trajectory_id': 4})
    ignored = status.payload({'reason': 'ignored', 'generation': 3})
    assert ignored['last_error']['error'] == 'blocked cell (8, 10)'
    assert ignored['last_error']['generation'] == 2
    accepted = status.payload({'reason': 'accepted', 'generation': 4})
    assert accepted['last_error'] == ignored['last_error']


def test_audit_keeps_native_spline_and_is_bounded_without_deletion(tmp_path):
    import json
    writer = RejectionAudit(str(tmp_path), max_records=1)
    record = {'candidate': {'generation': 2, 'traj_id': 4,
                            'knots': [-3., -2., -1., 0., 1., 2., 3., 4., 5., 6.],
                            'pos_pts': [[.5, 2.1, .55], [1., 2.1, .55]], 'order': 3},
              'rejection': {'error': 'blocked'}}
    path = writer.write(record)
    assert json.loads(open(path).read()) == record
    assert writer.write(record) == ''
    restarted = RejectionAudit(str(tmp_path), max_records=1)
    assert restarted.write(record) == ''
    assert len(list(tmp_path.glob('rejected_*.json'))) == 1
    assert RejectionAudit('').write(record) == ''


def test_malformed_numbers_remain_readable_valid_json(tmp_path):
    import json
    writer = RejectionAudit(str(tmp_path))
    path = writer.write({'candidate': {'generation': 1, 'traj_id': 1, 'pos_pts': [[np.nan, 0, 0]]}})
    record = json.loads(open(path).read())
    assert record['candidate']['pos_pts'][0][0] == {'nonfinite': 'nan'}


def test_wire_spline_payload_records_frame_times_and_native_arrays():
    from types import SimpleNamespace as Ns
    raw = Ns(traj_id=7, start_time=Ns(sec=100, nanosec=500000000), order=3,
             knots=[-3., -2., -1., 0.], pos_pts=[Ns(x=1., y=2., z=.55)], yaw_pts=[], yaw_dt=0.)
    payload = spline_audit_payload(Ns(session_id='abc', generation=2, frame_id='map', trajectory=raw))
    assert payload['start_time'] == {'sec': 100, 'nanosec': 500000000}
    assert payload['pos_pts'] == [[1., 2., .55]]
    assert payload['knots'] == raw.knots
    assert payload['order'] == 3 and not payload['truncated']
