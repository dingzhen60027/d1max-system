import numpy as np
import pytest

from d1max_pct_planner.measured_grid import MeasuredGrid
from d1max_pct_planner.preview_core import PreviewSelection, finite_xyz, unit_quaternion


@pytest.fixture
def selection(tmp_path):
    free = np.ones((12, 12), dtype=bool)
    free[0] = False
    grid = tmp_path / 'grid.npz'
    np.savez(grid, support=free, obstacles=~free, free=free,
             height=np.full((12, 12), -0.65), origin=[0., 0.], resolution=.2)
    return PreviewSelection(MeasuredGrid(grid, optimization_guard_cells=0), .08)


@pytest.mark.parametrize('bad', [[1, 2], [1, 2, float('nan')], [0, 0, float('inf')], [10001, 0, 0]])
def test_invalid_coordinates(bad):
    with pytest.raises(ValueError):
        finite_xyz(bad)


def test_free_xyz_edit_preserved_but_floating_plan_rejected(selection):
    selection.set_point('start', [1., 1., -.65])
    selection.set_point('goal', [1.5, 1.5, 4.])
    assert selection.snapshot()['goal_xyz'] == [1.5, 1.5, 4.]
    with pytest.raises(ValueError, match='Z differs'):
        selection.request()
    assert selection.points['goal'][2] == 4.


def test_snap_only_explicitly_changes_z(selection):
    selection.set_point('start', [1., 1., 2.])
    selection.snap_ground('start')
    assert list(selection.points['start']) == [1., 1., -.65]


def test_unknown_not_snapped_to_neighbor(selection):
    selection.set_point('start', [.1, 1., 1.])
    with pytest.raises(ValueError, match='outside measured'):
        selection.snap_ground('start')
    assert list(selection.points['start']) == [.1, 1., 1.]


def test_require_both_and_separate_xy(selection):
    with pytest.raises(ValueError, match='Select start'):
        selection.request()
    selection.set_point('start', [1., 1., -.65])
    with pytest.raises(ValueError, match='Select goal'):
        selection.request()
    selection.set_point('goal', [1.01, 1.01, -.65])
    with pytest.raises(ValueError, match='one grid cell'):
        selection.request()


def test_identical_feedback_does_not_invalidate_revision(selection):
    assert selection.set_point('start', [1., 1., -.65])
    revision = selection.revision
    assert not selection.set_point('start', [1., 1., -.65])
    assert selection.revision == revision


def test_result_retains_selected_endpoint_xyz(selection):
    selection.set_point('start', [1., 1., -.62])
    selection.set_point('goal', [1.5, 1.5, -.67])
    request = selection.request()
    result = {'generation': request['generation'],
              'path': [[1., 1., -.65], [1.2, 1.2, -.65], [1.5, 1.5, -.65]]}
    points = selection.checked_result(result, request)
    np.testing.assert_array_equal(points[0], [1., 1., -.62])
    np.testing.assert_array_equal(points[-1], [1.5, 1.5, -.67])
    assert result['path'][0][2] == -.65  # No mutation of native result.


def test_edit_and_clear_revoke_stale_result(selection):
    selection.set_point('start', [1., 1., -.65])
    selection.set_point('goal', [1.5, 1.5, -.65])
    request = selection.request()
    result = {'generation': request['generation'], 'path': [[1., 1., -.65], [1.5, 1.5, -.65]]}
    selection.set_point('goal', [1.7, 1.5, -.65])
    assert selection.checked_result(result, request) is None
    selection.clear()
    assert selection.checked_result(result, request) is None
    assert selection.snapshot() == {'start_xyz': None, 'goal_xyz': None}


def test_wrong_native_endpoints_rejected(selection):
    selection.set_point('start', [1., 1., -.65])
    selection.set_point('goal', [1.5, 1.5, -.65])
    request = selection.request()
    with pytest.raises(ValueError, match='endpoint contract'):
        selection.checked_result({'generation': request['generation'],
                                 'path': [[.9, 1., -.65], [1.5, 1.5, -.65]]}, request)


@pytest.mark.parametrize('bad', [[0, 0, 0, 0], [0, 0, 1], [0, 0, float('nan'), 1],
                               [0, 0, float('inf'), 1]])
def test_invalid_pose_editor_orientation(bad):
    with pytest.raises(ValueError):
        unit_quaternion(bad)


def test_pose_orientation_is_separate_from_position_planning(selection):
    selection.set_point('start', [1., 1., -.65])
    selection.set_point('goal', [1.5, 1.5, -.65])
    request = selection.request()
    assert selection.set_orientation('goal', [0., 0., 2., 2.])
    revision = selection.orientation_revision
    # Double-covered quaternion has no meaningful pose change.
    assert not selection.set_orientation('goal', [0., 0., -1., -1.])
    assert selection.orientation_revision == revision
    assert selection.request() == request
    np.testing.assert_allclose(selection.orientations['goal'], [0., 0., 2**-.5, 2**-.5])
    assert selection.orientation_snapshot()['orientation_semantics'] == 'editor_preview_only_xyz_planner'


def test_pose_orientation_survives_xyz_edit_ground_and_reactivation(selection):
    selection.set_point('goal', [1.5, 1.5, -.65])
    selection.set_orientation('goal', [1., 1., 1., 1.])
    selection.set_point('goal', [1.5, 1.5, 3.])
    selection.snap_ground('goal')
    selection.activate('goal', [0., 0.])
    np.testing.assert_array_equal(selection.orientations['goal'], [.5, .5, .5, .5])
    selection.clear()
    assert selection.orientation_snapshot()['goal_orientation_xyzw'] is None
    selection.activate('goal', [1., 1.])
    np.testing.assert_array_equal(selection.orientations['goal'], [0., 0., 0., 1.])


def test_pose_orientation_requires_existing_marker(selection):
    with pytest.raises(ValueError, match='Place the endpoint'):
        selection.set_orientation('goal', [0., 0., 0., 1.])


def test_activation_does_not_need_a_pointcloud_hit(selection):
    assert selection.activate('start', [1., 1.])
    assert selection.validate('start') is not None
    revision = selection.revision
    assert not selection.activate('start', [2., 2.])
    assert selection.revision == revision


def test_activation_preserves_arbitrary_edited_xyz(selection):
    selection.set_point('start', [1., 1., 5.])
    revision = selection.revision
    assert not selection.activate('start', [0., 0.])
    assert selection.revision == revision
    np.testing.assert_array_equal(selection.points['start'], [1., 1., 5.])
