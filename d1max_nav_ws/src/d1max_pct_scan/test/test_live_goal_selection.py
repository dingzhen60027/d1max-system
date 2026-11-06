"""Passive goal-editing regression on production surface/bridge contracts."""
from copy import deepcopy
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_planner import paths
from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_scan.live_global_contract import choose_floor_surface, floor_from_original_ground_z
from d1max_pct_scan.live_goal_selection import LiveGoalSelection
from d1max_pct_scan.source_identity import SourceIdentityBridge
from d1max_pct_scan.pointcloud_helpers.ground_path_bridge import FloorField, GroundPathBridge


def measured_model(*, duplicate_height=0., upper=True):
    """Actual TomogramMap and SourceIdentityBridge, not a substitute selector."""
    axis = np.linspace(-.4, .4, 17)
    lower_support = np.array([[x, y, 0.] for x in axis for y in axis])
    lower = SourceIdentityBridge(lower_support, [[5., 5.], [6., 6.]], floor_id='floor1')
    if upper:
        upper_support = lower_support + [0., 0., 3.]
        second = SourceIdentityBridge(upper_support, [[5., 5.], [6., 6.]], floor_id='floor2')
        bridge = SimpleNamespace(source_frame=lower.source_frame, planning_frame=lower.planning_frame,
            floors={'floor1': lower, 'floor2': second}, limits=None,
            protected_regions=lower.protected_regions)
    else:
        bridge = lower
    data = np.zeros((5, 3 if upper else 2, 9, 9), np.float32)
    data[3, 1] = duplicate_height
    if upper:
        data[3, 2] = 3.
    data[4] = data[3] + 2.
    tomogram = TomogramMap(dict(data=data, resolution=.1, center=[0., 0.],
        slice_h0=0., slice_dh=.5), unknown_ceiling_policy='reject')
    return LiveGoalSelection(tomogram, bridge, {'lower': (-.3, .3), 'upper': (2.7, 3.3)})


def assert_backend_accepts(model, report):
    xyz = report['xyz']
    floor = floor_from_original_ground_z(model.bridge, xyz)
    surface = choose_floor_surface(model.tomogram, xyz[:2], floor,
        model.floor_z_ranges, hint_z=xyz[2])
    assert floor == report['floor_id']
    assert surface['ground_z'] == report['ground_z']


def test_default_ground_follow_keeps_exact_xy_and_reports_native_evidence():
    model = measured_model()
    report = model.edit([.137, .063, .12])
    assert report['valid'] and report['selection_mode'] == 'ground'
    assert report['xyz'] == [.137, .063, 0.]
    assert report['floor_id'] == 'floor1' and report['layer_id'] == report['layer'] == 0
    assert report['cost'] == 0. and report['headroom_m'] == 2.
    assert report['height_error_m'] == 0. and report['equivalent_layers'] == [0, 1]
    assert report['motion_enabled'] is False
    assert_backend_accepts(model, report)
    json.dumps(report, allow_nan=False)


def test_ground_drag_preserves_floor_even_if_camera_feedback_changes_z():
    model = measured_model()
    assert model.edit([0., 0., 0.])['valid']
    report = model.edit([.05, -.03, 3.])
    assert report['valid'] and report['floor_id'] == 'floor1'
    assert report['xyz'] == [.05, -.03, 0.]
    assert_backend_accepts(model, report)


def test_explicit_floor_switch_and_free_z_selection_do_not_move_xy():
    model = measured_model()
    model.edit([.03, .07, 0.])
    report = model.set_floor('floor2')
    # A selected physical floor is an explicit Z edit, not nearest-XY repair.
    assert report['valid'] and report['xyz'] == [.03, .07, 3.]
    model.set_floor(None)
    model.set_mode('free')
    report = model.edit([.03, .07, .12])
    assert report['valid'] and report['xyz'] == [.03, .07, .12]
    assert report['floor_id'] == 'floor1'
    assert_backend_accepts(model, report)


def test_switching_to_free_z_does_not_retain_implicit_ground_floor_lock():
    model = measured_model()
    model.edit([0., 0., 0.])
    model.set_mode('free')
    report = model.edit([0., 0., 3.])
    assert report['valid'] and report['floor_id'] == 'floor2'
    assert_backend_accepts(model, report)


def test_free_height_uses_live_15cm_contract_not_legacy_8cm():
    model = measured_model()
    model.set_mode('free')
    good = model.edit([.03, .07, .12])
    assert good['valid'] and good['height_error_m'] == pytest.approx(.12)
    assert_backend_accepts(model, good)
    bad = model.edit([.03, .07, .16])
    assert not bad['valid'] and bad['reason_code'] == 'height_off_surface'
    assert bad['xyz'] == [.03, .07, .16]
    snap = model.snap_ground()
    assert snap['valid'] and snap['xyz'] == [.03, .07, 0.]


@pytest.mark.parametrize('fault,reason', [
    ('cost', 'pct_cost_blocked'), ('ceiling', 'insufficient_headroom'),
    ('unknown', 'no_traversable_surface'), ('outside', 'outside_map'),
])
def test_invalid_drafts_stay_editable_and_never_shift_xy(fault, reason):
    model = measured_model(upper=False)
    point = [.13, .07, 0.]
    if fault == 'cost':
        model.tomogram.cost[:, 5, 5] = 100.
    elif fault == 'ceiling':
        model.tomogram.headroom[:, 5, 5] = .2
    elif fault == 'unknown':
        model.tomogram.ground_known[:, 5, 5] = False
    else:
        point = [10., 10., 0.]
    report = model.edit(point)
    assert not report['valid'] and report['reason_code'] == reason
    assert report['xyz'][:2] == point[:2]
    assert report['detail']
    assert model.edit([-.1, -.1, 0.])['valid']


def test_explicit_native_slice_cannot_bypass_backend_ambiguous_height():
    model = measured_model(duplicate_height=.08)
    model.set_layer(0)
    report = model.edit([0., 0., 0.])
    assert not report['valid']
    assert report['reason_code'] == 'multiple_distinct_surfaces_on_declared_floor'
    assert report['xyz'] == [0., 0., 0.] and report['layer_id'] == 0


def test_missing_source_support_or_protected_stairs_remain_invalid():
    model = measured_model(upper=False)
    # PCT has measured support here; the original source support deliberately does not.
    report = model.edit([.4, .4, 0.])
    assert report['valid']
    model.bridge.protected_regions = [dict(min=[.3, .3, -1.], max=[.5, .5, 1.])]
    report = model.feedback()
    assert not report['valid'] and report['reason_code'] == 'goal_in_stair_transition_requires_floor_target'


def test_observed_pct_surface_cannot_replace_missing_original_source_support():
    model = measured_model(upper=False)
    support = model.bridge.support[model.bridge.support[:, 0] < -.2]
    model.bridge = SourceIdentityBridge(support, [[5., 5.], [6., 6.]], floor_id='floor1')
    report = model.edit([.3, 0., 0.])
    assert not report['valid']
    assert report['reason_code'] == 'source_3d_goal_z_does_not_uniquely_match_observed_floor'
    assert report['xyz'] == [.3, 0., 0.]


def test_failed_explicit_slice_selection_does_not_reappear_valid_on_status_tick():
    model = measured_model(upper=False)
    model.edit([0., 0., 0.])
    model.tomogram.cost[1, 4, 4] = 100.
    report = model.set_layer(1)
    assert not report['valid'] and report['reason_code'] == 'pct_cost_blocked'
    assert model.feedback() == report
    assert model.edit([.2, 0., 0.])['valid']


def test_surface_choices_expose_both_floors_without_silent_selection():
    model = measured_model()
    model.edit([0., 0., 0.])
    before = model.xyz
    surfaces = model.surfaces()
    assert [(value['floor_id'], value['layer_id']) for value in surfaces] == [('floor1', 0), ('floor2', 2)]
    assert model.xyz == before
    assert all(value['layer'] == value['layer_id'] for value in surfaces)


@pytest.mark.parametrize('operation,value', [('mode', 'unsupported'), ('floor', 'floor3'), ('layer', 900)])
def test_bad_editor_constraint_does_not_replace_draft(operation, value):
    model = measured_model()
    model.edit([0., 0., 0.])
    before = model.feedback()
    with pytest.raises(ValueError):
        getattr(model, 'set_' + operation)(value)
    assert model.feedback() == before


@pytest.fixture(scope='module')
def recorded_selection():
    explicit = os.environ.get('D1MAX_TEST_MAP_DIRECTORY')
    try:
        directory = Path(explicit).resolve() if explicit else paths.default_map_directory()
    except (ValueError, OSError):
        pytest.skip('requires the selected default source-identity map package')
    if not directory.is_dir():
        pytest.skip('requires the selected default source-identity map package')
    # A prepared-session identity without running ROS or re-reading source PCDs.
    from d1max_pct_scan.source_route import digest_file
    manifest = json.loads((directory / 'manifest.json').read_text())
    session = dict(frame_id=manifest['frame_id'], map_pcd=manifest['source_path'],
        planning_manifest=str(directory / 'manifest.json'), tomogram_npz=str(directory / 'tomogram.npz'),
        crossfloor_route_config=str(directory / 'route.yaml'))
    inputs = [directory / 'manifest.json', directory / 'tomogram.npz', directory / 'route.yaml',
              directory / manifest['source_indices_file']]
    session['input_hashes'] = {str(path.resolve()): digest_file(path) for path in inputs}
    session['input_hashes'][str(Path(manifest['source_path']).resolve())] = manifest['source_sha256']
    session['input_hashes'][str((directory / manifest['output_file']).resolve())] = manifest['output_sha256']
    patch = pytest.MonkeyPatch()
    def forbidden(*args, **kwargs):
        raise AssertionError('goal editor must not start ROS/processes or duplicate PCD loading')
    patch.setattr('subprocess.Popen', forbidden)
    patch.setattr('subprocess.run', forbidden)
    patch.setattr('d1max_pct_scan.pointcloud_helpers.pcd_io.read_pcd', forbidden)
    patch.setattr(SourceIdentityBridge, 'from_artifacts', forbidden)
    import rclpy
    patch.setattr(rclpy, 'init', forbidden)
    try:
        model = LiveGoalSelection.from_session(session)
    finally:
        patch.undo()
    return model, session


def test_actual_default_map_editor_matches_live_backend_and_is_json_safe(recorded_selection):
    model, _ = recorded_selection
    assert model.binding_status == 'verified_session_source_identity'
    accepted = 0
    for point in model.bridge.support[::max(1, len(model.bridge.support) // 120)]:
        result = model.edit(point)
        if result['valid']:
            assert result['xyz'][:2] == point[:2].tolist()
            assert_backend_accepts(model, result)
            json.dumps(result, allow_nan=False)
            accepted += 1
    assert accepted >= 10, 'actual map should supply admitted goals across several corridors'
    assert model.floor_choices == [dict(floor_id='floor1', label='一楼',
        z_range=list(model.floor_z_ranges['lower']))]


def test_real_map_loader_refuses_unsealed_or_foreign_binding(recorded_selection):
    _, session = recorded_selection
    wrong = deepcopy(session)
    wrong['input_hashes'][str(Path(session['planning_manifest']).resolve())] = '0' * 64
    with pytest.raises(ValueError, match='session_input_changed'):
        LiveGoalSelection.from_session(wrong)
    wrong = deepcopy(session)
    wrong['frame_id'] = 'another_map'
    with pytest.raises(ValueError, match='source_identity_binding_invalid'):
        LiveGoalSelection.from_session(wrong)


def test_actual_default_blocked_cells_remain_invalid_without_xy_repair(recorded_selection):
    model, _ = recorded_selection
    layer = 0
    blocked = np.argwhere(model.tomogram.ground_known[layer] &
                          (model.tomogram.cost[layer] > model.tomogram.COST_THRESHOLD))
    checked = 0
    for cell in blocked[::max(1, len(blocked) // 100)]:
        xy = model.tomogram.world(cell)
        z = float(model.tomogram.ground[(layer, *cell)])
        model.set_mode('free')
        model.set_layer(layer)
        report = model.edit([*xy, z])
        assert report['xyz'][:2] == xy.tolist()
        assert not report['valid'] and report['reason_code'] == 'pct_cost_blocked'
        checked += 1
    assert checked >= 10, 'actual map should retain blocked wall/obstacle cells'


def conditioned_model():
    """Real height-field bridge; the original ground is not the flat PCT Z."""
    model = measured_model()
    xy = model.bridge.floors['floor1'].support[:, :2]
    floors = {}
    for name, source_z, reference_z, band in (
            ('floor1', -.1, 0., (-2., 2.3)), ('floor2', 3.07, 3., (2.7, 7.))):
        floors[name] = FloorField(name, reference_z, band,
            dict(xy=xy, z=source_z + .02 * xy[:, 0],
                 slopes=np.tile([.02, 0.], (len(xy), 1))),
            xy, {'conditioning_support_radius_m': .12})
    bridge = GroundPathBridge(floors, [])
    return LiveGoalSelection(model.tomogram, bridge, model.floor_z_ranges)


def test_conditioned_floor_switch_keeps_public_source_ground_and_exact_xy():
    model = conditioned_model()
    lower = model.edit([.13, .07, -.0974])
    assert lower['valid'] and lower['xyz'] == pytest.approx([.13, .07, -.0974])
    assert lower['ground_z'] == pytest.approx(-.0974)
    assert lower['planning_ground_z'] == 0.
    assert lower['frame_id'] == 'd1max_loc_map'
    assert lower['projection_kind'] == 'non_rigid_ground_only_not_tf'
    upper = model.set_floor('floor2')
    assert upper['valid'] and upper['floor_id'] == 'floor2'
    assert upper['xyz'] == pytest.approx([.13, .07, 3.0726])
    assert model.selection.points['goal'].tolist() == [.13, .07, 3.]
    assert floor_from_original_ground_z(model.bridge, upper['xyz']) == 'floor2'
    assert upper['motion_enabled'] is False
    json.dumps(upper, allow_nan=False)


def test_conditioned_free_z_and_slice_selection_do_not_submit_planning_z():
    model = conditioned_model()
    model.edit([.03, .07, -.0994])
    model.set_layer(2)
    assert model.xyz == pytest.approx([.03, .07, 3.0706])
    model.set_floor(None)
    model.set_mode('free')
    point = [.03, .07, 3.1906]
    report = model.edit(point)
    assert report['valid'] and report['xyz'] == point
    assert report['height_error_m'] == pytest.approx(.12)
    assert report['planning_ground_z'] == 3.
    assert model.snap_ground()['xyz'] == pytest.approx([.03, .07, 3.0706])
    surfaces = model.surfaces()
    assert [s['floor_id'] for s in surfaces] == ['floor1', 'floor2']
    assert surfaces[1]['ground_z'] == pytest.approx(3.0706)


def test_conditioned_failed_source_projection_or_slice_stays_invalid_until_new_edit():
    model = conditioned_model()
    model.edit([0., 0., -.1])
    model.tomogram.cost[2, 4, 4] = 100.
    report = model.set_layer(2)
    assert not report['valid'] and report['reason_code'] == 'pct_cost_blocked'
    assert model.feedback() == report
    assert model.xyz == pytest.approx((0., 0., -.1))
    model.set_floor('floor2')
    assert model.edit([.3, 0., 3.076])['valid']
    report = model.edit([20., 20., 3.076])
    assert not report['valid'] and report['xyz'][:2] == [20., 20.]
    assert model.feedback() == report


def test_conditioned_stair_target_and_wrong_source_height_are_not_floor_goals():
    model = conditioned_model()
    model.bridge.protected_regions = [dict(min=[-.1, -.1, -2.], max=[.1, .1, 7.])]
    assert not model.edit([0., 0., -.1])['valid']
    model.set_mode('free')
    assert not model.edit([.3, 0., 1.5])['valid']


@pytest.fixture(scope='module')
def recorded_conditioned_selection():
    session_path = Path(os.environ.get('D1MAX_TEST_CONDITIONED_SESSION',
        Path(__file__).resolve().parents[3] / 'experiments/navigation_runtime/release_preflight/session.json'))
    if not session_path.is_file():
        pytest.skip('requires the actual formally prepared 10-02 crossfloor session')
    session = json.loads(session_path.read_text())
    if session.get('planning_scope') != 'global_route':
        pytest.skip('preflight session does not select the conditioned global-route package')
    return LiveGoalSelection.from_session(session), session


def test_actual_conditioned_package_both_floor_goals_have_source_ground_coordinates(recorded_conditioned_selection):
    model, session = recorded_conditioned_selection
    assert model.binding_status == 'verified_session_conditioned_ground_bridge'
    assert model.available_floors == ('floor1', 'floor2')
    assert len(session['map_runtime_dependencies']) == 14
    assert all(session['input_hashes'][path] == digest
               for path, digest in session['map_runtime_dependencies'].items())
    assert len(session['map_rebuild_input_references']) == 3
    assert all(ref['path'] not in session['map_runtime_dependencies']
               for ref in session['map_rebuild_input_references'])
    import yaml
    route = yaml.safe_load(Path(session['crossfloor_route_config']).read_text())
    for name, floor in [('start', 'floor1'), ('goal', 'floor2')]:
        planning = np.asarray(route['anchors'][name]['xyz'])
        source = model.bridge.to_localization_ground(planning[None, :], [floor]).xyz[0]
        model.set_floor(floor)
        report = model.edit(source)
        assert report['valid'] and report['floor_id'] == floor
        assert report['xyz'] == pytest.approx(source.tolist())
        assert report['xyz'][:2] == planning[:2].tolist()
        assert report['frame_id'] == 'd1max_loc_map'
        assert floor_from_original_ground_z(model.bridge, report['xyz']) == floor
        assert abs(report['xyz'][2] - planning[2]) > .001


@pytest.mark.parametrize('fault', ['missing_dependency', 'missing_evidence', 'missing_source_manifest',
    'missing_restore_evidence', 'missing_runtime_list', 'extra_runtime_dependency',
    'wrong_runtime_hash', 'wrong_dependency', 'frame', 'tomogram'])
def test_conditioned_loader_rejects_unpinned_or_mismatched_session(recorded_conditioned_selection, fault):
    _, session = recorded_conditioned_selection
    wrong = deepcopy(session)
    manifest = json.loads(Path(session['planning_manifest']).read_text())
    stage_path = str(Path(manifest['floor_processing_manifest']).resolve())
    if fault == 'missing_dependency':
        wrong['input_hashes'].pop(stage_path)
    elif fault == 'missing_evidence':
        wrong['input_hashes'].pop(str(Path(stage_path).parent/'audit/floor_evidence.npz'))
    elif fault == 'missing_source_manifest':
        wrong['input_hashes'].pop(str(Path(manifest['source_path']).parent/'manifest.json'))
    elif fault == 'missing_restore_evidence':
        dependency = manifest['configuration']['measured_restore']['evidence']['path']
        wrong['input_hashes'].pop(dependency)
    elif fault == 'missing_runtime_list':
        wrong.pop('map_runtime_dependencies')
    elif fault == 'extra_runtime_dependency':
        wrong['map_runtime_dependencies']['/unused/rebuild.yaml'] = '0'*64
    elif fault == 'wrong_runtime_hash':
        wrong['map_runtime_dependencies'][stage_path] = '0'*64
    elif fault == 'wrong_dependency':
        wrong['input_hashes'][stage_path] = '0'*64
    elif fault == 'frame':
        wrong['frame_id'] = 'd1max_multifloor_planning'
    else:
        wrong['input_hashes'][str(Path(session['tomogram_npz']).resolve())] = '0'*64
    with pytest.raises(ValueError, match='goal_editor_'):
        LiveGoalSelection.from_session(wrong)
