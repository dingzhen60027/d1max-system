"""Real TomogramMap/height-field selection; no ROS node or replacement map."""
import json
import os
from pathlib import Path
import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramMap, TomogramError
from d1max_pct_scan.navigation_surfaces import NavigationSurfaceResolver, NavigationSurfaceError
from d1max_pct_scan.live_goal_selection import LiveGoalSelection
from d1max_pct_scan.pointcloud_helpers.ground_path_bridge import FloorField, GroundPathBridge, StairGroundMapping


def stair_model():
    axis = np.linspace(-1., 1., 21)
    xy = np.array([[x, y] for x in axis for y in axis])
    floors = {}
    for name, source_z, target_z, band in [('floor1', -.1, 0., (-2., 2.3)),
                                         ('floor2', 3.07, 3., (2.7, 7.))]:
        floors[name] = FloorField(name, target_z, band,
            dict(xy=xy, z=np.full(len(xy), source_z), slopes=np.zeros_like(xy)),
            xy, {'conditioning_support_radius_m': .12})
    protected = [dict(min=[-.31, -.81, -1.], max=[.31, .81, 4.])]
    bridge = GroundPathBridge(floors, protected)
    data = np.zeros((5, 5, 21, 21), np.float32)
    for layer, z in enumerate([0., 1., 1.5, 2., 3.]):
        data[3, layer] = z
    data[4] = data[3]+.7
    tomogram = TomogramMap(dict(data=data, resolution=.1, center=[0., 0.], slice_h0=0., slice_dh=.5),
        unknown_ceiling_policy='reject', max_ground_step_m=.17)
    settings = dict(floor_z_ranges={'lower': [-.3, .3], 'upper': [2.7, 3.3]},
        limits={'max_selection_height_error_m': .08},
        stair_roi=dict(min=[-.4, -.9, -1.], max=[.4, .9, 3.5]),
        anchors={'entry': {'xyz': [0., -.8, 0.]}, 'landing': {'xyz': [0., 0., 1.5]},
                 'exit': {'xyz': [0., -.8, 3.]}})
    resolver = NavigationSurfaceResolver(tomogram, bridge, settings)
    editor = LiveGoalSelection(tomogram, bridge, settings['floor_z_ranges'], route_settings=settings)
    return resolver, editor


def regularized_stair_model():
    """Same native cells, with an explicit observed source/planning relation."""
    resolver, editor = stair_model()
    planning = np.array([[x, y, z] for z in (0., 1., 1.5, 2., 3.)
                         for x, y in ((0., -.4), (0., 0.))], np.float64)
    source = planning.copy()
    source[:, 2] -= .08
    ids = np.repeat(np.arange(1, 6, dtype=np.int32), 2)
    descriptor = dict(source_sha256='a'*64, mapping_sha256='b'*64,
        surfaces=[dict(id=i, kind='landing_plane' if i == 3 else 'flight_support_envelope')
                  for i in range(1, 6)], max_displacement_m=.08,
        association_max_distance_m=.12)
    resolver.bridge.stair_mapping = StairGroundMapping(descriptor,
        dict(source_indices=np.arange(len(source), dtype=np.int64),
             source_xyz=source, planning_xyz=planning, surface_id=ids), source_xyz=source)
    return resolver, editor


@pytest.mark.parametrize('planning,region', [([0., -.4, 0.], 'floor1'),
    ([0., -.4, 1.], 'stair_lower'), ([0., 0., 1.5], 'landing'),
    ([0., -.4, 2.], 'stair_upper'), ([0., -.4, 3.], 'floor2')])
def test_regularized_stair_editor_and_admission_share_source_ground(planning, region):
    resolver, editor = regularized_stair_model()
    source = np.asarray(planning)-[0., 0., .08]
    selected = resolver.resolve_goal(source, snap=True)
    assert selected['source_xyz'] == pytest.approx(source)
    assert selected['planning_xyz'] == pytest.approx(planning)
    assert selected['region_id'] == region
    from_planning = resolver.resolve_goal(planning, snap=True, frame_id=resolver.bridge.planning_frame)
    assert from_planning['source_xyz'] == pytest.approx(source)
    assert from_planning['planning_xyz'] == pytest.approx(planning)
    displayed = resolver.bridge.to_localization_ground(np.array([planning]), ['stairs']).xyz[0]
    assert selected['source_xyz'] == pytest.approx(displayed)
    report = editor.edit(source)
    assert report['valid'] and report['xyz'] == pytest.approx(source)
    assert report['planning_xyz'] == pytest.approx(planning)


def test_regularized_body_height_uses_measured_source_not_planning_height():
    resolver, _ = regularized_stair_model()
    selected = resolver.resolve_body([0., -.4, 1.42], [.49, .51])
    assert selected['body_height_m'] == pytest.approx(.5)
    assert selected['source_xyz'] == pytest.approx([0., -.4, .92])
    assert selected['planning_xyz'] == pytest.approx([0., -.4, 1.])


def test_regularized_stairs_do_not_admit_missing_mapping_or_blocked_surface():
    resolver, _ = regularized_stair_model()
    with pytest.raises(NavigationSurfaceError):
        resolver.resolve_goal([0., .5, 1.42], snap=True)
    resolver.tomogram.cost[1, 10, 6] = 100.
    with pytest.raises(TomogramError) as error:
        resolver.resolve_goal([0., -.4, .92], snap=True)
    assert error.value.code == 'pct_cost_blocked'
    # Outside the stair ROI the existing flat-floor correction is unchanged.
    flat = resolver.resolve_goal([.6, 0., -.1])
    assert flat['source_xyz'] == pytest.approx([.6, 0., -.1])
    assert flat['planning_xyz'] == pytest.approx([.6, 0., 0.])


@pytest.mark.parametrize('point,region', [([0., -.4, 1.], 'stair_lower'),
    ([0., 0., 1.5], 'landing'), ([0., -.4, 2.], 'stair_upper')])
def test_protected_stair_and_platform_goals_keep_true_xyz(point, region):
    resolver, editor = stair_model()
    result = resolver.resolve_goal(point)
    assert result['source_xyz'] == result['planning_xyz'] == point
    assert result['valid'] and result['region_id'] == region and result['floor_id'] is None
    report = editor.edit(point)
    assert report['valid'] and report['region_id'] == region and report['xyz'] == point
    assert report['ground_z'] == point[2] and report['height_error_m'] == 0.
    json.dumps(report, allow_nan=False)


def test_platform_route_classification_is_shared_with_native_not_a_tiny_anchor_radius():
    resolver, _ = stair_model()
    assert resolver.resolve_goal([0., .5, 1.5])['region_id'] == 'landing'
    # The routing token alone does not grant missing native support.
    resolver.tomogram.ground_known[2, 10, 15] = False
    with pytest.raises(NavigationSurfaceError):
        resolver.resolve_goal([0., .5, 1.5])


def test_floor_mapping_and_legacy_planning_frame_are_not_relabelled():
    resolver, _ = stair_model()
    for source, planning, region in [([.6, 0., -.1], [.6, 0., 0.], 'floor1'),
                                     ([.6, 0., 3.07], [.6, 0., 3.], 'floor2')]:
        result = resolver.resolve_goal(source)
        assert result['region_id'] == result['floor_id'] == region
        assert result['source_xyz'] == pytest.approx(source)
        assert result['planning_xyz'] == pytest.approx(planning)
        old = resolver.resolve_goal(planning, frame_id=resolver.bridge.planning_frame)
        assert old['source_xyz'] == pytest.approx(source)


@pytest.mark.parametrize('layer,z,region', [(0, .2, 'stair_lower'), (4, 2.8, 'stair_upper')])
def test_protected_tread_in_floor_routing_band_keeps_real_height(layer, z, region):
    resolver, _ = stair_model()
    resolver.tomogram.ground[layer, 10, 6] = z
    result = resolver.resolve_goal([0., -.4, z])
    assert result['region_id'] == region
    assert result['floor_id'] is None
    assert result['source_xyz'] == pytest.approx([0., -.4, z])
    assert result['source_xyz'] == result['planning_xyz']
    resolver.tomogram.cost[layer, 10, 6] = 100.
    with pytest.raises(TomogramError) as error:
        resolver.resolve_goal([0., -.4, z])
    assert error.value.code == 'pct_cost_blocked'


@pytest.mark.parametrize('z,floor', [(0., 'floor1'), (3., 'floor2')])
def test_stair_roi_does_not_relabel_portal_level_floor_support(z, floor):
    resolver, _ = stair_model()
    selected = resolver.resolve_goal([0., -.4, z], region='stairs')
    assert selected['region_id'] == selected['floor_id'] == floor
    assert selected['source_xyz'] == selected['planning_xyz'] == [0., -.4, z]


def test_free_xyz_and_layer_choose_true_stair_surface_without_xy_repair():
    _, editor = stair_model()
    editor.set_mode('free')
    report = editor.edit([.01, -.4, 1.12])
    assert report['valid'] and report['height_error_m'] == pytest.approx(.12)
    assert report['xyz'] == [.01, -.4, 1.12]
    report = editor.snap_ground()
    assert report['valid'] and report['xyz'] == [.01, -.4, 1.]
    report = editor.set_layer(3)
    assert report['valid'] and report['xyz'] == [.01, -.4, 2.]


@pytest.mark.parametrize('fault,code', [('cost', 'pct_cost_blocked'),
    ('headroom', 'insufficient_headroom'), ('unknown_ceiling', 'unobserved_ceiling')])
def test_stair_region_does_not_bypass_native_cost_or_clearance(fault, code):
    resolver, editor = stair_model()
    if fault == 'cost':
        resolver.tomogram.cost[1, 10, 6] = 100.
    elif fault == 'headroom':
        resolver.tomogram.headroom[1, 10, 6] = .2
    else:
        resolver.tomogram.ceiling_known[1, 10, 6] = False
    with pytest.raises(TomogramError) as error:
        resolver.resolve_goal([0., -.4, 1.])
    assert error.value.code == code
    report = editor.edit([0., -.4, 1.])
    assert not report['valid'] and report['reason_code'] == code
    assert editor.feedback() == report


def test_unknown_support_wrong_region_and_boundary_are_not_guessed():
    resolver, _ = stair_model()
    with pytest.raises(NavigationSurfaceError):
        resolver.resolve_goal([0., -.4, 1.], region='floor2')
    resolver.tomogram.ground_known[1, 10, 6] = False
    with pytest.raises(ValueError):
        resolver.resolve_goal([0., -.4, 1.])
    with pytest.raises(NavigationSurfaceError, match='outside'):
        resolver.resolve_goal([20., -.4, 1.])
    resolver, _ = stair_model()
    resolver.tomogram.cost[1, 11, 6] = 100.
    with pytest.raises(TomogramError):
        resolver.resolve_goal([.05, -.4, 1.])  # exact cell boundary checks both sides


def test_body_support_is_measured_unique_not_nominal_height_subtraction():
    resolver, _ = stair_model()
    result = resolver.resolve_body([0., -.4, 1.53], [.4, .6])
    assert result['source_xyz'] == [0., -.4, 1.]
    assert result['body_height_m'] == pytest.approx(.53)
    with pytest.raises(NavigationSurfaceError, match='unique'):
        resolver.resolve_body([0., -.4, 2.05], [.4, 1.1])


def test_body_support_coalesces_same_measured_ground_in_multiple_slices():
    resolver, _ = stair_model()
    resolver.tomogram.ground[2] = resolver.tomogram.ground[1]
    result = resolver.resolve_body([0., -.4, 1.53], [.25, .85])
    assert result['source_xyz'] == [0., -.4, 1.]
    assert result['equivalent_layers'] == [1, 2]


def test_goal_region_menu_does_not_add_stairs_to_initial_floor_choices():
    _, editor = stair_model()
    assert [item['floor_id'] for item in editor.floor_choices] == ['floor1', 'floor2']
    assert [item['region_id'] for item in editor.region_choices] == ['floor1', 'floor2', 'stairs']
    editor.edit([0., -.4, 1.])
    assert editor.set_floor('stairs')['valid']
    assert editor.feedback()['region_constraint'] == 'stairs'
    editor.set_floor(None)
    assert editor.region_constraint is None


@pytest.mark.parametrize('point,floor,region,outside', [
    ([0., -.4, .2], 'floor1', 'stair_lower', [.6, 0., -.1]),
    ([0., -.4, 3.], 'floor2', 'floor2', [.6, 0., 3.07]),
])
def test_stairs_editing_region_includes_floor_band_entry_exit_only_inside_roi(point, floor, region, outside):
    resolver, editor = stair_model()
    if floor == 'floor1':
        resolver.tomogram.ground[0, 10, 6] = .2
    selected = resolver.resolve_goal(point, region='stairs')
    assert selected['valid'] and selected['region_id'] == region
    assert selected['floor_id'] == (floor if region == floor else None)
    assert selected['source_xyz'] == pytest.approx(point)
    assert editor.edit(point)['valid']
    assert editor.set_floor('stairs')['valid']
    assert editor.feedback()['region_id'] == region
    # The same floor outside the staircase is not included by this menu item.
    assert resolver.resolve_goal(outside)['region_id'] == floor
    with pytest.raises(NavigationSurfaceError):
        resolver.resolve_goal(outside, region='stairs')


def test_releasing_invalid_explicit_region_revalidates_existing_draft():
    resolver, editor = stair_model()
    editor.set_mode('free')
    assert editor.edit([0., -.4, 1.])['valid']
    editor.set_floor('floor2')
    assert not editor.edit([0., -.4, 1.])['valid']
    assert editor.set_floor(None)['valid']
    editor.set_layer(4)
    assert not editor.edit([0., -.4, 1.])['valid']
    assert editor.set_layer(-1)['valid']


def test_automatic_ground_drag_climbs_measured_steps_without_floor_lock():
    resolver, editor = stair_model()
    # One actual sampled stair surface: 10 cm rise per measured cell. Other
    # slices remain unchanged, so layer/height selection still matters.
    for y in range(2, 19):
        resolver.tomogram.ground[0, :, y] = (y-2)*.1
        resolver.tomogram.headroom[0, :, y] = .7
    for y in range(2, 11):
        report = editor.edit([0., (y-10)*.1, 0.])
        assert report['valid'], report
        assert report['xyz'][2] == pytest.approx((y-2)*.1)
        assert editor.region_constraint is None


def test_actual_default_map_stair_surfaces_are_selectable_without_modifying_map():
    path = Path(os.environ.get('D1MAX_TEST_CONDITIONED_SESSION',
        Path(__file__).resolve().parents[3]/'experiments/navigation_runtime/release_preflight/session.json'))
    if not path.is_file():
        pytest.skip('requires the actual default crossfloor map package')
    session = json.loads(path.read_text())
    if not session.get('map_capabilities', {}).get('stairs_planning_enabled'):
        pytest.skip('default package has no declared staircase')
    editor = LiveGoalSelection.from_session(session)
    resolver = editor.surface_resolver
    assert resolver is not None
    tomogram = editor.tomogram
    selected = []
    low = np.maximum(tomogram.index(resolver.stair_roi[0, :2])-1, 0)
    high = np.minimum(tomogram.index(resolver.stair_roi[1, :2])+2,
                      [tomogram.nx, tomogram.ny])
    indices = np.argwhere(tomogram.valid[:, low[0]:high[0], low[1]:high[1]])
    indices[:, 1:] += low
    for layer, x, y in indices[::max(1, len(indices)//300)]:
        xyz = np.r_[tomogram.world([x, y]), tomogram.ground[layer, x, y]]
        if (not np.all(xyz >= resolver.stair_roi[0]) or not np.all(xyz <= resolver.stair_roi[1])
                or not resolver._protected(xyz) or not .1 < xyz[2] < 2.7):
            continue
        # Full native endpoint checks still apply at exact cell centres.
        try:
            value = resolver.resolve_goal(xyz, layer_id=int(layer), snap=True,
                                          frame_id=resolver.bridge.planning_frame)
        except ValueError:
            continue
        if value['region_id'] in ('stair_lower', 'stair_upper', 'landing'):
            selected.append(value)
    assert len(selected) >= 20
    assert {'stair_lower', 'stair_upper'} <= {item['region_id'] for item in selected}
    editor.set_mode('free')
    for value in selected[::max(1, len(selected)//30)]:
        report = editor.edit(value['source_xyz'])
        assert report['valid'], report
        expected = resolver.bridge.to_localization_ground(
            np.array([report['planning_xyz']]), ['stairs']).xyz[0]
        assert report['source_xyz'] == pytest.approx(expected)
        assert report['xyz'] == pytest.approx(value['source_xyz'])
        assert report['floor_id'] is None
    assert [item['floor_id'] for item in editor.floor_choices] == ['floor1', 'floor2']
