"""Stage boundary and physical-floor editor regressions, no ROS/SDK startup."""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_planner.tomogram_selection import TomogramSelection


def editor():
    data = np.zeros((5, 3, 9, 9), dtype=np.float32)
    data[3, 2] = 3.
    data[4] = data[3]+2.
    data[0, 2, 5, 4] = 50.
    grid = TomogramMap(dict(data=data, resolution=.2, center=[0., 0.], slice_h0=.5, slice_dh=.5))
    return TomogramSelection(grid, anchor_z=0., floor_ranges={'floor1': [-.2,.2], 'floor2': [2.8,3.2]})


def test_explicit_second_floor_preserves_xy_and_keeps_multiple_slices():
    e = editor()
    e.set_point('goal', [0., 0., 0.])
    e.set_active_floor('floor2')
    assert e.points['goal'][2] == 0.  # selecting a constraint is not a teleport
    e.snap_ground('goal')
    np.testing.assert_array_equal(e.validate('goal'), [0., 0., 3.])
    assert e.validation('goal')['floor_id'] == 'floor2'
    e.set_active_floor('floor1')
    e.snap_ground('goal')
    assert e.validation('goal')['floor_id'] == 'floor1'
    assert len([x for x in e.grid.sample_surfaces([0,0],deduplicate=False) if x['ground_z']==0]) == 2


def test_floor_change_does_not_mislabel_unchanged_endpoint():
    e = editor()
    e.set_point('start', [0.,0.,0.])
    e.set_active_floor('floor2')
    assert e.validation('start')['floor_id'] == 'floor1'


def test_no_xy_snap_no_borrowing_ground_from_other_floor():
    e = editor()
    e.set_active_floor('floor2')
    e.set_point('goal', [.2,0.,3.])
    assert not e.validation('goal')['valid']
    assert e.validation('goal')['reason_code'] == 'pct_cost_blocked'
    np.testing.assert_array_equal(e.points['goal'][:2], [.2,0.])
    e.set_point('goal', [50.,50.,3.])
    assert not e.validation('goal')['valid']
    np.testing.assert_array_equal(e.points['goal'][:2], [50.,50.])


def test_free_xyz_height_error_is_visible_and_explicit_snap_fixes_only_z():
    e = editor()
    e.set_active_floor('floor2')
    e.set_mode('free')
    e.set_point('goal', [0.,0.,3.4])
    assert e.validation('goal')['valid'] is False
    assert e.validation('goal')['height_error_m'] == pytest.approx(.4)
    e.snap_ground('goal')
    np.testing.assert_array_equal(e.validate('goal'), [0.,0.,3.])


def test_slices_and_masks_unchanged_by_floor_editor():
    e = editor()
    masks, data = e.grid.allowed.copy(), e.grid.data.copy()
    for floor in ('floor1','floor2',None):
        e.set_active_floor(floor)
        e.set_point('goal', [0.,0.,3.])
        e.snap_ground('goal')
    np.testing.assert_array_equal(e.grid.allowed, masks)
    np.testing.assert_array_equal(e.grid.data, data)


def test_global_stage_tree_uses_real_compute_and_no_fake_arrival():
    ws = Path(__file__).resolve().parents[3]
    tree = ET.parse(ws/'src/d1max_navigation_bt/trees/test_global_route_stage.xml')
    tags = [node.tag for node in tree.iter()]
    assert tags.count('ComputeGlobalRoute') == 1
    assert 'TaskContextValid' in tags and 'PauseOnUnavailable' in tags
    assert 'FollowCommittedRoute' not in tags and 'VerifyMeasuredArrival' not in tags


def test_stage_runner_has_no_sdk_controller_or_robot_connector():
    source = Path(__file__).resolve().parents[1]/'d1max_pct_scan/crossfloor_global_session.py'
    text = source.read_text()
    ast.parse(text)
    for module in ('sdk_writer', 'motion_tracker', 'single_floor_reference', 'live_scan_bridge', 'localization_session'):
        assert module not in text
    assert "'tcp/127.0.0.1:" in text or "f'tcp/127.0.0.1:" in text
    assert "'atomic_navigation_v3'" in text and "'single_floor_v3'" not in text
    assert "'goal_floor'" not in text or "goal_floor='floor2'" in text


def test_process_boundary_retains_crossfloor_semantics_and_terminal_failure():
    from d1max_pct_scan.native_global_worker import RESULT_FIELDS
    from d1max_pct_scan.bt_adapters import COMPUTE_FAILURE_STATES
    assert {'segments','anchors','layer_transitions','edge_legs','direction'} <= set(RESULT_FIELDS)
    assert 'result_rejected' in COMPUTE_FAILURE_STATES


@pytest.mark.parametrize('name,value', [('ROS_DOMAIN_ID','24'),
    ('D1MAX_NAV_ISOLATED','0'), ('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')])
def test_probe_rejects_non_private_transport_before_ros_init(monkeypatch,name,value):
    from d1max_pct_scan.global_stage_regression import require_private_domain
    for key, valid in {'ROS_DOMAIN_ID':'219', 'D1MAX_NAV_ISOLATED':'1',
                       'RMW_IMPLEMENTATION':'rmw_zenoh_cpp'}.items():
        monkeypatch.setenv(key,valid)
    require_private_domain()
    monkeypatch.setenv(name,value)
    with pytest.raises(ValueError,match='private offline Zenoh'):
        require_private_domain()


@pytest.mark.parametrize('name, expected', [('single_floor_v3',True),
    ('atomic_navigation_v3',True), ('legacy',False), ('',False)])
def test_atomic_wire_alias_does_not_change_floor_or_permission(name, expected):
    from d1max_pct_scan.atomic_navigation_inbox import uses_atomic_navigation
    assert uses_atomic_navigation(dict(pipeline_contract=name)) is expected
