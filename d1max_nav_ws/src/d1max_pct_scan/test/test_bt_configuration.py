"""Offline wiring and diagnostics regressions: never initialize a ROS node."""
import ast
import json
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest
import yaml

from d1max_pct_scan import bt_configuration as bt
from d1max_pct_scan import live_session, navigation_contract
from d1max_pct_scan.bt_presentation import tree_presentation


def session(tmp_path):
    source, tomogram, manifest = (tmp_path/name for name in ('map.pcd', 'tomogram.npz', 'manifest.json'))
    source.write_bytes(b'source-map-fixture')
    tomogram.write_bytes(b'tomogram-fixture')
    manifest.write_text(json.dumps({'source_sha256': bt.file_sha256(source)}))
    return dict(id='session', task_orchestrator=bt.ORCHESTRATOR, motion_control_enabled=False,
        map_pcd=str(source), tomogram_npz=str(tomogram), planning_manifest=str(manifest),
        frame_id='d1max_loc_map', navigation_contract={'frames': {
            'body_frame': 'd1max_loc_base_link', 'tracking_frame': 'd1max_loc_tracking'}},
        result_timeout_s=60., freshness_s=.5, body_height=.55)


def test_actual_xml_and_action_parameters_are_snapshotted_without_processes(tmp_path):
    s = session(tmp_path)
    with patch.object(live_session.subprocess, 'Popen') as spawn:
        bt.prepare_tree(tmp_path, s, live_session.WS)
        bt.validate_tree_wiring(tmp_path, s)
        spawn.assert_not_called()
    tree = ET.parse(tmp_path/'navigation.xml').getroot()
    assert tree.attrib['main_tree_to_execute'] == 'NavigateCommittedRoute'
    assert tree.find('.//PauseOnUnavailable/SequenceStar/ComputeGlobalRoute') is not None
    assert tree.find('.//FollowCommittedRoute') is not None
    assert tree.find('.//RetryUntilSuccessful') is None
    assert set(bt.FILES) <= set(navigation_contract.bundle_files(s))
    params = yaml.safe_load((tmp_path/'bt.yaml').read_text())['/**']['ros__parameters']
    assert params['tree_xml'] == str(tmp_path/'navigation.xml')
    assert params['preview_only'] is True


def test_route_deadline_is_shared_while_heavy_startup_has_an_independent_budget(tmp_path):
    s=session(tmp_path);s.update(result_timeout_s=10.,warmup_timeout_s=60.)
    bt.prepare_tree(tmp_path,s,live_session.WS)
    bt.validate_tree_wiring(tmp_path,s)
    params=yaml.safe_load((tmp_path/'bt.yaml').read_text())['/**']['ros__parameters']
    adapter=yaml.safe_load((tmp_path/'bt_adapter.yaml').read_text())['/**']['ros__parameters']
    compute=ET.parse(tmp_path/'navigation.xml').find('.//ComputeGlobalRoute')
    assert float(compute.get('timeout_s'))==params['compute_route_timeout_s']==adapter['result_timeout_s']==10.
    assert float(compute.get('startup_timeout_s'))==params['worker_startup_timeout_s']==adapter['worker_startup_timeout_s']==60.
    compute.set('timeout_s','60')
    tree=ET.parse(tmp_path/'navigation.xml');tree.find('.//ComputeGlobalRoute').set('timeout_s','60')
    tree.write(tmp_path/'navigation.xml')
    with pytest.raises(ValueError,match='deadline mismatch'):
        bt.validate_tree_wiring(tmp_path,s)


def test_localization_index_has_its_own_source_pinned_budget_not_a_longer_pose_wait(tmp_path):
    s=session(tmp_path);s['result_timeout_s']=10.
    s['behavior_tree_xml']=str(live_session.WS/'src/d1max_navigation_bt/trees/navigate_with_global_relocalization.xml')
    (tmp_path/'localization.yaml').write_text(yaml.safe_dump({'lio_localizer':{'ros__parameters':{
        'global_relocalization.registration.index_timeout_s':120.}}}))
    bt.prepare_tree(tmp_path,s,live_session.WS);bt.validate_tree_wiring(tmp_path,s)
    node=ET.parse(tmp_path/'navigation.xml').find('.//WaitForInitialLocalization')
    assert float(node.get('warmup_timeout_s'))==120.
    assert float(node.get('timeout_s'))==60.
    params=yaml.safe_load((tmp_path/'bt.yaml').read_text())['/**']['ros__parameters']
    assert params['initial_localization_map_sha256']==bt.file_sha256(s['map_pcd'])
    assert params['compute_route_timeout_s']==10.
    tree=ET.parse(tmp_path/'navigation.xml');tree.find('.//WaitForInitialLocalization').set('warmup_timeout_s','180')
    tree.write(tmp_path/'navigation.xml')
    with pytest.raises(ValueError,match='localization index deadline mismatch'):
        bt.validate_tree_wiring(tmp_path,s)


@pytest.mark.parametrize('timeout',[0.,-1.,180.001,float('nan'),float('inf'),True,'120'])
def test_localization_index_snapshot_cannot_request_an_unbounded_or_untyped_budget(tmp_path,timeout):
    (tmp_path/'localization.yaml').write_text(yaml.safe_dump({'lio_localizer':{'ros__parameters':{
        'global_relocalization.registration.index_timeout_s':timeout}}}))
    with pytest.raises(ValueError,match='bounded_localization_index_deadline_required'):
        bt.prepare_tree(tmp_path,session(tmp_path),live_session.WS)


@pytest.mark.parametrize('startup', [10.,60.])
def test_worker_startup_range_matches_adapter_and_bt_node_endpoints(tmp_path,startup):
    s=session(tmp_path);s.update(result_timeout_s=10.,warmup_timeout_s=startup)
    bt.prepare_tree(tmp_path,s,live_session.WS)
    bt.validate_tree_wiring(tmp_path,s)
    bt_params=yaml.safe_load((tmp_path/'bt.yaml').read_text())['/**']['ros__parameters']
    adapter=yaml.safe_load((tmp_path/'bt_adapter.yaml').read_text())['/**']['ros__parameters']
    compute=ET.parse(tmp_path/'navigation.xml').find('.//ComputeGlobalRoute')
    assert bt_params['worker_startup_timeout_s']==adapter['worker_startup_timeout_s']==startup
    assert float(compute.get('startup_timeout_s'))==startup


@pytest.mark.parametrize('startup', [9.999,60.001,120.,float('nan'),float('inf'),True,'60'])
def test_worker_startup_rejects_values_not_accepted_by_actual_adapter(tmp_path,startup):
    s=session(tmp_path);s.update(result_timeout_s=10.,warmup_timeout_s=startup)
    with pytest.raises(ValueError,match='bounded_global_route_and_startup_deadlines_required'):
        bt.prepare_tree(tmp_path,s,live_session.WS)


@pytest.mark.parametrize('file,key,value', [
    ('bt.yaml', 'session_id', 'old'), ('bt.yaml', 'preview_only', False),
    ('bt.yaml', 'tree_xml', '/tmp/not-the-effective-tree.xml'),
    ('bt.yaml', 'worker_startup_timeout_s', 40.),
    ('bt_adapter.yaml', 'worker_startup_timeout_s', 40.),
    ('bt_adapter.yaml', 'body_frame', 'wrong'),
    ('bt_adapter.yaml', 'body_height', 2.), ('bt_adapter.yaml', 'execution_mode', 'execution'),
])
def test_mismatched_action_worker_configuration_rejected(tmp_path, file, key, value):
    s = session(tmp_path)
    bt.prepare_tree(tmp_path, s, live_session.WS)
    config = yaml.safe_load((tmp_path/file).read_text())
    config['/**']['ros__parameters'][key] = value
    (tmp_path/file).write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='contract'):
        bt.validate_tree_wiring(tmp_path, s)


def test_xml_startup_budget_cannot_fork_from_owner_and_adapter(tmp_path):
    s=session(tmp_path);s.update(result_timeout_s=10.,warmup_timeout_s=60.)
    bt.prepare_tree(tmp_path,s,live_session.WS)
    tree=ET.parse(tmp_path/'navigation.xml')
    tree.find('.//ComputeGlobalRoute').set('startup_timeout_s','40')
    tree.write(tmp_path/'navigation.xml')
    with pytest.raises(ValueError,match='deadline mismatch'):
        bt.validate_tree_wiring(tmp_path,s)


def test_old_sessions_cannot_silently_run_the_previous_unowned_graph(tmp_path):
    with pytest.raises(ValueError, match='prepare a new session'):
        bt.validate_tree_wiring(tmp_path, {})


def test_global_worker_cannot_consume_public_goals_or_publish_public_reference():
    arguments = bt.worker_arguments()
    actual = dict(value.split(':=') for value in arguments[1::2])
    assert actual == bt.WORKER_REMAPS
    assert all(value.startswith(bt.PREFIX+'bt/worker_') for value in actual.values())
    assert {bt.PREFIX+n for n in ('goal', 'goal3d', 'cancel', 'reference_path',
                                  'reference_refresh_request')} == set(actual)
    source = Path(live_session.__file__).read_text()
    tree = ast.parse(source)
    # Verify the supervisor really uses this wiring, not an unused helper.
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
    calls = [n for n in ast.walk(run) if isinstance(n, ast.Call)]
    assert any(isinstance(n.func, ast.Name) and n.func.id == 'worker_arguments' for n in calls)
    names = [n.args[0].value for n in calls if isinstance(n.func, ast.Name) and n.func.id == 'spawn'
             and n.args and isinstance(n.args[0], ast.Constant)]
    assert names.count('navigator') == names.count('bt_adapters') == names.count('global') == 1


def test_tree_status_is_bounded_and_does_not_manufacture_motion_or_algorithm_success():
    result = tree_presentation(dict(phase='paused', task_id='s.1', root_status='RUNNING',
        active_node='WaitForNavigationInputs', reason='waiting_localization',
        nodes=[dict(name='ComputeRouteOnce', status='SUCCESS')]*1000,
        transitions=[dict(node='FollowRoute', previous='IDLE', current='RUNNING')]*1000))
    assert result['label'] == '等待输入恢复'
    assert len(result['nodes']) == 24 and len(result['transitions']) == 8
    assert len(json.dumps(result)) < 8000
    assert result['transitions'][0]['name'] == 'FollowRoute'
    assert not {'motion_enabled', 'ready', 'success'} & result.keys()
    assert tree_presentation({})['label'] == '等待任务状态'


def test_session_seal_includes_exact_xml_and_action_parameters(tmp_path):
    # Build the real config bundle while replacing only the expensive map-index
    # construction. The index bytes still participate in the real seal.
    def support(_session, directory):
        index = directory/'pct_ground_support.npz'
        index.write_bytes(b'offline fixture; not used by a planner')
        return dict(ground_support_index=str(index), ground_support_sha256='fixture',
            ground_support_source_pcd_sha256='fixture', ground_support_tomogram_sha256='fixture')
    with patch.object(live_session, 'ROOT', tmp_path), \
         patch.object(live_session, 'prepare_ground_support', support), \
         patch.object(live_session.subprocess, 'Popen') as process:
        directory, s = live_session.prepare()
        process.assert_not_called()
    navigation_contract.verify_bundle(directory, s)
    assert set(bt.FILES) <= set(s['bundle_fingerprints']['effective_files'])
    xml = directory/'navigation.xml'
    xml.write_text(xml.read_text().replace('timeout_s="30"', 'timeout_s="300"'))
    with pytest.raises(ValueError, match='bundle changed'):
        navigation_contract.verify_bundle(directory, s)
