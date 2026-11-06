"""One mainline policy, real parameter generation, no ROS graph or SDK."""
from dataclasses import asdict
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from d1max_pct_scan.navigation_runtime_policy import (
    NavigationRuntimePolicy, load, validate_session_policy)
from d1max_pct_scan import bt_configuration


ROOT = Path(__file__).resolve().parents[3]


def test_checked_in_policy_has_one_common_episode_not_short_stage_abort():
    policy = load(ROOT/'src/d1max_pct_scan/config/navigation_runtime.yaml')
    assert policy == NavigationRuntimePolicy()
    params = policy.follow_parameters()
    assert {params[key] for key in params if key.endswith('_wait_s')} == {30.}
    assert params['follow_recovery_episode_s'] == 30.
    assert policy.bt_parameters()['execution_blocked_timeout_s'] == 30.
    assert policy.bt_parameters()['execution_recovery_time_s'] == params['follow_stable_reset_s'] == .6
    assert policy.health_parameters()['minimum_new_samples'] == 3


@pytest.mark.parametrize('key,value', [
    ('recovery_timeout_s', 0), ('recovery_timeout_s', float('inf')),
    ('stable_duration_s', True), ('stable_duration_s', 0),
    ('dependency_heartbeat_timeout_s', float('nan')),
    ('worker_startup_timeout_s', 61), ('global_route_timeout_s', '10'),
])
def test_policy_cannot_disable_validity_or_request_unbounded_wait(key, value):
    values = asdict(NavigationRuntimePolicy()); values[key] = value
    with pytest.raises(ValueError):
        NavigationRuntimePolicy.from_mapping(values)


def test_policy_rejects_unknown_missing_or_incoherent_configuration():
    values = asdict(NavigationRuntimePolicy())
    with pytest.raises(ValueError, match='exact_fields'):
        NavigationRuntimePolicy.from_mapping(dict(values, unknown_space_free=True))
    del values['stable_duration_s']
    with pytest.raises(ValueError, match='exact_fields'):
        NavigationRuntimePolicy.from_mapping(values)
    with pytest.raises(ValueError, match='cover_detection'):
        NavigationRuntimePolicy(recovery_timeout_s=1.)


def parameters(path):
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


def prepared_tree(tmp_path, policy):
    policy_source=tmp_path/'runtime_policy.yaml'
    policy_source.write_text(yaml.safe_dump(asdict(policy)))
    source = tmp_path/'map.pcd'; source.write_bytes(b'fixture')
    tomogram = tmp_path/'tomogram.npz'; tomogram.write_bytes(b'fixture')
    manifest = tmp_path/'manifest.json'
    manifest.write_text(json.dumps(dict(source_sha256=bt_configuration.file_sha256(source))))
    session = dict(id='fixture', map_pcd=str(source), tomogram_npz=str(tomogram),
        planning_manifest=str(manifest), frame_id='d1max_loc_map',
        runtime_policy=asdict(policy),runtime_policy_source=str(policy_source), freshness_s=.4, body_height=.55,
        result_timeout_s=policy.global_route_timeout_s,
        warmup_timeout_s=policy.worker_startup_timeout_s,
        readiness_timeout_s=policy.recovery_timeout_s,commit_wait_timeout_s=policy.recovery_timeout_s,
        component_startup_timeout_s=policy.worker_startup_timeout_s,
        navigation_contract={'frames': {'body_frame': 'base', 'tracking_frame': 'tracking'}})
    bt_configuration.prepare_tree(tmp_path, session, ROOT)
    # The real session assembles adapter execution parameters after generating
    # the tree. Carry the same BT arrival contract into this passive fixture;
    # a policy-only map cannot stand in for that completed launch assembly.
    adapter_path=tmp_path/'bt_adapter.yaml'
    adapter=yaml.safe_load(adapter_path.read_text())
    adapter['/**']['ros__parameters']['arrival_evidence_timeout_s']=parameters(
        tmp_path/'bt.yaml')['arrival_evidence_timeout_s']
    adapter_path.write_text(yaml.safe_dump(adapter))
    for name, values in (('component_health.yaml', policy.health_parameters()),
                         ('global.yaml', {'result_timeout_s': policy.global_route_timeout_s,
                            'warmup_timeout_s':policy.worker_startup_timeout_s,
                            'readiness_timeout_s':policy.recovery_timeout_s,
                            'commit_wait_timeout_s':policy.recovery_timeout_s})):
        (tmp_path/name).write_text(yaml.safe_dump({'/**': {'ros__parameters': values}}))
    return session


def test_same_policy_drives_real_tree_owner_and_adapter_generation(tmp_path):
    policy = NavigationRuntimePolicy(recovery_timeout_s=45., stable_duration_s=.8,
        global_route_timeout_s=12., component_callback_timeout_s=2., dependency_heartbeat_timeout_s=2.)
    session = prepared_tree(tmp_path, policy)
    validate_session_policy(tmp_path, session)
    owner = parameters(tmp_path/'bt.yaml'); adapter = parameters(tmp_path/'bt_adapter.yaml')
    for key, value in policy.follow_parameters().items():
        assert owner[key] == adapter[key] == value
    assert owner['dependency_recovery_timeout_s'] == owner['execution_blocked_timeout_s'] == 45.
    tree = ET.parse(tmp_path/'navigation.xml')
    assert float(tree.find('.//PauseOnUnavailable').get('timeout_s')) == 45.
    assert float(tree.find('.//ComputeGlobalRoute').get('timeout_s')) == 12.
    assert float(tree.find('.//ComputeGlobalRoute').get('readiness_timeout_s')) == 45.
    assert owner['compute_readiness_timeout_s']==adapter['readiness_timeout_s']==45.
    assert owner['health_timeout_s'] == .6  # source/control proof was not relaxed
    assert owner['preview_only'] is True


@pytest.mark.parametrize('name,key,value', [
    ('bt.yaml', 'execution_blocked_timeout_s', 20.),
    ('bt.yaml', 'follow_reference_wait_s', 5.),
    ('bt.yaml', 'dependency_heartbeat_timeout_s', .6),
    ('bt_adapter.yaml', 'follow_trajectory_wait_s', 20.),
    ('component_health.yaml', 'callback_timeout_s', .6),
    ('component_health.yaml', 'minimum_new_samples', True),
    ('global.yaml', 'result_timeout_s', 60.),
    ('global.yaml', 'warmup_timeout_s', 30.),
    ('global.yaml', 'readiness_timeout_s', 10.),
    ('global.yaml', 'commit_wait_timeout_s', 10.),
    ('bt.yaml', 'compute_readiness_timeout_s', 10.),
    ('bt_adapter.yaml', 'readiness_timeout_s', 10.),
])
def test_one_component_cannot_silently_fork_its_budget(tmp_path, name, key, value):
    session = prepared_tree(tmp_path, NavigationRuntimePolicy())
    content = yaml.safe_load((tmp_path/name).read_text())
    content['/**']['ros__parameters'][key] = value
    (tmp_path/name).write_text(yaml.safe_dump(content))
    with pytest.raises(ValueError, match='component_mismatch'):
        validate_session_policy(tmp_path, session)


def test_competing_override_and_tree_fork_are_rejected(tmp_path):
    session = prepared_tree(tmp_path, NavigationRuntimePolicy())
    session['follow_recovery_policy'] = {'follow_reference_wait_s': 2.}
    with pytest.raises(ValueError, match='competing_follow_override'):
        bt_configuration.prepare_tree(tmp_path, session, ROOT)
    with pytest.raises(ValueError, match='competing_follow_override'):
        validate_session_policy(tmp_path, session)
    del session['follow_recovery_policy']
    tree = ET.parse(tmp_path/'navigation.xml')
    tree.find('.//PauseOnUnavailable').set('timeout_s', '5')
    tree.write(tmp_path/'navigation.xml')
    with pytest.raises(ValueError, match='tree_mismatch'):
        validate_session_policy(tmp_path, session)


@pytest.mark.parametrize('arrival', [None, 0., 3., True])
def test_completed_session_cannot_fork_arrival_evidence_from_bt(tmp_path,arrival):
    session=prepared_tree(tmp_path,NavigationRuntimePolicy())
    adapter_path=tmp_path/'bt_adapter.yaml'
    adapter=yaml.safe_load(adapter_path.read_text())
    adapter['/**']['ros__parameters']['arrival_evidence_timeout_s']=arrival
    adapter_path.write_text(yaml.safe_dump(adapter))
    with pytest.raises(ValueError,match='arrival_evidence_mismatch'):
        validate_session_policy(tmp_path,session)


def test_session_mapping_cannot_fork_from_the_pinned_policy_file(tmp_path):
    session=prepared_tree(tmp_path,NavigationRuntimePolicy())
    altered=dict(session['runtime_policy'],recovery_timeout_s=45.)
    Path(session['runtime_policy_source']).write_text(yaml.safe_dump(altered))
    with pytest.raises(ValueError,match='source_mismatch'):
        validate_session_policy(tmp_path,session)


def test_long_monitor_gap_retains_graph_for_same_bounded_bt_recovery():
    from d1max_pct_scan.component_health import SupervisorHealth
    policy = NavigationRuntimePolicy()
    supervisor = SupervisorHealth('s', started=0., timeout_s=policy.recovery_timeout_s,
                                  startup_s=policy.worker_startup_timeout_s)
    report = dict(schema=1, session_id='s', sequence=1, ready=True, fatal=False)
    assert supervisor.check(report, monotonic=1.) == ''
    assert supervisor.check(report, monotonic=5.) == ''
    assert supervisor.check(report, monotonic=31.1) == 'component_health_monitor_stalled'
    # Replaying the same file did not renew liveness. The BT's independent
    # dependency lease already denied motion during this unavailable period.
