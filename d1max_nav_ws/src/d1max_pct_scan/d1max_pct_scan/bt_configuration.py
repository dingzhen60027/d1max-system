"""Actual BehaviorTree.CPP session wiring; no ROS, SDK or process side effects."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml
from .bt_follow_policy import FOLLOW_DEFAULTS, validate_follow_policy
from .live_runtime import file_sha256

ORCHESTRATOR = 'behaviortree_cpp_v3'
FILES = ('navigation.xml', 'bt.yaml', 'bt_adapter.yaml', 'bt_lifecycle.yaml')
PREFIX = '/d1max/live_planning/'
WORKER_REMAPS = {
    PREFIX+'goal': PREFIX+'bt/worker_goal',
    PREFIX+'goal3d': PREFIX+'bt/worker_goal3d',
    PREFIX+'cancel': PREFIX+'bt/worker_cancel',
    PREFIX+'reference_path': PREFIX+'bt/worker_reference',
    PREFIX+'reference_refresh_request': PREFIX+'bt/worker_reference_refresh',
}


def worker_arguments():
    return [part for source, target in WORKER_REMAPS.items()
            for part in ('-r', source+':='+target)]


def route_provenance(session):
    """Pin the actual artifacts, not a path or the conditioning parent hash."""
    import json
    manifest = json.loads(Path(session['planning_manifest']).read_text())
    source = file_sha256(session['map_pcd'])
    if source != manifest['source_sha256']:
        raise ValueError('route source map contract mismatch')
    return dict(expected_source_map_sha256=source,
        expected_tomogram_sha256=file_sha256(session['tomogram_npz']),
        expected_conditioning_sha256=file_sha256(session['planning_manifest']))


def lifecycle_parameters():
    return dict(autostart=True, node_names=['d1max_navigation_bt'],
                bond_timeout=4.0, attempt_respawn_reconnection=False)


def localization_index_deadline(directory):
    """The native registration index budget, separate from pose verification."""
    snapshot=Path(directory)/'localization.yaml'
    config=yaml.safe_load(snapshot.read_text()) if snapshot.is_file() else {}
    params=config.get('lio_localizer',{}).get('ros__parameters',{})
    value=params.get('global_relocalization.registration.index_timeout_s',120.)
    if type(value) not in (int,float) or not math.isfinite(value) or not 0.<value<=180.:
        raise ValueError('bounded_localization_index_deadline_required')
    return float(value)


def prepare_tree(directory, session, workspace):
    """Snapshot editable task policy with the same lifetime as planner params."""
    directory = Path(directory)
    source = Path(session.get('behavior_tree_xml') or Path(workspace)/
        'src/d1max_navigation_bt/trees/navigate_committed_route.xml')
    if not source.is_absolute() or not source.is_file():
        raise ValueError('Missing absolute BehaviorTree XML')
    tree = source.read_text()
    root = ET.fromstring(tree)
    if root.tag != 'root' or root.find('BehaviorTree') is None:
        raise ValueError('Invalid BehaviorTree.CPP XML')
    route_deadline = session['result_timeout_s']
    startup_deadline = session.get('warmup_timeout_s', 60.)
    if (type(route_deadline) not in (int, float) or not math.isfinite(route_deadline)
            or not 5. <= route_deadline <= 60.
            or type(startup_deadline) not in (int, float) or not math.isfinite(startup_deadline)
            or not 10. <= startup_deadline <= 60.):
        raise ValueError('bounded_global_route_and_startup_deadlines_required')
    # Session XML and the independent action watchdog consume the SAME
    # end-to-end compute deadline, never 60 active seconds behind a 10s worker.
    for node in root.iter('ComputeGlobalRoute'):
        node.set('timeout_s', str(float(route_deadline)))
        node.set('startup_timeout_s',str(float(startup_deadline)))
    index_deadline=localization_index_deadline(directory)
    for node in root.iter('WaitForInitialLocalization'):
        node.set('warmup_timeout_s',str(index_deadline))
    (directory/'navigation.xml').write_text(ET.tostring(root, encoding='unicode'))
    params = dict(session_id=session['id'], frame_id=session['frame_id'],
        planning_frame='d1max_multifloor_planning',
        tree_xml=str(directory/'navigation.xml'), tick_hz=10., health_timeout_s=.6,
        cancellation_timeout_s=3., action_watchdog_timeout_s=3.,
        compute_route_timeout_s=float(route_deadline),
        worker_startup_timeout_s=float(startup_deadline),
        arrival_evidence_timeout_s=4., preview_only=True)
    provenance=route_provenance(session)
    params['initial_localization_map_sha256']=provenance['expected_source_map_sha256']
    frames = session['navigation_contract']['frames']
    adapter = dict(session_id=session['id'], localization_session_id=session['id'],
        execution_mode='preview', map_frame=session['frame_id'],
        body_frame=frames['body_frame'], result_timeout_s=session['result_timeout_s'],
        worker_startup_timeout_s=float(startup_deadline),
        cancel_timeout_s=2.5, freshness_s=session['freshness_s'], worker_status_timeout_s=1.,
        tracking_frame=frames['tracking_frame'],
        body_height=session['body_height'], goal_xy_tolerance_m=.20, goal_z_tolerance_m=.15)
    adapter.update(provenance)
    policy = session.get('follow_recovery_policy', {})
    if not isinstance(policy, dict) or set(policy)-set(FOLLOW_DEFAULTS):
        raise ValueError('Unknown FollowRoute recovery policy configuration')
    adapter.update({key: policy.get(key, value) for key, value in FOLLOW_DEFAULTS.items()})
    validate_follow_policy(adapter)
    for filename, values in (('bt.yaml', params), ('bt_adapter.yaml', adapter),
                             ('bt_lifecycle.yaml', lifecycle_parameters())):
        (directory/filename).write_text(yaml.safe_dump({'/**': {'ros__parameters': values}}))


def validate_tree_wiring(directory, session):
    if session.get('task_orchestrator') != ORCHESTRATOR:
        raise ValueError('Navigation requires BehaviorTree.CPP; prepare a new session')
    if session.get('motion_control_enabled') is not False:
        raise ValueError('Behavior tree worker integration is preview-only pending execution acceptance')
    directory = Path(directory)
    bt = yaml.safe_load((directory/'bt.yaml').read_text())['/**']['ros__parameters']
    adapter = yaml.safe_load((directory/'bt_adapter.yaml').read_text())['/**']['ros__parameters']
    lifecycle = yaml.safe_load((directory/'bt_lifecycle.yaml').read_text())['/**']['ros__parameters']
    if lifecycle != lifecycle_parameters():
        raise ValueError('Behavior tree lifecycle contract mismatch')
    if any(adapter.get(key) != value for key, value in route_provenance(session).items()):
        raise ValueError('Behavior tree route provenance contract mismatch')
    validate_follow_policy(adapter)
    expected = ((bt.get('session_id'), session['id']),
        (bt.get('initial_localization_map_sha256'),route_provenance(session)['expected_source_map_sha256']),
        (bt.get('compute_route_timeout_s'), float(session['result_timeout_s'])),
        (bt.get('worker_startup_timeout_s'),float(session.get('warmup_timeout_s',60.))),
        (adapter.get('result_timeout_s'), session['result_timeout_s']),
        (adapter.get('worker_startup_timeout_s'), float(session.get('warmup_timeout_s',60.))),
        (bt.get('frame_id'), session['frame_id']),
        (bt.get('planning_frame'), 'd1max_multifloor_planning'),
        (bt.get('tree_xml'), str(directory/'navigation.xml')),
        (bt.get('preview_only'), True),
        (adapter.get('session_id'), session['id']),
        (adapter.get('localization_session_id'), session['id']),
        (adapter.get('execution_mode'), 'preview'),
        (adapter.get('map_frame'), session['frame_id']),
        (adapter.get('body_frame'), session['navigation_contract']['frames']['body_frame']),
        (adapter.get('tracking_frame'), session['navigation_contract']['frames']['tracking_frame']),
        (adapter.get('body_height'), session['body_height']))
    if any(actual != value or type(actual) is not type(value) for actual, value in expected):
        raise ValueError('Behavior tree/worker session contract mismatch')
    tree = ET.parse(directory/'navigation.xml').getroot()
    if any(float(node.get('timeout_s','nan')) != float(session['result_timeout_s'])
           or float(node.get('startup_timeout_s','nan')) != float(session.get('warmup_timeout_s',60.))
           for node in tree.iter('ComputeGlobalRoute')):
        raise ValueError('Behavior tree/worker route deadline mismatch')
    if any(float(node.get('warmup_timeout_s','nan'))!=localization_index_deadline(directory)
           for node in tree.iter('WaitForInitialLocalization')):
        raise ValueError('Behavior tree/localization index deadline mismatch')
