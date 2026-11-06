"""One operational policy, projected into existing components, not a navigator.

Freshness and collision evidence remain independent safety contracts. A task
has one recovery budget rather than an earlier reference/map sub-timeout that
kills it while the owner is still recovering. This file does not grant motion.
"""
from dataclasses import asdict, dataclass, fields
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class NavigationRuntimePolicy:
    global_route_timeout_s: float = 10.
    worker_startup_timeout_s: float = 60.
    recovery_timeout_s: float = 30.
    stable_duration_s: float = .6
    component_callback_timeout_s: float = 1.5
    dependency_heartbeat_timeout_s: float = 1.5

    def __post_init__(self):
        bounds = dict(global_route_timeout_s=(5., 60.), worker_startup_timeout_s=(10., 60.),
            recovery_timeout_s=(1., 120.), stable_duration_s=(.4, 3.),
            component_callback_timeout_s=(.5, 5.), dependency_heartbeat_timeout_s=(.6, 5.))
        for key, value in asdict(self).items():
            lo, hi = bounds[key]
            if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError('invalid_navigation_runtime_policy:' + key)
        if (self.stable_duration_s >= self.recovery_timeout_s
                or self.component_callback_timeout_s >= self.recovery_timeout_s
                or self.dependency_heartbeat_timeout_s >= self.recovery_timeout_s
                or self.component_callback_timeout_s >= self.worker_startup_timeout_s):
            raise ValueError('navigation_recovery_budget_must_cover_detection_and_stability')

    @classmethod
    def from_mapping(cls, value):
        if not isinstance(value, dict) or set(value) != {field.name for field in fields(cls)}:
            raise ValueError('navigation_runtime_policy_requires_exact_fields')
        return cls(**value)

    def follow_parameters(self):
        # Stage names explain where we wait. They cannot impose an earlier
        # independent abort than the BT's common recovery episode.
        return dict(follow_pose_wait_s=self.recovery_timeout_s,
            follow_map_wait_s=self.recovery_timeout_s,
            follow_reference_wait_s=self.recovery_timeout_s,
            follow_trajectory_wait_s=self.recovery_timeout_s,
            follow_recovery_episode_s=self.recovery_timeout_s,
            follow_stable_reset_s=self.stable_duration_s)

    def bt_parameters(self):
        return dict(self.follow_parameters(),
            execution_blocked_timeout_s=self.recovery_timeout_s,
            execution_recovery_time_s=self.stable_duration_s,
            dependency_heartbeat_timeout_s=self.dependency_heartbeat_timeout_s,
            dependency_recovery_timeout_s=self.recovery_timeout_s,
            dependency_stable_duration_s=self.stable_duration_s)

    def health_parameters(self):
        return dict(callback_timeout_s=self.component_callback_timeout_s,
            stable_duration_s=self.stable_duration_s, minimum_new_samples=3,
            startup_timeout_s=self.worker_startup_timeout_s)


def load(path):
    return NavigationRuntimePolicy.from_mapping(yaml.safe_load(Path(path).read_text()))


def session_policy(session):
    """Older sealed snapshots keep their old contract; new sessions pin one file."""
    value = session.get('runtime_policy')
    return None if value is None else NavigationRuntimePolicy.from_mapping(value)


def validate_session_policy(directory, session):
    """Reject divergent generated budgets before launching any component."""
    policy = session_policy(session)
    if policy is None:
        return
    source=session.get('runtime_policy_source')
    if not isinstance(source,str) or not Path(source).is_absolute() or load(source)!=policy:
        raise ValueError('navigation_runtime_policy_source_mismatch')
    if session.get('follow_recovery_policy'):
        raise ValueError('navigation_policy_has_competing_follow_override')
    if (session.get('result_timeout_s') != policy.global_route_timeout_s
            or session.get('warmup_timeout_s') != policy.worker_startup_timeout_s
            or session.get('component_startup_timeout_s') != policy.worker_startup_timeout_s
            or session.get('readiness_timeout_s') != policy.recovery_timeout_s
            or session.get('commit_wait_timeout_s') != policy.recovery_timeout_s):
        raise ValueError('navigation_runtime_policy_session_mismatch')
    directory = Path(directory)
    expected = {'bt.yaml': dict(policy.bt_parameters(),compute_readiness_timeout_s=policy.recovery_timeout_s),
        'bt_adapter.yaml': dict(policy.follow_parameters(),readiness_timeout_s=policy.recovery_timeout_s),
        'component_health.yaml': policy.health_parameters(),
        'global.yaml': {'result_timeout_s': policy.global_route_timeout_s,
            'warmup_timeout_s':policy.worker_startup_timeout_s,
            'readiness_timeout_s':policy.recovery_timeout_s,
            'commit_wait_timeout_s':policy.recovery_timeout_s}}
    for name, values in expected.items():
        actual = yaml.safe_load((directory/name).read_text())['/**']['ros__parameters']
        if any(actual.get(key) != value or type(actual.get(key)) not in (int, float)
                or type(value) is int and type(actual.get(key)) is not int
                for key, value in values.items()):
            raise ValueError('navigation_runtime_policy_component_mismatch:' + name)
    # Follow's post-stop localization evidence and the BT's result admission
    # use the same existing contract. Cleanup keeps its independent deadline;
    # it must not silently acquire a shorter final-arrival wait in an adapter.
    bt = yaml.safe_load((directory/'bt.yaml').read_text())['/**']['ros__parameters']
    adapter = yaml.safe_load((directory/'bt_adapter.yaml').read_text())['/**']['ros__parameters']
    arrival = bt.get('arrival_evidence_timeout_s')
    if (type(arrival) not in (int, float) or not math.isfinite(arrival) or arrival <= 0
            or type(adapter.get('arrival_evidence_timeout_s')) not in (int, float)
            or adapter['arrival_evidence_timeout_s'] != arrival):
        raise ValueError('navigation_runtime_policy_arrival_evidence_mismatch')
    import xml.etree.ElementTree as ET
    tree = ET.parse(directory/'navigation.xml')
    if any(float(node.get('timeout_s', 'nan')) != policy.recovery_timeout_s
           for node in tree.iter('PauseOnUnavailable')):
        raise ValueError('navigation_runtime_policy_tree_mismatch')
