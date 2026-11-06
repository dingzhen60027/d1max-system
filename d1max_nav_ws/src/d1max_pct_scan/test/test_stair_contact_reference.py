"""Typed literal contact subset; passive production callbacks, no ROS graph."""
from copy import deepcopy
import hashlib
import math

import numpy as np
import pytest

from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from d1max_pct_scan.control_frame_contract import Rigid
from d1max_pct_scan.reference_support import ConditionedReferenceSupport
from d1max_pct_scan.source_route import SourceSupport, canonical
from d1max_pct_scan.stair_support import StairSupportSelection
from test_reference_support import request, transition_request, xyz
from test_stair_support import fixture, regularized_mapping


def provider(qualified=True):
    grid, source = fixture()
    selected = source[(source[:, 2] > .05) & (source[:, 2] < 1.)][::3]
    mapping = regularized_mapping(source, source)
    index = SourceSupport({'stairs': source, 'floor1': source[source[:, 2] < .01]},
                          'a'*64, stair_mapping=mapping)
    result = ConditionedReferenceSupport({'source_sha256': 'a'*64}, index, grid,
        structured_source_points=selected if qualified else None)
    return result, source, selected


def stair_request(reverse=False, *, inverse=None, kind=None):
    from geometry_msgs.msg import PoseStamped
    support, literal, selected = provider()
    node = object.__new__(ReferenceCallbacks)
    node.support_map = support
    inverse = inverse or Rigid((1., 2., .3), (0., 0., math.sin(.3), math.cos(.3)))
    req = request('a'*64, floor='stair_lower', kind=kind or ('stair_down' if reverse else 'stair_up'),
                  mode='stair', inverse=inverse)
    ground = literal[np.isclose(literal[:, 1], 0.) & (literal[:, 2] < 1.)]
    if reverse:
        ground = ground[::-1]
    for point in ground:
        pose = PoseStamped()
        body = inverse.point(tuple(float(v) for v in point+[0., 0., .55]))
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = body
        req[0].reference.path.poses.append(pose)
    return node, req, literal, selected


def source_rows(wire, inverse):
    return np.asarray([inverse.inverse().point(tuple(map(float, p))) for p in xyz(wire)])


@pytest.mark.parametrize('reverse', [False, True])
def test_single_stair_binds_only_qualified_original_subset_before_rigid_transform(reverse):
    node, req, literal, qualified = stair_request(reverse)
    frozen = deepcopy(req[0])
    _, support = node.build_support(req)
    assert support.verified and req[0] == frozen
    raw = source_rows(support, req[2])
    selected = list(support.stair_contact_indices)
    assert selected == sorted(set(selected)) and selected
    expected = node.support_map.stair_surface(
        literal[np.isclose(literal[:, 1], 0.) & (literal[:, 2] < 1.)],
        source_map_sha256='a'*64, floor_id='stair_lower',
        segment_kind='stair_down' if reverse else 'stair_up', required_mode='stair').points
    np.testing.assert_allclose(raw, expected, rtol=0, atol=2e-15)
    assert selected == [i for i, p in enumerate(expected) if tuple(p) in set(map(tuple, qualified))]
    assert len(selected) < len(raw)  # Legacy PCT support is not a contact permit.
    payload = dict(anchor='measured-rigid-anchor', map='a'*64, points=expected.tolist(),
                   stair_contact_indices=selected)
    assert support.support_hash == hashlib.sha256(canonical(payload).encode()).hexdigest()


def test_same_raw_support_without_contact_evidence_keeps_old_hash_and_empty_subset():
    node, req, _, _ = stair_request()
    _, qualified = node.build_support(req)
    node.support_map, _, _ = provider(False)
    _, ordinary = node.build_support(req)
    assert ordinary.support_ground_xyz == qualified.support_ground_xyz
    assert not ordinary.stair_contact_indices
    assert ordinary.support_hash != qualified.support_hash
    raw = node.support_map.stair_index.points
    raw = raw[raw[:, 2] < 1.]
    expected = canonical(dict(anchor='measured-rigid-anchor', map='a'*64, points=raw.tolist()))
    assert ordinary.support_hash == hashlib.sha256(expected.encode()).hexdigest()


@pytest.mark.parametrize('reverse', [False, True])
def test_transition_retains_qualified_identity_without_granting_floor_copies(reverse):
    node, req, _ = transition_request(reverse)
    node.support_map, _, qualified = provider()
    _, support = node.build_support(req)
    assert support.verified and len(support.regions) == 2
    assert support.stair_contact_indices
    raw = source_rows(support, req[2])
    for i in support.stair_contact_indices:
        region = next(r for r in support.regions if r.begin_index <= i < r.end_index)
        assert region.required_mode == 'stair'
        assert np.min(np.linalg.norm(qualified-raw[i], axis=1)) < 2e-15
    assert all(i not in support.stair_contact_indices for r in support.regions
        if r.required_mode == 'general' for i in range(r.begin_index, r.end_index))


@pytest.mark.parametrize('change', ['hash', 'floor', 'mode', 'kind', 'jitter'])
def test_contact_qualification_never_uses_nearest_point_or_foreign_semantics(change):
    support, _, qualified = provider()
    source = qualified.copy()
    arguments = dict(source_map_sha256='a'*64, floor_id='stair_lower',
                     segment_kind='stair_up', required_mode='stair')
    if change == 'hash':
        arguments['source_map_sha256'] = 'b'*64
    elif change == 'floor':
        arguments['floor_id'] = 'floor1'
    elif change == 'mode':
        arguments['required_mode'] = 'general'
    elif change == 'kind':
        arguments['segment_kind'] = 'floor'
    else:
        source[:, 0] += 1e-9
    assert not support.contact_indices(source, **arguments)


def test_platform_label_alone_cannot_qualify_unclassified_original_rows():
    support, literal, _ = provider()
    platform = literal[literal[:, 2] < .01]
    assert not support.contact_indices(platform, source_map_sha256='a'*64,
        floor_id='stairs', segment_kind='landing', required_mode='stair')


def test_unverified_selection_cannot_publish_contact_indices(monkeypatch):
    node, req, literal, _ = stair_request()
    monkeypatch.setattr(node.support_map, 'stair_surface', lambda *a, **k:
        StairSupportSelection(literal[literal[:, 2] < 1.], False, 'fixture_missing_coverage'))
    _, support = node.build_support(req)
    assert not support.verified and not support.stair_contact_indices


def test_indices_refer_to_final_downsampled_packet_not_preselection_offsets(monkeypatch):
    node, req, literal, qualified = stair_request()
    lower = literal[literal[:, 2] < 1.]
    repeated = np.repeat(lower, 230, axis=0)
    assert len(repeated) > 20000
    monkeypatch.setattr(node.support_map, 'stair_surface', lambda *a, **k:
        StairSupportSelection(repeated, True, 'original_stair_surface_returns_associated'))
    _, support = node.build_support(req)
    assert len(support.support_ground_xyz) == len(lower)
    assert list(support.stair_contact_indices) == [i for i, p in enumerate(lower)
        if tuple(p) in set(map(tuple, qualified))]


def test_typed_message_round_trip_preserves_contact_subset_and_bound_identity():
    from d1max_planning_interfaces.msg import SupportReference
    from rclpy.serialization import deserialize_message, serialize_message
    node, req, _, _ = stair_request()
    _, support = node.build_support(req)
    assert deserialize_message(serialize_message(support), SupportReference) == support
