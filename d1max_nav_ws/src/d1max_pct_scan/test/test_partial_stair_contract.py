"""Native partial routes keep source terrain through the existing typed lane.

Small measured-surface fixtures exercise the real tomogram validator, source
builder, wire codec, reference callbacks and stair-support classifier. No ROS
graph, SDK session, substitute local planner or motion authority is created.
"""
import numpy as np
import pytest

from d1max_pct_scan.reference_support import ConditionedReferenceSupport
from d1max_pct_scan.source_route import SourceRouteBuilder, SourceSupport
from d1max_pct_scan.source_route_ros import from_message, to_message
from d1max_pct_scan.static_route_validation import validate_static_route
from test_continuous_reference import IdentityGroundBridge
from test_continuous_reference_node import harness, local_geometry_commit, native_received
from test_continuous_reference_transport import BASE_NS, envelope, state
from test_stair_support import fixture


CASES = ('lower_up', 'lower_down', 'through_landing_up',
         'through_landing_down', 'landing')


def partial_fixture(case):
    grid, literal = fixture()
    grid.sha256 = 'c' * 64
    centerline = literal[np.isclose(literal[:, 1], 0.) & (literal[:, 2] < 1.)]
    if case == 'landing':
        points = centerline[6:9]
        legs = ['stair_landing'] * (len(points) - 1)
        direction = 'same_floor'
    elif case.startswith('lower_'):
        points = centerline[:6]
        legs = ['stair_lower'] * (len(points) - 1)
        direction = 'lower_to_upper'
    else:
        points = centerline[3:12]
        legs = ['stair_lower'] * 3 + ['stair_landing'] * 3 + ['stair_upper'] * 2
        direction = 'lower_to_upper'
    if case.endswith('_down'):
        points, legs = points[::-1], legs[::-1]
        direction = 'upper_to_lower'
    segments = []
    for i, leg in enumerate(legs):
        if not segments or leg != segments[-1]['name']:
            portal = 'start' if not segments else segments[-1]['to']
            segments.append(dict(name=leg, first_index=i, last_index=i+1,
                **{'from': portal, 'to': f'portal_{len(segments)+1}'}))
            if leg == 'stair_landing':
                segments[-1]['kind'] = 'landing'
        else:
            segments[-1]['last_index'] = i+1
    segments[-1]['to'] = 'goal'
    raw = dict(route_type='partial_stair', direction=direction, path=points.tolist(),
        layer_ids=[0]*len(points), segments=segments, edge_legs=legs,
        source_tomogram_sha256=grid.sha256, execution_authorized=False)
    support = SourceSupport({label:literal for label in ('stairs', 'stair_lower', 'stair_upper')}, 'a'*64)
    builder = SourceRouteBuilder(bridge=IdentityGroundBridge(), source_map_sha256='a'*64,
        conditioning_sha256='b'*64, tomogram_sha256=grid.sha256, support=support)
    provider = ConditionedReferenceSupport({'source_sha256':'a'*64}, support, grid)
    return grid, literal, raw, builder, provider


def checked_route(case):
    grid, literal, raw, builder, provider = partial_fixture(case)
    result = validate_static_route(raw, grid, builder.bridge, builder=builder, map_version_id='map')
    return result['route_snapshot'], raw, provider, literal


@pytest.mark.parametrize('case', CASES)
def test_partial_route_static_source_audit_and_typed_roundtrip(case):
    route, raw, _, _ = checked_route(case)
    value = route.payload()
    assert value['geometry_evidence']['measured_source_support']['all_samples_supported']
    assert value['geometry_evidence']['ground_to_body_height_applications'] == 0
    assert value['xyz'] == raw['path']
    assert value['edge_segments'] == raw['edge_legs']
    assert value['direction'] == raw['direction']
    assert not value['execution_eligible']
    for segment in value['segments']:
        assert segment['required_mode'] == 'stair'
        if segment['segment_id'] == 'stair_landing':
            assert segment['kind'] == 'landing' and segment['floor_id'] == 'stairs'
        else:
            assert segment['kind'] == ('stair_down' if case.endswith('_down') else 'stair_up')
    wire = to_message(route, session_id='session', task_id='task', route_id='route',
        epoch=1, seed_id='seed', stamp=state().source_stamp)
    assert from_message(wire).encoded == route.encoded
    assert from_message(wire).route_hash == route.route_hash


@pytest.mark.parametrize('case', CASES)
def test_partial_route_reaches_formal_reference_support_and_final_body_goal(case):
    route, raw, provider, literal = checked_route(case)
    node, clock, output = harness(execution_mode_policy='fixed_stair', geometry_owner='local_executor')
    node.support_map = provider
    start = np.asarray(raw['path'][0])
    sample = state(t=.01)
    for odometry in (sample.global_odometry, sample.local_odometry):
        odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z = (
            float(start[0]), float(start[1]), float(start[2]+.55))
    clock.ns, clock.mono = BASE_NS+10_000_000, 100.01
    node.on_navigation(sample)
    message = envelope(t=.01)
    message.route_hash = route.route_hash
    message.snapshot = to_message(route, session_id='session', task_id='task', route_id='route',
        epoch=1, seed_id='seed', stamp=message.source_stamp)
    node.on_route(message)
    proposal = native_received(node)
    support = node.prepared_for(proposal.version).support
    assert support.verified, support.reason
    assert support.required_mode == 'stair'
    assert support.floor_id == route.payload()['segments'][0]['floor_id']
    if case == 'landing':
        assert support.floor_id == 'stairs' and support.segment_kind == 'landing'
    assert node.p['floor_id'] == 'floor1'  # startup hint does not filter stair terrain
    measured = np.array([[p.x, p.y, p.z] for p in support.support_ground_xyz])
    for point in measured:
        assert np.min(np.linalg.norm(literal-point, axis=1)) < 1e-12
    expected_goal = np.array(raw['path'][-1])+[0., 0., .55]
    actual_goal = np.array([proposal.goal_position.x, proposal.goal_position.y, proposal.goal_position.z])
    np.testing.assert_allclose(actual_goal, expected_goal)
    last = proposal.reference.path.poses[-1].pose.position
    np.testing.assert_allclose([last.x, last.y, last.z], expected_goal)
    assert proposal.reference.point_reference == 'body_center'
    assert all(mode == 'stair' for mode in proposal.reference.point_required_modes)
    assert proposal.reference.path.header.stamp == sample.source_stamp
    # A geometry commit installs this exact route/window; it does not invent
    # an SDK permit, switch modes, or replace the partial endpoint by a floor.
    commit = local_geometry_commit(node, clock, proposal)
    assert node.on_local_commit(commit)
    assert node.transport.snapshot.route_hash == route.route_hash
    # Cross-portal lookahead may already include the task endpoint while the
    # current measured semantic segment is still the lower/upper flight.
    assert node.transport._accepted_end_arc_m == pytest.approx(node.transport.core._arc[-1])
    assert node.transport.covers_current_segment_end() == (len(route.payload()['segments']) == 1)
    assert node.permit is None
    assert not [name for name, _ in output if name in ('handoff', 'sdk', 'motion')]


@pytest.mark.parametrize('fault', ('floor_alias', 'missing_kind', 'same_floor_on_flight'))
def test_partial_landing_is_not_relabelled_as_a_flat_floor_or_stair_flight(fault):
    grid, _, raw, builder, _ = partial_fixture('landing' if fault != 'same_floor_on_flight' else 'lower_up')
    if fault == 'missing_kind':
        raw['segments'][0].pop('kind')
    elif fault == 'same_floor_on_flight':
        raw['direction'] = 'same_floor'
    else:
        raw['edge_legs'] = ['lower_floor']*len(raw['edge_legs'])
    with pytest.raises(ValueError):
        validate_static_route(raw, grid, builder.bridge, builder=builder, map_version_id='map')


def test_partial_landing_missing_source_support_is_not_filled_or_authorized():
    grid, _, raw, builder, _ = partial_fixture('landing')
    builder.support = SourceSupport({'stair_lower':[[100., 100., 0.]]}, 'a'*64)
    result = validate_static_route(raw, grid, builder.bridge, builder=builder, map_version_id='map')
    value = result['route_snapshot'].payload()
    assert value['point_floor_ids'] == ['stairs']*len(raw['path'])
    assert not value['geometry_evidence']['measured_source_support']['all_samples_supported']
    assert not value['execution_eligible']
    assert value['xyz'] == raw['path']
