import copy
import itertools
import numpy as np
import pytest
from bridge import validate_static_prior_state
from quadruped import pose_matrix
from runtime_geometry import body_certificate, floor_endpoint_certificate, measured_bounds


def fixture():
    matrix = np.eye(4); matrix[2, 3] = .5
    snap = [dict(path='/robot/body', shape=dict(type='Cube', size=.2), world_matrix=matrix.tolist())]
    registry = dict(root='/robot', colliders=[dict(path='/robot/body', type='Cube')])
    envelope = dict(radius=.7, offset=.3, above=.5, support_floor_z=0., support_penetration_m=.02)
    return snap, registry, envelope


def test_actual_link_corners_required_at_same_source():
    snap, registry, env = fixture()
    # primitive_aabb's shape schema is intentionally taken from the real model.
    snap[0]['shape'] = dict(type='Cube', size=.2)
    value = body_certificate(snap, [0, 0, .5, 0, 0, 0, 1], registry, env, 100)
    assert value['body_envelope_valid'] and value['body_envelope_checked_sim_time_ns'] == 100
    snap[0]['world_matrix'][0][3] = 2
    with pytest.raises(ValueError, match='horizontal_query_envelope'):
        body_certificate(snap, [0, 0, .5, 0, 0, 0, 1], registry, env, 100)


# Actual fl_foot transform from campus_cancel_v29_001 at source 5.08 s. Its
# sphere bottom is -9.935 micrometres; rotating its local enclosing box puts an
# artificial corner at -20.0046 mm and incorrectly revokes the 20 mm slab.
FAILED_FOOT_MATRIX = np.array([
    [.6265314796715524, .14849979942274694, -.7651183663669164, -7.547947406768799],
    [-.23713349777600645, .9714607424246875, -.005632953061391249, -3.826498508453369],
    [.7424459638337942, .1849644168457247, .6438650132503156, .034990064799785614],
    [0., 0., 0., 1.],
])


@pytest.mark.parametrize('rotation', [FAILED_FOOT_MATRIX[:3, :3],
    pose_matrix([0, 0, 0], [.8, .4, -.3, .2])[:3, :3],
    pose_matrix([0, 0, 0], [.7, -.5, .4, -.2])[:3, :3],
    pose_matrix([0, 0, 0], [1., 0., 0., 0.])[:3, :3]])
def test_rotated_support_sphere_uses_actual_solid_not_rotated_local_box(rotation):
    snap, registry, env = fixture()
    radius = .03500000014901161
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    # Preserve the actual failing sample's floor clearance for every pose.
    matrix[:3, 3] = [0., 0., radius * np.linalg.norm(rotation[2]) - .000009935349225990986]
    snap.append(dict(path='/robot/fl_foot', shape=dict(type='Sphere', radius=radius),
        world_matrix=matrix.tolist()))
    registry['colliders'].append(dict(path='/robot/fl_foot', type='Sphere'))
    unchanged = copy.deepcopy(snap)
    value = body_certificate(snap, [0, 0, .5, 0, 0, 0, 1], registry, env, 5080000000)
    assert value['body_envelope_valid']
    assert value['body_envelope_checked_sim_time_ns'] == 5080000000
    assert snap == unchanged
    bounds = measured_bounds(snap)[1]
    assert bounds['min'][2] == pytest.approx(-.000009935349225990986, abs=1e-15)
    assert min(p[2] for p in bounds['corners_world']) == bounds['min'][2]
    if np.array_equal(rotation, FAILED_FOOT_MATRIX[:3, :3]):
        old_minimum = matrix[2, 3] - radius * np.abs(rotation[2]).sum()
        assert old_minimum < -.020001


def test_actual_failed_world_snapshot_support_sphere_is_enclosed():
    snap, registry, env = fixture()
    snap[0]['world_matrix'][0][3] = FAILED_FOOT_MATRIX[0, 3]
    snap[0]['world_matrix'][1][3] = FAILED_FOOT_MATRIX[1, 3]
    snap.append(dict(path='/robot/fl_foot', shape=dict(type='Sphere', radius=.03500000014901161),
        world_matrix=FAILED_FOOT_MATRIX.tolist()))
    registry['colliders'].append(dict(path='/robot/fl_foot', type='Sphere'))
    body_certificate(snap, [*FAILED_FOOT_MATRIX[:2, 3], .5, 0, 0, 0, 1], registry, env, 5080000000)
    bounds = measured_bounds(snap)[1]
    assert bounds['min'][2] == pytest.approx(-9.935349225990986e-6, abs=1e-15)


@pytest.mark.parametrize('shape', [dict(type='Cube', size=.2), dict(type='Sphere', radius=.035),
    dict(type='Capsule', axis='Z', radius=.015, height=.35)])
@pytest.mark.parametrize('linear', [
    np.array([[0., -2., 0.], [.5, 0., 0.], [0., 0., 1.5]]),
    pose_matrix([0, 0, 0], [.8, .3, -.2, .1])[:3, :3] @ np.diag([1.4, .7, 1.2]),
    np.array([[1.2, .3, -.2], [.1, .6, .2], [-.4, .2, 1.3]]),
])
def test_world_bounds_include_complete_scaled_affine_primitive(shape, linear):
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = linear, [1.2, -3.4, 2.1]
    snap = [dict(path='/robot/link', shape=shape, world_matrix=matrix.tolist())]
    bounds = measured_bounds(snap)[0]
    lower, upper = np.asarray(bounds['min']), np.asarray(bounds['max'])
    corners = np.asarray(bounds['corners_world'])
    np.testing.assert_array_equal(corners.min(axis=0), lower)
    np.testing.assert_array_equal(corners.max(axis=0), upper)
    assert len(set(map(tuple, corners))) == 8
    assert bounds['path'] == '/robot/link'

    # For coordinate row a, the exact support is s/2*sum(abs(a)) for a
    # cube, r*norm(a) for a ball, and h/2*abs(a_axis)+r*norm(a) for the
    # capsule's segment-plus-ball. The local points below attain both extrema,
    # proving the bounds remain tight for nonuniform scale and affine shear.
    for dimension, row in enumerate(linear):
        if shape['type'] == 'Cube':
            extremum = np.sign(row) * shape['size'] / 2
        else:
            extremum = shape['radius'] * row / np.linalg.norm(row)
            if shape['type'] == 'Capsule':
                axis = 'XYZ'.index(shape['axis'])
                extremum[axis] += np.sign(row[axis]) * shape['height'] / 2
        world_extrema = np.asarray([-extremum, extremum]) @ linear.T + matrix[:3, 3]
        np.testing.assert_allclose(world_extrema[:, dimension], [lower[dimension], upper[dimension]], atol=1e-14)
        assert np.all(world_extrema >= lower - 1e-14)
        assert np.all(world_extrema <= upper + 1e-14)

    # Independently sample solid points, including cube corners and capsule
    # spherical caps, to retain the complete-solid enclosure invariant.
    rng = np.random.default_rng(732)
    if shape['type'] == 'Cube':
        half_size = shape['size'] / 2
        local = np.asarray(list(itertools.product(*[(-half_size, half_size)] * 3)))
    else:
        directions = rng.normal(size=(1000, 3))
        local = shape['radius'] * directions / np.linalg.norm(directions, axis=1)[:, None]
        if shape['type'] == 'Capsule':
            local[:, 2] += rng.choice([-.5, .5], size=len(local)) * shape['height']
    world = local @ linear.T + matrix[:3, 3]
    assert np.all(world >= lower - 1e-14)
    assert np.all(world <= upper + 1e-14)


@pytest.mark.parametrize('shape,translation,fault', [
    (dict(type='Sphere', radius=.035), [0., 0., .014], 'below_certified_support_slab'),
    (dict(type='Capsule', axis='Z', radius=.015, height=.2), [0., 0., .114], 'below_certified_support_slab'),
    (dict(type='Sphere', radius=.035), [0., 1., .5], 'horizontal_query_envelope'),
    (dict(type='Sphere', radius=.035), [0., 0., 1.], 'vertical_query_envelope'),
])
def test_complete_primitive_true_floor_horizontal_and_top_violations_still_reject(shape, translation, fault):
    snap, registry, env = fixture()
    matrix = np.eye(4)
    matrix[:3, 3] = translation
    snap.append(dict(path='/robot/extra_link', shape=shape, world_matrix=matrix.tolist()))
    registry['colliders'].append(dict(path='/robot/extra_link', type=shape['type']))
    with pytest.raises(ValueError, match=fault):
        body_certificate(snap, [0, 0, .5, 0, 0, 0, 1], registry, env, 5080000000)


@pytest.mark.parametrize('matrix', [np.eye(3).tolist(), np.full((4, 4), np.nan).tolist()])
def test_measured_bounds_invalid_transform_still_rejects(matrix):
    with pytest.raises(ValueError, match='invalid_actual_link_transform'):
        measured_bounds([dict(path='/robot/foot', shape=dict(type='Sphere', radius=.035), world_matrix=matrix)])


@pytest.mark.parametrize('yaw', [0., np.pi/8, np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, np.pi])
def test_query_yaw_does_not_enlarge_a_complete_chassis_solid(yaw):
    from dynamic_collision import certify_robot_in_body_envelope
    orientation = [0., 0., np.sin(yaw/2), np.cos(yaw/2)]
    pose = [3., -4., .5, *orientation]
    matrix = pose_matrix(pose[:3], [orientation[3], *orientation[:3]])
    matrix[:3, :3] = matrix[:3, :3] @ np.diag([.6, .2, .2])
    snap = [dict(path='/robot/body', shape=dict(type='Cube', size=1.), world_matrix=matrix.tolist())]
    registry = dict(colliders=[dict(path='/robot/body', type='Cube')])
    env = dict(radius=.23, offset=.2, above=.2, support_floor_z=0., support_penetration_m=.02)
    original = copy.deepcopy(snap)
    assert body_certificate(snap, pose, registry, env, 100)['body_envelope_valid']
    assert snap == original
    if yaw in (np.pi/4, -np.pi/4):
        # AABB reboxing invents (x=0,y=+/-.4); the physical chassis has
        # |y|<=.1. Every clipped half of the actual solid fits radius .23.
        with pytest.raises(ValueError, match='horizontal_query_envelope'):
            certify_robot_in_body_envelope(measured_bounds(snap), body_position=pose[:3],
                body_orientation_xyzw=orientation, radius=.23, offset=.2, below=.5,
                above=.2, floor_z=0.)


def test_original_v59_body_cube_false_rejection_uses_same_native_transform():
    pose = [-.27422767877578735, -5.583821773529053, .4927676022052765,
        -.03565135598182678, -.06719525158405304, -.3733566999435425, .9245639443397522]
    matrix = [[.6053522744434968, .13903510722606685, -.013668378491509516, pose[0]],
        [-.5827541752784, .14373349843260766, .016253932672968072, pose[1]],
        [.12824287720211927, -.0031496645789143326, .13837985800713723, pose[2]],
        [0., 0., 0., 1.]]
    snap = [dict(path='/World/Spot/body/collisions/mesh_0', shape=dict(type='Cube', size=1.), world_matrix=matrix)]
    registry = dict(colliders=[dict(path=snap[0]['path'], type='Cube')])
    env = dict(radius=.6078651750202781, offset=.3125, above=.589,
        support_floor_z=0., support_penetration_m=.02)
    assert body_certificate(snap, pose, registry, env, 83818000000)['body_envelope_valid']
    # It remains the same collision box: a real outward displacement must
    # still revoke the certificate, using the original query dimensions.
    shifted = copy.deepcopy(snap)
    shifted[0]['world_matrix'][1][3] += 1.
    with pytest.raises(ValueError, match='horizontal_query_envelope'):
        body_certificate(shifted, pose, registry, env, 83818000000)


@pytest.mark.parametrize('shape', [dict(type='Cube', size=.2), dict(type='Sphere', radius=.035),
    dict(type='Capsule', axis='X', radius=.015, height=.35),
    dict(type='Cylinder', axis='Z', radius=.04, height=.2)])
@pytest.mark.parametrize('yaw', [np.pi/4, -np.pi/3])
def test_query_support_box_encloses_complete_affine_solid(shape, yaw):
    matrix = np.eye(4)
    matrix[:3, :3] = [[1.2, .3, -.2], [.1, .6, .2], [-.4, .2, 1.3]]
    matrix[:3, 3] = [1.2, -3.4, 2.1]
    pose = [.7, -2., .5, 0., 0., np.sin(yaw/2), np.cos(yaw/2)]
    bounds = measured_bounds([dict(path='/robot/link', shape=shape, world_matrix=matrix)], query_pose=pose)[0]
    world_corners = np.asarray(bounds['corners_world'])
    rotation = pose_matrix([0., 0., 0.], [pose[6], *pose[3:6]])[:3, :3]
    query_corners = (world_corners - pose[:3]) @ rotation
    lower, upper = query_corners.min(axis=0), query_corners.max(axis=0)
    query_linear = rotation.T @ matrix[:3, :3]
    for dimension, row in enumerate(query_linear):
        if shape['type'] == 'Cube':
            extremum = np.sign(row) * shape['size']/2
        elif shape['type'] == 'Cylinder':
            axis = 'XYZ'.index(shape['axis'])
            radial = row.copy(); radial[axis] = 0.
            extremum = shape['radius'] * radial / np.linalg.norm(radial)
            extremum[axis] = np.sign(row[axis]) * shape['height']/2
        else:
            extremum = shape['radius'] * row/np.linalg.norm(row)
            if shape['type'] == 'Capsule':
                axis = 'XYZ'.index(shape['axis'])
                extremum[axis] += np.sign(row[axis]) * shape['height']/2
        world = np.asarray([-extremum, extremum]) @ matrix[:3, :3].T + matrix[:3, 3]
        query = (world - pose[:3]) @ rotation
        np.testing.assert_allclose(query[:, dimension], [lower[dimension], upper[dimension]], atol=1e-14)
        assert np.all(query >= lower-1e-14) and np.all(query <= upper+1e-14)


@pytest.mark.parametrize('pose', [[0.]*7, [0., 0., .5, np.nan, 0., 0., 1.], [0.]*6])
def test_query_support_box_rejects_invalid_measured_body_pose(pose):
    snap, _, _ = fixture()
    with pytest.raises(ValueError):
        measured_bounds(snap, query_pose=pose)


@pytest.mark.parametrize('key,value', [('body_envelope_valid', False), ('body_envelope_checked_sim_time_ns', 99), ('body_envelope_registry_sha256', 'f'*64)])
def test_bridge_rejects_incomplete_full_body_certificate(key, value):
    contract = dict(static_prior_geometry_sha256='a'*64, max_body_tilt_rad=.35,
        expected_body_world_z=.52, max_body_height_error_m=.12,
        body_envelope_attestation_required=True, body_envelope_registry_sha256='b'*64)
    packet = dict(sim_time_ns=100, pose=[0, 0, .48, 0, 0, 0, 1],
        static_prior_geometry_sha256='a'*64, static_prior_geometry_valid=True,
        static_prior_geometry_checked_sim_time_ns=100, body_envelope_valid=True,
        body_envelope_checked_sim_time_ns=100, body_envelope_registry_sha256='b'*64)
    validate_static_prior_state(packet, contract, np.eye(3))
    packet[key] = value
    with pytest.raises(ValueError, match='full_body_envelope_revoked'):
        validate_static_prior_state(packet, contract, np.eye(3))


def test_native_floor_identity_numeric_contract_preserves_raw_points():
    points = np.array([[0, 0, -.5+.00008], [0, 0, -.5+.01]], dtype=np.float32)
    original = points.copy()
    contact = dict(floor_path='/floor', floor_z=0., floor_endpoint_error_bound_m=.0002)
    result = floor_endpoint_certificate(points, [0, 0, .5, 0, 0, 0, 1], ['/floor', '/actor'], contact)
    assert result['native_floor_hits'] == 1
    assert result['floor_endpoint_max_abs_error_m'] < .0002
    np.testing.assert_array_equal(points, original)
    with pytest.raises(ValueError, match='exceeds_sealed_bound'):
        floor_endpoint_certificate(points, [0, 0, .5, 0, 0, 0, 1], ['/actor', '/floor'], contact)


@pytest.mark.parametrize('paths', [[], [''], [None]])
def test_missing_native_floor_identity_fails_closed(paths):
    with pytest.raises(ValueError, match='invalid_native_floor_hit_identity'):
        floor_endpoint_certificate([[0, 0, -.5]], [0, 0, .5, 0, 0, 0, 1], paths,
            dict(floor_path='/floor', floor_z=0., floor_endpoint_error_bound_m=.0002))
