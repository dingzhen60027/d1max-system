"""Fixed wheel-mode engineering geometry, without joints or broad clearing."""
from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import struct
import time

import numpy as np
import pytest

from d1max_pct_scan.robot_fixed_stance import FixedStanceModel, FixedStanceProfile
from d1max_pct_scan.robot_self_geometry import RobotSelfModel, SelfFilterConfig


CONFIG = SelfFilterConfig(max_classification_ms=1000)
OFFICIAL = Path('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/'
                'd1max_ros2/urdf_ws/src/max_description/urdf/max.urdf')


def leg_model(tmp_path, wheel_mesh=None):
    """The official chain lengths with tiny collision primitives for pure tests."""
    parts = ['<link name="BASE_LINK"/>']
    for sensor, x in [('F_AIRY_LINK', .4043), ('R_AIRY_LINK', -.4043)]:
        parts.append(f'<link name="{sensor}"/><joint name="{sensor}_fixed" type="fixed">'
                     f'<parent link="BASE_LINK"/><child link="{sensor}"/>'
                     f'<origin xyz="{x} 0 -.0377"/></joint>')
    for prefix, sx, sy in [('FAR', 1, -1), ('FBL', 1, 1),
                           ('RAR', -1, -1), ('RBL', -1, 1)]:
        origins = [(sx*.272, sy*.065, .0002), (sx*.0575, sy*.039, 0),
                   (0, sy*.0442, -.26), (0, sy*.0592, -.28)]
        parent = 'BASE_LINK'
        for part, origin in zip(('ABAD', 'HIP', 'KNEE', 'FOOT'), origins):
            name = prefix+'_'+part+'_LINK'
            # FOOT centre intentionally retains the CAD asymmetric y offset.
            geometry = (f'<origin xyz="0 {sy*.0073} 0" rpy="1.5707963267948966 0 0"/>'
                        '<geometry><cylinder radius=".095" length=".05"/></geometry>'
                        if part == 'FOOT' else '<geometry><box size=".01 .01 .01"/></geometry>')
            if part == 'FOOT' and wheel_mesh is not None:
                geometry = (f'<origin xyz="0 {sy*.0073} 0"/>'
                            f'<geometry><mesh filename="{wheel_mesh}"/></geometry>')
            parts.append(f'<link name="{name}"><collision>{geometry}</collision></link>')
            kind, axis = ('continuous', '0 1 0') if part == 'FOOT' else (
                'revolute', '1 0 0' if part == 'ABAD' else '0 1 0')
            xyz = ' '.join(str(value) for value in origin)
            parts.append(f'<joint name="{prefix}_{part}_JOINT" type="{kind}">'
                         f'<parent link="{parent}"/><child link="{name}"/>'
                         f'<origin xyz="{xyz}"/><axis xyz="{axis}"/>'
                         '<limit lower="-2.9" upper="2.9" effort="150" velocity="0"/>'
                         '</joint>')
            parent = name
    path = tmp_path/'official_chain.urdf'
    path.write_text('<robot name="test">'+''.join(parts)+'</robot>')
    return RobotSelfModel.from_urdf(path)


def test_all_four_legs_are_available_without_joint_source(tmp_path):
    model = FixedStanceModel.from_base_model(leg_model(tmp_path))
    assert model.joint_types == {}
    assert model.required_joint_names == ()
    for prefix, sx, sy in [('FAR', 1, -1), ('FBL', 1, 1),
                           ('RAR', -1, -1), ('RBL', -1, 1)]:
        np.testing.assert_allclose(model.nominal_wheel_centers[prefix],
                                   [sx*.375, sy*.2147, -.4427], atol=1e-9)
    value = model.to_dict()
    assert len(value['nominal_link_transforms_root']) == len(model.links)
    assert max(value['wheel_xz_fit_residual_m'].values()) < 1e-7
    assert set(value['feasible_bending_branch_counts'].values()) == {2}
    assert value['posture_source'] == 'engineering_nominal_not_measured'
    assert value['posture_calibrated'] is False
    assert value['extrinsics_calibrated'] is False
    assert value['requires_joint_source'] is False
    assert value['grants_motion_authority'] is False
    assert value['extends_free_rays'] is False
    assert value['body_proxy_used_as_collision'] is False
    applicability = value['applicability']
    assert applicability['posture'] == 'nominal_wheel_standing'
    assert applicability['lidar_ground_height_m'] == .5
    assert applicability['live_posture_verified'] is False
    assert applicability['outer_stance_width_measured'] is False
    assert applicability['does_not_clear_occupied_or_unknown_voxels'] is True


def test_filter_reports_assumption_and_original_beam_times_without_authority(tmp_path):
    from d1max_pct_scan.robot_self_filter import RobotSelfFilter
    model = FixedStanceModel.from_base_model(leg_model(tmp_path))
    settings = dict(mode='filter', pose_source='fixed_standing_cad',
                    classification_budget_ms=50.,
                    sensor_origins_body_m={0: [.4043, 0., -.0377],
                                           1: [-.4043, 0., -.0377]})
    entity_filter = RobotSelfFilter(model, settings)
    ground = [0., 0., -.5377]
    obstacle = [.6, .3, -.2]
    result = entity_filter.classify([ground, obstacle], settings['sensor_origins_body_m'][0],
                                  [900000000, 910000000], sensor_id=0)
    assert not result.mask.any()
    assert result.report['near_point_count'] == 2
    assert result.report['unmatched_near_point_count'] == 2
    assert result.report['source_start_ns'] == 900000000
    assert result.report['source_end_ns'] == 910000000
    assert result.report['applicability']['live_posture_verified'] is False
    assert result.report['physical_calibration_verified'] is False


def test_fixed_filter_covers_wheels_and_legs_but_preserves_ground_and_obstacle(tmp_path):
    model = FixedStanceModel.from_base_model(leg_model(tmp_path))
    poses = model.to_dict()['nominal_link_transforms_root']
    points = [np.asarray(poses['RBL_KNEE_LINK'])[:3, 3],
              model.nominal_wheel_centers['RBL'], [0, 0, -.5377],
              [0, .15, -.4], [-.375, .32, -.4427], [-.6, .2147, -.4427]]
    result = model.classify(points, [-.7, 0, -.1], include_dynamic=False, config=CONFIG)
    assert result.matched_mask.tolist() == [True, True, False, False, False, False]
    assert result.missing_links == ()
    assert result.extends_free_rays is False
    assert set(result.per_link_counts) == {'RBL_KNEE_LINK', 'RBL_FOOT_LINK'}


def test_no_joint_positions_can_be_passed_and_nominal_values_are_immutable(tmp_path):
    model = FixedStanceModel.from_base_model(leg_model(tmp_path))
    with pytest.raises(TypeError):
        model.classify([], [0, 0, 0], joint_positions={'RBL_HIP_JOINT': 2.})
    with pytest.raises(TypeError):
        model.nominal_joint_positions['RBL_HIP_JOINT'] = 0.
    with pytest.raises(FrozenInstanceError):
        model.profile = FixedStanceProfile(wheelbase_m=.8)
    assert not model.classify([[0, 0, 0]], [-1, 0, 0],
                              include_static=False, config=CONFIG).matched_mask.any()


def test_load_seals_assets_and_never_warms_in_live_query(tmp_path, monkeypatch):
    base = leg_model(tmp_path)
    path = tmp_path/'manifest.json'
    path.write_text(json.dumps(base.to_dict()))
    called = []
    original = RobotSelfModel.warmup
    monkeypatch.setattr(RobotSelfModel, 'warmup', lambda self, **kwargs: called.append(kwargs) or {})
    model = FixedStanceModel.load(path)
    assert not called
    assert model.asset_paths == base.asset_paths
    model.classify([[0, 0, -.5]], [-1, 0, 0], config=CONFIG)
    assert not called
    report = model.warmup(include_dynamic=False, sensor_origins=[[-.4043, 0, -.0377]])
    assert called[-1]['include_dynamic'] is True
    assert report['requires_joint_source'] is False
    assert model.content_hash == FixedStanceModel.load(path).content_hash
    monkeypatch.setattr(RobotSelfModel, 'warmup', original)
    altered = base.to_dict()
    altered['collisions'][0]['upper'][0] = 5.
    path.write_text(json.dumps(altered))
    with pytest.raises(ValueError, match='changed'):
        FixedStanceModel.load(path)


def test_proxy_dimensions_never_expand_the_entity_filter(tmp_path):
    base = leg_model(tmp_path)
    model = FixedStanceModel.from_base_model(base)
    changed = FixedStanceModel.from_base_model(base, profile={
        'body_box_width_m': .7, 'body_box_height_m': 1.})
    points = [[0, .3, -.5], [0, 0, -.5377], [0, .14, -.3]]
    np.testing.assert_array_equal(model.classify(points, [-1, 0, 0], config=CONFIG).matched_mask,
                                  changed.classify(points, [-1, 0, 0], config=CONFIG).matched_mask)
    assert changed.collisions is base.collisions
    assert model.content_hash != changed.content_hash


def _box_triangles(lower, upper):
    lo, hi = np.asarray(lower), np.asarray(upper)
    vertices = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
                         [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
                         [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
                         [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]])
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (3, 7, 6), (3, 6, 2),
             (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5)]
    return [vertices[list(face)] for face in faces]


def _write_triangles(path, triangles):
    path.write_bytes(bytes(80)+struct.pack('<I', len(triangles))+b''.join(
        struct.pack('<12fH', *(list(np.zeros(3))+list(np.asarray(triangle).ravel())), 0)
        for triangle in triangles))


def test_frozen_actual_mesh_preserves_concavity_far_side_and_external_obstacle(tmp_path, monkeypatch):
    triangles = (_box_triangles([-.09, -.09, -.09], [-.03, .09, .09])+
                 _box_triangles([-.09, -.09, -.09], [.09, -.03, .09]))
    mesh_path = tmp_path/'concave_wheel.stl'
    _write_triangles(mesh_path, triangles)
    model = FixedStanceModel.from_base_model(leg_model(tmp_path, mesh_path))
    model.warmup()
    from d1max_pct_scan.robot_self_geometry import _MESH_CACHE
    monkeypatch.setattr(_MESH_CACHE, 'get', lambda *args: pytest.fail('runtime mesh load'))
    pose = np.asarray(model.to_dict()['nominal_link_transforms_root']['RBL_FOOT_LINK'])
    collision = next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK')
    # Work in the collision frame, then apply the frozen parent-to-root FK.
    origin = pose[:3, :3] @ np.array([-.5, .05+.0073, 0])+pose[:3, 3]
    local_points = np.array([[-.09, .05, 0], [.05, .05, 0], [-.03, .05, 0],
                             [.3, .05, 0], [-.094, .05, 0], [-.096, .05, 0]])
    points = (local_points+np.asarray(collision.xyz)) @ pose[:3, :3].T+pose[:3, 3]
    result = model.classify(points, origin,
        config=SelfFilterConfig(measurement_padding_m=.005, max_classification_ms=1000))
    assert result.matched_mask.tolist() == [True, False, False, False, True, False]
    assert result.missing_links == ()
    assert result.unloaded_links == ()
    assert result.extends_free_rays is False


def test_batch_bvh_matches_independent_all_triangle_linear_solves(tmp_path):
    mesh_path = tmp_path/'reference_surface.stl'
    triangles = (_box_triangles([-.09, -.09, -.09], [-.03, .09, .09])+
                 _box_triangles([-.09, -.09, -.09], [.09, -.03, .09]))
    _write_triangles(mesh_path, triangles)
    model = leg_model(tmp_path, mesh_path)
    from d1max_pct_scan.robot_self_geometry import _MESH_CACHE
    mesh = _MESH_CACHE.get(next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK'))
    rng = np.random.default_rng(83)
    origins, endpoints = rng.uniform(-.3, .3, (96, 3)), rng.uniform(-.1, .1, (96, 3))
    expected = np.full(len(endpoints), np.inf)
    for index, (origin, endpoint) in enumerate(zip(origins, endpoints)):
        length = np.linalg.norm(endpoint-origin)
        direction = (endpoint-origin)/length
        for triangle in mesh.triangles.astype(float):
            vertex = triangle[0]
            system = np.column_stack((direction, vertex-triangle[1], vertex-triangle[2]))
            try:
                distance, u, v = np.linalg.solve(system, vertex-origin)
            except np.linalg.LinAlgError:
                continue
            if (0 <= distance <= length+.005+1e-9 and u >= -1e-9 and v >= -1e-9
                    and u+v <= 1+1e-9):
                expected[index] = min(expected[index], distance)
    actual, incomplete = mesh.first_intersections(origins, endpoints, .005)
    assert not incomplete
    np.testing.assert_allclose(actual, expected, atol=1e-10)
    actual, incomplete = mesh.first_intersections(origins, endpoints, .005, time.monotonic()-1)
    assert incomplete
    assert np.isinf(actual).all()  # expired batch never leaks partial hits


def test_oversized_coincident_triangle_leaf_has_bounded_pair_storage(tmp_path):
    mesh_path = tmp_path/'identical_triangles.stl'
    triangle = np.array([[-.1, -.1, 0], [.1, -.1, 0], [0, .1, 0]])
    _write_triangles(mesh_path, [triangle]*128)
    model = leg_model(tmp_path, mesh_path)
    from d1max_pct_scan.robot_self_geometry import _MESH_CACHE
    mesh = _MESH_CACHE.get(next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK'))
    assert mesh.bvh_children.shape == (1, 2)  # one leaf exceeds cKDTree leafsize
    rays = np.arange(400)
    spans = np.tile([0, 128], (len(rays), 1))
    assert max(len(ids) for ids, _ in mesh._leaf_pairs(rays, spans)) <= 8192
    actual, incomplete = mesh.first_intersections(np.tile([0, 0, 1], (400, 1)),
                                                  np.tile([0, 0, 0], (400, 1)), 0.)
    assert not incomplete
    np.testing.assert_allclose(actual, 1.)


def test_mesh_surface_candidates_equal_inclusive_leaf_union_across_chunks(tmp_path):
    mesh_path = tmp_path/'candidate_boundaries.stl'
    triangles = (_box_triangles([-.09, -.09, -.09], [-.03, .09, .09])+
                 _box_triangles([-.09, -.09, -.09], [.09, -.03, .09]))
    _write_triangles(mesh_path, triangles)
    from d1max_pct_scan.robot_self_geometry import _MESH_CACHE
    model = leg_model(tmp_path, mesh_path)
    mesh = _MESH_CACHE.get(next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK'))
    rng = np.random.default_rng(20261105)
    points = rng.uniform(-.2, .2, (8300, 3))
    pad = .005
    leaves = mesh.bvh_children[:, 0] < 0
    lower, upper = mesh.bvh_lower[leaves]-pad-1e-9, mesh.bvh_upper[leaves]+pad+1e-9
    # Exercise exact inclusive faces and their immediate representable neighbors.
    points = np.vstack((points, lower, upper, np.nextafter(lower, -np.inf),
                        np.nextafter(upper, np.inf), [[np.nan, 0, 0], [np.inf, 0, 0]]))
    expected = np.zeros(len(points), dtype=bool)
    for lo, hi in zip(lower, upper):
        expected |= np.isfinite(points).all(axis=1) & (points >= lo).all(axis=1) & (points <= hi).all(axis=1)
    actual, incomplete = mesh.surface_candidates(points, pad)
    assert not incomplete
    np.testing.assert_array_equal(actual, expected)
    expired, incomplete = mesh.surface_candidates(points, pad, time.monotonic()-1)
    assert incomplete and not expired.any()


def test_mesh_parallel_and_tiny_directions_match_all_triangle_reference(tmp_path):
    mesh_path = tmp_path/'parallel_surface.stl'
    _write_triangles(mesh_path, _box_triangles([-.09]*3, [.09]*3))
    from d1max_pct_scan.robot_self_geometry import _MESH_CACHE
    model = leg_model(tmp_path, mesh_path)
    mesh = _MESH_CACHE.get(next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK'))
    origins, endpoints = [], []
    for axis in range(3):
        for side in (-1., 1.):
            for offset in (0., .09, -.09, .090000001, -.090000001):
                for drift in (0., 1e-13, -1e-13, 2e-12, -2e-12):
                    origin, endpoint = np.zeros(3), np.zeros(3)
                    origin[axis], endpoint[axis] = side*.5, -side*.1
                    origin[(axis+1)%3], endpoint[(axis+1)%3] = offset, offset+drift
                    origins.append(origin); endpoints.append(endpoint)
    origins, endpoints = np.asarray(origins), np.asarray(endpoints)
    expected = np.full(len(endpoints), np.inf)
    # Independent scalar all-triangle solves; no BVH or paired-axis helper.
    for index, (origin, endpoint) in enumerate(zip(origins, endpoints)):
        length = np.linalg.norm(endpoint-origin)
        direction = (endpoint-origin)/length
        for triangle in mesh.triangles.astype(float):
            vertex = triangle[0]
            system = np.column_stack((direction, vertex-triangle[1], vertex-triangle[2]))
            try:
                distance, u, v = np.linalg.solve(system, vertex-origin)
            except np.linalg.LinAlgError:
                continue
            if (0 <= distance <= length+.005+1e-9 and u >= -1e-9 and v >= -1e-9
                    and u+v <= 1+1e-9):
                expected[index] = min(expected[index], distance)
    actual, incomplete = mesh.first_intersections(origins, endpoints, .005)
    assert not incomplete
    np.testing.assert_allclose(actual, expected, atol=1e-10)


def test_mesh_expiration_after_partial_triangle_work_discards_all_hits(tmp_path, monkeypatch):
    mesh_path = tmp_path/'partial_then_expired.stl'
    triangle = np.array([[-.1, -.1, 0], [.1, -.1, 0], [0, .1, 0]])
    _write_triangles(mesh_path, [triangle]*128)
    from d1max_pct_scan import robot_self_geometry as geometry
    model = leg_model(tmp_path, mesh_path)
    mesh = geometry._MESH_CACHE.get(next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK'))
    now = [0.]
    original = mesh._leaf_pairs
    def expire_after_first_batch(*args):
        for index, batch in enumerate(original(*args)):
            if index:
                now[0] = 6.
            yield batch
    monkeypatch.setattr(mesh, '_leaf_pairs', expire_after_first_batch)
    monkeypatch.setattr(geometry.time, 'monotonic', lambda: now[0])
    actual, incomplete = mesh._first_intersections_python(np.tile([0., 0., 1.], (400, 1)),
        np.zeros((400, 3)), 0., deadline=5.)
    assert now[0] == 6. and incomplete
    assert np.isinf(actual).all()


def test_fixed_stance_keeps_all_rays_on_candidate_exhaustion(tmp_path):
    model = FixedStanceModel.from_base_model(leg_model(tmp_path))
    center = model.nominal_wheel_centers['RBL']
    result = model.classify([center]*4, [-.7, 0, -.1],
        config=SelfFilterConfig(max_candidate_points=2, max_classification_ms=1000))
    assert result.candidate_limit_reached
    assert result.processed_candidate_points == 2
    assert not result.matched_mask.any()
    assert dict(result.per_link_counts) == {}


@pytest.mark.parametrize('profile', [
    {'lidar_ground_height_m': float('nan')}, {'wheelbase_m': True},
    {'wheel_diameter_m': .5}, {'standing_outer_width_m': .5}, {'wheelbase_m': None},
])
def test_invalid_or_unmeasured_dimension_contract_is_rejected(profile):
    with pytest.raises(ValueError):
        FixedStanceProfile.coerce(profile)


def test_unreachable_stance_cannot_silently_clip_to_joint_or_workspace_limit(tmp_path):
    with pytest.raises(ValueError, match='unreachable'):
        FixedStanceModel.from_base_model(leg_model(tmp_path),
            profile={'lidar_ground_height_m': 1.1})


@pytest.mark.skipif(not OFFICIAL.is_file(), reason='official D1 Max CAD unavailable')
def test_official_cad_dimensions_and_true_first_surface_not_voxel_center(tmp_path):
    model = FixedStanceModel.from_urdf(OFFICIAL)
    value = model.to_dict()
    assert len(model.collisions) == 29
    assert value['nominal_root_ground_height_m'] == pytest.approx(.5377)
    np.testing.assert_allclose(model.nominal_wheel_centers['RBL'],
                               [-.375, .2147, -.4427], atol=1e-8)
    assert model.nominal_joint_positions['RBL_HIP_JOINT'] == pytest.approx(-.525012263117)
    assert model.nominal_joint_positions['RBL_KNEE_JOINT'] == pytest.approx(1.203892495627)
    # This historical position is a voxel centre.  It is behind the CAD
    # wheel's first surface, not evidence for fitting a deleting posture.
    from d1max_pct_scan.robot_self_geometry import _MESH_CACHE
    collision = next(c for c in model.collisions if c.link == 'RBL_FOOT_LINK')
    mesh = _MESH_CACHE.get(collision)
    pose = np.asarray(value['nominal_link_transforms_root']['RBL_FOOT_LINK'])
    origin, endpoint = np.array([-.328, 0, -.033]), np.array([-.361, .223, -.412])
    local_origin = (origin-pose[:3, 3]) @ pose[:3, :3]
    local_endpoint = (endpoint-pose[:3, 3]) @ pose[:3, :3]
    first = mesh.first_intersection(local_origin, local_endpoint, .005)
    assert first is not None
    assert np.linalg.norm(endpoint-origin)-first == pytest.approx(.06129955, abs=1e-6)
    # Query the isolated wheel CAD so unsupported earlier bodies cannot hide
    # a permissive filled-wheel result.
    from dataclasses import replace
    wheel_only = replace(model.base_model, collisions=(collision,))
    result = wheel_only.classify([endpoint], origin, model.nominal_joint_positions, config=CONFIG)
    assert result.matched_mask.tolist() == [False]
    assert result.extends_free_rays is False


def test_prepared_fixed_collision_frames_match_uncached_actual_origin_queries(tmp_path, monkeypatch):
    model = FixedStanceModel.from_base_model(leg_model(tmp_path))
    points = np.asarray([model.nominal_wheel_centers['RBL'],
                         [0., 0., -.5377], [-.375, .32, -.4427],
                         model.nominal_wheel_centers['FAR']])
    # Actual rear origin, not the configured symmetric CAD origin.
    origin = [-.328, 0., -.033]
    expected = model.base_model._classify_at_poses(points, origin, model._poses, config=CONFIG)
    before = model.content_hash
    for frame in model._collision_frames[0]:
        with pytest.raises(TypeError):
            frame['world_lower'] = np.zeros(3)
        assert not frame['rotation'].flags.writeable
        assert not frame['translation'].flags.writeable
        assert not frame['world_lower'].flags.writeable
    def reprepare(*args, **kwargs):
        pytest.fail('fixed collision transforms were recomputed in a live query')
    monkeypatch.setattr(RobotSelfModel, '_prepare_collision_frames', reprepare)
    actual = model.classify(points, origin, config=CONFIG)
    np.testing.assert_array_equal(actual.matched_mask, expected.matched_mask)
    assert dict(actual.per_link_counts) == dict(expected.per_link_counts)
    assert actual.processed_candidate_points == expected.processed_candidate_points
    assert actual.origin_inside_links == expected.origin_inside_links
    assert model.content_hash == before
