"""Pure actual-entity tests, including unsafe near-body-clear counterexamples."""
import json
from pathlib import Path
import struct

import numpy as np
import pytest

from d1max_pct_scan.robot_self_geometry import RobotSelfModel, SelfFilterConfig


CONFIG = SelfFilterConfig(max_classification_ms=1000)


def model_at(tmp_path, contents):
    path = tmp_path/'robot.urdf'
    path.write_text('<robot name="test">'+contents+'</robot>')
    return RobotSelfModel.from_urdf(path)


def box_link(name, size='.2 .2 .2', xyz='0 0 0'):
    return f'''<link name="{name}"><visual><geometry><box size="10 10 10"/></geometry></visual>
      <collision><origin xyz="{xyz}"/><geometry><box size="{size}"/></geometry></collision></link>'''


def test_real_collision_not_visual_and_no_nearbody_box(tmp_path):
    model = model_at(tmp_path, box_link('BASE_LINK'))
    points = [[0, 0, 0], [.5, 0, 0], [0, 0, -.5], [0, .2, -.05]]
    result = model.classify(points, [1, 0, 0], config=CONFIG)
    assert result.matched_mask.tolist() == [True, False, False, False]
    assert dict(result.per_link_counts) == {'BASE_LINK': 1}
    assert result.extends_free_rays is False
    with pytest.raises(ValueError): result.matched_mask[0] = False
    with pytest.raises(TypeError): result.per_link_counts['BASE_LINK'] = 4


def test_missing_joint_is_not_zero_and_descendants_are_skipped(tmp_path):
    model = model_at(tmp_path, '<link name="BASE"/>'+box_link('HIP')+box_link('WHEEL')+'''
      <joint name="hip" type="revolute"><parent link="BASE"/><child link="HIP"/>
        <origin xyz="0 .3 -.2"/><axis xyz="0 1 0"/></joint>
      <joint name="wheel" type="continuous"><parent link="HIP"/><child link="WHEEL"/>
        <origin xyz=".3 0 0"/><axis xyz="0 1 0"/></joint>''')
    assert model.joint_types == {'hip': 'revolute', 'wheel': 'continuous'}
    result = model.classify([[0, .3, -.2], [.3, .3, -.2]], [1, 0, 0], config=CONFIG)
    assert not result.matched_mask.any()
    assert result.missing_links == ('HIP', 'WHEEL')
    partial = model.classify([[0, .3, -.2], [.3, .3, -.2]], [1, 0, 0],
                             {'hip': 0.}, config=CONFIG)
    assert partial.matched_mask.tolist() == [True, False]
    assert partial.missing_links == ('WHEEL',)


def test_fk_parent_to_child_direction_and_si_angles(tmp_path):
    model = model_at(tmp_path, '<link name="BASE"/>'+box_link('HIP')+box_link('WHEEL')+'''
      <joint name="hip" type="revolute"><parent link="BASE"/><child link="HIP"/>
        <origin xyz=".4 0 0"/><axis xyz="0 1 0"/></joint>
      <joint name="wheel" type="continuous"><parent link="HIP"/><child link="WHEEL"/>
        <origin xyz="0 0 -.3"/><axis xyz="0 1 0"/></joint>''')
    poses, missing = model.forward_kinematics({'hip': np.pi/2, 'wheel': 2*np.pi})
    assert missing == ()
    np.testing.assert_allclose(poses['WHEEL'][:3, 3], [.1, 0, 0], atol=1e-12)
    result = model.classify([[.1, 0, 0], [.4, 0, -.3]], [.1, 1, 0],
                            {'hip': np.pi/2, 'wheel': 2*np.pi}, config=CONFIG)
    assert result.matched_mask.tolist() == [True, False]
    with pytest.raises(ValueError, match='unmapped'): model.forward_kinematics({'SDK_1': 0})
    with pytest.raises(ValueError): model.forward_kinematics({'hip': float('nan')})
    with pytest.raises(ValueError): model.forward_kinematics({'hip': True})


def test_prismatic_is_source_pose_not_nominal_default(tmp_path):
    model = model_at(tmp_path, '<link name="BASE"/>'+box_link('PAYLOAD')+'''
      <joint name="slide" type="prismatic"><parent link="BASE"/><child link="PAYLOAD"/>
        <axis xyz="0 0 1"/></joint>''')
    poses, _ = model.forward_kinematics({'slide': .4})
    np.testing.assert_allclose(poses['PAYLOAD'][:3, 3], [0, 0, .4])


def test_static_dynamic_split_has_identical_union(tmp_path):
    model = model_at(tmp_path, box_link('BASE')+box_link('LEG')+'''
      <joint name="leg" type="revolute"><parent link="BASE"/><child link="LEG"/>
        <origin xyz="0 .3 -.3"/><axis xyz="1 0 0"/></joint>''')
    points, origin, joints = [[0, 0, 0], [0, .3, -.3], [0, .15, -.5]], [1, 0, 0], {'leg': 0}
    all_hits = model.classify(points, origin, joints, config=CONFIG).matched_mask
    static = model.classify(points, origin, joints, include_dynamic=False, config=CONFIG).matched_mask
    dynamic = model.classify(points, origin, joints, include_static=False, config=CONFIG).matched_mask
    np.testing.assert_array_equal(all_hits, static | dynamic)
    assert all_hits.tolist() == [True, True, False]  # floor between legs stays a real return


def test_origin_inside_entity_is_diagnostic_not_a_clearance_license(tmp_path):
    model = model_at(tmp_path, box_link('BASE'))
    result = model.classify([[0, 0, 0], [.5, 0, 0]], [0, 0, 0], config=CONFIG)
    assert not result.matched_mask.any()
    assert result.origin_inside_links == ('BASE',)


def test_explicit_cylinder_wheel_preserves_ground_and_neighbor_obstacle(tmp_path):
    model = model_at(tmp_path, '''<link name="BASE"><collision><origin xyz="0 .25 -.4" rpy="1.5707963267948966 0 0"/>
      <geometry><cylinder radius=".095958" length=".046499"/></geometry></collision></link>''')
    points = [[0, .25, -.4], [0, .25, -.51], [0, .3, -.4], [.15, .25, -.4]]
    result = model.classify(points, [1, .25, -.4], config=CONFIG)
    assert result.matched_mask.tolist() == [True, False, False, False]


def write_box_mesh(path, lower, upper, append=False):
    lo, hi = np.asarray(lower), np.asarray(upper)
    vertices = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
                         [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
                         [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
                         [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]])
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (3, 7, 6), (3, 6, 2),
             (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5)]
    previous = path.read_bytes()[84:] if append else b''
    records = b''.join(struct.pack('<12fH', *(list(np.zeros(3))+list(vertices[list(face)].ravel())), 0)
                       for face in faces)
    count = len(previous+records)//50
    path.write_bytes(bytes(80)+struct.pack('<I', count)+previous+records)


def test_actual_mesh_surface_not_convex_filled_concavity_or_ray_shadow(tmp_path):
    mesh = tmp_path/'entity.stl'
    write_box_mesh(mesh, [-.3, -.3, -.3], [-.1, .3, .3])
    write_box_mesh(mesh, [-.3, -.3, -.3], [.3, -.1, .3], append=True)
    model = model_at(tmp_path, '''<link name="BASE"><collision><geometry>
      <mesh filename="entity.stl"/></geometry></collision></link>''')
    model.prewarm()
    # Empty notch is inside the convex hull; an earlier body intersection must
    # not make the notch or the obstacle behind the body a self return.
    points = [[-.3, .05, 0], [.05, .05, 0], [.5, .05, 0], [-.2, .05, 0]]
    result = model.classify(points, [-1, .05, 0], config=CONFIG)
    assert result.matched_mask.tolist() == [True, False, False, False]
    assert result.extends_free_rays is False
    padded = model.classify([[-.305, .05, 0], [-.33, .05, 0]], [-1, .05, 0],
                            config=SelfFilterConfig(measurement_padding_m=.01,
                                                    max_classification_ms=1000))
    assert padded.matched_mask.tolist() == [True, False]


def test_surface_on_far_side_is_not_removed_as_first_body_return(tmp_path):
    write_box_mesh(tmp_path/'entity.stl', [-.1]*3, [.1]*3)
    model = model_at(tmp_path, '''<link name="BASE"><collision><geometry>
      <mesh filename="entity.stl"/></geometry></collision></link>''')
    model.prewarm()
    result = model.classify([[-.1, 0, 0], [.1, 0, 0]], [-1, 0, 0], config=CONFIG)
    assert result.matched_mask.tolist() == [True, False]


def test_manifest_seals_collision_source_and_never_certifies_extrinsics(tmp_path):
    model = model_at(tmp_path, box_link('BASE'))
    value = model.to_dict()
    assert value['uses_visual_geometry'] is False
    assert value['extrinsics_calibrated'] is False
    assert value['posture_calibrated'] is False
    assert value['grants_motion_authority'] is False
    restored = RobotSelfModel.from_dict(json.loads(json.dumps(value)))
    assert restored.content_hash == model.content_hash
    value['collisions'][0]['upper'][0] = 10
    with pytest.raises(ValueError, match='changed'): RobotSelfModel.from_dict(value)
    value = model.to_dict()
    Path(model.urdf_path).write_text('<robot name="changed"><link name="BASE"/></robot>')
    with pytest.raises(ValueError, match='changed'): RobotSelfModel.from_dict(value)


def test_manifest_mesh_hash_detects_modified_asset(tmp_path):
    write_box_mesh(tmp_path/'entity.stl', [-.1]*3, [.1]*3)
    model = model_at(tmp_path, '''<link name="BASE"><collision><geometry>
      <mesh filename="entity.stl"/></geometry></collision></link>''')
    value = model.to_dict()
    write_box_mesh(tmp_path/'entity.stl', [-.2]*3, [.2]*3)
    with pytest.raises(ValueError, match='changed'): RobotSelfModel.from_dict(value)


def test_warmed_mesh_is_frozen_and_not_a_mutable_disk_mmap(tmp_path):
    mesh = tmp_path/'frozen.stl'
    write_box_mesh(mesh, [-.111]*3, [.111]*3)
    model = model_at(tmp_path, '''<link name="BASE"><collision><geometry>
      <mesh filename="frozen.stl"/></geometry></collision></link>''')
    model.warmup()
    write_box_mesh(mesh, [-.222]*3, [.222]*3)
    result = model.classify([[-.111, 0, 0]], [-1, 0, 0], config=CONFIG)
    assert result.matched_mask.tolist() == [True]


@pytest.mark.parametrize('joint', [
    '<joint name="a" type="revolute"><parent link="BASE"/><child link="LEG"/><axis xyz="0 0 0"/></joint>',
    '<joint name="a" type="floating"><parent link="BASE"/><child link="LEG"/></joint>',
    '<joint name="a" type="revolute"><parent link="BASE"/><child link="LEG"/><mimic joint="other"/></joint>',
])
def test_unimplemented_or_ambiguous_pose_semantics_are_not_guessed(tmp_path, joint):
    with pytest.raises(ValueError): model_at(tmp_path, '<link name="BASE"/>'+box_link('LEG')+joint)


def test_budget_and_invalid_returns_preserve_unmatched_observations(tmp_path):
    model = model_at(tmp_path, box_link('BASE'))
    result = model.classify([[0, 0, 0]]*5+[[float('nan'), 0, 0]], [1, 0, 0],
                            config=SelfFilterConfig(max_candidate_points=2,
                                                    max_classification_ms=1000))
    assert not result.matched_mask.any()  # budget never silently makes partial free evidence
    assert result.candidate_limit_reached
    assert result.processed_candidate_points == 2


def test_cold_mesh_never_loads_inside_classification(tmp_path, monkeypatch):
    write_box_mesh(tmp_path/'cold_unique.stl', [-.123]*3, [.123]*3)
    model = model_at(tmp_path, '''<link name="BASE"><collision><geometry>
      <mesh filename="cold_unique.stl"/></geometry></collision></link>''')
    from d1max_pct_scan import robot_self_geometry as geometry
    def cannot_load(*args): raise AssertionError('live scan cannot construct a mesh tree')
    monkeypatch.setattr(geometry._MESH_CACHE, 'get', cannot_load)
    result = model.classify([[-.123, 0, 0]], [-1, 0, 0], config=CONFIG)
    assert not result.matched_mask.any()
    assert result.unloaded_links == ('BASE',)


def test_warmup_skips_static_mesh_when_all_origins_are_ambiguous(tmp_path, monkeypatch):
    write_box_mesh(tmp_path/'inside.stl', [-.321]*3, [.321]*3)
    model = model_at(tmp_path, '''<link name="BASE"><collision><geometry>
      <mesh filename="inside.stl"/></geometry></collision></link>''')
    from d1max_pct_scan import robot_self_geometry as geometry
    def cannot_load(*args): raise AssertionError('unused torso mesh must not be loaded')
    monkeypatch.setattr(geometry._MESH_CACHE, 'get', cannot_load)
    report = model.warmup(sensor_origins=[[.2, 0, 0], [-.2, 0, 0]])
    assert report['origin_ambiguous_skipped_links'] == ('BASE',)
    result = model.classify([[.321, 0, 0]], [.2, 0, 0], config=CONFIG)
    assert not result.matched_mask.any()
    assert result.origin_ambiguous_links == ('BASE',)


def test_manifest_load_does_not_prewarm_and_asset_paths_are_sealable(tmp_path, monkeypatch):
    model = model_at(tmp_path, box_link('BASE'))
    manifest = tmp_path/'model.json'
    manifest.write_text(json.dumps(model.to_dict()))
    assert model.asset_paths == (Path(model.urdf_path),)
    def cannot_load(*args): raise AssertionError('manifest preparation cannot warm meshes')
    monkeypatch.setattr(RobotSelfModel, 'warmup', cannot_load)
    assert RobotSelfModel.load(manifest).content_hash == model.content_hash


@pytest.mark.parametrize('padding', [-.001, .021, float('nan')])
def test_padding_cannot_become_safety_envelope_or_nearbody_erase(padding):
    with pytest.raises(ValueError): SelfFilterConfig(measurement_padding_m=padding)


def test_missing_collision_never_falls_back_to_visual(tmp_path):
    model = model_at(tmp_path, '<link name="BASE"><visual><geometry><box size="2 2 2"/></geometry></visual></link>')
    assert model.collisions == ()
    assert not model.classify([[0, 0, 0]], [1, 0, 0], config=CONFIG).matched_mask.any()


def test_real_d1max_cad_sensor_origins_and_wheel_dimensions_if_assets_present():
    asset = Path('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/urdf_ws/src/max_description/urdf/max.urdf')
    if not asset.is_file(): pytest.skip('official D1 Max assets unavailable on this host')
    model = RobotSelfModel.from_urdf(asset)
    assert model.root_link == 'BASE_LINK'
    assert len(model.collisions) == 29
    poses, missing = model.forward_kinematics()
    np.testing.assert_allclose(poses['F_AIRY_LINK'][:3, 3], [.4043, 0, -.0377])
    np.testing.assert_allclose(poses['R_AIRY_LINK'][:3, 3], [-.4043, 0, -.0377])
    assert all(name not in poses for name in ('FAR_FOOT_LINK', 'FBL_FOOT_LINK',
                                             'RAR_FOOT_LINK', 'RBL_FOOT_LINK'))
    assert len(missing) == 16
    body = next(c for c in model.collisions if c.link == 'BASE_LINK')
    assert body.lower[2] == pytest.approx(-.0893)
    wheel = next(c for c in model.collisions if c.link == 'FAR_FOOT_LINK')
    assert wheel.upper[2]-wheel.lower[2] == pytest.approx(.191916257)
    assert wheel.upper[1]-wheel.lower[1] == pytest.approx(.046498813)
    assert wheel.lower[1] != -wheel.upper[1]  # actual asymmetric link-local offset


def wheel_model(tmp_path, extra=''):
    return model_at(tmp_path, '''<link name="BASE"/><link name="WHEEL">
      <collision><origin xyz=".3 0 0" rpy="1.5707963267948966 0 0"/>
      <geometry><cylinder radius=".095958" length=".046499"/></geometry></collision></link>
      <joint name="angle" type="continuous"><parent link="BASE"/><child link="WHEEL"/>
        <origin xyz="-.2 0 -.4"/><axis xyz="0 0 1"/></joint>'''+extra)


def test_dense_ground_does_not_consume_entity_candidate_budget_and_exact_beam_angles(tmp_path):
    model = wheel_model(tmp_path)
    count = 12000
    ground = np.column_stack((np.linspace(-.7, .7, count), np.zeros(count),
                              np.full(count, -.55)))
    angles = np.linspace(-.6, .6, 7)
    wheels = np.column_stack((-.2+.3*np.cos(angles), .3*np.sin(angles),
                              np.full(len(angles), -.4)))
    points = np.vstack((ground, wheels))
    column = np.r_[np.linspace(-.9, .9, count), angles]
    result = model.classify_articulated(points, [1, 1, 0], {'angle': column},
        config=SelfFilterConfig(max_candidate_points=16, max_classification_ms=1000))
    assert not result.matched_mask[:count].any()
    assert result.matched_mask[count:].all()
    assert result.processed_candidate_points == 7
    assert not result.candidate_limit_reached
    assert not result.timed_out


def test_batch_uses_distinct_per_beam_q_not_group_rounded_or_latest(tmp_path):
    model = wheel_model(tmp_path)
    points = [[.1, 0, -.4]]*4
    snapshots = [{'angle': 0.}, {'angle': np.pi}, {}, {'angle': 0.}]
    result = model.classify_articulated(points, [1, 1, 0], snapshots, config=CONFIG)
    assert result.matched_mask.tolist() == [True, False, False, True]
    assert result.missing_links == ('WHEEL',)
    columns = model.classify_articulated(points, [1, 1, 0],
        {'angle': np.array([0., np.pi, np.nan, 0.])}, config=CONFIG)
    np.testing.assert_array_equal(result.matched_mask, columns.matched_mask)


def test_known_articulated_part_not_disabled_by_unrelated_missing_chain(tmp_path):
    extra = box_link('OTHER')+'''<joint name="other" type="revolute"><parent link="BASE"/>
       <child link="OTHER"/><origin xyz="-.7 .5 -.4"/><axis xyz="0 1 0"/></joint>'''
    model = wheel_model(tmp_path, extra)
    result = model.classify_articulated([[.1, 0, -.4]], [1, 1, 0],
                                        {'angle': np.array([0.])}, config=CONFIG)
    assert result.matched_mask.tolist() == [True]
    assert result.missing_links == ('OTHER',)
    assert result.unsupported_occluder_links == ('OTHER',)


def test_batch_deadline_preserves_all_rays_and_never_resets_per_fk(tmp_path, monkeypatch):
    model = wheel_model(tmp_path)
    from d1max_pct_scan import robot_self_geometry as geometry
    clock = iter([0., .002, .003, .004, .005, .006, .007])
    monkeypatch.setattr(geometry.time, 'monotonic', lambda: next(clock, 1.))
    result = model.classify_articulated([[.1, 0, -.4]]*10, [1, 1, 0],
        {'angle': np.zeros(10)}, config=SelfFilterConfig(max_classification_ms=1.))
    assert result.timed_out
    assert not result.matched_mask.any()
    assert dict(result.per_link_counts) == {}


def test_batch_cold_mesh_and_missing_q_do_not_load_or_erase(tmp_path, monkeypatch):
    write_box_mesh(tmp_path/'moving_cold.stl', [-.127379]*3, [.127379]*3)
    model = model_at(tmp_path, '''<link name="BASE"/><link name="LEG"><collision><geometry>
      <mesh filename="moving_cold.stl"/></geometry></collision></link>
      <joint name="q" type="revolute"><parent link="BASE"/><child link="LEG"/><axis xyz="0 1 0"/></joint>''')
    from d1max_pct_scan import robot_self_geometry as geometry
    monkeypatch.setattr(geometry._MESH_CACHE, 'get', lambda *args: pytest.fail('no live mesh load'))
    result = model.classify_articulated([[-.127379, 0, 0]]*2, [-1, 0, 0],
        {'q': np.array([0., np.nan])}, config=CONFIG)
    assert not result.matched_mask.any()
    assert result.unloaded_links == ('LEG',)
    assert result.missing_links == ('LEG',)


def test_split_and_combined_model_have_same_first_supported_surface_rule(tmp_path):
    near, far = tmp_path/'near.stl', tmp_path/'far.stl'
    write_box_mesh(near, [-.5, -.2, -.2], [-.49, .2, .2])
    write_box_mesh(far, [.1, -.2, -.2], [.11, .2, .2])
    split = model_at(tmp_path, '''<link name="BASE"><collision><geometry><mesh filename="near.stl"/></geometry></collision>
      <collision><geometry><mesh filename="far.stl"/></geometry></collision></link>''')
    split.warmup()
    split_result = split.classify([[.1, 0, 0], [-.5, 0, 0]], [-1, 0, 0], config=CONFIG)
    combined_file = tmp_path/'combined.stl'
    write_box_mesh(combined_file, [-.5, -.2, -.2], [-.49, .2, .2])
    write_box_mesh(combined_file, [.1, -.2, -.2], [.11, .2, .2], append=True)
    combined = model_at(tmp_path, '''<link name="BASE"><collision><geometry>
      <mesh filename="combined.stl"/></geometry></collision></link>''')
    combined.warmup()
    combined_result = combined.classify([[.1, 0, 0], [-.5, 0, 0]], [-1, 0, 0], config=CONFIG)
    assert split_result.matched_mask.tolist() == [False, True]
    np.testing.assert_array_equal(split_result.matched_mask, combined_result.matched_mask)


def test_batch_rejects_later_entity_surface_behind_supported_other_link(tmp_path):
    model = model_at(tmp_path, '<link name="BASE"/>'+box_link('NEAR', '.01 .2 .2')+
        box_link('FAR', '.01 .2 .2')+'''
        <joint name="near" type="fixed"><parent link="BASE"/><child link="NEAR"/><origin xyz="-.5 0 0"/></joint>
        <joint name="far" type="revolute"><parent link="BASE"/><child link="FAR"/><origin xyz=".1 0 0"/><axis xyz="0 0 1"/></joint>''')
    result = model.classify_articulated([[.1, 0, 0]], [-1, 0, 0],
                                       {'far': np.zeros(1)}, config=CONFIG)
    assert not result.matched_mask.any()


def test_full_model_batch_static_target_checks_exact_dynamic_blocker(tmp_path):
    model = model_at(tmp_path, box_link('BASE', '.01 .2 .2', '.1 0 0')+
        box_link('LEG', '.1 .1 .1', '0 .3 0')+'''
        <joint name="leg" type="revolute"><parent link="BASE"/><child link="LEG"/>
          <origin xyz="-.5 0 0"/><axis xyz="0 0 1"/></joint>''')
    points, origins = [[.1, 0, 0]]*2, [-1, 0, 0]
    # Same static endpoint: the first ray has a clear source/model connector;
    # the second dynamic joint places LEG between scanner and static BASE.
    angles = np.array([0., -np.pi/2])
    full = model.classify_articulated(points, origins, {'leg': angles},
                                      include_static=True, config=CONFIG)
    assert full.matched_mask.tolist() == [True, False]
    for index, angle in enumerate(angles):
        scalar = model.classify([points[index]], origins, {'leg': angle}, config=CONFIG)
        assert scalar.matched_mask[0] == full.matched_mask[index]
    dynamic_only = model.classify_articulated(points, origins, {'leg': angles}, config=CONFIG)
    assert not dynamic_only.matched_mask.any()


def test_full_model_batch_matches_scalar_for_known_and_missing_per_chain(tmp_path):
    model = wheel_model(tmp_path, box_link('PAYLOAD', '.1 .1 .1', '0 .4 .2')+'''
      <joint name="payload" type="fixed"><parent link="BASE"/><child link="PAYLOAD"/></joint>''')
    points = [[.1, 0, -.4], [0, .4, .2], [.1, 0, -.4]]
    snapshots = [{'angle': 0.}, {}, {}]
    full = model.classify_articulated(points, [1, 1, 0], snapshots,
                                      include_static=True, config=CONFIG)
    expected = [bool(model.classify([point], [1, 1, 0], snapshot,
                                   config=CONFIG).matched_mask[0])
                for point, snapshot in zip(points, snapshots)]
    assert full.matched_mask.tolist() == expected == [True, True, False]


@pytest.mark.parametrize('snapshots', [[], [{'angle': True}], {'angle': np.array([True])},
                                      {'angle': np.zeros(2)}, [{'unknown': 0.}]])
def test_batch_unmapped_or_malformed_pose_never_becomes_zero_snapshot(tmp_path, snapshots):
    model = wheel_model(tmp_path)
    with pytest.raises(ValueError):
        model.classify_articulated([[.1, 0, -.4]], [1, 1, 0], snapshots, config=CONFIG)


def test_axis_bounds_fast_path_is_exact_at_boundaries_invalid_and_strided_points():
    from d1max_pct_scan.robot_self_geometry import _inside_bounds
    lower, upper = np.asarray([-.5, -.2, -.8]), np.asarray([.4, .1, .2])
    points = np.array([lower, upper, [0., 0., 0.],
                       [np.nextafter(lower[0], -np.inf), 0., 0.],
                       [0., np.nextafter(upper[1], np.inf), 0.],
                       [np.nan, 0., 0.], [0., np.inf, 0.]])
    expected = ((points >= lower) & (points <= upper)).all(axis=1)
    np.testing.assert_array_equal(_inside_bounds(points, lower, upper), expected)
    np.testing.assert_array_equal(_inside_bounds(points[::-1], lower, upper), expected[::-1])
    assert _inside_bounds(np.empty((0, 3)), lower, upper).size == 0


@pytest.mark.parametrize('origin', [[-1., 0., 0.], [0., 0., 0.]])
def test_one_actual_origin_matches_repeated_origin_inside_and_occlusion(tmp_path, origin):
    model = model_at(tmp_path, box_link('BASE', '.2 .2 .2'))
    points = np.array([[-.1, 0., 0.], [0., 0., 0.], [.1, 0., 0.], [.4, 0., 0.]])
    single = model.classify(points, origin, config=CONFIG)
    repeated = model.classify(points, np.broadcast_to(origin, points.shape), config=CONFIG)
    np.testing.assert_array_equal(single.matched_mask, repeated.matched_mask)
    assert dict(single.per_link_counts) == dict(repeated.per_link_counts)
    assert single.origin_inside_links == repeated.origin_inside_links
    assert single.origin_ambiguous_links == repeated.origin_ambiguous_links
