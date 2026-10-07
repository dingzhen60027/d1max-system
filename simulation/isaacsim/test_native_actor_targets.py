"""Offline transport/identity guards with actual USD and an injected native API.

No Kit/solver is started. Physical equivalence is the separately sealed real
component A/B; these tests exercise target math and fail-closed dispatch.
"""
import copy
import ast
import importlib.util
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

import world_builder as world


@pytest.mark.parametrize('selected', [None, 'usd_cached_v1', 'native_kinematic_v1'])
def test_world_backend_is_explicit_and_default_retains_usd(selected):
    spec = world.load_world()
    if selected is None:
        # An absent field preserves the legacy API fallback; the generated
        # campus now explicitly selects the verified native fixture backend.
        spec['physics'].pop('dynamic_actor_target_backend', None)
    else:
        spec['physics']['dynamic_actor_target_backend'] = selected
    world.validate_world(spec)
    assert world.dynamic_actor_target_backend(spec) == (selected or 'usd_cached_v1')


def test_generated_campus_explicitly_selects_native_backend():
    assert world.load_world()['physics']['dynamic_actor_target_backend'] == 'native_kinematic_v1'


@pytest.mark.parametrize('selected', [None, False, True, 1, 'native', 'usd_teleport', {}])
def test_unknown_or_malformed_explicit_backend_cannot_fall_back(selected):
    spec = world.load_world()
    spec['physics']['dynamic_actor_target_backend'] = selected
    with pytest.raises(ValueError, match='invalid_dynamic_actor_target_backend'):
        world.validate_world(spec)


def test_native_backend_cannot_be_silently_unused_by_wheel_scene():
    with pytest.raises(ValueError, match='requires_quadruped_physics_callback'):
        world.dynamic_actor_target_backend(dict(physics=dict(dynamic_actor_target_backend='native_kinematic_v1'),
            robot=dict(kind='differential_drive')))


@pytest.mark.parametrize('selected', ['usd_cached_v1', 'native_kinematic_v1'])
def test_scene_selects_only_requested_backend_and_preserves_source_pre_step(selected):
    path = Path(__file__).with_name('scene.py')
    tree = ast.parse(path.read_text())
    block = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
        and isinstance(node.test, ast.Name) and node.test.id == 'IS_QUADRUPED'
        and any(isinstance(child, ast.Name) and child.id == 'dynamic_actor_target_backend'
                for child in ast.walk(node)))
    stage, dynamic, verifier = object(), object(), object()
    calls, registrations, targets = [], [], []
    def cached(s, spec):
        assert s is stage
        calls.append('usd_cached_v1')
        return targets.append
    def native(s, spec, d, v):
        assert s is stage and d is dynamic and v is verifier
        calls.append('native_kinematic_v1')
        return targets.append
    interface = SimpleNamespace(subscribe_physics_on_step_events=lambda **kw: registrations.append(kw))
    ns = dict(IS_QUADRUPED=True, CONFIG=dict(physics=dict(dynamic_actor_target_backend=selected),
            robot=dict(kind='official_spot_physx')), stage=stage, dynamic_view=dynamic,
        wire=SimpleNamespace(stage_geometry_verifier=verifier),
        dynamic_actor_target_backend=world.dynamic_actor_target_backend,
        make_actor_updater=cached, make_native_actor_updater=native,
        physics_core=SimpleNamespace(get_physics_simulation_interface=lambda:interface),
        profile_callback=lambda name, callback:callback,
        SimulationManager=SimpleNamespace(get_simulation_time=lambda:123.5), source_start_time=123.)
    exec(compile(ast.Module(body=[block], type_ignores=[]), str(path), 'exec'), ns)
    assert calls == [selected] and len(registrations) == 1
    assert registrations[0]['pre_step'] is True and registrations[0]['order'] == -1
    registrations[0]['on_update'](.002, None)
    assert targets == [.502]


HAS_USD = importlib.util.find_spec('pxr') is not None


@pytest.fixture
def native_fixture(monkeypatch):
    if not HAS_USD:
        pytest.skip('OpenUSD is required for actual stage identity guards')
    from test_truth_map import CollisionStageTests
    from truth_map import StageGeometryVerifier
    from pxr import Sdf
    stage, spec, prim, _, _ = CollisionStageTests().scripted_actor_stage()
    spec['robot']['kind'] = 'quadruped'  # Synthetic actor transport fixture, no quadruped body claim.
    spec['physics'] = dict(dynamic_actor_target_backend='native_kinematic_v1')
    second = copy.deepcopy(spec['dynamic_actors'][0]); second['id'] = 'person_second'
    for p in second['trajectory']['waypoints']:
        p['position'][0] += 1.
    spec['dynamic_actors'].append(second)
    Sdf.CopySpec(stage.GetRootLayer(), prim.GetPath(), stage.GetRootLayer(), '/World/Dynamic/person_second')
    verifier = StageGeometryVerifier(stage, spec)
    paths = ['/World/Dynamic/person_second', '/World/Dynamic/person']  # Native order is not author order.
    calls = []
    view = SimpleNamespace(prim_paths=paths, count=2, _frontend=SimpleNamespace(device='cpu'),
        check=lambda: True,
        set_kinematic_targets=lambda values, indices: calls.append((values.copy(), indices.copy())))
    dynamic = SimpleNamespace(_physics_rigid_body_view=view, is_physics_tensor_entity_valid=lambda: True)
    simulation = SimpleNamespace(is_valid=True)
    manager = SimpleNamespace(_physics_sim_view__warp=simulation)
    module = ModuleType('isaacsim.core.simulation_manager'); module.SimulationManager = manager
    monkeypatch.setitem(sys.modules, 'isaacsim.core.simulation_manager', module)
    wp = ModuleType('warp'); wp.float32=np.float32; wp.uint32=np.uint32
    wp.array = lambda values, dtype, device, **kw: np.asarray(values, dtype=dtype)
    monkeypatch.setitem(sys.modules, 'warp', wp)
    fixture = SimpleNamespace(stage=stage, spec=spec, prim=prim, verifier=verifier, view=view,
        dynamic=dynamic, simulation=simulation, manager=manager, calls=calls)
    yield fixture
    verifier.close()


def updater(f):
    return world.make_native_actor_updater(f.stage, f.spec, f.dynamic, f.verifier)


def test_targets_use_original_xyz_yaw_native_order_and_single_batch_without_usd_pose_write(native_fixture):
    f = native_fixture
    update = updater(f)
    old_pose = f.prim.GetAttribute('xformOp:translate').Get()
    update(1.)
    values, indices = f.calls[-1]
    assert len(f.calls) == 1 and values.dtype == np.float32
    np.testing.assert_array_equal(indices, np.array([0, 1], dtype=np.uint32))
    by_path = {'/World/Dynamic/'+a['id']:a for a in f.spec['dynamic_actors']}
    for row, path in zip(values, f.view.prim_paths):
        desired = world.actor_pose(by_path[path], 1.)
        half = desired['yaw']/2.
        np.testing.assert_array_equal(row, np.asarray([*desired['position'], 0., 0.,
            math.sin(half), math.cos(half)], dtype=np.float32))
    assert f.prim.GetAttribute('xformOp:translate').Get() == old_pose
    update(2.)
    assert len(f.calls) == 2
    assert not np.array_equal(f.calls[0][0], f.calls[1][0])  # Cached zero-copy buffer is refreshed.


@pytest.mark.parametrize('change', ['simulation', 'invalid_simulation', 'view', 'invalid_view', 'check'])
def test_native_epoch_or_view_loss_stops_before_target_submission(native_fixture, change):
    f = native_fixture; update = updater(f)
    if change == 'simulation': f.manager._physics_sim_view__warp = SimpleNamespace(is_valid=True)
    elif change == 'invalid_simulation': f.simulation.is_valid = False
    elif change == 'view': f.dynamic._physics_rigid_body_view = copy.copy(f.view)
    elif change == 'invalid_view': f.dynamic.is_physics_tensor_entity_valid = lambda: False
    else: f.view.check = lambda: False
    with pytest.raises(ValueError, match='actor_view_lost'): update(1.)
    assert f.calls == []


@pytest.mark.parametrize('change', ['enabled', 'kinematic', 'shape', 'parent', 'time_sample', 'delete'])
def test_original_geometry_or_actor_disable_revoke_without_target_or_usd_fallback(native_fixture, change):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    f = native_fixture; update = updater(f)
    if change == 'enabled': UsdPhysics.RigidBodyAPI(f.prim).CreateRigidBodyEnabledAttr(False)
    elif change == 'kinematic': UsdPhysics.RigidBodyAPI(f.prim).CreateKinematicEnabledAttr(False)
    elif change == 'shape': f.stage.GetPrimAtPath('/World/Dynamic/person/torso').GetAttribute('size').Set(2.)
    elif change == 'parent': UsdGeom.Xformable(f.stage.GetPrimAtPath('/World/Dynamic')).AddTranslateOp().Set(Gf.Vec3d(1.,0.,0.))
    elif change == 'time_sample': f.prim.GetAttribute('xformOp:rotateZ').Set(1., Usd.TimeCode(1.))
    else: f.stage.RemovePrim('/World/Dynamic/person')
    with pytest.raises(ValueError): update(1.)
    assert f.calls == []


@pytest.mark.parametrize('change', ['duplicates', 'missing', 'count', 'device', 'disabled', 'articulation', 'no_verifier', 'not_opted_in'])
def test_constructor_requires_exact_registered_kinematic_cpu_geometry_domain(native_fixture, change):
    from pxr import UsdPhysics
    f = native_fixture
    if change == 'duplicates': f.view.prim_paths[0] = f.view.prim_paths[1]
    elif change == 'missing': f.view.prim_paths[0] = '/World/Unregistered'
    elif change == 'count': f.view.count = 3
    elif change == 'device': f.view._frontend.device = 'cuda:0'
    elif change == 'disabled': UsdPhysics.RigidBodyAPI(f.prim).CreateRigidBodyEnabledAttr(False)
    elif change == 'articulation': UsdPhysics.ArticulationRootAPI.Apply(f.prim)
    elif change == 'no_verifier': f.verifier = None
    else: f.spec['physics'].pop('dynamic_actor_target_backend')
    with pytest.raises(ValueError): updater(f)
    assert f.calls == []


@pytest.mark.parametrize('time', [float('nan'), float('inf'), False])
def test_invalid_source_never_sends_targets(native_fixture, time):
    f = native_fixture; update = updater(f)
    with pytest.raises(ValueError): update(time)
    assert f.calls == []
