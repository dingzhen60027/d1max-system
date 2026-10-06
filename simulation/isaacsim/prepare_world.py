"""Seal a generated campus and the approved robot collision identity.

This authoring step uses OpenUSD only. No ROS participant or SimulationApp is
created, no gait is executed, and no navigation success is inferred from it.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil


def prepare_world(config_path, simulation_directory):
    from pxr import Gf, Usd, UsdGeom
    import quadruped
    from truth_map import robot_registry_from_stage, verify_stage_geometry
    from world_builder import author_world, validate_world

    config_path, directory = Path(config_path), Path(simulation_directory)
    spec = json.loads(config_path.read_text())
    validate_world(spec)
    robot = spec['robot']
    source_root = Path(robot.get('asset_root', os.environ.get(
        'D1MAX_QUADRUPED_ASSET_ROOT', quadruped.DEFAULT_ASSET_ROOT))).resolve()
    quadruped.verify_asset_manifest(source_root/'asset_manifest.json')
    target = directory/'robot_assets'
    shutil.copytree(source_root, target, symlinks=False, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    robot.update(kind=quadruped.MODEL_KIND, prim_path=quadruped.DEFAULT_ROOT_PATH,
        asset_root=str(target.resolve()), asset_manifest_path=str((target/'asset_manifest.json').resolve()),
        usd_path=str((target/quadruped.USD_RELATIVE).resolve()),
        policy_path=str((target/quadruped.POLICY_RELATIVE).resolve()),
        env_config_path=str((target/quadruped.ENV_RELATIVE).resolve()))
    spec['flat_support_contact'] = dict(enabled=True,
        floor_path='/World/GroundPlane/collisionPlane', floor_z=float(spec['floor']['z']),
        penetration_allowance_m=.02, floor_endpoint_error_bound_m=.0002)
    stage = Usd.Stage.CreateInMemory()
    author_world(stage, spec)
    root = UsdGeom.Xform.Define(stage, robot['prim_path'])
    root.GetPrim().GetReferences().AddReference(robot['usd_path'])
    variant = root.GetPrim().GetVariantSets().GetVariantSet('Physics')
    if 'physx' in variant.GetVariantNames():
        variant.SetVariantSelection('physx')
    x, y, z, yaw = robot['initial_pose']
    transform = UsdGeom.Xformable(root.GetPrim())
    # This is the authored initial spawn only. Runtime robot motion is PhysX.
    transform.ClearXformOpOrder()
    transform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    transform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(math.cos(yaw/2), Gf.Vec3d(0, 0, math.sin(yaw/2))))
    spec['robot_collision_registry'] = robot_registry_from_stage(stage, robot['prim_path'])
    if spec.get('scenario_suite_contract'):
        from scenario_suite import configuration_digest
        spec['scenario_suite_contract']['configuration_sha256'] = configuration_digest(spec)
    config_path.write_text(json.dumps(spec, indent=2, allow_nan=False)+'\n')
    artifact = directory/'assets/campus_scene.usda'
    stage.Flatten().Export(str(artifact.resolve()))
    # Certify the flattened collision source, rather than trusting JSON alone.
    loaded = Usd.Stage.Open(str(artifact.resolve()))
    geometry_hash = verify_stage_geometry(loaded, spec)
    return dict(collision_stage=str(artifact), static_geometry_sha256=geometry_hash,
        robot_registry_sha256=hashlib.sha256(json.dumps(spec['robot_collision_registry'],
            sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest(),
        robot_assets_manifest_sha256=hashlib.sha256((target/'asset_manifest.json').read_bytes()).hexdigest(),
        quadruped_locomotion_verified=False, physical_acceptance=False)
