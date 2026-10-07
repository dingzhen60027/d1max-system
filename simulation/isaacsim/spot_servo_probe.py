#!/usr/bin/env python3
"""Independent official Spot servo A/B; real joints, no navigation or video.

The candidate gain is not a measured inverse. This probe can reject it using
park and post-walk histories, actual foot flight and the original stop limits.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
import traceback

import numpy as np


def phases(short=False):
    low_frames = 4000 if short else 10000
    result = [('park', 10000, 0.), ('low_after_park', low_frames, .075),
              ('stop_after_park', 2500, 0.), ('walk', 4000, .15),
              ('stop_walk', 2500, 0.)]
    if not short:
        result += [('low_after_walk', low_frames, .075), ('stop_after_walk', 2500, 0.),
                   ('tiny_after_walk', 10000, .01), ('stop_tiny', 2500, 0.)]
    return result


def stop_completion(rows):
    """Actual 500 Hz full-norm .03/.05 continuously for one source second."""
    start = None
    origin = rows[0]['source_ns']
    for row in rows:
        good = (np.linalg.norm(row['linear_velocity']) <= .03
                and np.linalg.norm(row['angular_velocity']) <= .05)
        if good and start is None:
            start = row['source_ns']
        if not good:
            start = None
        if start is not None and row['source_ns'] - start >= 1_000_000_000:
            return (row['source_ns'] - origin) / 1e9
    return None


def summarize(rows, witnesses, schedule):
    result = {}
    for phase, frames, demand in schedule:
        values = [r for r in rows if r['phase'] == phase]
        readings = [r for r in witnesses if r['phase'] == phase]
        p = np.asarray([r['position'] for r in values])
        v = np.asarray([r['linear_velocity'] for r in values])
        a = np.asarray([r['angular_velocity'] for r in values])
        body_vx = np.asarray([r['body_vx'] for r in values])
        air = {side: sum(r['foot_bottom'][side] > .01 for r in readings)
               for side in ('fl', 'fr', 'hl', 'hr')}
        entry = dict(frames=frames, command_vx=demand, duration_s=frames / 500,
            displacement_xyz_m=(p[-1] - p[0]).tolist(),
            xy_distance_m=float(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1).sum()),
            last_velocity_window_s=min(10., frames / 500),
            last_mean_body_vx_mps=float(body_vx[-min(5000, frames):].mean()),
            max_full_linear_speed_mps=float(np.linalg.norm(v, axis=1).max()),
            max_abs_body_yaw_radps=max(abs(r['body_wz']) for r in values),
            foot_flight_samples_50hz=air,
            original_stop_1s_completed_elapsed_s=stop_completion(values) if demand == 0. else None)
        if demand != 0.:
            next_index = schedule.index((phase, frames, demand)) + 1
            zero_phase = schedule[next_index][0]
            stopped = [r for r in rows if r['phase'] == zero_phase]
            lasting = np.asarray(stopped[-1]['position']) - p[0]
            entry['lasting_displacement_after_zero_xyz_m'] = lasting.tolist()
            entry['measured_speed_tracking_passed'] = abs(entry['last_mean_body_vx_mps'] - demand) <= max(.01, .25 * demand)
            entry['lasting_forward_progress_passed'] = bool(lasting[0] >= .5 * demand * frames / 500)
            entry['real_footflight_passed'] = all(value > 0 for value in air.values())
        result[phase] = entry
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result-dir', type=Path, required=True)
    parser.add_argument('--asset-root', type=Path, required=True)
    parser.add_argument('--mode', choices=['spot_nonlinear_measured_v1', 'spot_monotone_measured_v2'], required=True)
    parser.add_argument('--short', action='store_true')
    args = parser.parse_args()
    result = args.result_dir.expanduser().resolve()
    result.mkdir(parents=True, exist_ok=False)
    snapshot = result / 'source_snapshot'
    snapshot.mkdir()
    for name in ('quadruped.py', 'runtime_geometry.py', 'dynamic_collision.py', 'spot_servo_probe.py'):
        shutil.copyfile(Path(__file__).resolve().parent / name, snapshot / name)
    sys.path.insert(0, str(snapshot))
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()}
    print('PROBE_PROCESS', json.dumps(dict(pid=os.getpid(), result_dir=str(result), mode=args.mode,
        source_sha256=hashes)), flush=True)
    sys.argv = [sys.argv[0]]
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'disable_viewport_updates': True,
                         'renderer': 'RaytracedLighting'})
    trajectory_stream = policy_stream = None
    subscription = None
    try:
        import omni.usd
        import omni.physics.core
        from pxr import UsdGeom
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.experimental.objects import GroundPlane
        from isaacsim.core.experimental.prims import GeomPrim
        from isaacsim.core.experimental.materials import RigidBodyMaterial
        from isaacsim.core.simulation_manager import SimulationManager
        from quadruped import QuadrupedPlant, array, rotation_wxyz
        from runtime_geometry import body_certificate
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            omni.usd.get_context().new_stage()
            stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        ground = GroundPlane('/World/GroundPlane', sizes=20., templates=None)
        material = RigidBodyMaterial('/World/Material', static_frictions=1.,
                                     dynamic_frictions=1., restitutions=0.)
        GeomPrim(ground.paths).apply_physics_materials(material)
        plant = QuadrupedPlant(stage, {'robot': dict(kind='official_spot_physx',
            asset_root=str(args.asset_root.resolve()), velocity_feedback_mode=args.mode,
            max_linear_speed=.15, max_angular_speed=.3)}, result)
        SimulationManager.setup_simulation(dt=.002, device='cpu')
        SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
        app_utils.play()
        app.update()
        plant.initialize()
        registry = dict(root=plant.root_path, colliders=[dict(path=r['path'], type=r['shape']['type'])
            for r in plant._registry])
        envelope = dict(radius=.6078651750202781, offset=.3125, above=.55,
                        support_floor_z=0., support_penetration_m=.02)
        command = [0., 0.]
        subscription = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=True, order=0, on_update=lambda dt, context: plant.step(dt, *command))
        trajectory_stream = (result / 'trajectory.jsonl').open('w')
        policy_stream = (result / 'policy_joints_geometry.jsonl').open('w')
        rows, witnesses = [], []
        origin = SimulationManager.get_simulation_time()
        wall_start = time.monotonic()
        schedule = phases(args.short)
        certificate_failures = []
        for phase, frames, demand in schedule:
            command[:] = [demand, 0.]
            for index in range(frames):
                SimulationManager.step(steps=1, update_fabric=False)
                position, quaternion = [array(v).copy()[0] for v in plant.get_world_poses()]
                linear, angular = [array(v).copy()[0] for v in plant.get_velocities()]
                source_ns = round((SimulationManager.get_simulation_time() - origin) * 1e9)
                rotation = rotation_wxyz(quaternion)
                row = dict(phase=phase, source_ns=source_ns, command=list(command),
                    position=position.tolist(), quaternion_wxyz=quaternion.tolist(),
                    linear_velocity=linear.tolist(), angular_velocity=angular.tolist(),
                    body_vx=float((rotation.T @ linear)[0]), body_wz=float((rotation.T @ angular)[2]),
                    policy_input=plant._command.cpu().tolist(), integral=plant.velocity_feedback.integral.tolist())
                rows.append(row)
                trajectory_stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')
                if index % 10 == 0:
                    colliders = plant.collider_snapshot()
                    foot_bottom = {c['path'].split('/')[3].split('_')[0]:
                        float(np.asarray(c['world_matrix'])[2, 3] - c['shape']['radius'] *
                              np.linalg.norm(np.asarray(c['world_matrix'])[2, :3]))
                        for c in colliders if c['shape']['type'] == 'Sphere'}
                    witness = dict(phase=phase, source_ns=source_ns, foot_bottom=foot_bottom,
                        colliders=colliders, policy_joint_reading=plant.policy_joint_reading(source_ns))
                    witnesses.append(witness)
                    policy_stream.write(json.dumps(witness, separators=(',', ':'), allow_nan=False) + '\n')
                    # Spawn/settle occurs before the navigation certificate applies.
                    if source_ns >= 2_000_000_000:
                        try:
                            body_certificate(colliders, [*position.tolist(), *quaternion[1:].tolist(),
                                float(quaternion[0])], registry, envelope, source_ns)
                        except ValueError as error:
                            certificate_failures.append(dict(source_ns=source_ns, reason=str(error)))
                if not np.isfinite(np.r_[position, quaternion, linear, angular]).all() or position[2] < .15 or position[2] > .85:
                    raise RuntimeError('unstable_actual_PhysX')
            print('PHASE_COMPLETED', phase, source_ns / 1e9, position.tolist(), flush=True)
        trajectory_stream.close()
        policy_stream.close()
        summary = dict(schema=1, scope='isolated_real_PhysX_servo_component_no_navigation_or_hardware_acceptance',
            mode=args.mode, candidate_gain_calibrated=False, runtime_algorithm_override=False,
            source_sha256=hashes, short=args.short, physics_hz=500, policy_hz=50,
            observation_hz=50, total_physics_steps=len(rows), wall_s=time.monotonic() - wall_start,
            phases=summarize(rows, witnesses, schedule), certificate_failures=certificate_failures,
            metadata=plant.asset_metadata())
        summary['original_stops_all_passed'] = all(
            entry['original_stop_1s_completed_elapsed_s'] is not None and
            entry['original_stop_1s_completed_elapsed_s'] <= 3.
            for key, entry in summary['phases'].items() if key.startswith('stop'))
        summary['measured_platform_reach_bounds_passed'] = all(
            entry['max_full_linear_speed_mps'] <= .6 and entry['max_abs_body_yaw_radps'] <= .8
            for key, entry in summary['phases'].items() if key != 'park')
        summary['all_motion_phases_passed'] = all(
            entry['measured_speed_tracking_passed'] and entry['lasting_forward_progress_passed'] and entry['real_footflight_passed']
            for entry in summary['phases'].values() if entry['command_vx'] != 0.)
        (result / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        print('PROBE_COMPLETED', json.dumps({key: summary[key] for key in
            ('all_motion_phases_passed', 'original_stops_all_passed', 'measured_platform_reach_bounds_passed')}), flush=True)
        subscription = None
        app_utils.stop()
    except BaseException:
        # SimulationApp.close may end the interpreter before uncaught Python
        # exceptions render their traceback. Preserve a diagnostic first.
        traceback.print_exc()
        raise
    finally:
        if trajectory_stream and not trajectory_stream.closed:
            trajectory_stream.close()
        if policy_stream and not policy_stream.closed:
            policy_stream.close()
        app.close()


if __name__ == '__main__':
    main()
