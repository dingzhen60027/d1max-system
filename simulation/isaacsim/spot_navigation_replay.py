#!/usr/bin/env python3
"""Independent Spot command-event replay; never navigation authorization.

The saved navigation trajectory is 50 Hz plus native capture witnesses, not a
500 Hz command recorder. Its exact source/command events are zero-order held
on a real 500 Hz plant. This cannot reconstruct unrecorded original ticks.
Only the alternative resets official action memory at an existing inference
boundary after a measured continuous stall. Joint state and servo are intact.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
import traceback

DT_NS = 2_000_000
DECIMATION = 10


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError('invalid_replay_vector')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError('invalid_replay_vector')
    return tuple(float(v) for v in value)


class CommandSchedule:
    def __init__(self, events, duration_ns):
        if type(duration_ns) is not int or duration_ns <= 0 or duration_ns % DT_NS:
            raise ValueError('replay_duration_not_on_physics_grid')
        self.duration_ns = duration_ns
        self.events = []
        previous = -1
        for event in events:
            stamp = event['source_ns']
            if type(stamp) is not int or stamp < 0 or stamp > duration_ns or stamp <= previous or stamp % DT_NS:
                raise ValueError('invalid_replay_source_order')
            command = finite_vector(event['command'], 2)
            if abs(command[0]) > .15 or abs(command[1]) > .3:
                raise ValueError('replay_command_outside_original_limits')
            if previous >= 0 and stamp - previous > 20_000_000:
                raise ValueError('unrecorded_replay_command_gap')
            self.events.append(dict(event, command=list(command)))
            previous = stamp
        if not self.events or self.events[0]['source_ns'] > 20_000_000 or self.events[0]['command'] != [0., 0.]:
            raise ValueError('replay_requires_original_initial_zero')
        if duration_ns - previous > 20_000_000:
            raise ValueError('replay_source_does_not_cover_duration')
        self.stamps = [e['source_ns'] for e in self.events]

    def at(self, source_ns):
        if type(source_ns) is not int or source_ns < 0 or source_ns > self.duration_ns:
            raise ValueError('replay_source_outside_frozen_domain')
        index = bisect.bisect_right(self.stamps, source_ns) - 1
        return [0., 0.] if index < 0 else self.events[index]['command'].copy()


def freeze_commands(trajectory, duration_ns):
    events = []
    with Path(trajectory).open() as stream:
        for line_number, line in enumerate(stream, 1):
            record = json.loads(line)
            stamp = record['sim_time_ns']
            if stamp > duration_ns:
                break
            events.append(dict(source_ns=stamp, command=record['command'], original_line=line_number))
    return CommandSchedule(events, duration_ns)


class StallResetGate:
    """Full measured stop window plus positive demand, once per episode."""
    def __init__(self):
        self.previous_ns = None
        self.positive = False
        self.episode = 0
        self.episode_start_ns = None
        self.stationary_start_ns = None
        self.consumed = False

    def observe(self, source_ns, command, linear, angular, policy_counter):
        command = finite_vector(command, 2)
        linear, angular = finite_vector(linear, 3), finite_vector(angular, 3)
        if type(source_ns) is not int or source_ns < 0 or type(policy_counter) is not int or policy_counter < 0:
            raise ValueError('invalid_stall_source')
        if abs(command[0]) > .15 or abs(command[1]) > .3:
            raise ValueError('replay_command_outside_original_limits')
        if self.previous_ns is not None:
            if source_ns <= self.previous_ns:
                raise ValueError('stall_source_rollback')
            # The rounded native simulation double may differ by one ns.
            if source_ns - self.previous_ns > DT_NS + 1:
                self.stationary_start_ns = None
        self.previous_ns = source_ns
        if command[0] <= 0.:
            self.positive = False
            self.stationary_start_ns = None
            self.episode_start_ns = None
            self.consumed = False
            return None
        if not self.positive:
            self.positive = True
            self.episode += 1
            self.episode_start_ns = source_ns
            self.consumed = False
        good = math.hypot(*linear) <= .03 and math.hypot(*angular) <= .05
        if not good:
            self.stationary_start_ns = None
            return None
        if self.stationary_start_ns is None:
            self.stationary_start_ns = source_ns
        if self.consumed or source_ns - self.stationary_start_ns < 1_000_000_000 or policy_counter % DECIMATION:
            return None
        self.consumed = True
        return dict(source_ns=source_ns, episode=self.episode,
            episode_start_ns=self.episode_start_ns, stationary_start_ns=self.stationary_start_ns,
            continuous_static_duration_ns=source_ns-self.stationary_start_ns,
            command=list(command), full_linear_norm=math.hypot(*linear), full_angular_norm=math.hypot(*angular),
            policy_counter=policy_counter)


def stop_completion(rows, zero_source_ns):
    if type(zero_source_ns) is not int or zero_source_ns < 0:
        raise ValueError('invalid_zero_command_source')
    start = None
    previous = None
    for row in rows:
        source = row['source_ns']
        if type(source) is not int or source < zero_source_ns or (previous is not None and source <= previous):
            raise ValueError('invalid_stop_source_order')
        if previous is not None and source-previous > DT_NS+1:
            start = None
        previous = source
        good = math.hypot(*row['linear_velocity']) <= .03 and math.hypot(*row['angular_velocity']) <= .05
        if not good:
            start = None
        elif start is None:
            start = row['source_ns']
        if start is not None and row['source_ns'] - start >= 1_000_000_000:
            return (row['source_ns'] - zero_source_ns) / 1e9
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-trajectory', type=Path, required=True)
    parser.add_argument('--result-dir', type=Path, required=True)
    parser.add_argument('--asset-root', type=Path, required=True)
    parser.add_argument('--variant', choices=('baseline', 'memory_reset'), required=True)
    parser.add_argument('--duration-s', type=float, default=42.)
    args = parser.parse_args()
    duration_ns = round(args.duration_s * 1e9)
    schedule = freeze_commands(args.source_trajectory, duration_ns)
    result = args.result_dir.expanduser().resolve()
    result.mkdir(parents=True, exist_ok=False)
    snapshot = result/'source_snapshot'; snapshot.mkdir()
    for name in ('quadruped.py', 'runtime_geometry.py', 'dynamic_collision.py', 'world_builder.py',
                 'policy_history.py', 'stall_recovery.py', 'spot_navigation_replay.py'):
        shutil.copyfile(Path(__file__).resolve().parent/name, snapshot/name)
    sys.path.insert(0, str(snapshot))
    hashes = {p.name:sha256(p) for p in snapshot.iterdir()}
    frozen = result/'command_events.json'
    frozen.write_text(json.dumps(dict(schema=1, duration_ns=duration_ns,
        source_trajectory=str(args.source_trajectory.resolve()), source_sha256=sha256(args.source_trajectory),
        semantics='exact saved source/command events; causal zero-order hold; original unrecorded physics ticks unknown',
        events=schedule.events), indent=2, allow_nan=False)+'\n')
    manifest = dict(schema=1, kind='official_spot_saved_navigation_command_component_replay',
        variant=args.variant, formal_navigation_pass=False, physical_acceptance=False,
        source_files=hashes, command_events_sha256=sha256(frozen),
        duration_ns=duration_ns, bootstrap_ns=2_000_000_000, added_zero_tail_ns=0,
        initial_position=[-8., -5., .8], initial_yaw=0., floor_friction=[1.,1.], restitution=0.,
        highlevel_command_limits=[.15,.3], reference_full_xyz_domain_mps=.50,
        physical_reachable_limits=[.6,.8], physics_hz=500, policy_hz=50,
        reset_rule='once per continuous positive-forward episode, measured full linear <=.03 and angular <=.05 for >=1s, next existing inference boundary; previous/current action only',
        component_scene='crop to the same flat-floor contact model and official articulated robot; no navigation, lidar, actors or proof leases')
    (result/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('REPLAY_PROCESS',json.dumps(dict(pid=os.getpid(),result_dir=str(result),**manifest)),flush=True)
    sys.argv=[sys.argv[0]]
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'disable_viewport_updates':True,'renderer':'RaytracedLighting'})
    streams=[];subscription=None
    try:
        import carb
        import omni.usd
        import omni.physics.core
        import omni.physx.bindings._physx as physx_bindings
        from pxr import UsdGeom
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.experimental.objects import GroundPlane
        from isaacsim.core.experimental.prims import GeomPrim
        from isaacsim.core.experimental.materials import RigidBodyMaterial
        from isaacsim.core.simulation_manager import SimulationManager
        from quadruped import QuadrupedPlant,array,rotation_wxyz
        from runtime_geometry import body_certificate
        import numpy as np
        stage=omni.usd.get_context().get_stage()
        if stage is None:
            omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        ground=GroundPlane('/World/GroundPlane',sizes=100.,templates=None)
        material=RigidBodyMaterial('/World/PhysicsMaterials/CampusFloor',static_frictions=1.,dynamic_frictions=1.,restitutions=0.)
        GeomPrim(ground.paths).apply_physics_materials(material)
        plant=QuadrupedPlant(stage,{'robot':dict(kind='official_spot_physx',asset_root=str(args.asset_root.resolve()),
            velocity_feedback_mode='spot_monotone_measured_v2',initial_position=manifest['initial_position'],initial_yaw=0.,
            max_linear_speed=.15,max_angular_speed=.3)},result)
        SimulationManager.setup_simulation(dt=.002,device='cpu')
        SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
        app_utils.play();app.update();plant.initialize()
        state=dict(command=[0.,0.],replay=False)
        gate=StallResetGate();events=[];rows=[];witnesses=[];failures=[]
        for name in ('trajectory.jsonl','policy_joints_geometry.jsonl','memory_events.jsonl'):
            stream=(result/name).open('w');streams.append(stream)
        trajectory_stream,geometry_stream,event_stream=streams
        def callback(dt,context):
            if state['replay']:
                pos,quat=[array(v)[0].tolist() for v in plant.get_world_poses()]
                linear,angular=[array(v)[0].tolist() for v in plant.get_velocities()]
                native_time=float(SimulationManager.get_simulation_time())
                source_ns=round((native_time-origin)*1e9)
                counter=int(plant.controller._policy_counter)
                trigger=gate.observe(source_ns,state['command'],linear,angular,counter)
                if trigger:
                    before=dict(previous_action=plant.controller._previous_action.cpu().tolist(),
                        current_action=plant.controller._current_action.cpu().tolist(),
                        policy_input=plant._command.cpu().tolist(),integral=plant.velocity_feedback.integral.tolist(),
                        filtered=plant.velocity_feedback.filtered.tolist(),position=pos,quaternion_wxyz=quat,
                        linear_velocity=linear,angular_velocity=angular,native_simulation_time_s=native_time,
                        plant_physics_steps=plant._steps,policy_counter=counter)
                    if args.variant=='memory_reset':
                        zero=plant._torch.zeros(12,dtype=plant.controller._current_action.dtype,device=str(plant.robot._device))
                        plant.controller._previous_action=zero.clone();plant.controller._current_action=zero.clone()
                    assert int(plant.controller._policy_counter)==counter
                    event=dict(trigger,applied=args.variant=='memory_reset',before=before,
                        after_previous_action=plant.controller._previous_action.cpu().tolist(),
                        after_current_action=plant.controller._current_action.cpu().tolist(),
                        after_policy_counter=int(plant.controller._policy_counter))
                    events.append(event);event_stream.write(json.dumps(event,allow_nan=False)+'\n');event_stream.flush()
                    print('MEMORY_WITNESS',json.dumps(event,allow_nan=False),flush=True)
            plant.step(dt,*state['command'])
        subscription=omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=True,order=3,on_update=callback)
        for _ in range(1000):SimulationManager.step(steps=1,update_fabric=False)
        carb.settings.get_settings().set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD,False)
        origin=float(SimulationManager.get_simulation_time());state['replay']=True
        registry=dict(root=plant.root_path,colliders=[dict(path=r['path'],type=r['shape']['type']) for r in plant._registry])
        envelope=dict(radius=.6078651750202781,offset=.3125,above=.589,support_floor_z=0.,support_penetration_m=.02)
        start_pos,start_quat=[array(v)[0].tolist() for v in plant.get_world_poses()]
        wall=time.monotonic();total_frames=duration_ns//DT_NS
        last_nonzero=next((i for i in range(len(schedule.events)-1,-1,-1)
            if schedule.events[i]['command']!=[0.,0.]),-1)
        terminal_zero_ns=(schedule.events[last_nonzero+1]['source_ns']
            if last_nonzero+1<len(schedule.events) else None)
        for index in range(total_frames):
            schedule_ns=index*DT_NS
            state['command']=schedule.at(schedule_ns)
            SimulationManager.step(steps=1,update_fabric=False)
            pos,quat=[array(v)[0].tolist() for v in plant.get_world_poses()]
            linear,angular=[array(v)[0].tolist() for v in plant.get_velocities()]
            native_time=float(SimulationManager.get_simulation_time());source_ns=round((native_time-origin)*1e9)
            rotation=rotation_wxyz(quat);tilt=math.acos(max(-1.,min(1.,float(rotation[2,2]))))
            row=dict(source_ns=source_ns,schedule_begin_ns=schedule_ns,
                native_simulation_time_s=native_time,
                phase='original_terminal_zero' if terminal_zero_ns is not None and schedule_ns>=terminal_zero_ns else 'replay',
                command=state['command'].copy(),position=pos,quaternion_wxyz=quat,linear_velocity=linear,angular_velocity=angular,
                body_vx=float((rotation.T@np.asarray(linear))[0]),body_wz=float((rotation.T@np.asarray(angular))[2]),
                body_tilt_rad=tilt,policy_input=plant._command.cpu().tolist(),integral=plant.velocity_feedback.integral.tolist(),
                policy_counter=int(plant.controller._policy_counter),plant_physics_steps=plant._steps)
            rows.append(row);trajectory_stream.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')
            if index%10==0:
                colliders=plant.collider_snapshot()
                feet={c['path'].split('/')[3].split('_')[0]:float(np.asarray(c['world_matrix'])[2,3]-c['shape']['radius']*np.linalg.norm(np.asarray(c['world_matrix'])[2,:3]))
                    for c in colliders if c['shape']['type']=='Sphere'}
                witness=dict(source_ns=source_ns,phase=row['phase'],command=row['command'],foot_bottom=feet,
                    colliders=colliders,policy_joint_reading=plant.policy_joint_reading(source_ns))
                witnesses.append(witness);geometry_stream.write(json.dumps(witness,separators=(',',':'),allow_nan=False)+'\n')
                try:body_certificate(colliders,[*pos,*quat[1:],quat[0]],registry,envelope,source_ns)
                except ValueError as error:failures.append(dict(source_ns=source_ns,reason=str(error)))
            if not np.isfinite([*pos,*quat,*linear,*angular]).all() or not .15<=pos[2]<=.85:
                raise RuntimeError('unstable_actual_PhysX')
            if (index+1)%5000==0:print('REPLAY_PROGRESS',source_ns/1e9,pos,flush=True)
        for stream in streams:stream.flush()
        drive=[r for r in rows if r['phase']=='replay'];tail=[r for r in rows if r['phase']=='original_terminal_zero']
        summary=dict(manifest,initial_actual_position=start_pos,initial_actual_quaternion_wxyz=start_quat,
            final_position=rows[-1]['position'],net_displacement_xyz_m=(np.asarray(rows[-1]['position'])-start_pos).tolist(),
            replay_displacement_xyz_m=(np.asarray(drive[-1]['position'])-start_pos).tolist(),
            cumulative_xy_m=float(np.linalg.norm(np.diff(np.asarray([r['position'][:2] for r in rows]),axis=0),axis=1).sum()),
            actual_full_linear_speed_max_mps=max(math.hypot(*r['linear_velocity']) for r in rows),
            actual_full_angular_speed_max_radps=max(math.hypot(*r['angular_velocity']) for r in rows),
            actual_body_tilt_max_rad=max(r['body_tilt_rad'] for r in rows),
            actual_full_xyz_within_reference_domain=all(math.hypot(*r['linear_velocity'])<=.50 for r in rows),
            actual_within_measured_reachable_domain=all(math.hypot(*r['linear_velocity'])<=.6 and abs(r['body_wz'])<=.8 for r in rows),
            actual_footflight_samples_50hz={side:sum(w['foot_bottom'][side]>.01 and w['command'][0]>0 for w in witnesses) for side in ('fl','fr','hl','hr')},
            memory_reset_count=sum(e['applied'] for e in events),memory_eligibility_count=len(events),
            original_terminal_zero_begin_ns=terminal_zero_ns,
            final_true_zero_original_stop_completed_s=stop_completion(tail,terminal_zero_ns) if tail else None,
            original_stop_limits=dict(full_linear=.03,full_angular=.05,continuous_s=1.,completion_deadline_s=3.),
            body_certificate_failures=failures,total_physics_frames=len(rows),wall_s=time.monotonic()-wall,
            command_events_original_sha256=sha256(frozen),metadata=plant.asset_metadata())
        summary['original_stop_passed']=summary['final_true_zero_original_stop_completed_s'] is not None and summary['final_true_zero_original_stop_completed_s']<=3.
        summary['windows']={}
        for lower,upper in [(0,10),(10,20),(20,30),(30,42)]:
            selected=[r for r in rows if lower*1e9<r['source_ns']<=upper*1e9]
            if selected:summary['windows'][f'{lower}_{upper}']=dict(samples=len(selected),
                actual_net_xy_m=float(np.linalg.norm(np.asarray(selected[-1]['position'])[:2]-np.asarray(selected[0]['position'])[:2])),
                mean_body_vx_mps=float(np.mean([r['body_vx'] for r in selected])),
                max_full_xyz_mps=max(math.hypot(*r['linear_velocity']) for r in selected),
                nonzero_commands=sum(r['command'][0]>0 for r in selected),
                policy_vx_min=min(r['policy_input'][0] for r in selected),policy_vx_max=max(r['policy_input'][0] for r in selected))
        (result/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        print('REPLAY_COMPLETED',json.dumps({k:summary[k] for k in ['variant','net_displacement_xyz_m','memory_reset_count','actual_footflight_samples_50hz','actual_full_linear_speed_max_mps','actual_body_tilt_max_rad','original_stop_passed']}),flush=True)
        subscription=None;app_utils.stop()
    except BaseException:
        traceback.print_exc();raise
    finally:
        for stream in streams:
            if not stream.closed:stream.close()
        app.close()


if __name__=='__main__':main()
