#!/usr/bin/env python3
"""Bounded real Spot physics probe; no ROS, UDP or navigation process.

Run with Isaac python.sh after downloading the pinned official asset cache.
Results are sampled actual physics evidence, not continuous collision proof or
physical-robot SDK acceptance. Default total duration is 18 source seconds.
"""
import sys,time,json,math,traceback,argparse
from pathlib import Path
import numpy as np
from isaacsim import SimulationApp
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--result-dir',type=Path,required=True)
parser.add_argument('--asset-root',type=Path)
parser.add_argument('--velocity-feedback',action=argparse.BooleanOptionalAction,default=True)
parser.add_argument('--linear-speed',type=float,default=.25)
parser.add_argument('--angular-speed',type=float,default=.4)
args=parser.parse_args()
if not math.isfinite(args.linear_speed) or abs(args.linear_speed)>.25 or not math.isfinite(args.angular_speed) or abs(args.angular_speed)>.4:
 parser.error('bounded probe requires linear <=.25 m/s and angular <=.4 rad/s')
sys.argv=[sys.argv[0]]
app=SimulationApp({'headless':True,'disable_viewport_updates':True,'renderer':'RaytracedLighting'})
try:
 import carb,omni.usd,omni.physics.core
 import isaacsim.core.experimental.utils.app as app_utils
 from isaacsim.core.experimental.objects import GroundPlane
 from isaacsim.core.experimental.prims import RigidPrim,GeomPrim
 from isaacsim.core.experimental.materials import RigidBodyMaterial
 from isaacsim.core.experimental.utils import backend
 from isaacsim.core.simulation_manager import SimulationManager
 from pxr import UsdGeom
 sys.path.insert(0,str(Path(__file__).resolve().parent))
 from quadruped import QuadrupedPlant,array,rotation_wxyz
 stage=omni.usd.get_context().get_stage()
 if stage is None:
  omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
 UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
 ground=GroundPlane('/World/GroundPlane',sizes=20.,templates=None)
 mat=RigidBodyMaterial('/World/Material',static_frictions=1.,dynamic_frictions=1.,restitutions=0.)
 GeomPrim(ground.paths).apply_physics_materials(mat)
 result=args.result_dir.expanduser().resolve()
 result.mkdir(parents=True,exist_ok=True)
 robot_config={'kind':'official_spot_physx','velocity_feedback':args.velocity_feedback}
 if args.asset_root:robot_config['asset_root']=str(args.asset_root.resolve())
 plant=QuadrupedPlant(stage,{'robot':robot_config},result)
 foot_paths=[plant.root_path+'/'+side+'_foot' for side in ('fl','fr','hl','hr')]
 contacts=RigidPrim(foot_paths,max_contact_count=256)
 contacts.set_enabled_contact_tracking([True],threshold=0.)
 SimulationManager.setup_simulation(dt=.002,device='cpu')
 SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
 print('PLANT_CREATED',plant.body_path,flush=True)
 app_utils.play();app.update()
 plant.initialize()
 print('PLANT_INITIALIZED',plant.asset_metadata()['collider_registry_sha256'],flush=True)
 print('JOINTORDER',plant.robot.dof_names,'DEFAULT',plant.controller.default_pos.tolist(),flush=True)
 command=[0.,0.]
 subscription=omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(pre_step=True,order=0,on_update=lambda dt,context: plant.step(dt,*command))
 trajectory=[]; trajectory_stream=(result/'trajectory.jsonl').open('w'); phases=[('settle',1000,0.,0.),('walk',2000,args.linear_speed,0.),('turn',1000,0.,args.angular_speed),('stop',5000,0.,0.)]
 start=time.monotonic();source_start=SimulationManager.get_simulation_time()
 for phase,n,vx,wz in phases:
  command[:]=[vx,wz]
  for index in range(n):
   SimulationManager.step(steps=1,update_fabric=False)
   pos,quat=[array(v).copy()[0] for v in plant.get_world_poses()]
   lin,ang=[array(v).copy()[0] for v in plant.get_velocities()]
   q=array(plant.get_dof_positions()).copy()[0];qd=array(plant.get_dof_velocities()).copy()[0]
   with backend.use_backend('tensor',raise_on_unsupported=True,raise_on_fallback=True):
    contact=array(contacts.get_net_contact_forces(dt=.002)).copy()
   row={'phase':phase,'sim_time':SimulationManager.get_simulation_time()-source_start,'position':pos.tolist(),'quaternion_wxyz':quat.tolist(),'linear_velocity':lin.tolist(),'angular_velocity':ang.tolist(),'joint_positions':q.tolist(),'joint_velocities':qd.tolist(),'foot_net_contact_forces':contact.tolist()}
   trajectory.append(row)
   trajectory_stream.write(json.dumps(row)+'\n')
   if index%200==0:
    snapshot=plant.collider_snapshot()
    print('STEP',phase,index,'Z',pos[2],'V',lin.tolist(),'CONTACTZ',contact[:,2].tolist(),flush=True)
   if not np.isfinite(np.r_[pos,quat,lin,ang,q,qd]).all() or pos[2]<.15 or pos[2]>.85:
    raise RuntimeError('unstable_actual_physics')
 trajectory_stream.close()
 summary={'scope':'sampled_real_PhysX_no_continuous_collision_proof','physical_sdk_acceptance':False,'physics_hz':500,'policy_hz':50,'physics_steps':len(trajectory),'wall_seconds':time.monotonic()-start,'sim_seconds':trajectory[-1]['sim_time'],'phases':{}}
 for phase,_,vx,wz in phases:
  rows=[r for r in trajectory if r['phase']==phase];p=np.array([r['position'] for r in rows]);v=np.array([r['linear_velocity'] for r in rows]);a=np.array([r['angular_velocity'] for r in rows]);qq=np.array([r['joint_positions'] for r in rows]);forces=np.array([r['foot_net_contact_forces'] for r in rows]);tilt=[math.acos(max(-1,min(1,rotation_wxyz(r['quaternion_wxyz'])[2,2]))) for r in rows]
  summary['phases'][phase]={'source_samples':len(rows),'command':[vx,wz],'start_position':p[0].tolist(),'end_position':p[-1].tolist(),'distance_xy_m':float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum()),'body_z_range_m':[float(p[:,2].min()),float(p[:,2].max())],'max_tilt_rad':max(tilt),'last1s_mean_linear_velocity':v[-500:].mean(axis=0).tolist(),'last1s_mean_angular_velocity':a[-500:].mean(axis=0).tolist(),'linear_speed_range_mps':[float(np.linalg.norm(v[:,:2],axis=1).min()),float(np.linalg.norm(v[:,:2],axis=1).max())],'angular_z_range_radps':[float(a[:,2].min()),float(a[:,2].max())],'joint_motion_range_rad':np.ptp(qq,axis=0).tolist(),'positive_ground_contact_samples_per_leg':(forces[:,:,2]>1.).sum(axis=0).tolist(),'peak_foot_force_z_N':float(forces[:,:,2].max())}
 summary['analytic_cylinders']=not bool(carb.settings.get_settings().get('/physics/collisionApproximateCylinders'))
 summary['metadata']=plant.asset_metadata(); summary['collider_snapshot']=plant.collider_snapshot()
 (result/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 stage.Flatten().Export(str(result/'spot_test.usda'))
 print('TEST_COMPLETE',json.dumps({k:v for k,v in summary.items() if k not in ('metadata','collider_snapshot')}),flush=True)
 subscription=None;app_utils.stop()
except Exception:
 if 'trajectory_stream' in globals():trajectory_stream.close()
 traceback.print_exc();raise
finally:
 app.close()
