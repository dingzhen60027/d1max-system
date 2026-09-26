#!/usr/bin/env python3
"""Read-only numerical audit of recorded central-IMU extrinsic composition.

No ROS imports, bag playback, robot connection, or configuration changes.
Prints JSON; caller can inspect stdout. Inputs are experiment artifacts.
"""
import ast
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

WS = Path('/home/dndx/d1max_nav_ws')
RUN = WS / 'maps/runs/20260919_125722_855277_central_imu_faster_lio_sc_pgo_zenoh/config'
CAL = WS / 'experiments/central_imu_fasterlio_20260918'
SNAP = WS / 'bags/slam_raw_20260917_171716_fe8f38/snapshots/extrinsics_20260917.json'


def angle(a, b):
    return float(np.degrees(Rotation.from_matrix(a @ b.T).magnitude()))


def main():
    cfg = yaml.safe_load((RUN / 'calibration.yaml').read_text())
    fit = json.loads((CAL / 'calibration/gyro_rotation_candidate.json').read_text())
    snap = json.loads(SNAP.read_text())
    tree = ast.parse((RUN / 'mapping.launch.py').read_text())
    # Execute ONLY the pure quaternion function; never import/execute launch code.
    pure = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'rotation_quaternion')
    namespace = {'math': math}
    exec(compile(ast.Module(body=[pure], type_ignores=[]), '<pure quaternion function>', 'exec'), namespace)
    funcs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'static_tf']
    front = next(n for n in funcs if ast.literal_eval(n.args[0]) == 'central_experiment_airy_to_ros')
    rear = next(n for n in funcs if ast.literal_eval(n.args[0]) == 'central_experiment_lidar_extrinsic')
    q_N_L = ast.literal_eval(front.args[4])
    R_N_L = Rotation.from_quat(q_N_L).as_matrix()
    R_L_R = Rotation.from_quat(ast.literal_eval(rear.args[4])).as_matrix()
    t_L_R = np.asarray(ast.literal_eval(rear.args[3]))
    A = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    R_C_F = np.asarray(fit['R_c_from_f_raw'])
    R_C_N = np.asarray(cfg['lio_extrinsic']['rotation']).reshape(3, 3)
    t_C_N = np.asarray(cfg['lio_extrinsic']['translation'])
    R_from_launch_quaternion = Rotation.from_quat(namespace['rotation_quaternion'](R_C_N.ravel())).as_matrix()
    R_F_L_implied = A.T @ R_N_L
    device = next(d for d in snap['factory_devices'] if d['sn_hex'] == '3009bede1530')
    R_F_L_device = Rotation.from_quat(device['quaternion_xyzw']).as_matrix()
    candidate = R_C_F @ R_F_L_device @ R_N_L.T
    # Named OTA chain and nominal origin used ONLY to check what the experiment
    # actually computed; not a validation of physical sensor origins.
    R_B_F = np.array([[-.001604,-.005109,-.999986],[-1.000060,.012370,.001598],[.001229,1.000048,-.005111]])
    t_B_F = np.array([.361718,.004206,-.004208])
    t_F_L = np.array([.00425,.00418,-.00446])
    R_B_F_nominal = np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
    t_B_L = R_B_F @ t_F_L + t_B_F
    R_C_B = R_C_F @ R_B_F_nominal.T
    t_recomputed = R_C_B @ t_B_L
    rng = np.random.default_rng(0)
    p_rear = rng.normal(size=(1000,3))
    # Column-transform equations, written as row-vector batches here.
    first_normalize_then_extrinsic = (p_rear @ R_L_R.T + t_L_R) @ R_N_L.T @ R_C_N.T + t_C_N
    single_composite = p_rear @ (R_C_N @ R_N_L @ R_L_R).T + (R_C_N @ R_N_L @ t_L_R + t_C_N)
    report = {
      'status': 'numerical/software direction audit only; not a physical calibration',
      'frames': {'L':'raw front LiDAR point frame','R':'raw rear LiDAR point frame',
                 'F':'raw front internal IMU vector axes','N':'retained normalized point axes, FRONT LiDAR origin',
                 'C':'raw central IMU axes, origin nominal/unknown','B':'nominal manufacturer base'},
      'convention':'p_target = R_target_source * p_source + t_target_source; column vectors',
      'checks': {
        'R_C_N_equals_R_C_F_A_transpose_maxabs': float(np.max(np.abs(R_C_N - R_C_F @ A.T))),
        'configured_rotation_det': float(np.linalg.det(R_C_N)),
        'configured_rotation_orthogonality_fro': float(np.linalg.norm(R_C_N.T @ R_C_N - np.eye(3))),
        'launch_xyzw_to_matrix_maxabs': float(np.max(np.abs(R_from_launch_quaternion - R_C_N))),
        'rear_composition_maxabs_m': float(np.max(np.abs(first_normalize_then_extrinsic-single_composite))),
        'nominal_lever_arm_recomputed_maxabs_m': float(np.max(np.abs(t_recomputed-t_C_N))),
        'historical_front_rotation_vs_conditional_current_device_deg': angle(R_F_L_implied,R_F_L_device),
        'conditional_DIFOP_candidate_vs_current_C_from_N_deg': angle(candidate,R_C_N),
      },
      'current_R_N_L':R_N_L.tolist(),
      'current_implied_R_F_L':R_F_L_implied.tolist(),
      'current_effective_R_C_L':(R_C_N@R_N_L).tolist(),
      'current_t_C_N_m':t_C_N.tolist(),
      'recomputed_nominal_t_C_N_m':t_recomputed.tolist(),
      'conditional_device_front_sn':device['sn_hex'],
      'conditional_device_R_F_L':R_F_L_device.tolist(),
      'diagnostic_only_R_C_N_if_device_convention_valid':candidate.tolist(),
      'conditional_DIFOP_assumptions':[
         'Device q maps raw front LiDAR coordinates into raw front internal IMU coordinates.',
         'Recorded cloud and IMU values have no unknown driver-side extra transform.',
         'Topic-to-device association matches captured device; recorded snapshot precedes bag.',
      ],
      'software_facts':[
        'dual_lidar_adapter lookupTransform target=N, source=raw cloud; applies rigid transform once.',
        'Central adapter scales accelerometer and shifts time, but never rotates vectors.',
        'Dual adapter front IMU subscription is moved to an unused topic in this run.',
        'Faster-LIO row-major MatFromArray is consistent with calibration list order.',
        'Faster-LIO uses p_C = offset_R_L_I*p_N + offset_T_L_I in deskew and world projection.',
        'Publishing a matching central-to-N static TF does not itself transform the LIO input cloud again.',
        'World gravity initialization rotates IMU state into world; it is not a duplicate extrinsic rotation.',
      ],
      'unvalidated_physical_assumptions':[
        'Historical cloud normalization has been treated as matching normalized front-IMU axes.',
        'Gyro fit calibrates raw front IMU to central IMU, not raw LiDAR to central IMU directly.',
        'Central IMU physical measurement origin is approximated at manufacturer base origin.',
        'Translation cannot change a static plane normal and hence cannot explain the 0.8 degree static direction discrepancy alone.',
        'Static ground direction cannot validate yaw or translation; time shift is signal alignment not proven hardware clock offset.',
      ],
      'input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [RUN/'calibration.yaml',RUN/'mapping.launch.py',CAL/'calibration/gyro_rotation_candidate.json',SNAP]},
    }
    assert report['checks']['R_C_N_equals_R_C_F_A_transpose_maxabs'] < 1e-12
    assert report['checks']['launch_xyzw_to_matrix_maxabs'] < 1e-12
    # Calibration used the stated nanometre-rounded nominal B<-L vector.
    assert report['checks']['nominal_lever_arm_recomputed_maxabs_m'] < 1e-9
    assert report['checks']['rear_composition_maxabs_m'] < 1e-12
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
