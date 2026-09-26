# Central IMU isolated Faster-LIO experiment: calibration provenance

This is an offline experimental candidate, not a factory calibration or a validated production configuration. This note does not modify any configuration. No local evidence identifies `imu_link` with either CAD IMU model. The central sensor origin is assumed to coincide with the manufacturer localization base origin for this experiment; this assumption has no measured error bound.

## Frame and direction definitions

All vectors are columns; `p_target = R_target_source * p_source + t_target_source`. Translations are metres. `t_target_source` is the source origin expressed relative to the target origin in target coordinates.

- `L`: raw front LiDAR point frame.
- `F`: raw front LiDAR internal IMU vector axes, before the adapter's axis conversion.
- `N`: existing normalized input-cloud axes. The experiment retains the historical point-cloud transform and its zero translation. The existing pipeline assumes these axes match the normalized front IMU axes.
- `C`: central IMU raw vector axes, `imu_link`.
- `B`: manufacturer localization base axes/origin from the original named OTA calibration chain. Its equivalence to the CAD `BASE_LINK` origin is not established.

The existing front IMU axis conversion is:

```text
R_N_F = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
```

The fitted `R_c_from_f_raw` maps `F` into `C`. The separately saved `R_c_from_front_normalized = R_C_F * R_N_F.T` maps normalized front IMU vectors into `C`. These two matrices must not be interchanged.

## Rotation used in the isolated LIO experiment

Use `R_lio = R_C_N = gyro_rotation_candidate.json["R_c_from_front_normalized"]`, retaining the existing cloud normalization. This retains the existing assumption that the normalized point axes match the normalized front IMU axes. It does not independently calibrate the raw LiDAR-to-internal-IMU rotation.

At the time this note was written:

```text
R_C_N =
[[ 0.9995380594227922,  0.0298710739683668, -0.0056023838943480],
 [ 0.0298295236435884, -0.9995279108342023, -0.0073590069039794],
 [-0.0058195605091714,  0.0071884910372461, -0.9999572282413326]]
```

Faster-LIO's implementation applies `p_imu = offset_R_L_I * p_lidar + offset_T_L_I` in `/home/dndx/d1max_nav_ws/src/faster_lio/src/laser_mapping.cc:1229`, so the chosen `C <- N` direction matches the required extrinsic direction.

## Translation provenance and computation

The original named OTA `imu_front_to_base` matrix is recorded in:

`/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/diagnostics/localization_runtime_20260917.log:3`

```text
R_B_F_original =
[[-0.001604, -0.005109, -0.999986],
 [-1.000060,  0.012370,  0.001598],
 [ 0.001229,  1.000048, -0.005111]]
t_B_F_original = [0.361718, 0.004206, -0.004208]
```

The matching OTA `lidar_front_to_imu_front` appears in the bag snapshot:

`/home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38/snapshots/extrinsics_20260917.json:166`

```text
R_F_L_OTA =
[[-0.01235975, -0.99992224, -0.00166076],
 [-0.99991046,  0.001236809, -0.00510868],
 [ 0.00512883,  0.00159747, -0.99998557]]
t_F_L_OTA = [0.00425, 0.00418, -0.00446]
```

These two original rotations are individually nonorthogonal. Composing them in their original named directions, rather than using the misleading runtime inverse variable, yields:

```text
R_B_L_original = R_B_F_original * R_F_L_OTA
              approximately [[0,0,1], [0,1,0], [-1,0,0]]
t_B_L_original = R_B_F_original * t_F_L_OTA + t_B_F_original
               = [0.366149765, 0.000000325, 0.000000219]
```

The composition's rotation differs from the displayed proper permutation by about 5e-7 in Frobenius norm using log-rounded inputs. This cancellation does not validate the individual matrices or the physical calibration. The runtime log instead describes storing the inverse of `imu_front_to_base` and multiplying it with `lidar2imu`; that runtime matrix is not substituted here.

For this experiment, use the explicitly nominal, proper coarse raw-front-IMU-to-base axis convention:

```text
R_B_F_nominal = [[0,0,-1], [-1,0,0], [0,1,0]]
R_C_B = gyro_rotation_candidate.json["R_c_from_f_raw"] * R_B_F_nominal.T
```

Because the experiment assumes the central origin is `p_B_C = [0,0,0]`, and the retained normalized-cloud transform is a rotation about the raw front LiDAR origin:

```text
t_C_N = R_C_B * (t_B_L_original - p_B_C)
      = R_C_B * t_B_L_original
      = [0.0109369606370237, -0.3659769209691018, 0.0026318472047164]
```

This is the front point-cloud origin relative to the central IMU, expressed in central axes. No extra multiplication by `R_C_N` is needed after this calculation. If a nonzero cloud-normalization translation is later introduced, this expression must be revisited.

Critical raw/normalized distinction: using `R_c_from_front_normalized` with the displayed `R_B_F_nominal` is incorrect. That mismatch would produce approximately `[0.002051, 0.002694, 0.366134]`, placing the forward lever arm almost vertically. The rotation for LIO uses the normalized fit; the translation's base-axis bridge uses the raw fit.

## What the other evidence does and does not establish

- The 4 mm OTA LiDAR/IMU translation exactly matches the front/rear LiDAR DIFOP internal IMU offsets. The explicit keys are `lidar_front_to_imu_front` and `front_lidar_to_front_imu`; this is not a central-IMU lever arm.
- The raw bag was inspected read-only with SQLite `mode=ro` and `PRAGMA query_only=ON`, taking the first 150 `/tf` and first 150 `/tf_static` messages of each of its three splits (900 messages total). Only `odom -> base_link` and identity `map -> odom` appeared. No sampled transform connected `imu_link` to a sensor or body frame. This was a bounded sample, not a complete TF enumeration.
- Vendor URDF `/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/urdf_ws/src/max_description/urdf/max.urdf` gives `F_AIRY_LINK` at `[0.4043,0,-0.0377]` (line 204), `IMU_LUA300C_LINK` at `[0,0,0.00569]` (line 146), and `IMU_ICM42688_LINK` at `[0,0,0.0362]` (line 88), all with zero joint rpy. These imply CAD link lever arms `[0.4043,0,-0.04339]` and `[0.4043,0,-0.07390]`. Neither CAD IMU is proven to be the message's `imu_link`, and CAD link origins are not established as raw sensor measurement origins. These centimetre-scale differences illustrate uncertainty; they are not a strict bound on the chosen nominal origin. This experiment does not mix these CAD translations into the OTA translation.
- The gyro fit estimates rotation and an apparent temporal lag, not sensor origins. It cannot validate this translation. Static gravity constrains tilt but cannot fix yaw or a lever arm.
- Keeping historical cloud normalization retains its known difference from the conditional DIFOP-based normalization (about 0.578 degrees in the earlier audit). The new run is an isolated central-IMU experiment, not a complete factory extrinsic recalibration.
- The previous run `/home/dndx/d1max_nav_ws/maps/runs/20260917_190115_bag_faster_lio_sc_pgo_loop_fix_zenoh/config/run.py:99` replayed only `/front_lidar/imu`; its `dual_lidar.yaml:7` selected that same input. The presence of central IMU in copied bag metadata did not make it participate in that earlier run.

Use the resulting map as an experimental comparison. Its performance cannot by itself certify the nominal translation, infer a central sensor model, or isolate IMU hardware quality from the retained LiDAR calibration and time-alignment assumptions.
