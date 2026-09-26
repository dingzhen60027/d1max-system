# Central IMU / Faster-LIO mapping experiment

This directory is an isolated experiment, not a production profile switch.
It leaves the original bag, production mapping launch/configuration and middleware choice unchanged.

## Scope

- Bag: `/home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38`.
- Front + rear LiDAR clouds use the existing adapter and existing cloud extrinsics.
- Only `/imu_driver/imu_central` drives this Faster-LIO run; the Airy IMU adapter input is disabled.
- Central vectors remain in their recorded axes. Acceleration is converted from inferred g to SI.
- LIO owns the LiDAR-to-central `R,t` transform from `calibration.yaml`.
- Central header is shifted by −13 ms as a relative phase estimate; the bag itself is unmodified.
- Duplicate samples are preserved and audited, not silently interpolated or dropped.
- Local-only Zenoh, ROS domain 219; no robot SDK, motion commands, height lock or post-hoc flattening.
- Frontend-only by default; `--with-loop` adds the installed SC-PGO backend without changing production configuration.

## Calibration status

Read `calibration/calibration_notes.md` before reusing the configuration.
Rotation is estimated from this bag's simultaneous gyro streams with held-out validation.
Translation uses a nominal central origin at manufacturer base and the manufacturer front LiDAR installation chain.
The central sensor identity/origin is not proven, and the existing point-to-front-IMU calibration uncertainty remains.
These parameters are **experimental**, not a factory calibration certificate.

The equation is `p_central = R * p_normalized_cloud + t`.
Do not transpose `R` merely because the TF parent is the IMU: the same child-to-parent mapping is expected by both this static TF and LIO.
Do not subtract the gyro fitting intercept as an absolute bias: LIO initialization estimates its own gyro bias.

## Run

```bash
bash /home/dndx/d1max_nav_ws/experiments/central_imu_fasterlio_20260918/run.sh \
  --bag /home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38 \
  --rate 1 --exit-after-save
```

Each invocation creates a fresh `maps/runs/<timestamp>_central_imu_faster_lio_zenoh` directory.
RViz is enabled by default; `--no-rviz` disables it. `--exit-after-save` shuts down all owned children after saving.
Without that option the supervisor retains the RViz/router until the viewer closes or the supervisor is interrupted.
It refuses an occupied domain and never kills an unrelated user's process.

For frontend plus SC-PGO (the 2026-09-19 experiment):

```bash
bash /home/dndx/d1max_nav_ws/experiments/central_imu_fasterlio_20260918/run.sh \
  --bag /home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38 \
  --with-loop --loop-detection-frequency 15 --rate 1 --exit-after-save
```

Loop mode uses a `_central_imu_faster_lio_sc_pgo_zenoh` output suffix, a `central_imu`
body frame, `camera_init` odometry frame and `map` global frame. Its input odometry
and body cloud both come from this same frontend. No second IMU time shift is applied
by PGO. RViz shows the optimized map by default and disables the accumulated frontend
map so the two do not overlap. The current backend publishes its accumulated map at
0.2 Hz (every five seconds); that visualization rate is not the frontend odometry rate.

The optional 15 Hz loop-check override avoids skipping new keyframes between checks
(the detector only inspects the latest keyframe). It retains the source configuration's
0.5 m / 4 degree keyframe admission and all geometric acceptance thresholds. It is not
a promise of 15 accepted loops per second. The source configuration itself is unchanged.

After playback, the supervisor waits for stable externally observable PGO output,
then requests graceful shutdown to save its map. These checks are heuristic: the
installed binary has no queue-drained or final-save service. A failed stabilization
check is recorded as incomplete and partial outputs are retained.

## Outputs

- `scans.pcd` and timestamped PCD: accumulated frontend map, without loop optimization.
- `scans.pcd` is a symlink to that one timestamped frontend map, not a duplicate physical map.
- With loop mode, `sc_pgo/optimized_map.pcd`: the other physical map. Keyframe scans and body debug samples are not saved.
- With loop mode, `sc_pgo/loop_events.csv`, KITTI-format poses and `times.txt`: loop decisions and corresponding keyframe trajectories.
- `pgo_result.json` / `pgo_drain.json`: actual accepted count and bounded stabilization evidence. Saving an optimized map with zero accepted loops is not loop-closure success.
- `frontend_odometry.tum`: streamed frontend poses, saved continuously.
- `frontend_state.csv`: gravity, bias, pose, matching and covariance diagnostics.
- `central_imu_adapter.json`: actual received/published counts, repeated vectors and header gaps.
- `result.json`: completion/coverage, saved-map identity and cleanup information. `complete=true` means replay/save finished, **not** that accuracy was validated.
- `config/`: exact launch/configuration/code snapshots, calibration evidence, source metadata.
- `manifest.json`: input, frame/time assumptions, middleware, binary/configuration SHA-256.
- `legacy_logs_before/after`: capture legacy fixed-path logs; preexisting logs are restored after this experiment.

Observer subscriptions deliberately use best-effort receipt counts so they cannot hold back mapping. Their counts are not proof of algorithm consumption; use frontend state rows and adapter audit together.

## Checks

`test_central_imu_adapter.py` verifies SI scaling without the Airy axis rotation, nanosecond timestamp borrowing, missing orientation semantics, covariance handling and invalid-value rejection.
SO(3) orthogonality/determinant, gyro-fit frame composition, gravity consistency and TF quaternion equivalence were also checked before replay.

The raw vertical trajectory change is not by itself ground-truth drift. Without independent ground truth, evaluate it alongside the map geometry, known floor structure and initial/final location. Comparisons with older runs also inherit software-version and calibration differences.

For a completed loop run, generate inspectable comparisons with:

```bash
/usr/bin/python3 /home/dndx/d1max_nav_ws/experiments/central_imu_fasterlio_20260918/analyze_loop_result.py \
  --run /home/dndx/d1max_nav_ws/maps/runs/<completed-run-directory>
```

Read the per-run report for calibration limitations. A loop accepted into the graph
does not independently validate the correspondence, and a smaller endpoint gap does
not prove every corridor is level or that frontend drift has been eliminated.
