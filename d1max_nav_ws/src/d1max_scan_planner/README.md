# D1 Max SCAN-Planner integration

This package connects the D1 Max LIO output to SCAN-Planner's rolling 3D local
map and B-spline planner. It intentionally launches no controller and publishes
no `/cmd_vel` command.

## Robot parameters and their sources

The live Web/RViz entry reads `config/d1max_robot.yaml`, snapshots it with the
session, checks the configured speed/body-height assumptions against it, and
applies its robot collision parameters to the native SCAN parameter file.
Algorithm weights stay in `d1max_scan_planner.yaml`. The older independent
launch files still use that native YAML directly.

The supplied **D1 Max product specification v2.0 (2026-02-24), page 1** lists a
standing size of **0.930 x 0.480 x 0.585 m**, prone size **0.930 x 0.630 x 0.200 m**,
25 cm continuous stairs and 45 degree maximum slope. These are laboratory
capabilities, not a guarantee that every staircase/obstacle is traversable.
The SDK `sdk_client_api_cn.md` SetSpeed section lists slow-mode forward/back
1.0 m/s, lateral 0.5 m/s and yaw 1.5 rad/s, while `sdk_type_cn.md` has a conflicting
SpeedLevel table. That discrepancy is recorded, not silently resolved by
changing the controller. `Move` uses normalized inputs, not SI velocity.

**Not official calibrated values:** the existing 0.55 m body-reference height,
0.29 m cylinder radius, +/-0.20 m cylinder centers, vertical dilation and
0.35 m/s² acceleration remain explicit engineering assumptions. The live
profile derives radius as documented width / 2 + 0.05 m lateral margin and
center distance as documented length / 2 + 0.025 m longitudinal margin - radius.
These margins are engineering choices, not official measured values. The native
two-circle model spans 0.98 x 0.58 m, but this is not proof that it contains the
rectangle's corners, moving legs, payload or prone posture. The vertical
dilation directions must not be confused with robot height. Body-envelope and
height validation remain false. In particular, do not substitute the 0.585 m
total standing height for a calibrated `base_link` height.

Preview stays at 0.30 m/s with **no motion output**. The user's 1.5 m/s project
ceiling is separate from the product's 8 m/s laboratory maximum. No product
capability or profile field bypasses collision, ground-support or motion gates.

## PCT + SCAN single-floor session (2026-09-22)

`pct_scan.launch.py` is a separate, strict mode-3 integration. The old standalone
LIO/goal launch remains available; do not use its untagged trajectory as authority
for the new navigation controller.

```bash
ros2 launch d1max_scan_planner pct_scan.launch.py session_id:=<session-id>
```

The session manager must supply an isolated namespace and the same session ID
to PCT, SCAN and the trajectory tracker. No robot session, router, controller or
simulator is started by this launch. Keep `rmw_zenoh_cpp`; the caller owns the
transport and process lifetime.

### Input and output contract

- World frame: `d1max_loc_map` (or explicit `frame_id` override). No implicit TF
  lookup and no relabeling of unrelated coordinates.
- `body_pose`: `nav_msgs/Odometry`, body origin in the world frame; standard
  child/body-frame linear twist is rotated into world coordinates by SCAN.
  Default `/d1max/localization/odometry/global`; fresh input required within 0.5 s.
- `sensor_pose`: independently remappable odometry of the ray origin in the
  same world frame. It is **not** necessarily the body origin. Default
  `/d1max/pct_scan/sensor_pose_map`. Sensor and cloud timestamps must differ by
  no more than 0.25 s in legacy mode. Strict world-cloud mode pairs the original
  stamps exactly, using bounded queues, rather than whichever pose arrived
  most recently. No Go2 hard-coded extrinsics are applied.
- `cloud`: finite XYZ `sensor_msgs/PointCloud2`, already in the world frame,
  normally the current dual-lidar scan. Default `/d1max/pct_scan/cloud_map`.
  Feeding a stored local PCD crop is useful for **offline static-map testing**,
  but does not validate live visibility, sensor timing or dynamic avoidance.
- Sensor/body/cloud subscriptions use `SensorDataQoS`; occupancy integrates at
  20 Hz, FSM runs at up to 100 Hz and checks trajectory collision at 20 Hz.
  These timer rates do **not** promise a 100 Hz optimization or state estimate.
  The strict launch requests a real local replan at least every 1 s while fresh
  odometry/clouds are available, including when the execution clock is frozen
  for turning. It never republishes an old spline as a new optimization result;
  the legacy launch keeps this periodic replan disabled by default.
- Relative `typed_initial_path`: `d1max_planning_interfaces/ReferencePath`,
  reliable, depth 1. The path contains **ground/surface** XYZ in the world frame.
  Exactly one `reference_z_offset` is added (default 0.55 m) to obtain body XYZ.
  If the upstream path already contains body height, explicitly set this offset
  to 0.0. First lifted point must be within 1 m of measured body position.
- The configured session must match; generations must strictly increase.
  Duplicate/old/wrong-session messages cannot replace a route. A fresh generation
  with an empty path cancels it. Invalid new paths also revoke the previous route
  instead of silently continuing it.
- Relative `planning/tagged_bspline`: reliable
  `d1max_planning_interfaces/TaggedBspline`, with session/generation/frame captured
  **inside SCAN** from the active reference; its trajectory is the real SCAN
  optimizer output. Untagged `planning/bspline` remains debug-only.
- Relative `planning/go2_execution_frozen`: `std_msgs/Bool`, from the tracker;
  freezing only pauses the internal trajectory clock and is not an SDK stop.
- Planning starts at 0.30 m/s; this launch does not authorize motion.

### Vendor fixes required for this interface

`fsm.reference_path_guidance=true` enables the PCT coupling. The configurable
local D1 extra reference weight currently defaults to `optimization.lambda_reference=20.0`;
zero retains the original rebound objective. This is not an upstream recommended
weight, and route guidance does not require it (legacy defaults false/0).
The global route remains the original piecewise-linear PCT curve in arc-length
coordinates; SCAN does not refit it with an overshooting global minimum-snap
polynomial. Local targets and seed points use the same monotone-progress route
slice. A time-parameterized seed accelerates/decelerates within the configured
limits. Before optimization, obstructed portions of the selected reference are
replaced by native A* detours in the current inflated occupancy map. The
correspondence cost uses this collision-checked seed, not the obstructed
original line. Search failure terminates the attempt instead of feeding an
invalid seed to LBFGS. Collision and feasibility costs remain enabled; the
final external ground-support/whole-curve safety guard is still mandatory.
Reference-mode Z follows the PCT surface and is not replaced by a linear ramp.
These are soft reference costs, **not** permission to cross unknown/blocked cells.

The final native curve is now checked over its whole duration, including the
last third and endpoint. Guided failures enter bounded cooldown and wait for
changed occupancy or measured body position, instead of repeating the same
seed at 100 Hz. Distinct failure phases identify search, optimization, final
collision and dynamics; a hover/emergency spline is not an accepted route.
Only a tagged spline paired with accepted diagnostics of the same plan ID is
green in RViz. Failed attempts have a separate, short-lived diagnostic layer.
These changes have not been validated by a new robot run; no motion was enabled.

For a numerically stationary initial velocity/acceleration (both norms <= 1e-6),
guided mode preserves the successful rebound geometry and uniformly stretches
knot times if necessary, rather than using an XYZ-changing refinement just to
slow down. Derivative-control convex-hull bounds enforce `manager.max_vel` /
`manager.max_acc`; the complete spatial curve is still collision checked.
For a moving initial state, its boundary-preserving refinement remains required.
Uniform stretching of measured nonzero initial derivatives is rejected, and the
final initial velocity/acceleration must agree with the measured boundary within
1e-3 numerical tolerance; otherwise `failed_dynamics` and no candidate output.
Static synthetic detours are not moving-handoff/controller acceptance. New
generations reset their own bounded failure budget. No execution freeze or stop
is overridden by these changes.

Reference mode now keeps the full PCT reference and progress during local
replans. Previously `planFromCurrentTraj()` replaced it with a direct polynomial
to the final goal, losing intermediate corridor turns. Duplicate first points
are removed to avoid a zero-duration minimum-snap segment; finite points and
frames are validated. RViz markers now use `grid_map.frame_id`, not hard-coded
`map`/`world`. These are interface/correctness fixes, not a replacement planner.

Unknown space in the upstream rolling occupancy map is **not** by itself a
ground-support guarantee. The integrating tracker must additionally check the
PCT map's supported corridor, footprint, fresh scan/state and input generation.
Do not deploy on hardware merely because a B-spline was produced.

## Upstream integration comparison (2026-09-24)

This comparison is based on the authors' source trees at pinned commits, not
on demonstration videos or third-party tutorials. It distinguishes an external
path interface, a working simulation integration, and a hardware safety case.

### Official SCAN-Planner and TravExplorer

Official SCAN-Planner mode 3 consumes an external `nav_msgs/Path` on
`/initial_path`; it does **not** include PCT. At commit
`348e8a590a50a5a6bbab8d8c6dcfd171f009be26`, `pathCallback()` adds `body_height_`
to the supplied points, thins the waypoints and builds its global reference.
Its local target is selected along that reference. Empty paths are ignored in
that upstream callback, so cancellation/session ownership must not be inferred
from the upstream interface. See the [official mode-3 implementation](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp#L357).

The official launch distinguishes body odometry, sensor odometry and lidar
clouds, and also has separate simulation/open-loop modes. Its closed-loop
controller is not part of this project's no-motion launch. Real-robot support
in the upstream project does not validate D1 Max extrinsics, footprint or
tracking gains. See [official launch wiring](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/launch/run.launch).

TravExplorer credits SCAN as its local planner and Elevator-LIO for localization,
but its public repository at `eab0765a22ba6cfef93c48b0e0edeaa1833887b5` still
announces a future code release. Therefore its complete cross-floor planner,
failure recovery and mode-3 connection cannot be audited or copied from that
snapshot. Its reported real-world experiments are the authors' results, not
validation of this integration. See the [pinned TravExplorer README](https://github.com/wuyi2121/TravExplorer/blob/eab0765a22ba6cfef93c48b0e0edeaa1833887b5/README.md).

### Public PCT + SCAN example: legbot_3D_Nav

`Robot-Nav/legbot_3D_Nav` at
`f60606a4903bad58fca813f76072f9940a284d1d` contains an actual integration:
PCT `/pct_path` → frame/height adapter → SCAN external reference → an A1
trajectory-tracking adapter. The documented quick-start uses Gazebo ground-truth
odometry; the repository describes real-robot validation as ongoing. This is
useful integration evidence, not a demonstrated D1 Max localization-to-motion
pipeline. See [repository scope](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/README.md)
and [planner wiring](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/launch/local_planners.launch).

The useful parts are concrete:

- Its default piecewise-linear reference preserves corridor corners and stairs;
  smoothing is delegated to the local trajectory. This agrees with our discrete
  arc-length reference, rather than refitting the whole route with a polynomial.
  See [reference construction](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/planner_manager.cpp#L356).
- Target progress considers height as well as horizontal distance and does not
  move backward; completion checks measured XYZ, not just elapsed spline time.
  Its forward search can still cover the remaining route, so our bounded
  projection window must remain for overlapping floors. See [target/progress implementation](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp).
- Its tracker combines trajectory velocity with position feedback, converts
  world velocity to body coordinates, pauses for large heading errors and checks
  odometry receipt age. These illustrate tracking responsibilities; they are not
  a substitute for session, source-time, trajectory-expiry and SDK safety gates.
  See [A1 tracker](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/a1_cmd_adapter.cpp).

Important differences must **not** be copied blindly:

- The frame adapter uses latest TF, and the SCAN fork can vertically shift a
  whole path to the first odometry height. Neither establishes correct map/LIO
  registration. Our explicit original-map transform, height semantics and exact
  cloud/pose timestamps remain authoritative. The example's own mapping guide
  says a static `map→odom` transform is not localization. See [adapter](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/src/reference_path_transform.cpp)
  and [mapping/localization caveat](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/docs/FAST_LIO_MAPPING_CN.md#L178).
- Its Gazebo setup feeds the same odometry as body and sensor pose. Its A1
  collision dimensions, stair minimum speed and control gains are platform/demo
  settings, not D1 Max calibration or recommended hardware defaults. See
  [simulation SCAN parameters](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/launch/scan.launch).
- An unoccupied voxel is not proof of a supporting floor. The inspected rolling
  occupancy query does not itself certify terrain support; unknown cells inside
  the map can have no inflated obstacle. The final curve therefore still needs
  a separate PCT support/corridor check, in addition to current 3D collision and
  dynamics checks. See [occupancy query](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_env/include/plan_env/grid_map.h#L357).

In short: reuse the **global 3D route → bounded local target → collision-aware
local trajectory** separation. Keep explicit floor/body height conventions,
source-time synchronization, bounded failure recovery and whole-curve support
validation. Neither successful optimization nor a Gazebo demonstration grants
motion authority; this D1 Max integration remains a no-motion preview until
its separate hardware acceptance requirements are met.

### Isolated native regression evidence

The final 2026-09-24 synthetic test used the current production parameters,
unchanged robot envelope and a private loopback Zenoh graph. Four cases passed:
straight corridor; box detour after separate, correctly stamped lidar views;
single-view box occlusion (no accepted trajectory); and a closed corridor (no
accepted trajectory). The accepted curves were independently evaluated over
their complete cubic domains against analytic solid geometry and the configured
two-circle proxy, not just their visible center lines. The box-detour minimum
sampled clearance was **0.112 m**, maximum speed **0.2895 m/s** and acceleration
**0.3340 m/s²**; these are synthetic regression measurements, not safety margins
certified for a physical D1 Max. Multiview acquisition moved the synthetic ray
origin while the test body stayed still; no controller or robot was involved.
All owned processes exited cleanly. See the [report](../../log/offline_native_scan_scenarios/20260924_203436_642c90e5/report.json)
and [actual-trajectory comparison](../../log/offline_native_scan_scenarios/20260924_203436_642c90e5/native_scan_scenarios.png).

The earlier failing reports are retained. They exposed hidden-space shortcutting,
voxel-boundary contact, skipped free-ray cells, endpoint traversal errors and
geometry changes during time refinement. Passing the cases required fixing those
causes; the fixture was not made transparent, the footprint was not reduced and
collision checks were not disabled. The script is
`d1max_pct_scan/test/offline_native_scan_scenarios.py`; `--render-report` only
reads stored results and never launches ROS. Finite curve sampling, ideal lidar
and static scenes do not validate live dynamic obstacles, tracking or hardware.

## Inputs

- `body_pose` and `sensor_pose`: selected LIO odometry
- `cloud`: current scan transformed into the `map` frame
- `/goal_pose`: interactive PoseStamped goal for `navi_mode:=1`
- `/d1max/scan/initial_path`: external 3D nav_msgs/Path for `navi_mode:=3`

## Outputs

- `/d1max/scan/planning/bspline`: optimized local trajectory
- `/d1max/scan/grid_map/occupancy`: rolling 3D occupancy cloud
- `/d1max/scan/grid_map/occupancy_inflate`: collision-inflated cloud
- `/d1max/scan/optimal_list`: RViz trajectory marker

## Run

Start one LIO backend first, then run one of:

```bash
/home/dndx/d1max_nav_ws/start_scan_planner.sh faster_lio
/home/dndx/d1max_nav_ws/start_scan_planner.sh fastlio2
```

For non-default Faster-LIO topic remaps:

```bash
ros2 launch d1max_scan_planner scan_planner.launch.py \
  backend:=faster_lio odom_topic:=/your/odom cloud_topic:=/your/world_cloud
```

For a multi-floor global planner path:

```bash
ros2 launch d1max_scan_planner scan_planner.launch.py \
  backend:=fastlio2 navi_mode:=3 use_rviz:=true
```
