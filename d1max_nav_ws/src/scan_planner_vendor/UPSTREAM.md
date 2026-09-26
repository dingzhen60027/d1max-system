# SCAN-Planner vendor source

Source: https://github.com/wuyi2121/SCAN-Planner/tree/ros2-community
Imported: 2026-08-23
Scope: ROS 2 planner packages only. Go2 simulation, descriptions, gait publishers,
and robot controllers are not launched by the D1 Max integration.

## Pinned audit references (verified 2026-09-24)

- Official ROS 1 `main`: [`348e8a590a50a5a6bbab8d8c6dcfd171f009be26`](https://github.com/wuyi2121/SCAN-Planner/tree/348e8a590a50a5a6bbab8d8c6dcfd171f009be26)
  (`add: sensor layout`, 2026-07-29).
- Official repository's `ros2-community`: [`d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`](https://github.com/wuyi2121/SCAN-Planner/tree/d0b921c9b05a6d291d144d60882b2e0e88d2c0e0)
  (`Update simulation configuration and map asset`, 2026-07-13).
- Author paper: [SCAN-Planner, arXiv:2606.19555v1](https://arxiv.org/html/2606.19555v1).

These are reproducible audit references, not a claim that the original import
was bit-for-bit either commit: the earlier import recorded its branch/date only.
The public branches are not algorithmically identical. In particular, current
main preserves the mode-3 global reference in `planFromCurrentTraj`; the ROS 2
community version recreates a start-to-goal polynomial there.

## D1 integration and deliberate deviations

- Mode 3 consumes the ordered PCT polyline in the localization map frame, with
  an explicit ground-to-body Z offset. Bounded arc projection and slicing retain
  corners, stair profile and route ordering; they do not fit a new global
  polynomial or seek a different floor by nearest XY.
- A blocked horizon endpoint is adjusted only along this same reference, within
  configured forward/backward margins, stopping at the first sampled rolling-map
  boundary. It is only a candidate endpoint. Native search, rebound optimization
  and complete final-curve collision validation remain required.
- Occupied subsegments may use native projected A* to make a checked local seed.
  Search is XY with its upstream interpolated height plane, not unrestricted
  flying 3-D A*. The optimizer still suppresses Z gradients. This is **not** a
  ground-support, edge/drop-off, or cross-floor locomotion safety proof.
- `optimization.lambda_reference` is an optional **local D1 extension** attracting
  the checked detour seed, not a blocked global centerline. The current module
  configuration retains 20 for compatibility; it is not an official recommended
  value. Zero selects the upstream rebound objective (smoothness, collision,
  feasibility). Guidance itself no longer requires a positive extra weight.
- Added bounded failures/environment-change retry, explicit A* initialization
  failures, complete-curve endpoint/yaw sampling and truthful attempt diagnostics.
  A hover or failed optimizer is not accepted as a navigable trajectory.
- At a numerically stationary start, guided mode preserves successful rebound
  geometry and enforces speed/acceleration bounds by uniform time scaling, then
  checks the whole curve. It does not invoke an XYZ-changing refinement merely
  to slow down. A nonzero measured initial velocity/acceleration is never silently
  scaled; its boundary-preserving refinement must pass actual initial-derivative
  checks or return `failed_dynamics`. Static synthetic detours are not validation
  of dynamic trajectory handoff or robot tracking.
- Session/generation tagging, exact-time cloud/sensor-pose association, input
  freshness, cancellation and diagnostic lifetime gates are D1 interface safety
  additions, not claims about upstream behavior. The integrated planner does not
  by itself authorize SDK motion.

## Parameter semantics and known limits

`double_cylinder_radius` is each XY disk's radius; `double_cylinder_offset` is
each cylinder center's distance in front of/behind the body reference along yaw.
The code inflates obstacles in Z by `[-obstacles_inflation_z_down,
+obstacles_inflation_z_up]`. Therefore the equivalent body envelope is reflected:
`[-z_up,+z_down]` about the queried body reference. Do not directly assign a
physical upper/lower body clearance by the parameter suffix alone.

Upstream in-map unobserved inflation cells are zero. The D1 strict mode
`grid_map.require_observed_free=true` also checks raw observed-free cells over
the whole two-cylinder proxy, not only its centerline. Legacy defaults to false
explicitly; live configuration enables it. Bounded cell caches are cleared on
integration, map mutation and sliding; unknown-to-observed changes wake pending
planning. Unknown space is not silently filled behind a measured obstacle.
`waiting_observed_space` is a blocked observation condition, not an accepted
trajectory. Collision status 0 means saturated observed free, 1 occupied, 2
unknown or uncertain, and -1 outside the rolling map. In particular, a first
obstacle hit below the probabilistic occupied threshold is **not** free evidence.

Strict integration walks the complete, actual floating-endpoint measured ray.
This statement assumes each input cloud really belongs to its supplied sensor
origin. The current live D1 bridge feeds **merged front/rear deskewed points**
with one tracking-origin pose; that is a synthetic-origin approximation, not
verified per-return visibility. The rear source has a nonzero extrinsic offset
and deskewing also moves returns to a common scan-end frame. Strict voxel rules
cannot recover the lost source/time attribution, so live free-space evidence
is not yet a robot-motion safety proof. The isolated single-/multi-view tests
supply the genuine analytic origin for each separate first-return cloud and do
not validate this live dual-LiDAR approximation. A future physical integration
must preserve source sensor ID and acquisition time through deskewing, and
provide each return's correspondingly deskewed origin (or separate per-sensor
cloud/origin pairs with justified within-scan motion error). A static TF alone
after dropping source IDs is insufficient to recover that information.

The legacy endpoint-deduplication / stop-at-first-shared-cell shortcuts can skip
other visible cells, so they remain only in explicitly legacy mode. Each free
cell receives at most one miss vote per scan; obstacle hit votes are unchanged.
The hit endpoint cell is never added as a free miss. Rays stop at their actual
hit or range/map truncation: no hidden backside or unmeasured beam volume is
filled. A bounded 16-million-cell traversal budget fails closed (no fresh map
authorization stamp) on exhaustion, retaining the accumulated map evidence.

Conservative voxel inflation includes both the occupied voxel's
volume and the queried center voxel's quantization; nominal body dimensions are
not reduced or arbitrarily increased. The .08 m grid discretization allowance
is distinct from the unchanged nominal radius .29 m / offset .20 m. The bounded
cylinder query cache is invalidated on **each actual integration**, every raw
classification change and ring-buffer slide/reset. Neither observed free volume nor the
collision proxy proves traversable ground support. Current D1 collision sampling is
bounded/discrete, not a continuous swept-volume proof. The paper's virtual
boundary-layer dead-end recovery was not found in either pinned public search/
planning implementation; it is not an existing switch enabled by this import.

Pure geometry unit tests and isolated synthetic ROS checks must remain clearly
separate from live robot acceptance. Official Go2 defaults/launch controllers are
not D1 Max calibration or permission to move the robot.
