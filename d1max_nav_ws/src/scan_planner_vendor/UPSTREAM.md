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
  configuration now uses 0 (2026-09-27 ROS 2 integration), selecting the upstream
  rebound objective (smoothness, collision, feasibility). The earlier value 20
  was not an official recommended value. Guidance itself does not require it.
- Added bounded failures/environment-change retry, explicit A* initialization
  failures, complete-curve endpoint/yaw sampling and truthful attempt diagnostics.
  A hover or failed optimizer is not accepted as a navigable trajectory.
- Internal rebound A* helper anchors must have actual lattice egress/ingress.
  On open-set exhaustion only, these helper anchors can move farther along the
  original line, within one shared 200 ms deadline and bounded shift count.
  This corrects documented one/two-cell heading traps without moving an external
  occupied start, changing the inflated map, or bypassing edge collision checks.
- Rebound's post-convergence collision check now covers the entire spline,
  including its endpoint, using derivative-control-point speed bounds for at
  most resolution/4 travel per sample. This triggers the existing maximum-three
  collision-weight restarts for a blocked tail, instead of repeatedly rejecting
  the same tail only at final admission. Invalid/budget-exhausted checks reject;
  final measured-yaw/whole-curve admission remains unchanged. The source of the
  occupied tail failure is recorded in the 2026-09-27 dual-ray offline regression.
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
the whole two-cylinder proxy, not only its centerline. This was the live policy
before the 2026-09-27 official-policy selection described below. Bounded cell caches are cleared on
integration, map mutation and sliding; unknown-to-observed changes wake pending
planning. Unknown space is not silently filled behind a measured obstacle.
`waiting_observed_space` is a blocked observation condition, not an accepted
trajectory. In this strict policy, collision status 0 means saturated observed free, 1 occupied, 2
unknown or uncertain, and -1 outside the rolling map. In particular, a first
obstacle hit below the probabilistic occupied threshold is **not** free evidence.

### Current default: official collision policy (2026-09-27 user decision)

Live sessions now explicitly select `scan_collision_policy: official`, yielding
`grid_map.require_observed_free=false`. `getInflateOccupancy` uses the pinned
upstream front/rear center queries into the inflated-obstacle buffer. Inflation
offsets also use upstream XY `norm < radius` and Z `ceil` discretization, not
the D1 conservative voxel-AABB kernel. In-map unobserved cells do not add a
planning veto, but are not written into the raw map as observed free. Actual
occupied inflation and out-of-map queries remain blocking.

The earlier strict policy described above is retained only when explicitly
selected as `observed_free`, for regression/backward comparison. It is **not**
the current live default. The D1-only current-body and output-spline PCT support
vetoes are not applied in official mode. Source identity, frame, timestamp,
native accepted-trajectory pairing and the no-motion entry remain enforced.
Robot dimensions are still the D1 profile, not copied from the Go2 demo.

The subsequent ROS 2 integration also uses the pinned upstream objective
weights: smoothness 1, collision 1, feasibility 0.1, refinement fitness 1,
with the additional D1 reference cost disabled (0). Both normal and standalone
PCT entries read the common D1 YAML; neither overrides the reference cost to 20.
Map resolution, D1 collision dimensions, clearance distance and velocity limits
remain D1 configuration values, not a copy of the Go2 simulation calibration.
Native A* endpoint failure classification uses actual official collision
queries in this mode, not raw-evidence unknown counts from diagnostics.

Per-sensor integration is independent of collision policy. It walks the complete,
actual floating-endpoint measured ray.
The current live configuration selects `per_sensor_rays`: source sensor IDs,
per-point acquisition times and separately motion-compensated ray origins are
preserved through `ProjectedRays`. Completed native integrations, not receipt
or projector heartbeats, establish each source's freshness. This interface is
present in the 09-27 stationary recording; physical extrinsics, self geometry
and near-body coverage are still unverified, so it is NOT motion approval.

The retained **legacy `deskewed_cloud` backend** feeds merged front/rear points
with a single tracking origin. It remains a synthetic-origin approximation;
strict voxel rules cannot reconstruct dropped source/time attribution. Do not
confuse it with the current per-sensor backend or with the separate motion
safety-scan branch, which still needs its own independent-input migration.

The legacy endpoint-deduplication / stop-at-first-shared-cell shortcuts can skip
other visible cells, so they remain only in explicitly legacy mode. Each free
cell receives at most one miss vote per scan; obstacle hit votes are unchanged.
The hit endpoint cell is never added as a free miss. Rays stop at their actual
hit or range/map truncation: no hidden backside or unmeasured beam volume is
filled. A bounded 16-million-cell traversal budget fails closed (no fresh map
authorization stamp) on exhaustion, retaining the accumulated map evidence.

Strict collision queries intersect the actual continuous two-cylinder centers
and Z envelope with each complete obstacle voxel AABB; candidate-offset lookup
does not add a second query-center voxel expansion. Nominal body dimensions are
not reduced or arbitrarily increased. The configured obstacle voxel volume is
distinct from the unchanged nominal radius .29 m / offset .20 m. The bounded
cylinder query cache is invalidated on **each actual integration**, every raw
classification change and ring-buffer slide/reset. Neither observed free volume nor the
collision proxy proves traversable ground support. Current D1 collision sampling is
bounded/discrete, not a continuous swept-volume proof. The paper's virtual
boundary-layer dead-end recovery was not found in either pinned public search/
planning implementation; it is not an existing switch enabled by this import.

## 09-27 recovery fixes (source changes, not deployment)

Soft input-lease loss still revokes the task immediately, but does not advance
the sensor epoch barrier for an unchanged, fault-free per-sensor map context.
Existing observations expire on their original source clocks. New localization
identity or geometry faults still invalidate old-context data.

Native reference rejection now explicitly reports odometry/frame/geometry
failure via the existing generation-tagged invalid debug message. The bridge
no longer treats publication as native acceptance: it withdraws the rejected
generation and, for a preview-only odometry race, requests bounded revalidation
from the global goal owner. It never queues an old route for automatic execution.

Pure geometry unit tests and isolated synthetic ROS checks must remain clearly
separate from live robot acceptance. Official Go2 defaults/launch controllers are
not D1 Max calibration or permission to move the robot.
