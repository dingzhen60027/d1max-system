# SCAN-Planner vendor source

Source: https://github.com/wuyi2121/SCAN-Planner/tree/ros2-community
Imported: 2026-08-23
Scope: ROS 2 planner packages only. Go2 simulation, descriptions, gait publishers,
and robot controllers are not launched by the D1 Max integration.

## Pinned audit references (rechecked 2026-10-16)

- Official ROS 1 `main`: [`348e8a590a50a5a6bbab8d8c6dcfd171f009be26`](https://github.com/wuyi2121/SCAN-Planner/tree/348e8a590a50a5a6bbab8d8c6dcfd171f009be26)
  (`add: sensor layout`, 2026-07-29).
- Official repository's `ros2-community`: [`d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`](https://github.com/wuyi2121/SCAN-Planner/tree/d0b921c9b05a6d291d144d60882b2e0e88d2c0e0)
  (`Update simulation configuration and map asset`, 2026-07-13).
- Author paper: [SCAN-Planner, arXiv:2606.19555v1](https://arxiv.org/html/2606.19555v1).

These are reproducible audit references, not a claim that the original import
was bit-for-bit either commit: the earlier import recorded its branch/date only.
The public branches are not algorithmically identical. In particular, current
main preserves the mode-3 global reference in `planFromCurrentTraj`; the ROS 2
community version recreates a start-to-goal polynomial there. The community
README identifies it as a derived community port, not an author-certified
controller release. Neither branch promises 100 Hz optimization: the 100 Hz
timer polls the FSM, and ordinary replanning is distance-triggered. Collision
checking is 20 Hz; planning in both pinned branches is synchronous.

The author's closed-loop controller uses a future chord for heading and
time-indexed velocity plus XY position feedback, allows lateral velocity, and
pauses translation for large heading errors. Its trajectory clock starts again
on reception. `legbot_3D_Nav` instead launches its own `a1_cmd_adapter`; despite
its FSM's actual-state input, the first manager initialization branch can still
sample the incumbent's time-indexed P/V/A. These are useful implementation
references, not evidence that a D1 forward-only percent-SDK controller can be
replaced verbatim. The D1 controller's actual-state projection, no-lateral
contract and source timestamps remain explicit.

## D1 integration and deliberate deviations

### Current D1 execution (2026-10-16)

The formal follower plans from the latest paired acquired P/V, not a future
point on an incumbent. The incumbent remains usable during candidate solving,
validation and preparation; a rejected candidate does not erase it. Formal
ordinary solve spacing is currently 0.1 s, on one bounded worker with a shared
400 ms solve budget. These are scheduling limits, not measured end-to-end
commit frequency. The owner harvests a ready result after a complete map write
transaction and refreshes acquired state again at adoption. See
[`ROLLING_REFERENCE_RECOVERY_20261016.md`](../../docs/reports/ROLLING_REFERENCE_RECOVERY_20261016.md)
for actual-source ages, rejected alternatives and limitations.

The forward-only D1 controller currently uses actual curve progress, soft
heading/cross-track feedback, measured yaw-rate damping and a spatial curvature
preview on the accepted curve. A single per-axis conditioner jointly bounds
command acceleration/jerk and mapped SDK rate/acceleration; the SDK writer does
not apply a second smoother. No lateral command is emitted. This is a deliberate
D1 adaptation, not the author's future-chord controller or a physical smoothness
acceptance claim. The older descriptions below retain their date-specific
historical scope.

### Author-aligned continuous execution (2026-10-15; historical)

The pinned author `main` above is the planning/control reference; the community
port is not assumed identical. FSM polling is 100 Hz and collision checking
remains 20 Hz. Bounded asynchronous solves retain the configured 0.2 s ordinary
refresh / 0.1 s submission spacing; these are not a claim of 100 Hz solving.
The non-upstream hard 0.5 s blocked/collision replan delay is removed. Numerical
failures can retry after the existing cooldown when both actual scanner source
times advance, even if a stationary saturated map has no new voxel changes.
Occupied/invalid/unknown input is not reclassified as free by that retry.

An accepted future splice cannot be repeatedly overwritten by either a
periodic or distance-progress ordinary trigger before its measured entry;
collision, cancellation, reference identity and measured reseed still take
priority. Global route order and non-final target velocity are preserved.
Visualization is separate from FSM rate: stable immutable committed curves
are updated by identity, diagnostics are bounded and phase changes immediate.

The existing D1 follower uses the author's velocity-feedforward plus full XY
position feedback in continuous odom. Heading follows our **current certified
tangent**, with geometric tangent-rate feedforward; it is not the author's
future-chord heading copied into an incompatible footprint certificate.
Unmeasured inertia prediction, default velocity damping, extra jerk shaping
and the SDK's one-second startup/PD input stage are not part of this mainline.
Original task/cancel/source-time contracts, actual occupancy and D1 geometry
remain; no extra collision owner or Go2 driver is introduced.

### Current local display (2026-10-14)

`grid_map/occupancy_inflate` now publishes the occupied-influence cells for the
configured D1 oriented box at the latest measured body yaw. It is a read-only
lattice configuration-space view, not the former raw compatibility alias or
the Go2 circular dilation. The native occupied-only box query remains unchanged.
XY uses its existing voxel-contact rule; inverse Z dilation uses the configured
body-above/below extents. Missing body heading yields no inflated display rather
than a fabricated zero yaw. The display mask is separate from collision buffers,
proofs and snapshots; an undrawn cell does not prove observed free space.

RViz enables this height-coloured layer by default with one 5 cm square per
published voxel. Raw occupancy is optional debug data and disabled on layout
selection. Cached Z-run dilation avoids a full robot collision query at every
map cell; the display rate remains bounded independently of map fusion.

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
  the checked detour seed, not a blocked global centerline. The D1 template uses
  1 from 2026-10-16 to retain the collision-repaired gentle entry guide; the
  generic optimizer default remains 0. The 2026-09-27 configuration used 0;
  neither this D1 value nor the earlier value 20 is an official recommendation.
  Guidance itself does not require the added cost.
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

## 10-09 formal BT mainline audit (source changes, not deployment)

The formal `navigation_session` has separate candidate and actuation queries;
the preceding legacy `require_observed_free=false` description must not be used
to infer its execution policy. Its generated configuration selects
`grid_map.planning_collision_policy=official`,
`fsm.trajectory_geometry_policy=official_inflated_geometry` and keeps the raw
observed-evidence layer enabled. This was also confirmed in the saved actual
`80a9625083cd42dd8e328f3f55081c1d` session parameters, not only the source template.

| Consumer | Native query | Meaning of raw unknown |
| --- | --- | --- |
| A*, rebound and complete candidate geometry | `getPlanningInflateOccupancy` | Does not create an occupied planning flag; not written as free |
| Actual command plus braking sweep | Raw observed snapshot | Insufficient evidence inside the requested sweep; unrelated unknown cells do not veto the entire map |

The pinned ROS 2 upstream inflated-buffer query and the formal candidate query
agree on this distinction. No collision policy, robot envelope, source lease,
near-body clear box or motion-acceptance flag was relaxed in this audit.

Changes are limited to ownership and truthful failure propagation:

- An internal A* 200 ms timeout is a resource failure, not a geometric dead end.
  It can start a new bounded attempt after the existing 500 ms cooldown without
  waiting for a stationary map to change. Each attempt still shares the original
  400 ms solve deadline; cancellation cannot be retried as resource recovery.
- Optimizer and final-curve failures carry their own terminal query evidence:
  state 1 is occupied, state 2 is uncertain/unobserved. Unknown cells visited by
  a previous search cannot overwrite the final curve's reason.
- The no-motion preview solver's original heading seed is value-copied with the
  curve into the independent validator. Millimetre-per-second drift cannot
  invent a different initial body yaw at that stage. Actual measured pose,
  translation/rotation sweep, complete-curve checks and source expiry remain
  required; this contract is rejected by the SDK writer lane.
- The typed BT path has a single reference owner and ignores the legacy JSON
  refresh callback. Legacy compatibility no longer kills a task after an
  unrelated 12 s/20-message limit. Central recovery budgets still apply; an
  already expired episode cannot be revived by a late third healthy message.

Tests use the production GridMap/solver and passive transaction ledger without
ROS initialization or SDK connection. These software regressions do not prove
that the recorded near-body occupied returns are robot self echoes. The frozen
raw-ray recording still contains real occupied classifications; the separately
approved offline-only three-voxel withdrawal is not a live collision repair.
Source changes and repository copies are not an update of the active sealed
runtime. No services were restarted and no robot commands were issued.

## 10-10 planning-only candidate contract

The author repository links the `ros2-community` migration; that branch is a
community ROS 2 port, not an author-certified D1 integration. Its `navi_mode=3`
receives `initial_path`. Its native solver publishes successful B-splines
without this project's motion-writer handoff transaction. The community port
also mixes measured odometry position with clock-sampled previous-curve
velocity/acceleration on replans; this is not evidence that a moving D1 has
those predicted derivatives. See the pinned upstream references at the top of
this file and the [community implementation](https://github.com/wuyi2121/SCAN-Planner/tree/ros2-community/src/planner/plan_manage).

The D1 `planning_only` lane now separates **new-candidate preview geometry**
from motion candidate admission, not just committed-preview progress:

- Keep the original measured solve state and immutable curve. Project the
  latest measured body onto a bounded initial arc; collision-check the real
  connector and full curve against the production GridMap and support.
- The manager, whole-curve validator and candidate ledger use the same typed
  geometry policy. A positive preview proof cannot populate motion join
  evidence or authorize the SDK writer.
- An out-of-range body requests a new measured solve on the same route.
  Cancellation, source expiry, map/context identity and stale-result checks
  remain mandatory. Positive evidence uses candidate/seed compare-and-swap;
  an older proof cannot overwrite a newer candidate or committed curve.
- Solve adoption retains the original 400 ms solve-to-body bound. Subsequent
  candidate revalidation is an explicit separate phase: the immutable solve
  source remains provenance, while acquired current body/map sources determine
  freshness. It cannot restamp the solve, extend an old proof without new evidence, accept
  a regressed body source, or certify an out-of-range connector. Validator
  scheduling delay alone therefore does not force an otherwise valid new solve.
- The only added tuning knobs are a 0.15 m preview connector and 0.50 m initial
  projection window. They are neither obstacle clearances nor actuation join
  tolerances. The execution lane's motion boundary checks are unchanged.

This is a planning repair. Native synthetic quality cases, recorded input
checks and physical navigation acceptance must be reported separately; neither
unknown planning flags nor a nominal CAD self model prove free space or a
verified robot posture. No tracker/SDK integration is claimed by this change.

## 10-10 native dynamics and separate tangent-rate diagnostics

Production SCAN timing constrains velocity and acceleration, as in the pinned
ROS2 community implementation. It does **not** equate XY velocity-tangent
turning with physical body yaw or use it as a new candidate-admission gate.
Continuous tangent-rate analysis is an offline saved-curve diagnostic, not a
new production parameter, motion limit or permission switch.

- A stationary start preserves optimized XYZ through speed/acceleration time
  allocation. A moving start retains its original position, velocity and declared
  acceleration boundary in the native interior refit. An assumed zero
  acceleration is not labelled as an acceleration measurement.
- Saved-curve tangent diagnostics use the final curve's physical
  direction-confidence threshold, matching the OBB held-heading policy.
  They run outside the production solver; their cost, absence, excess or
  analysis failure cannot consume the collision-check budget or reject a valid
  planning candidate. They do not certify body-yaw feasibility or steering.
- Search, optimization, timing and whole-curve OBB validation share the original
  400 ms deadline and cancellation token; diagnostic work does not renew it.
- An attempted extra tangent-rate retiming was **not deployed**: the same
  recorded moving boundary produced a 119.55 s, then 100.87 s, two-metre curve.
  Global knot dilation enlarges the fixed-velocity endpoint controls and their
  reversal while a normalized confidence threshold incorrectly promotes a
  near-zero velocity direction into a body-yaw constraint. The failure evidence
  and bounded mathematical helpers remain available, not wired into production
  timing. Steering/hold-yaw behavior belongs to the later controller work, not
  an unproved SCAN-only restriction.

Reference replacement reads its committed predecessor version and trajectory
ID together under one lock. Previously a concurrent commit could occur between
two getters and manufacture a predecessor pair that never actually existed.
The corrected transaction does not relax task/route identity or clear the
active trajectory when a different candidate is rejected.

Evidence is recorded in `experiments/scan_turning_dynamics_20261010` and the
same-input quality report's `after_turning` and `after_turning_tightened`
directories (failed experiments), plus its final planning-only diagnostic
comparison. These are passive checks
of production components, not another runtime entry or robot motion acceptance.

## 10-11 one native SCAN collision policy and incomplete-proof semantics

The formal lane now selects the upstream-style `official` occupied-only policy
for search, candidate geometry, incumbent revalidation and the actual
reaction/braking sweep. The sweep reads the same production raw GridMap
snapshot: definite occupied and map exterior reject it; in-map never-observed
and insufficient odds remain unchanged and counted in raw diagnostics, but are
not separately vetoed as observed-free requirements. Nonfinite map contents
remain invalid. Explicit `observed_free` comparison mode still requires fresh
per-voxel FREE evidence. No near-body clearing or unknown-to-free rewriting is
introduced.

SCAN remains the collision owner. Its current-curve and actual-command checks
retain the original acquired front/rear source ages, body state, task/context,
curve, permit and exact demand identity. A positive native certificate expires
at the minimum of these original leases; an official result does not pretend
to be an observed-free certificate, extend a sensor stamp, bypass SDK limits
or establish physical acceptance. The downstream consumer checks that native
certificate and its expiry instead of independently reclassifying unknown
space. The full existing current-curve collision scope is unchanged here;
this is not a new validated-prefix or distant-obstacle continuation protocol.

Bounded projection, whole-curve query and command-sweep exhaustion are typed
as **incomplete**, not a witnessed collision. They cannot issue a new positive
or false negative proof. Existing exact-identity evidence may be used only
until its original expiry; an expired source, observed occupied cell, outside
cell, canceled/replaced task or invalid connector is not a scheduling miss.
This change is exercised by passive production-map tests and does not assert
that the D1 sensor calibration, braking or physical navigation has passed.

## 10-18 map writer / planning owner separation and measured remaining display

The formal `task_follower_v1` lane now separates the GridMap writer callback
group from the reference/FSM owner. A third executor thread allows the existing
independent control inbox to run alongside them. This is a D1 integration
change, not an upstream SCAN algorithm or frequency claim.

- The writer always uses its fixed production GridMap. Reference geometry,
  result adoption and FSM queries borrow an exclusive, completed validation
  snapshot. The original three-slot pool is retained: two owner/validator slots
  and one solver/admission slot. A busy slot is not overwritten or replaced
  with an unbounded allocation. Query caches are never shared concurrently.
- A pending owner input is coalesced to one latest original message per input
  type. Source times, generations and deadlines are not renewed. Body/progress
  inputs drain before reference geometry. A busy pool does not consume a ready
  solve result or clear the incumbent. Cancellation and exact commit identity
  remain checked by the existing execution ledger.
- Snapshot-to-snapshot copies carry the already aged query clock. Map context
  reset is published before its ACK, even before the next ray; a context/fault
  watermark prevents new borrows of the previous identity when a copy misses.
- A verified rolling extension may name a predecessor curve that naturally
  advanced within the same exact execution version. This input recovery does
  not relax prepare/commit compare-and-swap or accept a future curve, another
  route, anchor, map context, localization epoch or canceled task.
- The D1 extra 100 ms submission throttle is removed. The default moving
  refresh request interval is 50 ms; one worker, the original shared solve
  deadline and adoption backpressure still bound actual throughput. A request
  interval is neither guaranteed commit frequency nor a new official ceiling.
- RViz renders only the remaining domain of the exact committed spline, using
  fresh, identity-matched measured progress. It does not move the spline's
  start, predict progress by wall clock, alter control or display a candidate
  as execution. A temporary progress gap freezes the last observed trim.

The 10-11 command-sweep description above is historical: since 10-12 the formal
task follower has SCAN as its sole collision owner, without a second downstream
voxel/swept-volume veto. This round does not add one. Controls, SDK response,
raw evidence classification, self-filtering and the current 5 cm per-side
planning clearance are unchanged. See report E9 and its isolated evidence;
software timing counterexamples are not physical smooth-navigation acceptance.

## 10-18 E10: complete-snapshot and command delivery, not faster clocks

The E9 robot run reduced worker-to-owner delay but exposed independent renewal
and command-phase losses. The fixed three-slot production pool now reserves the
newest completed validation head for the fast current-curve reader; the FSM
borrows the other completed same-context view. Solver acquisition must not
overwrite an already newer third-slot publication with that older owner view.
If publication meets borrowed slots, the map writer coalesces one pending copy
and retries on its own 10 ms callback group; this copies already integrated data
with original ray sources, not another integration or a renewed observation.

An expired strictly older ray pair that did not complete a geometry query cannot
replace a still-unexpired positive for the exact current job solely because its
occupancy revision is larger. Actual occupied/unknown/invalid geometry, mixed or
new sources, identity changes and original expiry retain their existing fences.
Passive diagnostics distinguish fast borrow misses, fallback borrows, owner
misses, publication misses and the actual selected source pair.

Follower geometry prepare/ACK/commit is event-driven, with a 10 ms preparation
completion poll and unchanged bounded 50 ms identical-message retries. Velocity
control remains 20 Hz. Ordinary small anchor corrections remain accumulated
without turning every rolling window into a full reanchor. SDK receipt wakes
the sole writer rather than awaiting an independent periodic phase; actual
nonzero submissions remain at least 50 ms apart, with original deadline-driven
zero and no source renewal. See E10 report and the recorded external SDK patch.
These are D1 transport contracts, not new upstream SCAN algorithm claims.
