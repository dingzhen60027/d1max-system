# D1 Max SCAN trajectory tracker

The formal executable is `local_follower`: one accepted local curve and one
20 Hz control owner, with asynchronous bounded candidate preparation. BT owns
the task and task-level lease; Native SCAN owns environment evidence; SDK owns
the unique command sender. The old `trajectory_tracker` / per-curve writer CAS
implementation is retained for historical regression, not launched in the
mainline. See [current lifecycle](../../docs/design/NAVIGATION_TASK_LIFECYCLE.md).
Source, isolated binaries, deployed release and physical acceptance remain
separate. This package never contacts the SDK, changes posture, takes control
or arms the robot. An unchecked demand or debug Twist cannot authorize motion.

It links the vendored `bspline_opt::UniformBspline` implementation. `order` is
the polynomial degree (SCAN emits 3); explicit knots are preserved and relative
time is evaluated with `evaluateDeBoorT`, i.e. shifted by knot[order]. Position
and derivative therefore have exactly SCAN's semantics. Like the reference
`legbot_3D_Nav` adapter, progress follows the measured nearest point and a
lookahead, not elapsed execution time. No Go2 motor controller is launched.
Reference reviewed: [a1_cmd_adapter.cpp at f60606a](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/a1_cmd_adapter.cpp).

In formal Native transport, current-curve checking and candidate admission
retain separate snapshot/cache lanes. SCAN owns collision decisions; a 50 ms
scheduling target does not promise 20 Hz completion of a longer computation.
No tracker/SDK collision query, extra reader or map slot is added.

Since 2026-11-05, formal `task_follower_v1` uses ordered SCAN decisions with
`TrajectoryValidation.valid_until=0`, **not a renewable map-proof lease**.
An incomplete ordinary update does not revoke the last complete decision;
a newer complete negative decision immediately supersedes it. Matching task,
curve and sequence prevent an old positive message from reviving newer negative
state. SCAN independently detects genuinely stalled sensing/checking. The
20 Hz controller and unique SDK writer do not approve map freshness again.
Original body/command deadlines, cancellation, emergency stop and ownership
remain independent. No map heartbeat resets the controller conditioner or
renews source time. Progress arriving before its source body gets a bounded
one-shot recheck on arrival of that original body, not a new timestamp.

For a rear-facing target, forward-only steering remembers the selected turn
side through ordinary curve replacement and temporary holds. Entering the
front half-plane releases that rear intent; cancellation/new tasks reset it.
Path tracking and explicit terminal heading have separate intent state. The
sole speed/acceleration/jerk conditioner remains responsible for output, with
zero sideways demand. SCAN's existing body-entry/terminal orientation check
covers both rear branches with a full-rotation envelope; ordinary forward
entry and curve-internal heading changes retain the shortest sweep. Rear/front
classification uses the uncorrected immutable path heading, not lateral-error
feedback. There is no new tracker/SDK collision layer. With no signed-turn
field on the current wire, a rear target whose only free turn is the short
branch may be refused; this implementation does not claim direction-selective
turn planning in that case.

Runtime evaluation uses an immutable `FixedBsplineSampler` prepared from the
vendor's read-out control points, knots and degree, including native derivative
curves. Its fixed 3D De Boor recurrence is bitwise-tested against the vendor at
nonuniform knots, both neighbours, endpoints and every admitted degree. It does
not interpolate the sample table, change join/speed bounds, accept repeated
knots, or bypass the full geometry equality check. Stack storage removes the
per-tick dynamic vectors, and left-span binary lookup replaces the native linear
knot scan. This adds bounded immutable preparation memory; isolated core timing
does **not** establish the ROS/transport/SDK end-to-end 20 ms deadline.

The controller caches actual vendor-spline XYZ arc length (bounded to 6000
intervals) at admission and searches only a bounded local arc window on each
new source-stamped body sample. It permits 0.15 m physical backtracking and a
bounded forward window, retaining a separate curve-local high water mark.
Ties and repeated positions do not advance progress. A large projection jump
outside the window revokes the curve rather than ratcheting toward a branch.
Accepted geometry is durable, not a heartbeat-based movement lease. Ordered
Native collision state, original body deadline and the task lease govern output;
spatial progress never refreshes stale data. Search work is independent of
the spline's total duration. For holonomic tracking, `lookahead` is a preview
time: measured planar speed times this time is clipped to the configured
minimum/maximum distance, then sampled by XYZ arc length, not by future spline
time. Legacy forward/yaw fixtures retain their historical controller semantics.

Asynchronous replacements carry a measured join into the **original full
spline**, not a translated or re-timed copy. `valid_start_time` and
`valid_start_arc_length` identify that join; `join_source_stamp`, pose and
world-frame twist retain their source evidence. Formal rolling replacements
use a bounded **geometric connection**, not equality between acquired velocity
and spline derivative. Native SCAN checks the actual body-to-curve connector
and curve; its 0.15 m connection domain is shared with the follower. This is
not a cleared radius or a blanket collision certificate. Source time and XYZ
bound projection locally, and the original candidate's forward arc is capped
at 0.5 m so refreshed proofs cannot ratchet to a distant branch. Complete
derivative speed/acceleration bounds still apply. Legacy exact-state callers
and future-prefix splices retain 1.25 cm / 0.05 m/s admission. A valid join can
begin at nonzero curve time and preserves command velocity, acceleration and
conditioner history. Time passage alone never advances that point.
No acceleration measurement exists on the current source contract, so native
`join_acceleration_valid` remains false: this does not claim measured C2
continuity. A stale, cross-context or failed join does not replace an already
accepted curve; explicit revocation and stale-current-data handling remain
separate.

`TaggedBspline` separately carries Native's immutable `body_heading_seed_yaw`
and `body_heading_sampling_intervals`. The measured join quaternion retains
its actual source meaning; it is not substituted for the solve/splice yaw.
The formal follower requires these fields, prepares the identical full-curve
heading-memory table on its existing worker, and looks up measured progress
without replaying the whole lattice each tick. Low-speed handoffs therefore
do not inherit an unrelated previous curve's heading. Historical core fixtures
without this metadata remain explicitly legacy, not current formal input.

The formal node prepares spline samples, derivatives, support bins and the
curvature/braking envelope on a **single bounded worker**. There is at most
one running job, one latest pending job and one completion; newer input and
cancel retire older work synchronously. The worker receives immutable inputs
and never accesses the live controller. The single control owner polls the
result, checks its exact geometry/configuration identity, re-observes the
actual entry against current source-bound state and proof, and retains the
actual local predecessor identity, without an SDK curve transaction. A planned
safe common prefix and future tail may be prepared while moving; switching
waits for measured progress at the splice and checks actual position/velocity.
No preparation grants permission or refreshes a source timestamp.
Candidate failure leaves the incumbent intact; a genuine current SCAN
negative decision or input fault still stops output. Same-ID geometry cannot be
rewritten, and changed support contents invalidate cached preparation even
when a producer erroneously reuses a hash.

### One owner for traversability

The formal `LocalFollower` fixes **Native SCAN** as geometry authority at
construction; it is not a ROS parameter or a UI bypass. SCAN's matching current
trajectory verdict decides environmental feasibility. The follower no longer
reclassifies historical ground returns by nearest-point height, neighbouring
step/slope or body-to-ground residual, at either preparation, control, recovery
or terminal alignment. Raw support slope also no longer reduces the speed
envelope; actual spline derivatives and planar/spatial conversion remain.

Support packets still bind the task, reference generation, frame and source
identity, with finite bounded data. Entry pose/velocity, original state and
command deadlines, cancellation and task ownership retain their existing
semantics. A negative SCAN verdict stops output, but no per-map deadline is
used in the formal task follower. Removing the
duplicate classifier does not clear obstacles or turn unknown cells into free
space. Historical standalone `TrackerCore` fixtures retain their old geometry
policy; the formal constructor cannot select it, and cached geometry is bound
to the selected authority.

Local-state ingress orders original epoch and nanosecond source time before
decoding usability or interpreting a fault. Retired epochs and duplicate or
late observations do not erase fresh state or refresh recovery. A newer
expected epoch reaches hard-reset handling even when its clock moved back;
foreign session/map/interface packets are discarded before fault handling;
obviously future samples cannot poison the stream watermark. A fresh
source fault still enters hold, and its source barrier prevents an earlier
valid sample from clearing that fault. Source leases and the three-distinct-
sample/0.6-second recovery window are unchanged.

These changes remove heavy preparation from the 20 Hz control owner lane; they
do not certify hardware scheduling deadlines. Owner admission still performs bounded
actual-entry checks and support-identity/input comparisons. CPU-pressure/NUC timing,
full ROS scheduling and physical navigation remain separate acceptance work.

## Continuous holonomic tracking

The platform remains holonomic, but the **default navigation strategy is
forward + continuous steering, with body `vy` exactly zero**. The legacy
`align_then_forward` parameter selects this strategy; it does not mean a
stop/fully-align/go state machine. Saved lateral profiles are not the default.

Measured XYZ projection on the committed SCAN curve supplies tangent, curvature
and progress. Forward demand follows the curve speed and along-track error;
signed cross-track error changes desired heading over a speed-scaled spatial
convergence distance. Forward scales continuously by `max(0,cos(error))²`.
Yaw combines curvature times the accepted forward command, soft proportional
heading correction, and measured angular-rate damping. The soft error is
`e - s*tanh(e/s)`: its slope is bounded by one, without an engage/release angle,
fixed 20% turn relay, reversal latch, or an assumed inertia predictor.

Heading and cross-track feedback are evaluated at the ordered projection;
curvature anticipates the same accepted curve's .40–.80 m spatial chord. The
chord starts on the curve, not at the measured body, so lateral error is not
counted a second time. In ordinary tracking, the preview supplies anticipation
while angular-rate damping references the **current** accepted tangent's
curvature. Before twist allocation the law is
`v * preview_curvature + P - d * (measured_heading_rate - v * current_curvature)`;
the existing forward-feasibility weight applies to `v`. Previously the damping
reference also used preview curvature, multiplying future bending by `1+d`.
Position/heading gain, measured-rate damping gain and the command conditioner
are unchanged. Near zero curve velocity, the existing heading contract may
hold direction rather than define a tangent rate. The correction therefore
blends in with C2 weight from the contract's minimum direction speed to twice
that speed; below it the established law is retained exactly. This weights
only the rate reference, not motion permission or forward speed. At stair
transitions this correction blends out with C2 weight;
the established full-flight law stays unchanged. The shared special-entry heading contract keeps its
resolved curvature instead. This is not an author-style future-heading command
and does not move the accepted curve or its certified nominal heading.
A zero-derivative entry uses a small bootstrap derived from
the configured SDK effective point. It is independent of steering lookahead:
enlarging lateral convergence distance must not raise the entire curve's speed.
No trajectory clock or measured source timestamp is advanced artificially.

One `MotionCommandConditioner` per commanded axis intersects SI acceleration,
SI jerk, mapped SDK rate and mapped SDK acceleration before selecting output.
There are no serial controller/SDK smoothing delays. Normal target changes and
curve replacements preserve its state. Explicit stop/fault resets it. A hard
cap conflict has priority and is reported, not disguised as smooth control or
latched into a permanent navigation failure.

`../d1max_pct_scan/config/motion_tracking.yaml` owns the 20 Hz controller, .40–.80 m steering
distance (1.2 s speed scaling), position gain .8, yaw gain .9 and measured yaw
damping .4. Normal command limits: .30 m/s, .50 rad/s, .35 m/s², .80 rad/s²,
.70 m/s³ and 4 rad/s³. These are **command values**, not physical calibration.
The 20 Hz unique SDK writer uses the existing static V4 mapping, without a
second filter; forward reaches approximately 30–40% and yaw blends through
20% toward 30%. These fractions are response clues, not measured SI speeds.

The derivative limits above apply at the controller's calculation timestamps.
The independent latest-command SDK writer may repeat or skip a sample under
delivery jitter; equal nominal 20 Hz rates do not establish the same derivative
bounds on its send timestamps, or on physical robot motion. The 2026-10-17
sampling audit reproduced this coverage gap without an extra stop or steering
sign reversal. It is not evidence that another filter or higher gain fixes
physical oscillation; see the E6 section of the rolling-reference report.

`ControlTiming` now accumulates **every control tick** and reports once per
second: actual monotonic interval, velocity-difference acceleration,
acceleration-difference jerk, stop edges and hard-cap overrides. Its derivative
history survives log windows and curve replacements. All-event peaks include
forced stops and recovery; the separate normal-tracking peaks require adjacent
normal samples, but still include hard-cap overrides. Zero stop edges alone do
not prove smoothness. This is passive bounded storage: it neither filters nor
gates commands, and does not measure SDK-send or physical acceleration.

Production curve handoff reserves one candidate in Native before entering the
measured seam, then commits and acknowledges the exact version. An ordinary
new candidate cannot overwrite that reservation. The incumbent continues
during preparation; references consume only the accepted committed receipt.
Prepared is not Committed: a new curve cannot become the follower's incumbent
until the final Native ACK. Formal stateful verdicts do not have the historical
map-proof handover runway or proof-renewal wait. Original state validity,
measured entry, transaction ownership and cancellation remain explicit.
Candidate failure leaves the accepted task/curve intact; occupied verdicts
and cancellation remain immediate. A local endpoint is never task completion.

Offline control-driven reproduction (real core, spline and SDK map; simulated
response only, no ROS/SDK connection):

```bash
bash tools/run_forward_closed_loop.sh /tmp/d1max-forward-closed-loop
```

This separates software convergence from physical navigation acceptance.

### Historical large-body sensitivity work (2026-10-13; superseded)

The following records the old damping/input-shaping experiment, **not the
current default**. It is retained as reproducible historical evidence; the
2026-10-15 mainline above removes its assumed-inertia tuning.

The control change is implemented in this order, on the existing mainline:

1. Bind the real frozen release, controller parameters and SDK profile; never
   infer deployment from editable source or a newer directory name.
2. Keep body-frame SI demand and `Move` fractions separate. The operator's
   approximate 20%/30% response applies to forward/reverse, lateral and yaw.
   The SDK's smooth low-input compensation retains exact zero and ramps input
   changes; it does not turn every nonzero component into a fixed 20% command.
3. Use actual-state velocity damping, gentle spatial correction and coordinated
   XY/yaw acceleration. Keep control history across accepted curve replacements.
4. Retain task/route identity through local endpoint recovery. Small final
   corrections stay in the current SCAN-validated connector domain; larger
   residuals require a fresh actual-body SCAN solve, not an arbitrary direct-goal
   command. Being inside the goal position tolerance while still moving is
   settling, not completion.
5. Classify SDK writes using the actual final float arguments. Sending zero,
   receiving its ACK and proving physical standstill remain different facts.
6. Compare the production controller and SDK mapping against delayed,
   deadzone-bearing inertial response, then update and seal the one default
   bundle. No SDK connection or physical motion is part of these passive checks.

The scalar SDK input map has a smooth blend from zero to the observed effective
fraction (20%), followed by a smooth increase toward 30%. Planar direction is
preserved for the static target using dominant-axis normalization and a combined
30% norm cap; yaw has its own map. The writer slews the XY vector and yaw at
.30 fraction/s rising and .45 falling, using at most 50 ms per tick. Complete
zero/HOLD/stop bypasses this shaping immediately; ordinary component zero and
sign reversal ramp down. Curve changes retain history; new task/epoch/control
generation, fault and explicit stop reset it. Input zero does not prove the
body has stopped. Mixed-axis response and physical speed are uncalibrated.

Pure sensitivity reproduction (no ROS initialization or vendor SDK linkage):

```bash
cd /home/dndx/d1max_nav_ws
bash tools/validation/run_sdk_mapping_inertia.sh /tmp/d1max-control-sensitivity
```

The fixture integrates the robot state from the controller's mapped commands,
an assumed deadzone, delayed input and first-order inertia. It does not copy
demand into measured velocity. `summary.csv`, `trace.csv`, `source_hashes.txt`
and the test XML identify the input/model/source. It excludes Native SCAN
reseed and therefore cannot establish complete navigation success; production
Native/follower regressions cover that reseed contract separately. Failed
extreme-response cases are retained, not turned into physical acceptance.

The frozen before/after run in `experiments/controller_inertia_20261013` uses
the same model definitions and initial conditions, with state evolving from
each controller's own output. It is not an identical recorded-pose replay.

| Assumed response | Before terminal/damping correction | 10-13 experimental control |
| --- | --- | --- |
| Short-delay model, mapped arrival cases | 8/8 | 8/8 |
| Larger-body .8 s inertia / .2 s delay, mapped arrival cases | 8/8 | 8/8 |
| Larger-body 90° translate/turn and pure-yaw physical sign reversals | 1 / 1 | 0 / 0 |
| High-gain 1.8 s inertia / .6 s delay, peak physical XY speed | .400 m/s | .352 m/s |
| Same high-gain model, translate/turn and pure-yaw sign reversals | 8 / 6 | 2 / 3 |

Across all 80 arrival cases (both mappings, five assumed responses), 42 still
do not complete in the sensitivity fixture. That fixture has no Native SCAN
reseed; strict deadzones and an assumed 24% friction threshold additionally
expose uncalibrated response. The high-gain physical speed still exceeds the
.30 m/s SI demand limit. Thus neither the 30% input cap nor passing protocol
tests proves a physical .30 m/s speed bound or universal non-oscillatory control.
Native integration tests separately exercise genuine curve proof, measured
terminal/reentry requests, the .15–.20 m short connection, fresh curve commit
and resumed tracking, including an obstacle blocking that connection.

An initial goal already within tolerance with **no previously accepted curve**
is outside this terminal-reseed coverage. It must not be represented as having
passed full navigation/stop acceptance. This change repairs an existing committed
curve's recovery; it does not add a second task manager or manufacture a zero-
length SCAN trajectory for the unverified initial-goal case.

The body provider's original consumption deadline (at most 100 ms) is carried
unchanged. Derived command and task leases can only cap that deadline, never
extend it; Native selects a causal lease issued before the command's source.
This does not refresh original timestamps, extend a native verdict, or relax
the latest state, collision, emergency-stop or control-ownership checks.
Delayed moving handoffs compare geometry to a same-source historical witness
and independently verify the **latest measured** candidate entry; normal
motion is not compared to an old fixed entry point. A rejected candidate leaves
the incumbent untouched while its actual state/proof remain usable.

This design borrows measured-state closed-loop control and bounded velocity
smoothing from [Nav2 Humble Controller Server](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp)
and [Velocity Smoother](https://github.com/ros-navigation/navigation2/blob/humble/nav2_velocity_smoother/src/velocity_smoother.cpp).
It is not unmodified Nav2 RPP: RPP's vx/wz controller does not implement the D1
holonomic lateral motion contract. Pure production-core simulations exercise
tracking; they are not physical D1 speed/braking or navigation acceptance.

### Same-input comparison (2026-10-11)

The frozen old core and current production core were compiled against one
identical measured-input fixture. This is a controller regression, not bag-driven
navigation or physical timing evidence.

| Input | Frozen old core | Current core |
|---|---|---|
| On a bend, zero lateral tracking error, native tangent speed 0.253180 m/s | vx 0.269414, vy 0.006534 m/s | vx 0.253180, vy 0 m/s |
| Local 1 m curve, actual progress 0.82 m, task goal farther away | waiting and braking at vx 0.182500 m/s | tracking at vx 0.217500 m/s |
| Moving entry acquired 125 ms earlier, newest measured body on same curve | duplicate 100 ms budget rejects | configured 200 ms source budget accepts; actual body remains independently fresh |

20 Hz ideal response fixtures cover straight, curved, lateral, reverse and
simultaneous translate/turn tracking. Typed tests cover moving writer-ACK
replacement with preserved vx/vy/wz history, bad-candidate retention, cancellation,
occupied feedback and expired proof. Exact SDK geometry fixtures additionally
reject forged acquisition poses, changed same-ID geometry and stale actual state.
See `experiments/controller_20hz_20261011` for frozen inputs, comparison XML,
build logs and passive regression results. No ROS graph or SDK connection was
started for these checks.

The SDK may cache immutable geometry for the next previewed task before its
grant arrives. This cache grants no movement: a successful grant prunes other
task identities, and prepared motion still needs the exact current task,
permit, native proof and measured entry. This accounts for SCAN's event-driven
trajectory publication; a second task must not depend on a fortunate re-publish.

## Contract

All topic names are ROS parameters. `session_id` and `map_version_id` must be
explicit. These contracts do not bypass runtime/physical execution acceptance.

| Parameter | Default | Contract |
|---|---|---|
| task_topic | `/d1max/pct_scan/task` | String JSON at 5 Hz, explicit navigation task only |
| trajectory_topic | `/d1max/pct_scan/planning/tagged_bspline` | d1max_planning_interfaces/TaggedBspline |
| local_state_enabled | `false` | Compatibility default for old isolated fixtures; formal sessions enable independent local transport |
| local_navigation_state_topic | `/d1max/localization/navigation/local_state` | source-bound continuous local state, independent of asynchronous global corrections |
| navigation_state_topic | `/d1max/localization/navigation/state` | legacy same-source pair, subscribed only when local_state_enabled is false |
| progress_topic | `/d1max/pct_scan/planning/tracking_progress` | typed measured current-curve progress, original body source stamp |
| command_topic | `/d1max/live_planning/execution/command_debug` | debug-only body SI Twist, not motion authority |
| controller_frequency_hz | `20.0` | steady wall timer; measured progress, not clock-driven position |
| diagnostics_frequency_hz | `5.0` | bounded JSON rate, with immediate phase changes |
| command_pipeline_bound_s | record-bound | original command deadline; formal session projects the hashed timing record |
| writer_period_s | `0.05` | existing 20 Hz SDK timing record; used with the controller period to schedule proof handover, not to extend evidence |
| status_topic | `/d1max/pct_scan/tracker_status` | String JSON with generation, reason and command |
| stop_service | `/d1max/pct_scan/tracker_stop` | Trigger: stop and revoke current generation |

Task JSON: `{"session_id":"...","generation":1,"active":true,
"issued_at":<ROS seconds>,"frame_id":"d1max_loc_odom",
"target_xyz":[x,y,body_z],"control_identity":{...}}`. The identity requires
schema_version=2, task_id, route_id, route_hash, segment_id, map_version_id,
anchor_id/revision, context_sequence, localization_epoch and seed.
`issued_at`, identity and target are immutable for all
heartbeats of a generation. A new target requires a strictly newer generation.
Stop JSON also requires the same complete control_identity. Same-generation
heartbeats cannot undo cancel, completion or a stale-task timeout. A planner
must carry the generation captured from its **accepted reference path**, not
tag its output with whichever task is currently newest.

The default local body contract is header.frame_id=`d1max_loc_odom` and
child_frame_id=`d1max_loc_base_link`. Both paired odometries and source_stamp
must match exactly. Session/map/epoch/seed are carried by the source, never
copied from the task. Original posterior and IMU evidence ages are checked
independently of predicted output time; a fresh publish cannot lease stale
measurements. Progress twist is rotated from child frame into local odom.
Lidar pose is not a body pose; global and local poses cannot be relabeled.
The task target
has body Z (ground plus body height), not ground Z. The single-floor controller
uses XY and yaw and does not command vertical motion or stairs.
Tagged trajectories must explicitly carry `point_reference=body_center`;
ground-path coordinates cannot silently become a body trajectory.
Goal position uses XY 0.20 m and body Z 0.15 m; formal task completion belongs to
the BT with its actual stopped-state witness. Formal execution requires verified
floor/general support, slope/step checks and body height relative to that support.
The historical no-support core fixtures retain a first-body absolute Z envelope
(default 0.25 m, ceiling 0.30 m); this is not the formal supported runtime rule.
Neither check proves stair capability: the task owner must reject stairs and
cross-floor execution until those capabilities are separately accepted.

## Safety and limitations

- Formal configuration: total XY demand 0.30 m/s, yaw demand 0.50 rad/s,
  XY acceleration 0.35 m/s², yaw acceleration 0.80 rad/s²; signed X/Y translation is enabled by the
  default `holonomic` profile. The legacy `forward_yaw` option keeps Y zero.
- Stationary re-entry and turn-before-drive use the session's accepted record:
  `stationary_linear_threshold_mps` / `stationary_angular_threshold_radps`
  (defaults 0.03 m/s / 0.05 rad/s; ceilings 0.05 / 0.10),
  `stationary_reentry_duration_s` (0.6–5 s) and
  `stationary_minimum_samples` (3–512). SDK handoff retains original MC timing,
  zero-write ACK and owner identity checks. Turn phases count new original
  posterior/IMU evidence, not predicted headers, repeated messages or timer
  ticks. A fresh abnormal sample or interrupted proof resets the stable window.
  These settings do not change legacy exact-state splice bounds and do not authorize
  unmeasured braking, geometry or SDK capabilities.
- Task timeout 0.75 s, odometry timeout 0.40 s, local trajectory timeout 5 s;
  timestamp and receipt clocks both checked. Stale/malformed input stops.
- Reject session/generation/frame mismatch, repeated/out-of-order trajectory
  IDs, pre-task trajectories, nonfinite arrays, wrong knot counts, repeated
  knots, future starts and expired trajectories before vendor evaluation.
- Cancel/timeout/goal completion cannot resume from a delayed task heartbeat.
- A local spline endpoint is not a global goal: it stops awaiting replan.
- More than 2 m planar tracking error or 0.5 m vertical mismatch stops instead
  of chasing a disconnected trajectory or accidentally treating stairs as flat.
- Upstream collision protection remains mandatory: this tracker is not an
  obstacle detector or an independent full-robot collision checker.
- Acceleration limits apply during normal tracking; faults emit immediate
  zero and the downstream motor watchdog remains independent.
- Valid same-generation replacement splines retain linear **and** angular
  limiter history; replanning does not reset the command to the first ramp
  tick. Replacement is committed only after validation. Invalid candidates do
  not clear an incumbent accepted by SCAN. A newer negative SCAN collision
  conclusion, a real SCAN input/pipeline fault, expired measured state, cancel
  or clock discontinuity still stops output. The formal task chain does not
  stop merely because an individual map proof has aged.
- No dynamic speed guarantee is implied by clamping commanded speed.
- TrackingProgress.arc_length and s_committed belong to the accepted local
  spline, reset on trajectory replacement and do not claim global-route
  progress. Cross-segment route progress still needs its own XYZ proof.

Unit tests run without ROS discovery or a robot. ROS launch and robot arming
are owned by the session manager, not this package.
Regression coverage includes valid-replan continuity, invalid-candidate retention,
current-proof stop, both signs of 0.30 m / 0.70 m parallel offsets, bounded forward-only
kinematic convergence under repeated replans, turn-then-drive recovery, and
existing session/frame/time/height safeguards. These kinematic checks do not
establish physical speed mapping, collision clearance or robot stability.
