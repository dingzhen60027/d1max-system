# D1 Max SCAN trajectory tracker

The source-bound core and formal schema-3 ROS transport compile and have offline
regression coverage. The node participates in the BT-owned proposal, admission,
permit and writer-applied handoff graph; it is not an SDK actuation endpoint.
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

The controller caches actual vendor-spline XYZ arc length (bounded to 6000
intervals) at admission and searches only a bounded local arc window on each
new source-stamped body sample. It permits 0.15 m physical backtracking and a
bounded forward window, retaining a separate curve-local high water mark.
Ties and repeated positions do not advance progress. A large projection jump
outside the window revokes the curve rather than ratcheting toward a branch.
The lease still uses the original trajectory start and receipt timestamps:
spatial progress never refreshes stale data. Search work is independent of
the spline's total duration. `lookahead` remains seconds of spline parameter,
not a distance in metres.

Asynchronous replacements carry a measured join into the **original full
spline**, not a translated or re-timed copy. `valid_start_time` and
`valid_start_arc_length` identify that join; `join_source_stamp`, pose and
world-frame twist retain their source evidence. Admission independently
recomputes the full-curve arc and verifies position within 1.25 cm, velocity
within 0.05 m/s, and complete derivative control-hull speed/acceleration
bounds. A newer body sample is projected only within the source-time bounded
same-curve window. A valid join can begin at nonzero curve time and preserves
both command limiters. Time passage alone never advances that point.
No acceleration measurement exists on the current source contract, so native
`join_acceleration_valid` remains false: this does not claim measured C2
continuity. A stale, cross-context or failed join does not replace an already
accepted curve; explicit revocation and stale-current-data handling remain
separate.

The formal node prepares spline samples, derivatives, support bins and the
curvature/braking envelope on a **single bounded worker**. There is at most
one running job, one latest pending job and one completion; newer input and
cancel retire older work synchronously. The worker receives immutable inputs
and never accesses the live controller. The single control owner polls the
result, checks its exact geometry/configuration identity, re-observes the
actual entry against current source-bound state and proof, and retains the
existing writer-applied CAS protocol. No preparation grants permission or
renews a proof. Candidate failure leaves the incumbent intact; a real expired
or negative safety proof still stops output. Same-ID geometry cannot be
rewritten, and changed support contents invalidate cached preparation even
when a producer erroneously reuses a hash.

Local-state ingress orders original epoch and nanosecond source time before
decoding usability or interpreting a fault. Retired epochs and duplicate or
late observations do not erase fresh state or refresh recovery. A newer
expected epoch reaches hard-reset handling even when its clock moved back;
foreign session/map/interface packets are discarded before fault handling;
obviously future samples cannot poison the stream watermark. A fresh
source fault still enters hold, and its source barrier prevents an earlier
valid sample from clearing that fault. Source leases and the three-distinct-
sample/0.6-second recovery window are unchanged.

These changes remove heavy preparation from the 50 Hz owner lane; they do
not certify end-to-end 20 ms deadlines. Owner admission still performs bounded
actual-entry/support queries and input comparisons. CPU-pressure/NUC timing,
full ROS scheduling and physical navigation remain separate acceptance work.

The reference robot can correct error with sideways velocity. D1 Max execution
is currently forward-only, so both feed-forward and **cross-track positional
feedback steer yaw**. A large heading error first requests a bounded turn;
after alignment it drives toward the path. The frozen signal indicates heading
or cross-track recovery; recovery movement can continue without advancing an
artificial trajectory clock. At the local endpoint feed-forward is removed,
and stopping there never reports completion of a different global goal.

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
| frozen_topic | `/d1max/pct_scan/execution_frozen` | std_msgs/Bool, remap to SCAN freeze input |
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

- Defaults: 0.30 m/s, 0.50 rad/s; hard planar limit 1.5 m/s; y is always zero.
- Stationary re-entry and turn-before-drive use the session's accepted record:
  `stationary_linear_threshold_mps` / `stationary_angular_threshold_radps`
  (defaults 0.03 m/s / 0.05 rad/s; ceilings 0.05 / 0.10),
  `stationary_reentry_duration_s` (0.6–5 s) and
  `stationary_minimum_samples` (3–512). SDK handoff retains original MC timing,
  zero-write ACK and owner identity checks. Turn phases count new original
  posterior/IMU evidence, not predicted headers, repeated messages or timer
  ticks. A fresh abnormal sample or interrupted proof resets the stable window.
  These settings do not change the 0.05 m/s spline join bound and do not authorize
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
  not clear a fresh, still-certified incumbent. A stale/negative current proof,
  expired measured state, cancel or clock discontinuity still stops output.
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
