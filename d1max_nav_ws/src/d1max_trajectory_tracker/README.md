# D1 Max SCAN trajectory tracker

This package only publishes body-frame SI commands and status. It never contacts
the SDK, changes posture, takes control, or arms the robot. In the live stack the
task coordinator admits the status-tagged command before the collision monitor
and navigation command gate; the raw `Twist` is not a direct actuation endpoint.

It links the vendored `bspline_opt::UniformBspline` implementation. `order` is
the polynomial degree (SCAN emits 3); explicit knots are preserved and relative
time is evaluated with `evaluateDeBoorT`, i.e. shifted by knot[order]. Position
and derivative therefore have exactly SCAN's semantics. Like the reference
`legbot_3D_Nav` adapter, progress follows the measured nearest point and a
lookahead, not elapsed execution time. No Go2 motor controller is launched.
Reference reviewed: [a1_cmd_adapter.cpp at f60606a](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/a1_cmd_adapter.cpp).

Unlike the reference adapter's whole-spline search at 0.01 s increments, this
controller searches only forward from its previous projection, within the
configured `lookahead` window (default 0.8 s, hard ceiling 2 s), using 40 chord
projections per tick. Progress cannot move backward; ties do not advance it.
The lease still uses the original trajectory start and receipt timestamps:
spatial progress never refreshes stale data. Search work is independent of
the spline's total duration. `lookahead` remains seconds of spline parameter,
not a distance in metres.

The reference robot can correct error with sideways velocity. D1 Max execution
is currently forward-only, so both feed-forward and **cross-track positional
feedback steer yaw**. A large heading error first requests a bounded turn;
after alignment it drives toward the path. The frozen signal indicates heading
or cross-track recovery; recovery movement can continue without advancing an
artificial trajectory clock. At the local endpoint feed-forward is removed,
and stopping there never reports completion of a different global goal.

## Contract

All topic names are ROS parameters. `session_id` is mandatory at startup.

| Parameter | Default | Contract |
|---|---|---|
| task_topic | `/d1max/pct_scan/task` | String JSON at 5 Hz, explicit navigation task only |
| trajectory_topic | `/d1max/pct_scan/planning/tagged_bspline` | d1max_planning_interfaces/TaggedBspline |
| odom_topic | `/localization/odometry/global` | nav_msgs/Odometry, body pose in planning frame |
| command_topic | `/d1max/pct_scan/cmd_vel_raw` | body SI Twist, forward only, no SDK |
| frozen_topic | `/d1max/pct_scan/execution_frozen` | std_msgs/Bool, remap to SCAN freeze input |
| status_topic | `/d1max/pct_scan/tracker_status` | String JSON with generation, reason and command |
| stop_service | `/d1max/pct_scan/tracker_stop` | Trigger: stop and revoke current generation |

Task JSON: `{"session_id":"...","generation":1,"active":true,
"issued_at":<ROS seconds>,"frame_id":"d1max_loc_map",
"target_xyz":[x,y,body_z]}`. `issued_at` and target are immutable for all
heartbeats of a generation. A new target requires a strictly newer generation.
Stop JSON only needs session_id, generation and active=false. Same-generation
heartbeats cannot undo cancel, completion or a stale-task timeout. A planner
must carry the generation captured from its **accepted reference path**, not
tag its output with whichever task is currently newest.

The default odometry contract is header.frame_id=`d1max_loc_map` and
child_frame_id=`d1max_loc_base_link`. Lidar pose is not a body pose, and a local
odom-frame pose must be transformed upstream, not relabeled. The task target
has body Z (ground plus body height), not ground Z. The single-floor controller
uses XY and yaw and does not command vertical motion or stairs.
Completion requires both XY within 0.20 m and body Z within 0.15 m. Each task
generation pins its first fresh measured body height; targets, measured body
height and complete spline control hull must remain in the bounded single-floor
envelope (default 0.25 m, configuration ceiling 0.30 m). A spline's complete
control-hull height range is also bounded by that envelope. Violations terminate
the generation. These checks do not classify stairs: the task coordinator must
explicitly reject stairs/cross-floor routes before granting an execution lease.

## Safety and limitations

- Defaults: 0.30 m/s, 0.50 rad/s; hard planar limit 1.5 m/s; y is always zero.
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
  tick. Replacement is committed only after validation. Invalid replacement,
  stale inputs, cancel and clock discontinuity still stop immediately.
- No dynamic speed guarantee is implied by clamping commanded speed.

Unit tests run without ROS discovery or a robot. ROS launch and robot arming
are owned by the session manager, not this package.
Regression coverage includes valid-replan continuity, invalid-replan immediate
stop, both signs of 0.30 m / 0.70 m parallel offsets, bounded forward-only
kinematic convergence under repeated replans, turn-then-drive recovery, and
existing session/frame/time/height safeguards. These kinematic checks do not
establish physical speed mapping, collision clearance or robot stability.
