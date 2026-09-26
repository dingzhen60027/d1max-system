# Live SCAN execution admission contract

`live_scan_bridge` defaults to `execution_mode=preview`. Preview preserves the
existing frozen-clock sensor gate and never forwards a validated spline. The
explicit `execution` mode admits evidence for a separate motion coordinator;
the bridge has no SDK calls, arm service or velocity publisher.

In execution mode `/d1max/live_planning/execution_frozen` must have exactly one
publisher. Its fully qualified node name must equal `execution_tracker_node`
(default `/d1max/live_planning/motion_coordinator`). Either Bool value is valid,
but its local receipt must remain within `input_timeout`. The coordinator owns
this feedback: it publishes true while disarmed/paused and forwards tracker
feedback only when actual execution is allowed. The tracker should publish its
own feedback on a separate topic.

The two new volatile, reliable, depth-one outputs are:

- `/d1max/live_planning/validated_tagged_bspline`, `TaggedBspline`. An unchanged
  native message is forwarded once after matching current session/generation,
  frame, increasing trajectory ID, original source time, accepted native debug
  with the same plan ID, and complete PCT support validation. Emergency/hover
  splines without accepted debug are not forwarded. No timestamp is refreshed.
- `/d1max/live_planning/execution_admission`, `std_msgs/String` containing JSON
  schema 1. Admission is evidence, not an arm or motion authorization.

JSON fields are `schema`, `session_id`, `execution_mode`, `generation`,
`trajectory_id`, `valid`, `reason`, `sequence`, `frame_id`, `issued_at`,
`source_stamp`, `debug_stamp`, `stamp`, `received_at_unix`, `lease_timeout_sec`,
`localization_context`, and `motion_authorized` (always false).

`generation` is owned by the accepted `scan_reference`/ReferenceGate and is not
the global worker generation. `issued_at` is its original reference issue time,
`source_stamp` the admitted spline's original start time, `debug_stamp` its
accepted debug's source time, `stamp` publication ROS time, and
`received_at_unix` publication wall time. `sequence` strictly increases within
one bridge process. `localization_context` is `[session_id, epoch, seed_id]` or
null. A valid message identifies the admitted spline, not a newer pending
candidate. While invalid, trajectory ID may be -1 and source times may be zero
or debug time null; they are diagnostic, never permission.

The status is published on state transitions and every 0.2 seconds; consumers
must check source/publication time, monotonic local receipt and the advertised
0.35-second `lease_timeout_sec`, as well as their own task/session/frame/context.
The admitted spline and accepted debug each expire after 2 seconds according to
both their unchanged source time and original local receipt. Heartbeats cannot
extend these evidence lifetimes. Cross-topic delivery may reorder either the
two input messages or the two output messages; consumers must pair identities.

A previous admitted record remains usable within its original lease while a
new valid candidate waits for its matching message. A newly completed pair
replaces it atomically. Explicit current-plan failure, malformed current
geometry/debug, support rejection, input/context loss, reference cancellation,
shutdown, or evidence expiry publishes `valid=false`; foreign/obsolete packets
cannot erase a newer plan. Consumers must stop on false/stale admission and must
not infer authority from RViz markers. Their operator task issue time can be
newer than reference issue time, so they must additionally require a newly
produced spline whose original start time follows that task issue time.

This is still centreline support and native planner admission, not foot-placement
or swept-body certification. The motion coordinator must enforce explicit
single-floor scope, localization/SDK readiness, task cancellation and arming.
