# SDK input mapping

Source changes, the default installed bundle and physical navigation acceptance
are separate facts; the navigation entry validates its pinned installed bundle. The
operator reports that Move inputs below approximately 20% barely move the
robot, 20% is slow and 30% is normal on forward/backward, lateral and yaw axes.
These observations are not a measured percentage-to-m/s or rad/s calibration.

The runtime profile uses parameter basis
`vendor_nominal_engineering_and_operator_observation`. Older validators reject
that basis instead of silently ignoring the new `sdk_input_mapping`. The
mapping has schema 1, kind `smooth_deadzone_ratio_v1` and normalization
`dominant_axis_direction_preserving_norm_cap`. Each of `planar` and `yaw` has
`effective_fraction=0.20`, `maximum_fraction=0.30`,
`blend_demand_ratio=0.10`, `rise_rate_per_s=0.30`, `fall_rate_per_s=0.45`.
The existing maximum planar and yaw SI demands provide the reference demand;
they are not an assertion of actual speed achieved by this mapping.

For normalized demand ratio `r`, the scalar response is
`effective*smoothstep(r/blend)` through the blend point and
`effective+(maximum-effective)*smoothstep((r-blend)/(1-blend))` above it,
with `smoothstep(t)=t*t*(3-2*t)` and `r` clipped to `[0,1]`. The scalar shape
is C1 and retains exact zero. Planar demand uses its combined XY magnitude for
the ratio, scales the XY direction by its dominant axis, then scales both SDK
fractions together if their combined norm exceeds 30%. This puts a 45-degree
starting demand near 20% on each axis; at saturation it is about 21.2% each.
Mixed-axis response is uncalibrated. Dominant-axis boundaries and the hard norm
cap can introduce derivatives' corners, so the complete 2D mapping is continuous
but is not claimed to be globally C1. Yaw uses its own odd-symmetric response.

The unique SDK writer limits the change of the SDK XY vector and yaw input.
Direction preservation describes the static target; while slewing from the
previous input, the SDK vector can temporarily differ from the newest demand's
direction. This includes residual fractions on ordinary zero components.
Elapsed steady-clock time is capped at 50 ms, so a delayed tick cannot make up
missed steps. Ordinary zero components ramp down at the falling rate while
another component remains requested. Complete original zero demands bypass
slew and clear history immediately; they are the existing HOLD/STOP outputs.
A sign reversal reduces that SDK input to zero before accepting the opposite
sign. Passing zero input does not prove physical standstill and adds no task
gate or wait-for-standstill policy. Task-bound execution identity, control epoch,
arm generation, no-command, fault and explicit reset clear the history. Curve
changes retain it.

Only `task_follower_v1` uses this mapping. A curve-v3 writer cannot start with a
profile containing it; legacy profiles retain their existing linear mapping.
Raw SI commands and raw measurements remain unchanged. Actual write zero/nonzero
classification uses the final float fractions passed to `Move`, not the original
SI demand. A smoothing tick that sends zero is not a nonzero motion attempt;
its ACK still does not prove physical standstill. The
existing low-rate diagnostic reports the last original demand, last attempted
SDK fractions and mapping kind. Input shaping is not a physical velocity,
acceleration, braking or collision guarantee. No physical verification flags
are changed, and no new collision check, task gate, PI loop or integral is added.
