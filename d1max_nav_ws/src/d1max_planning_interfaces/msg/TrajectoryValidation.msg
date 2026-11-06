# SCAN collision decision; never task permission to actuate.
# Formal task_follower_v1: ordered state for one exact curve. A completed
# positive/negative decision persists until a newer completed decision or
# explicit retirement. While a new check runs, the existing state is retained.
ExecutionVersion version
string proposal_id
int64 trajectory_id
uint64 sequence
builtin_interfaces/Time source_stamp
builtin_interfaces/Time check_begin
builtin_interfaces/Time check_end
# Formal task_follower_v1 MUST use zero (state, not a map-proof lease).
# Nonzero is the historical curve_writer_v3 timed-evidence contract, which
# must not be mixed into a formal local-follower session. Original acquisition
# and check stamps below remain unchanged for diagnosis, not lease renewal.
builtin_interfaces/Time valid_until
builtin_interfaces/Time body_source_stamp
builtin_interfaces/Time front_ray_source_stamp
builtin_interfaces/Time rear_ray_source_stamp
uint64 map_snapshot_revision
string support_reference_id
string support_hash
bool valid
bool whole_curve
# Only an already committed curve may be checked from measured progress back
# by reverse_margin_m through its real end. This never certifies a new curve.
bool remaining_curve
float64 checked_from_time
float64 checked_to_time
float64 curve_duration
float64 reverse_margin_m
bool goal_yaw_checked
float64 checked_goal_yaw
float64 valid_start_time
float64 valid_start_arc_length
string frame_id
# observed_free: raw evidence across the checked curve envelope (legacy).
# official_inflated_geometry: native SCAN configured robot collision geometry.
# Formal task_follower_v1 has no additional command/braking voxel sweep.
# SCAN owns collision and input/pipeline failure transitions; SDK still owns
# current command/task expiry, control ownership and emergency stop.
string collision_policy
string transport_mode
string reason
TrajectorySplice splice
