# Native collision evidence; never permission to actuate.
ExecutionVersion version
string proposal_id
int64 trajectory_id
uint64 sequence
builtin_interfaces/Time source_stamp
builtin_interfaces/Time check_begin
builtin_interfaces/Time check_end
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
string collision_policy
string transport_mode
string reason
