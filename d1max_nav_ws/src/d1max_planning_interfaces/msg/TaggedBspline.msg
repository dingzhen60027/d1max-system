string session_id
uint64 generation
string frame_id
scan_planner_msgs/Bspline trajectory
uint32 schema_version
string point_reference
string task_id
string route_id
string route_hash
string map_version_id
string segment_id
string segment_kind
string required_mode
uint64 anchor_revision
uint64 map_geometry_revision
string anchor_id
uint64 context_sequence
uint64 localization_epoch
string localization_seed_id
# The original spline is not trimmed, translated or re-timed at adoption.
# These parameters identify its bounded, measured entry point. Arc length is
# local to this full spline from parameter zero, never global-route progress.
float64 valid_start_time
float64 valid_start_arc_length
builtin_interfaces/Time join_source_stamp
geometry_msgs/Pose join_pose
# World/planning-frame measured velocity, not the derivative labelled as odom.
geometry_msgs/Twist join_twist
geometry_msgs/Accel join_acceleration
# False when no trustworthy acceleration source exists; never claims C2 proof.
bool join_acceleration_valid
