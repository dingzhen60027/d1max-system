# Proof of the actual body-frame velocity reaction + braking reachable sweep.
# Not a path certificate and never execution permission. All stamps keep source time.
ExecutionVersion version
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
int64 trajectory_id
uint64 permit_sequence
# Actual curve evidence used by native, >= the originating demand floor.
# Cannot cross an intervening invalid verdict for that full version/curve.
uint64 trajectory_validation_sequence
uint64 demand_sequence
builtin_interfaces/Time demand_source_stamp
builtin_interfaces/Time demand_valid_until
builtin_interfaces/Time demand_body_source_stamp
builtin_interfaces/Time body_source_stamp
uint64 sequence
builtin_interfaces/Time front_ray_source_stamp
builtin_interfaces/Time rear_ray_source_stamp
uint64 map_snapshot_revision
builtin_interfaces/Time check_begin
builtin_interfaces/Time check_end
builtin_interfaces/Time valid_until
geometry_msgs/Twist velocity
string frame_id
bool valid
string reason
string transport_mode
# SHA256 of the exact original JSON record bytes, never canonicalized JSON.
string braking_model_sha256
# Empty for ordinary active-curve checks. Prepared checks bind the actual
# entry witness, not a candidate's unverified error/tolerance declaration.
string handoff_id
uint64 entry_admission_sequence
geometry_msgs/Pose entry_curve_pose
geometry_msgs/Twist entry_curve_twist
float64 entry_curve_time
float64 entry_position_error_m
float64 entry_velocity_error_mps
