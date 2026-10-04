# SI body-frame command, bound to current permission AND collision proof.
ExecutionVersion version
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
uint64 sequence
uint64 permit_sequence
# Minimum same-version/same-curve evidence sequence, not a pin to an old
# snapshot. MotionValidation records its actual sequence >= this floor.
# valid_until remains the original control deadline; proof TTL is independent.
uint64 validation_sequence
# Filled only by the independent safety filter after matching native motion proof.
uint64 motion_validation_sequence
string braking_model_sha256
int64 trajectory_id
builtin_interfaces/Time source_stamp
builtin_interfaces/Time body_source_stamp
builtin_interfaces/Time valid_until
geometry_msgs/Twist velocity
bool hold
# Filled only by the independent raw-ray safety filter, never by Tracker.
bool safety_checked
builtin_interfaces/Time safety_source_stamp
string transport_mode
string reason
