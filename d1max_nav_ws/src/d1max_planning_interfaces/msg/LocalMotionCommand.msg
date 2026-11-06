# Native local executor's checked command; SDK does no curve/CAS admission.
# Original state deadline survives every transport hop.
uint32 schema_version
ExecutionVersion version
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
uint64 lease_sequence
uint64 sequence
# Unmodified controller echo; never synthesized from a new proof or source.
uint64 acknowledged_discontinuity_write
builtin_interfaces/Time source_stamp
builtin_interfaces/Time body_source_stamp
builtin_interfaces/Time body_valid_until
builtin_interfaces/Time valid_until
string body_frame
geometry_msgs/Twist velocity
bool hold
int64 trajectory_id
string reason
string transport_mode
