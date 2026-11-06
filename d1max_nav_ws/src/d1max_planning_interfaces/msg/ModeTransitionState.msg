# Exact response to one adjacent route portal request, from the sole SDK writer.
uint8 schema_version
ExecutionVersion version
string request_id
uint64 request_sequence
uint64 sequence
string target_segment_id
string target_mode
string transport_mode
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
string phase
string reason
bool confirmed
bool capability_verified
# SDK RobotState has no vendor timestamp. Do not treat this as sensor source time.
string time_basis
uint64 robot_callback_sequence
builtin_interfaces/Time robot_callback_stamp
builtin_interfaces/Time source_stamp
builtin_interfaces/Time valid_until
int32 sdk_mode
int32 motion_status
int32 speed_level
bool control_owned
bool emergency_clear

