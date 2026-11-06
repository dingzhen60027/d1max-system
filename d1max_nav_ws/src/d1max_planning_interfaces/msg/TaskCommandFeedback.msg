# Passive fact from the sole SDK writer, NOT physical motion/stop evidence,
# collision clearance, a task grant, or permission to renew any source time.
uint32 schema_version
ExecutionVersion version
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
string transport_mode
uint64 write_sequence
# Original Move call event, not publication/receipt time.
builtin_interfaces/Time source_stamp
bool write_submitted
bool command_selected
uint64 command_sequence
builtin_interfaces/Time command_source_stamp
builtin_interfaces/Time command_body_source_stamp
builtin_interfaces/Time command_valid_until
# Selected SI intent and exact float Move arguments are different quantities.
geometry_msgs/Twist velocity
bool sdk_arguments_available
bool sdk_nonzero
float32 sdk_forward
float32 sdk_lateral
float32 sdk_yaw
# Sticky within this execution: one watermark per real forced-zero gap, not
# per zero tick. Latest-only delivery must retain a missed zero transition.
uint64 discontinuity_write_sequence
builtin_interfaces/Time discontinuity_stamp
uint64 interrupted_command_sequence
builtin_interfaces/Time interrupted_command_source_stamp
builtin_interfaces/Time interrupted_command_valid_until
string reason
