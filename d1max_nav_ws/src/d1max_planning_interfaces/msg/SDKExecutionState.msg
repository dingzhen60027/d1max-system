# The one SDK writer acknowledges its bound grant asynchronously.
ExecutionVersion version
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
uint64 sequence
builtin_interfaces/Time source_stamp
bool grant_ready
bool control_owned
bool general_low_speed_confirmed
# Literal active locomotion mode. General flag above keeps its original meaning.
string motion_mode
bool motion_mode_confirmed
bool mode_transition_active
bool fault_latched
string transport_mode
string phase
string reason
uint64 writer_commit_sequence
int64 applied_trajectory_id
