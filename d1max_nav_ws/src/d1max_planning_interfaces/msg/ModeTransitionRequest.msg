# Adjacent route portal request. Only the existing SDK writer may dispatch SetMode.
uint8 schema_version
ExecutionVersion version
string request_id
uint64 sequence
string target_segment_id
string current_mode
string target_mode
string transport_mode
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
builtin_interfaces/Time source_stamp
builtin_interfaces/Time valid_until
builtin_interfaces/Time body_source_stamp
geometry_msgs/Point portal_body_source_xyz
# True only for a session created without a motion pipeline. Never SDK permission.
bool preview_only

