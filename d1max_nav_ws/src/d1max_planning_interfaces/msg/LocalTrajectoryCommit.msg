# Sole local executor's geometry fact. This does NOT grant movement or assert
# that SDK applied a command. Expected active pair fences late candidates.
uint32 schema_version
ExecutionVersion version
int64 trajectory_id
ExecutionVersion expected_version
int64 expected_trajectory_id
string proposal_id
uint64 commit_sequence
builtin_interfaces/Time source_stamp
builtin_interfaces/Time body_source_stamp
geometry_msgs/Pose measured_pose
geometry_msgs/Twist measured_twist
float64 curve_time
float64 arc_length
uint64 native_validation_sequence
uint64 map_geometry_revision
bool accepted
bool retired
string disposition
string reason
string transport_mode
