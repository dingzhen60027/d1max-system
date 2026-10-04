# Irreversible software selection/call outcome from the sole writer, NOT
# physical motion proof. Once called, uncertain/error outcomes retain the new
# applied identity and HOLD; they can never roll back to the incumbent.
uint8 schema_version
string handoff_id
uint64 grant_sequence
uint64 sequence
uint64 previous_commit_sequence
uint64 commit_sequence
ExecutionVersion incumbent_version
ExecutionVersion candidate_version
int64 incumbent_trajectory_id
int64 candidate_trajectory_id
string execution_id
string sdk_session
string transport_mode
uint64 control_epoch
uint64 sdk_arm_generation
uint64 permit_sequence
uint64 demand_sequence
uint64 motion_validation_sequence
uint64 entry_admission_sequence
builtin_interfaces/Time applied_at
builtin_interfaces/Time demand_source_stamp
# Original controller/prepared-boundary source, never restamped by the writer.
builtin_interfaces/Time demand_body_source_stamp
builtin_interfaces/Time entry_source_stamp
# Latest actual local state used by the writer's final entry check.
builtin_interfaces/Time body_source_stamp
builtin_interfaces/Time valid_until
geometry_msgs/PoseStamped measured_pose
geometry_msgs/Twist measured_twist
geometry_msgs/Twist applied_velocity
float64 curve_time
bool applied
# applied is an irreversible writer software commit, never physical motion proof.
# submitted/acknowledged distinguish a local write call from its vendor callback.
bool write_submitted
bool write_acknowledged
string reason
