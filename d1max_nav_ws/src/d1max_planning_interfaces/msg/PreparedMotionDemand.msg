# Candidate command. Only the unique writer's successful CAS may apply it.
uint8 schema_version
string handoff_id
uint64 grant_sequence
uint64 expected_commit_sequence
MotionDemand demand
TrajectoryAdmission entry_admission
# Real body acquisition, and the corresponding production-spline projection.
# All vectors below are expressed at the body reference in continuous odom.
geometry_msgs/PoseStamped measured_pose
geometry_msgs/Twist measured_twist
builtin_interfaces/Time entry_source_stamp
geometry_msgs/Pose curve_entry_pose
geometry_msgs/Twist curve_entry_twist
float64 curve_time
float64 position_error_m
float64 velocity_error_mps
# Diagnostic only: receivers enforce their own configured upper bounds.
float64 position_tolerance_m
float64 velocity_tolerance_mps
