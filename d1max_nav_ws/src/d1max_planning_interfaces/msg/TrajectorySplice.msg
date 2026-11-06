# Candidate tail starts at a PLANNED boundary on the immutable incumbent.
# This boundary is never presented as a newly measured robot pose. The local
# executor switches only when real progress reaches it and actual entry agrees.
bool enabled
ExecutionVersion incumbent_version
int64 incumbent_trajectory_id
builtin_interfaces/Time measured_progress_stamp
float64 splice_curve_time
float64 splice_arc_length
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
bool acceleration_valid
