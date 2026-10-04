# Measured controller progress, never elapsed wall-clock trajectory time.
# header.stamp is the original body acquisition stamp in the continuous odom frame.
std_msgs/Header header
uint32 schema_version
string session_id
string task_id
string route_hash
uint64 anchor_revision
uint64 generation
int64 trajectory_id
string route_id
string segment_id
string map_version_id
uint64 localization_epoch
string localization_seed_id
string anchor_id
uint64 context_sequence
float64 curve_time
float64 arc_length
# Both arc fields are local to trajectory_id, not task/global-route distance.
# Measured arc can briefly retreat; this is the same-spline high-water mark.
float64 s_committed
bool valid
string reason
geometry_msgs/Pose pose
# Twist and acceleration use header.frame_id axes at the body reference point.
geometry_msgs/Twist twist
geometry_msgs/Accel acceleration
bool acceleration_valid
bool holding
