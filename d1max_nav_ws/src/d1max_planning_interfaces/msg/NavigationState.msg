# Exact-paired local/global body state; state prediction time and input
# measurement times are distinct and must never be renewed from receipt time.
uint32 schema_version
string session_id
string map_version_id
uint64 localization_epoch
string localization_seed_id
nav_msgs/Odometry local_odometry
nav_msgs/Odometry global_odometry
builtin_interfaces/Time source_stamp
builtin_interfaces/Time posterior_stamp
builtin_interfaces/Time imu_stamp
float64 extrapolation_sec
bool usable
string reason
