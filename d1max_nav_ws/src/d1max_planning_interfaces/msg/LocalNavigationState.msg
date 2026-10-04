# Continuous local state, independent of an asynchronous global correction.
# usable=False revokes control; it carries no usable geometry. Never restamp a
# held pose. Epoch/seed are estimator-owned and bind an existing map identity.
uint32 schema_version
string session_id
string map_version_id
uint64 localization_epoch
string localization_seed_id
nav_msgs/Odometry local_odometry
builtin_interfaces/Time source_stamp
builtin_interfaces/Time posterior_stamp
builtin_interfaces/Time imu_stamp
float64 extrapolation_sec
bool usable
string reason
