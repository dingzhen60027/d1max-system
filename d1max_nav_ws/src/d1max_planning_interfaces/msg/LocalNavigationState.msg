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
# Schema 2: provider-issued consumption deadline, <= source_stamp + 100 ms.
# The provider verifies raw IMU/posterior at source_stamp; consumers verify
# this original deadline, not raw IMU age again at their delivery time.
# Repeated delivery MUST NOT extend it. Unusable events may leave it zero.
builtin_interfaces/Time valid_until
float64 extrapolation_sec
bool usable
string reason
