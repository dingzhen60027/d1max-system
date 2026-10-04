# Sole task-owner lease. Zero/expired/replayed source time never renews it.
ExecutionVersion version
string execution_id
uint64 control_epoch
string confirmation_id
string sdk_session
uint64 sdk_arm_generation
uint64 sequence
builtin_interfaces/Time source_stamp
# Owner authority lease only; NOT a collision-evidence deadline. Consumers
# independently require the actual native curve/motion proof's original TTL.
builtin_interfaces/Time valid_until
int64 trajectory_id
# Minimum native proof sequence at signing. The same complete version/curve
# may use a fresher proof >= this floor, but MotionDemand must identify its
# exact proof. A newer explicit invalid verdict cannot be bypassed by history.
uint64 validation_sequence
string frame_id
geometry_msgs/Point goal_position
bool allowed
bool geometry_committed
bool revoked
bool has_goal_yaw
float64 goal_yaw
float64 goal_yaw_tolerance_rad
# preview, arming, tracking, holding, aligning, stopping, terminal
string phase
# live or isolated_mock; a live consumer MUST reject isolated_mock.
string transport_mode
string reason
