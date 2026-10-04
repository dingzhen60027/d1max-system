# Report time, not a replacement for any sensor/odometry measurement timestamp.
uint32 schema_version
builtin_interfaces/Time stamp
string session_id
string map_version_id
bool ready
bool hard_fault
string reason
int64 localization_epoch
string localization_seed_id
# Source evidence, not report/receipt time. Capabilities have distinct users:
# startup/Compute/new anchor need global; incumbent odom control needs local.
builtin_interfaces/Time evidence_source_stamp
bool local_control_ready
bool global_planning_ready
