# Task authority. Only ExecutionVersion's stable task/route/map/localization
# fields are authoritative; reference, anchor and curve revisions are NOT.
uint32 schema_version
ExecutionVersion version
string execution_id
uint64 control_epoch
string confirmation_id
string sdk_session
uint64 sdk_arm_generation
uint64 sequence
builtin_interfaces/Time source_stamp
builtin_interfaces/Time valid_until
bool allowed
bool revoked
string phase
string reason
string transport_mode
