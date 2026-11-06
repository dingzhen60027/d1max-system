# Single-writer software retirement evidence, NOT physical-stop evidence.
# Published only for this exact admitted grant; source_stamp is observation
# time, not an invented MC acquisition timestamp. Late commands retain their
# original identities and cannot revive a retired grant.
uint32 schema_version
ExecutionVersion version
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
string grant_request_id
uint64 sequence
builtin_interfaces/Time source_stamp
string transport_mode
bool nonzero_blocked
bool software_retired
bool motion_never_submitted
uint32 pending_sdk_transactions
string reason
