# Published only by the localization owner processing the matching mailbox.
# applied means the seed was installed; it is NOT stable localization readiness.
uint32 schema_version
string session_id
string request_id
string map_version_id
builtin_interfaces/Time intent_source_stamp
builtin_interfaces/Time source_stamp
uint64 localization_epoch
string localization_seed_id
bool accepted
bool applied
string reason
