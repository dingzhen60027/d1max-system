# The reference manager's disposition of one exact RouteReference delivery.
# Neither ACCEPTED nor RETIRED is motion authority or physical stop evidence.
uint8 ACCEPTED=1
uint8 REJECTED=2
uint8 RETIRED=3
uint32 schema_version
builtin_interfaces/Time source_stamp
builtin_interfaces/Time receipt_stamp
string session_id
string task_id
string route_id
string route_hash
uint64 delivery_sequence
bool active
uint8 status
bool native_retirement_required
bool native_retirement_confirmed
uint64 native_generation
string reason
