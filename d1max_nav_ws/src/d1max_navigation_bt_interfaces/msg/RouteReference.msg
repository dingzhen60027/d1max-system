# Delivery identity is independent of immutable geometry identity. Cancellation
# retains the full snapshot; an empty Path is not an authorization protocol.
uint32 schema_version
builtin_interfaces/Time source_stamp
string session_id
string task_id
string route_id
string route_hash
uint64 delivery_sequence
bool active
RouteSnapshot snapshot
