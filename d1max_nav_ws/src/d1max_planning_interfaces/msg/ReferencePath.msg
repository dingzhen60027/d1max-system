string session_id
uint64 generation
nav_msgs/Path path
uint32 schema_version
# ground: apply configured support-to-body height once; body_center: offset must be zero.
string point_reference
string task_id
string route_id
string route_hash
string map_version_id
string segment_id
string segment_kind
string required_mode
uint64 anchor_revision
uint64 map_geometry_revision
string anchor_id
uint64 context_sequence
uint64 localization_epoch
string localization_seed_id
# Same length/order as path.poses for schema 2; empty only for cancel or legacy preview.
string[] point_segment_ids
string[] point_segment_kinds
string[] point_required_modes
