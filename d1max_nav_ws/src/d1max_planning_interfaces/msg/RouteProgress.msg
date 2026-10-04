# Source-bound projection on the immutable global route. Supervisory progress
# only: never a collision proof, SDK permission, new lease, or arrival evidence.
uint32 schema_version
ExecutionVersion version
uint64 sequence
string frame_id
string transport_mode
builtin_interfaces/Time body_source_stamp
# Literal original local body pose at body_source_stamp, in continuous odom.
geometry_msgs/PoseStamped odom_body_pose
# The accepted ReferenceCore's fixed source-map anchor, not the latest TF.
builtin_interfaces/Time anchor_source_stamp
geometry_msgs/Pose map_from_odom
geometry_msgs/Point source_map_body_xyz
float64 body_reference_height_m
# Arc is measured in the full fixed source-map ground route, not a local spline.
uint32 edge_index
float64 measured_arc_m
float64 confirmed_arc_m
float64 cross_track_m
