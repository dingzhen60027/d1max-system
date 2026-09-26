# Genuine per-sensor rays, projected using one map alignment and acquisition-time
# local pose interpolation. Not a free-space mask or an authorization to move.
string session_id
uint64 epoch
string seed_id
uint64 context_sequence
uint64 barrier_ns
uint64 projection_sequence
# rays.header.stamp remains the first acquisition time; never reception time.
builtin_interfaces/Time acquisition_end
# One map_from_odom alignment, derived from an exact-stamp pose pair, for all rays.
builtin_interfaces/Time alignment_stamp
# Version-1 64-byte ray schema documented by d1max_localization/PERCEPTION_RAYS.md.
# XYZ and origin_XYZ are in rays.header.frame_id (map); all source fields unchanged.
sensor_msgs/PointCloud2 rays
