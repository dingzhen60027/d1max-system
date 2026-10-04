# Genuine per-sensor rays, projected into map OR continuous local odom using acquisition-time
# local pose interpolation. Not a free-space mask or an authorization to move.
string session_id
uint64 epoch
string seed_id
uint64 context_sequence
uint64 barrier_ns
uint64 projection_sequence
# rays.header.stamp remains acquisition BEGIN; collision proof freshness uses
# this integrated BEGIN, not receipt time or the separately retained END below.
builtin_interfaces/Time acquisition_end
# One map_from_odom alignment, derived from an exact-stamp pose pair, for all rays.
builtin_interfaces/Time alignment_stamp
# Version-1 64-byte ray schema documented by d1max_localization/PERCEPTION_RAYS.md.
# XYZ and origin_XYZ are in rays.header.frame_id (map or odom); source fields unchanged.
sensor_msgs/PointCloud2 rays
