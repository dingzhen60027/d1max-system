# Map-derived support geometry; absence is not flat-ground evidence.
ExecutionVersion version
string support_reference_id
string support_hash
string support_map_sha256
string floor_id
string segment_kind
string required_mode
string frame_id
builtin_interfaces/Time source_stamp
geometry_msgs/Point[] support_ground_xyz
# Exact indices into support_ground_xyz qualified by the map package's
# independently classified literal stair-contact evidence. Empty means no
# contact qualification; ordinary support provenance alone grants none.
# Sorted, unique, and bound into support_hash when nonempty. This is source
# geometry, not permission to clear occupancy or ignore a chassis collision.
uint32[] stair_contact_indices
# Separate literal original-map observations near the stair risers. These are
# NOT ground support and do not enter height/attitude fitting. SCAN must still
# prove the observed face against adjacent qualified treads; ROI membership
# alone never permits contact. Empty means no independent riser evidence.
# Parallel arrays, <=20000 rows. Owner is a regions index, or 0 for a single
# stair flight; only stair_up/stair_down owners are valid. Both arrays are
# transformed by the same anchor and bound into support_hash when nonempty.
geometry_msgs/Point[] stair_riser_xyz
uint32[] stair_riser_owner
float64 support_xy_radius_m
float64 body_reference_height_m
# Legacy flat-floor geometry limits. Both zero for stair source associations:
# no physical step/slope capability is certified by this static map packet.
float64 max_support_slope_rad
float64 max_support_step_m
bool verified
string reason
# Empty for the established single-segment contract. A stair-transition window
# keeps its measured current segment above, and explicitly labels each source
# region here; future stairs must never masquerade as flat floor support.
SupportRegion[] regions
# Independent bounded stair geometry. Empty preserves the original contract.
# The model is a planning prior, not proof that every cell is observed/free.
# Up to 64 surfaces and 256 finite obstacles, bound into support_hash and the
# same execution version/coordinate anchor as the literal observations above.
# Transmission is independent of the literal observation 'verified' flag.
string stair_model_hash
StairSurfaceModel[] stair_model_surfaces
StairBoundaryModel[] stair_model_obstacles
