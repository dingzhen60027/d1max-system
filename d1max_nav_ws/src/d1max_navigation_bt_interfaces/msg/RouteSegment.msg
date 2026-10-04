# Inclusive point indices; adjacent segments may share a boundary point.
string segment_id
# floor, stair_up, stair_down, landing, or unknown. No implicit mode inference.
string kind
uint32 begin_index
uint32 end_index
int32 source_layer_id
int32 target_layer_id
string floor_id
string entry_portal_id
string exit_portal_id
# general, stair, or unverified; not a vendor SDK integer.
string required_mode
string direction
bool execution_eligible
string eligibility_reason
