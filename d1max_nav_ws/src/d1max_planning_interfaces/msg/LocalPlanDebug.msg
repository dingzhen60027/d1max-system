# Read-only snapshot of the reference actually selected by the native local planner.
# valid=false always carries an empty selected_reference, including on cancellation.
std_msgs/Header header
string session_id
uint64 generation
uint64 plan_id
bool valid
string phase
geometry_msgs/Point projection
geometry_msgs/Point local_target
float64 progress_arc_m
float64 target_arc_m
nav_msgs/Path selected_reference
# Evidence for a bounded handover only. This is a fresh complete collision
# recheck of the preceding same-generation curve, never a renewed old lease.
uint64 predecessor_id
bool predecessor_safe
builtin_interfaces/Time predecessor_check_stamp
# Explicit no-motion native whole-curve revalidation, never a new spline or
# controller lease. Zero for ordinary acceptance. Source stamps are original
# acquisition stamps, not receipt/check time; header.stamp is check completion.
int64 checked_map_source_stamp_ns
int64 checked_body_source_stamp_ns
uint64 checked_map_revision
uint64 checked_context_sequence
