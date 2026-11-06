# Writer-owned stop evidence. Submission/retirement are not measured stop.
ExecutionVersion version
string execution_id
uint64 control_epoch
string sdk_session
uint64 sdk_arm_generation
string stop_request_id
uint64 sequence
# Original accepted MC host receipt / capture upper bound; zero before the
# first accepted sample. Not a report heartbeat or the approximate mapped SDK
# clock point. Mapped time remains in SDKStationaryEvidence and MC telemetry.
# Stop duration is proved with the conservative capture lower bound.
builtin_interfaces/Time source_stamp
bool nonzero_blocked
bool stop_submitted
bool measured_stop_confirmed
bool physical_acceptance_verified
string mc_clock_epoch
uint64 mc_raw_stamp_ns
uint32 stationary_samples
float64 stationary_duration_sec
float64 measured_linear_mps
float64 measured_angular_radps
# Original 3D linear/angular norms above are never replaced by a planar norm.
# Empty is legacy spatial_norm_3d. Planar requires a fresh confirmed runtime
# profile mode: GENERAL/low, or fixed STAIR explicitly using general_runtime_reuse.
# XY and absolute body-Z must EACH satisfy the configured linear threshold.
string velocity_metric
float64 measured_planar_mps
# Absolute body-Z speed, not a height or a discarded measurement.
float64 measured_vertical_mps
string time_basis
string transport_mode
string reason
