#pragma once
#include <array>
#include <cstdint>
#include <memory>

namespace scan_planner {
struct ProjectedRayContext;
// Component liveness only: never an occupancy verdict or per-command map
// certificate. Sources are original acquisition BEGIN stamps accepted by the
// independent ingress lane, not the older stamps in a borrowed map snapshot.
struct NativePipelineHealth {
  std::shared_ptr<const ProjectedRayContext> context;
  std::array<std::int64_t,2> source_ns{{0,0}};
  std::uint64_t fusion_sequence{0};
  std::int64_t fusion_completed_steady_ns{0},sampled_steady_ns{0};
  bool clock_fault{false};
};
} // namespace scan_planner
