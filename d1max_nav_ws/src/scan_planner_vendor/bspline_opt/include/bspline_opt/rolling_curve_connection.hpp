#pragma once

namespace scan_planner {

// Shared geometry-follow connection contract. These are not permissions to
// skip collision checks: Native SCAN validates the actual body-to-curve
// connector, and the follower keeps the immutable curve and measured state.
// The numerical origin tolerance only verifies a newly solved curve's source;
// it is deliberately separate from a moving robot's connection distance.
struct RollingCurveConnectionPolicy {
  static constexpr double max_connector_m = .15;
  static constexpr double max_forward_arc_m = .5;
  static constexpr double max_source_gap_s = .4;
  static constexpr double numerical_origin_tolerance_m = .0125;
};

} // namespace scan_planner
