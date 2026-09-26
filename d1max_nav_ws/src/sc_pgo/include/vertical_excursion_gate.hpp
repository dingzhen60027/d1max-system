#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace sc_pgo {

// Raw, uninterrupted LIO positions from the historical loop keyframe through
// the current keyframe, in one gravity-aligned odometry frame.
struct VerticalPoseSample {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct VerticalExcursionGateOptions {
  double max_xy_window_m = 20.0;
  double min_excursion_evidence_m = 1.5;
  double max_vertical_disagreement_m = 1.0;
};

enum class VerticalExcursionGateReason {
  accepted,
  height_disagreement,
  invalid_input,
};

struct VerticalExcursionGateResult {
  bool accepted = false;
  VerticalExcursionGateReason reason = VerticalExcursionGateReason::invalid_input;
  double max_ascent_m = 0.0;
  double max_descent_m = 0.0;
  double excursion_evidence_m = 0.0;
  double raw_relative_z_m = 0.0;
  double vertical_disagreement_m = 0.0;
};

// For each endpoint, find the lowest and highest preceding z within a sliding
// window of XY path length. This finds local stair/elevator-like excursions
// without assuming that the floor itself is level. Each sample enters and
// leaves each monotonic deque once: O(N) time and O(N) memory.
//
// icp_relative_world_z_m is the registered current-to-history BODY transform's
// translation rotated into the same gravity-aligned world frame as the raw
// trajectory. Callers must do that frame conversion before invoking this gate.
inline VerticalExcursionGateResult evaluateVerticalExcursionGate(
  const std::vector<VerticalPoseSample>& trajectory,
  double icp_relative_world_z_m,
  const VerticalExcursionGateOptions& options = {})
{
  VerticalExcursionGateResult result;
  if (trajectory.size() < 2 || !std::isfinite(icp_relative_world_z_m) ||
      !std::isfinite(options.max_xy_window_m) || options.max_xy_window_m <= 0.0 ||
      !std::isfinite(options.min_excursion_evidence_m) ||
      options.min_excursion_evidence_m <= 0.0 ||
      !std::isfinite(options.max_vertical_disagreement_m) ||
      options.max_vertical_disagreement_m <= 0.0) {
    return result;
  }

  std::vector<double> xy_distance;
  xy_distance.reserve(trajectory.size());
  std::deque<std::size_t> minima;
  std::deque<std::size_t> maxima;
  double cumulative_xy_m = 0.0;
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const auto& point = trajectory[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return VerticalExcursionGateResult{};
    }
    if (i > 0) {
      const auto& previous = trajectory[i - 1];
      const double xy_step = std::hypot(point.x - previous.x, point.y - previous.y);
      // A gap larger than the complete window cannot establish whether an
      // ascent or descent happened between the two samples.
      if (!std::isfinite(xy_step) || xy_step > options.max_xy_window_m) {
        return VerticalExcursionGateResult{};
      }
      cumulative_xy_m += xy_step;
      if (!std::isfinite(cumulative_xy_m)) return VerticalExcursionGateResult{};
    }
    xy_distance.push_back(cumulative_xy_m);
    while (!minima.empty() &&
           cumulative_xy_m - xy_distance[minima.front()] > options.max_xy_window_m) {
      minima.pop_front();
    }
    while (!maxima.empty() &&
           cumulative_xy_m - xy_distance[maxima.front()] > options.max_xy_window_m) {
      maxima.pop_front();
    }
    if (!minima.empty()) {
      const double ascent = point.z - trajectory[minima.front()].z;
      if (!std::isfinite(ascent)) return VerticalExcursionGateResult{};
      result.max_ascent_m = std::max(result.max_ascent_m, ascent);
    }
    if (!maxima.empty()) {
      const double descent = trajectory[maxima.front()].z - point.z;
      if (!std::isfinite(descent)) return VerticalExcursionGateResult{};
      result.max_descent_m = std::max(result.max_descent_m, descent);
    }
    while (!minima.empty() && trajectory[minima.back()].z >= point.z) minima.pop_back();
    minima.push_back(i);
    while (!maxima.empty() && trajectory[maxima.back()].z <= point.z) maxima.pop_back();
    maxima.push_back(i);
  }

  result.raw_relative_z_m = trajectory.back().z - trajectory.front().z;
  result.excursion_evidence_m = result.max_ascent_m - result.max_descent_m;
  // The endpoint difference includes long-term LIO height drift. Once a
  // concentrated climb/descent is observed, compare registration against
  // that local evidence instead of the drift-contaminated endpoint delta.
  result.vertical_disagreement_m =
    std::abs(icp_relative_world_z_m - result.excursion_evidence_m);
  if (!std::isfinite(result.raw_relative_z_m) ||
      !std::isfinite(result.excursion_evidence_m) ||
      !std::isfinite(result.vertical_disagreement_m)) {
    return VerticalExcursionGateResult{};
  }
  result.accepted =
    std::abs(result.excursion_evidence_m) < options.min_excursion_evidence_m ||
    result.vertical_disagreement_m <= options.max_vertical_disagreement_m;
  result.reason = result.accepted ? VerticalExcursionGateReason::accepted :
    VerticalExcursionGateReason::height_disagreement;
  return result;
}

}  // namespace sc_pgo
