#pragma once

#include <bspline_opt/reference_path.hpp>
#include <string>

namespace scan_planner {

struct ReferenceTargetOptions {
  double forward_margin{1.0};
  double backward_margin{2.0};
  double min_advance{0.3};
  double exit_margin{0.2};
  double sample_step{0.04};
  std::size_t max_queries{4096};
};

struct ReferenceTargetResult {
  bool valid{false};
  double arc{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  std::size_t queries{0};
  std::size_t free_queries{0}, unknown_queries{0}, occupied_queries{0}, outside_queries{0};
  bool has_first_blocked{false};
  Eigen::Vector3d first_blocked{Eigen::Vector3d::Zero()};
  int first_blocked_value{0};
  // Actual endpoint-query locations, not inferred obstacles or a path proof.
  // Bound diagnostic work independently of the unchanged selector query cap.
  static constexpr std::size_t max_blocked_points{256};
  std::vector<Eigen::Vector3d> blocked_points;
  std::string reason{"failed_reference_target_occupied"};

  void observe(const Eigen::Vector3d &position, int value) {
    ++queries;
    if (value == 0) { ++free_queries; return; }
    if (value < 0) ++outside_queries;
    else if (value == 2) ++unknown_queries;
    else ++occupied_queries;
    if (!has_first_blocked) {
      has_first_blocked=true; first_blocked=position; first_blocked_value=value;
    }
    if (blocked_points.size() < max_blocked_points) blocked_points.push_back(position);
  }
};

// Select only points on the original, ordered reference. This is an endpoint
// selector, NOT a proof that the intervening route is collision-free: native
// projected A*, rebound optimization and full-trajectory validation still run.
// occupancy: negative = outside the rolling map; zero = existing map policy's
// free value (not a guarantee of observed ground support); positive = blocked.
// Native value 2 denotes unknown support; its classification is diagnostic only.
// Never search beyond the first sampled out-of-map point and re-enter elsewhere.
template <class Occupancy>
ReferenceTargetResult selectReferenceTarget(
    const DiscreteReference &reference, const Eigen::Vector3d &actual_start,
    double progress, double horizon, const ReferenceTargetOptions &options,
    Occupancy occupancy) {
  ReferenceTargetResult result;
  if (!actual_start.allFinite() || !std::isfinite(progress) ||
      !std::isfinite(horizon) || horizon <= 0.0 || reference.length() <= 0.0 ||
      !std::isfinite(options.forward_margin) || options.forward_margin < 0.0 ||
      !std::isfinite(options.backward_margin) || options.backward_margin < 0.0 ||
      !std::isfinite(options.min_advance) || options.min_advance < 0.2 ||
      !std::isfinite(options.exit_margin) || options.exit_margin < 0.0 ||
      !std::isfinite(options.sample_step) || options.sample_step <= 0.0 ||
      options.max_queries < 2) {
    result.reason = "failed_reference_geometry";
    return result;
  }
  progress = std::clamp(progress, 0.0, reference.length());
  const double nominal = std::min(reference.length(), progress + horizon);
  const double upper = std::min(reference.length(), nominal + options.forward_margin);
  // A short genuine final leg is permitted; a return to the current body is not.
  const double minimum = std::min(reference.length(), progress + options.min_advance);
  const double lower = std::max(minimum, nominal - options.backward_margin);
  result.arc = nominal;
  result.point = reference.sample(nominal);
  const double count = std::ceil((upper - progress) / options.sample_step);
  if (count + 5 > static_cast<double>(options.max_queries)) {
    result.reason = "failed_reference_search_budget";
    return result;
  }
  std::vector<double> samples{progress, nominal, upper, lower};
  for (int i = 1; i < static_cast<int>(count); ++i)
    samples.push_back(progress + (upper - progress) * i / count);
  // Include corners exactly, retaining stair landings and other piecewise Z.
  for (const double arc : reference.arc)
    if (arc > progress && arc < upper) samples.push_back(arc);
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(),
      [](double a, double b) { return std::abs(a-b) < 1e-9; }), samples.end());
  if (samples.size() > options.max_queries) {
    result.reason = "failed_reference_search_budget";
    return result;
  }
  std::vector<double> available;
  double free_begin=std::numeric_limits<double>::quiet_NaN();
  for (const double arc : samples) {
    if (arc < progress || arc > upper) continue;
    const auto point = reference.sample(arc);
    const Eigen::Vector3d tangent = reference.sample(std::min(reference.length(), arc + options.sample_step)) -
                                    reference.sample(std::max(0.0, arc - options.sample_step));
    const double yaw = std::atan2(tangent.y(), tangent.x());
    const int value = occupancy(point, yaw);
    result.observe(point, value);
    if (value < 0) break;
    if (value != 0) { free_begin=std::numeric_limits<double>::quiet_NaN(); continue; }
    if (!std::isfinite(free_begin)) free_begin=arc;
    // Do not end on the first free voxel immediately after an obstruction:
    // the detour initializer needs a free departure segment, not just one point.
    const bool free_approach=arc-free_begin+1e-9>=options.exit_margin;
    // 0.2 m matches the manager's existing no-hover-as-navigation boundary.
    if (free_approach && arc >= lower - 1e-9 && arc > progress + 1e-6 &&
        (point - actual_start).norm() > 0.2 + 1e-9)
      available.push_back(arc);
  }
  if (available.empty()) return result;
  // Keep a free nominal endpoint. If blocked, prefer the next free point after
  // the obstacle, within the configured margin; only then shorten the horizon.
  auto forward = std::lower_bound(available.begin(), available.end(), nominal - 1e-9);
  result.arc = forward == available.end() ? available.back() : *forward;
  result.point = reference.sample(result.arc);
  result.valid = true;
  result.reason = std::abs(result.arc - nominal) < 1e-8 ? "reference_target_nominal" :
      (result.arc > nominal ? "reference_target_forward" : "reference_target_shortened");
  return result;
}

}  // namespace scan_planner
