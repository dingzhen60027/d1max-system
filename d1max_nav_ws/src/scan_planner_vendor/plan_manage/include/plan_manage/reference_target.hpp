#pragma once

#include <bspline_opt/reference_path.hpp>
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace scan_planner {

struct FinalEndpointReseed {
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  double goal_xy_tolerance{.20},goal_z_tolerance{.15};
  bool actual_and_endpoint_supported{false};
  bool measured_connection_reseed{false};
  double connector_bound_m{.15};
};
struct ReferenceTargetOptions {
  double forward_margin{1.0};
  double backward_margin{2.0};
  double min_advance{0.3};
  double exit_margin{0.2};
  double sample_step{0.04};
  std::size_t max_queries{4096};
  std::optional<FinalEndpointReseed> final_endpoint_reseed;
  // Preserve the historical flat selector. A source-backed stair search can
  // use exact native reference anchors with a checked incident edge instead
  // of assuming its final body heading is the distant start-to-target chord.
  // This only selects an endpoint; it cannot certify the connector or curve.
  bool require_chord_heading{true};
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
  double first_blocked_yaw{0.};
  // Actual endpoint-query locations, not inferred obstacles or a path proof.
  // Bound diagnostic work independently of the unchanged selector query cap.
  static constexpr std::size_t max_blocked_points{256};
  std::vector<Eigen::Vector3d> blocked_points;
  std::string reason{"failed_reference_target_occupied"};

  void observe(const Eigen::Vector3d &position, int value,double yaw=0.) {
    ++queries;
    if (value == 0) { ++free_queries; return; }
    if (value < 0) ++outside_queries;
    else if (value == 2) ++unknown_queries;
    else ++occupied_queries;
    if (!has_first_blocked) {
      has_first_blocked=true; first_blocked=position; first_blocked_value=value;
      first_blocked_yaw=yaw;
    }
    if (blocked_points.size() < max_blocked_points) blocked_points.push_back(position);
  }
};

// The measured body is prepended only as a connector for local slices. Route
// progress begins at the first route waypoint: when a new window arrives while
// the body is off-centerline (e.g. mid box detour), the lateral connector is
// not route and must not become progress or a "shortened" target that pulls
// the body back beside the obstacle. Returns the route-begin arc.
inline double setReferenceFromBody(DiscreteReference &reference, const Eigen::Vector3d &body,
                                   const std::vector<Eigen::Vector3d> &waypoints) {
  if (waypoints.empty()) throw std::invalid_argument("reference needs route waypoints");
  std::vector<Eigen::Vector3d> full{body};
  full.insert(full.end(), waypoints.begin(), waypoints.end());
  reference.set(full);
  // set() drops a connector shorter than 1e-6; the norm is then equally ~0.
  return std::min(reference.length(), (waypoints.front() - body).norm());
}

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
  // At the true task endpoint, measured overshoot has no forward route arc.
  // Reconnect from the real body to that SAME endpoint only. This selects a
  // solve target; existing A*, optimization and full Native proof still decide
  // whether a connector can be committed and followed.
  if(options.final_endpoint_reseed&&progress>=reference.length()-1e-6) {
    const auto& final=*options.final_endpoint_reseed;
    const auto endpoint=reference.sample(reference.length());
    if(!final.goal.allFinite()||!std::isfinite(final.goal_xy_tolerance)||
       final.goal_xy_tolerance<.02||final.goal_xy_tolerance>.30||
       !std::isfinite(final.goal_z_tolerance)||final.goal_z_tolerance<.01||final.goal_z_tolerance>.20) {
      result.reason="failed_reference_geometry";return result;
    }
    if(!std::isfinite(final.connector_bound_m)||final.connector_bound_m<=0.||final.connector_bound_m>.15) {
      result.reason="failed_reference_geometry";return result;
    }
    if((endpoint-final.goal).norm()<=1e-4&&
       ((endpoint-actual_start).head<2>().norm()>final.goal_xy_tolerance||
        std::abs(endpoint.z()-actual_start.z())>final.goal_z_tolerance||
        (final.measured_connection_reseed&&(endpoint-actual_start).norm()>final.connector_bound_m+1e-9))) {
      result.arc=reference.length();result.point=endpoint;
      if(!final.actual_and_endpoint_supported)return result;
      const auto tangent=(endpoint-reference.sample(std::max(0.,reference.length()-options.sample_step))).eval();
      const double yaw=std::atan2(tangent.y(),tangent.x());
      const int value=occupancy(endpoint,yaw);result.observe(endpoint,value,yaw);
      if(value!=0)return result;
      const auto approach=(endpoint-actual_start).head<2>().eval();
      if(options.require_chord_heading&&approach.norm()>1e-6) {
        const double approach_yaw=std::atan2(approach.y(),approach.x());
        if(std::abs(std::remainder(approach_yaw-yaw,2.*M_PI))>1e-6) {
          const int value=occupancy(endpoint,approach_yaw);result.observe(endpoint,value,approach_yaw);
          if(value!=0)return result;
        }
      }
      result.valid=true;result.reason="reference_target_final_endpoint_reseed";return result;
    }
  }
  // Exhaustion is missing forward reference, not an occupied target. A side
  // search cannot reconstruct the next semantic segment and must not hide
  // this condition behind waiting_side_target. The true final-goal reconnect
  // above remains available and still undergoes native collision checking.
  if(progress>=reference.length()-1e-6) {
    result.arc=reference.length();result.point=reference.sample(result.arc);
    result.reason="waiting_reference_extension";
    if(options.final_endpoint_reseed&&
       (result.point-options.final_endpoint_reseed->goal).norm()<=1e-4)
      result.reason="waiting_goal_reached";
    return result;
  }
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
    result.observe(point, value,yaw);
    if (value < 0) break;
    if (value != 0) { free_begin=std::numeric_limits<double>::quiet_NaN(); continue; }
    if (!std::isfinite(free_begin)) free_begin=arc;
    // Do not end on the first free voxel immediately after an obstruction:
    // the detour initializer needs a free departure segment, not just one point.
    const bool free_approach=arc-free_begin+1e-9>=options.exit_margin;
    // 0.2 m matches the manager's existing no-hover-as-navigation boundary.
    if (!(free_approach && arc >= lower - 1e-9 && arc > progress + 1e-6 &&
          (point - actual_start).norm() > 0.2 + 1e-9)) continue;
    // Legacy flat search requires both headings. Source-backed stair search
    // uses the existing exact-anchor incident-edge contract: the robot can
    // turn on the landing before entering a narrow flight, so a distant
    // start-to-target chord need not be a physical endpoint orientation.
    // Route tangent occupancy above is always checked; native A*, optimizer
    // and complete curve/rotation checks still own the actual connection.
    const Eigen::Vector2d approach = (point - actual_start).head<2>();
    if (options.require_chord_heading && approach.norm() > 1e-6) {
      const double approach_yaw = std::atan2(approach.y(), approach.x());
      if (std::abs(std::remainder(approach_yaw - yaw, 2.0 * M_PI)) > 1e-6) {
        const int endpoint = occupancy(point, approach_yaw);
        result.observe(point, endpoint,approach_yaw);
        if (endpoint != 0) continue;
      }
    }
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
