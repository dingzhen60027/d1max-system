#pragma once

#include <Eigen/Core>
#include <d1max_planning_interfaces/msg/local_plan_debug.hpp>
#include <std_msgs/msg/header.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace scan_planner {

// This bound affects diagnostics only. The optimizer always receives the full
// unmodified reference even if a diagnostic message cannot be emitted.
constexpr std::size_t kMaxLocalDebugReferencePoints = 4096;

inline d1max_planning_interfaces::msg::LocalPlanDebug makeInvalidLocalPlanDebug(
    const std_msgs::msg::Header &header, const std::string &session_id,
    std::uint64_t generation, std::uint64_t plan_id, const std::string &phase) {
  d1max_planning_interfaces::msg::LocalPlanDebug message;
  message.header = header;
  message.session_id = session_id;
  message.generation = generation;
  message.plan_id = plan_id;
  message.phase = phase;
  message.selected_reference.header = header;
  return message;
}

inline std::optional<d1max_planning_interfaces::msg::LocalPlanDebug> makeAcceptedLocalPlanDebug(
    const std_msgs::msg::Header &header, const std::string &session_id,
    std::uint64_t generation, std::uint64_t plan_id,
    const Eigen::Vector3d &projection, const Eigen::Vector3d &target,
    double progress_arc_m, double target_arc_m,
    const std::vector<Eigen::Vector3d> &actual_selected_reference) {
  if (header.frame_id.empty() || session_id.empty() ||
      !projection.allFinite() || !target.allFinite() ||
      !std::isfinite(progress_arc_m) || !std::isfinite(target_arc_m) ||
      progress_arc_m < 0.0 || target_arc_m < progress_arc_m ||
      actual_selected_reference.size() < 2 ||
      actual_selected_reference.size() > kMaxLocalDebugReferencePoints ||
      (actual_selected_reference.back() - target).norm() > 1e-5) {
    return std::nullopt;
  }

  auto message = makeInvalidLocalPlanDebug(header, session_id, generation, plan_id, "accepted");
  message.valid = true;
  message.projection.x = projection.x();
  message.projection.y = projection.y();
  message.projection.z = projection.z();
  message.local_target.x = target.x();
  message.local_target.y = target.y();
  message.local_target.z = target.z();
  message.progress_arc_m = progress_arc_m;
  message.target_arc_m = target_arc_m;
  message.selected_reference.poses.reserve(actual_selected_reference.size());
  for (const auto &point : actual_selected_reference) {
    if (!point.allFinite()) return std::nullopt;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    pose.pose.orientation.w = 1.0;
    message.selected_reference.poses.push_back(std::move(pose));
  }
  return message;
}

}  // namespace scan_planner
