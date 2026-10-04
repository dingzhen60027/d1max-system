#pragma once
#include "d1max_navigation_bt/task_contract.hpp"
#include <d1max_navigation_bt_interfaces/msg/route_snapshot.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>

namespace d1max_navigation_bt {
inline bool sha256String(const std::string& value) {
  return value.size() == 64 && std::all_of(value.begin(), value.end(), [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}
// Producer owns canonical content hashing; this consumer freezes its typed
// payload and validates all identities/shape before accepting it. Path is only
// a compatibility view and must not override the typed route.
inline std::string validateSnapshot(const d1max_navigation_bt_interfaces::msg::RouteSnapshot& s,
    const RouteBinding& task, std::int64_t epoch, const std::string& seed,
    const std::string& frame, const std::string& map_version, const nav_msgs::msg::Path& legacy) {
  if (s.schema_version != task_schema_version) return "route_schema_mismatch";
  if (s.session_id != task.session || s.task_id != task.task || s.route_id.empty() ||
      s.localization_epoch != epoch || s.localization_seed_id != seed ||
      s.map_version_id.empty() || s.map_version_id != map_version) return "route_snapshot_identity_mismatch";
  if (!sha256String(s.route_hash) || !sha256String(s.source_map_sha256) ||
      !sha256String(s.tomogram_sha256) || !sha256String(s.conditioning_sha256)) return "route_provenance_hash_invalid";
  const auto n = s.path.poses.size();
  if (!s.preview_ready || s.frame_id != frame || s.path.header.frame_id != frame ||
      s.point_reference != "ground" || n < 2 || n > 20000 ||
      s.layer_ids.size() != n || s.source_layer_ids.size() != n ||
      s.point_floor_ids.size() != n || s.edge_segments.size() != n-1 ||
      s.segments.empty() || s.segments.size() > 64 || s.geometry_evidence_json.empty() ||
      s.geometry_evidence_json.size() > 8000000) return "route_snapshot_shape_invalid";
  if (s.has_goal_yaw && (!std::isfinite(s.goal_yaw) || !std::isfinite(s.goal_yaw_tolerance_rad) ||
      s.goal_yaw_tolerance_rad < .01 || s.goal_yaw_tolerance_rad > .5)) return "route_goal_yaw_invalid";
  if (legacy.header.frame_id != s.path.header.frame_id || legacy.poses.size() != n) return "legacy_route_disagrees_with_snapshot";
  for (std::size_t i=0; i<n; ++i) {
    const auto& pose = s.path.poses[i]; const auto& p = pose.pose.position;
    const auto& q = pose.pose.orientation;
    const auto norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::abs(p.x)>10000 || std::abs(p.y)>10000 || std::abs(p.z)>10000 ||
        !std::isfinite(norm) || norm < .999 || norm > 1.001 ||
        (!pose.header.frame_id.empty() && pose.header.frame_id != frame) || s.point_floor_ids[i].empty()) return "route_point_invalid";
    if (pose.pose != legacy.poses[i].pose) return "legacy_route_disagrees_with_snapshot";
  }
  std::unordered_set<std::string> ids;
  std::size_t previous_end=0;
  for (const auto& segment : s.segments) {
    if (segment.segment_id.empty() || !ids.insert(segment.segment_id).second ||
        segment.begin_index != previous_end || segment.end_index <= segment.begin_index ||
        segment.end_index >= n || segment.floor_id.empty() ||
        segment.source_layer_id != s.source_layer_ids[segment.begin_index] ||
        segment.target_layer_id != s.source_layer_ids[segment.end_index] ||
        (segment.kind != "floor" && segment.kind != "stair_up" && segment.kind != "stair_down" &&
         segment.kind != "landing" && segment.kind != "unknown") ||
        (segment.required_mode != "general" && segment.required_mode != "stair" && segment.required_mode != "unverified"))
      return "route_segment_invalid";
    if (s.execution_eligible && (!segment.execution_eligible || segment.required_mode == "unverified" ||
        segment.kind == "unknown")) return "route_eligibility_inconsistent";
    for (std::size_t edge=segment.begin_index; edge<segment.end_index; ++edge)
      if (s.edge_segments[edge] != segment.segment_id) return "route_edge_segment_mismatch";
    previous_end = segment.end_index;
  }
  return previous_end == n-1 ? "" : "route_segments_incomplete";
}
}  // namespace d1max_navigation_bt
