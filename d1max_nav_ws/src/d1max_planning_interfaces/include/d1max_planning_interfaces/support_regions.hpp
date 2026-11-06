#pragma once
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <set>
#include <string>

namespace d1max_planning_interfaces {
// This is source ownership, not a gait capability or a second collision test.
// Existing flat/single-stair packets retain their original path unchanged.
inline bool validStairTransitionRegions(const msg::SupportReference& s) {
  if(s.reason!="original_transition_surface_returns_associated" ||
      s.regions.size()<2 || s.regions.size()>16 ||
      s.max_support_slope_rad!=0. || s.max_support_step_m!=0.)return false;
  const auto& first=s.regions.front();
  if(first.segment_id!=s.version.segment_id || first.floor_id!=s.floor_id ||
      first.segment_kind!=s.segment_kind || first.required_mode!=s.required_mode)return false;
  std::size_t next=0;bool stairs=false;std::set<std::string> ids;
  for(const auto& r:s.regions) {
    if(r.segment_id.empty() || !ids.insert(r.segment_id).second ||
        r.begin_index!=next || r.end_index<=r.begin_index ||
        r.end_index>s.support_ground_xyz.size())return false;
    const bool flat=r.required_mode=="general" &&
      (r.floor_id=="floor1"||r.floor_id=="floor2") &&
      (r.segment_kind=="floor"||r.segment_kind=="landing");
    const bool stair=r.required_mode=="stair" &&
      (r.floor_id=="stairs"||r.floor_id=="stair_lower"||r.floor_id=="stair_upper") &&
      (r.segment_kind=="stair_up"||r.segment_kind=="stair_down"||r.segment_kind=="landing");
    if(!flat&&!stair)return false;
    stairs|=stair;next=r.end_index;
  }
  return stairs && next==s.support_ground_xyz.size();
}
} // namespace d1max_planning_interfaces
