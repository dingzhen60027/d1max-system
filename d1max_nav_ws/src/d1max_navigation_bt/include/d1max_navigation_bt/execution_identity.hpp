#pragma once
#include <d1max_planning_interfaces/msg/execution_version.hpp>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <string>
namespace d1max_navigation_bt::exec3 {
using Version=d1max_planning_interfaces::msg::ExecutionVersion;
inline double seconds(const builtin_interfaces::msg::Time&t){return t.sec+t.nanosec*1e-9;}
inline builtin_interfaces::msg::Time stamp(double t){
  builtin_interfaces::msg::Time out;out.sec=static_cast<int32_t>(std::floor(t));
  out.nanosec=static_cast<uint32_t>(std::max(0.,(t-out.sec)*1e9));return out;
}
inline bool fresh(double source,double now,double age){
  return std::isfinite(source)&&std::isfinite(now)&&source>0&&now>=source&&now-source<=age;
}
inline builtin_interfaces::msg::Time earliestStamp(std::initializer_list<builtin_interfaces::msg::Time> values){
  return *std::min_element(values.begin(),values.end(),[](const auto&a,const auto&b){
    return a.sec<b.sec||(a.sec==b.sec&&a.nanosec<b.nanosec);
  });
}
inline bool sameTask(const Version&a,const Version&b){
  return a.schema_version==3&&b.schema_version==3&&a.session_id==b.session_id&&
    a.task_id==b.task_id&&a.route_id==b.route_id&&a.route_hash==b.route_hash&&
    a.map_version_id==b.map_version_id&&a.localization_epoch==b.localization_epoch&&
    a.localization_seed_id==b.localization_seed_id;
}
inline bool complete(const Version&a){
  return a.schema_version==3&&!a.session_id.empty()&&!a.task_id.empty()&&!a.route_id.empty()&&
    a.route_hash.size()==64&&!a.map_version_id.empty()&&a.localization_epoch>0&&
    !a.localization_seed_id.empty()&&a.reference_generation>0&&!a.segment_id.empty()&&
    !a.anchor_id.empty()&&a.anchor_revision>0&&a.context_sequence>0;
}
inline bool sameVersion(const Version&a,const Version&b){
  return sameTask(a,b)&&a.reference_generation==b.reference_generation&&a.segment_id==b.segment_id&&
    a.anchor_id==b.anchor_id&&a.anchor_revision==b.anchor_revision&&
    a.context_sequence==b.context_sequence&&a.map_geometry_revision==b.map_geometry_revision;
}
}
