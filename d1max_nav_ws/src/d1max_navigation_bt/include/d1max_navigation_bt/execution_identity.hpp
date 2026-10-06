#pragma once
#include <d1max_planning_interfaces/msg/execution_version.hpp>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <cstdint>
#include <string>
namespace d1max_navigation_bt::exec3 {
using Version=d1max_planning_interfaces::msg::ExecutionVersion;
// Keep the ROS source clock in integer nanoseconds at real callbacks.  Double
// seconds remain supported for approximate SDK clocks and small-time tests.
// Converting sec + nanosec separately is not equivalent to ROS ns / 1e9 at
// Unix epochs: the same exact source can otherwise look 238 ns in the future.
struct SourceClock {
  double seconds_value=0.;
  std::optional<std::int64_t> exact_ns;
  SourceClock(double value=0.):seconds_value(value){}
  static SourceClock fromNanoseconds(std::int64_t value) {
    SourceClock out(static_cast<double>(value)/1e9);out.exact_ns=value;return out;
  }
  operator double()const{return seconds_value;}
};
inline std::int64_t nanoseconds(const builtin_interfaces::msg::Time&t){
  return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;
}
inline double seconds(const builtin_interfaces::msg::Time&t){return static_cast<double>(nanoseconds(t))/1e9;}
inline builtin_interfaces::msg::Time stamp(double t){
  builtin_interfaces::msg::Time out;out.sec=static_cast<int32_t>(std::floor(t));
  out.nanosec=static_cast<uint32_t>(std::max(0.,(t-out.sec)*1e9));return out;
}
inline builtin_interfaces::msg::Time stamp(SourceClock t){
  if(!t.exact_ns)return stamp(t.seconds_value);
  builtin_interfaces::msg::Time out;out.sec=static_cast<int32_t>(*t.exact_ns/1000000000LL);
  const auto remainder=*t.exact_ns%1000000000LL;
  if(remainder<0){--out.sec;out.nanosec=static_cast<uint32_t>(remainder+1000000000LL);}
  else out.nanosec=static_cast<uint32_t>(remainder);
  return out;
}
inline SourceClock after(SourceClock t,double duration){
  if(!t.exact_ns)return SourceClock(t.seconds_value+duration);
  return SourceClock::fromNanoseconds(*t.exact_ns+static_cast<std::int64_t>(duration*1e9));
}
inline int compare(const builtin_interfaces::msg::Time&t,SourceClock now){
  if(now.exact_ns){const auto value=nanoseconds(t);return value<*now.exact_ns?-1:value>*now.exact_ns?1:0;}
  const double value=seconds(t);return value<now.seconds_value?-1:value>now.seconds_value?1:0;
}
inline double elapsed(const builtin_interfaces::msg::Time&end,const builtin_interfaces::msg::Time&begin){
  return static_cast<double>(nanoseconds(end)-nanoseconds(begin))/1e9;
}
inline bool fresh(double source,double now,double age){
  return std::isfinite(source)&&std::isfinite(now)&&source>0&&now>=source&&now-source<=age;
}
inline bool fresh(double source,SourceClock now,double age){return fresh(source,now.seconds_value,age);}
inline bool fresh(SourceClock source,double now,double age){return fresh(source.seconds_value,now,age);}
inline bool fresh(SourceClock source,SourceClock now,double age){
  if(source.exact_ns&&now.exact_ns){
    return *source.exact_ns>0&&*now.exact_ns>=*source.exact_ns&&std::isfinite(age)&&age>=0.&&
      *now.exact_ns-*source.exact_ns<=static_cast<std::int64_t>(age*1e9);
  }
  return fresh(source.seconds_value,now.seconds_value,age);
}
inline bool fresh(const builtin_interfaces::msg::Time&source,SourceClock now,double age){
  if(source.nanosec>=1000000000U)return false;
  return fresh(SourceClock::fromNanoseconds(nanoseconds(source)),now,age);
}
inline bool fresh(const builtin_interfaces::msg::Time&source,const builtin_interfaces::msg::Time&now,double age){
  if(now.nanosec>=1000000000U)return false;
  return fresh(source,SourceClock::fromNanoseconds(nanoseconds(now)),age);
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
