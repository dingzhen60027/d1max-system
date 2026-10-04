#pragma once
#include "d1max_navigation_bt/execution_owner.hpp"
#include <d1max_planning_interfaces/msg/navigation_state.hpp>

namespace d1max_navigation_bt {
// Coarse task-stage continuity, NOT collision evidence or an execution lease.
// Admission and high-frequency control still require the original 100 ms IMU
// lease. A later BT timer may cross that lease while an already admitted local
// state retains its genuine identity/source/posterior; this must not restart
// the FollowRoute workflow or re-date any state. Explicit loss still pauses.
inline bool localTaskInputsFresh(const d1max_planning_interfaces::msg::LocalNavigationState& m,
    const std::string& session,const std::string& map,uint64_t epoch,
    const std::string& seed,double now) {
  const auto ns=[](const builtin_interfaces::msg::Time&t){return int64_t(t.sec)*1000000000LL+t.nanosec;};
  for(const auto*t:{&m.source_stamp,&m.posterior_stamp,&m.imu_stamp})
    if(t->nanosec>=1000000000U||ns(*t)<=0)return false;
  const auto source=ns(m.source_stamp),posterior=ns(m.posterior_stamp),imu=ns(m.imu_stamp);
  if(m.schema_version!=1||!m.usable||m.session_id!=session||map.empty()||m.map_version_id!=map||
     epoch==0||seed.empty()||m.localization_epoch!=epoch||m.localization_seed_id!=seed||
     m.local_odometry.header.stamp!=m.source_stamp||
     m.local_odometry.header.frame_id!="d1max_loc_odom"||m.local_odometry.child_frame_id!="d1max_loc_base_link"||
     posterior>source||imu>source||!std::isfinite(m.extrapolation_sec)||
     m.extrapolation_sec<0||m.extrapolation_sec>.1||
     std::abs((source-imu)*1e-9-m.extrapolation_sec)>1e-5||
     !exec3::fresh(exec3::seconds(m.source_stamp),now,.4)||
     !exec3::fresh(exec3::seconds(m.posterior_stamp),now,.4))return false;
  const auto&o=m.local_odometry;const auto&p=o.pose.pose.position;const auto&q=o.pose.pose.orientation;
  const auto&v=o.twist.twist;
  for(double value:{p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z})
    if(!std::isfinite(value))return false;
  return std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<=.001;
}
inline bool acceptLocalState(const d1max_planning_interfaces::msg::LocalNavigationState& m,
    const d1max_planning_interfaces::msg::LocalNavigationState& previous,
    const std::string& session,const std::string& map,double now) {
  const auto ns=[](const builtin_interfaces::msg::Time&t){return int64_t(t.sec)*1000000000LL+t.nanosec;};
  if(m.schema_version!=1||m.session_id!=session||map.empty()||m.map_version_id!=map||
     m.localization_epoch==0||m.localization_seed_id.empty()||m.source_stamp.nanosec>=1000000000U||
     m.localization_epoch<previous.localization_epoch||
     (m.localization_epoch==previous.localization_epoch&&ns(m.source_stamp)<=ns(previous.source_stamp))||
     !exec3::fresh(exec3::seconds(m.source_stamp),now,.4)||m.local_odometry.header.stamp!=m.source_stamp||
     m.local_odometry.header.frame_id!="d1max_loc_odom"||m.local_odometry.child_frame_id!="d1max_loc_base_link"||
     !std::isfinite(m.extrapolation_sec)||m.extrapolation_sec<0)return false;
  // Unavailable states withdraw capability without supplying synthetic pose.
  if(!m.usable)return m.reason.size()<=1024;
  // Validate the SAME original propagation/time/geometry contract at entry.
  // Task use may later survive the instantaneous IMU lease, but admission and
  // control may not. Unavailable events above deliberately need no fake IMU.
  return localTaskInputsFresh(m,session,map,m.localization_epoch,m.localization_seed_id,now)&&
    exec3::fresh(exec3::seconds(m.imu_stamp),now,.1);
}
// Malformed/foreign/old messages neither erase the last good snapshot nor
// renew its lifetime. An ordered, authenticated-by-context unusable event DOES
// replace it, immediately withdrawing motion without waiting for old TTL.
inline bool acceptBodyPair(const d1max_planning_interfaces::msg::NavigationState& m,
    const d1max_planning_interfaces::msg::NavigationState& previous,
    const std::string& session,const std::string& map,double now) {
  const auto ns=[](const builtin_interfaces::msg::Time&t){return int64_t(t.sec)*1000000000LL+t.nanosec;};
  if(m.schema_version!=2||m.session_id!=session||map.empty()||m.map_version_id!=map||
     m.localization_epoch==0||m.localization_seed_id.empty()||m.source_stamp.nanosec>=1000000000U||
     m.localization_epoch<previous.localization_epoch||
     (m.localization_epoch==previous.localization_epoch&&ns(m.source_stamp)<=ns(previous.source_stamp))||
     !exec3::fresh(exec3::seconds(m.source_stamp),now,.4)||
     m.local_odometry.header.stamp!=m.source_stamp||m.global_odometry.header.stamp!=m.source_stamp||
     m.local_odometry.header.frame_id!="d1max_loc_odom"||m.global_odometry.header.frame_id!="d1max_loc_map"||
     m.local_odometry.child_frame_id!="d1max_loc_base_link"||m.global_odometry.child_frame_id!="d1max_loc_base_link"||
     !std::isfinite(m.extrapolation_sec)||m.extrapolation_sec<0)return false;
  for(const auto*o:{&m.local_odometry,&m.global_odometry}) {
    const auto&p=o->pose.pose.position;const auto&q=o->pose.pose.orientation;const auto&v=o->twist.twist;
    for(double value:{p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z})
      if(!std::isfinite(value))return false;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;if(std::abs(norm-1.)>.001)return false;
  }
  return true;
}
}
