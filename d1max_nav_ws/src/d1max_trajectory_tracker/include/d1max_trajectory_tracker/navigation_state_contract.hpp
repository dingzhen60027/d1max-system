#pragma once

#include <d1max_planning_interfaces/msg/navigation_state.hpp>
#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include <type_traits>

namespace d1max_trajectory_tracker {

// Run before decoding usability/geometry. Both formal local transport and
// legacy atomic-pair fixtures share this ingress ordering. No source is
// restamped and ignored packets do not renew any receipt or recovery lease.
class NavigationStateIngress {
public:
  template<class State> bool inspect(const State& state,const Config& config,SourceTime now,std::uint64_t epoch_floor=0) {
    constexpr std::uint32_t expected_schema=std::is_same_v<State,
      d1max_planning_interfaces::msg::LocalNavigationState>?1:2;
    // Foreign sessions/maps/interface versions are not this estimator's
    // events. They neither revoke its task nor refresh its source watermarks.
    if(state.schema_version!=expected_schema||state.session_id!=config.session_id||
       state.map_version_id!=config.map_version_id)return false;
    if(state.localization_epoch>0&&state.localization_epoch<std::max(epoch_,epoch_floor))return false;
    const auto& t=state.source_stamp;
    if(t.sec<0||t.nanosec>=1000000000U||(t.sec==0&&t.nanosec==0))return true; // fresh protocol fault
    const auto ns=std::int64_t(t.sec)*1000000000LL+t.nanosec;
    const bool identity_present=state.localization_epoch>0&&!state.localization_seed_id.empty();
    const bool new_epoch=identity_present&&state.localization_epoch>epoch_;
    if(!new_epoch&&ns<=source_ns_)return false;
    // Invalid future stamps fail closed, but cannot poison the ordering
    // watermark so that every later genuine measurement is silently ignored.
    if(finite(now)&&now.nanoseconds()>0&&ns-now.nanoseconds()<=20000000LL) {
      if(new_epoch)source_ns_=0;
      source_ns_=ns;
      if(identity_present)epoch_=std::max(epoch_,state.localization_epoch);
    }
    return true;
  }
private:
  std::int64_t source_ns_{0};std::uint64_t epoch_{0};
};

// A malformed/unusable measurement can still carry a genuine estimator
// reset. Foreign identity or obviously future source time is a hold, not a
// trustworthy reset event. Use this only AFTER ingress ordering.
template<class State> bool navigationContextReset(const State& state,const Config& config,
    const ControlIdentity& expected,SourceTime now) {
  constexpr std::uint32_t schema=std::is_same_v<State,
    d1max_planning_interfaces::msg::LocalNavigationState>?1:2;
  const auto& t=state.source_stamp;
  if(state.schema_version!=schema||state.session_id!=config.session_id||state.map_version_id!=config.map_version_id||
     state.localization_epoch==0||state.localization_seed_id.empty()||t.sec<0||t.nanosec>=1000000000U||
     (t.sec==0&&t.nanosec==0)||!finite(now)||now.nanoseconds()<=0||
     std::int64_t(t.sec)*1000000000LL+t.nanosec-now.nanoseconds()>20000000LL)return false;
  return state.localization_epoch!=expected.localization_epoch||state.localization_seed_id!=expected.localization_seed_id;
}

// Local schema 1 is a separate transport, not a fabricated global/local pair.
// Internally Odom schema 2 denotes the same source-bound estimator identity
// contract already enforced by TrackerCore; source times are copied unchanged.
inline bool decodeLocalNavigationState(const d1max_planning_interfaces::msg::LocalNavigationState& state,
                                      const Config& config,Odom& out,std::string& reason) {
  const auto valid_stamp=[](const builtin_interfaces::msg::Time& stamp) {
    return stamp.sec>=0&&stamp.nanosec<1000000000U&&(stamp.sec>0||stamp.nanosec>0);
  };
  const auto seconds=[](const builtin_interfaces::msg::Time& stamp) {
    return double(stamp.sec)+double(stamp.nanosec)*1e-9;
  };
  const auto& local=state.local_odometry;
  if(state.schema_version!=1||!state.usable||state.session_id!=config.session_id||
      state.map_version_id!=config.map_version_id||state.localization_epoch==0||state.localization_seed_id.empty()||
      !valid_stamp(state.source_stamp)||!valid_stamp(state.posterior_stamp)||!valid_stamp(state.imu_stamp)||
      local.header.stamp!=state.source_stamp||local.header.frame_id!=config.planning_frame||
      local.child_frame_id!=config.base_frame) {
    reason="local_navigation_state_identity_or_source_invalid";return false;
  }
  const auto& p=local.pose.pose.position;const auto& q=local.pose.pose.orientation;
  const auto& v=local.twist.twist.linear;const auto& w=local.twist.twist.angular;
  for(double value:{p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z})
    if(!finite(value)){reason="local_navigation_state_nonfinite_geometry";return false;}
  if(std::abs(std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1.)>.001) {
    reason="local_navigation_state_invalid_rotation";return false;
  }
  out.schema_version=2;out.session_id=state.session_id;out.map_version_id=state.map_version_id;
  out.localization_epoch=state.localization_epoch;out.localization_seed_id=state.localization_seed_id;
  out.frame_id=local.header.frame_id;out.child_frame_id=local.child_frame_id;
  out.stamp=seconds(state.source_stamp);
  out.source_stamp_ns=std::int64_t(state.source_stamp.sec)*1000000000LL+state.source_stamp.nanosec;
  out.posterior_stamp=seconds(state.posterior_stamp);
  out.posterior_stamp_ns=std::int64_t(state.posterior_stamp.sec)*1000000000LL+state.posterior_stamp.nanosec;
  out.imu_stamp_ns=std::int64_t(state.imu_stamp.sec)*1000000000LL+state.imu_stamp.nanosec;
  out.imu_stamp=seconds(state.imu_stamp);out.extrapolation_sec=state.extrapolation_sec;
  out.position={p.x,p.y,p.z};out.orientation={q.w,q.x,q.y,q.z};
  out.yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
  out.planar_speed=std::hypot(v.x,v.y);
  out.velocity_in_frame=out.orientation*Eigen::Vector3d(v.x,v.y,v.z);
  out.angular_velocity_in_frame=out.orientation*Eigen::Vector3d(w.x,w.y,w.z);
  reason="independent_source_bound_local_body_state";return true;
}

// Transport conversion only. Admission (source age/order and task ownership)
// remains in TrackerCore; no task fields or TF lookup manufacture provenance.
inline bool decodeNavigationState(const d1max_planning_interfaces::msg::NavigationState& state,
                                  const Config& config, Odom& out, std::string& reason) {
  const auto seconds=[](const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec)+static_cast<double>(stamp.nanosec)*1e-9;
  };
  const auto valid_stamp=[](const builtin_interfaces::msg::Time& stamp) {
    return stamp.sec>=0 && stamp.nanosec<1000000000U && (stamp.sec>0 || stamp.nanosec>0);
  };
  const auto& local=state.local_odometry;
  const auto& global=state.global_odometry;
  if (state.schema_version!=2 || !state.usable || state.session_id!=config.session_id ||
      state.map_version_id!=config.map_version_id || state.localization_epoch==0 ||
      state.localization_seed_id.empty() || !valid_stamp(state.source_stamp) ||
      !valid_stamp(state.posterior_stamp) || !valid_stamp(state.imu_stamp) ||
      local.header.stamp!=state.source_stamp || global.header.stamp!=state.source_stamp ||
      local.header.frame_id!=config.planning_frame || global.header.frame_id!=config.map_frame ||
      local.child_frame_id!=config.base_frame || global.child_frame_id!=config.base_frame) {
    reason="navigation_state_identity_or_pair_invalid"; return false;
  }
  for (const auto* odom : {&local,&global}) {
    const auto& p=odom->pose.pose.position; const auto& q=odom->pose.pose.orientation;
    const auto& v=odom->twist.twist.linear; const auto& w=odom->twist.twist.angular;
    for (double value : {p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z})
      if (!finite(value)) { reason="navigation_state_nonfinite_geometry"; return false; }
    if (std::abs(std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1.)>.001) {
      reason="navigation_state_invalid_rotation"; return false;
    }
  }
  const auto& p=local.pose.pose.position; const auto& q=local.pose.pose.orientation;
  const auto& v=local.twist.twist.linear; const auto& w=local.twist.twist.angular;
  out.schema_version=state.schema_version;
  out.session_id=state.session_id; out.map_version_id=state.map_version_id;
  out.localization_epoch=state.localization_epoch; out.localization_seed_id=state.localization_seed_id;
  out.frame_id=local.header.frame_id; out.child_frame_id=local.child_frame_id;
  out.stamp=seconds(state.source_stamp);
  out.source_stamp_ns=std::int64_t(state.source_stamp.sec)*1000000000LL+state.source_stamp.nanosec;
  out.posterior_stamp=seconds(state.posterior_stamp);
  out.posterior_stamp_ns=std::int64_t(state.posterior_stamp.sec)*1000000000LL+state.posterior_stamp.nanosec;
  out.imu_stamp_ns=std::int64_t(state.imu_stamp.sec)*1000000000LL+state.imu_stamp.nanosec;
  out.imu_stamp=seconds(state.imu_stamp); out.extrapolation_sec=state.extrapolation_sec;
  out.position={p.x,p.y,p.z}; out.orientation={q.w,q.x,q.y,q.z};
  out.yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
  out.planar_speed=std::hypot(v.x,v.y);
  // nav_msgs/Odometry twist is in child_frame_id, progress twist in header frame.
  out.velocity_in_frame=out.orientation*Eigen::Vector3d(v.x,v.y,v.z);
  out.angular_velocity_in_frame=out.orientation*Eigen::Vector3d(w.x,w.y,w.z);
  reason="source_bound_local_body_state";
  return true;
}
}  // namespace d1max_trajectory_tracker
