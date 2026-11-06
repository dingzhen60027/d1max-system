#pragma once

#include <plan_manage/trajectory_collision.hpp>
#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include <d1max_planning_interfaces/msg/execution_version.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace scan_planner {

// Source transport only. No reference, trajectory or task lifecycle belongs
// here. Native and the follower consume the SAME provider-issued local state;
// neither a Python reference callback nor a new receipt creates its source.
struct LocalBodyIdentity {
  std::string session,map,seed;
  std::uint64_t epoch{0};
  bool operator==(const LocalBodyIdentity& other)const {
    return session==other.session&&map==other.map&&seed==other.seed&&epoch==other.epoch;
  }
  bool matches(const d1max_planning_interfaces::msg::ExecutionVersion& v)const {
    return session==v.session_id&&map==v.map_version_id&&seed==v.localization_seed_id&&
      epoch==v.localization_epoch;
  }
};
struct LocalBodySample {
  MeasuredBodyPose pose;
  geometry_msgs::msg::Twist world_twist;
  LocalBodyIdentity identity;
  std::int64_t source_ns{0},valid_until_ns{0};
};
class LocalNavigationBodyInput {
public:
  enum class Result { Ignore, Invalid, Accepted };
  void configure(std::string session,std::string map,std::string frame,std::string child) {
    session_=std::move(session);map_=std::move(map);frame_=std::move(frame);child_=std::move(child);
  }
  bool enabled()const{return !session_.empty()&&!map_.empty();}
  Result accept(const d1max_planning_interfaces::msg::LocalNavigationState& s,
      std::int64_t now,LocalBodySample& out) {
    if(!enabled()||s.schema_version!=2||s.session_id!=session_||s.map_version_id!=map_)
      return Result::Ignore;
    if(s.localization_epoch>0&&s.localization_epoch<epoch_)return Result::Ignore;
    const auto source=stamp(s.source_stamp);
    const bool new_epoch=s.localization_epoch>epoch_&&!s.localization_seed_id.empty();
    if(source>0&&!new_epoch&&source<=source_ns_)return Result::Ignore;
    // A genuine reset may restart the source clock. Future/malformed packets
    // cannot raise the watermark and suppress the next actual measurement.
    if(source>0&&now>0&&source<=now+20000000LL) {
      if(new_epoch)source_ns_=0;
      source_ns_=source;
      if(s.localization_epoch>0&&!s.localization_seed_id.empty())
        epoch_=std::max(epoch_,s.localization_epoch);
    }
    const auto deadline=stamp(s.valid_until),posterior=stamp(s.posterior_stamp),imu=stamp(s.imu_stamp);
    const auto& odom=s.local_odometry;
    if(!s.usable||s.localization_epoch==0||s.localization_seed_id.empty()||
       source<=0||now<=0||source>now+20000000LL||
       deadline<=source||deadline-source>100000000LL||now>=deadline||
       posterior<=0||imu<=0||posterior>source||imu>source||
       !std::isfinite(s.extrapolation_sec)||s.extrapolation_sec<0.||
       source-posterior>400000000LL||source-imu>100000000LL||s.extrapolation_sec>.10||
       std::abs(double(source-imu)*1e-9-s.extrapolation_sec)>=1e-5||
       odom.header.stamp!=s.source_stamp||odom.header.frame_id!=frame_||odom.child_frame_id!=child_)
      return Result::Invalid;
    // These are the formal follower's schema-2 provenance bounds AT SOURCE,
    // not a second IMU timeout measured at callback delivery. Consume the
    // provider's original lease above; never add transport age to raw age.
    const auto& p=odom.pose.pose.position;const auto& q=odom.pose.pose.orientation;
    const auto& v=odom.twist.twist.linear;const auto& w=odom.twist.twist.angular;
    for(double value:{p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z})
      if(!std::isfinite(value))return Result::Invalid;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    if(std::abs(rotation.norm()-1.)>.001)return Result::Invalid;
    out.pose={{p.x,p.y,p.z},rotation,double(source)*1e-9,frame_};
    out.source_ns=source;out.valid_until_ns=deadline;
    out.identity={session_,map_,s.localization_seed_id,s.localization_epoch};
    // Odometry twist is in child_frame_id, while Native uses world-frame
    // velocity. Rotate exactly once with THIS original acquisition's pose.
    const Eigen::Vector3d linear=rotation*Eigen::Vector3d(v.x,v.y,v.z);
    const Eigen::Vector3d angular=rotation*Eigen::Vector3d(w.x,w.y,w.z);
    out.world_twist.linear.x=linear.x();out.world_twist.linear.y=linear.y();out.world_twist.linear.z=linear.z();
    out.world_twist.angular.x=angular.x();out.world_twist.angular.y=angular.y();out.world_twist.angular.z=angular.z();
    return Result::Accepted;
  }
private:
  static std::int64_t stamp(const builtin_interfaces::msg::Time& t) {
    return t.sec<0||t.nanosec>=1000000000U?0:std::int64_t(t.sec)*1000000000LL+t.nanosec;
  }
  std::string session_,map_,frame_,child_;
  std::int64_t source_ns_{0};
  std::uint64_t epoch_{0};
};
}
