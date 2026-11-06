#pragma once
#include <cmath>
#include <stdexcept>
#include <string>

namespace d1monitor::execution3 {
inline bool validKinematicProfile(const std::string&profile) {
  return profile=="forward_yaw"||profile=="holonomic_xyyaw";
}
inline void validateKinematicProfile(const std::string&profile) {
  if(!validKinematicProfile(profile))throw std::invalid_argument("invalid_kinematic_profile");
}
template<class Twist>inline bool planarMotionNonzero(const Twist&v) {
  return v.linear.x!=0.||v.linear.y!=0.||v.angular.z!=0.;
}
struct SdkMoveFractions {double left_right=0.,forward_back=0.,yaw=0.;};
// Move takes normalized lateral, forward and yaw fractions, in that order.
// Only explicit profile-bound mappings are used; vendor speed tables conflict.
// Runtime engineering mappings do not imply physical speed calibration.
inline SdkMoveFractions sdkMoveFractions(double vx,double vy,double omega,
    const std::string&profile,double forward_scale,double lateral_scale,double yaw_scale) {
  validateKinematicProfile(profile);
  if(!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(omega)||
     !std::isfinite(forward_scale)||forward_scale<=0.||!std::isfinite(yaw_scale)||yaw_scale<=0.||
     (profile=="forward_yaw"&&(vx<0.||vy!=0.))||
     (profile=="holonomic_xyyaw"&&(!std::isfinite(lateral_scale)||lateral_scale<=0.)))
    throw std::invalid_argument("invalid_record_bound_sdk_mapping");
  SdkMoveFractions out{profile=="holonomic_xyyaw"?vy/lateral_scale:0.,vx/forward_scale,omega/yaw_scale};
  if(std::abs(out.left_right)>1.||std::abs(out.forward_back)>1.||std::abs(out.yaw)>1.)
    throw std::invalid_argument("validated_speed_exceeds_sdk_fraction");
  return out;
}
}
