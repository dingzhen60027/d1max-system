#pragma once
#include <cmath>
#include <string>

namespace d1max_planning_interfaces {
inline bool stationaryVelocityMetricKnown(const std::string& metric) {
  return metric=="spatial_norm_3d"||metric=="planar_xy_with_vertical_guard_v1";
}

// measured_linear_mps always retains the norm of the ORIGINAL three MC axes.
// A planar witness is valid only under an explicitly matching wheel profile;
// it does not discard vertical motion. Angular motion always uses the 3D norm.
template<class Evidence>
bool stationaryVelocityWithin(const Evidence& e,const std::string& configured_metric,
                              double linear_limit,double angular_limit) {
  if(!stationaryVelocityMetricKnown(configured_metric)||
     !std::isfinite(linear_limit)||linear_limit<=0.||!std::isfinite(angular_limit)||angular_limit<=0.||
     !std::isfinite(e.measured_linear_mps)||e.measured_linear_mps<0.||
     !std::isfinite(e.measured_angular_radps)||e.measured_angular_radps<0.||
     e.measured_angular_radps>angular_limit)return false;
  if(e.velocity_metric.empty()||e.velocity_metric=="spatial_norm_3d")
    return e.measured_linear_mps<=linear_limit;
  if(e.velocity_metric!="planar_xy_with_vertical_guard_v1"||e.velocity_metric!=configured_metric||
     !std::isfinite(e.measured_planar_mps)||e.measured_planar_mps<0.||
     !std::isfinite(e.measured_vertical_mps)||e.measured_vertical_mps<0.)return false;
  return std::abs(std::hypot(e.measured_planar_mps,e.measured_vertical_mps)-e.measured_linear_mps)<=1e-6&&
    e.measured_planar_mps<=linear_limit&&e.measured_vertical_mps<=linear_limit;
}
}
