#pragma once
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace scan_planner {
// Unordered, source-map support samples. Never join neighbouring array
// records into artificial ground. This check is additional to raw-ray swept
// collision evidence, not a replacement for it.
inline bool measuredConnectionSupported(
    const d1max_planning_interfaces::msg::SupportReference& s,
    const Eigen::Vector3d& from,const Eigen::Vector3d& to,const std::string& frame,
    double resolution) {
  if(!s.verified||s.floor_id!="floor1"||s.segment_kind!="floor"||s.required_mode!="general"||
     s.frame_id!=frame||s.support_hash.size()!=64||s.support_map_sha256.size()!=64||
     s.support_ground_xyz.size()<2||s.support_ground_xyz.size()>20000||
     !std::isfinite(s.support_xy_radius_m)||s.support_xy_radius_m<=0.||s.support_xy_radius_m>.5||
     !std::isfinite(s.body_reference_height_m)||s.body_reference_height_m<.2||s.body_reference_height_m>.8||
     !std::isfinite(s.max_support_slope_rad)||s.max_support_slope_rad<=0.||s.max_support_slope_rad>.176||
     !std::isfinite(s.max_support_step_m)||s.max_support_step_m<=0.||s.max_support_step_m>.08||
     !from.allFinite()||!to.allFinite()||(to-from).norm()>.15||
     !std::isfinite(resolution)||resolution<=0.)return false;
  for(const auto& p:s.support_ground_xyz)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
  const auto count=std::max(1,static_cast<int>(std::ceil((to-from).norm()/std::min(.0125,resolution*.25))));
  if(count>100)return false;
  for(int i=0;i<=count;++i) {
    const Eigen::Vector3d body=from+(double(i)/count)*(to-from);
    double nearest=std::numeric_limits<double>::infinity();const geometry_msgs::msg::Point* anchor=nullptr;
    for(const auto& p:s.support_ground_xyz) {
      const double d=std::hypot(p.x-body.x(),p.y-body.y());
      if(d<nearest){nearest=d;anchor=&p;}
    }
    if(!anchor||nearest>s.support_xy_radius_m||std::abs(body.z()-anchor->z-s.body_reference_height_m)>.15)return false;
    for(const auto& p:s.support_ground_xyz) {
      const double distance=std::hypot(p.x-anchor->x,p.y-anchor->y);
      if(distance<=s.support_xy_radius_m&&distance>=1e-6&&
         std::abs(p.z-anchor->z)>s.max_support_step_m+std::tan(s.max_support_slope_rad)*distance)return false;
    }
  }
  return true;
}
} // namespace scan_planner
