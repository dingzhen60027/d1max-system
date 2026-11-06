#pragma once
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace scan_planner {
// One failed sample only. This is diagnostic data, never a second verdict or
// a cache; no source points are copied and the original predicate is unchanged.
struct SupportConnectionDiagnostic {
  bool checked{false},supported{false};const char* reason{"not_checked"};
  std::size_t source_points{0};int sample_index{-1};
  Eigen::Vector3d from{Eigen::Vector3d::Constant(NAN)},to{Eigen::Vector3d::Constant(NAN)};
  Eigen::Vector3d sample{Eigen::Vector3d::Constant(NAN)},anchor{Eigen::Vector3d::Constant(NAN)};
  Eigen::Vector3d neighbor{Eigen::Vector3d::Constant(NAN)};
  double connector_m{NAN},nearest_xy_m{NAN},body_height_error_m{NAN};
  double neighbor_height_delta_m{NAN},neighbor_allowed_delta_m{NAN};
  double support_xy_radius_m{NAN},body_reference_height_m{NAN},max_support_step_m{NAN},max_support_slope_rad{NAN};
};
// Unordered, source-map support samples. Never join neighbouring array
// records into artificial ground. This check is additional to raw-ray swept
// collision evidence, not a replacement for it.
inline bool measuredConnectionSupported(
    const d1max_planning_interfaces::msg::SupportReference& s,
    const Eigen::Vector3d& from,const Eigen::Vector3d& to,const std::string& frame,
    double resolution,SupportConnectionDiagnostic* diagnostic=nullptr) {
  if(diagnostic) {
    *diagnostic={};diagnostic->checked=true;diagnostic->from=from;diagnostic->to=to;
    diagnostic->connector_m=(to-from).norm();diagnostic->source_points=s.support_ground_xyz.size();
    diagnostic->support_xy_radius_m=s.support_xy_radius_m;diagnostic->body_reference_height_m=s.body_reference_height_m;
    diagnostic->max_support_step_m=s.max_support_step_m;diagnostic->max_support_slope_rad=s.max_support_slope_rad;
  }
  const auto reject=[&](const char* reason){if(diagnostic)diagnostic->reason=reason;return false;};
  if(!s.verified||(s.floor_id!="floor1"&&s.floor_id!="floor2")||(s.segment_kind!="floor"&&s.segment_kind!="landing")||s.required_mode!="general"||
     s.frame_id!=frame||s.support_hash.size()!=64||s.support_map_sha256.size()!=64||
     s.support_ground_xyz.size()<2||s.support_ground_xyz.size()>20000||
     !std::isfinite(s.support_xy_radius_m)||s.support_xy_radius_m<=0.||s.support_xy_radius_m>.5||
     !std::isfinite(s.body_reference_height_m)||s.body_reference_height_m<.2||s.body_reference_height_m>.8||
     !std::isfinite(s.max_support_slope_rad)||s.max_support_slope_rad<=0.||s.max_support_slope_rad>.176||
     !std::isfinite(s.max_support_step_m)||s.max_support_step_m<=0.||s.max_support_step_m>.08||
     !from.allFinite()||!to.allFinite()||
     !std::isfinite(resolution)||resolution<=0.)return reject("support_contract_invalid");
  if((to-from).norm()>.15)return reject("connection_length_exceeded");
  for(const auto& p:s.support_ground_xyz)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return reject("source_point_nonfinite");
  const auto count=std::max(1,static_cast<int>(std::ceil((to-from).norm()/std::min(.0125,resolution*.25))));
  if(count>100)return reject("connection_sampling_limit");
  for(int i=0;i<=count;++i) {
    const Eigen::Vector3d body=from+(double(i)/count)*(to-from);
    double nearest=std::numeric_limits<double>::infinity();const geometry_msgs::msg::Point* anchor=nullptr;
    for(const auto& p:s.support_ground_xyz) {
      const double d=std::hypot(p.x-body.x(),p.y-body.y());
      if(d<nearest){nearest=d;anchor=&p;}
    }
    if(diagnostic) {
      diagnostic->sample_index=i;diagnostic->sample=body;diagnostic->nearest_xy_m=nearest;
      if(anchor) {
        diagnostic->anchor={anchor->x,anchor->y,anchor->z};
        diagnostic->body_height_error_m=body.z()-anchor->z-s.body_reference_height_m;
      }
    }
    if(!anchor||nearest>s.support_xy_radius_m)return reject("support_xy_coverage_missing");
    if(std::abs(body.z()-anchor->z-s.body_reference_height_m)>.15)return reject("body_support_height_mismatch");
    for(const auto& p:s.support_ground_xyz) {
      const double distance=std::hypot(p.x-anchor->x,p.y-anchor->y);
      if(distance<=s.support_xy_radius_m&&distance>=1e-6&&
         std::abs(p.z-anchor->z)>s.max_support_step_m+std::tan(s.max_support_slope_rad)*distance) {
        if(diagnostic) {
          diagnostic->neighbor={p.x,p.y,p.z};diagnostic->neighbor_height_delta_m=std::abs(p.z-anchor->z);
          diagnostic->neighbor_allowed_delta_m=s.max_support_step_m+std::tan(s.max_support_slope_rad)*distance;
        }
        return reject("support_neighbor_height_discontinuity");
      }
    }
  }
  if(diagnostic){diagnostic->supported=true;diagnostic->reason="supported";}
  return true;
}
} // namespace scan_planner
