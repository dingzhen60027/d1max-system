#pragma once
#include <plan_manage/reference_target.hpp>
#include <plan_manage/motion_sweep.hpp>
#include <plan_env/solve_budget.hpp>
#include <algorithm>
#include <array>

namespace scan_planner {
struct VisibleReferenceTarget {
  ReferenceTargetResult target;
  std::vector<Eigen::Vector3d> support_path;
  std::size_t candidates{0};
};

// A bounded, observed-free side step when the ordered route has no reachable
// visible endpoint. This is a LOCAL reference only: never alter route identity,
// its order, the destination, or invent free space behind an obstacle. The
// native A*/optimizer/admission and each command's braking proof still run.
inline VisibleReferenceTarget selectVisibleSideTarget(
    const DiscreteReference& route,const Eigen::Vector3d& body,double measured_yaw,
    double progress,double horizon,const MotionSupport& support,GridMap& map,const std::string& frame,
    const SolveBudget::Ptr& budget,std::size_t query_limit=1024,const BrakingModel* braking=nullptr) {
  VisibleReferenceTarget out;out.target.reason="waiting_observed_side_target";
  if(!body.allFinite()||!std::isfinite(measured_yaw)||!std::isfinite(progress)||
     !std::isfinite(horizon)||horizon<=0.||route.length()<=0.||!support.valid||
     frame.empty()||support.source.frame_id!=frame||!map.requiresObservedFree()||
     query_limit<1||query_limit>4096||!solveAllowed(budget))return out;
  progress=std::clamp(progress,0.,route.length());
  const auto origin=route.sample(progress);
  Eigen::Vector2d forward=(route.sample(std::min(route.length(),progress+.2))-origin).head<2>();
  if(forward.norm()<1e-6)return out;
  forward.normalize();const Eigen::Vector2d left(-forward.y(),forward.x());
  const double current_side=(body-origin).head<2>().dot(left);
  const std::array<double,2> sides=current_side<-.1?std::array<double,2>{-1.,1.}:std::array<double,2>{1.,-1.};
  const auto observed=[&](const Eigen::Vector3d& point,double yaw) {
    if(out.target.queries>=query_limit||!solveAllowed(budget))return false;
    const int state=map.getInflateOccupancy(point,yaw);out.target.observe(point,state,yaw);
    return state==0;
  };
  const auto body_support=support.nearest(body.x(),body.y());
  if(!body_support||std::abs(body.z()-body_support->z-support.source.body_reference_height_m)>.15)return out;
  const double slope=std::tan(support.source.max_support_slope_rad);
  // At most 40 candidates. Prefer progress but allow a pure side step to gain
  // visibility. Once off the route, retain that side rather than oscillating.
  for(double advance:{.65,.40,.20,0.})for(double side:sides)for(double offset:{.45,.65,.85,1.05,1.25}) {
    if(std::abs(current_side)>.2&&side*current_side<0.)continue;
    if(!solveAllowed(budget)||out.target.queries>=query_limit) {
      out.target.reason="visible_side_target_budget_exhausted";return out;
    }
    ++out.candidates;
    const double arc=std::min(route.length(),progress+std::min(advance,horizon));
    const auto on_route=route.sample(arc);
    const Eigen::Vector2d xy=on_route.head<2>()+side*offset*left;
    const auto ground=support.nearest(xy.x(),xy.y());if(!ground)continue;
    Eigen::Vector3d target(xy.x(),xy.y(),ground->z+support.source.body_reference_height_m);
    const auto displacement=(target-body).eval();
    if(displacement.head<2>().norm()<.25||displacement.norm()>std::min(2.,horizon)||
       std::abs(ground->z-body_support->z)>support.source.max_support_step_m+slope*displacement.head<2>().norm())continue;
    const double yaw=std::atan2(displacement.y(),displacement.x());
    if(!observed(target,yaw))continue;
    // Query the actual body rotation as well; a side target cannot ignore the
    // long rectangular footprint while it turns away from the route tangent.
    const double turn=std::atan2(std::sin(yaw-measured_yaw),std::cos(yaw-measured_yaw));
    const auto shape=map.bodyEnvelope();
    const int turns=std::max(1,int(std::ceil(std::abs(turn)*(shape.offset+shape.radius)/
      std::max(.005,map.getResolution()*.5))));
    if(turns>128)continue;
    bool valid=true;
    for(int i=0;i<=turns&&valid;++i)valid=observed(body,measured_yaw+turn*double(i)/turns);
    if(!valid)continue;
    const int steps=std::max(1,int(std::ceil(displacement.head<2>().norm()/std::min(.025,map.getResolution()*.5))));
    if(steps>128)continue;
    std::vector<Eigen::Vector3d> path{body};
    double previous_z=body_support->z;
    Eigen::Vector2d previous_xy=body.head<2>();
    for(int i=1;i<=steps&&valid;++i) {
      const auto sample_xy=(body.head<2>()+displacement.head<2>()*(double(i)/steps)).eval();
      const auto p=support.nearest(sample_xy.x(),sample_xy.y());
      if(!p||std::abs(p->z-previous_z)>support.source.max_support_step_m+slope*(sample_xy-previous_xy).norm()||
         std::abs(p->z-body_support->z)>support.source.max_support_step_m+slope*(sample_xy-body.head<2>()).norm()) {valid=false;break;}
      // Height comes only from the measured source support at this XY, never
      // a copy of the route centerline height onto an unobserved side floor.
      Eigen::Vector3d sample(sample_xy.x(),sample_xy.y(),p->z+support.source.body_reference_height_m);
      valid=observed(sample,yaw);if(valid)path.push_back(sample);
      previous_z=p->z;previous_xy=sample_xy;
    }
    if(!valid)continue;
    if(braking) {
      if(!map.isRawRaySnapshot()) {out.target.reason="private_raw_snapshot_required";return out;}
      const double remaining=budget?budget->remainingSeconds():.08;
      if(remaining<=0.) {out.target.reason="visible_side_target_budget_exhausted";return out;}
      const auto executable=validateStraightBrakingCorridor(map,support,*braking,body,target,frame,
          std::min(.08,remaining));
      if(!executable.valid)continue;
    }
    out.target.valid=true;out.target.arc=arc;out.target.point=path.back();
    out.target.reason="reference_target_visible_side";out.support_path=std::move(path);return out;
  }
  return out;
}
} // namespace scan_planner
