#pragma once
#include <plan_manage/reference_target.hpp>
#include <bspline_opt/spline_heading_contract.hpp>
#include <plan_manage/motion_sweep.hpp>
#include <plan_manage/planning_support.hpp>
#include <plan_env/solve_budget.hpp>
#include <algorithm>
#include <array>
#include <type_traits>

namespace scan_planner {
struct VisibleReferenceTarget {
  ReferenceTargetResult target;
  std::vector<Eigen::Vector3d> support_path;
  std::size_t candidates{0};
};

namespace side_target_detail {
enum class CollisionPolicy { PlanningGeometry, ObservedEvidence };
inline auto nearestSupport(const MotionSupport& support,double x,double y,double) {
  return support.nearest(x,y);
}
inline auto nearestSupport(const PlanningSupport& support,double x,double y,double z) {
  return support.nearest(x,y,z);
}
inline bool explicitBodyModel(const MotionSupport&) {return false;}
inline bool explicitBodyModel(const PlanningSupport& support) {return support.model()!=nullptr;}
inline std::optional<Eigen::Vector3d> modeledBody(const MotionSupport&,const Eigen::Vector3d&) {return {};}
inline std::optional<Eigen::Vector3d> modeledBody(const PlanningSupport& support,const Eigen::Vector3d& body) {
  return support.bodyPositionOnSupport(body);
}
inline bool supportHeightAllowed(const MotionSupport& support,double dz,double distance) {
  return std::abs(dz)<=support.source.max_support_step_m+
    std::tan(support.source.max_support_slope_rad)*distance;
}
inline bool supportHeightAllowed(const PlanningSupport& support,double dz,double distance) {
  // A stair candidate is covered by literal, XYZ-matched original support
  // samples. The packet's zero legacy flat limits are NOT a level-floor claim
  // or permission to send stair-mode commands. Native collision is unchanged.
  if(const auto* flat=support.flatSupport())return supportHeightAllowed(*flat,dz,distance);
  return support.isStair();
}

// Shared source-support and rotation geometry. PlanningGeometry generates a
// candidate, not a FREE certificate or a command braking proof. The strict
// compatibility path keeps its independent observed-evidence checks.
template<class Support> inline VisibleReferenceTarget selectSideTarget(
    const DiscreteReference& route,const Eigen::Vector3d& body,double measured_yaw,
    double progress,double horizon,const Support& support,GridMap& map,const std::string& frame,
    const SolveBudget::Ptr& budget,std::size_t query_limit,CollisionPolicy policy,const BrakingModel* braking,
    BodyHeadingPolicy heading_policy=BodyHeadingPolicy::PathTangent) {
  const bool observed_evidence=policy==CollisionPolicy::ObservedEvidence;
  VisibleReferenceTarget out;out.target.reason=observed_evidence ? "waiting_observed_side_target" : "waiting_side_target";
  if(!body.allFinite()||!std::isfinite(measured_yaw)||!std::isfinite(progress)||
     !std::isfinite(horizon)||horizon<=0.||route.length()<=0.||!support.valid||
     frame.empty()||support.source.frame_id!=frame||(observed_evidence&&!map.requiresObservedFree())||
     query_limit<1||query_limit>4096||!solveAllowed(budget)||
     (heading_policy!=BodyHeadingPolicy::PathTangent&&heading_policy!=BodyHeadingPolicy::HoldHeading))return out;
  progress=std::clamp(progress,0.,route.length());
  const auto origin=route.sample(progress);
  Eigen::Vector2d forward=(route.sample(std::min(route.length(),progress+.2))-origin).head<2>();
  if(forward.norm()<1e-6)return out;
  forward.normalize();const Eigen::Vector2d left(-forward.y(),forward.x());
  const double current_side=(body-origin).head<2>().dot(left);
  const std::array<double,2> sides=current_side<-.1?std::array<double,2>{-1.,1.}:std::array<double,2>{1.,-1.};
  const auto collision_free=[&](const Eigen::Vector3d& point,double yaw) {
    if(out.target.queries>=query_limit||!solveAllowed(budget))return false;
    const int state=observed_evidence ? map.getInflateOccupancy(point,yaw) :
        map.getPlanningInflateOccupancy(point,yaw);
    out.target.observe(point,state,yaw);
    return state==0;
  };
  const auto body_support=nearestSupport(support,body.x(),body.y(),body.z()-support.source.body_reference_height_m);
  const bool modeled=explicitBodyModel(support);
  const auto model_body=modeled?modeledBody(support,body):std::optional<Eigen::Vector3d>{};
  if(modeled) {if(!model_body||std::abs(body.z()-model_body->z())>.15)return out;}
  else if(!body_support||std::abs(body.z()-body_support->z-support.source.body_reference_height_m)>.15)return out;
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
    const auto ground=nearestSupport(support,xy.x(),xy.y(),on_route.z()-support.source.body_reference_height_m);
    const auto model_target=modeled?modeledBody(support,{xy.x(),xy.y(),on_route.z()}):std::optional<Eigen::Vector3d>{};
    if(modeled?!model_target:!ground)continue;
    Eigen::Vector3d target=modeled?*model_target:Eigen::Vector3d(xy.x(),xy.y(),ground->z+support.source.body_reference_height_m);
    const auto displacement=(target-body).eval();
    if(displacement.head<2>().norm()<.25||displacement.norm()>std::min(2.,horizon)||
       !supportHeightAllowed(support,modeled?target.z()-model_body->z():ground->z-body_support->z,
           displacement.head<2>().norm()))continue;
    const double yaw=heading_policy==BodyHeadingPolicy::HoldHeading?measured_yaw:
        std::atan2(displacement.y(),displacement.x());
    if(!collision_free(target,yaw))continue;
    // Query the actual body rotation as well; a side target cannot ignore the
    // long rectangular footprint while it turns away from the route tangent.
    const double turn=std::atan2(std::sin(yaw-measured_yaw),std::cos(yaw-measured_yaw));
    const auto shape=map.bodyEnvelope();
    const int turns=std::max(1,int(std::ceil(std::abs(turn)*bodyExtent(shape)/
      std::max(.005,map.getResolution()*.5))));
    if(turns>128)continue;
    bool valid=true;
    for(int i=0;i<=turns&&valid;++i)valid=collision_free(body,measured_yaw+turn*double(i)/turns);
    if(!valid)continue;
    const int steps=std::max(1,int(std::ceil(displacement.head<2>().norm()/std::min(.025,map.getResolution()*.5))));
    if(steps>128)continue;
    std::vector<Eigen::Vector3d> path{body};
    double previous_z=modeled?model_body->z():body_support->z;
    Eigen::Vector2d previous_xy=body.head<2>();
    for(int i=1;i<=steps&&valid;++i) {
      const auto sample_xy=(body.head<2>()+displacement.head<2>()*(double(i)/steps)).eval();
      if(modeled) {
        const double expected_body_z=route.sample(progress+(arc-progress)*(double(i)/steps)).z();
        const auto sample=modeledBody(support,{sample_xy.x(),sample_xy.y(),expected_body_z});
        if(!sample||!supportHeightAllowed(support,sample->z()-previous_z,(sample_xy-previous_xy).norm())||
           !supportHeightAllowed(support,sample->z()-model_body->z(),(sample_xy-body.head<2>()).norm())) {
          valid=false;break;
        }
        valid=collision_free(*sample,yaw);if(valid)path.push_back(*sample);
        previous_z=sample->z();previous_xy=sample_xy;
        continue;
      }
      // The route supplies only the expected level; height is still read from
      // the original classified surface. At a stair/landing or overlapping
      // floors, XY proximity alone cannot pick another surface above/below us.
      const double expected_z=route.sample(progress+(arc-progress)*(double(i)/steps)).z()-
        support.source.body_reference_height_m;
      const auto p=nearestSupport(support,sample_xy.x(),sample_xy.y(),expected_z);
      if(!p||!supportHeightAllowed(support,p->z-previous_z,(sample_xy-previous_xy).norm())||
         !supportHeightAllowed(support,p->z-body_support->z,(sample_xy-body.head<2>()).norm())) {valid=false;break;}
      // Height comes only from the measured source support at this XY, never
      // a copy of the route centerline height onto an unobserved side floor.
      Eigen::Vector3d sample(sample_xy.x(),sample_xy.y(),p->z+support.source.body_reference_height_m);
      valid=collision_free(sample,yaw);if(valid)path.push_back(sample);
      previous_z=p->z;previous_xy=sample_xy;
    }
    if(!valid)continue;
    if constexpr(std::is_same_v<Support,MotionSupport>) {if(braking) {
      if(!map.isRawRaySnapshot()) {out.target.reason="private_raw_snapshot_required";return out;}
      const double remaining=budget?budget->remainingSeconds():.08;
      if(remaining<=0.) {out.target.reason="visible_side_target_budget_exhausted";return out;}
      const auto executable=validateStraightBrakingCorridor(map,support,*braking,body,target,frame,
          std::min(.08,remaining),heading_policy==BodyHeadingPolicy::HoldHeading?
            std::optional<double>(measured_yaw):std::nullopt);
      if(!executable.valid)continue;
    }}
    out.target.valid=true;out.target.arc=arc;out.target.point=path.back();
    out.target.reason="reference_target_visible_side";out.support_path=std::move(path);return out;
  }
  return out;
}
} // namespace side_target_detail

// The production target/A*/optimizer pipeline uses one geometry policy. Raw
// visibility/braking comparisons remain in the explicitly requested legacy
// API below; they are not another gate in formal task_follower_v1 commands.
inline VisibleReferenceTarget selectPlanningSideTarget(
    const DiscreteReference& route,const Eigen::Vector3d& body,double measured_yaw,
    double progress,double horizon,const MotionSupport& support,GridMap& map,const std::string& frame,
    const SolveBudget::Ptr& budget,std::size_t query_limit=1024,
    BodyHeadingPolicy heading_policy=BodyHeadingPolicy::PathTangent) {
  return side_target_detail::selectSideTarget(route,body,measured_yaw,progress,horizon,support,map,frame,
      budget,query_limit,side_target_detail::CollisionPolicy::PlanningGeometry,nullptr,heading_policy);
}

inline VisibleReferenceTarget selectPlanningSideTarget(
    const DiscreteReference& route,const Eigen::Vector3d& body,double measured_yaw,
    double progress,double horizon,const PlanningSupport& support,GridMap& map,const std::string& frame,
    const SolveBudget::Ptr& budget,std::size_t query_limit=1024,
    BodyHeadingPolicy heading_policy=BodyHeadingPolicy::PathTangent) {
  // Flat behavior is deliberately identical to the existing implementation.
  if(const auto* flat=support.flatSupport())return selectPlanningSideTarget(route,body,measured_yaw,
      progress,horizon,*flat,map,frame,budget,query_limit,heading_policy);
  return side_target_detail::selectSideTarget(route,body,measured_yaw,progress,horizon,support,map,frame,
      budget,query_limit,side_target_detail::CollisionPolicy::PlanningGeometry,nullptr,heading_policy);
}

// Existing strict API is retained for callers that explicitly request an
// observed-free side corridor. Never substitute it for motion authorization.
inline VisibleReferenceTarget selectVisibleSideTarget(
    const DiscreteReference& route,const Eigen::Vector3d& body,double measured_yaw,
    double progress,double horizon,const MotionSupport& support,GridMap& map,const std::string& frame,
    const SolveBudget::Ptr& budget,std::size_t query_limit=1024,const BrakingModel* braking=nullptr,
    BodyHeadingPolicy heading_policy=BodyHeadingPolicy::PathTangent) {
  return side_target_detail::selectSideTarget(route,body,measured_yaw,progress,horizon,support,map,frame,
      budget,query_limit,side_target_detail::CollisionPolicy::ObservedEvidence,braking,heading_policy);
}
} // namespace scan_planner
