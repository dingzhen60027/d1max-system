#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

namespace scan_planner {

// Complete curve and yaw-sweep validation shared by primary acceptance and
// optional predecessor evidence. A budget exhaustion is never a safe result.
template<class Query>
bool wholeCurveCollisionFree(UniformBspline &curve, double resolution, double body_extent,
    Query occupied, std::size_t query_budget=200000, double wall_budget_seconds=0.) {
  const auto started=std::chrono::steady_clock::now();
  const auto expired=[&]() {
    return wall_budget_seconds>0. &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>wall_budget_seconds;
  };
  if (!std::isfinite(resolution) || resolution<=0. || !std::isfinite(body_extent) || body_extent<0. ||
      !std::isfinite(wall_budget_seconds) || wall_budget_seconds<0. || !query_budget) return false;
  const double duration=curve.getTimeSum();
  auto velocity=curve.getDerivative();
  const auto controls=velocity.getControlPoint();
  if (!std::isfinite(duration) || duration<=0. || !controls.allFinite()) return false;
  double speed_bound=0.;
  for (int i=0;i<controls.cols();++i) speed_bound=std::max(speed_bound,controls.col(i).norm());
  const double dt=std::min(.02,resolution*.25/std::max(speed_bound,.01));
  if (duration/dt>50000.) return false;
  const int count=std::max(1,static_cast<int>(std::ceil(duration/dt)));
  if (static_cast<std::size_t>(count)+1>query_budget) return false;
  const double yaw_step=std::min(.05,resolution*.25/std::max(body_extent,.01));
  Eigen::Vector3d previous_position;
  double previous_yaw=0.;
  std::size_t queries=0;
  const auto query=[&](const Eigen::Vector3d &p,double yaw) {
    return !expired() && ++queries<=query_budget && occupied(p,yaw)==0;
  };
  for (int i=0;i<=count;++i) {
    const double t=duration*static_cast<double>(i)/count;
    const Eigen::Vector3d p=curve.evaluateDeBoorT(t);
    Eigen::Vector3d tangent=velocity.evaluateDeBoorT(t);
    if (tangent.head<2>().norm()<1e-8)
      tangent=curve.evaluateDeBoorT(std::min(duration,t+dt))-
              curve.evaluateDeBoorT(std::max(0.,t-dt));
    if (!p.allFinite() || !tangent.allFinite()) return false;
    const double yaw=std::atan2(tangent.y(),tangent.x());
    if (!query(p,yaw)) return false;
    if (i>0) {
      const double turn=std::atan2(std::sin(yaw-previous_yaw),std::cos(yaw-previous_yaw));
      const int turns=std::max(1,static_cast<int>(std::ceil(std::abs(turn)/yaw_step)));
      for (int k=1;k<turns;++k) {
        const double ratio=static_cast<double>(k)/turns;
        if (!query(previous_position+ratio*(p-previous_position),previous_yaw+ratio*turn)) return false;
      }
    }
    previous_position=p; previous_yaw=yaw;
  }
  return !expired();
}

}  // namespace scan_planner
