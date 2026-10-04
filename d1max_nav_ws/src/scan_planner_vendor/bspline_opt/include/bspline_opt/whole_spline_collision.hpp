#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/spline_heading_contract.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace scan_planner {

enum class SplineCollisionState { Clear, Collision, InvalidInput, SampleBudgetExceeded };

struct SplineCollisionResult {
  SplineCollisionState state{SplineCollisionState::InvalidInput};
  double time{0.};
  int occupancy{0};
  std::size_t queries{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double yaw{0.};
};

// Shared full-curve query for internal optimization and preview admission.
// Cubic derivative control points bound speed by the convex-hull property,
// so each time interval travels at most resolution/4, even for a closed loop.
// Default execution behavior remains tangent based. The explicitly enabled
// no-motion preview contract additionally validates measured yaw and all sweeps;
// the outer admission check still owns body pose freshness and source validity.
template<class Query>
SplineCollisionResult checkWholeSplineCollision(UniformBspline &curve,double resolution,
    Query occupied,std::size_t query_budget=50000,
    const SplineHeadingContract &heading={}) {
  SplineCollisionResult result;
  const auto points=curve.getControlPoint();
  // Check the empty/default spline before reading its uninitialized scalar data.
  if (points.rows()!=3 || points.cols()<4 || !points.allFinite() ||
      !std::isfinite(resolution) || resolution<=0. || query_budget<2) return result;
  if (heading.preview_only_enabled && !validPreviewHeadingContract(heading)) return result;
  query_budget=std::min<std::size_t>(query_budget,50000);
  if (curve.getOrder()!=3 || !std::isfinite(curve.getInterval()) || curve.getInterval()<=0.)
    return result;
  const auto knots=curve.getKnot();
  if (knots.size()!=points.cols()+4 || !knots.allFinite()) return result;
  // Production uniform/retimed cubic knots are strictly increasing. Reject
  // degenerate knots before derivative division or de Boor evaluation.
  for (int i=1;i<knots.size();++i) if (knots[i]<=knots[i-1]) return result;
  const double duration=curve.getTimeSum();
  if (!std::isfinite(duration) || duration<=0.) return result;
  auto velocity=curve.getDerivative();
  const auto velocity_points=velocity.getControlPoint();
  if (velocity_points.rows()!=3 || !velocity_points.allFinite()) return result;
  double speed_bound=0.,previous_yaw=0.;
  bool have_initial_heading=false;
  for (int i=0;i<velocity_points.cols();++i) {
    speed_bound=std::max(speed_bound,velocity_points.col(i).norm());
    if (!have_initial_heading && velocity_points.col(i).head<2>().norm()>1e-8) {
      previous_yaw=std::atan2(velocity_points(1,i),velocity_points(0,i));
      have_initial_heading=true;
    }
  }
  if (!std::isfinite(speed_bound)) return result;
  if (heading.preview_only_enabled) previous_yaw=heading.measured_yaw;
  const double required=std::ceil(std::max(duration*speed_bound/(resolution*.25),
      heading.preview_only_enabled ? duration/.02 : 0.));
  if (!std::isfinite(required) || required>static_cast<double>(query_budget-1)) {
    result.state=SplineCollisionState::SampleBudgetExceeded;return result;
  }
  const std::size_t intervals=std::max<std::size_t>(1,static_cast<std::size_t>(required));
  const double dt=duration/static_cast<double>(intervals);
  Eigen::Vector3d previous_position=curve.evaluateDeBoorT(0.);
  for (std::size_t i=0;i<=intervals;++i) {
    result.time=duration*(static_cast<double>(i)/intervals);
    const Eigen::Vector3d position=curve.evaluateDeBoorT(result.time);
    Eigen::Vector3d tangent=velocity.evaluateDeBoorT(result.time);
    if (!position.allFinite() || !tangent.allFinite()) return result;
    result.velocity=tangent;
    if (heading.preview_only_enabled) {
      const double yaw=previewBodyHeading(tangent,previous_yaw,heading);
      const auto query=[&](const Eigen::Vector3d &p,double candidate_yaw) {
        result.position=p;result.yaw=candidate_yaw;
        if (result.queries>=query_budget) {
          result.state=SplineCollisionState::SampleBudgetExceeded;return false;
        }
        ++result.queries;result.occupancy=occupied(p,candidate_yaw);
        if (result.occupancy!=0) {result.state=SplineCollisionState::Collision;return false;}
        return true;
      };
      if (!headingTransitionFree(previous_position,position,previous_yaw,yaw,
                                  resolution,heading.body_extent,query)) return result;
      previous_position=position;previous_yaw=yaw;
      continue;
    }
    if (tangent.head<2>().norm()<1e-8) {
      tangent=curve.evaluateDeBoorT(std::min(duration,result.time+dt))-
              curve.evaluateDeBoorT(std::max(0.,result.time-dt));
      if (!tangent.allFinite()) return result;
    }
    const double yaw=tangent.head<2>().norm()<1e-8 ? previous_yaw :
        std::atan2(tangent.y(),tangent.x());
    if (!std::isfinite(yaw)) return result;
    ++result.queries;
    result.position=position;result.yaw=yaw;
    result.occupancy=occupied(position,yaw);
    if (result.occupancy!=0) {result.state=SplineCollisionState::Collision;return result;}
    previous_yaw=yaw;
  }
  result.state=SplineCollisionState::Clear;
  return result;
}

} // namespace scan_planner
