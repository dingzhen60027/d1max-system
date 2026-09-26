#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace scan_planner {

inline double derivativeControlBound(UniformBspline derivative)
{
  const auto points = derivative.getControlPoint();
  if (points.cols() == 0 || !points.allFinite())
    throw std::invalid_argument("invalid B-spline derivative controls");
  double bound = 0.0;
  for (int i = 0; i < points.cols(); ++i) bound = std::max(bound, points.col(i).norm());
  return bound;
}

// B-spline values lie in their control polygon's convex hull. Bounding the
// Euclidean norm of all derivative controls is therefore sufficient for every
// time, unlike per-axis/sample-only checks. Never changes the spatial curve.
inline double enforceDerivativeBounds(UniformBspline &trajectory,
                                      double maximum_speed, double maximum_acceleration)
{
  if (!std::isfinite(maximum_speed) || !std::isfinite(maximum_acceleration) ||
      maximum_speed <= 0.0 || maximum_acceleration <= 0.0)
    throw std::invalid_argument("invalid B-spline derivative limits");
  auto velocity = trajectory.getDerivative();
  const double v = derivativeControlBound(velocity);
  const double a = derivativeControlBound(velocity.getDerivative());
  double scale = std::max({1.0, v / maximum_speed, std::sqrt(a / maximum_acceleration)});
  if (scale > 1.0) {
    scale *= 1.001;  // numerical margin, not a limit relaxation
    trajectory.scaleTime(scale);
  }
  return scale;
}

inline bool stationaryBoundary(const Eigen::Vector3d &velocity,const Eigen::Vector3d &acceleration) {
  return velocity.allFinite() && acceleration.allFinite() &&
      velocity.norm()<=1e-6 && acceleration.norm()<=1e-6;
}

struct CubicMotionBoundary {
  Eigen::Vector3d start_position, start_velocity, start_acceleration;
  Eigen::Vector3d end_position, end_velocity, end_acceleration;
};

// A cubic's endpoint position, velocity and acceleration uniquely determine
// its three endpoint controls. Unlike the legacy overdetermined point/derivative
// least squares, only the interior is fitted; measured motion is a hard equality.
inline Eigen::MatrixXd fitCubicWithFixedBoundary(
    const std::vector<Eigen::Vector3d> &samples, double dt,
    const CubicMotionBoundary &boundary) {
  if (!std::isfinite(dt) || dt<=0. || samples.size()<5 ||
      !boundary.start_position.allFinite() || !boundary.start_velocity.allFinite() ||
      !boundary.start_acceleration.allFinite() || !boundary.end_position.allFinite() ||
      !boundary.end_velocity.allFinite() || !boundary.end_acceleration.allFinite())
    throw std::invalid_argument("invalid fixed cubic boundary");
  const int k=static_cast<int>(samples.size()), n=k+2;
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,n);
  const auto set_endpoint=[&](int offset,const Eigen::Vector3d &p,
      const Eigen::Vector3d &v,const Eigen::Vector3d &a) {
    controls.col(offset)=p-v*dt+a*(dt*dt/3.);
    controls.col(offset+1)=p-a*(dt*dt/6.);
    controls.col(offset+2)=p+v*dt+a*(dt*dt/3.);
  };
  set_endpoint(0,boundary.start_position,boundary.start_velocity,boundary.start_acceleration);
  set_endpoint(n-3,boundary.end_position,boundary.end_velocity,boundary.end_acceleration);
  Eigen::MatrixXd design=Eigen::MatrixXd::Zero(k,n), observations(k,3);
  for (int i=0;i<k;++i) {
    if (!samples[i].allFinite()) throw std::invalid_argument("non-finite cubic sample");
    design(i,i)=1./6.; design(i,i+1)=4./6.; design(i,i+2)=1./6.;
    observations.row(i)=samples[i].transpose();
  }
  observations-=design*controls.transpose();
  controls.middleCols(3,n-6)=design.middleCols(3,n-6).colPivHouseholderQr()
      .solve(observations).transpose();
  if (!controls.allFinite()) throw std::invalid_argument("invalid constrained cubic fit");
  return controls;
}

inline bool cubicBoundaryMatches(UniformBspline trajectory,
    const CubicMotionBoundary &boundary,double tolerance=1e-7) {
  auto velocity=trajectory.getDerivative(), acceleration=velocity.getDerivative();
  const double end=trajectory.getTimeSum();
  return (trajectory.evaluateDeBoorT(0.)-boundary.start_position).norm()<=tolerance &&
      (velocity.evaluateDeBoorT(0.)-boundary.start_velocity).norm()<=tolerance &&
      (acceleration.evaluateDeBoorT(0.)-boundary.start_acceleration).norm()<=tolerance &&
      (trajectory.evaluateDeBoorT(end)-boundary.end_position).norm()<=tolerance &&
      (velocity.evaluateDeBoorT(end)-boundary.end_velocity).norm()<=tolerance &&
      (acceleration.evaluateDeBoorT(end)-boundary.end_acceleration).norm()<=tolerance;
}

struct BoundaryTimingResult {
  bool success=false;
  int refinements=0;
  double speed_bound=0., acceleration_bound=0.;
  std::string reason;
};

// Moving starts cannot be uniformly slowed. Allocate a larger knot interval,
// refit the interior with exact endpoint conditions, and reoptimize that interior.
// Reoptimization can change geometry, so the caller MUST recheck the whole curve
// against the current collision map before publishing. Failure leaves input intact.
template<typename Refine>
inline BoundaryTimingResult refineTimingWithFixedBoundary(UniformBspline &trajectory,
    const CubicMotionBoundary &boundary,double maximum_speed,double maximum_acceleration,
    Refine refine,int max_refinements=8) {
  BoundaryTimingResult result;
  if (!std::isfinite(maximum_speed) || maximum_speed<=0. ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration<=0. || max_refinements<0)
    throw std::invalid_argument("invalid constrained timing limits");
  if (!boundary.start_velocity.allFinite() || !boundary.start_acceleration.allFinite() ||
      !boundary.end_velocity.allFinite() || !boundary.end_acceleration.allFinite()) {
    result.reason="invalid_motion_boundary"; return result;
  }
  // Instantaneously satisfying a lower speed limit while preserving v(0) is
  // mathematically impossible. A separate collision-checked braking policy is
  // required; do not fabricate zero velocity or relax the whole-curve limit.
  if (boundary.start_velocity.norm()>maximum_speed+1e-9) {
    result.reason="initial_speed_exceeds_limit"; return result;
  }
  if (boundary.start_acceleration.norm()>maximum_acceleration+1e-9) {
    result.reason="initial_acceleration_exceeds_limit"; return result;
  }
  if (boundary.end_velocity.norm()>maximum_speed+1e-9 ||
      boundary.end_acceleration.norm()>maximum_acceleration+1e-9) {
    result.reason="terminal_derivative_exceeds_limit"; return result;
  }
  UniformBspline candidate=trajectory, spatial_reference=trajectory;
  const int sample_count=spatial_reference.getControlPoint().cols()-2;
  if (sample_count<5 || !std::isfinite(spatial_reference.getTimeSum()) ||
      spatial_reference.getTimeSum()<=0.) {
    result.reason="invalid_timing_reference"; return result;
  }
  std::vector<Eigen::Vector3d> samples;
  for (int i=0;i<sample_count;++i)
    samples.push_back(spatial_reference.evaluateDeBoorT(
        spatial_reference.getTimeSum()*i/(sample_count-1.)));
  for (int iteration=0;iteration<=max_refinements;++iteration) {
    auto velocity=candidate.getDerivative();
    result.speed_bound=derivativeControlBound(velocity);
    result.acceleration_bound=derivativeControlBound(velocity.getDerivative());
    if (!cubicBoundaryMatches(candidate,boundary)) {
      result.reason="motion_boundary_mismatch"; return result;
    }
    if (result.speed_bound<=maximum_speed+1e-9 &&
        result.acceleration_bound<=maximum_acceleration+1e-9) {
      trajectory=candidate; result.success=true; result.reason="ok"; return result;
    }
    if (iteration==max_refinements) break;
    const double ratio=std::max(result.speed_bound/maximum_speed,
        std::sqrt(result.acceleration_bound/maximum_acceleration));
    const double dt=candidate.getInterval()*std::clamp(ratio*1.025,1.08,2.);
    auto controls=fitCubicWithFixedBoundary(samples,dt,boundary);
    ++result.refinements;
    if (!refine(controls,dt)) {
      result.reason="interior_refinement_failed"; return result;
    }
    candidate=UniformBspline(controls,3,dt);
  }
  result.reason="constrained_timing_exhausted";
  return result;
}

// Uniform retiming changes even the initial derivatives. A moving robot cannot
// silently accept a slower initial velocity/acceleration; keep the input intact
// on rejection. The 1e-3 check is numerical boundary agreement, not speed slack.
inline double enforceDerivativeBoundsAtStart(UniformBspline &trajectory,
    double maximum_speed,double maximum_acceleration,
    const Eigen::Vector3d &start_velocity,const Eigen::Vector3d &start_acceleration) {
  if (!start_velocity.allFinite() || !start_acceleration.allFinite())
    throw std::invalid_argument("invalid initial motion boundary");
  UniformBspline candidate=trajectory;
  const double scale=enforceDerivativeBounds(candidate,maximum_speed,maximum_acceleration);
  if (!stationaryBoundary(start_velocity,start_acceleration)) {
    if (scale>1.+1e-12)
      throw std::invalid_argument("moving initial boundary forbids uniform retiming");
    auto velocity=candidate.getDerivative();
    if ((velocity.evaluateDeBoorT(0.)-start_velocity).norm()>1e-3 ||
        (velocity.getDerivative().evaluateDeBoorT(0.)-start_acceleration).norm()>1e-3)
      throw std::invalid_argument("refined initial derivatives do not match measured motion");
  }
  trajectory=candidate;
  return scale;
}

}  // namespace scan_planner
