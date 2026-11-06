#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/initial_motion_limits.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
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

// This constrains XY tangent turning, not a physical body-yaw trajectory. Below
// the direction domain (including standstill/cusps), hold-yaw and direction
// re-entry still need a separate physical heading policy before execution.
struct TurningRateLimits {
  double maximum_yaw_rate{.50};
  double minimum_direction_speed{.02};
  int maximum_subdivision_depth{12};
  std::size_t maximum_analysis_nodes{8192};
  bool valid() const {
    return std::isfinite(maximum_yaw_rate) && maximum_yaw_rate>0. &&
        std::isfinite(minimum_direction_speed) && minimum_direction_speed>0. &&
        minimum_direction_speed<=.05 && maximum_subdivision_depth>=0 &&
        maximum_subdivision_depth<=20 && maximum_analysis_nodes>0;
  }
};

// Fix the direction domain BEFORE retiming. The threshold is on dp/d(t/T),
// rather than a new physical speed threshold after each scale/refit. Thus a
// uniform slowdown cannot hide a formerly confident bend below .02 m/s. A
// moving refit uses this same normalized domain but is analysed anew because
// its interior geometry is allowed to change.
struct TurningRateDomain {
  double reference_duration{0.};
  double minimum_normalized_direction{0.};
  bool valid() const {
    return std::isfinite(reference_duration) && reference_duration>0. &&
        std::isfinite(minimum_normalized_direction) && minimum_normalized_direction>0.;
  }
};

inline TurningRateDomain turningRateDomain(UniformBspline trajectory,
    const TurningRateLimits &limits) {
  if (!limits.valid()) throw std::invalid_argument("invalid turning rate limits");
  const auto controls=trajectory.getControlPoint();
  auto knots=trajectory.getKnot();
  if (trajectory.getOrder()!=3 || controls.rows()!=3 || controls.cols()<4 ||
      controls.cols()>1024 || !controls.allFinite() || knots.size()!=controls.cols()+4 ||
      !knots.allFinite()) throw std::invalid_argument("invalid turning rate curve");
  for (int i=1;i<knots.size();++i)
    if (!(knots[i]>knots[i-1])) throw std::invalid_argument("invalid turning rate knots");
  const double duration=knots[controls.cols()]-knots[3];
  const TurningRateDomain domain{duration,limits.minimum_direction_speed*duration};
  if (!domain.valid()) throw std::invalid_argument("invalid turning rate domain");
  return domain;
}

enum class TurningRateAnalysisStatus { Complete, InvalidCurve, BudgetExhausted,
  AnalysisLimit, NonFiniteBound };

inline const char *turningRateAnalysisReason(TurningRateAnalysisStatus status) {
  switch(status) {
    case TurningRateAnalysisStatus::Complete:return "ok";
    case TurningRateAnalysisStatus::InvalidCurve:return "invalid_turning_curve";
    case TurningRateAnalysisStatus::BudgetExhausted:return "timing_budget_exhausted";
    case TurningRateAnalysisStatus::AnalysisLimit:return "turning_analysis_limit";
    case TurningRateAnalysisStatus::NonFiniteBound:return "nonfinite_turning_bound";
  }
  return "invalid_turning_analysis";
}

struct TurningRateBound {
  TurningRateAnalysisStatus status{TurningRateAnalysisStatus::InvalidCurve};
  // Conservative continuous bound, only over the declared direction domain.
  // sampled_peak is diagnostic and is never used as the acceptance proof.
  double continuous_bound{0.}, sampled_peak{0.};
  double domain_speed_threshold{0.};
  std::size_t analysis_nodes{0}, low_direction_spans{0};
  bool complete() const { return status==TurningRateAnalysisStatus::Complete; }
};

inline bool timingAllowed(const std::function<bool()> &allowed) {
  return !allowed || allowed();
}

namespace timing_detail {
inline double crossXY(const Eigen::Vector2d &v,const Eigen::Vector2d &a) {
  return v.x()*a.y()-v.y()*a.x();
}
struct VelocitySpan {
  std::array<Eigen::Vector2d,3> controls;
  double duration;
  int depth{0};
};
}  // namespace timing_detail

// On each REAL cubic knot span, XY velocity is quadratic Bernstein and
// cross(v,a) is quadratic (its apparent cubic term cancels). The numerator's
// control hull bounds |cross(v,a)|; the velocity control box's distance to zero
// is a valid speed LOWER bound, unlike the minimum control-point norm. Adaptive
// de Casteljau subdivision tightens this continuous, conservative ratio bound.
// A depth-limited leaf retains its conservative bound; a node/deadline limit is
// explicit failure, never a fabricated finite/zero certificate. Floating-point
// guards are engineering roundoff margins, not a formal interval-arithmetic proof.
inline TurningRateBound boundTurningRate(UniformBspline trajectory,
    const TurningRateLimits &limits,const TurningRateDomain &domain,
    const std::function<bool()> &allowed={}) {
  if (!limits.valid() || !domain.valid())
    throw std::invalid_argument("invalid turning rate policy/domain");
  TurningRateBound result;
  if (!timingAllowed(allowed)) {
    result.status=TurningRateAnalysisStatus::BudgetExhausted; return result;
  }
  const auto controls=trajectory.getControlPoint();
  auto knots=trajectory.getKnot();
  if (trajectory.getOrder()!=3 || controls.rows()!=3 || controls.cols()<4 ||
      controls.cols()>1024 || !controls.allFinite() || knots.size()!=controls.cols()+4 ||
      !knots.allFinite()) return result;
  for (int i=1;i<knots.size();++i) if (!(knots[i]>knots[i-1])) return result;
  const double duration=knots[controls.cols()]-knots[3];
  if (!std::isfinite(duration) || duration<=0. ||
      duration<domain.reference_duration*(1.-1e-12)) return result;
  // evaluateDeBoorT adds the knot origin to t. At a huge finite origin, that
  // addition can round a span midpoint onto an endpoint and reconstruct the
  // WRONG Bernstein polynomial. Normalize the private copy before sampling or
  // differentiating, preserving all represented span differences and leaving
  // the caller's knots/source curve untouched.
  const double original_origin=knots[3];
  for (int i=0;i<knots.size();++i) knots[i]-=original_origin;
  if (!knots.allFinite()) return result;
  for (int i=1;i<knots.size();++i) if (!(knots[i]>knots[i-1])) return result;
  trajectory.setKnot(knots);
  result.domain_speed_threshold=domain.minimum_normalized_direction/duration;
  if (!std::isfinite(result.domain_speed_threshold) || result.domain_speed_threshold<=0.)
    return result;
  const double threshold_squared=result.domain_speed_threshold*result.domain_speed_threshold;
  if (!std::isfinite(threshold_squared) || threshold_squared<=0.) {
    result.status=TurningRateAnalysisStatus::NonFiniteBound; return result;
  }
  auto velocity=trajectory.getDerivative();
  const double roundoff=64.*std::numeric_limits<double>::epsilon();
  for (int span=3;span<controls.cols();++span) {
    if (!timingAllowed(allowed)) {
      result.status=TurningRateAnalysisStatus::BudgetExhausted; return result;
    }
    const double width=knots[span+1]-knots[span], begin=knots[span];
    const double midpoint=begin+width*.5, fraction=(midpoint-begin)/width;
    // A late narrow span in an enormous-duration curve can still have a
    // rounded midpoint after origin normalization. Use its ACTUAL fraction,
    // or explicitly reject if no well-conditioned interior value is representable.
    if (!std::isfinite(width) || width<=0. || !std::isfinite(fraction) ||
        fraction<.25 || fraction>.75) return result;
    const Eigen::Vector2d v0=velocity.evaluateDeBoorT(begin).head<2>();
    const Eigen::Vector2d vm=velocity.evaluateDeBoorT(midpoint).head<2>();
    const Eigen::Vector2d v1=velocity.evaluateDeBoorT(knots[span+1]).head<2>();
    const double one=1.-fraction;
    timing_detail::VelocitySpan initial{{v0,
        (vm-one*one*v0-fraction*fraction*v1)/(2.*fraction*one),v1},width,0};
    std::vector<timing_detail::VelocitySpan> stack{initial};
    bool low_direction=false;
    while (!stack.empty()) {
      if (!timingAllowed(allowed)) {
        result.status=TurningRateAnalysisStatus::BudgetExhausted; return result;
      }
      if (result.analysis_nodes>=limits.maximum_analysis_nodes) {
        result.status=TurningRateAnalysisStatus::AnalysisLimit; return result;
      }
      ++result.analysis_nodes;
      const auto current=stack.back(); stack.pop_back();
      const auto &b=current.controls;
      if (!b[0].allFinite() || !b[1].allFinite() || !b[2].allFinite() ||
          !std::isfinite(current.duration) || current.duration<=0.) {
        result.status=TurningRateAnalysisStatus::NonFiniteBound; return result;
      }
      const double upper_speed=std::max({b[0].norm(),b[1].norm(),b[2].norm()});
      Eigen::Vector2d lower=b[0],upper=b[0];
      for (int i=1;i<3;++i) { lower=lower.cwiseMin(b[i]); upper=upper.cwiseMax(b[i]); }
      const Eigen::Vector2d distance=lower.cwiseMax(0.)+upper.cwiseMin(0.);
      const double lower_speed=std::max(0.,distance.norm()-roundoff*upper_speed);
      if (lower_speed<result.domain_speed_threshold) low_direction=true;
      if (upper_speed*(1.+roundoff)<result.domain_speed_threshold) continue;
      const std::array<double,3> numerator{{
          2.*timing_detail::crossXY(b[0],b[1])/current.duration,
          timing_detail::crossXY(b[0],b[2])/current.duration,
          2.*timing_detail::crossXY(b[1],b[2])/current.duration}};
      const double upper_acceleration=std::max(
          (2.*(b[1]-b[0])/current.duration).norm(),
          (2.*(b[2]-b[1])/current.duration).norm());
      const double numerator_bound=std::nextafter(
          std::max({std::abs(numerator[0]),std::abs(numerator[1]),std::abs(numerator[2])})+
              roundoff*upper_speed*upper_acceleration,std::numeric_limits<double>::infinity());
      const double bound=numerator_bound/std::max(lower_speed*lower_speed,threshold_squared);
      double sampled=0.;
      for (int sample=0;sample<3;++sample) {
        const double u=sample*.5,one=1.-u;
        const Eigen::Vector2d v=one*one*b[0]+2.*one*u*b[1]+u*u*b[2];
        const double squared=v.squaredNorm();
        if (squared>=threshold_squared)
          sampled=std::max(sampled,std::abs(one*one*numerator[0]+
              2.*one*u*numerator[1]+u*u*numerator[2])/squared);
      }
      if (!std::isfinite(bound) || !std::isfinite(sampled)) {
        result.status=TurningRateAnalysisStatus::NonFiniteBound; return result;
      }
      result.sampled_peak=std::max(result.sampled_peak,sampled);
      // Tighten against a REAL global-domain sample lower bound, not merely
      // "already below the limit". This preserves a tight quality diagnostic
      // rather than stopping with a needlessly loose yet feasible upper bound.
      // Sampling only guides subdivision; the retained continuous U is always
      // the acceptance bound. Low-rate leaves need not each converge separately.
      if (current.depth>=limits.maximum_subdivision_depth ||
          bound<=result.sampled_peak*1.01+1e-7) {
        result.continuous_bound=std::max(result.continuous_bound,bound); continue;
      }
      const Eigen::Vector2d left=(b[0]+b[1])*.5,right=(b[1]+b[2])*.5,
          middle=(left+right)*.5;
      stack.push_back({{middle,right,b[2]},current.duration*.5,current.depth+1});
      stack.push_back({{b[0],left,middle},current.duration*.5,current.depth+1});
    }
    if (low_direction) ++result.low_direction_spans;
  }
  result.status=TurningRateAnalysisStatus::Complete;
  return result;
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
  int tightening_trials=0;
  double speed_bound=0., acceleration_bound=0.;
  double nominal_speed_bound=0.;
  double time_scale{1.};
  TurningRateDomain turning_domain;
  TurningRateBound turning;
  std::string reason;
  std::string timing_search_stop_reason;
};

// Stationary starts permit one affine time allocation, preserving every XYZ
// point. The direction domain is frozen on the original geometry, and the new
// curve is independently reanalysed; slowing below a physical threshold is not
// a shortcut around a bend. The caller must still recheck OBB heading/collision:
// its low-physical-speed hold-yaw transition can move when time is scaled.
inline BoundaryTimingResult retimeStationaryWithTurningLimits(UniformBspline &trajectory,
    double maximum_speed,double maximum_acceleration,const TurningRateLimits &limits,
    const std::function<bool()> &allowed={}) {
  if (!std::isfinite(maximum_speed) || maximum_speed<=0. ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration<=0. || !limits.valid())
    throw std::invalid_argument("invalid stationary timing limits");
  BoundaryTimingResult result;
  if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
  result.turning_domain=turningRateDomain(trajectory,limits);
  UniformBspline candidate=trajectory;
  auto velocity=candidate.getDerivative(),acceleration=velocity.getDerivative();
  if (!stationaryBoundary(velocity.evaluateDeBoorT(0.),acceleration.evaluateDeBoorT(0.))) {
    result.reason="moving_initial_boundary_forbids_uniform_timing"; return result;
  }
  result.speed_bound=derivativeControlBound(velocity);
  result.acceleration_bound=derivativeControlBound(acceleration);
  result.turning=boundTurningRate(candidate,limits,result.turning_domain,allowed);
  if (!result.turning.complete()) {
    result.reason=turningRateAnalysisReason(result.turning.status); return result;
  }
  double scale=std::max({1.,result.speed_bound/maximum_speed,
      std::sqrt(result.acceleration_bound/maximum_acceleration),
      result.turning.continuous_bound/limits.maximum_yaw_rate});
  if (!std::isfinite(scale)) { result.reason="nonfinite_timing_scale"; return result; }
  if (scale>1.) {
    scale*=1.005; // roundoff/upper-bound guard, never a relaxed limit
    if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
    candidate.scaleTime(scale);
    result.refinements=1;
    velocity=candidate.getDerivative(); acceleration=velocity.getDerivative();
    result.speed_bound=derivativeControlBound(velocity);
    result.acceleration_bound=derivativeControlBound(acceleration);
    result.turning=boundTurningRate(candidate,limits,result.turning_domain,allowed);
    if (!result.turning.complete()) {
      result.reason=turningRateAnalysisReason(result.turning.status); return result;
    }
  }
  result.time_scale=scale;
  if (result.speed_bound>maximum_speed+1e-9 ||
      result.acceleration_bound>maximum_acceleration+1e-9 ||
      result.turning.continuous_bound>limits.maximum_yaw_rate+1e-9) {
    result.reason="stationary_timing_recheck_failed"; return result;
  }
  if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
  trajectory=candidate; result.success=true; result.reason="ok"; return result;
}

// Moving starts cannot be uniformly slowed. Allocate a larger knot interval,
// refit the interior with exact endpoint conditions, and reoptimize that interior.
// Reoptimization can change geometry, so the caller MUST recheck the whole curve
// against the current collision map before publishing. With turning limits,
// remaining refinements bisect the last failing/first passing knot interval:
// only the shortest ACTUALLY rechecked feasible candidate is retained. Moving
// refits need not be monotone; this is bounded evidence-based search, not a claim
// of a globally minimal duration. The SAME original refinement/deadline quota
// covers expansion and tightening. Failure leaves input intact.
template<typename Refine>
inline BoundaryTimingResult refineTimingWithFixedBoundary(UniformBspline &trajectory,
    const CubicMotionBoundary &boundary,double maximum_speed,double maximum_acceleration,
    Refine refine,int max_refinements=8,
    const std::optional<TurningRateLimits> &turning_limits=std::nullopt,
    const std::function<bool()> &allowed={},bool allow_initial_speed_recovery=false) {
  BoundaryTimingResult result;
  if (!std::isfinite(maximum_speed) || maximum_speed<=0. ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration<=0. || max_refinements<0 ||
      (turning_limits && !turning_limits->valid()))
    throw std::invalid_argument("invalid constrained timing limits");
  if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
  if (!boundary.start_velocity.allFinite() || !boundary.start_acceleration.allFinite() ||
      !boundary.end_velocity.allFinite() || !boundary.end_acceleration.allFinite()) {
    result.reason="invalid_motion_boundary"; return result;
  }
  // Instantaneously satisfying a lower speed limit while preserving v(0) is
  // mathematically impossible. A separate collision-checked braking policy is
  // required; do not fabricate zero velocity or relax the whole-curve limit.
  if (!allow_initial_speed_recovery && boundary.start_velocity.norm()>maximum_speed+1e-9) {
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
  if (turning_limits) result.turning_domain=turningRateDomain(spatial_reference,*turning_limits);
  const int sample_count=spatial_reference.getControlPoint().cols()-2;
  if (sample_count<5 || !std::isfinite(spatial_reference.getTimeSum()) ||
      spatial_reference.getTimeSum()<=0.) {
    result.reason="invalid_timing_reference"; return result;
  }
  std::vector<Eigen::Vector3d> samples;
  for (int i=0;i<sample_count;++i) {
    if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
    samples.push_back(spatial_reference.evaluateDeBoorT(
        spatial_reference.getTimeSum()*i/(sample_count-1.)));
  }
  const auto inspect=[&](UniformBspline &curve,BoundaryTimingResult &check) {
    if (!timingAllowed(allowed)) { check.reason="timing_budget_exhausted"; return false; }
    if(allow_initial_speed_recovery&&boundary.start_velocity.norm()>maximum_speed+1e-9)
      projectInitialRecoveryInterior(curve,maximum_speed,maximum_acceleration,allowed);
    auto velocity=curve.getDerivative();
    check.speed_bound=derivativeControlBound(velocity);
    check.nominal_speed_bound=check.speed_bound;
    check.acceleration_bound=derivativeControlBound(velocity.getDerivative());
    if(allow_initial_speed_recovery) {
      const auto limits=inspectInitialMotionLimits(curve,maximum_speed,maximum_acceleration);
      if(limits.recovering) {
        if(limits.recovery_end<=0.||limits.recovery_end>limits.recovery_limit+1e-9) {
          check.reason=limits.reason;return false;
        }
        check.nominal_speed_bound=limits.nominal_speed_bound;
      }
    }
    if (!cubicBoundaryMatches(curve,boundary)) {
      check.reason="motion_boundary_mismatch"; return false;
    }
    if (turning_limits) {
      const double threshold=check.turning_domain.minimum_normalized_direction/curve.getTimeSum();
      const auto boundary_rate=[threshold](const Eigen::Vector3d &v,const Eigen::Vector3d &a) {
        const double squared=v.head<2>().squaredNorm();
        return squared>=threshold*threshold ?
            std::abs(timing_detail::crossXY(v.head<2>(),a.head<2>()))/squared : 0.;
      };
      // Re-evaluate each enlarged-duration domain. A retained small but real
      // initial velocity can become direction-confident later; no refit can
      // change its prescribed instantaneous p/v/a to cure an excessive rate.
      if (boundary_rate(boundary.start_velocity,boundary.start_acceleration)>
          turning_limits->maximum_yaw_rate+1e-9) {
        check.reason="initial_turning_rate_exceeds_limit"; return false;
      }
      if (boundary_rate(boundary.end_velocity,boundary.end_acceleration)>
          turning_limits->maximum_yaw_rate+1e-9) {
        check.reason="terminal_turning_rate_exceeds_limit"; return false;
      }
      check.turning=boundTurningRate(curve,*turning_limits,check.turning_domain,allowed);
      if (!check.turning.complete()) {
        check.reason=turningRateAnalysisReason(check.turning.status); return false;
      }
    }
    check.reason.clear(); return true;
  };
  const auto feasible=[&](const BoundaryTimingResult &check) {
    return check.nominal_speed_bound<=maximum_speed+1e-9 &&
        check.acceleration_bound<=maximum_acceleration+1e-9 &&
        (!turning_limits || check.turning.continuous_bound<=turning_limits->maximum_yaw_rate+1e-9);
  };
  double last_infeasible_dt=0.;
  for (int iteration=0;iteration<=max_refinements;++iteration) {
    if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
    if (!inspect(candidate,result)) return result;
    if (feasible(result)) {
      UniformBspline best_candidate=candidate;
      BoundaryTimingResult best_result=result;
      double low_dt=last_infeasible_dt, high_dt=candidate.getInterval();
      int total_refinements=result.refinements, trials=0;
      std::string stop_reason="original_curve_feasible";
      if (turning_limits && low_dt>0.) {
        stop_reason="refinement_quota";
        while (total_refinements<max_refinements) {
          if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
          if (high_dt-low_dt<=high_dt*.01) { stop_reason="verified_bracket_tight"; break; }
          const double dt=low_dt+(high_dt-low_dt)*.5;
          auto controls=fitCubicWithFixedBoundary(samples,dt,boundary);
          ++total_refinements; ++trials;
          result.refinements=total_refinements; result.tightening_trials=trials;
          if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
          if (!refine(controls,dt)) {
            if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
            stop_reason="optional_interior_refinement_failed"; break;
          }
          if (controls.rows()!=3 || controls.cols()!=spatial_reference.getControlPoint().cols() ||
              !controls.allFinite()) {
            stop_reason="optional_invalid_refinement_output"; break;
          }
          UniformBspline trial(controls,3,dt);
          BoundaryTimingResult trial_result=best_result;
          if (!inspect(trial,trial_result)) {
            if (!timingAllowed(allowed) || trial_result.reason=="timing_budget_exhausted") {
              result.reason="timing_budget_exhausted"; return result;
            }
            // Failed optional work does not erase an already analysed curve.
            // Its reason is preserved; never substitute the unchecked trial.
            stop_reason=trial_result.reason; break;
          }
          if (feasible(trial_result)) {
            best_candidate=trial; best_result=trial_result; high_dt=dt;
          } else low_dt=dt;
        }
      }
      if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
      result=best_result;
      result.refinements=total_refinements; result.tightening_trials=trials;
      result.timing_search_stop_reason=stop_reason;
      result.time_scale=best_candidate.getTimeSum()/spatial_reference.getTimeSum();
      trajectory=best_candidate; result.success=true; result.reason="ok"; return result;
    }
    if (iteration==max_refinements) break;
    last_infeasible_dt=candidate.getInterval();
    const double turning_ratio=turning_limits ?
        result.turning.continuous_bound/turning_limits->maximum_yaw_rate : 1.;
    const double ratio=std::max({result.nominal_speed_bound/maximum_speed,
        std::sqrt(result.acceleration_bound/maximum_acceleration),turning_ratio});
    const double dt=candidate.getInterval()*std::clamp(ratio*1.025,1.08,2.);
    if (!std::isfinite(dt) || dt<=0.) { result.reason="nonfinite_timing_scale"; return result; }
    if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
    auto controls=fitCubicWithFixedBoundary(samples,dt,boundary);
    ++result.refinements;
    if (!timingAllowed(allowed)) { result.reason="timing_budget_exhausted"; return result; }
    if (!refine(controls,dt)) {
      result.reason=timingAllowed(allowed)?"interior_refinement_failed":"timing_budget_exhausted";
      return result;
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
