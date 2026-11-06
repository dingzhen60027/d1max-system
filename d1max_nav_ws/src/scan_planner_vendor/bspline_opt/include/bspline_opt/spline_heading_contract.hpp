#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <optional>
#include <memory>
#include <vector>

namespace scan_planner {

enum class BodyHeadingPolicy { PathTangent, HoldHeading };

// An immutable, curve-local spatial parameterization for the exceptional
// backwards acquired entry.  Its knots are the existing Native full-curve
// lattice, clipped at the existing 0.4 m horizon.  Copies of a frozen heading
// contract share this table; a control query never walks the complete curve.
struct ForwardEntryHeadingKnot {
  double time{0.}, arc{0.}, arc_rate{0.}, tangent_yaw{0.};
};
struct ForwardEntryHeadingProfile {
  std::vector<ForwardEntryHeadingKnot> knots;
  double seed_yaw{0.};
  double blend_start_arc{0.};
  double blend_end_arc{0.};
};
struct HeadingGeometrySample {
  double yaw{0.};
  // d(body heading)/d(actual XY arc), not the curvature of the XYZ tangent
  // when the acquired velocity briefly points backwards.
  double curvature{0.};
};
inline BodyHeadingPolicy parseBodyHeadingPolicy(const std::string &name) {
  if(name=="path_tangent")return BodyHeadingPolicy::PathTangent;
  if(name=="hold_heading")return BodyHeadingPolicy::HoldHeading;
  throw std::invalid_argument("unsupported body heading policy");
}

// Collision geometry is independent of whether the session owns a motion
// writer. Formal projected-ray sessions have a measured body pose contract in
// both preview and execution; opting in here grants NO execution permission.
// Unmanaged legacy callers retain their existing opt-in/fallback behavior.
inline bool measuredHeadingGeometryContract(bool legacy_requested,
    bool grid_preview_only, bool execution_protocol, bool writer_handoff_enabled,
    bool projected_rays, bool planning_requires_observed_free) {
  (void)writer_handoff_enabled;
  if (execution_protocol) {
    if (!projected_rays)
      throw std::invalid_argument("measured heading geometry requires projected-ray pose provenance");
    return true;
  }
  if (!legacy_requested) return false;
  if (!projected_rays || !grid_preview_only || planning_requires_observed_free)
    throw std::invalid_argument("legacy measured heading geometry requires supported preview inputs");
  return true;
}

// The BT preview is identified by the actual writer contract, not the legacy
// GridMap preview label. Formal sessions keep the execution protocol/map
// evidence pipeline active while deliberately omitting the motion writer.
// Never infer this mode from a planning collision policy (which says nothing
// about whether a trajectory is allowed to drive a robot).
inline bool noMotionMeasuredHeadingContract(bool legacy_requested,
    bool grid_preview_only, bool execution_protocol, bool writer_handoff_enabled,
    bool projected_rays, bool planning_requires_observed_free) {
  const bool managed_preview=execution_protocol && !writer_handoff_enabled;
  if (!legacy_requested && !managed_preview) return false;
  if (!projected_rays || (legacy_requested && execution_protocol && writer_handoff_enabled) ||
      (!managed_preview && (!grid_preview_only || planning_requires_observed_free)))
    throw std::invalid_argument("measured heading contract requires explicit no-motion projected-ray preview");
  return true;
}

// Measured-body geometry, not a motion authorization. Velocity is NOT changed:
// a mm/s position derivative is not a trustworthy estimate of body orientation.
// 0.02 m/s is an engineering direction-confidence threshold above the observed
// 1--10 mm/s stationary jitter, NOT a calibrated stop/motion detection threshold.
struct SplineHeadingContract {
  // Compatibility label for old no-motion callers/serialized snapshots. New
  // callers select geometry using measured_heading_enabled instead.
  bool preview_only_enabled{false};
  double measured_yaw{0.};
  double minimum_direction_speed{.02};
  double body_extent{.49};
  bool measured_heading_enabled{false};
  // Held yaw is acquired at solve admission and frozen with that curve. It is
  // never inferred from lateral velocity or replaced by the latest body yaw.
  BodyHeadingPolicy policy{BodyHeadingPolicy::PathTangent};
  // Explicit navigation strategy, frozen per curve. Disabled legacy/no-motion
  // callers retain low-speed measured-heading behavior; held yaw never turns.
  bool align_then_forward{false};
  // Derived from this immutable curve + its original measured yaw, never
  // serialized as a new measurement. A short acquired backwards entry may
  // have negative velocity while the robot's head faces its forward route.
  // Every geometry reader reconstructs this same bounded prefix.
  double forward_entry_end{-1.};
  double forward_entry_yaw{0.};
  std::shared_ptr<const ForwardEntryHeadingProfile> forward_entry_profile;
};

inline bool measuredHeadingEnabled(const SplineHeadingContract &contract) {
  return contract.measured_heading_enabled || contract.preview_only_enabled;
}

inline bool validMeasuredHeadingContract(const SplineHeadingContract &contract) {
  return (contract.policy==BodyHeadingPolicy::PathTangent||contract.policy==BodyHeadingPolicy::HoldHeading)&&
      (contract.policy!=BodyHeadingPolicy::HoldHeading||measuredHeadingEnabled(contract))&&
      std::isfinite(contract.measured_yaw) &&
      std::isfinite(contract.minimum_direction_speed) &&
      contract.minimum_direction_speed>0. && contract.minimum_direction_speed<=.05 &&
      std::isfinite(contract.forward_entry_end)&&contract.forward_entry_end>=-1.&&contract.forward_entry_end<=120.&&
      std::isfinite(contract.forward_entry_yaw)&&
      std::isfinite(contract.body_extent) && contract.body_extent>0.;
}

inline double bodyHeadingFromVelocity(const Eigen::Vector3d &velocity,double previous_yaw,
                                     const SplineHeadingContract &contract) {
  if(contract.policy==BodyHeadingPolicy::HoldHeading)return contract.measured_yaw;
  return velocity.head<2>().norm()<contract.minimum_direction_speed ? previous_yaw :
      std::atan2(velocity.y(),velocity.x());
}

struct ForwardEntryArcSample {
  double arc{0.}, rate{0.}, previous_tangent_yaw{0.};
};
inline std::optional<ForwardEntryArcSample> forwardEntryArcAt(
    const SplineHeadingContract& contract,double time) {
  const auto& profile=contract.forward_entry_profile;
  if(!profile||profile->knots.size()<2||!std::isfinite(time)||time<0.||
     time>contract.forward_entry_end)return {};
  const auto& knots=profile->knots;
  auto upper=std::upper_bound(knots.begin(),knots.end(),time,
      [](double t,const ForwardEntryHeadingKnot& knot){return t<knot.time;});
  const std::size_t right=upper==knots.end()?knots.size()-1:
      static_cast<std::size_t>(upper-knots.begin());
  const auto& a=knots[right-1];const auto& b=knots[right];
  const double dt=b.time-a.time;
  const double u=std::clamp((time-a.time)/dt,0.,1.);
  const double u2=u*u,u3=u2*u;
  // C1, monotone cubic Hermite arc interpolation. The shared node slopes
  // have been limited once during construction, not independently per query.
  const double arc=(2.*u3-3.*u2+1.)*a.arc+(u3-2.*u2+u)*dt*a.arc_rate+
      (-2.*u3+3.*u2)*b.arc+(u3-u2)*dt*b.arc_rate;
  const double rate=((6.*u2-6.*u)*a.arc+(-6.*u2+6.*u)*b.arc)/dt+
      (3.*u2-4.*u+1.)*a.arc_rate+(3.*u2-2.*u)*b.arc_rate;
  return ForwardEntryArcSample{arc,std::max(0.,rate),a.tangent_yaw};
}

// Shared orientation AND its spatial derivative. The zero-speed cusp lies
// before the blend, so it retains the original measured heading without
// dividing by its speed. In the blend, the chain rule includes the derivative
// of the same interpolated arc used to evaluate the orientation. In particular
// a table's chord rate must not silently be treated as the true curve speed.
inline std::optional<HeadingGeometrySample> forwardEntryHeadingGeometryAt(
    double time,const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration,
    const SplineHeadingContract& contract) {
  if(!contract.align_then_forward||contract.policy!=BodyHeadingPolicy::PathTangent||
     !contract.forward_entry_profile||time<0.||time>contract.forward_entry_end)return {};
  const auto state=forwardEntryArcAt(contract,time);if(!state)return {};
  const auto& profile=*contract.forward_entry_profile;
  if(state->arc<=profile.blend_start_arc)
    return HeadingGeometrySample{profile.seed_yaw,0.};
  const double span=profile.blend_end_arc-profile.blend_start_arc;
  const double u=std::clamp((state->arc-profile.blend_start_arc)/span,0.,1.);
  const double u2=u*u,u3=u2*u;
  const double weight=u3*(10.+u*(-15.+6.*u));
  const double weight_per_arc=30.*u2*(1.-u)*(1.-u)/span;
  const double raw=std::atan2(velocity.y(),velocity.x());
  const double tangent=state->previous_tangent_yaw+
      std::atan2(std::sin(raw-state->previous_tangent_yaw),
                 std::cos(raw-state->previous_tangent_yaw));
  const double delta=tangent-profile.seed_yaw;
  const double yaw=profile.seed_yaw+weight*delta;
  const double speed=velocity.head<2>().norm();
  // Construction puts the blend after the final sampled backwards/low-speed
  // part. A truly stationary query has no traveled arc or yaw feed-forward.
  // No motion boundary or original velocity is replaced here.
  double curvature=0.;
  if(speed>1e-12) {
    const double raw_curvature=(velocity.x()*acceleration.y()-velocity.y()*acceleration.x())/
        (speed*speed*speed);
    curvature=weight*raw_curvature+weight_per_arc*state->rate/speed*delta;
  }
  return HeadingGeometrySample{std::atan2(std::sin(yaw),std::cos(yaw)),curvature};
}

inline double bodyHeadingAtCurveTime(const Eigen::Vector3d& velocity,double previous_yaw,
                                    const SplineHeadingContract& contract,double time) {
  if(const auto sample=forwardEntryHeadingGeometryAt(time,velocity,Eigen::Vector3d::Zero(),contract))
    return sample->yaw;
  if(contract.align_then_forward&&contract.policy==BodyHeadingPolicy::PathTangent&&
     contract.forward_entry_end>=0.&&time<contract.forward_entry_end)
    return contract.forward_entry_yaw;
  return bodyHeadingFromVelocity(velocity,previous_yaw,contract);
}

// Preserve the production full spatial/time heading lattice. Transmitting its
// exact size alongside the original seed prevents consumers with a different
// entry-projection table from inventing another low-speed heading history.
inline std::optional<std::uint32_t> bodyHeadingSamplingIntervals(double duration,
    double derivative_control_speed_bound,double resolution) {
  if(!std::isfinite(duration)||duration<=0.||!std::isfinite(derivative_control_speed_bound)||
     derivative_control_speed_bound<0.||!std::isfinite(resolution)||resolution<=0.)return {};
  const auto count=std::ceil(std::max(duration*derivative_control_speed_bound/(resolution*.25),duration/.02));
  if(!std::isfinite(count)||count>49999.)return {};
  return std::max<std::uint32_t>(1,static_cast<std::uint32_t>(count));
}

// One shared deterministic replay of the FROZEN full-curve heading memory.
// Geometry queries keep their existing sampling/budget; this supplies only
// the same orientation at a bounded curve time. allowed() supports preparation
// cancellation and native's original shared deadline, never a renewed lease.
template<class VelocityAt,class Allowed>
std::optional<double> sampledBodyHeadingAt(double time,double duration,std::uint32_t intervals,
    const SplineHeadingContract& contract,VelocityAt velocity_at,Allowed allowed) {
  if(!std::isfinite(time)||!std::isfinite(duration)||duration<=0.||time<0.||time>duration||
     !intervals||intervals>49999||!validMeasuredHeadingContract(contract)||!allowed())return {};
  if(contract.policy==BodyHeadingPolicy::HoldHeading)return contract.measured_yaw;
  if(contract.forward_entry_profile&&time<=contract.forward_entry_end) {
    const Eigen::Vector3d velocity=velocity_at(time);if(!velocity.allFinite())return {};
    return bodyHeadingAtCurveTime(velocity,contract.measured_yaw,contract,time);
  }
  double yaw=contract.measured_yaw;
  for(std::uint32_t i=0;i<=intervals;++i) {
    const double at=duration*static_cast<double>(i)/intervals;
    if(at>time)break;
    if(!allowed())return {};
    const Eigen::Vector3d velocity=velocity_at(at);if(!velocity.allFinite())return {};
    yaw=bodyHeadingAtCurveTime(velocity,yaw,contract,at);
  }
  if(!allowed())return {};
  const Eigen::Vector3d velocity=velocity_at(time);if(!velocity.allFinite())return {};
  return bodyHeadingAtCurveTime(velocity,yaw,contract,time);
}

// Existing Bspline yaw controls transport the SAME frozen geometry policy to
// the tracker. Constant yaw has exactly the XYZ control count and interval;
// empty yaw controls retain the legacy/default path-tangent contract.
template<class BsplineMessage>
bool encodeFrozenHeading(BsplineMessage &spline,const SplineHeadingContract &contract) {
  if(contract.policy==BodyHeadingPolicy::PathTangent) {
    spline.yaw_pts.clear();spline.yaw_dt=0.;return true;
  }
  if(!validMeasuredHeadingContract(contract)||spline.order!=3||spline.pos_pts.size()<4||
     spline.knots.size()!=spline.pos_pts.size()+4)return false;
  const double interval=spline.knots[4]-spline.knots[3];
  if(!std::isfinite(spline.knots.front())||!std::isfinite(interval)||interval<=0.)return false;
  for(std::size_t i=1;i<spline.knots.size();++i)
    if(!std::isfinite(spline.knots[i])||
       std::abs(spline.knots[i]-spline.knots[i-1]-interval)>1e-8*std::max(1.,interval))return false;
  spline.yaw_pts.assign(spline.pos_pts.size(),contract.measured_yaw);spline.yaw_dt=interval;
  return true;
}
template<class BsplineMessage>
bool frozenHeadingWireMatches(const BsplineMessage &spline,const SplineHeadingContract &contract) {
  if(contract.policy==BodyHeadingPolicy::PathTangent)return spline.yaw_pts.empty()&&spline.yaw_dt==0.;
  if(!validMeasuredHeadingContract(contract)||spline.order!=3||spline.pos_pts.size()<4||
     spline.knots.size()!=spline.pos_pts.size()+4||spline.yaw_pts.size()!=spline.pos_pts.size())return false;
  const double interval=spline.knots[4]-spline.knots[3];
  if(!std::isfinite(spline.knots.front())||!std::isfinite(interval)||interval<=0.||!std::isfinite(spline.yaw_dt)||
     std::abs(spline.yaw_dt-interval)>1e-8*std::max(1.,interval))return false;
  for(std::size_t i=1;i<spline.knots.size();++i)
    if(!std::isfinite(spline.knots[i])||
       std::abs(spline.knots[i]-spline.knots[i-1]-interval)>1e-8*std::max(1.,interval))return false;
  for(double yaw:spline.yaw_pts)
    if(!std::isfinite(yaw)||std::abs(yaw-contract.measured_yaw)>1e-12)return false;
  return true;
}

// Source-compatible wrappers. They carry no authorization semantics.
inline bool validPreviewHeadingContract(const SplineHeadingContract &contract) {
  return validMeasuredHeadingContract(contract);
}
inline double previewBodyHeading(const Eigen::Vector3d &velocity,double previous_yaw,
                                const SplineHeadingContract &contract) {
  return bodyHeadingFromVelocity(velocity,previous_yaw,contract);
}

// Full translation x rotation sweep, including BOTH endpoint positions and
// every intermediate heading. Changing direction confidence cannot jump over
// a colliding rotation. The caller owns the shared query/time budget.
template<class Query>
bool headingSweepFree(const Eigen::Vector3d &from,const Eigen::Vector3d &to,
    double from_yaw,double turn,double resolution,double body_extent,Query query) {
  if (!from.allFinite() || !to.allFinite() || !std::isfinite(from_yaw) ||
      !std::isfinite(turn) || !std::isfinite(resolution) || resolution<=0. ||
      !std::isfinite(body_extent) || body_extent<=0.) return false;
  const double yaw_step=std::min(.05,resolution*.25/body_extent);
  const double linear_count=std::ceil((to-from).norm()/(resolution*.25));
  const double angular_count=std::ceil(std::abs(turn)/yaw_step);
  if (!std::isfinite(linear_count) || !std::isfinite(angular_count) ||
      linear_count>50000. || angular_count>50000.) return false;
  const int translations=std::max(1,static_cast<int>(linear_count));
  const int turns=std::max(1,static_cast<int>(angular_count));
  for(int j=0;j<=translations;++j) {
    const Eigen::Vector3d p=from+(to-from)*(static_cast<double>(j)/translations);
    for(int k=0;k<=turns;++k)
      if (!query(p,from_yaw+turn*(static_cast<double>(k)/turns))) return false;
  }
  return true;
}

template<class Query>
bool headingTransitionFree(const Eigen::Vector3d &from,const Eigen::Vector3d &to,
    double from_yaw,double to_yaw,double resolution,double body_extent,Query query) {
  if(!std::isfinite(to_yaw))return false;
  const double turn=std::atan2(std::sin(to_yaw-from_yaw),std::cos(to_yaw-from_yaw));
  return headingSweepFree(from,to,from_yaw,turn,resolution,body_extent,query);
}

// A forward-only controller may retain either rear-turn direction across
// replans to avoid +/-pi oscillation. The current wire carries no signed turn
// branch. Certify BOTH branches at this measured body connection/terminal
// turn, using one full rotation whenever the destination is behind the body.
// This is not a curve-segment policy: ordinary forward and adjacent path
// headings retain their original shortest sweep. Caller query/time budgets
// are shared unchanged; an incomplete rotation is never certified free.
inline bool rearBodyHeadingTransition(double from_yaw,double to_yaw) {
  constexpr double pi=3.14159265358979323846;
  const double shortest=std::atan2(std::sin(to_yaw-from_yaw),std::cos(to_yaw-from_yaw));
  return std::abs(shortest)>pi/2.;
}

template<class Query>
bool bodyEntryHeadingTransitionFree(const Eigen::Vector3d &from,const Eigen::Vector3d &to,
    double from_yaw,double to_yaw,double resolution,double body_extent,Query query) {
  if(!std::isfinite(to_yaw))return false;
  constexpr double pi=3.14159265358979323846;
  const double shortest=std::atan2(std::sin(to_yaw-from_yaw),std::cos(to_yaw-from_yaw));
  const bool rear=rearBodyHeadingTransition(from_yaw,to_yaw);
  const double turn=rear?2.*pi:shortest;
  if(!headingSweepFree(from,to,from_yaw,turn,resolution,body_extent,query))return false;
  // Full-circle sampling ends at the original heading. Also query the exact
  // destination heading, not merely its two nearest angular samples.
  return !rear||headingSweepFree(from,to,to_yaw,0.,resolution,body_extent,query);
}

} // namespace scan_planner
