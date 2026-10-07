#pragma once

#include <plan_manage/trajectory_collision.hpp>
#include <bspline_opt/trajectory_timing.hpp>
#include <plan_env/solve_budget.hpp>

namespace scan_planner {

struct CandidateJoinEvidence {
  double curve_time{0.};
  double arc_length{0.};
  MeasuredBodyPose measured;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool acceleration_valid{false};
};

// The original worker curve is immutable. Only its certified entry parameter
// changes; neither elapsed publication time nor a remote nearest point is an
// execution position. Keep the existing resolution/4 connector tolerance.
inline std::optional<CandidateJoinEvidence> measuredCandidateJoin(
    UniformBspline &curve, const MeasuredBodyPose &solve_body,
    const MeasuredBodyPose &current_body, const Eigen::Vector3d &velocity,
    double resolution, double max_speed, double max_acceleration,
    bool single_segment, const SolveBudget::Ptr &budget,double measured_travel_max_speed=0.) {
  // Legacy callers keep one bound. A sealed isolated model supplies an
  // independent measured XYZ travel bound; it never changes curve geometry.
  const double travel_speed=measured_travel_max_speed==0.?max_speed:measured_travel_max_speed;
  if (!budget || !budget->allowed() || !velocity.allFinite() ||
      !solve_body.position.allFinite() || !current_body.position.allFinite() ||
      solve_body.frame!=current_body.frame || current_body.frame.empty() ||
      !std::isfinite(resolution) || resolution<=0. || !std::isfinite(max_speed) ||
      max_speed<=0. || !std::isfinite(travel_speed)||travel_speed<=0. ||
      !std::isfinite(max_acceleration) || max_acceleration<=0. ||
      (measured_travel_max_speed!=0.&&velocity.stableNorm()>travel_speed)) return {};
  const auto solve_ns=measuredBodySourceNs(solve_body),current_ns=measuredBodySourceNs(current_body);
  if (solve_ns<=0||current_ns<solve_ns||current_ns-solve_ns>400000000LL)
    return {};
  const double elapsed=static_cast<double>(current_ns-solve_ns)*1e-9;
  const auto controls=curve.getControlPoint();
  const auto knots=curve.getKnot();
  if (curve.getOrder()!=3 || controls.rows()!=3 || controls.cols()<4 ||
      !controls.allFinite() || knots.size()!=controls.cols()+4 || !knots.allFinite() ||
      !std::isfinite(curve.getInterval()) || curve.getInterval()<=0.) return {};
  for (int i=1;i<knots.size();++i) if (knots[i]<=knots[i-1]) return {};
  const double duration=curve.getTimeSum();
  if (!std::isfinite(duration) || duration<=0. || duration>120.) return {};
  auto derivative=curve.getDerivative();
  auto acceleration=derivative.getDerivative();
  if (!derivative.getControlPoint().allFinite() ||
      !acceleration.getControlPoint().allFinite()) return {};
  const double speed_bound=derivativeControlBound(derivative);
  if (!std::isfinite(speed_bound) || speed_bound>max_speed+1e-6 ||
      derivativeControlBound(acceleration)>max_acceleration+1e-6) return {};
  const double join_limit=std::min(resolution*.25,.0125);
  Eigen::Vector3d previous=curve.evaluateDeBoorT(0.);
  if (!previous.allFinite() || (previous-solve_body.position).norm()>join_limit ||
      (current_body.position-solve_body.position).norm()>travel_speed*elapsed+join_limit)
    return {};
  const double maximum_arc=std::min(.5,travel_speed*elapsed+join_limit);
  const double dt=std::min(.02,resolution*.125/std::max(speed_bound,.01));
  double nearest_time=0.,nearest_square=(previous-current_body.position).squaredNorm(),arc=0.;
  std::size_t samples=0;
  for (double time=dt;time<=duration;time=std::min(duration,time+dt)) {
    if (++samples>10000 || !budget->allowed()) return {};
    const Eigen::Vector3d point=curve.evaluateDeBoorT(time);
    if (!point.allFinite()) return {};
    arc+=(point-previous).norm();
    if (arc>maximum_arc) break;
    const double square=(point-current_body.position).squaredNorm();
    if (square<nearest_square) {nearest_square=square;nearest_time=time;}
    previous=point;
    if (time>=duration) break;
  }
  if (nearest_square>join_limit*join_limit || (!single_segment && nearest_time>1e-9) ||
      (derivative.evaluateDeBoorT(nearest_time)-velocity).norm()>.05) return {};
  // Integrate in the same <=20 ms partition used by the consumer, from the
  // ORIGINAL curve's zero. Parameter metadata never invents a new route origin.
  const int intervals=std::max(1,static_cast<int>(std::ceil(duration/.02)));
  if (intervals>6000) return {};
  arc=0.; previous=curve.evaluateDeBoorT(0.);
  for (int i=1;i<=intervals;++i) {
    if (!budget->allowed()) return {};
    const double time=std::min(nearest_time,duration*double(i)/intervals);
    const Eigen::Vector3d point=curve.evaluateDeBoorT(time);
    arc+=(point-previous).norm(); previous=point;
    if (time>=nearest_time) break;
  }
  return CandidateJoinEvidence{nearest_time,arc,current_body,velocity,
                               Eigen::Vector3d::Zero(),false};
}

struct CandidateSourceLease {
  std::int64_t map_source_ns{0};
  std::uint64_t context{0},revision{0};
  double body_source{0.};
  std::int64_t body_source_ns{0};
};

// Called both before and immediately after the full real-map collision check.
// Source freshness is supplied by GridMap's own age policy, not a new lease.
inline bool candidateLeaseStillCurrent(const CandidateSourceLease &before,
    const CandidateSourceLease &after, bool map_fresh, bool body_fresh,
    const SolveBudget::Ptr &budget) {
  const auto before_body=before.body_source_ns>0?before.body_source_ns:poseSecondsNs(before.body_source);
  const auto after_body=after.body_source_ns>0?after.body_source_ns:poseSecondsNs(after.body_source);
  return budget && budget->allowed() && map_fresh && body_fresh &&
      before.map_source_ns>0 && before_body>0 &&
      before.map_source_ns==after.map_source_ns && before.context==after.context &&
      before.revision==after.revision && before_body==after_body;
}

// The production adoption transaction. Injection is only for its clock/source
// reads and the existing real collision checker, never a replacement planner.
// No caller may commit an optional join before BOTH source gates have passed.
template<class ReadLease,class SourcesFresh,class CheckCollision>
std::optional<CandidateJoinEvidence> certifyCandidateAdoption(
    UniformBspline &curve,const MeasuredBodyPose &solve_body,
    const MeasuredBodyPose &current_body,const Eigen::Vector3d &velocity,
    double resolution,double max_speed,double max_acceleration,bool single_segment,
    std::uint64_t candidate_context,const SolveBudget::Ptr &budget,
    ReadLease read_lease,SourcesFresh sources_fresh,CheckCollision check_collision,
    double measured_travel_max_speed=0.) {
  const auto before=read_lease();
  const auto gate=[&]() {
    const auto fresh=sources_fresh();
    return candidateLeaseStillCurrent(before,read_lease(),fresh.first,fresh.second,budget);
  };
  if (candidate_context!=before.context || !gate()) return {};
  const auto join=measuredCandidateJoin(curve,solve_body,current_body,velocity,
      resolution,max_speed,max_acceleration,single_segment,budget,measured_travel_max_speed);
  if (!join || !check_collision(join->curve_time) || !gate()) return {};
  return join;
}

}  // namespace scan_planner
