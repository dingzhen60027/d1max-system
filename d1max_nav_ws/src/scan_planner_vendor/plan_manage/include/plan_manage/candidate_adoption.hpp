#pragma once

#include <plan_manage/trajectory_collision.hpp>
#include <bspline_opt/trajectory_timing.hpp>
#include <plan_env/solve_budget.hpp>
#include <plan_manage/forward_motion_boundary.hpp>
#include <array>

namespace scan_planner {

struct CandidateJoinEvidence {
  double curve_time{0.};
  double arc_length{0.};
  MeasuredBodyPose measured;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool acceleration_valid{false};
};

// A planning preview is not a motion hand-off. Its acquired solve boundary and
// actual geometric connector travel in a different type, never in join_twist
// or a controller admission certificate. Defaults share the committed preview
// connector bound; the projection window is an ARC bound, not a position error.
struct PreviewCandidateAcceptancePolicy {
  double connector_m{NoMotionPreviewConnectionPolicy::max_connector_m};
  double projection_window_m{.5};
  bool valid() const {
    return std::isfinite(connector_m)&&connector_m>0.&&
        connector_m<=NoMotionPreviewConnectionPolicy::max_connector_m&&
        std::isfinite(projection_window_m)&&projection_window_m>0.&&
        projection_window_m<=NoMotionPreviewConnectionPolicy::max_forward_arc_m;
  }
};
struct PreviewCandidateOrigin {
  MeasuredBodyPose solve_body;
  Eigen::Vector3d solve_velocity{Eigen::Vector3d::Zero()};
  double maximum_speed{0.},maximum_acceleration{0.};
  bool single_segment{false};
  PreviewCandidateAcceptancePolicy policy;
};
struct PreviewCandidateGeometryEvidence {
  PreviewCandidateOrigin origin;
  NoMotionPreviewProgress projection;
};
inline bool samePreviewCandidateOrigin(const PreviewCandidateOrigin& a,const PreviewCandidateOrigin& b) {
  return a.solve_body.source_stamp==b.solve_body.source_stamp&&a.solve_body.frame==b.solve_body.frame&&
      a.solve_body.position==b.solve_body.position&&a.solve_body.orientation.coeffs()==b.solve_body.orientation.coeffs()&&
      a.solve_velocity==b.solve_velocity&&a.maximum_speed==b.maximum_speed&&
      a.maximum_acceleration==b.maximum_acceleration&&a.single_segment==b.single_segment&&
      a.policy.connector_m==b.policy.connector_m&&a.policy.projection_window_m==b.policy.projection_window_m;
}
enum class CandidateAcceptanceOutcome {
  NotAttempted, MotionAccepted, MotionRejected, PreviewGeometryAccepted,
  InvalidPreviewCandidate, PreviewSourceExpired, PreviewSourceChanged,
  PreviewBudgetExhausted, PreviewNeedsMeasuredReseed, PreviewSegmentBoundary,
  PreviewGeometryBlocked, Count
};
inline const char* candidateAcceptanceReason(CandidateAcceptanceOutcome outcome) {
  switch(outcome) {
    case CandidateAcceptanceOutcome::MotionAccepted:return "motion_candidate_accepted";
    case CandidateAcceptanceOutcome::MotionRejected:return "motion_candidate_rejected";
    case CandidateAcceptanceOutcome::PreviewGeometryAccepted:return "preview_candidate_geometry_accepted";
    case CandidateAcceptanceOutcome::InvalidPreviewCandidate:return "preview_candidate_invalid";
    case CandidateAcceptanceOutcome::PreviewSourceExpired:return "preview_candidate_source_expired";
    case CandidateAcceptanceOutcome::PreviewSourceChanged:return "preview_candidate_source_changed";
    case CandidateAcceptanceOutcome::PreviewBudgetExhausted:return "preview_candidate_budget_exhausted";
    case CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed:return "preview_candidate_requires_measured_reseed";
    case CandidateAcceptanceOutcome::PreviewSegmentBoundary:return "preview_candidate_segment_boundary";
    case CandidateAcceptanceOutcome::PreviewGeometryBlocked:return "preview_candidate_geometry_blocked";
    default:return "candidate_not_attempted";
  }
}
struct CandidateAcceptanceDiagnostics {
  CandidateAcceptanceOutcome last{CandidateAcceptanceOutcome::NotAttempted};
  std::array<std::uint64_t,static_cast<std::size_t>(CandidateAcceptanceOutcome::Count)> counts{};
  void record(CandidateAcceptanceOutcome outcome) {
    last=outcome;++counts.at(static_cast<std::size_t>(outcome));
  }
};
struct PreviewCandidateAcceptanceResult {
  CandidateAcceptanceOutcome outcome{CandidateAcceptanceOutcome::InvalidPreviewCandidate};
  std::optional<PreviewCandidateGeometryEvidence> geometry;
};
enum class PreviewCandidateProjectionPhase { SolveAdoption, CandidateRevalidation };

// First connected local XYZ arc of the ORIGINAL candidate. Acquired source time
// bounds the solve-to-adoption interval, but it never predicts a position.
// Independent candidate revalidation uses the newest acquired body's OWN age
// lease, not the immutable solve origin's age (it remains geometric provenance).
// Velocity changes during computation do not invalidate a no-motion drawing.
// Projection alone is not acceptance: the transaction below still checks the
// ACTUAL body-to-curve swept envelope and the WHOLE original curve/support.
template<class IsAllowed>
PreviewCandidateAcceptanceResult projectPreviewCandidateGeometryWhileAllowed(
    UniformBspline& curve,const PreviewCandidateOrigin& origin,
    const MeasuredBodyPose& current_body,double resolution,
    IsAllowed allowed,
    PreviewCandidateProjectionPhase phase=PreviewCandidateProjectionPhase::SolveAdoption) {
  using O=CandidateAcceptanceOutcome;
  const auto fail=[](O outcome){return PreviewCandidateAcceptanceResult{outcome,{}};};
  if(!allowed())return fail(O::PreviewBudgetExhausted);
  const auto& solve_body=origin.solve_body;
  if(!origin.policy.valid()||!solve_body.position.allFinite()||!current_body.position.allFinite()||
     !solve_body.orientation.coeffs().allFinite()||solve_body.orientation.norm()<1e-8||
     !current_body.orientation.coeffs().allFinite()||current_body.orientation.norm()<1e-8||
     solve_body.frame.empty()||solve_body.frame!=current_body.frame||!origin.solve_velocity.allFinite()||
     !std::isfinite(resolution)||resolution<=0.||!std::isfinite(origin.maximum_speed)||origin.maximum_speed<=0.||
     !std::isfinite(origin.maximum_acceleration)||origin.maximum_acceleration<=0.||
     !std::isfinite(solve_body.source_stamp)||solve_body.source_stamp<=0.||
     !std::isfinite(current_body.source_stamp)||current_body.source_stamp<=0.)return fail(O::InvalidPreviewCandidate);
  const double elapsed=current_body.source_stamp-solve_body.source_stamp;
  if(elapsed<0.)return fail(O::PreviewSourceChanged);
  if(phase==PreviewCandidateProjectionPhase::SolveAdoption&&
     elapsed>NoMotionPreviewConnectionPolicy::max_source_gap_s)return fail(O::PreviewNeedsMeasuredReseed);
  const double displacement=(current_body.position-solve_body.position).norm();
  if(elapsed==0.&&(displacement>1e-8||
      solve_body.orientation.normalized().angularDistance(current_body.orientation.normalized())>1e-8))
    return fail(O::PreviewSourceChanged); // no different pose under the same acquisition stamp
  if(displacement>origin.policy.projection_window_m+origin.policy.connector_m)
    return fail(O::PreviewNeedsMeasuredReseed);
  const auto controls=curve.getControlPoint();const auto knots=curve.getKnot();
  const double duration=curve.getTimeSum();
  if(curve.getOrder()!=3||controls.rows()!=3||controls.cols()<4||controls.cols()>10000||
     !controls.allFinite()||knots.size()!=controls.cols()+4||!knots.allFinite()||
     !std::isfinite(curve.getInterval())||curve.getInterval()<=0.||
     !std::isfinite(duration)||duration<=0.||duration>120.)return fail(O::InvalidPreviewCandidate);
  for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return fail(O::InvalidPreviewCandidate);
  auto velocity=curve.getDerivative();auto acceleration=velocity.getDerivative();
  if(!velocity.getControlPoint().allFinite()||!acceleration.getControlPoint().allFinite())
    return fail(O::InvalidPreviewCandidate);
  const double speed=derivativeControlBound(velocity);
  if(!inspectInitialMotionLimits(curve,origin.maximum_speed,origin.maximum_acceleration).valid)
    return fail(O::InvalidPreviewCandidate);
  Eigen::Vector3d previous=curve.evaluateDeBoorT(0.);
  // Numerical agreement with the ORIGINAL acquired solve boundary is still
  // required. This is not a fit to the newer body's changed velocity/position.
  if(!previous.allFinite()||(previous-solve_body.position).norm()>std::min(resolution*.25,.0125)||
     (velocity.evaluateDeBoorT(0.)-origin.solve_velocity).norm()>1e-3)return fail(O::InvalidPreviewCandidate);
  const double maximum_arc=std::min(origin.policy.projection_window_m,displacement+2.*resolution);
  const double dt=std::min(.02,resolution*.125/std::max(speed,.01));
  double nearest_time=0.,nearest_arc=0.,nearest_square=(previous-current_body.position).squaredNorm(),arc=0.;
  std::size_t samples=0;
  for(double previous_time=0.;previous_time<duration;) {
    if(++samples>10000||!allowed())return fail(O::PreviewBudgetExhausted);
    const double time=std::min(duration,previous_time+dt);
    if(time==previous_time)return fail(O::InvalidPreviewCandidate);
    const Eigen::Vector3d point=curve.evaluateDeBoorT(time);
    if(!point.allFinite())return fail(O::InvalidPreviewCandidate);
    const Eigen::Vector3d delta=point-previous;const double length=delta.norm();
    if(length>1e-12) {
      const double upper=std::clamp((maximum_arc-arc)/length,0.,1.);
      const double fraction=std::clamp((current_body.position-previous).dot(delta)/delta.squaredNorm(),0.,upper);
      const double projected_time=previous_time+fraction*(time-previous_time);
      const double square=(curve.evaluateDeBoorT(projected_time)-current_body.position).squaredNorm();
      if(!std::isfinite(square))return fail(O::InvalidPreviewCandidate);
      if(square<nearest_square-1e-16) {
        nearest_square=square;nearest_time=projected_time;nearest_arc=arc+fraction*length;
      }
    }
    arc+=length;if(arc>=maximum_arc)break;
    previous=point;previous_time=time;
  }
  if(!allowed())return fail(O::PreviewBudgetExhausted);
  if(nearest_square>origin.policy.connector_m*origin.policy.connector_m)return fail(O::PreviewNeedsMeasuredReseed);
  if(!origin.single_segment&&nearest_time>1e-9)return fail(O::PreviewSegmentBoundary);
  NoMotionPreviewProgress projection{NoMotionPreviewStatus::Connected,nearest_time,nearest_arc,nearest_arc,
      std::sqrt(nearest_square),current_body.source_stamp,current_body.position};
  return {O::PreviewGeometryAccepted,PreviewCandidateGeometryEvidence{origin,projection}};
}
inline PreviewCandidateAcceptanceResult projectPreviewCandidateGeometry(
    UniformBspline& curve,const PreviewCandidateOrigin& origin,const MeasuredBodyPose& current_body,
    double resolution,const SolveBudget::Ptr& budget) {
  return projectPreviewCandidateGeometryWhileAllowed(curve,origin,current_body,resolution,
      [&]{return budget&&budget->allowed();});
}

// The original worker curve is immutable. Only its certified entry parameter
// changes; neither elapsed publication time nor a remote nearest point is an
// execution position. Formal rolling following checks a real geometric
// connector instead of requiring the acquired state to equal the spline P/V.
// Legacy exact-state handoff retains its original contract.
inline std::optional<CandidateJoinEvidence> measuredCandidateJoin(
    UniformBspline &curve, const MeasuredBodyPose &solve_body,
    const MeasuredBodyPose &current_body, const Eigen::Vector3d &velocity,
    double resolution, double max_speed, double max_acceleration,
    bool single_segment, const SolveBudget::Ptr &budget,
    MeasuredConnectionPolicy connection=MeasuredConnectionPolicy::CandidateAdmission) {
  if (!budget || !budget->allowed() || !velocity.allFinite() ||
      !solve_body.position.allFinite() || !current_body.position.allFinite() ||
      solve_body.frame!=current_body.frame || current_body.frame.empty() ||
      !std::isfinite(resolution) || resolution<=0. || !std::isfinite(max_speed) ||
      max_speed<=0. || !std::isfinite(max_acceleration) || max_acceleration<=0.) return {};
  const double elapsed=current_body.source_stamp-solve_body.source_stamp;
  if (!std::isfinite(elapsed) || solve_body.source_stamp<=0. || elapsed<0. || elapsed>.4)
    return {};
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
  const auto dynamics=inspectInitialMotionLimits(curve,max_speed,max_acceleration);
  if (!dynamics.valid) return {};
  const double travel_speed=dynamics.recovering?dynamics.peak_speed:max_speed;
  const bool rolling=connection==MeasuredConnectionPolicy::RollingCandidateConnection;
  const double origin_limit=std::min(resolution*.25,RollingCurveConnectionPolicy::numerical_origin_tolerance_m);
  const double join_limit=rolling?RollingCurveConnectionPolicy::max_connector_m:origin_limit;
  Eigen::Vector3d previous=curve.evaluateDeBoorT(0.);
  if (!previous.allFinite() || (previous-solve_body.position).norm()>origin_limit ||
      (current_body.position-solve_body.position).norm()>travel_speed*elapsed+join_limit)
    return {};
  const double maximum_arc=std::min(RollingCurveConnectionPolicy::max_forward_arc_m,
      travel_speed*elapsed+join_limit);
  const double dt=std::min(.02,resolution*.125/std::max(speed_bound,.01));
  double nearest_time=0.,nearest_square=(previous-current_body.position).squaredNorm(),arc=0.;
  std::size_t samples=0;
  for (double previous_time=0.;previous_time<duration;) {
    if (++samples>10000 || !budget->allowed()) return {};
    const double time=std::min(duration,previous_time+dt);
    const Eigen::Vector3d point=curve.evaluateDeBoorT(time);
    if (!point.allFinite()) return {};
    const Eigen::Vector3d delta=point-previous;
    const double length=delta.norm();
    if(length>1e-12) {
      const double upper=std::clamp((maximum_arc-arc)/length,0.,1.);
      const double fraction=std::clamp((current_body.position-previous).dot(delta)/delta.squaredNorm(),0.,upper);
      const double projected_time=previous_time+fraction*(time-previous_time);
      const double square=(curve.evaluateDeBoorT(projected_time)-current_body.position).squaredNorm();
      if(square<nearest_square){nearest_square=square;nearest_time=projected_time;}
    }
    arc+=length;if(arc>=maximum_arc)break;
    previous=point;previous_time=time;
  }
  if (nearest_square>join_limit*join_limit || (!single_segment && nearest_time>1e-9) ||
      (!rolling&&(derivative.evaluateDeBoorT(nearest_time)-velocity).norm()>
        measured_candidate_join_velocity_tolerance_mps)) return {};
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
};
enum class MotionCandidateAdoptionOutcome {
  NotAttempted, Accepted, SourceExpired, SourceChanged, ContextChanged,
  MeasuredJoinRejected, CollisionRejected, BudgetExhausted
};
inline const char* motionCandidateAdoptionReason(MotionCandidateAdoptionOutcome outcome) {
  switch(outcome) {
    case MotionCandidateAdoptionOutcome::Accepted:return "motion_candidate_accepted";
    case MotionCandidateAdoptionOutcome::SourceExpired:return "candidate_source_expired";
    case MotionCandidateAdoptionOutcome::SourceChanged:return "candidate_source_changed";
    case MotionCandidateAdoptionOutcome::ContextChanged:return "candidate_context_changed";
    case MotionCandidateAdoptionOutcome::MeasuredJoinRejected:return "candidate_measured_join_rejected";
    case MotionCandidateAdoptionOutcome::CollisionRejected:return "candidate_latest_geometry_rejected";
    case MotionCandidateAdoptionOutcome::BudgetExhausted:return "candidate_adoption_budget_exhausted";
    default:return "candidate_adoption_not_attempted";
  }
}

// Called both before and immediately after the full real-map collision check.
// Source freshness is supplied by GridMap's own age policy, not a new lease.
inline bool candidateLeaseStillCurrent(const CandidateSourceLease &before,
    const CandidateSourceLease &after, bool map_fresh, bool body_fresh,
    const SolveBudget::Ptr &budget) {
  return budget && budget->allowed() && map_fresh && body_fresh &&
      before.map_source_ns>0 && before.body_source>0. &&
      before.map_source_ns==after.map_source_ns && before.context==after.context &&
      before.revision==after.revision && before.body_source==after.body_source;
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
    MotionCandidateAdoptionOutcome* outcome=nullptr,
    MeasuredConnectionPolicy connection=MeasuredConnectionPolicy::CandidateAdmission) {
  using O=MotionCandidateAdoptionOutcome;
  const auto result=[&](O reason){if(outcome)*outcome=reason;return reason;};
  result(O::NotAttempted);
  const auto before=read_lease();
  const auto gate=[&]() {
    if(!budget||!budget->allowed())return result(O::BudgetExhausted);
    const auto after=read_lease();
    const auto fresh=sources_fresh();
    if(candidate_context!=before.context||before.context!=after.context)return result(O::ContextChanged);
    if(!fresh.first||!fresh.second||before.map_source_ns<=0||before.body_source<=0.)return result(O::SourceExpired);
    return result(candidateLeaseStillCurrent(before,after,fresh.first,fresh.second,budget)?
      O::Accepted:O::SourceChanged);
  };
  if (gate()!=O::Accepted) return {};
  const auto join=measuredCandidateJoin(curve,solve_body,current_body,velocity,
      resolution,max_speed,max_acceleration,single_segment,budget,connection);
  if(!join){result(budget->allowed()?O::MeasuredJoinRejected:O::BudgetExhausted);return {};}
  const bool geometry_clear=check_collision(join->curve_time);
  if(gate()!=O::Accepted)return {};
  if(!geometry_clear){result(O::CollisionRejected);return {};}
  return join;
}

// A distinct transaction/result type makes geometric acceptance unavailable to
// the motion admission API. Same map/source identity and absolute SolveBudget
// fences as Motion; the callback must certify actual connector + whole curve.
template<class ReadLease,class SourcesFresh,class CheckGeometry>
PreviewCandidateAcceptanceResult certifyPreviewCandidateGeometry(
    UniformBspline& curve,const PreviewCandidateOrigin& origin,
    const MeasuredBodyPose& current_body,double resolution,std::uint64_t candidate_context,
    const SolveBudget::Ptr& budget,ReadLease read_lease,SourcesFresh sources_fresh,
    CheckGeometry check_geometry) {
  using O=CandidateAcceptanceOutcome;
  const auto before=read_lease();
  const MeasuredBodyPose acquired_body=current_body;
  const auto gate=[&]() {
    if(!budget||!budget->allowed())return O::PreviewBudgetExhausted;
    const auto after=read_lease();
    if(candidate_context!=before.context||before.map_source_ns<=0||before.body_source<=0.||
       before.map_source_ns!=after.map_source_ns||before.context!=after.context||
       before.revision!=after.revision||before.body_source!=after.body_source)
      return O::PreviewSourceChanged;
    if(acquired_body.source_stamp!=current_body.source_stamp||acquired_body.frame!=current_body.frame||
       acquired_body.position!=current_body.position||
       acquired_body.orientation.coeffs()!=current_body.orientation.coeffs())return O::PreviewSourceChanged;
    const auto fresh=sources_fresh();
    return fresh.first&&fresh.second?O::PreviewGeometryAccepted:O::PreviewSourceExpired;
  };
  if(const auto outcome=gate();outcome!=O::PreviewGeometryAccepted)return {outcome,{}};
  auto projected=projectPreviewCandidateGeometry(curve,origin,acquired_body,resolution,budget);
  if(!projected.geometry)return projected;
  const bool clear=check_geometry(*projected.geometry);
  // Do not turn expired/replaced sources or an exhausted budget into collision.
  if(const auto outcome=gate();outcome!=O::PreviewGeometryAccepted)return {outcome,{}};
  if(!clear)return {O::PreviewGeometryBlocked,{}};
  return projected;
}

}  // namespace scan_planner
