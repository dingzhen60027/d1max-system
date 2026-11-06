#pragma once
#include <plan_manage/trajectory_collision.hpp>
#include <d1max_planning_interfaces/msg/trajectory_splice.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>

namespace scan_planner {
// Value-copied immutable incumbent. The acquired solve pose remains separate;
// these derivatives describe a PLANNED boundary, never an IMU/body sample.
struct PlannedTrajectorySplice {
  d1max_planning_interfaces::msg::TrajectorySplice wire;
  UniformBspline incumbent;
  SplineHeadingContract heading;
  double measured_time{0.};
  double entry_yaw{0.};
  std::int64_t progress_source_ns{0};
  double committed_arc{0.};
};
// Passive failure attribution. No field supplies a body, renews a timestamp or
// changes the existing geometric/source admission result.
struct SpliceCheckDiagnostics {
  const char* phase{"not_checked"};
  std::int64_t progress_source_ns{0};
  double body_source_seconds{0.},source_delta_ms{0.};
};
inline std::optional<UniformBspline> decodeSpliceCurve(
    const scan_planner_msgs::msg::Bspline& s) {
  if(s.order!=3||s.pos_pts.size()<4||s.pos_pts.size()>10000||s.knots.size()!=s.pos_pts.size()+4)return {};
  Eigen::MatrixXd points(3,s.pos_pts.size());Eigen::VectorXd knots(s.knots.size());
  for(std::size_t i=0;i<s.pos_pts.size();++i)points.col(i)=Eigen::Vector3d(s.pos_pts[i].x,s.pos_pts[i].y,s.pos_pts[i].z);
  for(std::size_t i=0;i<s.knots.size();++i)knots[i]=s.knots[i];
  if(!points.allFinite()||!knots.allFinite())return {};
  for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return {};
  UniformBspline result(points,3,.1);result.setKnot(knots);
  if(!std::isfinite(result.getTimeSum())||result.getTimeSum()<=0.)return {};
  return result;
}
inline double curveArcTo(UniformBspline& c,double time) {
  const double end=std::min(c.getTimeSum(),time);
  if(!std::isfinite(end)||end<0.)return std::numeric_limits<double>::quiet_NaN();
  const int n=std::max(1,static_cast<int>(std::ceil(end/.01)));
  if(n>50000)return std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d previous=c.evaluateDeBoorT(0.);double arc=0.;
  for(int i=1;i<=n;++i){const Eigen::Vector3d p=c.evaluateDeBoorT(end*double(i)/n);arc+=(p-previous).norm();previous=p;}
  return arc;
}
inline double immutableHeadingAt(UniformBspline& c,const SplineHeadingContract& contract,
    double time,double resolution,SplineHeadingContract* resolved_out=nullptr) {
  auto velocity=c.getDerivative();const double duration=c.getTimeSum();
  double speed=0.;const auto controls=velocity.getControlPoint();
  for(int i=0;i<controls.cols();++i)speed=std::max(speed,controls.col(i).norm());
  const auto tangent=velocity.evaluateDeBoorT(time);
  if(!measuredHeadingEnabled(contract)) {
    if(resolved_out)*resolved_out=contract;
    return tangent.head<2>().norm()>1e-8?std::atan2(tangent.y(),tangent.x()):contract.measured_yaw;
  }
  const auto n=bodyHeadingSamplingIntervals(duration,speed,resolution);
  if(!n)return std::numeric_limits<double>::quiet_NaN();
  auto resolved=contract;
  if(!resolveForwardEntryHeading(resolved,duration,*n,
       [&](double at){return c.evaluateDeBoorT(at);},
       [&](double at){return velocity.evaluateDeBoorT(at);},[]{return true;}))
    return std::numeric_limits<double>::quiet_NaN();
  if(resolved_out)*resolved_out=resolved;
  return sampledBodyHeadingAt(time,duration,*n,resolved,
    [&](double at){return velocity.evaluateDeBoorT(at);},[]{return true;})
    .value_or(std::numeric_limits<double>::quiet_NaN());
}
inline bool spliceBoundaryMatches(UniformBspline& tail,PlannedTrajectorySplice& splice,
    double position_limit=1e-6,double velocity_limit=1e-5) {
  const auto& w=splice.wire;auto old_v=splice.incumbent.getDerivative();auto tail_v=tail.getDerivative();
  const Eigen::Vector3d p{w.position.x,w.position.y,w.position.z},v{w.velocity.x,w.velocity.y,w.velocity.z};
  if(!w.enabled||!p.allFinite()||!v.allFinite()||!std::isfinite(w.splice_curve_time)||
      w.splice_curve_time<0.||w.splice_curve_time>splice.incumbent.getTimeSum()||
      !std::isfinite(splice.entry_yaw)||
      (p-splice.incumbent.evaluateDeBoorT(w.splice_curve_time)).norm()>1e-6||
      (v-old_v.evaluateDeBoorT(w.splice_curve_time)).norm()>1e-6||
      (tail.evaluateDeBoorT(0.)-p).norm()>position_limit||
      (tail_v.evaluateDeBoorT(0.)-v).norm()>velocity_limit)return false;
  if(w.acceleration_valid) {
    const Eigen::Vector3d a{w.acceleration.x,w.acceleration.y,w.acceleration.z};
    auto old_a=old_v.getDerivative();auto tail_a=tail_v.getDerivative();
    if(!a.allFinite()||(a-old_a.evaluateDeBoorT(w.splice_curve_time)).norm()>1e-5||
        (tail_a.evaluateDeBoorT(0.)-a).norm()>1e-4)return false;
  }
  return true;
}
// A control tick may pass the splice along the SAME tail. That distance is
// not its lateral connection error. Search only the first existing 5 cm / .4 s
// handoff domain; preserve the ordinary 1.25 cm body-to-curve residual. This
// geometric search does not fabricate a sensor timestamp or renew a proof.
inline std::optional<MeasuredCurveDomain> measuredSpliceTailEntry(UniformBspline& tail,
    const Eigen::Vector3d& body,double resolution) {
  auto velocity=tail.getDerivative();const auto controls=velocity.getControlPoint();
  if(!body.allFinite()||!controls.allFinite()||!std::isfinite(resolution)||resolution<=0.)return {};
  double speed=0.;for(int i=0;i<controls.cols();++i)speed=std::max(speed,controls.col(i).norm());
  const double duration=tail.getTimeSum(),end=std::min(duration,.4);
  if(!std::isfinite(duration)||duration<=0.)return {};
  const double dt=std::min(.02,resolution*.125/std::max(speed,.01));
  const auto count=std::ceil(end/dt);if(!std::isfinite(count)||count>512.)return {};
  Eigen::Vector3d previous=tail.evaluateDeBoorT(0.);
  double previous_time=0.,arc=0.,best_time=0.,best_arc=0.,best_square=(body-previous).squaredNorm();
  for(int i=1;i<=static_cast<int>(count)&&arc<.05;++i) {
    const double time=std::min(end,i*dt);const Eigen::Vector3d point=tail.evaluateDeBoorT(time);
    if(!point.allFinite())return {};
    const Eigen::Vector3d delta=point-previous;const double length=delta.norm();
    if(length>1e-12) {
      const double upper=std::clamp((.05-arc)/length,0.,1.);
      const double fraction=std::clamp((body-previous).dot(delta)/delta.squaredNorm(),0.,upper);
      const double projected=previous_time+fraction*(time-previous_time);
      const double square=(body-tail.evaluateDeBoorT(projected)).squaredNorm();
      if(square<best_square){best_square=square;best_time=projected;best_arc=arc+fraction*length;}
    }
    arc+=length;previous=point;previous_time=time;
  }
  if(best_square>std::pow(std::min(.0125,resolution*.25),2))return {};
  // The connection tolerance is lateral, not another forward extension of
  // the already bounded handoff prefix.
  if(best_arc>=.05-1e-6||best_time>=end-1e-6) {
    const Eigen::Vector3d tangent=velocity.evaluateDeBoorT(best_time);
    if(tangent.norm()>1e-8&&(body-tail.evaluateDeBoorT(best_time)).dot(tangent)/tangent.norm()>1e-6)return {};
  }
  return MeasuredCurveDomain{best_time,best_arc,0.,duration};
}
// Exactly one native query round certifies the actual incumbent prefix and
// planned tail. No straight line from the current robot to a future point is
// invented; a remote tail alone cannot claim measured admission.
template<class Query>
bool commonPrefixTailCollisionFree(UniformBspline& tail,PlannedTrajectorySplice splice,
    double resolution,double extent,Query&& query,const MeasuredBodyPose& actual,
    const std::string& frame,double now,double age,double wall_budget,
    const SplineHeadingContract& tail_heading,CurveCheckCompletion* completion=nullptr,
    SpliceCheckDiagnostics* diagnostics=nullptr,MeasuredCurveDomain* actual_tail_domain=nullptr) {
  if(completion)*completion=CurveCheckCompletion::InvalidInput;
  const auto phase=[&](const char* value){if(diagnostics)diagnostics->phase=value;};
  if(diagnostics) {
    diagnostics->progress_source_ns=splice.progress_source_ns;
    diagnostics->body_source_seconds=actual.source_stamp;
    diagnostics->source_delta_ms=splice.progress_source_ns>0?
      (actual.source_stamp-splice.progress_source_ns*1e-9)*1000.:0.;
  }
  phase("boundary_mismatch");if(!spliceBoundaryMatches(tail,splice))return false;
  const auto begun=std::chrono::steady_clock::now();
  const auto remaining=[&]{return std::max(0.,wall_budget-std::chrono::duration<double>(
      std::chrono::steady_clock::now()-begun).count());};
  // Acquired progress can pass the exact boundary by only the ordinary join
  // distance. Then current tail entry must itself pass actual pose geometry.
  const auto boundary=splice.wire.splice_curve_time;
  const double source_dt=splice.progress_source_ns>0?
      actual.source_stamp-splice.progress_source_ns*1e-9:0.;
  if(source_dt<-.02||source_dt>.4) {
    phase(source_dt<-.02?"progress_after_body":"progress_source_gap");return false;
  }
  const auto actual_tail=[&](double projected_time) {
    phase("tail_boundary_overshoot");
    const double overshoot=curveArcTo(splice.incumbent,projected_time)-splice.wire.splice_arc_length;
    if(!std::isfinite(overshoot)||overshoot>.05)return false;
    phase("tail_entry_projection");
    const auto entry=measuredSpliceTailEntry(tail,actual.position,resolution);
    if(!entry)return false;
    phase("tail_entry_curve");
    const bool safe=wholeCurveCollisionFree(tail,resolution,extent,query,actual,frame,now,age,
      200000,remaining(),entry->measured_time,tail_heading,entry->checked_from_time,
      MeasuredConnectionPolicy::CandidateAdmission,NoMotionPreviewConnectionPolicy::max_connector_m,completion);
    if(safe){phase("clear");if(actual_tail_domain)*actual_tail_domain=*entry;}
    return safe;
  };
  if(splice.measured_time>boundary)return actual_tail(splice.measured_time);
  phase(source_dt<-.02?"progress_after_body":source_dt>.4?"progress_source_gap":"incumbent_projection");
  auto domain=measuredRemainingCurveDomain(splice.incumbent,actual.position,splice.measured_time,
      source_dt,splice.committed_arc,.3,.15,MeasuredConnectionPolicy::CommittedSweptConnection);
  // New body ingress may be ahead of the last progress callback. Projected
  // actual progress must obey the SAME bounded boundary crossing; clamping it
  // to the future boundary would hide a missed splice behind the old .15 m
  // incumbent connector allowance.
  if(domain&&domain->measured_time>boundary)return actual_tail(domain->measured_time);
  if(!domain)return false;
  phase("prefix_curve");
  if(remaining()<=0.){if(completion)*completion=CurveCheckCompletion::BudgetExhausted;return false;}
  if(!wholeCurveCollisionFree(splice.incumbent,resolution,extent,query,
      actual,frame,now,age,200000,remaining(),std::min(domain->measured_time,boundary),splice.heading,
      std::min(domain->checked_from_time,boundary),MeasuredConnectionPolicy::CommittedSweptConnection,
      NoMotionPreviewConnectionPolicy::max_connector_m,completion,boundary))return false;
  phase("tail_curve");
  if(remaining()<=0.){if(completion)*completion=CurveCheckCompletion::BudgetExhausted;return false;}
  const bool safe=wholeCurveCollisionFree(tail,resolution,extent,query,actual,frame,now,age,200000,
    remaining(),0.,tail_heading,0.,MeasuredConnectionPolicy::CandidateAdmission,
    NoMotionPreviewConnectionPolicy::max_connector_m,completion,std::numeric_limits<double>::infinity(),
    PlannedCurveEntry{{splice.wire.position.x,splice.wire.position.y,splice.wire.position.z},splice.entry_yaw});
  if(safe)phase("clear");
  return safe;
}
} // namespace scan_planner
