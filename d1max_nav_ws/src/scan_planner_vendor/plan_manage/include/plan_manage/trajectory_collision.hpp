#pragma once
#include <bspline_opt/forward_entry_heading.hpp>
#include <bspline_opt/rolling_curve_connection.hpp>

#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/whole_spline_collision.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <limits>

namespace scan_planner {

enum class CurveCheckEvidence { Clear, Occupied, Uncertified };
// A failed proof is not necessarily a witnessed collision. Keep resource
// exhaustion typed all the way to the incumbent renewal decision.
enum class CurveCheckCompletion { InvalidInput, Clear, Blocked, Uncertified, BudgetExhausted };
inline const char* curveCheckCompletionName(CurveCheckCompletion completion) {
  switch(completion) {
    case CurveCheckCompletion::InvalidInput:return "invalid_input";
    case CurveCheckCompletion::Clear:return "clear";
    case CurveCheckCompletion::Blocked:return "blocked";
    case CurveCheckCompletion::Uncertified:return "uncertified";
    case CurveCheckCompletion::BudgetExhausted:return "budget_exhausted";
  }
  return "invalid_input";
}

// Distinguish a witnessed occupied query from absence of a complete proof.
// The latter includes budgets, disconnected measured pose and invalid sources;
// both deny validity, but only an actual witness says the curve is occupied.
inline CurveCheckEvidence curveCheckEvidence(bool complete_and_clear, bool occupied_witness) {
  return complete_and_clear ? CurveCheckEvidence::Clear :
      (occupied_witness ? CurveCheckEvidence::Occupied : CurveCheckEvidence::Uncertified);
}

// A measured body pose, not a velocity/tangent estimate. Source time and frame
// travel with the quaternion so receipt time cannot renew stale orientation.
struct MeasuredBodyPose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double source_stamp{0.};
  std::string frame;
};

inline bool measuredPoseSourceAccepted(std::int64_t stamp_ns, std::int64_t now_ns,
    double maximum_age, std::int64_t previous_stamp_ns=0) {
  if (stamp_ns<=0 || stamp_ns<=previous_stamp_ns || !std::isfinite(maximum_age) || maximum_age<=0.)
    return false;
  const long double age=(static_cast<long double>(now_ns)-stamp_ns)*1e-9L;
  return age>=-.1L && age<=maximum_age;
}

inline bool measuredBodyYaw(const MeasuredBodyPose &pose, const std::string &frame,
    double now, double maximum_age, double &yaw) {
  if (frame.empty() || pose.frame!=frame || !pose.position.allFinite() ||
      !pose.orientation.coeffs().allFinite() || !std::isfinite(pose.orientation.norm()) ||
      pose.orientation.norm()<1e-8 ||
      !std::isfinite(pose.source_stamp) || pose.source_stamp<=0. ||
      !std::isfinite(now) || !std::isfinite(maximum_age) || maximum_age<=0. ||
      now-pose.source_stamp<-.1 || now-pose.source_stamp>maximum_age) return false;
  const Eigen::Vector3d heading=pose.orientation.normalized().toRotationMatrix().col(0);
  // No invented yaw for a body X axis perpendicular to the horizontal plane.
  if (heading.head<2>().squaredNorm()<1e-8) return false;
  yaw=std::atan2(heading.y(),heading.x());
  return std::isfinite(yaw);
}

// Actual XYZ projection, not elapsed spline time. The previous certified
// projection is a lower bound; measured displacement bounds the searched arc
// (at most 1 m). This prevents jumping to a later crossing or another floor.
// Projection alone is NOT validity: the caller must still certify the actual
// body connector and the complete curve against the current native map.
inline std::optional<double> measuredPreviewCurveTime(UniformBspline &curve,
    const Eigen::Vector3d &body, double previous_time, double resolution,
    std::size_t sample_budget=10000, double wall_budget_seconds=.005,
    CurveCheckCompletion* completion=nullptr) {
  if(completion)*completion=CurveCheckCompletion::InvalidInput;
  const auto began=std::chrono::steady_clock::now();
  const auto points=curve.getControlPoint();
  if (!body.allFinite() || points.rows()!=3 || points.cols()<4 || !points.allFinite() ||
      !std::isfinite(previous_time) || previous_time<0. ||
      !std::isfinite(resolution) || resolution<=0. || sample_budget<2 ||
      !std::isfinite(wall_budget_seconds) || wall_budget_seconds<0.) return std::nullopt;
  if (curve.getOrder()!=3 || !std::isfinite(curve.getInterval()) || curve.getInterval()<=0.)
    return std::nullopt;
  const auto knots=curve.getKnot();
  if (knots.size()!=points.cols()+4 || !knots.allFinite()) return std::nullopt;
  for (int i=1;i<knots.size();++i) if (knots[i]<=knots[i-1]) return std::nullopt;
  const double duration=curve.getTimeSum();
  if (!std::isfinite(duration) || duration<=0. || previous_time>duration) return std::nullopt;
  auto velocity=curve.getDerivative();
  const auto controls=velocity.getControlPoint();
  if (!controls.allFinite()) return std::nullopt;
  double speed_bound=0.;
  for (int i=0;i<controls.cols();++i) speed_bound=std::max(speed_bound,controls.col(i).norm());
  if (!std::isfinite(speed_bound)) return std::nullopt;
  if(completion)*completion=CurveCheckCompletion::Uncertified;
  const double dt=std::min(.02,resolution*.125/std::max(speed_bound,.01));
  Eigen::Vector3d previous=curve.evaluateDeBoorT(previous_time);
  if (!previous.allFinite()) return std::nullopt;
  const double maximum_arc=std::min(1.,(body-previous).norm()+2.*resolution);
  double nearest_time=previous_time, nearest_square=(body-previous).squaredNorm(), arc=0.;
  std::size_t samples=0;
  for (double time=previous_time; ; time=std::min(duration,time+dt)) {
    if (++samples>sample_budget || (wall_budget_seconds>0. &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>wall_budget_seconds)) {
      if(completion)*completion=CurveCheckCompletion::BudgetExhausted;
      return std::nullopt;
    }
    const Eigen::Vector3d point=curve.evaluateDeBoorT(time);
    if (!point.allFinite()) return std::nullopt;
    arc+=(point-previous).norm();
    if (arc>maximum_arc) break;
    const double square=(point-body).squaredNorm();
    if (square<nearest_square) { nearest_square=square; nearest_time=time; }
    previous=point;
    if (time>=duration) break;
  }
  if (nearest_square>std::pow(std::min(resolution*.25,.0125),2)) return std::nullopt;
  if(completion)*completion=CurveCheckCompletion::Clear;
  return nearest_time;
}

// This type is deliberately not controller TrackingProgress. Connected is
// only an acquired-pose projection; its source history is independent of the
// collision certificate. A current whole-curve/connector/support check must
// still certify it before it can renew a no-motion preview. Replan requests change
// geometry from acquired odometry, never authorize driving the old connector.
enum class NoMotionPreviewStatus { Uncertified, Connected, ReplanFromMeasuredBody, BudgetExhausted };
struct NoMotionPreviewConnectionPolicy {
  // Geometry-only engineering bounds, not localization confidence or motion
  // acceptance. Motion candidate admission retains its separate strict contract.
  static constexpr double max_connector_m=.15,max_retreat_m=.15;
  static constexpr double max_forward_arc_m=1.,max_source_gap_s=.4;
};
struct NoMotionPreviewProgress {
  NoMotionPreviewStatus status{NoMotionPreviewStatus::Uncertified};
  double curve_time{0.},arc_length{0.},high_water_arc{0.},residual{0.},source_stamp{0.};
  Eigen::Vector3d measured_position{Eigen::Vector3d::Zero()};
  bool connected() const {return status==NoMotionPreviewStatus::Connected;}
};
inline bool sameNoMotionPreviewSeed(const NoMotionPreviewProgress& a,const NoMotionPreviewProgress& b) {
  return a.connected()&&b.connected()&&a.curve_time==b.curve_time&&a.arc_length==b.arc_length&&
    a.high_water_arc==b.high_water_arc&&a.source_stamp==b.source_stamp&&
    a.measured_position==b.measured_position;
}
// A body observation is not a collision proof. Keep the independently acquired
// history monotonic, including its retreat high-water mark, while leaving the
// certified seed/proof untouched. A repeated, changed-at-same-time or late
// projection cannot extend the observation history either.
inline bool advanceNoMotionPreviewObservation(const NoMotionPreviewProgress& prior,
    const NoMotionPreviewProgress& next) {
  return prior.connected()&&next.connected()&&std::isfinite(next.source_stamp)&&
    next.source_stamp>prior.source_stamp+1e-9&&
    next.source_stamp-prior.source_stamp<=NoMotionPreviewConnectionPolicy::max_source_gap_s+1e-9&&
    std::isfinite(next.curve_time)&&next.curve_time>=0.&&
    std::isfinite(next.arc_length)&&next.arc_length>=0.&&std::isfinite(next.high_water_arc)&&
    next.high_water_arc+1e-6>=prior.high_water_arc&&next.high_water_arc+1e-6>=next.arc_length&&
    next.high_water_arc-next.arc_length<=NoMotionPreviewConnectionPolicy::max_retreat_m+1e-6&&
    next.measured_position.allFinite()&&std::isfinite(next.residual)&&
    next.residual<=NoMotionPreviewConnectionPolicy::max_connector_m+1e-9;
}

inline NoMotionPreviewProgress projectCommittedNoMotionPreview(UniformBspline& curve,
    const Eigen::Vector3d& body,double source_stamp,const NoMotionPreviewProgress& seed,
    double resolution,std::size_t sample_budget=10000,double wall_budget_seconds=.005,
    std::chrono::steady_clock::time_point deadline=std::chrono::steady_clock::time_point::max()) {
  NoMotionPreviewProgress result;
  const auto began=std::chrono::steady_clock::now();
  const auto controls=curve.getControlPoint();const auto knots=curve.getKnot();
  const double duration=curve.getTimeSum();
  if(!seed.connected()||!body.allFinite()||!seed.measured_position.allFinite()||
     !std::isfinite(source_stamp)||source_stamp<=0.||!std::isfinite(seed.source_stamp)||seed.source_stamp<=0.||
     source_stamp<seed.source_stamp-1e-9||!std::isfinite(seed.curve_time)||seed.curve_time<0.||
     !std::isfinite(seed.arc_length)||seed.arc_length<0.||!std::isfinite(seed.high_water_arc)||
     seed.high_water_arc<seed.arc_length-1e-6||
     seed.high_water_arc-seed.arc_length>NoMotionPreviewConnectionPolicy::max_retreat_m+1e-6||
     controls.rows()!=3||controls.cols()<4||controls.cols()>10000||!controls.allFinite()||
     curve.getOrder()!=3||knots.size()!=controls.cols()+4||!knots.allFinite()||
     !std::isfinite(duration)||duration<=0.||duration>120.||seed.curve_time>duration||
     !std::isfinite(resolution)||resolution<=0.||sample_budget<2||
     !std::isfinite(wall_budget_seconds)||wall_budget_seconds<0.)return result;
  for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return result;
  const double displacement=(body-seed.measured_position).norm();
  // A changed value at the same acquired stamp is not a new pose sample.
  if(source_stamp<=seed.source_stamp+1e-9&&displacement>1e-8)return result;
  result=seed;result.status=NoMotionPreviewStatus::ReplanFromMeasuredBody;
  result.measured_position=body;result.source_stamp=source_stamp;
  if(source_stamp-seed.source_stamp>NoMotionPreviewConnectionPolicy::max_source_gap_s+1e-9||
     displacement>NoMotionPreviewConnectionPolicy::max_forward_arc_m)return result;
  auto velocity=curve.getDerivative();const auto v=velocity.getControlPoint();
  if(!v.allFinite())return {};
  double speed=0.;for(int i=0;i<v.cols();++i)speed=std::max(speed,v.col(i).norm());
  if(!std::isfinite(speed))return {};
  const double dt=std::min(.02,resolution*.125/std::max(speed,.01));
  const double forward=std::min(NoMotionPreviewConnectionPolicy::max_forward_arc_m,displacement+2.*resolution);
  const double reverse_boundary=std::max(0.,seed.high_water_arc-NoMotionPreviewConnectionPolicy::max_retreat_m);
  const double backward=std::max(0.,std::min(forward,seed.arc_length-reverse_boundary));
  const Eigen::Vector3d anchor=curve.evaluateDeBoorT(seed.curve_time);
  if(!anchor.allFinite())return {};
  double best_square=(body-anchor).squaredNorm(),best_time=seed.curve_time,best_arc=seed.arc_length;
  std::size_t samples=1;
  const auto expired=[&] {return samples>sample_budget||std::chrono::steady_clock::now()>=deadline||
    (wall_budget_seconds>0.&&
    std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>wall_budget_seconds);};
  const auto budget_miss=[] {NoMotionPreviewProgress out;out.status=NoMotionPreviewStatus::BudgetExhausted;return out;};
  if(expired())return budget_miss();
  // Search only the connected local arc on either side of the certified seed.
  // Reverse search can never ratchet beyond 15 cm behind its high-water mark.
  for(int direction:{1,-1}) {
    const double extent=direction>0?forward:backward;if(extent<=0.)continue;
    double previous_time=seed.curve_time,arc=0.;Eigen::Vector3d previous=anchor;
    while(direction>0?previous_time<duration:previous_time>0.) {
      if(expired())return budget_miss();
      const double time=std::clamp(previous_time+direction*dt,0.,duration);
      if(time==previous_time)break;
      const Eigen::Vector3d point=curve.evaluateDeBoorT(time);++samples;
      if(!point.allFinite())return {};
      if(expired())return budget_miss();
      const Eigen::Vector3d delta=point-previous;const double length=delta.norm();
      if(length>1e-12) {
        const double upper=std::clamp((extent-arc)/length,0.,1.);
        const double fraction=std::clamp((body-previous).dot(delta)/delta.squaredNorm(),0.,upper);
        const double projected_time=previous_time+fraction*(time-previous_time);
        const double square=(body-curve.evaluateDeBoorT(projected_time)).squaredNorm();++samples;
        if(expired())return budget_miss();
        if(!std::isfinite(square))return {};
        if(square<best_square-1e-16) {
          best_square=square;best_time=projected_time;
          best_arc=std::max(0.,seed.arc_length+direction*(arc+fraction*length));
        }
      }
      arc+=length;if(arc>=extent)break;
      previous=point;previous_time=time;
    }
  }
  result.residual=std::sqrt(best_square);
  if(result.residual>NoMotionPreviewConnectionPolicy::max_connector_m+1e-9)return result;
  // The connector is not another reverse allowance. At the high-water reverse
  // boundary a body still farther behind along the local tangent must reseed;
  // a bounded lateral offset can still connect after the actual swept check.
  if(best_arc<=reverse_boundary+1e-6) {
    const Eigen::Vector3d tangent=velocity.evaluateDeBoorT(best_time);
    const Eigen::Vector3d point=curve.evaluateDeBoorT(best_time);
    if(!tangent.allFinite()||!point.allFinite())return {};
    if(expired())return budget_miss();
    const double tangent_length=tangent.norm();
    // Near the beginning, an actual connector behind arc zero may use only
    // the unused part of the same retreat allowance, never another full one.
    const double unused_retreat=std::max(0.,NoMotionPreviewConnectionPolicy::max_retreat_m-seed.high_water_arc);
    if(tangent_length>1e-8&&(body-point).dot(tangent)/tangent_length < -unused_retreat-1e-6)return result;
  }
  result.status=NoMotionPreviewStatus::Connected;result.curve_time=best_time;
  result.arc_length=best_arc;result.high_water_arc=std::max(seed.high_water_arc,best_arc);
  return result;
}

struct MeasuredCurveDomain {
  double measured_time{0.},measured_arc{0.},checked_from_time{0.},duration{0.};
};
enum class MeasuredConnectionPolicy {
  CandidateAdmission, CommittedSweptConnection, CommittedNoMotionPreview, PreviewCandidateGeometry,
  RollingCandidateConnection
};
// An execution-only suffix proof is tied to the controller's same-curve
// measured parameter, never the spline clock or global-route high-water mark.
// The range includes a full 15 cm actual-arc reverse margin and all remaining
// curve geometry. Projection is XYZ and bounded by measured source-time travel.
inline std::optional<MeasuredCurveDomain> measuredRemainingCurveDomain(UniformBspline& curve,
    const Eigen::Vector3d& body,double previous_measured_time,double source_dt,
    double committed_arc,double max_speed=.3,double reverse_margin=.15,
    MeasuredConnectionPolicy connection=MeasuredConnectionPolicy::CandidateAdmission,
    double* measured_residual=nullptr) {
  if(measured_residual)*measured_residual=std::numeric_limits<double>::quiet_NaN();
  const auto controls=curve.getControlPoint();const auto knots=curve.getKnot();
  const double duration=curve.getTimeSum();
  const double retreat_limit=connection==MeasuredConnectionPolicy::CommittedNoMotionPreview?
    NoMotionPreviewConnectionPolicy::max_retreat_m:.15;
  if(!body.allFinite()||controls.rows()!=3||controls.cols()<4||!controls.allFinite()||
     curve.getOrder()!=3||knots.size()!=controls.cols()+4||!knots.allFinite()||
     !std::isfinite(duration)||duration<=0.||duration>120.||
     !std::isfinite(previous_measured_time)||previous_measured_time<0.||previous_measured_time>duration||
     !std::isfinite(source_dt)||source_dt<-.02||source_dt>.4||
     !std::isfinite(committed_arc)||committed_arc<0.||max_speed<=0.||max_speed>.3||
     !std::isfinite(reverse_margin)||reverse_margin<retreat_limit||reverse_margin>retreat_limit+1e-9)return {};
  for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return {};
  const int n=std::max(1,static_cast<int>(std::ceil(duration/.02)));
  std::vector<double> times(n+1),arcs(n+1);std::vector<Eigen::Vector3d> points(n+1);
  for(int i=0;i<=n;++i) {
    times[i]=duration*double(i)/n;points[i]=curve.evaluateDeBoorT(times[i]);
    if(!points[i].allFinite())return {};
    if(i)arcs[i]=arcs[i-1]+(points[i]-points[i-1]).norm();
  }
  const auto arcAt=[&](double t) {
    const auto it=std::lower_bound(times.begin(),times.end(),t);
    if(it==times.begin())return 0.;
    if(it==times.end())return arcs.back();
    const auto hi=static_cast<std::size_t>(it-times.begin()),lo=hi-1;
    return arcs[lo]+(t-times[lo])/(times[hi]-times[lo])*(arcs[hi]-arcs[lo]);
  };
  const double seed_arc=arcAt(previous_measured_time);
  if(committed_arc>arcs.back()+.01||seed_arc<committed_arc-retreat_limit-.01)return {};
  const double travel=max_speed*std::max(0.,source_dt)+.0125;
  const double low=std::max({0.,seed_arc-std::min(retreat_limit,travel),committed_arc-retreat_limit});
  const double high=std::min(arcs.back(),seed_arc+travel);
  double best_square=std::numeric_limits<double>::infinity(),best_time=0.,best_arc=0.;
  for(int i=0;i<n;++i) {
    const double length=arcs[i+1]-arcs[i];if(length<1e-12||arcs[i+1]<low||arcs[i]>high)continue;
    const auto delta=points[i+1]-points[i];
    const double f=std::clamp((body-points[i]).dot(delta)/delta.squaredNorm(),
      std::clamp((low-arcs[i])/length,0.,1.),std::clamp((high-arcs[i])/length,0.,1.));
    const double time=times[i]+f*(times[i+1]-times[i]);
    const double square=(body-curve.evaluateDeBoorT(time)).squaredNorm();
    if(square<best_square){best_square=square;best_time=time;best_arc=arcs[i]+f*length;}
  }
  if(measured_residual)*measured_residual=std::sqrt(best_square);
  // A committed controller may slow down or have a small tracking error.
  // This only bounds its search domain. It is NOT a collision clearance:
  // the caller must verify every actual body-to-curve footprint and support.
  const double connection_limit=connection==MeasuredConnectionPolicy::CommittedNoMotionPreview?
    NoMotionPreviewConnectionPolicy::max_connector_m:
    (connection==MeasuredConnectionPolicy::CommittedSweptConnection||
     connection==MeasuredConnectionPolicy::RollingCandidateConnection)?
      RollingCurveConnectionPolicy::max_connector_m:.0125;
  if(best_square>connection_limit*connection_limit)return {};
  const double from_arc=std::max(0.,best_arc-reverse_margin);
  const auto it=std::upper_bound(arcs.begin(),arcs.end(),from_arc);
  const auto index=it==arcs.begin()?0:static_cast<std::size_t>(it-arcs.begin()-1);
  return MeasuredCurveDomain{best_time,best_arc,times[index],duration};
}

// Complete curve and yaw-sweep validation shared by primary acceptance and
// optional predecessor evidence. A budget exhaustion is never a safe result.
// A future tail has a planned entry, distinct from an acquired body pose. Only
// the explicit common-prefix checker supplies it, after checking the actual
// body's connection to the incumbent and its safe prefix to this boundary.
struct PlannedCurveEntry {
  Eigen::Vector3d position;
  double yaw;
};
template<class Query>
bool wholeCurveCollisionFree(UniformBspline &curve, double resolution, double body_extent,
    Query occupied, const MeasuredBodyPose &measured, const std::string &frame,
    double now, double maximum_pose_age, std::size_t query_budget=200000,
    double wall_budget_seconds=0., double measured_curve_time=0.,
    const SplineHeadingContract &preview_heading={},double checked_from_time=0.,
    MeasuredConnectionPolicy connection=MeasuredConnectionPolicy::CandidateAdmission,
    double preview_candidate_connector_m=NoMotionPreviewConnectionPolicy::max_connector_m,
    CurveCheckCompletion* completion=nullptr,
    double checked_to_time=std::numeric_limits<double>::infinity(),
    const std::optional<PlannedCurveEntry>& planned_entry={},
    bool reuse_exact_immutable_queries=false) {
  CurveCheckCompletion completed=CurveCheckCompletion::InvalidInput;
  const auto record=[&](CurveCheckCompletion value) {completed=value;if(completion)*completion=value;return false;};
  record(completed);
  const auto started=std::chrono::steady_clock::now();
  const auto expired=[&]() {
    const bool over=wall_budget_seconds>0. &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>wall_budget_seconds;
    if(over&&completed!=CurveCheckCompletion::Blocked)record(CurveCheckCompletion::BudgetExhausted);
    return over;
  };
  if (!std::isfinite(resolution) || resolution<=0. || !std::isfinite(body_extent) || body_extent<0. ||
      !std::isfinite(wall_budget_seconds) || wall_budget_seconds<0. || !query_budget) return false;
  if(preview_heading.policy!=BodyHeadingPolicy::PathTangent&&
     !validMeasuredHeadingContract(preview_heading))return false;
  if(connection==MeasuredConnectionPolicy::PreviewCandidateGeometry&&
     (!std::isfinite(preview_candidate_connector_m)||preview_candidate_connector_m<=0.||
      preview_candidate_connector_m>NoMotionPreviewConnectionPolicy::max_connector_m))return false;
  const double duration=curve.getTimeSum();
  const double checked_end=std::isinf(checked_to_time)&&checked_to_time>0.?duration:checked_to_time;
  double measured_yaw=0.;
  if (!measuredBodyYaw(measured,frame,now,maximum_pose_age,measured_yaw) ||
      !std::isfinite(measured_curve_time) || measured_curve_time<0. ||
      measured_curve_time>duration||!std::isfinite(checked_from_time)||checked_from_time<0.||
      checked_from_time>measured_curve_time||!std::isfinite(checked_end)||
      checked_end<measured_curve_time||checked_end>duration) return false;
  if(planned_entry&&(!planned_entry->position.allFinite()||!std::isfinite(planned_entry->yaw)||
      measured_curve_time!=0.||checked_from_time!=0.))return false;
  auto velocity=curve.getDerivative();
  const auto controls=velocity.getControlPoint();
  if (!std::isfinite(duration) || duration<=0. || !controls.allFinite()) return false;
  record(CurveCheckCompletion::Uncertified);
  double speed_bound=0.;
  for (int i=0;i<controls.cols();++i) speed_bound=std::max(speed_bound,controls.col(i).norm());
  if(!std::isfinite(speed_bound))return false;
  const double dt=std::min(.02,resolution*.25/std::max(speed_bound,.01));
  const double checked_duration=checked_end-checked_from_time;
  if (checked_duration/dt>50000.) return record(CurveCheckCompletion::BudgetExhausted);
  const int count=std::max(1,static_cast<int>(std::ceil(checked_duration/dt)));
  if (static_cast<std::size_t>(count)+1>query_budget) return record(CurveCheckCompletion::BudgetExhausted);
  const double yaw_step=std::min(.05,resolution*.25/std::max(body_extent,.01));
  Eigen::Vector3d previous_position;
  double previous_yaw=0.;
  std::size_t queries=0;
  // Adjacent sweeps include both endpoints, and a zero-angle sweep includes
  // the same pose twice. Only an explicitly immutable geometry query may
  // reuse an EXACT consecutive clear pose within its fixed body/support scope.
  // Generic callbacks can check cancellation or expiring observed-free cells,
  // so default to asking again. Never quantize, retain across checks, or skip
  // the original logical query/wall budgets; nearby poses need full queries.
  std::optional<std::pair<Eigen::Vector3d,double>> last_clear_query;
  const auto query=[&](const Eigen::Vector3d &p,double yaw) {
    if(expired())return false;
    if(++queries>query_budget)return record(CurveCheckCompletion::BudgetExhausted);
    if(reuse_exact_immutable_queries&&last_clear_query&&
       last_clear_query->first==p&&last_clear_query->second==yaw)return true;
    const int state=occupied(p,yaw);
    if(state!=0)return record(state==1?CurveCheckCompletion::Blocked:CurveCheckCompletion::Uncertified);
    if(reuse_exact_immutable_queries)last_clear_query=std::make_pair(p,yaw);
    return true;
  };
  const auto tangentAt=[&](double t) -> Eigen::Vector3d {
    Eigen::Vector3d tangent=velocity.evaluateDeBoorT(t);
    if (tangent.head<2>().norm()<1e-8)
      tangent=curve.evaluateDeBoorT(std::min(duration,t+dt))-
              curve.evaluateDeBoorT(std::max(0.,t-dt));
    return tangent;
  };
  const Eigen::Vector3d join=curve.evaluateDeBoorT(measured_curve_time);
  const Eigen::Vector3d join_tangent=tangentAt(measured_curve_time);
  const Eigen::Vector3d connection_position=planned_entry?planned_entry->position:measured.position;
  const double connection_yaw=planned_entry?planned_entry->yaw:measured_yaw;
  // A remote measured pose cannot certify the curve's start. For optional old
  // curve continuity, callers pass the current curve time; a tracking error
  // beyond the normal sample distance denies that optional proof.
  const double connection_limit=connection==MeasuredConnectionPolicy::PreviewCandidateGeometry?
    preview_candidate_connector_m:connection==MeasuredConnectionPolicy::CommittedNoMotionPreview?
    NoMotionPreviewConnectionPolicy::max_connector_m:
    (connection==MeasuredConnectionPolicy::CommittedSweptConnection||
     connection==MeasuredConnectionPolicy::RollingCandidateConnection)?
      RollingCurveConnectionPolicy::max_connector_m:std::min(resolution*.25,.0125);
  if (!join.allFinite() || !join_tangent.allFinite() ||
      (join-connection_position).norm()>(planned_entry?1e-6:connection_limit)) return false;
  if (measuredHeadingEnabled(preview_heading)) {
    // Keep the accepted curve's original heading contract; current body yaw
    // must be connected to its measured geometric progress, not retroactively
    // applied to the curve's historical beginning.
    auto contract=preview_heading;
    contract.body_extent=body_extent;
    if (!validMeasuredHeadingContract(contract)) return false;
    // Replay the same FULL sampling lattice as the optimizer to preserve the
    // accepted heading memory. Only the relevant remaining spatial domain is
    // queried: history establishes orientation, not fresh collision evidence.
    const auto sampling=bodyHeadingSamplingIntervals(duration,speed_bound,resolution);
    if(!sampling)return record(CurveCheckCompletion::BudgetExhausted);
    if(!resolveForwardEntryHeading(contract,duration,*sampling,
         [&](double t){return curve.evaluateDeBoorT(t);},
         [&](double t){return velocity.evaluateDeBoorT(t);},[&]{return !expired();}))return false;
    const int intervals=static_cast<int>(*sampling);
    // Low initial derivative is not a trustworthy body direction. Certify
    // align-then-forward's actual in-place turn into the SAME bounded near
    // immutable curve direction used by control; HoldHeading never turns.
    double initial_heading=contract.measured_yaw;
    if(contract.align_then_forward&&contract.policy==BodyHeadingPolicy::PathTangent&&
       velocity.evaluateDeBoorT(0.).head<2>().norm()<contract.minimum_direction_speed) {
      const auto direction=nearCurveDirection(
        [&](double t){const auto p=curve.evaluateDeBoorT(t);return CurveDirectionPoint{p.x(),p.y(),p.z()};},
        [&](double t){const auto v=velocity.evaluateDeBoorT(t);return CurveDirectionPoint{v.x(),v.y(),v.z()};},
        0.,duration,[&]{return !expired();});
      if(direction)initial_heading=std::atan2(direction->direction[1],direction->direction[0]);
    }
    const auto headingAt=[&](double time) {
      auto replay=contract;replay.measured_yaw=initial_heading;
      return sampledBodyHeadingAt(time,duration,*sampling,replay,
        [&](double at){return velocity.evaluateDeBoorT(at);},[&]{return !expired();})
        .value_or(std::numeric_limits<double>::quiet_NaN());
    };
    double join_yaw=headingAt(measured_curve_time);
    if(contract.align_then_forward&&contract.policy==BodyHeadingPolicy::PathTangent&&measured_curve_time>0.&&
       measured_curve_time>=contract.forward_entry_end&&
       velocity.evaluateDeBoorT(measured_curve_time).head<2>().norm()<contract.minimum_direction_speed) {
      const auto direction=nearCurveDirection(
        [&](double t){const auto p=curve.evaluateDeBoorT(t);return CurveDirectionPoint{p.x(),p.y(),p.z()};},
        [&](double t){const auto v=velocity.evaluateDeBoorT(t);return CurveDirectionPoint{v.x(),v.y(),v.z()};},
        measured_curve_time,duration,[&]{return !expired();});
      if(direction)join_yaw=std::atan2(direction->direction[1],direction->direction[0]);
    }
    // Only the ACTUAL forward-only body may retain a rear turn branch. A
    // future splice's planned entry and curve-internal transitions keep their
    // frozen shortest-heading geometry; they are not fresh in-place turns.
    const bool entry_free=!planned_entry&&contract.align_then_forward?
      bodyEntryHeadingTransitionFree(connection_position,join,connection_yaw,join_yaw,
        resolution,body_extent,query):
      headingTransitionFree(connection_position,join,connection_yaw,join_yaw,
        resolution,body_extent,query);
    if(!entry_free)return false;
    Eigen::Vector3d previous=curve.evaluateDeBoorT(checked_from_time);
    double yaw=headingAt(checked_from_time);
    if (!previous.allFinite() || !std::isfinite(yaw) || !query(previous,yaw)) return false;
    for (int i=0;i<=intervals;++i) {
      const double t=std::min(checked_end,duration*static_cast<double>(i)/intervals);
      if (t<=checked_from_time) continue;
      if (expired()) return false;
      const Eigen::Vector3d position=curve.evaluateDeBoorT(t);
      const Eigen::Vector3d tangent=velocity.evaluateDeBoorT(t);
      if (!position.allFinite() || !tangent.allFinite()) return false;
      const double next=bodyHeadingAtCurveTime(tangent,yaw,contract,t);
      if (!headingTransitionFree(previous,position,yaw,next,resolution,body_extent,query)) return false;
      previous=position;yaw=next;
      if(t>=checked_end)break;
    }
    if(expired())return false;
    record(CurveCheckCompletion::Clear);return true;
  }
  const double join_yaw=join_tangent.head<2>().norm()<1e-8 ? connection_yaw :
      std::atan2(join_tangent.y(),join_tangent.x());
  const double initial_turn=std::atan2(std::sin(join_yaw-connection_yaw),
                                      std::cos(join_yaw-connection_yaw));
  const double translation_step=resolution*.25;
  const int translations=std::max(0,static_cast<int>(std::ceil(
      (join-connection_position).norm()/translation_step)));
  const int initial_turns=std::max(1,static_cast<int>(std::ceil(std::abs(initial_turn)/yaw_step)));
  // Check the actual initial footprint AND intermediate headings, including
  // the short connection to the spline. Endpoint-only tests miss the corner of
  // a double cylinder during an in-place turn. These share the same budgets.
  for (int j=0;j<=translations;++j) {
    const Eigen::Vector3d p=connection_position+(double(j)/std::max(1,translations))*(join-connection_position);
    for (int k=0;k<=initial_turns;++k)
      if (!query(p,connection_yaw+(double(k)/initial_turns)*initial_turn)) return false;
  }
  for (int i=0;i<=count;++i) {
    const double t=checked_from_time+checked_duration*static_cast<double>(i)/count;
    const Eigen::Vector3d p=curve.evaluateDeBoorT(t);
    Eigen::Vector3d tangent=tangentAt(t);
    if (!p.allFinite() || !tangent.allFinite()) return false;
    const double yaw=tangent.head<2>().norm()<1e-8 ?
        (i==0 ? connection_yaw : previous_yaw) : std::atan2(tangent.y(),tangent.x());
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
  if(expired())return false;
  record(CurveCheckCompletion::Clear);return true;
}

}  // namespace scan_planner
