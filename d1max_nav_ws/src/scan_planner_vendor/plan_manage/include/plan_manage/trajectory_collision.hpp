#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/whole_spline_collision.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <limits>

namespace scan_planner {

enum class CurveCheckEvidence { Clear, Occupied, Uncertified };
enum class MeasuredSpeedProfile { D1FirstAcceptance, IsolatedOfficialSpot };

struct CurveQueryWitness {
  Eigen::Vector3d position;
  double yaw;
  int state;
  std::size_t query_index;
};
// Passive evidence from the original query, with no second collision lookup.
// A budget/source/geometry rejection without a query leaves this empty.
struct CurveCheckTrace {
  std::optional<CurveQueryWitness> first_non_free;
};

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
  std::int64_t source_stamp_ns{0}; // Original ROS source; seconds are only control arithmetic.
};

inline std::int64_t poseSecondsNs(double seconds) {
  const long double ns=static_cast<long double>(seconds)*1000000000.L;
  if(!std::isfinite(seconds)||ns<=0.||ns>=std::numeric_limits<std::int64_t>::max())return 0;
  return static_cast<std::int64_t>(std::llround(ns));
}
inline std::int64_t measuredBodySourceNs(const MeasuredBodyPose& pose) {
  return pose.source_stamp_ns>0?pose.source_stamp_ns:poseSecondsNs(pose.source_stamp);
}

inline bool measuredPoseSourceAccepted(std::int64_t stamp_ns, std::int64_t now_ns,
    double maximum_age, std::int64_t previous_stamp_ns=0) {
  if (stamp_ns<=0 || now_ns<=0 || stamp_ns<=previous_stamp_ns || !std::isfinite(maximum_age) || maximum_age<=0.)
    return false;
  return stamp_ns-now_ns<=100000000LL && now_ns-stamp_ns<=poseSecondsNs(maximum_age);
}

inline bool measuredBodyYaw(const MeasuredBodyPose &pose, const std::string &frame,
    double now, double maximum_age, double &yaw,std::int64_t now_ns=0) {
  if (frame.empty() || pose.frame!=frame || !pose.position.allFinite() ||
      !pose.orientation.coeffs().allFinite() || !std::isfinite(pose.orientation.norm()) ||
      pose.orientation.norm()<1e-8 ||
      !std::isfinite(pose.source_stamp) || pose.source_stamp<=0. ||
      !std::isfinite(now) || !std::isfinite(maximum_age) || maximum_age<=0. ||
      !measuredPoseSourceAccepted(measuredBodySourceNs(pose),now_ns>0?now_ns:poseSecondsNs(now),maximum_age)) return false;
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
    std::size_t sample_budget=10000, double wall_budget_seconds=.005) {
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
  const double dt=std::min(.02,resolution*.125/std::max(speed_bound,.01));
  Eigen::Vector3d previous=curve.evaluateDeBoorT(previous_time);
  if (!previous.allFinite()) return std::nullopt;
  const double maximum_arc=std::min(1.,(body-previous).norm()+2.*resolution);
  double nearest_time=previous_time, nearest_square=(body-previous).squaredNorm(), arc=0.;
  std::size_t samples=0;
  for (double time=previous_time; ; time=std::min(duration,time+dt)) {
    if (++samples>sample_budget || (wall_budget_seconds>0. &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>wall_budget_seconds))
      return std::nullopt;
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
  return nearest_time;
}

struct MeasuredCurveDomain {
  double measured_time{0.},measured_arc{0.},checked_from_time{0.},duration{0.};
};
enum class MeasuredConnectionPolicy { CandidateAdmission, CommittedSweptConnection };
// An execution-only suffix proof is tied to the controller's same-curve
// measured parameter, never the spline clock or global-route high-water mark.
// The range includes a full 15 cm actual-arc reverse margin and all remaining
// curve geometry. Projection is XYZ and bounded by measured source-time travel.
inline std::optional<MeasuredCurveDomain> measuredRemainingCurveDomain(UniformBspline& curve,
    const Eigen::Vector3d& body,double previous_measured_time,double source_dt,
    double committed_arc,double max_speed=.3,double reverse_margin=.15,
    MeasuredConnectionPolicy connection=MeasuredConnectionPolicy::CandidateAdmission,
    double* measured_residual=nullptr,
    MeasuredSpeedProfile speed_profile=MeasuredSpeedProfile::D1FirstAcceptance) {
  if(measured_residual)*measured_residual=std::numeric_limits<double>::quiet_NaN();
  const auto controls=curve.getControlPoint();const auto knots=curve.getKnot();
  const double duration=curve.getTimeSum();
  if(!body.allFinite()||controls.rows()!=3||controls.cols()<4||!controls.allFinite()||
     curve.getOrder()!=3||knots.size()!=controls.cols()+4||!knots.allFinite()||
     !std::isfinite(duration)||duration<=0.||duration>120.||
     !std::isfinite(previous_measured_time)||previous_measured_time<0.||previous_measured_time>duration||
     !std::isfinite(source_dt)||source_dt<-.02||source_dt>.4||
     !std::isfinite(committed_arc)||committed_arc<0.||!std::isfinite(max_speed)||max_speed<=0.||
     max_speed>(speed_profile==MeasuredSpeedProfile::IsolatedOfficialSpot?.65:.3)||
     !std::isfinite(reverse_margin)||reverse_margin<.15||reverse_margin>.15+1e-9)return {};
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
  if(committed_arc>arcs.back()+.01||seed_arc<committed_arc-.15-.01)return {};
  const double travel=max_speed*std::max(0.,source_dt)+.0125;
  const double low=std::max({0.,seed_arc-std::min(.15,travel),committed_arc-.15});
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
  const double connection_limit=connection==MeasuredConnectionPolicy::CommittedSweptConnection?.15:.0125;
  if(best_square>connection_limit*connection_limit)return {};
  const double from_arc=std::max(0.,best_arc-reverse_margin);
  const auto it=std::upper_bound(arcs.begin(),arcs.end(),from_arc);
  const auto index=it==arcs.begin()?0:static_cast<std::size_t>(it-arcs.begin()-1);
  return MeasuredCurveDomain{best_time,best_arc,times[index],duration};
}

// Complete curve and yaw-sweep validation shared by primary acceptance and
// optional predecessor evidence. A budget exhaustion is never a safe result.
template<class Query>
bool wholeCurveCollisionFree(UniformBspline &curve, double resolution, double body_extent,
    Query occupied, const MeasuredBodyPose &measured, const std::string &frame,
    double now, double maximum_pose_age, std::size_t query_budget=200000,
    double wall_budget_seconds=0., double measured_curve_time=0.,
    const SplineHeadingContract &preview_heading={},double checked_from_time=0.,
    MeasuredConnectionPolicy connection=MeasuredConnectionPolicy::CandidateAdmission,std::int64_t now_ns=0,
    CurveCheckTrace* trace=nullptr) {
  if(trace)trace->first_non_free.reset();
  const auto started=std::chrono::steady_clock::now();
  const auto expired=[&]() {
    return wall_budget_seconds>0. &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>wall_budget_seconds;
  };
  if (!std::isfinite(resolution) || resolution<=0. || !std::isfinite(body_extent) || body_extent<0. ||
      !std::isfinite(wall_budget_seconds) || wall_budget_seconds<0. || !query_budget) return false;
  const double duration=curve.getTimeSum();
  double measured_yaw=0.;
  if (!measuredBodyYaw(measured,frame,now,maximum_pose_age,measured_yaw,now_ns) ||
      !std::isfinite(measured_curve_time) || measured_curve_time<0. ||
      measured_curve_time>duration||!std::isfinite(checked_from_time)||checked_from_time<0.||
      checked_from_time>measured_curve_time||(preview_heading.preview_only_enabled&&checked_from_time>0.)) return false;
  auto velocity=curve.getDerivative();
  const auto controls=velocity.getControlPoint();
  if (!std::isfinite(duration) || duration<=0. || !controls.allFinite()) return false;
  double speed_bound=0.;
  for (int i=0;i<controls.cols();++i) speed_bound=std::max(speed_bound,controls.col(i).norm());
  const double dt=std::min(.02,resolution*.25/std::max(speed_bound,.01));
  const double checked_duration=duration-checked_from_time;
  if (checked_duration/dt>50000.) return false;
  const int count=std::max(1,static_cast<int>(std::ceil(checked_duration/dt)));
  if (static_cast<std::size_t>(count)+1>query_budget) return false;
  const double yaw_step=std::min(.05,resolution*.25/std::max(body_extent,.01));
  Eigen::Vector3d previous_position;
  double previous_yaw=0.;
  std::size_t queries=0;
  const auto query=[&](const Eigen::Vector3d &p,double yaw) {
    if(expired() || ++queries>query_budget)return false;
    const int state=occupied(p,yaw);
    if(trace && state!=0 && !trace->first_non_free)
      trace->first_non_free=CurveQueryWitness{p,yaw,state,queries};
    return state==0;
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
  // A remote measured pose cannot certify the curve's start. For optional old
  // curve continuity, callers pass the current curve time; a tracking error
  // beyond the normal sample distance denies that optional proof.
  const double connection_limit=connection==MeasuredConnectionPolicy::CommittedSweptConnection?
      .15:std::min(resolution*.25,.0125);
  if (!join.allFinite() || !join_tangent.allFinite() ||
      (join-measured.position).norm()>connection_limit) return false;
  if (preview_heading.preview_only_enabled) {
    // Keep the accepted curve's original heading contract; current body yaw
    // must be connected to its measured geometric progress, not retroactively
    // applied to the curve's historical beginning.
    auto contract=preview_heading;
    contract.body_extent=body_extent;
    if (!validPreviewHeadingContract(contract)) return false;
    double join_yaw=contract.measured_yaw;
    // Same sampling and low-speed heading memory as checkWholeSplineCollision.
    for (int i=0;i<=count;++i) {
      const double t=duration*static_cast<double>(i)/count;
      if (t>measured_curve_time) break;
      if (expired()) return false;
      join_yaw=previewBodyHeading(velocity.evaluateDeBoorT(t),join_yaw,contract);
    }
    join_yaw=previewBodyHeading(velocity.evaluateDeBoorT(measured_curve_time),join_yaw,contract);
    if (!headingTransitionFree(measured.position,join,measured_yaw,join_yaw,
                                resolution,body_extent,query)) return false;
    if (queries>=query_budget) return false;
    const auto result=checkWholeSplineCollision(curve,resolution,
        [&](const Eigen::Vector3d &p,double yaw) {return query(p,yaw)?0:1;},
        query_budget-queries,contract);
    return result.state==SplineCollisionState::Clear && !expired();
  }
  const double join_yaw=join_tangent.head<2>().norm()<1e-8 ? measured_yaw :
      std::atan2(join_tangent.y(),join_tangent.x());
  const double initial_turn=std::atan2(std::sin(join_yaw-measured_yaw),
                                      std::cos(join_yaw-measured_yaw));
  const double translation_step=resolution*.25;
  const int translations=std::max(0,static_cast<int>(std::ceil(
      (join-measured.position).norm()/translation_step)));
  const int initial_turns=std::max(1,static_cast<int>(std::ceil(std::abs(initial_turn)/yaw_step)));
  // Check the actual initial footprint AND intermediate headings, including
  // the short connection to the spline. Endpoint-only tests miss the corner of
  // a double cylinder during an in-place turn. These share the same budgets.
  for (int j=0;j<=translations;++j) {
    const Eigen::Vector3d p=measured.position+(double(j)/std::max(1,translations))*(join-measured.position);
    for (int k=0;k<=initial_turns;++k)
      if (!query(p,measured_yaw+(double(k)/initial_turns)*initial_turn)) return false;
  }
  for (int i=0;i<=count;++i) {
    const double t=checked_from_time+checked_duration*static_cast<double>(i)/count;
    const Eigen::Vector3d p=curve.evaluateDeBoorT(t);
    Eigen::Vector3d tangent=tangentAt(t);
    if (!p.allFinite() || !tangent.allFinite()) return false;
    const double yaw=tangent.head<2>().norm()<1e-8 ?
        (i==0 ? measured_yaw : previous_yaw) : std::atan2(tangent.y(),tangent.x());
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
