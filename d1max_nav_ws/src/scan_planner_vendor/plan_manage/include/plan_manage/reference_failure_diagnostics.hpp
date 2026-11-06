#pragma once

#include <plan_env/grid_map.h>
#include <plan_env/solve_budget.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <sstream>

namespace scan_planner {

// Called only for a witnessed native collision and under the existing failed
// proof log throttle. Both descriptions use the SAME exclusively-held snapshot;
// re-querying enriches the failure report, never changes that proof's verdict.
inline std::string describeCurveCollisionWitness(GridMap& map,const Eigen::Vector3d& point,
    double query_yaw,const MeasuredBodyPose& acquired) {
  std::ostringstream out;out.precision(9);
  out<<"query_xyz=["<<point.x()<<","<<point.y()<<","<<point.z()<<"] query_yaw="<<query_yaw
     <<" acquired_body_xyz=["<<acquired.position.x()<<","<<acquired.position.y()<<","<<acquired.position.z()<<"]"
     <<" query_to_acquired_body_m="<<(point-acquired.position).norm()
     <<" acquired_body_source_s="<<std::fixed<<acquired.source_stamp<<std::defaultfloat
     <<" query_evidence={"<<map.describeInflateOccupancy(point,query_yaw)<<"}";
  double yaw=0.;
  if(!measuredBodyYaw(acquired,map.collisionFrame(),acquired.source_stamp,.4,yaw))
    return out.str()+" acquired_body_evidence=unavailable_frame_or_pose";
  const auto rotation=acquired.orientation.normalized().toRotationMatrix();
  const double pitch=std::atan2(-rotation(2,0),std::hypot(rotation(0,0),rotation(1,0)));
  const double roll=std::atan2(rotation(2,1),rotation(2,2));
  out<<" acquired_body_rpy=["<<roll<<","<<pitch<<","<<yaw<<"] acquired_body_evidence={"
     <<map.describeInflateOccupancy(acquired.position,yaw,pitch,roll)<<"}";
  return out.str();
}

// A detour's start is not necessarily the robot. Retain the LAST native
// search endpoints, which own the reported failure, without changing search.
struct ReferenceSearchFailureDiagnostic {
  Eigen::Vector3d start,end,solve_start,solve_goal;
  double query_yaw{0.};
  int native_result{0};
  bool planned_splice{false};
};

inline const char *referenceSearchStartRole(const ReferenceSearchFailureDiagnostic &query) {
  return (query.start-query.solve_start).norm()<=1e-9?
      "solve_start":"intermediate_reference_anchor";
}

inline std::string describeReferenceSearchFailure(GridMap &map,
    const ReferenceSearchFailureDiagnostic &query,const MeasuredBodyPose &acquired,
    const std::string &body_frame,double now,double maximum_age,
    const SolveBudget::Ptr &budget) {
  std::ostringstream out;out.precision(9);
  out<<"role="<<referenceSearchStartRole(query)
     <<" native_result="<<query.native_result
     <<" endpoint_role="<<((query.end-query.solve_goal).norm()<=1e-9?"solve_goal":"intermediate_reference_anchor")
     <<" planned_splice="<<query.planned_splice
     <<" query_start_xyz=["<<query.start.x()<<","<<query.start.y()<<","<<query.start.z()<<"]"
     <<" query_end_xyz=["<<query.end.x()<<","<<query.end.y()<<","<<query.end.z()<<"]"
     <<" query_yaw="<<query.query_yaw
     <<" solve_start=["<<query.solve_start.x()<<","<<query.solve_start.y()<<","<<query.solve_start.z()<<"]"
     <<" acquired_body_xyz=["<<acquired.position.x()<<","<<acquired.position.y()<<","<<acquired.position.z()<<"]"
     <<" acquired_body_source_s="<<std::fixed<<acquired.source_stamp<<std::defaultfloat
     <<" acquired_body_frame="<<acquired.frame<<" scope_frame="<<map.collisionFrame()
     <<" scope_context_sequence="<<map.localizationContextSequence()
     <<" query_to_solve_start_m="<<(query.start-query.solve_start).norm()
     <<" query_to_acquired_body_m="<<(query.start-acquired.position).norm();
  // This is called only after a failed solve's return/timeout has been fixed.
  // Never consume the last part of its budget to enrich a log or reinterpret
  // a deadline/cancel as a geometric rejection. No new budget is allocated.
  if(budget&&budget->remainingSeconds()<.020) {
    out<<" evidence=skipped_diagnostic_budget";return out.str();
  }
  out<<" search_start_evidence={"<<map.describeInflateOccupancy(query.start,query.query_yaw)<<"}";
  if(budget&&budget->remainingSeconds()<.010) {
    out<<" acquired_body_evidence=skipped_diagnostic_budget";return out.str();
  }
  double yaw=0.;
  if(body_frame!=map.collisionFrame()||
     !measuredBodyYaw(acquired,body_frame,now,maximum_age,yaw)) {
    out<<" acquired_body_evidence=unavailable_frame_pose_or_source";return out.str();
  }
  const Eigen::Matrix3d rotation=acquired.orientation.normalized().toRotationMatrix();
  const double pitch=std::atan2(-rotation(2,0),std::hypot(rotation(0,0),rotation(1,0)));
  const double roll=std::atan2(rotation(2,1),rotation(2,2));
  // Explicit RPY is essential: a hypothetical detour yaw/support estimate is
  // NOT a measurement of the current body. The original quaternion is kept.
  out<<" acquired_body_rpy=["<<roll<<","<<pitch<<","<<yaw<<"]"
     <<" acquired_body_evidence={"<<map.describeInflateOccupancy(acquired.position,yaw,pitch,roll)<<"}";
  return out.str();
}

} // namespace scan_planner
