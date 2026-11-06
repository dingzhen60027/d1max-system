#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <chrono>
#include <vector>

namespace scan_planner {

// Legacy prediction may be discarded when it points away from the goal, but
// actual odometry is a physical boundary, including reverse/sideways motion.
inline bool shouldSuppressOpposedPrediction(bool measured_boundary,
    const Eigen::Vector2d &to_goal, const Eigen::Vector3d &velocity)
{
  return !measured_boundary && to_goal.norm() > 1e-3 &&
         velocity.head<2>().dot(to_goal) < 0.0;
}

inline bool periodicReplanDue(double now, double last_attempt, double interval)
{
  return std::isfinite(now) && std::isfinite(last_attempt) &&
         std::isfinite(interval) && interval > 0.0 && now - last_attempt >= interval;
}

// Event requests share the actual solver submission clock, not the periodic
// preview clock (which can advance without a solve). There is still only one
// worker and every admitted solve retains its original 400 ms SolveBudget.
// Zero disables only the additional submission spacing, not worker/deadline
// backpressure. Negative/non-finite intervals and a backwards clock reject.
inline bool formalReseedSubmissionDue(std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point last_submission,bool worker_running,
    double minimum_interval_s=.1)
{
  return !worker_running && std::isfinite(minimum_interval_s) && minimum_interval_s>=0. &&
    (last_submission==std::chrono::steady_clock::time_point{} ||
    std::chrono::duration<double>(now-last_submission).count()>=minimum_interval_s);
}

// A false dispatch result may mean no solve was attempted. The 100 Hz FSM
// must not count waiting for its single worker/snapshot/submission clock as
// repeated optimizer failures. A completed failed turn is counted separately.
inline bool nativePlanningAttemptDeferred(bool worker_running,bool snapshot_pending,
    const std::string& phase) {
  return worker_running||snapshot_pending||phase=="waiting_solve_budget";
}

// A source-pair/connection or measured-heading miss can recover with a new actual body sample;
// it is not evidence of a blocked environment. Do not require the stationary
// robot to move 15 cm before retrying the very candidate needed to move it.
inline bool nativeConnectionWait(const std::string& phase) {
  return phase=="waiting_measured_target_heading" ||
      phase=="failed_curve_validation" || phase=="waiting_splice_incumbent" ||
      phase=="waiting_splice_validation" || phase=="candidate_measured_join_rejected" ||
      phase=="candidate_source_expired" || phase=="candidate_source_changed";
}

// The robot moving while a candidate is being computed is a new planning
// boundary, not an obstacle/optimizer failure. Only these adoption outcomes
// can skip the environmental backoff, and only with fresh changed input.
inline bool nativeLatestStateRetry(const std::string& phase) {
  return phase=="candidate_measured_join_rejected"||
      phase=="candidate_source_expired"||phase=="candidate_source_changed";
}

// Snapshot contention and an exhausted whole-turn deadline are scheduling
// failures, not evidence that geometry is blocked. After the old worker and
// its budget retire, the FSM may start a distinct bounded turn after its normal
// cooldown. It must not wait for a stationary saturated map to change before
// retrying recovered compute resources, nor renew the expired SolveBudget.
// Collision, unknown-space and invalid-input failures remain evidence-gated.
inline bool nativeResourceWait(const std::string &phase)
{
  return phase == "waiting_snapshot_slot" || phase == "solve_deadline_exceeded" ||
      phase == "solve_search_deadline_exceeded" || phase=="candidate_adoption_budget_exhausted" ||
      phase == "waiting_reference_support";
}

// A solver/retiming miss is not proof that the environment must physically
// change. Retry under the configured cooldown when fresh REAL planning input
// arrives, even if a saturated map's occupancy bits stay unchanged. This
// classification only requests computation; geometry/unknown, malformed
// input, cancellation and unclassified exceptions keep their existing fences.
inline bool nativeNumericalReplanFailure(const std::string &phase)
{
  return phase=="failed_optimization"||phase=="failed_rebound_search"||phase=="failed_dynamics";
}

// These are selector envelope QUERY counts, never unique physical voxels.
// Diagnostic-only classification; occupied/unknown policy is unchanged.
inline const char* referenceTargetEvidenceClass(std::size_t occupied,std::size_t unknown,
    std::size_t outside) {
  if(occupied&&unknown)return "target_mixed_occupied_unknown";
  if(occupied)return "target_occupied";
  if(unknown)return "target_unknown";
  return outside?"target_outside":"target_no_blocking_query";
}

inline bool measuredReferenceGoalReached(const Eigen::Vector3d &body,
    const Eigen::Vector3d &goal, double xy_tolerance, double z_tolerance)
{
  return body.allFinite() && goal.allFinite() &&
      (body.head<2>() - goal.head<2>()).norm() <= xy_tolerance &&
      std::abs(body.z() - goal.z()) <= z_tolerance;
}

// A stationary map does not imply unchanged planning boundary conditions.
// Retry when an infeasible measured speed becomes admissible, or a meaningful
// measured velocity change can resolve a failed dynamics solve. The FSM also
// enforces its monotonic cooldown, so noisy feedback cannot spin the optimizer.
inline bool failedDynamicsBoundaryChanged(const Eigen::Vector3d &failed_velocity,
    const Eigen::Vector3d &velocity, double speed_limit)
{
  if (!failed_velocity.allFinite() || !velocity.allFinite() ||
      !std::isfinite(speed_limit) || speed_limit <= 0. || velocity.norm() > speed_limit + 1e-9)
    return false;
  return failed_velocity.norm() > speed_limit + 1e-9 ||
      (velocity - failed_velocity).norm() >= .02;
}

// Rotation changes the measured initial heading and the two-cylinder envelope
// even when translation and voxel occupancy are unchanged. Ignore attitude
// noise and quaternion sign flips; this is a retry trigger, never free space.
inline bool failedOrientationBoundaryChanged(const Eigen::Quaterniond &failed,
    const Eigen::Quaterniond &current, double angle_threshold = .10)
{
  if (!failed.coeffs().allFinite() || !current.coeffs().allFinite() ||
      failed.norm() < 1e-6 || current.norm() < 1e-6 ||
      !std::isfinite(angle_threshold) || angle_threshold <= 0.) return false;
  return failed.normalized().angularDistance(current.normalized()) >= angle_threshold;
}

inline bool acceptsReferenceGeneration(const std::string &configured_session,
                                      const std::string &message_session,
                                      bool already_seen, uint64_t previous, uint64_t incoming)
{
  return !configured_session.empty() && configured_session == message_session &&
         (!already_seen || incoming > previous);
}

// A PCT reference contains surface positions; SCAN optimizes body positions.
// Remove only zero-length segments, never simplify away corridor corners.
inline std::vector<Eigen::Vector3d> prepareReferenceWaypoints(
    const std::vector<Eigen::Vector3d> &surface_points,
    const Eigen::Vector3d &body_position, double z_offset,
    double maximum_start_distance = 1.0)
{
  if (surface_points.empty() || !body_position.allFinite() ||
      !std::isfinite(z_offset) || maximum_start_distance <= 0.0)
    throw std::invalid_argument("invalid reference path or body position");
  std::vector<Eigen::Vector3d> out;
  Eigen::Vector3d previous = body_position;
  for (std::size_t i = 0; i < surface_points.size(); ++i) {
    Eigen::Vector3d point = surface_points[i];
    if (!point.allFinite()) throw std::invalid_argument("non-finite reference point");
    point.z() += z_offset;
    if (i == 0 && (point - body_position).norm() > maximum_start_distance)
      throw std::invalid_argument("reference start is too far from current body pose");
    if ((point - previous).norm() <= 1e-3) continue;
    out.push_back(point);
    previous = point;
  }
  if (out.empty()) throw std::invalid_argument("reference path has no nonzero segment");
  return out;
}

inline Eigen::Vector3d odometryVelocityInWorld(
    const Eigen::Vector3d &velocity, Eigen::Quaterniond orientation, bool twist_in_body)
{
  if (!velocity.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-6)
    throw std::invalid_argument("invalid odometry orientation or velocity");
  orientation.normalize();
  return twist_in_body ? orientation * velocity : velocity;
}

}  // namespace scan_planner
