#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace scan_planner {

// The existing actual-state candidate join tolerance, not a new permission or
// a noise/physics assertion. Selection below can never hide a larger residual.
inline constexpr double measured_candidate_join_velocity_tolerance_mps=.05;

struct ForwardMotionBoundary {
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d reference_direction{Eigen::Vector3d::Zero()};
  double velocity_residual_mps{0.};
  bool regularized{false};
  // Diagnostic only: a real reverse transient remains a literal solve PVA.
  bool reverse_transient{false};
};

// Select an OPTIMIZER boundary, never rewrite odometry. In forward-only mode a
// near-rest backward/lateral measurement has no reliable forward tangent; an
// exact negative derivative would instead force a tiny loop and a pi body turn.
// The ordered reference supplies forward direction, independently of heading
// error or a position-to-path vector. Actual join/collision checks still decide
// adoption against the unchanged acquired state.
inline ForwardMotionBoundary selectForwardMotionBoundary(const Eigen::Vector3d& position,
    const Eigen::Vector3d& measured_velocity,const std::vector<Eigen::Vector3d>& reference,
    bool forward_only,bool fixed_splice,std::optional<double> measured_body_yaw={},
    bool has_start_projection_connector=false) {
  ForwardMotionBoundary out;out.velocity=measured_velocity;
  if(!forward_only||fixed_splice||!position.allFinite()||!measured_velocity.allFinite()||
      reference.size()<2)return out;
  double nearest=std::numeric_limits<double>::infinity();
  // Guided slices prepend actual_start before the ordered route projection.
  // That first connector has zero distance to the body by construction, but
  // is NOT the route's forward tangent (it can be sideways or nearly vertical
  // under ordinary height noise). Select direction from the ordered route.
  // The slice producer explicitly identifies this connector: when actual and
  // projected positions coincide, slice() removes the duplicate and the first
  // segment is real route geometry (possibly just before a bend or stairs).
  const std::size_t first=has_start_projection_connector?2:1;
  for(std::size_t i=first;i<reference.size();++i) {
    if(!reference[i-1].allFinite()||!reference[i].allFinite())continue;
    const Eigen::Vector3d delta=reference[i]-reference[i-1];
    if(delta.head<2>().squaredNorm()<1e-12)continue;
    const double alpha=std::clamp((position-reference[i-1]).dot(delta)/delta.squaredNorm(),0.,1.);
    const double square=(position-reference[i-1]-alpha*delta).squaredNorm();
    if(square<=nearest) {nearest=square;out.reference_direction=delta;}
  }
  const double planar_length=out.reference_direction.head<2>().norm();
  if(!(planar_length>1e-6))return out;
  out.reference_direction/=planar_length;
  const double along=measured_velocity.head<2>().dot(out.reference_direction.head<2>());
  if(measured_velocity.norm()>=measured_candidate_join_velocity_tolerance_mps) {
    // A real reverse transient cannot be replaced by zero outside the actual
    // join budget. Record when body and ordered reference agree on ahead,
    // without suppressing the solve: shared body-heading geometry and native
    // candidate checks decide whether the literal boundary can be followed.
    // A goal behind the body, corners and fixed splices are not tagged.
    if(measured_body_yaw&&std::isfinite(*measured_body_yaw)) {
      const Eigen::Vector2d heading{std::cos(*measured_body_yaw),std::sin(*measured_body_yaw)};
      out.reverse_transient=along<=-measured_candidate_join_velocity_tolerance_mps&&
        heading.dot(out.reference_direction.head<2>())>.5;
    }
    return out;
  }
  const double lateral=(measured_velocity.head<2>()-along*out.reference_direction.head<2>()).norm();
  if(along>0.&&along>=lateral)return out; // Preserve a meaningful forward start exactly.
  out.velocity.setZero();out.velocity_residual_mps=measured_velocity.norm();
  out.regularized=out.velocity_residual_mps>0.;return out;
}

} // namespace scan_planner
