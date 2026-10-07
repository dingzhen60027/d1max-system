#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <optional>

namespace scan_planner {

struct EntryDerivativeFit {
  Eigen::Vector3d solver_velocity, solver_acceleration;
  bool low_speed_fitted{false};
};

// Candidate selection inside the existing 0.05 m/s XYZ join tolerance. This
// does not certify physical rest, change measured state or authorize a write.
// Both incoming boundary and original measured velocity must be low; preserve
// any nonzero acceleration boundary instead of relabelling it as measured zero.
inline EntryDerivativeFit fitLowSpeedEntryDerivatives(
    bool enabled, bool guided, const std::optional<double>& certified_floor_z,
    std::int64_t measured_source_ns, const Eigen::Vector3d& start_velocity,
    const Eigen::Vector3d& start_acceleration, const Eigen::Vector3d& measured_velocity) {
  EntryDerivativeFit result{start_velocity,start_acceleration,false};
  if(!enabled||!guided||!certified_floor_z||!std::isfinite(*certified_floor_z)||
      measured_source_ns<=0||!start_velocity.allFinite()||!measured_velocity.allFinite()||
      !start_acceleration.allFinite()||(start_acceleration.array()!=0.).any()||
      std::hypot(start_velocity.x(),start_velocity.y(),start_velocity.z())>.03||
      std::hypot(measured_velocity.x(),measured_velocity.y(),measured_velocity.z())>.03)return result;
  result.solver_velocity.setZero();result.solver_acceleration.setZero();result.low_speed_fitted=true;
  return result;
}

}  // namespace scan_planner
