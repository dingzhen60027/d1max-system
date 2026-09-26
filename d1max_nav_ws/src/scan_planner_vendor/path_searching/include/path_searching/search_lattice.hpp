#pragma once
#include <Eigen/Core>
#include <cmath>

namespace scan_planner {

// The search lattice is centred BETWEEN the endpoints. Signed truncation after
// adding .5 shifts the negative half by a whole cell; round both halves alike.
inline bool nearestSearchIndex(const Eigen::Vector3d &point,
    const Eigen::Vector3d &center, double resolution,
    const Eigen::Vector3i &center_index, const Eigen::Vector3i &size,
    Eigen::Vector3i &index) {
  if (!point.allFinite() || !center.allFinite() || !std::isfinite(resolution) ||
      resolution<=0. || (size.array()<=0).any()) return false;
  for (int axis=0; axis<3; ++axis) {
    const double value=std::round((point[axis]-center[axis])/resolution)+center_index[axis];
    if (!std::isfinite(value) || value<0. || value>=size[axis]) return false;
    index[axis]=static_cast<int>(value);
  }
  return true;
}

inline const char *occupancyStateName(int state) {
  switch (state) {
    case 0: return "observed_free";
    case 1: return "occupied";
    case 2: return "unobserved_or_uncertain";
    case -1: return "outside_map";
    default: return "not_evaluated";
  }
}

}  // namespace scan_planner
