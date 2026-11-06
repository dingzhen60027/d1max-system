#pragma once
#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace scan_planner {

// Do not re-create exact endpoints by subtracting and adding floats. The
// collision cache intentionally distinguishes exact poses; a few ULPs here
// cause needless misses and can change a touching contact at the endpoint.
// Interior samples retain the existing complete half-voxel edge sampling.
inline Eigen::Vector3d searchEdgeSample(const Eigen::Vector3d &from,
    const Eigen::Vector3d &to,int sample,int samples) {
  if(samples<1 || sample<0 || sample>samples)
    throw std::invalid_argument("invalid search edge sample");
  if(sample==0)return from;
  if(sample==samples)return to;
  return from+(to-from)*(static_cast<double>(sample)/samples);
}

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
