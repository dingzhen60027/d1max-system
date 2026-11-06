#pragma once
#include <Eigen/Core>
#include <vector>

namespace scan_planner {
// A structural exception to the old .2 m no-hover solve shortcut. The caller
// must have selected the SAME typed final task endpoint with current support
// and endpoint occupancy evidence. This grants only a short SCAN solve, never
// motion: normal A*, optimization, dynamics and whole-curve proof still run.
inline bool finalEndpointShortConnectionAllowed(bool requested,
    const std::vector<Eigen::Vector3d>& reference,const Eigen::Vector3d& actual,
    const Eigen::Vector3d& target,bool planned_splice) {
  if(!requested||planned_splice||reference.size()!=2||!actual.allFinite()||!target.allFinite()||
     !reference.front().allFinite()||!reference.back().allFinite()||
     (reference.front()-actual).norm()>1e-6||(reference.back()-target).norm()>1e-6)return false;
  const double distance=(target-actual).norm();
  return distance>.15&&distance<.2;
}
} // namespace scan_planner
