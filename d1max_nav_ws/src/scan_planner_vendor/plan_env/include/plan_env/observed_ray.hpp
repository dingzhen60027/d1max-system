#pragma once
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>

namespace scan_planner {
// Clip a measured ray, not its evidence, at the sensor-centred update box.
// A clipped endpoint is NOT a surface hit. Returns false for invalid geometry.
inline bool clipObservedRay(const Eigen::Vector3d &origin,
    const Eigen::Vector3d &half_range, Eigen::Vector3d &end, bool &hit) {
  if (!origin.allFinite() || !end.allFinite() || !half_range.allFinite() ||
      (half_range.array() <= 1e-6).any()) return false;
  const Eigen::Vector3d direction=end-origin;
  double fraction=1.;
  for (int axis=0;axis<3;++axis)
    if (std::abs(direction[axis])>half_range[axis])
      fraction=std::min(fraction,(half_range[axis]-1e-6)/std::abs(direction[axis]));
  if (fraction<1.) { end=origin+fraction*direction;hit=false; }
  return true;
}

// Exact endpoint-direction voxel traversal. A previously visited voxel is not
// proof that the rest of another ray was traversed. Never stop at shared cells.
// Hit endpoint cells are excluded from free evidence. Truncated no-hit rays may
// include the endpoint. Simultaneous crossings skip zero-length corner cells.
template <class Visitor>
bool visitObservedRay(const Eigen::Vector3d &origin, const Eigen::Vector3d &end,
    double resolution, bool include_end, std::size_t &budget, Visitor visit) {
  if (!origin.allFinite() || !end.allFinite() || !std::isfinite(resolution) || resolution<=0.)
    return false;
  const Eigen::Vector3d start=origin/resolution, finish=end/resolution;
  Eigen::Vector3i cell=start.array().floor().cast<int>(), final=finish.array().floor().cast<int>();
  const Eigen::Vector3d direction=finish-start;
  Eigen::Vector3i step;
  Eigen::Vector3d next,delta;
  for(int axis=0;axis<3;++axis) {
    if(direction[axis]>0.) {
      step[axis]=1;delta[axis]=1./direction[axis];
      next[axis]=(cell[axis]+1.-start[axis])/direction[axis];
    } else if(direction[axis]<0.) {
      step[axis]=-1;delta[axis]=-1./direction[axis];
      next[axis]=(cell[axis]-start[axis])/direction[axis];
    } else {step[axis]=0;delta[axis]=next[axis]=std::numeric_limits<double>::infinity();}
  }
  while(true) {
    const bool at_end=(cell==final);
    if(!at_end || include_end) {
      if(!budget) return false;
      --budget;visit(cell);
    }
    if(at_end) return true;
    const double crossing=next.minCoeff();
    // A negative-direction endpoint exactly on a grid face belongs to the cell
    // on the other side of that face by floor(), but the ray must not advance
    // beyond t=1 while trying to reach every endpoint index at a tied crossing.
    // There is no positive-length free segment beyond the measured endpoint.
    if (crossing>=1.-1e-12 && std::isfinite(crossing)) {
      if (include_end) {
        if (!budget) return false;
        --budget;visit(final);
      }
      return true;
    }
    if(!std::isfinite(crossing) || crossing>1.+1e-10) return false;
    for(int axis=0;axis<3;++axis) if(next[axis]<=crossing+1e-12) {
      cell[axis]+=step[axis];next[axis]+=delta[axis];
    }
  }
}
} // namespace scan_planner
