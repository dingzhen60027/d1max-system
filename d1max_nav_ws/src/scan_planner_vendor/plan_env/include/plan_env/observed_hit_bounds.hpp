#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace scan_planner {
// Conservative bounding box of ALL original hit endpoints since the last
// measured-free/reset event. Six bytes per voxel, no allocation per ray. A hit
// in an already-set octant still extends these bounds: octant membership alone
// cannot distinguish a tread from a new object inside the same octant.
// Validity belongs to occupied_hit_subvoxels_: zero/coarse-only history is not
// explained by this record. Quantization rounds OUTWARDS (<= resolution/255).
struct ObservedHitBounds {
  std::array<std::uint8_t,3> low{},high{};
  bool include(const Eigen::Vector3d& point,const Eigen::Vector3i& cell,
               double resolution,bool first) {
    const Eigen::Vector3d relative=(point/resolution-cell.cast<double>())*255.;
    bool changed=first;
    for(int i=0;i<3;++i) {
      const auto lo=static_cast<std::uint8_t>(std::clamp(std::floor(relative[i]-1e-8),0.,255.));
      const auto hi=static_cast<std::uint8_t>(std::clamp(std::ceil(relative[i]+1e-8),0.,255.));
      const auto next_lo=first?lo:std::min(low[i],lo);
      const auto next_hi=first?hi:std::max(high[i],hi);
      changed=changed||next_lo!=low[i]||next_hi!=high[i];low[i]=next_lo;high[i]=next_hi;
    }
    return changed;
  }
  std::pair<Eigen::Vector3d,Eigen::Vector3d> world(const Eigen::Vector3i& cell,double resolution) const {
    Eigen::Vector3d lo,hi;
    for(int i=0;i<3;++i){lo[i]=(cell[i]+double(low[i])/255.)*resolution;
      hi[i]=(cell[i]+double(high[i])/255.)*resolution;}
    return {lo,hi};
  }
};
static_assert(sizeof(ObservedHitBounds)==6,"fixed bounded ray-evidence storage");
}
