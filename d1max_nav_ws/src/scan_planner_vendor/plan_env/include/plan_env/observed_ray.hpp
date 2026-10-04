#pragma once
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>

namespace scan_planner {
// Ring-buffer address for an ALREADY bounds-checked voxel in one fixed window.
// Its offset from minimum is in [0,size), and minimum's positive remainder is
// also in [0,size), so one conditional subtraction is exactly global % size.
// Construct again after sliding; never use this mapping for an outside voxel.
class ObservedRayMapIndex {
 public:
  ObservedRayMapIndex(const Eigen::Vector3i &minimum,const Eigen::Vector3i &size)
      : minimum_(minimum),size_(size),yz_stride_(size.y()*size.z()) {
    for(int axis=0;axis<3;++axis) {
      minimum_local_[axis]=minimum[axis]%size[axis];
      if(minimum_local_[axis]<0) minimum_local_[axis]+=size[axis];
    }
  }
  int address(const Eigen::Vector3i &cell) const {
    int x=cell.x()-minimum_.x()+minimum_local_.x();
    int y=cell.y()-minimum_.y()+minimum_local_.y();
    int z=cell.z()-minimum_.z()+minimum_local_.z();
    if(x>=size_.x()) x-=size_.x();
    if(y>=size_.y()) y-=size_.y();
    if(z>=size_.z()) z-=size_.z();
    return x*yz_stride_+y*size_.z()+z;
  }
 private:
  Eigen::Vector3i minimum_,size_,minimum_local_;
  int yz_stride_;
};

// Incremental ring address for the exact DDA below. Only the axis crossed by
// the ray changes, so do not recompute three ring coordinates and products for
// every visited cell. Bounds remain the caller's responsibility; wrapping an
// address never makes an outside cell observable/free.
class ObservedRayAddressCursor {
 public:
  explicit ObservedRayAddressCursor(const Eigen::Vector3i &size)
      : size_(size),stride_(size.y()*size.z(),size.z(),1) {}
  void set(const Eigen::Vector3i &cell) {
    for(int axis=0;axis<3;++axis) {
      local_[axis]=cell[axis]%size_[axis];
      if(local_[axis]<0) local_[axis]+=size_[axis];
    }
    address_=local_.dot(stride_);
  }
  void advance(int axis,int direction) {
    local_[axis]+=direction;
    address_+=direction*stride_[axis];
    if(local_[axis]>=size_[axis]) {
      local_[axis]=0;address_-=size_[axis]*stride_[axis];
    } else if(local_[axis]<0) {
      local_[axis]=size_[axis]-1;address_+=size_[axis]*stride_[axis];
    }
  }
  int address() const {return address_;}
 private:
  Eigen::Vector3i size_,stride_,local_;
  int address_=0;
};

struct UnindexedObservedRayCursor {
  void set(const Eigen::Vector3i &) {}
  void advance(int,int) {}
  int address() const {return 0;}
};

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
template <class Visitor,class Cursor>
bool visitObservedRayWithCursor(const Eigen::Vector3d &origin, const Eigen::Vector3d &end,
    double resolution, bool include_end, std::size_t &budget, Visitor visit,Cursor cursor) {
  if (!origin.allFinite() || !end.allFinite() || !std::isfinite(resolution) || resolution<=0.)
    return false;
  const Eigen::Vector3d start=origin/resolution, finish=end/resolution;
  Eigen::Vector3i cell=start.array().floor().cast<int>(), final=finish.array().floor().cast<int>();
  const Eigen::Vector3d direction=finish-start;
  Eigen::Vector3i step;
  Eigen::Vector3d next,delta;
  cursor.set(cell);
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
      --budget;visit(cell,cursor.address());
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
        --budget;cursor.set(final);visit(final,cursor.address());
      }
      return true;
    }
    if(!std::isfinite(crossing) || crossing>1.+1e-10) return false;
    for(int axis=0;axis<3;++axis) if(next[axis]<=crossing+1e-12) {
      cell[axis]+=step[axis];next[axis]+=delta[axis];
      cursor.advance(axis,step[axis]);
    }
  }
}

template <class Visitor>
bool visitObservedRay(const Eigen::Vector3d &origin,const Eigen::Vector3d &end,
    double resolution,bool include_end,std::size_t &budget,Visitor visit) {
  return visitObservedRayWithCursor(origin,end,resolution,include_end,budget,
      [&](const Eigen::Vector3i &cell,int){visit(cell);},UnindexedObservedRayCursor{});
}

template <class Visitor>
bool visitObservedRayIndexed(const Eigen::Vector3d &origin,const Eigen::Vector3d &end,
    double resolution,bool include_end,std::size_t &budget,const Eigen::Vector3i &size,
    Visitor visit) {
  return visitObservedRayWithCursor(origin,end,resolution,include_end,budget,visit,
      ObservedRayAddressCursor(size));
}
} // namespace scan_planner
