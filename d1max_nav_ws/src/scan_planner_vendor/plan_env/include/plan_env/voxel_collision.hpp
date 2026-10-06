#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace scan_planner {

// The obstacle dilation is not the robot's vertical extent. A query at q
// examines raw obstacles at q-kernel_offset: the body is the REFLECTED kernel.
// Named types keep an asymmetric up/down calibration from silently changing
// meaning between pose, rotation and command-sweep checks.
struct ObstacleDilation { double radius, offset, up, down; };
struct BodyEnvelope { double radius, offset, below, above; };
inline BodyEnvelope reflectedBodyEnvelope(const ObstacleDilation& kernel) {
  return {kernel.radius,kernel.offset,kernel.up,kernel.down};
}

// Native low occupied probability is not itself free evidence: one hit added to
// the negative unknown prior can still lie below the occupied threshold. Strict
// mode admits only saturated free cells; intermediate evidence stays uncertain.
inline int strictRawVoxelStatus(double odds,double free_log,double occupied_log) {
  if (!std::isfinite(odds) || odds<free_log-1e-9) return 2;
  if (odds>occupied_log) return 1;
  return odds<=free_log+1e-9 ? 0:2;
}

// Diagnostic refinement only: native status 2 intentionally remains blocked
// for BOTH never-observed and measured-but-insufficient evidence.
enum class RawVoxelDiagnostic {Free,Occupied,NeverObserved,Insufficient,Outside,Invalid,StaleFree};
inline RawVoxelDiagnostic diagnoseRawVoxel(double odds,double free_log,double occupied_log,double unknown_flag) {
  if (!std::isfinite(odds)) return RawVoxelDiagnostic::Invalid;
  if (std::abs(odds-(free_log-unknown_flag))<=1e-9) return RawVoxelDiagnostic::NeverObserved;
  if (odds<free_log-1e-9) return RawVoxelDiagnostic::Invalid;
  const int native=strictRawVoxelStatus(odds,free_log,occupied_log);
  return native==0 ? RawVoxelDiagnostic::Free : native==1 ? RawVoxelDiagnostic::Occupied :
      RawVoxelDiagnostic::Insufficient;
}
inline const char *diagnosticName(RawVoxelDiagnostic state) {
  switch(state) {
    case RawVoxelDiagnostic::Free:return "observed_free";
    case RawVoxelDiagnostic::Occupied:return "occupied";
    case RawVoxelDiagnostic::NeverObserved:return "never_observed";
    case RawVoxelDiagnostic::Insufficient:return "observed_insufficient";
    case RawVoxelDiagnostic::Outside:return "outside";
    case RawVoxelDiagnostic::StaleFree:return "stale_free";
    default:return "invalid";
  }
}
struct CollisionVoxelDiagnostic {
  Eigen::Vector3i index{Eigen::Vector3i::Zero()};
  double log_odds{std::numeric_limits<double>::quiet_NaN()};
  int native_state{-1};
  int raw_state{-1};
  std::int64_t free_observation_stamp_ns{0};
  RawVoxelDiagnostic classification{RawVoxelDiagnostic::Outside};
  unsigned cylinder_mask{0}; // bit 0 rear, bit 1 front. Never a ring-buffer address.
};

// Snapshot-scoped: the owner must clear it for every raw-map mutation/integration
// and ring-buffer slide. The bound limits memory even during a failed search.
class VoxelStatusCache {
  friend struct VoxelStatusCacheTestAccess;
 public:
  explicit VoxelStatusCache(std::size_t capacity=32768):capacity_(capacity) {
    if (!capacity_) throw std::invalid_argument("zero voxel cache capacity");
    if (capacity_>std::numeric_limits<std::uint32_t>::max())
      throw std::length_error("voxel cache capacity exceeds index range");
  }
  void clear() {
    entries_.clear();
    if(clear_serial_<std::numeric_limits<std::uint64_t>::max())++clear_serial_;
    // Retain bounded storage across cloud/proof changes. A new generation
    // invalidates the index without freeing/reallocating one node per query.
    if (++generation_==0) {
      for(auto& slot:slots_)slot.generation=0;
      generation_=1;
    }
  }
  // Dense dependants must follow every clear, even when the slot generation
  // wraps to an old value. At the serial limit they fall back to uncached
  // evidence, rather than ever reuse an ambiguous generation.
  std::uint64_t generationIdentifier() const {return clear_serial_;}
  bool generationReusable() const {return clear_serial_<std::numeric_limits<std::uint64_t>::max();}
  std::size_t size() const { return entries_.size(); }
  // Reserve accounting is independent of cold/hot cache population. Readers
  // must budget the bounded eventual allocation before accepting a snapshot.
  std::size_t maximumStorageBytes() const {
    std::size_t slots=1;
    while(slots<capacity_*2)slots*=2;
    return capacity_*sizeof(Entry)+slots*sizeof(Slot);
  }
  template <class Query> int get(int address, Query query) {
    return get(address, 0, 0, query);
  }
  // The same center voxel can straddle two different actual-height envelopes.
  // Reusing only its address could hide a newly included low obstacle.
  template <class Query> int get(int address, int low_z, int high_z, Query query) {
    return getExact(address,low_z,high_z,0.,0.,query);
  }
  template <class Query> int getExact(int address, int low_z, int high_z,
      double x, double y, Query query) {
    std::uint64_t x_bits,y_bits;
    std::memcpy(&x_bits,&x,sizeof(x)); std::memcpy(&y_bits,&y,sizeof(y));
    const Key key{address,low_z,high_z,x_bits,y_bits};
    if(!slots_.empty()) {
      const auto slot=findSlot(key);
      if(slots_[slot].generation==generation_)return entries_[slots_[slot].index].state;
    }
    const int result=query();
    if (entries_.size()>=capacity_) clear();
    ensureStorage();
    const auto slot=findSlot(key);
    // Preserve emplace semantics even if a query itself populated the key.
    if(slots_[slot].generation!=generation_) {
      slots_[slot]={static_cast<std::uint32_t>(entries_.size()),generation_};
      entries_.push_back({key,result});
    }
    return result;
  }
 private:
  struct Key {
    int address,low_z,high_z;
    std::uint64_t x_bits,y_bits;
    bool operator==(const Key &other) const {
      return address==other.address && low_z==other.low_z && high_z==other.high_z
          && x_bits==other.x_bits && y_bits==other.y_bits;
    }
  };
  struct Hash {
    std::size_t operator()(const Key &key) const {
      std::size_t h=std::hash<int>{}(key.address);
      for (int v:{key.low_z,key.high_z}) h^=std::hash<int>{}(v)+0x9e3779b9+(h<<6)+(h>>2);
      for (auto v:{key.x_bits,key.y_bits}) h^=std::hash<std::uint64_t>{}(v)+0x9e3779b9+(h<<6)+(h>>2);
      return h;
    }
  };
  struct Entry {Key key;int state;};
  struct Slot {std::uint32_t index{0},generation{0};};
  std::size_t findSlot(const Key& key) const {
    auto slot=Hash{}(key)&(slots_.size()-1);
    while(slots_[slot].generation==generation_ &&
        !(entries_[slots_[slot].index].key==key))slot=(slot+1)&(slots_.size()-1);
    return slot;
  }
  void ensureStorage() {
    if(!slots_.empty() && entries_.size()<slots_.size()/2 &&
        entries_.size()<entries_.capacity())return;
    const auto target=std::min(capacity_,std::max(std::size_t(16),entries_.size()*2));
    std::size_t slot_count=1;
    while(slot_count<target*2)slot_count*=2;
    entries_.reserve(target);
    std::vector<Slot> slots(slot_count);
    slots_.swap(slots);
    for(std::size_t i=0;i<entries_.size();++i) {
      const auto slot=findSlot(entries_[i].key);
      slots_[slot]={static_cast<std::uint32_t>(i),generation_};
    }
  }
  std::size_t capacity_;
  std::uint32_t generation_{1};
  std::uint64_t clear_serial_{1};
  std::vector<Entry> entries_;
  std::vector<Slot> slots_;
};

struct VerticalVoxelSpan { int low,high; };
struct CollisionEvidence {
  // Slots: measured free, occupied, unknown, outside. Cylinder overlap may
  // count a voxel twice; this does not change the decision or marker identity.
  std::array<std::size_t,4> counts{{0,0,0,0}};
  std::array<Eigen::Vector3i,3> first;
  std::array<std::size_t,4> unique_counts{{0,0,0,0}};
  std::vector<CollisionVoxelDiagnostic> voxels;
  // Indexed by RawVoxelDiagnostic. Only detailed queries populate these;
  // counts are UNIQUE global voxel indices, never envelope-query counts.
  std::array<std::size_t,7> classification_counts{{0,0,0,0,0,0,0}};
  int state() const {
    return counts[1] ? 1 : counts[3] ? -1 : counts[2] ? 2 : 0;
  }
};

// Retain the obstacle voxel's full AABB and touching contacts. The query
// centre is known exactly; expanding its whole voxel again is unnecessary.
inline bool cylinderIntersectsVoxelXY(const Eigen::Vector3d &center,
    const Eigen::Vector3i &voxel, double resolution, double radius) {
  if (!center.allFinite() || !std::isfinite(resolution) || resolution<=0. ||
      !std::isfinite(radius) || radius<0.)
    throw std::invalid_argument("invalid exact cylinder geometry");
  double distance_squared=0.;
  for (int axis=0;axis<2;++axis) {
    const double lo=voxel[axis]*resolution, hi=(voxel[axis]+1.)*resolution;
    const double delta=std::max({lo-center[axis],center[axis]-hi,0.});
    distance_squared+=delta*delta;
  }
  return distance_squared<=radius*radius+1e-12;
}

// Query position is known, not any possible height in its enclosing voxel.
// Keep the obstacle's full voxel interval (including touching boundaries),
// but avoid adding an extra query-voxel height that turns the ground into an
// obstacle after extending the leg envelope downwards.
inline VerticalVoxelSpan verticalVoxelSpan(double z, double below, double above, double resolution) {
  if (!std::isfinite(z) || !std::isfinite(below) || !std::isfinite(above) ||
      !std::isfinite(resolution) || below<0. || above<0. || resolution<=0.)
    throw std::invalid_argument("invalid actual-height collision envelope");
  const double lo=std::ceil((z-below)/resolution-1e-10)-1.;
  const double hi=std::floor((z+above)/resolution+1e-10);
  if (!std::isfinite(lo) || !std::isfinite(hi) ||
      lo<std::numeric_limits<int>::min() || hi>std::numeric_limits<int>::max())
    throw std::invalid_argument("collision height exceeds index range");
  return {static_cast<int>(lo),static_cast<int>(hi)};
}

// Both the occupied cell and queried cylinder-center cell have volume. Their
// closest XY separation is max(|index delta|*resolution-resolution, 0), not
// their center-to-center separation. This conservative discrete Minkowski sum
// includes contact and cannot lose a real obstacle at a voxel corner.
inline std::vector<Eigen::Vector3i> conservativeCylinderInflation(
    double resolution, double radius, double inflation_up, double inflation_down) {
  if (!std::isfinite(resolution) || resolution<=0. || !std::isfinite(radius) || radius<0. ||
      !std::isfinite(inflation_up) || inflation_up<0. ||
      !std::isfinite(inflation_down) || inflation_down<0.)
    throw std::invalid_argument("invalid cylinder voxel geometry");
  const double xy=std::ceil(radius/resolution)+1.;
  const double z_up=std::ceil(inflation_up/resolution)+1.;
  const double z_down=std::ceil(inflation_down/resolution)+1.;
  if ((2.*xy+1.)*(2.*xy+1.)*(z_up+z_down+1.)>200000.)
    throw std::invalid_argument("cylinder voxel kernel exceeds bounded budget");
  std::vector<Eigen::Vector3i> result;
  for (int x=-static_cast<int>(xy);x<=static_cast<int>(xy);++x)
    for (int y=-static_cast<int>(xy);y<=static_cast<int>(xy);++y) {
      const double dx=std::max(0.,std::abs(x)*resolution-resolution);
      const double dy=std::max(0.,std::abs(y)*resolution-resolution);
      if (std::hypot(dx,dy)>radius+1e-12) continue;
      for (int z=-static_cast<int>(z_down);z<=static_cast<int>(z_up);++z) {
        // Exact cell-interval intersection, avoiding ceil-only overgrowth.
        if (z*resolution>inflation_up+resolution+1e-12 ||
            z*resolution< -inflation_down-resolution-1e-12) continue;
        result.emplace_back(x,y,z);
      }
    }
  return result;
}

// Reverse the obstacle-inflation offsets to visit all raw cells that could
// intersect a cylinder whose actual center lies anywhere in center_cell.
// queryRaw: 0 measured free, 1 occupied, 2 unobserved, -1 outside map.
template <class RawQuery>
int observedCylinderStatus(const Eigen::Vector3i &center_cell,
    const std::vector<Eigen::Vector3i> &inflation_offsets, RawQuery queryRaw) {
  for (const auto &offset:inflation_offsets) {
    const Eigen::Vector3i body_cell=center_cell-offset;
    const int state=queryRaw(body_cell);
    if (state!=0) return state;
  }
  return 0;
}

template <class RawQuery>
int observedCylinderStatusAtHeight(const Eigen::Vector3i &center_cell,
    const VerticalVoxelSpan &span, const std::vector<Eigen::Vector3i> &inflation_offsets,
    RawQuery queryRaw) {
  for (const auto &offset:inflation_offsets) {
    const Eigen::Vector3i body_cell=center_cell-offset;
    if (body_cell.z()<span.low || body_cell.z()>span.high) continue;
    const int state=queryRaw(body_cell);
    if (state!=0) return state;
  }
  return 0;
}

template <class RawQuery>
int observedCylinderStatusAtPosition(const Eigen::Vector3d &center,
    const Eigen::Vector3i &center_cell, const VerticalVoxelSpan &span,
    const std::vector<Eigen::Vector3i> &inflation_offsets,
    double resolution, double radius, RawQuery queryRaw) {
  for (const auto &offset:inflation_offsets) {
    const Eigen::Vector3i cell=center_cell-offset;
    if (cell.z()<span.low || cell.z()>span.high ||
        !cylinderIntersectsVoxelXY(center,cell,resolution,radius)) continue;
    const int state=queryRaw(cell);
    if (state!=0) return state;
  }
  return 0;
}

}  // namespace scan_planner
