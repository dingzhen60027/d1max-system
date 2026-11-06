#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
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
enum class CollisionModel { DoubleCylinder, OrientedBox };
// Trailing defaults preserve the legacy four-field aggregate contract.
// OBB length/width are FULL dimensions, centred on the body reference point.
// The support reference height is deliberately not a collision dimension.
struct ObstacleDilation {
  double radius, offset, up, down;
  CollisionModel model{CollisionModel::DoubleCylinder};
  double length{0.}, width{0.};
};
struct BodyEnvelope {
  double radius, offset, below, above;
  CollisionModel model{CollisionModel::DoubleCylinder};
  double length{0.}, width{0.};
};
inline BodyEnvelope reflectedBodyEnvelope(const ObstacleDilation& kernel) {
  return {kernel.radius,kernel.offset,kernel.up,kernel.down,kernel.model,kernel.length,kernel.width};
}
inline void validateBodyEnvelope(const BodyEnvelope& body) {
  if (!std::isfinite(body.below) || !std::isfinite(body.above) || body.below<0. || body.above<0.)
    throw std::invalid_argument("invalid body vertical extent");
  if (body.model==CollisionModel::OrientedBox) {
    if (!std::isfinite(body.length) || !std::isfinite(body.width) || body.length<=0. || body.width<=0.)
      throw std::invalid_argument("invalid oriented box dimensions");
  } else if (body.model!=CollisionModel::DoubleCylinder || !std::isfinite(body.radius) ||
      !std::isfinite(body.offset) || body.radius<0. || body.offset<0.) {
    throw std::invalid_argument("invalid double cylinder dimensions");
  }
}
// Farthest footprint point from the body reference: used by angular sampling
// and swept-volume bounds, not as an inflated obstacle radius for an OBB.
inline double bodyExtent(const BodyEnvelope& body) {
  validateBodyEnvelope(body);
  return body.model==CollisionModel::OrientedBox ? std::hypot(body.length*.5,body.width*.5) :
      body.radius+body.offset;
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
  // Legacy bits 0/1 are rear/front cylinder; bit 2 is the one OBB footprint.
  // The public name stays compatible with existing diagnostic consumers.
  unsigned cylinder_mask{0}; // Never a ring-buffer address.
  // Preserve RAW classifications/counts above. A coarse occupied candidate
  // need not intersect after source-backed vertical narrowphase; its centre
  // alone is not the endpoint which caused the final collision decision.
  std::uint16_t occupied_hit_subvoxel_mask{0};
  bool occupied_narrowphase_applied{false};
  bool occupied_intersects{false};
};

// Snapshot-scoped: the owner must clear it for every raw-map mutation/integration
// and ring-buffer slide. The bound limits memory even during a failed search.
class VoxelStatusCache {
 public:
  explicit VoxelStatusCache(std::size_t capacity=32768):capacity_(capacity) {
    if (!capacity_) throw std::invalid_argument("zero voxel cache capacity");
    if (capacity_>std::numeric_limits<std::uint32_t>::max())
      throw std::length_error("voxel cache capacity exceeds index range");
  }
  void clear() {
    entries_.clear();
    // Retain bounded storage across cloud/proof changes. A new generation
    // invalidates the index without freeing/reallocating one node per query.
    if (++generation_==0) {
      for(auto& slot:slots_)slot.generation=0;
      generation_=1;
    }
  }
  std::size_t size() const { return entries_.size(); }
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
    return getPoseExact(address,low_z,high_z,x,y,0.,query);
  }
  // A rotated non-circular footprint must never reuse the result of another
  // heading, even when its body origin lies in the same raw-map voxel.
  template <class Query> int getPoseExact(int address, int low_z, int high_z,
      double x, double y, double yaw, Query query) {
    return getPoseShapeExact(address,low_z,high_z,x,y,yaw,0.,0.,query);
  }
  // A source-backed support transition may select different extra clearance
  // at the same voxel/heading. The actual OBB dimensions belong to the key.
  template <class Query> int getPoseShapeExact(int address, int low_z, int high_z,
      double x, double y, double yaw, double length, double width, Query query) {
    std::uint64_t x_bits,y_bits,yaw_bits,length_bits,width_bits;
    std::memcpy(&x_bits,&x,sizeof(x)); std::memcpy(&y_bits,&y,sizeof(y));
    std::memcpy(&yaw_bits,&yaw,sizeof(yaw));
    std::memcpy(&length_bits,&length,sizeof(length));std::memcpy(&width_bits,&width,sizeof(width));
    const Key key{address,low_z,high_z,x_bits,y_bits,yaw_bits,length_bits,width_bits};
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
    std::uint64_t x_bits,y_bits,yaw_bits,length_bits,width_bits;
    bool operator==(const Key &other) const {
      return address==other.address && low_z==other.low_z && high_z==other.high_z
          && x_bits==other.x_bits && y_bits==other.y_bits && yaw_bits==other.yaw_bits
          && length_bits==other.length_bits && width_bits==other.width_bits;
    }
  };
  struct Hash {
    std::size_t operator()(const Key &key) const {
      std::size_t h=std::hash<int>{}(key.address);
      for (int v:{key.low_z,key.high_z}) h^=std::hash<int>{}(v)+0x9e3779b9+(h<<6)+(h>>2);
      for (auto v:{key.x_bits,key.y_bits,key.yaw_bits,key.length_bits,key.width_bits}) h^=std::hash<std::uint64_t>{}(v)+0x9e3779b9+(h<<6)+(h>>2);
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
  std::vector<Entry> entries_;
  std::vector<Slot> slots_;
};

struct VerticalVoxelSpan { int low,high; };
struct SubvoxelCollisionCounts {
  std::size_t qualified_support_contacts{0};
  // A tested subcell is one vertical half of the original voxel. XY remains
  // the whole coarse cell; original 2x2x2 endpoint bits are aggregated by Z.
  std::size_t refined_coarse{0},tested_subcells{0},intersecting_subcells{0},coarse_fallback{0};
};
inline constexpr std::uint16_t kCoarseOnlyHitEvidence=0x100U;
// Use the global lattice, including negative coordinates. If a caller's
// coarse index disagrees at a floating-point boundary, retain the coarse
// cell rather than moving the original endpoint into an invented octant.
inline std::uint16_t hitSubvoxelBit(const Eigen::Vector3d& endpoint,
    const Eigen::Vector3i& coarse,double resolution) {
  if(!endpoint.allFinite()||!std::isfinite(resolution)||resolution<=0.)return 0;
  int code=0;
  for(int axis=0;axis<3;++axis) {
    const double fine=std::floor(endpoint[axis]/(resolution*.5));
    const double offset=fine-2.*static_cast<double>(coarse[axis]);
    if(offset!=0.&&offset!=1.)return 0;
    code=2*code+static_cast<int>(offset);
  }
  return static_cast<std::uint16_t>(1U<<code);
}
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
  // A missing attitude contract is not a measured unknown voxel. Preserve
  // empty counts instead of inventing a raw cell to represent this condition.
  bool attitude_resolved{true};
  // Explicit finite model structure is not a measured voxel. Keep the raw
  // counts/state separate rather than manufacturing an occupied source hit.
  bool modeled_hard_boundary_collision{false};
  // Separate narrowphase report. Raw counts and state() remain raw evidence;
  // absence of a hit bit is never labelled observed free.
  bool subvoxel_narrowphase{false};
  SubvoxelCollisionCounts subvoxels;
  int state() const {
    return !attitude_resolved ? 2 : counts[1] ? 1 : counts[3] ? -1 : counts[2] ? 2 : 0;
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

// Exact 2-D OBB vs voxel-square intersection: the four separating axes are
// the voxel's world X/Y and the body's longitudinal/lateral axes. Looking at
// only voxel centres, or only testing corners contained by the other shape,
// misses thin walls, diagonal contacts and crossing rectangles.
struct OrientedBoxVoxelXYQuery {
  double x,y,resolution,c,s,ac,as,ex,ey,longitudinal,lateral;
  OrientedBoxVoxelXYQuery(const Eigen::Vector3d &position,double yaw,
      double cell_resolution,double length,double width)
      : x(position.x()),y(position.y()),resolution(cell_resolution) {
    if (!position.allFinite() || !std::isfinite(yaw) || !std::isfinite(resolution) ||
        resolution<=0. || !std::isfinite(length) || !std::isfinite(width) || length<=0. || width<=0.)
      throw std::invalid_argument("invalid oriented box voxel geometry");
    c=std::cos(yaw);s=std::sin(yaw);ac=std::abs(c);as=std::abs(s);
    const double hx=length*.5,hy=width*.5,h=resolution*.5;
    ex=hx*ac+hy*as;ey=hx*as+hy*ac;
    longitudinal=hx+h*(ac+as)+1e-12;lateral=hy+h*(ac+as)+1e-12;
  }
  bool intersects(int voxel_x,int voxel_y) const {
    const double dx=(voxel_x+.5)*resolution-x,dy=(voxel_y+.5)*resolution-y;
    return std::abs(dx)<=ex+resolution*.5+1e-12 &&
        std::abs(dy)<=ey+resolution*.5+1e-12 &&
        std::abs(dx*c+dy*s)<=longitudinal &&
        std::abs(-dx*s+dy*c)<=lateral;
  }
};

inline bool orientedBoxIntersectsVoxelXY(const Eigen::Vector3d &position,double yaw,
    const Eigen::Vector3i &voxel,double resolution,double length,double width) {
  return OrientedBoxVoxelXYQuery(position,yaw,resolution,length,width).intersects(voxel.x(),voxel.y());
}

// Visit only intersecting real raw voxels. Contact is retained on every face.
// This is an upright yaw OBB, not a six-DoF articulated robot or self-mask.
// No allocation, yaw bins or occupancy dilation are needed per query.
// The upright OBB has the same exact Z span in every intersecting XY column.
// Precompute the SAT invariants once, keeping the original contact tolerance,
// iteration order and voxel set. Readers can avoid recomputing ring XY indices
// for each Z without adding a second map or a quantized collision model.
template <class Visitor>
bool forEachOrientedBoxColumn(const Eigen::Vector3d &position,double yaw,
    double resolution,const BodyEnvelope &body,Visitor visit) {
  validateBodyEnvelope(body);
  if (body.model!=CollisionModel::OrientedBox || !position.allFinite() || !std::isfinite(yaw))
    throw std::invalid_argument("invalid oriented box query");
  const OrientedBoxVoxelXYQuery xy(position,yaw,resolution,body.length,body.width);
  const auto xs=verticalVoxelSpan(position.x(),xy.ex,xy.ex,resolution);
  const auto ys=verticalVoxelSpan(position.y(),xy.ey,xy.ey,resolution);
  const auto zs=verticalVoxelSpan(position.z(),body.below,body.above,resolution);
  const double cells=(double(xs.high)-xs.low+1.)*(double(ys.high)-ys.low+1.)*
      (double(zs.high)-zs.low+1.);
  if (!std::isfinite(cells) || cells>200000.)
    throw std::invalid_argument("oriented box voxel query exceeds bounded budget");
  for (std::int64_t x=xs.low;x<=xs.high;++x) for (std::int64_t y=ys.low;y<=ys.high;++y) {
    if (xy.intersects(static_cast<int>(x),static_cast<int>(y)) &&
        !visit(static_cast<int>(x),static_cast<int>(y),zs)) return false;
  }
  return true;
}

template <class Visitor>
bool forEachOrientedBoxVoxel(const Eigen::Vector3d &position,double yaw,
    double resolution,const BodyEnvelope &body,Visitor visit) {
  return forEachOrientedBoxColumn(position,yaw,resolution,body,
    [&](int x,int y,const VerticalVoxelSpan &zs) {
      Eigen::Vector3i cell(x,y,zs.low);
      for (std::int64_t z=zs.low;z<=zs.high;++z) {
        cell.z()=static_cast<int>(z);
        if (!visit(cell)) return false;
      }
      return true;
    });
}

template <class RawQuery>
int orientedBoxStatus(const Eigen::Vector3d &position,double yaw,double resolution,
    const BodyEnvelope &body,RawQuery queryRaw) {
  int state=0;
  forEachOrientedBoxVoxel(position,yaw,resolution,body,[&](const Eigen::Vector3i &cell) {
    const int next=queryRaw(cell);
    // Same priority as mergeCollisionStatus below; definite occupancy cannot
    // be concealed by an earlier unknown or outside cell in this footprint.
    state=state==1 || next==1 ? 1 : state<0 || next<0 ? -1 : state!=0 || next!=0 ? 2 : 0;
    return state!=1;
  });
  return state;
}

// Exact 3-D OBB/voxel SAT. Angles use the ROS fixed-axis convention
// R = Rz(yaw) Ry(pitch) Rx(roll); nose-up is a NEGATIVE pitch. The body
// dimensions and reference point do not change on stairs. In particular,
// asymmetric below/above extents shift the OBB centre along rotated body Z,
// not world Z. This is rigid geometry, not a foot-placement/self-filter model.
struct OrientedBoxVoxel3DQuery {
  Eigen::Vector3d center,extent;
  std::array<Eigen::Vector3d,15> axes;
  std::array<double,15> limits{};
  double resolution,body_half_height;
  OrientedBoxVoxel3DQuery(const Eigen::Vector3d &position,double yaw,double pitch,double roll,
      double cell_resolution,const BodyEnvelope &body):resolution(cell_resolution) {
    validateBodyEnvelope(body);
    if(body.model!=CollisionModel::OrientedBox || !position.allFinite() ||
        !std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(roll)||
        !std::isfinite(resolution)||resolution<=0.)
      throw std::invalid_argument("invalid spatial oriented box geometry");
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d half(body.length*.5,body.width*.5,(body.below+body.above)*.5);
    body_half_height=half.z();
    center=position+rotation.col(2)*(body.above-body.below)*.5;
    extent=rotation.cwiseAbs()*half;
    for(int i=0;i<3;++i){axes[i]=Eigen::Vector3d::Unit(i);axes[i+3]=rotation.col(i);}
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)axes[6+3*i+j]=axes[i].cross(rotation.col(j));
    for(std::size_t i=0;i<axes.size();++i)
      limits[i]=(rotation.transpose()*axes[i]).cwiseAbs().dot(half)+
          resolution*.5*axes[i].cwiseAbs().sum()+1e-12;
  }
  bool intersects(const Eigen::Vector3i &cell) const {
    const Eigen::Vector3d delta=(cell.cast<double>().array()+.5).matrix()*resolution-center;
    for(std::size_t i=0;i<axes.size();++i)
      if(std::abs(delta.dot(axes[i]))>limits[i])return false;
    return true;
  }
  // Narrow only vertical quantization: a 5 cm voxel becomes a 5x5x2.5 cm
  // lower/upper half, never a smaller horizontal obstacle. The same exact SAT
  // axes and touching-contact tolerance apply to this axis-aligned cuboid.
  bool intersectsVerticalHalf(const Eigen::Vector3i &cell,bool upper) const {
    Eigen::Vector3d delta=(cell.cast<double>().array()+.5).matrix()*resolution-center;
    delta.z()+=upper?resolution*.25:-resolution*.25;
    for(std::size_t i=0;i<axes.size();++i)
      if(std::abs(delta.dot(axes[i]))>limits[i]-resolution*.25*std::abs(axes[i].z()))return false;
    return true;
  }
  // Same full coarse XY footprint; tighter Z comes from the conservative
  // union of EVERY actual endpoint, not just which 2.5 cm half it occupied.
  bool intersectsVerticalBounds(const Eigen::Vector3i& cell,double lo,double hi) const {
    Eigen::Vector3d delta=(cell.cast<double>().array()+.5).matrix()*resolution-center;
    delta.z()=(lo+hi)*.5-center.z();
    for(std::size_t i=0;i<axes.size();++i)
      if(std::abs(delta.dot(axes[i]))>limits[i]+((hi-lo)*.5-resolution*.5)*std::abs(axes[i].z()))return false;
    return true;
  }
  // ALL original hits below the real rotated body's bottom cannot collide
  // merely because a world-XY voxel column extends across that sloping plane.
  // This is ONLY a separating lower-plane proof: side faces retain their full
  // coarse XY footprint. Missing/mixed/coarse evidence is rejected by caller;
  // invalid bounds, a straddling union and touching contact remain blocking.
  bool allHitBoundsBelowBody(const Eigen::Vector3d& lo,const Eigen::Vector3d& hi) const {
    if(!lo.allFinite()||!hi.allFinite()||(lo.array()>hi.array()).any())return false;
    const Eigen::Vector3d up=axes[5];
    const double highest=((lo+hi)*.5-center).dot(up)+((hi-lo)*.5).dot(up.cwiseAbs());
    return highest < -body_half_height-1e-12;
  }
};

template<class Visitor>
bool forEachOrientedBoxVoxel(const Eigen::Vector3d &position,double yaw,double pitch,double roll,
    double resolution,const BodyEnvelope &body,Visitor visit) {
  // Exact old iteration order, touching-contact tolerance and fast path on
  // level ground. Do not substitute an epsilon or quantized attitude here.
  if(pitch==0. && roll==0.)return forEachOrientedBoxVoxel(position,yaw,resolution,body,visit);
  const OrientedBoxVoxel3DQuery query(position,yaw,pitch,roll,resolution,body);
  const auto xs=verticalVoxelSpan(query.center.x(),query.extent.x(),query.extent.x(),resolution);
  const auto ys=verticalVoxelSpan(query.center.y(),query.extent.y(),query.extent.y(),resolution);
  const auto zs=verticalVoxelSpan(query.center.z(),query.extent.z(),query.extent.z(),resolution);
  const double cells=(double(xs.high)-xs.low+1.)*(double(ys.high)-ys.low+1.)*(double(zs.high)-zs.low+1.);
  if(!std::isfinite(cells)||cells>200000.)
    throw std::invalid_argument("spatial oriented box voxel query exceeds bounded budget");
  for(std::int64_t x=xs.low;x<=xs.high;++x)for(std::int64_t y=ys.low;y<=ys.high;++y)
    for(std::int64_t z=zs.low;z<=zs.high;++z) {
      const Eigen::Vector3i cell(static_cast<int>(x),static_cast<int>(y),static_cast<int>(z));
      if(query.intersects(cell)&&!visit(cell))return false;
    }
  return true;
}

template<class RawQuery>
int orientedBoxStatus(const Eigen::Vector3d &position,double yaw,double pitch,double roll,
    double resolution,const BodyEnvelope &body,RawQuery queryRaw) {
  int state=0;
  forEachOrientedBoxVoxel(position,yaw,pitch,roll,resolution,body,[&](const Eigen::Vector3i &cell) {
    const int next=queryRaw(cell);
    state=state==1||next==1?1:state<0||next<0?-1:state!=0||next!=0?2:0;
    return state!=1;
  });
  return state;
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

// A blocking classification must not depend on traversal order. Keep the same
// rejection set, but a confirmed obstacle outranks missing evidence; outside
// outranks unknown just as CollisionEvidence::state(). Unexpected nonzero
// values remain uncertain/blocked rather than being accidentally made free.
inline int mergeCollisionStatus(int first,int second) {
  if(first==1 || second==1)return 1;
  if(first<0 || second<0)return -1;
  return first!=0 || second!=0 ? 2:0;
}

// Reverse the obstacle-inflation offsets to visit all raw cells that could
// intersect a cylinder whose actual center lies anywhere in center_cell.
// queryRaw: 0 measured free, 1 occupied, 2 unobserved, -1 outside map.
// Work stays bounded by the existing finite kernel, without adding another
// region or a fresh query deadline. Occupied is the definitive early exit;
// unknown/outside must not conceal an occupied voxel later in this SAME set.
template <class RawQuery>
int observedCylinderStatus(const Eigen::Vector3i &center_cell,
    const std::vector<Eigen::Vector3i> &inflation_offsets, RawQuery queryRaw) {
  int result=0;
  for (const auto &offset:inflation_offsets) {
    const Eigen::Vector3i body_cell=center_cell-offset;
    result=mergeCollisionStatus(result,queryRaw(body_cell));
    if (result==1) return result;
  }
  return result;
}

template <class RawQuery>
int observedCylinderStatusAtHeight(const Eigen::Vector3i &center_cell,
    const VerticalVoxelSpan &span, const std::vector<Eigen::Vector3i> &inflation_offsets,
    RawQuery queryRaw) {
  int result=0;
  for (const auto &offset:inflation_offsets) {
    const Eigen::Vector3i body_cell=center_cell-offset;
    if (body_cell.z()<span.low || body_cell.z()>span.high) continue;
    result=mergeCollisionStatus(result,queryRaw(body_cell));
    if (result==1) return result;
  }
  return result;
}

template <class RawQuery>
int observedCylinderStatusAtPosition(const Eigen::Vector3d &center,
    const Eigen::Vector3i &center_cell, const VerticalVoxelSpan &span,
    const std::vector<Eigen::Vector3i> &inflation_offsets,
    double resolution, double radius, RawQuery queryRaw) {
  int result=0;
  for (const auto &offset:inflation_offsets) {
    const Eigen::Vector3i cell=center_cell-offset;
    if (cell.z()<span.low || cell.z()>span.high ||
        !cylinderIntersectsVoxelXY(center,cell,resolution,radius)) continue;
    result=mergeCollisionStatus(result,queryRaw(cell));
    if (result==1) return result;
  }
  return result;
}

}  // namespace scan_planner
