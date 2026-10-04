#pragma once

// Read-only, opt-in witness storage. No evidence here is an input to collision
// classification, occupancy voting, sensor leases or motion admission.
#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <map>
#include <limits>
#include <optional>
#include <stdexcept>

namespace scan_planner {
struct RayDiagnosticMetadata {
  std::uint16_t sensor_id{0}, ring{0};
  std::uint32_t source_index{0}, offset_time_ns{0};
  std::int64_t scan_stamp_ns{0}, received_ns{0}, integration_ns{0}, alignment_stamp_ns{0}, acquisition_end_ns{0};
  std::uint64_t context_sequence{0}, projection_sequence{0};
  double timestamp{0}, source_timestamp{0}, raw_timestamp{0};
  bool source_fields_available{false};
};
struct RayWitness {
  RayDiagnosticMetadata metadata;
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()}, endpoint{Eigen::Vector3d::Zero()},
      integrated_endpoint{Eigen::Vector3d::Zero()};
  bool hit{false}, contributed_vote{false};
};
struct VoxelWitness {
  // One last hit and miss per sensor; counts are ROI/budget-limited ray visits,
  // NOT log-odds updates and NOT a complete historical archive.
  std::array<std::optional<RayWitness>,4> last;
  std::array<std::uint64_t,4> visits{{0,0,0,0}}, votes{{0,0,0,0}};
};
class NearFieldDiagnostics {
 public:
  using Key=std::array<int,3>;
  bool enabled() const {return enabled_;}
  void configure(bool enabled,const Eigen::Vector3d &center,const Eigen::Vector3d &half,
      double resolution,std::size_t cells=16384,std::size_t events=200000) {
    if (!center.allFinite() || !half.allFinite() || (half.array()<=0.).any() ||
        half.maxCoeff()>5. || !std::isfinite(resolution) || resolution<=0. ||
        cells==0 || cells>32768 || events==0 || events>1000000)
      throw std::invalid_argument("invalid bounded diagnostic ROI");
    const Eigen::Vector3d low=(center-half)/resolution,high=(center+half)/resolution;
    if(!low.allFinite() || !high.allFinite() || low.minCoeff()<std::numeric_limits<int>::min()+1. ||
        high.maxCoeff()>std::numeric_limits<int>::max()-1.)
      throw std::invalid_argument("diagnostic ROI exceeds global index range");
    enabled_=enabled;center_=center;half_=half;resolution_=resolution;
    low_=((center-half)/resolution).array().floor().cast<int>();
    high_=((center+half)/resolution).array().floor().cast<int>();
    max_cells_=cells;max_events_=events;clear();
  }
  void clear() {entries_.clear();events_=dropped_events_=dropped_cells_=0;++generation_;}
  void beginIntegration() {events_=0;}
  void erase(const Eigen::Vector3i &cell) {entries_.erase(key(cell));}
  void record(const Eigen::Vector3i &cell,const RayWitness &witness) {
    if (!enabled_ || (cell.array()<low_.array()).any() || (cell.array()>high_.array()).any()) return;
    if(witness.metadata.sensor_id>1) return;
    if (events_>=max_events_) {++dropped_events_;return;}
    ++events_;
    const auto k=key(cell);
    auto it=entries_.find(k);
    if (it==entries_.end()) {
      if(entries_.size()>=max_cells_) {++dropped_cells_;return;}
      it=entries_.emplace(k,VoxelWitness{}).first;
    }
    const auto sensor=witness.metadata.sensor_id;
    const std::size_t slot=2*sensor+(witness.hit?0:1);
    it->second.last[slot]=witness;
    ++it->second.visits[slot];
    if(witness.contributed_vote) ++it->second.votes[slot];
  }
  const VoxelWitness *find(const Eigen::Vector3i &cell) const {
    const auto it=entries_.find(key(cell));return it==entries_.end()?nullptr:&it->second;
  }
  bool contains(const Eigen::Vector3i &cell) const {
    return enabled_ && !(cell.array()<low_.array()).any() && !(cell.array()>high_.array()).any();
  }
  std::size_t size() const {return entries_.size();}
  std::size_t droppedEvents() const {return dropped_events_;}
  std::size_t droppedCells() const {return dropped_cells_;}
  std::size_t capacity() const {return max_cells_;}
  std::size_t eventLimit() const {return max_events_;}
  std::uint64_t generation() const {return generation_;}
  const Eigen::Vector3d &center() const {return center_;}
  const Eigen::Vector3d &halfExtent() const {return half_;}
 private:
  static Key key(const Eigen::Vector3i &cell) {return {cell.x(),cell.y(),cell.z()};}
  bool enabled_{false};
  double resolution_{1.};
  Eigen::Vector3d center_{Eigen::Vector3d::Zero()},half_{Eigen::Vector3d::Ones()};
  Eigen::Vector3i low_{Eigen::Vector3i::Zero()},high_{Eigen::Vector3i::Zero()};
  std::size_t max_cells_{16384},max_events_{200000},events_{0},dropped_events_{0},dropped_cells_{0};
  std::uint64_t generation_{0};
  std::map<Key,VoxelWitness> entries_;
};
} // namespace scan_planner
