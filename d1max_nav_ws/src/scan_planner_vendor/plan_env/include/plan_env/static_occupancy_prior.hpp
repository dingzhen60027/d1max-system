#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace scan_planner {

// A certified complete static volume is a separate evidence source. It never
// supplies an acquisition stamp, changes live odds, or renews a sensor lease.
// All fields are immutable after the checked loader has returned.
class StaticOccupancyPrior final {
 public:
  struct Expected {
    std::string manifest_sha256, geometry_sha256, map_version, frame_id, odom_frame_id;
    std::string transform_contract{"fixed_identity_map_from_odom_v1"};
    double resolution{0.};
    // Simulation-only flat support contact evidence. State 3 is never FREE and
    // is rejected by the default loader, including certified production maps.
    bool allow_floor_contact{false};
    double support_floor_z{0.}, support_penetration_m{0.};
    // The wider bound is accepted only by an explicitly matching isolated
    // floor-contact certificate. It classifies unchanged native endpoints;
    // it is never an odds, geometry, time or FREE-space modification.
    double support_endpoint_error_bound_m{1e-5};
    std::string dynamic_registry_sha256;
  };
  static std::shared_ptr<const StaticOccupancyPrior> load(
      const std::string &manifest_path,const Expected &expected);
  static std::string sha256(const void *data,std::size_t size);
  int state(const std::array<int,3> &world_index) const;
  // Immutable contiguous Z bytes, only when the entire requested column is
  // inside this certified volume. A null view requires ordinary state queries.
  const std::uint8_t* columnStates(const std::array<int,3>& first,int high_z) const;
  bool matchesContext(const nlohmann::json &context) const;
  const std::string &manifestSha256() const {return expected_.manifest_sha256;}
  const std::string &geometrySha256() const {return expected_.geometry_sha256;}
  const std::string &mapVersion() const {return expected_.map_version;}
  std::size_t bytes() const {return data_.size();}
  bool floorContactEnabled() const {return expected_.allow_floor_contact;}
  double floorZ() const {return expected_.support_floor_z;}
  double floorPenetrationM() const {return expected_.support_penetration_m;}
  double floorEndpointErrorBoundM() const {return expected_.support_endpoint_error_bound_m;}
  bool dynamicRegistryMatches(const std::string &digest,const std::vector<std::string> &actors) const {
    const std::set<std::string> unique(actors.begin(),actors.end());
    return !digest.empty()&&digest==dynamic_registry_sha256_&&unique.size()==actors.size()&&unique==dynamic_actor_ids_;
  }
 private:
  Expected expected_;
  std::array<int,3> origin_{};
  std::array<std::size_t,3> shape_{};
  std::vector<std::uint8_t> data_;
  std::string dynamic_registry_sha256_;
  std::set<std::string> dynamic_actor_ids_;
};

} // namespace scan_planner
