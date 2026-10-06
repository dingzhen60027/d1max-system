#pragma once

#include <array>
#include <cstdint>
#include <memory>
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
  };
  static std::shared_ptr<const StaticOccupancyPrior> load(
      const std::string &manifest_path,const Expected &expected);
  static std::string sha256(const void *data,std::size_t size);
  int state(const std::array<int,3> &world_index) const;
  bool matchesContext(const nlohmann::json &context) const;
  const std::string &manifestSha256() const {return expected_.manifest_sha256;}
  const std::string &geometrySha256() const {return expected_.geometry_sha256;}
  const std::string &mapVersion() const {return expected_.map_version;}
  std::size_t bytes() const {return data_.size();}
 private:
  Expected expected_;
  std::array<int,3> origin_{};
  std::array<std::size_t,3> shape_{};
  std::vector<std::uint8_t> data_;
};

} // namespace scan_planner
