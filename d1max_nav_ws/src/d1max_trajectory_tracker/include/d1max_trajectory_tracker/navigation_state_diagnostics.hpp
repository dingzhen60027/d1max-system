#pragma once

#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include <cstdint>
#include <string>

namespace d1max_trajectory_tracker {

// Passive, bounded evidence. This record never admits a state, advances an
// ingress watermark, renews a source lease, or changes recovery decisions.
// Keep the latest delivered event AND the last accepted measurement because
// a provider's unavailable event intentionally contains no pose/IMU evidence.
struct NavigationSourceRecord {
  bool present{false}, usable{false}, decoded{false}, accepted{false};
  std::uint32_t schema{0};
  std::uint64_t epoch{0};
  std::int64_t received_ns{0}, source_ns{0}, posterior_ns{0}, imu_ns{0}, valid_until_ns{0};
  double extrapolation_sec{0.};
  std::string seed, provider_reason, receiver_reason;
};

class NavigationStateDiagnostics {
 public:
  void record(const d1max_planning_interfaces::msg::LocalNavigationState& state,
      std::int64_t received_ns, bool decoded, bool accepted, const std::string& receiver_reason) {
    latest_ = {true, state.usable, decoded, accepted, state.schema_version,
      state.localization_epoch, received_ns, stamp(state.source_stamp),
      stamp(state.posterior_stamp), stamp(state.imu_stamp), stamp(state.valid_until),
      state.extrapolation_sec, bounded(state.localization_seed_id),
      bounded(state.reason), bounded(receiver_reason)};
    if (accepted) accepted_ = latest_;
  }
  const NavigationSourceRecord& latest() const { return latest_; }
  const NavigationSourceRecord& lastAccepted() const { return accepted_; }

 private:
  static std::int64_t stamp(const builtin_interfaces::msg::Time& t) {
    return t.sec < 0 || t.nanosec >= 1000000000U ? -1 :
      std::int64_t(t.sec) * 1000000000LL + t.nanosec;
  }
  static std::string bounded(const std::string& text) {
    std::string out = text.substr(0, 160);
    for (char& c : out) if (static_cast<unsigned char>(c) < 32U) c = ' ';
    return out;
  }
  NavigationSourceRecord latest_, accepted_;
};

}  // namespace d1max_trajectory_tracker
