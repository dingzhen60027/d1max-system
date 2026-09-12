#pragma once
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>

namespace faster_lio {
// Budget uses monotonic time. Only missing IMU coverage is recoverable;
// clock resets, invalid filter state and geometric failures remain latched.
class ImuRecoveryBudget {
 public:
  void configure(bool enabled, int maximum, double window) {
    if (maximum < 1 || maximum > 10 || !std::isfinite(window) || window < 10. || window > 600.)
      throw std::invalid_argument("Invalid bounded IMU recovery settings");
    enabled_ = enabled; maximum_ = maximum; window_ = window;
  }
  static bool recoverable(const std::string& reason) {
    return reason == "imu_gap" || reason == "imu_start_uncovered";
  }
  bool enabled() const { return enabled_; }
  bool allow(const std::string& reason, double now) {
    if (!enabled_ || !recoverable(reason) || !std::isfinite(now)) return false;
    if (!attempts_.empty() && now < attempts_.back()) return false;
    while (!attempts_.empty() && now - attempts_.front() >= window_) attempts_.pop_front();
    if (attempts_.size() >= static_cast<size_t>(maximum_)) return false;
    attempts_.push_back(now); return true;
  }
 private:
  bool enabled_{false}; int maximum_{3}; double window_{60.};
  std::deque<double> attempts_;
};
}  // namespace faster_lio
