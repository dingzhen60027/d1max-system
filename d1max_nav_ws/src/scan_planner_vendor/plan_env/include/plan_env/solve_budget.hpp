#pragma once

#include <atomic>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>

namespace scan_planner {

// One absolute deadline shared by search, optimization, retiming and validation.
// Cancellation may be requested by a separate ingress owner; all geometry/map
// access remains owned by the solving thread. A budget never authorizes motion.
class SolveBudget {
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Now = std::function<Time()>;
  using Ptr = std::shared_ptr<SolveBudget>;
  enum class Stop { None, Cancelled, Deadline };

  explicit SolveBudget(std::chrono::milliseconds duration = std::chrono::milliseconds(400),
                       Now now = [] { return Clock::now(); },Ptr parent = {})
      : now_(std::move(now)), deadline_(parent?std::min(now_()+duration,parent->deadline()):now_()+duration),
        parent_(std::move(parent)) {
    if (duration.count() <= 0 || duration.count() > 400)
      throw std::invalid_argument("native solve budget must be in (0,400] ms");
  }
  void cancel() noexcept { cancelled_.store(true, std::memory_order_release); }
  Stop stop() const {
    if(parent_) {const auto parent_stop=parent_->stop();if(parent_stop!=Stop::None)return parent_stop;}
    if (cancelled_.load(std::memory_order_acquire)) return Stop::Cancelled;
    return now_() >= deadline_ ? Stop::Deadline : Stop::None;
  }
  bool allowed() const { return stop() == Stop::None; }
  Time deadline() const { return deadline_; }
  double remainingSeconds() const {
    if (!allowed()) return 0.;
    return std::chrono::duration<double>(deadline_ - now_()).count();
  }
  const char *reason() const {
    switch (stop()) {
      case Stop::Cancelled: return "solve_cancelled";
      case Stop::Deadline: return "solve_deadline_exceeded";
      default: return "";
    }
  }
private:
  Now now_;
  const Time deadline_;
  const Ptr parent_;
  std::atomic<bool> cancelled_{false};
};

inline bool solveAllowed(const SolveBudget::Ptr &budget) {
  return !budget || budget->allowed();
}
}  // namespace scan_planner
