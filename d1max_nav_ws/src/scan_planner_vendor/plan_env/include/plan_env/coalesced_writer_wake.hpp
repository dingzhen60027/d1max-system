#pragma once

#include <rclcpp/guard_condition.hpp>
#include <rclcpp/waitable.hpp>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

namespace scan_planner {

// One readiness bit, not a work queue or a second map writer. Register this
// waitable in the existing mutually-exclusive writer callback group. Ingress
// may notify from another executor; only execute() invokes the writer callback.
class CoalescedWriterWake final : public rclcpp::Waitable {
public:
  using SharedPtr=std::shared_ptr<CoalescedWriterWake>;
  CoalescedWriterWake(rclcpp::Context::SharedPtr context,std::function<void()> callback)
      :guard_(std::move(context)),callback_(std::move(callback)),token_(std::make_shared<unsigned char>(0)) {
    if(!callback_)throw std::invalid_argument("missing writer wake callback");
  }
  ~CoalescedWriterWake() override {close();}

  bool notify() noexcept {
    if(closed_.load(std::memory_order_acquire))return false;
    if(!pending_.exchange(true,std::memory_order_acq_rel)) {
      try {guard_.trigger();}
      catch(...) {
        // Keep readiness set. Another writer wake/watchdog can still observe
        // it; a transport shutdown must not throw from an ingress callback.
        failures_.fetch_add(1,std::memory_order_relaxed);return false;
      }
    }
    return true;
  }
  void close() noexcept {closed_.store(true,std::memory_order_release);}
  std::uint64_t failureCount() const noexcept {return failures_.load(std::memory_order_relaxed);}
  size_t get_number_of_ready_guard_conditions() override {return 1;}
  void add_to_wait_set(rcl_wait_set_t* set) override {guard_.add_to_wait_set(set);}
  bool is_ready(rcl_wait_set_t*) override {
    return !closed_.load(std::memory_order_acquire)&&pending_.load(std::memory_order_acquire);
  }
  std::shared_ptr<void> take_data() override {
    // Consume BEFORE executing. A notification during a busy writer sets a
    // distinct pending turn, instead of being erased on callback completion.
    if(closed_.load(std::memory_order_acquire)||!pending_.exchange(false,std::memory_order_acq_rel))return {};
    return token_;
  }
  void execute(std::shared_ptr<void>& data) override {
    if(data&&!closed_.load(std::memory_order_acquire))callback_();
  }
private:
  rclcpp::GuardCondition guard_;
  const std::function<void()> callback_;
  const std::shared_ptr<void> token_;
  std::atomic<bool> pending_{false},closed_{false};
  std::atomic<std::uint64_t> failures_{0};
};

} // namespace scan_planner
