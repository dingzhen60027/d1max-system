#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>

namespace d1monitor {
// SDK callbacks cannot take the state/dispatch mutex: the vendor may invoke a
// callback inline or wait for its callback thread while submitting a command.
// These bounded, sticky bits cannot be overwritten by newer healthy telemetry.
class NavigationEvents {
 public:
  enum Event : uint32_t {
    LostControl=1u<<0, RobotFault=1u<<1, MoveWriteFailure=1u<<2,
    UnsafeRobotState=1u<<3, EmergencyStop=1u<<4, ConnectionError=1u<<5,
    WorkerStopped=1u<<6, McDisabled=1u<<7
  };
  void raise(Event event) noexcept { pending_.fetch_or(event,std::memory_order_release); }
  uint32_t take() noexcept { return pending_.exchange(0,std::memory_order_acq_rel); }
 private:
  std::atomic<uint32_t> pending_{0};
};

// Authorization selection and nonblocking SDK submission have ONE ordering
// with all worker/operator state mutations. Do not return/cache the selected
// command outside this lock. Callbacks may only enqueue events, never lock this
// mutex. The select function drains callback events before checking health;
// the submit function drains them again after submission (inline callbacks).
// A command already submitted to the vendor cannot be recalled. An event that
// arrives during submission is ordered after that submission and revokes the
// next one; this is not a physical stop or vendor queue-cancellation guarantee.
template<class Select,class Submit>
void dispatch_navigation(std::mutex& mutex,Select&& select,Submit&& submit) {
  std::lock_guard<std::mutex> lock(mutex);
  const auto command=select();
  if(command)submit(*command);
}
}
