#pragma once
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>

namespace d1monitor {
// One pending sample: a blocked diagnostic consumer cannot accumulate control
// work. Externally locked; source timestamps stay on the copied sample.
template<class T> struct LatestMailbox {
  std::optional<T> pending;
  uint64_t overwritten=0;
  void put(T value){if(pending)++overwritten;pending=std::move(value);}
  std::optional<T> take(){auto value=std::move(pending);pending.reset();return value;}
};
// Steady-clock phase scheduling, not a ROS callback queue. Missed periods are
// skipped, never replayed as a burst of stale Move calls. This is soft-real-time
// isolation only: vendor calls, whole-process death and OS scheduling still
// require independent hardware acceptance and cannot be bounded by this class.
class PeriodicDeadline {
 public:
  PeriodicDeadline(double start,double period):next_(start),period_(period){
    if(!std::isfinite(start)||!std::isfinite(period)||period<=0)throw std::invalid_argument("invalid_writer_period");}
  double next()const{return next_;}
  uint64_t skipped()const{return skipped_;}
  void complete(double now){
    if(!std::isfinite(now)||now<next_)throw std::invalid_argument("writer_steady_clock_regressed");
    next_+=period_;
    if(next_<=now){const auto count=static_cast<uint64_t>(std::floor((now-next_)/period_))+1;skipped_+=count;next_+=count*period_;}
  }
 private:double next_,period_;uint64_t skipped_=0;
};
}
