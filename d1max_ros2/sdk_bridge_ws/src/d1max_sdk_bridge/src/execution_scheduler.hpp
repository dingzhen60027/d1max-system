#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace d1monitor {
// The notifier and the waiter must serialize revision publication with the
// same wait mutex. Atomic+notify alone can lose a wake between the predicate
// check and entering wait. The writer never holds this mutex while querying
// state, calling a vendor or publishing telemetry.
inline void notifyWriterRevision(std::mutex&mutex,std::condition_variable&wake,std::atomic<uint64_t>&revision){
  {std::lock_guard<std::mutex>lock(mutex);revision.fetch_add(1,std::memory_order_release);}
  wake.notify_one();
}
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

// Normal writer opportunities keep the steady phase while also enforcing a
// minimum interval between actual admissions. A delayed OS wake must not
// produce a compressed catch-up pair (e.g. 60 ms, then 100 ms). Urgent zero
// wakeups are independent and never become nonzero admission opportunities.
class WriterDeadline {
 public:
  WriterDeadline(double start,double period):phase_(start,period),period_(period),normal_after_(start){}
  double next()const{return std::max(phase_.next(),normal_after_);}
  uint64_t skipped()const{return phase_.skipped();}
  void complete(double started,double completed){
    if(!std::isfinite(started)||!std::isfinite(completed)||started<next()||completed<started)
      throw std::invalid_argument("writer_admission_clock_regressed");
    normal_after_=started+period_;phase_.complete(completed);
  }
 private:PeriodicDeadline phase_;double period_,normal_after_;
};

// The first real command establishes the writer's absolute steady phase.
// Later opportunities advance from that phase, NOT from actual admission:
// adding 50 ms to each delayed wake accumulates normal scheduling jitter into
// a full producer period of latency. Fresh input still uses an idle slot at
// once, and only the newest accepted original command can be selected.
// A very late send skips a following slot less than half a period away, so
// overdue work cannot become a compressed catch-up burst. Normal small jitter
// can produce e.g. 49.7/50.3 ms intervals without an accumulating phase shift.
// Empty/expired/HOLD polls do not spend the next ordinary opportunity. The
// periodic health watchdog and original source deadline stay independent, so
// an event-driven sender cannot retain stale velocity while waiting for input.
// Only the unique writer thread owns this scheduler; callbacks publish a
// monotonically increasing wake revision rather than a command queue.
class CommandWriterDeadline {
 public:
  CommandWriterDeadline(double start,double period):watchdog_(start,period),period_(period),normal_after_(start){}
  void observe(uint64_t revision){if(revision>observed_)observed_=revision;}
  double next(std::optional<double> original_deadline={})const{
    auto due=watchdog_.next();
    if(observed_>consumed_)due=std::min(due,normal_after_);
    if(original_deadline&&std::isfinite(*original_deadline))due=std::min(due,*original_deadline);
    return due;
  }
  bool normalDue(double now)const{return std::isfinite(now)&&now>=normal_after_;}
  bool pending()const{return observed_>consumed_;}
  uint64_t observed()const{return observed_;}
  uint64_t skipped()const{return watchdog_.skipped()+normal_skipped_;}
  void complete(double started,double completed,uint64_t selected_revision,bool normal_opportunity,
      std::optional<double> nonzero_submitted_at={}){
    if(!std::isfinite(started)||!std::isfinite(completed)||completed<started||
       (nonzero_submitted_at&&(!std::isfinite(*nonzero_submitted_at)||!normal_opportunity||
         *nonzero_submitted_at<normal_after_||*nonzero_submitted_at<started||*nonzero_submitted_at>completed)))
      throw std::invalid_argument("writer_command_admission_clock_regressed");
    if(normal_opportunity)consumed_=std::max(consumed_,selected_revision);
    if(nonzero_submitted_at){
      if(!phase_started_){normal_after_=*nonzero_submitted_at+period_;phase_started_=true;}
      else {
        normal_after_+=period_;
        // Keep the original phase. Skip, rather than send, a slot too close
        // to the preceding actual call after a major OS/vendor delay.
        const auto earliest=*nonzero_submitted_at+period_*.5;
        if(normal_after_<earliest){
          const auto count=static_cast<uint64_t>(std::ceil((earliest-normal_after_)/period_));
          normal_after_+=count*period_;normal_skipped_+=count;
        }
      }
      // Vendor blocking must not turn completed slots into replay work.
      if(normal_after_<=completed){
        const auto count=static_cast<uint64_t>(std::floor((completed-normal_after_)/period_))+1;
        normal_after_+=count*period_;normal_skipped_+=count;
      }
    }
    if(completed>=watchdog_.next())watchdog_.complete(completed);
  }
 private:
  PeriodicDeadline watchdog_;double period_,normal_after_;uint64_t observed_=0,consumed_=0,normal_skipped_=0;
  bool phase_started_=false;
};
}
