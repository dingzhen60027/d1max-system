#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

namespace scan_planner {
// No slow admission belongs on the command-validation lane. These are wall
// budgets, not sensor leases; they never renew a source stamp or make unknown
// cells free. A full proof that overruns its lane simply cannot be renewed.
struct ValidationCycle {
  using Clock=std::chrono::steady_clock;
  static constexpr auto period=std::chrono::milliseconds(50);
  static constexpr double motion_budget_s=.025;
  static constexpr double renewal_motion_reserve_s=.005;
  static constexpr double prepared_motion_reserve_s=.005;
  static constexpr double slow_curve_budget_s=.120;
  static double activeBudget(Clock::time_point begin,Clock::time_point now) {
    return std::max(0.,std::chrono::duration<double>(begin+period-now).count());
  }
  static bool inconclusiveRefresh(bool refreshing_current,const std::string& reason) {
    return refreshing_current&&reason=="curve_check_budget_exhausted";
  }
  static bool originalLeaseUsable(bool valid,std::int64_t original_deadline,std::int64_t now) {
    return valid&&original_deadline>now;
  }
  template<class Motion,class Active>
  static void priorityPass(Clock::time_point begin,Motion motion,Active active) {
    motion(); // newest actual command first, never behind candidate admission
    const double remaining=activeBudget(begin,Clock::now());
    if(remaining>0.)active(remaining);
  }
  template<class Motion,class Active,class Now>
  static unsigned priorityPassWithRenewal(Clock::time_point begin,Motion motion,Active active,Now now) {
    // First actual sweep remains ahead of the full curve: occupied must stop
    // immediately. Reserve only part of this SAME 50 ms round for a second
    // sweep after an actually published positive incumbent renewal. It is not
    // a second 50 ms budget, sensor lease or timer-rate increase.
    motion(std::min(motion_budget_s,activeBudget(begin,now())));
    const double remaining=activeBudget(begin,now());
    if(remaining<=renewal_motion_reserve_s) return 1;
    const bool renewed=active(remaining-renewal_motion_reserve_s);
    const double after=activeBudget(begin,now());
    if(!renewed||after<=0.) return 1;
    motion(std::min(motion_budget_s,after)); // <= one extra, re-read latest fenced demand.
    return 2;
  }
  template<class Motion,class Active,class Prepared,class Now>
  static void priorityPassWithPrepared(Clock::time_point begin,Motion motion,Active active,Prepared prepared,Now now) {
    priorityPassWithRenewal(begin,motion,[&](double budget) {
      const double incumbent=std::max(0.,budget-prepared_motion_reserve_s);
      return incumbent>0.&&active(incumbent);
    },now);
    // A prepared command never blocks the incumbent's actual collision check
    // or same-curve renewal/reproof. This is part of the original 50 ms round,
    // not a fresh admission budget or another reader of the mutable map.
    // The 5 ms reserve above is a reservation, not a cap: unused time from
    // this same round can complete a cold prepared sweep, bounded by the
    // existing actual-motion cap and the original round deadline.
    const double remaining=activeBudget(begin,now());
    if(remaining>0.)prepared(std::min(motion_budget_s,remaining));
  }
};
} // namespace scan_planner
