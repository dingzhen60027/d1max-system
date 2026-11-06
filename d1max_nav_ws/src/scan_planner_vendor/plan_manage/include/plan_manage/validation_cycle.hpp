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
  static constexpr auto slow_curve_budget=std::chrono::milliseconds(120);
  static constexpr double slow_curve_budget_s=std::chrono::duration<double>(slow_curve_budget).count();
  // Keep the historical short-reader capability separate from the formal
  // follower's complete-current reader below. The legacy 50 ms round and its
  // command scheduling remain unchanged.
  static constexpr auto local_curve_budget=period;
  static constexpr double local_curve_budget_s=std::chrono::duration<double>(local_curve_budget).count();
  static double activeBudget(Clock::time_point begin,Clock::time_point now) {
    return std::max(0.,std::chrono::duration<double>(begin+period-now).count());
  }
  static double admissionBudget(Clock::time_point begin,Clock::time_point now) {
    return std::max(0.,std::chrono::duration<double>(begin+slow_curve_budget-now).count());
  }
  template<class Active,class Now>
  static bool currentCurvePass(Clock::time_point begin,Active active,Now now) {
    const double remaining=activeBudget(begin,now());
    return remaining>0.&&active(remaining,begin+period);
  }
  // The formal follower has no motion sweep on this reader. Give its CURRENT
  // full-curve check the same complete budget formerly supplied by the slow
  // fallback, independently of candidate admission. The scheduling target is
  // still 50 ms; a 100 ms check completes at 10 Hz, not a claimed 20 Hz. This
  // wall budget never extends the original 250 ms/body/ray proof deadlines.
  static constexpr auto formal_curve_budget=slow_curve_budget;
  template<class Active,class Now>
  static bool formalCurrentCurvePass(Clock::time_point begin,Active active,Now now) {
    const auto deadline=begin+formal_curve_budget;
    const double remaining=std::max(0.,std::chrono::duration<double>(deadline-now()).count());
    return remaining>0.&&active(remaining,deadline);
  }
  static bool admissionWorkWanted(bool formal_follower,bool has_active,bool has_distinct_pending) {
    return has_distinct_pending||(!formal_follower&&has_active);
  }
  template<class Pending,class Now>
  static void formalAdmissionPass(Clock::time_point begin,Pending pending,Now now) {
    const double remaining=admissionBudget(begin,now());
    if(remaining>0.)pending(remaining,begin+slow_curve_budget);
  }
  // Historical non-follower transport retains its original shared slow lane.
  template<class Active,class Pending,class Now>
  static void admissionPass(Clock::time_point begin,Active active,Pending pending,Now now) {
    const auto deadline=begin+slow_curve_budget;
    double remaining=admissionBudget(begin,now());
    if(remaining>0.)active(remaining,deadline);
    remaining=admissionBudget(begin,now());
    if(remaining>0.)pending(remaining,deadline);
  }
  static bool inconclusiveRefresh(bool refreshing_current,const std::string& reason) {
    return refreshing_current&&reason=="curve_check_budget_exhausted";
  }
  // Typed resource incompletion is valid for either an incumbent refresh or
  // an optional candidate. Neither produces a new proof; only the incumbent
  // may still have an ORIGINAL, independently expiring lease to consume.
  static bool incompleteProof(bool budget_exhausted,bool sources_current) {
    return budget_exhausted&&sources_current;
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
    const double remaining=activeBudget(begin,now());
    if(remaining>0.)prepared(std::min(prepared_motion_reserve_s,remaining));
  }
};
} // namespace scan_planner
