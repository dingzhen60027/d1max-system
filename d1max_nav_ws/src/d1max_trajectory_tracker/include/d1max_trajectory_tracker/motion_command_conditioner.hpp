#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "d1max_trajectory_tracker/response_command_limiter.hpp"

namespace d1max_trajectory_tracker {

enum MotionConstraintOverride : std::uint32_t {
  MotionOverrideNone = 0,
  MotionOverrideAcceleration = 1u << 0,
  MotionOverrideJerk = 1u << 1,
  MotionOverrideMappedRate = 1u << 2,
  MotionOverrideMappedAcceleration = 1u << 3,
  MotionOverrideFeasibility = 1u << 4
};

struct MotionCommandState {
  double command = 0.;
  double acceleration = 0.;
  double mapped_rate = 0.;
};

struct MotionCommandStep {
  double value = 0.;
  double acceleration = 0.;
  double mapped_fraction = 0.;
  double mapped_rate = 0.;
  bool limited = false;
  bool priority_override = false;
  std::uint32_t overridden_constraints = MotionOverrideNone;
};

// One scalar command conditioner.  Its three numbers describe commanded
// velocity and the discrete changes of that command and its SDK image.  They
// are not measured robot acceleration, an inertia model, or an SDK timer.
//
// All normal constraints are intersected BEFORE selecting an output.  A short
// command-space braking construction reserves enough room to reduce both
// carried derivatives to zero.  In particular, it cannot charge into the V4
// high-gain transition and then silently discard the SI jerk bound next tick.
// The construction never publishes future points or retains a pending command.
class MotionCommandConditioner {
 public:
  const MotionCommandState& state() const { return state_; }
  // Supply an actually applied command only when the enclosing writer has
  // changed it (or an explicit stop has taken effect).  This resets command
  // derivative history; it makes no claim about measured robot acceleration.
  void reset(double applied_command = 0.) {
    if (!std::isfinite(applied_command))
      throw std::invalid_argument("motion_conditioner_reset_nonfinite");
    state_ = {applied_command, 0., 0.};last_dt_=0.;
  }

  // Re-express the SAME forward command history in a newly acquired body
  // direction. This is not a measured-velocity correction: callers must first
  // establish that measurement fully supports the projected old command.
  // Transform both endpoints of the last actual discrete command interval so
  // the static SDK-image derivative stays consistent with its SI history.
  void projectForwardHistory(double factor,const CommandResponseCurve& curve) {
    curve.validate();
    if(!std::isfinite(factor)||factor<0.||factor>1.||state_.command<0.)
      throw std::invalid_argument("motion_conditioner_projection_invalid");
    const double previous=state_.command-state_.acceleration*last_dt_;
    state_.command*=factor;state_.acceleration*=factor;
    state_.mapped_rate=last_dt_>0.?
      (curve.map(state_.command)-curve.map(previous*factor))/last_dt_:0.;
  }

  MotionCommandStep step(double target_si, double lower_si, double upper_si,
      double max_si_acceleration, double max_si_jerk,
      const CommandResponseCurve& curve, double max_fraction_rate,
      double max_fraction_acceleration, double dt) {
    curve.validate();
    for (double value : {target_si, lower_si, upper_si, max_si_acceleration,
                        max_si_jerk, max_fraction_rate,
                        max_fraction_acceleration, dt}) {
      if (!std::isfinite(value))
        throw std::invalid_argument("motion_conditioner_nonfinite");
    }
    // A zero projected acceleration is a legitimate degenerate axis (for
    // example a vertical spline with no planar authority), not bad input.
    if (dt <= 0. || lower_si > upper_si || max_si_acceleration < 0. ||
        max_si_jerk < 0. || max_fraction_rate < 0. ||
        max_fraction_acceleration < 0. ||
        (curve.enabled() && (max_fraction_rate <= 0. || max_fraction_acceleration <= 0.)))
      throw std::invalid_argument("motion_conditioner_bounds_invalid");

    const Limits limits{lower_si, upper_si, max_si_acceleration, max_si_jerk,
      max_fraction_rate, max_fraction_acceleration, dt, curve};
    const MotionCommandState previous = state_;
    const double target = std::clamp(target_si, lower_si, upper_si);
    const Interval allowed = feasible(previous, limits);
    double selected = std::clamp(previous.command, lower_si, upper_si);
    bool feasibility_override = false;

    if (!allowed.valid()) {
      // A changed hard cap (or changed limits) may make normal smoothing
      // impossible.  Honor the actual cap now and report the lost guarantees.
      const double lo = std::max(lower_si, previous.command - max_si_acceleration * dt);
      const double hi = std::min(upper_si, previous.command + max_si_acceleration * dt);
      selected = lo <= hi ? std::clamp(target, lo, hi) :
        std::clamp(previous.command, lower_si, upper_si);
      feasibility_override = true;
    } else {
      const double braking = std::clamp(previous.command, allowed.lower, allowed.upper);
      const double preferred = std::clamp(target, allowed.lower, allowed.upper);
      const double direction = target > previous.command ? 1. :
        (target < previous.command ? -1. : 0.);
      const auto admissible = [&](double command, bool avoid_target_overshoot) {
        const MotionCommandState next = advanced(previous, command, limits);
        for (double time_scale : {.5, 1., 1.5}) {
          const Limits future{limits.lower, limits.upper, limits.acceleration,
            limits.jerk, limits.mapped_rate, limits.mapped_acceleration,
            limits.dt * time_scale, limits.curve};
          const RestResult rest = restPosition(next, future);
          if (!rest.valid || (avoid_target_overshoot && direction != 0. &&
              direction * (rest.position - target) > kNumericTolerance))
            return false;
        }
        return true;
      };

      if (admissible(preferred, true)) {
        selected = preferred;
      } else if (admissible(braking, true)) {
        // Rest endpoints vary continuously along this scalar interval.  Keep
        // the greatest admissible progress instead of serially filtering an
        // already jerk-shaped command through a second response limiter.
        double accepted = braking;
        double rejected = preferred;
        for (int i = 0; i < kSelectionIterations; ++i) {
          const double candidate = accepted + (rejected - accepted) * .5;
          if (admissible(candidate, true)) accepted = candidate;
          else rejected = candidate;
        }
        selected = accepted;
      } else if (admissible(braking, false)) {
        // A newly moved target can be inside the command's existing stopping
        // distance.  Shed the carried rate normally; do not reverse it in one
        // tick or create an artificial hold/cooldown state.
        selected = braking;
      } else {
        // Prefer reserve for ordinary scheduling jitter.  If only that extra
        // reserve cannot be kept, take the valid nominal braking continuation;
        // it has not violated a configured derivative limit.  A genuinely
        // infeasible nominal continuation is explicitly reported.
        selected = braking;
        feasibility_override = !restPosition(advanced(previous, braking, limits), limits).valid;
      }
    }

    // Exact target/zero is accepted whenever jointly feasible.  No epsilon
    // bypass may violate a derivative bound merely to eliminate a tiny tail.
    if (allowed.valid() && target >= allowed.lower && target <= allowed.upper &&
        std::abs(selected - target) <= kNumericTolerance) {
      const auto at_target = advanced(previous, target, limits);
      const Interval next = feasible(at_target, limits);
      if (next.valid() && target >= next.lower && target <= next.upper)
        selected = target;
    }

    state_ = advanced(previous, selected, limits);last_dt_=dt;
    MotionCommandStep out;
    out.value = state_.command;
    out.acceleration = state_.acceleration;
    out.mapped_fraction = curve.map(selected);
    out.mapped_rate = state_.mapped_rate;
    out.limited = selected != target_si;
    out.overridden_constraints = violations(previous, state_, limits);
    if (feasibility_override) out.overridden_constraints |= MotionOverrideFeasibility;
    out.priority_override = out.overridden_constraints != MotionOverrideNone;
    return out;
  }

 private:
  static constexpr double kNumericTolerance = 1e-12;
  static constexpr int kSelectionIterations = 32;
  static constexpr int kMaximumBrakingSteps = 256;

  struct Limits {
    double lower, upper, acceleration, jerk, mapped_rate, mapped_acceleration, dt;
    const CommandResponseCurve& curve;
  };
  struct Interval {
    double lower, upper;
    bool valid() const { return lower <= upper; }
  };
  struct RestResult { bool valid; double position; };

  static MotionCommandState advanced(const MotionCommandState& previous,
      double command, const Limits& limits) {
    return {command, (command - previous.command) / limits.dt,
      limits.curve.enabled() ?
        (limits.curve.map(command) - limits.curve.map(previous.command)) / limits.dt : 0.};
  }

  static Interval feasible(const MotionCommandState& state, const Limits& limits) {
    double acceleration_low = -limits.acceleration;
    double acceleration_high = limits.acceleration;
    if (limits.jerk > 0.) {
      acceleration_low = std::max(acceleration_low, state.acceleration - limits.jerk * limits.dt);
      acceleration_high = std::min(acceleration_high, state.acceleration + limits.jerk * limits.dt);
    }
    Interval out{std::max(limits.lower, state.command + acceleration_low * limits.dt),
      std::min(limits.upper, state.command + acceleration_high * limits.dt)};
    if (!out.valid() || !limits.curve.enabled()) return out;

    const double previous = limits.curve.map(state.command);
    const double rate_low = std::max(-limits.mapped_rate,
      state.mapped_rate - limits.mapped_acceleration * limits.dt);
    const double rate_high = std::min(limits.mapped_rate,
      state.mapped_rate + limits.mapped_acceleration * limits.dt);
    const double fraction_low = previous + rate_low * limits.dt;
    const double fraction_high = previous + rate_high * limits.dt;
    const double mapped_low = limits.curve.map(out.lower);
    const double mapped_high = limits.curve.map(out.upper);
    if (fraction_low > mapped_high || fraction_high < mapped_low)
      return {1., 0.};

    // Search SI preimages INSIDE the actual interval.  This preserves a valid
    // command on a saturated/plateau curve instead of choosing an unrelated
    // canonical inverse and manufacturing a command jump.
    if (fraction_low > mapped_low) {
      double lo = out.lower, hi = out.upper;
      for (int i = 0; i < 48; ++i) {
        const double mid = lo + (hi - lo) * .5;
        if (limits.curve.map(mid) < fraction_low) lo = mid;
        else hi = mid;
      }
      out.lower = hi;
    }
    if (fraction_high < mapped_high) {
      double lo = out.lower, hi = out.upper;
      for (int i = 0; i < 48; ++i) {
        const double mid = lo + (hi - lo) * .5;
        if (limits.curve.map(mid) > fraction_high) hi = mid;
        else lo = mid;
      }
      out.upper = lo;
    }
    return out;
  }

  static RestResult restPosition(MotionCommandState state, const Limits& limits) {
    // Reduce both command derivatives as quickly as their common feasible
    // interval permits.  Monotone static response makes both changes have the
    // same sign.  This is a bounded scalar construction, never a robot rollout.
    for (int i = 0; i < kMaximumBrakingSteps; ++i) {
      const Interval allowed = feasible(state, limits);
      if (!allowed.valid()) return {false, state.command};
      const double command = std::clamp(state.command, allowed.lower, allowed.upper);
      if (command == state.command) return {true, command};
      const MotionCommandState next = advanced(state, command, limits);
      if (std::abs(next.command - state.command) < kNumericTolerance &&
          std::abs(next.acceleration) < kNumericTolerance &&
          std::abs(next.mapped_rate) < kNumericTolerance)
        return {true, next.command};
      state = next;
    }
    return {false, state.command};
  }

  static std::uint32_t violations(const MotionCommandState& previous,
      const MotionCommandState& current, const Limits& limits) {
    constexpr double tolerance = 1e-9;
    std::uint32_t out = MotionOverrideNone;
    if (std::abs(current.acceleration) > limits.acceleration + tolerance)
      out |= MotionOverrideAcceleration;
    if (limits.jerk > 0. && std::abs(current.acceleration - previous.acceleration) >
        limits.jerk * limits.dt + tolerance)
      out |= MotionOverrideJerk;
    if (limits.curve.enabled()) {
      if (std::abs(current.mapped_rate) > limits.mapped_rate + tolerance)
        out |= MotionOverrideMappedRate;
      if (std::abs(current.mapped_rate - previous.mapped_rate) >
          limits.mapped_acceleration * limits.dt + tolerance)
        out |= MotionOverrideMappedAcceleration;
    }
    return out;
  }

  MotionCommandState state_;
  double last_dt_=0.;
};

}  // namespace d1max_trajectory_tracker
