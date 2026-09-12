#pragma once

#include <algorithm>
#include <cmath>

namespace d1max_localization {

struct IcpGateLimits {
  double max_translation_delta{0.45};
  double max_yaw_delta{0.60};
  double max_fitness_score{1.0};
};

struct IcpGateDecision {
  bool update_candidate{false};
  bool publish_pose{false};
  bool just_locked{false};
  bool restarted_confirmation{false};
};

class FusedIcpGate {
public:
  void configure(const IcpGateLimits &tracking_limits,
                 const IcpGateLimits &acquisition_limits,
                 int required_confirmations) {
    tracking_limits_ = tracking_limits;
    acquisition_limits_ = acquisition_limits;
    confirmation_limits_ = tracking_limits;
    confirmation_limits_.max_fitness_score = std::min(
        tracking_limits.max_fitness_score,
        acquisition_limits.max_fitness_score);
    required_confirmations_ = std::max(1, required_confirmations);
    reset();
  }

  void reset() {
    locked_ = false;
    confirmation_count_ = 0;
  }

  IcpGateDecision evaluate(double translation_delta, double yaw_delta,
                           double fitness_score) {
    if (locked_) {
      const bool accepted = passes(tracking_limits_, translation_delta,
                                   yaw_delta, fitness_score);
      return {accepted, accepted, false, false};
    }

    if (confirmation_count_ == 0) {
      if (!passes(acquisition_limits_, translation_delta, yaw_delta,
                  fitness_score)) {
        return {};
      }
      confirmation_count_ = 1;
      return acceptCandidate(false);
    }

    if (passes(confirmation_limits_, translation_delta, yaw_delta,
               fitness_score)) {
      ++confirmation_count_;
      return acceptCandidate(false);
    }

    if (passes(acquisition_limits_, translation_delta, yaw_delta,
               fitness_score)) {
      confirmation_count_ = 1;
      return acceptCandidate(true);
    }

    confirmation_count_ = 0;
    return {};
  }

  // Every rejected processed scan breaks acquisition continuity, but a bad
  // tracking scan must never silently unlock the matcher / widen its search.
  void rejectAttempt() {
    if (!locked_) confirmation_count_ = 0;
  }

  bool locked() const { return locked_; }
  int confirmationCount() const { return confirmation_count_; }
  int requiredConfirmations() const { return required_confirmations_; }

private:
  static bool passes(const IcpGateLimits &limits, double translation_delta,
                     double yaw_delta, double fitness_score) {
    return std::isfinite(translation_delta) && std::isfinite(yaw_delta) &&
           std::isfinite(fitness_score) && translation_delta >= 0.0 &&
           yaw_delta >= 0.0 && fitness_score >= 0.0 &&
           translation_delta <= limits.max_translation_delta &&
           yaw_delta <= limits.max_yaw_delta &&
           fitness_score <= limits.max_fitness_score;
  }

  IcpGateDecision acceptCandidate(bool restarted_confirmation) {
    if (confirmation_count_ >= required_confirmations_) {
      locked_ = true;
      return {true, true, true, restarted_confirmation};
    }
    return {true, false, false, restarted_confirmation};
  }

  IcpGateLimits tracking_limits_;
  IcpGateLimits acquisition_limits_;
  IcpGateLimits confirmation_limits_;
  int required_confirmations_{3};
  int confirmation_count_{0};
  bool locked_{false};
};

}  // namespace d1max_localization
