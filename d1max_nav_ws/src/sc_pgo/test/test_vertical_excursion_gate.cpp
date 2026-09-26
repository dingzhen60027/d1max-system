#include "vertical_excursion_gate.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using sc_pgo::VerticalExcursionGateReason;
using sc_pgo::VerticalPoseSample;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::vector<VerticalPoseSample> climb() {
  std::vector<VerticalPoseSample> samples;
  for (int i = 0; i <= 40; ++i) {
    const double z = i < 8 ? 3.7 * i / 8.0 : 3.7;
    samples.push_back({0.5 * i, 0.0, z});
  }
  return samples;
}
}  // namespace

int main() {
  // Several metres of accumulated height drift over a long corridor do not
  // constitute a local staircase. A true large-loop correction stays possible.
  std::vector<VerticalPoseSample> gradual;
  for (int i = 0; i <= 200; ++i) gradual.push_back({0.5 * i, 0.0, 4.6 * i / 200.0});
  const auto long_drift = sc_pgo::evaluateVerticalExcursionGate(gradual, 0.0);
  require(long_drift.accepted && long_drift.max_ascent_m < 1.5 &&
          long_drift.raw_relative_z_m > 4.5,
          "long gradual drift incorrectly classified as a floor change");

  const auto stairs = climb();
  const auto false_floor_match = sc_pgo::evaluateVerticalExcursionGate(stairs, 0.0);
  require(!false_floor_match.accepted &&
          false_floor_match.reason == VerticalExcursionGateReason::height_disagreement &&
          false_floor_match.max_ascent_m > 3.6 &&
          false_floor_match.max_descent_m < 0.01,
          "cross-floor geometry match accepted despite local ascent");

  const auto correct_height = sc_pgo::evaluateVerticalExcursionGate(stairs, 3.1);
  require(correct_height.accepted &&
          correct_height.vertical_disagreement_m < 1.0,
          "height-compatible registration incorrectly rejected");

  // A loop spanning a long drifting corridor and then a staircase must not
  // compare ICP with the raw endpoints: the latter contain corridor drift.
  std::vector<VerticalPoseSample> drift_then_stairs;
  for (int i = 0; i <= 200; ++i) {
    drift_then_stairs.push_back({0.5 * i, 0.0, -2.6 * i / 200.0});
  }
  for (int i = 1; i <= 28; ++i) {
    drift_then_stairs.push_back({100.0 + 0.5 * i, 0.0,
      -2.6 + 3.7 * i / 28.0});
  }
  const auto true_cross_floor = sc_pgo::evaluateVerticalExcursionGate(
    drift_then_stairs, 3.7);
  require(true_cross_floor.accepted && true_cross_floor.raw_relative_z_m < 1.2 &&
          true_cross_floor.vertical_disagreement_m < 1.0,
          "long-term drift incorrectly rejected a height-compatible loop");
  require(!sc_pgo::evaluateVerticalExcursionGate(
      drift_then_stairs, 0.0).accepted,
    "long-term drift hid a false zero-height cross-floor match");

  // Ascending and descending the same stairs before re-entering the old
  // level produces balanced local excursions and no floor-change evidence.
  std::vector<VerticalPoseSample> round_trip;
  for (int i = 0; i <= 80; ++i) {
    const double z = i <= 8 ? 3.7 * i / 8.0 :
      (i < 50 ? 3.7 : (i <= 58 ? 3.7 * (58 - i) / 8.0 : 0.0));
    round_trip.push_back({0.5 * i, 0.0, z});
  }
  const auto same_level = sc_pgo::evaluateVerticalExcursionGate(round_trip, 0.0);
  require(same_level.accepted && std::abs(same_level.excursion_evidence_m) < 1e-9 &&
          same_level.max_ascent_m > 3.6 && same_level.max_descent_m > 3.6,
          "up/down same-level loop rejected");

  std::vector<VerticalPoseSample> descent;
  for (int i = 0; i <= 12; ++i) descent.push_back({i * 0.5, 0.0, -3.7 * i / 12.0});
  const auto false_descent_match = sc_pgo::evaluateVerticalExcursionGate(descent, 0.0);
  require(!false_descent_match.accepted && false_descent_match.excursion_evidence_m < -3.6,
          "cross-floor descent accepted");

  // Vertical travel while standing at one XY location, as in a lift, counts.
  const auto stationary = sc_pgo::evaluateVerticalExcursionGate(
    {{0, 0, 0}, {0, 0, 1.0}, {0, 0, 2.3}, {0, 0, 3.7}}, 0.0);
  require(!stationary.accepted && stationary.max_ascent_m > 3.6,
          "stationary XY vertical excursion ignored");

  auto invalid = sc_pgo::evaluateVerticalExcursionGate({{0, 0, 0}}, 0.0);
  require(!invalid.accepted && invalid.reason == VerticalExcursionGateReason::invalid_input,
          "single sample admitted");
  invalid = sc_pgo::evaluateVerticalExcursionGate(
    {{0, 0, 0}, {1, 0, std::numeric_limits<double>::infinity()}}, 0.0);
  require(!invalid.accepted, "nonfinite trajectory admitted");
  invalid = sc_pgo::evaluateVerticalExcursionGate({{0, 0, 0}, {1, 0, 0}},
    std::numeric_limits<double>::quiet_NaN());
  require(!invalid.accepted, "nonfinite ICP displacement admitted");
  invalid = sc_pgo::evaluateVerticalExcursionGate(
    {{0, 0, 0}, {21, 0, 3.7}}, 0.0);
  require(!invalid.accepted, "unobservable gap larger than window admitted");
  sc_pgo::VerticalExcursionGateOptions options;
  options.max_vertical_disagreement_m = -1.0;
  invalid = sc_pgo::evaluateVerticalExcursionGate(stairs, 0.0, options);
  require(!invalid.accepted, "invalid threshold admitted");

  std::cout << "PASS: vertical excursion gate tests\n";
}
