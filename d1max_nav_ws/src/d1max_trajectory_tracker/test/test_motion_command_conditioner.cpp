#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <vector>

#include "d1max_trajectory_tracker/motion_command_conditioner.hpp"

using namespace d1max_trajectory_tracker;

namespace {
constexpr double kDt = .05;
constexpr double kRate = 1.6;
constexpr double kMappedAcceleration = 12.;
CommandResponseCurve forwardCurve() { return {.30, .31328213472822414, .40, .10}; }
CommandResponseCurve yawCurve() { return {.50, .20, .35, .10}; }

void expectNormal(const MotionCommandState& previous, const MotionCommandStep& out,
    const CommandResponseCurve& curve, double max_acceleration, double max_jerk,
    double dt = kDt) {
  EXPECT_FALSE(out.priority_override) << out.overridden_constraints;
  EXPECT_LE(std::abs(out.acceleration), max_acceleration + 1e-9);
  if (max_jerk > 0.) {
    EXPECT_LE(std::abs(out.acceleration - previous.acceleration), max_jerk * dt + 1e-9);
  }
  EXPECT_NEAR(out.acceleration, (out.value - previous.command) / dt, 1e-12);
  EXPECT_NEAR(out.mapped_fraction, curve.map(out.value), 1e-12);
  if (curve.enabled()) {
    EXPECT_LE(std::abs(out.mapped_rate), kRate + 1e-9);
    EXPECT_LE(std::abs(out.mapped_rate - previous.mapped_rate), kMappedAcceleration * dt + 1e-9);
    EXPECT_NEAR(out.mapped_rate, (out.mapped_fraction - curve.map(previous.command)) / dt, 1e-12);
  }
}
}

TEST(MotionCommandConditioner, StaticTargetsArriveExactlyWithoutJerkOrMappedRateViolations) {
  for (const auto curve : {CommandResponseCurve{}, forwardCurve(), yawCurve()}) {
    MotionCommandConditioner conditioner;
    for (double target : {.20, -.20, .012, -.012, 0., .30, 0.}) {
      for (int i = 0; i < 160; ++i) {
        SCOPED_TRACE(::testing::Message() << "reference=" << curve.reference << " target=" << target << " tick=" << i);
        const auto previous = conditioner.state();
        const auto out = conditioner.step(target, -.5, .5, .35, 1.0,
          curve, kRate, kMappedAcceleration, kDt);
        expectNormal(previous, out, curve, .35, 1.0);
      }
      EXPECT_DOUBLE_EQ(conditioner.state().command, target);
      EXPECT_DOUBLE_EQ(conditioner.state().acceleration, 0.);
      EXPECT_DOUBLE_EQ(conditioner.state().mapped_rate, 0.);
    }
  }
}

TEST(MotionCommandConditioner, FixedTargetApproachDoesNotOvershootAcrossHighGainResponseRegion) {
  for (const auto curve : {forwardCurve(), yawCurve()}) {
    for (double target : {.001, .01, .03, .05, .10, .20, -.20}) {
      MotionCommandConditioner conditioner;
      double previous_error = std::abs(target);
      for (int i = 0; i < 120; ++i) {
        const auto previous = conditioner.state();
        const auto out = conditioner.step(target, -.5, .5, .8, 3.,
          curve, kRate, kMappedAcceleration, kDt);
        expectNormal(previous, out, curve, .8, 3.);
        const double error = std::abs(target - out.value);
        EXPECT_LE(error, previous_error + 2e-12);
        EXPECT_LE(std::min(0., target) - 2e-12, out.value);
        EXPECT_GE(std::max(0., target) + 2e-12, out.value);
        previous_error = error;
      }
      EXPECT_DOUBLE_EQ(conditioner.state().command, target);
    }
  }
}

TEST(MotionCommandConditioner, ChangingTurnTargetsCrossZeroWithoutFixedWaitOrReset) {
  MotionCommandConditioner conditioner;
  const auto curve = yawCurve();
  double most_positive = 0., most_negative = 0.;
  for (int i = 0; i < 200; ++i) {
    const double target = i < 50 ? .30 : (i < 110 ? -.30 : .15);
    const auto previous = conditioner.state();
    const auto out = conditioner.step(target, -.5, .5, .8, 3.,
      curve, kRate, kMappedAcceleration, kDt);
    expectNormal(previous, out, curve, .8, 3.);
    most_positive = std::max(most_positive, out.value);
    most_negative = std::min(most_negative, out.value);
  }
  EXPECT_GT(most_positive, .29);
  EXPECT_LT(most_negative, -.29);
  EXPECT_DOUBLE_EQ(conditioner.state().command, .15);
}

TEST(MotionCommandConditioner, OrdinaryZeroTargetConvergesInFiniteTimeAndNeverGoesNegative) {
  for (const auto curve : {forwardCurve(), yawCurve()}) {
    MotionCommandConditioner conditioner;
    for (int i = 0; i < 80; ++i)
      conditioner.step(.25, 0., .5, .35, 1., curve, kRate, kMappedAcceleration, kDt);
    ASSERT_DOUBLE_EQ(conditioner.state().command, .25);
    int zero_tick = -1;
    for (int i = 0; i < 100; ++i) {
      const auto previous = conditioner.state();
      const auto out = conditioner.step(0., 0., .5, .35, 1.,
        curve, kRate, kMappedAcceleration, kDt);
      expectNormal(previous, out, curve, .35, 1.);
      EXPECT_GE(out.value, 0.);
      EXPECT_LE(out.value, previous.command + 1e-12);
      if (out.value == 0. && zero_tick < 0) zero_tick = i;
      if (i == 0) { EXPECT_GT(out.value, 0.); }
    }
    EXPECT_GE(zero_tick, 0);
    EXPECT_LT(zero_tick, 50);
    EXPECT_DOUBLE_EQ(conditioner.state().command, 0.);
    EXPECT_DOUBLE_EQ(conditioner.state().acceleration, 0.);
    EXPECT_DOUBLE_EQ(conditioner.state().mapped_rate, 0.);
  }
}

TEST(MotionCommandConditioner, CopyRetainsOneHistoryAcrossOrdinaryCurveReplacement) {
  MotionCommandConditioner original;
  const auto curve = forwardCurve();
  for (int i = 0; i < 7; ++i)
    original.step(.20, 0., .3, .35, 1., curve, kRate, kMappedAcceleration, kDt);
  MotionCommandConditioner replacement = original;
  for (int i = 0; i < 40; ++i) {
    const double target = i < 20 ? .24 : .12;
    const auto a = original.step(target, 0., .3, .35, 1., curve, kRate, kMappedAcceleration, kDt);
    const auto b = replacement.step(target, 0., .3, .35, 1., curve, kRate, kMappedAcceleration, kDt);
    EXPECT_DOUBLE_EQ(a.value, b.value);
    EXPECT_DOUBLE_EQ(a.acceleration, b.acceleration);
    EXPECT_DOUBLE_EQ(a.mapped_rate, b.mapped_rate);
  }
}

TEST(MotionCommandConditioner, ActualHardCapWinsAndReportsWhichSmoothnessBoundsWereLost) {
  MotionCommandConditioner conditioner;
  const auto curve = yawCurve();
  for (int i = 0; i < 80; ++i)
    conditioner.step(.3, -.5, .5, .8, 3., curve, kRate, kMappedAcceleration, kDt);
  const auto out = conditioner.step(0., 0., 0., .8, 3., curve,
    kRate, kMappedAcceleration, kDt);
  EXPECT_DOUBLE_EQ(out.value, 0.);
  EXPECT_DOUBLE_EQ(out.mapped_fraction, 0.);
  EXPECT_TRUE(out.priority_override);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideAcceleration);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideJerk);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideMappedRate);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideMappedAcceleration);
}

TEST(MotionCommandConditioner, ResetIsExactZeroAndCannotRetainOldTurnOrDerivativeHistory) {
  MotionCommandConditioner conditioner, fresh;
  const auto curve = yawCurve();
  for (int i = 0; i < 5; ++i)
    conditioner.step(.3, -.5, .5, .8, 3., curve, kRate, kMappedAcceleration, kDt);
  conditioner.reset();
  EXPECT_DOUBLE_EQ(conditioner.state().command, 0.);
  EXPECT_DOUBLE_EQ(conditioner.state().acceleration, 0.);
  EXPECT_DOUBLE_EQ(conditioner.state().mapped_rate, 0.);
  const auto reset = conditioner.step(-.3, -.5, .5, .8, 3., curve, kRate, kMappedAcceleration, kDt);
  const auto clean = fresh.step(-.3, -.5, .5, .8, 3., curve, kRate, kMappedAcceleration, kDt);
  EXPECT_DOUBLE_EQ(reset.value, clean.value);
  EXPECT_DOUBLE_EQ(reset.mapped_rate, clean.mapped_rate);
}

TEST(MotionCommandConditioner, ZeroAxisAuthorityIsValidAndCannotInventMovement) {
  for (const auto curve : {CommandResponseCurve{}, forwardCurve()}) {
    MotionCommandConditioner conditioner;
    const auto previous = conditioner.state();
    const auto zero_cap = conditioner.step(.3, 0., 0., 0., 0., curve,
      kRate, kMappedAcceleration, kDt);
    expectNormal(previous, zero_cap, curve, 0., 0.);
    EXPECT_DOUBLE_EQ(zero_cap.value, 0.);
    const auto nonzero_cap = conditioner.step(.3, 0., .3, 0., 1., curve,
      kRate, kMappedAcceleration, kDt);
    expectNormal(previous, nonzero_cap, curve, 0., 1.);
    EXPECT_DOUBLE_EQ(nonzero_cap.value, 0.);
  }
}

TEST(MotionCommandConditioner, LossOfAxisAuthorityPrioritizesZeroCapAndReportsDiscontinuity) {
  MotionCommandConditioner conditioner;
  conditioner.reset(.2);
  const auto out = conditioner.step(.3, 0., 0., 0., 1., forwardCurve(),
    kRate, kMappedAcceleration, kDt);
  EXPECT_DOUBLE_EQ(out.value, 0.);
  EXPECT_DOUBLE_EQ(out.mapped_fraction, 0.);
  EXPECT_TRUE(out.priority_override);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideAcceleration);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideJerk);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideMappedRate);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideMappedAcceleration);
  EXPECT_TRUE(out.overridden_constraints & MotionOverrideFeasibility);
}

TEST(MotionCommandConditioner, DisabledResponseAndDisabledJerkHaveTheirExplicitMeaning) {
  MotionCommandConditioner conditioner;
  const auto out = conditioner.step(.3, 0., .3, .35, 0., {}, 0., 0., kDt);
  EXPECT_DOUBLE_EQ(out.value, .35 * kDt);
  EXPECT_DOUBLE_EQ(out.acceleration, .35);
  EXPECT_DOUBLE_EQ(out.mapped_rate, 0.);
  EXPECT_FALSE(out.priority_override);
}

TEST(MotionCommandConditioner, RapidlyChangingTargetsAndTimerJitterKeepJointDerivativeBounds) {
  for (const auto curve : {forwardCurve(), yawCurve()}) {
    for (double jerk : {0., 1., 3.}) {
      MotionCommandConditioner conditioner;
      for (int i = 0; i < 1500; ++i) {
        const double dt = .05 + .003 * std::sin(i * .17);
        const double target = .27 * std::sin(i * .11) + .02 * std::sin(i * 1.3);
        const auto previous = conditioner.state();
        const auto out = conditioner.step(target, -.5, .5, .8, jerk,
          curve, kRate, kMappedAcceleration, dt);
        expectNormal(previous, out, curve, .8, jerk, dt);
      }
    }
  }
}

TEST(MotionCommandConditioner, AppliedCommandSeedUsesTheActualOutputWithoutAHiddenZeroRestart) {
  MotionCommandConditioner conditioner;
  const auto curve = forwardCurve();
  conditioner.reset(.17);
  EXPECT_DOUBLE_EQ(conditioner.state().command, .17);
  EXPECT_DOUBLE_EQ(conditioner.state().acceleration, 0.);
  EXPECT_DOUBLE_EQ(conditioner.state().mapped_rate, 0.);
  const auto previous = conditioner.state();
  const auto out = conditioner.step(.20, 0., .3, .35, 1., curve,
    kRate, kMappedAcceleration, kDt);
  expectNormal(previous, out, curve, .35, 1.);
  EXPECT_GT(out.value, .17);
  EXPECT_LE(out.value, .17 + .35 * kDt);
  EXPECT_THROW(conditioner.reset(NAN), std::invalid_argument);
  EXPECT_DOUBLE_EQ(conditioner.state().command, out.value);
}

TEST(MotionCommandConditioner, BoundedWorkHasInspectableLatencyForContinuousNonlinearInputs) {
  using Clock = std::chrono::steady_clock;
  MotionCommandConditioner conditioner;
  const auto curve = yawCurve();
  std::vector<double> elapsed_us;
  elapsed_us.reserve(3000);
  for (int i = 0; i < 3000; ++i) {
    const double dt = .05 + .003 * std::sin(i * .17);
    const double target = .37 * std::sin(i * .11) + .02 * std::sin(i * 1.3);
    const auto previous = conditioner.state();
    const auto start = Clock::now();
    const auto out = conditioner.step(target, -.5, .5, .8, 3., curve,
      kRate, kMappedAcceleration, dt);
    const auto finish = Clock::now();
    elapsed_us.push_back(std::chrono::duration<double, std::micro>(finish - start).count());
    expectNormal(previous, out, curve, .8, 3., dt);
  }
  std::sort(elapsed_us.begin(), elapsed_us.end());
  const double p50 = elapsed_us[elapsed_us.size()/2];
  const double p95 = elapsed_us[elapsed_us.size()*95/100];
  const double p99 = elapsed_us[elapsed_us.size()*99/100];
  const double maximum = elapsed_us.back();
  std::cout << "conditioner_step_us p50=" << p50 << " p95=" << p95
    << " p99=" << p99 << " max=" << maximum << '\n';
  RecordProperty("step_p50_us", std::to_string(p50));
  RecordProperty("step_p95_us", std::to_string(p95));
  RecordProperty("step_p99_us", std::to_string(p99));
  RecordProperty("step_max_us", std::to_string(maximum));
  // Wall latency is recorded, not asserted: CPU scheduling on another host
  // cannot legitimately change this command-domain correctness test.
}

TEST(MotionCommandConditioner, LegacyPlateauDoesNotManufactureItsLowestInverseAsAnSiJump) {
  MotionCommandConditioner conditioner;
  const CommandResponseCurve curve{.50, .20, .20, .10};
  for (double target : {.25, .35, .15, 0.}) {
    for (int i = 0; i < 120; ++i) {
      const auto previous = conditioner.state();
      const auto out = conditioner.step(target, -.5, .5, .8, 3.,
        curve, kRate, kMappedAcceleration, kDt);
      expectNormal(previous, out, curve, .8, 3.);
    }
    EXPECT_DOUBLE_EQ(conditioner.state().command, target);
  }
}

TEST(MotionCommandConditioner, InvalidInputsCannotConsumeStateOrInventTimeCredit) {
  MotionCommandConditioner conditioner;
  const auto curve = forwardCurve();
  for (double dt : {0., -.05, double(NAN), double(INFINITY)})
    EXPECT_THROW(conditioner.step(.2, 0., .3, .35, 1., curve,
      kRate, kMappedAcceleration, dt), std::invalid_argument);
  EXPECT_THROW(conditioner.step(NAN, 0., .3, .35, 1., curve,
    kRate, kMappedAcceleration, kDt), std::invalid_argument);
  EXPECT_THROW(conditioner.step(.2, .3, 0., .35, 1., curve,
    kRate, kMappedAcceleration, kDt), std::invalid_argument);
  for (double acceleration : {-.01, double(NAN), double(INFINITY)})
    EXPECT_THROW(conditioner.step(.2, 0., .3, acceleration, 1., curve,
      kRate, kMappedAcceleration, kDt), std::invalid_argument);
  EXPECT_THROW(conditioner.step(.2, 0., .3, .35, -1., curve,
    kRate, kMappedAcceleration, kDt), std::invalid_argument);
  EXPECT_THROW(conditioner.step(.2, 0., .3, .35, 1., curve,
    0., kMappedAcceleration, kDt), std::invalid_argument);
  EXPECT_DOUBLE_EQ(conditioner.state().command, 0.);
  EXPECT_DOUBLE_EQ(conditioner.state().acceleration, 0.);
  EXPECT_DOUBLE_EQ(conditioner.state().mapped_rate, 0.);
}
