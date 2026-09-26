#include <gtest/gtest.h>

#include <limits>

#include "imu_continuity.hpp"

namespace {
using faster_lio::ImuContinuityConfig;
using faster_lio::ImuContinuityGuard;
using faster_lio::ImuContinuityMode;
using faster_lio::ImuContinuitySample;

std::vector<ImuContinuitySample> Samples(std::initializer_list<double> stamps) {
    std::vector<ImuContinuitySample> result;
    for (double stamp : stamps) result.push_back({stamp, {0.0, 0.0, -9.80665}, {0.0, 0.0, 0.0}});
    return result;
}

ImuContinuityConfig StrictConfig() {
    return {ImuContinuityMode::FailClosed, 0.015, 0.030};
}
}  // namespace

TEST(ImuContinuity, DefaultDoesNotChangeExistingMappingAdmission) {
    ImuContinuityGuard guard;
    const auto result = guard.Check(1.0, 1.1, {});
    EXPECT_TRUE(result.accepted);
    EXPECT_FALSE(result.checked);
    EXPECT_FALSE(guard.faulted());
}

TEST(ImuContinuity, AcceptsBracketingSamplesWithRealTimeAndRepeatedValues) {
    ImuContinuityGuard guard(StrictConfig());
    const auto samples = Samples({0.995, 1.000, 1.005, 1.010, 1.015, 1.020});
    const auto original = samples;
    const auto result = guard.Check(1.001, 1.019, samples);
    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(result.healthy);
    EXPECT_TRUE(result.checked);
    EXPECT_NEAR(result.maximum_gap_sec, 0.005, 1e-12);
    ASSERT_EQ(samples.size(), original.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_EQ(samples[i].stamp, original[i].stamp);
        EXPECT_EQ(samples[i].acceleration, original[i].acceleration);
    }
}

TEST(ImuContinuity, GapWithinWindowLatchesAndCannotResumeOnNextGoodScan) {
    ImuContinuityGuard guard(StrictConfig());
    const auto bad = guard.Check(1.0, 1.1, Samples({0.995, 1.0, 1.005, 1.080, 1.100}));
    EXPECT_FALSE(bad.accepted);
    EXPECT_FALSE(bad.healthy);
    EXPECT_TRUE(bad.latched);
    EXPECT_EQ(bad.reason, "imu_gap");
    EXPECT_NEAR(bad.maximum_gap_sec, 0.075, 1e-12);
    EXPECT_EQ(bad.offending_begin, 1.005);
    EXPECT_EQ(bad.offending_end, 1.080);
    const auto good = guard.Check(2.0, 2.01, Samples({2.0, 2.005, 2.01}));
    EXPECT_FALSE(good.accepted);
    EXPECT_EQ(good.reason, bad.reason);
    EXPECT_EQ(good.begin, bad.begin);
    EXPECT_TRUE(guard.faulted());
}

TEST(ImuContinuity, RightBracketFindsGapHiddenByScanEndExtrapolation) {
    ImuContinuityGuard guard(StrictConfig());
    const auto result = guard.Check(1.001, 1.015, Samples({1.0, 1.005, 1.100}));
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.reason, "imu_gap");
    EXPECT_NEAR(result.maximum_gap_sec, 0.095, 1e-12);
}

TEST(ImuContinuity, PreviousScanEndIncludesGapBetweenLidarScans) {
    ImuContinuityGuard guard(StrictConfig());
    // The new lidar scan begins at 1.100, but propagation resumes at 1.000.
    const auto result = guard.Check(1.0, 1.11, Samples({0.999, 1.004, 1.1, 1.105, 1.11}));
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.reason, "imu_gap");
    EXPECT_NEAR(result.maximum_gap_sec, 0.096, 1e-12);
}

TEST(ImuContinuity, MappingInitializationAndFirstPropagationUseActualSupportEpoch) {
    // A lidar scan began at 1.000, but initialization IMU starts at 1.020.
    // There is no propagated state yet: do not pretend it existed at 1.000.
    const auto startup = Samples({1.020, 1.025, 1.030, 1.035});
    ImuContinuityGuard initializing(StrictConfig());
    const double initialization_begin = faster_lio::MappingImuContinuityBegin(false, 0.0, startup.front().stamp);
    EXPECT_EQ(initialization_begin, 1.020);
    EXPECT_TRUE(initializing.Check(initialization_begin, 1.032, startup).accepted);

    // Final initialization IMU was 2.998. First propagated scan must include
    // this real tail, not start only at the new scan's 3.010 lidar timestamp.
    const auto first = Samples({2.998, 3.003, 3.008, 3.013, 3.018});
    ImuContinuityGuard propagation(StrictConfig());
    const double first_begin = faster_lio::MappingImuContinuityBegin(false, 0.0, first.front().stamp);
    EXPECT_EQ(first_begin, 2.998);
    EXPECT_TRUE(propagation.Check(first_begin, 3.016, first).accepted);
}

TEST(ImuContinuity, MappingSubsequentPropagationDoesNotReintegratePreviousTailTime) {
    const auto samples = Samples({3.014, 3.019, 3.024, 3.029});
    ImuContinuityGuard guard(StrictConfig());
    const double begin = faster_lio::MappingImuContinuityBegin(true, 3.016, samples.front().stamp);
    EXPECT_EQ(begin, 3.016);
    EXPECT_TRUE(guard.Check(begin, 3.027, samples).accepted);
}

TEST(ImuContinuity, RejectsMissingLeftOrRightCoverage) {
    ImuContinuityGuard left(StrictConfig()), right(StrictConfig());
    EXPECT_EQ(left.Check(1.0, 1.01, Samples({1.005, 1.01})).reason, "imu_start_uncovered");
    EXPECT_EQ(right.Check(1.0, 1.01, Samples({1.0, 1.005})).reason, "imu_end_uncovered");
}

TEST(ImuContinuity, NonfiniteValuesAndNonincreasingTimeAreRejected) {
    ImuContinuityGuard nan(StrictConfig()), reversed(StrictConfig()), duplicate(StrictConfig());
    auto samples = Samples({1.0, 1.005, 1.01});
    samples[1].angular_velocity[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(nan.Check(1.0, 1.01, samples).reason, "imu_nonfinite");
    EXPECT_EQ(reversed.Check(1.0, 1.01, Samples({1.0, 1.008, 1.005, 1.01})).reason,
              "imu_nonmonotonic");
    EXPECT_EQ(duplicate.Check(1.0, 1.01, Samples({1.0, 1.005, 1.005, 1.01})).reason,
              "imu_nonmonotonic");
}

TEST(ImuContinuity, WarningsAreNotFaultsAndOutsideGapsAreNotClippedIntoWindow) {
    ImuContinuityGuard guard(StrictConfig());
    const auto result = guard.Check(1.0, 1.04, Samples({0.0, 1.0, 1.02, 1.04, 2.0}));
    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(result.healthy);
    EXPECT_EQ(result.warning_intervals, 2u);
    EXPECT_NEAR(result.maximum_gap_sec, 0.02, 1e-12);
}

TEST(ImuContinuity, ObserveModeReportsUnhealthyInputWithoutPretendingToProtect) {
    ImuContinuityGuard guard({ImuContinuityMode::Observe, 0.015, 0.030});
    const auto result = guard.Check(1.0, 1.1, Samples({1.0, 1.1}));
    EXPECT_TRUE(result.accepted);
    EXPECT_FALSE(result.healthy);
    EXPECT_TRUE(result.checked);
    EXPECT_FALSE(result.latched);
    EXPECT_EQ(result.reason, "imu_gap");
    EXPECT_FALSE(guard.faulted());
    EXPECT_TRUE(guard.Check(2.0, 2.01, Samples({2.0, 2.005, 2.01})).healthy);
}

TEST(ImuContinuity, CallbackClockFaultLatchesBeforeQueueCanBeCleared) {
    ImuContinuityGuard guard(StrictConfig());
    EXPECT_FALSE(guard.ExternalFault("imu_clock_reset", 10.0, 1.0).accepted);
    EXPECT_EQ(guard.fault().reason, "imu_clock_reset");
    EXPECT_FALSE(guard.Check(1.0, 1.01, Samples({1.0, 1.005, 1.01})).accepted);
}

TEST(ImuContinuity, EpochPrecisionToleranceIsNotAnImuPeriod) {
    ImuContinuityGuard guard(StrictConfig());
    constexpr double epoch = 1790000000.0;
    EXPECT_TRUE(guard.Check(epoch, epoch + 0.030, Samples({epoch, epoch + 0.030})).accepted);
    ImuContinuityGuard missing(StrictConfig());
    EXPECT_FALSE(missing.Check(epoch, epoch + 0.030, Samples({epoch + 0.001, epoch + 0.030})).accepted);
}

TEST(ImuContinuity, InvalidConfigurationAndScanWindowsFailClearly) {
    EXPECT_THROW(ImuContinuityGuard(ImuContinuityConfig{ImuContinuityMode::FailClosed, 0.04, 0.03}),
                 std::invalid_argument);
    EXPECT_THROW(ImuContinuityGuard(ImuContinuityConfig{ImuContinuityMode::FailClosed, 0.01, 0.0}),
                 std::invalid_argument);
    EXPECT_THROW(faster_lio::ParseImuContinuityMode("true"), std::invalid_argument);
    EXPECT_EQ(faster_lio::ParseImuContinuityMode("fail_closed"), ImuContinuityMode::FailClosed);
    ImuContinuityGuard guard(StrictConfig());
    EXPECT_FALSE(guard.Check(1.0, 1.0, Samples({1.0, 1.005})).accepted);
    EXPECT_EQ(guard.fault().reason, "invalid_scan_interval");
}
