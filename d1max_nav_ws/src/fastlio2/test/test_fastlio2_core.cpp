#include <gtest/gtest.h>

#include <memory>

#include "ieskf.h"
#include "imu_processor.h"

namespace {

SyncPackage imuPackage(const V3D &acceleration, const V3D &gyro, int count) {
  SyncPackage package;
  package.cloud_end_time = static_cast<double>(count) * 0.005;
  for (int i = 0; i < count; ++i) {
    double time = static_cast<double>(i + 1) * 0.005;
    package.imus.emplace_back(acceleration, gyro, time);
  }
  return package;
}

TEST(IeskfUpdate, KeepsPredictionWhenNoLidarConstraintIsAvailable) {
  IESKF filter;
  filter.P().setIdentity();
  filter.P() *= 2.0;
  filter.x().t_wi = V3D(1.0, 2.0, 3.0);
  const State expected_state = filter.x();
  const M21D expected_covariance = filter.P();
  filter.setLossFunction([](State &, SharedState &shared) { shared.valid = false; });
  filter.setStopFunction([](const V21D &) { return true; });

  filter.update();

  EXPECT_TRUE((filter.x() - expected_state).isZero(1e-12));
  EXPECT_TRUE(filter.P().isApprox(expected_covariance, 1e-12));
}

TEST(ImuInitialization, AcceptsStationaryWindowAndEstimatesGyroBias) {
  Config config;
  config.imu_init_num = 20;
  auto filter = std::make_shared<IESKF>();
  IMUProcessor processor(config, filter);
  const V3D gyro_bias(0.01, -0.02, 0.005);
  SyncPackage package = imuPackage(V3D(0.0, 0.0, 9.80665), gyro_bias,
                                   config.imu_init_num);

  EXPECT_TRUE(processor.initialize(package));
  EXPECT_TRUE(filter->x().bg.isApprox(gyro_bias, 1e-12));
  EXPECT_EQ(processor.initializationStatus(), "ready");
}

TEST(ImuInitialization, RejectsMovingWindow) {
  Config config;
  config.imu_init_num = 20;
  auto filter = std::make_shared<IESKF>();
  IMUProcessor processor(config, filter);
  SyncPackage package;
  package.cloud_end_time = 0.1;
  for (int i = 0; i < config.imu_init_num; ++i) {
    double time = static_cast<double>(i + 1) * 0.005;
    const double lateral_acceleration = i % 2 == 0 ? 1.0 : -1.0;
    package.imus.emplace_back(V3D(lateral_acceleration, 0.0, 9.80665),
                              V3D::Zero(), time);
  }

  EXPECT_FALSE(processor.initialize(package));
  EXPECT_NE(processor.initializationStatus().find("moving"), std::string::npos);
}

}  // namespace
