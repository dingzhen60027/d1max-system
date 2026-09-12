#include "d1max_localization/rotation_deskewer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace d1max_localization {
namespace {

constexpr int64_t kMillisecond = 1000000;

std::deque<AngularVelocitySample> constantImu(
    const Eigen::Vector3d &angular_velocity) {
  std::deque<AngularVelocitySample> samples;
  for (int64_t time_ms = -5; time_ms <= 105; time_ms += 5) {
    samples.push_back({time_ms * kMillisecond, angular_velocity});
  }
  return samples;
}

TEST(RotationDeskewerTest, LeavesStationaryPointsUnchanged) {
  RotationDeskewer deskewer;
  std::string error;
  ASSERT_TRUE(deskewer.build(
      constantImu(Eigen::Vector3d::Zero()), 0, 100 * kMillisecond,
      Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.02, &error))
      << error;
  const Eigen::Vector3d point(2.0, -1.0, 0.5);
  EXPECT_TRUE(
      deskewer.compensate(point, 37 * kMillisecond).isApprox(point, 1e-12));
}

TEST(RotationDeskewerTest, CompensatesConstantYawToScanEnd) {
  constexpr double yaw_rate = M_PI / 2.0;
  RotationDeskewer deskewer;
  std::string error;
  ASSERT_TRUE(deskewer.build(
      constantImu(Eigen::Vector3d(0.0, 0.0, yaw_rate)), 0,
      100 * kMillisecond, Eigen::Vector3d::Zero(),
      Eigen::Matrix3d::Identity(), 0.02, &error))
      << error;
  const double end_yaw = yaw_rate * 0.1;
  const Eigen::Vector3d expected(std::cos(end_yaw), -std::sin(end_yaw), 0.0);
  EXPECT_TRUE(deskewer.compensate(Eigen::Vector3d::UnitX(), 0)
                  .isApprox(expected, 1e-9));
}

TEST(RotationDeskewerTest, RemovesConfiguredGyroBias) {
  const Eigen::Vector3d bias(0.01, -0.02, 0.03);
  RotationDeskewer deskewer;
  std::string error;
  ASSERT_TRUE(deskewer.build(
      constantImu(bias), 0, 100 * kMillisecond, bias,
      Eigen::Matrix3d::Identity(), 0.02, &error))
      << error;
  const Eigen::Vector3d point(1.0, 2.0, 3.0);
  EXPECT_TRUE(deskewer.compensate(point, 50 * kMillisecond)
                  .isApprox(point, 1e-12));
}

TEST(RotationDeskewerTest, RejectsIncompleteImuCoverage) {
  auto samples = constantImu(Eigen::Vector3d::Zero());
  while (samples.back().stamp_ns >= 90 * kMillisecond) {
    samples.pop_back();
  }
  RotationDeskewer deskewer;
  std::string error;
  EXPECT_FALSE(deskewer.build(
      samples, 0, 100 * kMillisecond, Eigen::Vector3d::Zero(),
      Eigen::Matrix3d::Identity(), 0.02, &error));
  EXPECT_EQ(error, "IMU does not cover scan end");
}

}  // namespace
}  // namespace d1max_localization

TEST(RotationDeskewerRegression, NonCoincidentScanBoundariesMaterializeEigenValues) {
  using namespace d1max_localization;
  std::deque<AngularVelocitySample> imu;
  for (int i=0;i<=105;i+=5) imu.push_back({i*1000000LL,Eigen::Vector3d(0.,0.,1.)});
  RotationDeskewer deskewer; std::string error;
  ASSERT_TRUE(deskewer.build(imu,1000000,99000000,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.02,&error))<<error;
  const Eigen::Vector3d expected(std::cos(.098),-std::sin(.098),0.);
  EXPECT_TRUE(deskewer.compensate(Eigen::Vector3d::UnitX(),1000000).isApprox(expected,1e-10));
}

TEST(TranslationDeskewerRegression, ConstantThreeAxisTranslationAndMissingCoverage) {
  using namespace d1max_localization;
  std::deque<AngularVelocitySample> imu;
  std::deque<LinearVelocitySample> velocity;
  for(int i=0;i<=100;i+=5) imu.push_back({i*1000000LL,Eigen::Vector3d::Zero()});
  for(int i=0;i<=100;i+=20) velocity.push_back({i*1000000LL,Eigen::Vector3d(1.,-.5,.2)});
  RotationDeskewer deskewer;std::string error;
  ASSERT_TRUE(deskewer.build(imu,1000000,99000000,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.02,&error));
  ASSERT_TRUE(deskewer.addTranslation(velocity,.06,&error))<<error;
  EXPECT_TRUE(deskewer.compensate(Eigen::Vector3d(2.,3.,4.),1000000)
      .isApprox(Eigen::Vector3d(1.902,3.049,3.9804),1e-10));
  EXPECT_TRUE(deskewer.compensate(Eigen::Vector3d(2.,3.,4.),99000000).isApprox(Eigen::Vector3d(2.,3.,4.),1e-10));
  velocity.pop_back();
  EXPECT_FALSE(deskewer.addTranslation(velocity,.06,&error));
}
TEST(TranslationDeskewerRegression, TranslationDuringRotationUsesScanStartBasis) {
  using namespace d1max_localization;
  std::deque<AngularVelocitySample> imu;
  std::deque<LinearVelocitySample> velocity;
  for(int i=0;i<=100;i+=1)imu.push_back({i*1000000LL,Eigen::Vector3d(0.,0.,1.)});
  for(int i=0;i<=100;i+=20)velocity.push_back({i*1000000LL,Eigen::Vector3d::UnitX()});
  RotationDeskewer deskewer;std::string error;
  ASSERT_TRUE(deskewer.build(imu,0,100000000,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.02,&error));
  ASSERT_TRUE(deskewer.addTranslation(velocity,.06,&error));
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d expected=q.conjugate()*(Eigen::Vector3d(2.,0.,0.)-Eigen::Vector3d(std::sin(.1),1-std::cos(.1),0.));
  EXPECT_TRUE(deskewer.compensate(Eigen::Vector3d(2.,0.,0.),0).isApprox(expected,1e-8));
}
TEST(TranslationDeskewerRegression, RejectsGapsAndNonfiniteVelocity) {
  using namespace d1max_localization;
  std::deque<AngularVelocitySample> imu;
  for(int i=0;i<=100;i+=5)imu.push_back({i*1000000LL,Eigen::Vector3d::Zero()});
  RotationDeskewer deskewer;std::string error;
  ASSERT_TRUE(deskewer.build(imu,0,100000000,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.02,&error));
  std::deque<LinearVelocitySample> velocity{{0,Eigen::Vector3d::Zero()},{100000000,Eigen::Vector3d::Zero()}};
  EXPECT_FALSE(deskewer.addTranslation(velocity,.06,&error));
  ASSERT_TRUE(deskewer.build(imu,0,100000000,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.02,&error));
  velocity[1].linear_velocity.x()=std::nan("");
  EXPECT_FALSE(deskewer.addTranslation(velocity,.2,&error));
}
