#include <gtest/gtest.h>
#include "imu_recovery.hpp"

TEST(ImuRecovery, ExplicitOptInAndReasonWhitelist) {
  faster_lio::ImuRecoveryBudget budget;
  EXPECT_FALSE(budget.allow("imu_gap",1.));
  budget.configure(true,3,60.);
  EXPECT_TRUE(budget.allow("imu_gap",1.));
  EXPECT_TRUE(budget.allow("imu_start_uncovered",2.));
  for(const auto* reason:{"imu_clock_reset","lidar_clock_reset","weak_geometry","local_pose_jump"})
    EXPECT_FALSE(budget.allow(reason,3.));
}
TEST(ImuRecovery, BoundedRollingWindowAndMonotonicTime) {
  faster_lio::ImuRecoveryBudget budget;budget.configure(true,3,60.);
  EXPECT_TRUE(budget.allow("imu_gap",1.));EXPECT_TRUE(budget.allow("imu_gap",2.));
  EXPECT_FALSE(budget.allow("imu_gap",1.5));EXPECT_TRUE(budget.allow("imu_gap",3.));
  EXPECT_FALSE(budget.allow("imu_gap",60.));EXPECT_TRUE(budget.allow("imu_gap",61.));
  EXPECT_FALSE(budget.allow("imu_gap",61.1));
}
TEST(ImuRecovery, InvalidConfiguration) {
  faster_lio::ImuRecoveryBudget budget;
  EXPECT_THROW(budget.configure(true,0,60.),std::invalid_argument);
  EXPECT_THROW(budget.configure(true,3,1.),std::invalid_argument);
}
