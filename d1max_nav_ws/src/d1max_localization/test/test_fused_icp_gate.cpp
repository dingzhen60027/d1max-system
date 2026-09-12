#include "d1max_localization/fused_icp_gate.hpp"

#include <gtest/gtest.h>

namespace d1max_localization {
namespace {

FusedIcpGate makeGate(int confirmations = 3) {
  FusedIcpGate gate;
  gate.configure({0.45, 0.60, 1.0}, {2.5, 1.57, 1.0}, confirmations);
  return gate;
}

TEST(FusedIcpGateTest, RoughInitialCorrectionIsNotPublishedImmediately) {
  auto gate = makeGate();
  const auto decision = gate.evaluate(1.2, 0.8, 0.4);
  EXPECT_TRUE(decision.update_candidate);
  EXPECT_FALSE(decision.publish_pose);
  EXPECT_FALSE(gate.locked());
}

TEST(FusedIcpGateTest, LocksAfterStableConfirmations) {
  auto gate = makeGate();
  ASSERT_TRUE(gate.evaluate(1.2, 0.8, 0.4).update_candidate);
  ASSERT_TRUE(gate.evaluate(0.08, 0.04, 0.3).update_candidate);
  const auto decision = gate.evaluate(0.05, 0.03, 0.25);
  EXPECT_TRUE(decision.just_locked);
  EXPECT_TRUE(decision.publish_pose);
  EXPECT_TRUE(gate.locked());
}

TEST(FusedIcpGateTest, NeverFallsBackAfterTrackingLock) {
  auto gate = makeGate(1);
  ASSERT_TRUE(gate.evaluate(1.2, 0.8, 0.4).just_locked);
  EXPECT_FALSE(gate.evaluate(0.8, 0.1, 0.2).update_candidate);
  EXPECT_TRUE(gate.locked());
  EXPECT_FALSE(gate.evaluate(0.8, 0.1, 0.2).update_candidate);
  EXPECT_TRUE(gate.locked());
}

TEST(FusedIcpGateTest, ResetRequiresAcquisitionAgain) {
  auto gate = makeGate();
  ASSERT_TRUE(gate.evaluate(1.2, 0.8, 0.4).update_candidate);
  ASSERT_TRUE(gate.evaluate(0.08, 0.04, 0.3).update_candidate);
  ASSERT_TRUE(gate.evaluate(0.05, 0.03, 0.25).just_locked);
  gate.reset();
  const auto decision = gate.evaluate(1.2, 0.8, 0.4);
  EXPECT_TRUE(decision.update_candidate);
  EXPECT_FALSE(decision.publish_pose);
  EXPECT_FALSE(gate.locked());
}

}  // namespace
}  // namespace d1max_localization

TEST(FusedIcpGateRegression, RejectedAttemptBreaksAcquisitionButNeverUnlocksTracking) {
  d1max_localization::FusedIcpGate gate;
  gate.configure({.45,.6,1.},{2.5,1.57,1.},3);
  gate.evaluate(0.,0.,.1);
  gate.rejectAttempt();
  EXPECT_EQ(gate.confirmationCount(),0);
  EXPECT_FALSE(gate.evaluate(0.,0.,.1).publish_pose);
  EXPECT_FALSE(gate.evaluate(0.,0.,.1).publish_pose);
  EXPECT_TRUE(gate.evaluate(0.,0.,.1).publish_pose);
  gate.rejectAttempt();
  EXPECT_TRUE(gate.locked());
}
