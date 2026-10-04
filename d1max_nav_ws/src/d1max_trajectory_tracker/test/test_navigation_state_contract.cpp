#include <gtest/gtest.h>
#include "d1max_trajectory_tracker/navigation_state_contract.hpp"
using namespace d1max_trajectory_tracker;

namespace {
Config configuration() { Config c; c.session_id="session"; c.map_version_id="map"; return c; }
d1max_planning_interfaces::msg::NavigationState state() {
  d1max_planning_interfaces::msg::NavigationState s;
  s.schema_version=2; s.session_id="session"; s.map_version_id="map";
  s.localization_epoch=3; s.localization_seed_id="seed3"; s.usable=true;
  s.source_stamp.sec=100; s.source_stamp.nanosec=123456789U;
  s.posterior_stamp=s.imu_stamp=s.source_stamp;
  auto& l=s.local_odometry; auto& g=s.global_odometry;
  l.header.stamp=g.header.stamp=s.source_stamp;
  l.header.frame_id="d1max_loc_odom"; g.header.frame_id="d1max_loc_map";
  l.child_frame_id=g.child_frame_id="d1max_loc_base_link";
  l.pose.pose.orientation.w=g.pose.pose.orientation.w=1.;
  l.pose.pose.position.x=1.; g.pose.pose.position.x=1001.;
  return s;
}
}

TEST(NavigationStateContract, UsesLocalSourcePoseNeverGlobalCorrectionOrTaskEpoch) {
  Odom o; std::string reason; const auto s=state();
  ASSERT_TRUE(decodeNavigationState(s,configuration(),o,reason));
  EXPECT_DOUBLE_EQ(o.position.x(),1.); EXPECT_EQ(o.localization_epoch,3U);
  EXPECT_EQ(o.localization_seed_id,"seed3"); EXPECT_EQ(o.frame_id,"d1max_loc_odom");
  EXPECT_DOUBLE_EQ(o.stamp,100.123456789); EXPECT_EQ(o.session_id,"session");
  EXPECT_EQ(o.source_stamp_ns,100123456789LL);
  EXPECT_EQ(s.source_stamp.nanosec,123456789U);
}

TEST(NavigationStateIngress, OldUnusableEpochAndExpiredDuplicateCannotClearNewObservation) {
  NavigationStateIngress inbox;auto fresh=state();
  ASSERT_TRUE(inbox.inspect(fresh,configuration(),100.13));
  auto old=fresh;old.usable=false;old.source_stamp.sec=99;
  EXPECT_FALSE(inbox.inspect(old,configuration(),101.)); // expires before decode, still ignored
  old=fresh;old.usable=false;EXPECT_FALSE(inbox.inspect(old,configuration(),100.2));
  old=fresh;old.source_stamp.sec=101;old.localization_epoch=2;
  EXPECT_FALSE(inbox.inspect(old,configuration(),101.)); // retired identity even with newer time
  fresh.source_stamp.nanosec+=20000000U;ASSERT_TRUE(inbox.inspect(fresh,configuration(),100.15));
}

TEST(NavigationStateIngress, NewFaultCreatesSourceBarrierButFutureClockCannotPoisonRecovery) {
  NavigationStateIngress inbox;auto fault=state();fault.usable=false;
  fault.source_stamp.nanosec+=40000000U;ASSERT_TRUE(inbox.inspect(fault,configuration(),100.17));
  EXPECT_FALSE(inbox.inspect(state(),configuration(),100.18)); // old valid cannot undo fresh fault
  auto future=fault;future.source_stamp.sec=5000;
  EXPECT_TRUE(inbox.inspect(future,configuration(),100.18)); // decoder/core must fail closed
  auto recovered=state();recovered.source_stamp.nanosec+=60000000U;
  EXPECT_TRUE(inbox.inspect(recovered,configuration(),100.19)); // future stamp never leased
  recovered.localization_epoch=4;recovered.source_stamp.nanosec+=20000000U;
  EXPECT_TRUE(inbox.inspect(recovered,configuration(),100.21)); // genuine fresh reset reaches core
}
TEST(NavigationStateIngress, EpochClockResetIsNotAnOldPacketAndForeignStreamCannotPoisonWatermark) {
  NavigationStateIngress inbox;auto first=state();ASSERT_TRUE(inbox.inspect(first,configuration(),100.13));
  auto foreign=first;foreign.session_id="other";foreign.source_stamp.sec=100;foreign.source_stamp.nanosec=170000000U;
  EXPECT_FALSE(inbox.inspect(foreign,configuration(),100.17)); // ignored before decode/hold
  auto next=first;next.source_stamp.nanosec=150000000U;
  EXPECT_TRUE(inbox.inspect(next,configuration(),100.17));
  auto reset=first;reset.localization_epoch=4;reset.localization_seed_id="reset";reset.source_stamp.sec=99;
  EXPECT_TRUE(inbox.inspect(reset,configuration(),100.17,3)); // hard reset must reach owner
  next.source_stamp.sec=101;EXPECT_FALSE(inbox.inspect(next,configuration(),101.,4));
}
TEST(NavigationStateIngress, ForeignMapSessionOrSchemaNeverReachesFaultHandlingButOwnMalformedInputDoes) {
  for(int variant=0;variant<3;++variant) {
    NavigationStateIngress inbox;auto current=state();ASSERT_TRUE(inbox.inspect(current,configuration(),100.13));
    auto foreign=current;foreign.source_stamp.sec=100;foreign.source_stamp.nanosec=190000000U;foreign.usable=false;
    if(variant==0)foreign.session_id="other";
    if(variant==1)foreign.map_version_id="other";
    if(variant==2)foreign.schema_version=99;
    EXPECT_FALSE(inbox.inspect(foreign,configuration(),100.19));
    current.source_stamp.nanosec=150000000U;EXPECT_TRUE(inbox.inspect(current,configuration(),100.19));
    auto fault=current;fault.source_stamp.nanosec=200000000U;fault.localization_seed_id.clear();
    EXPECT_TRUE(inbox.inspect(fault,configuration(),100.2)); // own fresh malformed must hold
    current.source_stamp.nanosec=190000000U;EXPECT_FALSE(inbox.inspect(current,configuration(),100.21));
    current.source_stamp.nanosec=210000000U;EXPECT_TRUE(inbox.inspect(current,configuration(),100.21));
  }
}
TEST(NavigationStateIngress, UnusableFreshResetIsHardButForeignOrFutureEnvelopeIsNotAnEstimatorReset) {
  auto s=state();ControlIdentity expected;expected.localization_epoch=s.localization_epoch;
  expected.localization_seed_id=s.localization_seed_id;
  s.usable=false;s.localization_epoch=4;s.localization_seed_id="reset";s.source_stamp.sec=99;
  EXPECT_TRUE(navigationContextReset(s,configuration(),expected,100.2));
  s.source_stamp.sec=5000;EXPECT_FALSE(navigationContextReset(s,configuration(),expected,100.2));
  s.source_stamp.sec=100;s.session_id="foreign";
  EXPECT_FALSE(navigationContextReset(s,configuration(),expected,100.2));
}

TEST(NavigationStateContract, RejectsUnpairedStampsFramesIdentityAndUnusableState) {
  for(int variant=0;variant<10;++variant) {
    auto s=state(); Odom o; std::string reason;
    if(variant==0)++s.global_odometry.header.stamp.nanosec;
    if(variant==1)s.local_odometry.header.frame_id="d1max_loc_map";
    if(variant==2)s.global_odometry.child_frame_id="lidar";
    if(variant==3)s.session_id="old-session";
    if(variant==4)s.map_version_id="other-map";
    if(variant==5)s.localization_epoch=0;
    if(variant==6)s.localization_seed_id.clear();
    if(variant==7)s.usable=false;
    if(variant==8)s.schema_version=1;
    if(variant==9)s.posterior_stamp.nanosec=1000000000U;
    EXPECT_FALSE(decodeNavigationState(s,configuration(),o,reason)) << variant;
  }
}

TEST(NavigationStateContract, RotatesMeasuredChildTwistToLocalFrameNotGlobalFrame) {
  auto s=state(); auto& q=s.local_odometry.pose.pose.orientation;
  q.w=q.z=std::sqrt(.5); s.local_odometry.twist.twist.linear.x=.2;
  s.local_odometry.twist.twist.angular.x=.3;
  Odom o; std::string reason;
  ASSERT_TRUE(decodeNavigationState(s,configuration(),o,reason));
  EXPECT_NEAR(o.velocity_in_frame.x(),0.,1e-9); EXPECT_NEAR(o.velocity_in_frame.y(),.2,1e-9);
  EXPECT_NEAR(o.angular_velocity_in_frame.y(),.3,1e-9);
}

TEST(NavigationStateContract, InvalidGlobalHalfCannotHideBehindValidLocalHalf) {
  auto s=state(); Odom o; std::string reason;
  s.global_odometry.pose.pose.position.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(decodeNavigationState(s,configuration(),o,reason));
  s=state(); s.global_odometry.pose.pose.orientation.w=2.;
  EXPECT_FALSE(decodeNavigationState(s,configuration(),o,reason));
}
TEST(NavigationStateContract, IndependentLocalTransportHasNoGlobalDependencyOrRestamping) {
  const auto pair=state();d1max_planning_interfaces::msg::LocalNavigationState local;
  local.schema_version=1;local.session_id=pair.session_id;local.map_version_id=pair.map_version_id;
  local.localization_epoch=pair.localization_epoch;local.localization_seed_id=pair.localization_seed_id;
  local.local_odometry=pair.local_odometry;local.source_stamp=pair.source_stamp;
  local.posterior_stamp=pair.posterior_stamp;local.imu_stamp=pair.imu_stamp;
  local.extrapolation_sec=pair.extrapolation_sec;local.usable=true;
  Odom output;std::string reason;
  ASSERT_TRUE(decodeLocalNavigationState(local,configuration(),output,reason));
  EXPECT_DOUBLE_EQ(output.position.x(),1.);EXPECT_DOUBLE_EQ(output.stamp,100.123456789);
  EXPECT_EQ(output.source_stamp_ns,100123456789LL);
  EXPECT_EQ(output.localization_epoch,3U);EXPECT_EQ(output.schema_version,2U);
  auto global_unavailable=pair;global_unavailable.usable=false;
  EXPECT_FALSE(decodeNavigationState(global_unavailable,configuration(),output,reason));
  ASSERT_TRUE(decodeLocalNavigationState(local,configuration(),output,reason));
  local.usable=false;EXPECT_FALSE(decodeLocalNavigationState(local,configuration(),output,reason));
}
TEST(NavigationStateContract, LocalTransportRejectsInvalidIdentitySourceFrameAndGeometry) {
  const auto pair=state();
  for(int variant=0;variant<11;++variant) {
    d1max_planning_interfaces::msg::LocalNavigationState local;
    local.schema_version=1;local.session_id=pair.session_id;local.map_version_id=pair.map_version_id;
    local.localization_epoch=pair.localization_epoch;local.localization_seed_id=pair.localization_seed_id;
    local.local_odometry=pair.local_odometry;local.source_stamp=pair.source_stamp;
    local.posterior_stamp=pair.posterior_stamp;local.imu_stamp=pair.imu_stamp;local.usable=true;
    if(variant==0)local.schema_version=2;
    if(variant==1)local.session_id="old";
    if(variant==2)local.map_version_id="other";
    if(variant==3)local.localization_epoch=0;
    if(variant==4)local.localization_seed_id.clear();
    if(variant==5)++local.local_odometry.header.stamp.nanosec;
    if(variant==6)local.posterior_stamp.nanosec=1000000000U;
    if(variant==7)local.imu_stamp.sec=0,local.imu_stamp.nanosec=0;
    if(variant==8)local.local_odometry.header.frame_id="map";
    if(variant==9)local.local_odometry.pose.pose.orientation.w=2.;
    if(variant==10)local.local_odometry.pose.pose.position.x=NAN;
    Odom out;std::string reason;
    EXPECT_FALSE(decodeLocalNavigationState(local,configuration(),out,reason))<<variant;
  }
}
TEST(NavigationStateContract, UnixSourceNanosecondsArePreservedIndependentlyOfDoubleArithmetic) {
  auto pair=state();pair.source_stamp.sec=1791002417;pair.source_stamp.nanosec=245872020U;
  pair.local_odometry.header.stamp=pair.global_odometry.header.stamp=pair.posterior_stamp=pair.imu_stamp=pair.source_stamp;
  Odom out;std::string reason;
  ASSERT_TRUE(decodeNavigationState(pair,configuration(),out,reason));
  EXPECT_EQ(out.source_stamp_ns,1791002417245872020LL);
  d1max_planning_interfaces::msg::LocalNavigationState local;
  local.schema_version=1;local.session_id=pair.session_id;local.map_version_id=pair.map_version_id;
  local.localization_epoch=pair.localization_epoch;local.localization_seed_id=pair.localization_seed_id;
  local.local_odometry=pair.local_odometry;local.source_stamp=pair.source_stamp;
  local.posterior_stamp=pair.posterior_stamp;local.imu_stamp=pair.imu_stamp;local.usable=true;
  ASSERT_TRUE(decodeLocalNavigationState(local,configuration(),out,reason));
  EXPECT_EQ(out.source_stamp_ns,1791002417245872020LL);
}
