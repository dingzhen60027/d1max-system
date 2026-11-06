#include "native_local_follower_test_fixture.hpp"
#include <plan_manage/local_navigation_body_input.hpp>
#include <d1max_trajectory_tracker/navigation_state_contract.hpp>

namespace {
w::LocalNavigationState localState(double source,double x=0.,double speed=.2,double lease=.1) {
  w::LocalNavigationState s;s.schema_version=2;s.session_id="session";s.map_version_id="map";
  s.localization_epoch=1;s.localization_seed_id="seed";s.usable=true;
  s.source_stamp=s.posterior_stamp=s.imu_stamp=t::stamp(source);s.valid_until=t::stamp(source+lease);
  s.local_odometry.header.stamp=s.source_stamp;s.local_odometry.header.frame_id="odom";
  s.local_odometry.child_frame_id="base_link";s.local_odometry.pose.pose.position.x=x;
  s.local_odometry.pose.pose.position.z=.55;s.local_odometry.pose.pose.orientation.w=1.;
  s.local_odometry.twist.twist.linear.x=speed;return s;
}
void typedAcquire(Integrated& f,double source,double x,double speed,double received) {
  const auto s=localState(source,x,speed);
  EXPECT_TRUE(f.follower.body(body(source,x,speed),received,received-90.));
  f.native.localBodyAt(s,t::timeNs(t::stamp(received)));
  GridMapTestAccess::acquire(f.map,t::timeNs(s.source_stamp));
  f.pipeline_health.source_ns={t::timeNs(s.source_stamp),t::timeNs(s.source_stamp)};
  f.pipeline_health.fusion_sequence=f.map.fusionTiming().sequence;
  f.pipeline_health.fusion_completed_steady_ns=f.pipeline_health.sampled_steady_ns=t::timeNs(t::stamp(received));
}
void enableAndWarm(Integrated& f) {
  f.native.configureLocalBodyInput("session","map","base_link");
  for(unsigned i=1;i<20;++i) {
    const double source=100.+i*.05;
    typedAcquire(f,source,(source-100.)*.2,.2,source+.002);
    EXPECT_TRUE(f.command(source+.006,(source-100.)*.2,.2,i,true,false));
  }
}
}

TEST(NativeLocalBodyInput, PreservesProviderSourceIdentityDeadlineAndRotatesChildTwistExactlyOnce) {
  scan_planner::LocalNavigationBodyInput input;input.configure("session","map","odom","base_link");
  auto s=localState(100.123456789,1.,.2,.06);
  s.local_odometry.pose.pose.orientation.w=s.local_odometry.pose.pose.orientation.z=std::sqrt(.5);
  s.local_odometry.twist.twist.angular.x=.3;
  scan_planner::LocalBodySample out;
  ASSERT_EQ(input.accept(s,100130000000LL,out),scan_planner::LocalNavigationBodyInput::Result::Accepted);
  EXPECT_EQ(out.source_ns,100123456789LL);EXPECT_EQ(out.valid_until_ns,100183456789LL);
  EXPECT_TRUE(out.identity.matches(version()));EXPECT_DOUBLE_EQ(out.pose.position.x(),1.);
  EXPECT_NEAR(out.world_twist.linear.x,0.,1e-9);EXPECT_NEAR(out.world_twist.linear.y,.2,1e-9);
  EXPECT_NEAR(out.world_twist.angular.y,.3,1e-9);
  EXPECT_EQ(input.accept(s,100140000000LL,out),scan_planner::LocalNavigationBodyInput::Result::Ignore);
}

TEST(NativeLocalBodyInput, ForeignOldFutureExpiredAndMalformedStatesCannotRenewSource) {
  using Result=scan_planner::LocalNavigationBodyInput::Result;
  scan_planner::LocalNavigationBodyInput input;input.configure("session","map","odom","base_link");
  scan_planner::LocalBodySample out;auto s=localState(100.);
  ASSERT_EQ(input.accept(s,100000000000LL,out),Result::Accepted);
  auto foreign=localState(101.);foreign.map_version_id="foreign";
  EXPECT_EQ(input.accept(foreign,101000000000LL,out),Result::Ignore);
  auto future=localState(5000.);EXPECT_EQ(input.accept(future,100020000000LL,out),Result::Invalid);
  s=localState(100.02);EXPECT_EQ(input.accept(s,100030000000LL,out),Result::Accepted);
  auto fault=localState(100.04);fault.usable=false;
  EXPECT_EQ(input.accept(fault,100050000000LL,out),Result::Invalid);
  EXPECT_EQ(input.accept(s,100050000000LL,out),Result::Ignore);
  auto expired=localState(100.06,.1,.2,.03);
  EXPECT_EQ(input.accept(expired,100100000000LL,out),Result::Invalid);
  auto reset=localState(99.);reset.localization_epoch=2;reset.localization_seed_id="reset";
  EXPECT_EQ(input.accept(reset,99000000000LL,out),Result::Accepted);
  s=localState(101.);EXPECT_EQ(input.accept(s,101000000000LL,out),Result::Ignore);
}

TEST(NativeLocalBodyInput, MatchesFormalFollowerProviderContractAtOriginalSource) {
  for(unsigned variant=0;variant<12;++variant) {
    SCOPED_TRACE(variant);
    auto state=localState(100.);
    if(variant==1)state.posterior_stamp=t::stamp(99.599);
    if(variant==2)state.imu_stamp=t::stamp(99.899);
    if(variant==3)state.extrapolation_sec=.101,state.imu_stamp=t::stamp(99.899);
    if(variant==4)state.valid_until=t::stamp(100.101);
    if(variant==5)state.extrapolation_sec=.05; // source-IMU mismatch
    if(variant==6)state.local_odometry.child_frame_id="lidar";
    if(variant==7)state.local_odometry.pose.pose.orientation.w=1.01;
    if(variant==8)state.local_odometry.twist.twist.linear.z=NAN;
    if(variant==9)state.local_odometry.header.stamp=t::stamp(100.001);
    if(variant==10)state.posterior_stamp=t::stamp(99.6); // exact valid boundary
    if(variant==11)state.imu_stamp=t::stamp(99.9),state.extrapolation_sec=.1;
    scan_planner::LocalNavigationBodyInput input;input.configure("session","map","odom","base_link");
    scan_planner::LocalBodySample sample;t::Odom follower;std::string reason;
    const bool native_ok=input.accept(state,100000000000LL,sample)==
      scan_planner::LocalNavigationBodyInput::Result::Accepted;
    EXPECT_EQ(native_ok,t::decodeLocalNavigationState(state,config(),follower,reason))<<reason;
  }
}

TEST(NativeLocalBodyInput, ReferenceRelayTwentyFiftyEightyMillisecondsBehindCannotInterruptCommands) {
  for(const double relay_lag:{.02,.05,.08}) {
    SCOPED_TRACE(relay_lag);Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    enableAndWarm(f);
    for(unsigned i=0;i<20;++i) {
      const double source=101.+i*.05,x=(source-100.)*.2;
      typedAcquire(f,source,x,.2,source+.002);
      // The OLD reference/FSM relay can arrive late, even with future receipt
      // time. It is planning input, never a second independent body authority.
      geometry_msgs::msg::Twist old_twist;old_twist.linear.x=.2;
      f.native.bodyAt({{x-relay_lag*.2,0.,.55},Eigen::Quaterniond::Identity(),source-relay_lag,"odom"},
        old_twist,t::timeNs(t::stamp(source-relay_lag)),t::timeNs(t::stamp(source+.003)));
      EXPECT_NEAR(Access::bodySource(f.native),source,1e-9);
      auto command=f.command(source+.006,x,.2,i+20,true,false);
      ASSERT_TRUE(command);EXPECT_FALSE(command->hold);EXPECT_GT(command->velocity.linear.x,0.);
      EXPECT_EQ(command->body_source_stamp,t::stamp(source));
      EXPECT_LE(t::timeNs(command->valid_until),t::timeNs(t::stamp(source+.1)));
      // The same command still covers a 25 ms SDK phase, without restamping.
      EXPECT_GT(t::timeNs(command->valid_until),t::timeNs(t::stamp(source+.025)));
    }
    EXPECT_FALSE(Access::checkDemand(f.native,f.map,t::timeNs(t::stamp(102.051))));
  }
}

TEST(NativeLocalBodyInput, PreviousForwardedBodyChainDefersAnOtherwiseFreshMotionDemand) {
  for(const double relay_lag:{.02,.05,.08}) {
    SCOPED_TRACE(relay_lag);
    Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    // This is the existing legacy/raw production input, not a second map or
    // controller. Reproduce the previous formal connection using it.
    for(unsigned i=1;i<20;++i) {
      const double source=100.+i*.05;
      ASSERT_TRUE(f.command(source,(source-100.)*.2,.2,i));
    }
    const double source=101.03,old_source=source-relay_lag;
    ASSERT_TRUE(f.follower.body(body(source,.206,.2),source+.002,source-90.+.002));
    geometry_msgs::msg::Twist twist;twist.linear.x=.2;
    f.native.bodyAt({{(old_source-100.)*.2,0.,.55},Eigen::Quaterniond::Identity(),old_source,"odom"},
      twist,t::timeNs(t::stamp(old_source)),t::timeNs(t::stamp(source+.002)));
    auto grant=lease(source+.002,20);ASSERT_TRUE(f.follower.lease(grant,source+.002));
    ASSERT_TRUE(f.native.taskLeaseAt(grant,t::timeNs(t::stamp(source+.002))));
    const auto demand=f.follower.step(source+.002,source-90.+.002);
    ASSERT_FALSE(demand.hold);std::optional<w::LocalMotionCommand> delivered;
    f.native.demandAt(demand,t::timeNs(t::stamp(source+.002)),[&](const auto& c){delivered=c;});
    EXPECT_FALSE(delivered);EXPECT_TRUE(Access::pendingDemand(f.native));
    // At 6 ms source age this intent is still fresh, yet its independent
    // Native pose came through the slower reference-window process.
    EXPECT_LT(t::timeNs(t::stamp(source+.006)),t::timeNs(demand.valid_until));
    EXPECT_FALSE(Access::checkDemand(f.native,f.map,t::timeNs(t::stamp(source+.006))));
  }
}

TEST(NativeLocalBodyInput, SourceArrivalCompletesPendingDemandButDelayedRelayNeverDoes) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));enableAndWarm(f);
  const double source=100.98;const auto o=body(source,.196,.2);
  ASSERT_TRUE(f.follower.body(o,source+.002,source-90.+.002));
  auto grant=lease(source+.002,20);ASSERT_TRUE(f.follower.lease(grant,source+.002));
  ASSERT_TRUE(f.native.taskLeaseAt(grant,t::timeNs(t::stamp(source+.002))));
  const auto demand=f.follower.step(source+.002,source-90.+.002);
  ASSERT_FALSE(demand.hold);std::optional<w::LocalMotionCommand> delivered;
  f.native.demandAt(demand,t::timeNs(t::stamp(source+.002)),[&](const auto& c){delivered=c;});
  EXPECT_FALSE(delivered);ASSERT_TRUE(Access::pendingDemand(f.native));
  geometry_msgs::msg::Twist twist;twist.linear.x=.2;
  f.native.bodyAt({o.position,o.orientation,o.stamp,"odom"},twist,o.source_stamp_ns,
    t::timeNs(t::stamp(source+.003)),[&](const auto& c){delivered=c;});
  EXPECT_FALSE(delivered);EXPECT_TRUE(Access::pendingDemand(f.native));
  f.native.localBodyAt(localState(source,.196,.2),t::timeNs(t::stamp(source+.004)),
    [&](const auto& c){delivered=c;});
  ASSERT_TRUE(delivered);EXPECT_EQ(delivered->sequence,demand.sequence);
  EXPECT_EQ(delivered->source_stamp,demand.source_stamp);
  EXPECT_EQ(delivered->body_source_stamp,demand.body_source_stamp);
  EXPECT_FALSE(Access::pendingDemand(f.native));
}

TEST(NativeLocalBodyInput, UnusableAndResetSourcesCannotReviveOldPendingTask) {
  for(const bool reset:{false,true}) {
    SCOPED_TRACE(reset);Integrated f;enableAndWarm(f);
    auto state=localState(100.98,.196,.2);
    if(reset){state.localization_epoch=2;state.localization_seed_id="seed2";}else state.usable=false;
    f.native.localBodyAt(state,100980000000LL);
    EXPECT_FALSE(f.native.latestPlanningBodyAt(version(),100981000000LL,.4));
    EXPECT_FALSE(Access::checkDemand(f.native,f.map,100981000000LL));
    // Neither an old typed source nor a restamped bare pose restores the
    // retired source context. No new task/lease is invented in this adapter.
    f.native.localBodyAt(localState(100.95),100985000000LL);
    geometry_msgs::msg::Twist twist;twist.linear.x=.2;
    f.native.bodyAt({{.2,0.,.55},Eigen::Quaterniond::Identity(),100.99,"odom"},
      twist,100990000000LL,100990000000LL);
    EXPECT_FALSE(Access::checkDemand(f.native,f.map,100990000000LL));
  }
}

TEST(NativeLocalBodyInput, ShorterOriginalProviderLeaseIsNotReplacedWithHundredMilliseconds) {
  Integrated f;enableAndWarm(f);
  f.native.localBodyAt(localState(101.,.2,.2,.03),101001000000LL);
  EXPECT_TRUE(f.native.latestPlanningBodyAt(version(),101020000000LL,.4));
  EXPECT_FALSE(f.native.latestPlanningBodyAt(version(),101030000000LL,.4));
  // Repeated delivery cannot issue a new lease at the later receipt time.
  f.native.localBodyAt(localState(101.,.2,.2,.03),101040000000LL);
  EXPECT_FALSE(f.native.latestPlanningBodyAt(version(),101040000000LL,.4));
}
