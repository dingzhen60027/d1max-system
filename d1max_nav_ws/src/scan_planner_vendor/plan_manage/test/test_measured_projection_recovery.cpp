#include "native_local_follower_test_fixture.hpp"

namespace {
void rejectActualConnection(Integrated& f, double now = 100.10) {
  ASSERT_TRUE(f.command(100.05,.005,.1,1));
  auto actual=body(now,.01,.1);actual.position.y()=.20;
  ASSERT_TRUE(f.follower.body(actual,now,now-90.));
  geometry_msgs::msg::Twist twist;twist.linear.x=.1;
  f.native.body({actual.position,actual.orientation,now,"odom"},twist,actual.source_stamp_ns);
  GridMapTestAccess::acquire(f.map,actual.source_stamp_ns);
  const auto proof=Access::checkCurve(f.native,f.map,actual.source_stamp_ns,true);
  ASSERT_TRUE(proof);ASSERT_FALSE(proof->valid);
  ASSERT_EQ(proof->reason,"measured_projection_or_connection_invalid");
  ASSERT_TRUE(f.follower.validation(*proof));
  // A negative native verdict makes the follower hold; this status is not an
  // endpoint and cannot enter the old !holding deviation-reseed branch.
  auto progress=f.progress(now);progress.holding=true;progress.reason="waiting_native_validation";
  f.native.progressAt(progress,now);
}
}

TEST(MeasuredProjectionRecovery,HoldingAfterCurrentConnectionFailureRequestsActualBodyOnce) {
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  rejectActualConnection(f);
  const auto request=f.native.consumeFormalReseedAt(100100000000LL);
  ASSERT_TRUE(request);EXPECT_TRUE(request->actual_body);EXPECT_FALSE(request->terminal);
  EXPECT_EQ(request->reason,"measured_current_connection_changed");
  EXPECT_EQ(request->trajectory_id,1);EXPECT_EQ(request->version,version());
  EXPECT_EQ(request->body_source_stamp,t::stamp(100.10));
  EXPECT_FALSE(f.native.consumeFormalReseedAt(100110000000LL));
  EXPECT_EQ(f.native.committedTrajectory(),1);
  const auto proof=Access::checkCurve(f.native,f.map,100110000000LL,true);
  ASSERT_TRUE(proof);EXPECT_FALSE(proof->valid);
}

TEST(MeasuredProjectionRecovery,OldSourceAndCanceledTaskCannotRequestRecovery) {
  for(bool cancel:{false,true}) {
    Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
    rejectActualConnection(f);
    if(cancel){auto barrier=version();++barrier.reference_generation;ASSERT_TRUE(f.native.cancelReference(barrier));}
    EXPECT_FALSE(f.native.consumeFormalReseedAt(cancel?100100000000LL:100600000000LL));
  }
}

TEST(MeasuredProjectionRecovery,OccupiedVerdictIsNotReinterpretedAsProjectionFailure) {
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  ASSERT_TRUE(f.command(100.05,.005,.1,1));
  f.acquire(100.1,.01,.1);GridMapTestAccess::voxel(f.map,{.2,0.,.55},2.);
  const auto proof=Access::checkCurve(f.native,f.map,100100000000LL,true);
  ASSERT_TRUE(proof);ASSERT_FALSE(proof->valid);ASSERT_EQ(proof->reason,"occupied");
  auto progress=f.progress(100.1);progress.holding=true;progress.reason="waiting_native_validation";
  f.native.progressAt(progress,100.1);
  EXPECT_FALSE(f.native.consumeFormalReseedAt(100100000000LL));
  EXPECT_EQ(f.native.committedTrajectory(),1);
}

TEST(MeasuredProjectionRecovery,ExistingTerminalReseedKeepsPriorityAndIsNotDuplicated) {
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  rejectActualConnection(f);
  // A later actually acquired state is required. Reusing the earlier progress
  // source stamp would correctly reject this status as duplicate/relabeling.
  auto actual=body(100.11,.01,.1);actual.position.y()=.20;
  ASSERT_TRUE(f.follower.body(actual,100.11,10.11));
  geometry_msgs::msg::Twist twist;twist.linear.x=.1;
  f.native.body({actual.position,actual.orientation,100.11,"odom"},twist,actual.source_stamp_ns);
  auto progress=f.progress(100.11);ASSERT_TRUE(progress.valid);progress.holding=true;
  progress.reason="local_segment_finished_waiting_replan";
  f.native.progressAt(progress,100.11);
  const auto view=f.native.committedView();ASSERT_TRUE(view&&view->progress);
  ASSERT_EQ(view->progress->header.stamp,t::stamp(100.11));
  ASSERT_EQ(view->progress->reason,progress.reason);
  const auto request=f.native.consumeFormalReseedAt(100110000000LL);
  ASSERT_TRUE(request);EXPECT_TRUE(request->terminal);EXPECT_FALSE(request->actual_body);
  EXPECT_EQ(request->reason,"measured_local_horizon_finished");
  EXPECT_FALSE(f.native.consumeFormalReseedAt(100120000000LL));
  EXPECT_EQ(f.native.committedTrajectory(),1);
}
