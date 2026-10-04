#include "d1max_navigation_bt/initial_pose_transaction.hpp"
#include "d1max_navigation_bt/dependency_health.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace d1max_navigation_bt;
namespace {
constexpr int64_t start=100000000000LL;
auto request() {
  InitialPoseTransaction::Request r;r.schema_version=1;r.operation=r.PREPARE;
  r.session_id="session";r.request_id="intent";r.source_map_sha256=std::string(64,'a');
  r.body_pose.header.frame_id="map";r.body_pose.header.stamp.sec=100;
  r.body_pose.pose.pose.position.z=.5;r.body_pose.pose.pose.orientation.w=1.;return r;
}
auto ledger(){return InitialPoseTransaction("session","map",std::string(64,'a'));}
auto ask(InitialPoseTransaction&l,const InitialPoseTransaction::Request&r,bool retired=true,
    bool physical=false,bool stopped=false,int64_t now=start,double mono=10.) {
  return l.request(r,now,mono,true,retired,physical,stopped,true,1,"old");
}
auto outcome(const InitialPoseTransaction::Request&r) {
  InitialPoseTransaction::Outcome o;o.schema_version=1;o.session_id="session";o.request_id=r.request_id;
  o.intent_source_stamp=r.body_pose.header.stamp;o.source_stamp.sec=100;o.source_stamp.nanosec=100000000;
  o.map_version_id="version";o.localization_epoch=1;o.localization_seed_id="new";o.accepted=o.applied=true;return o;
}
}
TEST(InitialPoseTransaction, InvalidDraftDoesNotRetireOrBlock) {
  for(unsigned scenario=0;scenario<5;++scenario) {
    auto l=ledger();auto r=request();
    if(scenario==0)r.body_pose.header.frame_id="foreign";
    if(scenario==1)r.body_pose.header.stamp.sec=90;
    if(scenario==2)r.body_pose.pose.pose.orientation.w=0.;
    if(scenario==3)r.body_pose.pose.covariance[0]=-1.;
    if(scenario==4)r.body_pose.pose.pose.position.x=std::numeric_limits<double>::quiet_NaN();
    const auto d=ask(l,r);EXPECT_FALSE(d.accepted);EXPECT_FALSE(d.begin_retirement);EXPECT_FALSE(l.blocked());
  }
}
TEST(InitialPoseTransaction, ValidPrepareRetiresOnceAndCannotSkipActualStop) {
  auto l=ledger();auto r=request();auto d=ask(l,r,false,true);
  ASSERT_TRUE(d.accepted);EXPECT_TRUE(d.begin_retirement);EXPECT_FALSE(d.ready);
  d=ask(l,r,true,true);EXPECT_FALSE(d.begin_retirement);EXPECT_FALSE(d.ready);
  r.operation=r.COMMIT;EXPECT_FALSE(ask(l,r,true,true).application_authorized);
  d=ask(l,r,true,true,true);EXPECT_TRUE(d.ready);EXPECT_TRUE(d.application_authorized);EXPECT_TRUE(d.physical_stop_confirmed);
  EXPECT_TRUE(l.blocked());
}
TEST(InitialPoseTransaction, BindingConflictAndConcurrentIntentCannotCancelAgain) {
  auto l=ledger();auto r=request();ASSERT_TRUE(ask(l,r).accepted);
  r.body_pose.pose.pose.position.x=.01;EXPECT_FALSE(ask(l,r).accepted);
  r=request();r.request_id="other";EXPECT_FALSE(ask(l,r).begin_retirement);EXPECT_TRUE(l.blocked());
}
TEST(InitialPoseTransaction, PreviewRetirementIsNotPhysicalStopProof) {
  auto l=ledger();auto r=request();ASSERT_TRUE(ask(l,r).ready);r.operation=r.COMMIT;
  const auto d=ask(l,r);EXPECT_TRUE(d.application_authorized);EXPECT_FALSE(d.physical_stop_confirmed);
}
TEST(InitialPoseTransaction, UnrelatedNewSeedOrForeignOutcomeCannotReleaseAdmission) {
  auto l=ledger();auto r=request();ASSERT_TRUE(ask(l,r).accepted);r.operation=r.COMMIT;ASSERT_TRUE(ask(l,r).application_authorized);
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+100000000,start+100000000,10.1,true));
  auto o=outcome(r);o.request_id="foreign";EXPECT_FALSE(l.outcome(o,start+100000000,10.1));
  o=outcome(r);o.intent_source_stamp.nanosec=1;EXPECT_FALSE(l.outcome(o,start+100000000,10.1));
  o=outcome(r);o.localization_seed_id="old";EXPECT_FALSE(l.outcome(o,start+100000000,10.1));
  EXPECT_TRUE(l.blocked());
}
TEST(InitialPoseTransaction, MatchingAppliedWaitsStableDistinctRealSamples) {
  auto l=ledger();auto r=request();ask(l,r);r.operation=r.COMMIT;ask(l,r);
  ASSERT_TRUE(l.outcome(outcome(r),start+100000000,10.1));
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+100000000,start+100000000,10.1,true));
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+200000000,start+200000000,10.2,true));
  EXPECT_TRUE(l.readyIdentity(1,"new","version",start+800000000,start+800000000,10.8,true));
  EXPECT_FALSE(l.blocked());EXPECT_FALSE(ask(l,r).application_authorized);
}
TEST(InitialPoseTransaction, BrokenReadinessResetsWindowAndDuplicateDoesNotCount) {
  auto l=ledger();auto r=request();ask(l,r);r.operation=r.COMMIT;ask(l,r);l.outcome(outcome(r),start+100000000,10.1);
  l.readyIdentity(1,"new","version",start+100000000,start+100000000,10.1,true);
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+100000000,start+300000000,10.3,true));
  l.readyIdentity(1,"wrong","version",start+400000000,start+400000000,10.4,true);
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+800000000,start+800000000,10.8,true));
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+1000000000,start+1000000000,11.,true));
  EXPECT_TRUE(l.readyIdentity(1,"new","version",start+1500000000,start+1500000000,11.5,true));
}
TEST(InitialPoseTransaction, KnownRejectionDiffersFromPartialApplication) {
  auto l=ledger();auto r=request();ask(l,r);r.operation=r.COMMIT;ask(l,r);
  auto o=outcome(r);o.accepted=o.applied=false;ASSERT_TRUE(l.outcome(o,start+100000000,10.1));EXPECT_FALSE(l.blocked());
  auto partial=ledger();r=request();ask(partial,r);r.operation=r.COMMIT;ask(partial,r);
  o=outcome(r);o.accepted=false;ASSERT_FALSE(partial.outcome(o,start+100000000,10.1));
  EXPECT_EQ(partial.blocker(),"initial_pose_partial_application_requires_review");
}
TEST(InitialPoseTransaction, PrepareExpiryAndPostCommitUnknownResultNeverRevive) {
  auto l=ledger();auto r=request();ask(l,r);l.expire(20.);EXPECT_FALSE(l.blocked());EXPECT_FALSE(ask(l,r).accepted);
  auto committed=ledger();r=request();ask(committed,r);r.operation=r.COMMIT;ask(committed,r);
  committed.expire(40.);EXPECT_TRUE(committed.blocked());
  auto o=outcome(r);o.source_stamp.sec=130;
  EXPECT_FALSE(committed.outcome(o,130000000000LL,40.));EXPECT_TRUE(committed.blocked());
  r.operation=r.ABORT;EXPECT_FALSE(ask(committed,r).accepted);
}
TEST(DependencyHealth, ReplayCannotRenewAndTimeoutLatches) {
  DependencyHealthGuard g(true);
  EXPECT_EQ(g.blocker(10.),"waiting_functional_dependencies");
  ASSERT_TRUE(g.observe(1,"session",1,true,false,"ready",100.,100.,10.,"session"));
  EXPECT_TRUE(g.blocker(10.5).empty());
  EXPECT_FALSE(g.observe(1,"session",1,true,false,"ready",100.5,100.5,10.5,"session"));
  EXPECT_TRUE(g.fatal(10.7));
  g.observe(1,"session",2,true,false,"ready",100.7,100.7,10.7,"session");
  EXPECT_FALSE(g.blocker(10.7).empty());
}
TEST(DependencyHealth, MalformedForeignFutureAndFatalAreNotReadiness) {
  DependencyHealthGuard g(true);
  EXPECT_FALSE(g.observe(1,"other",1,true,false,"ready",100.,100.,10.,"session"));
  EXPECT_FALSE(g.observe(1,"session",1,true,false,"ready",101.,100.,10.,"session"));
  ASSERT_TRUE(g.observe(1,"session",1,false,true,"tracker_no_progress",100.,100.,10.,"session"));
  EXPECT_EQ(g.blocker(10.),"dependency_fault:tracker_no_progress");
}
TEST(TaskAdmission, QuarantineRetirementAndSeedGateApplyBeforeReservingGoal) {
  EXPECT_EQ(taskAdmissionBlocker(true,"stop_unconfirmed",false,"",""),"stop_unconfirmed");
  EXPECT_EQ(taskAdmissionBlocker(true,"",true,"",""),"retiring_previous_task");
  EXPECT_EQ(taskAdmissionBlocker(true,"",false,"matching_seed",""),"matching_seed");
  EXPECT_TRUE(taskAdmissionBlocker(true,"",false,"","").empty());
}
