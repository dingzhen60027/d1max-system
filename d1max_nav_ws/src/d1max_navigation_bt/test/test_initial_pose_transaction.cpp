#include "d1max_navigation_bt/initial_pose_transaction.hpp"
#include "d1max_navigation_bt/dependency_health.hpp"
#include "d1max_navigation_bt/stable_recovery.hpp"
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
TEST(InitialPoseTransaction, BindingConflictAndOldSourceCannotReplaceValidIntent) {
  auto l=ledger();auto r=request();ASSERT_TRUE(ask(l,r).accepted);
  r.body_pose.pose.pose.position.x=.01;EXPECT_FALSE(ask(l,r).accepted);
  r=request();r.request_id="other";const auto old=ask(l,r);
  EXPECT_FALSE(old.begin_retirement);EXPECT_EQ(old.reason,"initial_pose_intent_source_not_new");EXPECT_TRUE(l.blocked());
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
TEST(InitialPoseTransaction, PrepareExpiryAndLateDefiniteFactNeverRestoreOldMotion) {
  auto l=ledger();auto r=request();ask(l,r);l.expire(20.);EXPECT_FALSE(l.blocked());EXPECT_FALSE(ask(l,r).accepted);
  auto committed=ledger();r=request();ask(committed,r);r.operation=r.COMMIT;ask(committed,r);
  committed.expire(40.);EXPECT_TRUE(committed.blocked());
  auto o=outcome(r); // Retain the old exact application fact, not a new receipt-time stamp.
  EXPECT_TRUE(committed.outcome(o,130000000000LL,40.));EXPECT_TRUE(committed.blocked());
  r.operation=r.ABORT;EXPECT_FALSE(ask(committed,r).accepted);
  EXPECT_FALSE(committed.readyIdentity(1,"new","version",130000000000LL,130000000000LL,40.,true));
  EXPECT_FALSE(committed.readyIdentity(1,"new","version",130200000000LL,130200000000LL,40.2,true));
  EXPECT_TRUE(committed.readyIdentity(1,"new","version",130800000000LL,130800000000LL,40.8,true));
  EXPECT_FALSE(committed.blocked());
}

TEST(InitialPoseTransaction, NewValidSamePoseIntentSupersedesUncommittedDraftAndFencesOldReplies) {
  auto l=ledger();auto a=request();ASSERT_TRUE(ask(l,a,false,true).accepted);
  auto b=a;b.request_id="new";b.body_pose.header.stamp.nanosec=200000000;
  const auto d=ask(l,b,false,true,false,start+200000000,10.2);
  ASSERT_TRUE(d.accepted);EXPECT_TRUE(d.begin_retirement);EXPECT_FALSE(d.ready);
  a.operation=a.COMMIT;
  EXPECT_EQ(ask(l,a,true,true,true,start+200000000,10.2).reason,
            "initial_pose_superseded:prepared_unapplied");
  EXPECT_FALSE(l.outcome(outcome(a),start+200000000,10.2));
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+200000000,start+200000000,10.2,true));
  b.operation=b.COMMIT;
  EXPECT_FALSE(ask(l,b,true,true,false,start+200000000,10.2).application_authorized);
  EXPECT_TRUE(ask(l,b,true,true,true,start+200000000,10.2).application_authorized);
}

TEST(InitialPoseTransaction, AppliedButUnlocalizedSeedDoesNotBlockExplicitNewCorrection) {
  auto l=ledger();auto a=request();ask(l,a);a.operation=a.COMMIT;ask(l,a);
  ASSERT_TRUE(l.outcome(outcome(a),start+100000000,10.1));
  auto b=request();b.request_id="correction";b.body_pose.header.stamp.nanosec=200000000;
  ASSERT_TRUE(ask(l,b,true,false,false,start+200000000,10.2).accepted);
  EXPECT_EQ(ask(l,a,true,false,false,start+200000000,10.2).reason,
            "initial_pose_superseded:applied_waiting_readiness");
  EXPECT_FALSE(l.outcome(outcome(a),start+200000000,10.2));
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+200000000,start+200000000,10.2,true));
  b.operation=b.COMMIT;ASSERT_TRUE(ask(l,b,true,false,false,start+200000000,10.2).application_authorized);
  auto o=outcome(b);o.source_stamp.nanosec=300000000;o.localization_seed_id="corrected";
  ASSERT_TRUE(l.outcome(o,start+300000000,10.3));
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+400000000,start+400000000,10.4,true));
  EXPECT_FALSE(l.readyIdentity(1,"corrected","version",start+400000000,start+400000000,10.4,true));
  EXPECT_FALSE(l.readyIdentity(1,"corrected","version",start+500000000,start+500000000,10.5,true));
  EXPECT_TRUE(l.readyIdentity(1,"corrected","version",start+1100000000,start+1100000000,11.1,true));
}

TEST(InitialPoseTransaction, SupersessionAndLateAdmissionNeverForgetOriginalStopObligation) {
  auto l=ledger();auto a=request();ask(l,a,false,true,false);
  auto b=request();b.request_id="new";b.body_pose.header.stamp.nanosec=200000000;
  ASSERT_FALSE(ask(l,b,true,false,false,start+200000000,10.2).ready);
  b.operation=b.COMMIT;
  EXPECT_FALSE(ask(l,b,true,false,false,start+200000000,10.2).application_authorized);
  EXPECT_TRUE(ask(l,b,true,false,true,start+200000000,10.2).application_authorized);
  auto late=ledger();a=request();ask(late,a,false,false);a.operation=a.COMMIT;
  EXPECT_FALSE(ask(late,a,true,true,false).application_authorized);
  EXPECT_FALSE(ask(late,a,true,false,false).application_authorized);
  EXPECT_TRUE(ask(late,a,true,false,true).application_authorized);
}

TEST(InitialPoseTransaction, NewCorrectionSupersedesExpiredUnknownAndPartialWithFactsPreserved) {
  for(bool partial:{false,true}) {
    auto l=ledger();auto a=request();ask(l,a);a.operation=a.COMMIT;ask(l,a);
    if(partial) {
      auto o=outcome(a);o.accepted=false;
      ASSERT_FALSE(l.outcome(o,start+100000000,10.1));
    } else l.expire(40.);
    auto b=request();b.request_id="new";b.body_pose.header.stamp.sec=131;
    ASSERT_TRUE(ask(l,b,false,true,false,131000000000LL,41.).accepted);
    const auto old=ask(l,a,true,true,true,131000000000LL,41.);
    EXPECT_EQ(old.reason,std::string("initial_pose_superseded:authorized_application_unknown:")+
      (partial?"initial_pose_partial_application_requires_review":
               "initial_pose_matching_outcome_or_readiness_timeout_requires_review"));
    auto old_outcome=outcome(a);old_outcome.source_stamp.sec=131;
    EXPECT_FALSE(l.outcome(old_outcome,131100000000LL,41.1));
    b.operation=b.COMMIT;
    EXPECT_FALSE(ask(l,b,false,true,true,131100000000LL,41.1).application_authorized);
    EXPECT_FALSE(ask(l,b,true,true,false,131100000000LL,41.1).application_authorized);
    EXPECT_TRUE(ask(l,b,true,true,true,131100000000LL,41.1).application_authorized);
  }
}

TEST(InitialPoseTransaction, InvalidNewDraftCannotDisplaceAuthorizedPriorIntent) {
  for(unsigned scenario=0;scenario<10;++scenario) {
    auto l=ledger();auto a=request();ask(l,a);a.operation=a.COMMIT;ASSERT_TRUE(ask(l,a).application_authorized);
    auto b=request();b.request_id="new";b.body_pose.header.stamp.nanosec=200000000;
    if(scenario==0)b.schema_version=2;
    if(scenario==1)b.session_id="foreign";
    if(scenario==2)b.source_map_sha256=std::string(64,'b');
    if(scenario==3)b.request_id="invalid id";
    if(scenario==4)b.body_pose.header.frame_id="foreign";
    if(scenario==5)b.body_pose.pose.pose.orientation.w=0.;
    if(scenario==6)b.body_pose.header.stamp.sec=90;
    if(scenario==7)b.body_pose.pose.covariance[0]=-1.;
    if(scenario==8)b.body_pose.pose.pose.position.x=std::numeric_limits<double>::quiet_NaN();
    const auto d=l.request(b,start+200000000,10.2,scenario!=9,true,false,false,true,1,"old");
    EXPECT_FALSE(d.accepted);EXPECT_FALSE(d.begin_retirement);
    EXPECT_TRUE(ask(l,a,true,false,false,start+200000000,10.2).application_authorized);
  }
}

TEST(InitialPoseTransaction, InactiveOrFatalAdmissionCannotApplyExistingCommit) {
  auto l=ledger();auto r=request();ask(l,r);r.operation=r.COMMIT;
  EXPECT_FALSE(l.request(r,start,10.,false,true,false,false,true,1,"old").application_authorized);
  EXPECT_TRUE(l.blocked());EXPECT_TRUE(ask(l,r).application_authorized);
  EXPECT_TRUE(InitialPoseTransaction::admissionBlocker(true,"",false,"component_executor_stalled").empty());
  EXPECT_EQ(InitialPoseTransaction::admissionBlocker(false,"",false,""),"navigation_lifecycle_not_active");
  EXPECT_EQ(InitialPoseTransaction::admissionBlocker(true,"previous_worker_cancellation_timeout",false,""),
            "previous_worker_cancellation_timeout");
  EXPECT_EQ(InitialPoseTransaction::admissionBlocker(true,"",true,"dependency_recovery_timeout:global"),
            "dependency_recovery_timeout:global");
  EXPECT_EQ(InitialPoseTransaction::admissionBlocker(true,"",true,""),"dependency_fatal");
}

TEST(InitialPoseTransaction, OutcomeIsDurableOriginalFactNotSensorLeaseOrPartialSuccess) {
  for(unsigned scenario=0;scenario<4;++scenario) {
    auto l=ledger();auto r=request();ask(l,r);r.operation=r.COMMIT;ask(l,r);l.expire(40.);
    auto o=outcome(r);const int64_t now=130000000000LL;
    if(scenario==0)o.source_stamp.sec=99;
    if(scenario==1){o.source_stamp.sec=130;o.source_stamp.nanosec=50000001;}
    if(scenario==2)o.intent_source_stamp.nanosec=1;
    if(scenario==3)o.request_id="foreign";
    EXPECT_FALSE(l.outcome(o,now,40.));EXPECT_TRUE(l.blocked());
  }
  auto l=ledger();auto r=request();ask(l,r);r.operation=r.COMMIT;ask(l,r);l.expire(40.);
  const auto original=outcome(r);
  ASSERT_TRUE(l.outcome(original,130000000000LL,40.)); // 29.9-second delivery delay, original fact unchanged.
  EXPECT_EQ(original.source_stamp.sec,100);EXPECT_EQ(original.source_stamp.nanosec,100000000u);
  EXPECT_FALSE(l.readyIdentity(1,"new","version",start+100000000,130000000000LL,40.,true));
}

TEST(InitialPoseTransaction, BoundedTombstonesDoNotLockSessionAndHighwaterRejectsEvictedReplay) {
  auto l=ledger();auto first=request();first.request_id="intent0";
  for(unsigned i=0;i<140;++i) {
    auto r=request();r.request_id="intent"+std::to_string(i);r.body_pose.header.stamp.nanosec=i*10000000;
    // Keep nanoseconds canonical across the second boundary.
    r.body_pose.header.stamp.sec+=r.body_pose.header.stamp.nanosec/1000000000;
    r.body_pose.header.stamp.nanosec%=1000000000;
    const auto d=ask(l,r,true,false,false,start+int64_t(i)*10000000,10.+i*.01);
    ASSERT_TRUE(d.accepted)<<i<<":"<<d.reason;
  }
  EXPECT_EQ(ask(l,first,true,false,false,start+1400000000,11.4).reason,"initial_pose_intent_source_not_new");
  auto recycled=first;recycled.body_pose.header.stamp.sec=102;
  EXPECT_TRUE(ask(l,recycled,true,false,false,start+2000000000,12.).accepted);
  EXPECT_FALSE(l.outcome(outcome(first),start+2000000000,12.));
}
TEST(DependencyHealth, ShortGapWithdrawsReadinessWithoutDestroyingTheSession) {
  DependencyHealthGuard g(true);
  EXPECT_EQ(g.blocker(10.),"waiting_functional_dependencies");
  ASSERT_TRUE(g.observe(1,"session",1,true,false,"ready",100.,100.,10.,"session"));
  EXPECT_TRUE(g.blocker(10.5).empty());
  EXPECT_FALSE(g.observe(1,"session",1,true,false,"ready",100.5,100.5,10.5,"session"));
  // The old 0.6-second latching behavior killed otherwise healthy tasks.
  EXPECT_FALSE(g.fatal(10.7));EXPECT_TRUE(g.blocker(10.7).empty());
  EXPECT_EQ(g.blocker(11.6),"dependency_heartbeat_unavailable");
  EXPECT_FALSE(g.fatal(11.6));
  ASSERT_TRUE(g.observe(1,"session",2,true,false,"ready",101.7,101.7,11.7,"session"));
  EXPECT_TRUE(g.blocker(11.7).empty()); // Tree owns measured recovery, no serial window.
  ASSERT_TRUE(g.observe(1,"session",3,true,false,"ready",102.,102.,12.,"session"));
  EXPECT_TRUE(g.blocker(12.).empty());
  ASSERT_TRUE(g.observe(1,"session",4,true,false,"ready",102.4,102.4,12.4,"session"));
  EXPECT_TRUE(g.blocker(12.4).empty());EXPECT_FALSE(g.fatal(12.4));
}
TEST(DependencyHealth, RepeatedAndRewrittenSourcesCannotConfirmRecovery) {
  DependencyHealthGuard g(true);
  ASSERT_TRUE(g.observe(1,"session",1,true,false,"ready",100.,100.,10.,"session"));
  ASSERT_TRUE(g.observe(1,"session",2,false,false,"map_stalled",100.1,100.1,10.1,"session"));
  ASSERT_TRUE(g.observe(1,"session",3,true,false,"ready",100.2,100.2,10.2,"session"));
  EXPECT_FALSE(g.observe(1,"session",3,true,false,"ready",100.8,100.8,10.8,"session"));
  EXPECT_FALSE(g.observe(1,"session",4,true,false,"ready",100.2,100.8,10.8,"session"));
  EXPECT_TRUE(g.blocker(10.9).empty());
  ASSERT_TRUE(g.observe(1,"session",4,true,false,"ready",100.9,100.9,10.9,"session"));
  EXPECT_TRUE(g.blocker(10.9).empty());
  ASSERT_TRUE(g.observe(1,"session",5,true,false,"ready",101.,101.,11.,"session"));
  EXPECT_TRUE(g.blocker(11.).empty());
}
TEST(DependencyHealth, FlappingReadyReportsCannotRenewTheRecoveryDeadline) {
  DependencyHealthGuard g(true,1.5,3.,.6);
  ASSERT_TRUE(g.observe(1,"session",1,true,false,"ready",100.,100.,10.,"session"));
  ASSERT_TRUE(g.observe(1,"session",2,false,false,"map_stalled",100.1,100.1,10.1,"session"));
  for(unsigned i=3;i<=8;++i) {
    const double at=10.2+(i-3)*.5;
    ASSERT_TRUE(g.observe(1,"session",i,i%2==0,false,"temporary_wait",90.+at,90.+at,at,"session"));
    EXPECT_FALSE(g.fatal(at));
  }
  EXPECT_TRUE(g.fatal(13.1));
  EXPECT_EQ(g.blocker(13.1),"dependency_recovery_timeout:temporary_wait");
  ASSERT_TRUE(g.observe(1,"session",9,true,false,"ready",103.2,103.2,13.2,"session"));
  EXPECT_TRUE(g.fatal(13.2));
}
TEST(DependencyHealth, LateSuccessCannotHideAnUnobservedExpiredGap) {
  DependencyHealthGuard g(true,1.5,3.,.6);
  ASSERT_TRUE(g.observe(1,"session",1,true,false,"ready",100.,100.,10.,"session"));
  // No polling occurred during the gap. Its deadline begins at the original
  // lease expiry, not at receipt of the eventual replacement snapshot.
  ASSERT_TRUE(g.observe(1,"session",2,true,false,"ready",104.6,104.6,14.6,"session"));
  EXPECT_TRUE(g.fatal(14.6));
}
TEST(DependencyHealth, ColdPreparationUsesItsProducerStartupDeadline) {
  DependencyHealthGuard g(true);
  for(unsigned i=1;i<=5;++i) {
    const double at=i*10.;
    ASSERT_TRUE(g.observe(1,"session",i,false,false,"loading_map",100.+at,100.+at,at,"session"));
    EXPECT_FALSE(g.fatal(at));EXPECT_EQ(g.blocker(at),"loading_map");
  }
  ASSERT_TRUE(g.observe(1,"session",6,true,false,"ready",151.,151.,51.,"session"));
  EXPECT_TRUE(g.blocker(51.).empty());
}
TEST(DependencyHealth, PolicyMustBeFiniteBoundedAndFitItsRecoveryEpisode) {
  EXPECT_THROW(DependencyHealthGuard(true,0.,30.,.6),std::invalid_argument);
  EXPECT_THROW(DependencyHealthGuard(true,1.5,1.,.6),std::invalid_argument);
  EXPECT_THROW(DependencyHealthGuard(true,1.5,30.,.1),std::invalid_argument);
  EXPECT_THROW(DependencyHealthGuard(true,1.5,30.,std::numeric_limits<double>::infinity()),std::invalid_argument);
}
TEST(DependencyHealth, MalformedForeignFutureAndFatalAreNotReadiness) {
  DependencyHealthGuard g(true);
  EXPECT_FALSE(g.observe(1,"other",1,true,false,"ready",100.,100.,10.,"session"));
  EXPECT_FALSE(g.observe(1,"session",1,true,false,"ready",101.,100.,10.,"session"));
  ASSERT_TRUE(g.observe(1,"session",1,false,true,"tracker_no_progress",100.,100.,10.,"session"));
  EXPECT_EQ(g.blocker(10.),"dependency_fault:tracker_no_progress");
}

TEST(DependencyHealth, CommittedRouteIgnoresOnlyGlobalAndNewComputeRestoresDependency) {
  DependencyHealthGuard g(true);
  std::vector<FunctionalComponent>parts{{"global",true,true,2.,98.},
    {"reference",true,true,0.,100.},{"tracker",true,true,0.,100.},
    {"map",false,false,0.,0.}};
  ASSERT_TRUE(g.observe(1,"session",1,false,false,"component_executor_stalled:global",
    100.,100.,10.,"session",parts,false));
  EXPECT_TRUE(g.blocker(10.,false).empty());
  EXPECT_EQ(g.blocker(10.,true),"component_executor_stalled:global");
  EXPECT_TRUE(g.blocker(10.,false).empty());
  // A frozen global producer cannot kill a committed route at 30 seconds.
  for(unsigned i=2;i<180;++i) {
    const double dt=i*.2;
    parts[0].callback_age=2.+dt;
    parts[1].source=parts[2].source=100.+dt;
    ASSERT_TRUE(g.observe(1,"session",i,false,false,"component_executor_stalled:global",
      100.+dt,100.+dt,10.+dt,"session",parts,false));
    ASSERT_TRUE(g.blocker(10.+dt,false).empty());
    ASSERT_FALSE(g.fatal(10.+dt,false));
  }
  EXPECT_EQ(g.blocker(45.81,true),"component_executor_stalled:global");
}
TEST(DependencyHealth, CommittedRouteNeverIgnoresOtherRequiredComponentsOrFatal) {
  for(const auto&dead:{"reference","tracker","safety"}) {
    DependencyHealthGuard g(true);
    std::vector<FunctionalComponent>parts{{"global",true,true,3.,97.},
      {"reference",true,true,0.,100.},{"tracker",true,true,0.,100.},{"safety",true,true,0.,100.}};
    for(auto&p:parts)if(p.name==dead){p.callback_age=2.;p.source=98.;}
    ASSERT_TRUE(g.observe(1,"session",1,true,false,"",100.,100.,10.,"session",parts,false));
    EXPECT_EQ(g.blocker(10.,false),std::string("component_executor_stalled:")+dead);
  }
  DependencyHealthGuard g(true);
  ASSERT_TRUE(g.observe(1,"session",1,true,true,"global_process_exited",100.,100.,10.,"session",
    {{"global",true,true,2.,98.},{"reference",true,true,0.,100.}},false));
  EXPECT_EQ(g.blocker(10.,false),"dependency_fault:global_process_exited");
}
TEST(DependencyHealth, DeadMonitorAndMalformedComponentsCannotBecomePhaseReadiness) {
  DependencyHealthGuard g(true);
  EXPECT_FALSE(g.observe(1,"session",1,true,false,"",100.,100.,10.,"session",
    {{"reference",false,true,0.,100.}},false));
  EXPECT_FALSE(g.observe(1,"session",1,true,false,"",100.,100.,10.,"session",
    {{"reference",true,true,-1.,100.}},false));
  EXPECT_FALSE(g.observe(1,"session",1,true,false,"",100.,100.,10.,"session",
    {{"reference",true,true,0.,102.}},false));
  ASSERT_TRUE(g.observe(1,"session",1,true,false,"",100.,100.,10.,"session",
    {{"global",true,true,2.,98.},{"reference",true,true,0.,100.}},false));
  EXPECT_EQ(g.blocker(11.6,false),"dependency_heartbeat_unavailable");
  EXPECT_TRUE(g.fatal(41.6,false));
}
TEST(DependencyHealth, ComponentAgeIncludesMonitorDeliveryAndSubsequentResidence) {
  DependencyHealthGuard g(true);
  ASSERT_TRUE(g.observe(1,"session",1,true,false,"",100.,100.8,10.,"session",
    {{"global",true,true,0.,100.},{"reference",true,true,.8,99.2}},false));
  EXPECT_EQ(g.blocker(10.,false),"component_executor_stalled:reference");
}
TEST(DependencyHealth, TreeAndFunctionalRecoveryRunConcurrentlyOnDistinctNewEvidence) {
  DependencyHealthGuard health(true);StableRecoveryGate tree;
  ASSERT_TRUE(health.observe(1,"session",1,true,false,"",100.,100.,10.,"session"));
  ASSERT_TRUE(tree.observe(health.blocker(10.).empty(),10.,100,"epoch"));
  ASSERT_TRUE(health.observe(1,"session",2,false,false,"tracker_stalled",100.1,100.1,10.1,"session"));
  ASSERT_FALSE(tree.observe(health.blocker(10.1).empty(),10.1,100,"epoch"));
  for(unsigned i=0;i<3;++i) {
    const double t=10.2+i*.31;
    ASSERT_TRUE(health.observe(1,"session",3+i,true,false,"",90.+t,90.+t,t,"session"));
    EXPECT_EQ(tree.observe(health.blocker(t).empty(),t,101+i,"epoch"),i==2);
  }
  // All recovery was 0.62 s, not monitor 0.6 + dependency 0.6 + tree 0.6.
  EXPECT_TRUE(tree.observe(health.blocker(10.83).empty(),10.83,103,"epoch"));
}
TEST(DependencyHealth, ReadyHeartbeatsDoNotReplaceNewMeasuredRecoverySamples) {
  DependencyHealthGuard health(true);StableRecoveryGate tree;
  ASSERT_TRUE(health.observe(1,"session",1,true,false,"",100.,100.,10.,"session"));
  ASSERT_TRUE(tree.observe(true,10.,100,"epoch"));
  ASSERT_FALSE(tree.observe(false,10.1,100,"epoch"));
  for(unsigned i=0;i<6;++i) {
    const double t=10.2+i*.2;
    ASSERT_TRUE(health.observe(1,"session",2+i,true,false,"",90.+t,90.+t,t,"session"));
    EXPECT_FALSE(tree.observe(health.blocker(t).empty(),t,100,"epoch"));
  }
}
TEST(TaskAdmission, QuarantineRetirementAndSeedGateApplyBeforeReservingGoal) {
  EXPECT_EQ(taskAdmissionBlocker(true,"stop_unconfirmed",false,"",""),"stop_unconfirmed");
  EXPECT_EQ(taskAdmissionBlocker(true,"",true,"",""),"retiring_previous_task");
  EXPECT_EQ(taskAdmissionBlocker(true,"",false,"matching_seed",""),"matching_seed");
  EXPECT_TRUE(taskAdmissionBlocker(true,"",false,"","").empty());
}

TEST(TaskAdmission, ReplacementIntentIsAcceptedDuringNormalDrainWithoutAuthorizingWork) {
  EXPECT_EQ(taskAdmissionBlocker(true,"",true,"",""),"retiring_previous_task");
  EXPECT_TRUE(replacementGoalAdmissionBlocker(true,"","",false,"").empty());
  EXPECT_EQ(replacementGoalAdmissionBlocker(false,"","",false,""),"lifecycle_not_active");
  EXPECT_EQ(replacementGoalAdmissionBlocker(true,"previous_worker_cancellation_timeout","",false,""),
    "previous_worker_cancellation_timeout");
  EXPECT_TRUE(replacementGoalAdmissionBlocker(true,"previous_physical_stop_unconfirmed","",false,"").empty());
  // Only latest target intent is accepted; the normal admission path still
  // blocks confirmation/work until real measured stop evidence is received.
  EXPECT_EQ(taskAdmissionBlocker(true,"previous_physical_stop_unconfirmed",true,"",""),
    "previous_physical_stop_unconfirmed");
  EXPECT_EQ(replacementGoalAdmissionBlocker(true,"","matching_seed",false,""),"matching_seed");
  EXPECT_TRUE(replacementGoalAdmissionBlocker(true,"","",false,"waiting_functional_dependencies").empty());
  EXPECT_TRUE(replacementGoalAdmissionBlocker(true,"","",false,"dependency_heartbeat_unavailable").empty());
  EXPECT_TRUE(replacementGoalAdmissionBlocker(true,"","",false,"confirming_dependency_recovery").empty());
  EXPECT_EQ(replacementGoalAdmissionBlocker(true,"","",true,"dependency_fault:worker_exit"),
    "dependency_fault:worker_exit");
}
