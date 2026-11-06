#include "native_local_follower_test_fixture.hpp"
#include <future>
#include <condition_variable>
using namespace std::chrono_literals;

namespace {
void warmDelivery(Integrated& f) {
  for(unsigned i=1;i<20;++i)ASSERT_TRUE(f.command(100.+i*.05,i*.01,.2,i));
}
w::MotionDemand nextDemand(Integrated& f,double now,unsigned sequence) {
  f.acquire(now,(now-100.)*.2,.2);
  f.native.progressAt(f.progress(now),now);
  auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)),true);
  EXPECT_TRUE(proof&&proof->valid);if(proof)EXPECT_TRUE(f.follower.validation(*proof));
  const auto grant=lease(now,sequence);
  EXPECT_TRUE(f.native.taskLeaseAt(grant,t::timeNs(t::stamp(now))));
  EXPECT_TRUE(f.follower.lease(grant,now));
  EXPECT_TRUE(f.native.commit(f.follower.geometryPermit(now)));
  auto demand=f.follower.step(now,now-90.);EXPECT_GT(demand.velocity.linear.x,0.);return demand;
}
struct BlockingSink {
  std::promise<void> entered,release;
  std::shared_future<void> released=release.get_future().share();
  std::future<void> started=entered.get_future();
  std::atomic<unsigned> sent{0};
  void operator()(const w::LocalMotionCommand&) {
    entered.set_value();released.wait_for(2s);++sent;
  }
};
w::LocalNavigationState deliveryBody(double source,bool usable=true) {
  w::LocalNavigationState s;s.schema_version=2;s.session_id="session";s.map_version_id="map";
  s.localization_epoch=1;s.localization_seed_id="seed";s.usable=usable;
  s.source_stamp=s.posterior_stamp=s.imu_stamp=t::stamp(source);s.valid_until=t::stamp(source+.1);
  s.local_odometry.header.stamp=s.source_stamp;s.local_odometry.header.frame_id="odom";
  s.local_odometry.child_frame_id="base_link";s.local_odometry.pose.pose.position.x=(source-100.)*.2;
  s.local_odometry.pose.pose.position.z=.55;s.local_odometry.pose.pose.orientation.w=1.;
  s.local_odometry.twist.twist.linear.x=.2;return s;
}
}

TEST(LocalCommandDelivery,BlockedReliableSendDoesNotHoldBodyOrProofLedger) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  const auto demand=nextDemand(f,101.,20);BlockingSink sink;
  auto sending=std::async(std::launch::async,[&]{f.native.demandAt(demand,101000000000LL,std::ref(sink));});
  const bool started=sink.started.wait_for(500ms)==std::future_status::ready;
  EXPECT_TRUE(started);
  geometry_msgs::msg::Twist twist;twist.linear.x=.2;
  auto body_update=std::async(std::launch::async,[&]{
    f.native.bodyAt({{.202,0.,.55},Eigen::Quaterniond::Identity(),101.01,"odom"},twist,
      101010000000LL,101010000000LL);
  });
  const bool body_ready=body_update.wait_for(100ms)==std::future_status::ready;
  EXPECT_TRUE(body_ready)<<"blocked RMW sender retained the ledger mutex";
  std::future<std::optional<w::TrajectoryValidation>> proof_update;
  if(body_ready)proof_update=std::async(std::launch::async,[&]{
    return Access::checkCurve(f.native,f.map,101010000000LL,true);
  });
  if(proof_update.valid())EXPECT_EQ(proof_update.wait_for(100ms),std::future_status::ready);
  sink.release.set_value();sending.get();body_update.get();
  if(proof_update.valid()){const auto proof=proof_update.get();EXPECT_TRUE(proof&&proof->valid);}
  EXPECT_DOUBLE_EQ(Access::bodySource(f.native),101.01);EXPECT_EQ(sink.sent.load(),1U);
}

TEST(LocalCommandDelivery,CancelCannotAcknowledgeAheadOfAnInflightOldSend) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  auto demand=nextDemand(f,101.,20);BlockingSink sink;
  auto sending=std::async(std::launch::async,[&]{f.native.demandAt(demand,101000000000LL,std::ref(sink));});
  EXPECT_EQ(sink.started.wait_for(500ms),std::future_status::ready);
  auto barrier=version();++barrier.reference_generation;
  auto cancel=std::async(std::launch::async,[&]{return f.native.cancelReference(barrier);});
  EXPECT_EQ(cancel.wait_for(30ms),std::future_status::timeout);
  sink.release.set_value();sending.get();EXPECT_TRUE(cancel.get());
  EXPECT_FALSE(f.native.committedIdentity());
  ++demand.sequence;unsigned stale_emissions=0;
  f.native.demandAt(demand,101010000000LL,[&](const auto&){++stale_emissions;});
  EXPECT_EQ(stale_emissions,0U);EXPECT_EQ(sink.sent.load(),1U);
}

TEST(LocalCommandDelivery,CommitReplyCannotOvertakeAnInflightOldSend) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  const auto demand=nextDemand(f,101.,20);BlockingSink sink;
  auto sending=std::async(std::launch::async,[&]{f.native.demandAt(demand,101000000000LL,std::ref(sink));});
  EXPECT_EQ(sink.started.wait_for(500ms),std::future_status::ready);
  auto exchange=std::async(std::launch::async,[&]{
    return f.native.localCommitExchangeAt(*f.initial_commit,101010000000LL);
  });
  EXPECT_EQ(exchange.wait_for(30ms),std::future_status::timeout);
  sink.release.set_value();sending.get();exchange.get();EXPECT_EQ(sink.sent.load(),1U);
}

TEST(LocalCommandDelivery,QueuedCommandCannotRenewItsExpiredOriginalDeadline) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  auto demand=nextDemand(f,101.,20);BlockingSink sink;
  auto sending=std::async(std::launch::async,[&]{f.native.demandAt(demand,101000000000LL,std::ref(sink));});
  EXPECT_EQ(sink.started.wait_for(500ms),std::future_status::ready);
  ++demand.sequence;std::atomic<unsigned> queued_emissions{0};
  auto queued=std::async(std::launch::async,[&]{
    f.native.demandAt(demand,101000000000LL,[&](const auto&){++queued_emissions;});
  });
  EXPECT_EQ(queued.wait_for(140ms),std::future_status::timeout);
  sink.release.set_value();sending.get();queued.get();
  EXPECT_EQ(queued_emissions.load(),0U);EXPECT_EQ(sink.sent.load(),1U);
}

TEST(LocalCommandDelivery,EmitterExceptionReleasesBothLocksAndIsNotReportedAsSuccess) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  auto demand=nextDemand(f,101.,20);
  EXPECT_THROW(f.native.demandAt(demand,101000000000LL,[](const auto&){
    throw std::runtime_error("injected RMW failure");
  }),std::runtime_error);
  const auto before=Access::executionTrace(f.native);
  bool failed=false;for(const auto& e:before.events)
    failed|=e.kind==scan_planner::BoundedExecutionTrace::Kind::CommandPublishFailed;
  EXPECT_TRUE(failed);
  demand=nextDemand(f,101.05,21);unsigned sent=0;
  f.native.demandAt(demand,101050000000LL,[&](const auto&){++sent;});EXPECT_EQ(sent,1U);
}

TEST(LocalCommandDelivery,QueuedIntentCannotCrossANativeNegativeThenPositiveProof) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  auto demand=nextDemand(f,101.,20);BlockingSink sink;
  auto sending=std::async(std::launch::async,[&]{f.native.demandAt(demand,101000000000LL,std::ref(sink));});
  EXPECT_EQ(sink.started.wait_for(500ms),std::future_status::ready);
  ++demand.sequence;std::atomic<unsigned> queued_emissions{0};
  auto queued=std::async(std::launch::async,[&]{
    f.native.demandAt(demand,101030000000LL,[&](const auto&){++queued_emissions;});
  });
  bool staged=false;const auto until=std::chrono::steady_clock::now()+50ms;
  while(std::chrono::steady_clock::now()<until) {
    {const auto lock=Access::holdLedger(f.native);const auto current=Access::latestDemand(f.native);
      staged=current&&current->sequence==demand.sequence;}
    if(staged)break;std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(staged);
  // Real production GridMap queries, not a fabricated verdict: an occupied
  // voxel appears and then clears while the prior intent is queued for send.
  GridMapTestAccess::voxel(f.map,{.30,.03,.55},2.);
  const auto negative=Access::checkCurve(f.native,f.map,101010000000LL,true);
  EXPECT_TRUE(negative&&!negative->valid);if(negative)EXPECT_EQ(negative->reason,"occupied");
  GridMapTestAccess::voxel(f.map,{.30,.03,.55},-1.);
  GridMapTestAccess::acquire(f.map,101020000000LL);
  const auto positive=Access::checkCurve(f.native,f.map,101020000000LL,true);
  EXPECT_TRUE(positive&&positive->valid);
  sink.release.set_value();sending.get();queued.get();
  EXPECT_EQ(queued_emissions.load(),0U);EXPECT_EQ(sink.sent.load(),1U);
}

TEST(LocalCommandDelivery,InvalidSourceWritesLedgerBeforeWaitingForOldSendAndCannotReviveIt) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  f.native.configureLocalBodyInput("session","map","base_link");
  f.native.localBodyAt(deliveryBody(101.),101000000000LL);
  auto demand=nextDemand(f,101.,20);BlockingSink sink;
  auto sending=std::async(std::launch::async,[&]{f.native.demandAt(demand,101000000000LL,std::ref(sink));});
  EXPECT_EQ(sink.started.wait_for(500ms),std::future_status::ready);
  auto invalid=std::async(std::launch::async,[&]{
    f.native.localBodyAt(deliveryBody(101.01,false),101010000000LL);
  });
  auto inspect=std::async(std::launch::async,[&]{
    const auto deadline=std::chrono::steady_clock::now()+100ms;
    while(std::chrono::steady_clock::now()<deadline) {
      if(!f.native.latestPlanningBodyAt(version(),101010000000LL,.1))return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  });
  const bool readable=inspect.wait_for(150ms)==std::future_status::ready;
  EXPECT_TRUE(readable);if(readable)EXPECT_TRUE(inspect.get());
  EXPECT_EQ(invalid.wait_for(0ms),std::future_status::timeout);
  sink.release.set_value();sending.get();invalid.get();if(inspect.valid())inspect.get();
  EXPECT_FALSE(f.native.latestPlanningBodyAt(version(),101010000000LL,.1));
  ++demand.sequence;unsigned stale=0;
  f.native.demandAt(demand,101010000000LL,[&](const auto&){++stale;});EXPECT_EQ(stale,0U);
}

TEST(LocalCommandDelivery,QuickPipelineFaultRecoveryCannotReviveStagedOrRelayedOldIntent) {
  // Fault/recovery is pipeline metadata, never a fabricated collision verdict.
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));warmDelivery(f);
  auto demand=nextDemand(f,101.,20);BlockingSink sink;
  auto sending=std::async(std::launch::async,[&]{f.native.demandAt(demand,101000000000LL,std::ref(sink));});
  EXPECT_EQ(sink.started.wait_for(500ms),std::future_status::ready);
  ++demand.sequence;std::atomic<unsigned> queued_emissions{0};
  auto queued=std::async(std::launch::async,[&]{
    f.native.demandAt(demand,101005000000LL,[&](const auto&){++queued_emissions;});
  });
  bool staged=false;const auto until=std::chrono::steady_clock::now()+50ms;
  while(std::chrono::steady_clock::now()<until) {
    {const auto lock=Access::holdLedger(f.native);const auto current=Access::latestDemand(f.native);
      staged=current&&current->sequence==demand.sequence;}
    if(staged)break;std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(staged);const auto barrier=Access::stopBarrier(f.native);
  f.pipeline_health.clock_fault=true;
  EXPECT_FALSE(Access::pipelineReady(f.native,101010000000LL));
  EXPECT_EQ(Access::stopBarrier(f.native),barrier+1);EXPECT_FALSE(Access::relayedIntent(f.native));
  EXPECT_FALSE(Access::pendingDemand(f.native));
  EXPECT_FALSE(Access::pipelineReady(f.native,101011000000LL));
  EXPECT_EQ(Access::stopBarrier(f.native),barrier+1); // No moving fault floor/barrier.
  f.pipeline_health.clock_fault=false;f.acquire(101.02,.204,.2);
  f.native.progressAt(f.progress(101.02),101.02);
  const auto positive=Access::checkCurve(f.native,f.map,101020000000LL,true);
  EXPECT_TRUE(positive&&positive->valid);EXPECT_FALSE(Access::pipelineFault(f.native));
  EXPECT_FALSE(Access::relayedIntent(f.native));
  sink.release.set_value();sending.get();queued.get();
  EXPECT_EQ(queued_emissions.load(),0U);EXPECT_EQ(sink.sent.load(),1U);
  f.native.demandAt(demand,101025000000LL,[&](const auto&){++queued_emissions;});
  EXPECT_EQ(queued_emissions.load(),0U); // Original sequence cannot be replayed.
  auto fresh=nextDemand(f,101.05,22);fresh.sequence=demand.sequence+1;
  f.native.demandAt(fresh,101050000000LL,[&](const auto& c){EXPECT_FALSE(c.hold);++queued_emissions;});
  EXPECT_EQ(queued_emissions.load(),1U);
}

#ifndef D1MAX_TEST_LEGACY_COMMAND_DELIVERY
TEST(LocalCommandDelivery,CommandBatchOverflowFailsExplicitlyWithoutEmission) {
  Integrated f;
  EXPECT_THROW(Access::overflowCommandBatch(f.native),std::logic_error);
}
#endif
