#include "native_local_follower_test_fixture.hpp"
#include "execution_transport_core.hpp"
#include "test_execution_fixture.hpp"
#include "execution_scheduler.hpp"
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/reference_path.hpp>
#include <cstdlib>
#include <iostream>
#include <future>

namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void configure(BsplineOptimizer& optimizer,const GridMap::Ptr& map) {
    optimizer.order_=3;optimizer.lambda1_=optimizer.lambda2_=optimizer.lambda3_=optimizer.lambda4_=1.;
    optimizer.dist0_=.2;optimizer.max_vel_=.3;optimizer.max_acc_=.35;optimizer.setEnvironment(map);
    optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,{80,40,32});
  }
};
}

namespace {
namespace sdk=d1monitor::execution3;
sdk::Health sdkHealth(){return {true,true,true,true,false,true};}
sdk::Transport sdkWriter(double source_budget,double pipeline,double now=101.) {
  sdk::Transport writer("sdk","isolated_mock");writer.configureTaskFollower(true);
  writer.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
  auto policy=testExecutionPolicy();policy.sensor_source_age_bound_s=source_budget;
  policy.command_pipeline_bound_s=pipeline;policy.writer_period_s=.05;
  policy.source_time_uncertainty_s=.25;writer.configureExecutionPolicy(policy,1.0);
  writer.configureBrakingModel(std::string(64,'d'),.3,.5,"holonomic_xyyaw");
  writer.setSDKTransactions(0,false);
  sdk::Grant::Request request;request.version=version();request.sdk_session="sdk";
  request.execution_id="execution";request.control_epoch=1;request.confirmation_id="click";
  request.request_id="phase_test";request.source_stamp=t::stamp(now);
  request.activate=true;request.transport_mode="isolated_mock";
  EXPECT_TRUE(writer.grant(request,now,sdkHealth()).accepted);
  return writer;
}
bool sdkNonzero(const std::optional<geometry_msgs::msg::Twist>& v) {
  return v&&sdk::planarMotionNonzero(*v);
}
void acquiredRays(Integrated& f,double source,double completed) {
  GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(source)));
  f.pipeline_health.source_ns={t::timeNs(t::stamp(source)),t::timeNs(t::stamp(source))};
  f.pipeline_health.fusion_sequence=f.map.fusionTiming().sequence;
  f.pipeline_health.sampled_steady_ns=f.pipeline_health.fusion_completed_steady_ns=
    t::timeNs(t::stamp(completed));
}
void warm(Integrated& f) {
  for(unsigned i=1;i<20;++i){const double now=100.+i*.05;
    f.acquire(now,(now-100.)*.2,.2);
    acquiredRays(f,std::max(100.,now-.44),now);
    EXPECT_TRUE(f.command(now,(now-100.)*.2,.2,i,true,false));
  }
}
struct PhaseStats {unsigned native_nonzero=0,sdk_nonzero=0,sdk_zero=0;};
PhaseStats phaseRun(double source_budget,double pipeline,double sdk_phase) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  f.braking.sensor_source_age=source_budget;f.braking.command_pipeline_bound=pipeline;
  Access::braking(f.native,f.braking);warm(f);auto writer=sdkWriter(source_budget,pipeline);
  PhaseStats stats;
  for(unsigned i=0;i<20;++i) {
    const double now=101.+i*.05,ray_age=i%2?.49:.44;
    f.acquire(now,(now-100.)*.2,.2);
    acquiredRays(f,now-ray_age,now);
    const auto grant=lease(now,i+20);EXPECT_TRUE(writer.taskLease(grant,now));
    auto command=f.command(now,(now-100.)*.2,.2,i+20,true,false);
    EXPECT_TRUE(command.has_value());if(!command)continue;
    stats.native_nonzero+=sdk::planarMotionNonzero(command->velocity);
    // Fixture uses base_link; production uses d1max_loc_base_link in both
    // Native and SDK. This changes only the literal frame label, not data.
    command->body_frame="d1max_loc_base_link";
    EXPECT_TRUE(writer.localCommand(*command,now+.001));
    const auto output=writer.tick(now+sdk_phase,sdkHealth());
    if(sdkNonzero(output))++stats.sdk_nonzero;else ++stats.sdk_zero;
    writer.writeCalled(now+sdk_phase,true,sdkNonzero(output));
    // Close the production feedback path with the actual simulated write
    // fact, not an inferred stop or a manually acknowledged watermark.
    // The next real follower step echoes it through Native to the SDK.
    const auto& feedback=writer.taskCommandFeedback();
    EXPECT_TRUE(feedback.has_value());
    if(feedback)EXPECT_TRUE(f.follower.commandFeedback(*feedback,now+sdk_phase));
  }
  std::cout<<"phase source="<<source_budget<<" pipeline="<<pipeline
    <<" phase_ms="<<sdk_phase*1000<<" native="<<stats.native_nonzero
    <<" sdk="<<stats.sdk_nonzero<<" zero="<<stats.sdk_zero<<'\n';
  return stats;
}
std::optional<w::LocalMotionCommand> atSourceAge(Integrated& f,double source_budget,
    double sensor_age,double body_age=0.) {
  f.braking.sensor_source_age=source_budget;f.braking.command_pipeline_bound=.1;
  Access::braking(f.native,f.braking);GridMapTestAccess::sourceBudget(f.map,source_budget);
  // Initial ray source is 100; never regress it while warming the real tracker.
  for(unsigned i=1;i<20;++i){const double now=100.+i*.05;
    f.acquire(now,(now-100.)*.2,.2);
    acquiredRays(f,std::max(100.,now-sensor_age),now);
    EXPECT_TRUE(f.command(now,(now-100.)*.2,.2,i,true,false));
  }
  const double body_time=101.-body_age;
  f.acquire(body_time,(body_time-100.)*.2,.2);
  acquiredRays(f,101.-sensor_age,101.);
  // Collision geometry is completed state; true input loss is a distinct
  // Native health fault, not a fabricated negative occupancy verdict.
  return f.command(101.,(body_time-100.)*.2,.2,20,true,false);
}
}
TEST(NativeSdkPhase,PersistentStateDoesNotCopyMapAgeIntoEitherCommandBudget) {
  // Both historical allocations retain their exact original sources. Neither
  // turns an accepted collision state into a map-derived command deadline.
  for(const auto phase:{.005,.025,.045}) {
    const auto old=phaseRun(.5,.2,phase),updated=phaseRun(.6,.1,phase);
    EXPECT_EQ(old.native_nonzero,20u);EXPECT_EQ(updated.native_nonzero,20u);
    EXPECT_EQ(old.sdk_nonzero,20u);EXPECT_EQ(old.sdk_zero,0u);
    EXPECT_EQ(updated.sdk_nonzero,20u);EXPECT_EQ(updated.sdk_zero,0u);
  }
}
TEST(NativeSdkPhase,ActualAboveCruiseBoundaryCommitsAndProducesFreshBoundedMotion) {
 for(const double initial_speed:{.304981,.41698}) {
  SCOPED_TRACE(initial_speed);
  auto native_map=std::make_shared<GridMap>();GridMapTestAccess::configure(*native_map,true);
  scan_planner::BsplineOptimizer optimizer;scan_planner::BsplineOptimizerTestAccess::configure(optimizer,native_map);
  auto budget=std::make_shared<scan_planner::SolveBudget>();optimizer.setSolveBudget(budget);
  scan_planner::DiscreteReference path;path.set({{0,0,.55},{1.,0,.55}});
  const auto seed=scan_planner::sampleReferenceSeed(path,.3,.35,initial_speed,0.,.2);
  const scan_planner::CubicMotionBoundary boundary{{0,0,.55},{initial_speed,0,0},{0,0,0},
      {1.,0,.55},{0,0,0},{0,0,0}};
  auto cp=scan_planner::fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
  optimizer.reference_path_samples_=seed.samples;optimizer.initControlPoints(cp);
  ASSERT_TRUE(optimizer.controlPointsInitialized());Eigen::MatrixXd optimized;
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(optimized,seed.dt));
  scan_planner::UniformBspline solved(optimized,3,seed.dt);
  const auto timed=scan_planner::refineTimingWithFixedBoundary(solved,boundary,.3,.35,
    [&](Eigen::MatrixXd& points,double dt) {
      scan_planner::UniformBspline reference(points,3,dt);optimizer.ref_pts_.clear();
      for(int i=0;i<=points.cols()-3;++i)optimizer.ref_pts_.push_back(reference.evaluateDeBoorT(i*dt));
      Eigen::MatrixXd out;const bool ok=optimizer.BsplineOptimizeTrajRefine(points,dt,out);if(ok)points=out;return ok;
    },8,std::nullopt,[&]{return budget->allowed();},true);
  ASSERT_TRUE(timed.success)<<timed.reason;ASSERT_TRUE(scan_planner::cubicBoundaryMatches(solved,boundary));
  auto q=tag(1,{0,0,.55},{initial_speed,0,0},{0,0,0});q.trajectory.pos_pts.clear();q.trajectory.knots.clear();
  cp=solved.getControlPoint();const auto knots=solved.getKnot();
  q.trajectory.yaw_pts.assign(cp.cols(),0.);q.trajectory.yaw_dt=solved.getInterval();
  for(int i=0;i<cp.cols();++i){geometry_msgs::msg::Point p;p.x=cp(0,i);p.y=cp(1,i);p.z=cp(2,i);q.trajectory.pos_pts.push_back(p);}
  for(int i=0;i<knots.size();++i)q.trajectory.knots.push_back(knots[i]);freezeHeading(q);
  auto curve=scan_planner::decodeSpliceCurve(q.trajectory);ASSERT_TRUE(curve);auto velocity=curve->getDerivative();
  const auto dynamics=scan_planner::inspectInitialMotionLimits(*curve,.3,.35);
  ASSERT_TRUE(dynamics.valid);ASSERT_TRUE(dynamics.recovering);
  Integrated f(true,false,{},q);ASSERT_TRUE(f.initial_commit);EXPECT_EQ(f.follower.controller().trajectoryId(),1);
  auto writer=sdkWriter(.6,.1,100.);unsigned nonzero=0;
  for(unsigned i=1;i<=25;++i) {
    const double time=i*.05,now=100.+time;const auto p=curve->evaluateDeBoorT(time),v=velocity.evaluateDeBoorT(time);
    auto command=f.command(now,p.x(),v.x(),i);ASSERT_TRUE(command)<<f.follower.controller().reason();
    EXPECT_TRUE(writer.taskLease(lease(now,i),now));command->body_frame="d1max_loc_base_link";
    ASSERT_TRUE(writer.localCommand(*command,now+.001));const auto output=writer.tick(now+.025,sdkHealth());
    ASSERT_TRUE(output);EXPECT_GE(output->linear.x,0.);EXPECT_LE(output->linear.x,.3);EXPECT_DOUBLE_EQ(output->linear.y,0.);
    nonzero+=sdkNonzero(output);writer.writeCalled(now+.025,true,sdkNonzero(output));
  }
  EXPECT_EQ(nonzero,25u);
  // Original evidence expiry, not a newly stamped heartbeat, terminates use.
  EXPECT_FALSE(sdkNonzero(writer.tick(101.36,sdkHealth())));
 }
}
TEST(NativeSdkPhase,OriginalBodyDeadlineAndRevokeStillStopWithoutRestamping) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  f.braking.sensor_source_age=.6;f.braking.command_pipeline_bound=.1;
  Access::braking(f.native,f.braking);warm(f);auto writer=sdkWriter(.6,.1);
  auto grant=lease(101.,20);ASSERT_TRUE(writer.taskLease(grant,101.));
  auto command=f.command(101.,.2,.2,20);ASSERT_TRUE(command);
  command->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*command,101.001));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.045,sdkHealth())));
  writer.writeCalled(101.045,true,true);
  EXPECT_FALSE(sdkNonzero(writer.tick(101.101,sdkHealth())));
  EXPECT_FALSE(writer.localCommand(*command,101.102));
  auto next=f.command(101.15,.23,.2,21);ASSERT_TRUE(next);
  next->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.taskLease(lease(101.15,21),101.15));
  ASSERT_TRUE(writer.localCommand(*next,101.151));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.155,sdkHealth())));
  writer.writeCalled(101.155,true,true);
  auto revoke=lease(101.16,22);revoke.revoked=true;revoke.allowed=false;
  ASSERT_TRUE(writer.taskLease(revoke,101.16));
  EXPECT_FALSE(sdkNonzero(writer.tick(101.165,sdkHealth())));
  EXPECT_FALSE(writer.localCommand(*next,101.166));
}
TEST(NativeSdkPhase,RecordedTailAgesDoNotShortenOriginalBodyAndCommandDeadline) {
  for(const double age:{.546,.557})for(const double phase:{.005,.025,.045}) {
    Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    auto command=atSourceAge(f,.6,age);ASSERT_TRUE(command);
    ASSERT_GT(command->velocity.linear.x,0.);
    EXPECT_EQ(command->valid_until,t::stamp(101.1));
    EXPECT_EQ(command->body_source_stamp,t::stamp(101.));
    EXPECT_EQ(command->source_stamp,t::stamp(101.));
    auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
    command->body_frame="d1max_loc_base_link";
    ASSERT_TRUE(writer.localCommand(*command,101.001));
    EXPECT_TRUE(sdkNonzero(writer.tick(101.+phase,sdkHealth())));
    EXPECT_FALSE(sdkNonzero(writer.tick(101.101,sdkHealth())));
    std::cout<<"tail sensor_age_ms="<<age*1000<<" phase_ms="<<phase*1000
      <<" expected_nonzero=1 original_deadline=101.1"<<'\n';
  }
}
TEST(NativeSdkPhase,FreshCurveStateDoesNotRestampOrRenewOriginalCommand) {
  // The old ray-derived ~41 ms deadline is gone. A completed ordinary map
  // update neither interrupts this command nor manufactures a renewal.
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  auto command=atSourceAge(f,.6,.559);ASSERT_TRUE(command);
  ASSERT_GT(command->velocity.linear.x,0.);
  EXPECT_EQ(command->valid_until,t::stamp(101.1));
  auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
  command->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*command,101.001));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));
  writer.writeCalled(101.005,true,true);
  // Actual acquired sources advance. The old control and body stamps may NOT.
  f.acquire(101.02,.204,.2);
  GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(100.60)));
  std::optional<w::LocalMotionCommand> renewed;
  const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(101.025)),true,
    [&](const auto& c){renewed=c;});
  ASSERT_TRUE(proof&&proof->valid);EXPECT_EQ(proof->valid_until,t::stamp(0.));
  EXPECT_FALSE(renewed);
  EXPECT_FALSE(writer.localCommand(*command,101.027)); // duplicate cannot renew
  EXPECT_TRUE(sdkNonzero(writer.tick(101.045,sdkHealth())));
  // New environmental evidence never buys an old controller/body more time.
  EXPECT_FALSE(sdkNonzero(writer.tick(101.101,sdkHealth())));
  EXPECT_FALSE(writer.localCommand(*command,101.102));
}
TEST(NativeSdkPhase,CompletedCurrentStateAndCommandsDoNotWaitForBlockedProofPublisher) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  auto command=atSourceAge(f,.6,.559);ASSERT_TRUE(command);
  ASSERT_GT(command->velocity.linear.x,0.);
  ASSERT_EQ(command->valid_until,t::stamp(101.1));
  auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
  command->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*command,101.001));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));
  writer.writeCalled(101.005,true,true);
  f.acquire(101.02,.204,.2);
  GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(100.60)));
  ASSERT_TRUE(f.pool.publishValidation(f.map,101025000000LL,std::chrono::steady_clock::now()));
  ASSERT_TRUE(f.pool.refreshOwnerReplica(f.map,101025000000LL,std::chrono::steady_clock::now()));
  std::atomic<std::int64_t> now{101025000000LL};
  std::promise<void> publishing,release;auto started=publishing.get_future();auto unblocked=release.get_future();
  std::optional<w::LocalMotionCommand> renewed;
  auto query=std::async(std::launch::async,[&]{
    return Access::checkPooledCurveClock(f.native,[&]{return now.load();},[&](const auto&){
      publishing.set_value();unblocked.wait(); // Real output boundary, deterministic blocked sink.
    },[&](auto c){
      renewed=c;c.body_frame="d1max_loc_base_link";
      EXPECT_TRUE(writer.localCommand(c,now.load()*1e-9));
    });
  });
  const auto blocked=started.wait_for(std::chrono::seconds(3));
  EXPECT_EQ(blocked,std::future_status::ready);
  if(blocked==std::future_status::ready) {
    // No callback can touch writer/renewed until release; future synchronization
    // makes this a real ordering test, not a sleep or an assumed ROS delay.
    EXPECT_FALSE(renewed.has_value()); // No map-derived renewal is needed.
    const auto output=writer.tick(101.045,sdkHealth());
    EXPECT_TRUE(sdkNonzero(output));
    EXPECT_EQ(writer.taskCommandExpiry().count,0U);
    writer.writeCalled(101.045,true,sdkNonzero(output));
    // Continue the actual next follower tick while its notification remains
    // blocked. The Native ledger already owns the completed verdict; each
    // fresh controller sample retains only its original motion deadlines.
    for(unsigned i=1;i<=2;++i) {
      const double tick=101.+.05*i,source=tick-.01;
      f.acquire(source,(source-100.)*.2,.2);
      const auto grant=lease(tick,20+i);EXPECT_TRUE(f.follower.lease(grant,tick));
      EXPECT_TRUE(f.native.taskLeaseAt(grant,t::timeNs(t::stamp(tick))));EXPECT_TRUE(writer.taskLease(grant,tick));
      const auto before_progress=f.progress(tick);EXPECT_FALSE(before_progress.holding);
      f.native.progressAt(before_progress,tick);
      EXPECT_TRUE(f.native.commit(f.follower.geometryPermit(tick)));
      const auto next=f.follower.step(tick,tick-90.);EXPECT_FALSE(next.hold)<<next.reason;
      f.native.demandAt(next,t::timeNs(t::stamp(tick)),[&](auto c){c.body_frame="d1max_loc_base_link";
        EXPECT_TRUE(writer.localCommand(c,tick));});
      const auto actual_progress=f.progress(tick);EXPECT_FALSE(actual_progress.holding);
      f.native.progressAt(actual_progress,tick);
      const auto actual=writer.tick(tick+.005,sdkHealth());EXPECT_TRUE(sdkNonzero(actual));
      writer.writeCalled(tick+.005,true,sdkNonzero(actual));
      EXPECT_EQ(writer.taskCommandExpiry().count,0U);
    }
  }
  now.store(101125000000LL);release.set_value();const auto proof=query.get();
  ASSERT_TRUE(proof&&proof->valid);EXPECT_FALSE(renewed);
  EXPECT_EQ(proof->valid_until,t::stamp(0.));
  EXPECT_EQ(proof->check_end,t::stamp(101.025));
  EXPECT_EQ(proof->front_ray_source_stamp,t::stamp(100.60));
  EXPECT_EQ(proof->body_source_stamp,t::stamp(101.02));
  EXPECT_TRUE(f.follower.validation(*proof)); // Delayed notification, unchanged source/expiry.
  auto next=f.command(101.15,.23,.2,23);ASSERT_TRUE(next);next->body_frame="d1max_loc_base_link";
  EXPECT_TRUE(writer.taskLease(lease(101.15,23),101.15));EXPECT_TRUE(writer.localCommand(*next,101.151));
  EXPECT_TRUE(sdkNonzero(writer.tick(101.155,sdkHealth())));
  EXPECT_EQ(writer.taskCommandExpiry().count,0U);
  // The old intent cannot revive; even the newest real body retains its bound.
  EXPECT_FALSE(writer.localCommand(*command,101.16));
  EXPECT_FALSE(sdkNonzero(writer.tick(101.251,sdkHealth())));
}
TEST(NativeSdkPhase,LocalNotificationWaitDoesNotOverrideInputLossNegativeOrCancellation) {
 for(unsigned fault=0;fault<3;++fault) {
  SCOPED_TRACE(fault);
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  auto initial=atSourceAge(f,.6,.559);ASSERT_TRUE(initial);
  auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
  initial->body_frame="d1max_loc_base_link";ASSERT_TRUE(writer.localCommand(*initial,101.001));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));writer.writeCalled(101.005,true,true);
  // Notification is deliberately absent. Only genuine Native facts decide
  // relay: real input loss, obstacle or cancellation each still stop. The
  // already sent command remains bounded by its own original 100 ms expiry.
  if(fault==1) {
    f.acquire(101.02,.204,.2);GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
    const auto negative=Access::checkCurve(f.native,f.map,101025000000LL,true);
    ASSERT_TRUE(negative&&!negative->valid);ASSERT_EQ(negative->reason,"occupied");
  }
  if(fault==2) {auto revoke=lease(101.025,21);revoke.revoked=true;revoke.allowed=false;
    ASSERT_TRUE(f.native.taskLeaseAt(revoke,101025000000LL));ASSERT_TRUE(writer.taskLease(revoke,101.025));}
  unsigned emitted=0;
  for(unsigned i=1;i<=2;++i) {
    const double tick=101.+i*.05,source=tick-.01;
    if(fault==0) {
      f.acquireBody(source,(source-100.)*.2,.2); // Rays really stopped at 100.441.
      f.pipeline_health.sampled_steady_ns=t::timeNs(t::stamp(tick));
    } else f.acquire(source,(source-100.)*.2,.2);
    const auto grant=lease(tick,21+i);ASSERT_TRUE(f.follower.lease(grant,tick));
    if(fault!=2){ASSERT_TRUE(f.native.taskLeaseAt(grant,t::timeNs(t::stamp(tick))));ASSERT_TRUE(writer.taskLease(grant,tick));}
    f.native.progressAt(f.progress(tick),tick);ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(tick)));
    const auto demand=f.follower.step(tick,tick-90.);ASSERT_FALSE(demand.hold);
    f.native.demandAt(demand,t::timeNs(t::stamp(tick)),[&](const auto&){++emitted;});
    EXPECT_EQ(sdkNonzero(writer.tick(tick+.005,sdkHealth())),fault!=2&&i==1);
  }
  EXPECT_EQ(emitted,0U);EXPECT_FALSE(writer.localCommand(*initial,101.11));
  if(fault==0) {
    EXPECT_TRUE(Access::pipelineFault(f.native));
    // All buffered intents have expired before genuinely newer rays/fusion
    // and a complete current check can recover the input-loss latch.
    // A fresh proof alone cannot resurrect them; a new real demand is needed.
    f.acquire(101.20,.24,.2);
    const auto proof=Access::checkCurve(f.native,f.map,101205000000LL,true,[&](const auto&){++emitted;});
    ASSERT_TRUE(proof&&proof->valid);EXPECT_EQ(emitted,0U);
    EXPECT_FALSE(sdkNonzero(writer.tick(101.21,sdkHealth())));
    auto next=f.command(101.25,.25,.2,24);ASSERT_TRUE(next);next->body_frame="d1max_loc_base_link";
    EXPECT_TRUE(writer.taskLease(lease(101.25,24),101.25));EXPECT_TRUE(writer.localCommand(*next,101.251));
    EXPECT_TRUE(sdkNonzero(writer.tick(101.255,sdkHealth())));
  }
 }
}
TEST(NativeSdkPhase,CompletedCurrentProofDeliversPendingDemandBeforeBlockedProofPublisher) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  auto old=atSourceAge(f,.6,.559);ASSERT_TRUE(old);
  auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
  old->body_frame="d1max_loc_base_link";ASSERT_TRUE(writer.localCommand(*old,101.001));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));writer.writeCalled(101.005,true,true);
  // A real new control sample reaches Native with its exact current verdict
  // unavailable (not merely old). A completed current check must release this
  // live pending sample before its notification can block.
  f.acquire(101.04,.208,.2);f.native.progressAt(f.progress(101.04),101.04);
  const auto grant=lease(101.04,21);ASSERT_TRUE(writer.taskLease(grant,101.04));
  ASSERT_TRUE(f.follower.lease(grant,101.04));ASSERT_TRUE(f.native.taskLeaseAt(grant,101040000000LL));
  ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(101.04)));
  const auto demand=f.follower.step(101.04,101.04-90.);ASSERT_FALSE(demand.hold)<<demand.reason;
  Access::removeActiveProof(f.native);
  unsigned delivered=0;
  f.native.demandAt(demand,101042000000LL,[&](const auto&){++delivered;});
  ASSERT_EQ(delivered,0U);ASSERT_TRUE(Access::pendingDemand(f.native));
  GridMapTestAccess::acquire(f.map,100600000000LL);
  ASSERT_TRUE(f.pool.publishValidation(f.map,101045000000LL,std::chrono::steady_clock::now()));
  ASSERT_TRUE(f.pool.refreshOwnerReplica(f.map,101045000000LL,std::chrono::steady_clock::now()));
  std::atomic<std::int64_t> now{101045000000LL};
  std::promise<void> publishing,release;auto started=publishing.get_future();auto unblocked=release.get_future();
  std::optional<w::LocalMotionCommand> sent;
  auto query=std::async(std::launch::async,[&]{
    return Access::checkPooledCurveClock(f.native,[&]{return now.load();},[&](const auto&){
      publishing.set_value();unblocked.wait();
    },[&](auto c){
      ++delivered;sent=c;c.body_frame="d1max_loc_base_link";
      EXPECT_TRUE(writer.localCommand(c,now.load()*1e-9));
    });
  });
  const auto blocked=started.wait_for(std::chrono::seconds(3));
  EXPECT_EQ(blocked,std::future_status::ready);
  if(blocked==std::future_status::ready) {
    EXPECT_EQ(delivered,1U);
    const auto output=writer.tick(101.065,sdkHealth());EXPECT_TRUE(sdkNonzero(output));
    EXPECT_EQ(writer.taskCommandExpiry().count,0U);writer.writeCalled(101.065,true,sdkNonzero(output));
  }
  now.store(101160000000LL);release.set_value();const auto proof=query.get();
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(sent);EXPECT_EQ(delivered,1U);
  EXPECT_EQ(sent->sequence,demand.sequence);EXPECT_EQ(sent->source_stamp,demand.source_stamp);
  EXPECT_EQ(sent->body_source_stamp,demand.body_source_stamp);EXPECT_EQ(sent->velocity,demand.velocity);
  EXPECT_EQ(sent->valid_until,t::stamp(101.14));EXPECT_FALSE(Access::pendingDemand(f.native));
  EXPECT_FALSE(sdkNonzero(writer.tick(101.141,sdkHealth())));
}
TEST(NativeSdkPhase,OutputPriorityCannotRenewNegativeCanceledSupersededOrExpiredIntent) {
  for(unsigned fault=0;fault<5;++fault) {
    SCOPED_TRACE(fault);
    Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    auto command=atSourceAge(f,.6,.559);ASSERT_TRUE(command);
    auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
    command->body_frame="d1max_loc_base_link";ASSERT_TRUE(writer.localCommand(*command,101.001));
    ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));writer.writeCalled(101.005,true,true);
    if(fault==1) {
      f.acquireBody(101.02,.204,.2); // Keep last accepted ray source 100.441.
      f.pipeline_health.sampled_steady_ns=101050000000LL;
    } else {f.acquire(101.02,.204,.2);GridMapTestAccess::acquire(f.map,100600000000LL);}
    if(fault==0)GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.); // actual new obstacle
    ASSERT_TRUE(f.pool.publishValidation(f.map,101025000000LL,std::chrono::steady_clock::now()));
    ASSERT_TRUE(f.pool.refreshOwnerReplica(f.map,101025000000LL,std::chrono::steady_clock::now()));
    std::int64_t now=fault==1?101050000000LL:101025000000LL;unsigned renewed=0;
    const auto sink=[&](auto c){if(!c.hold)++renewed;c.body_frame="d1max_loc_base_link";
      EXPECT_TRUE(writer.localCommand(c,now*1e-9));};
    const auto proof=Access::checkPooledCurveClock(f.native,[&]{return now;},{},sink,[&]{
      // Interleave between the completed query/released map and ledger commit.
      if(fault==2){auto revoke=lease(101.025,22);revoke.revoked=true;revoke.allowed=false;
        EXPECT_TRUE(f.native.taskLeaseAt(revoke,now));EXPECT_TRUE(writer.taskLease(revoke,now*1e-9));}
      if(fault==3)now=101101000000LL; // original command/body expired; no replacement source
      if(fault==4){auto hold=*Access::latestDemand(f.native);++hold.sequence;hold.hold=true;hold.velocity={};
        hold.source_stamp=t::stamp(101.025);hold.body_source_stamp=t::stamp(101.02);
        f.native.demandAt(hold,now,sink);}
    });
    EXPECT_EQ(renewed,0U);
    if(fault==0){ASSERT_TRUE(proof);EXPECT_FALSE(proof->valid);EXPECT_EQ(proof->reason,"occupied");}
    if(fault==1){ASSERT_TRUE(proof&&proof->valid);EXPECT_TRUE(Access::pipelineFault(f.native));}
    // A lost notification cannot extend an already emitted command. Revoke
    // and explicit HOLD stop immediately; loss/negative block further Native
    // motion and the original SDK command expires without any source rewrite.
    EXPECT_EQ(sdkNonzero(writer.tick(fault==3?101.102:101.055,sdkHealth())),fault<2);
    EXPECT_FALSE(sdkNonzero(writer.tick(101.103,sdkHealth())));
  }
}
TEST(NativeSdkPhase,RejectedCommitAckAndIncumbentMotionDoNotWaitForSelectedCandidatePublisher) {
  Integrated f(true,false,{},tag(2,{0,0,.55},{.2,0,0},{0,0,0}));
  auto incumbent=f.command(100.10,.02,.2,1);ASSERT_TRUE(incumbent);
  auto writer=sdkWriter(.5,.1,100.10);ASSERT_TRUE(writer.taskLease(lease(100.10,1),100.10));
  incumbent->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*incumbent,100.101));
  ASSERT_TRUE(sdkNonzero(writer.tick(100.105,sdkHealth())));writer.writeCalled(100.105,true,true);
  auto replacement=tag(4,{.02,0,.55},{.2,0,0},{0,0,0});
  replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.10);
  f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
  auto proof=Access::checkCurve(f.native,f.map,100120000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(f.follower.validation(*proof));
  const auto prepare=f.follower.exchangeCommit(100.12,100.12-90.);ASSERT_TRUE(prepare);
  const auto prepared=f.native.localCommitExchangeAt(*prepare,100121000000LL);
  ASSERT_TRUE(prepared.accepted);ASSERT_TRUE(f.follower.commitResult(prepared,100.122,100.122-90.));
  f.acquire(100.13,.026,.2);
  const auto commit=f.follower.exchangeCommit(100.13,100.13-90.);ASSERT_TRUE(commit);
  ASSERT_EQ(commit->disposition,"committed");ASSERT_EQ(f.follower.controller().trajectoryId(),2);
  ASSERT_EQ(f.native.committedTrajectory(),2);
  // Model the actual publication mutex contention: a later candidate is being
  // sent by another worker. Its slow subscriber must not delay this ACK.
  std::promise<void> publishing,release;auto started=publishing.get_future();auto unblocked=release.get_future();
  Access::selectedObserver(f.native,[&](const auto&){publishing.set_value();unblocked.wait();});
  auto selected=std::async(std::launch::async,[&]{
    auto newer=tag(5,{.026,0,.55},{.2,0,0},{0,0,0});
    newer.trajectory.start_time=newer.join_source_stamp=t::stamp(100.13);
    f.native.candidate(newer,f.reference,f.heading);
    // New code uses this exact single consumer from its timer; old code is
    // already blocked in candidate(). Identical production methods either way.
    Access::dispatchSelected(f.native);
  });
  const auto blocking=started.wait_for(std::chrono::seconds(3));
  EXPECT_EQ(blocking,std::future_status::ready);
  if(blocking!=std::future_status::ready){release.set_value();selected.get();return;}
  Access::forgetOriginalProofs(f.native); // A legitimate final rejection, never an actuation bypass.
  auto answer=std::async(std::launch::async,[&]{return f.native.localCommitExchangeAt(*commit,100134000000LL);});
  const auto timely=answer.wait_for(std::chrono::milliseconds(100));
  EXPECT_EQ(timely,std::future_status::ready);
  if(timely!=std::future_status::ready) {
    // The real ROS control group cannot process body/command callbacks while
    // its commit callback is blocked. Observe the old deadline, then always
    // release the sink before unwinding either asynchronous task.
    EXPECT_TRUE(sdkNonzero(writer.tick(100.205,sdkHealth())));
    release.set_value();selected.get();const auto late=answer.get();
    EXPECT_FALSE(late.accepted);Access::selectedObserver(f.native,{});return;
  }
  std::optional<w::LocalTrajectoryCommit> ack;
  if(timely==std::future_status::ready){ack=answer.get();EXPECT_FALSE(ack->accepted);
    EXPECT_EQ(ack->reason,"local_commit_measured_join_rejected");
    EXPECT_TRUE(f.follower.commitResult(*ack,100.135,100.135-90.));}
  f.acquire(100.15,.03,.2);
  const auto next_grant=lease(100.15,2);EXPECT_TRUE(f.follower.lease(next_grant,100.15));
  EXPECT_TRUE(f.native.taskLeaseAt(next_grant,100150000000LL));
  EXPECT_TRUE(f.native.commit(f.follower.geometryPermit(100.15)));
  const auto demand=f.follower.step(100.15,100.15-90.);
  std::optional<w::LocalMotionCommand> next;
  f.native.demandAt(demand,100150000000LL,[&](const auto& c){next=c;});
  EXPECT_TRUE(next.has_value());
  if(next){EXPECT_EQ(next->trajectory_id,2);next->body_frame="d1max_loc_base_link";
    EXPECT_TRUE(writer.taskLease(lease(100.15,2),100.15));EXPECT_TRUE(writer.localCommand(*next,100.151));}
  const auto output=writer.tick(100.205,sdkHealth());
  EXPECT_TRUE(sdkNonzero(output));EXPECT_EQ(writer.taskCommandExpiry().count,0U);
  release.set_value();selected.get();if(!ack)ack=answer.get();
  Access::selectedObserver(f.native,{});
  EXPECT_EQ(f.native.committedTrajectory(),2);
}
TEST(NativeSdkPhase,LateLeaseHeartbeatCannotRebindAnAlreadyRelayedCommandsAuthorization) {
  // The heartbeat is generated before the control sample but delivered after
  // its command. New SCAN state must retain the ORIGINAL authorization,
  // not select this later-delivered heartbeat and renew the original command.
 for(const bool original_lease_expires:{false,true}) {
  SCOPED_TRACE(original_lease_expires);
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  f.braking.sensor_source_age=.6;f.braking.command_pipeline_bound=.1;
  Access::braking(f.native,f.braking);GridMapTestAccess::sourceBudget(f.map,.6);
  for(unsigned i=1;i<20;++i){const double now=100.+i*.05;
    f.acquire(now,(now-100.)*.2,.2);
    GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(std::max(100.,now-.559))));
    EXPECT_TRUE(f.command(now,(now-100.)*.2,.2,i,true,false));
  }
  f.acquire(101.,.2,.2);
  GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(100.441)));
  const auto initial_proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(101.)),true);
  ASSERT_TRUE(initial_proof&&initial_proof->valid)
    <<(initial_proof?initial_proof->reason:"no proof");
  ASSERT_TRUE(f.follower.validation(*initial_proof));
  auto original_lease=lease(100.95,20);
  if(original_lease_expires)original_lease.valid_until=t::stamp(101.02);
  ASSERT_TRUE(f.follower.lease(original_lease,101.));
  ASSERT_TRUE(f.native.taskLeaseAt(original_lease,t::timeNs(t::stamp(101.))));
  ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(101.)));
  const auto demand=f.follower.step(101.,11.);ASSERT_FALSE(demand.hold);
  std::optional<w::LocalMotionCommand> command;
  f.native.demandAt(demand,t::timeNs(t::stamp(101.001)),[&](const auto& c){command=c;});
  ASSERT_TRUE(command);ASSERT_EQ(command->lease_sequence,original_lease.sequence);
  ASSERT_EQ(command->valid_until,t::stamp(original_lease_expires?101.02:101.1));
  auto writer=sdkWriter(.6,.1);
  ASSERT_TRUE(writer.taskLease(original_lease,101.));
  command->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*command,101.002));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));
  writer.writeCalled(101.005,true,true);

  const auto late_lease=lease(100.99,21);
  ASSERT_TRUE(f.native.taskLeaseAt(late_lease,t::timeNs(t::stamp(101.01))));
  ASSERT_TRUE(writer.taskLease(late_lease,101.01));
  f.acquire(101.02,.204,.2);
  GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(100.60)));
  std::optional<w::LocalMotionCommand> renewed;
  const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(101.025)),true,
    [&](const auto& c){renewed=c;});
  ASSERT_TRUE(proof&&proof->valid);
  if(original_lease_expires) {
    // A later same-grant heartbeat cannot rescue the old command after its
    // original authorization has expired; the next real demand is required.
    EXPECT_FALSE(renewed);
    EXPECT_FALSE(sdkNonzero(writer.tick(101.045,sdkHealth())));
    continue;
  }
  EXPECT_FALSE(renewed);
  EXPECT_EQ(command->lease_sequence,original_lease.sequence);
  EXPECT_EQ(command->valid_until,t::stamp(101.1));
  EXPECT_FALSE(writer.localCommand(*command,101.026));
  EXPECT_TRUE(sdkNonzero(writer.tick(101.045,sdkHealth())));
  EXPECT_FALSE(sdkNonzero(writer.tick(101.101,sdkHealth())));
 }
}
TEST(NativeSdkPhase,EvidenceRefreshCannotReviveExpiredStoppedCanceledOrNegativeCommands) {
 for(int barrier=0;barrier<5;++barrier) {
  SCOPED_TRACE(barrier);
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  const auto command=atSourceAge(f,.6,.559);ASSERT_TRUE(command);
  ASSERT_GT(command->velocity.linear.x,0.);
  if(barrier==1) {
    GridMapTestAccess::voxel(f.map,{.5,0,.55},2.);
    const auto negative=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(101.01)),true);
    ASSERT_TRUE(negative&&!negative->valid);ASSERT_EQ(negative->reason,"occupied");
    GridMapTestAccess::voxel(f.map,{.5,0,.55},-1.);
  } else if(barrier==2) {
    auto zero=*Access::latestDemand(f.native);++zero.sequence;
    zero.source_stamp=t::stamp(101.015);zero.hold=true;zero.velocity={};
    f.native.demandAt(zero,t::timeNs(t::stamp(101.015)),{});
  } else if(barrier==3) {
    auto cancel=version();++cancel.reference_generation;ASSERT_TRUE(f.native.cancelReference(cancel));
  } else if(barrier==4) {
    // Pinning the original emitted lease never ignores a newer revocation.
    auto revoke=lease(101.015,21);revoke.revoked=true;revoke.allowed=false;
    ASSERT_TRUE(f.native.taskLeaseAt(revoke,t::timeNs(t::stamp(101.015))));
  }
  const double now=barrier==0?101.101:101.025;
  f.acquire(now,(now-100.)*.2,.2);
  unsigned emitted=0;
  const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)),true,
    [&](const auto&){++emitted;});
  if(barrier!=3)ASSERT_TRUE(proof&&proof->valid);
  EXPECT_EQ(emitted,0u);
 }
}
TEST(NativeSdkPhase,ExistingBodyHundredMillisecondLimitIsNotRedistributedToSensorBudget) {
  for(const double phase:{.005,.025,.045,.065,.095}) {
    Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    auto command=atSourceAge(f,.6,.44,.04);ASSERT_TRUE(command);
    ASSERT_GT(command->velocity.linear.x,0.);
    EXPECT_EQ(t::timeNs(command->valid_until),t::timeNs(t::stamp(101.06)));
    EXPECT_EQ(command->body_source_stamp,t::stamp(100.96));
    auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
    command->body_frame="d1max_loc_base_link";
    ASSERT_TRUE(writer.localCommand(*command,101.001));
    const auto output=writer.tick(101.+phase,sdkHealth());
    EXPECT_EQ(sdkNonzero(output),phase<.060);
    writer.writeCalled(101.+phase,true,sdkNonzero(output));
    EXPECT_FALSE(sdkNonzero(writer.tick(101.100,sdkHealth())));
    EXPECT_FALSE(writer.localCommand(*command,101.101));
  }
}
TEST(NativeSdkPhase,FreshNativeFollowerCommandDoesNotWaitAnIndependentWriterPhase) {
  // Real Native/GridMap/follower output, original 100 ms body deadline and
  // production sender scheduler/transport. Only time and the vendor endpoint
  // are passive: no SDK client or robot is constructed.
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  auto command=atSourceAge(f,.6,.44,.04);ASSERT_TRUE(command);
  ASSERT_GT(command->velocity.linear.x,0.);
  command->body_frame="d1max_loc_base_link";
  EXPECT_EQ(command->body_source_stamp,t::stamp(100.96));
  EXPECT_EQ(command->body_valid_until,t::stamp(101.06));
  auto old=sdkWriter(.6,.1),updated=sdkWriter(.6,.1);
  const auto grant=lease(101.,20);
  ASSERT_TRUE(old.taskLease(grant,101.));ASSERT_TRUE(updated.taskLease(grant,101.));
  // 22 ms from producer to receipt plus a possible 43 ms independent phase:
  // source ages remain 40 ms at production and 62 ms at SDK receipt.
  ASSERT_TRUE(old.localCommand(*command,101.022));
  ASSERT_TRUE(updated.localCommand(*command,101.022));
  EXPECT_FALSE(sdkNonzero(old.tick(101.065,sdkHealth())));
  d1monitor::CommandWriterDeadline scheduler(101.02,.05);
  scheduler.complete(101.02,101.0201,0,true); // empty health poll, not a Move
  scheduler.observe(1);
  ASSERT_LE(scheduler.next(),101.022);ASSERT_TRUE(scheduler.normalDue(101.022));
  const auto output=updated.tick(101.022,sdkHealth());
  ASSERT_TRUE(sdkNonzero(output));
  scheduler.complete(101.022,101.0221,1,true,101.022);
  updated.writeCalled(101.022,true,true);
  EXPECT_FALSE(scheduler.pending());
  EXPECT_FALSE(sdkNonzero(updated.tick(101.061,sdkHealth())));
  EXPECT_FALSE(updated.localCommand(*command,101.062));
}
TEST(NativeSdkPhase,RealSourceBeyondSixHundredMillisecondsNeverRelaysNonzero) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  // Warm with fresh, monotonic source at the allowed budget before holding it.
  auto command=atSourceAge(f,.6,.59);ASSERT_TRUE(command);
  ASSERT_GT(command->velocity.linear.x,0.);
  f.acquireBody(101.05,.21,.2); // Body continues; both rays really stop at 100.41.
  f.pipeline_health.sampled_steady_ns=101050000000LL;
  const auto expired=f.command(101.05,.21,.2,21,true,false);
  EXPECT_TRUE(!expired||!sdk::planarMotionNonzero(expired->velocity));
  EXPECT_TRUE(Access::pipelineFault(f.native));
  auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
  command->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*command,101.001));
  EXPECT_TRUE(sdkNonzero(writer.tick(101.011,sdkHealth())));
  EXPECT_FALSE(sdkNonzero(writer.tick(101.101,sdkHealth())));
  EXPECT_FALSE(writer.localCommand(*command,101.102));
  // Neither fresh localization nor repeated checks of the unchanged map can
  // make genuine sensor loss healthy again.
  f.acquireBody(101.15,.23,.2);f.pipeline_health.sampled_steady_ns=101150000000LL;
  const auto still_lost=f.command(101.15,.23,.2,22,true,false);
  EXPECT_TRUE(!still_lost||!sdk::planarMotionNonzero(still_lost->velocity));
  EXPECT_TRUE(Access::pipelineFault(f.native));
}
TEST(NativeSdkPhase,CancelAndOwnershipLossCannotResumeBufferedCurrentOrLateCommands) {
  for(const bool lost_owner:{false,true}) {
    Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    auto command=atSourceAge(f,.6,.44);ASSERT_TRUE(command);
    auto writer=sdkWriter(.6,.1);ASSERT_TRUE(writer.taskLease(lease(101.,20),101.));
    command->body_frame="d1max_loc_base_link";
    ASSERT_TRUE(writer.localCommand(*command,101.001));
    ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));
    writer.writeCalled(101.005,true,true);
    if(lost_owner){auto h=sdkHealth();h.owned=false;
      EXPECT_FALSE(sdkNonzero(writer.tick(101.015,h)));
    }else writer.stop("operator_cancel",101.015);
    EXPECT_FALSE(writer.localCommand(*command,101.016));
    writer.taskLease(lease(101.02,21),101.02);
    EXPECT_FALSE(sdkNonzero(writer.tick(101.025,sdkHealth())));
    auto late=*command;late.sequence+=100;
    EXPECT_FALSE(writer.localCommand(late,101.026));
    EXPECT_FALSE(sdkNonzero(writer.tick(101.03,sdkHealth())));
  }
}

TEST(NativeSdkTaskHandoff,NewTaskStartsAtLeaseOneAfterMovingIncumbentReallyRetires) {
  // Same production Native ledger, LocalFollower and SDK Transport throughout.
  // Only the acquired room/MC input and vendor zero ACK are simulated; neither
  // LocalMotionCommand nor the writer's selected velocity is fabricated.
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  f.braking.sensor_source_age=.6;f.braking.command_pipeline_bound=.1;
  Access::braking(f.native,f.braking);warm(f);auto writer=sdkWriter(.6,.1);
  auto incumbent_lease=lease(101.,1000);
  ASSERT_TRUE(writer.taskLease(incumbent_lease,101.));
  auto incumbent=f.command(101.,.2,.2,1000);
  ASSERT_TRUE(incumbent);ASSERT_GT(incumbent->velocity.linear.x,0.);
  incumbent->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*incumbent,101.001));
  ASSERT_TRUE(sdkNonzero(writer.tick(101.005,sdkHealth())));
  writer.writeCalled(101.005,true,true);

  auto revoke=lease(101.1,1001);revoke.allowed=false;revoke.revoked=true;
  ASSERT_TRUE(f.follower.lease(revoke,101.1));
  ASSERT_TRUE(f.native.taskLeaseAt(revoke,t::timeNs(revoke.source_stamp)));
  ASSERT_TRUE(writer.taskLease(revoke,101.1));
  auto barrier=version();barrier.reference_generation=2;
  ASSERT_TRUE(f.native.cancelReference(barrier));
  w::RouteDeliveryReceipt retired;retired.schema_version=1;
  retired.session_id=barrier.session_id;retired.task_id=barrier.task_id;
  retired.route_id=barrier.route_id;retired.route_hash=barrier.route_hash;
  retired.delivery_sequence=1;retired.active=false;retired.status=w::RouteDeliveryReceipt::RETIRED;
  retired.source_stamp=retired.receipt_stamp=t::stamp(101.1);
  retired.native_retirement_required=retired.native_retirement_confirmed=true;
  retired.native_generation=barrier.reference_generation;
  ASSERT_TRUE(f.follower.retireRoute(retired,101.1));
  ASSERT_FALSE(sdkNonzero(writer.tick(101.105,sdkHealth())));
  writer.writeCalled(101.105,true,false);writer.stopWritten(101.105);

  sdk::Grant::Request request;request.version=version();
  request.version.task_id="task-b";request.version.route_id="route-b";
  request.version.route_hash=std::string(64,'e');request.version.reference_generation=3;
  request.sdk_session="sdk";request.execution_id="execution-b";request.control_epoch=2;
  request.confirmation_id=request.request_id="task-b-click";
  request.activate=true;request.transport_mode="isolated_mock";
  request.source_stamp=t::stamp(101.11);
  EXPECT_FALSE(writer.grant(request,101.11,sdkHealth()).accepted);
  EXPECT_FALSE(writer.report(101.11).measured_stop_confirmed);
  // Fresh, monotonically acquired zero MC measurements, not merely a zero
  // command or transport ACK, satisfy the unchanged production stop contract.
  for(unsigned i=0;i<=25;++i) {
    const double now=101.15+i*.05;
    writer.mc(static_cast<std::uint64_t>(t::timeNs(t::stamp(now))),now,now,0.,0.,now,"mc-clock");
  }
  const auto stopped=writer.report(102.4);
  ASSERT_TRUE(stopped.measured_stop_confirmed);
  ASSERT_GE(stopped.stationary_duration_sec,1.);
  ASSERT_TRUE(stopped.stop_submitted);
  request.source_stamp=t::stamp(102.45);
  const auto admission=writer.grant(request,102.45,sdkHealth());
  ASSERT_TRUE(admission.accepted)<<admission.reason;
  ASSERT_EQ(admission.sdk_arm_generation,2u);

  auto next_reference=proposal();next_reference.version=request.version;
  next_reference.proposal_id="proposal-b";next_reference.source_stamp=t::stamp(102.45);
  next_reference.valid_until=t::stamp(103.35);
  ASSERT_TRUE(f.native.replacePendingReference(next_reference));
  ASSERT_TRUE(f.follower.proposal(next_reference,102.45));
  auto next_support=support();next_support.version=request.version;
  next_support.support_reference_id="support-b";next_support.source_stamp=t::stamp(102.45);
  f.native.support(next_support);ASSERT_TRUE(f.follower.support(next_support));
  auto next=tag(2,{.2,0,.55});next.task_id=request.version.task_id;
  next.route_id=request.version.route_id;next.route_hash=request.version.route_hash;
  next.generation=request.version.reference_generation;
  next.trajectory.start_time=next.join_source_stamp=t::stamp(102.45);
  f.acquire(102.45,.2,0.);
  f.native.candidate(next,next_reference,f.heading);ASSERT_TRUE(f.follower.candidate(next));
  auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(102.45)));
  ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
  ASSERT_TRUE(f.follower.validation(*proof));
  const auto committed=f.follower.tryCommit(102.45,12.45);
  ASSERT_TRUE(committed)<<f.follower.candidateReason();
  ASSERT_TRUE(f.native.localCommitAt(*committed,t::timeNs(t::stamp(102.45))));

  // The delayed old task has an even higher sequence. It may neither poison
  // task B's low sequence nor revoke its already committed geometry.
  auto late=lease(102.45,2000);
  EXPECT_FALSE(f.native.taskLeaseAt(late,t::timeNs(late.source_stamp)));
  EXPECT_FALSE(f.follower.lease(late,102.45));
  EXPECT_FALSE(writer.taskLease(late,102.45));
  EXPECT_FALSE(writer.localCommand(*incumbent,102.45));
  unsigned published=0,nonzero=0;
  for(unsigned i=0;i<5;++i) {
    const double now=102.5+i*.05;const auto ns=t::timeNs(t::stamp(now));
    f.acquire(now,.2,0.);f.native.progressAt(f.progress(now),now);
    auto current=Access::checkCurve(f.native,f.map,ns,true);
    ASSERT_TRUE(current);ASSERT_TRUE(current->valid)<<current->reason;
    ASSERT_TRUE(f.follower.validation(*current));
    auto granted=lease(now,i+1);granted.version=request.version;
    granted.execution_id=request.execution_id;granted.confirmation_id=request.confirmation_id;
    granted.control_epoch=2;granted.sdk_arm_generation=2;
    ASSERT_TRUE(f.follower.lease(granted,now));
    ASSERT_TRUE(f.native.taskLeaseAt(granted,ns))<<"new-task sequence="<<granted.sequence;
    ASSERT_TRUE(writer.taskLease(granted,now));
    EXPECT_FALSE(f.native.taskLeaseAt(granted,ns)); // Same-task replay still fails.
    ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(now)));
    const auto demand=f.follower.step(now,now-90.);
    std::optional<w::LocalMotionCommand> command;
    f.native.demandAt(demand,ns,[&](const auto& c){command=c;});
    ASSERT_TRUE(command);++published;
    EXPECT_EQ(command->lease_sequence,granted.sequence);
    EXPECT_EQ(command->execution_id,request.execution_id);
    command->body_frame="d1max_loc_base_link";
    ASSERT_TRUE(writer.localCommand(*command,now+.001));
    const auto output=writer.tick(now+.005,sdkHealth());
    if(sdkNonzero(output))++nonzero;
    writer.writeCalled(now+.005,true,sdkNonzero(output));
  }
  EXPECT_EQ(published,5u);EXPECT_GT(nonzero,0u);
  std::cout<<"task handoff A lease=1000 moved; stop_samples="<<stopped.stationary_samples
    <<" stop_duration="<<stopped.stationary_duration_sec<<" B lease=1..5 relayed="<<published
    <<" sdk_nonzero="<<nonzero<<'\n';
}
TEST(NativeSdkPhase,ReservedCommitBodyAndDemandAllDeliveryOrdersKeepOriginalCommandContinuous) {
  // The 2026-10-20 log has SDK expiry without a follower zero. A cross-topic
  // ordering is a hypothesis, not a log-proven cause. Exercise the deployed
  // Native/Follower/SDK implementation before attributing it to this boundary.
  // 0=actual body delivery, 1=real geometry commit/ACK, 2=original incumbent
  // demand emitted while the final ACK was pending. None may change identity.
  std::array<int,3> order{0,1,2};
  unsigned permutations=0;
  do {
    SCOPED_TRACE(::testing::Message()<<order[0]<<order[1]<<order[2]);
    Integrated f(true,false,{},tag(2,{0,0,.55},{.2,0,0},{0,0,0}));
    auto incumbent=f.command(100.10,.02,.2,1);ASSERT_TRUE(incumbent);
    ASSERT_GT(incumbent->velocity.linear.x,0.);
    auto writer=sdkWriter(.5,.1,100.10);
    ASSERT_TRUE(writer.taskLease(lease(100.10,1),100.10));
    incumbent->body_frame="d1max_loc_base_link";
    ASSERT_TRUE(writer.localCommand(*incumbent,100.101));
    ASSERT_TRUE(sdkNonzero(writer.tick(100.105,sdkHealth())));
    writer.writeCalled(100.105,true,true);

    auto replacement=tag(4,{.02,0,.55},{.2,0,0},{0,0,0});
    replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.10);
    f.native.candidate(replacement,f.reference,f.heading);
    ASSERT_TRUE(f.follower.candidate(replacement));
    const auto proof=Access::checkCurve(f.native,f.map,100120000000LL);
    ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(f.follower.validation(*proof));
    const auto prepare=f.follower.exchangeCommit(100.12,10.12);ASSERT_TRUE(prepare);
    ASSERT_EQ(prepare->disposition,"prepare");
    const auto prepared=f.native.localCommitExchangeAt(*prepare,100121000000LL);
    ASSERT_TRUE(prepared.accepted);ASSERT_TRUE(f.follower.commitResult(prepared,100.122,10.122));

    // Concurrent newer solves/publications cannot evict the pinned curve4
    // exchange or reinterpret its first demand as curve5/6. This also tests
    // the independent selected consumer between prepare and commit delivery.
    Access::selectedObserver(f.native,[&](const auto& selected){EXPECT_TRUE(f.follower.candidate(selected));});
    for(unsigned id:{5u,6u}) {
      auto newest=tag(id,{.02,0,.55},{.2,0,0},{0,0,0});
      newest.trajectory.start_time=newest.join_source_stamp=t::stamp(100.12);
      f.native.candidate(newest,f.reference,f.heading);Access::dispatchSelected(f.native);
    }

    // Only the follower has received this raw sample yet. Native cannot
    // substitute the commit's pose for its independent raw body delivery.
    const auto measured=body(100.13,.026,.2);
    ASSERT_TRUE(f.follower.body(measured,100.13,10.13));
    const auto commit=f.follower.exchangeCommit(100.13,10.13);ASSERT_TRUE(commit);
    ASSERT_EQ(commit->disposition,"committed");ASSERT_EQ(commit->trajectory_id,4);
    ASSERT_EQ(f.native.committedTrajectory(),2);
    ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(100.13)));
    const auto demand=f.follower.step(100.13,10.13);
    ASSERT_FALSE(demand.hold);ASSERT_EQ(demand.trajectory_id,2);
    ASSERT_EQ(demand.body_source_stamp,t::stamp(100.13));
    std::vector<w::LocalMotionCommand> emitted;
    double delivery=100.13;
    const auto emit=[&](const auto& c) {
      emitted.push_back(c);auto outbound=c;outbound.body_frame="d1max_loc_base_link";
      EXPECT_TRUE(writer.localCommand(outbound,delivery));
    };
    for(unsigned i=0;i<order.size();++i) {
      delivery=100.135+i*.005;const auto now_ns=t::timeNs(t::stamp(delivery));
      if(order[i]==0) {
        geometry_msgs::msg::Twist twist;twist.linear.x=.2;
        f.native.bodyAt({measured.position,measured.orientation,measured.stamp,"odom"},
          twist,measured.source_stamp_ns,now_ns,emit);
      } else if(order[i]==1) {
        const auto ack=f.native.localCommitExchangeAt(*commit,now_ns,emit);
        ASSERT_TRUE(ack.accepted)<<ack.reason;
        ASSERT_TRUE(f.follower.commitResult(ack,delivery,delivery-90.));
      } else f.native.demandAt(demand,now_ns,emit);
    }
    ASSERT_EQ(emitted.size(),1u);
    EXPECT_EQ(emitted.front().source_stamp,demand.source_stamp);
    EXPECT_EQ(emitted.front().body_source_stamp,demand.body_source_stamp);
    EXPECT_EQ(emitted.front().sequence,demand.sequence);
    EXPECT_EQ(emitted.front().velocity,demand.velocity);
    EXPECT_LE(t::timeNs(emitted.front().valid_until),100230000000LL);
    ASSERT_EQ(f.follower.controller().trajectoryId(),4);
    ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(100.15)));
    const auto successor=f.follower.step(100.15,100.15-90.);
    ASSERT_FALSE(successor.hold);ASSERT_EQ(successor.trajectory_id,4);
    delivery=100.151;f.native.demandAt(successor,100151000000LL,emit);
    ASSERT_EQ(emitted.size(),2u);EXPECT_EQ(emitted.back().trajectory_id,4);
    // Even a higher-sequence predecessor cannot reopen the overlap after the
    // first exact successor demand has selected the acknowledged geometry.
    auto reordered=demand;reordered.sequence=successor.sequence+1;
    f.native.demandAt(reordered,100152000000LL,emit);EXPECT_EQ(emitted.size(),2u);
    EXPECT_TRUE(sdkNonzero(writer.tick(100.155,sdkHealth())));
    writer.writeCalled(100.155,true,true);
    EXPECT_FALSE(sdkNonzero(writer.tick(100.231,sdkHealth())));
    ++permutations;
  } while(std::next_permutation(order.begin(),order.end()));
  EXPECT_EQ(permutations,6u);
}
TEST(NativeSdkPhase,RejectedAndDeferredDemandDiagnosticsIdentifyOriginalSequenceWithoutChangingOutput) {
  using Stage=scan_planner::MotionCheckStage;
  const std::array<Stage,7> expected{{Stage::DemandContractInvalid,Stage::DemandAuthorityMismatch,
    Stage::DemandSequenceNotNew,Stage::ActiveGeometryMismatch,Stage::CandidateProofUnavailable,
    Stage::CandidateSourceExpired,Stage::BodyBehindDemand}};
  for(unsigned fault=0;fault<expected.size();++fault) {
    SCOPED_TRACE(fault);
    Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    const auto sent=f.command(100.05,.01,.2,1);ASSERT_TRUE(sent);
    auto demand=*Access::latestDemand(f.native);++demand.sequence;
    demand.source_stamp=demand.body_source_stamp=t::stamp(100.06);
    demand.valid_until=t::stamp(100.26);double now=100.06;
    if(fault==0)demand.transport_mode="wrong_transport";
    if(fault==1)demand.execution_id="other_execution";
    if(fault==2)demand.sequence=sent->sequence;
    if(fault==3)demand.trajectory_id=999;
    if(fault==4||fault==5) {
      auto candidate=tag(2,{.01,0,.55},{.2,0,0},{0,0,0});
      candidate.trajectory.start_time=candidate.join_source_stamp=t::stamp(100.05);
      f.native.candidate(candidate,f.reference,f.heading);demand.trajectory_id=2;
      if(fault==5) {
        const auto proof=Access::checkCurve(f.native,f.map,100060000000LL);
        ASSERT_TRUE(proof&&proof->valid);demand.validation_sequence=proof->sequence;
        demand.body_source_stamp=t::stamp(99.95);
      }
    }
    const auto original=demand;
    unsigned emitted=0;f.native.demandAt(demand,t::timeNs(t::stamp(now)),[&](const auto&){++emitted;});
    EXPECT_EQ(emitted,0u);EXPECT_EQ(demand,original);
    const auto diagnostics=Access::diagnostics(f.native);
    ASSERT_EQ(diagnostics.stage,expected[fault]);
    ASSERT_GT(diagnostics.event_sequence,0u);
    const auto& event=diagnostics.events[(diagnostics.event_sequence-1)%
      scan_planner::MotionCheckDiagnostics::event_capacity];
    EXPECT_EQ(event.stage,expected[fault]);EXPECT_EQ(event.command_sequence,demand.sequence);
    EXPECT_EQ(event.curve,demand.trajectory_id);EXPECT_EQ(event.active_curve,1);
    EXPECT_EQ(event.command_source_ns,t::timeNs(demand.source_stamp));
    EXPECT_EQ(event.body_source_ns,t::timeNs(demand.body_source_stamp));
    EXPECT_EQ(event.native_body_source_ns,100050000000LL);
    EXPECT_EQ(event.observed_at_ns,100060000000LL);
  }
}
TEST(NativeSdkPhase,PreparedAckBetweenTicksDoesNotRetireTheInFlightIncumbentDemand) {
  // Recorded E17 order: demand 1383 (curve 428) was computed at a control
  // tick, then Native committed 429 before receiving that demand. Rejecting
  // the old curve is correct, but command 1382 expired 44 ms later. Reproduce
  // the same transport order with the production follower/Native/SDK cores.
  // This switch reproduces ONLY the old callback scheduling in this test.
  const bool old_callback_activation=std::getenv("D1MAX_TEST_ASYNC_GEOMETRY_ACTIVATION")!=nullptr;
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  f.acquire(100.09,.018,.2);
  auto incumbent=f.command(100.10,.018,.2,1,true,false);ASSERT_TRUE(incumbent);
  auto writer=sdkWriter(.5,.1,100.10);
  ASSERT_TRUE(writer.taskLease(lease(100.10,1),100.10));
  incumbent->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*incumbent,100.101));
  ASSERT_TRUE(sdkNonzero(writer.tick(100.105,sdkHealth())));
  writer.writeCalled(100.105,true,true);

  f.acquire(100.14,.028,.2);
  auto replacement=tag(2,{.028,0,.55},{.2,0,0},{0,0,0});
  replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.14);
  f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
  const auto proof=Access::checkCurve(f.native,f.map,100140000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(f.follower.validation(*proof));
  const auto prepare=f.follower.exchangeCommit(100.14,10.14);ASSERT_TRUE(prepare);
  ASSERT_EQ(prepare->disposition,"prepare");
  const auto prepared=f.native.localCommitExchangeAt(*prepare,100141000000LL);
  ASSERT_TRUE(prepared.accepted);

  // At the next actual control tick the prepared ACK is still in transit.
  ASSERT_EQ(f.follower.exchangeCommit(100.15,10.15)->disposition,"prepare");
  ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(100.15)));
  const auto old_demand=f.follower.step(100.15,10.15);
  ASSERT_FALSE(old_demand.hold);ASSERT_EQ(old_demand.trajectory_id,1);
  const double old_forward=old_demand.velocity.linear.x;
  ASSERT_TRUE(f.follower.commitResult(prepared,100.1502,10.1502));
  using Clock=t::LocalFollower::ExchangeClock;
  const auto callback=f.follower.exchangeCommit(100.1503,10.1503,t::stamp(100.1503),
    old_callback_activation?Clock::ControlTick:Clock::InputDelivery);
  if(callback) {
    ASSERT_EQ(callback->disposition,"committed");
    const auto ack=f.native.localCommitExchangeAt(*callback,100151000000LL);
    ASSERT_TRUE(ack.accepted);ASSERT_TRUE(f.follower.commitResult(ack,100.151,10.151));
  }
  std::optional<w::LocalMotionCommand> relayed;
  f.native.demandAt(old_demand,100152000000LL,[&](const auto& c){relayed=c;});
  if(relayed) {
    auto command=*relayed;command.body_frame="d1max_loc_base_link";
    ASSERT_TRUE(writer.localCommand(command,100.152));
    EXPECT_EQ(command.source_stamp,old_demand.source_stamp);
    EXPECT_EQ(command.body_source_stamp,old_demand.body_source_stamp);
  }
  EXPECT_EQ(f.native.committedTrajectory(),1);
  EXPECT_TRUE(relayed.has_value());
  // Same source/100 ms deadline: only preventing premature retirement fills
  // this gap. Extending an old command or accepting it as curve 2 is forbidden.
  EXPECT_TRUE(sdkNonzero(writer.tick(100.195,sdkHealth())));
  writer.writeCalled(100.195,true,relayed.has_value());

  f.acquire(100.20,.04,.2);
  const auto commit=f.follower.exchangeCommit(100.20,100.20-90.,t::stamp(100.20),Clock::ControlTick);
  if(commit) {
    ASSERT_EQ(commit->disposition,"committed");
    const auto ack=f.native.localCommitExchangeAt(*commit,100201000000LL);
    ASSERT_TRUE(ack.accepted);ASSERT_TRUE(f.follower.commitResult(ack,100.201,10.201));
  }
  ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(100.20)));
  const auto current=f.follower.step(100.20,100.20-90.);
  EXPECT_EQ(current.trajectory_id,2);EXPECT_FALSE(current.hold)<<current.reason<<' '<<f.follower.controller().reason();
  EXPECT_GT(current.velocity.linear.x,0.);
  EXPECT_LE(std::abs(current.velocity.linear.x-old_forward),.35*.05+1e-9);
  relayed.reset();f.native.demandAt(current,100202000000LL,[&](const auto& c){relayed=c;});
  ASSERT_TRUE(relayed);relayed->body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(*relayed,100.202));
  EXPECT_TRUE(sdkNonzero(writer.tick(100.205,sdkHealth())));
  writer.writeCalled(100.205,true,true);
  EXPECT_FALSE(sdkNonzero(writer.tick(100.301,sdkHealth())));
}
TEST(NativeSdkPhase,RejectedCommitKeepsFreshIncumbentDemandUntilReply) {
  using Trace=scan_planner::BoundedExecutionTrace;
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  ASSERT_TRUE(f.command(100.10,.02,.2,1));
  auto replacement=tag(2,{.02,0,.55},{.2,0,0},{0,0,0});
  replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.10);
  f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
  const auto proof=Access::checkCurve(f.native,f.map,100120000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(f.follower.validation(*proof));
  const auto prepare=f.follower.exchangeCommit(100.12,10.12);ASSERT_TRUE(prepare);
  const auto prepared=f.native.localCommitExchangeAt(*prepare,100121000000LL);
  ASSERT_TRUE(prepared.accepted);ASSERT_TRUE(f.follower.commitResult(prepared,100.122,10.122));
  f.acquire(100.13,.026,.2);const auto commit=f.follower.exchangeCommit(100.13,10.13);
  ASSERT_TRUE(commit);ASSERT_EQ(commit->disposition,"committed");
  auto newer=tag(3,{.026,0,.55},{.2,0,0},{0,0,0});
  newer.trajectory.start_time=newer.join_source_stamp=t::stamp(100.13);
  f.native.candidate(newer,f.reference,f.heading);
  Access::forgetOriginalProofs(f.native);
  const auto rejected=f.native.localCommitExchangeAt(*commit,100134000000LL);
  ASSERT_FALSE(rejected.accepted);
  const auto wrong=f.follower.step(100.135,10.135);ASSERT_EQ(wrong.trajectory_id,1);
  unsigned emitted=0;f.native.demandAt(wrong,100136000000LL,[&](const auto&){++emitted;});
  EXPECT_EQ(emitted,1u);
  ASSERT_TRUE(f.follower.commitResult(rejected,100.14,10.14));
  const auto resumed=f.command(100.15,.03,.2,2);ASSERT_TRUE(resumed);EXPECT_EQ(resumed->trajectory_id,1);
  const auto trace=Access::executionTrace(f.native);
  const auto curve=[&](std::uint64_t id){
    if(!id)return std::int64_t{-1};const auto& value=trace.identities[(id-1)%Trace::identity_capacity];
    EXPECT_EQ(value.id,id);return value.curve;
  };
  bool rejection=false,mismatch=false,recovery=false;
  for(const auto& e:trace.events)if(e.id) {
    if(e.kind==Trace::Kind::CommitDecision&&e.reason.str()=="local_commit_measured_join_rejected") {
      rejection=true;EXPECT_FALSE(e.accepted);EXPECT_EQ(e.sequence,commit->commit_sequence);
      EXPECT_EQ(curve(e.identity[0]),2);EXPECT_EQ(curve(e.identity[1]),1);
      EXPECT_EQ(curve(e.identity[2]),1);EXPECT_EQ(curve(e.identity[3]),-1);
      EXPECT_EQ(curve(e.before_identity[3]),2);EXPECT_EQ(curve(e.identity[4]),3);
      EXPECT_GT(e.begin_steady_ns,0);EXPECT_GE(e.steady_ns,e.begin_steady_ns);
    }
    if(e.kind==Trace::Kind::DemandRejected&&e.reason.str()=="active_geometry_mismatch") {
      mismatch=true;EXPECT_EQ(e.sequence,wrong.sequence);EXPECT_EQ(e.source_ns,t::timeNs(wrong.source_stamp));
      EXPECT_EQ(e.body_ns,t::timeNs(wrong.body_source_stamp));EXPECT_EQ(curve(e.identity[0]),2);
      EXPECT_EQ(curve(e.identity[2]),1);EXPECT_EQ(curve(e.identity[4]),3);
    }
    if(e.kind==Trace::Kind::CommandPublishReturned&&e.sequence==resumed->sequence) {
      recovery=true;EXPECT_EQ(curve(e.identity[0]),1);EXPECT_EQ(e.velocity[0],resumed->velocity.linear.x);
      EXPECT_EQ(e.valid_until_ns,t::timeNs(resumed->valid_until));EXPECT_GE(e.steady_ns,e.begin_steady_ns);
    }
  }
  EXPECT_TRUE(rejection);EXPECT_FALSE(mismatch);EXPECT_TRUE(recovery);
}

TEST(NativeSdkPhase,DelayedCommitAckBridgesExactIncumbentThenRetiresItWithoutZero) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  const auto before=f.command(100.10,.02,.2,1);ASSERT_TRUE(before);
  auto writer=sdkWriter(.5,.1,100.10);ASSERT_TRUE(writer.taskLease(lease(100.10,1),100.10));
  auto initial=*before;initial.body_frame="d1max_loc_base_link";
  ASSERT_TRUE(writer.localCommand(initial,100.101));
  ASSERT_TRUE(sdkNonzero(writer.tick(100.105,sdkHealth())));writer.writeCalled(100.105,true,true);
  auto replacement=tag(2,{.02,0,.55},{.2,0,0},{0,0,0});
  replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.10);
  f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
  auto proof=Access::checkCurve(f.native,f.map,100120000000LL);ASSERT_TRUE(proof&&proof->valid);
  ASSERT_TRUE(f.follower.validation(*proof));
  const auto request=f.follower.exchangeCommit(100.12,10.12);ASSERT_TRUE(request);
  auto reply=f.native.localCommitExchangeAt(*request,100121000000LL);ASSERT_TRUE(reply.accepted);
  ASSERT_TRUE(f.follower.commitResult(reply,100.122,10.122));
  f.acquire(100.15,.03,.2);
  const auto commit=f.follower.exchangeCommit(100.15,100.15-90.);ASSERT_TRUE(commit);
  ASSERT_EQ(commit->disposition,"committed");
  // The commit request and command can be delivered in either order. Tracker
  // keeps the exact incumbent until ACK; SCAN keeps its exact predecessor
  // across ACK flight, so neither transport ordering creates a missing tick.
  f.follower.geometryPermit(100.15);auto demand=f.follower.step(100.15,100.15-90.);
  ASSERT_EQ(demand.trajectory_id,1);ASSERT_FALSE(demand.hold)<<demand.reason<<' '<<f.follower.controller().reason();
  reply=f.native.localCommitExchangeAt(*commit,100151000000LL);ASSERT_TRUE(reply.accepted);
  EXPECT_EQ(f.native.committedTrajectory(),2);
  std::optional<w::LocalMotionCommand> command;
  f.native.demandAt(demand,100152000000LL,[&](const auto& c){command=c;});
  ASSERT_TRUE(command);EXPECT_EQ(command->trajectory_id,1);
  EXPECT_EQ(command->source_stamp,demand.source_stamp);EXPECT_EQ(command->body_source_stamp,demand.body_source_stamp);
  command->body_frame="d1max_loc_base_link";ASSERT_TRUE(writer.localCommand(*command,100.152));
  EXPECT_TRUE(sdkNonzero(writer.tick(100.195,sdkHealth())));writer.writeCalled(100.195,true,true);
  ASSERT_TRUE(f.follower.commitResult(reply,100.196,10.196));
  f.acquire(100.20,.04,.2);f.follower.geometryPermit(100.20);
  const auto next=f.follower.step(100.20,100.20-90.);ASSERT_EQ(next.trajectory_id,2);ASSERT_FALSE(next.hold)<<next.reason;
  command.reset();f.native.demandAt(next,100201000000LL,[&](const auto& c){command=c;});
  ASSERT_TRUE(command);EXPECT_EQ(command->trajectory_id,2);
  EXPECT_LE(std::abs(next.velocity.linear.x-demand.velocity.linear.x),.35*.05+1e-9);
  command->body_frame="d1max_loc_base_link";ASSERT_TRUE(writer.localCommand(*command,100.201));
  EXPECT_TRUE(sdkNonzero(writer.tick(100.205,sdkHealth())));writer.writeCalled(100.205,true,true);
  // Even a fresh malicious predecessor sequence cannot reopen the handover.
  demand.sequence=next.sequence+1;demand.source_stamp=demand.body_source_stamp=t::stamp(100.202);
  demand.valid_until=t::stamp(100.302);unsigned late=0;
  f.native.demandAt(demand,100202000000LL,[&](const auto&){++late;});EXPECT_EQ(late,0u);
  EXPECT_FALSE(sdkNonzero(writer.tick(100.301,sdkHealth()))); // no source renewal
}

TEST(NativeSdkPhase,CommitAckOverlapCannotBypassNegativeExpiryCancelOrTaskIdentity) {
 for(int barrier=0;barrier<6;++barrier) {
  SCOPED_TRACE(barrier);
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  ASSERT_TRUE(f.command(100.10,.02,.2,1));
  auto replacement=tag(2,{.02,0,.55},{.2,0,0},{0,0,0});
  replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.10);
  f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
  const auto proof=Access::checkCurve(f.native,f.map,100120000000LL);ASSERT_TRUE(proof&&proof->valid);
  ASSERT_TRUE(f.follower.validation(*proof));
  const auto request=f.follower.exchangeCommit(100.12,10.12);ASSERT_TRUE(request);
  auto ack=f.native.localCommitExchangeAt(*request,100121000000LL);ASSERT_TRUE(ack.accepted);
  ASSERT_TRUE(f.follower.commitResult(ack,100.122,10.122));
  f.acquire(100.15,.03,.2);f.native.progressAt(f.progress(100.15),100.15);
  const auto commit=f.follower.exchangeCommit(100.15,100.15-90.);ASSERT_TRUE(commit);
  f.follower.geometryPermit(100.15);auto demand=f.follower.step(100.15,100.15-90.);
  ASSERT_EQ(demand.trajectory_id,1);ASSERT_FALSE(demand.hold)<<demand.reason<<' '<<f.follower.controller().reason();
  if(barrier==0) {
    // The current reader copied the old Job before the new commit. Its actual
    // occupied result must still land on that exact predecessor afterward.
    GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
    const auto negative=Access::checkCurrentAcrossCommit(f.native,f.map,100151000000LL,[&]{
      ack=f.native.localCommitExchangeAt(*commit,100151000000LL);EXPECT_TRUE(ack.accepted);
    });
    ASSERT_TRUE(negative);ASSERT_FALSE(negative->valid);EXPECT_EQ(negative->trajectory_id,1);
  } else {ack=f.native.localCommitExchangeAt(*commit,100151000000LL);ASSERT_TRUE(ack.accepted);}
  std::int64_t now=100152000000LL;
  if(barrier==1)now=100251000000LL; // original command/body expiry, ACK remains lost
  if(barrier==2) {
    auto cancel=version();++cancel.reference_generation;ASSERT_TRUE(f.native.cancelReference(cancel));
    f.follower.retire("cancel");EXPECT_FALSE(f.follower.commitResult(ack,100.152,10.152));
  }
  if(barrier==3)demand.version.task_id="other_task";
  if(barrier==4) {
    auto hold=lease(100.151,2);hold.allowed=false;hold.phase="holding";
    ASSERT_TRUE(f.native.taskLeaseAt(hold,100151000000LL));
  }
  if(barrier==5) {
    // Persistent predecessor state cannot bypass genuine input loss. Body,
    // demand and task are fresh; the last ray pair remains at 100.15.
    now=100751000000LL;f.acquireBody(100.751,.1502,.2);
    f.pipeline_health.sampled_steady_ns=now;
    demand.source_stamp=demand.body_source_stamp=t::stamp(100.751);
    demand.valid_until=t::stamp(100.851);
    ASSERT_TRUE(f.native.taskLeaseAt(lease(100.751,2),now));
  }
  unsigned emitted=0;f.native.demandAt(demand,now,[&](const auto&){++emitted;});EXPECT_EQ(emitted,0u);
 }
}

TEST(NativeSdkPhase,IncumbentBodyDeliveryPendingAcrossCommitKeepsOriginalCommand) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  ASSERT_TRUE(f.command(100.10,.02,.2,1));
  auto replacement=tag(2,{.02,0,.55},{.2,0,0},{0,0,0});
  replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.10);
  f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
  const auto proof=Access::checkCurve(f.native,f.map,100120000000LL);ASSERT_TRUE(proof&&proof->valid);
  ASSERT_TRUE(f.follower.validation(*proof));
  const auto request=f.follower.exchangeCommit(100.12,10.12);ASSERT_TRUE(request);
  auto reply=f.native.localCommitExchangeAt(*request,100121000000LL);ASSERT_TRUE(reply.accepted);
  ASSERT_TRUE(f.follower.commitResult(reply,100.122,10.122));
  // Tracker receives the real acquired state first; the exact same state is
  // still in transit to Native, independent of the commit channel.
  const auto measured=body(100.15,.03,.2);ASSERT_TRUE(f.follower.body(measured,100.15,100.15-90.));
  const auto commit=f.follower.exchangeCommit(100.15,100.15-90.);ASSERT_TRUE(commit);
  f.follower.geometryPermit(100.15);const auto demand=f.follower.step(100.15,100.15-90.);
  ASSERT_EQ(demand.trajectory_id,1);ASSERT_FALSE(demand.hold);
  std::vector<w::LocalMotionCommand> emitted;const auto emit=[&](const auto& c){emitted.push_back(c);};
  f.native.demandAt(demand,100151000000LL,emit);ASSERT_TRUE(emitted.empty());
  ASSERT_TRUE(Access::pendingDemand(f.native));
  reply=f.native.localCommitExchangeAt(*commit,100152000000LL,emit);ASSERT_TRUE(reply.accepted);
  EXPECT_TRUE(emitted.empty());ASSERT_TRUE(Access::pendingDemand(f.native));
  geometry_msgs::msg::Twist twist;twist.linear.x=.2;
  f.native.bodyAt({measured.position,measured.orientation,measured.stamp,"odom"},twist,
    measured.source_stamp_ns,100153000000LL,emit);
  ASSERT_EQ(emitted.size(),1u);EXPECT_EQ(emitted[0].trajectory_id,1);
  EXPECT_EQ(emitted[0].sequence,demand.sequence);EXPECT_EQ(emitted[0].source_stamp,demand.source_stamp);
  EXPECT_EQ(emitted[0].body_source_stamp,demand.body_source_stamp);
  EXPECT_LE(t::timeNs(emitted[0].valid_until),100250000000LL);
  EXPECT_FALSE(Access::pendingDemand(f.native));
}
