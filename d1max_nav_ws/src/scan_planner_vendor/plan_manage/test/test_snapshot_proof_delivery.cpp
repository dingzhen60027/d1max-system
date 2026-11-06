#include "native_local_follower_test_fixture.hpp"
#include <future>
#include <fstream>

namespace {
void publishBoth(Integrated& f,std::int64_t now) {
  ASSERT_TRUE(f.pool.publishValidation(f.map,now,std::chrono::steady_clock::now()));
  ASSERT_TRUE(f.pool.refreshOwnerReplica(f.map,now,std::chrono::steady_clock::now()));
}
}

TEST(SnapshotProofDelivery,ProofSinkDoesNotPinCompletedNativeQueryOrEraseRecycledProviders) {
  for(const bool stair:{false,true})for(const bool obstacle:{false,true}) {
    auto s=support();
    if(stair){s.floor_id="stairs";s.segment_kind="stair_up";s.required_mode="stair";
      s.reason="original_stair_surface_returns_associated";s.max_support_slope_rad=s.max_support_step_m=0.;}
    Integrated f(true,false,{},{},false,s);
    f.acquire(100.01,0.,0.);
    if(obstacle)GridMapTestAccess::voxel(f.map,{.2,0.,.55},2.);
    publishBoth(f,100010000000LL);
    auto owner=f.pool.borrowOwnerValidation();ASSERT_TRUE(owner);
    unsigned delivered=0;bool head_available=false,providers_restored=false,published=false;
    const auto proof=Access::checkPooledCurve(f.native,100010000000LL,[&](const auto& result) {
      ++delivered;
      // This sink executes inside real checkAt, after the actual whole-curve
      // GridMap query. Owner keeps the other validation slot throughout.
      auto head=f.pool.borrowValidation();head_available=bool(head);
      if(head)providers_restored=!GridMapTestAccess::anyQueryProvider(*head);
      head.reset();
      EXPECT_EQ(result.valid,!obstacle);
      if(obstacle){EXPECT_EQ(result.reason,"occupied");}
      EXPECT_EQ(t::timeNs(result.front_ray_source_stamp),100010000000LL);
      EXPECT_EQ(t::timeNs(result.rear_ray_source_stamp),100010000000LL);
      EXPECT_EQ(t::timeNs(result.body_source_stamp),100010000000LL);
      EXPECT_EQ(t::timeNs(result.valid_until),0); // Ordered completed state, not a map lease.
      // Publish a genuinely later acquisition during the output callback.
      // It must not rewrite result's source, deadline or collision verdict.
      GridMapTestAccess::acquire(f.map,100100000000LL);
      published=f.pool.publishValidation(f.map,100100000000LL,std::chrono::steady_clock::now());
      if(published) {
        auto recycled=f.pool.borrowValidation();ASSERT_TRUE(recycled);
        EXPECT_EQ(recycled->integratedRaySourceStamp(0),100100000000LL);
        recycled->setQueryAttitudeProvider([](const Eigen::Vector3d&,double){
          return std::optional<Eigen::Vector2d>(Eigen::Vector2d::Zero());});
      }
    });
    ASSERT_TRUE(proof);EXPECT_EQ(delivered,1U);
    EXPECT_TRUE(head_available);EXPECT_TRUE(providers_restored);EXPECT_TRUE(published);
    auto after=f.pool.borrowValidation();ASSERT_TRUE(after);
    // A delayed old StairAttitudeScope destructor would overwrite this new
    // owner's binding after recycling. No old provider may survive release.
    if(published){EXPECT_TRUE(GridMapTestAccess::anyQueryProvider(*after));}
    EXPECT_EQ(proof->valid,!obstacle);
  }
}

TEST(SnapshotProofDelivery,CommandSinkDoesNotPinMapAndPreservesDemandSource) {
  Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
  f.braking.sensor_source_age=.6;f.braking.command_pipeline_bound=.1;
  Access::braking(f.native,f.braking);GridMapTestAccess::sourceBudget(f.map,.6);
  // A fresh command whose exact current state is unavailable is deferred.
  // Completing that state must release the map before command publication;
  // ordinary map updates never renew an already emitted command.
  for(unsigned i=1;i<20;++i){const double now=100.+i*.05;
    f.acquire(now,(now-100.)*.2,.2);
    GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(std::max(100.,now-.559))));
    ASSERT_TRUE(f.command(now,(now-100.)*.2,.2,i,true,false));
  }
  f.acquire(101.,.2,.2);GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(100.441)));
  const auto first=f.command(101.,.2,.2,20,true,false);ASSERT_TRUE(first);
  ASSERT_FALSE(first->hold);ASSERT_GT(first->velocity.linear.x,0.);
  ASSERT_EQ(first->valid_until,t::stamp(101.1));
  f.acquire(101.02,.204,.2);GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(100.60)));
  f.native.progressAt(f.progress(101.02),101.02);
  const auto grant=lease(101.02,21);ASSERT_TRUE(f.follower.lease(grant,101.02));
  ASSERT_TRUE(f.native.taskLeaseAt(grant,101020000000LL));
  ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(101.02)));
  const auto demand=f.follower.step(101.02,101.02-90.);ASSERT_FALSE(demand.hold);
  Access::removeActiveProof(f.native);
  unsigned early=0;f.native.demandAt(demand,101021000000LL,[&](const auto&){++early;});
  ASSERT_EQ(early,0U);ASSERT_TRUE(Access::pendingDemand(f.native));
  publishBoth(f,101025000000LL);
  auto owner=f.pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  unsigned delivered=0;bool published=false;
  const auto proof=Access::checkPooledCurve(f.native,101025000000LL,{},[&](const auto& command) {
    ++delivered;
    EXPECT_EQ(command.sequence,demand.sequence);
    EXPECT_EQ(command.source_stamp,demand.source_stamp);
    EXPECT_EQ(command.body_source_stamp,demand.body_source_stamp);
    EXPECT_EQ(command.velocity,demand.velocity);
    EXPECT_EQ(command.valid_until,t::stamp(101.12));
    EXPECT_EQ(command.body_valid_until,t::stamp(101.12));
    GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(100.61)));
    published=f.pool.publishValidation(f.map,101025000000LL,std::chrono::steady_clock::now());
  });
  ASSERT_TRUE(proof);EXPECT_TRUE(proof->valid);
  EXPECT_EQ(delivered,1U);EXPECT_TRUE(published);
  EXPECT_FALSE(Access::pendingDemand(f.native));
  EXPECT_FALSE(Access::checkDemand(f.native,f.map,101121000000LL)); // Original body/command expired.
}

TEST(SnapshotProofDelivery,RetirementBetweenQueryAndLedgerCannotPublishOldProof) {
  Integrated f;publishBoth(f,100000000000LL);
  bool released=false;
  const auto proof=Access::checkPooledCurve(f.native,100000000000LL,{}, {},[&] {
    released=true;
    auto barrier=version();++barrier.reference_generation;
    EXPECT_TRUE(f.native.cancelReference(barrier));
  });
  EXPECT_TRUE(released);EXPECT_FALSE(proof);
}

TEST(SnapshotProofDelivery,ThrowingOutputStillLeavesRestoredReusableNativeSnapshot) {
  auto s=support();s.segment_kind="stair_down";s.floor_id="stairs";s.required_mode="stair";
  s.reason="original_stair_surface_returns_associated";s.max_support_slope_rad=s.max_support_step_m=0.;
  Integrated f(true,false,{},{},false,s);publishBoth(f,100000000000LL);
  auto owner=f.pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  EXPECT_THROW(Access::checkPooledCurve(f.native,100000000000LL,[](const auto&){
    throw std::runtime_error("output_sink_failure");}),std::runtime_error);
  auto head=f.pool.borrowValidation();ASSERT_TRUE(head);
  EXPECT_FALSE(GridMapTestAccess::anyQueryProvider(*head));head.reset();
  EXPECT_TRUE(f.pool.publishValidation(f.map,100000000000LL,std::chrono::steady_clock::now()));
}

TEST(SnapshotProofDelivery,AdmissionSlowProofSinkDoesNotBlockNextSolverLease) {
  for(const bool stair:{false,true})for(const bool obstacle:{false,true}) {
    auto s=support();
    if(stair){s.floor_id="stairs";s.segment_kind="stair_down";s.required_mode="stair";
      s.reason="original_stair_surface_returns_associated";s.max_support_slope_rad=s.max_support_step_m=0.;}
    Integrated f(true,false,{},{},false,s);
    f.acquire(100.01,0.,0.);
    if(obstacle)GridMapTestAccess::voxel(f.map,{.2,0.,.55},2.);
    auto candidate=tag(2);candidate.trajectory.start_time=candidate.join_source_stamp=t::stamp(100.01);
    f.native.candidate(candidate,f.reference,f.heading);
    publishBoth(f,100010000000LL);
    ASSERT_TRUE(f.pool.publishSolver(f.map,100010000000LL,std::chrono::steady_clock::now()));
    std::promise<void> publishing,release;auto publishing_started=publishing.get_future();
    auto released=release.get_future();
    auto worker=std::async(std::launch::async,[&] {
      return Access::checkPooledCandidate(f.native,100010000000LL,[&](const auto&){
        publishing.set_value();released.wait(); // Deterministic blocked publication, no timing assumption.
      });
    });
    const auto status=publishing_started.wait_for(std::chrono::seconds(3));
    EXPECT_EQ(status,std::future_status::ready);
    bool available=false,restored=false;
    if(status==std::future_status::ready) {
      auto next=f.pool.borrowSolver();available=bool(next);
      if(next) {
        restored=!GridMapTestAccess::anyQueryProvider(*next);
        EXPECT_EQ(next->integratedRaySourceStamp(0),100010000000LL);
        EXPECT_EQ(next->integratedRaySourceStamp(1),100010000000LL);
        // A recycled third slot may bind a different reader while old output
        // is still blocked. The old scope must not overwrite this binding.
        next->setQueryAttitudeProvider([](const Eigen::Vector3d&,double){
          return std::optional<Eigen::Vector2d>(Eigen::Vector2d::Zero());});
      }
    }
    release.set_value();const auto proof=worker.get();
    EXPECT_TRUE(available);EXPECT_TRUE(restored);ASSERT_TRUE(proof);
    EXPECT_EQ(proof->trajectory_id,2);EXPECT_EQ(proof->valid,!obstacle);
    if(obstacle){EXPECT_EQ(proof->reason,"occupied");}
    EXPECT_EQ(proof->body_source_stamp,t::stamp(100.01));
    EXPECT_EQ(proof->front_ray_source_stamp,t::stamp(100.01));
    EXPECT_EQ(proof->rear_ray_source_stamp,t::stamp(100.01));
    EXPECT_EQ(proof->valid_until,t::stamp(0.));
    auto next=f.pool.borrowSolver();ASSERT_TRUE(next);
    if(available){EXPECT_TRUE(GridMapTestAccess::anyQueryProvider(*next));}
    EXPECT_EQ(f.native.committedTrajectory(),1); // Checking a candidate never commits it.
  }
}

TEST(SnapshotProofDelivery,AdmissionRetirementAfterQueryDoesNotPublishOldPositive) {
  Integrated f;auto candidate=tag(2);f.native.candidate(candidate,f.reference,f.heading);
  publishBoth(f,100000000000LL);
  ASSERT_TRUE(f.pool.publishSolver(f.map,100000000000LL,std::chrono::steady_clock::now()));
  bool callback=false;
  const auto proof=Access::checkPooledCandidate(f.native,100000000000LL,{},[&] {
    callback=true;auto barrier=version();++barrier.reference_generation;
    EXPECT_TRUE(f.native.cancelReference(barrier));
  });
  EXPECT_TRUE(callback);EXPECT_FALSE(proof);EXPECT_EQ(f.native.committedTrajectory(),-1);
  EXPECT_TRUE(f.pool.borrowSolver());
}

TEST(SnapshotProofDelivery,AdmissionThrowingPublisherReturnsRestoredSolverSlot) {
  Integrated f;auto candidate=tag(2);f.native.candidate(candidate,f.reference,f.heading);
  publishBoth(f,100000000000LL);
  ASSERT_TRUE(f.pool.publishSolver(f.map,100000000000LL,std::chrono::steady_clock::now()));
  EXPECT_THROW(Access::checkPooledCandidate(f.native,100000000000LL,[](const auto&){
    throw std::runtime_error("publisher failure");}),std::runtime_error);
  auto next=f.pool.borrowSolver();ASSERT_TRUE(next);
  EXPECT_FALSE(GridMapTestAccess::anyQueryProvider(*next));
  EXPECT_EQ(f.native.committedTrajectory(),1);
}

namespace {
auto latestCheck(const Integrated& f) {
  const auto trace=Access::checkTrace(f.native);
  EXPECT_GT(trace.last_event,0u);
  return trace.events[(trace.last_event-1)%scan_planner::BoundedExecutionTrace::capacity];
}
}
TEST(CheckCompletionTrace,CurrentAndAdmissionKeepExactProofSourcesAndSnapshotTiming) {
  using Trace=scan_planner::BoundedExecutionTrace;
  for(const bool current:{false,true})for(const bool obstacle:{false,true}) {
    Integrated f;f.acquire(100.01,0.,0.);
    if(obstacle)GridMapTestAccess::voxel(f.map,{.2,0.,.55},2.);
    if(!current){auto candidate=tag(2);candidate.trajectory.start_time=candidate.join_source_stamp=t::stamp(100.01);
      f.native.candidate(candidate,f.reference,f.heading);}
    publishBoth(f,100010000000LL);
    ASSERT_TRUE(f.pool.publishSolver(f.map,100010000000LL,std::chrono::steady_clock::now()));
    const auto proof=Access::checkObservedPooled(f.native,100010000000LL,current);
    ASSERT_TRUE(proof);const auto event=latestCheck(f);
    EXPECT_EQ(event.kind,current?Trace::Kind::CurrentCheckFinished:Trace::Kind::AdmissionCheckFinished);
    EXPECT_EQ(event.proof,proof->sequence);EXPECT_EQ(event.source_ns,t::timeNs(proof->front_ray_source_stamp));
    EXPECT_EQ(event.rear_source_ns,t::timeNs(proof->rear_ray_source_stamp));
    EXPECT_EQ(event.body_ns,t::timeNs(proof->body_source_stamp));
    EXPECT_EQ(event.valid_until_ns,t::timeNs(proof->valid_until));
    EXPECT_EQ(event.check.check_begin_ns,t::timeNs(proof->check_begin));
    EXPECT_EQ(event.check.check_end_ns,t::timeNs(proof->check_end));
    EXPECT_EQ(event.map_revision,proof->map_snapshot_revision);
    EXPECT_TRUE(event.accepted);EXPECT_EQ(event.check.valid,!obstacle);
    EXPECT_EQ(event.check.disposition.str(),"proof_publish_returned");
    EXPECT_GT(event.check.borrow_end_steady_ns,event.begin_steady_ns);
    EXPECT_LE(event.check.lease_end_steady_ns,event.check.ledger_steady_ns);
    EXPECT_LE(event.check.ledger_steady_ns,event.check.output_begin_steady_ns);
    EXPECT_LE(event.check.output_begin_steady_ns,event.check.output_end_steady_ns);
    if(current)EXPECT_GT(event.check.publication_sequence,0u);
    else EXPECT_EQ(event.check.publication_sequence,0u); // Pool API exposes no admission envelope.
    const auto trace=Access::checkTrace(f.native);
    const auto& identity=trace.identities[(event.identity[0]-1)%Trace::identity_capacity];
    EXPECT_EQ(identity.text[1].str(),proof->version.task_id);
    EXPECT_EQ(identity.text[2].str(),proof->version.route_id);
    EXPECT_EQ(identity.number[1],proof->version.reference_generation);
    EXPECT_EQ(identity.curve,proof->trajectory_id);
  }
}
TEST(CheckCompletionTrace,MissingSnapshotAndIncompleteQueryAreDistinctWithoutNewProof) {
  Integrated missing;
  const auto old=Access::executionTrace(missing.native).last_event;
  EXPECT_FALSE(Access::checkObservedPooled(missing.native,100000000000LL));
  auto event=latestCheck(missing);EXPECT_EQ(event.check.disposition.str(),"snapshot_unavailable");
  EXPECT_EQ(event.proof,0u);EXPECT_EQ(event.check.check_begin_ns,0);
  EXPECT_FALSE(event.accepted);EXPECT_EQ(Access::executionTrace(missing.native).last_event,old);
  publishBoth(missing,100000000000LL);
  EXPECT_FALSE(Access::checkObservedPooled(missing.native,100000000000LL,true,
    scan_planner::ValidationCycle::Clock::now()-std::chrono::seconds(1)));
  event=latestCheck(missing);EXPECT_EQ(event.check.disposition.str(),"projection_incomplete_no_proof");
  EXPECT_GT(event.proof,0u);EXPECT_EQ(event.check.check_begin_ns,100000000000LL);
  EXPECT_FALSE(event.accepted);EXPECT_EQ(event.valid_until_ns,0);
  // A causal measured-progress pair avoids that projection branch, reaching
  // the actual curve-query budget terminal on the same native inputs.
  missing.native.progressAt(missing.progress(100.),100.);
  EXPECT_FALSE(Access::checkObservedPooled(missing.native,100000000000LL,true,
    scan_planner::ValidationCycle::Clock::now()-std::chrono::seconds(1)));
  event=latestCheck(missing);EXPECT_EQ(event.check.disposition.str(),"incomplete_no_proof");
  EXPECT_FALSE(event.accepted);EXPECT_EQ(event.valid_until_ns,0);
}
TEST(CheckCompletionTrace,RetiredAndSupersededResultsNeverClaimPublication) {
  using Clock=scan_planner::ValidationCycle::Clock;
  Integrated retired;publishBoth(retired,100000000000LL);
  EXPECT_FALSE(Access::checkObservedPooled(retired.native,100000000000LL,true,Clock::time_point::max(),[&] {
    auto barrier=version();++barrier.reference_generation;EXPECT_TRUE(retired.native.cancelReference(barrier));
  }));
  auto event=latestCheck(retired);EXPECT_EQ(event.check.disposition.str(),"job_retired_or_replaced");
  EXPECT_FALSE(event.accepted);EXPECT_EQ(event.check.output_begin_steady_ns,0);
  Integrated superseded;publishBoth(superseded,100000000000LL);
  EXPECT_FALSE(Access::checkObservedPooled(superseded.native,100000000000LL,true,Clock::time_point::max(),[&] {
    EXPECT_TRUE(Access::checkObservedPooled(superseded.native,100000000000LL));
  }));
  event=latestCheck(superseded);EXPECT_EQ(event.check.disposition.str(),"proof_superseded");
  EXPECT_FALSE(event.accepted);EXPECT_EQ(event.check.output_begin_steady_ns,0);
}
TEST(CheckCompletionTrace,OutputExceptionStillReportsCommittedVerdictAndOriginalStateContract) {
  Integrated f;publishBoth(f,100000000000LL);
  EXPECT_THROW(Access::checkObservedPooled(f.native,100000000000LL,true,
    scan_planner::ValidationCycle::Clock::time_point::max(),{},[](const auto&){throw std::runtime_error("sink failure");}),std::runtime_error);
  const auto event=latestCheck(f);EXPECT_EQ(event.check.disposition.str(),"output_exception");
  EXPECT_TRUE(event.accepted);EXPECT_TRUE(event.check.valid);EXPECT_GT(event.check.ledger_steady_ns,0);
  EXPECT_GT(event.check.output_begin_steady_ns,0);EXPECT_EQ(event.check.output_end_steady_ns,0);
  EXPECT_EQ(event.valid_until_ns,0);EXPECT_TRUE(f.pool.borrowValidation());
}
TEST(CheckCompletionTrace,MissingCausalBodyProgressPairIsWaitingNotAnExceptionOrProof) {
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  f.acquire(100.04,.004,.1);
  ASSERT_TRUE(f.follower.body(body(100.08,.008,.1),100.08,10.08));
  f.native.progressAt(f.progress(100.08),100.08);
  GridMapTestAccess::acquire(f.map,100080000000LL);publishBoth(f,100081000000LL);
  const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
  EXPECT_FALSE(Access::checkObservedPooled(f.native,100081000000LL));
  const auto event=latestCheck(f);
  EXPECT_EQ(event.check.disposition.str(),"waiting_measured_source_pair");
  EXPECT_EQ(event.body_ns,100040000000LL);EXPECT_EQ(event.check.progress_source_ns,100080000000LL);
  EXPECT_FALSE(event.accepted);EXPECT_EQ(event.valid_until_ns,0);
  const auto after=f.native.committedView();ASSERT_TRUE(after&&after->proof);
  EXPECT_EQ(after->proof->sequence,before->proof->sequence);
  EXPECT_EQ(after->proof->valid_until,before->proof->valid_until);
}
TEST(CheckCompletionTrace,FinalDrainPreservesPartialIntervalInIndependentFiles) {
  const auto directory=std::filesystem::temp_directory_path()/
    ("native-check-tail-"+std::to_string(::getpid())+"-"+std::to_string(scan_planner::BoundedExecutionTrace::steadyNowNs()));
  Integrated f;publishBoth(f,100000000000LL);ASSERT_TRUE(Access::checkObservedPooled(f.native,100000000000LL));
  Access::flushTraces(f.native,directory);
  unsigned checks=0,proofs=0;
  for(const auto& entry:std::filesystem::recursive_directory_iterator(directory)) {
    if(entry.path().extension()!=".jsonl")continue;
    std::ifstream file(entry.path());std::string line;
    while(std::getline(file,line)) {
      const auto batch=nlohmann::json::parse(line);const auto& payload=batch["payload"];
      for(const auto& event:payload["events"]) {
        const auto kind=event[1].get<unsigned>();
        checks+=kind==static_cast<unsigned>(scan_planner::BoundedExecutionTrace::Kind::CurrentCheckFinished);
        proofs+=kind==static_cast<unsigned>(scan_planner::BoundedExecutionTrace::Kind::CurveProof);
      }
    }
  }
  EXPECT_EQ(checks,1u);EXPECT_GT(proofs,0u);
  std::filesystem::remove_all(directory);
}
