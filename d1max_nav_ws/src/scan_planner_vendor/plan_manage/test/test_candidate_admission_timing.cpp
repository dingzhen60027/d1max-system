#include "native_local_follower_test_fixture.hpp"
#include <d1max_trajectory_tracker/local_commit_dispatch.hpp>
#include <future>

namespace {
struct SelectedPipeline {
  Integrated run;
  std::vector<std::int64_t> published;
  SelectedPipeline() {
    Access::selectedObserver(run.native,[&](const auto& selected) {
      published.push_back(selected.trajectory.traj_id);
      EXPECT_TRUE(run.follower.candidate(selected));
    });
  }
  void offer(unsigned id,double now) {
    auto curve=tag(id);curve.trajectory.start_time=curve.join_source_stamp=t::stamp(now);
    curve.generation=run.reference.version.reference_generation;
    curve.segment_id=run.reference.version.segment_id;
    run.native.candidate(curve,run.reference,run.heading);
    Access::dispatchSelected(run.native); // Independent candidate-delivery tick.
  }
  void slide(unsigned generation,double now) {
    auto next=run.reference;next.version.reference_generation=generation;
    next.proposal_id="window-"+std::to_string(generation);
    const auto incumbent=run.native.committedIdentity();ASSERT_TRUE(incumbent);
    next.expected_version=incumbent->version;next.expected_trajectory_id=incumbent->trajectory_id;
    next.source_stamp=t::stamp(now);next.valid_until=t::stamp(now+.9);
    ASSERT_TRUE(run.native.replacePendingReference(next,true));
    ASSERT_TRUE(run.follower.proposal(next,now));
    auto terrain=support();terrain.version=next.version;
    run.native.support(terrain);ASSERT_TRUE(run.follower.support(terrain));run.reference=next;
  }
  void complete(const w::TrajectoryValidation& proof,double now) {
    run.acquire(now,0.,0.);ASSERT_TRUE(run.follower.validation(proof));
    const auto prepare=run.follower.exchangeCommit(now,now-90.);ASSERT_TRUE(prepare);
    const auto reserved=run.native.localCommitExchangeAt(*prepare,t::timeNs(t::stamp(now+.001)));
    ASSERT_TRUE(reserved.accepted)<<reserved.reason;
    Access::dispatchSelected(run.native);
    // Native may publish the selected successor before the prepared ACK.
    EXPECT_EQ(run.follower.controller().trajectoryId(),run.native.committedTrajectory());
    ASSERT_TRUE(run.follower.commitResult(reserved,now+.002,now-90.+.002));
    const auto commit=run.follower.exchangeCommit(now+.003,now-90.+.003);ASSERT_TRUE(commit);
    ASSERT_EQ(commit->trajectory_id,proof.trajectory_id);
    const auto result=run.native.localCommitExchangeAt(*commit,t::timeNs(t::stamp(now+.004)));
    ASSERT_TRUE(result.accepted)<<result.reason;
    ASSERT_TRUE(run.follower.commitResult(result,now+.005,now-90.+.005));
  }
};
}

TEST(CandidateAdmissionTiming,RollingWindowDuringCheckKeepsSelectedThenCommitsNewestWindow) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
  unsigned clocks=0;
  const auto original=Access::checkCurveClock(p.run.native,p.run.map,[&] {
    if(++clocks==2) {p.slide(2,100.06);p.offer(3,100.06);}
    return clocks>=2?100070000000LL:100050000000LL;
  });
  ASSERT_TRUE(original&&original->valid);
  EXPECT_EQ(original->version.reference_generation,1u);
  EXPECT_EQ(original->body_source_stamp,t::stamp(100.05));
  EXPECT_EQ(original->front_ray_source_stamp,t::stamp(100.05));
  EXPECT_EQ(p.published,(std::vector<std::int64_t>{2}));
  ASSERT_TRUE(Access::queuedCurve(p.run.native));
  EXPECT_EQ(Access::queuedCurve(p.run.native)->generation,2u);
  p.complete(*original,100.071);
  EXPECT_EQ(p.published,(std::vector<std::int64_t>{2,3}));
  EXPECT_EQ(p.run.native.committedTrajectory(),2);
  p.run.acquire(100.09,0.,0.);
  const auto next=Access::checkCurve(p.run.native,p.run.map,100100000000LL);
  ASSERT_TRUE(next&&next->valid);EXPECT_EQ(next->version.reference_generation,2u);
  p.complete(*next,100.101);
  EXPECT_EQ(p.run.native.committedTrajectory(),3);
  EXPECT_EQ(p.run.follower.controller().trajectoryId(),3);
  EXPECT_EQ(p.run.native.committedVersion()->reference_generation,2u);
}

TEST(CandidateAdmissionTiming,RepeatedWindowsCoalesceOneNewestWithoutExtendingSelectedOffer) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
  const auto first=Access::checkCurve(p.run.native,p.run.map,100060000000LL);
  ASSERT_TRUE(first&&first->valid);const auto deadline=Access::candidateOfferDeadline(p.run.native);
  ASSERT_EQ(deadline,0); // Completed formal state has no selected-proof lease.
  for(unsigned generation=2;generation<=6;++generation) {
    const double now=100.06+.01*generation;p.slide(generation,now);p.offer(generation+1,now);
    ASSERT_TRUE(Access::pendingCurve(p.run.native));
    EXPECT_EQ(Access::pendingCurve(p.run.native)->trajectory.traj_id,2);
    ASSERT_TRUE(Access::queuedCurve(p.run.native));
    EXPECT_EQ(Access::queuedCurve(p.run.native)->generation,generation);
    EXPECT_EQ(Access::candidateOfferDeadline(p.run.native),deadline);
  }
  EXPECT_EQ(p.published,(std::vector<std::int64_t>{2}));
  Access::expireCandidateOffer(p.run.native,100300000000LL);
  EXPECT_EQ(p.published.size(),1u);
  p.complete(*first,100.13);
  EXPECT_EQ(p.published,(std::vector<std::int64_t>{2,7}));
  EXPECT_FALSE(Access::queuedCurve(p.run.native));
  EXPECT_EQ(Access::pendingCurve(p.run.native)->generation,6u);
  // A delayed solve from an older accepted window cannot overtake that newest
  // search owner merely because its trajectory number is numerically larger.
  p.run.native.candidate(tag(999),proposal(),p.run.heading);
  EXPECT_EQ(p.published,(std::vector<std::int64_t>{2,7}));
  EXPECT_EQ(Access::pendingCurve(p.run.native)->generation,6u);
}

TEST(CandidateAdmissionTiming,RollingRetentionDoesNotShieldNegativeOrReviveCanceledWork) {
  for(const bool cancel:{false,true}) {
    SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
    p.slide(2,100.06);p.offer(3,100.06);
    if(cancel) {
      auto barrier=p.run.reference.version;++barrier.reference_generation;
      ASSERT_TRUE(p.run.native.cancelReference(barrier));p.run.follower.retire("new_target");
      EXPECT_FALSE(Access::checkCurve(p.run.native,p.run.map,100070000000LL));
      EXPECT_FALSE(Access::pendingCurve(p.run.native));EXPECT_FALSE(Access::queuedCurve(p.run.native));
    } else {
      GridMapTestAccess::voxel(p.run.map,{0.,0.,.55},2.);
      const auto negative=Access::checkCurve(p.run.native,p.run.map,100070000000LL);
      Access::dispatchSelected(p.run.native);
      ASSERT_TRUE(negative);EXPECT_FALSE(negative->valid);EXPECT_EQ(negative->reason,"occupied");
      EXPECT_EQ(p.published,(std::vector<std::int64_t>{2,3}));
      ASSERT_TRUE(p.run.follower.validation(*negative));
      EXPECT_FALSE(p.run.follower.exchangeCommit(100.07,10.07));
      const auto current=Access::checkCurve(p.run.native,p.run.map,100080000000LL,true);
      ASSERT_TRUE(current);EXPECT_FALSE(current->valid);EXPECT_EQ(current->reason,"occupied");
    }
  }
}

TEST(CandidateAdmissionTiming,SemanticPortalAndHardReplacementRetainOriginalCandidateFence) {
  for(const bool portal:{false,true}) {
    SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
    auto next=p.run.reference;next.version.reference_generation=2;
    next.expected_version=p.run.reference.version;next.expected_trajectory_id=1;
    if(portal)next.version.segment_id="stair-entry";
    else {++next.version.anchor_revision;next.version.anchor_id="new-anchor";}
    ASSERT_TRUE(p.run.native.replacePendingReference(next,portal,portal,
        portal?std::optional<w::ExecutionVersion>{p.run.reference.version}:std::nullopt));
    EXPECT_FALSE(Access::pendingCurve(p.run.native));EXPECT_FALSE(Access::queuedCurve(p.run.native));
    EXPECT_EQ(p.run.native.committedTrajectory(),1);
    EXPECT_EQ(p.run.native.committedVersion(),std::optional<w::ExecutionVersion>{p.run.reference.version});
  }
}

TEST(CandidateAdmissionTiming,HardReferenceRetiresOldReservationWithoutErasingIncumbentOrNewSlot) {
 for(unsigned order=0;order<3;++order)for(bool anchor_change:{false,true}) {
  SCOPED_TRACE(::testing::Message()<<"order="<<order<<" anchor_change="<<anchor_change);
  Integrated run;run.acquire(100.05,0.,0.);
  auto next_curve=tag(2);next_curve.trajectory.start_time=next_curve.join_source_stamp=t::stamp(100.05);
  run.native.candidate(next_curve,run.reference,run.heading);
  ASSERT_TRUE(run.follower.candidate(next_curve));
  const auto proof=Access::checkCurve(run.native,run.map,100050000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(run.follower.validation(*proof));
  const auto prepare=run.follower.exchangeCommit(100.05,10.05);ASSERT_TRUE(prepare);
  const auto reserved=run.native.localCommitExchangeAt(*prepare,100051000000LL);
  ASSERT_TRUE(reserved.accepted)<<reserved.reason;
  ASSERT_TRUE(run.native.localCommitInFlight());
  std::optional<w::LocalTrajectoryCommit> old_commit;
  if(order>0)ASSERT_TRUE(run.follower.commitResult(reserved,100.052,10.052));
  if(order==2)old_commit=run.follower.exchangeCommit(100.053,10.053);

  auto hard=run.reference;hard.version.reference_generation=2;
  hard.proposal_id="hard-replacement";
  if(anchor_change){hard.version.anchor_id="new-anchor";++hard.version.anchor_revision;}
  else hard.version.segment_id="non-continuous-segment";
  hard.expected_version=run.reference.version;hard.expected_trajectory_id=1;
  hard.source_stamp=t::stamp(100.052);hard.valid_until=t::stamp(100.9);
  ASSERT_TRUE(run.native.replacePendingReference(hard));
  EXPECT_FALSE(run.native.localCommitInFlight())<<"hard replacement left the old reservation live";
  EXPECT_EQ(run.native.committedTrajectory(),1);
  // New search input and its candidate may arrive before the retired exchange's
  // response. That stale response must not erase the newer candidate slot.
  ASSERT_TRUE(run.follower.proposal(hard,100.054));
  auto terrain=support();terrain.version=hard.version;
  run.native.support(terrain);ASSERT_TRUE(run.follower.support(terrain));
  auto fresh=tag(3);fresh.generation=hard.version.reference_generation;
  fresh.anchor_id=hard.version.anchor_id;fresh.anchor_revision=hard.version.anchor_revision;
  fresh.segment_id=hard.version.segment_id;
  fresh.trajectory.start_time=fresh.join_source_stamp=t::stamp(100.055);
  run.native.candidate(fresh,hard,run.heading);ASSERT_TRUE(run.follower.candidate(fresh));
  if(order==0)ASSERT_TRUE(run.follower.commitResult(reserved,100.056,10.056));
  if(!old_commit)old_commit=run.follower.exchangeCommit(100.057,10.057);
  ASSERT_TRUE(old_commit);ASSERT_EQ(old_commit->trajectory_id,2);
  const auto rejected=run.native.localCommitExchangeAt(*old_commit,100058000000LL);
  EXPECT_FALSE(rejected.accepted)<<"retired old-reference reservation crossed the new-reference fence";
  EXPECT_EQ(rejected.reason,"local_commit_reservation_missing");
  ASSERT_TRUE(run.follower.commitResult(rejected,100.059,10.059));
  EXPECT_EQ(run.native.committedTrajectory(),1);
  EXPECT_EQ(run.follower.controller().trajectoryId(),1);
  EXPECT_FALSE(run.native.localCommitInFlight());
  ASSERT_TRUE(Access::pendingCurve(run.native));
  EXPECT_EQ(Access::pendingCurve(run.native)->trajectory.traj_id,3);
  run.acquire(100.06,0.,0.);
  const auto new_proof=Access::checkCurve(run.native,run.map,100060000000LL);
  ASSERT_TRUE(new_proof&&new_proof->valid);ASSERT_TRUE(run.follower.validation(*new_proof));
  const auto new_prepare=run.follower.exchangeCommit(100.061,10.061);ASSERT_TRUE(new_prepare);
  EXPECT_EQ(new_prepare->trajectory_id,3);
  const auto new_reserved=run.native.localCommitExchangeAt(*new_prepare,100062000000LL);
  ASSERT_TRUE(new_reserved.accepted)<<new_reserved.reason;
  ASSERT_TRUE(run.follower.commitResult(new_reserved,100.063,10.063));
  const auto new_commit=run.follower.exchangeCommit(100.064,10.064);ASSERT_TRUE(new_commit);
  const auto result=run.native.localCommitExchangeAt(*new_commit,100065000000LL);
  ASSERT_TRUE(result.accepted)<<result.reason;
  ASSERT_TRUE(run.follower.commitResult(result,100.066,10.066));
  EXPECT_EQ(run.native.committedTrajectory(),3);
  EXPECT_EQ(run.follower.controller().trajectoryId(),3);
  EXPECT_EQ(run.native.committedVersion(),std::optional<w::ExecutionVersion>{hard.version});
 }
}

TEST(CandidateAdmissionTiming,TypedIngressFaultInjectionHardReferenceClearsOnlyRetiredReservationDemand) {
  for(unsigned kind=0;kind<3;++kind) {
    SCOPED_TRACE(::testing::Message()<<"demand_kind="<<kind);
    // 0/1 deliberately inject premature typed demands for the prepared/other
    // known candidate. A real follower cannot emit either before final ACK.
    // 2 is the real incumbent awaiting independently acquired raw body input.
    Integrated run(true,false,{},tag(1,{0.,0.,.55},{.2,0.,0.},{0.,0.,0.}));
    const auto initial=run.command(100.10,.02,.2,1);ASSERT_TRUE(initial);
    ASSERT_GT(initial->velocity.linear.x,0.);
    auto curve=tag(2,{.02,0.,.55},{.2,0.,0.},{0.,0.,0.});
    curve.trajectory.start_time=curve.join_source_stamp=t::stamp(100.10);
    run.native.candidate(curve,run.reference,run.heading);ASSERT_TRUE(run.follower.candidate(curve));
    const auto proof=Access::checkCurve(run.native,run.map,100120000000LL);
    ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(run.follower.validation(*proof));
    const auto prepare=run.follower.exchangeCommit(100.12,10.12);ASSERT_TRUE(prepare);
    const auto reserved=run.native.localCommitExchangeAt(*prepare,100121000000LL);
    ASSERT_TRUE(reserved.accepted);ASSERT_TRUE(run.follower.commitResult(reserved,100.122,10.122));
    const auto measured=body(100.13,.026,.2);
    ASSERT_TRUE(run.follower.body(measured,100.13,10.13));
    if(kind<2) {
      const auto commit=run.follower.exchangeCommit(100.13,10.13);
      ASSERT_TRUE(commit);ASSERT_EQ(commit->trajectory_id,2);
    }
    ASSERT_TRUE(run.native.commit(run.follower.geometryPermit(100.13)));
    auto demand=run.follower.step(100.13,10.13);
    ASSERT_FALSE(demand.hold);ASSERT_GT(demand.velocity.linear.x,0.);
    ASSERT_EQ(demand.trajectory_id,1); // Prepared is not Committed.
    ASSERT_EQ(run.follower.controller().trajectoryId(),1);
    ASSERT_EQ(run.native.committedTrajectory(),1);
    ASSERT_TRUE(run.native.localCommitInFlight());
    if(kind==0) {
      demand.trajectory_id=2;demand.validation_sequence=proof->sequence;
    }
    if(kind==1) {
      auto other=curve;other.trajectory.traj_id=3;
      run.native.candidate(other,run.reference,run.heading);
      const auto other_proof=Access::checkCurve(run.native,run.map,100130000000LL);
      ASSERT_TRUE(other_proof&&other_proof->valid);
      demand.trajectory_id=3;demand.validation_sequence=other_proof->sequence;
    }
    ASSERT_EQ(demand.trajectory_id,kind==0?2:kind==1?3:1);
    std::vector<w::LocalMotionCommand> emitted;
    run.native.demandAt(demand,100135000000LL,[&](const auto& c){emitted.push_back(c);});
    ASSERT_TRUE(emitted.empty());ASSERT_TRUE(Access::pendingDemand(run.native));
    auto hard=run.reference;hard.proposal_id="reanchor";hard.version.reference_generation=2;
    hard.version.anchor_id="new-anchor";++hard.version.anchor_revision;
    hard.expected_version=run.reference.version;hard.expected_trajectory_id=1;
    ASSERT_TRUE(run.native.replacePendingReference(hard));
    EXPECT_FALSE(run.native.localCommitInFlight());
    EXPECT_EQ(Access::pendingDemand(run.native),kind!=0);
    ASSERT_TRUE(Access::latestDemand(run.native));
    EXPECT_EQ(*Access::latestDemand(run.native),demand); // Do not reset sequence watermark.
    EXPECT_EQ(run.native.committedTrajectory(),1);
    if(kind==2) {
      geometry_msgs::msg::Twist twist;twist.linear.x=.2;
      run.native.bodyAt({measured.position,measured.orientation,measured.stamp,"odom"},
          twist,measured.source_stamp_ns,100140000000LL,[&](const auto& c){emitted.push_back(c);});
      ASSERT_EQ(emitted.size(),1u);EXPECT_EQ(emitted.front().trajectory_id,1);
      EXPECT_EQ(emitted.front().source_stamp,demand.source_stamp);
      EXPECT_EQ(emitted.front().velocity,demand.velocity);
    }
  }
}

TEST(CandidateAdmissionTiming,RealCommitExchangeKeepsIncumbentUntilFinalCommittedAck) {
  Integrated run(true,false,{},tag(1,{0.,0.,.55},{.2,0.,0.},{0.,0.,0.}));
  const auto initial=run.command(100.10,.02,.2,1);ASSERT_TRUE(initial);
  ASSERT_GT(initial->velocity.linear.x,0.);
  auto curve=tag(2,{.02,0.,.55},{.2,0.,0.},{0.,0.,0.});
  curve.trajectory.start_time=curve.join_source_stamp=t::stamp(100.10);
  run.native.candidate(curve,run.reference,run.heading);ASSERT_TRUE(run.follower.candidate(curve));
  const auto proof=Access::checkCurve(run.native,run.map,100120000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(run.follower.validation(*proof));
  const auto expect_incumbent=[&](double now) {
    const auto demand=run.follower.step(now,now-90.);
    EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);
    EXPECT_EQ(demand.trajectory_id,1);
    EXPECT_EQ(run.follower.controller().trajectoryId(),1);
  };
  expect_incumbent(100.12); // Candidate/state present, before Prepare.
  const auto prepare=run.follower.exchangeCommit(100.12,10.12);ASSERT_TRUE(prepare);
  ASSERT_EQ(prepare->disposition,"prepare");ASSERT_EQ(prepare->trajectory_id,2);
  const auto reserved=run.native.localCommitExchangeAt(*prepare,100121000000LL);
  ASSERT_TRUE(reserved.accepted)<<reserved.reason;
  ASSERT_EQ(reserved.disposition,"prepared");
  ASSERT_TRUE(run.native.localCommitInFlight());
  EXPECT_EQ(run.native.committedTrajectory(),1);
  expect_incumbent(100.121); // Native reservation exists; Prepared ACK not received.
  ASSERT_TRUE(run.follower.commitResult(reserved,100.122,10.122));
  expect_incumbent(100.123); // Prepared ACK is still not final authority.

  run.acquire(100.13,.026,.2); // Independent original body reaches both readers.
  const auto commit=run.follower.exchangeCommit(100.13,10.13);ASSERT_TRUE(commit);
  ASSERT_EQ(commit->disposition,"committed");ASSERT_EQ(commit->trajectory_id,2);
  EXPECT_EQ(commit->commit_sequence,prepare->commit_sequence);
  EXPECT_EQ(commit->body_source_stamp,t::stamp(100.13));
  expect_incumbent(100.131); // Sending Commit cannot switch local geometry.
  const auto committed=run.native.localCommitExchangeAt(*commit,100132000000LL);
  ASSERT_TRUE(committed.accepted)<<committed.reason;
  ASSERT_EQ(committed.disposition,"committed");
  EXPECT_FALSE(run.native.localCommitInFlight());
  EXPECT_EQ(run.native.committedTrajectory(),2);
  expect_incumbent(100.132); // Even Native acceptance must reach the follower.
  ASSERT_TRUE(run.follower.commitResult(committed,100.133,10.133));
  EXPECT_EQ(run.follower.controller().trajectoryId(),2);
  ASSERT_TRUE(run.native.commit(run.follower.geometryPermit(100.134)));
  const auto demand=run.follower.step(100.134,10.134);
  ASSERT_FALSE(demand.hold);ASSERT_GT(demand.velocity.linear.x,0.);
  ASSERT_EQ(demand.trajectory_id,2);
  std::vector<w::LocalMotionCommand> emitted;
  run.native.demandAt(demand,100134000000LL,[&](const auto& c){emitted.push_back(c);});
  ASSERT_EQ(emitted.size(),1u);
  EXPECT_EQ(emitted.front().trajectory_id,2);
  EXPECT_EQ(emitted.front().source_stamp,demand.source_stamp);
  EXPECT_EQ(emitted.front().body_source_stamp,t::stamp(100.13));
  EXPECT_EQ(emitted.front().velocity,demand.velocity);
  EXPECT_FALSE(run.follower.commitResult(reserved,100.135,10.135));
  EXPECT_EQ(run.follower.controller().trajectoryId(),2); // Stale Prepared cannot roll back.
}

TEST(CandidateAdmissionTiming,WriterHintNeverWaitsForTheBusyLedgerAndDoesNotLoseWork) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
  ASSERT_TRUE(p.run.native.slowSnapshotWanted());
  auto held=Access::holdLedger(p.run.native);
  std::promise<void> entered;
  auto hint=std::async(std::launch::async,[&]{entered.set_value();return p.run.native.trySlowSnapshotWanted();});
  entered.get_future().wait();
  const auto ready=hint.wait_for(std::chrono::seconds(1));
  // Release even on regression so a failed test never leaves a blocked worker.
  held.unlock();
  EXPECT_EQ(ready,std::future_status::ready);
  EXPECT_FALSE(hint.get().has_value());
  EXPECT_EQ(p.run.native.slowSnapshotHintMisses(),1u);
  ASSERT_TRUE(p.run.native.trySlowSnapshotWanted().has_value());
  EXPECT_TRUE(*p.run.native.trySlowSnapshotWanted());
  auto barrier=version();++barrier.reference_generation;
  ASSERT_TRUE(p.run.native.cancelReference(barrier));
  ASSERT_TRUE(p.run.native.trySlowSnapshotWanted().has_value());
  EXPECT_FALSE(*p.run.native.trySlowSnapshotWanted());
}

TEST(CandidateAdmissionTiming,CompletedCurrentReadReleasesHeadBeforeDiagnosticsCanBlock) {
  Integrated run;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(run.pool.publishValidation(run.map,100000000000LL,now));
  ASSERT_TRUE(run.pool.refreshOwnerReplica(run.map,100000000000LL,now));
  auto owner=run.pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  std::promise<void> query_entered,query_continue,diagnostic_entered,diagnostic_continue;
  auto query_release=query_continue.get_future();auto diagnostic_release=diagnostic_continue.get_future();
  auto reader=std::async(std::launch::async,[&] {
    const auto read=Access::currentSnapshotRead(run.native,[&](GridMap& view) {
      EXPECT_EQ(view.integratedRaySourceStamp(0),100000000000LL);
      query_entered.set_value();query_release.wait();
      const auto proof=Access::checkCurve(run.native,view,100000000000LL,true);
      EXPECT_TRUE(proof&&proof->valid);
    });
    diagnostic_entered.set_value();diagnostic_release.wait();
    return read;
  });
  query_entered.get_future().wait();
  // Both slots are genuinely queried. Publication must not overwrite either.
  EXPECT_FALSE(run.pool.publishValidation(run.map,100000000000LL,now));
  query_continue.set_value();diagnostic_entered.get_future().wait();
  // Simulate blocked logging after the exact production lease boundary.
  // The owner's slot stays pinned, so success requires releasing the head.
  for(const std::int64_t source:{100100000000LL,100200000000LL}) {
    GridMapTestAccess::acquire(run.map,source);
    EXPECT_TRUE(run.pool.publishValidation(run.map,source,std::chrono::steady_clock::now()));
    auto latest=run.pool.borrowValidation();EXPECT_TRUE(latest);
    if(latest)EXPECT_EQ(latest->integratedRaySourceStamp(0),source);
  }
  diagnostic_continue.set_value();const auto read=reader.get();
  EXPECT_TRUE(read.available);EXPECT_EQ(read.front_source_ns,100000000000LL);
  EXPECT_EQ(read.rear_source_ns,100000000000LL);
  EXPECT_GE(read.lease_end_steady_ns,read.lease_begin_steady_ns);
  EXPECT_EQ(owner->integratedRaySourceStamp(0),100000000000LL);
}

TEST(CandidateAdmissionTiming,CurrentReadExceptionsStillReleaseExclusiveSnapshot) {
  Integrated run;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(run.pool.publishValidation(run.map,100000000000LL,now));
  ASSERT_TRUE(run.pool.refreshOwnerReplica(run.map,100000000000LL,now));
  auto owner=run.pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  EXPECT_THROW(Access::currentSnapshotRead(run.native,[](GridMap&){throw std::runtime_error("query_failure");}),
    std::runtime_error);
  GridMapTestAccess::acquire(run.map,100100000000LL);
  EXPECT_TRUE(run.pool.publishValidation(run.map,100100000000LL,std::chrono::steady_clock::now()));
}

TEST(CandidateAdmissionTiming,SlowDiagnosticSinkDoesNotRetainTheBodyAndCommandLedger) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
  std::promise<void> copied,finish;auto released=finish.get_future();
  auto diagnostic=std::async(std::launch::async,[&] {
    const auto snapshot=Access::localDiagnosticSnapshot(p.run.native);
    copied.set_value();released.wait();return snapshot;
  });
  copied.get_future().wait();
  // A slow sink holds only values, never the live ledger. Cancel and acquired
  // body delivery are allowed to progress while this old diagnostic persists.
  const auto wanted=p.run.native.trySlowSnapshotWanted();EXPECT_TRUE(wanted.has_value());
  auto barrier=version();++barrier.reference_generation;
  EXPECT_TRUE(p.run.native.cancelReference(barrier));
  p.run.acquire(100.06,0.,0.);
  finish.set_value();const auto snapshot=diagnostic.get();
  EXPECT_EQ(snapshot.task,version().task_id);
  EXPECT_FALSE(*p.run.native.trySlowSnapshotWanted());
}

TEST(CandidateAdmissionTiming,RepeatedPublicationDuringValidationStillCommitsSixSuccessors) {
  SelectedPipeline p;p.offer(2,100.05);
  for(unsigned i=0;i<6;++i) {
    const double begin=100.05+.15*i;p.run.acquire(begin,0.,0.);
    unsigned calls=0;
    const auto proof=Access::checkCurveClock(p.run.native,p.run.map,[&] {
      if(++calls==2)p.offer(3+i,begin+.1);
      return t::timeNs(t::stamp(begin+(calls>=2?.11:0.)));
    });
    ASSERT_TRUE(proof&&proof->valid)<<"round "<<i;
    ASSERT_EQ(proof->trajectory_id,2+i);
    p.complete(*proof,begin+.112);
    EXPECT_EQ(p.run.native.committedTrajectory(),2+i);
    EXPECT_EQ(p.run.follower.controller().trajectoryId(),2+i);
  }
  ASSERT_EQ(p.published.size(),7u);
  for(unsigned i=0;i<p.published.size();++i)EXPECT_EQ(p.published[i],2+i);
}

TEST(CandidateAdmissionTiming,CoalescesOnlyNewestAndNeverPublishesRawUnselectedCandidates) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
  for(unsigned id=3;id<=30;++id)p.offer(id,100.05+.001*id);
  ASSERT_EQ(p.published.size(),1u);EXPECT_EQ(p.published.front(),2);
  ASSERT_TRUE(Access::queuedCurve(p.run.native));EXPECT_EQ(Access::queuedCurve(p.run.native)->trajectory.traj_id,30);
  const auto proof=Access::checkCurve(p.run.native,p.run.map,100090000000LL);ASSERT_TRUE(proof&&proof->valid);
  p.complete(*proof,100.091);
  ASSERT_EQ(p.published.size(),2u);EXPECT_EQ(p.published.back(),30);
  EXPECT_FALSE(Access::queuedCurve(p.run.native));
}

TEST(CandidateAdmissionTiming,CompletedSelectedStateDoesNotExpireAndCommitReleasesItsSlot) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);p.offer(3,100.06);
  auto first=Access::checkCurve(p.run.native,p.run.map,100060000000LL);ASSERT_TRUE(first&&first->valid);
  const auto deadline=Access::candidateOfferDeadline(p.run.native);ASSERT_EQ(deadline,0);
  p.run.acquire(100.2,0.,0.);
  auto renewed=Access::checkCurve(p.run.native,p.run.map,100210000000LL);ASSERT_TRUE(renewed&&renewed->valid);
  ASSERT_EQ(t::timeNs(renewed->valid_until),0);
  EXPECT_EQ(Access::candidateOfferDeadline(p.run.native),deadline);
  Access::expireCandidateOffer(p.run.native,100400000000LL);EXPECT_EQ(p.published.size(),1u);
  p.complete(*renewed,100.22);ASSERT_EQ(p.published.size(),2u);
  EXPECT_EQ(p.published.back(),3);
  // A late original proof for released curve2 cannot reserve it again.
  EXPECT_FALSE(p.run.follower.validation(*first)); // Older sequence cannot replace the committed newer state.
  EXPECT_FALSE(p.run.follower.exchangeCommit(100.23,10.23));
}

TEST(CandidateAdmissionTiming,OccupiedCurrentCurveIsNotShieldedByCandidateSelection) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);p.offer(3,100.06);
  GridMapTestAccess::voxel(p.run.map,{0.,0.,.55},2.);
  const auto active=Access::checkCurve(p.run.native,p.run.map,100070000000LL,true);
  ASSERT_TRUE(active);EXPECT_FALSE(active->valid);EXPECT_EQ(active->reason,"occupied");
  ASSERT_TRUE(p.run.follower.validation(*active));
  const auto selected=Access::checkCurve(p.run.native,p.run.map,100080000000LL);
  Access::dispatchSelected(p.run.native);
  ASSERT_TRUE(selected);EXPECT_FALSE(selected->valid);
  ASSERT_EQ(p.published.size(),2u);EXPECT_EQ(p.published.back(),3);
  EXPECT_FALSE(p.run.follower.exchangeCommit(100.08,10.08));
}

TEST(CandidateAdmissionTiming,CancelClearsSelectedAndLatestAndRejectsLateProofAndCandidate) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);p.offer(3,100.06);
  auto barrier=version();++barrier.reference_generation;
  ASSERT_TRUE(p.run.native.cancelReference(barrier));p.run.follower.retire("new_target");
  EXPECT_FALSE(Access::pendingCurve(p.run.native));EXPECT_FALSE(Access::queuedCurve(p.run.native));
  const auto count=p.published.size();p.offer(4,100.07);
  EXPECT_EQ(p.published.size(),count);EXPECT_FALSE(Access::pendingCurve(p.run.native));
  EXPECT_FALSE(p.run.follower.exchangeCommit(100.08,10.08));
}

TEST(CandidateAdmissionTiming,DuplicateOldAndForeignCandidatesNeverOverwriteLatestSlot) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);p.offer(4,100.06);p.offer(3,100.07);
  auto foreign=tag(9);foreign.task_id="other";auto proposal=p.run.reference;proposal.version.task_id="other";
  p.run.native.candidate(foreign,proposal,p.run.heading);
  ASSERT_TRUE(Access::queuedCurve(p.run.native));EXPECT_EQ(Access::queuedCurve(p.run.native)->trajectory.traj_id,4);
  EXPECT_EQ(p.published.size(),1u);
}

TEST(CandidateAdmissionTiming,IndependentSelectedMailboxDropsCanceledUnsentAndFencesInFlightGeometry) {
 for(const bool in_flight:{false,true}) {
  Integrated run;run.acquire(100.05,0.,0.);
  auto candidate=tag(2);candidate.trajectory.start_time=candidate.join_source_stamp=t::stamp(100.05);
  run.native.candidate(candidate,run.reference,run.heading);
  std::promise<void> publishing,release;auto started=publishing.get_future();auto unblock=release.get_future();
  unsigned delivered=0;bool accepted=false;
  Access::selectedObserver(run.native,[&](const auto& message){
    if(in_flight){publishing.set_value();unblock.wait();}
    ++delivered;accepted=run.follower.candidate(message);
  });
  std::future<void> consumer;
  if(in_flight)consumer=std::async(std::launch::async,[&]{Access::dispatchSelected(run.native);});
  const bool publishing_now=!in_flight||started.wait_for(std::chrono::seconds(3))==std::future_status::ready;
  EXPECT_TRUE(publishing_now);
  if(publishing_now) {
    auto fence=version();++fence.reference_generation;
    EXPECT_TRUE(run.native.cancelReference(fence));run.follower.retire("new_target");
    EXPECT_FALSE(Access::pendingCurve(run.native));EXPECT_FALSE(Access::queuedCurve(run.native));
  }
  release.set_value();if(consumer.valid())consumer.get();
  Access::dispatchSelected(run.native);
  EXPECT_EQ(delivered,in_flight?1u:0u);EXPECT_FALSE(accepted);
  EXPECT_FALSE(run.follower.exchangeCommit(100.06,100.06-90.));
 }
}

TEST(CandidateAdmissionTiming,MovingBodyAndShared120msRoundStillAdmitOriginalSelectedCurve) {
 for(const double speed:{.2,.3}) {
  SCOPED_TRACE(speed);
  Integrated run(true,false,{},tag(1,{0.,0.,.55},{speed,0.,0.},Eigen::Vector3d::Zero()));
  std::vector<std::int64_t> selected;
  Access::selectedObserver(run.native,[&](const auto& c){selected.push_back(c.trajectory.traj_id);
    EXPECT_TRUE(run.follower.candidate(c));});
  const auto body_only=[&](double source) {
    const auto o=body(source,speed*(source-100.),speed);
    EXPECT_TRUE(run.follower.body(o,source,source-90.));
    geometry_msgs::msg::Twist twist;twist.linear.x=speed;
    run.native.body({o.position,o.orientation,o.stamp,"odom"},twist,o.source_stamp_ns);
  };
  run.acquire(100.05,speed*.05,speed);
  auto first=tag(2,{speed*.05,0.,.55},{speed,0.,0.},Eigen::Vector3d::Zero());
  first.join_source_stamp=first.trajectory.start_time=t::stamp(100.05);
  run.native.candidate(first,run.reference,run.heading);
  Access::dispatchSelected(run.native);
  using Cycle=scan_planner::ValidationCycle;
  const auto begin=Cycle::Clock::now();auto scheduled_now=begin;
  double active_budget=0.,candidate_budget=0.;
  std::optional<w::TrajectoryValidation> active,proof;
  Cycle::admissionPass(begin,[&](double budget,auto deadline) {
    active_budget=budget;EXPECT_EQ(deadline,begin+std::chrono::milliseconds(120));
    active=Access::checkCurveRound(run.native,run.map,[]{return 100090000000LL;},budget,deadline,true);
    scheduled_now=begin+std::chrono::milliseconds(40);
  },[&](double budget,auto deadline) {
    candidate_budget=budget;EXPECT_EQ(deadline,begin+std::chrono::milliseconds(120));
    unsigned calls=0;
    proof=Access::checkCurveRound(run.native,run.map,[&] {
      if(++calls==2) {
        body_only(100.10);body_only(100.15);
        auto next=tag(3,{speed*.15,0.,.55},{speed,0.,0.},Eigen::Vector3d::Zero());
        next.join_source_stamp=next.trajectory.start_time=t::stamp(100.15);
        run.native.candidate(next,run.reference,run.heading);
      }
      return calls>=2?100160000000LL:100090000000LL;
    },budget,deadline,false);
    scheduled_now=begin+std::chrono::milliseconds(110);
  },[&]{return scheduled_now;});
  EXPECT_NEAR(active_budget,.12,1e-9);EXPECT_NEAR(candidate_budget,.08,1e-9);
  ASSERT_TRUE(active&&active->valid);ASSERT_TRUE(proof&&proof->valid)<< (proof?proof->reason:"no proof");
  EXPECT_EQ(proof->body_source_stamp,t::stamp(100.05));EXPECT_EQ(proof->front_ray_source_stamp,t::stamp(100.05));
  EXPECT_EQ(proof->check_end,t::stamp(100.16));EXPECT_EQ(proof->trajectory_id,2);
  ASSERT_TRUE(run.follower.validation(*proof));body_only(100.17);
  const auto request=run.follower.exchangeCommit(100.17,10.17);ASSERT_TRUE(request);
  const auto reply=run.native.localCommitExchangeAt(*request,100171000000LL);
  Access::dispatchSelected(run.native);
  ASSERT_TRUE(reply.accepted)<<reply.reason;
  ASSERT_EQ(selected.size(),2u);EXPECT_EQ(selected[1],3); // Before ACK, not an overwrite of exchange2.
  ASSERT_TRUE(run.follower.commitResult(reply,100.172,10.172));
  const auto commit=run.follower.exchangeCommit(100.173,10.173);ASSERT_TRUE(commit)<<run.follower.candidateReason();
  EXPECT_GT(commit->curve_time,.10);EXPECT_EQ(commit->body_source_stamp,t::stamp(100.17));
  const auto committed=run.native.localCommitExchangeAt(*commit,100174000000LL);
  ASSERT_TRUE(committed.accepted)<<committed.reason;
  ASSERT_TRUE(run.follower.commitResult(committed,100.175,10.175));
  EXPECT_EQ(run.native.committedTrajectory(),2);EXPECT_EQ(run.follower.controller().trajectoryId(),2);
 }
}

TEST(CandidateAdmissionTiming,IndependentHundredMillisecondCurrentAndCandidateKeepOriginalProofCoverage) {
  // Two EXISTING readers, two exclusive production GridMap snapshots. The
  // deterministic 100 ms check cost is real wall time inside Native checkAt,
  // not a virtual scheduler jump after a fast query. Source clocks describe
  // acquisition/check events; original body/ray/proof limits are unchanged.
  SelectedPipeline p;p.offer(2,100.05);GridMap candidate_map;
  GridMapTestAccess::configure(candidate_map,true);
  using Cycle=scan_planner::ValidationCycle;
  std::optional<w::TrajectoryValidation> selected_proof;
  auto previous=p.run.native.committedView();ASSERT_TRUE(previous&&previous->proof);
  auto original_deadline=t::timeNs(previous->proof->check_end)+250000000LL;
  for(unsigned round=0;round<5;++round) {
    const double source=100.05+round*.10;p.run.acquire(source,0.,0.);
    GridMapTestAccess::acquire(candidate_map,t::timeNs(t::stamp(source)));
    const auto run_reader=[&](bool current) {
      const auto begin=Cycle::Clock::now();unsigned clock_calls=0;
      std::optional<w::TrajectoryValidation> proof;
      const auto work=[&](double budget,auto deadline) {
        EXPECT_EQ(deadline,begin+std::chrono::milliseconds(120));
        proof=Access::checkCurveRound(p.run.native,current?p.run.map:candidate_map,[&] {
          if(++clock_calls==2)std::this_thread::sleep_for(std::chrono::milliseconds(100));
          return t::timeNs(t::stamp(source+(clock_calls>=2?.1:0.)));
        },budget,deadline,current);
        return proof&&proof->valid;
      };
      if(current)Cycle::formalCurrentCurvePass(begin,work,[]{return Cycle::Clock::now();});
      else Cycle::formalAdmissionPass(begin,work,[]{return Cycle::Clock::now();});
      EXPECT_GE(Cycle::Clock::now()-begin,std::chrono::milliseconds(100));
      return proof;
    };
    auto current_future=std::async(std::launch::async,[&]{return run_reader(true);});
    auto candidate_future=std::async(std::launch::async,[&]{return run_reader(false);});
    const auto current=current_future.get();selected_proof=candidate_future.get();
    ASSERT_TRUE(current&&current->valid)<<"round "<<round;
    ASSERT_TRUE(selected_proof&&selected_proof->valid)<<"round "<<round;
    EXPECT_EQ(current->body_source_stamp,t::stamp(source));
    EXPECT_EQ(current->front_ray_source_stamp,t::stamp(source));
    EXPECT_EQ(current->check_end,t::stamp(source+.1));
    EXPECT_LE(t::timeNs(current->check_end),original_deadline); // no 400 ms alternating gap
    original_deadline=t::timeNs(current->check_end)+250000000LL; // Scheduling observation only.
    EXPECT_EQ(t::timeNs(current->valid_until),0);
    EXPECT_EQ(p.run.native.committedTrajectory(),1); // checking never installs a candidate
  }
  ASSERT_TRUE(selected_proof);p.complete(*selected_proof,100.551);
  EXPECT_EQ(p.run.native.committedTrajectory(),2);EXPECT_EQ(p.run.follower.controller().trajectoryId(),2);
}

TEST(CandidateAdmissionTiming,IndependentPendingDoesNotPublishOrCommitCandidateCanceledDuringItsCheck) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);p.offer(3,100.06);
  using Cycle=scan_planner::ValidationCycle;
  const auto begin=Cycle::Clock::now();auto scheduled_now=begin;
  unsigned clock_calls=0,pending_calls=0;std::optional<w::TrajectoryValidation> proof;
  Cycle::formalAdmissionPass(begin,[&](double budget,auto deadline) {
    ++pending_calls;
    proof=Access::checkCurveRound(p.run.native,p.run.map,[&] {
      if(++clock_calls==2) {
        auto barrier=version();++barrier.reference_generation;
        EXPECT_TRUE(p.run.native.cancelReference(barrier));p.run.follower.retire("new_target");
      }
      return 100060000000LL;
    },budget,deadline,false);
  },[&]{return scheduled_now;});
  EXPECT_EQ(pending_calls,1u);EXPECT_GE(clock_calls,2u);EXPECT_FALSE(proof);
  EXPECT_FALSE(Access::pendingCurve(p.run.native));EXPECT_FALSE(Access::queuedCurve(p.run.native));
  EXPECT_FALSE(p.run.follower.exchangeCommit(100.07,10.07));
}

TEST(CandidateAdmissionTiming,CompleteCurrentReaderRetainsTheFormerSlowFallbackCapability) {
  Integrated f;auto curve=scan_planner::decodeSpliceCurve(tag().trajectory);ASSERT_TRUE(curve);
  const scan_planner::MeasuredBodyPose measured{{0.,0.,.55},Eigen::Quaterniond::Identity(),100.,"odom"};
  const auto check=[&](double budget,scan_planner::CurveCheckCompletion& completion) {
    bool first=true;
    return scan_planner::wholeCurveCollisionFree(*curve,.05,
      scan_planner::bodyExtent(f.map.planningBodyEnvelope()),
      [&](const Eigen::Vector3d& point,double yaw) {
        if(first){first=false;std::this_thread::sleep_for(std::chrono::milliseconds(60));}
        return f.map.getPlanningInflateOccupancy(point,yaw);
      },measured,"odom",100.,.4,200000,budget,0.,f.heading,0.,
      scan_planner::MeasuredConnectionPolicy::CommittedSweptConnection,.15,&completion);
  };
  scan_planner::CurveCheckCompletion completion;
  EXPECT_FALSE(check(scan_planner::ValidationCycle::local_curve_budget_s,completion));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::BudgetExhausted);
  EXPECT_TRUE(check(std::chrono::duration<double>(scan_planner::ValidationCycle::formal_curve_budget).count(),completion));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::Clear);
}

TEST(CandidateAdmissionTiming,FormalActiveOnlyAndCanceledCandidateDoNotRequestThirdSlot) {
  SelectedPipeline p;EXPECT_FALSE(p.run.native.slowSnapshotWanted()); // candidate == active
  p.run.acquire(100.05,0.,0.);p.offer(2,100.05);EXPECT_TRUE(p.run.native.slowSnapshotWanted());
  const auto proof=Access::checkCurve(p.run.native,p.run.map,100060000000LL);
  ASSERT_TRUE(proof&&proof->valid);p.complete(*proof,100.061);
  EXPECT_FALSE(p.run.native.slowSnapshotWanted());
  p.offer(3,100.07);EXPECT_TRUE(p.run.native.slowSnapshotWanted());
  auto barrier=version();++barrier.reference_generation;ASSERT_TRUE(p.run.native.cancelReference(barrier));
  EXPECT_FALSE(p.run.native.slowSnapshotWanted());
}

TEST(CandidateAdmissionTiming,LongCurrentCheckStillPublishesOccupiedAndFencesCancellation) {
  using Cycle=scan_planner::ValidationCycle;
  for(const bool canceled:{false,true}) {
    Integrated f;f.acquire(100.05,0.,0.);
    if(!canceled)GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
    const auto begin=Cycle::Clock::now();unsigned clocks=0;
    std::optional<w::TrajectoryValidation> proof;
    Cycle::formalCurrentCurvePass(begin,[&](double budget,auto deadline) {
      proof=Access::checkCurveRound(f.native,f.map,[&] {
        if(++clocks==2) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          if(canceled){auto barrier=version();++barrier.reference_generation;EXPECT_TRUE(f.native.cancelReference(barrier));}
        }
        return clocks>=2?100150000000LL:100050000000LL;
      },budget,deadline,true);
      return proof&&proof->valid;
    },[]{return Cycle::Clock::now();});
    if(canceled)EXPECT_FALSE(proof);
    else {ASSERT_TRUE(proof);EXPECT_FALSE(proof->valid);EXPECT_EQ(proof->reason,"occupied");}
  }
}

TEST(CandidateAdmissionTiming,ReferenceReplacementFencesBothSlotsAndLateOldGeneration) {
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);p.offer(3,100.06);
  auto next=p.run.reference;++next.version.reference_generation;next.proposal_id="new-window";
  next.expected_version=version();next.expected_trajectory_id=1;
  ASSERT_TRUE(p.run.native.replacePendingReference(next));
  EXPECT_FALSE(Access::pendingCurve(p.run.native));EXPECT_FALSE(Access::queuedCurve(p.run.native));
  const auto count=p.published.size();p.offer(4,100.07);EXPECT_EQ(p.published.size(),count);
  EXPECT_FALSE(Access::pendingCurve(p.run.native));
}

TEST(CandidateAdmissionTiming, PreparedAckCanCommitWithoutWaitingAnotherVelocityTick) {
  // Production Native/GridMap/typed follower and transport coalescer. Geometry
  // is already prepared here; asynchronous worker completion is separately
  // covered. Acknowledgement processing is NOT a velocity-control tick.
  SelectedPipeline p;p.run.acquire(100.05,0.,0.);p.offer(2,100.05);
  const auto proof=Access::checkCurve(p.run.native,p.run.map,100060000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(p.run.follower.validation(*proof));
  p.run.acquire(100.061,0.,0.);
  t::LocalCommitDispatch dispatch;std::vector<w::LocalTrajectoryCommit> outgoing;
  const auto pump=[&](double now) {
    return dispatch.forward(p.run.follower.exchangeCommit(now,now-90.),now-90.,
      [&](const auto& fact){outgoing.push_back(fact);});
  };
  ASSERT_TRUE(pump(100.061));ASSERT_EQ(outgoing.size(),1u);
  const auto prepare=outgoing.back();ASSERT_EQ(prepare.disposition,"prepare");
  EXPECT_FALSE(pump(100.062)); // Polls cannot create a reliable retry flood.
  const auto reserved=p.run.native.localCommitExchangeAt(prepare,100063000000LL);
  ASSERT_TRUE(reserved.accepted)<<reserved.reason;
  ASSERT_TRUE(p.run.follower.commitResult(reserved,100.064,10.064));
  ASSERT_TRUE(pump(100.064));ASSERT_EQ(outgoing.size(),2u);
  const auto commit=outgoing.back();ASSERT_EQ(commit.disposition,"committed");
  EXPECT_EQ(commit.commit_sequence,prepare.commit_sequence);
  EXPECT_EQ(commit.body_source_stamp,t::stamp(100.061));
  // An old exact request/ACK is not a second geometry transaction.
  const auto accepted=p.run.native.localCommitExchangeAt(commit,100065000000LL);
  ASSERT_TRUE(accepted.accepted)<<accepted.reason;
  ASSERT_TRUE(p.run.follower.commitResult(accepted,100.066,10.066));
  EXPECT_FALSE(pump(100.066));
  EXPECT_EQ(p.run.native.committedTrajectory(),2);
  EXPECT_EQ(p.run.follower.controller().trajectoryId(),2);
  EXPECT_LT(100.066-100.061,.05); // Both phases complete before the next 20 Hz tick.
  EXPECT_FALSE(p.run.follower.commitResult(reserved,100.067,10.067));
  p.run.follower.retire("new_target");
  EXPECT_FALSE(pump(100.068));
  EXPECT_FALSE(p.run.follower.commitResult(accepted,100.069,10.069));
}
