#include <gtest/gtest.h>
#include <string>
#include <plan_manage/execution_validator.hpp>
namespace scan_planner {
struct ExecutionValidatorTestAccess {
  static bool matches(const ew::MotionDemand& d,const ew::ExecutionPermit& p){return ExecutionValidator::demandPermitMatches(d,p);}
  static bool matches(const ew::MotionDemand& d,const ew::ExecutionPermit& p,std::uint64_t sequence){return ExecutionValidator::demandPermitMatches(d,p,sequence);}
  static auto proof(const std::deque<ew::TrajectoryValidation>& history,const ew::MotionDemand& d,std::int64_t now){return ExecutionValidator::motionProof(history,d,now);}
  static auto barrier(const ExecutionValidator& v){return v.motion_stop_barrier_;}
  static const auto& latest(const ExecutionValidator& v){return v.latest_permit_;}
  static void record(ExecutionValidator& v,const ew::MotionValidation& p){v.motion_proofs_.push_back(p);}
  static void recordWhole(ExecutionValidator& v,const ew::TrajectoryValidation& p){v.proofs_.push_back(p);}
  static bool supersedes(const ew::TrajectoryValidation& a,const ew::TrajectoryValidation& b){return ExecutionValidator::proofSupersedes(a,b);}
  static auto preparedCurve(const ExecutionValidator& v){return v.prepared_?v.prepared_->spline.trajectory.traj_id:-1;}
  static auto writerSequence(const ExecutionValidator& v){return v.writer_commit_sequence_;}
  static bool activeProof(const ExecutionValidator& v){return v.active_&&v.active_->proof.has_value();}
  static bool activeProgress(const ExecutionValidator& v){return v.active_&&v.active_->progress.has_value();}
  static bool entry(const ew::PreparedMotionDemand& x,const ew::ExecutionHandoffGrant& g,std::int64_t now){
    return ExecutionValidator::preparedEntryMatches(x,g,"odom",now);}
};
}
namespace {
namespace w=scan_planner::ew;
struct Fixture {
  scan_planner::CollisionSnapshotPool pool;
  scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},pool,"odom","isolated_mock"};
  w::ReferenceProposal proposal;
  w::TaggedBspline spline;
  Fixture(){
    auto& v=proposal.version;v.schema_version=3;v.session_id="session";v.task_id="task";v.route_id="route";
    v.route_hash="hash";v.map_version_id="map";v.localization_epoch=1;v.localization_seed_id="seed";
    v.reference_generation=1;v.anchor_id="anchor";v.anchor_revision=1;v.context_sequence=1;v.segment_id="floor1";
    proposal.proposal_id="proposal";spline.generation=1;spline.trajectory.traj_id=1;
  }
  w::ExecutionPermit permit(std::uint64_t sequence=1)const {
    w::ExecutionPermit p;p.version=proposal.version;p.geometry_committed=true;
    p.trajectory_id=spline.trajectory.traj_id;p.sequence=sequence;p.transport_mode="isolated_mock";return p;
  }
  w::TrackingProgress progress()const {
    const auto& v=proposal.version;w::TrackingProgress p;p.schema_version=2;p.valid=true;p.holding=true;
    p.header.stamp=rclcpp::Time(10000000000LL);p.header.frame_id="odom";
    p.session_id=v.session_id;p.task_id=v.task_id;p.route_id=v.route_id;p.route_hash=v.route_hash;
    p.map_version_id=v.map_version_id;p.localization_epoch=v.localization_epoch;p.localization_seed_id=v.localization_seed_id;
    p.generation=v.reference_generation;p.anchor_id=v.anchor_id;p.anchor_revision=v.anchor_revision;
    p.context_sequence=v.context_sequence;p.segment_id=v.segment_id;p.trajectory_id=spline.trajectory.traj_id;
    p.pose.orientation.w=1.;return p;
  }
};
}
TEST(ExecutionLedger,WriterCASPinsPreparedUntilMatchedAckAndNeverRollsBack) {
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  f.ledger.candidate(f.spline,f.proposal);ledger.candidate(f.spline,f.proposal);
  auto old=f.permit();old.allowed=true;old.execution_id="exec";old.control_epoch=1;
  old.sdk_session="sdk";old.sdk_arm_generation=1;old.source_stamp=rclcpp::Time(10000000000LL);
  old.valid_until=rclcpp::Time(10500000000LL);old.frame_id="odom";old.phase="tracking";
  ASSERT_TRUE(ledger.commit(old));w::ExecutionCommitAck first;first.schema_version=1;first.sequence=1;
  first.applied=first.write_submitted=true;first.commit_sequence=1;first.candidate_version=old.version;
  first.candidate_trajectory_id=1;first.execution_id=old.execution_id;first.control_epoch=1;first.sdk_session="sdk";
  first.sdk_arm_generation=1;first.permit_sequence=1;first.transport_mode="isolated_mock";
  first.applied_at=rclcpp::Time(10010000000LL);
  auto heartbeat=old;heartbeat.sequence=2;ASSERT_TRUE(ledger.commit(heartbeat));
  // The latest heartbeat may precede receipt of the first actual writer ACK.
  ASSERT_TRUE(ledger.commitAckAt(first,10020000000LL));
  // Real graph replacements can share the reference/anchor while changing
  // only trajectory identity. Ordinary permits must not bypass writer CAS.
  f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
  w::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="handoff";g.sequence=1;g.expected_commit_sequence=1;
  g.incumbent=heartbeat;g.candidate=heartbeat;g.candidate.version=f.proposal.version;g.candidate.trajectory_id=2;
  g.candidate.sequence=3;g.candidate.geometry_committed=false;g.source_stamp=rclcpp::Time(10020000000LL);
  g.valid_until=g.transition_deadline=rclcpp::Time(10300000000LL);
  ASSERT_TRUE(ledger.handoffAt(g,10020000000LL));EXPECT_EQ(ledger.committedTrajectory(),1);
  using A=scan_planner::ExecutionValidatorTestAccess;EXPECT_EQ(A::preparedCurve(ledger),2);
  auto changed=g;changed.candidate.trajectory_id=3;EXPECT_FALSE(ledger.handoffAt(changed,10030000000LL));
  f.spline.trajectory.traj_id=3;ledger.candidate(f.spline,f.proposal);ledger.cancelPending();EXPECT_EQ(A::preparedCurve(ledger),2);
  auto premature=g.candidate;premature.geometry_committed=true;EXPECT_FALSE(ledger.commit(premature));
  w::MotionValidation proof;proof.handoff_id=g.handoff_id;proof.sequence=30;proof.valid=true;
  proof.demand_sequence=7;proof.entry_admission_sequence=8;proof.demand_source_stamp=rclcpp::Time(10040000000LL);
  proof.demand_body_source_stamp=rclcpp::Time(10040000000LL);proof.entry_curve_time=.1;A::record(ledger,proof);
  auto ack=first;ack.sequence=2;ack.handoff_id=g.handoff_id;ack.grant_sequence=1;
  ack.previous_commit_sequence=1;ack.commit_sequence=2;ack.incumbent_version=g.incumbent.version;
  ack.incumbent_trajectory_id=1;ack.candidate_version=g.candidate.version;ack.candidate_trajectory_id=2;
  ack.permit_sequence=3;ack.motion_validation_sequence=30;ack.demand_sequence=7;ack.entry_admission_sequence=8;
  ack.demand_source_stamp=proof.demand_source_stamp;ack.demand_body_source_stamp=proof.demand_body_source_stamp;
  ack.curve_time=.1;ack.valid_until=rclcpp::Time(10140000000LL);
  ASSERT_TRUE(ledger.commitAckAt(ack,10050000000LL));EXPECT_EQ(ledger.committedTrajectory(),2);
  EXPECT_EQ(A::writerSequence(ledger),2U);EXPECT_TRUE(A::latest(ledger)->geometry_committed);
  heartbeat.sequence=9;EXPECT_FALSE(ledger.commit(heartbeat));EXPECT_EQ(ledger.committedTrajectory(),2);
  ++ack.sequence;ack.write_acknowledged=true;EXPECT_TRUE(ledger.commitAckAt(ack,10200000000LL));
}
TEST(ExecutionLedger,LateInitialAckRestoresOnlyBoundedOriginalGeometryAndHoldsWithoutRenewingProof) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const auto canceled:{false,true}) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto original=f.permit();original.allowed=true;
    original.execution_id="exec";original.control_epoch=original.sdk_arm_generation=1;original.sdk_session="sdk";
    original.source_stamp=rclcpp::Time(10000000000LL);original.valid_until=rclcpp::Time(10500000000LL);
    original.frame_id="odom";original.phase="tracking";ASSERT_TRUE(ledger.commit(original));
    auto next=original;next.sequence=2;
    if(canceled)next.revoked=true;
    else {f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);next.trajectory_id=2;}
    ASSERT_TRUE(ledger.commit(next));
    w::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;a.applied=a.write_submitted=true;
    a.commit_sequence=1;a.candidate_version=original.version;a.candidate_trajectory_id=1;
    a.execution_id=original.execution_id;a.control_epoch=a.sdk_arm_generation=1;a.sdk_session="sdk";
    a.permit_sequence=1;a.transport_mode="isolated_mock";a.applied_at=rclcpp::Time(10010000000LL);
    a.valid_until=rclcpp::Time(10110000000LL);
    ASSERT_TRUE(ledger.commitAckAt(a,10200000000LL));
    EXPECT_EQ(A::writerSequence(ledger),1U);EXPECT_EQ(ledger.committedTrajectory(),1);
    ASSERT_TRUE(A::latest(ledger));EXPECT_FALSE(A::latest(ledger)->allowed);EXPECT_EQ(A::latest(ledger)->phase,"holding");
    EXPECT_EQ(A::latest(ledger)->source_stamp,original.source_stamp);EXPECT_EQ(A::latest(ledger)->valid_until,original.valid_until);
    EXPECT_FALSE(A::activeProof(ledger));EXPECT_FALSE(A::activeProgress(ledger));
    const auto barrier=A::barrier(ledger);++a.sequence;a.write_acknowledged=true;
    ASSERT_TRUE(ledger.commitAckAt(a,10210000000LL));EXPECT_EQ(A::barrier(ledger),barrier);
    EXPECT_FALSE(A::latest(ledger)->allowed);EXPECT_EQ(ledger.committedTrajectory(),1);
    next.revoked=false;next.trajectory_id=2;++next.sequence;EXPECT_FALSE(ledger.commit(next));
  }
}
TEST(ExecutionLedger,InitialAckNeverGuessesMissingJobOrForeignWriterAndHistoryIsBounded) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const auto variant:{0U,1U,2U}) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto p=f.permit();p.allowed=true;p.execution_id="exec";
    p.control_epoch=p.sdk_arm_generation=1;p.sdk_session="sdk";p.frame_id="odom";p.phase="tracking";
    p.source_stamp=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10500000000LL);
    if(variant==0) {auto missing=p;missing.trajectory_id=99;EXPECT_FALSE(ledger.commit(missing));}
    else ASSERT_TRUE(ledger.commit(p));
    if(variant==2)for(int id=2;id<=9;++id) {
      f.spline.trajectory.traj_id=id;ledger.candidate(f.spline,f.proposal);auto next=p;next.sequence=id;next.trajectory_id=id;
      ASSERT_TRUE(ledger.commit(next));
    }
    w::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;a.applied=a.write_submitted=true;
    a.commit_sequence=1;a.candidate_version=p.version;a.candidate_trajectory_id=variant==0?99:1;
    a.execution_id=p.execution_id;a.control_epoch=a.sdk_arm_generation=1;a.sdk_session=variant==1?"foreign":"sdk";
    a.permit_sequence=1;a.transport_mode="isolated_mock";a.applied_at=rclcpp::Time(10010000000LL);
    a.valid_until=rclcpp::Time(10110000000LL);
    EXPECT_FALSE(ledger.commitAckAt(a,10020000000LL));EXPECT_EQ(A::writerSequence(ledger),0U);
  }
}
TEST(ExecutionLedger,ExactNegativeWriterFactReleasesPreparedButNeverAdvancesOrClearsIncumbent) {
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  ledger.candidate(f.spline,f.proposal);auto old=f.permit();old.allowed=true;old.execution_id="exec";
  old.control_epoch=old.sdk_arm_generation=1;old.sdk_session="sdk";old.frame_id="odom";old.phase="tracking";
  old.source_stamp=rclcpp::Time(10000000000LL);old.valid_until=rclcpp::Time(10500000000LL);
  ASSERT_TRUE(ledger.commit(old));w::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;
  a.applied=a.write_submitted=true;a.commit_sequence=1;a.candidate_version=old.version;a.candidate_trajectory_id=1;
  a.execution_id="exec";a.control_epoch=a.sdk_arm_generation=1;a.sdk_session="sdk";a.permit_sequence=1;
  a.transport_mode="isolated_mock";a.applied_at=rclcpp::Time(10010000000LL);
  ASSERT_TRUE(ledger.commitAckAt(a,10020000000LL));
  f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
  w::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="handoff";g.sequence=1;g.expected_commit_sequence=1;
  g.incumbent=old;g.candidate=old;g.candidate.sequence=2;g.candidate.trajectory_id=2;g.candidate.geometry_committed=false;
  g.source_stamp=rclcpp::Time(10020000000LL);g.valid_until=g.transition_deadline=rclcpp::Time(10300000000LL);
  ASSERT_TRUE(ledger.handoffAt(g,10020000000LL));
  a.sequence=2;a.handoff_id=g.handoff_id;a.grant_sequence=1;a.previous_commit_sequence=a.commit_sequence=1;
  a.incumbent_version=old.version;a.incumbent_trajectory_id=1;a.candidate_trajectory_id=2;
  a.applied=a.write_submitted=a.write_acknowledged=false;
  a.permit_sequence=0;EXPECT_FALSE(ledger.commitAckAt(a,10310000000LL));
  a.permit_sequence=2;auto advanced=a;advanced.commit_sequence=2;
  EXPECT_FALSE(ledger.commitAckAt(advanced,10310000000LL));
  auto uncertain=a;uncertain.write_submitted=true;EXPECT_FALSE(ledger.commitAckAt(uncertain,10310000000LL));
  ASSERT_TRUE(ledger.commitAckAt(a,10310000000LL));
  using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_EQ(A::writerSequence(ledger),1U);EXPECT_EQ(ledger.committedTrajectory(),1);
  auto heartbeat=old;heartbeat.sequence=3;ASSERT_TRUE(ledger.commit(heartbeat));
  auto retry=g;retry.handoff_id="retry";retry.sequence=2;retry.source_stamp=rclcpp::Time(10310000000LL);
  retry.valid_until=retry.transition_deadline=rclcpp::Time(10450000000LL);
  ASSERT_TRUE(ledger.handoffAt(retry,10310000000LL));
}
TEST(ExecutionLedger,PreparedZeroStillRequiresFreshExactEntryAndStrictBoundary) {
  using A=scan_planner::ExecutionValidatorTestAccess;Fixture f;w::ExecutionHandoffGrant g;
  g.schema_version=2;g.handoff_id="h";g.sequence=1;g.expected_commit_sequence=1;g.candidate=f.permit();
  auto& p=g.candidate;p.allowed=true;p.geometry_committed=false;p.frame_id="odom";p.phase="tracking";
  p.valid_until=rclcpp::Time(10500000000LL);g.valid_until=g.transition_deadline=p.valid_until;
  w::PreparedMotionDemand x;x.schema_version=1;x.handoff_id="h";x.grant_sequence=1;x.expected_commit_sequence=1;
  auto& d=x.demand;d.version=p.version;d.execution_id=p.execution_id;d.control_epoch=p.control_epoch;
  d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.permit_sequence=p.sequence;
  d.trajectory_id=p.trajectory_id;d.transport_mode=p.transport_mode;d.source_stamp=d.body_source_stamp=rclcpp::Time(10000000000LL);
  d.valid_until=rclcpp::Time(10100000000LL);d.hold=true;
  auto& a=x.entry_admission;a.sequence=1;a.version=d.version;a.trajectory_id=d.trajectory_id;
  a.body_source_stamp=d.body_source_stamp;a.checked_at=d.source_stamp;a.valid_until=d.valid_until;a.accepted=true;a.transport_mode=d.transport_mode;
  x.entry_source_stamp=d.body_source_stamp;x.measured_pose.header.stamp=x.entry_source_stamp;x.measured_pose.header.frame_id="odom";
  x.position_tolerance_m=.0125;x.velocity_tolerance_mps=.05;
  EXPECT_TRUE(A::entry(x,g,10010000000LL));
  for(int variant=0;variant<7;++variant){auto bad=x;
    if(variant==0)bad.entry_source_stamp=rclcpp::Time(9990000000LL);
    if(variant==1)bad.entry_admission.sequence=0;if(variant==2)bad.position_tolerance_m=.02;
    if(variant==3)bad.velocity_tolerance_mps=.1;if(variant==4)bad.position_error_m=.0126;
    if(variant==5)bad.measured_pose.header.frame_id="map";if(variant==6)bad.entry_admission.accepted=false;
    EXPECT_FALSE(A::entry(bad,g,10010000000LL))<<variant;}
  EXPECT_FALSE(A::entry(x,g,10100000000LL));
}
TEST(ExecutionLedger,StationaryReentryPinsSdkZeroAndMcEvidenceWithoutGrantingIncumbentMotion) {
  for(int variant=0;variant<10;++variant) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto old=f.permit();old.allowed=true;old.execution_id="exec";
    old.control_epoch=old.sdk_arm_generation=1;old.sdk_session="sdk";old.frame_id="odom";old.phase="tracking";
    old.source_stamp=rclcpp::Time(10000000000LL);old.valid_until=rclcpp::Time(10500000000LL);ASSERT_TRUE(ledger.commit(old));
    w::ExecutionCommitAck first;first.schema_version=1;first.sequence=1;first.applied=first.write_submitted=true;
    first.commit_sequence=1;first.candidate_version=old.version;first.candidate_trajectory_id=1;
    first.execution_id="exec";first.control_epoch=first.sdk_arm_generation=1;first.sdk_session="sdk";
    first.permit_sequence=1;first.transport_mode="isolated_mock";first.applied_at=rclcpp::Time(10010000000LL);
    ASSERT_TRUE(ledger.commitAckAt(first,10020000000LL));
    auto holding=old;holding.allowed=false;holding.phase="holding";holding.sequence=2;ASSERT_TRUE(ledger.commit(holding));
    f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
    w::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="reentry";g.sequence=1;g.expected_commit_sequence=1;
    g.transition_mode=w::ExecutionHandoffGrant::STATIONARY_REENTRY;g.incumbent=holding;g.candidate=old;
    g.candidate.sequence=3;g.candidate.trajectory_id=2;g.candidate.geometry_committed=false;
    g.source_stamp=rclcpp::Time(10040000000LL);g.valid_until=g.transition_deadline=rclcpp::Time(10240000000LL);
    g.retain_incumbent_until=g.source_stamp;g.candidate.source_stamp=g.source_stamp;g.candidate.valid_until=g.valid_until;
    auto& e=g.stationary_evidence;e.schema_version=1;e.version=old.version;e.execution_id="exec";
    e.control_epoch=e.sdk_arm_generation=1;e.sdk_session="sdk";e.transport_mode="isolated_mock";
    e.sequence=e.zero_write_sequence=e.writer_commit_sequence=1;e.applied_trajectory_id=1;
    e.zero_ack_at=rclcpp::Time(9400000000LL);e.mc_raw_stamp_ns=900;e.mc_clock_epoch="clock";
    e.time_basis="isolated_simulated_source_clock";e.source_stamp=e.received_stamp=g.source_stamp;
    e.capture_lower_bound=rclcpp::Time(10030000000LL);e.capture_upper_bound=g.source_stamp;e.valid_until=g.valid_until;
    e.stationary_samples=3;e.stationary_duration_sec=.64;e.nonzero_blocked=e.usable=true;
    if(variant==1)g.schema_version=1;if(variant==2)e.usable=false;
    if(variant==3)e.zero_write_sequence=0;if(variant==4)e.capture_lower_bound=e.zero_ack_at;
    if(variant==5)e.sdk_session="foreign";if(variant==6)g.incumbent.allowed=true;
    if(variant==7)e.valid_until=rclcpp::Time(10030000000LL);if(variant==8)e.nonzero_blocked=false;
    if(variant==9)g.candidate.valid_until=rclcpp::Time(10250000000LL);
    EXPECT_EQ(ledger.handoffAt(g,10040000000LL),variant==0)<<variant;
    EXPECT_EQ(ledger.committedTrajectory(),1);EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(ledger)->allowed);
  }
}
TEST(ExecutionLedger,PreparedSweepComesAfterIncumbentRenewalAndSharesFixedRoundBudget) {
  using C=scan_planner::ValidationCycle;const auto begin=C::Clock::time_point{};auto now=begin;
  std::vector<std::string> order;
  C::priorityPassWithPrepared(begin,[&](double){order.push_back("current_sweep");now+=std::chrono::milliseconds(2);},
    [&](double budget){order.push_back("current_curve");EXPECT_LE(budget,.041);now+=std::chrono::milliseconds(30);return true;},
    [&](double budget){order.push_back("prepared_sweep");EXPECT_LE(budget,.005);now+=std::chrono::milliseconds(5);},[&]{return now;});
  EXPECT_EQ(order,(std::vector<std::string>{"current_sweep","current_curve","current_sweep","prepared_sweep"}));
  EXPECT_LT(now,begin+C::period);
  order.clear();now=begin;
  C::priorityPassWithPrepared(begin,[&](double){order.push_back("current_sweep");now=begin+std::chrono::milliseconds(51);},
    [&](double){ADD_FAILURE()<<"expired round must not renew";return true;},
    [&](double){ADD_FAILURE()<<"expired round must not prepare";},[&]{return now;});
  EXPECT_EQ(order,(std::vector<std::string>{"current_sweep"}));
}
TEST(ExecutionLedger,CandidateAndDebugNeverReplaceCommittedOwner) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);EXPECT_FALSE(f.ledger.committedView());
  w::LocalPlanDebug d;d.generation=1;d.plan_id=1;d.valid=true;f.ledger.candidateDebug(d);
  ASSERT_TRUE(f.ledger.commit(f.permit()));
  f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);d.plan_id=2;f.ledger.candidateDebug(d);
  auto active=f.ledger.committedView();ASSERT_TRUE(active&&active->debug);
  EXPECT_EQ(active->spline.trajectory.traj_id,1);EXPECT_EQ(active->debug->plan_id,1U);
  f.ledger.cancelPending();EXPECT_EQ(f.ledger.committedTrajectory(),1);
}
TEST(ExecutionLedger,SlowProofCannotOverwriteNewerSourceMapOrNegativeEvidence) {
  w::TrajectoryValidation current,incoming;
  current.sequence=10;current.map_snapshot_revision=20;current.body_source_stamp=rclcpp::Time(10000000000LL);
  current.valid=false;incoming=current;incoming.valid=true;
  using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_FALSE(A::supersedes(incoming,current));
  incoming.sequence=11;incoming.map_snapshot_revision=19;EXPECT_FALSE(A::supersedes(incoming,current));
  incoming.map_snapshot_revision=20;incoming.body_source_stamp=rclcpp::Time(9999999999LL);
  EXPECT_FALSE(A::supersedes(incoming,current));
  incoming.body_source_stamp=current.body_source_stamp;EXPECT_TRUE(A::supersedes(incoming,current));
  // Validity itself never overrides chronological source / snapshot evidence.
}
TEST(ExecutionLedger,FastLaneChecksLatestMotionBeforeIncumbentAndNeverRunsCandidate) {
  const auto begin=scan_planner::ValidationCycle::Clock::now();std::vector<std::string> order;
  scan_planner::ValidationCycle::priorityPass(begin,[&]{order.push_back("motion");},
    [&](double budget){order.push_back("active");EXPECT_GT(budget,0.);EXPECT_LE(budget,.050);});
  EXPECT_EQ(order,(std::vector<std::string>{"motion","active"}));
  EXPECT_EQ(scan_planner::ValidationCycle::activeBudget(begin,begin+std::chrono::milliseconds(55)),0.);
  order.clear();
  scan_planner::ValidationCycle::priorityPass(begin-std::chrono::seconds(1),[&]{order.push_back("motion");},
    [&](double){order.push_back("active");});
  EXPECT_EQ(order,(std::vector<std::string>{"motion"}));
}

TEST(ExecutionLedger,PositiveCurrentRenewalRechecksLatestDemandWithinSameRoundAtMostOnce) {
  using Cycle=scan_planner::ValidationCycle;
  const auto begin=Cycle::Clock::time_point{};auto now=begin;
  std::vector<std::string> order;std::vector<double> motion_budgets;
  const auto count=Cycle::priorityPassWithRenewal(begin,[&](double budget){
      order.push_back("motion");motion_budgets.push_back(budget);now+=std::chrono::milliseconds(1);
    },[&](double budget){
      order.push_back("published_positive_current_renewal");
      EXPECT_NEAR(budget,.044,1e-9);now+=std::chrono::milliseconds(42);return true;
    },[&]{return now;});
  EXPECT_EQ(count,2U);
  EXPECT_EQ(order,(std::vector<std::string>{"motion","published_positive_current_renewal","motion"}));
  EXPECT_DOUBLE_EQ(motion_budgets[0],.025);
  EXPECT_NEAR(motion_budgets[1],.007,1e-9); // original 50 ms end, not another 25 ms.
  EXPECT_LT(now-begin,Cycle::period);
}

TEST(ExecutionLedger,FailedExpiredRetiredOrOverBudgetRenewalCannotRequestPositiveMotionRetry) {
  using Cycle=scan_planner::ValidationCycle;
  for(const auto* reason:{"occupied","source_expired_during_check","retired_job","curve_check_budget_exhausted"}) {
    const auto begin=Cycle::Clock::time_point{};auto now=begin;unsigned motions=0;
    EXPECT_EQ(Cycle::priorityPassWithRenewal(begin,[&](double){++motions;},
      [&](double){now+=std::chrono::milliseconds(10);return false;},[&]{return now;}),1U)<<reason;
    EXPECT_EQ(motions,1U)<<reason; // actual check() returns false for all these dispositions.
  }
  const auto begin=Cycle::Clock::time_point{};auto now=begin;unsigned motions=0;
  EXPECT_EQ(Cycle::priorityPassWithRenewal(begin,[&](double){++motions;},
    [&](double){now=begin+std::chrono::milliseconds(51);return true;},[&]{return now;}),1U);
  EXPECT_EQ(motions,1U); // even a late positive cannot obtain a renewed CPU deadline.
}

TEST(ExecutionLedger,CurrentBudgetMissRetainsOnlyOriginalLeaseAndNeverSuppressesActualOccupied) {
  using Cycle=scan_planner::ValidationCycle;
  const std::int64_t original_deadline=1250000000LL;
  EXPECT_TRUE(Cycle::inconclusiveRefresh(true,"curve_check_budget_exhausted"));
  EXPECT_TRUE(Cycle::originalLeaseUsable(true,original_deadline,1200000000LL));
  EXPECT_FALSE(Cycle::originalLeaseUsable(true,original_deadline,1250000000LL));
  EXPECT_FALSE(Cycle::originalLeaseUsable(true,original_deadline,1300000000LL));
  EXPECT_FALSE(Cycle::originalLeaseUsable(false,original_deadline,1200000000LL));
  // Candidate budget failure is never an admission, and real new occupied /
  // unknown / expired-source evidence is published even for the incumbent.
  EXPECT_FALSE(Cycle::inconclusiveRefresh(false,"curve_check_budget_exhausted"));
  EXPECT_FALSE(Cycle::inconclusiveRefresh(true,"occupied"));
  EXPECT_FALSE(Cycle::inconclusiveRefresh(true,"unknown_or_unobserved"));
  EXPECT_FALSE(Cycle::inconclusiveRefresh(true,"source_expired_during_check"));
  EXPECT_EQ(original_deadline,1250000000LL); // No completion-time extension.
}
TEST(ExecutionLedger,BlockedEntryRequiresExactNativeOccupiedProofAndNeverGrantsMotion) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  w::MotionValidation p;p.version=f.proposal.version;p.trajectory_id=1;p.sequence=5;
  p.reason="motion_sweep_occupied";p.velocity.linear.x=.01;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10090000000LL);
  constexpr std::int64_t now=10010000000LL;
  EXPECT_FALSE(f.ledger.submitBlockedEntryAt(p,now)); // Not a native published proof.
  scan_planner::ExecutionValidatorTestAccess::record(f.ledger,p);
  for(int mode=0;mode<5;++mode) {
    auto bad=p;
    if(mode==0)bad.velocity.linear.x=.02;
    if(mode==1)bad.version.task_id="other";
    if(mode==2)bad.valid=true;
    if(mode==3)bad.reason="motion_sweep_unknown_or_expired";
    EXPECT_FALSE(f.ledger.submitBlockedEntryAt(bad,mode==4?10090000000LL:now));
  }
  ASSERT_TRUE(f.ledger.submitBlockedEntryAt(p,now));
  EXPECT_FALSE(f.ledger.submitBlockedEntryAt(p,now)); // One event, no periodic replan loop.
  auto event=f.ledger.consumeBlockedEntry();ASSERT_TRUE(event);EXPECT_EQ(*event,p);
  EXPECT_FALSE(f.ledger.consumeBlockedEntry());
  EXPECT_EQ(f.ledger.committedTrajectory(),1); // Task/curve is not cleared or replaced.
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(f.ledger)->allowed);
}
TEST(ExecutionLedger,OccupiedInPlaceTurnIsBlockedEntryButStillAndReverseAreNot) {
  // Each variant is recorded as a genuine native proof, so only the command
  // shape decides: an in-place rotation is an entry, zero/reverse never are.
  struct Case{double vx,wz;bool accepted;};
  for(const auto c:{Case{0.,.3,true},Case{0.,-.3,true},Case{0.,0.,false},
                    Case{0.,1e-10,false},Case{-.01,.3,false},Case{-.01,0.,false}}) {
    SCOPED_TRACE(std::to_string(c.vx)+","+std::to_string(c.wz));
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    w::MotionValidation p;p.version=f.proposal.version;p.trajectory_id=1;p.sequence=5;
    p.reason="motion_sweep_occupied";p.velocity.linear.x=c.vx;p.velocity.angular.z=c.wz;
    p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10090000000LL);
    scan_planner::ExecutionValidatorTestAccess::record(f.ledger,p);
    auto unknown=p;unknown.sequence=4;unknown.reason="motion_sweep_unknown_or_expired";
    scan_planner::ExecutionValidatorTestAccess::record(f.ledger,unknown);
    EXPECT_FALSE(f.ledger.submitBlockedEntryAt(unknown,10010000000LL));
    EXPECT_EQ(f.ledger.submitBlockedEntryAt(p,10010000000LL),c.accepted);
    EXPECT_EQ(static_cast<bool>(f.ledger.consumeBlockedEntry()),c.accepted);
    EXPECT_EQ(f.ledger.committedTrajectory(),1);
    EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(f.ledger)->allowed);
  }
}
TEST(ExecutionLedger,RevokedOrReplacedOwnerCannotConsumeOldBlockedEntry) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  w::MotionValidation p;p.version=f.proposal.version;p.trajectory_id=1;p.sequence=5;
  p.reason="motion_sweep_occupied";p.velocity.linear.x=.01;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10090000000LL);
  scan_planner::ExecutionValidatorTestAccess::record(f.ledger,p);
  ASSERT_TRUE(f.ledger.submitBlockedEntryAt(p,10010000000LL));
  auto permit=f.permit(2);permit.revoked=true;ASSERT_TRUE(f.ledger.commit(permit));
  EXPECT_FALSE(f.ledger.consumeBlockedEntry());
}
TEST(ExecutionLedger,OnlyMatchingOwnerGeometryCommitSwitchesCurve) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);auto p=f.permit(2);
  p.trajectory_id=3;EXPECT_FALSE(f.ledger.commit(p));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  p.trajectory_id=2;ASSERT_TRUE(f.ledger.commit(p));EXPECT_EQ(f.ledger.committedTrajectory(),2);
}
TEST(ExecutionLedger,HoldingFreshMeasuredProgressIsNotMissingOrVirtualMotion) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto p=f.progress();f.ledger.progressAt(p,10.02);auto active=f.ledger.committedView();
  ASSERT_TRUE(active&&active->progress);EXPECT_TRUE(active->progress->holding);
  EXPECT_DOUBLE_EQ(active->progress->curve_time,0.);
  p.trajectory_id=2;p.curve_time=2.;p.arc_length=p.s_committed=.5;f.ledger.progressAt(p,10.02);
  EXPECT_DOUBLE_EQ(f.ledger.committedView()->progress->curve_time,0.);
}
TEST(ExecutionLedger,ExactFreshCandidateEntryRejectionIsOneBoundedReseedNotGeometryRetirement) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const auto* reason:{"candidate:entry_measured_velocity_off_candidate","candidate:entry_measured_body_not_on_candidate"}) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
    w::TrajectoryValidation v;v.version=f.proposal.version;v.trajectory_id=2;v.sequence=50;v.valid=v.whole_curve=true;
    v.frame_id="odom";v.transport_mode="isolated_mock";
    v.check_end=rclcpp::Time(10000000000LL);v.valid_until=rclcpp::Time(10200000000LL);A::recordWhole(f.ledger,v);
    w::TrajectoryAdmission a;a.sequence=1;a.version=v.version;a.trajectory_id=2;a.validation_sequence=50;
    a.transport_mode="isolated_mock";a.reason=reason;a.body_source_stamp=rclcpp::Time(10010000000LL);
    a.checked_at=rclcpp::Time(10020000000LL);a.valid_until=rclcpp::Time(10170000000LL);
    ASSERT_TRUE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL));
    EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL));
    auto event=f.ledger.consumeFormalReseedAt(10030000000LL);ASSERT_TRUE(event);
    EXPECT_EQ(event->version,a.version);EXPECT_EQ(event->trajectory_id,2);EXPECT_FALSE(event->terminal);
    EXPECT_EQ(event->body_source_stamp,a.body_source_stamp);EXPECT_EQ(f.ledger.committedTrajectory(),1);
    EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10030000000LL));
    ++a.sequence;a.body_source_stamp=rclcpp::Time(10040000000LL);a.checked_at=rclcpp::Time(10050000000LL);
    EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10055000000LL));
    // A NEW real solver curve may schedule its own bounded retry, not a queue
    // of old failures against curve 2 or a new task/global route.
    f.spline.trajectory.traj_id=3;f.ledger.candidate(f.spline,f.proposal);
    v.trajectory_id=3;++v.sequence;A::recordWhole(f.ledger,v);
    a.trajectory_id=3;a.validation_sequence=v.sequence;++a.sequence;
    ASSERT_TRUE(f.ledger.submitCandidateEntryRejectionAt(a,10055000000LL));
    EXPECT_EQ(f.ledger.consumeFormalReseedAt(10060000000LL)->trajectory_id,3);
    EXPECT_EQ(f.ledger.committedTrajectory(),1);
  }
}
TEST(ExecutionLedger,EntryReseedRejectsWrongTaskProofSourceUnknownAndSuccess) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int variant=0;variant<17;++variant) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
    w::TrajectoryValidation v;v.version=f.proposal.version;v.trajectory_id=2;v.sequence=50;v.valid=v.whole_curve=true;
    v.frame_id="odom";v.transport_mode="isolated_mock";
    v.check_end=rclcpp::Time(10000000000LL);v.valid_until=rclcpp::Time(10200000000LL);
    if(variant==9)v.valid=false;if(variant==10)v.valid_until=rclcpp::Time(10020000000LL);
    if(variant==13)v.whole_curve=false;if(variant==14)v.remaining_curve=true;
    if(variant==15)v.frame_id="map";if(variant==16)v.transport_mode="live";
    A::recordWhole(f.ledger,v);
    w::TrajectoryAdmission a;a.sequence=1;a.version=v.version;a.trajectory_id=2;a.validation_sequence=50;
    a.transport_mode="isolated_mock";a.reason="candidate:entry_measured_velocity_off_candidate";
    a.body_source_stamp=rclcpp::Time(10010000000LL);a.checked_at=rclcpp::Time(10020000000LL);
    a.valid_until=rclcpp::Time(10170000000LL);
    if(variant==0)a.version.task_id="other";if(variant==1)++a.version.context_sequence;
    if(variant==2)++a.version.map_geometry_revision;if(variant==3)a.trajectory_id=1;
    if(variant==4)++a.validation_sequence;if(variant==5)a.transport_mode="live";
    if(variant==6)a.body_source_stamp=rclcpp::Time(9900000000LL);
    if(variant==7)a.body_source_stamp=rclcpp::Time(10060000000LL);
    if(variant==8)a.valid_until=rclcpp::Time(10025000000LL);
    if(variant==11)a.reason="candidate:unknown_space";if(variant==12)a.accepted=true;
    EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL))<<variant;
    EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10030000000LL))<<variant;
    EXPECT_EQ(f.ledger.committedTrajectory(),1);
  }
}
TEST(ExecutionLedger,QueuedEntryFailureCoalescesAndIsDiscardedForAcceptedReplacedExpiredOrRevokedCandidate) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int variant=0;variant<4;++variant) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
    w::TrajectoryValidation v;v.version=f.proposal.version;v.trajectory_id=2;v.sequence=50;v.valid=v.whole_curve=true;
    v.frame_id="odom";v.transport_mode="isolated_mock";
    v.check_end=rclcpp::Time(10000000000LL);v.valid_until=rclcpp::Time(10200000000LL);A::recordWhole(f.ledger,v);
    w::TrajectoryAdmission a;a.sequence=1;a.version=v.version;a.trajectory_id=2;a.validation_sequence=50;
    a.transport_mode="isolated_mock";a.reason="candidate:entry_measured_velocity_off_candidate";
    a.body_source_stamp=rclcpp::Time(10010000000LL);a.checked_at=rclcpp::Time(10020000000LL);
    a.valid_until=rclcpp::Time(10170000000LL);
    ASSERT_TRUE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL));
    if(variant==0){++a.sequence;a.accepted=true;EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10026000000LL));}
    if(variant==1){++f.spline.trajectory.traj_id;f.ledger.candidate(f.spline,f.proposal);}
    if(variant==3){auto p=f.permit(2);p.revoked=true;ASSERT_TRUE(f.ledger.commit(p));}
    EXPECT_FALSE(f.ledger.consumeFormalReseedAt(variant==2?10180000000LL:10030000000LL));
  }
}
TEST(ExecutionLedger,MeasuredTerminalReseedIsOncePerAppliedCurveNotElapsedCandidateClockOrPreviewHold) {
  for(int variant=0;variant<5;++variant) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    auto p=f.progress();p.reason="local_segment_finished_waiting_replan";
    if(variant==1)p.reason="braking_envelope_reentry_required";if(variant==2)p.holding=false;
    if(variant==3)p.trajectory_id=2;if(variant==4)p.header.stamp=rclcpp::Time(9500000000LL);
    f.ledger.progressAt(p,10.02);
    auto event=f.ledger.consumeFormalReseedAt(10030000000LL);
    ASSERT_EQ(static_cast<bool>(event),variant==0)<<variant;
    if(event) {
      EXPECT_TRUE(event->terminal);EXPECT_EQ(event->trajectory_id,1);
      EXPECT_EQ(event->body_source_stamp,p.header.stamp);
      EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10031000000LL));
      p.header.stamp=rclcpp::Time(10040000000LL);f.ledger.progressAt(p,10.05);
      EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10060000000LL));
      EXPECT_EQ(f.ledger.committedTrajectory(),1);EXPECT_TRUE(f.ledger.committedView()->progress->holding);
    }
  }
}
TEST(ExecutionLedger,WrongFloorAnchorAndExpiredProgressNeverReplaceSource) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  for(int mode=0;mode<4;++mode) {
    auto p=f.progress();if(mode==0)p.anchor_id="other";if(mode==1)p.segment_id="floor2";
    if(mode==2)p.valid=false;
    f.ledger.progressAt(p,mode==3?10.5:10.02);EXPECT_FALSE(f.ledger.committedView()->progress);
  }
}
TEST(ExecutionLedger,RevocationRetiresCandidateAndRejectsLateGeometry) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto p=f.permit(2);p.revoked=true;ASSERT_TRUE(f.ledger.commit(p));
  f.ledger.candidate(f.spline,f.proposal);EXPECT_FALSE(f.ledger.commit(f.permit(3)));
  EXPECT_FALSE(f.ledger.committedView());
}
TEST(ExecutionLedger,ReferenceCancellationRetiresOwnerBeforeAcknowledgementButNotForeignTasks) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto barrier=f.proposal.version;++barrier.reference_generation;auto foreign=barrier;foreign.task_id="other";
  EXPECT_FALSE(f.ledger.cancelReference(foreign));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  EXPECT_FALSE(f.ledger.cancelReference(f.proposal.version));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  ASSERT_TRUE(f.ledger.cancelReference(barrier));EXPECT_FALSE(f.ledger.committedView());
  f.ledger.candidate(f.spline,f.proposal);EXPECT_FALSE(f.ledger.commit(f.permit(2)));
  // A genuinely new owner is independent of the retired route's generation.
  f.proposal.version.task_id="new-task";f.proposal.version.reference_generation=3;
  f.ledger.candidate(f.spline,f.proposal);EXPECT_TRUE(f.ledger.commit(f.permit(3)));
}
TEST(ExecutionLedger,ReplanBoundaryUsesActualTwistAndNeverInventsMeasuredAcceleration) {
  Fixture f;auto p=f.progress();p.twist.linear.x=.0648;p.twist.linear.z=.003;
  p.acceleration.linear.z=-.4;p.acceleration_valid=false;
  auto b=scan_planner::ExecutionValidator::measuredBoundary(p,Eigen::Vector3d::Zero());ASSERT_TRUE(b);
  EXPECT_DOUBLE_EQ(b->velocity.z(),.003);EXPECT_TRUE(b->acceleration.isZero());
  p.acceleration_valid=true;b=scan_planner::ExecutionValidator::measuredBoundary(p,Eigen::Vector3d::Zero());
  ASSERT_TRUE(b);EXPECT_DOUBLE_EQ(b->acceleration.z(),-.4);
  EXPECT_FALSE(scan_planner::ExecutionValidator::measuredBoundary(p,Eigen::Vector3d(.02,0,0)));
}
TEST(ExecutionLedger,ActualConnectorRequiresSourceGroundNotImaginaryLineOrAnotherFloor) {
  w::SupportReference s;s.verified=true;s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";
  s.frame_id="odom";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.support_xy_radius_m=.02;s.body_reference_height_m=.55;s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  for(int i=0;i<=12;++i){geometry_msgs::msg::Point p;p.y=i*.01;s.support_ground_xyz.push_back(p);}
  const Eigen::Vector3d a(0,0,.55),b(0,.12,.55);
  EXPECT_TRUE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  const auto full=s.support_ground_xyz;s.support_ground_xyz={full.front(),full.back()};
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  s.support_ground_xyz=full;s.floor_id="floor2";
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  s.floor_id="floor1";for(auto& p:s.support_ground_xyz)p.z=3.;
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
}
TEST(ExecutionLedger,CommandProofRequiresExactCurrentOwnerAndCannotAuthorizeHolding) {
  Fixture f;auto p=f.permit();p.allowed=true;p.phase="tracking";p.execution_id="exec";
  p.sdk_session="sdk";p.sdk_arm_generation=2;p.control_epoch=3;
  w::MotionDemand d;d.version=p.version;d.trajectory_id=p.trajectory_id;d.execution_id=p.execution_id;
  d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.control_epoch=p.control_epoch;
  using A=scan_planner::ExecutionValidatorTestAccess;EXPECT_TRUE(A::matches(d,p));
  for(int mode=0;mode<7;++mode){auto bad=p;
    if(mode==0)bad.allowed=false;if(mode==1)bad.phase="holding";if(mode==2)bad.revoked=true;
    if(mode==3)++bad.sdk_arm_generation;if(mode==4)++bad.control_epoch;
    if(mode==5)bad.version.anchor_id="other";if(mode==6)++bad.trajectory_id;
    EXPECT_FALSE(A::matches(d,bad));
  }
}
TEST(ExecutionLedger,LaterZeroAndPermissionRevocationFenceLateMotionProof) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  using A=scan_planner::ExecutionValidatorTestAccess;
  w::MotionDemand d;d.version=f.proposal.version;d.transport_mode="isolated_mock";d.sequence=1;d.execution_id="execution";d.velocity.linear.x=.2;
  f.ledger.demand(d);auto barrier=A::barrier(f.ledger);++d.sequence;d.velocity.linear.x=.1;f.ledger.demand(d);
  EXPECT_EQ(A::barrier(f.ledger),barrier);++d.sequence;d.hold=true;f.ledger.demand(d);
  EXPECT_GT(A::barrier(f.ledger),barrier);
  auto pause=f.permit(2);pause.geometry_committed=false;pause.phase="holding";
  EXPECT_FALSE(f.ledger.commit(pause));ASSERT_TRUE(A::latest(f.ledger));EXPECT_EQ(A::latest(f.ledger)->phase,"holding");
  EXPECT_FALSE(f.ledger.commit(f.permit(1)));EXPECT_EQ(A::latest(f.ledger)->phase,"holding");
}
TEST(ExecutionLedger,NativeMotionDemandMayUseNewerEvidenceButNeverBelowOwnerFloor) {
  Fixture f;auto p=f.permit();p.allowed=true;p.phase="tracking";p.execution_id="exec";
  p.sdk_session="sdk";p.sdk_arm_generation=2;p.control_epoch=3;p.validation_sequence=40;
  w::MotionDemand d;d.version=p.version;d.trajectory_id=p.trajectory_id;d.execution_id=p.execution_id;
  d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.control_epoch=p.control_epoch;
  using A=scan_planner::ExecutionValidatorTestAccess;
  d.validation_sequence=41;EXPECT_TRUE(A::matches(d,p));
  d.validation_sequence=40;EXPECT_TRUE(A::matches(d,p));
  d.validation_sequence=39;EXPECT_FALSE(A::matches(d,p));
  d.validation_sequence=41;d.version.anchor_revision++;EXPECT_FALSE(A::matches(d,p));
}
TEST(ExecutionLedger,RefreshedCurveProofDoesNotInheritExpiredTwoHundredMillisecondFloor) {
  Fixture f;w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.validation_sequence=40;
  w::TrajectoryValidation old;old.version=d.version;old.trajectory_id=1;old.sequence=40;old.valid=true;
  old.check_end=rclcpp::Time(10000000000LL);old.valid_until=rclcpp::Time(10200000000LL);
  auto newer=old;newer.sequence=41;newer.check_end=rclcpp::Time(10200000000LL);newer.valid_until=rclcpp::Time(10400000000LL);
  std::deque<w::TrajectoryValidation> history{old,newer};
  const auto selected=scan_planner::ExecutionValidatorTestAccess::proof(history,d,10210000000LL);
  ASSERT_TRUE(selected);EXPECT_EQ(selected->sequence,41U);EXPECT_EQ(rclcpp::Time(selected->valid_until).nanoseconds(),10400000000LL);
  // Production motionEvidenceDeadline still caps the ORIGINAL command/source;
  // selecting a new map check does not create another 100 ms command lease.
  EXPECT_EQ(scan_planner::motionEvidenceDeadline(10190000000LL,10180000000LL,10200000000LL,
    10280000000LL,10400000000LL,rclcpp::Time(selected->valid_until).nanoseconds(),10400000000LL,
    10000000000LL,10000000000LL),10280000000LL);
}
TEST(ExecutionLedger,NegativeCurveRevisionFencesOldCommandEvenAfterPositiveRecovery) {
  Fixture f;w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.validation_sequence=40;
  w::TrajectoryValidation p;p.version=d.version;p.trajectory_id=1;p.sequence=40;p.valid=true;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10400000000LL);
  auto negative=p;negative.sequence=41;negative.valid=false;auto recovered=p;recovered.sequence=42;
  std::deque<w::TrajectoryValidation> history{p,negative,recovered};using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_FALSE(A::proof(history,d,10200000000LL));d.validation_sequence=41;EXPECT_FALSE(A::proof(history,d,10200000000LL));
  d.validation_sequence=42;ASSERT_TRUE(A::proof(history,d,10200000000LL));
}
TEST(ExecutionLedger,MotionProofCannotBorrowOtherVersionCurveFutureOrExpiredNewestEvidence) {
  Fixture f;w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.validation_sequence=40;
  w::TrajectoryValidation p;p.version=d.version;p.trajectory_id=1;p.sequence=40;p.valid=true;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10400000000LL);
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int mode=0;mode<6;++mode){auto bad=p;
    if(mode==0)++bad.version.anchor_revision;
    if(mode==1)++bad.trajectory_id;
    if(mode==2)bad.sequence=39;
    if(mode==3)bad.valid_until=rclcpp::Time(10200000000LL);
    if(mode==4)bad.check_end=rclcpp::Time(10200000001LL);
    if(mode==5)bad.valid=false;
    EXPECT_FALSE(A::proof({bad},d,10200000000LL));
  }
  auto latest=p;latest.sequence=41;latest.valid_until=rclcpp::Time(10200000000LL);
  EXPECT_FALSE(A::proof({p,latest},d,10200000000LL));
  auto foreign=latest;foreign.valid=false;++foreign.version.anchor_revision;
  ASSERT_TRUE(A::proof({p,foreign},d,10200000000LL));
}
TEST(ExecutionLedger,ActuallyCheckedRevisionCanSatisfyNewerOwnerFloorWithoutChangingAuthority) {
  Fixture f;auto p=f.permit();p.allowed=true;p.phase="tracking";p.execution_id="exec";p.validation_sequence=41;
  w::MotionDemand d;d.version=p.version;d.trajectory_id=p.trajectory_id;d.execution_id=p.execution_id;
  d.validation_sequence=40;using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_FALSE(A::matches(d,p));EXPECT_TRUE(A::matches(d,p,41));
  EXPECT_FALSE(A::matches(d,p,39));++p.control_epoch;EXPECT_FALSE(A::matches(d,p,41));
}
