#include <gtest/gtest.h>
#include <plan_manage/motion_check_diagnostics.hpp>
#include <plan_manage/bounded_execution_trace.hpp>
#include <future>
#include <iostream>
#include <set>

namespace scan_planner {
struct BoundedExecutionTraceTestAccess {
  static auto hold(BoundedExecutionTrace& trace){return std::unique_lock<std::mutex>(trace.mutex_);}
};
}

namespace {
using scan_planner::MotionCheckDiagnostics;
using scan_planner::MotionCheckStage;
MotionCheckDiagnostics::Demand demand() {
  MotionCheckDiagnostics::Demand d;d.version.schema_version=3;
  d.version.session_id="session";d.version.task_id="task";d.version.route_hash="original-route";
  d.execution_id="execution";d.control_epoch=2;d.sdk_session="original-sdk";
  d.sdk_arm_generation=3;d.sequence=4;d.permit_sequence=5;d.trajectory_id=6;
  d.source_stamp.sec=10;d.body_source_stamp.sec=9;d.body_source_stamp.nanosec=990000000;
  d.valid_until.sec=10;d.valid_until.nanosec=200000000;
  d.velocity.linear.x=.2;return d;
}

TEST(MotionCheckDiagnostics, FixedCountersAndEventRingStayBounded) {
  MotionCheckDiagnostics diagnostics;auto d=demand();
  for(std::uint64_t i=1;i<=10000;++i) {
    d.sequence=i;
    diagnostics.record(MotionCheckStage::PermitNotDelivered,d,10000000000LL+i);
  }
  const auto s=diagnostics.snapshot();
  EXPECT_EQ(s.counts.size(),MotionCheckDiagnostics::stage_count);
  EXPECT_EQ(s.counts[static_cast<std::size_t>(MotionCheckStage::PermitNotDelivered)],10000u);
  ASSERT_TRUE(s.last_nonzero_demand);EXPECT_EQ(s.last_nonzero_demand->sequence,10000u);
  EXPECT_EQ(s.last_nonzero_demand->source_stamp,d.source_stamp);
  EXPECT_EQ(s.last_nonzero_demand->valid_until,d.valid_until);
  EXPECT_EQ(s.events.size(),64u);EXPECT_EQ(s.event_sequence,10000u);
  for(std::uint64_t id=10000-63;id<=10000;++id) {
    const auto& event=s.events[(id-1)%MotionCheckDiagnostics::event_capacity];
    EXPECT_EQ(event.event_id,id);EXPECT_EQ(event.command_sequence,id);
  }
}

TEST(MotionCheckDiagnostics, LaterIdleKeepsExplicitlyHistoricalOriginalNonzeroRequest) {
  MotionCheckDiagnostics diagnostics;const auto original=demand();
  diagnostics.record(MotionCheckStage::BodyBehindDemand,original,10001000000LL,8,9);
  auto zero=original;zero.sequence+=1;zero.hold=true;zero.velocity.linear.x=0.;
  diagnostics.record(MotionCheckStage::Idle,zero,10002000000LL);
  const auto s=diagnostics.snapshot();
  EXPECT_EQ(s.stage,MotionCheckStage::Idle);EXPECT_EQ(s.observed_at_ns,10002000000LL);
  EXPECT_EQ(s.last_nonzero_stage,MotionCheckStage::BodyBehindDemand);
  EXPECT_EQ(s.last_nonzero_observed_at_ns,10001000000LL);
  ASSERT_TRUE(s.last_nonzero_demand);EXPECT_EQ(*s.last_nonzero_demand,original);
  EXPECT_EQ(s.proof_sequence,8u);EXPECT_EQ(s.map_snapshot_revision,9u);
}

TEST(MotionCheckDiagnostics, SnapshotIsIndependentAndWaitNeverMutatesOriginalAuthority) {
  MotionCheckDiagnostics diagnostics;auto d=demand();const auto original=d;
  diagnostics.record(MotionCheckStage::SweepBudgetExhausted,d,10010000000LL);
  auto snapshot=diagnostics.snapshot();ASSERT_TRUE(snapshot.last_nonzero_demand);
  snapshot.last_nonzero_demand->valid_until.sec=99;
  snapshot.counts.fill(99);
  EXPECT_EQ(d,original);
  const auto retained=diagnostics.snapshot();ASSERT_TRUE(retained.last_nonzero_demand);
  EXPECT_EQ(retained.last_nonzero_demand->valid_until,original.valid_until);
  EXPECT_FALSE(retained.last_nonzero_demand->safety_checked);
  EXPECT_EQ(retained.counts[static_cast<std::size_t>(MotionCheckStage::SweepBudgetExhausted)],1u);
}

TEST(MotionCheckDiagnostics, EnumLabelsAreCompleteAndUnknownValuesDoNotAllocateCounters) {
  MotionCheckDiagnostics diagnostics;
  for(std::size_t i=0;i<MotionCheckDiagnostics::stage_count;++i) {
    const auto stage=static_cast<MotionCheckStage>(i);
    EXPECT_STRNE(MotionCheckDiagnostics::name(stage),"unknown_diagnostic_stage");
    diagnostics.record(stage,{},10000000000LL);
  }
  diagnostics.record(MotionCheckStage::Count,{},10000000001LL);
  for(const auto count:diagnostics.snapshot().counts)EXPECT_EQ(count,1u);
}
TEST(MotionCheckDiagnostics, RejectionEventsKeepSourceClocksAndSuccessDoesNotFillTheRing) {
  MotionCheckDiagnostics diagnostics;auto d=demand();const auto original=d;
  diagnostics.record(MotionCheckStage::BodyBehindDemand,d,10030000000LL,8,9,5,9970000000LL);
  const auto first=diagnostics.snapshot();
  ASSERT_EQ(first.event_sequence,1u);const auto& event=first.events[0];
  EXPECT_EQ(event.command_sequence,4u);EXPECT_EQ(event.curve,6);EXPECT_EQ(event.active_curve,5);
  EXPECT_EQ(event.command_source_ns,10000000000LL);EXPECT_EQ(event.body_source_ns,9990000000LL);
  EXPECT_EQ(event.native_body_source_ns,9970000000LL);EXPECT_EQ(event.observed_at_ns,10030000000LL);
  EXPECT_EQ(event.command_valid_until_ns,10200000000LL);EXPECT_EQ(event.proof_sequence,8u);
  for(unsigned i=0;i<1000;++i)diagnostics.record(MotionCheckStage::LocalCommandRelayed,d,10040000000LL);
  EXPECT_EQ(d,original);EXPECT_EQ(diagnostics.snapshot().event_sequence,1u);
  d.velocity={};d.hold=true;++d.sequence;
  diagnostics.record(MotionCheckStage::DemandAuthorityMismatch,d,10050000000LL);
  const auto second=diagnostics.snapshot();ASSERT_EQ(second.event_sequence,2u);
  EXPECT_TRUE(second.events[1].hold);EXPECT_EQ(second.events[1].command_sequence,5u);
  EXPECT_EQ(second.last_nonzero_demand->sequence,4u);
}

using Trace=scan_planner::BoundedExecutionTrace;
TEST(BoundedExecutionTrace, OptionalWireAdaptationExceptionsCannotInterruptObservedActions) {
  Trace trace;unsigned published=0,admission_notifications=0;
  EXPECT_NO_THROW(trace.attempt([] {throw std::bad_alloc();}));
  ++published;
  EXPECT_NO_THROW(trace.attempt([] {throw std::runtime_error("diagnostic clock unavailable");}));
  ++admission_notifications;
  EXPECT_EQ(published,1u);EXPECT_EQ(admission_notifications,1u);
  EXPECT_EQ(trace.snapshot().producer_failures,2u);
  EXPECT_TRUE(trace.record({},{}));
  EXPECT_EQ(trace.snapshot().last_event,1u);
}
Trace::Version traceVersion() {
  Trace::Version v;v.schema_version=3;v.session_id="session";v.task_id="task";
  v.route_id="route";v.route_hash="hash";v.map_version_id="map";v.localization_seed_id="seed";
  v.localization_epoch=3;v.reference_generation=7;v.segment_id="stair_up";v.anchor_id="anchor";
  v.anchor_revision=8;v.context_sequence=9;v.map_geometry_revision=10;return v;
}
TEST(BoundedExecutionTrace, TaskAndRetirementBoundariesStaySeparateWithoutNewProducerQueues) {
  Trace trace;auto first=traceVersion(),second=first;second.task_id="next-task";
  Trace::Event e;e.ros_ns=1000000000LL;e.retention.set(&first,true);
  ASSERT_TRUE(trace.record(e,{{&first,1,{}},{},{},{},{}}));
  ASSERT_TRUE(trace.record(e,{{&first,2,{}},{},{},{},{}}));
  e.retention.set(&first,false);
  ASSERT_TRUE(trace.record(e,{{&first,2,{}},{},{},{},{}}));
  e.retention.set(&second,true);
  ASSERT_TRUE(trace.record(e,{{&second,3,{}},{},{},{},{}}));
  std::uint64_t cursor=0;Trace::Retention retention;
  auto batch=nlohmann::json::parse(trace.drain(cursor,Trace::max_batch_bytes,&retention));
  EXPECT_EQ(cursor,2u);EXPECT_EQ(batch["events"].size(),2u);EXPECT_TRUE(retention.active);
  EXPECT_EQ(retention.task.str(),"task");EXPECT_GT(retention.steady_ns,0);
  batch=nlohmann::json::parse(trace.drain(cursor,Trace::max_batch_bytes,&retention));
  EXPECT_EQ(cursor,3u);EXPECT_FALSE(retention.active);EXPECT_EQ(retention.task.str(),"task");
  EXPECT_FALSE(batch["retention"]["active"].get<bool>());
  batch=nlohmann::json::parse(trace.drain(cursor,Trace::max_batch_bytes,&retention));
  EXPECT_EQ(cursor,4u);EXPECT_TRUE(retention.active);EXPECT_EQ(retention.task.str(),"next-task");
  EXPECT_EQ(batch["retention"]["task"],"next-task");
  EXPECT_TRUE(trace.drain(cursor).empty());
}
TEST(BoundedExecutionTrace, TruncatedRetentionIdentityIsExplicitAndNeverClaimsComplete) {
  Trace trace;auto v=traceVersion();v.task_id=std::string(159,'x')+"楼";
  Trace::Event e;e.retention.set(&v,true);ASSERT_TRUE(trace.record(e,{}));
  std::uint64_t cursor=0;Trace::Retention retention;
  const auto batch=nlohmann::json::parse(trace.drain(cursor,Trace::max_batch_bytes,&retention));
  EXPECT_EQ(cursor,1u);EXPECT_FALSE(retention.task.complete());
  EXPECT_FALSE(batch["retention"]["complete"].get<bool>());
}
TEST(BoundedExecutionTrace, CompleteVersionsAndAllSlotIdentitiesSurviveDrain) {
  Trace trace;auto v=traceVersion(),old=v;old.anchor_id="old_anchor";old.reference_generation=6;
  Trace::Event e;e.kind=Trace::Kind::CommitDecision;e.ros_ns=10000000001LL;
  e.source_ns=9990000000LL;e.body_ns=9980000000LL;e.valid_until_ns=10100000000LL;
  e.sequence=15;e.proof=22;e.reason.set("local_commit_measured_join_rejected");
  ASSERT_TRUE(trace.record(e,{{&v,4,"new"},{&old,2,"old"},{&old,2,"old"},
    {&v,4,"new"},{&v,6,"latest"}}));
  std::uint64_t cursor=0;const auto encoded=trace.drain(cursor);ASSERT_LE(encoded.size(),Trace::max_batch_bytes);
  const auto j=nlohmann::json::parse(encoded);EXPECT_EQ(cursor,1u);
  const auto ids=j["events"][0][4];EXPECT_EQ(ids[1],ids[2]);EXPECT_EQ(ids[0],ids[3]);
  EXPECT_NE(ids[0],ids[4]);EXPECT_NE(ids[0],ids[1]);
  const auto identity=j["identities"][std::to_string(ids[0].get<std::uint64_t>())];
  EXPECT_TRUE(identity["complete"].get<bool>());EXPECT_EQ(identity["strings"][1],"anchor");
  EXPECT_EQ(identity["strings"][2],"new");EXPECT_EQ(identity["numbers"],nlohmann::json::array({7,8,9,10}));
  const auto task=j["tasks"][identity["task"].get<std::string>()];
  EXPECT_EQ(task["localization_epoch"],3);EXPECT_EQ(task["strings"][0],"session");
  const auto base=j["clock_base_ns"][0].get<std::int64_t>();
  EXPECT_EQ(j["events"][0][6].get<std::int64_t>()+base,e.source_ns);
  EXPECT_EQ(j["events"][0][7].get<std::int64_t>()+base,e.body_ns);
  EXPECT_EQ(j["events"][0][8].get<std::int64_t>()+base,e.valid_until_ns);
  EXPECT_GT(j["clock_base_ns"][1].get<std::int64_t>(),0);
  EXPECT_EQ(j["events"][0][1],static_cast<unsigned>(Trace::Kind::CommitDecision));
  EXPECT_TRUE(trace.drain(cursor).empty());
}
TEST(BoundedExecutionTrace, EverySuccessfulSequenceAndZeroRemainVisibleWithoutRestamping) {
  Trace trace;auto v=traceVersion();
  for(unsigned n=1;n<=40;++n) {
    Trace::Event e;e.kind=Trace::Kind::CommandPublishReturned;e.sequence=n;
    e.ros_ns=10000000000LL+n*50000000LL;e.source_ns=e.ros_ns-10000000;
    e.body_ns=e.source_ns-2000000;e.valid_until_ns=e.source_ns+100000000;
    e.velocity={{n==20?0.:.3,0.,0.}};e.hold=n==20;
    ASSERT_TRUE(trace.record(e,{{&v,2,""},{},{&v,2,"p"},{},{}}));
  }
  std::uint64_t cursor=0,seen=0;bool zero=false;
  while(cursor<40) {
    const auto batch=trace.drain(cursor);ASSERT_FALSE(batch.empty());ASSERT_LE(batch.size(),Trace::max_batch_bytes);
    const auto j=nlohmann::json::parse(batch);EXPECT_EQ(j["overwritten"],0);
    for(const auto& e:j["events"]) {
      EXPECT_EQ(e[5],++seen);EXPECT_EQ(e[8].get<std::int64_t>()-e[6].get<std::int64_t>(),100000000);
      if(e[5]==20){EXPECT_EQ(e[18].get<unsigned>(),1u);EXPECT_EQ(e[17][0],0.);zero=true;}
    }
  }
  EXPECT_EQ(seen,40u);EXPECT_TRUE(zero);
}
TEST(BoundedExecutionTrace, OverloadAndIdentityEvictionAreExplicitAndStorageRemainsFixed) {
  Trace trace;auto v=traceVersion();Trace::Event e;
  for(unsigned n=1;n<=1000;++n){v.reference_generation=n;e.sequence=n;
    ASSERT_TRUE(trace.record(e,{{&v,n,"p"},{},{},{},{}}));}
  const auto s=trace.snapshot();EXPECT_EQ(s.events.size(),128u);EXPECT_EQ(s.identities.size(),64u);
  std::uint64_t cursor=0;const auto first=nlohmann::json::parse(trace.drain(cursor));
  EXPECT_EQ(first["overwritten"],872u);EXPECT_EQ(first["identity_evictions"],936u);
  EXPECT_FALSE(first["missing_identities"].empty());
  while(cursor<1000){const auto last=cursor;const auto batch=trace.drain(cursor);
    ASSERT_LE(batch.size(),Trace::max_batch_bytes);ASSERT_GT(cursor,last);}
}
TEST(BoundedExecutionTrace, OversizedUtf8IdentityCannotAliasOrBlockBoundedDrain) {
  Trace trace;auto a=traceVersion(),b=a;
  a.task_id=std::string(159,'x')+"楼"+std::string(200,'a');b.task_id=a.task_id+"b";
  Trace::Event e;e.reason.set(std::string(95,'x')+"楼");
  ASSERT_TRUE(trace.record(e,{{&a,1,""},{},{},{},{}}));
  ASSERT_TRUE(trace.record(e,{{&b,1,""},{},{},{},{}}));
  const auto s=trace.snapshot();EXPECT_NE(s.events[0].identity[0],s.events[1].identity[0]);
  std::uint64_t cursor=0;const auto text=trace.drain(cursor);ASSERT_LE(text.size(),Trace::max_batch_bytes);
  const auto j=nlohmann::json::parse(text);EXPECT_EQ(cursor,2u);
  for(const auto& i:j["identities"])EXPECT_FALSE(i["complete"].get<bool>());
  EXPECT_EQ(j["events"][0][21],98u);
}
TEST(BoundedExecutionTrace, ContentionDropsDiagnosticsInsteadOfWaitingForReader) {
  Trace trace;auto lock=scan_planner::BoundedExecutionTraceTestAccess::hold(trace);
  auto work=std::async(std::launch::async,[&]{Trace::Event e;return trace.record(e,{});});
  const auto ready=work.wait_for(std::chrono::milliseconds(50))==std::future_status::ready;
  lock.unlock();ASSERT_TRUE(ready);EXPECT_FALSE(work.get());
  EXPECT_EQ(trace.snapshot().contention_drops,1u);
}
TEST(BoundedExecutionTrace, CheckDetailUsesOriginalClocksAndKeepsLegacyArrayLayout) {
  Trace trace;auto version=traceVersion();Trace::Event e;
  e.kind=Trace::Kind::CurrentCheckFinished;e.retention.set(&version,true);
  e.ros_ns=10000000200LL;e.source_ns=9600000000LL;e.rear_source_ns=9700000000LL;
  e.body_ns=9990000000LL;e.valid_until_ns=10200000000LL;e.proof=31;e.validation_floor=30;
  e.accepted=true;e.reason.set("occupied");e.check.valid=false;
  e.check.disposition.set("proof_publish_returned");e.check.check_begin_ns=10000000000LL;
  e.check.check_end_ns=10000000100LL;e.check.publication_sequence=7;e.check.fusion_sequence=6;
  ASSERT_TRUE(trace.record(e,{{&version,12,"proposal"},{},{},{},{}}));
  std::uint64_t cursor=0;const auto batch=nlohmann::json::parse(trace.drain(cursor));
  const auto& event=batch["events"][0];ASSERT_EQ(event.size(),27u);
  EXPECT_EQ(event[6].get<std::int64_t>()+batch["clock_base_ns"][0].get<std::int64_t>(),e.source_ns);
  EXPECT_EQ(event[22].get<std::int64_t>()+batch["clock_base_ns"][0].get<std::int64_t>(),e.rear_source_ns);
  EXPECT_EQ(event[10],31u);EXPECT_EQ(event[11],30u);EXPECT_EQ(event[19],1);
  EXPECT_FALSE(event[26]["valid"].get<bool>());
  EXPECT_EQ(event[26]["publication_sequence"],7u);
  EXPECT_EQ(event[26]["check_ros_ns"][0].get<std::int64_t>()+e.ros_ns,e.check.check_begin_ns);
  EXPECT_EQ(batch["check_event_schema"]["detail_index"],26);
  e.kind=Trace::Kind::CommandPublishReturned;ASSERT_TRUE(trace.record(e,{}));
  EXPECT_EQ(nlohmann::json::parse(trace.drain(cursor))["events"][0].size(),26u);
}
TEST(BoundedExecutionTrace, SeparateCheckRingCannotEvictCommandOrProofHistory) {
  Trace commands,checks;auto v=traceVersion();Trace::Event e;e.retention.set(&v,true);
  for(unsigned i=1;i<=40;++i) {
    e.kind=i%2?Trace::Kind::CommandPublishReturned:Trace::Kind::CurveProof;e.sequence=i;
    ASSERT_TRUE(commands.record(e,{{&v,2,"proposal"},{},{},{},{}}));
    e.kind=Trace::Kind::CurrentCheckFinished;
    for(unsigned check=0;check<20;++check)ASSERT_TRUE(checks.record(e,{{&v,2,"proposal"},{},{},{},{}}));
  }
  std::uint64_t cursor=0;unsigned retained=0;
  commands.drainPages(cursor,[&](const auto& batch,const auto&) {
    const auto j=nlohmann::json::parse(batch);EXPECT_EQ(j["overwritten"],0);retained+=j["events"].size();return true;
  });
  EXPECT_EQ(retained,40u);EXPECT_EQ(cursor,40u);
  EXPECT_EQ(checks.snapshot().last_event,800u);EXPECT_EQ(commands.snapshot().last_event,40u);
}
TEST(BoundedExecutionTrace, BoundedMultiPageDrainKeepsUpAndFailedSinkRetainsCursor) {
  Trace trace;auto v=traceVersion();Trace::Event e;e.retention.set(&v,true);
  e.kind=Trace::Kind::CurrentCheckFinished;e.check.disposition.set("proof_publish_returned");
  e.reason.set("occupied");e.check.valid=false;e.accepted=true;
  std::uint64_t cursor=0,events=0,bytes=0;
  // 100 completion events/s is above the intended current+admission rates.
  // Each half-second contains fresh long identities and full timing details.
  for(unsigned half_second=0;half_second<20;++half_second) {
    for(unsigned i=0;i<50;++i) {
      e.ros_ns=10000000000LL+half_second*500000000LL+i*10000000LL;
      e.check.check_begin_ns=e.ros_ns-1000000;e.check.check_end_ns=e.ros_ns;
      e.check.fusion_begin_ns=e.ros_ns-200000000;e.check.fusion_end_ns=e.ros_ns-100000000;
      e.check.publication_sequence=i+1;e.source_ns=e.rear_source_ns=e.ros_ns-300000000;
      e.body_ns=e.ros_ns-10000000;e.proof=i+1;
      v.route_hash=std::string(64,'r');v.task_id=std::string(32,'t');
      ASSERT_TRUE(trace.record(e,{{&v,2,"proposal"},{},{},{},{}}));
    }
    const auto saved=cursor;
    EXPECT_EQ(trace.drainPages(cursor,[](const auto&,const auto&){return false;}),0u);
    EXPECT_EQ(cursor,saved);
    const auto pages=trace.drainPages(cursor,[&](const auto& batch,const auto&) {
      EXPECT_LE(batch.size(),Trace::max_batch_bytes);bytes+=batch.size();
      const auto j=nlohmann::json::parse(batch);EXPECT_EQ(j["overwritten"],0);
      EXPECT_TRUE(j["missing_identities"].empty());events+=j["events"].size();return true;
    });
    EXPECT_LE(pages,8u);EXPECT_EQ(cursor,trace.snapshot().last_event);
  }
  EXPECT_EQ(events,1000u);
  std::cout<<"CHECK_TRACE_LOAD events="<<events<<" bytes="<<bytes<<" trace_size="<<sizeof(Trace)
    <<" event_size="<<sizeof(Trace::Event)<<" max_batch_bytes="<<Trace::max_batch_bytes<<"\n";
}
TEST(BoundedExecutionTrace, TenSecondTwentyHzCommandsAndTenHzHandoffsFitBoundedDrain) {
  const unsigned ticks=std::getenv("D1MAX_TRACE_SUSTAINED_60S")?1200u:200u;
  Trace trace;auto v=traceVersion();v.session_id=std::string(32,'s');v.task_id=std::string(34,'t');
  v.route_id=std::string(32,'r');v.route_hash=std::string(64,'h');v.map_version_id=std::string(64,'m');
  v.anchor_id=std::string(32,'a');v.localization_seed_id=std::string(32,'i');
  std::uint64_t cursor=0,bytes=0,overwritten=0,batches=0;std::set<std::uint64_t> commands;
  std::vector<std::int64_t> capture_times,drain_times;std::int64_t curve=1;
  auto append=[&](Trace::Event e,const Trace::Context& c) {
    const auto begin=Trace::steadyNowNs();e.retention.set(&v,true);EXPECT_TRUE(trace.record(e,c));
    capture_times.push_back(Trace::steadyNowNs()-begin);
  };
  const auto drain=[&] {
    const auto begin=Trace::steadyNowNs();const auto batch=trace.drain(cursor);
    drain_times.push_back(Trace::steadyNowNs()-begin);if(batch.empty())return;
    EXPECT_LE(batch.size(),Trace::max_batch_bytes);bytes+=batch.size();++batches;
    const auto j=nlohmann::json::parse(batch);overwritten+=j["overwritten"].get<std::uint64_t>();
    for(const auto& e:j["events"])if(e[1]==static_cast<unsigned>(Trace::Kind::CommandPublishReturned))
      commands.insert(e[5].get<std::uint64_t>());
  };
  for(unsigned n=1;n<=ticks;++n) {
    const auto now=1793022645000000000LL+n*50000000LL;
    if(n%2==0) {
      auto old=v;++v.reference_generation;++curve;
      const Trace::Context context{{&v,curve,"proposal"},{&old,curve-1,"old"},
        {&old,curve-1,"old"},{&v,curve,"proposal"},{&v,curve,"proposal"}};
      for(const auto kind:{Trace::Kind::CandidatePublishReturned,
           Trace::Kind::CommitDecision,Trace::Kind::CommitDecision}) {
        Trace::Event e;e.kind=kind;e.ros_ns=now;e.source_ns=now-30000000;e.body_ns=now-10000000;
        e.proof_until_ns=e.valid_until_ns=now+50000000;e.lease_until_ns=now+150000000;
        e.sequence=n;e.proof=n;e.map_revision=n/4;e.accepted=true;e.reason.set("local_geometry_synchronized");
        e.begin_steady_ns=Trace::steadyNowNs();e.before_identity=trace.capture(context);append(e,context);
      }
    }
    Trace::Event e;e.kind=Trace::Kind::CommandPublishReturned;e.sequence=n;e.ros_ns=now;
    e.source_ns=now-1000000;e.body_ns=now-5000000;e.valid_until_ns=now+50000000;
    e.native_body_ns=e.body_ns;e.proof=n;e.proof_until_ns=e.valid_until_ns;
    e.lease_sequence=n;e.lease_until_ns=now+150000000;e.velocity={{.3,0.,0.}};
    e.accepted=true;e.reason.set("tracking");e.begin_steady_ns=Trace::steadyNowNs();
    const Trace::Context context{{&v,curve,"proposal"},{},{&v,curve,"proposal"},{},{}};
    append(e,context);
    if(n%2==0){auto p=e;p.kind=Trace::Kind::CurveProof;p.reason.set("candidate_clear");append(p,context);}
    if(n%4==0){e.kind=Trace::Kind::CurveProof;e.reason.set("current_new_sources_clear");append(e,context);}
    if(n%10==0)drain();
  }
  const auto during_bytes=bytes,during_batches=batches,during_commands=commands.size();
  while(cursor<trace.snapshot().last_event)drain();
  std::sort(capture_times.begin(),capture_times.end());
  std::sort(drain_times.begin(),drain_times.end());
  std::cout<<"TRACE_LOAD seconds="<<ticks/20<<" command_hz=20 handoff_hz=10 candidate_proof_hz=10 current_source_pair_hz=5 events="<<trace.snapshot().last_event
    <<" bytes_during="<<during_bytes<<" batches_during="<<during_batches
    <<" commands_during="<<during_commands<<" bytes_total="<<bytes<<" tail_batches="<<(batches-during_batches)
    <<" commands_total="<<commands.size()<<" overwritten="<<overwritten
    <<" contention_drops="<<trace.snapshot().contention_drops
    <<" capture_p50_ns="<<capture_times[capture_times.size()/2]
    <<" capture_p99_ns="<<capture_times[capture_times.size()*99/100]
    <<" capture_max_ns="<<capture_times.back()
    <<" drain_p50_ns="<<drain_times[drain_times.size()/2]
    <<" drain_p99_ns="<<drain_times[drain_times.size()*99/100]
    <<" drain_max_ns="<<drain_times.back()<<'\n';
  EXPECT_LE(during_bytes,(ticks/10)*Trace::max_batch_bytes);EXPECT_EQ(overwritten,0u);EXPECT_EQ(commands.size(),ticks);
  // The last scheduled drain should keep up, not merely postpone an overflow.
  EXPECT_LE(ticks-during_commands,10u);
}
TEST(BoundedExecutionTrace, TenThousandProducerSamplesRemainBoundedAndDoNotAllocateQueues) {
  Trace trace;auto v=traceVersion();Trace::Event e;e.kind=Trace::Kind::CommandPublishReturned;
  e.reason.set("tracking");std::vector<std::int64_t> elapsed;elapsed.reserve(10000);
  std::int64_t sum=0;
  for(unsigned n=0;n<10000;++n) {
    if(n%5==0)++v.reference_generation;
    e.sequence=n;
    const Trace::Context context{{&v,1,"proposal"},{},{&v,1,"proposal"},{},{}};
    const auto begin=Trace::steadyNowNs();e.retention.set(&v,true);ASSERT_TRUE(trace.record(e,context));
    const auto dt=Trace::steadyNowNs()-begin;elapsed.push_back(dt);sum+=dt;
  }
  std::sort(elapsed.begin(),elapsed.end());
  std::cout<<"TRACE_PRODUCER samples=10000 mean_ns="<<sum/10000
    <<" p99_ns="<<elapsed[9900]<<" max_ns="<<elapsed.back()
    <<" fixed_bytes="<<sizeof(Trace)<<" contention_drops="<<trace.snapshot().contention_drops<<'\n';
  EXPECT_EQ(trace.snapshot().last_event,10000u);EXPECT_EQ(trace.snapshot().events.size(),128u);
}
} // namespace
