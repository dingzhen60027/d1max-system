#include <gtest/gtest.h>
#include <limits>
#include <future>
#include <thread>
#include "d1max_trajectory_tracker/command_timing_diagnostics.hpp"
#include "d1max_trajectory_tracker/control_trace_diagnostics.hpp"

using d1max_trajectory_tracker::CommandTimingDiagnostics;
using d1max_trajectory_tracker::CommandTransitionDiagnostics;
using d1max_trajectory_tracker::BoundedDiagnosticEvents;
using d1max_trajectory_tracker::ControlTraceBuffer;
using d1max_trajectory_tracker::ControlTraceIdentity;
using d1max_trajectory_tracker::ControlTraceRecord;

TEST(CommandTimingDiagnostics, NonzeroCommandsCanStillHaveLargeAccelerationAndJerk) {
  CommandTimingDiagnostics stats;
  stats.observe(1., .20, .05, true);
  stats.observe(1.05, .21, .06, true);
  stats.observe(1.10, .01, -.05, true);
  const auto w = stats.takeWindow();
  EXPECT_EQ(w.stop_edges, 0u);
  EXPECT_EQ(w.normal_intervals, 2u);
  EXPECT_NEAR(w.normal_max_forward_acceleration, 4., 1e-10);
  EXPECT_NEAR(w.normal_max_forward_jerk, 84., 1e-9);
  EXPECT_NEAR(w.normal_max_yaw_acceleration, 2.2, 1e-10);
}

TEST(CommandTimingDiagnostics, UsesActualTickIntervalsAndPreservesHistoryAcrossReports) {
  CommandTimingDiagnostics stats;
  stats.observe(1., .1, 0., true);
  stats.observe(1.04, .108, 0., true);
  stats.takeWindow();
  stats.observe(1.10, .12, 0., true);
  const auto w = stats.takeWindow();
  EXPECT_EQ(w.samples, 1u);
  EXPECT_EQ(w.intervals, 1u);
  EXPECT_NEAR(w.max_interval_s, .06, 1e-12);
  EXPECT_NEAR(w.normal_max_forward_acceleration, .2, 1e-12);
  EXPECT_NEAR(w.normal_max_forward_jerk, 0., 1e-11);
}

TEST(CommandTimingDiagnostics, ForcedStopRemainsVisibleButIsNotCalledNormalSmoothing) {
  CommandTimingDiagnostics stats;
  stats.observe(1., .20, .10, true);
  stats.observe(1.05, .20, .10, true);
  stats.observe(1.10, 0., 0., false);
  stats.observe(1.15, .001, .001, true);
  const auto w = stats.takeWindow();
  EXPECT_EQ(w.stop_edges, 1u);
  EXPECT_NEAR(w.max_forward_acceleration, 4., 1e-10);
  EXPECT_GT(w.max_forward_jerk, 79.);
  EXPECT_EQ(w.normal_intervals, 1u);
  EXPECT_EQ(w.normal_max_forward_acceleration, 0.);
  EXPECT_EQ(w.normal_max_forward_jerk, 0.);
}

TEST(CommandTimingDiagnostics, DuplicateOrReversedTimeDoesNotReplaceTheLastRealSample) {
  CommandTimingDiagnostics stats;
  stats.observe(1., .1, 0., true);
  stats.observe(1., 10., 10., true);
  stats.observe(.9, 10., 10., true);
  stats.observe(1.05, .11, 0., true);
  const auto w = stats.takeWindow();
  EXPECT_EQ(w.nonmonotonic_samples, 2u);
  EXPECT_EQ(w.intervals, 1u);
  EXPECT_NEAR(w.max_forward_acceleration, .2, 1e-12);
}

TEST(CommandTimingDiagnostics, HardCapOverrideIsCountedWithoutHidingItsNormalDerivativePeak) {
  CommandTimingDiagnostics stats;
  stats.observe(1., .2, 0., true);
  stats.observe(1.05, .1, 0., true, true);
  const auto w = stats.takeWindow();
  EXPECT_EQ(w.constraint_overrides, 1u);
  EXPECT_EQ(w.normal_intervals, 1u);
  EXPECT_NEAR(w.normal_max_forward_acceleration, 2., 1e-12);
}

namespace {
CommandTransitionDiagnostics::Sample sample(double steady,double forward,double yaw=0.) {
  CommandTransitionDiagnostics::Sample s;s.steady_s=steady;s.forward=forward;s.yaw=yaw;
  s.source_ns=100000000001LL;s.body_source_ns=99990000003LL;
  s.curve_id=12;s.command_sequence=34;s.reference_generation=5;return s;
}
}
TEST(CommandTransitionDiagnostics, NonzeroDropKeepsOriginalSourcesAndActualTickInterval) {
 CommandTransitionDiagnostics detector;const auto before=sample(1.,.3,.1);
 EXPECT_FALSE(detector.observe(before));auto after=sample(1.083,.15,.1);after.command_sequence=35;
 const auto event=detector.observe(after);ASSERT_TRUE(event);
 EXPECT_EQ(event->kind,CommandTransitionDiagnostics::SpeedDrop);EXPECT_NEAR(event->interval_s,.083,1e-12);
 EXPECT_EQ(event->previous.command_sequence,34u);EXPECT_EQ(event->current.command_sequence,35u);
 EXPECT_EQ(event->current.source_ns,100000000001LL);EXPECT_EQ(event->current.body_source_ns,99990000003LL);
 EXPECT_EQ(event->current.reference_generation,5u);EXPECT_EQ(event->current.curve_id,12);
 EXPECT_DOUBLE_EQ(after.forward,.15); // Passive observation never mutates the demand.
}
TEST(CommandTransitionDiagnostics, WalkingStopDuringTurnAndFullStopAreDifferentEdges) {
 CommandTransitionDiagnostics detector;detector.observe(sample(1.,.2,.1));
 const auto translation=detector.observe(sample(1.05,0.,.1));ASSERT_TRUE(translation);
 EXPECT_TRUE(translation->kind&CommandTransitionDiagnostics::TranslationZero);
 EXPECT_FALSE(translation->kind&CommandTransitionDiagnostics::Zero);
 const auto stop=detector.observe(sample(1.1,0.,0.));ASSERT_TRUE(stop);
 EXPECT_EQ(stop->kind,CommandTransitionDiagnostics::Zero);
 EXPECT_FALSE(detector.observe(sample(1.15,0.,0.)));EXPECT_EQ(detector.events(),2u);
}
TEST(CommandTransitionDiagnostics, OrdinaryRampAndSmallNoiseDoNotProduceFrameRateEvents) {
 CommandTransitionDiagnostics detector;detector.observe(sample(1.,.3));
 for(unsigned i=1;i<=10;++i)EXPECT_FALSE(detector.observe(sample(1.+i*.05,.3-i*.01)));
 EXPECT_EQ(detector.events(),0u);
 auto lateral=sample(1.55,.19);lateral.lateral=.05;EXPECT_FALSE(detector.observe(lateral));
}
TEST(CommandTransitionDiagnostics, SourceClockMayPauseButSteadyClockCannotRegress) {
 CommandTransitionDiagnostics detector;detector.observe(sample(1.,.3));
 EXPECT_FALSE(detector.observe(sample(.9,0.)));EXPECT_FALSE(detector.observe(sample(1.,0.)));
 auto bad=sample(1.01,0.);bad.yaw=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(detector.observe(bad));
 auto current=sample(1.05,0.);current.source_ns=1;const auto event=detector.observe(current);ASSERT_TRUE(event);
 EXPECT_NEAR(event->interval_s,.05,1e-12);EXPECT_DOUBLE_EQ(event->previous.forward,.3);
 EXPECT_EQ(event->current.source_ns,1);EXPECT_EQ(detector.nonmonotonicSamples(),2u);EXPECT_EQ(detector.invalidSamples(),1u);
}
TEST(BoundedDiagnosticEvents, CapturesValuesKeepsOldestAndCountsOverflowWithoutGrowth) {
 BoundedDiagnosticEvents<CommandTransitionDiagnostics::Sample,2> queue;
 auto value=sample(1.,.3);ASSERT_TRUE(queue.push(value));value.forward=0.;value.command_sequence=35;
 ASSERT_TRUE(queue.push(value));EXPECT_FALSE(queue.push(value));EXPECT_EQ(queue.size(),2u);EXPECT_EQ(queue.dropped(),1u);
 auto first=queue.pop();ASSERT_TRUE(first);EXPECT_DOUBLE_EQ(first->forward,.3);EXPECT_EQ(first->command_sequence,34u);
 ASSERT_TRUE(queue.push(sample(2.,.2)));EXPECT_EQ(queue.size(),2u);
 EXPECT_EQ(queue.pop()->command_sequence,35u);EXPECT_DOUBLE_EQ(queue.pop()->steady_s,2.);
 EXPECT_FALSE(queue.pop());EXPECT_EQ(queue.dropped(),1u);
}
TEST(CommandTransitionDiagnostics, EveryTickRetainsSmoothDecelerationBeforeZero) {
 CommandTransitionDiagnostics detector;
 for(unsigned i=0;i<=20;++i) {
   const auto tick=detector.observeTick(sample(1.+i*.05,.2-.01*i));ASSERT_TRUE(tick);
   EXPECT_NEAR(tick->interval_s,i?.05:0.,1e-12);
   EXPECT_NEAR(tick->current.forward,.2-.01*i,1e-12);
   if(i<20)EXPECT_EQ(tick->kind,0u);
 }
 EXPECT_EQ(detector.events(),1u);
}
namespace {
std::shared_ptr<const ControlTraceIdentity> traceIdentity() {
 auto c=std::make_shared<ControlTraceIdentity>();c->id=1;c->task="task";c->session="session";
 c->route="route";c->route_hash=std::string(64,'a');c->map="map";c->seed="seed";
 c->segment="segment";c->anchor="anchor";c->execution="execution";return c;
}
ControlTraceRecord traceRecord(unsigned sequence,bool event=false) {
 ControlTraceRecord r;r.identity=traceIdentity();r.tick.current=sample(100.+sequence*.05,.3);
 r.tick.current.source_ns=1793022800000000001LL+sequence*50000000LL;
 r.tick.current.body_source_ns=r.tick.current.source_ns-12345679;
 r.tick.current.command_sequence=sequence;r.tick.interval_s=.05;
 r.tick.previous=r.tick.current;r.tick.previous.command_sequence=sequence-1;
 r.proof_sequence=400+sequence;r.proof_source_ns=r.tick.current.source_ns-98765432;
 r.proof_valid_until_ns=r.tick.current.source_ns+100000000;
 r.front_source_ns=r.tick.current.source_ns-198765432;r.rear_source_ns=r.front_source_ns+2000;
 r.demand_valid_until_ns=r.tick.current.source_ns+80000000;
 r.body_valid_until_ns=r.tick.current.source_ns+120000000;r.tick_to_demand_publish_s=.0012;
 if(event){r.tick.kind=CommandTransitionDiagnostics::Zero;r.tick.current.forward=0.;
   r.reason="waiting_native_validation";r.phase="waiting_native_validation";
   r.proof_failure="original_proof_lease_expired";r.latest.source_ns=r.tick.current.body_source_ns;
   r.accepted=r.latest;}
 return r;
}
}
TEST(ControlTraceBuffer, FullControlSecondHasEveryTickOriginalSourcesAndTransitionIdentity) {
 ControlTraceBuffer buffer;
 for(unsigned seq=1;seq<=20;++seq)buffer.push(traceRecord(seq,seq==10));
 const auto batch=buffer.take();ASSERT_EQ(batch.size,20u);
 const auto line=ControlTraceBuffer::format(batch,1793022801000000001LL,101.);
 ASSERT_LE(line.size(),ControlTraceBuffer::MaxLineBytes);const auto j=nlohmann::json::parse(line);
 ASSERT_EQ(j["ticks"].size(),20u);EXPECT_EQ(j["omitted_ticks"],0);EXPECT_EQ(j["omitted_details"],0);
 ASSERT_EQ(j["events"].size(),1u);EXPECT_EQ(j["events"][0]["command_seq"],10);
 EXPECT_EQ(j["contexts"]["1"]["task"],"task");
 const std::int64_t base=j["base_source_ns"];
 for(unsigned i=0;i<20;++i) {
   const auto expected=traceRecord(i+1);const auto row=j["ticks"][i];
   EXPECT_EQ(base+row[4].get<std::int64_t>(),expected.tick.current.source_ns);
   EXPECT_EQ(base+row[5].get<std::int64_t>(),expected.tick.current.body_source_ns);
   EXPECT_EQ(base+row[17].get<std::int64_t>(),expected.proof_source_ns);
   EXPECT_EQ(row[7],50000000);EXPECT_EQ(row[0],i+1);
 }
}
TEST(ControlTraceBuffer, OverflowAndByteBudgetLossAreExplicitAndMemoryIsBounded) {
 ControlTraceBuffer buffer;
 for(unsigned seq=1;seq<=100;++seq)buffer.push(traceRecord(seq,true));
 const auto batch=buffer.take();EXPECT_EQ(batch.size,64u);EXPECT_EQ(batch.overflow,36u);
 const auto line=ControlTraceBuffer::format(batch,1793022805000000001LL,105.);
 EXPECT_LE(line.size(),ControlTraceBuffer::MaxLineBytes);const auto j=nlohmann::json::parse(line);
 EXPECT_EQ(j["ticks"].size()+j["omitted_ticks"].get<std::size_t>(),64u);
 EXPECT_EQ(j["events"].size()+j["omitted_details"].get<std::size_t>(),64u);
 EXPECT_EQ(j["overflow_total"],36);EXPECT_EQ(buffer.take().size,0u);
}
TEST(ControlTraceBuffer, TypicalTwentyHzWithFullIdentitiesDoesNotOmitTickOrZeroEvidence) {
 ControlTraceBuffer buffer;auto c=std::make_shared<ControlTraceIdentity>();c->id=1234;
 c->session=std::string(32,'s');c->task=std::string(64,'t');c->route=std::string(64,'r');
 c->route_hash=std::string(64,'h');c->map=std::string(64,'m');c->seed=std::string(64,'z');
 c->segment=std::string(48,'g');c->anchor=std::string(64,'a');c->execution=std::string(64,'e');
 for(unsigned seq=1;seq<=20;++seq){auto r=traceRecord(seq,seq==19);r.identity=c;
   if(seq!=19){r.tick.current.forward=.29763591765;r.tick.current.yaw=.03924596876;}
   buffer.push(r);}
 const auto j=nlohmann::json::parse(ControlTraceBuffer::format(buffer.take(),1793022801000000001LL,101.));
 EXPECT_EQ(j["ticks"].size(),20u);EXPECT_EQ(j["omitted_ticks"],0);EXPECT_EQ(j["omitted_details"],0);
 ASSERT_EQ(j["events"].size(),1u);EXPECT_EQ(j["events"][0]["command_seq"],19);
}
TEST(ControlTraceBuffer, SlowConsumerDoesNotHoldProducerAndCopiedBatchCannotChange) {
 ControlTraceBuffer buffer;buffer.push(traceRecord(1));
 std::promise<void> copied,release;auto ready=copied.get_future();auto resume=release.get_future();
 std::string line;
 std::thread consumer([&]{const auto batch=buffer.take();copied.set_value();resume.wait();
   line=ControlTraceBuffer::format(batch,1793022801000000001LL,101.);});
 ready.wait();for(unsigned seq=2;seq<=21;++seq)buffer.push(traceRecord(seq));
 const auto next=buffer.take();EXPECT_EQ(next.size,20u);release.set_value();consumer.join();
 const auto j=nlohmann::json::parse(line);ASSERT_EQ(j["ticks"].size(),1u);EXPECT_EQ(j["ticks"][0][0],1);
}
TEST(ControlTraceBuffer, ContextChangeAndMalformedLabelsDoNotCorruptJsonOrAliasTasks) {
 ControlTraceBuffer buffer;buffer.push(traceRecord(1));auto changed=traceRecord(2,true);
 auto identity=std::make_shared<ControlTraceIdentity>(*changed.identity);identity->id=2;identity->task="new-task";
 changed.identity=identity;changed.reason=std::string("partial utf8:")+char(0xe4);buffer.push(changed);
 auto oversized=traceRecord(3);auto huge=std::make_shared<ControlTraceIdentity>(*oversized.identity);
 huge->task=std::string(257,'x');oversized.identity=huge;buffer.push(oversized);
 const auto j=nlohmann::json::parse(ControlTraceBuffer::format(buffer.take(),0,0.));
 EXPECT_EQ(j["contexts"].size(),2u);EXPECT_EQ(j["contexts"]["2"]["task"],"new-task");
 EXPECT_EQ(j["ticks"].size(),2u);EXPECT_EQ(j["omitted_ticks"],1);
 EXPECT_EQ(j["events"][0]["command_seq"],2);
}
