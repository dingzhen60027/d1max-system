#include <gtest/gtest.h>
#include "localization_pipeline_trace.hpp"
#include <thread>
using Trace=faster_lio::LocalizationPipelineTrace;
static Trace::Event event(int64_t i,Trace::Stage stage=Trace::ImuReceived) {
  return {1793022772000000000LL+i,1793022772000000999LL+i,
    10000000000LL+i,1793022771000000000LL+i,7,42,stage};
}
TEST(LocalizationPipelineTrace, NoOutputWithoutAnomaly) {
  Trace t;for(int i=0;i<50;++i)t.record(event(i));EXPECT_FALSE(t.take());
}
TEST(LocalizationPipelineTrace, BoundedChronologicalRingPreservesOriginalTimes) {
  Trace t;for(int i=0;i<3000;++i)t.record(event(i));t.request(event(3000,Trace::SourceExpired));
  auto s=t.take();ASSERT_TRUE(s);EXPECT_EQ(s->count,Trace::kCapacity);
  EXPECT_EQ(s->overwritten,3001-Trace::kCapacity);
  for(std::size_t i=0;i<s->count;++i){
    const auto expected=event(3001-Trace::kCapacity+i);
    EXPECT_EQ(s->events[i].source_ns,expected.source_ns);
    EXPECT_EQ(s->events[i].ros_ns,expected.ros_ns);
    EXPECT_EQ(s->events[i].steady_ns,expected.steady_ns);
    EXPECT_EQ(s->events[i].auxiliary_ns,expected.auxiliary_ns);
  }
  EXPECT_EQ(s->trigger.stage,Trace::SourceExpired);EXPECT_FALSE(t.take());
}
TEST(LocalizationPipelineTrace, RepeatedRequestsDoNotAllocateOrFloodExport) {
  Trace t;t.request(event(0,Trace::SlowPhase));t.request(event(1,Trace::SourceExpired));
  auto s=t.take();ASSERT_TRUE(s);EXPECT_EQ(s->suppressed_requests,1);
  t.request(event(1000000000,Trace::SlowPhase));EXPECT_FALSE(t.take());
  t.request(event(2000000000,Trace::SlowPhase));ASSERT_TRUE(t.take());
}
TEST(LocalizationPipelineTrace, ReporterStopsWithoutAnEvent) {
  Trace t;bool returned=false;
  std::thread reader([&]{returned=!t.wait();});t.stop();reader.join();EXPECT_TRUE(returned);
}
TEST(LocalizationPipelineTrace, ReporterWakeupDoesNotModifyInput) {
  Trace t;bool returned=false;
  std::thread reader([&]{returned=t.wait();});t.request(event(1));reader.join();
  EXPECT_TRUE(returned);auto s=t.take();ASSERT_TRUE(s);EXPECT_EQ(s->trigger.epoch,7);
}
TEST(LocalizationPipelineTrace, ConcurrentProducersRemainBounded) {
  Trace t;
  std::thread a([&]{for(int i=0;i<20000;++i)t.record(event(i));});
  std::thread b([&]{for(int i=0;i<20000;++i)t.record(event(i,Trace::ScanReceived));});
  a.join();b.join();t.request(event(50000));auto s=t.take();ASSERT_TRUE(s);
  EXPECT_LE(s->count,Trace::kCapacity);
  EXPECT_EQ(s->count+s->overwritten+s->contention_drops,40001);
}
TEST(LocalizationPipelineTrace, SummarySeparatesSourceGapReceiptGapAndPhaseTime) {
  Trace t;auto a=event(0),b=event(5000000);
  b.steady_ns=a.steady_ns+400000000; // Continuous IMU source, stalled callback.
  t.record(a);t.record(b);
  auto c=event(600000000,Trace::MapEnd);c.auxiliary_ns=150000000;t.record(c);
  auto d=event(700000000,Trace::SyncWaitingImu);d.auxiliary_ns=d.source_ns-80000000;t.record(d);
  t.request(event(800000000,Trace::SlowPhase));auto snapshot=t.take();ASSERT_TRUE(snapshot);
  auto s=Trace::summarize(*snapshot);
  EXPECT_EQ(s[Trace::ImuReceived].source_gap,5000000);
  EXPECT_EQ(s[Trace::ImuReceived].receipt_gap,400000000);
  EXPECT_EQ(s[Trace::ImuReceived].receipt_left.source_ns,a.source_ns);
  EXPECT_EQ(s[Trace::ImuReceived].receipt_right.source_ns,b.source_ns);
  EXPECT_EQ(s[Trace::MapEnd].max_duration,150000000);
  EXPECT_EQ(s[Trace::SyncWaitingImu].max_coverage_lag,80000000);
}
TEST(LocalizationPipelineTrace, EpochChangeDoesNotBecomeSensorGap) {
  Trace t;auto a=event(0),b=event(1000000000);b.epoch=8;
  t.record(a);t.record(b);t.request(event(1000000001));auto snapshot=t.take();ASSERT_TRUE(snapshot);
  auto s=Trace::summarize(*snapshot);EXPECT_EQ(s[Trace::ImuReceived].source_gap,0);
  EXPECT_EQ(s[Trace::ImuReceived].receipt_gap,0);
}
