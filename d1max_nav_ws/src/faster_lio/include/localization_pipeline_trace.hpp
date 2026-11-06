#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

namespace faster_lio {
// Diagnostic-only, fixed memory. Producers NEVER wait for the exporter and
// never allocate/format/write files. Dropped diagnostic events are explicit.
// No event is a substitute for an IMU measurement or localization sample.
class LocalizationPipelineTrace {
 public:
  enum Stage : uint16_t {
    ScanReceived=1, ImuReceived, ScanTaken, PreprocessBegin, PreprocessEnd,
    SyncWaitingImu, SyncAccepted, SyncDiscarded, ImuProcessBegin, ImuProcessEnd,
    SolveBegin, SolveEnd, PosteriorBegin, PosteriorEnd, MapBegin, MapEnd,
    LegacyOdomBegin, LegacyOdomEnd, BodyCloudBegin, BodyCloudEnd, SourceExpired,
    PosteriorSourceGap, SlowPhase, InvalidNotice, ImuStored, ScanStored
  };
  static constexpr std::size_t kCapacity=2048;
  static constexpr int64_t kExportIntervalNs=2000000000LL;
  struct Event {
    int64_t source_ns=0, ros_ns=0, steady_ns=0, auxiliary_ns=0;
    uint64_t epoch=0;
    uint32_t value=0;
    Stage stage=ScanReceived;
  };
  struct Snapshot {
    std::array<Event,kCapacity> events{};
    std::size_t count=0;
    uint64_t overwritten=0, contention_drops=0, suppressed_requests=0;
    Event trigger{};
  };
  struct StageSummary {
    uint64_t count=0;
    int64_t source_gap=0,receipt_gap=0,max_duration=0,max_coverage_lag=0;
    Event source_left{},source_right{},receipt_left{},receipt_right{},previous{};
  };
  static std::array<StageSummary,27> summarize(const Snapshot& snapshot) noexcept {
    std::array<StageSummary,27> out{};
    for(std::size_t i=0;i<snapshot.count;++i){const auto&e=snapshot.events[i];
      if(e.stage>=out.size())continue;
      auto&s=out[e.stage];
      if(s.count && e.epoch==s.previous.epoch){
        const auto source_gap=e.source_ns-s.previous.source_ns;
        const auto receipt_gap=e.steady_ns-s.previous.steady_ns;
        if(source_gap>s.source_gap){s.source_gap=source_gap;s.source_left=s.previous;s.source_right=e;}
        if(receipt_gap>s.receipt_gap){s.receipt_gap=receipt_gap;s.receipt_left=s.previous;s.receipt_right=e;}
      }
      if(e.stage==PreprocessEnd||e.stage==ImuProcessEnd||e.stage==SolveEnd||
         e.stage==PosteriorEnd||e.stage==MapEnd||e.stage==LegacyOdomEnd||e.stage==BodyCloudEnd)
        s.max_duration=std::max(s.max_duration,e.auxiliary_ns);
      if(e.stage==SyncWaitingImu && e.auxiliary_ns>0)
        s.max_coverage_lag=std::max(s.max_coverage_lag,e.source_ns-e.auxiliary_ns);
      ++s.count;s.previous=e;
    }
    return out;
  }
  static int64_t SteadyNs() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void record(const Event& event) noexcept {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock()){++contention_drops_;return;}
    append(event);
  }
  void request(const Event& event) noexcept {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock()){++contention_drops_;return;}
    if(pending_ || (last_requested_ && event.steady_ns-last_requested_<kExportIntervalNs)) {
      ++suppressed_requests_;return;
    }
    append(event);trigger_=event;last_requested_=event.steady_ns;pending_=true;
    ready_.notify_one();
  }
  // Called only by the independent diagnostic exporter (or deterministic test).
  std::optional<Snapshot> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!pending_)return std::nullopt;
    Snapshot out;out.count=size_;out.trigger=trigger_;
    out.overwritten=overwritten_;out.contention_drops=contention_drops_.load();
    out.suppressed_requests=suppressed_requests_;
    const std::size_t begin=(next_+kCapacity-size_)%kCapacity;
    for(std::size_t i=0;i<size_;++i)out.events[i]=events_[(begin+i)%kCapacity];
    pending_=false;return out;
  }
  bool wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock,[this]{return pending_||stopped_;});
    return !stopped_;
  }
  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);stopped_=true;ready_.notify_all();
  }
 private:
  void append(const Event& event) noexcept {
    events_[next_]=event;next_=(next_+1)%kCapacity;
    if(size_<kCapacity)++size_;else ++overwritten_;
  }
  std::array<Event,kCapacity> events_{};
  std::size_t next_=0,size_=0;
  uint64_t overwritten_=0,suppressed_requests_=0;
  std::atomic<uint64_t> contention_drops_{0};
  Event trigger_{};
  int64_t last_requested_=0;
  bool pending_=false,stopped_=false;
  std::mutex mutex_;
  std::condition_variable ready_;
};
}  // namespace faster_lio
