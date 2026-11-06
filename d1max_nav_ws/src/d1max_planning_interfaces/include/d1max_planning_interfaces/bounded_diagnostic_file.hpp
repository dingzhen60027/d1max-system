#pragma once

// Diagnostic-only sink. Call exclusively from a non-control diagnostic worker.
// No transport, robot state, callbacks, or authorization are touched here.
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <stdexcept>
#include <cstdint>
#include <array>
#include <memory>
#include <algorithm>
#include <sys/stat.h>
#include <unistd.h>

namespace d1max_diagnostics {

class BoundedDiagnosticFile {
  friend class BoundedTaskDiagnosticFile;
 public:
  static constexpr std::size_t kSlots = 4;
  static constexpr std::size_t kSlotBytes = 4 * 1024 * 1024;
  static constexpr std::size_t kMaxBatchBytes = 16 * 1024;

  explicit BoundedDiagnosticFile(std::string component,
      std::filesystem::path root = {}, std::size_t slot_bytes = kSlotBytes)
      : component_(std::move(component)), root_(std::move(root)), slot_bytes_(slot_bytes) {}

  // Input is already serialized JSON, bounded before this call. Errors only
  // disable this sink; they must never escape into navigation or renew a lease.
  bool write(std::string_view batch) noexcept {
    if (disabled_) { ++failed_writes_; return false; }
    try {
      if (batch.empty() || batch.size() > kMaxBatchBytes ||
          batch.find('\n') != std::string_view::npos) {
        ++rejected_batches_; return false;
      }
      if (directory_.empty()) initialize();
      const auto sequence = ++batch_sequence_;
      const auto make_prefix = [&] { return "{\"trace_file_schema\":1,\"component\":\"" + component_ +
        "\",\"pid\":" + std::to_string(::getpid()) + ",\"batch_sequence\":" +
        std::to_string(sequence) + ",\"write_unix_ns\":" + std::to_string(unixNs()) +
        ",\"overwritten_slots\":" + std::to_string(overwritten_slots_) + ",\"payload\":"; };
      auto prefix = make_prefix();
      auto bytes = prefix.size() + batch.size() + 2;
      if (bytes > slot_bytes_) { ++rejected_batches_; return false; }
      if (!stream_.is_open() || current_bytes_ + bytes > slot_bytes_) {
        rotate(); prefix = make_prefix(); bytes = prefix.size() + batch.size() + 2;
        if (bytes > slot_bytes_) { ++rejected_batches_; return false; }
      }
      stream_.write(prefix.data(), prefix.size());
      stream_.write(batch.data(), batch.size());
      stream_ << "}\n";
      stream_.flush();  // diagnostic worker only; never fsync the control path
      if (!stream_) throw std::runtime_error("diagnostic_write_failed");
      current_bytes_ += bytes;
      return true;
    } catch (...) {
      ++failed_writes_; disabled_ = true;
      return false;
    }
  }

  const std::filesystem::path &directory() const noexcept { return directory_; }
  std::uint64_t failedWrites() const noexcept { return failed_writes_; }
  std::uint64_t rejectedBatches() const noexcept { return rejected_batches_; }
  std::uint64_t overwrittenSlots() const noexcept { return overwritten_slots_; }
  bool disabled() const noexcept { return disabled_; }

 private:
  // Only the task-aware diagnostic consumer can recycle a bucket. Never
  // recursively remove a root or another process's files: these exact four
  // files were created inside this instance's private directory.
  void recycle() {
    if (stream_.is_open()) stream_.close();
    for (std::size_t i=0;i<kSlots;++i)
      std::filesystem::remove(directory_ / ("trace-" + std::to_string(i) + ".jsonl"));
    stream_.clear(); current_bytes_=0; opened_slots_=0;
    batch_sequence_=0; overwritten_slots_=0;
  }
  static std::int64_t unixNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }
  static std::filesystem::path defaultRoot() {
    if (const char *p = std::getenv("D1MAX_TRACE_DIRECTORY"); p && *p) return p;
    if (const char *p = std::getenv("ROS_LOG_DIR"); p && *p)
      return std::filesystem::path(p) / "d1max-traces";
    if (const char *p = std::getenv("HOME"); p && *p)
      return std::filesystem::path(p) / ".ros/d1max-traces";
    return std::filesystem::temp_directory_path() / ("d1max-traces-" + std::to_string(::getuid()));
  }
  void initialize() {
    if (component_.empty() || component_.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_-") != std::string::npos)
      throw std::runtime_error("invalid_diagnostic_component");
    if (root_.empty()) root_ = defaultRoot();
    std::filesystem::create_directories(root_);
    const auto candidate = root_ / (component_ + "-" + std::to_string(::getpid()) + "-" + std::to_string(unixNs()));
    // A fresh, private directory prevents another run's files from being
    // truncated. Rotation replaces only four files this instance created.
    if (!std::filesystem::create_directory(candidate)) throw std::runtime_error("diagnostic_directory_exists");
    if (::chmod(candidate.c_str(), 0700) != 0) throw std::runtime_error("diagnostic_directory_permissions");
    directory_ = candidate;
    std::ofstream index(directory_ / "index.json");
    index << "{\"schema\":1,\"component\":\"" << component_ << "\",\"pid\":" << ::getpid()
      << ",\"created_unix_ns\":" << unixNs() << ",\"slots\":" << kSlots
      << ",\"slot_bytes\":" << slot_bytes_
      << ",\"order\":\"sort records by batch_sequence, not slot filename\","
         "\"source_time\":\"payload source timestamps are unmodified; write time is not acquisition time\","
         "\"robot_state\":\"command and SDK ACK are not measured robot motion\"}\n";
    index.flush();
    if (!index) throw std::runtime_error("diagnostic_index_failed");
  }
  void rotate() {
    if (stream_.is_open()) stream_.close();
    const auto slot = opened_slots_ % kSlots;
    if (opened_slots_ >= kSlots) ++overwritten_slots_;
    stream_.clear();
    stream_.open(directory_ / ("trace-" + std::to_string(slot) + ".jsonl"), std::ios::out | std::ios::trunc);
    if (!stream_) throw std::runtime_error("diagnostic_open_failed");
    ++opened_slots_; current_bytes_ = 0;
  }
  std::string component_;
  std::filesystem::path root_, directory_;
  std::size_t slot_bytes_, current_bytes_{0};
  std::uint64_t batch_sequence_{0}, opened_slots_{0}, overwritten_slots_{0}, failed_writes_{0}, rejected_batches_{0};
  bool disabled_{false};
  std::ofstream stream_;
};

// Consumer-only partitioning. Four recent tasks keep independent 4 MiB
// rings; idle traffic has its own 256 KiB ring. Task completion therefore
// cannot destroy the motion trace merely because the process stays running.
// A ten-second tail preserves stop/retirement events, then only a NEW task
// can evict one of the four retained task buckets. No I/O or new queues are
// introduced in control callbacks.
class BoundedTaskDiagnosticFile {
 public:
  static constexpr std::size_t kTasks=4, kTaskSlotBytes=1024*1024, kIdleSlotBytes=64*1024;
  static constexpr std::size_t kTraceLimitBytes=
    BoundedDiagnosticFile::kSlots*(kTasks*kTaskSlotBytes+kIdleSlotBytes);
  static constexpr std::int64_t kPostTaskNs=10000000000LL;
  explicit BoundedTaskDiagnosticFile(std::string component,std::filesystem::path root={},
      std::size_t task_slot_bytes=kTaskSlotBytes,std::size_t idle_slot_bytes=kIdleSlotBytes)
      : component_(std::move(component)),root_(std::move(root)),
        task_slot_bytes_(task_slot_bytes),idle_slot_bytes_(idle_slot_bytes) {}

  // key must be the exact length-framed session/task identity, not a curve or
  // rolling window. active is captured with the producer's task lifecycle;
  // a stale task ID in an idle command must not keep a task bucket writable.
  bool write(std::string_view batch,std::string_view key,bool active,std::int64_t steady_ns) noexcept {
    if(disabled_){++failed_writes_;return false;}
    try {
      if(key.size()>1024||batch.empty()||batch.size()>BoundedDiagnosticFile::kMaxBatchBytes||
          batch.find('\n')!=std::string_view::npos){++rejected_batches_;return false;}
      if(directory_.empty())initialize();
      // Producer steady time determines retention only. Late/invalid records
      // still get logged, but cannot reopen a task or reset its stop tail.
      const bool ordered=steady_ns>0&&steady_ns>=last_event_ns_;
      last_event_ns_=std::max(last_event_ns_,steady_ns);
      Slot* selected=nullptr;
      if(active&&!key.empty()&&ordered) {
        for(auto& slot:tasks_)if(slot.key==key){selected=&slot;break;}
        if(!selected) {
          selected=&*std::min_element(tasks_.begin(),tasks_.end(),
            [](const Slot& a,const Slot& b){return a.generation<b.generation;});
          const auto index=static_cast<std::size_t>(selected-tasks_.data());
          if(selected->file){selected->file->recycle();++evicted_tasks_;}
          else selected->file=std::make_unique<BoundedDiagnosticFile>(
            "task"+std::to_string(index),directory_,task_slot_bytes_);
          selected->key=key;selected->generation=++generation_;selected->last_active_ns=0;
          writeIndex();
        }
        selected->last_active_ns=std::max(selected->last_active_ns,steady_ns);
        focus_=selected;
      } else if(ordered) {
        if(!key.empty()) {
          for(auto& slot:tasks_)if(slot.key==key){selected=&slot;break;}
        } else selected=focus_;
        if(selected&&(selected->last_active_ns<=0||
            last_event_ns_-selected->last_active_ns>kPostTaskNs))selected=nullptr;
      }
      if(!selected&&!idle_)idle_=std::make_unique<BoundedDiagnosticFile>("idle",directory_,idle_slot_bytes_);
      auto& sink=selected?*selected->file:*idle_;
      const bool ok=sink.write(batch);
      if(!ok){++failed_writes_;disabled_=sink.disabled();}
      return ok;
    } catch(...) {++failed_writes_;disabled_=true;return false;}
  }
  static std::string taskKey(std::string_view session,std::string_view task) {
    return task.empty()?std::string{}:std::to_string(session.size())+":"+std::string(session)+std::string(task);
  }
  const std::filesystem::path& directory()const noexcept{return directory_;}
  std::uint64_t failedWrites()const noexcept{return failed_writes_;}
  std::uint64_t rejectedBatches()const noexcept{return rejected_batches_;}
  std::uint64_t evictedTasks()const noexcept{return evicted_tasks_;}
  bool disabled()const noexcept{return disabled_;}
 private:
  struct Slot {
    std::string key;
    std::unique_ptr<BoundedDiagnosticFile> file;
    std::int64_t last_active_ns{0};
    std::uint64_t generation{0};
  };
  static std::string hex(std::string_view value) {
    constexpr char digits[]="0123456789abcdef";
    std::string result;result.reserve(value.size()*2);
    for(unsigned char c:value){result+=digits[c>>4];result+=digits[c&15];}
    return result;
  }
  void initialize() {
    if(component_.empty()||component_.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_-")!=std::string::npos)
      throw std::runtime_error("invalid_diagnostic_component");
    if(root_.empty())root_=BoundedDiagnosticFile::defaultRoot();
    std::filesystem::create_directories(root_);
    const auto candidate=root_/(component_+"-tasks-"+std::to_string(::getpid())+"-"+
      std::to_string(BoundedDiagnosticFile::unixNs()));
    if(!std::filesystem::create_directory(candidate)||::chmod(candidate.c_str(),0700)!=0)
      throw std::runtime_error("diagnostic_task_directory_failed");
    directory_=candidate;writeIndex();
  }
  void writeIndex() {
    std::ofstream index(directory_/"task-index.json",std::ios::trunc);
    index<<"{\"schema\":1,\"component\":\""<<component_<<"\",\"task_buckets\":"<<kTasks
      <<",\"task_slot_bytes\":"<<task_slot_bytes_<<",\"idle_slot_bytes\":"<<idle_slot_bytes_
      <<",\"slots_per_bucket\":"<<BoundedDiagnosticFile::kSlots<<",\"post_task_ns\":"<<kPostTaskNs
      <<",\"evicted_tasks\":"<<evicted_tasks_<<",\"tasks\":[";
    for(std::size_t i=0;i<tasks_.size();++i) {
      if(i)index<<',';
      index<<"{\"bucket\":\"task"<<i<<"\",\"generation\":"<<tasks_[i].generation
        <<",\"key_hex\":\""<<hex(tasks_[i].key)<<"\"}";
    }
    index<<"],\"key_encoding\":\"hex of decimal session byte length, colon, session, task\","
      "\"retention\":\"last four distinct active tasks; task ring rotates only during its activity and ten-second tail; idle is separate\"}\n";
    index.flush();if(!index)throw std::runtime_error("diagnostic_task_index_failed");
  }
  std::string component_;
  std::filesystem::path root_,directory_;
  std::size_t task_slot_bytes_,idle_slot_bytes_;
  std::array<Slot,kTasks> tasks_{};
  std::unique_ptr<BoundedDiagnosticFile> idle_;
  Slot* focus_{nullptr};
  std::int64_t last_event_ns_{0};
  std::uint64_t generation_{0},evicted_tasks_{0},failed_writes_{0},rejected_batches_{0};
  bool disabled_{false};
};
}  // namespace d1max_diagnostics
