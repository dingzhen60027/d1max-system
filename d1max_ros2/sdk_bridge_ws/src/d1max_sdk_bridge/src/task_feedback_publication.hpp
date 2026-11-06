#pragma once
#include "task_command_feedback.hpp"
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace d1monitor::execution3 {
// Dedicated passive facts consumer, NOT a command writer or ROS executor.
// Exactly one pending latest sample; publication never holds the put mutex.
// A blocked status/JSON consumer cannot block this feedback lane. A blocked
// feedback middleware call itself still has no hard real-time guarantee.
class TaskFeedbackPublication {
public:
  using Publish=std::function<void(const TaskFeedback&)>;
  TaskFeedbackPublication(Publish publish,std::function<void()> failed={}):
      publish_(std::move(publish)),failed_(std::move(failed)),worker_([this]{run();}){}
  ~TaskFeedbackPublication(){close();}
  TaskFeedbackPublication(const TaskFeedbackPublication&)=delete;
  TaskFeedbackPublication& operator=(const TaskFeedbackPublication&)=delete;
  bool put(TaskFeedback fact) {
    {std::lock_guard<std::mutex> lock(mutex_);if(closed_)return false;
      // write_sequence is execution-local; arm generation is writer-wide.
      // A new grant must publish write 1 immediately, even after a long task.
      // This mailbox belongs to one SDK session for its entire lifetime.
      // No publication/receipt timestamp participates in this passive cursor.
      if(!fact.write_sequence)return false;
      if(last_queued_) {
        const auto& old=*last_queued_;
        if(fact.sdk_session!=old.sdk_session||fact.sdk_arm_generation<old.sdk_arm_generation)return false;
        if(fact.sdk_arm_generation==old.sdk_arm_generation&&
           (fact.execution_id!=old.execution_id||fact.control_epoch!=old.control_epoch||
            fact.version.session_id!=old.version.session_id||fact.version.task_id!=old.version.task_id||
            fact.write_sequence<=old.write_sequence))return false;
      }
      last_queued_=fact;
      pending_=std::move(fact);}
    wake_.notify_one();return true;
  }
  void close() {
    {std::lock_guard<std::mutex> lock(mutex_);closed_=true;pending_.reset();}
    wake_.notify_one();if(worker_.joinable())worker_.join();
  }
private:
  void run()noexcept {
    try{for(;;){
      std::optional<TaskFeedback> fact;
      {std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock,[this]{return closed_||pending_.has_value();});
        if(closed_)return;fact=std::move(pending_);pending_.reset();}
      publish_(*fact);
    }}catch(...){
      {std::lock_guard<std::mutex> lock(mutex_);closed_=true;pending_.reset();}
      try{if(failed_)failed_();}catch(...){} // Never unwind a background thread.
    }
  }
  Publish publish_;std::function<void()>failed_;
  std::mutex mutex_;std::condition_variable wake_;std::optional<TaskFeedback>pending_;
  std::optional<TaskFeedback>last_queued_;
  bool closed_{false};std::thread worker_;
};
} // namespace d1monitor::execution3
