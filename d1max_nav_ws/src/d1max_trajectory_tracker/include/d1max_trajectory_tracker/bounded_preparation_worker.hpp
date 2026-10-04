#pragma once
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace d1max_trajectory_tracker {
// One running job, one latest pending job, one completion. The worker owns
// only immutable request data; it must never capture the live controller.
// Invalidating work is synchronous and does not wait behind a long build.
template<class Request,class Result> class BoundedPreparationWorker {
public:
  using Allowed=std::function<bool()>;
  using Builder=std::function<Result(const Request&,const Allowed&)>;
  struct Completion {
    std::uint64_t token;
    Request request;
    std::optional<Result> result;
    std::string failure;
    double elapsed_sec{0.};
  };
  explicit BoundedPreparationWorker(Builder builder):builder_(std::move(builder)),thread_([this]{run();}) {}
  BoundedPreparationWorker(const BoundedPreparationWorker&)=delete;
  BoundedPreparationWorker& operator=(const BoundedPreparationWorker&)=delete;
  ~BoundedPreparationWorker() {
    stopping_.store(true);invalidate();wake_.notify_one();thread_.join();
  }
  std::uint64_t submit(Request request) {
    const auto token=token_.fetch_add(1)+1;
    {std::lock_guard<std::mutex> lock(mutex_);pending_=Job{token,std::move(request)};completed_.reset();}
    wake_.notify_one();return token;
  }
  void invalidate() {
    token_.fetch_add(1);
    std::lock_guard<std::mutex> lock(mutex_);pending_.reset();completed_.reset();
  }
  std::optional<Completion> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto out=std::move(completed_);completed_.reset();
    if(out&&out->token!=token_.load())return {};
    return out;
  }
private:
  struct Job {std::uint64_t token;Request request;};
  void run() {
    for(;;) {
      std::optional<Job> job;
      {std::unique_lock<std::mutex> lock(mutex_);
       wake_.wait(lock,[this]{return stopping_.load()||pending_.has_value();});
       if(stopping_.load())return;
       job=std::move(pending_);pending_.reset();}
      const Allowed allowed=[this,token=job->token]{return !stopping_.load()&&token_.load()==token;};
      Completion done{job->token,std::move(job->request),{}, {}};
      const auto began=std::chrono::steady_clock::now();
      try {if(allowed())done.result=builder_(done.request,allowed);}
      catch(const std::exception& e){done.failure=e.what();}
      catch(...){done.failure="preparation_worker_exception";}
      done.elapsed_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
      if(!allowed())continue;
      std::lock_guard<std::mutex> lock(mutex_);
      if(allowed())completed_=std::move(done);
    }
  }
  Builder builder_;
  std::atomic<std::uint64_t> token_{0};std::atomic<bool> stopping_{false};
  std::mutex mutex_;std::condition_variable wake_;
  std::optional<Job> pending_;std::optional<Completion> completed_;
  std::thread thread_;
};
} // namespace d1max_trajectory_tracker
