#pragma once
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace scan_planner {
struct ExecutorRole {
  std::string name;
  rclcpp::CallbackGroup::SharedPtr group;
};

// Callback groups provide mutual exclusion, not execution capacity. Each
// existing role receives one blocking waitset/thread, so a slow diagnostic or
// candidate middleware write cannot consume the map/control executor's thread.
// This changes neither callback rate nor map ownership and is not RT priority.
class RoleExecutors {
public:
  using ThreadFactory=std::function<std::thread(std::function<void()>)>;
  RoleExecutors(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr& node,
                const std::vector<ExecutorRole>& roles,
                ThreadFactory factory=[](std::function<void()> work){return std::thread(std::move(work));})
      :context_(node->get_context()),factory_(std::move(factory)) {
    std::unordered_set<const void*> seen;
    for(const auto& role:roles) {
      if(!role.group)continue; // Archived map/FSM compatibility has one owner.
      if(!seen.insert(role.group.get()).second)throw std::invalid_argument("duplicate executor role");
      // ExecutorOptions owns a mutable memory_strategy. Never copy one options
      // object into several executors: their waitsets would race that strategy.
      rclcpp::ExecutorOptions options;
      options.context=context_;
      auto executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);
      executor->add_callback_group(role.group,node);
      lanes_.push_back({role.name,std::move(executor)});
    }
    if(lanes_.empty())throw std::invalid_argument("missing executor roles");
  }
  RoleExecutors(const RoleExecutors&)=delete;
  RoleExecutors& operator=(const RoleExecutors&)=delete;
  ~RoleExecutors(){requestStop();join();}

  void spin() {
    if(started_.exchange(true))throw std::logic_error("role executors already started");
    try {
      workers_.reserve(lanes_.size()-1);
      for(std::size_t i=1;i<lanes_.size();++i)
        workers_.push_back(factory_([this,i]{run(i);}));
      run(0); // The caller owns the FSM lane and waits for shutdown/failure.
    } catch(...) {recordFailure(std::current_exception());}
    requestStop();join();
    std::exception_ptr failure;
    {std::lock_guard<std::mutex> lock(failure_mutex_);failure=failure_;}
    if(failure)std::rethrow_exception(failure);
  }
  void requestStop() noexcept {
    stopping_.store(true,std::memory_order_release);
    for(const auto& lane:lanes_) {try {lane.executor->cancel();}catch(...) {}}
  }
  std::size_t size()const noexcept{return lanes_.size();}
private:
  struct Lane {std::string name;std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;};
  void run(std::size_t i) noexcept {
    try {
      while(!stopping_.load(std::memory_order_acquire)&&rclcpp::ok(context_))
        // Finite wait closes cancel-before-spin startup races. Ready work is
        // dispatched immediately; this is NOT a 20 ms polling/control period.
        lanes_[i].executor->spin_once(std::chrono::milliseconds(20));
    }catch(...){recordFailure(std::current_exception());}
  }
  void recordFailure(std::exception_ptr error) noexcept {
    {std::lock_guard<std::mutex> lock(failure_mutex_);if(!failure_)failure_=error;}
    requestStop();
  }
  void join() noexcept {for(auto& worker:workers_)if(worker.joinable())worker.join();}
  rclcpp::Context::SharedPtr context_;
  ThreadFactory factory_;
  std::vector<Lane> lanes_;
  std::vector<std::thread> workers_;
  std::atomic<bool> stopping_{false},started_{false};
  std::mutex failure_mutex_;
  std::exception_ptr failure_;
};
} // namespace scan_planner
