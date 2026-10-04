#pragma once

#include <behaviortree_cpp_v3/basic_types.h>

#include <cstdint>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "d1max_navigation_bt/initial_localization_budget.hpp"

namespace d1max_navigation_bt {

struct TaskIdentity {
  std::string task_id;
  // Opaque session identity. The backend binds localization epoch/seed only
  // after they become known; an absent initial pose is not a context fault.
  std::string context_id;
};

struct ReadyState {
  bool ready{false};
  std::string reason;
  std::int64_t source_ns{0};
  std::string source_identity;
};

struct Result {
  BT::NodeStatus status{BT::NodeStatus::RUNNING};
  std::string reason;
};

struct RouteTiming {
  // Heavy worker prewarm is a separate capability, not consumed compute time.
  bool waiting_worker{false};
  // Set once by the actual adapter-stage feedback. Pausing the tree must not
  // restart this clock or grant another route deadline.
  std::optional<std::chrono::steady_clock::time_point> work_started;
};

// All methods must be non-blocking. Each asynchronous reply is correlated with
// this task identity AND the backend's action-request generation. A paused
// backend must not turn a cached visualization into permission to execute.
class Backend {
public:
  virtual ~Backend() = default;
  virtual ReadyState contextValid(const TaskIdentity&) = 0;
  // Legacy backends retain their input gate; production overrides this with
  // verified continuous localization, never a coarse candidate.
  virtual ReadyState initialLocalizationReady(const TaskIdentity&) {
    return {true, "initial_localization_managed_externally"};
  }
  virtual InitialLocalizationTiming initialLocalizationTiming(const TaskIdentity&) {return {};}
  virtual ReadyState inputsReady(const TaskIdentity&) = 0;
  virtual void requestRoute(const TaskIdentity&) = 0;
  virtual RouteTiming routeTiming(const TaskIdentity&) { return {}; }
  virtual Result pollRoute(const TaskIdentity&) = 0;
  virtual void haltRoute(const TaskIdentity&) = 0;
  virtual void requestFollow(const TaskIdentity&) = 0;
  virtual Result pollFollow(const TaskIdentity&) = 0;
  virtual void haltFollow(const TaskIdentity&) = 0;
  // Must use current, correctly identified measured arrival proof. Time spent
  // following, simulated elapsed progress and a displayed path are not proof.
  virtual bool measuredGoalReached(const TaskIdentity&) = 0;
  virtual void setPaused(const TaskIdentity&, bool paused, const std::string& reason) = 0;
  virtual void finishTask(const TaskIdentity&, bool success, const std::string& reason) = 0;
  virtual void cancelTask(const TaskIdentity&, const std::string& reason) = 0;
};

struct NodeSnapshot {
  std::uint16_t uid{0};
  std::string name;
  std::string type;
  BT::NodeStatus status{BT::NodeStatus::IDLE};
  std::string reason;
};

struct Transition {
  std::uint64_t sequence{0};
  std::uint16_t uid{0};
  std::string node;
  BT::NodeStatus previous{BT::NodeStatus::IDLE};
  BT::NodeStatus current{BT::NodeStatus::IDLE};
};

struct Snapshot {
  TaskIdentity task;
  bool active{false};
  BT::NodeStatus root_status{BT::NodeStatus::IDLE};
  std::string active_node;
  std::string reason;
  std::vector<NodeSnapshot> nodes;
  std::vector<Transition> transitions;
};

class Engine {
public:
  Engine(std::shared_ptr<Backend> backend, const std::string& xml_path);
  ~Engine();
  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;
  // Repeated delivery of the identical task is idempotent. Reusing a task ID
  // with a different context is rejected. A new task explicitly halts the old.
  void submit(const TaskIdentity& task);
  void cancel(const std::string& reason = "cancelled");
  BT::NodeStatus tick();
  Snapshot snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace d1max_navigation_bt
