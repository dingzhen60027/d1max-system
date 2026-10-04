#include "d1max_navigation_bt/engine.hpp"
#include "d1max_navigation_bt/stable_recovery.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace d1max_navigation_bt {
namespace {
using Clock = std::chrono::steady_clock;

struct Runtime {
  std::shared_ptr<Backend> backend;
  TaskIdentity task;
  std::unordered_map<std::uint16_t, std::string> reasons;
  std::deque<Transition> transitions;
  std::uint64_t next_transition{0};
  std::string active_node;
  std::string reason;
  Clock::time_point now{Clock::now()};
  double active_time{0.0};

  void explain(const BT::TreeNode& node, const std::string& why) {
    active_node = node.name();
    reason = why;
    reasons[node.UID()] = why;
  }
};

class TransitionLog final : public BT::StatusChangeLogger {
public:
  TransitionLog(BT::TreeNode* root, Runtime& runtime)
      : BT::StatusChangeLogger(root), runtime_(runtime) {}
  void callback(BT::Duration, const BT::TreeNode& node, BT::NodeStatus before,
                BT::NodeStatus after) override {
    runtime_.transitions.push_back(
        {++runtime_.next_transition, node.UID(), node.name(), before, after});
    if (runtime_.transitions.size() > 256) runtime_.transitions.pop_front();
  }
  void flush() override {}

private:
  Runtime& runtime_;
};

class TaskContextValid final : public BT::ConditionNode {
public:
  TaskContextValid(const std::string& name, const BT::NodeConfiguration& config,
                   Runtime& runtime)
      : BT::ConditionNode(name, config), runtime_(runtime) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    const auto state = runtime_.backend->contextValid(runtime_.task);
    if (!state.ready) {
      runtime_.explain(*this, state.reason.empty() ? "task_context_invalid" : state.reason);
      return BT::NodeStatus::FAILURE;
    }
    runtime_.reasons[UID()] = "task_context_valid";
    return BT::NodeStatus::SUCCESS;
  }

private:
  Runtime& runtime_;
};

// Unlike ReactiveSequence(Ready, Follow), this decorator deliberately does NOT
// halt its running child when data briefly disappears. Native action identity,
// an in-flight asynchronous reply and the committed route remain intact.
class PauseOnUnavailable final : public BT::DecoratorNode {
public:
  PauseOnUnavailable(const std::string& name, const BT::NodeConfiguration& config,
                     Runtime& runtime)
      : BT::DecoratorNode(name, config), runtime_(runtime) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("timeout_s", 30.0, "Bounded positive continuous input wait")};
  }
  BT::NodeStatus tick() override {
    const auto ready = runtime_.backend->inputsReady(runtime_.task);
    const auto now = runtime_.now;
    const auto monotonic=std::chrono::duration<double>(now.time_since_epoch()).count();
    const bool admitted=recovery_.observe(ready.ready,monotonic,ready.source_ns,ready.source_identity);
    if (!admitted) {
      if (!paused_) {
        paused_ = true;
        paused_at_ = now;
        runtime_.backend->setPaused(runtime_.task, true, ready.ready?"confirming_navigation_recovery":ready.reason);
      }
      last_ready_ = false;
      last_tick_ = now;
      runtime_.explain(*this, ready.ready?"confirming_navigation_recovery":
        ready.reason.empty()?"waiting_for_navigation_inputs":ready.reason);
      const auto timeout = getInput<double>("timeout_s").value();
      if (timeout > 0 && std::chrono::duration<double>(now - paused_at_).count() >= timeout) {
        runtime_.explain(*this, "navigation_input_wait_timeout:" + runtime_.reason);
        // A decorator returning FAILURE is no longer RUNNING, so its parent
        // need not call halt() on it. Explicitly cancel a retained RUNNING
        // child here; otherwise a timed-out async action can remain alive.
        haltChild();
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    if (last_ready_) {
      runtime_.active_time += std::chrono::duration<double>(now - last_tick_).count();
    }
    last_ready_ = true;
    last_tick_ = now;
    if (paused_) {
      paused_ = false;
      runtime_.backend->setPaused(runtime_.task, false, "navigation_inputs_restored");
    }
    runtime_.reasons[UID()] = "navigation_inputs_ready";
    return child_node_->executeTick();
  }
  void halt() override {
    // Halting is a terminal/new-task operation, not a resume permission.
    paused_ = false;
    last_ready_ = false;
    BT::DecoratorNode::halt();
  }

private:
  Runtime& runtime_;
  bool paused_{false};
  bool last_ready_{false};
  Clock::time_point paused_at_{};
  Clock::time_point last_tick_{};
  StableRecoveryGate recovery_;
};

class WaitForInitialLocalization final : public BT::StatefulActionNode {
public:
  WaitForInitialLocalization(const std::string& name, const BT::NodeConfiguration& config,
                            Runtime& runtime)
      : BT::StatefulActionNode(name, config), runtime_(runtime) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("timeout_s", 60.0, "Bounded post-index localization wait"),
      BT::InputPort<double>("warmup_timeout_s",120.0,"Independent single map-index preparation deadline")};
  }
  BT::NodeStatus onStart() override {
    budget_=InitialLocalizationBudget(getInput<double>("warmup_timeout_s").value(),getInput<double>("timeout_s").value());
    budget_.start(runtime_.now);return onRunning();
  }
  BT::NodeStatus onRunning() override {
    const auto state = runtime_.backend->initialLocalizationReady(runtime_.task);
    if (state.ready) {
      runtime_.reasons[UID()] = "initial_localization_verified";
      return BT::NodeStatus::SUCCESS;
    }
    const auto timing=runtime_.backend->initialLocalizationTiming(runtime_.task);
    const auto failure=budget_.observe(runtime_.now,timing);
    runtime_.explain(*this,timing.preparing_index?"preparing_localization_index":
      (!timing.waiting_reason.empty()?timing.waiting_reason:
        state.reason.empty()?"waiting_initial_localization":state.reason));
    if (!failure.empty()) {
      runtime_.explain(*this, failure+":"+runtime_.reason);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}  // Does not cancel the estimator or issue a seed.
private:
  Runtime& runtime_;
  InitialLocalizationBudget budget_;
};

class ComputeGlobalRoute final : public BT::StatefulActionNode {
public:
  ComputeGlobalRoute(const std::string& name, const BT::NodeConfiguration& config,
                     Runtime& runtime)
      : BT::StatefulActionNode(name, config), runtime_(runtime) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("timeout_s", 10.0, "Route work end-to-end steady-clock deadline; pauses included"),
      BT::InputPort<double>("startup_timeout_s",60.0,"Separate heavy worker prewarm deadline")};
  }
  BT::NodeStatus onStart() override {
    started_ = runtime_.now;
    for(const auto* port:{"timeout_s","startup_timeout_s"}) {
      const auto timeout=getInput<double>(port).value();
      if(!std::isfinite(timeout)||timeout<=0.)throw std::invalid_argument("invalid_global_route_deadline");
    }
    runtime_.backend->requestRoute(runtime_.task);
    runtime_.explain(*this, "global_route_requested");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    const auto result = runtime_.backend->pollRoute(runtime_.task);
    if (result.status == BT::NodeStatus::IDLE) throw std::runtime_error("route_backend_returned_idle");
    // A real result received while paused is not still pending work. Its
    // transport owner already enforces dispatch-to-result deadline/retirement.
    if(result.status!=BT::NodeStatus::RUNNING) {
      runtime_.explain(*this,result.reason);return result.status;
    }
    const auto timing=runtime_.backend->routeTiming(runtime_.task);
    const auto since=timing.waiting_worker?started_:timing.work_started.value_or(started_);
    const auto timeout=getInput<double>(timing.waiting_worker?"startup_timeout_s":"timeout_s").value();
    if (std::chrono::duration<double>(runtime_.now-since).count() >= timeout) {
      runtime_.backend->haltRoute(runtime_.task);
      runtime_.explain(*this,timing.waiting_worker?"global_worker_startup_timeout":"global_route_timeout");
      return BT::NodeStatus::FAILURE;
    }
    runtime_.explain(*this, result.reason.empty() ? "computing_global_route" : result.reason);
    return result.status;
  }
  void onHalted() override { runtime_.backend->haltRoute(runtime_.task); }

private:
  Runtime& runtime_;
  Clock::time_point started_{};
};

class FollowCommittedRoute final : public BT::StatefulActionNode {
public:
  FollowCommittedRoute(const std::string& name, const BT::NodeConfiguration& config,
                       Runtime& runtime)
      : BT::StatefulActionNode(name, config), runtime_(runtime) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    runtime_.backend->requestFollow(runtime_.task);
    runtime_.explain(*this, "committed_route_follow_requested");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    const auto result = runtime_.backend->pollFollow(runtime_.task);
    if (result.status == BT::NodeStatus::IDLE) throw std::runtime_error("follow_backend_returned_idle");
    runtime_.explain(*this, result.reason.empty() ? "following_committed_route" : result.reason);
    return result.status;
  }
  void onHalted() override { runtime_.backend->haltFollow(runtime_.task); }

private:
  Runtime& runtime_;
};

class VerifyMeasuredArrival final : public BT::StatefulActionNode {
public:
  VerifyMeasuredArrival(const std::string& name, const BT::NodeConfiguration& config,
                        Runtime& runtime)
      : BT::StatefulActionNode(name, config), runtime_(runtime) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("timeout_s", 3.0,
      "Bounded active-time wait for final measured-arrival proof")};
  }
  BT::NodeStatus onStart() override {
    started_ = runtime_.active_time;
    const auto timeout = getInput<double>("timeout_s").value();
    if (!std::isfinite(timeout) || timeout <= 0) {
      throw std::invalid_argument("invalid_measured_arrival_timeout");
    }
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    if (runtime_.backend->measuredGoalReached(runtime_.task)) {
      runtime_.explain(*this, "measured_goal_reached");
      return BT::NodeStatus::SUCCESS;
    }
    if (runtime_.active_time - started_ >= getInput<double>("timeout_s").value()) {
      runtime_.explain(*this, "measured_arrival_proof_timeout");
      return BT::NodeStatus::FAILURE;
    }
    runtime_.explain(*this, "waiting_for_measured_arrival");
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}

private:
  Runtime& runtime_;
  double started_{0.0};
};
}  // namespace

class Engine::Impl {
public:
  Impl(std::shared_ptr<Backend> backend, std::string xml_path)
      : xml_path_(std::move(xml_path)) {
    if (!backend) throw std::invalid_argument("navigation_backend_is_null");
    runtime_.backend = std::move(backend);
    registerNode<TaskContextValid>("TaskContextValid");
    registerNode<PauseOnUnavailable>("PauseOnUnavailable");
    registerNode<WaitForInitialLocalization>("WaitForInitialLocalization");
    registerNode<ComputeGlobalRoute>("ComputeGlobalRoute");
    registerNode<FollowCommittedRoute>("FollowCommittedRoute");
    registerNode<VerifyMeasuredArrival>("VerifyMeasuredArrival");
    std::ifstream xml_file(xml_path_);
    if (!xml_file) throw std::invalid_argument("navigation_tree_xml_unreadable:" + xml_path_);
    xml_text_.assign(std::istreambuf_iterator<char>(xml_file), std::istreambuf_iterator<char>());
    if (xml_text_.empty() || xml_text_.size() > 1024 * 1024) {
      throw std::invalid_argument("navigation_tree_xml_empty_or_oversized");
    }
    // Parse at startup, not at first goal, so malformed XML cannot leave an
    // accepted navigation goal without a usable tree.
    rebuild();
  }
  template <typename Node> void registerNode(const std::string& id) {
    factory_.registerBuilder<Node>(id, [this](const std::string& name, const BT::NodeConfiguration& cfg) {
      return std::make_unique<Node>(name, cfg, runtime_);
    });
  }
  void rebuild() {
    logger_.reset();
    // Freeze the startup XML for the entire session. Editing the source file
    // cannot silently change the policy on the next goal.
    tree_ = factory_.createTreeFromText(xml_text_);
    for (const auto& node : tree_.nodes) {
      const auto& type = node->registrationName();
      if (type == "WaitForInitialLocalization" || type == "PauseOnUnavailable" || type == "ComputeGlobalRoute" ||
          type == "VerifyMeasuredArrival") {
        const auto timeout = node->getInput<double>("timeout_s");
        if (!timeout || !std::isfinite(timeout.value()) || timeout.value() <= 0) {
          throw std::invalid_argument(type + "_timeout_must_be_finite_positive");
        }
        if(type=="WaitForInitialLocalization") {
          const auto warmup=node->getInput<double>("warmup_timeout_s");
          if(!warmup)throw std::invalid_argument("initial_localization_warmup_deadline_missing");
          InitialLocalizationBudget(warmup.value(),timeout.value());
        }
      }
    }
    logger_ = std::make_unique<TransitionLog>(tree_.rootNode(), runtime_);
  }
  void submit(const TaskIdentity& task) {
    if (task.task_id.empty()) throw std::invalid_argument("empty_navigation_task_id");
    if (task.task_id == runtime_.task.task_id) {
      if (task.context_id != runtime_.task.context_id) throw std::invalid_argument("task_id_context_reuse");
      return;
    }
    cancel("superseded_by_new_task");
    runtime_.task = task;
    runtime_.reasons.clear();
    runtime_.transitions.clear();
    runtime_.active_time = 0;
    runtime_.active_node.clear();
    runtime_.reason = "task_accepted";
    rebuild();
    status_ = BT::NodeStatus::IDLE;
    active_ = true;
  }
  void cancel(const std::string& reason) {
    if (!active_) return;
    tree_.haltTree();
    runtime_.backend->cancelTask(runtime_.task, reason);
    runtime_.reason = reason;
    runtime_.active_node.clear();
    status_ = BT::NodeStatus::IDLE;
    active_ = false;
  }
  BT::NodeStatus tick() {
    if (!active_) return status_;
    runtime_.now = Clock::now();
    try {
      status_ = tree_.tickRoot();
    } catch (const std::exception& error) {
      runtime_.reason = std::string("behavior_tree_exception:") + error.what();
      status_ = BT::NodeStatus::FAILURE;
    }
    if (status_ == BT::NodeStatus::SUCCESS || status_ == BT::NodeStatus::FAILURE) {
      // Latch terminal result; repeated timer ticks never restart the route.
      active_ = false;
      if (status_ == BT::NodeStatus::FAILURE) tree_.haltTree();
      runtime_.backend->finishTask(runtime_.task, status_ == BT::NodeStatus::SUCCESS, runtime_.reason);
    }
    return status_;
  }
  Snapshot snapshot() const {
    Snapshot out;
    out.task = runtime_.task;
    out.active = active_;
    out.root_status = status_;
    out.active_node = runtime_.active_node;
    out.reason = runtime_.reason;
    for (const auto& node : tree_.nodes) {
      const auto why = runtime_.reasons.find(node->UID());
      out.nodes.push_back({node->UID(), node->name(), node->registrationName(), node->status(),
                          why == runtime_.reasons.end() ? "" : why->second});
    }
    out.transitions.assign(runtime_.transitions.begin(), runtime_.transitions.end());
    return out;
  }

  Runtime runtime_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<TransitionLog> logger_;
  std::string xml_path_;
  std::string xml_text_;
  BT::NodeStatus status_{BT::NodeStatus::IDLE};
  bool active_{false};
};

Engine::Engine(std::shared_ptr<Backend> backend, const std::string& xml_path)
    : impl_(std::make_unique<Impl>(std::move(backend), xml_path)) {}
Engine::~Engine() {
  try { impl_->cancel("navigator_shutdown"); } catch (...) { /* destructors must not throw */ }
}
void Engine::submit(const TaskIdentity& task) { impl_->submit(task); }
void Engine::cancel(const std::string& reason) { impl_->cancel(reason); }
BT::NodeStatus Engine::tick() { return impl_->tick(); }
Snapshot Engine::snapshot() const { return impl_->snapshot(); }

}  // namespace d1max_navigation_bt
