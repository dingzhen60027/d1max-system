#include "d1max_navigation_bt/engine.hpp"
#include "d1max_navigation_bt/preparation_budget.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>
#include <unistd.h>

TEST(PreparationOwnership, StartupAndGlobalSearchDoNotSpendFollowBudget) {
  using d1max_navigation_bt::preparationStarted;
  EXPECT_FALSE(preparationStarted(false,false));
  EXPECT_FALSE(preparationStarted(true,false));
  EXPECT_FALSE(preparationStarted(false,true));
  EXPECT_TRUE(preparationStarted(true,true));
}

using namespace d1max_navigation_bt;
using BT::NodeStatus;

namespace {
struct FakeBackend final : Backend {
  ReadyState context{true, "context_valid"};
  ReadyState initial{true, "initial_localization_verified"};
  InitialLocalizationTiming initial_timing;
  ReadyState inputs{true, "ready"};
  Result route{NodeStatus::RUNNING, "route_pending"};
  RouteTiming route_timing;
  Result follow{NodeStatus::RUNNING, "following"};
  bool arrived{false};
  int routes{0}, follows{0}, route_halts{0}, follow_halts{0};
  int pauses{0}, resumes{0}, cancelled{0}, finished{0};
  bool completed_success{false};
  std::vector<std::string> calls;
  std::int64_t source_sequence{0};
  ReadyState contextValid(const TaskIdentity&) override { return context; }
  ReadyState initialLocalizationReady(const TaskIdentity&) override { return initial; }
  InitialLocalizationTiming initialLocalizationTiming(const TaskIdentity&) override {return initial_timing;}
  ReadyState inputsReady(const TaskIdentity&) override {
    auto state=inputs;if(state.ready){state.source_ns=++source_sequence;state.source_identity="epoch1:seed1";}return state;
  }
  void requestRoute(const TaskIdentity& task) override { ++routes; calls.push_back("route:" + task.task_id); }
  RouteTiming routeTiming(const TaskIdentity&) override { return route_timing; }
  Result pollRoute(const TaskIdentity&) override { return route; }
  void haltRoute(const TaskIdentity& task) override { ++route_halts; calls.push_back("halt_route:" + task.task_id); }
  void requestFollow(const TaskIdentity& task) override { ++follows; calls.push_back("follow:" + task.task_id); }
  Result pollFollow(const TaskIdentity&) override { return follow; }
  void haltFollow(const TaskIdentity& task) override { ++follow_halts; calls.push_back("halt_follow:" + task.task_id); }
  bool measuredGoalReached(const TaskIdentity&) override { return arrived; }
  void setPaused(const TaskIdentity&, bool paused, const std::string&) override {
    if (paused) ++pauses; else ++resumes;
  }
  void finishTask(const TaskIdentity&, bool success, const std::string&) override {
    ++finished; completed_success = success;
  }
  void cancelTask(const TaskIdentity& task, const std::string&) override {
    ++cancelled; calls.push_back("cancel:" + task.task_id);
  }
};

class NavigationTree : public ::testing::Test {
protected:
  std::shared_ptr<FakeBackend> backend = std::make_shared<FakeBackend>();
  Engine engine{backend, DEFAULT_BT_XML};
  void startFollow() {
    engine.submit({"task-1", "session-1"});
    ASSERT_EQ(engine.tick(), NodeStatus::RUNNING);
    backend->route = {NodeStatus::SUCCESS, "route_committed"};
    ASSERT_EQ(engine.tick(), NodeStatus::RUNNING);
    ASSERT_EQ(backend->routes, 1);
    ASSERT_EQ(backend->follows, 1);
  }
  void stableResume() {
    ASSERT_EQ(engine.tick(),NodeStatus::RUNNING);
    EXPECT_EQ(engine.snapshot().active_node,"WaitForNavigationInputs");
    std::this_thread::sleep_for(std::chrono::milliseconds(310));
    ASSERT_EQ(engine.tick(),NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(310));
    ASSERT_EQ(engine.tick(),NodeStatus::RUNNING);
  }
};

TEST_F(NavigationTree, NoTaskDoesNotRequestAnything) {
  EXPECT_EQ(engine.tick(), NodeStatus::IDLE);
  EXPECT_EQ(backend->routes, 0);
  EXPECT_FALSE(engine.snapshot().active);
}

TEST_F(NavigationTree, RealXmlAndRealBehaviorTreeNodesAreVisible) {
  startFollow();
  const auto snapshot = engine.snapshot();
  EXPECT_GT(snapshot.transitions.size(), 3u);
  EXPECT_EQ(snapshot.root_status, NodeStatus::RUNNING);
  EXPECT_EQ(snapshot.active_node, "FollowRoute");
  const auto found = std::find_if(snapshot.nodes.begin(), snapshot.nodes.end(), [](const auto& node) {
    return node.name == "CommittedRouteLifecycle" && node.type == "SequenceStar";
  });
  EXPECT_NE(found, snapshot.nodes.end());
}

TEST_F(NavigationTree, InitialLocalizationWaitDoesNotBecomeContextFault) {
  backend->inputs = {false, "waiting_initial_pose"};
  engine.submit({"task-1", "session-1"});
  for (int i = 0; i < 20; ++i) EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->routes, 0);
  EXPECT_EQ(backend->pauses, 1);
  EXPECT_EQ(backend->finished, 0);
  backend->inputs = {true, "ready"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->resumes, 1);
}

TEST_F(NavigationTree, NormalMovementNeverRecomputesCommittedRoute) {
  startFollow();
  for (int i = 0; i < 500; ++i) {
    backend->follow.reason = "progress_" + std::to_string(i);
    ASSERT_EQ(engine.tick(), NodeStatus::RUNNING);
  }
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 1);
  EXPECT_EQ(backend->cancelled, 0);
}

TEST_F(NavigationTree, CoarseCandidateCannotStartPlanningAndUpdatesDoNotReenterStartup) {
  backend->initial = {false, "global_candidate_confirming"};
  engine.submit({"task-1", "session-1"});
  for (int i=0; i<10; ++i) EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->routes, 0);
  EXPECT_EQ(engine.snapshot().active_node, "InitialLocalization");
  backend->initial = {true, "initial_localization_verified"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  backend->route = {NodeStatus::SUCCESS, "route_committed"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  backend->initial = {false, "ordinary_map_update_not_a_new_seed"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 1);
}

TEST_F(NavigationTree, CancelDuringInitialLocalizationDoesNotRequestRouteOrResumeOnLateReadiness) {
  backend->initial = {false,"global_searching"};
  engine.submit({"task-1","session-1"});
  EXPECT_EQ(engine.tick(),NodeStatus::RUNNING);
  engine.cancel("user_cancelled");
  backend->initial = {true,"late_confirmed_pose"};
  EXPECT_EQ(engine.tick(),NodeStatus::IDLE);
  EXPECT_EQ(backend->routes,0);EXPECT_EQ(backend->cancelled,1);
}

TEST_F(NavigationTree, InitialLocalizationHasBoundedFailureWithoutStartingRoute) {
  backend->initial = {false,"ambiguous_place_manual_available"};
  const auto path="/tmp/d1max_bt_initial_timeout_"+std::to_string(getpid())+".xml";
  { std::ofstream file(path);file << R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
    <SequenceStar><WaitForInitialLocalization timeout_s="0.001"/>
      <ComputeGlobalRoute/></SequenceStar></BehaviorTree></root>)"; }
  Engine initial(backend,path);
  initial.submit({"task-1","session-1"});
  EXPECT_EQ(initial.tick(),NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_EQ(initial.tick(),NodeStatus::FAILURE);
  EXPECT_EQ(backend->routes,0);EXPECT_EQ(backend->finished,1);
  EXPECT_NE(initial.snapshot().reason.find("initial_localization_wait_timeout"),std::string::npos);
  std::remove(path.c_str());
}

TEST_F(NavigationTree, InitialIndexWarmupDoesNotConsumePoseBudgetAndCannotRestartAfterCompletion) {
  backend->initial={false,"waiting_global_pose"};backend->initial_timing.preparing_index=true;
  const auto path="/tmp/d1max_bt_initial_index_"+std::to_string(getpid())+".xml";
  {std::ofstream file(path);file<<R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
    <SequenceStar><WaitForInitialLocalization timeout_s="0.002" warmup_timeout_s="0.030"/>
    <ComputeGlobalRoute/></SequenceStar></BehaviorTree></root>)";}
  Engine initial(backend,path);initial.submit({"task-1","session-1"});
  ASSERT_EQ(initial.tick(),NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  EXPECT_EQ(initial.tick(),NodeStatus::RUNNING);
  EXPECT_EQ(initial.snapshot().reason,"preparing_localization_index");
  backend->initial_timing={false,std::chrono::steady_clock::now(),{}};
  EXPECT_EQ(initial.tick(),NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  backend->initial_timing={true,std::nullopt,{}}; // stale index flashback cannot grant a new warmup
  EXPECT_EQ(initial.tick(),NodeStatus::FAILURE);
  EXPECT_NE(initial.snapshot().reason.find("initial_localization_wait_timeout"),std::string::npos);
  EXPECT_EQ(backend->routes,0);std::remove(path.c_str());
}

namespace {
auto startupTime(double seconds) {
  return std::chrono::steady_clock::time_point{}+
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
}
}
TEST(InitialLocalizationBudget, ColdIndexHasOne120SecondDeadlineThenOne60SecondWorkDeadline) {
  InitialLocalizationBudget budget;budget.start(startupTime(1.));
  EXPECT_TRUE(budget.observe(startupTime(1.),{true,{},{}}).empty());
  EXPECT_TRUE(budget.observe(startupTime(110.),{true,{},{}}).empty());
  EXPECT_TRUE(budget.observe(startupTime(111.),{false,startupTime(111.),{}}).empty());
  EXPECT_TRUE(budget.observe(startupTime(150.),{true,{},{}}).empty());
  EXPECT_EQ(budget.observe(startupTime(171.),{}),"initial_localization_wait_timeout");
}
TEST(InitialLocalizationBudget, IndexHeartbeatsUnknownStatesAndLioResetDoNotRenewWarmup) {
  InitialLocalizationBudget budget;budget.start(startupTime(1.));
  budget.observe(startupTime(1.),{true,{},{}});
  for(int i=1;i<120;++i)EXPECT_TRUE(budget.observe(startupTime(1.+i),{i%2==0,{},{}}).empty());
  EXPECT_EQ(budget.observe(startupTime(121.),{true,{},{}}),"initial_localization_index_warmup_timeout");
}
TEST(InitialLocalizationBudget, UnknownInitialStatusDoesNotReceiveExtraIndexAllowance) {
  InitialLocalizationBudget budget;budget.start(startupTime(1.));
  EXPECT_TRUE(budget.observe(startupTime(60.),{}).empty());
  EXPECT_EQ(budget.observe(startupTime(61.),{}),"initial_localization_wait_timeout");
  InitialLocalizationBudget preheated;preheated.start(startupTime(100.));
  EXPECT_TRUE(preheated.observe(startupTime(100.),{false,startupTime(1.),{}}).empty());
  EXPECT_EQ(preheated.observe(startupTime(160.),{}),"initial_localization_wait_timeout");
}
TEST(InitialLocalizationEvidence, OnlyMatchingFreshSequencedComponentStatusCanBudgetWarmup) {
  InitialLocalizationEvidence evidence;const auto hash=std::string(64,'a');
  auto send=[&](unsigned schema,const std::string& session,const std::string& map,std::uint64_t seq,
      std::int64_t epoch,const std::string& phase,double stamp,double now,bool prepared=false,double receipt=1.) {
    return evidence.observe(schema,session,map,seq,epoch,phase,phase,prepared,
      stamp,now,startupTime(receipt),"nav",hash);
  };
  EXPECT_FALSE(send(2,"nav",hash,99,0,"indexing",10.,10.));
  EXPECT_FALSE(send(1,"other",hash,99,0,"indexing",10.,10.));
  EXPECT_FALSE(send(1,"nav",std::string(64,'b'),99,0,"indexing",10.,10.));
  EXPECT_FALSE(send(1,"nav",hash,99,-1,"indexing",10.,10.));
  EXPECT_FALSE(send(1,"nav",hash,99,0,"unknown_phase",10.,10.));
  EXPECT_FALSE(send(1,"nav",hash,99,0,"indexing",9.,10.));
  EXPECT_FALSE(send(1,"nav",hash,99,0,"indexing",10.01,10.));
  ASSERT_TRUE(send(1,"nav",hash,1,0,"indexing",10.,10.));
  EXPECT_TRUE(evidence.timing(10.,startupTime(1.)).preparing_index);
  EXPECT_FALSE(send(1,"nav",hash,1,0,"waiting_stationary",10.,10.,true,1.1));
  EXPECT_FALSE(evidence.timing(10.,startupTime(1.1)).index_completed.has_value());
  EXPECT_FALSE(evidence.timing(10.,startupTime(1.7)).preparing_index);
}
TEST(InitialLocalizationEvidence, GenuineIndexCompletionWorksDuringPausedROSClockAndNeverResets) {
  InitialLocalizationEvidence evidence;const auto hash=std::string(64,'a');
  ASSERT_TRUE(evidence.observe(1,"nav",hash,1,0,"indexing","indexing",false,10.,10.,startupTime(1.),"nav",hash));
  // A phase name without positive index metadata is not completion proof.
  ASSERT_TRUE(evidence.observe(1,"nav",hash,2,0,"waiting_stationary","waiting_stationary",false,10.,10.,startupTime(2.),"nav",hash));
  EXPECT_FALSE(evidence.timing(10.,startupTime(2.)).index_completed.has_value());
  ASSERT_TRUE(evidence.observe(1,"nav",hash,3,0,"waiting_lio_and_head_forward","waiting_lio_and_head_forward",true,10.,10.,startupTime(3.),"nav",hash));
  EXPECT_EQ(evidence.timing(10.,startupTime(3.)).waiting_reason,"waiting_lio_and_head_forward");
  EXPECT_TRUE(evidence.timing(10.,startupTime(3.7)).waiting_reason.empty());
  ASSERT_TRUE(evidence.observe(1,"nav",hash,4,0,"waiting_stationary","waiting_stationary",true,10.,10.,startupTime(5.),"nav",hash));
  auto timing=evidence.timing(10.,startupTime(5.));ASSERT_TRUE(timing.index_completed.has_value());
  EXPECT_EQ(*timing.index_completed,startupTime(3.));
  ASSERT_TRUE(evidence.observe(1,"nav",hash,5,2,"indexing","indexing",false,10.,10.,startupTime(20.),"nav",hash));
  EXPECT_EQ(evidence.timing(10.,startupTime(20.)).index_completed,timing.index_completed);
}

TEST_F(NavigationTree, RepeatedPauseResumePreservesBothActionsAndTaskIdentity) {
  startFollow();
  for (int gap = 0; gap < 3; ++gap) {
    backend->inputs = {false, "pose_temporarily_stale"};
    for (int i = 0; i < 10; ++i) ASSERT_EQ(engine.tick(), NodeStatus::RUNNING);
    EXPECT_EQ(engine.snapshot().active_node, "WaitForNavigationInputs");
    backend->inputs = {true, "fresh_pose"};
    stableResume();
  }
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 1);
  EXPECT_EQ(backend->route_halts, 0);
  EXPECT_EQ(backend->follow_halts, 0);
  EXPECT_EQ(backend->cancelled, 0);
  EXPECT_EQ(backend->pauses, 3);
  EXPECT_EQ(backend->resumes, 3);
  EXPECT_EQ(engine.snapshot().task.task_id, "task-1");
}

TEST_F(NavigationTree, AsyncRouteCompletionDuringPauseIsConsumedOnlyAfterReadinessReturns) {
  engine.submit({"task-1", "session-1"});
  engine.tick();
  backend->inputs = {false, "waiting"};
  backend->route = {NodeStatus::SUCCESS, "route_committed"};
  engine.tick();
  EXPECT_EQ(backend->follows, 0);
  backend->inputs = {true, "ready"};
  stableResume();
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 1);
}

TEST_F(NavigationTree, TemporaryLocalBlockIsNotNewGoalOrGlobalReplan) {
  startFollow();
  backend->follow = {NodeStatus::RUNNING, "blocked_waiting_for_native_replan"};
  for (int i = 0; i < 100; ++i) ASSERT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 1);
  EXPECT_EQ(backend->follow_halts, 0);
}

TEST_F(NavigationTree, PreparationTimeoutRetiresRealTreeWithoutRecomputeOrRecoveryRetry) {
  startFollow();PreparationBudget budget;
  ASSERT_TRUE(budget.observe(PreparationStage::Trajectory,0.).empty());
  const auto failure=budget.observe(PreparationStage::Trajectory,20.);
  ASSERT_EQ(failure,"follow_trajectory_timeout");
  // Navigator surfaces the task-owned supervisor fault through this context
  // interface; exercise the actual XML, halt and finish callbacks below.
  backend->context={false,failure};
  EXPECT_EQ(engine.tick(),NodeStatus::FAILURE);
  EXPECT_EQ(backend->follow_halts,1);EXPECT_EQ(backend->finished,1);
  EXPECT_FALSE(backend->completed_success);
  backend->context={true,"late_valid_curve"};
  EXPECT_EQ(engine.tick(),NodeStatus::FAILURE);
  EXPECT_EQ(backend->routes,1);EXPECT_EQ(backend->follows,1);
}

TEST_F(NavigationTree, HardContextFaultHaltsFollowAndLatchesFailure) {
  startFollow();
  backend->context = {false, "localization_epoch_changed"};
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(backend->follow_halts, 1);
  EXPECT_EQ(backend->finished, 1);
  EXPECT_FALSE(backend->completed_success);
  backend->context = {true, "ready"};
  for (int i = 0; i < 20; ++i) EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->finished, 1);
}

TEST_F(NavigationTree, LostActionTransportIsNotHiddenBySoftInputPause) {
  startFollow();
  backend->inputs = {false, "pose_temporarily_stale"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->follow_halts, 0);
  // The ROS runtime independently runs its action transport watchdog even
  // while the decorator suspends child ticking, then exposes this hard fault.
  backend->context = {false, "follow_action_feedback_timeout"};
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(engine.snapshot().reason, "follow_action_feedback_timeout");
  EXPECT_EQ(backend->follow_halts, 1);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_FALSE(backend->completed_success);
}

TEST_F(NavigationTree, CancelHaltsExactlyOnceAndNeverRestarts) {
  startFollow();
  engine.cancel("operator_cancel");
  engine.cancel("duplicate_cancel");
  EXPECT_EQ(engine.tick(), NodeStatus::IDLE);
  EXPECT_EQ(backend->follow_halts, 1);
  EXPECT_EQ(backend->cancelled, 1);
  EXPECT_FALSE(engine.snapshot().active);
}

TEST_F(NavigationTree, CancelDuringInitialReadinessNeverDispatchesAWorkerAfterRecovery) {
  backend->inputs = {false, "waiting_initial_pose"};
  engine.submit({"task-1", "session-1"});
  engine.tick();
  engine.cancel("operator_cancel");
  backend->inputs = {true, "ready"};
  for (int i = 0; i < 10; ++i) EXPECT_EQ(engine.tick(), NodeStatus::IDLE);
  EXPECT_EQ(backend->routes, 0);
  EXPECT_EQ(backend->follows, 0);
  EXPECT_EQ(backend->cancelled, 1);
}

TEST_F(NavigationTree, CancellationAtComputeCompletionDoesNotLeakIntoFollow) {
  engine.submit({"task-1", "session-1"});
  engine.tick();
  backend->route = {NodeStatus::SUCCESS, "late_route"};
  engine.cancel("operator_cancel");
  for (int i = 0; i < 10; ++i) EXPECT_EQ(engine.tick(), NodeStatus::IDLE);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 0);
  EXPECT_EQ(backend->route_halts, 1);
}

TEST_F(NavigationTree, NewTaskRetiresOldBeforeRequestingNewRoute) {
  startFollow();
  engine.submit({"task-2", "session-1"});
  engine.tick();
  const auto halt = std::find(backend->calls.begin(), backend->calls.end(), "halt_follow:task-1");
  const auto cancel = std::find(backend->calls.begin(), backend->calls.end(), "cancel:task-1");
  const auto route = std::find(backend->calls.begin(), backend->calls.end(), "route:task-2");
  ASSERT_NE(halt, backend->calls.end());
  ASSERT_NE(cancel, backend->calls.end());
  ASSERT_NE(route, backend->calls.end());
  EXPECT_LT(halt, cancel);
  EXPECT_LT(cancel, route);
  EXPECT_EQ(backend->routes, 2);
}

TEST_F(NavigationTree, DuplicateGoalDeliveryIsIdempotentAndIdentityReuseRejected) {
  startFollow();
  for (int i = 0; i < 10; ++i) engine.submit({"task-1", "session-1"});
  EXPECT_EQ(backend->cancelled, 0);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_THROW(engine.submit({"task-1", "different-session"}), std::invalid_argument);
  EXPECT_THROW(engine.submit({"", "session-1"}), std::invalid_argument);
}

TEST_F(NavigationTree, FollowSuccessWithoutMeasuredArrivalCannotCompleteTask) {
  startFollow();
  backend->follow = {NodeStatus::SUCCESS, "path_execution_complete"};
  for (int i = 0; i < 100; ++i) ASSERT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(engine.snapshot().active_node, "VerifyArrival");
  EXPECT_EQ(backend->finished, 0);
  EXPECT_EQ(backend->routes, 1);
  backend->arrived = true;
  EXPECT_EQ(engine.tick(), NodeStatus::SUCCESS);
  EXPECT_EQ(backend->finished, 1);
  EXPECT_TRUE(backend->completed_success);
  EXPECT_EQ(engine.tick(), NodeStatus::SUCCESS);
  EXPECT_EQ(backend->finished, 1);
}

TEST_F(NavigationTree, TerminalRouteFailureDoesNotBlindlyRetry) {
  engine.submit({"task-1", "session-1"});
  engine.tick();
  backend->route = {NodeStatus::FAILURE, "no_global_route"};
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 0);
  EXPECT_EQ(backend->finished, 1);
}

TEST_F(NavigationTree, TerminalFollowFailureDoesNotBlindlyRestartLocalTask) {
  startFollow();
  backend->follow = {NodeStatus::FAILURE, "explicit_unrecoverable_failure"};
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(backend->follows, 1);
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->finished, 1);
}

TEST_F(NavigationTree, IllegalBackendIdleIsDiagnosableAndFailsClosed) {
  engine.submit({"task-1", "session-1"});
  engine.tick();
  backend->route = {NodeStatus::IDLE, "bad_backend"};
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_NE(engine.snapshot().reason.find("route_backend_returned_idle"), std::string::npos);
  EXPECT_EQ(backend->route_halts, 1);
}

TEST(NavigationTreeLimits, PersistentReadinessGapHasBoundedFailureNotEndlessRetry) {
  const auto path = "/tmp/d1max_bt_timeout_test_" + std::to_string(getpid()) + ".xml";
  {
    std::ofstream file(path);
    file << R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <PauseOnUnavailable timeout_s="0.001"><ComputeGlobalRoute/></PauseOnUnavailable>
      </BehaviorTree></root>)";
  }
  auto backend = std::make_shared<FakeBackend>();
  backend->inputs = {false, "sensor_missing"};
  Engine engine(backend, path);
  engine.submit({"task", "session"});
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(backend->routes, 0);
  EXPECT_EQ(backend->finished, 1);
  EXPECT_NE(engine.snapshot().reason.find("navigation_input_wait_timeout"), std::string::npos);
  std::remove(path.c_str());
}

TEST(NavigationTreeLimits, MissingXmlIsRejectedAtStartup) {
  EXPECT_THROW(Engine(std::make_shared<FakeBackend>(), "/nonexistent/d1max.xml"), std::exception);
}

TEST(NavigationRouteDeadline, WorkerPrewarmDoesNotSpendTheTenSecondWorkContract) {
  const auto path="/tmp/d1max_bt_route_stage_"+std::to_string(getpid())+".xml";
  {std::ofstream file(path);file << R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
    <ComputeGlobalRoute timeout_s="0.010" startup_timeout_s="0.100"/>
    </BehaviorTree></root>)";}
  auto backend=std::make_shared<FakeBackend>();backend->route_timing.waiting_worker=true;
  Engine engine(backend,path);engine.submit({"task","session"});
  EXPECT_EQ(engine.tick(),NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  EXPECT_EQ(engine.tick(),NodeStatus::RUNNING); // old unified request clock expired here
  EXPECT_EQ(backend->route_halts,0);
  backend->route_timing={false,std::chrono::steady_clock::now()};
  EXPECT_EQ(engine.tick(),NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  EXPECT_EQ(engine.tick(),NodeStatus::FAILURE);
  EXPECT_EQ(engine.snapshot().reason,"global_route_timeout");
  EXPECT_EQ(backend->routes,1);EXPECT_EQ(backend->route_halts,1);
  std::remove(path.c_str());
}

TEST(NavigationRouteDeadline, WorkerPrewarmHasItsOwnBoundedFailure) {
  const auto path="/tmp/d1max_bt_worker_stage_"+std::to_string(getpid())+".xml";
  {std::ofstream file(path);file << R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
    <ComputeGlobalRoute timeout_s="1" startup_timeout_s="0.002"/>
    </BehaviorTree></root>)";}
  auto backend=std::make_shared<FakeBackend>();backend->route_timing.waiting_worker=true;
  Engine engine(backend,path);engine.submit({"task","session"});engine.tick();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(engine.tick(),NodeStatus::FAILURE);
  EXPECT_EQ(engine.snapshot().reason,"global_worker_startup_timeout");
  EXPECT_EQ(backend->routes,1);EXPECT_EQ(backend->route_halts,1);
  std::remove(path.c_str());
}

TEST(NavigationRouteDeadline, PauseAndStableRecoveryCannotRenewStartedRouteWork) {
  const auto path="/tmp/d1max_bt_paused_stage_"+std::to_string(getpid())+".xml";
  {std::ofstream file(path);file << R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
    <PauseOnUnavailable><ComputeGlobalRoute timeout_s="0.010" startup_timeout_s="1"/>
    </PauseOnUnavailable></BehaviorTree></root>)";}
  auto backend=std::make_shared<FakeBackend>();backend->route_timing.waiting_worker=true;
  Engine engine(backend,path);engine.submit({"task","session"});engine.tick();
  backend->inputs={false,"temporary_gap"};EXPECT_EQ(engine.tick(),NodeStatus::RUNNING);
  // Adapter admits the actual work while the tree is paused. The backend's
  // first start timestamp remains authoritative, not the later BT resume.
  backend->route_timing={false,std::chrono::steady_clock::now()};
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  backend->inputs={true,"restored"};EXPECT_EQ(engine.tick(),NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(310));engine.tick();
  std::this_thread::sleep_for(std::chrono::milliseconds(310));
  EXPECT_EQ(engine.tick(),NodeStatus::FAILURE);
  EXPECT_EQ(engine.snapshot().reason,"global_route_timeout");
  EXPECT_EQ(backend->routes,1);EXPECT_EQ(backend->route_halts,1);
  std::remove(path.c_str());
}

TEST(NavigationTreeLimits, UnprovedFollowSuccessFailsWithoutLoopingOrRecomputing) {
  const auto path = "/tmp/d1max_bt_arrival_timeout_test_" + std::to_string(getpid()) + ".xml";
  {
    std::ofstream file(path);
    file << R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <PauseOnUnavailable><SequenceStar><ComputeGlobalRoute/><FollowCommittedRoute/>
      <VerifyMeasuredArrival timeout_s="0.002"/></SequenceStar></PauseOnUnavailable>
      </BehaviorTree></root>)";
  }
  auto backend = std::make_shared<FakeBackend>();
  Engine engine(backend, path);
  engine.submit({"task", "session"});
  engine.tick();
  backend->route = {NodeStatus::SUCCESS, "committed"};
  engine.tick();
  backend->follow = {NodeStatus::SUCCESS, "claimed_arrival_without_proof"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(engine.snapshot().reason, "measured_arrival_proof_timeout");
  EXPECT_EQ(backend->routes, 1);
  EXPECT_EQ(backend->follows, 1);
  EXPECT_EQ(backend->finished, 1);
  EXPECT_FALSE(backend->completed_success);
  backend->arrived = true;
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  std::remove(path.c_str());
}

TEST(NavigationTreeLimits, MeasuredArrivalBudgetExcludesPausedInputTime) {
  const auto path = "/tmp/d1max_bt_arrival_pause_test_" + std::to_string(getpid()) + ".xml";
  {
    std::ofstream file(path);
    file << R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <PauseOnUnavailable><VerifyMeasuredArrival timeout_s="0.01"/></PauseOnUnavailable>
      </BehaviorTree></root>)";
  }
  auto backend = std::make_shared<FakeBackend>();
  Engine engine(backend, path);
  engine.submit({"task", "session"});
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  backend->inputs = {false, "short_input_gap"};
  engine.tick();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  backend->inputs = {true, "ready"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  backend->arrived = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(310));
  EXPECT_EQ(engine.tick(),NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(310));
  EXPECT_EQ(engine.tick(), NodeStatus::SUCCESS);
  EXPECT_EQ(backend->routes, 0);
  std::remove(path.c_str());
}

TEST(NavigationTreeLimits, UnboundedArrivalTimeoutIsRejectedInsteadOfHanging) {
  const auto path = "/tmp/d1max_bt_arrival_invalid_test_" + std::to_string(getpid()) + ".xml";
  {
    std::ofstream file(path);
    file << R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <VerifyMeasuredArrival timeout_s="0"/></BehaviorTree></root>)";
  }
  auto backend = std::make_shared<FakeBackend>();
  EXPECT_THROW(Engine(backend, path), std::invalid_argument);
  EXPECT_FALSE(backend->completed_success);
  std::remove(path.c_str());
}

TEST(NavigationTreeLimits, InvalidBudgetsFailAtStartupBeforeAnyAcceptedTask) {
  const auto path = "/tmp/d1max_bt_invalid_budget_test_" + std::to_string(getpid()) + ".xml";
  auto backend = std::make_shared<FakeBackend>();
  for (const auto& node : {"PauseOnUnavailable", "ComputeGlobalRoute", "VerifyMeasuredArrival"}) {
    for (const auto& value : {"0", "-1", "nan", "inf"}) {
      {
        std::ofstream file(path);
        file << "<root main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\"><" << node
             << " timeout_s=\"" << value << "\">";
        if (std::string(node) == "PauseOnUnavailable") file << "<AlwaysSuccess/>";
        file << "</" << node << "></BehaviorTree></root>";
      }
      EXPECT_THROW(Engine(backend, path), std::exception) << node << ":" << value;
    }
  }
  EXPECT_EQ(backend->routes, 0);
  EXPECT_EQ(backend->follows, 0);
  std::remove(path.c_str());
}

TEST(NavigationTreeLimits, InputWaitTimeoutHaltsRetainedRunningAction) {
  const auto path = "/tmp/d1max_bt_running_timeout_test_" + std::to_string(getpid()) + ".xml";
  {
    std::ofstream file(path);
    file << R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <PauseOnUnavailable timeout_s="0.001"><ComputeGlobalRoute/></PauseOnUnavailable>
      </BehaviorTree></root>)";
  }
  auto backend = std::make_shared<FakeBackend>();
  Engine engine(backend, path);
  engine.submit({"task", "session"});
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  ASSERT_EQ(backend->routes, 1);
  backend->inputs = {false, "sensor_missing"};
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->route_halts, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_EQ(engine.tick(), NodeStatus::FAILURE);
  EXPECT_EQ(backend->route_halts, 1);
  EXPECT_EQ(backend->finished, 1);
  std::remove(path.c_str());
}

TEST(NavigationTreeLimits, XmlPolicyIsFrozenAcrossTasksWithinOneEngine) {
  const auto path = "/tmp/d1max_bt_frozen_xml_test_" + std::to_string(getpid()) + ".xml";
  {
    std::ofstream file(path);
    file << R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <ComputeGlobalRoute/></BehaviorTree></root>)";
  }
  auto backend = std::make_shared<FakeBackend>();
  Engine engine(backend, path);
  engine.submit({"task-1", "session"});
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  {
    std::ofstream file(path);
    file << R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <AlwaysFailure/></BehaviorTree></root>)";
  }
  engine.submit({"task-2", "session"});
  EXPECT_EQ(engine.tick(), NodeStatus::RUNNING);
  EXPECT_EQ(backend->routes, 2);
  EXPECT_EQ(backend->route_halts, 1);
  std::remove(path.c_str());
}
}  // namespace
