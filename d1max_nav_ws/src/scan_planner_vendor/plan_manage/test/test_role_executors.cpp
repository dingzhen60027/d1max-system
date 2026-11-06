#include <gtest/gtest.h>
#include <plan_manage/role_executors.hpp>
#include <plan_env/coalesced_writer_wake.hpp>
#include <atomic>
#include <cstdlib>
#include <future>
#include <condition_variable>
using namespace std::chrono_literals;
using scan_planner::RoleExecutors;
using scan_planner::ExecutorRole;

class RoleExecutorTest:public ::testing::Test {
protected:
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  void SetUp()override {
    // Timers/guards only. No router, listener, discovery, SDK or application
    // graph. Override inherited live-session settings BEFORE RMW context init.
    unsetenv("ZENOH_CONFIG_OVERRIDE");unsetenv("ZENOH_ROUTER_CONFIG_URI");
    setenv("RMW_IMPLEMENTATION","rmw_zenoh_cpp",1);
    setenv("ZENOH_SESSION_CONFIG_URI",D1MAX_EXECUTOR_TEST_SESSION,1);
    setenv("ZENOH_ROUTER_CHECK_ATTEMPTS","1",1);
    context=std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;init.set_domain_id(239);context->init(0,nullptr,init);
    rclcpp::NodeOptions options;
    options.context(context).enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
    node=std::make_shared<rclcpp::Node>("role_executor_test",options);
  }
  void TearDown()override {node.reset();context->shutdown("isolated test complete");context.reset();}
  auto group(){return node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);}
};

TEST_F(RoleExecutorTest,SingleOwnerCompatibilityAndRepeatedSpinRejected) {
  const auto owner=node->get_node_base_interface()->get_default_callback_group();
  RoleExecutors roles(node->get_node_base_interface(),{{"owner",owner},{"absent_map",nullptr}});
  unsigned ticks=0;
  auto timer=node->create_wall_timer(2ms,[&]{if(++ticks==4)roles.requestStop();},owner);
  roles.spin();
  EXPECT_EQ(roles.size(),1U);EXPECT_EQ(ticks,4U);
  EXPECT_THROW(roles.spin(),std::logic_error);
}

TEST_F(RoleExecutorTest,DuplicateGroupRejected) {
  const auto owner=group();
  EXPECT_THROW((RoleExecutors(node->get_node_base_interface(),{{"a",owner},{"b",owner}})),std::invalid_argument);
}

TEST_F(RoleExecutorTest,NoRolesRejected) {
  EXPECT_THROW((RoleExecutors(node->get_node_base_interface(),{{"absent",nullptr}})),std::invalid_argument);
}

TEST_F(RoleExecutorTest,StopBeforeSpinDoesNotLoseCancellation) {
  auto first=group(),second=group();unsigned ticks=0;
  RoleExecutors roles(node->get_node_base_interface(),{{"first",first},{"second",second}});
  auto timer=node->create_wall_timer(1ms,[&]{++ticks;},second);
  roles.requestStop();roles.spin();EXPECT_EQ(ticks,0U);
}

TEST_F(RoleExecutorTest,PartialThreadStartupFailureJoinsStartedWorkers) {
  auto first=group(),second=group(),third=group();
  std::atomic<unsigned> running{0};unsigned starts=0;
  RoleExecutors roles(node->get_node_base_interface(),{{"first",first},{"second",second},{"third",third}},
    [&](std::function<void()> work)->std::thread {
      if(++starts==2)throw std::runtime_error("injected thread creation failure");
      return std::thread([&,work=std::move(work)]{++running;work();--running;});
    });
  EXPECT_THROW(roles.spin(),std::runtime_error);EXPECT_EQ(starts,2U);EXPECT_EQ(running.load(),0U);
}

TEST_F(RoleExecutorTest,WorkerCallbackFailureStopsAndJoinsEveryRole) {
  auto first=group(),second=group(),third=group();
  RoleExecutors roles(node->get_node_base_interface(),{{"first",first},{"second",second},{"third",third}});
  auto timer=node->create_wall_timer(2ms,[]{throw std::runtime_error("injected callback failure");},second);
  EXPECT_THROW(roles.spin(),std::runtime_error);
}

TEST_F(RoleExecutorTest,OwnerCallbackFailureStopsAndJoinsEveryRole) {
  auto first=group(),second=group();
  RoleExecutors roles(node->get_node_base_interface(),{{"first",first},{"second",second}});
  auto timer=node->create_wall_timer(2ms,[]{throw std::runtime_error("injected owner failure");},first);
  EXPECT_THROW(roles.spin(),std::runtime_error);
}

TEST_F(RoleExecutorTest,SlowOwnerAndTwoIoRolesDoNotStarveMapOrControl) {
  std::vector<ExecutorRole> lanes;
  for(const auto* name:{"owner","map","control","selected_io","display_io"})lanes.push_back({name,group()});
  RoleExecutors roles(node->get_node_base_interface(),lanes);
  std::mutex mutex;std::condition_variable wake;unsigned entered=0;bool release=false;
  std::atomic<unsigned> maps{0},controls{0};
  std::vector<rclcpp::TimerBase::SharedPtr> timers(5);
  for(const auto i:{0U,3U,4U})timers[i]=node->create_wall_timer(1ms,[&,i]{
    timers[i]->cancel();std::unique_lock<std::mutex> lock(mutex);++entered;wake.notify_all();
    wake.wait_for(lock,1s,[&]{return release;});
  },lanes[i].group);
  timers[1]=node->create_wall_timer(100ms,[&]{++maps;},lanes[1].group);
  timers[2]=node->create_wall_timer(20ms,[&]{++controls;},lanes[2].group);
  timers[1]->cancel();timers[2]->cancel();
  auto spin=std::async(std::launch::async,[&]{roles.spin();});
  bool ready;
  {std::unique_lock<std::mutex> lock(mutex);ready=wake.wait_for(lock,500ms,[&]{return entered==3;});}
  timers[1]->reset();timers[2]->reset();std::this_thread::sleep_for(320ms);
  const auto map_count=maps.load(),control_count=controls.load();
  {std::lock_guard<std::mutex> lock(mutex);release=true;}wake.notify_all();roles.requestStop();spin.get();
  EXPECT_TRUE(ready);EXPECT_GE(map_count,2U);EXPECT_GE(control_count,10U);
}

TEST_F(RoleExecutorTest,WriterWakeCoalescesAndPreservesNotifyAfterTake) {
  unsigned calls=0;
  scan_planner::CoalescedWriterWake wake(context,[&]{++calls;});
  for(unsigned i=0;i<100;++i)EXPECT_TRUE(wake.notify());
  EXPECT_TRUE(wake.is_ready(nullptr));auto first=wake.take_data();
  EXPECT_FALSE(wake.is_ready(nullptr));EXPECT_TRUE(wake.notify());
  wake.execute(first);EXPECT_EQ(calls,1U);
  EXPECT_TRUE(wake.is_ready(nullptr));auto second=wake.take_data();wake.execute(second);
  EXPECT_EQ(calls,2U);EXPECT_FALSE(wake.is_ready(nullptr));
  auto empty=wake.take_data();wake.execute(empty);EXPECT_EQ(calls,2U);
  EXPECT_TRUE(wake.notify());auto taken=wake.take_data();wake.close();wake.execute(taken);
  EXPECT_EQ(calls,2U);EXPECT_FALSE(wake.notify());EXPECT_FALSE(wake.is_ready(nullptr));
  EXPECT_EQ(wake.failureCount(),0U);
}

TEST_F(RoleExecutorTest,WriterWakeDispatchesWithoutTimerAndRetainsBusyArrival) {
  auto writer=group();
  rclcpp::ExecutorOptions options;options.context=context;
  rclcpp::executors::SingleThreadedExecutor executor(options);
  std::promise<void> entered,release,second_seen;auto unblock=release.get_future();
  std::atomic<unsigned> calls{0};
  auto wake=std::make_shared<scan_planner::CoalescedWriterWake>(context,[&]{
    if(++calls==1){entered.set_value();unblock.wait();}
    else second_seen.set_value();
  });
  node->get_node_waitables_interface()->add_waitable(wake,writer);
  executor.add_callback_group(writer,node->get_node_base_interface());
  // Trigger before spin: no timer or sensor callback supplies a fallback wake.
  EXPECT_TRUE(wake->notify());
  auto first=std::async(std::launch::async,[&]{executor.spin_once(1s);});
  const bool running=entered.get_future().wait_for(1s)==std::future_status::ready;
  if(running)for(unsigned i=0;i<100;++i)EXPECT_TRUE(wake->notify());
  release.set_value();first.get();
  EXPECT_TRUE(running);
  executor.spin_once(1s);
  EXPECT_EQ(second_seen.get_future().wait_for(0s),std::future_status::ready);
  EXPECT_EQ(calls.load(),2U);executor.spin_once(0ns);EXPECT_EQ(calls.load(),2U);
  EXPECT_EQ(wake->failureCount(),0U);
  wake->close();node->get_node_waitables_interface()->remove_waitable(wake,writer);
}
