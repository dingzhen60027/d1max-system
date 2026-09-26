#include <chrono>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "d1max_motion_safety/continuous_collision_monitor.hpp"
#include "nav2_collision_monitor/source.hpp"

using Twist = geometry_msgs::msg::Twist;
using Native = nav2_collision_monitor::CollisionMonitor;
using Adapter = d1max_motion_safety::ContinuousCollisionMonitor;

// In-memory obstacle input exercises the real native polygon STOP calculation.
// It has no ROS sensor subscription, TF lookup, robot topic or network source.
class MemorySource : public nav2_collision_monitor::Source
{
public:
  explicit MemorySource(const nav2_util::LifecycleNode::WeakPtr & node)
  : Source(node, "memory", nullptr, "base", "odom", tf2::durationFromSec(0.1),
      rclcpp::Duration::from_seconds(0.5), false)
  {
    enabled_ = true;
  }

  void getData(const rclcpp::Time &, std::vector<nav2_collision_monitor::Point> & data) const override
  {
    if (obstacle) {
      data.insert(data.end(), 4, nav2_collision_monitor::Point{0.0, 0.0});
    }
  }

  bool obstacle = false;
};

template<class Base>
class Probe : public Base
{
public:
  explicit Probe(const rclcpp::NodeOptions & options) : Base(options) {}

  void ageStop()
  {
    this->stop_stamp_ = this->now() - rclcpp::Duration::from_seconds(2.0);
  }

  void addMemorySource(const std::shared_ptr<MemorySource> & source)
  {
    this->sources_.push_back(source);
  }
};

class NativeCollisionContract : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // The wrapper creates a private loopback-only Zenoh router/session. Refuse
    // direct execution so this test cannot inherit the production ROS graph.
    auto env = [](const char * name) {const auto value = std::getenv(name); return value ? std::string(value) : "";};
    if (env("D1MAX_OFFLINE_ZENOH_TEST") != "1" ||
      env("RMW_IMPLEMENTATION") != "rmw_zenoh_cpp" || env("ROS_DOMAIN_ID") != "219")
    {
      throw std::runtime_error("Run through test/run_offline_test.py for isolated Zenoh");
    }
    // Default context is intentional: native TF listener creates its own node.
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    options_.use_intra_process_comms(true).enable_rosout(false)
      .start_parameter_services(false).start_parameter_event_publisher(false);
    observer_ = std::make_shared<rclcpp::Node>("offline_collision_observer", options_);
    subscription_ = observer_->create_subscription<Twist>(
      "/offline_collision/output", 1,
      [this](Twist::ConstSharedPtr msg) {received_.push_back(*msg);});
    input_ = observer_->create_publisher<Twist>("/offline_collision/input", 1);
    executor_.add_node(observer_);
  }

  void TearDown() override
  {
    if (monitor_) {
      if (monitor_->get_current_state().label() == "active") {
        monitor_->deactivate();
      }
      if (monitor_->get_current_state().label() == "inactive") {
        monitor_->cleanup();
      }
      executor_.remove_node(monitor_->get_node_base_interface());
      monitor_.reset();
    }
    executor_.remove_node(observer_);
  }

  template<class Base>
  std::shared_ptr<Probe<Base>> configure(bool activate = true)
  {
    auto options = options_;
    options.parameter_overrides({
      rclcpp::Parameter("cmd_vel_in_topic", "/offline_collision/input"),
      rclcpp::Parameter("cmd_vel_out_topic", "/offline_collision/output"),
      rclcpp::Parameter("stop_pub_timeout", 0.5),
      rclcpp::Parameter("bond_heartbeat_period", 0.0),
      rclcpp::Parameter("observation_sources", std::vector<std::string>{}),
      rclcpp::Parameter("polygons", std::vector<std::string>{"Stop"}),
      rclcpp::Parameter("Stop.type", "polygon"),
      rclcpp::Parameter("Stop.points", std::vector<double>{-1., -1., 1., -1., 1., 1., -1., 1.}),
      rclcpp::Parameter("Stop.action_type", "stop"),
      rclcpp::Parameter("Stop.max_points", 3),
      rclcpp::Parameter("Stop.visualize", false),
      rclcpp::Parameter("Stop.enabled", true),
    });
    auto node = std::make_shared<Probe<Base>>(options);
    monitor_ = node;
    executor_.add_node(node->get_node_base_interface());
    EXPECT_EQ(node->configure().label(), "inactive");
    source_ = std::make_shared<MemorySource>(node->shared_from_this());
    node->addMemorySource(source_);
    if (activate) {
      EXPECT_EQ(node->activate().label(), "active");
    }
    return node;
  }

  void input(double x = 0.0)
  {
    auto msg = std::make_unique<Twist>();
    msg->linear.x = x;
    input_->publish(std::move(msg));
    drain();
  }

  void drain()
  {
    // Execute only already-ready same-process callbacks; no waits for discovery.
    for (int i = 0; i < 4; ++i) {
      executor_.spin_some();
    }
  }

  rclcpp::NodeOptions options_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<Twist>::SharedPtr subscription_;
  rclcpp::Publisher<Twist>::SharedPtr input_;
  std::shared_ptr<Native> monitor_;
  std::shared_ptr<MemorySource> source_;
  std::vector<Twist> received_;
};

TEST_F(NativeCollisionContract, NativeSuppressesNewZeroInputAfterStopTimeout)
{
  auto node = configure<Native>();
  input();
  ASSERT_EQ(received_.size(), 1U);
  node->ageStop();
  input();
  EXPECT_EQ(received_.size(), 1U);
}

TEST_F(NativeCollisionContract, AdapterKeepsZeroOutputForEachActualNewInput)
{
  auto node = configure<Adapter>();
  for (std::size_t i = 1; i <= 8; ++i) {
    node->ageStop();
    input();
    ASSERT_EQ(received_.size(), i);
    EXPECT_DOUBLE_EQ(received_.back().linear.x, 0.0);
  }
}

TEST_F(NativeCollisionContract, NativeCollisionStopStillOverridesNonzeroCommands)
{
  auto node = configure<Adapter>();
  input(0.3);
  ASSERT_EQ(received_.size(), 1U);
  EXPECT_DOUBLE_EQ(received_.back().linear.x, 0.3);
  source_->obstacle = true;
  for (std::size_t i = 2; i <= 5; ++i) {
    node->ageStop();
    input(0.3);
    ASSERT_EQ(received_.size(), i);
    EXPECT_DOUBLE_EQ(received_.back().linear.x, 0.0);
  }
}

TEST_F(NativeCollisionContract, NoInputNeverReplaysCachedZeroOrMovingCommand)
{
  auto node = configure<Adapter>();
  for (const double x : {0.0, 0.3}) {
    input(x);
    const auto count = received_.size();
    node->ageStop();
    // Observe beyond native stop_pub_timeout, not merely an empty event queue.
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(650);
    while (std::chrono::steady_clock::now() < until) {
      drain();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(received_.size(), count);
  }
}

TEST_F(NativeCollisionContract, InactiveAndDeactivatedNodesNeverPublish)
{
  auto node = configure<Adapter>(false);
  input(0.3);
  EXPECT_TRUE(received_.empty());
  ASSERT_EQ(node->activate().label(), "active");
  input();
  ASSERT_EQ(received_.size(), 1U);
  ASSERT_EQ(node->deactivate().label(), "inactive");
  node->ageStop();
  input(0.3);
  input();
  EXPECT_EQ(received_.size(), 1U);
}

TEST_F(NativeCollisionContract, NativeRejectsNanAndInfinityWithoutOutput)
{
  configure<Adapter>();
  input(std::numeric_limits<double>::quiet_NaN());
  input(std::numeric_limits<double>::infinity());
  EXPECT_TRUE(received_.empty());
}
