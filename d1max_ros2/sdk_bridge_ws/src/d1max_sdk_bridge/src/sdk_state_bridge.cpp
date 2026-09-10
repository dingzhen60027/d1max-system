#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot_sdk/sdk_client.hpp"

namespace d1max_sdk_bridge {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// Values defined by D1 Max SDK Guide V0.0.9.
constexpr int kEmergencyRecover = 1;
constexpr int kControlSdk = 2;
constexpr int kModeGeneral = 1;
constexpr int kModeInPlace = 2;
constexpr int kModeStair = 3;
constexpr int kMotionUnknown = 0;
constexpr int kMotionStand = 1;
constexpr int kMotionLieDown = 2;
constexpr int kMotionCrawl = 3;
constexpr int kMotionLocked = 4;
constexpr int kMotionGeneral = 5;
constexpr int kMotionInPlace = 6;
constexpr int kMotionStair = 7;
constexpr int kMotionClimb = 8;
constexpr int kMotionSlim = 9;
constexpr int kMotionGait = 10;
constexpr int kFaultFatal = 1;
constexpr int kFaultError = 2;

enum class StepKind {
  kTakeControl,
  kStop,
  kStand,
  kLieDown,
  kCrawl,
  kGeneralMode,
  kInPlaceMode,
  kStairMode,
  kSetNavigationSpeed,
  kLock,
  kReleaseControl,
};

struct RobotSnapshot {
  bool received{false};
  int motion_status{kMotionUnknown};
  int sport_mode{0};
  int control_source{0};
  int speed_level{0};
  int software_emergency{0};
  int hardware_emergency{0};
  double forward_speed{0.0};
  double lateral_speed{0.0};
  double yaw_speed{0.0};
  SteadyClock::time_point received_at{};
};

class SdkBridge;

class BridgeDataCallback final : public robot_sdk::IDataCallback {
 public:
  explicit BridgeDataCallback(SdkBridge* bridge) : bridge_(bridge) {}
  void OnRobotStateData(const robot_sdk::RobotState& data) override;
  void OnFaultData(const robot_sdk::FaultDatas& data) override;
  void OnControlLost(const robot_sdk::ControlLostInfo&) override;
  void OnControlAvailable(const robot_sdk::ControlAvailableInfo&) override;

 private:
  SdkBridge* bridge_;
};

class BridgeControlCallback final : public robot_sdk::IControlCallback {
 public:
  explicit BridgeControlCallback(SdkBridge* bridge) : bridge_(bridge) {}
  void OnStandUp() override;
  void OnLieDown() override;
  void OnCrawl() override;
  void OnMode(int mode) override;
  void OnSpeed(int speed) override;
  void OnLocked() override;
  void OnTakeControlAck(const robot_sdk::TakeControlAck& ack) override;
  void OnReleaseControlAck(const robot_sdk::ReleaseControlAck& ack) override;

 private:
  SdkBridge* bridge_;
};

class SdkBridge final : public rclcpp::Node {
 public:
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  SdkBridge() : Node("d1max_sdk_bridge") {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.168.168");
    robot_port_ = declare_parameter<int>("robot_port", 8081);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 3000);
    transition_timeout_sec_ =
        declare_parameter<double>("transition_timeout_sec", 15.0);
    robot_state_stale_sec_ =
        declare_parameter<double>("robot_state_stale_sec", 2.5);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_timeout_sec_ =
        declare_parameter<double>("cmd_vel_timeout_sec", 0.30);
    cmd_vel_rate_hz_ = declare_parameter<double>("cmd_vel_rate_hz", 20.0);
    max_forward_speed_ =
        declare_parameter<double>("max_forward_speed", 0.45);
    max_lateral_speed_ =
        declare_parameter<double>("max_lateral_speed", 0.35);
    max_yaw_speed_ = declare_parameter<double>("max_yaw_speed", 0.80);
    navigation_speed_level_ =
        declare_parameter<int>("navigation_speed_level", 1);
    const int connect_timeout_ms =
        declare_parameter<int>("connect_timeout_ms", 5000);
    const int reconnect_interval_ms =
        declare_parameter<int>("reconnect_interval_ms", 1000);

    robot_state_pub_ = create_publisher<std_msgs::msg::String>("~/robot_state", 10);
    faults_pub_ = create_publisher<std_msgs::msg::String>("~/faults", 10);
    behavior_state_pub_ =
        create_publisher<std_msgs::msg::String>("~/behavior_state", 10);
    transition_pub_ =
        create_publisher<std_msgs::msg::String>("~/transition_event", 10);
    ready_pub_ =
        create_publisher<std_msgs::msg::Bool>("~/ready_for_navigation", 10);
    connection_state_pub_ =
        create_publisher<std_msgs::msg::UInt8>("~/connection_state", 10);
    connection_text_pub_ =
        create_publisher<std_msgs::msg::String>("~/connection_state_text", 10);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_cmd_vel_ = *message;
          latest_cmd_vel_at_ = SteadyClock::now();
          have_cmd_vel_ = true;
        });
    CreateServices();

    robot_sdk::ConnectionConfig connection_config;
    connection_config.connect_timeout_ms = connect_timeout_ms;
    connection_config.auto_reconnect = true;
    connection_config.reconnect_interval_ms = reconnect_interval_ms;
    sdk_client_ = std::make_unique<robot_sdk::SDKClient>(
        [this](const std::error_code& error) {
          if (!shutting_down_.load() && error) {
            RCLCPP_ERROR(get_logger(), "Robot SDK error: %s",
                         error.message().c_str());
          }
        },
        connection_config);
    data_callback_ = std::make_shared<BridgeDataCallback>(this);
    control_callback_ = std::make_shared<BridgeControlCallback>(this);
    sdk_client_->SetDataCallback(data_callback_);
    sdk_client_->SetControlCallback(control_callback_);

    RCLCPP_INFO(get_logger(), "D1 Max SDK version: %s, protocol: %s",
                sdk_client_->Version().c_str(),
                sdk_client_->ProtocolVersion().c_str());
    RCLCPP_WARN(get_logger(),
                "Guarded mode: startup never takes control or sends motion");
    const auto error =
        sdk_client_->Connect(robot_ip_, std::to_string(robot_port_), true);
    if (error) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to %s:%d: %s",
                   robot_ip_.c_str(), robot_port_, error.message().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Connected to D1 Max SDK at %s:%d",
                  robot_ip_.c_str(), robot_port_);
    }

    fsm_timer_ = create_wall_timer(100ms, [this]() { RunStateMachine(); });
    status_timer_ = create_wall_timer(200ms, [this]() { PublishStatus(); });
    connection_timer_ =
        create_wall_timer(1s, [this]() { PublishConnectionState(); });
    const auto velocity_period =
        std::chrono::duration<double>(1.0 / std::max(1.0, cmd_vel_rate_hz_));
    velocity_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(velocity_period),
        [this]() { RunVelocityGate(); });
    PublishConnectionState();
  }

  ~SdkBridge() override {
    shutting_down_.store(true);
    if (!sdk_client_) {
      return;
    }
    const auto state = Snapshot();
    if (sdk_client_->IsConnected() && state.control_source == kControlSdk) {
      if (CanUseMove(state)) {
        sdk_client_->Move(0.0F, 0.0F, 0.0F, 500);
      }
      sdk_client_->ReleaseControl(1000);
    }
    if (sdk_client_->GetConnectionState() !=
        robot_sdk::ConnectionState::DISCONNECTED) {
      sdk_client_->Disconnect(true);
    }
    data_callback_.reset();
    control_callback_.reset();
  }

  void UpdateRobotState(const robot_sdk::RobotState& data) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_.received = true;
      state_.motion_status = static_cast<int>(data.motion_status);
      state_.sport_mode = static_cast<int>(data.sport_mode);
      state_.control_source = static_cast<int>(data.control_source);
      state_.speed_level = static_cast<int>(data.speed_level);
      state_.software_emergency =
          static_cast<int>(data.software_emergency_status);
      state_.hardware_emergency =
          static_cast<int>(data.hardware_emergency_status);
      state_.forward_speed = data.speed.line;
      state_.lateral_speed = data.speed.translation;
      state_.yaw_speed = data.speed.angle;
      state_.received_at = SteadyClock::now();
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"motion_status\":" << static_cast<int>(data.motion_status)
         << ",\"motion_status_text\":\"" 
         << MotionName(static_cast<int>(data.motion_status)) << "\""
         << ",\"sport_mode\":" << static_cast<int>(data.sport_mode)
         << ",\"control_source\":" << static_cast<int>(data.control_source)
         << ",\"speed_level\":" << static_cast<int>(data.speed_level)
         << ",\"software_emergency_status\":"
         << static_cast<int>(data.software_emergency_status)
         << ",\"hardware_emergency_status\":"
         << static_cast<int>(data.hardware_emergency_status)
         << ",\"battery_power_1\":" << data.battery.power1
         << ",\"battery_power_2\":" << data.battery.power2
         << ",\"forward_speed\":" << data.speed.line
         << ",\"lateral_speed\":" << data.speed.translation
         << ",\"yaw_speed\":" << data.speed.angle << "}";
    message.data = json.str();
    robot_state_pub_->publish(message);
  }

  void HandleFaults(const robot_sdk::FaultDatas& faults) {
    if (faults.empty()) {
      return;
    }
    bool latch = false;
    std_msgs::msg::String message;
    std::ostringstream json;
    json << '[';
    for (std::size_t i = 0; i < faults.size(); ++i) {
      const int level = static_cast<int>(faults[i].level);
      latch = latch || level == kFaultFatal || level == kFaultError;
      if (i != 0) {
        json << ',';
      }
      json << "{\"level\":" << level
           << ",\"code\":" << static_cast<int>(faults[i].code)
           << ",\"message\":\"" << EscapeJson(faults[i].message) << "\"}";
    }
    json << ']';
    message.data = json.str();
    faults_pub_->publish(message);
    RCLCPP_WARN(get_logger(), "Robot fault report: %s", message.data.c_str());
    if (latch) {
      FailPlan("robot reported Error/FatalError", true);
    }
  }

  void HandleControlLost() {
    FailPlan("SDK control ownership lost", false);
    PublishEvent("control_lost");
  }
  void HandleControlAvailable() { PublishEvent("control_available"); }
  void HandleCommandAck(const std::string& command) {
    PublishEvent("ack:" + command + "; waiting for RobotState confirmation");
  }
  void HandleTakeControlAck(const robot_sdk::TakeControlAck& ack) {
    if (ack.error_code != 0U) {
      FailPlan("TakeControl rejected: " + ack.reason, false);
    } else {
      HandleCommandAck("take_control");
    }
  }
  void HandleReleaseControlAck(const robot_sdk::ReleaseControlAck& ack) {
    if (ack.error_code != 0U) {
      FailPlan("ReleaseControl rejected: " + ack.reason, false);
    } else {
      HandleCommandAck("release_control");
    }
  }

 private:
  void CreateServices() {
    take_control_srv_ = create_service<Trigger>(
        "~/take_control", [this](const std::shared_ptr<Trigger::Request>,
                                std::shared_ptr<Trigger::Response> response) {
          QueueSimple({StepKind::kTakeControl}, "take_control", response);
        });
    release_control_srv_ = create_service<Trigger>(
        "~/release_control", [this](const std::shared_ptr<Trigger::Request>,
                                   std::shared_ptr<Trigger::Response> response) {
          QueueReleaseControl(response);
        });
    stand_srv_ = create_service<Trigger>(
        "~/stand", [this](const std::shared_ptr<Trigger::Request>,
                         std::shared_ptr<Trigger::Response> response) {
          QueuePosture("stand", response);
        });
    lie_down_srv_ = create_service<Trigger>(
        "~/lie_down", [this](const std::shared_ptr<Trigger::Request>,
                            std::shared_ptr<Trigger::Response> response) {
          QueuePosture("lie_down", response);
        });
    crawl_srv_ = create_service<Trigger>(
        "~/crawl", [this](const std::shared_ptr<Trigger::Request>,
                         std::shared_ptr<Trigger::Response> response) {
          QueuePosture("crawl", response);
        });
    general_mode_srv_ = create_service<Trigger>(
        "~/general_mode", [this](const std::shared_ptr<Trigger::Request>,
                                std::shared_ptr<Trigger::Response> response) {
          QueueMode(StepKind::kGeneralMode, response);
        });
    in_place_mode_srv_ = create_service<Trigger>(
        "~/in_place_mode", [this](const std::shared_ptr<Trigger::Request>,
                                 std::shared_ptr<Trigger::Response> response) {
          QueueMode(StepKind::kInPlaceMode, response);
        });
    stair_mode_srv_ = create_service<Trigger>(
        "~/stair_mode", [this](const std::shared_ptr<Trigger::Request>,
                              std::shared_ptr<Trigger::Response> response) {
          QueueMode(StepKind::kStairMode, response);
        });
    lock_srv_ = create_service<Trigger>(
        "~/lock", [this](const std::shared_ptr<Trigger::Request>,
                        std::shared_ptr<Trigger::Response> response) {
          const auto state = Snapshot();
          std::vector<StepKind> plan;
          if (CanUseMove(state)) {
            plan.push_back(StepKind::kStop);
          }
          plan.push_back(StepKind::kLock);
          QueueSimple(plan, "lock", response);
        });
    unlock_srv_ = create_service<Trigger>(
        "~/unlock_to_stand",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          const auto state = Snapshot();
          if (!ValidateBehaviorRequest(state, response)) {
            return;
          }
          if (state.motion_status != kMotionLocked) {
            SetResponse(response, true, "robot is not locked");
            return;
          }
          QueueSimple({StepKind::kStand}, "unlock_to_stand", response);
        });
    prepare_navigation_srv_ = create_service<Trigger>(
        "~/prepare_navigation",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          QueuePrepareNavigation(response);
        });
    halt_srv_ = create_service<Trigger>(
        "~/halt", [this](const std::shared_ptr<Trigger::Request>,
                        std::shared_ptr<Trigger::Response> response) {
          const auto state = Snapshot();
          if (!sdk_client_ || !sdk_client_->IsConnected() ||
              state.control_source != kControlSdk || !CanUseMove(state)) {
            SetResponse(response, false,
                        "zero Move is legal only in SDK-controlled GENERAL/STAIR");
            return;
          }
          const auto error = sdk_client_->Move(0.0F, 0.0F, 0.0F, 500);
          SetResponse(response, !error,
                      error ? error.message() : "zero velocity sent");
        });
    soft_estop_srv_ = create_service<SetBool>(
        "~/soft_estop",
        [this](const std::shared_ptr<SetBool::Request> request,
               std::shared_ptr<SetBool::Response> response) {
          if (!sdk_client_ || !sdk_client_->IsConnected()) {
            response->success = false;
            response->message = "SDK disconnected";
            return;
          }
          if (request->data) {
            FailPlan("soft emergency stop requested", false);
          }
          const auto error =
              sdk_client_->SoftEmergencyStop(request->data, command_timeout_ms_);
          response->success = !error;
          response->message =
              error ? error.message()
                    : (request->data ? "soft e-stop requested"
                                     : "soft e-stop recovery requested");
        });
  }

  void QueuePosture(const std::string& target,
                    const std::shared_ptr<Trigger::Response>& response) {
    const auto state = Snapshot();
    if (!ValidateBehaviorRequest(state, response)) {
      return;
    }
    std::vector<StepKind> plan;
    if (target == "stand") {
      if (IsUpright(state.motion_status)) {
        SetResponse(response, true, "robot is already upright");
        return;
      }
      if (state.motion_status != kMotionLieDown &&
          state.motion_status != kMotionCrawl) {
        SetResponse(response, false,
                    "StandUp is legal only from LIE_DOWN or CRAWL");
        return;
      }
      plan.push_back(StepKind::kStand);
    } else if (target == "lie_down") {
      if (state.motion_status == kMotionLieDown) {
        SetResponse(response, true, "robot is already lying down");
        return;
      }
      if (state.motion_status == kMotionCrawl) {
        plan.push_back(StepKind::kStand);
      } else if (!IsUpright(state.motion_status)) {
        SetResponse(response, false, "wait for a stable motion state");
        return;
      } else if (CanUseMove(state)) {
        plan.push_back(StepKind::kStop);
      }
      plan.push_back(StepKind::kLieDown);
    } else {
      if (state.motion_status == kMotionCrawl) {
        SetResponse(response, true, "robot is already crawling");
        return;
      }
      if (state.motion_status == kMotionLieDown ||
          state.motion_status == kMotionGeneral) {
        if (state.motion_status == kMotionGeneral) {
          plan.push_back(StepKind::kStop);
        }
        plan.push_back(StepKind::kCrawl);
      } else if (state.motion_status == kMotionInPlace ||
                 state.motion_status == kMotionStair ||
                 state.motion_status == kMotionStand) {
        if (state.motion_status == kMotionStair) {
          plan.push_back(StepKind::kStop);
        }
        plan.push_back(StepKind::kGeneralMode);
        plan.push_back(StepKind::kCrawl);
      } else {
        SetResponse(response, false,
                    "Crawl is legal only from LIE_DOWN or GENERAL");
        return;
      }
    }
    QueueSimple(plan, target, response);
  }

  void QueueMode(StepKind target,
                 const std::shared_ptr<Trigger::Response>& response) {
    const auto state = Snapshot();
    if (!ValidateBehaviorRequest(state, response)) {
      return;
    }
    if (state.motion_status == kMotionLocked ||
        IsTransientAction(state.motion_status)) {
      SetResponse(response, false,
                  "mode switch blocked from LOCKED/transient action");
      return;
    }
    std::vector<StepKind> plan;
    if (state.motion_status == kMotionLieDown ||
        state.motion_status == kMotionCrawl) {
      plan.push_back(StepKind::kStand);
    } else if (!IsUpright(state.motion_status)) {
      SetResponse(response, false, "unknown motion state");
      return;
    } else if (CanUseMove(state)) {
      plan.push_back(StepKind::kStop);
    }
    plan.push_back(target);
    QueueSimple(plan, StepName(target), response);
  }

  void QueuePrepareNavigation(
      const std::shared_ptr<Trigger::Response>& response) {
    const auto state = Snapshot();
    if (!ValidateCommon(state, response, true)) {
      return;
    }
    if (state.motion_status == kMotionLocked ||
        IsTransientAction(state.motion_status) ||
        state.motion_status == kMotionUnknown) {
      SetResponse(response, false,
                  "navigation blocked from LOCKED/UNKNOWN/transient state");
      return;
    }
    std::vector<StepKind> plan;
    if (state.control_source != kControlSdk) {
      plan.push_back(StepKind::kTakeControl);
    }
    if (CanUseMove(state)) {
      plan.push_back(StepKind::kStop);
    }
    if (state.motion_status == kMotionLieDown ||
        state.motion_status == kMotionCrawl) {
      plan.push_back(StepKind::kStand);
    }
    plan.push_back(StepKind::kGeneralMode);
    plan.push_back(StepKind::kSetNavigationSpeed);
    QueueSimple(plan, "prepare_navigation", response, false);
  }

  void QueueReleaseControl(
      const std::shared_ptr<Trigger::Response>& response) {
    if (!sdk_client_ || !sdk_client_->IsConnected()) {
      SetResponse(response, false, "SDK disconnected");
      return;
    }
    const auto state = Snapshot();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_step_.has_value() || !plan_.empty()) {
        SetResponse(response, false, "another transition is active");
        return;
      }
    }
    if (StateFresh(state) && state.control_source != kControlSdk) {
      SetResponse(response, true, "SDK control is already released");
      return;
    }
    if (StateFresh(state) && EmergencyRecovered(state) &&
        CanUseMove(state)) {
      const auto stop_error = sdk_client_->Move(0.0F, 0.0F, 0.0F, 500);
      if (stop_error) {
        RCLCPP_WARN(get_logger(),
                    "Best-effort zero velocity before ReleaseControl failed: %s",
                    stop_error.message().c_str());
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan_.assign({StepKind::kReleaseControl});
      goal_ = "release_control";
      fsm_state_ = "QUEUED";
      last_error_.clear();
      have_cmd_vel_ = false;
    }
    PublishEvent("goal_accepted:release_control");
    SetResponse(response, true,
                "release accepted even if RobotState is stale or safety is active");
  }

  bool ValidateBehaviorRequest(
      const RobotSnapshot& state,
      const std::shared_ptr<Trigger::Response>& response) {
    if (!ValidateCommon(state, response, true)) {
      return false;
    }
    if (state.control_source != kControlSdk) {
      SetResponse(response, false,
                  "SDK does not own control; call take_control first");
      return false;
    }
    return true;
  }

  bool ValidateCommon(const RobotSnapshot& state,
                      const std::shared_ptr<Trigger::Response>& response,
                      bool require_safe) {
    if (!sdk_client_ || !sdk_client_->IsConnected()) {
      SetResponse(response, false, "SDK disconnected");
      return false;
    }
    if (!StateFresh(state)) {
      SetResponse(response, false, "RobotState missing or stale");
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_step_.has_value() || !plan_.empty()) {
        SetResponse(response, false, "another transition is active");
        return false;
      }
      if (require_safe && fault_latched_) {
        SetResponse(response, false, "fault latch active");
        return false;
      }
    }
    if (require_safe && !EmergencyRecovered(state)) {
      SetResponse(response, false, "hardware/software e-stop not recovered");
      return false;
    }
    return true;
  }

  void QueueSimple(const std::vector<StepKind>& steps,
                   const std::string& goal,
                   const std::shared_ptr<Trigger::Response>& response,
                   bool require_safe = true, bool allow_no_control = false) {
    const auto state = Snapshot();
    if (!ValidateCommon(state, response, require_safe)) {
      return;
    }
    if (!allow_no_control && goal != "take_control" &&
        goal != "prepare_navigation" &&
        state.control_source != kControlSdk) {
      SetResponse(response, false, "SDK does not own control");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan_.assign(steps.begin(), steps.end());
      goal_ = goal;
      fsm_state_ = "QUEUED";
      last_error_.clear();
    }
    PublishEvent("goal_accepted:" + goal);
    SetResponse(response, true,
                "goal accepted; watch behavior_state for completion");
  }

  void RunStateMachine() {
    if (shutting_down_.load() || !sdk_client_ ||
        !sdk_client_->IsConnected()) {
      return;
    }
    const auto state = Snapshot();
    const auto now = SteadyClock::now();
    std::optional<StepKind> issue;
    std::string event;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_step_.has_value() && plan_.empty()) {
        if (fsm_state_ != "FAULT" && fsm_state_ != "ERROR") {
          fsm_state_ = "IDLE";
        }
        return;
      }
      const bool releasing_control = goal_ == "release_control";
      if (!releasing_control && (!state.received ||
          std::chrono::duration<double>(now - state.received_at).count() >
              robot_state_stale_sec_)) {
        CancelPlanLocked("RobotState became stale", false);
        return;
      }
      if (!releasing_control &&
          (fault_latched_ || !EmergencyRecovered(state))) {
        CancelPlanLocked("safety interlock active", fault_latched_);
        return;
      }
      if (active_step_.has_value()) {
        if (StepComplete(*active_step_, state)) {
          event = "transition_complete:" + StepName(*active_step_);
          active_step_.reset();
          if (plan_.empty()) {
            fsm_state_ = "IDLE";
            event += ";goal_complete:" + goal_;
            goal_.clear();
          }
        } else if (std::chrono::duration<double>(
                       now - active_step_started_).count() >
                   transition_timeout_sec_) {
          CancelPlanLocked("transition timeout: " +
                               StepName(*active_step_),
                           false);
        }
      } else {
        while (!plan_.empty() && StepComplete(plan_.front(), state)) {
          event = "transition_already_satisfied:" +
                  StepName(plan_.front());
          plan_.pop_front();
        }
        if (plan_.empty()) {
          fsm_state_ = "IDLE";
          event += ";goal_complete:" + goal_;
          goal_.clear();
        } else {
          issue = plan_.front();
          plan_.pop_front();
          active_step_ = issue;
          active_step_started_ = now;
          fsm_state_ = "TRANSITIONING";
        }
      }
    }
    if (!event.empty()) {
      PublishEvent(event);
    }
    if (issue.has_value()) {
      PublishEvent("transition_start:" + StepName(*issue));
      const auto error = IssueStep(*issue);
      if (error) {
        FailPlan("SDK command failed for " + StepName(*issue) + ": " +
                     error.message(),
                 false);
      }
    }
  }

  std::error_code IssueStep(StepKind step) {
    switch (step) {
      case StepKind::kTakeControl:
        return sdk_client_->TakeControl(command_timeout_ms_);
      case StepKind::kStop:
        return sdk_client_->Move(0.0F, 0.0F, 0.0F, 500);
      case StepKind::kStand:
        return sdk_client_->StandUp(command_timeout_ms_);
      case StepKind::kLieDown:
        return sdk_client_->LieDown(command_timeout_ms_);
      case StepKind::kCrawl:
        return sdk_client_->Crawl(command_timeout_ms_);
      case StepKind::kGeneralMode:
        return sdk_client_->SetMode(kModeGeneral, command_timeout_ms_);
      case StepKind::kInPlaceMode:
        return sdk_client_->SetMode(kModeInPlace, command_timeout_ms_);
      case StepKind::kStairMode:
        return sdk_client_->SetMode(kModeStair, command_timeout_ms_);
      case StepKind::kSetNavigationSpeed:
        return sdk_client_->SetSpeed(navigation_speed_level_,
                                     command_timeout_ms_);
      case StepKind::kLock:
        return sdk_client_->Locked(command_timeout_ms_);
      case StepKind::kReleaseControl:
        return sdk_client_->ReleaseControl(command_timeout_ms_);
    }
    return std::make_error_code(std::errc::invalid_argument);
  }

  bool StepComplete(StepKind step, const RobotSnapshot& state) const {
    switch (step) {
      case StepKind::kTakeControl:
        return state.control_source == kControlSdk;
      case StepKind::kStop:
        return std::abs(state.forward_speed) < 0.03 &&
               std::abs(state.lateral_speed) < 0.03 &&
               std::abs(state.yaw_speed) < 0.03;
      case StepKind::kStand:
        return IsUpright(state.motion_status);
      case StepKind::kLieDown:
        return state.motion_status == kMotionLieDown;
      case StepKind::kCrawl:
        return state.motion_status == kMotionCrawl;
      case StepKind::kGeneralMode:
        return state.sport_mode == kModeGeneral &&
               state.motion_status == kMotionGeneral;
      case StepKind::kInPlaceMode:
        return state.sport_mode == kModeInPlace &&
               state.motion_status == kMotionInPlace;
      case StepKind::kStairMode:
        return state.sport_mode == kModeStair &&
               state.motion_status == kMotionStair;
      case StepKind::kSetNavigationSpeed:
        return state.speed_level == navigation_speed_level_;
      case StepKind::kLock:
        return state.motion_status == kMotionLocked;
      case StepKind::kReleaseControl:
        return StateFresh(state) && state.control_source != kControlSdk;
    }
    return false;
  }

  void RunVelocityGate() {
    if (shutting_down_.load() || !sdk_client_ ||
        !sdk_client_->IsConnected()) {
      return;
    }
    const auto state = Snapshot();
    geometry_msgs::msg::Twist command;
    bool send = false;
    bool send_zero = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool ready = ReadyForNavigationLocked(state);
      const bool fresh =
          have_cmd_vel_ &&
          std::chrono::duration<double>(
              SteadyClock::now() - latest_cmd_vel_at_).count() <=
              cmd_vel_timeout_sec_;
      if (ready && fresh) {
        command = latest_cmd_vel_;
        send = true;
        zero_velocity_sent_ = false;
      } else if (CanUseMove(state) &&
                 state.control_source == kControlSdk &&
                 (!zero_velocity_sent_ || zero_required_)) {
        send_zero = true;
        zero_velocity_sent_ = true;
        zero_required_ = false;
      }
    }
    if (send) {
      const float left_right = static_cast<float>(std::clamp(
          command.linear.y, -max_lateral_speed_, max_lateral_speed_));
      const float forward_back = static_cast<float>(std::clamp(
          command.linear.x, -max_forward_speed_, max_forward_speed_));
      const float yaw = static_cast<float>(std::clamp(
          command.angular.z, -max_yaw_speed_, max_yaw_speed_));
      const auto error =
          sdk_client_->Move(left_right, forward_back, yaw);
      if (error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "SDK Move failed: %s",
                              error.message().c_str());
      }
    } else if (send_zero) {
      sdk_client_->Move(0.0F, 0.0F, 0.0F);
    }
  }

  void PublishStatus() {
    const auto state = Snapshot();
    std::string fsm;
    std::string goal;
    std::string active;
    std::string error;
    std::size_t queued = 0;
    bool fault = false;
    bool ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fsm = fsm_state_;
      goal = goal_;
      active =
          active_step_.has_value() ? StepName(*active_step_) : "none";
      error = last_error_;
      queued = plan_.size();
      fault = fault_latched_;
      ready = ReadyForNavigationLocked(state);
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"fsm_state\":\"" << fsm << "\""
         << ",\"observed_state\":\"" << MotionName(state.motion_status)
         << "\",\"goal\":\"" << EscapeJson(goal)
         << "\",\"active_transition\":\"" << active
         << "\",\"queued_steps\":" << queued
         << ",\"sdk_has_control\":"
         << (state.control_source == kControlSdk ? "true" : "false")
         << ",\"fault_latched\":" << (fault ? "true" : "false")
         << ",\"ready_for_navigation\":"
         << (ready ? "true" : "false")
         << ",\"last_error\":\"" << EscapeJson(error) << "\"}";
    message.data = json.str();
    behavior_state_pub_->publish(message);
    std_msgs::msg::Bool ready_message;
    ready_message.data = ready;
    ready_pub_->publish(ready_message);
  }

  void PublishConnectionState() {
    if (!sdk_client_) {
      return;
    }
    const auto state = sdk_client_->GetConnectionState();
    std_msgs::msg::UInt8 numeric;
    numeric.data = static_cast<std::uint8_t>(state);
    connection_state_pub_->publish(numeric);
    std_msgs::msg::String text;
    text.data = ConnectionName(state);
    connection_text_pub_->publish(text);
    if (state != last_connection_state_) {
      RCLCPP_INFO(get_logger(), "SDK connection state: %s",
                  text.data.c_str());
      last_connection_state_ = state;
      if (state != robot_sdk::ConnectionState::CONNECTED) {
        FailPlan("SDK connection lost", false);
      }
    }
  }

  void FailPlan(const std::string& reason, bool fault) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      CancelPlanLocked(reason, fault);
    }
    PublishEvent((fault ? "fault:" : "transition_error:") + reason);
  }
  void CancelPlanLocked(const std::string& reason, bool fault) {
    plan_.clear();
    active_step_.reset();
    goal_.clear();
    last_error_ = reason;
    fault_latched_ = fault_latched_ || fault;
    fsm_state_ = fault_latched_ ? "FAULT" : "ERROR";
    zero_required_ = true;
    have_cmd_vel_ = false;
  }
  void PublishEvent(const std::string& event) {
    std_msgs::msg::String message;
    message.data = event;
    transition_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", event.c_str());
  }
  RobotSnapshot Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }
  bool ReadyForNavigationLocked(const RobotSnapshot& state) const {
    return fsm_state_ == "IDLE" && last_error_.empty() &&
           !fault_latched_ && !active_step_.has_value() && plan_.empty() &&
           StateFresh(state) && EmergencyRecovered(state) &&
           state.control_source == kControlSdk &&
           state.motion_status == kMotionGeneral &&
           state.sport_mode == kModeGeneral &&
           state.speed_level == navigation_speed_level_;
  }
  bool StateFresh(const RobotSnapshot& state) const {
    return state.received &&
           std::chrono::duration<double>(
               SteadyClock::now() - state.received_at).count() <=
               robot_state_stale_sec_;
  }
  static bool EmergencyRecovered(const RobotSnapshot& state) {
    return state.software_emergency == kEmergencyRecover &&
           state.hardware_emergency == kEmergencyRecover;
  }
  static bool IsUpright(int motion) {
    return motion == kMotionStand || motion == kMotionGeneral ||
           motion == kMotionInPlace || motion == kMotionStair;
  }
  static bool IsTransientAction(int motion) {
    return motion == kMotionClimb || motion == kMotionSlim ||
           motion == kMotionGait;
  }
  static bool CanUseMove(const RobotSnapshot& state) {
    return state.motion_status == kMotionGeneral ||
           state.motion_status == kMotionStair;
  }
  static const char* MotionName(int motion) {
    switch (motion) {
      case kMotionStand: return "STAND_UP";
      case kMotionLieDown: return "LIE_DOWN";
      case kMotionCrawl: return "CRAWL";
      case kMotionLocked: return "LOCKED";
      case kMotionGeneral: return "GENERAL";
      case kMotionInPlace: return "IN_PLACE";
      case kMotionStair: return "STAIR";
      case kMotionClimb: return "CLIMB";
      case kMotionSlim: return "SLIM";
      case kMotionGait: return "GAIT";
      default: return "UNKNOWN";
    }
  }
  static std::string StepName(StepKind step) {
    switch (step) {
      case StepKind::kTakeControl: return "TAKE_CONTROL";
      case StepKind::kStop: return "STOP";
      case StepKind::kStand: return "STAND";
      case StepKind::kLieDown: return "LIE_DOWN";
      case StepKind::kCrawl: return "CRAWL";
      case StepKind::kGeneralMode: return "GENERAL_MODE";
      case StepKind::kInPlaceMode: return "IN_PLACE_MODE";
      case StepKind::kStairMode: return "STAIR_MODE";
      case StepKind::kSetNavigationSpeed: return "SET_NAVIGATION_SPEED";
      case StepKind::kLock: return "LOCK";
      case StepKind::kReleaseControl: return "RELEASE_CONTROL";
    }
    return "UNKNOWN";
  }
  static const char* ConnectionName(robot_sdk::ConnectionState state) {
    switch (state) {
      case robot_sdk::ConnectionState::DISCONNECTING: return "disconnecting";
      case robot_sdk::ConnectionState::DISCONNECTED: return "disconnected";
      case robot_sdk::ConnectionState::CONNECTING: return "connecting";
      case robot_sdk::ConnectionState::HANDSHAKING: return "handshaking";
      case robot_sdk::ConnectionState::CONNECTED: return "connected";
      case robot_sdk::ConnectionState::RECONNECTING: return "reconnecting";
    }
    return "unknown";
  }
  static std::string EscapeJson(const std::string& input) {
    std::string output;
    output.reserve(input.size());
    for (const char character : input) {
      if (character == '\\' || character == '\"') {
        output.push_back('\\');
      }
      output.push_back(character);
    }
    return output;
  }
  static void SetResponse(
      const std::shared_ptr<Trigger::Response>& response, bool success,
      const std::string& message) {
    response->success = success;
    response->message = message;
  }

  std::string robot_ip_;
  int robot_port_{8081};
  int command_timeout_ms_{3000};
  double transition_timeout_sec_{15.0};
  double robot_state_stale_sec_{2.5};
  std::string cmd_vel_topic_{"/cmd_vel"};
  double cmd_vel_timeout_sec_{0.30};
  double cmd_vel_rate_hz_{20.0};
  double max_forward_speed_{0.45};
  double max_lateral_speed_{0.35};
  double max_yaw_speed_{0.80};
  int navigation_speed_level_{1};

  mutable std::mutex mutex_;
  RobotSnapshot state_;
  std::deque<StepKind> plan_;
  std::optional<StepKind> active_step_;
  SteadyClock::time_point active_step_started_{};
  std::string fsm_state_{"OBSERVING"};
  std::string goal_;
  std::string last_error_;
  bool fault_latched_{false};
  bool zero_required_{false};
  bool zero_velocity_sent_{true};
  bool have_cmd_vel_{false};
  geometry_msgs::msg::Twist latest_cmd_vel_;
  SteadyClock::time_point latest_cmd_vel_at_{};
  std::atomic<bool> shutting_down_{false};
  robot_sdk::ConnectionState last_connection_state_{
      robot_sdk::ConnectionState::DISCONNECTED};

  std::unique_ptr<robot_sdk::SDKClient> sdk_client_;
  std::shared_ptr<BridgeDataCallback> data_callback_;
  std::shared_ptr<BridgeControlCallback> control_callback_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr faults_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr transition_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr connection_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr connection_text_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;
  rclcpp::TimerBase::SharedPtr velocity_timer_;
  rclcpp::Service<Trigger>::SharedPtr take_control_srv_;
  rclcpp::Service<Trigger>::SharedPtr release_control_srv_;
  rclcpp::Service<Trigger>::SharedPtr stand_srv_;
  rclcpp::Service<Trigger>::SharedPtr lie_down_srv_;
  rclcpp::Service<Trigger>::SharedPtr crawl_srv_;
  rclcpp::Service<Trigger>::SharedPtr general_mode_srv_;
  rclcpp::Service<Trigger>::SharedPtr in_place_mode_srv_;
  rclcpp::Service<Trigger>::SharedPtr stair_mode_srv_;
  rclcpp::Service<Trigger>::SharedPtr lock_srv_;
  rclcpp::Service<Trigger>::SharedPtr unlock_srv_;
  rclcpp::Service<Trigger>::SharedPtr prepare_navigation_srv_;
  rclcpp::Service<Trigger>::SharedPtr halt_srv_;
  rclcpp::Service<SetBool>::SharedPtr soft_estop_srv_;
};

void BridgeDataCallback::OnRobotStateData(
    const robot_sdk::RobotState& data) {
  bridge_->UpdateRobotState(data);
}
void BridgeDataCallback::OnFaultData(
    const robot_sdk::FaultDatas& data) {
  bridge_->HandleFaults(data);
}
void BridgeDataCallback::OnControlLost(
    const robot_sdk::ControlLostInfo&) {
  bridge_->HandleControlLost();
}
void BridgeDataCallback::OnControlAvailable(
    const robot_sdk::ControlAvailableInfo&) {
  bridge_->HandleControlAvailable();
}
void BridgeControlCallback::OnStandUp() {
  bridge_->HandleCommandAck("stand");
}
void BridgeControlCallback::OnLieDown() {
  bridge_->HandleCommandAck("lie_down");
}
void BridgeControlCallback::OnCrawl() {
  bridge_->HandleCommandAck("crawl");
}
void BridgeControlCallback::OnMode(int mode) {
  bridge_->HandleCommandAck("mode:" + std::to_string(mode));
}
void BridgeControlCallback::OnSpeed(int speed) {
  bridge_->HandleCommandAck("speed:" + std::to_string(speed));
}
void BridgeControlCallback::OnLocked() {
  bridge_->HandleCommandAck("lock");
}
void BridgeControlCallback::OnTakeControlAck(
    const robot_sdk::TakeControlAck& ack) {
  bridge_->HandleTakeControlAck(ack);
}
void BridgeControlCallback::OnReleaseControlAck(
    const robot_sdk::ReleaseControlAck& ack) {
  bridge_->HandleReleaseControlAck(ack);
}

}  // namespace d1max_sdk_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<d1max_sdk_bridge::SdkBridge>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("d1max_sdk_bridge"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
