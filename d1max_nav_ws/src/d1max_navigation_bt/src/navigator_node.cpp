#include "d1max_navigation_bt/engine.hpp"
#include "d1max_navigation_bt/runtime_policy.hpp"
#include "d1max_navigation_bt/task_contract.hpp"
#include "d1max_navigation_bt/route_contract.hpp"
#include "d1max_navigation_bt/execution_owner.hpp"
#include "d1max_navigation_bt/body_pair_admission.hpp"
#include "d1max_navigation_bt/record_binding.hpp"
#include "d1max_navigation_bt/preparation_budget.hpp"
#include "d1max_navigation_bt/initial_pose_transaction.hpp"
#include "d1max_navigation_bt/dependency_health.hpp"
#include <d1max_planning_interfaces/msg/navigation_state.hpp>
#include <d1max_planning_interfaces/msg/reference_proposal.hpp>
#include <d1max_planning_interfaces/srv/execution_grant.hpp>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <d1max_navigation_bt_interfaces/action/compute_route.hpp>
#include <d1max_navigation_bt_interfaces/action/follow_route.hpp>
#include <d1max_navigation_bt_interfaces/action/navigate.hpp>
#include <d1max_navigation_bt_interfaces/msg/navigation_health.hpp>
#include <d1max_navigation_bt_interfaces/srv/confirm_execution.hpp>
#include <d1max_navigation_bt_interfaces/srv/prepare_transition.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <map>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace d1max_navigation_bt {
namespace {
using Clock = std::chrono::steady_clock;
using Compute = d1max_navigation_bt_interfaces::action::ComputeRoute;
using Follow = d1max_navigation_bt_interfaces::action::FollowRoute;
using Navigate = d1max_navigation_bt_interfaces::action::Navigate;
using Health = d1max_navigation_bt_interfaces::msg::NavigationHealth;
using RouteSnapshot = d1max_navigation_bt_interfaces::msg::RouteSnapshot;
using ConfirmExecution = d1max_navigation_bt_interfaces::srv::ConfirmExecution;
using PrepareTransition = d1max_navigation_bt_interfaces::srv::PrepareTransition;
using PrepareInitialPose = d1max_navigation_bt_interfaces::srv::PrepareInitialPose;
using InitialPoseOutcome = d1max_navigation_bt_interfaces::msg::InitialPoseOutcome;
using ComputeHandle = rclcpp_action::ClientGoalHandle<Compute>;
using FollowHandle = rclcpp_action::ClientGoalHandle<Follow>;
using NavigateHandle = rclcpp_action::ServerGoalHandle<Navigate>;
constexpr const char* prefix = "/d1max/live_planning/";

double elapsed(Clock::time_point since) {
  return std::chrono::duration<double>(Clock::now() - since).count();
}

// Goal acceptance, result receipt and cancellation are separate states. In
// particular, a cancel ACK does not prove the old worker has stopped.
template<class Handle>
struct Operation : AsyncRequestState {
  typename Handle::SharedPtr handle;
  Result result{BT::NodeStatus::RUNNING, "not_requested"};
  Clock::time_point requested_at{};
  Clock::time_point feedback_at{};
  // Compute-only stage clock; first admitted work phase wins permanently.
  std::optional<Clock::time_point> work_started;
};

struct Task {
  TaskIdentity identity;
  std::string goal_kind;
  geometry_msgs::msg::PoseStamped goal;
  bool has_goal_yaw{false};
  double goal_yaw_tolerance{.15};
  ConfirmationLedger confirmation;
  std::unique_ptr<exec3::Owner> execution;
  exec3::GoalLedger execution_goals;
  std::unique_ptr<PreparationBudget> preparation;
  std::string follow_phase{"waiting_reference"},follow_reason{"waiting_follow_feedback"};
  std::shared_ptr<NavigateHandle> action;
  Operation<ComputeHandle> compute;
  Operation<FollowHandle> follow;
  Compute::Result route;
  std::int64_t epoch{0};
  std::string seed;
  bool bound{false};
  bool retired{false};
  bool finished{false};
  bool public_terminal{false};
  bool finish_success{false};
  bool cancellation_requested{false};
  bool paused{false};
  bool arrived{false};
  std::int64_t measured_arrival_stamp{0};
  Clock::time_point arrival_received_at{};
  Clock::time_point retired_at{};
  std::string phase{"accepted"};
  std::string reason;
  std::string finish_reason;
  std::string retirement_error;
  std::string transport_fault;
  bool pending() const { return compute.pending() || follow.pending(); }
};
}  // namespace

class NavigatorNode final : public nav2_util::LifecycleNode, public Backend {
public:
  NavigatorNode() : nav2_util::LifecycleNode("d1max_navigation_bt") {
    session_ = declare_parameter<std::string>("session_id", "");
    frame_ = declare_parameter<std::string>("frame_id", "d1max_loc_map");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "d1max_multifloor_planning");
    health_timeout_ = declare_parameter<double>("health_timeout_s", 0.6);
    cancellation_timeout_ = declare_parameter<double>("cancellation_timeout_s", 3.0);
    action_watchdog_timeout_ = declare_parameter<double>("action_watchdog_timeout_s", 3.0);
    compute_route_timeout_ = declare_parameter<double>("compute_route_timeout_s",10.);
    worker_startup_timeout_ = declare_parameter<double>("worker_startup_timeout_s",60.);
    initial_localization_map_sha_=declare_parameter<std::string>("initial_localization_map_sha256","");
    initial_pose_transaction_=std::make_unique<InitialPoseTransaction>(session_,frame_,initial_localization_map_sha_);
    dependency_health_=DependencyHealthGuard(declare_parameter<bool>("dependency_health_required",false));
    arrival_evidence_timeout_ = declare_parameter<double>("arrival_evidence_timeout_s", 4.0);
    execution_blocked_timeout_ = declare_parameter<double>("execution_blocked_timeout_s",30.);
    execution_recovery_time_ = declare_parameter<double>("execution_recovery_time_s",1.);
    stationary_limits_.linear=declare_parameter<double>("stationary_linear_threshold_mps",.03);
    stationary_limits_.angular=declare_parameter<double>("stationary_angular_threshold_radps",.05);
    stationary_limits_.stop_duration=declare_parameter<double>("stationary_stop_duration_s",1.);
    stationary_limits_.reentry_duration=declare_parameter<double>("stationary_reentry_duration_s",.6);
    const int stationary_samples=declare_parameter<int>("stationary_minimum_samples",3);
    if(stationary_samples<3||stationary_samples>1000)throw std::invalid_argument("bounded_stationary_minimum_samples_required");
    stationary_limits_.minimum_samples=static_cast<unsigned>(stationary_samples);
    // Validate before accepting any task, not only after the first route.
    exec3::Owner("live",execution_blocked_timeout_,execution_recovery_time_,stationary_limits_);
    preparation_limits_.waits={declare_parameter<double>("follow_pose_wait_s",30.),
      declare_parameter<double>("follow_map_wait_s",15.),declare_parameter<double>("follow_reference_wait_s",5.),
      declare_parameter<double>("follow_trajectory_wait_s",20.)};
    preparation_limits_.episode=declare_parameter<double>("follow_recovery_episode_s",30.);
    preparation_limits_.stable=declare_parameter<double>("follow_stable_reset_s",.6);
    preparation_limits_.validate();
    const double goal_source_timeout=declare_parameter<double>("goal_source_timeout_s",5.0);
    if (!std::isfinite(goal_source_timeout) || goal_source_timeout<=0. || goal_source_timeout>30.)
      throw std::invalid_argument("goal_source_timeout_s must be in (0,30]");
    goal_admission_=GoalAdmission(static_cast<std::int64_t>(goal_source_timeout*1e9));
    const double tick_hz = declare_parameter<double>("tick_hz", 10.0);
    preview_only_ = declare_parameter<bool>("preview_only", true);
    execution_purpose_=declare_parameter<std::string>("execution_purpose","execution");
    transport_mode_ = declare_parameter<std::string>("execution_transport_mode", "live");
    expected_sdk_session_ = declare_parameter<std::string>("expected_sdk_session", "");
    // Deprecated compatibility setting cannot grant permission.
    declare_parameter<bool>("execution_live_verified", false);
    acceptance_record_=declare_parameter<std::string>("execution_acceptance_record", "");
    record_sha256_=declare_parameter<std::string>("execution_record_sha256", "");
    record_bound_=recordHashMatches(acceptance_record_,record_sha256_);
    if(transport_mode_!="live"&&transport_mode_!="isolated_mock")throw std::invalid_argument("unknown_execution_transport");
    if(!validExecutionPurpose(execution_purpose_,preview_only_,transport_mode_))
      throw std::invalid_argument("invalid_execution_purpose");
    if(transport_mode_=="isolated_mock") {
      const auto env=[](const char*key){const auto*v=std::getenv(key);return v?std::string(v):std::string();};
      if(env("D1MAX_OFFLINE_ZENOH_TEST")!="1"||env("D1MAX_NAV_ISOLATED")!="1"||
         env("D1MAX_NAV_TRANSPORT")!="isolated_mock"||env("D1MAX_NAV_ISOLATION_TOKEN").size()!=32)
        throw std::invalid_argument("isolated_mock_requires_private_test_environment");
    }
    const auto xml = declare_parameter<std::string>("tree_xml",
      ament_index_cpp::get_package_share_directory("d1max_navigation_bt") +
      "/trees/navigate_committed_route.xml");
    if (!validSessionId(session_) || (!preview_only_&&expected_sdk_session_.empty()) || !std::isfinite(tick_hz) || tick_hz < 1.0 ||
        tick_hz > 50.0 || !std::isfinite(health_timeout_) || health_timeout_ <= 0.0 ||
        !std::isfinite(cancellation_timeout_) || cancellation_timeout_ <= 0.0 ||
        !std::isfinite(action_watchdog_timeout_) || action_watchdog_timeout_ <= 0.0 ||
        !std::isfinite(compute_route_timeout_) || compute_route_timeout_<5. || compute_route_timeout_>60. ||
        !std::isfinite(worker_startup_timeout_) || worker_startup_timeout_<10. || worker_startup_timeout_>60. ||
        !std::isfinite(arrival_evidence_timeout_) || arrival_evidence_timeout_ <= 0.0) {
      throw std::invalid_argument("A session and bounded preview-only BT configuration are required");
    }

    compute_client_ = rclcpp_action::create_client<Compute>(this, std::string(prefix)+"bt/compute_route");
    follow_client_ = rclcpp_action::create_client<Follow>(this, std::string(prefix)+"bt/follow_route");
    if(permitsMotionAuthority(execution_purpose_,preview_only_))
      execution_grant_=create_client<d1max_planning_interfaces::srv::ExecutionGrant>(std::string(prefix)+"execution/grant");
    execution_pub_=rclcpp::create_publisher<exec3::Permit>(*this,std::string(prefix)+"execution/permit",rclcpp::QoS(1).reliable());
    handoff_pub_=rclcpp::create_publisher<exec3::Handoff>(*this,std::string(prefix)+"execution/handoff_grant",rclcpp::QoS(1).reliable());
    commit_ack_sub_=create_subscription<exec3::CommitAck>(std::string(prefix)+"execution/commit_ack",10,
      [this](exec3::CommitAck::SharedPtr m){if(current_&&current_->execution)current_->execution->observe(*m,sourceNow());});
    stationary_sub_=create_subscription<exec3::Stationary>(std::string(prefix)+"execution/stationary_evidence",10,
      [this](exec3::Stationary::SharedPtr m){if(current_&&current_->execution&&current_->execution->observe(*m,sourceNow()))tryPrepareReadyHandoff();});
    geometry_receipt_sub_=create_subscription<exec3::GeometryReceipt>(std::string(prefix)+"execution/tracker_geometry_receipt",10,
      [this](exec3::GeometryReceipt::SharedPtr m){if(current_&&current_->execution)current_->execution->observe(*m,sourceNow());});
    route_progress_sub_=create_subscription<exec3::RouteProgress>(std::string(prefix)+"execution/route_progress",10,
      [this](exec3::RouteProgress::SharedPtr m){if(current_&&current_->execution)current_->execution->observe(*m,sourceNow());});
    validation_sub_=create_subscription<exec3::Validation>(std::string(prefix)+"execution/validation",10,
      [this](exec3::Validation::SharedPtr m){if(current_&&current_->execution&&current_->execution->observe(*m,sourceNow()))tryPrepareReadyHandoff();});
    admission_sub_=create_subscription<exec3::Admission>(std::string(prefix)+"execution/admission",10,
      [this](exec3::Admission::SharedPtr m){if(current_&&current_->execution&&current_->execution->observe(*m,sourceNow()))tryPrepareReadyHandoff();});
    safe_demand_sub_=create_subscription<exec3::MotionDemand>(std::string(prefix)+"execution/safe_demand",10,
      [this](exec3::MotionDemand::SharedPtr m){if(current_&&current_->execution)current_->execution->observe(*m,sourceNow());});
    motion_validation_sub_=create_subscription<exec3::MotionValidation>(std::string(prefix)+"execution/motion_validation",10,
      [this](exec3::MotionValidation::SharedPtr m){if(current_&&current_->execution)current_->execution->observe(*m,sourceNow());});
    receipt_sub_=create_subscription<exec3::Receipt>(std::string(prefix)+"execution/reference_receipt",10,
      [this](exec3::Receipt::SharedPtr m){if(current_&&current_->execution&&current_->execution->observe(*m,sourceNow()))tryPrepareReadyHandoff();});
    sdk_state_sub_=create_subscription<exec3::SDKState>(std::string(prefix)+"execution/sdk_state",10,
      [this](exec3::SDKState::SharedPtr m){if(m->sdk_session!=expected_sdk_session_)return;
        for(auto&t:tasks_)if(t.second->execution)t.second->execution->observe(*m,sourceNow());});
    stop_report_sub_=create_subscription<exec3::Stop>(std::string(prefix)+"execution/stop_report",10,
      [this](exec3::Stop::SharedPtr m){for(auto&t:tasks_)if(t.second->execution)t.second->execution->observe(*m,sourceNow());});
    navigation_state_sub_=create_subscription<d1max_planning_interfaces::msg::NavigationState>(
      "/d1max/localization/navigation/state",5,[this](d1max_planning_interfaces::msg::NavigationState::SharedPtr m){
        if(acceptBodyPair(*m,body_state_,session_,health_.map_version_id,now().seconds()))body_state_=*m;});
    local_state_sub_=create_subscription<d1max_planning_interfaces::msg::LocalNavigationState>(
      "/d1max/localization/navigation/local_state",5,[this](d1max_planning_interfaces::msg::LocalNavigationState::SharedPtr m){
        if(!acceptLocalState(*m,local_state_,session_,health_.map_version_id,now().seconds()))return;
        local_state_=*m;
        for(auto&entry:tasks_) {
          auto&t=entry.second;
          if(t->execution)t->execution->observeBody(*m);
          if(!t->bound||t->retired)continue;
          const bool changed=t->epoch!=static_cast<int64_t>(m->localization_epoch)||t->seed!=m->localization_seed_id;
          const bool hard=!m->usable&&(m->reason=="hard_localization_lost"||m->reason=="lio_reset"||m->reason=="localization_reset");
          if(changed||hard) {
            t->transport_fault=changed?"localization_context_replaced":"hard_localization_lost";
            t->confirmation.revoke();if(t->execution)t->execution->stop(t->transport_fault,sourceNow());
          }
        }});
    startup_status_sub_=create_subscription<std_msgs::msg::String>(
      "/d1max/localization/global_relocalization/status",5,[this](std_msgs::msg::String::SharedPtr m){
        try {
          const auto value=nlohmann::json::parse(m->data);
          if(!value.at("schema").is_number_unsigned()||value.at("schema").get<std::uint64_t>()!=1||
             !value.at("status_sequence").is_number_unsigned()||
             !value.at("local_epoch").is_number_integer())return;
          const auto index=value.value("index",nlohmann::json::object());
          const bool prepared=index.is_object()&&index.value("kind",std::string{})=="index_ready"&&
            index.value("map_sha256",std::string{})==initial_localization_map_sha_;
          startup_evidence_.observe(value.at("schema").get<unsigned>(),value.at("session_id").get<std::string>(),
            value.at("requested_map_sha256").get<std::string>(),value.at("status_sequence").get<std::uint64_t>(),
            value.at("local_epoch").get<std::int64_t>(),value.at("state").get<std::string>(),
            value.at("reason").get<std::string>(),prepared,value.at("received_at_unix").get<double>(),
            now().seconds(),Clock::now(),session_,initial_localization_map_sha_);
        } catch(const nlohmann::json::exception&) {}  // An invalid report grants no time or readiness.
      });
    proposal_sub_=create_subscription<d1max_planning_interfaces::msg::ReferenceProposal>(std::string(prefix)+"execution/reference_proposal",5,
      [this](d1max_planning_interfaces::msg::ReferenceProposal::SharedPtr m){if(m->version.session_id==session_&&
        current_&&m->version.task_id==current_->identity.task_id&&m->transport_mode==transport_mode_&&
        exec3::fresh(exec3::seconds(m->source_stamp),now().seconds(),.5)) {
          if(current_->execution_goals.observe(*m))tryPrepareReadyHandoff();
        }});
    // Engine is destroyed before Node and never owns/deletes its backend.
    engine_ = std::make_unique<Engine>(std::shared_ptr<Backend>(this, [](Backend*) {}), xml);
    // Drain diagnostics must remain available while the lifecycle is inactive.
    // This node owns permission, never direct SDK velocity writes.
    status_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(*this, std::string(prefix)+"bt/status",
      rclcpp::QoS(1).reliable().transient_local());
    snapshot_pub_ = rclcpp::create_publisher<RouteSnapshot>(*this, std::string(prefix)+"bt/route_snapshot",
      rclcpp::QoS(1).reliable().transient_local());
    confirm_service_ = create_service<ConfirmExecution>(std::string(prefix)+"bt/confirm_execution",
      [this](const std::shared_ptr<ConfirmExecution::Request> request,
             std::shared_ptr<ConfirmExecution::Response> response) { confirm(*request, *response); });
    prepare_service_ = create_service<PrepareTransition>(std::string(prefix)+"bt/prepare_transition",
      [this](const std::shared_ptr<PrepareTransition::Request> request,
             std::shared_ptr<PrepareTransition::Response> response) {
        response->schema_version = task_schema_version;
        response->reason=transitionAdmission(request->schema_version,request->session_id,session_);
        if (!response->reason.empty()) return;
        beginDrain(request->reason.empty() ? "lifecycle_transition_requested" : request->reason);
        response->accepted = true;
        response->drained = !lifecycle_.draining() && !lifecycle_.quarantined();
        response->physical_stop_confirmed = physicalStopConfirmed();
        response->reason = response->drained ? (response->physical_stop_confirmed?"workers_retired_and_physical_stop_confirmed":
          transport_mode_=="isolated_mock"&&measuredStopConfirmed()?"workers_retired_and_mock_measured_stop_confirmed":"preview_workers_retired_not_physical_stop") : "retirement_pending";
      });
    health_sub_ = create_subscription<Health>(std::string(prefix)+"bt/health", 5,
      [this](Health::SharedPtr message) { onHealth(*message); });
    dependency_sub_=create_subscription<std_msgs::msg::String>(std::string(prefix)+"component_health",5,
      [this](std_msgs::msg::String::SharedPtr m) {
        try {
          const auto v=nlohmann::json::parse(m->data);
          if(!v.at("schema").is_number_unsigned()||!v.at("sequence").is_number_unsigned()||
              !v.at("ready").is_boolean()||!v.at("fatal").is_boolean()||!v.at("observed_at_unix").is_number())return;
          dependency_health_.observe(v.at("schema").get<unsigned>(),v.at("session_id").get<std::string>(),
            v.at("sequence").get<uint64_t>(),v.at("ready").get<bool>(),v.at("fatal").get<bool>(),
            v.at("reason").get<std::string>(),v.at("observed_at_unix").get<double>(),wallTime(),monotonic(),session_);
        }catch(const nlohmann::json::exception&){}
      });
    initial_pose_service_=create_service<PrepareInitialPose>(std::string(prefix)+"bt/prepare_initial_pose",
      [this](const std::shared_ptr<PrepareInitialPose::Request> r,std::shared_ptr<PrepareInitialPose::Response> out) {
        bool physical_required=false;
        for(const auto&entry:tasks_)physical_required|=entry.second->execution&&entry.second->execution->confirmed();
        const bool retired=retiredOperationsDone()&&(!current_||current_->retired||current_->finished);
        const auto d=initial_pose_transaction_->request(*r,now().nanoseconds(),monotonic(),
          lifecycle_.accepting()&&retirement_fault_.empty()&&dependency_health_.blocker(monotonic()).empty(),
          retired,physical_required,measuredStopConfirmed(),transport_mode_=="live",
          local_state_.localization_epoch,local_state_.localization_seed_id);
        if(d.begin_retirement) {
          goal_admission_.fenceAll(now().nanoseconds());
          for(auto&entry:tasks_)entry.second->confirmation.revoke();
          engine_->cancel("validated_initial_pose_requested:"+r->request_id);
        }
        out->schema_version=1;out->accepted=d.accepted;out->ready=d.ready;
        out->application_authorized=d.application_authorized;
        out->physical_stop_confirmed=d.physical_stop_confirmed;out->reason=d.reason;
        publishStatus();
      });
    initial_outcome_sub_=create_subscription<InitialPoseOutcome>("/d1max/localization/initial_pose/outcome",
      rclcpp::QoS(1).reliable().transient_local(),[this](InitialPoseOutcome::SharedPtr o) {
        initial_pose_transaction_->outcome(*o,now().nanoseconds(),monotonic());
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(std::string(prefix)+"goal", 5,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr goal) { acceptTopicGoal("2d", *goal); });
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(std::string(prefix)+"goal3d", 5,
      [this](geometry_msgs::msg::PointStamped::SharedPtr point) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header = point->header;
        goal.pose.position = point->point;
        goal.pose.orientation.w = 1.0;
        acceptTopicGoal("3d", goal);
      });
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(std::string(prefix)+"cancel", 5,
      [this](std_msgs::msg::Empty::SharedPtr) { cancel("user_cancelled"); });
    navigate_server_ = rclcpp_action::create_server<Navigate>(this, std::string(prefix)+"bt/navigate",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Navigate::Goal> goal) {
        if (!(navigationAdmissionReason().empty() && goal->schema_version == task_schema_version &&
          validateYaw(goal->has_goal_yaw, goal->goal_yaw_tolerance_rad) &&
          validateGoal(goal->goal_kind, goal->goal))) return rclcpp_action::GoalResponse::REJECT;
        const auto reason=goal_admission_.reserve(goalStamp(goal->goal),now().nanoseconds());
        if (!reason.empty()) {
          RCLCPP_WARN(get_logger(),"Rejected action goal: %s",reason.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<NavigateHandle> handle) {
        // rclcpp_action enters CANCELING only after this callback returns.
        // Defer completion until the next executor tick so canceled(result)
        // follows the action protocol, rather than aborting an accepted cancel.
        if (!current_ || current_->action != handle || current_->finished || !handle->is_active()) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        goal_admission_.fenceThrough(goalStamp(current_->goal));
        pending_action_cancel_ = current_->identity.task_id;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<NavigateHandle> handle) {
        const auto goal = handle->get_goal();
        // Lifecycle deactivation can occur between acceptance and this callback.
        auto admission_reason=navigationAdmissionReason();
        if(admission_reason.empty())admission_reason=goal_admission_.commit(goalStamp(goal->goal),now().nanoseconds());
        if (!admission_reason.empty()) {
          auto result = std::make_shared<Navigate::Result>();
          result->schema_version = task_schema_version;
          result->reason = admission_reason;
          result->retirement_confirmed = true;
          handle->abort(result); return;
        }
        submit(goal->goal_kind, goal->goal, handle, goal->has_goal_yaw, goal->goal_yaw_tolerance_rad);
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/tick_hz), [this]() { tick(); });
  }

  ~NavigatorNode() override {
    // Shutdown must not block waiting for ROS services. Runtime shutdown is an
    // explicit process boundary; the owning supervisor stops all workers.
    timer_.reset();
    engine_.reset();
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    return lifecycle_.configure() ? nav2_util::CallbackReturn::SUCCESS : nav2_util::CallbackReturn::FAILURE;
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    if (!retiredOperationsDone() || !lifecycle_.activate()) return nav2_util::CallbackReturn::FAILURE;
    createBond();
    publishStatus();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    beginDrain("lifecycle_deactivated");
    destroyBond();
    // Action IO and the timer intentionally survive INACTIVE to collect real
    // retirement replies. Supervisor must wait for drained before peers stop.
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    if (!retiredOperationsDone() || !lifecycle_.cleanup()) return nav2_util::CallbackReturn::FAILURE;
    // Resource objects are session-scoped; teardown is forbidden while drain is
    // pending. Reactivation never resumes an old task or confirmation.
    current_.reset(); tasks_.clear(); health_seen_ = false;
    publishStatus();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    beginDrain("lifecycle_shutdown");
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
    beginDrain("lifecycle_error");
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  ReadyState contextValid(const TaskIdentity& identity) override {
    const auto task = lookup(identity);
    if (!task || task->retired) return {false, "task_retired"};
    if (!retirement_fault_.empty()) return {false, retirement_fault_};
    if (!task->transport_fault.empty()) return {false, task->transport_fault};
    if (task->execution) {
      const auto failure=task->execution->failureReason();
      if (!failure.empty()) return {false,failure};
    }
    if (health_seen_ && health_.hard_fault) return {false, health_.reason};
    if (task->bound && validHealthIdentity() &&
        (task->epoch != health_.localization_epoch || task->seed != health_.localization_seed_id)) {
      return {false, "localization_context_replaced"};
    }
    if (!task->route.snapshot.map_version_id.empty() && validHealthIdentity() &&
        task->route.snapshot.map_version_id != health_.map_version_id)
      return {false, "map_version_replaced"};
    return {true, {}};
  }

  ReadyState initialLocalizationReady(const TaskIdentity& identity) override {
    const auto task = lookup(identity);
    if (!task || task->retired) return {false, "task_retired"};
    // Health comes from the verified continuous pose, not a registration
    // candidate. Binding task epoch/seed remains in inputsReady below.
    if (!healthFresh()) return {false, "waiting_fresh_initial_localization"};
    if (!health_.global_planning_ready || !validHealthIdentity())
      return {false, health_.reason.empty() ? "waiting_initial_localization_verification" : health_.reason};
    return {true, "initial_localization_verified"};
  }
  InitialLocalizationTiming initialLocalizationTiming(const TaskIdentity&) override {
    return startup_evidence_.timing(now().seconds(),Clock::now());
  }

  ReadyState inputsReady(const TaskIdentity& identity) override {
    const auto task = lookup(identity);
    if (!task || task->retired) return {false, "task_retired"};
    if (!retiredOperationsDone()) return {false, "retiring_previous_task"};
    if (!healthFresh()) return {false, "waiting_fresh_navigation_health"};
    const bool global_phase=task->compute.result.status!=BT::NodeStatus::SUCCESS;
    if (!validHealthIdentity() || (global_phase&&!health_.global_planning_ready)) {
      return {false, health_.reason.empty() ? "waiting_localization" : health_.reason};
    }
    if (!task->bound) {
      task->epoch = health_.localization_epoch;
      task->seed = health_.localization_seed_id;
      task->bound = true;
    }
    if(!global_phase&&!localTaskInputsFresh(local_state_,session_,task->route.snapshot.map_version_id,
        static_cast<uint64_t>(task->epoch),task->seed,now().seconds()))
      return {false,!local_state_.usable&&!local_state_.reason.empty()?local_state_.reason:"waiting_fresh_local_task_state"};
    const auto admission_wait = plannerAdmissionWait(task->compute.sent,
      task->compute.result.status == BT::NodeStatus::SUCCESS, task->follow.sent,
      compute_client_->action_server_is_ready(), follow_client_->action_server_is_ready());
    if (!admission_wait.empty()) return {false, admission_wait};
    return {true, {},rclcpp::Time(global_phase?health_.evidence_source_stamp:local_state_.source_stamp).nanoseconds(),
      session_+":"+std::to_string(task->epoch)+":"+task->seed};
  }

  void requestRoute(const TaskIdentity& identity) override {
    auto task = lookup(identity);
    if (!task || task->retired || task->compute.sent) return;
    // This check is repeated at the side-effect boundary, never relying solely
    // on a condition checked by an earlier BT tick.
    if (!retiredOperationsDone() || !task->bound) {
      task->compute.result = {BT::NodeStatus::FAILURE, "previous_task_not_retired"};
      return;
    }
    if (!task->compute.begin()) return;
    task->compute.requested_at = Clock::now();
    task->phase = "computing_route";
    task->reason.clear();
    Compute::Goal goal;
    goal.schema_version = task_schema_version;
    goal.session_id = session_;
    goal.task_id = identity.task_id;
    goal.goal_kind = task->goal_kind;
    goal.goal = task->goal;
    goal.has_goal_yaw = task->has_goal_yaw;
    goal.goal_yaw_tolerance_rad = task->goal_yaw_tolerance;
    rclcpp_action::Client<Compute>::SendGoalOptions options;
    options.goal_response_callback = [this, task](ComputeHandle::SharedPtr handle) {
      task->compute.handle = handle;
      task->compute.feedback_at = Clock::now();
      if (!handle) {
        task->compute.terminal = true;
        task->compute.result = {BT::NodeStatus::FAILURE, "compute_goal_rejected"};
      } else if (task->retired) cancelCompute(task);
    };
    options.feedback_callback = [task](ComputeHandle::SharedPtr,
        std::shared_ptr<const Compute::Feedback> feedback) {
      if (!task->retired && task->compute.pending()) {
        task->compute.feedback_at = Clock::now();
        if(!task->compute.work_started&&
           (feedback->phase=="computing"||feedback->phase=="waiting_localization"))
          task->compute.work_started=task->compute.feedback_at;
        task->phase = feedback->phase;
        task->reason = feedback->reason;
      }
    };
    options.result_callback = [this, task](const ComputeHandle::WrappedResult& result) {
      const bool pending = task->compute.pending();
      const bool accepted = task->compute.acceptTerminal(task->retired);
      if (pending && (!result.result || result.result->schema_version != task_schema_version || retirementResultUnconfirmed(
          result.code == rclcpp_action::ResultCode::SUCCEEDED,
          result.code == rclcpp_action::ResultCode::CANCELED,
          result.result && result.result->retirement_confirmed))) {
        task->retirement_error = result.result && !result.result->reason.empty() ?
          result.result->reason : "compute_retirement_failed";
      }
      if (!accepted) return;
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        task->compute.result = {BT::NodeStatus::FAILURE,
          result.result ? result.result->reason : "compute_result_missing"};
        return;
      }
      const auto& route = *result.result;
      if (route.schema_version != task_schema_version) {
        task->compute.result = {BT::NodeStatus::FAILURE, "compute_result_schema_mismatch"}; return;
      }
      if (route.route_id.empty() || route.route.poses.size() < 2 || route.route.header.frame_id != frame_ ||
          route.localization_epoch != task->epoch || route.localization_seed_id != task->seed) {
        task->compute.result = {BT::NodeStatus::FAILURE, "route_identity_mismatch"};
        return;
      }
      for (const auto& pose : route.route.poses) {
        if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
            !std::isfinite(pose.pose.position.z)) {
          task->compute.result = {BT::NodeStatus::FAILURE, "route_nonfinite"};
          return;
        }
      }
      auto invalid = validateSnapshot(route.snapshot,
        {session_, task->identity.task_id, "", ""}, task->epoch, task->seed,
        frame_, health_.map_version_id, route.route);
      if (invalid.empty() && route.snapshot.route_id != route.route_id) invalid = "route_id_disagrees_with_snapshot";
      if (invalid.empty() && route.snapshot.has_goal_yaw != task->has_goal_yaw) invalid = "route_goal_yaw_intent_changed";
      if (invalid.empty() && task->has_goal_yaw) {
        const auto& q = task->goal.pose.orientation;
        const auto yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
        if (std::abs(std::remainder(route.snapshot.goal_yaw-yaw, 2*3.141592653589793)) > 1e-9 ||
            std::abs(route.snapshot.goal_yaw_tolerance_rad-task->goal_yaw_tolerance) > 1e-9)
          invalid = "route_goal_yaw_intent_changed";
      }
      if (!invalid.empty()) { task->compute.result = {BT::NodeStatus::FAILURE, invalid}; return; }
      task->route = route;
      if(usesGeometryOwner(execution_purpose_,preview_only_)) {
        task->execution=std::make_unique<exec3::Owner>(transport_mode_,execution_blocked_timeout_,execution_recovery_time_,stationary_limits_);
        exec3::Version binding;binding.schema_version=3;binding.session_id=session_;binding.task_id=task->identity.task_id;
        binding.route_id=route.route_id;binding.route_hash=route.snapshot.route_hash;binding.map_version_id=route.snapshot.map_version_id;
        binding.localization_epoch=task->epoch;binding.localization_seed_id=task->seed;task->execution->bind(binding);
        if(!task->execution->bindRoute(route.snapshot)) {
          task->compute.result={BT::NodeStatus::FAILURE,"fixed_route_progress_contract_invalid"};return;
        }
      }
      task->compute.result = {BT::NodeStatus::SUCCESS, route.reason};
      snapshot_pub_->publish(route.snapshot);
    };
    try { compute_client_->async_send_goal(goal, options); }
    catch (const std::exception& e) {
      task->compute.terminal = true;
      task->compute.result = {BT::NodeStatus::FAILURE, std::string("compute_send_failed:")+e.what()};
    }
  }

  RouteTiming routeTiming(const TaskIdentity& identity) override {
    const auto task=lookup(identity);
    if(!task)return {};
    return {!task->compute.work_started.has_value(),task->compute.work_started};
  }

  Result pollRoute(const TaskIdentity& identity) override {
    auto task = lookup(identity);
    if (!task) return {BT::NodeStatus::FAILURE, "task_missing"};
    watchOperation(task->compute, "compute");
    auto result = task->compute.result;
    result.reason = operationExplanation(result.status == BT::NodeStatus::RUNNING,
      result.reason, task->reason, task->phase);
    return result;
  }
  void haltRoute(const TaskIdentity& identity) override { if (auto task = lookup(identity)) cancelCompute(task); }

  void requestFollow(const TaskIdentity& identity) override {
    auto task = lookup(identity);
    if (!task || task->retired || task->follow.sent) return;
    if (task->compute.result.status != BT::NodeStatus::SUCCESS || !retiredOperationsDone()) {
      task->follow.result = {BT::NodeStatus::FAILURE, "no_committed_route"};
      return;
    }
    // A compute result can advance the memory sequence into Follow in the
    // same tick. If discovery changed meanwhile, defer dispatch without
    // manufacturing an action failure or recomputing the global route.
    if (!follow_client_->action_server_is_ready()) {
      task->follow.result = {BT::NodeStatus::RUNNING, "waiting_follow_route_action"};
      task->phase = "waiting_follow_route_action";
      return;
    }
    if (!task->follow.begin()) return;
    task->follow.requested_at = Clock::now();
    task->phase = "following_route";
    task->reason.clear();
    Follow::Goal goal;
    goal.schema_version = task_schema_version;
    goal.session_id = session_;
    goal.task_id = identity.task_id;
    goal.route_id = task->route.route_id;
    goal.localization_epoch = task->epoch;
    goal.localization_seed_id = task->seed;
    goal.route = task->route.route;
    goal.snapshot = task->route.snapshot;
    goal.execution_confirmed = false;
    goal.confirmation_id.clear();
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback = [this, task](FollowHandle::SharedPtr handle) {
      task->follow.handle = handle;
      task->follow.feedback_at = Clock::now();
      if (!handle) {
        task->follow.terminal = true;
        task->follow.result = {BT::NodeStatus::FAILURE, "follow_goal_rejected"};
      } else if (task->retired) cancelFollow(task);
    };
    options.feedback_callback = [task](FollowHandle::SharedPtr,
        std::shared_ptr<const Follow::Feedback> feedback) {
      if (!task->retired && task->follow.pending()) {
        task->follow.feedback_at = Clock::now();
        task->phase = feedback->phase;
        task->reason = feedback->reason;
        task->follow_phase=feedback->phase;task->follow_reason=feedback->reason;
      }
    };
    options.result_callback = [this, task](const FollowHandle::WrappedResult& result) {
      const bool pending = task->follow.pending();
      const bool accepted = task->follow.acceptTerminal(task->retired);
      if (pending && (!result.result || result.result->schema_version != task_schema_version || retirementResultUnconfirmed(
          result.code == rclcpp_action::ResultCode::SUCCEEDED,
          result.code == rclcpp_action::ResultCode::CANCELED,
          result.result && result.result->retirement_confirmed))) {
        task->retirement_error = result.result && !result.result->reason.empty() ?
          result.result->reason : "follow_retirement_failed";
      }
      if (!accepted) return;
      if (result.result && result.result->schema_version != task_schema_version) {
        task->follow.result = {BT::NodeStatus::FAILURE, "follow_result_schema_mismatch"}; return;
      }
      const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->success;
      if (success) {
        task->measured_arrival_stamp = rclcpp::Time(result.result->measured_arrival_stamp).nanoseconds();
        task->arrival_received_at = Clock::now();
        const auto age = (now().nanoseconds() - task->measured_arrival_stamp) * 1e-9;
        if (task->measured_arrival_stamp <= 0 ||
            !reportFresh(age, 0, arrival_evidence_timeout_)) {
          task->follow.result = {BT::NodeStatus::FAILURE, "measured_arrival_evidence_invalid"};
          return;
        }
      }
      task->arrived = success;
      task->follow.result = {success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE,
        result.result ? result.result->reason : "follow_result_missing"};
    };
    try { follow_client_->async_send_goal(goal, options); }
    catch (const std::exception& e) {
      task->follow.terminal = true;
      task->follow.result = {BT::NodeStatus::FAILURE, std::string("follow_send_failed:")+e.what()};
    }
  }
  Result pollFollow(const TaskIdentity& identity) override {
    auto task = lookup(identity);
    if (!task) return {BT::NodeStatus::FAILURE, "task_missing"};
    if (task->execution) {
      const auto failure=task->execution->failureReason();
      if (!failure.empty()) return {BT::NodeStatus::FAILURE,failure};
    }
    if (!task->follow.sent) requestFollow(identity);
    watchOperation(task->follow, "follow");
    auto result = task->follow.result;
    result.reason = operationExplanation(result.status == BT::NodeStatus::RUNNING,
      result.reason, task->reason, task->phase);
    return result;
  }
  void haltFollow(const TaskIdentity& identity) override { if (auto task = lookup(identity)) cancelFollow(task); }
  bool measuredGoalReached(const TaskIdentity& identity) override {
    auto task = lookup(identity);
    // FollowRoute's protocol reserves success for measured final arrival, not
    // SCAN's local endpoint or possession of a candidate spline. This is a
    // task-owned measured event before the bounded cleanup ACK fence, not a
    // claim that the robot can never move afterward. Never refresh its source
    // timestamp or resurrect it after a long readiness pause.
    return task && (!task->execution||task->execution->goalStopped()) && task->arrived && task->measured_arrival_stamp > 0 &&
      reportFresh((now().nanoseconds() - task->measured_arrival_stamp) * 1e-9,
        elapsed(task->arrival_received_at), arrival_evidence_timeout_) &&
      healthFresh() && contextValid(identity).ready && health_.ready;
  }
  void setPaused(const TaskIdentity& identity, bool paused, const std::string& reason) override {
    if (auto task = lookup(identity)) {
      task->paused = paused;
      task->reason = reason;
    }
  }
  void finishTask(const TaskIdentity& identity, bool success, const std::string& reason) override {
    auto task = lookup(identity);
    if (!task || task->finished) return;
    task->finished = true;
    task->finish_success = success;
    task->finish_reason = reason;
    task->paused = false;
    task->phase = terminalPhase(success, task->cancellation_requested, false);
    task->reason = reason;
    if (!success) retire(task);
    completePublicResult(task);
  }
  void cancelTask(const TaskIdentity& identity, const std::string& reason) override {
    auto task = lookup(identity);
    if (!task) return;
    task->cancellation_requested = true;
    retire(task);
    finishTask(identity, false, reason);
  }

private:
  static double monotonic(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
  static double wallTime(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
  std::string navigationAdmissionReason() {
    bool retiring=false;
    for(const auto&entry:tasks_) {
      const auto&t=entry.second;
      retiring|=t->retired&&(t->pending()||!t->retirement_error.empty()||
        (t->execution&&t->execution->confirmed()&&!t->execution->stopped()));
    }
    return taskAdmissionBlocker(lifecycle_.accepting(),retirement_fault_,retiring,
      initial_pose_transaction_->blocker(),dependency_health_.blocker(monotonic()));
  }
  static bool validateYaw(bool required, double tolerance) {
    return !required || (std::isfinite(tolerance) && tolerance >= .01 && tolerance <= .5);
  }
  void beginDrain(const std::string& reason) {
    goal_admission_.fenceAll(now().nanoseconds());
    lifecycle_.begin();
    for (auto& entry : tasks_) entry.second->confirmation.revoke();
    engine_->cancel(reason);
    lifecycle_.observe(retiredOperationsDone(), !retirement_fault_.empty());
    publishStatus();
  }
  void confirm(const ConfirmExecution::Request& request, ConfirmExecution::Response& response) {
    response.schema_version = task_schema_version;
    response.preview_only = preview_only_;
    response.reason=navigationAdmissionReason();
    if(!response.reason.empty())return;
    response.reason=executionConfirmationBlocker(execution_purpose_);
    if(!response.reason.empty())return; // Server-side capability, independent of RViz/UI flags.
    if (!current_) { response.reason = "no_committed_route"; return; }
    const auto& snapshot = current_->route.snapshot;
    const RouteBinding binding{session_, current_->identity.task_id, snapshot.route_id, snapshot.route_hash};
    if(!preview_only_&&(!current_->execution||
       (!current_->execution->confirmed()&&!current_->execution->canConfirm(sourceNow())))) {
      response.reason="waiting_matching_native_and_tracker_preparation";return;
    }
    if(!preview_only_&&!execution_grant_->service_is_ready()){response.reason="waiting_single_sdk_writer";return;}
    record_bound_=recordHashMatches(acceptance_record_,record_sha256_);
    const auto decision = current_->confirmation.confirm(request.schema_version, request.request_id,
      {request.session_id, request.task_id, request.route_id, request.route_hash}, binding,
      lifecycle_.accepting() && !current_->retired && !current_->finished,
      healthFresh() && health_.local_control_ready && contextValid(current_->identity).ready,
      snapshot.execution_eligible, preview_only_,transport_mode_=="isolated_mock"||record_bound_);
    response.accepted = decision.accepted;
    response.execution_authorized = decision.authorized;
    response.duplicate = decision.duplicate;
    response.confirmation_id = decision.confirmation_id;
    response.reason = decision.reason;
    if(decision.authorized&&current_->execution&&!current_->execution->confirmed()) {
      auto task=current_;
      if(!task->execution->begin(task->identity.task_id+":execution",decision.confirmation_id,++execution_epoch_,sourceNow())) {
        task->confirmation.revoke();response.accepted=response.execution_authorized=false;response.reason="execution_prepare_changed";return;
      }
      auto req=std::make_shared<d1max_planning_interfaces::srv::ExecutionGrant::Request>();
      req->version=task->execution->grantVersion(sourceNow());req->execution_id=task->execution->executionId();
      req->control_epoch=task->execution->controlEpoch();req->confirmation_id=decision.confirmation_id;
      req->request_id=decision.confirmation_id;req->sdk_session=expected_sdk_session_;req->acceptance_record_sha256=record_sha256_;req->source_stamp=now();req->activate=true;
      req->transport_mode=transport_mode_;
      execution_grant_->async_send_request(req,[this,task](rclcpp::Client<d1max_planning_interfaces::srv::ExecutionGrant>::SharedFuture f){
        try {const auto reply=f.get();if(!reply->accepted)task->execution->stop("sdk_grant_rejected:"+reply->reason,sourceNow());}
        catch(const std::exception&){task->execution->stop("sdk_grant_transport_failure",sourceNow());}
      });
    }
  }
  template<class Handle>
  std::string watchOperation(Operation<Handle>& operation, const std::string& name) {
    const bool warming=name=="compute"&&!operation.work_started;
    const auto since=warming?operation.requested_at:operation.work_started.value_or(operation.requested_at);
    const auto why = name=="compute"&&routeDeadlineExpired(operation.pending(),elapsed(since),
      warming?worker_startup_timeout_:compute_route_timeout_)?
      std::string(warming?"worker_startup_timeout":"route_end_to_end_timeout"):actionTransportTimeout(operation.pending(),
      static_cast<bool>(operation.handle), elapsed(operation.requested_at),
      elapsed(operation.feedback_at), action_watchdog_timeout_);
    if (!why.empty()) {
      // Do not mark terminal: the retirement fence must still wait for the
      // worker's real result, even after the tree has reported this failure.
      operation.result = {BT::NodeStatus::FAILURE, name + "_" + why};
    }
    return why.empty() ? "" : name + "_" + why;
  }
  std::shared_ptr<Task> lookup(const TaskIdentity& identity) const {
    const auto it = tasks_.find(identity.task_id);
    if (it == tasks_.end() || it->second->identity.context_id != identity.context_id) return {};
    return it->second;
  }
  bool validateGoal(const std::string& kind, const geometry_msgs::msg::PoseStamped& goal) const {
    const auto& p = goal.pose.position;
    const auto& q = goal.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return validGoalFrame(kind, goal.header.frame_id, frame_, planning_frame_) &&
      std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::abs(p.x) < 100000 && std::abs(p.y) < 100000 && std::abs(p.z) < 100000 &&
      std::isfinite(norm) && norm > 0.999 && norm < 1.001;
  }
  static std::int64_t goalStamp(const geometry_msgs::msg::PoseStamped& goal) {
    const auto& stamp=goal.header.stamp;
    if (stamp.sec<0 || stamp.nanosec>=1000000000U) return 0;
    return static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
  }
  void acceptTopicGoal(const std::string& kind, const geometry_msgs::msg::PoseStamped& goal) {
    const auto blocker=navigationAdmissionReason();
    if(!blocker.empty()){RCLCPP_WARN(get_logger(),"Rejected topic goal: %s",blocker.c_str());return;}
    if (!validateGoal(kind, goal)) {
      RCLCPP_WARN(get_logger(), "Rejected nonfinite goal, quaternion, kind or frame");
      return;
    }
    const auto stamp=goalStamp(goal);
    auto reason=goal_admission_.reserve(stamp,now().nanoseconds());
    if (reason.empty()) reason=goal_admission_.commit(stamp,now().nanoseconds());
    if (!reason.empty()) {
      RCLCPP_WARN(get_logger(),"Rejected topic goal: %s",reason.c_str()); return;
    }
    submit(kind, goal, {});
  }
  void submit(const std::string& kind, const geometry_msgs::msg::PoseStamped& goal,
              std::shared_ptr<NavigateHandle> action, bool has_yaw=false, double yaw_tolerance=.15) {
    bool old_work_pending = false;
    for (const auto& entry : tasks_) old_work_pending |= unresolvedRetirement(
      entry.second->retired, entry.second->pending(), entry.second->retirement_error);
    if (!old_work_pending) retirement_fault_.clear();
    auto task = std::make_shared<Task>();
    task->preparation=std::make_unique<PreparationBudget>(preparation_limits_);
    task->identity = {session_ + "." + std::to_string(++task_sequence_), session_};
    task->goal_kind = kind;
    task->goal = goal;
    task->has_goal_yaw = has_yaw;
    task->goal_yaw_tolerance = yaw_tolerance;
    task->action = std::move(action);
    if (healthFresh() && validHealthIdentity()) {
      task->bound = true;
      task->epoch = health_.localization_epoch;
      task->seed = health_.localization_seed_id;
    }
    tasks_.emplace(task->identity.task_id, task);
    engine_->submit(task->identity);  // Old task is halted before the new one ticks.
    current_ = std::move(task);
    publishStatus();
  }
  void cancel(const std::string& reason, bool cancel_all=true) {
    if (cancel_all) goal_admission_.fenceAll(now().nanoseconds());
    engine_->cancel(reason);
    publishStatus();
  }
  void onHealth(const Health& health) {
    if (health.schema_version != task_schema_version || health.session_id != session_ || health.map_version_id.empty()) return;
    const auto stamp = rclcpp::Time(health.stamp).nanoseconds();
    if(health_seen_&&health.localization_epoch>0&&health.localization_epoch<health_.localization_epoch)return;
    if(health_seen_&&health.localization_epoch==health_.localization_epoch&&
        stamp<=rclcpp::Time(health_.stamp).nanoseconds())return;
    // A queued old report is not fresh simply because it was received now.
    const double age = (now().nanoseconds() - stamp) * 1e-9;
    if (stamp <= 0 || !reportFresh(age, 0.0, health_timeout_)) return;
    if((health.local_control_ready||health.global_planning_ready)&&
       !exec3::fresh(exec3::seconds(health.evidence_source_stamp),now().seconds(),.4))return;
    health_ = health;
    health_seen_ = true;
    health_received_ = Clock::now();
  }
  bool healthFresh() const {
    if (!health_seen_) return false;
    const auto age = (now().nanoseconds() - rclcpp::Time(health_.stamp).nanoseconds()) * 1e-9;
    return reportFresh(age, elapsed(health_received_), health_timeout_);
  }
  bool validHealthIdentity() const {
    return health_seen_ && health_.localization_epoch > 0 && !health_.localization_seed_id.empty();
  }
  void retire(const std::shared_ptr<Task>& task) {
    task->confirmation.revoke();
    if(task->execution)task->execution->stop("task_retired",sourceNow());
    if (!task->retired) {
      task->retired = true;
      task->retired_at = Clock::now();
    }
    cancelCompute(task);
    cancelFollow(task);
  }
  void cancelCompute(const std::shared_ptr<Task>& task) {
    auto& operation = task->compute;
    if (!operation.shouldCancel(static_cast<bool>(operation.handle))) return;
    operation.cancel_sent = true;
    try { compute_client_->async_cancel_goal(operation.handle); }
    catch (const std::exception&) { operation.cancel_error = true; }
  }
  void cancelFollow(const std::shared_ptr<Task>& task) {
    auto& operation = task->follow;
    if (!operation.shouldCancel(static_cast<bool>(operation.handle))) return;
    operation.cancel_sent = true;
    try { follow_client_->async_cancel_goal(operation.handle); }
    catch (const std::exception&) { operation.cancel_error = true; }
  }
  bool retiredOperationsDone() {
    bool ready = true;
    for (const auto& entry : tasks_) {
      const auto& task = entry.second;
      if(task->retired&&task->execution&&task->execution->confirmed()&&!task->execution->stopped()) {
        ready=false;
        if(task->execution->needsReview())retirement_fault_="previous_physical_stop_unconfirmed";
      }
      if (task->retired && !task->retirement_error.empty()) {
        retirement_fault_ = "previous_worker_retirement_unconfirmed:" + task->retirement_error;
        ready = false;
      }
      if (!task->retired || !task->pending()) continue;
      ready = false;
      if (retirementExpired(task->retired, task->pending(), elapsed(task->retired_at), cancellation_timeout_)) {
        retirement_fault_ = "previous_worker_cancellation_timeout";
      }
    }
    return ready && retirement_fault_.empty();
  }
  void completePublicResult(const std::shared_ptr<Task>& task) {
    if (!task->finished || task->public_terminal) return;
    if(task->execution&&task->execution->confirmed()&&!task->execution->stopped()) {
      if(!task->execution->needsReview())return;
      task->retirement_error="physical_stop_unconfirmed";
    }
    const bool quarantine = retirementExpired(task->retired, task->pending(),
      elapsed(task->retired_at), cancellation_timeout_);
    const auto completion = publicCompletion(task->finished, task->pending(), quarantine,
      !task->retirement_error.empty(), task->finish_success,
      task->action && task->action->is_canceling());
    if (completion == PublicCompletion::None) return;
    task->phase = terminalPhase(task->finish_success, task->cancellation_requested,
      quarantine || !task->retirement_error.empty());
    if (quarantine) task->reason = "previous_worker_cancellation_timeout";
    else if (!task->retirement_error.empty()) task->reason = task->retirement_error;
    if (!task->action || !task->action->is_active()) {
      task->public_terminal = true;
      return;
    }
    auto result = std::make_shared<Navigate::Result>();
    result->schema_version = task_schema_version;
    result->retirement_confirmed = !task->pending() && task->retirement_error.empty();
    result->physical_stop_confirmed = task->execution&&task->execution->stopped()&&transport_mode_=="live";
    result->success = completion == PublicCompletion::Succeeded;
    result->reason = quarantine ? "previous_worker_cancellation_timeout" :
      (!task->retirement_error.empty() ? task->retirement_error : task->finish_reason);
    if (completion == PublicCompletion::Canceled) task->action->canceled(result);
    else if (completion == PublicCompletion::Succeeded) task->action->succeed(result);
    else task->action->abort(result);
    task->public_terminal = true;
  }
  void tick() {
    try {
      if (!pending_action_cancel_.empty()) {
        if (current_ && current_->identity.task_id == pending_action_cancel_) cancel("action_cancelled",false);
        pending_action_cancel_.clear();
      }
      retiredOperationsDone();
      lifecycle_.observe(retiredOperationsDone(), !retirement_fault_.empty());
      if(dependency_health_.fatal(monotonic())&&lifecycle_.accepting())
        beginDrain(dependency_health_.blocker(monotonic()));
      const bool initial_ready=healthFresh()&&health_.global_planning_ready&&health_.local_control_ready&&
        !health_.hard_fault&&localTaskInputsFresh(local_state_,session_,health_.map_version_id,
          static_cast<uint64_t>(health_.localization_epoch),health_.localization_seed_id,now().seconds());
      initial_pose_transaction_->readyIdentity(static_cast<uint64_t>(health_.localization_epoch),
        health_.localization_seed_id,health_.map_version_id,rclcpp::Time(health_.evidence_source_stamp).nanoseconds(),
        now().nanoseconds(),monotonic(),initial_ready);
      if (current_ && !current_->retired && !current_->finished && current_->transport_fault.empty()) {
        // The input-pause decorator deliberately does not tick its child.
        // Action transport supervision must therefore run independently, or
        // an unresponsive worker would be hidden behind a pose/map pause.
        current_->transport_fault = watchOperation(current_->compute, "compute");
        if (current_->transport_fault.empty()) {
          current_->transport_fault = watchOperation(current_->follow, "follow");
        }
      }
      preparationTick();
      if (lifecycle_.accepting()) engine_->tick();
      executionTick();
      publishStatus();
      for (auto it = tasks_.begin(); it != tasks_.end();) {
        completePublicResult(it->second);
        if (it->second != current_ && it->second->finished && it->second->public_terminal &&
            !it->second->pending() && it->second->retirement_error.empty()) it = tasks_.erase(it);
        else ++it;
      }
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "BT tick failed: %s", error.what());
      engine_->cancel(std::string("bt_exception:")+error.what());
      publishStatus();
    }
  }
  void preparationTick() {
    auto task=current_;
    // Legacy visualization profiles already have adapter-owned preparation
    // budgets and do not publish the schema-3 native execution proofs.
    if(!usesGeometryOwner(execution_purpose_,preview_only_)||!task||task->retired||task->finished||!task->transport_fault.empty()||!task->preparation||
       (task->execution&&task->execution->confirmed())||
       !preparationStarted(task->compute.result.status==BT::NodeStatus::SUCCESS,task->follow.sent))return;
    const bool inputs=healthFresh()&&validHealthIdentity()&&
      localTaskInputsFresh(local_state_,session_,task->route.snapshot.map_version_id,
        static_cast<uint64_t>(task->epoch),task->seed,now().seconds());
    const auto evidence=task->execution?task->execution->preparationEvidence(sourceNow()):0;
    const auto stage=preparationStage(inputs,task->compute.result.status==BT::NodeStatus::SUCCESS,
      task->follow_phase,evidence);
    const auto reason=inputs?task->follow_reason:"waiting_current_localization";
    const double monotonic=std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    const auto problem=task->preparation->observe(stage,monotonic,evidence);
    if(!problem.empty())task->transport_fault=problem+":"+reason;
  }
  bool localExecutionInputsFresh(const Task&task,double time)const {
    return local_state_.session_id==session_&&local_state_.usable&&
      local_state_.map_version_id==task.route.snapshot.map_version_id&&
      local_state_.localization_epoch==static_cast<uint64_t>(task.epoch)&&local_state_.localization_seed_id==task.seed&&
      exec3::fresh(exec3::seconds(local_state_.source_stamp),time,.4)&&
      exec3::fresh(exec3::seconds(local_state_.imu_stamp),time,.1)&&
      exec3::fresh(exec3::seconds(local_state_.posterior_stamp),time,.4);
  }
  // Proof and admission use independent topics. Their complete pair may have
  // less than a 10 Hz BT period left, so prepare immediately once both arrive.
  // This does not tick the tree, reset a budget, renew the incumbent lease or
  // apply geometry; only the sole SDK writer can report that irreversible fact.
  exec3::SourceClock sourceNow()const{return exec3::SourceClock::fromNanoseconds(now().nanoseconds());}
  void tryPrepareReadyHandoff() {
    if(!permitsMotionAuthority(execution_purpose_,preview_only_)||!current_||current_->retired||
       !current_->execution)return;
    const auto time=sourceNow();auto&task=*current_;
    if(!healthFresh()||!health_.local_control_ready||!localExecutionInputsFresh(task,time))return;
    const auto*version=task.execution->pendingVersion(time);
    const auto*incumbent=task.execution->publishedHandoffIncumbent(time);
    if(!version||!incumbent)return;
    const auto*goal=task.execution_goals.find(*version);if(!goal)return;
    auto candidate=*incumbent;candidate.version=*version;
    if(!permit_publication_.prepare(candidate,task.identity.task_id,true,false)||
       !task.execution->prepareHandoff(*incumbent,*goal,candidate.sequence,time))return;
    handoff_pub_->publish(*task.execution->handoff());
  }
  void executionTick() {
    for(auto& entry:tasks_) {
      auto& task=entry.second;if(!task->execution)continue;
      const auto time=sourceNow();
      const bool body_fresh=localExecutionInputsFresh(*task,time);
      const auto evidence=rclcpp::Time(local_state_.source_stamp).nanoseconds();
      const auto identity=session_+":"+std::to_string(local_state_.localization_epoch)+":"+local_state_.localization_seed_id;
      const auto monotonic=std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
      auto p=task->execution->tick(time,!task->retired&&healthFresh()&&health_.local_control_ready&&body_fresh,
        evidence,identity,monotonic);
      const auto* proposal=task->execution_goals.committed(p.version);
      if(proposal) {
        p.goal_position=proposal->goal_position;p.frame_id=proposal->reference.path.header.frame_id;
        p.has_goal_yaw=task->has_goal_yaw;p.goal_yaw_tolerance_rad=task->goal_yaw_tolerance;
        p.goal_yaw=proposal->goal_yaw;
        const bool yaw_contract=!task->has_goal_yaw||(proposal->has_goal_yaw&&std::isfinite(p.goal_yaw)&&
          std::abs(proposal->goal_yaw_tolerance_rad-task->goal_yaw_tolerance)<=1e-9);
        if(!yaw_contract){p.allowed=false;p.phase="holding";p.reason="final_yaw_anchor_contract_mismatch";}
        if(body_fresh) {
          const auto& local=local_state_.local_odometry.pose.pose;
          const auto yaw=[](const geometry_msgs::msg::Quaternion&q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
          const bool reached=std::hypot(local.position.x-p.goal_position.x,local.position.y-p.goal_position.y)<=.20&&
            std::abs(local.position.z-p.goal_position.z)<=.15;
          const bool yaw_ok=std::abs(std::remainder(p.goal_yaw-yaw(local.orientation),2*M_PI))<=task->goal_yaw_tolerance;
          if(yaw_contract)task->execution->measuredGoal(reached,task->has_goal_yaw,yaw_ok,time,exec3::seconds(local_state_.source_stamp));
          if(task->execution->stopping()){p.allowed=false;p.revoked=true;p.phase=task->execution->phase();p.reason=task->execution->reason();}
          else if(task->execution->phase()=="aligning") {
            p.phase="aligning";
            if(!task->execution->yawProof(p.goal_yaw,time)) {p.allowed=false;p.reason="waiting_goal_yaw_swept_volume_proof";}
          }
        }
      } else if(p.allowed){p.allowed=false;p.phase="holding";p.reason="waiting_final_body_goal_for_committed_anchor";}
      const auto blocked=task->execution->supervise(p,time,body_fresh?&local_state_:nullptr);
      if(!blocked.empty())task->transport_fault=blocked;
      if(!permitsMotionAuthority(execution_purpose_,preview_only_)) {
        p.allowed=false; // Keep exact geometry identity/proof, never an SDK movement lease.
        if(!p.revoked){p.phase="preview";p.reason="planning_only_execution_disabled";}
      }
      if(permit_publication_.prepare(p,current_?current_->identity.task_id:"",
           task->execution->confirmed(),task->execution->stopped())) {
        task->execution->notePublication(p,time);
        execution_pub_->publish(p);
        // The globally ordered candidate permit lives ONLY in this conditional
        // wrapper.  Publishing it on the active-permit channel would retire the
        // old tracker/map geometry before the writer can actually submit it.
        if(permitsMotionAuthority(execution_purpose_,preview_only_)&&!p.revoked&&current_==task) {
          if(const auto* version=task->execution->pendingVersion(time)) {
            if(const auto* goal=task->execution_goals.find(*version)) {
              auto candidate=p;candidate.version=*version;
              if(permit_publication_.prepare(candidate,task->identity.task_id,true,false))
                task->execution->prepareHandoff(p,*goal,candidate.sequence,time);
            }
          }
        }
        if(const auto* grant=task->execution->handoff()) {
          if(current_==task||(task->execution->confirmed()&&!task->execution->stopped()&&grant->revoked))
            handoff_pub_->publish(*grant);
        }
      }
      if(task->execution->confirmed()&&!task->retired) {
        task->phase=p.phase;task->reason=p.reason;
        const auto failure=task->execution->failureReason();
        if(!failure.empty()) {
          task->confirmation.revoke();
          if(task->transport_fault.empty())task->transport_fault=failure;
        }
      }
    }
  }
  bool measuredStopConfirmed()const {
    bool any=false;for(const auto&entry:tasks_)if(entry.second->execution&&entry.second->execution->confirmed()) {
      any=true;if(!entry.second->execution->stopped())return false;
    }return any;
  }
  bool physicalStopConfirmed()const {return transport_mode_=="live"&&measuredStopConfirmed();}
  void publishStatus() {
    const auto snapshot = engine_->snapshot();
    nlohmann::json out = {{"schema", task_schema_version}, {"session_id", session_},
      {"stamp", now().seconds()}, {"received_at_unix", now().seconds()},
      {"task_id", snapshot.task.task_id},
      {"active", snapshot.active}, {"root_status", BT::toStr(snapshot.root_status)},
      {"active_node", snapshot.active_node}, {"reason", snapshot.reason},
      {"preview_only", preview_only_},{"execution_purpose",execution_purpose_},
      {"motion_enabled", permitsMotionAuthority(execution_purpose_,preview_only_)}, {"owner", "BehaviorTree.CPP"},
      {"lifecycle_active", lifecycle_.accepting()}, {"lifecycle_draining", lifecycle_.draining()},
      {"lifecycle_quarantined", lifecycle_.quarantined()},
      {"goal_admission_barrier_ns", goal_admission_.barrier()},
      {"initial_pose_transaction_blocked",initial_pose_transaction_->blocked()},
      {"initial_pose_transaction_reason",initial_pose_transaction_->blocker()},
      {"dependency_health_reason",dependency_health_.blocker(monotonic())},
      {"physical_stop_confirmed", physicalStopConfirmed()}, {"measured_stop_confirmed",measuredStopConfirmed()},
      {"physical_stop_state", physicalStopConfirmed()?"confirmed":transport_mode_=="isolated_mock"&&measuredStopConfirmed()?"mock_measured_stop_confirmed":"not_confirmed"},
      {"execution_authorized", false},
      {"nodes", nlohmann::json::array()}, {"transitions", nlohmann::json::array()}};
    if (current_) {
      out["phase"] = current_->retired && current_->pending() ? "retiring_task" :
        (current_->paused ? "paused" : current_->phase);
      out["worker_reason"] = current_->reason;
      out["route_id"] = current_->route.route_id;
      out["route_hash"] = current_->route.snapshot.route_hash;
      out["execution_eligible"] = current_->route.snapshot.execution_eligible;
      out["execution_blocker"] = execution_purpose_=="planning_only"?"planning_only_execution_disabled":
        preview_only_?"preview_only":transport_mode_=="live"&&!record_bound_?"acceptance_record_binding_missing":"";
      out["execution_authorized"]=current_->execution&&current_->execution->confirmed()&&!current_->execution->stopping();
      out["execution_transport_mode"]=transport_mode_;
      out["execution_id"]=current_->execution?current_->execution->executionId():"";
      out["control_epoch"]=current_->execution?current_->execution->controlEpoch():0;
      out["execution_progress_reason"]=current_->execution?current_->execution->progressReason():"";
      out["execution_blocked_age_s"]=current_->execution?current_->execution->blockedAge(sourceNow()):0.;
      out["execution_progress_credit_samples"]=current_->execution?current_->execution->progressCreditSamples():0;
      out["execution_last_progress_source_s"]=current_->execution?current_->execution->lastProgressSource():-1.;
      out["execution_last_admitted_motion_source_s"]=current_->execution?current_->execution->lastAdmittedMotionSource():0.;
      out["execution_progress_body_source_ns"]=current_->execution?current_->execution->progressBodySourceNs():0;
      out["writer_commit_sequence"]=current_->execution?current_->execution->appliedCommitSequence():0;
      out["handoff_reason"]=current_->execution?current_->execution->handoffReason():"";
      out["handoff_ready_window_s"]=current_->execution?current_->execution->handoffReadyWindow():-1.;
      out["prepared_handoff_id"]=current_->execution&&current_->execution->handoff()?current_->execution->handoff()->handoff_id:"";
      out["measured_stop_confirmed"]=current_->execution&&current_->execution->stopped();
      out["worker_retirement_pending"] = current_->retired && current_->pending();
      out["localization_epoch"] = current_->epoch;
      out["localization_seed_id"] = current_->seed;
    }
    for (const auto& node : snapshot.nodes) out["nodes"].push_back({{"uid", node.uid},
      {"name", node.name}, {"type", node.type}, {"status", BT::toStr(node.status)}, {"reason", node.reason}});
    for (const auto& event : snapshot.transitions) out["transitions"].push_back({{"sequence", event.sequence},
      {"uid", event.uid}, {"node", event.node}, {"previous", BT::toStr(event.previous)},
      {"current", BT::toStr(event.current)}});
    std_msgs::msg::String message;
    message.data = out.dump();
    status_pub_->publish(message);
    if (current_ && current_->action && current_->action->is_active()) {
      auto feedback = std::make_shared<Navigate::Feedback>();
      feedback->task_id = current_->identity.task_id;
      feedback->route_id = current_->route.route_id;
      feedback->route_hash = current_->route.snapshot.route_hash;
      feedback->preview_only = preview_only_;
      feedback->execution_authorized = current_->execution&&current_->execution->confirmed()&&!current_->execution->stopping();
      feedback->phase = current_->retired && current_->pending() ? "retiring_task" :
        (current_->paused ? "paused" : current_->phase);
      feedback->active_node = snapshot.active_node;
      feedback->reason = snapshot.reason.empty() ? current_->reason : snapshot.reason;
      current_->action->publish_feedback(feedback);
    }
  }

  std::string session_, frame_, planning_frame_, retirement_fault_, pending_action_cancel_;
  bool preview_only_=true,record_bound_=false;
  std::string execution_purpose_="execution";
  std::string acceptance_record_,record_sha256_;
  std::string transport_mode_,expected_sdk_session_;
  uint64_t execution_epoch_=0;
  d1max_planning_interfaces::msg::NavigationState body_state_;
  d1max_planning_interfaces::msg::LocalNavigationState local_state_;
  rclcpp::Client<d1max_planning_interfaces::srv::ExecutionGrant>::SharedPtr execution_grant_;
  rclcpp::Publisher<exec3::Permit>::SharedPtr execution_pub_;
  rclcpp::Publisher<exec3::Handoff>::SharedPtr handoff_pub_;
  rclcpp::Subscription<exec3::CommitAck>::SharedPtr commit_ack_sub_;
  rclcpp::Subscription<exec3::Stationary>::SharedPtr stationary_sub_;
  rclcpp::Subscription<exec3::GeometryReceipt>::SharedPtr geometry_receipt_sub_;
  rclcpp::Subscription<exec3::RouteProgress>::SharedPtr route_progress_sub_;
  rclcpp::Subscription<exec3::Validation>::SharedPtr validation_sub_;
  rclcpp::Subscription<exec3::Admission>::SharedPtr admission_sub_;
  rclcpp::Subscription<exec3::MotionDemand>::SharedPtr safe_demand_sub_;
  rclcpp::Subscription<exec3::MotionValidation>::SharedPtr motion_validation_sub_;
  rclcpp::Subscription<exec3::Receipt>::SharedPtr receipt_sub_;
  rclcpp::Subscription<exec3::SDKState>::SharedPtr sdk_state_sub_;
  rclcpp::Subscription<exec3::Stop>::SharedPtr stop_report_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::NavigationState>::SharedPtr navigation_state_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::LocalNavigationState>::SharedPtr local_state_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::ReferenceProposal>::SharedPtr proposal_sub_;
  LifecycleDrain lifecycle_;
  GoalAdmission goal_admission_;
  DependencyHealthGuard dependency_health_;
  std::unique_ptr<InitialPoseTransaction>initial_pose_transaction_;
  double health_timeout_{0.6}, cancellation_timeout_{3.0}, action_watchdog_timeout_{3.0};
  double compute_route_timeout_{10.};
  double worker_startup_timeout_{60.};
  std::string initial_localization_map_sha_;
  InitialLocalizationEvidence startup_evidence_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr startup_status_sub_;
  double arrival_evidence_timeout_{4.0};
  double execution_blocked_timeout_{30.},execution_recovery_time_{1.};
  PreparationLimits preparation_limits_;
  std::uint64_t task_sequence_{0};
  exec3::PermitPublication permit_publication_;
  exec3::StationaryLimits stationary_limits_;
  Health health_;
  bool health_seen_{false};
  Clock::time_point health_received_{};
  std::unordered_map<std::string, std::shared_ptr<Task>> tasks_;
  std::shared_ptr<Task> current_;
  rclcpp_action::Client<Compute>::SharedPtr compute_client_;
  rclcpp_action::Client<Follow>::SharedPtr follow_client_;
  rclcpp_action::Server<Navigate>::SharedPtr navigate_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<RouteSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Service<ConfirmExecution>::SharedPtr confirm_service_;
  rclcpp::Service<PrepareTransition>::SharedPtr prepare_service_;
  rclcpp::Service<PrepareInitialPose>::SharedPtr initial_pose_service_;
  rclcpp::Subscription<InitialPoseOutcome>::SharedPtr initial_outcome_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dependency_sub_;
  rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  std::unique_ptr<Engine> engine_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace d1max_navigation_bt

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    // Single-threaded ownership: no callback mutates task generations in
    // parallel with a BT tick; action IO remains fully asynchronous.
    auto node = std::make_shared<d1max_navigation_bt::NavigatorNode>();
    rclcpp::spin(node->get_node_base_interface());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("d1max_navigation_bt"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
