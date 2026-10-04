#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <d1max_planning_interfaces/msg/tagged_bspline.hpp>
#include <d1max_planning_interfaces/msg/tracking_progress.hpp>
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include "d1max_trajectory_tracker/execution_contract.hpp"
#include "d1max_trajectory_tracker/navigation_state_contract.hpp"

namespace d1max_trajectory_tracker
{
using Json = nlohmann::json;
static double monotonicNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static double stampSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode() : Node("trajectory_tracker")
  {
    Config c;
    c.session_id = declare_parameter<std::string>("session_id", "");
    c.map_version_id = declare_parameter<std::string>("map_version_id", "");
    c.map_frame = declare_parameter<std::string>("map_frame", c.map_frame);
    c.planning_frame = declare_parameter<std::string>("planning_frame", c.planning_frame);
    c.base_frame = declare_parameter<std::string>("base_frame", c.base_frame);
    c.max_speed = declare_parameter<double>("max_speed", c.max_speed);
    c.max_yaw_rate = declare_parameter<double>("max_yaw_rate", c.max_yaw_rate);
    c.max_acceleration = declare_parameter<double>("max_acceleration", c.max_acceleration);
    c.max_yaw_acceleration = declare_parameter<double>("max_yaw_acceleration", c.max_yaw_acceleration);
    c.task_timeout = declare_parameter<double>("task_timeout", c.task_timeout);
    c.odom_timeout = declare_parameter<double>("odom_timeout", c.odom_timeout);
    c.trajectory_timeout = declare_parameter<double>("trajectory_timeout", c.trajectory_timeout);
    c.lookahead = declare_parameter<double>("lookahead", c.lookahead);
    c.kp_position = declare_parameter<double>("kp_position", c.kp_position);
    c.kp_yaw = declare_parameter<double>("kp_yaw", c.kp_yaw);
    c.heading_threshold = declare_parameter<double>("heading_threshold", c.heading_threshold);
    c.goal_tolerance = declare_parameter<double>("goal_tolerance", c.goal_tolerance);
    c.goal_height_tolerance = declare_parameter<double>("goal_height_tolerance", c.goal_height_tolerance);
    c.single_floor_max_height_change = declare_parameter<double>(
        "single_floor_max_height_change", c.single_floor_max_height_change);
    c.position_freeze_distance = declare_parameter<double>("position_freeze_distance", c.position_freeze_distance);
    c.stationary_linear_threshold_mps=declare_parameter<double>(
        "stationary_linear_threshold_mps",c.stationary_linear_threshold_mps);
    c.stationary_angular_threshold_radps=declare_parameter<double>(
        "stationary_angular_threshold_radps",c.stationary_angular_threshold_radps);
    c.stationary_reentry_duration_s=declare_parameter<double>(
        "stationary_reentry_duration_s",c.stationary_reentry_duration_s);
    const int stationary_samples=declare_parameter<int>("stationary_minimum_samples",3);
    if(stationary_samples<3||stationary_samples>512)
      throw std::invalid_argument("bounded_stationary_minimum_samples_required");
    c.stationary_minimum_samples=static_cast<unsigned>(stationary_samples);
    c.require_support_reference=true;
    c.external_goal_completion=true;
    config_ = c;
    execution_=std::make_unique<ExecutionContract>(c,declare_parameter<std::string>("transport_mode","live"),
      declare_parameter<std::string>("execution_braking_model_sha256",""),declare_parameter<bool>("writer_handoff_enabled",false),true);
    core_=&execution_->core();
    const auto topic = [this](const char *key, const char *value) {
      return declare_parameter<std::string>(key, value);
    };
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        topic("command_topic", "/d1max/live_planning/execution/command_debug"), rclcpp::QoS(1));
    frozen_pub_ = create_publisher<std_msgs::msg::Bool>(
        topic("frozen_topic", "/d1max/pct_scan/execution_frozen"), rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>(
        topic("status_topic", "/d1max/pct_scan/tracker_status"), rclcpp::QoS(1));
    progress_pub_ = create_publisher<d1max_planning_interfaces::msg::TrackingProgress>(
        topic("progress_topic", "/d1max/pct_scan/planning/tracking_progress"), rclcpp::QoS(1));
    demand_pub_=create_publisher<wire::MotionDemand>(topic("demand_topic","/d1max/live_planning/execution/demand_unchecked"),rclcpp::QoS(1));
    prepared_pub_=create_publisher<wire::PreparedMotionDemand>(topic("prepared_demand_topic","/d1max/live_planning/execution/prepared_demand"),rclcpp::QoS(1));
    admission_pub_=create_publisher<wire::TrajectoryAdmission>(topic("admission_topic","/d1max/live_planning/execution/admission"),rclcpp::QoS(1));
    geometry_receipt_pub_=create_publisher<wire::TrackerGeometryReceipt>(
      topic("geometry_receipt_topic","/d1max/live_planning/execution/tracker_geometry_receipt"),rclcpp::QoS(4));
    blocked_entry_pub_=create_publisher<wire::MotionValidation>(
      topic("blocked_entry_topic","/d1max/live_planning/execution/blocked_entry"),rclcpp::QoS(4));
    proposal_sub_=create_subscription<wire::ReferenceProposal>(topic("proposal_topic","/d1max/live_planning/execution/reference_proposal"),rclcpp::QoS(1),
      [this](wire::ReferenceProposal::ConstSharedPtr m){execution_->proposal(*m,now().seconds());});
    support_sub_=create_subscription<wire::SupportReference>(topic("support_topic","/d1max/live_planning/execution/support"),rclcpp::QoS(1),
      [this](wire::SupportReference::ConstSharedPtr m){execution_->support(*m);});
    validation_sub_=create_subscription<wire::TrajectoryValidation>(topic("validation_topic","/d1max/live_planning/execution/validation"),rclcpp::QoS(2),
      [this](wire::TrajectoryValidation::ConstSharedPtr m){
        execution_->validation(*m,now().seconds());
        // Prepare on the fresh proof itself: the join evidence has a 0.4 s
        // lease from its body sample, and waiting for the timer phase (up to
        // 0.2 s) plus the owner tick routinely expired it before commit.
        prepareAndPublish(now().seconds(),monotonicNow());
      });
    motion_validation_sub_=create_subscription<wire::MotionValidation>(
      topic("motion_validation_topic","/d1max/live_planning/execution/motion_validation"),rclcpp::QoS(8),
      [this](wire::MotionValidation::ConstSharedPtr m){
        if(execution_->motionValidation(*m,now().seconds()))
          if(const auto blocked=execution_->takeBlockedEntry())blocked_entry_pub_->publish(*blocked);
      });
    permit_sub_=create_subscription<wire::ExecutionPermit>(topic("permit_topic","/d1max/live_planning/execution/permit"),rclcpp::QoS(1),
      [this](wire::ExecutionPermit::ConstSharedPtr m){
        if(execution_->permit(*m,now().seconds(),monotonicNow()))
          if(const auto installed=execution_->geometryReceipt())geometry_receipt_pub_->publish(*installed);
      });
    handoff_sub_=create_subscription<wire::ExecutionHandoffGrant>(topic("handoff_grant_topic","/d1max/live_planning/execution/handoff_grant"),rclcpp::QoS(2),
      [this](wire::ExecutionHandoffGrant::ConstSharedPtr m){execution_->handoff(*m,now().seconds(),monotonicNow());});
    commit_ack_sub_=create_subscription<wire::ExecutionCommitAck>(topic("commit_ack_topic","/d1max/live_planning/execution/commit_ack"),rclcpp::QoS(8),
      [this](wire::ExecutionCommitAck::ConstSharedPtr m){
        if(execution_->commitAck(*m,now().seconds(),monotonicNow()))
          if(const auto installed=execution_->geometryReceipt())geometry_receipt_pub_->publish(*installed);
      });
    spline_sub_ = create_subscription<d1max_planning_interfaces::msg::TaggedBspline>(
        topic("trajectory_topic", "/d1max/pct_scan/planning/tagged_bspline"), rclcpp::QoS(1),
        [this](const d1max_planning_interfaces::msg::TaggedBspline::ConstSharedPtr msg) {
          const auto &spline = msg->trajectory;
          Trajectory input;
          input.session_id = msg->session_id;
          input.generation = msg->generation;
          input.frame_id = msg->frame_id;
          input.point_reference = msg->point_reference;
          input.id = spline.traj_id;
          input.start_time = stampSeconds(spline.start_time);
          input.order = spline.order;
          input.identity.schema_version = msg->schema_version;
          input.identity.task_id = msg->task_id;
          input.identity.route_id = msg->route_id;
          input.identity.route_hash = msg->route_hash;
          input.identity.segment_id = msg->segment_id;
          input.identity.map_version_id = msg->map_version_id;
          input.identity.anchor_id = msg->anchor_id;
          input.identity.anchor_revision = msg->anchor_revision;
          input.identity.map_geometry_revision = msg->map_geometry_revision;
          input.identity.context_sequence = msg->context_sequence;
          input.identity.localization_epoch = msg->localization_epoch;
          input.identity.localization_seed_id = msg->localization_seed_id;
          input.valid_start_time=msg->valid_start_time;
          input.valid_start_arc_length=msg->valid_start_arc_length;
          input.join_source_stamp=stampSeconds(msg->join_source_stamp);
          input.join_source_stamp_ns=timeNs(msg->join_source_stamp);
          const auto& p=msg->join_pose.position; const auto& q=msg->join_pose.orientation;
          const auto& v=msg->join_twist.linear; const auto& a=msg->join_acceleration.linear;
          input.join_position={p.x,p.y,p.z}; input.join_orientation={q.w,q.x,q.y,q.z};
          input.join_velocity={v.x,v.y,v.z}; input.join_acceleration={a.x,a.y,a.z};
          input.join_acceleration_valid=msg->join_acceleration_valid;
          input.knots = spline.knots;
          input.points.reserve(spline.pos_pts.size());
          for (const auto &point : spline.pos_pts) input.points.emplace_back(point.x, point.y, point.z);
          execution_->candidate(std::move(input));
        });
    const bool local_state_enabled=declare_parameter<bool>("local_state_enabled",false);
    if(local_state_enabled) {
      local_state_sub_=create_subscription<d1max_planning_interfaces::msg::LocalNavigationState>(
        topic("local_navigation_state_topic","/d1max/localization/navigation/local_state"),rclcpp::QoS(1),
        [this](const d1max_planning_interfaces::msg::LocalNavigationState::ConstSharedPtr msg) {
          if(!state_ingress_.inspect(*msg,config_,now().seconds(),core_->task().identity.localization_epoch))return;
          Odom input;std::string reason;
          if(!decodeLocalNavigationState(*msg,config_,input,reason)) {
            rejectedState(*msg,reason);return;
          }
          if(execution_->receiveOdom(input,now().seconds(),monotonicNow()))odom_source_stamp_=msg->source_stamp;
        });
    } else {
      // Legacy isolated fixtures supply an atomic pair. Formal local transport
      // never falls back to it: a soft global unavailable message cannot clear
      // local state, and a missing local stream cannot silently restamp global.
      state_sub_=create_subscription<d1max_planning_interfaces::msg::NavigationState>(
        topic("navigation_state_topic","/d1max/localization/navigation/state"),rclcpp::QoS(1),
        [this](const d1max_planning_interfaces::msg::NavigationState::ConstSharedPtr msg) {
          if(!state_ingress_.inspect(*msg,config_,now().seconds(),core_->task().identity.localization_epoch))return;
          Odom input;
          std::string reason;
          if (!decodeNavigationState(*msg,config_,input,reason)) {
            rejectedState(*msg,reason);return;
          }
          if(execution_->receiveOdom(input, now().seconds(), monotonicNow())) odom_source_stamp_=msg->source_stamp;
        });
    }
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        topic("stop_service", "/d1max/pct_scan/tracker_stop"),
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          execution_->cancel("operator_stopped");
          publish(core_->step(now().seconds(), monotonicNow()));
          response->success = true;
          response->message = "Stopped. A new task generation is required.";
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      const double ros_now=now().seconds(),receipt=monotonicNow();
      const bool preparation_ready=execution_->pollPreparation();
      // Fallback refresh only; a new validation prepares immediately.
      if(preparation_ready||receipt-last_prepare_>=.2) prepareAndPublish(ros_now,receipt);
      // Both controllers see the prior applied limiter history. Computing the
      // incumbent first would advance its clock to this tick and give the
      // prepared controller a zero dt, manufacturing a HOLD every 50 Hz tick.
      if(const auto prepared=execution_->preparedStep(ros_now,receipt))prepared_pub_->publish(*prepared);
      const auto demand=execution_->step(ros_now,receipt);
      demand_pub_->publish(demand);
      Output out;out.forward=demand.velocity.linear.x;out.yaw_rate=demand.velocity.angular.z;
      out.frozen=demand.hold;out.reason=demand.reason;publish(out);
    });
    RCLCPP_INFO(get_logger(), "Guarded SCAN tracker ready; idle until an explicit session task. Max %.2f m/s.", c.max_speed);
  }

private:
  template<class State> void rejectedState(const State& state,const std::string& reason) {
    const auto& expected=core_->task().identity;
    if(core_->active()&&navigationContextReset(state,config_,expected,now().seconds()))
      execution_->cancel("odometry_context_changed");
    else core_->hold(reason,monotonicNow());
  }
  void prepareAndPublish(double ros_now,double receipt)
  {
    last_prepare_=receipt;
    if(const auto admission=execution_->prepare(ros_now,receipt)) {
      admission_pub_->publish(*admission);
      if(admission->accepted) ++accepted_; else ++rejected_;
    }
  }
  void taskCallback(const std_msgs::msg::String &message)
  {
    try {
      const Json data = Json::parse(message.data);
      Task task;
      task.session_id = data.at("session_id").get<std::string>();
      if (!data.at("generation").is_number_unsigned()) throw std::invalid_argument("generation is not uint");
      task.generation = data.at("generation").get<std::uint64_t>();
      task.active = data.at("active").get<bool>();
        const auto& identity = data.at("control_identity");
        task.identity.schema_version = identity.at("schema_version").get<std::uint32_t>();
        task.identity.task_id = identity.at("task_id").get<std::string>();
        task.identity.route_id = identity.at("route_id").get<std::string>();
        task.identity.route_hash = identity.at("route_hash").get<std::string>();
        task.identity.segment_id = identity.at("segment_id").get<std::string>();
        task.identity.map_version_id = identity.at("map_version_id").get<std::string>();
        task.identity.anchor_id = identity.at("anchor_id").get<std::string>();
        task.identity.anchor_revision = identity.at("anchor_revision").get<std::uint64_t>();
        task.identity.map_geometry_revision = identity.value("map_geometry_revision",std::uint64_t(0));
        task.identity.context_sequence = identity.at("context_sequence").get<std::uint64_t>();
        task.identity.localization_epoch = identity.at("localization_epoch").get<std::uint64_t>();
        task.identity.localization_seed_id = identity.at("localization_seed_id").get<std::string>();
      if (task.active) {
        task.frame_id = data.at("frame_id").get<std::string>();
        task.issued_at = data.at("issued_at").get<double>();
        const auto &xyz = data.at("target_xyz");
        if (!xyz.is_array() || xyz.size() != 3) throw std::invalid_argument("target_xyz must have three entries");
        task.goal = Eigen::Vector3d(xyz[0].get<double>(), xyz[1].get<double>(), xyz[2].get<double>());
      }
      core_->receiveTask(task, now().seconds(), monotonicNow());
    } catch (const std::exception &) {
      core_->cancel("malformed_task");
      publish(core_->step(now().seconds(), monotonicNow()));
    }
  }

  void publish(const Output &output)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = output.forward;
    command.angular.z = output.yaw_rate;
    command_pub_->publish(command);
    std_msgs::msg::Bool frozen;
    frozen.data = output.frozen;
    frozen_pub_->publish(frozen);
    const auto measured = core_->progress(now().seconds());
    d1max_planning_interfaces::msg::TrackingProgress progress;
    progress.header.stamp = odom_source_stamp_;
    progress.header.frame_id = config_.planning_frame;
    progress.schema_version = 2;
    progress.session_id = config_.session_id;
    progress.generation = core_->generation();
    progress.trajectory_id = core_->trajectoryId();
    progress.task_id = measured.identity.task_id;
    progress.route_id = measured.identity.route_id;
    progress.route_hash = measured.identity.route_hash;
    progress.segment_id = measured.identity.segment_id;
    progress.map_version_id = measured.identity.map_version_id;
    progress.localization_epoch = measured.identity.localization_epoch;
    progress.localization_seed_id = measured.identity.localization_seed_id;
    progress.anchor_id = measured.identity.anchor_id;
    progress.anchor_revision = measured.identity.anchor_revision;
    progress.context_sequence = measured.identity.context_sequence;
    progress.curve_time = measured.curve_time;
    progress.arc_length = measured.arc_length;
    progress.s_committed = measured.s_committed;
    progress.pose.position.x=measured.position.x(); progress.pose.position.y=measured.position.y();
    progress.pose.position.z=measured.position.z();
    progress.pose.orientation.x=measured.orientation.x(); progress.pose.orientation.y=measured.orientation.y();
    progress.pose.orientation.z=measured.orientation.z(); progress.pose.orientation.w=measured.orientation.w();
    progress.twist.linear.x=measured.velocity_in_frame.x(); progress.twist.linear.y=measured.velocity_in_frame.y();
    progress.twist.linear.z=measured.velocity_in_frame.z();
    progress.twist.angular.x=measured.angular_velocity_in_frame.x(); progress.twist.angular.y=measured.angular_velocity_in_frame.y();
    progress.twist.angular.z=measured.angular_velocity_in_frame.z();
    progress.holding = output.frozen;
    progress.valid = measured.valid;
    progress.reason = measured.reason;
    // Acceleration is not in Odometry; do not substitute planned derivatives.
    progress.acceleration_valid = false;
    progress_pub_->publish(progress);
    const auto installed=execution_->geometryReceipt();
    if(installed)geometry_receipt_pub_->publish(*installed);
    const auto& join=core_->joinDiagnostic();
    Json status = {
      // Process heartbeat for lifecycle supervision, not sensor provenance.
      {"callback_wall_time",std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count()},
      {"session_id", config_.session_id}, {"generation", core_->generation()},
      {"active", core_->active()}, {"finished", output.finished}, {"reason", output.reason},
      {"trajectory_id", core_->trajectoryId()}, {"accepted_trajectories", accepted_},
      {"rejected_trajectories", rejected_}, {"frame_id", config_.planning_frame},
      {"prepare_reason", execution_->lastPrepareReason()},
      {"preparation_pending",execution_->preparationPending()},
      {"preparation_worker_sec",execution_->preparationWorkerSeconds()},
      {"handoff_reason",execution_->handoffReason()},
      {"writer_commit_sequence",execution_->writerCommitSequence()},
      {"permit_reject_reason", execution_->lastPermitReject()},
      {"initial_writer_ack_reason",execution_->initialAckReason()},
      {"installation_sequence",installed?installed->installation_sequence:0},
      {"join_diagnostic",{{"reason",join.reason},{"trajectory_id",join.trajectory_id},
        {"body_present",join.body_present},{"checked_now",join.now},
        {"body_source_stamp_ns",join.body_source_stamp_ns},{"join_source_stamp_ns",join.join_source_stamp_ns},
        {"entry_reobserved",join.entry_reobserved},{"original_join_source_stamp_ns",join.original_join_source_stamp_ns},
        {"body_source_age_sec",join.now-join.body_stamp},{"join_source_age_sec",join.now-join.join_stamp},
        {"posterior_source_age_sec",join.now-join.posterior_stamp},{"imu_source_age_sec",join.now-join.imu_stamp},
        {"extrapolation_sec",join.extrapolation_sec},{"join_minus_body_sec",join.join_stamp-join.body_stamp},
        {"curve_time",join.curve_time},{"curve_duration",join.curve_duration},{"arc",join.arc}}},
      {"command", {{"x", output.forward}, {"y", 0.0}, {"yaw", output.yaw_rate}}},
      {"execution_frozen", output.frozen}, {"max_speed", config_.max_speed},
      {"maneuver_phase", core_->maneuverPhase()},
      {"hard_planar_limit", HARD_PLANAR_SPEED}, {"stamp", now().seconds()}
    };
    std_msgs::msg::String message;
    message.data = status.dump();
    status_pub_->publish(message);
  }
  Config config_;
  NavigationStateIngress state_ingress_;
  builtin_interfaces::msg::Time odom_source_stamp_;
  std::unique_ptr<ExecutionContract> execution_;
  TrackerCore* core_{nullptr};
  double last_prepare_{0.};
  rclcpp::Publisher<wire::MotionDemand>::SharedPtr demand_pub_;
  rclcpp::Publisher<wire::PreparedMotionDemand>::SharedPtr prepared_pub_;
  rclcpp::Publisher<wire::TrajectoryAdmission>::SharedPtr admission_pub_;
  rclcpp::Publisher<wire::TrackerGeometryReceipt>::SharedPtr geometry_receipt_pub_;
  rclcpp::Publisher<wire::MotionValidation>::SharedPtr blocked_entry_pub_;
  rclcpp::Subscription<wire::ReferenceProposal>::SharedPtr proposal_sub_;
  rclcpp::Subscription<wire::SupportReference>::SharedPtr support_sub_;
  rclcpp::Subscription<wire::TrajectoryValidation>::SharedPtr validation_sub_;
  rclcpp::Subscription<wire::MotionValidation>::SharedPtr motion_validation_sub_;
  rclcpp::Subscription<wire::ExecutionPermit>::SharedPtr permit_sub_;
  rclcpp::Subscription<wire::ExecutionHandoffGrant>::SharedPtr handoff_sub_;
  rclcpp::Subscription<wire::ExecutionCommitAck>::SharedPtr commit_ack_sub_;
  std::uint64_t accepted_{0}, rejected_{0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<d1max_planning_interfaces::msg::TrackingProgress>::SharedPtr progress_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::TaggedBspline>::SharedPtr spline_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::NavigationState>::SharedPtr state_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::LocalNavigationState>::SharedPtr local_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace d1max_trajectory_tracker

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<d1max_trajectory_tracker::TrackerNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_tracker"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
