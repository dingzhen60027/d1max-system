
#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <stdexcept>
#include <optional>
#include <plan_manage/input_contract.hpp>
#include <plan_manage/local_plan_debug.hpp>
#include <plan_manage/reference_target.hpp>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    strict_input_frames_ = load_parameter<bool>(node_, "fsm.strict_input_frames", false);
    odom_twist_in_body_frame_ = load_parameter<bool>(node_, "fsm.odom_twist_in_body_frame", false);
    reference_path_z_offset_ = load_parameter<double>(node_, "fsm.reference_path_z_offset", body_height_);
    reference_start_tolerance_ = load_parameter<double>(node_, "fsm.reference_start_tolerance", 1.0);
    odom_timeout_ = load_parameter<double>(node_, "fsm.odom_timeout", 0.5);
    max_replan_interval_ = load_parameter<double>(node_, "fsm.max_replan_interval", 0.0);
    failed_replan_cooldown_=load_parameter<double>(node_, "fsm.failed_replan_cooldown", .5);
    failed_replan_body_distance_=load_parameter<double>(node_, "fsm.failed_replan_body_distance", .15);
    reference_goal_xy_tolerance_=load_parameter<double>(node_, "fsm.reference_goal_xy_tolerance", .20);
    reference_goal_z_tolerance_=load_parameter<double>(node_, "fsm.reference_goal_z_tolerance", .15);
    if (!std::isfinite(reference_goal_xy_tolerance_) || reference_goal_xy_tolerance_<.02 ||
        reference_goal_xy_tolerance_>.30 || !std::isfinite(reference_goal_z_tolerance_) ||
        reference_goal_z_tolerance_<.01 || reference_goal_z_tolerance_>.20)
      throw std::runtime_error("invalid measured reference goal tolerance");
    reference_target_forward_margin_=load_parameter<double>(node_, "fsm.reference_target_forward_margin", 1.0);
    reference_target_backward_margin_=load_parameter<double>(node_, "fsm.reference_target_backward_margin", 2.0);
    reference_target_min_advance_=load_parameter<double>(node_, "fsm.reference_target_min_advance", .3);
    reference_target_exit_margin_=load_parameter<double>(node_, "fsm.reference_target_exit_margin", .2);
    if (!std::isfinite(reference_target_forward_margin_) || reference_target_forward_margin_<0. || reference_target_forward_margin_>3. ||
        !std::isfinite(reference_target_backward_margin_) || reference_target_backward_margin_<0. || reference_target_backward_margin_>5. ||
        !std::isfinite(reference_target_min_advance_) || reference_target_min_advance_<.2 || reference_target_min_advance_>1. ||
        !std::isfinite(reference_target_exit_margin_) || reference_target_exit_margin_<0. || reference_target_exit_margin_>1.)
      throw std::runtime_error("invalid bounded reference target policy");
    if (!std::isfinite(failed_replan_cooldown_) || failed_replan_cooldown_<.2 || failed_replan_cooldown_>5. ||
        !std::isfinite(failed_replan_body_distance_) || failed_replan_body_distance_<.05 || failed_replan_body_distance_>.5)
      throw std::runtime_error("invalid bounded failed replan policy");
    reference_path_guidance_ = load_parameter<bool>(node_, "fsm.reference_path_guidance", false);
    if (reference_path_guidance_ && navi_mode_ != NAVI_MODE::REFERENCE_PATH)
      throw std::runtime_error("reference_path_guidance is only valid in navigation mode 3");
    if (!std::isfinite(max_replan_interval_) || max_replan_interval_ < 0.0)
      throw std::runtime_error("max_replan_interval must be finite and nonnegative");
    require_tagged_reference_ = load_parameter<bool>(node_, "fsm.require_tagged_reference", false);
    navigation_session_id_ = load_parameter<std::string>(node_, "fsm.navigation_session_id", "");
    if (require_tagged_reference_ && (navigation_session_id_.empty() || navi_mode_ != NAVI_MODE::REFERENCE_PATH))
      throw std::runtime_error("tagged reference requires mode 3 and a non-empty navigation session id");
    if (!std::isfinite(reference_path_z_offset_) || reference_start_tolerance_ <= 0.0 || odom_timeout_ <= 0.0)
      throw std::runtime_error("invalid SCAN input contract parameters");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);
    // Zero keeps the upstream smoothness/collision/feasibility objective.
    // Optional reference attraction is a D1 extension, not a mode-3 requirement.

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    if (max_replan_interval_ > 0.0)
      cloud_health_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          "cloud", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            const double age = (node_->now() - rclcpp::Time(msg->header.stamp)).seconds();
            have_fresh_cloud_ = msg->header.frame_id == self_inflation_frame_id_ &&
                msg->width > 0 && msg->height > 0 && !msg->data.empty() &&
                age >= -0.1 && age <= odom_timeout_;
            if (have_fresh_cloud_) last_cloud_time_ = node_->now();
          });
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    tagged_bspline_pub_ = node_->create_publisher<d1max_planning_interfaces::msg::TaggedBspline>(
        "planning/tagged_bspline", rclcpp::QoS(1).reliable());
    if (reference_path_guidance_ && require_tagged_reference_)
    {
      local_plan_debug_pub_ = node_->create_publisher<d1max_planning_interfaces::msg::LocalPlanDebug>(
          "planning/local_plan_debug", rclcpp::QoS(1).reliable());
      local_attempt_debug_pub_=node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "planning/local_attempt_debug", rclcpp::QoS(1).reliable());
    }
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(2).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      if (require_tagged_reference_)
        typed_path_sub_ = node_->create_subscription<d1max_planning_interfaces::msg::ReferencePath>(
            "typed_initial_path", rclcpp::QoS(1).reliable(),
            std::bind(&SCANReplanFSM::typedPathCallback, this, std::placeholders::_1));
      else
        path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    }
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();
    if (reference_path_guidance_)
    {
      std::vector<Eigen::Vector3d> full_reference{odom_pos_};
      full_reference.insert(full_reference.end(), waypoints.begin(), waypoints.end());
      discrete_reference_.set(full_reference);
      discrete_progress_ = 0.0;
      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      visualization_->displayGlobalPathList(full_reference, 0.1, 0);
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
      return true;
    }

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    // Preserve the accepted PCT endpoint; do not silently shorten its task.
    if (reference_path_guidance_) return true;
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::typedPathCallback(
      const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr &msg)
  {
    if (!acceptsReferenceGeneration(navigation_session_id_, msg->session_id,
                                    have_reference_generation_, reference_generation_, msg->generation))
    {
      RCLCPP_WARN(node_->get_logger(), "Reject reference from wrong session or old/duplicate generation");
      return;
    }
    // SingleThreadedExecutor: input replacement and trajectory publication are
    // serialized. Revoke the previous route even when the new route is invalid.
    have_reference_generation_ = true;
    reference_generation_ = msg->generation;
    accepted_curve_id_ = -1;
    predecessor_id_ = 0;
    predecessor_safe_ = false;
    predecessor_check_stamp_ = builtin_interfaces::msg::Time{};
    publishAttemptDebug(localPlanDebugHeader(), true);
    publishInvalidLocalPlanDebug(msg->path.poses.empty() ? "cancelled" : "reference_replaced", true);
    local_debug_selected_reference_.clear();
    local_target_query_debug_=ReferenceTargetResult{};
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    active_waypoints_.clear();
    // A fresh generation owns its own planning attempts. Failures from the
    // previous task must not send a newly accepted route into emergency wait.
    replan_fail_count_ = 0;
    need_hover_stop_ = false;
    flag_escape_emergency_ = true;
    changeFSMExecState(WAIT_TARGET, "REFERENCE_REPLACE");
    if (msg->path.poses.empty())
    {
      if (have_odom_ && (node_->now() - last_odom_time_).seconds() <= odom_timeout_)
        callEmergencyStop(odom_pos_);
      RCLCPP_INFO(node_->get_logger(), "Reference generation %lu canceled", reference_generation_);
      return;
    }
    pathCallback(std::make_shared<nav_msgs::msg::Path>(msg->path));
  }

  void SCANReplanFSM::publishTrajectory(const scan_planner_msgs::msg::Bspline &trajectory)
  {
    bspline_pub_->publish(trajectory);  // legacy/debug only; not a motion authority
    if (require_tagged_reference_ && have_reference_generation_)
    {
      d1max_planning_interfaces::msg::TaggedBspline tagged;
      tagged.session_id = navigation_session_id_;
      tagged.generation = reference_generation_;
      tagged.frame_id = self_inflation_frame_id_;
      tagged.trajectory = trajectory;
      tagged_bspline_pub_->publish(tagged);
    }
  }

  std_msgs::msg::Header SCANReplanFSM::localPlanDebugHeader()
  {
    std_msgs::msg::Header header;
    const auto clock_type = node_->get_clock()->get_clock_type();
    const auto stamp_ns = std::max(node_->now().nanoseconds(), last_local_debug_stamp_ns_ + 1);
    last_local_debug_stamp_ns_ = stamp_ns;
    header.stamp = rclcpp::Time(stamp_ns, clock_type);
    header.frame_id = self_inflation_frame_id_;
    return header;
  }

  void SCANReplanFSM::publishInvalidLocalPlanDebug(const std::string &phase, bool force)
  {
    if (!local_plan_debug_pub_) return;
    if (!force && have_local_debug_state_ && last_local_debug_phase_ == phase &&
        last_local_debug_generation_ == reference_generation_) return;
    const auto plan_id = static_cast<std::uint64_t>(std::max(0, planner_manager_->local_data_.traj_id_));
    local_plan_debug_pub_->publish(makeInvalidLocalPlanDebug(
        localPlanDebugHeader(), navigation_session_id_, reference_generation_, plan_id, phase));
    have_local_debug_state_ = true;
    last_local_debug_phase_ = phase;
    last_local_debug_generation_ = reference_generation_;
  }

  void SCANReplanFSM::publishAcceptedLocalPlanDebug(std::uint64_t plan_id)
  {
    if (!local_plan_debug_pub_) return;
    const auto &selected = local_debug_selected_reference_;
    const auto header = localPlanDebugHeader();
    auto message = makeAcceptedLocalPlanDebug(
        header, navigation_session_id_, reference_generation_, plan_id,
        local_debug_projection_, local_target_pt_, discrete_progress_,
        local_debug_target_arc_, selected);
    if (message)
    {
      message->predecessor_id = predecessor_id_;
      message->predecessor_safe = predecessor_safe_;
      message->predecessor_check_stamp = predecessor_check_stamp_;
      local_plan_debug_pub_->publish(*message);
      accepted_curve_generation_ = reference_generation_;
      accepted_curve_id_ = static_cast<std::int64_t>(plan_id);
      have_local_debug_state_ = true;
      last_local_debug_phase_ = "accepted";
      last_local_debug_generation_ = reference_generation_;
      return;
    }
    const auto reason = selected.size() > kMaxLocalDebugReferencePoints ? "debug_overflow" : "debug_invalid";
    local_plan_debug_pub_->publish(makeInvalidLocalPlanDebug(
        header, navigation_session_id_, reference_generation_, plan_id, reason));
    have_local_debug_state_ = true;
    last_local_debug_phase_ = reason;
    last_local_debug_generation_ = reference_generation_;
  }

  void SCANReplanFSM::publishAttemptDebug(const std_msgs::msg::Header &header, bool clear_only,
                                         bool include_optimizer_diagnostics)
  {
    if (!local_attempt_debug_pub_) return;
    visualization_msgs::msg::MarkerArray array;
    const std::string prefix="local_attempt/"+navigation_session_id_+"/"+
        std::to_string(reference_generation_)+"/";
    visualization_msgs::msg::Marker clear;
    clear.header=header; clear.ns=prefix+"reference";
    clear.pose.orientation.w=1.;
    clear.action=visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    const double remaining=2.-(node_->now()-rclcpp::Time(header.stamp)).seconds();
    if (!clear_only && remaining>0.) {
      std::size_t point_budget=4096;
      const auto add=[&](const std::string &kind, int type,
          const std::vector<Eigen::Vector3d> &points, double scale,
          float red, float green, float blue) {
        if (points.empty() || points.size()>point_budget) return;
        visualization_msgs::msg::Marker marker;
        marker.header=header; marker.ns=prefix+kind; marker.id=0;
        marker.type=type; marker.action=visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w=1.;
        marker.scale.x=scale; marker.scale.y=scale; marker.scale.z=scale;
        marker.color.r=red; marker.color.g=green; marker.color.b=blue; marker.color.a=.9;
        marker.lifetime=rclcpp::Duration::from_seconds(std::min(2., remaining));
        for (const auto &p:points) {
          if (!p.allFinite()) return;
          geometry_msgs::msg::Point point; point.x=p.x(); point.y=p.y(); point.z=p.z();
          marker.points.push_back(point);
        }
        array.markers.push_back(std::move(marker));
        point_budget-=points.size();
      };
      // Endpoint rejection happens before reboundReplan. Show the actual
      // rejected queries from this attempt, not an earlier optimizer's points.
      // Give these points budget priority so a dense reference cannot hide them.
      if (!include_optimizer_diagnostics)
        add("blocked", visualization_msgs::msg::Marker::POINTS,
            local_target_query_debug_.blocked_points, .08, 1.f, .1f, .1f);
      add("reference", visualization_msgs::msg::Marker::LINE_STRIP,
          local_debug_selected_reference_, .025, 1.f, .55f, .08f);
      add("target", visualization_msgs::msg::Marker::POINTS,
          std::vector<Eigen::Vector3d>{local_target_pt_}, .15, 1.f, .4f, 0.f);
      if (include_optimizer_diagnostics) {
        add("blocked", visualization_msgs::msg::Marker::POINTS,
            planner_manager_->attemptBlockedPoints(), .08, 1.f, .1f, .1f);
        add("detour", visualization_msgs::msg::Marker::LINE_STRIP,
            planner_manager_->attemptDetourSeed(), .03, 1.f, 0.f, .85f);
      }
    }
    local_attempt_debug_pub_->publish(array);
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    if (!have_odom_ || (node_->now() - last_odom_time_).seconds() > odom_timeout_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore reference path: no fresh body odometry");
      return;
    }
    if (strict_input_frames_ && msg->header.frame_id != self_inflation_frame_id_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference path frame does not match grid_map.frame_id");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      if (strict_input_frames_ && !pose_stamped.header.frame_id.empty() &&
          pose_stamped.header.frame_id != self_inflation_frame_id_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Reference path contains a mismatched pose frame");
        return;
      }
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z;
      waypoints.push_back(wp);
    }
    try
    {
      waypoints = prepareReferenceWaypoints(waypoints, odom_pos_, reference_path_z_offset_,
                                           reference_start_tolerance_);
    }
    catch (const std::invalid_argument &error)
    {
      RCLCPP_ERROR(node_->get_logger(), "Reject reference path: %s", error.what());
      return;
    }
    trigger_ = true;
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    const double age = (node_->now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (strict_input_frames_ && (msg->header.frame_id != self_inflation_frame_id_ ||
                                age < -0.1 || age > odom_timeout_))
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Reject body odometry: world-frame mismatch or stale/future timestamp");
      return;
    }
    const Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
    Eigen::Quaterniond orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d velocity;
    try
    {
      if (!position.allFinite()) throw std::invalid_argument("non-finite position");
      velocity = odometryVelocityInWorld(
          Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z), orientation, odom_twist_in_body_frame_);
      orientation.normalize();
    }
    catch (const std::invalid_argument &error)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Reject body odometry: %s", error.what());
      return;
    }
    last_odom_time_ = node_->now();
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_ = velocity;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_ = orientation;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    // Obstacle inflation is [-z_down,+z_up]; the equivalent body envelope
    // is its reflection [-z_up,+z_down]. This only corrects the RViz marker.
    center(2) += 0.5 * (z_down - z_up);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "WAIT_ENVIRONMENT"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "WAIT_ENVIRONMENT"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();
    // A downstream controller must also timeout its own odometry/trajectory.
    // Do not generate apparently fresh trajectories from a stale robot pose.
    if (have_odom_ && (node_->now() - last_odom_time_).seconds() > odom_timeout_)
      return;
    // One measured XYZ criterion in every active reference state. A spline
    // clock endpoint cannot establish physical arrival, and a replan at an
    // already reached goal must not enter a permanent optimizer failure wait.
    if (reference_path_guidance_ && have_target_ && have_odom_ &&
        (exec_state_==GEN_NEW_TRAJ || exec_state_==REPLAN_TRAJ ||
         exec_state_==EXEC_TRAJ || exec_state_==WAIT_ENVIRONMENT) &&
        measuredReferenceGoalReached(odom_pos_, end_pt_,
            reference_goal_xy_tolerance_, reference_goal_z_tolerance_)) {
      have_target_=false;
      publishInvalidLocalPlanDebug("completed");
      publishAttemptDebug(localPlanDebugHeader(), true);
      changeFSMExecState(WAIT_TARGET, "MEASURED_GOAL_REACHED");
      return;
    }
    if (max_replan_interval_ > 0.0 || reference_path_guidance_) {
      const double map_stamp=planner_manager_->grid_map_->latestCloudStamp();
      const double age=node_->now().seconds()-map_stamp;
      if (map_stamp<=0. || age<-.1 || age>odom_timeout_) {
        publishInvalidLocalPlanDebug("waiting_sensor_map");
        return;
      }
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        if (reference_path_guidance_) waitForChangedEnvironment();
        else {
          publishInvalidLocalPlanDebug("failed");
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        if (reference_path_guidance_) waitForChangedEnvironment();
        else {
          publishInvalidLocalPlanDebug("failed");
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case WAIT_ENVIRONMENT:
    {
      if (!have_target_) { changeFSMExecState(WAIT_TARGET, "NO_TARGET"); break; }
      const double mono=std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      if (mono-failed_replan_monotonic_<failed_replan_cooldown_) return;
      const bool new_supported_map=last_attempt_failure_phase_=="waiting_sensor_map" &&
          planner_manager_->grid_map_->latestCloudStamp()>failed_map_stamp_;
      const bool dynamics_recovered=last_attempt_failure_phase_=="failed_dynamics" &&
          failedDynamicsBoundaryChanged(failed_body_velocity_, odom_vel_, planner_manager_->pp_.max_vel_);
      if (!new_supported_map && !dynamics_recovered &&
          planner_manager_->grid_map_->occupancyRevision()==failed_environment_revision_ &&
          (odom_pos_-failed_body_position_).norm()<failed_replan_body_distance_) return;
      changeFSMExecState(GEN_NEW_TRAJ, "ENVIRONMENT_CHANGED");
      break;
    }

    case EXEC_TRAJ:
    {
      // A frozen body still needs a live, validated optimizer heartbeat. This
      // produces a new real trajectory rather than refreshing stale messages.
      if (periodicReplanDue(node_->now().seconds(), last_replan_time_.seconds(), max_replan_interval_))
      {
        changeFSMExecState(REPLAN_TRAJ, "PERIODIC");
        return;
      }
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (reference_path_guidance_) {
          // The local time horizon ended while the measured body is still
          // outside the common goal tolerance. Produce another real segment.
          changeFSMExecState(REPLAN_TRAJ, "LOCAL_HORIZON_ENDED");
          return;
        }
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;
        publishInvalidLocalPlanDebug("completed");

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (reference_path_guidance_) return; // One deterministic solve per changed environment, never hover-as-navigation.
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  void SCANReplanFSM::waitForChangedEnvironment()
  {
    failed_replan_monotonic_=std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    failed_environment_revision_=planner_manager_->grid_map_->occupancyRevision();
    failed_map_stamp_=planner_manager_->grid_map_->latestCloudStamp();
    failed_body_position_=odom_pos_;
    failed_body_velocity_=odom_vel_;
    const auto &reason=last_attempt_failure_phase_;
    publishInvalidLocalPlanDebug(reason.empty() ? "failed_optimization" : reason);
    RCLCPP_WARN(node_->get_logger(), "Local navigation waiting for changed environment: %s",
                reason.c_str());
    changeFSMExecState(WAIT_ENVIRONMENT, "BOUNDED_FAILURE");
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    if (go2_execution_frozen_ || reference_path_guidance_)
    {
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (shouldSuppressOpposedPrediction(
            go2_execution_frozen_ || reference_path_guidance_, to_goal, start_vel_))
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // In reference mode the complete PCT route, including its progress index,
    // must survive every local replan. Replacing it with a direct polynomial
    // to the final goal silently discards corridor turns and topology.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH && !planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      if (reference_path_guidance_) return false; // No identical pseudo-random second seed.
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    if (reference_path_guidance_ || go2_execution_frozen_) return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (shouldSuppressOpposedPrediction(false, to_goal, start_vel_))
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();
    if (!have_odom_ || (node_->now() - last_odom_time_).seconds() > odom_timeout_)
      return;
    if (max_replan_interval_ > 0.0 || reference_path_guidance_) {
      const double stamp=planner_manager_->grid_map_->latestCloudStamp();
      const double age=node_->now().seconds()-stamp;
      if (stamp<=0. || age<-.1 || age>odom_timeout_) return;
    }

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || exec_state_ == WAIT_ENVIRONMENT || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    constexpr int maximum_samples=20000;
    const double raw_t_cur=(node_->now()-info->start_time_).seconds();
    if (!std::isfinite(info->duration_) || info->duration_<=0. ||
        !std::isfinite(raw_t_cur) || raw_t_cur<-.1) {
      last_attempt_failure_phase_="failed_final_collision";
      if (reference_path_guidance_) waitForChangedEnvironment();
      else changeFSMExecState(EMERGENCY_STOP,"INVALID_TRAJECTORY_TIME");
      return;
    }
    const double t_cur=std::clamp(raw_t_cur,0.,info->duration_);
    const double remaining=info->duration_-t_cur;
    if (remaining/time_step>maximum_samples-1) {
      // Fail closed instead of silently making the collision sampling coarser.
      last_attempt_failure_phase_="failed_final_collision";
      if (reference_path_guidance_) waitForChangedEnvironment();
      else changeFSMExecState(EMERGENCY_STOP,"COLLISION_SAMPLE_BUDGET");
      return;
    }
    const int intervals=std::max(1,static_cast<int>(std::ceil(remaining/time_step)));
    for (int sample=0; sample<=intervals; ++sample)
    {
      const double t=t_cur+remaining*static_cast<double>(sample)/intervals;
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (reference_path_guidance_) {
            waitForChangedEnvironment();
            return;
          }
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    predecessor_id_ = 0;
    predecessor_safe_ = false;
    predecessor_check_stamp_ = builtin_interfaces::msg::Time{};
    last_replan_time_ = node_->now();
    const auto attempt_header=localPlanDebugHeader();
    planner_manager_->grid_map_->resetCollisionDiagnostics();
    local_target_query_debug_=ReferenceTargetResult{};

    if (!getLocalTarget()) {
      if (planner_manager_->grid_map_->requiresObservedFree() &&
          planner_manager_->grid_map_->unknownCollisionQueries()>0)
        last_attempt_failure_phase_="waiting_observed_space";
      const auto &query=local_target_query_debug_;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Reference target unavailable: %s queries=%zu free=%zu unknown=%zu occupied=%zu outside=%zu "
          "first_blocked_present=%d first_blocked=[%.3f,%.3f,%.3f] value=%d shown_points=%zu",
          last_attempt_failure_phase_.c_str(), query.queries, query.free_queries,
          query.unknown_queries, query.occupied_queries, query.outside_queries,
          query.has_first_blocked ? 1:0, query.first_blocked.x(), query.first_blocked.y(),
          query.first_blocked.z(), query.first_blocked_value, query.blocked_points.size());
      // No optimizer ran: never re-stamp an earlier attempt's blocked/seed data.
      have_new_target_=false;
      publishAttemptDebug(attempt_header, false, false);
      return false;
    }

    // At most one old curve copy, before a successful solve overwrites
    // local_data_. Never associate a curve from another goal generation.
    const auto preceding_generation = reference_generation_;
    const auto preceding_id = accepted_curve_id_;
    std::optional<UniformBspline> predecessor;
    if (reference_path_guidance_ && require_tagged_reference_ && preceding_id > 0 &&
        accepted_curve_generation_ == preceding_generation &&
        planner_manager_->local_data_.traj_id_ == preceding_id)
      predecessor.emplace(planner_manager_->local_data_.position_traj_);

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;
    last_attempt_failure_phase_=planner_manager_->lastFailurePhase();
    // A long solve must not turn expired input into a fresh accepted output.
    if (reference_path_guidance_ && plan_success) {
      const double map_stamp=planner_manager_->grid_map_->latestCloudStamp();
      const double map_age=node_->now().seconds()-map_stamp;
      if (!have_odom_ || (node_->now()-last_odom_time_).seconds()>odom_timeout_ ||
          map_stamp<=0. || map_age<-.1 || map_age>odom_timeout_) {
        plan_success=false;
        last_attempt_failure_phase_="waiting_sensor_map";
      }
    }
    publishAttemptDebug(attempt_header);

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      if (predecessor && reference_generation_ == preceding_generation &&
          planner_manager_->local_data_.traj_id_ > preceding_id) {
        predecessor_id_ = static_cast<std::uint64_t>(preceding_id);
        predecessor_safe_ = planner_manager_->recheckPredecessor(*predecessor);
        if (predecessor_safe_) predecessor_check_stamp_ = node_->now();
      }

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      publishTrajectory(bspline);
      publishAcceptedLocalPlanDebug(static_cast<std::uint64_t>(std::max(0, info->traj_id_)));

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    publishInvalidLocalPlanDebug("emergency_stop");

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    publishTrajectory(bspline);

    return true;
  }

  bool SCANReplanFSM::getLocalTarget()
  {
    if (reference_path_guidance_)
    {
      discrete_progress_ = discrete_reference_.project(
          start_pt_, discrete_progress_, std::min(discrete_reference_.length(), discrete_progress_ + 2.0));
      ReferenceTargetOptions options;
      options.forward_margin=reference_target_forward_margin_;
      options.backward_margin=reference_target_backward_margin_;
      options.min_advance=reference_target_min_advance_;
      options.exit_margin=reference_target_exit_margin_;
      options.sample_step=planner_manager_->grid_map_->getResolution()*.5;
      const auto selection=selectReferenceTarget(discrete_reference_, start_pt_,
          discrete_progress_, planning_horizon_, options,
          [this](const Eigen::Vector3d &point, double yaw) {
            const auto map=planner_manager_->grid_map_;
            const Eigen::Vector3d offset=self_double_cylinder_offset_*Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0.);
            const Eigen::Vector3d front=point+offset, rear=point-offset;
            // Test both centers first: the grid's occupied front center would
            // otherwise mask an out-of-map rear center due to short-circuiting.
            if (!map->isInMap(front) || !map->isInMap(rear)) return -1;
            return map->getInflateOccupancy(point,yaw);
          });
      const double target_arc=selection.arc;
      local_target_query_debug_=selection;
      local_debug_projection_ = discrete_reference_.sample(discrete_progress_);
      local_debug_target_arc_ = target_arc;
      local_target_pt_ = discrete_reference_.sample(target_arc);
      local_target_vel_.setZero();
      local_debug_selected_reference_ = discrete_reference_.slice(discrete_progress_, target_arc, start_pt_);
      if (!selection.valid) {
        last_attempt_failure_phase_=selection.reason;
        planner_manager_->setLocalReference({});
        return false;
      }
      if (target_arc < discrete_reference_.length() - 1e-4)
      {
        const Eigen::Vector3d tangent = discrete_reference_.sample(std::min(target_arc + .05, discrete_reference_.length())) -
                                        discrete_reference_.sample(std::max(0.0, target_arc - .05));
        if (tangent.norm() > 1e-6) local_target_vel_ = tangent.normalized() * planner_manager_->pp_.max_vel_;
      }
      planner_manager_->setLocalReference(local_debug_selected_reference_);
      return true;
    }
    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
    return true;
  }

} // namespace scan_planner
