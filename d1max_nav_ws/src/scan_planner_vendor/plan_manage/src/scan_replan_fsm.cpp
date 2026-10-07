
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/worker_lifecycle.hpp>
#include <cmath>
#include <stdexcept>
#include <optional>
#include <plan_manage/input_contract.hpp>
#include <plan_manage/local_plan_debug.hpp>
#include <plan_manage/reference_target.hpp>
#include <plan_manage/visible_reference_target.hpp>

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
    solve_worker_enabled_=load_parameter<bool>(node_,"fsm.solve_worker_enabled",true);
    require_schema_v2_=load_parameter<bool>(node_,"fsm.require_reference_schema_v2",false);
    if (solve_worker_enabled_ && reference_path_guidance_) {
      solve_worker_=std::make_unique<SCANPlannerManager>();
      solve_worker_->initSolveWorker(*planner_manager_);
      planner_manager_->releaseOwnerSolver();
    }
    if(load_parameter<bool>(node_,"fsm.execution_protocol",false)) {
      const auto mode=load_parameter<std::string>(node_,"fsm.execution_transport_mode","live");
      if(!require_schema_v2_ || !require_tagged_reference_ || std::abs(reference_path_z_offset_)>1e-9)
        throw std::runtime_error("execution protocol requires tagged body-center reference and zero z offset");
      std::optional<BrakingModel> braking;
      const auto braking_record=load_parameter<std::string>(node_,"fsm.execution_braking_model_record","");
      const auto braking_sha=load_parameter<std::string>(node_,"fsm.execution_braking_model_sha256","");
      if(!braking_record.empty()||!braking_sha.empty())braking=BrakingModel::load(braking_record,braking_sha,mode);
      else RCLCPP_WARN(node_->get_logger(),"No measured braking model: geometry preview only; motion proof unavailable");
      if(braking&&load_parameter<double>(node_,"grid_map.cloud_pose_max_age",.5)>braking->sensor_source_age)
        throw std::runtime_error("map sensor source age exceeds bound execution record");
      execution_validator_=std::make_unique<ExecutionValidator>(node_,snapshot_pool_,self_inflation_frame_id_,mode,
          self_double_cylinder_radius_+self_double_cylinder_offset_,std::move(braking),
          load_parameter<bool>(node_,"fsm.writer_handoff_enabled",false));
      execution_receipt_pub_=node_->create_publisher<ew::ReferenceReceipt>(
          "/d1max/live_planning/execution/reference_receipt",rclcpp::QoS(2));
      execution_support_sub_=node_->create_subscription<ew::SupportReference>(
          "/d1max/live_planning/execution/support",rclcpp::QoS(2),
          [this](ew::SupportReference::ConstSharedPtr m){execution_validator_->support(*m);});
      // Control and source-measured progress touch only the locked validator
      // ledger, never FSM/GridMap buffers. Neither waits behind map fusion.
      execution_control_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions control_options;control_options.callback_group=execution_control_group_;
      // Latest-body inbox for the independent validator: only copy trusted
      // source geometry under its mutex. FSM and production GridMap still
      // consume odometry on their one writer; map integration cannot delay this
      // safety reader's body stamp behind a 100+ ms ray transaction.
      execution_body_sub_=node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose",rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto& p=m->pose.pose.position;const auto& q=m->pose.pose.orientation;
          MeasuredBodyPose body{{p.x,p.y,p.z},Eigen::Quaterniond(q.w,q.x,q.y,q.z),
            rclcpp::Time(m->header.stamp).seconds(),m->header.frame_id,rclcpp::Time(m->header.stamp).nanoseconds()};
          double yaw=0.;
          if(measuredBodyYaw(body,self_inflation_frame_id_,node_->now().seconds(),odom_timeout_,yaw,node_->now().nanoseconds()))
            execution_validator_->body(body);
        },control_options);
      execution_permit_sub_=node_->create_subscription<ew::ExecutionPermit>(
          "/d1max/live_planning/execution/permit",rclcpp::QoS(2),
          [this](ew::ExecutionPermit::ConstSharedPtr m){execution_validator_->commit(*m);},control_options);
      execution_handoff_sub_=node_->create_subscription<ew::ExecutionHandoffGrant>(
          "/d1max/live_planning/execution/handoff_grant",rclcpp::QoS(2),
          [this](ew::ExecutionHandoffGrant::ConstSharedPtr m){execution_validator_->handoff(*m);},control_options);
      execution_prepared_sub_=node_->create_subscription<ew::PreparedMotionDemand>(
          "/d1max/live_planning/execution/prepared_demand",rclcpp::QoS(1),
          [this](ew::PreparedMotionDemand::ConstSharedPtr m){execution_validator_->preparedDemand(*m);},control_options);
      execution_commit_ack_sub_=node_->create_subscription<ew::ExecutionCommitAck>(
          "/d1max/live_planning/execution/commit_ack",rclcpp::QoS(8),
          [this](ew::ExecutionCommitAck::ConstSharedPtr m){execution_validator_->commitAck(*m);},control_options);
      execution_progress_sub_=node_->create_subscription<ew::TrackingProgress>(
          "planning/tracking_progress",rclcpp::QoS(1),
          [this](ew::TrackingProgress::ConstSharedPtr m){execution_validator_->progress(*m);},control_options);
      execution_demand_sub_=node_->create_subscription<ew::MotionDemand>(
          "/d1max/live_planning/execution/demand",rclcpp::QoS(1),
          [this](ew::MotionDemand::ConstSharedPtr m){execution_validator_->demand(*m);},control_options);
      execution_blocked_entry_sub_=node_->create_subscription<ew::MotionValidation>(
          "/d1max/live_planning/execution/blocked_entry",rclcpp::QoS(4),
          [this](ew::MotionValidation::ConstSharedPtr m){execution_validator_->submitBlockedEntry(*m);},control_options);
      execution_admission_sub_=node_->create_subscription<ew::TrajectoryAdmission>(
          "/d1max/live_planning/execution/admission",rclcpp::QoS(1),
          [this](ew::TrajectoryAdmission::ConstSharedPtr m){execution_validator_->submitCandidateEntryRejection(*m);},control_options);
      execution_proposal_sub_=node_->create_subscription<ew::ReferenceProposal>(
          "/d1max/live_planning/execution/reference_proposal",rclcpp::QoS(1),
          [this,mode](ew::ReferenceProposal::ConstSharedPtr m){
            ew::ReferenceReceipt ack;ack.version=m->version;ack.proposal_id=m->proposal_id;
            ack.expected_version=m->expected_version;ack.expected_trajectory_id=m->expected_trajectory_id;
            ack.source_stamp=node_->now();ack.valid_until=node_->now()+rclcpp::Duration::from_seconds(.5);
            ack.transport_mode=mode;ack.reason="reference_contract_rejected";
            const double age=(node_->now()-rclcpp::Time(m->source_stamp)).seconds();
            const auto current=execution_validator_->committedVersion();
            const bool expected=current?(*current==m->expected_version && sameExecutionTask(*current,m->version) &&
              execution_validator_->committedTrajectory()==m->expected_trajectory_id):m->expected_version.task_id.empty();
            if(m->transport_mode==mode&&!m->proposal_id.empty()&&age>=-.02&&age<=1.&&
               rclcpp::Time(m->valid_until)>node_->now()&&m->version==executionVersion(m->reference)&&expected&&
               m->reference.point_reference=="body_center"&&m->reference.path.header.frame_id==self_inflation_frame_id_&&
               !m->reference.path.poses.empty()) {
              execution_validator_->cancelPending();
              execution_proposal_=*m;
              typedPathCallback(std::make_shared<ew::ReferencePath>(m->reference));
              ack.accepted=have_target_&&reference_generation_==m->reference.generation;
              ack.reason=ack.accepted?"pending_reference_received_not_motion_authority":"reference_geometry_rejected";
            }
            execution_receipt_pub_->publish(ack);
          });
      // Publish immediately after the writer's fusion transaction. A second
      // independent 200 ms timer can add an entire stale-map cycle by phase.
      // The 20 Hz validator thread still only reads immutable snapshots.
      planner_manager_->grid_map_->setCollisionUpdateCallback([this]{
        if(snapshot_pool_.publishValidation(*planner_manager_->grid_map_,node_->now().nanoseconds(),std::chrono::steady_clock::now()) &&
           execution_validator_) execution_validator_->snapshotPublished();
        if(execution_validator_&&execution_validator_->slowSnapshotWanted()&&
           snapshot_pool_.publishSolver(*planner_manager_->grid_map_,node_->now().nanoseconds(),std::chrono::steady_clock::now()))
          execution_validator_->admissionSnapshotPublished();
      });
    }
    // Zero keeps the upstream smoothness/collision/feasibility objective.
    // Optional reference attraction is a D1 extension, not a mode-3 requirement.

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    tracking_progress_sub_=node_->create_subscription<d1max_planning_interfaces::msg::TrackingProgress>(
        "planning/tracking_progress",rclcpp::QoS(1).best_effort(),
        std::bind(&SCANReplanFSM::trackingProgressCallback,this,std::placeholders::_1));
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
      discrete_progress_ = setReferenceFromBody(discrete_reference_, odom_pos_, waypoints);
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
    if(execution_validator_&&msg->path.poses.empty()) {
      // A task-scoped higher-generation cancellation retires geometry before
      // publishing its ACK. It is not an unsuccessful replacement candidate,
      // and must never be swallowed by the incumbent's visualization path.
      if(msg->schema_version!=2||!execution_validator_->cancelReference(executionVersion(*msg))) {
        RCLCPP_WARN(node_->get_logger(),"Reject cancellation without matching native owner");return;
      }
      execution_proposal_.reset();
    }
    cancelSolveWorker();
    tracking_progress_.reset();
    reference_metadata_=*msg;
    // SingleThreadedExecutor: input replacement and trajectory publication are
    // serialized. Revoke the previous route even when the new route is invalid.
    have_reference_generation_ = true;
    reference_generation_ = msg->generation;
    force_visible_side_target_=blocked_entry_replan_pending_=false;
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
      if (!execution_validator_ && have_odom_ && (node_->now() - last_odom_time_).seconds() <= odom_timeout_)
        callEmergencyStop(odom_pos_);
      RCLCPP_INFO(node_->get_logger(), "Reference generation %lu canceled", reference_generation_);
      return;
    }
    if ((require_schema_v2_ && msg->schema_version!=2) ||
        (msg->schema_version==2 &&
         ((msg->point_reference!="ground" && msg->point_reference!="body_center") ||
          (msg->point_reference=="body_center" && std::abs(reference_path_z_offset_)>1e-9) ||
          msg->task_id.empty() || msg->route_id.empty() || msg->route_hash.empty() ||
          msg->map_version_id.empty() || msg->segment_id.empty() ||
          msg->point_segment_ids.size()!=msg->path.poses.size() ||
          msg->point_segment_kinds.size()!=msg->path.poses.size() ||
          msg->point_required_modes.size()!=msg->path.poses.size()))) {
      publishInvalidLocalPlanDebug("invalid_reference_schema",true);
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
      tagged.schema_version=reference_metadata_.schema_version;
      tagged.point_reference="body_center";
      tagged.task_id=reference_metadata_.task_id;
      tagged.route_id=reference_metadata_.route_id;
      tagged.route_hash=reference_metadata_.route_hash;
      tagged.map_version_id=reference_metadata_.map_version_id;
      tagged.segment_id=reference_metadata_.segment_id;
      tagged.segment_kind=reference_metadata_.segment_kind;
      tagged.required_mode=reference_metadata_.required_mode;
      tagged.anchor_revision=reference_metadata_.anchor_revision;
      tagged.map_geometry_revision=reference_metadata_.map_geometry_revision;
      tagged.anchor_id=reference_metadata_.anchor_id;
      tagged.context_sequence=reference_metadata_.context_sequence;
      tagged.localization_epoch=reference_metadata_.localization_epoch;
      tagged.localization_seed_id=reference_metadata_.localization_seed_id;
      if (const auto &join=planner_manager_->acceptedJoinEvidence()) {
        tagged.valid_start_time=join->curve_time;
        tagged.valid_start_arc_length=join->arc_length;
        tagged.join_source_stamp=rclcpp::Time(
            measuredBodySourceNs(join->measured),
            node_->get_clock()->get_clock_type());
        tagged.join_pose.position.x=join->measured.position.x();
        tagged.join_pose.position.y=join->measured.position.y();
        tagged.join_pose.position.z=join->measured.position.z();
        tagged.join_pose.orientation.x=join->measured.orientation.x();
        tagged.join_pose.orientation.y=join->measured.orientation.y();
        tagged.join_pose.orientation.z=join->measured.orientation.z();
        tagged.join_pose.orientation.w=join->measured.orientation.w();
        tagged.join_twist.linear.x=join->velocity.x();
        tagged.join_twist.linear.y=join->velocity.y();
        tagged.join_twist.linear.z=join->velocity.z();
        tagged.join_acceleration.linear.x=join->acceleration.x();
        tagged.join_acceleration.linear.y=join->acceleration.y();
        tagged.join_acceleration.linear.z=join->acceleration.z();
        tagged.join_acceleration_valid=join->acceleration_valid;
      }
      tagged_bspline_pub_->publish(tagged);
      if(execution_validator_&&execution_proposal_&&execution_proposal_->reference.generation==tagged.generation)
        execution_validator_->candidate(tagged,*execution_proposal_);
    }
  }

  std_msgs::msg::Header SCANReplanFSM::localPlanDebugHeader()
  {
    std_msgs::msg::Header header;
    // A publication stamp is the actual source clock, not an event sequence.
    // Generation and receipt order distinguish same-tick cancellation events.
    header.stamp = node_->now();
    header.frame_id = self_inflation_frame_id_;
    return header;
  }

  void SCANReplanFSM::publishInvalidLocalPlanDebug(const std::string &phase, bool force)
  {
    // An unsuccessful candidate is not an invalidation of the BT-committed
    // trajectory. Only the independent validator's actual proof describes it.
    if(phase!="cancelled"&&execution_validator_&&publishExecutionLedgerDebug())return;
    // A source gap suspends proof, not the immutable accepted preview geometry.
    // Collision, cancellation and every other failure permanently revoke it.
    if (phase!="waiting_sensor_map" && phase!="waiting_recheck") accepted_preview_debug_.reset();
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
      accepted_preview_debug_=*message;
      accepted_curve_context_sequence_=planner_manager_->grid_map_->localizationContextSequence();
      if(execution_validator_)execution_validator_->candidateDebug(*message);
      else local_plan_debug_pub_->publish(*message);
      accepted_curve_generation_ = reference_generation_;
      accepted_curve_id_ = static_cast<std::int64_t>(plan_id);
      have_local_debug_state_ = true;
      last_local_debug_phase_ = "accepted";
      last_local_debug_generation_ = reference_generation_;
      return;
    }
    const auto reason = selected.size() > kMaxLocalDebugReferencePoints ? "debug_overflow" : "debug_invalid";
    accepted_preview_debug_.reset();
    local_plan_debug_pub_->publish(makeInvalidLocalPlanDebug(
        header, navigation_session_id_, reference_generation_, plan_id, reason));
    have_local_debug_state_ = true;
    last_local_debug_phase_ = reason;
    last_local_debug_generation_ = reference_generation_;
  }

  bool SCANReplanFSM::publishExecutionLedgerDebug()
  {
    if(!execution_validator_||!local_plan_debug_pub_)return false;
    const auto view=execution_validator_->committedView();if(!view)return false;
    const auto& spline=view->spline;
    const auto now=node_->now();
    const bool fresh=view->proof&&view->proof->valid&&
      rclcpp::Time(view->proof->valid_until)>now&&
      (now-rclcpp::Time(view->proof->body_source_stamp)).seconds()<=odom_timeout_;
    if(fresh&&view->debug) {
      auto debug=*view->debug;debug.header=localPlanDebugHeader();debug.phase="accepted";debug.valid=true;
      debug.checked_map_source_stamp_ns=std::min(rclcpp::Time(view->proof->front_ray_source_stamp).nanoseconds(),
                                                 rclcpp::Time(view->proof->rear_ray_source_stamp).nanoseconds());
      debug.checked_body_source_stamp_ns=rclcpp::Time(view->proof->body_source_stamp).nanoseconds();
      debug.checked_map_revision=view->proof->map_snapshot_revision;
      debug.checked_context_sequence=view->proof->version.context_sequence;
      local_plan_debug_pub_->publish(debug);
      Eigen::MatrixXd points(3,spline.trajectory.pos_pts.size());Eigen::VectorXd knots(spline.trajectory.knots.size());
      for(std::size_t i=0;i<spline.trajectory.pos_pts.size();++i) {
        const auto& p=spline.trajectory.pos_pts[i];points.col(i)=Eigen::Vector3d(p.x,p.y,p.z);
      }
      for(std::size_t i=0;i<spline.trajectory.knots.size();++i)knots[i]=spline.trajectory.knots[i];
      UniformBspline curve(points,3,.1);curve.setKnot(knots);visualization_->displayOptimalTraj(curve,0);
    } else {
      local_plan_debug_pub_->publish(makeInvalidLocalPlanDebug(localPlanDebugHeader(),spline.session_id,
        spline.generation,static_cast<std::uint64_t>(spline.trajectory.traj_id),
        view->proof&&!view->proof->valid?view->proof->reason:"waiting_current_validation"));
    }
    return true;
  }

  SCANReplanFSM::PreviewRecheck SCANReplanFSM::revalidatePreviewIncumbent()
  {
    // The heading contract can only be enabled in explicit official-policy,
    // per-sensor-ray, no-motion preview. Default/execution paths never use this.
    if (!reference_path_guidance_ || !require_tagged_reference_ || !have_target_ ||
        !planner_manager_->hasPreviewHeadingContract() || !accepted_preview_debug_ ||
        accepted_curve_generation_!=reference_generation_ || accepted_curve_id_<=0 ||
        accepted_curve_id_!=planner_manager_->local_data_.traj_id_ ||
        accepted_preview_debug_->plan_id!=static_cast<std::uint64_t>(accepted_curve_id_))
      return PreviewRecheck::Unavailable;
    const auto map=planner_manager_->grid_map_;
    if (accepted_curve_context_sequence_==0 ||
        accepted_curve_context_sequence_!=map->localizationContextSequence()) {
      accepted_preview_debug_.reset();
      return PreviewRecheck::Unavailable;
    }
    const auto sources_fresh=[&]() {
      const auto now=node_->now();
      const double age=(now-last_odom_time_).seconds();
      return have_odom_ && age>=0. && age<=odom_timeout_ &&
          map->integratedCloudFreshAt(now.nanoseconds());
    };
    if (!sources_fresh()) return PreviewRecheck::Stale;
    const auto source_stamp=map->latestCloudStampNs();
    const auto body_stamp=last_odom_time_.nanoseconds();
    const auto context=map->localizationContextSequence();
    const auto revision=map->occupancyRevision();
    const auto checked=planner_manager_->recheckPreviewTrajectory();
    // The bounded whole-curve check can consume time; do not turn aged input
    // into fresh evidence. These identity checks also protect future executors.
    if (!sources_fresh() || source_stamp!=map->latestCloudStampNs() ||
        body_stamp!=last_odom_time_.nanoseconds() ||
        context!=map->localizationContextSequence() || revision!=map->occupancyRevision())
      return PreviewRecheck::Stale;
    if (checked==CurveCheckEvidence::Occupied) return PreviewRecheck::Unsafe;
    if (checked!=CurveCheckEvidence::Clear) return PreviewRecheck::Uncertified;
    const auto proof=makeRevalidatedLocalPlanDebug(*accepted_preview_debug_,
        localPlanDebugHeader(),source_stamp,body_stamp,revision,context);
    if (!proof || !local_plan_debug_pub_) return PreviewRecheck::Stale;
    local_plan_debug_pub_->publish(*proof);
    // Do not save the newly dated proof as the original acceptance, nor modify
    // local_data_/traj_id/start_time/controls or grant execution admission.
    have_local_debug_state_=true;
    last_local_debug_phase_="revalidated";
    last_local_debug_generation_=reference_generation_;
    return PreviewRecheck::Safe;
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
      // Reference and odometry travel on different topics. The bridge may have
      // fresh odometry while this callback still sees the previous sample.
      // This generation was already consumed by typedPathCallback: report its
      // rejection explicitly so the owner can revalidate a NEW generation.
      // Never queue this geometry for automatic execution after odometry returns.
      publishInvalidLocalPlanDebug("reference_rejected_odometry", true);
      return;
    }
    if (strict_input_frames_ && msg->header.frame_id != self_inflation_frame_id_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference path frame does not match grid_map.frame_id");
      publishInvalidLocalPlanDebug("reference_rejected_frame", true);
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
        publishInvalidLocalPlanDebug("reference_rejected_frame", true);
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
      publishInvalidLocalPlanDebug("reference_rejected_geometry", true);
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
      publishInvalidLocalPlanDebug("reference_rejected_geometry", true);
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    const rclcpp::Time source_time(msg->header.stamp);
    if ((strict_input_frames_ && msg->header.frame_id != self_inflation_frame_id_) ||
        !measuredPoseSourceAccepted(source_time.nanoseconds(),node_->now().nanoseconds(),
                                  odom_timeout_,have_odom_ ? last_odom_time_.nanoseconds():0))
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Reject body odometry: world-frame mismatch or missing/stale/future/old timestamp");
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
    // Keep the acquisition time: a delayed message must not receive a second
    // freshness lease merely because the callback just ran.
    last_odom_time_ = source_time;
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
    planner_manager_->setMeasuredBodyPose(
        MeasuredBodyPose{position,orientation,source_time.seconds(),msg->header.frame_id,source_time.nanoseconds()},
        self_inflation_frame_id_,odom_timeout_);
    planner_manager_->setMeasuredBodyVelocity(velocity);
    if(execution_validator_) execution_validator_->body(
      MeasuredBodyPose{position,orientation,source_time.seconds(),msg->header.frame_id,source_time.nanoseconds()});

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
    // Execution geometry uses measured progress and explicit current proofs;
    // never rewrite its original publication time while waiting for authority.
    if(execution_validator_)return;
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
    if(execution_validator_) {
      const auto blocked=execution_validator_->consumeBlockedEntry();
      if(blocked&&have_target_&&blocked->version==executionVersion(reference_metadata_)) {
        // Strictly matched native command proof, not an arbitrary "blocked"
        // status. Retire only the pending candidate, never the active curve or
        // global task. The shared solve budget/rate limit still applies.
        cancelSolveWorker();execution_validator_->cancelPending();
        force_visible_side_target_=blocked_entry_replan_pending_=true;
        blocked_entry_position_=odom_pos_;
        RCLCPP_WARN(node_->get_logger(),"Confirmed command entry blocked; requesting bounded local side corridor for curve %ld",blocked->trajectory_id);
      }
    }
    if (pollSolveWorker()) return;
    if(pending_snapshot_request_.pending()&&!snapshotAcquisitionCurrent()) {
      // Cancellation, original deadline and REAL source validity end this
      // acquisition. No new deadline is allocated by a timer/slot release.
      waitForChangedEnvironment();return;
    }
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
      cancelSolveWorker();
      publishInvalidLocalPlanDebug("completed");
      publishAttemptDebug(localPlanDebugHeader(), true);
      changeFSMExecState(WAIT_TARGET, "MEASURED_GOAL_REACHED");
      return;
    }
    if (max_replan_interval_ > 0.0 || reference_path_guidance_) {
      if (!planner_manager_->grid_map_->integratedCloudFreshAt(node_->now().nanoseconds())) {
        publishInvalidLocalPlanDebug("waiting_sensor_map");
        return;
      }
    }
    if(execution_validator_&&reference_path_guidance_&&have_target_&&have_odom_&&
       (exec_state_==EXEC_TRAJ||exec_state_==WAIT_ENVIRONMENT||exec_state_==REPLAN_TRAJ||exec_state_==GEN_NEW_TRAJ)&&
       formalReseedSubmissionDue(std::chrono::steady_clock::now(),last_solve_submit_,solve_future_.valid())) {
      const auto reseed=execution_validator_->consumeFormalReseedAt(node_->now().nanoseconds());
      if(reseed&&sameExecutionTask(reseed->version,executionVersion(reference_metadata_))&&
         (reseed->terminal||reseed->version==executionVersion(reference_metadata_))) {
        // A stale join boundary is not an obstacle and does not justify a side
        // corridor or changing the task. Keep writer-applied geometry and its
        // real safety reader alive; only reseed the next bounded local solve.
        last_attempt_failure_phase_=reseed->reason;
        changeFSMExecState(GEN_NEW_TRAJ,reseed->terminal?"MEASURED_LOCAL_ENDPOINT":"MEASURED_ENTRY_RESEED");
        RCLCPP_INFO(node_->get_logger(),"Formal local reseed curve=%ld reason=%s original_body_source_ns=%ld",
          reseed->trajectory_id,reseed->reason.c_str(),rclcpp::Time(reseed->body_source_stamp).nanoseconds());
      }
    }
    if(force_visible_side_target_&&!blocked_entry_replan_pending_&&
       (odom_pos_-blocked_entry_position_).norm()>=.3)force_visible_side_target_=false;
    if(blocked_entry_replan_pending_) {
      if(!periodicReplanDue(node_->now().seconds(),last_replan_time_.seconds(),
          std::max(.5,failed_replan_cooldown_)))return;
      blocked_entry_replan_pending_=false;
      last_attempt_failure_phase_="blocked_executable_entry";
      changeFSMExecState(GEN_NEW_TRAJ,"COMMAND_ENTRY_BLOCKED");
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
        if(pending_snapshot_request_.pending())break;
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
        if(pending_snapshot_request_.pending())break;
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
      if (!failedAttemptRetryReady(mono)) return;
      changeFSMExecState(GEN_NEW_TRAJ,
          nativeResourceWait(last_attempt_failure_phase_) ? "RESOURCE_RETRY" : "ENVIRONMENT_CHANGED");
      break;
    }

    case EXEC_TRAJ:
    {
      // A frozen body still needs a live, validated optimizer heartbeat. This
      // produces a new real trajectory rather than refreshing stale messages.
      if (periodicReplanDue(node_->now().seconds(), last_replan_time_.seconds(), max_replan_interval_))
      {
        if (keepPreviewIncumbentForPeriodicReplan()) {
          // Only the scheduling clock changes. Curve ID, controls, source
          // timestamps and physical progress remain unchanged.
          last_replan_time_=node_->now();
          break;
        }
        changeFSMExecState(REPLAN_TRAJ, "PERIODIC");
        return;
      }
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      if (reference_path_guidance_) {
        if (go2_execution_frozen_ && planner_manager_->hasPreviewHeadingContract())
          t_cur=planner_manager_->previewCurveProgressTime();
        else {
          if (!tracking_progress_ || !tracking_progress_->valid ||
              tracking_progress_->trajectory_id!=info->traj_id_ ||
              (node_->now()-rclcpp::Time(tracking_progress_->header.stamp)).seconds()>odom_timeout_)
            return;
          t_cur=tracking_progress_->curve_time;
        }
      }
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
    // A submitted request is not an optimizer failure. Its one result is
    // consumed by pollSolveWorker; callbacks keep validating the incumbent.
    if (solve_future_.valid() || pending_snapshot_request_.pending() ||
        last_attempt_failure_phase_=="waiting_solve_budget") return;
    failed_replan_monotonic_=std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    failed_environment_revision_=planner_manager_->grid_map_->occupancyRevision();
    failed_map_stamp_=planner_manager_->grid_map_->latestCloudStamp();
    failed_ray_stamps_={planner_manager_->grid_map_->integratedRaySourceStamp(0),
                        planner_manager_->grid_map_->integratedRaySourceStamp(1)};
    failed_body_position_=odom_pos_;
    failed_body_velocity_=odom_vel_;
    failed_body_orientation_=odom_orient_;
    failed_body_source_ns_=last_odom_time_.nanoseconds();
    const auto &reason=last_attempt_failure_phase_;
    // A rejected replacement is not proof that the already accepted curve is
    // unsafe. Keep only an incumbent freshly checked against real body/map.
    const auto incumbent=revalidatePreviewIncumbent();
    if (incumbent==PreviewRecheck::Stale)
      publishInvalidLocalPlanDebug("waiting_sensor_map", true);
    else if (incumbent==PreviewRecheck::Uncertified)
      publishInvalidLocalPlanDebug("waiting_recheck", true);
    else if (incumbent==PreviewRecheck::Unsafe)
      publishInvalidLocalPlanDebug("failed_current_validation", true);
    else if (incumbent!=PreviewRecheck::Safe)
      // Each call follows an actual failed solve. Repeated same-phase failures
      // must remain diagnosable, not disappear after a diagnostic lease expires.
      publishInvalidLocalPlanDebug(reason.empty() ? "failed_optimization" : reason, true);
    RCLCPP_WARN(node_->get_logger(), "Local navigation waiting for %s: %s",
                nativeResourceWait(reason) ? "bounded snapshot retry" : "changed environment", reason.c_str());
    changeFSMExecState(WAIT_ENVIRONMENT, "BOUNDED_FAILURE");
  }

  bool SCANReplanFSM::failedAttemptRetryReady(double monotonic_now)
  {
    // An exclusive reader releasing the third slot need not change a voxel or
    // body pose. Re-attempt resources at most 2 Hz, then acquire through the
    // same arbiter; contention never bypasses ownership. Every new attempt gets
    // one ORIGINAL 400 ms deadline in callReboundReplan, not a continuation or
    // extension of the failed request. Existing actual-solve cooldown remains.
    const bool resource=nativeResourceWait(last_attempt_failure_phase_);
    const double cooldown=resource ? std::max(.5,failed_replan_cooldown_) : failed_replan_cooldown_;
    if (!std::isfinite(monotonic_now) || monotonic_now-failed_replan_monotonic_<cooldown) return false;
    return resource || environmentChangedSinceFailure();
  }

  bool SCANReplanFSM::environmentChangedSinceFailure()
  {
    const auto map=planner_manager_->grid_map_;
    const auto& reason=last_attempt_failure_phase_;
    // A geometrically unchanged saturated map can acquire fresh FREE evidence.
    // Recheck/unknown failures must not wait forever for a changed occupancy
    // bit. Both real sensor watermarks must advance; receipt/timer/republication
    // alone cannot recover the attempt, and WAIT_ENVIRONMENT still enforces the
    // existing 500 ms solve cooldown plus the one 400 ms shared solve budget.
    const bool evidence_dependent=reason=="waiting_sensor_map"||reason=="waiting_recheck"||
        reason=="waiting_observed_space"||reason=="failed_final_collision"||
        reason=="failed_current_validation"||reason=="blocked_executable_entry"||
        reason=="waiting_observed_side_target"||reason=="visible_side_target_budget_exhausted";
    const bool new_supported_map=evidence_dependent &&
        map->latestCloudStamp()>failed_map_stamp_ &&
        map->integratedRaySourceStamp(0)>failed_ray_stamps_[0] &&
        map->integratedRaySourceStamp(1)>failed_ray_stamps_[1] &&
        map->integratedCloudFreshAt(node_->now().nanoseconds());
    const double body_age=(node_->now()-last_odom_time_).seconds();
    const bool new_body=have_odom_ && last_odom_time_.nanoseconds()>failed_body_source_ns_ &&
        body_age>=0. && body_age<=odom_timeout_;
    // A solve can reject an old body sample without any geometric change.
    // A genuinely new valid sample resolves that precondition; unchanged or
    // stale source timestamps do not manufacture a recovery event.
    const bool body_recovered=new_body && last_attempt_failure_phase_=="waiting_body_pose";
    const bool dynamics_recovered=new_body && last_attempt_failure_phase_=="failed_dynamics" &&
        failedDynamicsBoundaryChanged(failed_body_velocity_, odom_vel_, planner_manager_->pp_.max_vel_);
    const bool attitude_changed=new_body &&
        failedOrientationBoundaryChanged(failed_body_orientation_, odom_orient_);
    return new_supported_map || body_recovered || dynamics_recovered || attitude_changed ||
        map->occupancyRevision()!=failed_environment_revision_ ||
        (odom_pos_-failed_body_position_).norm()>=failed_replan_body_distance_;
  }

  bool SCANReplanFSM::keepPreviewIncumbentForPeriodicReplan()
  {
    if(execution_validator_&&go2_execution_frozen_) {
      const auto view=execution_validator_->committedView();
      return view&&view->spline.generation==reference_generation_&&view->proof&&view->proof->valid&&
        rclcpp::Time(view->proof->valid_until)>node_->now()&&view->progress&&view->progress->valid&&
        // Writer-applied geometry may remain safe while the local horizon is
        // finished, or a newer solver curve cannot meet the actual entry
        // boundary. That is not an idle preview: freezing its solve clock
        // would retain the unjoinable candidate forever. Preserve the active
        // proof/identity, but reseed the next bounded solve from real odometry.
        !view->progress->holding&&
        view->spline.trajectory.traj_id==planner_manager_->local_data_.traj_id_&&
        (node_->now()-rclcpp::Time(view->progress->header.stamp)).seconds()<=odom_timeout_&&
        view->progress->twist.linear.x*view->progress->twist.linear.x+
        view->progress->twist.linear.y*view->progress->twist.linear.y+
        view->progress->twist.linear.z*view->progress->twist.linear.z<.0001;
    }
    if (!go2_execution_frozen_ || !reference_path_guidance_ ||
        !planner_manager_->hasPreviewHeadingContract()) return false;
    const auto incumbent=revalidatePreviewIncumbent();
    if (incumbent!=PreviewRecheck::Safe) return false;
    const auto &curve=planner_manager_->local_data_;
    const double advance=std::min(1.,planning_horizon_*.25);
    // Progress is projected from measured XYZ, not wall-clock spline time.
    // The fraction bound also refreshes a short final leg or a curved segment
    // which approaches its end while its chord is still close to the start.
    return std::isfinite(advance) && advance>0. && curve.duration_>0. &&
        (odom_pos_-curve.start_pos_).norm()<advance &&
        planner_manager_->previewCurveProgressTime()<curve.duration_*.25;
  }

  bool SCANReplanFSM::shouldReplanUncertifiedPreview()
  {
    if (!go2_execution_frozen_ || !reference_path_guidance_ || !have_target_ ||
        !planner_manager_->hasPreviewHeadingContract() ||
        (exec_state_!=EXEC_TRAJ && exec_state_!=WAIT_ENVIRONMENT) ||
        !periodicReplanDue(node_->now().seconds(),last_replan_time_.seconds(),
                          std::max(.1,failed_replan_cooldown_))) return false;
    const auto &previous=exec_state_==WAIT_ENVIRONMENT ? failed_body_position_ :
        planner_manager_->local_data_.start_pos_;
    return (odom_pos_-previous).norm()>=planner_manager_->grid_map_->getResolution()*.25;
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    // Reference guidance selects WHERE to go, not WHETHER the body is following
    // a trajectory. Keep the upstream measured-position + incumbent-derivative
    // hand-off while executing; an explicitly frozen preview uses measured
    // motion because its spline clock is not the robot's physical progress.
    setStartStateFromOdomOrCurrentTraj();

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
    if (go2_execution_frozen_ || have_new_target_) return;

    if(execution_validator_) {
      if(const auto boundary=execution_validator_->committedBoundary(odom_pos_,node_->now().seconds(),node_->now().nanoseconds())) {
        start_vel_=boundary->velocity;start_acc_=boundary->acceleration;
      }
      return;
    }

    LocalTrajData *info = &planner_manager_->local_data_;
    if (require_tagged_reference_ &&
        (accepted_curve_id_ <= 0 || accepted_curve_generation_ != reference_generation_ ||
         info->traj_id_ != accepted_curve_id_))
      return;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    // Use a controller's measured curve progress, never time since publication.
    // Without a matching fresh proof keep measured velocity / zero acceleration;
    // no virtual prediction is licensed by a running spline clock.
    if (!tracking_progress_ || !tracking_progress_->valid || tracking_progress_->holding ||
        tracking_progress_->trajectory_id!=info->traj_id_ ||
        tracking_progress_->generation!=reference_generation_ ||
        (node_->now()-rclcpp::Time(tracking_progress_->header.stamp)).seconds()>odom_timeout_)
      return;
    const double t_cur=tracking_progress_->curve_time;
    if (!std::isfinite(t_cur) || t_cur<0. || t_cur>info->duration_ ||
        (info->position_traj_.evaluateDeBoorT(t_cur)-odom_pos_).norm()>
            planner_manager_->grid_map_->getResolution()*.25) return;
    const auto velocity = info->velocity_traj_.evaluateDeBoorT(t_cur);
    const auto acceleration = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    if (!velocity.allFinite() || !acceleration.allFinite()) return;
    start_vel_ = velocity;
    start_acc_ = acceleration;

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    // A routed corridor may initially lead away from the final goal. Do not
    // introduce a zero-velocity discontinuity merely because of that geometry.
    if (!reference_path_guidance_ && shouldSuppressOpposedPrediction(false, to_goal, start_vel_))
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::trackingProgressCallback(
      const d1max_planning_interfaces::msg::TrackingProgress::ConstSharedPtr &msg)
  {
    if(execution_validator_)execution_validator_->progress(*msg);
    const auto &identity=reference_metadata_;
    const double age=(node_->now()-rclcpp::Time(msg->header.stamp)).seconds();
    if (!msg->valid && msg->session_id==navigation_session_id_ &&
        msg->generation==reference_generation_ && msg->trajectory_id==accepted_curve_id_) {
      tracking_progress_.reset();
      return;
    }
    if (!have_odom_ || !msg->valid || msg->session_id!=navigation_session_id_ ||
        msg->generation!=reference_generation_ || msg->trajectory_id!=accepted_curve_id_ ||
        msg->header.frame_id!=self_inflation_frame_id_ || age<0. || age>odom_timeout_ ||
        !std::isfinite(msg->curve_time) || msg->curve_time<0. ||
        !std::isfinite(msg->arc_length) || msg->arc_length<0. ||
        !std::isfinite(msg->s_committed) || msg->s_committed<msg->arc_length ||
        msg->route_id!=identity.route_id || msg->segment_id!=identity.segment_id ||
        msg->map_version_id!=identity.map_version_id || msg->anchor_id!=identity.anchor_id ||
        msg->context_sequence!=identity.context_sequence ||
        msg->localization_epoch!=identity.localization_epoch ||
        msg->localization_seed_id!=identity.localization_seed_id) return;
    if (identity.schema_version==2 &&
        (msg->schema_version!=2 || msg->task_id!=identity.task_id ||
         msg->route_hash!=identity.route_hash || msg->anchor_revision!=identity.anchor_revision ||
         msg->anchor_id.empty() || !msg->context_sequence)) return;
    const Eigen::Vector3d measured(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    const auto &curve=planner_manager_->local_data_;
    if (!measured.allFinite() || msg->curve_time>curve.duration_ ||
        (measured-odom_pos_).norm()>planner_manager_->grid_map_->getResolution()*.25) return;
    if (tracking_progress_ && tracking_progress_->trajectory_id==msg->trajectory_id) {
      if (rclcpp::Time(msg->header.stamp)<=rclcpp::Time(tracking_progress_->header.stamp) ||
          msg->s_committed<tracking_progress_->s_committed ||
          msg->arc_length<tracking_progress_->s_committed-.15) return;
      // A new source sample cannot jump to an overlapping floor/branch. The
      // downstream tracker also bounds its XYZ projection; this is independent.
      const double dt=(rclcpp::Time(msg->header.stamp)-
                       rclcpp::Time(tracking_progress_->header.stamp)).seconds();
      if (msg->arc_length-tracking_progress_->arc_length>
          planner_manager_->pp_.max_vel_*dt+.10) return;
    }
    tracking_progress_=*msg;
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    if(execution_validator_) {
      // The independent validator checks the committed curve on immutable
      // snapshots. Never compare its progress with the latest solver candidate.
      publishExecutionLedgerDebug();return;
    }
    updateLocalTrajTimeFreeze();
    if (reference_path_guidance_ && planner_manager_->hasPreviewHeadingContract() &&
        have_target_ && exec_state_!=WAIT_TARGET) {
      // Continue checking an incumbent even when a candidate solve is waiting
      // for changed evidence. No optimization is needed to prove it still free.
      const auto incumbent=revalidatePreviewIncumbent();
      if (incumbent==PreviewRecheck::Stale) {
        publishInvalidLocalPlanDebug("waiting_sensor_map");
        return;
      }
      if (incumbent==PreviewRecheck::Uncertified) {
        publishInvalidLocalPlanDebug("waiting_recheck");
        if (shouldReplanUncertifiedPreview())
          changeFSMExecState(REPLAN_TRAJ,"MEASURED_PREVIEW_JOIN_CHANGED");
        return;
      }
      if (incumbent==PreviewRecheck::Safe) return;
      if (incumbent==PreviewRecheck::Unsafe) {
        publishInvalidLocalPlanDebug("failed_current_validation");
        last_attempt_failure_phase_="failed_current_validation";
        if (exec_state_!=WAIT_ENVIRONMENT) {
          if (planFromCurrentTraj()) changeFSMExecState(EXEC_TRAJ,"PREVIEW_SAFETY");
          else waitForChangedEnvironment();
        }
        return;
      }
      // A revoked or never-accepted preview cannot resurrect from a collision
      // check; only a genuinely accepted new spline may establish an incumbent.
      return;
    }
    if (!have_odom_ || (node_->now() - last_odom_time_).seconds() > odom_timeout_)
      return;
    if (max_replan_interval_ > 0.0 || reference_path_guidance_) {
      if (!planner_manager_->grid_map_->integratedCloudFreshAt(node_->now().nanoseconds())) return;
    }

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || exec_state_ == WAIT_ENVIRONMENT || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    constexpr int maximum_samples=20000;
    double raw_t_cur=(node_->now()-info->start_time_).seconds();
    if (reference_path_guidance_) {
      if (!tracking_progress_ || !tracking_progress_->valid ||
          tracking_progress_->trajectory_id!=info->traj_id_ ||
          (node_->now()-rclcpp::Time(tracking_progress_->header.stamp)).seconds()>odom_timeout_) {
        publishInvalidLocalPlanDebug("waiting_measured_progress");
        return;
      }
      raw_t_cur=tracking_progress_->curve_time;
    }
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
    if (solve_future_.valid()) return false;
    const auto submitted=std::chrono::steady_clock::now();
    if(pending_snapshot_request_.pending()&&!snapshotAcquisitionCurrent())return false;
    if ((solve_worker_||execution_validator_)&&
        !formalReseedSubmissionDue(submitted,last_solve_submit_,false)) {
      last_attempt_failure_phase_="waiting_solve_budget";
      return false;
    }
    // One absolute budget includes target selection, snapshot, worker solve
    // and the owner-thread latest-map final check; no stage renews it.
    const auto request_budget=pending_snapshot_request_.begin(reference_generation_);
    planner_manager_->discardPreviewCandidate();
    predecessor_id_ = 0;
    predecessor_safe_ = false;
    predecessor_check_stamp_ = builtin_interfaces::msg::Time{};
    last_replan_time_ = node_->now();
    const auto attempt_header=localPlanDebugHeader();
    planner_manager_->grid_map_->resetCollisionDiagnostics();
    local_target_query_debug_=ReferenceTargetResult{};

    const auto target_started=std::chrono::steady_clock::now();
    GridMap::Ptr target_snapshot;
    const bool target_available=getLocalTarget(request_budget,&target_snapshot);
    const auto target_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-target_started).count();
    RCLCPP_INFO(node_->get_logger(),"NativeTarget generation=%lu target_selection_ms=%.3f success=%d reason=%s",
      reference_generation_,target_ms,target_available?1:0,local_target_query_debug_.reason.c_str());
    if (!target_available) {
      if (!request_budget->allowed()) last_attempt_failure_phase_=request_budget->reason();
      else if (nativeResourceWait(local_target_query_debug_.reason))
        last_attempt_failure_phase_=local_target_query_debug_.reason;
      if (request_budget->allowed() && !nativeResourceWait(last_attempt_failure_phase_) &&
          planner_manager_->grid_map_->requiresObservedFree() &&
          planner_manager_->grid_map_->unknownCollisionQueries()>0)
        last_attempt_failure_phase_="waiting_observed_space";
      const auto &query=local_target_query_debug_;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Reference target unavailable: %s target_query_classification=%s queries=%zu free=%zu unknown=%zu occupied=%zu outside=%zu "
          "first_blocked_present=%d first_blocked=[%.3f,%.3f,%.3f] value=%d shown_points=%zu",
          last_attempt_failure_phase_.c_str(),referenceTargetEvidenceClass(query.occupied_queries,
            query.unknown_queries,query.outside_queries), query.queries, query.free_queries,
          query.unknown_queries, query.occupied_queries, query.outside_queries,
          query.has_first_blocked ? 1:0, query.first_blocked.x(), query.first_blocked.y(),
          query.first_blocked.z(), query.first_blocked_value, query.blocked_points.size());
      // Query THIS failed reference pose and its exact tested heading. The
      // selector's 'unknown queries' count is not a count of physical voxels.
      // Diagnostic inspection retains the production collision policy and is
      // bounded/rate-limited independently of selector search work.
      const auto diagnostic_now=std::chrono::steady_clock::now();
      if(query.has_first_blocked&&(last_target_evidence_diagnostic_==std::chrono::steady_clock::time_point{}||
          diagnostic_now-last_target_evidence_diagnostic_>=std::chrono::seconds(1))) {
        last_target_evidence_diagnostic_=diagnostic_now;
        const auto detail=planner_manager_->grid_map_->describeInflateOccupancy(query.first_blocked,query.first_blocked_yaw);
        RCLCPP_WARN(node_->get_logger(),"Reference endpoint evidence generation=%lu endpoint_queries=%zu yaw=%.4f %s",
          reference_generation_,query.queries,query.first_blocked_yaw,detail.c_str());
      }
      // No optimizer ran: never re-stamp an earlier attempt's blocked/seed data.
      if(!nativeResourceWait(last_attempt_failure_phase_)) {
        cancelSnapshotAcquisition();have_new_target_=false;
      }
      publishAttemptDebug(attempt_header, false, false);
      return false;
    }

    // At most one old curve copy, before a successful solve overwrites
    // local_data_. Never associate a curve from another goal generation.
    const auto preceding_generation = reference_generation_;
    const auto preceding_id = accepted_curve_id_;
    std::optional<UniformBspline> predecessor;
    const double predecessor_measured_time=(tracking_progress_ && tracking_progress_->valid &&
        tracking_progress_->trajectory_id==preceding_id) ? tracking_progress_->curve_time :
        planner_manager_->previewCurveProgressTime();
    if (reference_path_guidance_ && require_tagged_reference_ && preceding_id > 0 &&
        accepted_curve_generation_ == preceding_generation &&
        planner_manager_->local_data_.traj_id_ == preceding_id)
      predecessor.emplace(planner_manager_->local_data_.position_traj_);

    if (solve_worker_) {
      const auto copy_start=std::chrono::steady_clock::now();
      // A side-target search and the subsequent solve are ONE solver turn.
      // Transfer its exclusive third-slot lease; do not release/reacquire and
      // consume a second quota before admission has had its promised turn.
      auto snapshot=std::move(target_snapshot);
      if(!snapshot) {
        snapshot=snapshot_pool_.acquireSolver(*planner_manager_->grid_map_,
            node_->now().nanoseconds(),request_budget->deadline());
      }
      snapshot_copy_ms_=std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-copy_start).count();
      if (!snapshot || !request_budget->allowed()) {
        last_attempt_failure_phase_=!request_budget->allowed()?request_budget->reason():"waiting_snapshot_slot";
        if(!request_budget->allowed())cancelSnapshotAcquisition();
        return false;
      }
      // A slot miss did not start a solve. Do not add a 500 ms cooldown to the
      // pending reader, otherwise a fair admission grant could strand solver
      // intent until its lease expires. Actual solve starts remain <= 2 Hz.
      last_solve_submit_=std::chrono::steady_clock::now();
      // A moving entry may never skip the declared segment boundary. The
      // odom transport sends one semantic segment at a time; mixed legacy map
      // previews remain visible but cannot use nonzero v2 entry parameters.
      const auto &meta=reference_metadata_;
      const bool single_segment=meta.schema_version!=2 ||
          (!meta.point_segment_ids.empty() && std::all_of(
              meta.point_segment_ids.begin(),meta.point_segment_ids.end(),
              [&](const std::string &id){return id==meta.segment_id;}));
      planner_manager_->setMeasuredJoinSingleSegment(single_segment);
      solve_worker_->prepareSolveWorker(*planner_manager_,snapshot);
      active_solve_budget_=request_budget;
      pending_snapshot_request_.release();
      solve_generation_=reference_generation_;
      solve_attempt_header_=attempt_header;
      solve_predecessor_=std::move(predecessor);
      solve_predecessor_id_=preceding_id;
      solve_predecessor_measured_time_=predecessor_measured_time;
      const auto start=start_pt_, velocity=start_vel_, acceleration=start_acc_;
      const auto target=local_target_pt_, target_velocity=local_target_vel_;
      const bool poly=have_new_target_ || flag_use_poly_init;
      have_new_target_=false;
      try {
        solve_future_=std::async(std::launch::async,
          [this,request_budget,submitted,start,velocity,acceleration,target,target_velocity,poly,flag_randomPolyTraj]() {
        SolveScopeExit release_snapshot([this]() noexcept {solve_worker_->releaseSolveSnapshot();});
        WorkerResult result;
        const auto begun=std::chrono::steady_clock::now();
        result.queue_ms=std::chrono::duration<double,std::milli>(begun-submitted).count();
        try {
          if (solve_worker_->reboundReplan(start,velocity,acceleration,target,target_velocity,
                                          poly,flag_randomPolyTraj,request_budget))
            result.candidate=solve_worker_->takeSolvedCandidate();
          result.failure=solve_worker_->lastFailurePhase();
        } catch (const std::bad_alloc &) {
          throw; // Resource exhaustion is not a recoverable/safe planning result.
        } catch (const std::exception &error) {
          RCLCPP_WARN(node_->get_logger(),"NativeSolve exception stage=worker what=%.256s",error.what());
          result.candidate.reset();
          result.failure="solve_exception";
        } catch (...) {
          result.candidate.reset();
          result.failure="solve_exception";
        }
        result.solve_ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-begun).count();
        return result;
      });
      } catch (const std::bad_alloc &) {
        request_budget->cancel();
        solve_worker_->releaseSolveSnapshot();
        active_solve_budget_.reset();solve_predecessor_.reset();
        throw;
      } catch (...) {
        solve_worker_->releaseSolveSnapshot();
        active_solve_budget_.reset();solve_predecessor_.reset();
        last_attempt_failure_phase_="solve_launch_failed";
        return false;
      }
      return false; // pending, not failed; incumbent safety checks continue
    }

    pending_snapshot_request_.release();
    snapshot_pool_.cancelSolverRequest();
    if(execution_validator_)last_solve_submit_=submitted;
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj,request_budget);
    have_new_target_ = false;
    last_attempt_failure_phase_=planner_manager_->lastFailurePhase();
    // A long solve must not turn expired input into a fresh accepted output.
    if (reference_path_guidance_ && plan_success) {
      if (!have_odom_ || (node_->now()-last_odom_time_).seconds()>odom_timeout_ ||
          !planner_manager_->grid_map_->integratedCloudFreshAt(node_->now().nanoseconds())) {
        plan_success=false;
        last_attempt_failure_phase_="waiting_sensor_map";
      }
    }
    if (plan_success && !planner_manager_->commitPreviewCandidate()) {
      plan_success=false;
      last_attempt_failure_phase_="waiting_sensor_map";
    }
    if (!plan_success) planner_manager_->discardPreviewCandidate();
    publishAttemptDebug(attempt_header);

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      if (predecessor && reference_generation_ == preceding_generation &&
          planner_manager_->local_data_.traj_id_ > preceding_id) {
        predecessor_id_ = static_cast<std::uint64_t>(preceding_id);
        predecessor_safe_ = planner_manager_->recheckPredecessor(*predecessor,predecessor_measured_time);
        if (predecessor_safe_) predecessor_check_stamp_ = node_->now();
      }

      publishCommittedTrajectory();
    }

    return plan_success;
  }

  void SCANReplanFSM::cancelSnapshotAcquisition()
  {
    pending_snapshot_request_.cancel();snapshot_pool_.cancelSolverRequest();
  }

  bool SCANReplanFSM::snapshotAcquisitionCurrent()
  {
    if(!pending_snapshot_request_.pending())return false;
    const auto budget=pending_snapshot_request_.budget();
    const double body_age=(node_->now()-last_odom_time_).seconds();
    const bool source_fresh=have_target_&&have_odom_&&body_age>=-.02&&body_age<=odom_timeout_&&
        planner_manager_->grid_map_->integratedCloudFreshAt(node_->now().nanoseconds());
    if(pending_snapshot_request_.current(reference_generation_,source_fresh))return true;
    last_attempt_failure_phase_=!budget->allowed()?budget->reason():
        (!source_fresh?"waiting_sensor_map":"solve_cancelled");
    cancelSnapshotAcquisition();return false;
  }

  void SCANReplanFSM::cancelSolveWorker()
  {
    cancelSnapshotAcquisition();
    if (active_solve_budget_) active_solve_budget_->cancel();
  }

  bool SCANReplanFSM::pollSolveWorker()
  {
    if (!solve_future_.valid()) return false;
    if (solve_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return true;
    const auto budget=std::move(active_solve_budget_);
    // Retire the pending token even when get() rethrows a worker/future error.
    // This consumes the future once; no retry renews its deadline or identity.
    WorkerResult result;
    try {
      result=solve_future_.get(); // RAII already released the worker snapshot.
    } catch (const std::bad_alloc &) {
      if (budget) budget->cancel();
      solve_predecessor_.reset();
      throw; // Fail-stop; never continue with a purported safe incumbent after OOM.
    } catch (const std::exception &error) {
      RCLCPP_WARN(node_->get_logger(),"NativeSolve exception stage=future what=%.256s",error.what());
      result.candidate.reset();
      result.failure="solve_exception";
    } catch (...) {
      result.candidate.reset();
      result.failure="solve_exception";
    }
    RCLCPP_INFO(node_->get_logger(),
        "NativeSolve generation=%lu queue_ms=%.3f snapshot_ms=%.3f solve_ms=%.3f snapshot_bytes=%zu deadline=%s",
        solve_generation_,result.queue_ms,snapshot_copy_ms_,result.solve_ms,
        planner_manager_->grid_map_->collisionSnapshotBytes(),budget ? budget->reason():"missing");
    // Input replacement already revoked the old generation. No old diagnostics,
    // candidate or previous result may overwrite the replacement's state.
    if (solve_generation_!=reference_generation_ || !have_target_ ||
        !budget || budget->stop()==SolveBudget::Stop::Cancelled) {
      solve_predecessor_.reset();
      return false;
    }
    planner_manager_->importAttemptDiagnostics(*solve_worker_);
    last_attempt_failure_phase_=result.failure;
    predecessor_id_=0;
    predecessor_safe_=false;
    predecessor_check_stamp_=builtin_interfaces::msg::Time{};
    if (result.candidate && solve_predecessor_ && solve_predecessor_id_>0 &&
        budget->remainingSeconds()>.012) {
      predecessor_id_=static_cast<std::uint64_t>(solve_predecessor_id_);
      predecessor_safe_=planner_manager_->recheckPredecessor(*solve_predecessor_,
                                                             solve_predecessor_measured_time_);
      if (predecessor_safe_) predecessor_check_stamp_=node_->now();
    }
    solve_predecessor_.reset();
    bool adopted=false;
    if (result.candidate && have_odom_ &&
        (node_->now()-last_odom_time_).seconds()<=odom_timeout_)
      adopted=planner_manager_->adoptSolvedCandidate(std::move(*result.candidate),budget);
    if (!budget->allowed()) last_attempt_failure_phase_=budget->reason();
    else if (result.candidate && !adopted) last_attempt_failure_phase_="waiting_recheck";
    publishAttemptDebug(solve_attempt_header_);
    if (!adopted) {
      ++replan_fail_count_;
      waitForChangedEnvironment();
      return true;
    }
    replan_fail_count_=0;
    flag_escape_emergency_=true;
    publishCommittedTrajectory();
    changeFSMExecState(EXEC_TRAJ,"SOLVE_WORKER_ACCEPTED");
    return true;
  }

  void SCANReplanFSM::publishCommittedTrajectory()
  {
    auto &info=planner_manager_->local_data_;
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order=3;
    bspline.start_time=info.start_time_;
    bspline.traj_id=info.traj_id_;
    const auto points=info.position_traj_.getControlPoint();
    for (int i=0;i<points.cols();++i) {
      geometry_msgs::msg::Point point;
      point.x=points(0,i);point.y=points(1,i);point.z=points(2,i);
      bspline.pos_pts.push_back(point);
    }
    const auto knots=info.position_traj_.getKnot();
    for (int i=0;i<knots.rows();++i) bspline.knots.push_back(knots(i));
    publishTrajectory(bspline);
    publishAcceptedLocalPlanDebug(static_cast<std::uint64_t>(std::max(0,info.traj_id_)));
    if(!execution_validator_)visualization_->displayOptimalTraj(info.position_traj_,0);
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

  bool SCANReplanFSM::getLocalTarget(const SolveBudget::Ptr &budget,GridMap::Ptr* reserved_snapshot)
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
      auto selection=selectReferenceTarget(discrete_reference_, start_pt_,
          discrete_progress_, planning_horizon_, options,
          [this,&budget](const Eigen::Vector3d &point, double yaw) {
            if (!solveAllowed(budget)) return -1;
            const auto map=planner_manager_->grid_map_;
            const Eigen::Vector3d offset=self_double_cylinder_offset_*Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0.);
            const Eigen::Vector3d front=point+offset, rear=point-offset;
            // Test both centers first: the grid's occupied front center would
            // otherwise mask an out-of-map rear center due to short-circuiting.
            if (!map->isInMap(front) || !map->isInMap(rear)) return -1;
            return map->getInflateOccupancy(point,yaw);
          });
      std::vector<Eigen::Vector3d> visible_side_reference;
      if(force_visible_side_target_) {
        selection.valid=false;selection.reason="blocked_executable_entry";
      }
      if(!selection.valid&&execution_validator_&&solveAllowed(budget)) {
        const auto support=execution_validator_->planningSupport(executionVersion(reference_metadata_));
        const auto braking=execution_validator_->brakingModel();
        double measured_yaw=0.;
        const MeasuredBodyPose measured{start_pt_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_,last_odom_time_.nanoseconds()};
        if(support&&braking&&measuredBodyYaw(measured,self_inflation_frame_id_,node_->now().seconds(),odom_timeout_,measured_yaw,node_->now().nanoseconds())) {
          auto snapshot=snapshot_pool_.acquireSolver(*planner_manager_->grid_map_,
              node_->now().nanoseconds(),budget->deadline());
          if(!snapshot){selection.reason="waiting_snapshot_slot";local_target_query_debug_=selection;return false;}
          // Waiting is part of the parent's ORIGINAL 400 ms, not the 80 ms
          // side computation quota. Allocate that sub-budget only after the
          // exclusive fair lease exists; it is capped by original remaining.
          const auto side_budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(80),
              []{return SolveBudget::Clock::now();},budget);
          snapshot->requireObservedSnapshot();
          auto visible=selectVisibleSideTarget(discrete_reference_,start_pt_,measured_yaw,discrete_progress_,
              planning_horizon_,*support,*snapshot,self_inflation_frame_id_,side_budget,1024,&*braking);
          if(visible.target.valid) {
            if(reserved_snapshot)*reserved_snapshot=std::move(snapshot);
            visible_side_reference=std::move(visible.support_path);
            visible.target.queries+=selection.queries;visible.target.free_queries+=selection.free_queries;
            visible.target.unknown_queries+=selection.unknown_queries;visible.target.occupied_queries+=selection.occupied_queries;
            visible.target.outside_queries+=selection.outside_queries;
            selection=std::move(visible.target);
            RCLCPP_INFO(node_->get_logger(),"Visible side reference selected: candidates=%zu target=[%.3f,%.3f,%.3f] queries=%zu; original global route unchanged",
              visible.candidates,selection.point.x(),selection.point.y(),selection.point.z(),selection.queries);
          } else {
            selection.reason=visible.target.reason;
          }
        }
      }
      const double target_arc=selection.arc;
      local_target_query_debug_=selection;
      local_debug_projection_ = discrete_reference_.sample(discrete_progress_);
      local_debug_target_arc_ = target_arc;
      local_target_pt_ = selection.point;
      local_target_vel_.setZero();
      local_debug_selected_reference_ = visible_side_reference.empty() ?
          discrete_reference_.slice(discrete_progress_, target_arc, start_pt_) : std::move(visible_side_reference);
      if (!selection.valid) {
        last_attempt_failure_phase_=selection.reason;
        planner_manager_->setLocalReference({});
        return false;
      }
      if (selection.reason!="reference_target_visible_side"&&target_arc < discrete_reference_.length() - 1e-4)
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
