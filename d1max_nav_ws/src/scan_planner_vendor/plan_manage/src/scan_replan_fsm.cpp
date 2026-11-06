
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/worker_lifecycle.hpp>
#include <cmath>
#include <stdexcept>
#include <optional>
#include <plan_manage/input_contract.hpp>
#include <plan_manage/subscription_purpose.hpp>
#include <plan_manage/local_plan_debug.hpp>
#include <plan_manage/reference_target.hpp>
#include <plan_manage/visible_reference_target.hpp>
#include <plan_manage/stair_attitude_scope.hpp>

namespace
{
  std::uint64_t ownerSourceOrder(const builtin_interfaces::msg::Time& source)
  {
    return static_cast<std::uint64_t>(std::max<std::int64_t>(0,
      static_cast<std::int64_t>(source.sec)*1000000000LL+source.nanosec));
  }

  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  std::vector<ExecutorRole> SCANReplanFSM::executorRoles()const
  {
    return {{"planning_owner",node_->get_node_base_interface()->get_default_callback_group()},
      {"map_writer",map_writer_group_},
      {"map_ingress",grid_map_writer_?grid_map_writer_->projectedIngressGroup():nullptr},
      {"control_inbox",execution_control_group_},
      {"selected_candidate",execution_validator_?execution_validator_->selectedPublicationGroup():nullptr},
      {"display_diagnostics",execution_validator_?execution_validator_->committedDisplayGroup():nullptr}};
  }

  SCANReplanFSM::OwnerMapScope::OwnerMapScope(SCANReplanFSM& owner):owner_(owner)
  {
    if(!owner_.map_writer_group_||owner_.owner_map_depth_>0) {
      ++owner_.owner_map_depth_;entered_=true;return;
    }
    CollisionSnapshotPool::PublicationTiming timing;
    auto view=owner_.snapshot_pool_.borrowOwnerValidation(&timing);
    if(!view)return;
    previous_=std::move(owner_.planner_manager_->grid_map_);
    owner_.planner_manager_->grid_map_=std::move(view);
    owner_.owner_snapshot_timing_=timing;
    borrowed_=entered_=true;++owner_.owner_map_depth_;
  }

  SCANReplanFSM::OwnerMapScope::~OwnerMapScope()
  {
    if(!entered_)return;
    if(borrowed_)owner_.planner_manager_->grid_map_=std::move(previous_);
    --owner_.owner_map_depth_;
  }

  void SCANReplanFSM::deferOwnerInput(OwnerInput input,std::uint64_t order,std::function<void()> callback)
  {
    // One latest message per input, not an unbounded callback/work queue.
    // The original message (and its source/deadline) is captured unchanged.
    const auto i=static_cast<std::size_t>(input);
    if(owner_inputs_[i]&&order<=owner_input_orders_[i])return;
    owner_input_orders_[i]=order;owner_inputs_[i]=std::move(callback);
  }

  void SCANReplanFSM::dispatchOwnerInput(OwnerInput input,std::uint64_t order,std::function<void()> callback)
  {
    if(!map_writer_group_) {callback();return;}
    deferOwnerInput(input,order,std::move(callback));
    OwnerMapScope scope(*this);
    if(scope)drainOwnerInputs();
  }

  void SCANReplanFSM::drainOwnerInputs()
  {
    // Acquire odometry/progress before a pending reference geometry operation.
    // Nested handlers reuse this same snapshot and cannot pin another slot.
    for(auto& pending:owner_inputs_) {
      auto callback=std::move(pending);pending={};
      if(callback)callback();
    }
  }

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    odom_source_history_.clear();
    pending_tracking_progress_.reset();
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
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    strict_input_frames_ = load_parameter<bool>(node_, "fsm.strict_input_frames", false);
    odom_twist_in_body_frame_ = load_parameter<bool>(node_, "fsm.odom_twist_in_body_frame", false);
    reference_path_z_offset_ = load_parameter<double>(node_, "fsm.reference_path_z_offset", body_height_);
    reference_start_tolerance_ = load_parameter<double>(node_, "fsm.reference_start_tolerance", 1.0);
    odom_timeout_ = load_parameter<double>(node_, "fsm.odom_timeout", 0.5);
    max_replan_interval_ = load_parameter<double>(node_, "fsm.max_replan_interval", 0.0);
    min_replan_interval_ = load_parameter<double>(node_, "fsm.min_replan_interval", 0.);
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
    if (!std::isfinite(failed_replan_cooldown_) || failed_replan_cooldown_<=0. || failed_replan_cooldown_>5. ||
        !std::isfinite(failed_replan_body_distance_) || failed_replan_body_distance_<.05 || failed_replan_body_distance_>.5)
      throw std::runtime_error("invalid bounded failed replan policy");
    reference_path_guidance_ = load_parameter<bool>(node_, "fsm.reference_path_guidance", false);
    if (reference_path_guidance_ && navi_mode_ != NAVI_MODE::REFERENCE_PATH)
      throw std::runtime_error("reference_path_guidance is only valid in navigation mode 3");
    if (!std::isfinite(max_replan_interval_) || max_replan_interval_ < 0.0)
      throw std::runtime_error("max_replan_interval must be finite and nonnegative");
    if (!std::isfinite(min_replan_interval_) || min_replan_interval_<0. || min_replan_interval_>5.)
      throw std::runtime_error("min_replan_interval must be between zero and 5 seconds");
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
    local_follower_=load_parameter<bool>(node_,"fsm.local_follower_enabled",false);
    const auto execution_mode_policy=load_parameter<std::string>(node_,"fsm.execution_mode_policy","segment_modes");
    if(execution_mode_policy!="segment_modes"&&execution_mode_policy!="fixed_stair")
      throw std::runtime_error("invalid execution mode policy");
    fixed_stair_reference_continuity_=local_follower_&&execution_mode_policy=="fixed_stair";
    solve_worker_enabled_=load_parameter<bool>(node_,"fsm.solve_worker_enabled",true);
    const bool execution_protocol=load_parameter<bool>(node_,"fsm.execution_protocol",false);
    if(local_follower_&&solve_worker_enabled_&&reference_path_guidance_&&execution_protocol)
      map_writer_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_,map_writer_group_);
    grid_map_writer_=planner_manager_->grid_map_;
    require_schema_v2_=load_parameter<bool>(node_,"fsm.require_reference_schema_v2",false);
    planning_only_=load_parameter<bool>(node_,"fsm.planning_only",false);
    if (solve_worker_enabled_ && reference_path_guidance_) {
      solve_worker_=std::make_unique<SCANPlannerManager>();
      solve_worker_->initSolveWorker(*planner_manager_);
      planner_manager_->releaseOwnerSolver();
    }
    const auto kinematic_profile=parseKinematicProfile(
        load_parameter<std::string>(node_,"fsm.kinematic_profile","forward_yaw"));
    if(planning_only_&&!execution_protocol)
      throw std::runtime_error("planning-only requires the formal geometry evidence protocol");
    if(execution_protocol) {
      const auto mode=load_parameter<std::string>(node_,"fsm.execution_transport_mode","live");
      execution_transport_mode_=mode;
      continuation_max_jerk_=load_parameter<double>(node_,"fsm.continuation_max_jerk",.7);
      if(!require_schema_v2_ || !require_tagged_reference_ || std::abs(reference_path_z_offset_)>1e-9)
        throw std::runtime_error("execution protocol requires tagged body-center reference and zero z offset");
      std::optional<BrakingModel> braking;
      const auto braking_record=load_parameter<std::string>(node_,"fsm.execution_braking_model_record","");
      const auto braking_sha=load_parameter<std::string>(node_,"fsm.execution_braking_model_sha256","");
      const bool writer_handoff=load_parameter<bool>(node_,"fsm.writer_handoff_enabled",false);
      local_follower_=load_parameter<bool>(node_,"fsm.local_follower_enabled",false);
      if(local_follower_&&(writer_handoff||planning_only_))throw std::runtime_error("local follower cannot use legacy writer/preview ownership");
      if(writer_handoff&&kinematic_profile==KinematicProfile::ForwardYaw&&
         planner_manager_->configuredHeadingPolicy()==BodyHeadingPolicy::HoldHeading)
        throw std::runtime_error("writer hold heading requires holonomic kinematic profile");
      if(planning_only_&&(writer_handoff||!braking_record.empty()||!braking_sha.empty()))
        throw std::runtime_error("planning-only cannot bind a motion writer or braking model");
      if(!braking_record.empty()||!braking_sha.empty())braking=BrakingModel::load(braking_record,braking_sha,mode);
      else RCLCPP_WARN(node_->get_logger(),"No motion parameter record: geometry preview only; motion proof unavailable");
      if(braking&&braking->kinematic_profile!=kinematic_profile)
        throw std::runtime_error("kinematic profile does not match hashed motion parameter record");
      if(braking&&load_parameter<double>(node_,"grid_map.cloud_pose_max_age",.5)>braking->sensor_source_age)
        throw std::runtime_error("map sensor source age exceeds bound execution record");
      const auto geometry_policy=load_parameter<std::string>(node_,"fsm.trajectory_geometry_policy","observed_free");
      if(geometry_policy=="official_inflated_geometry"&&planner_manager_->grid_map_->planningRequiresObservedFree())
        throw std::runtime_error("official trajectory geometry requires official planning collision policy");
      execution_validator_=std::make_unique<ExecutionValidator>(node_,snapshot_pool_,self_inflation_frame_id_,mode,
          bodyExtent(planner_manager_->grid_map_->planningBodyEnvelope()),std::move(braking),
          writer_handoff,geometry_policy,planning_only_,local_follower_,
          load_parameter<std::string>(node_,"fsm.control_body_frame","base_link"),
          [writer=planner_manager_->grid_map_]{return writer->nativePipelineHealth();},
          load_parameter<double>(node_,"fsm.component_callback_timeout_s",1.5));
      execution_receipt_pub_=node_->create_publisher<ew::ReferenceReceipt>(
          "/d1max/live_planning/execution/reference_receipt",rclcpp::QoS(2));
      execution_support_sub_=node_->create_subscription<ew::SupportReference>(
          "/d1max/live_planning/execution/support",local_follower_?rclcpp::QoS(8).reliable().transient_local():rclcpp::QoS(2),
          [this](ew::SupportReference::ConstSharedPtr m){execution_validator_->support(*m);});
      // Control and source-measured progress touch only the locked validator
      // ledger, never FSM/GridMap buffers. Neither waits behind map fusion.
      execution_control_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
      rclcpp::SubscriptionOptions control_options;control_options.callback_group=execution_control_group_;
      // The formal writer and tracker consume the SAME original local state.
      // Reference-window preparation must not sit in this 50 Hz data path.
      // The FSM/GridMap may still use body_pose for planning, but may not
      // overwrite the validator's typed independent acquired-body ledger.
      if(local_follower_) {
        execution_validator_->configureLocalBodyInput(navigation_session_id_,
          load_parameter<std::string>(node_,"fsm.navigation_map_version_id",""),
          load_parameter<std::string>(node_,"fsm.control_body_frame","base_link"));
        execution_local_body_sub_=node_->create_subscription<ew::LocalNavigationState>(
          load_parameter<std::string>(node_,"fsm.local_navigation_state_topic",
            "/d1max/localization/navigation/local_state"),rclcpp::QoS(1).reliable(),
          [this](ew::LocalNavigationState::ConstSharedPtr state){
            execution_validator_->localBody(*state);
          },control_options);
      } else execution_body_sub_=node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose",rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto& p=m->pose.pose.position;const auto& q=m->pose.pose.orientation;
          MeasuredBodyPose body{{p.x,p.y,p.z},Eigen::Quaterniond(q.w,q.x,q.y,q.z),
            rclcpp::Time(m->header.stamp).seconds(),m->header.frame_id};
          double yaw=0.;
          if(measuredBodyYaw(body,self_inflation_frame_id_,node_->now().seconds(),odom_timeout_,yaw)) {
            try {
              auto twist=m->twist.twist;
              const auto linear=odometryVelocityInWorld(
                {twist.linear.x,twist.linear.y,twist.linear.z},body.orientation,odom_twist_in_body_frame_);
              const auto angular=odometryVelocityInWorld(
                {twist.angular.x,twist.angular.y,twist.angular.z},body.orientation,odom_twist_in_body_frame_);
              twist.linear.x=linear.x();twist.linear.y=linear.y();twist.linear.z=linear.z();
              twist.angular.x=angular.x();twist.angular.y=angular.y();twist.angular.z=angular.z();
              execution_validator_->body(body,twist,rclcpp::Time(m->header.stamp).nanoseconds());
            } catch(const std::invalid_argument&) {
              // Invalid source velocity cannot become a braking-state sample.
            }
          }
        },control_options);
      execution_permit_sub_=node_->create_subscription<ew::ExecutionPermit>(
          "/d1max/live_planning/execution/permit",rclcpp::QoS(2),
          [this](ew::ExecutionPermit::ConstSharedPtr m){execution_validator_->commit(*m);},control_options);
      if (nativeMotionSubscriptionsEnabled(planning_only_)) {
        if(local_follower_) {
          execution_local_commit_result_pub_=node_->create_publisher<ew::LocalTrajectoryCommit>(
            "/d1max/live_planning/execution/local_commit_result",rclcpp::QoS(8).reliable().transient_local());
          execution_local_commit_sub_=node_->create_subscription<ew::LocalTrajectoryCommit>(
            "/d1max/live_planning/execution/local_commit",rclcpp::QoS(8).reliable().transient_local(),
            [this](ew::LocalTrajectoryCommit::ConstSharedPtr m){
              execution_local_commit_result_pub_->publish(execution_validator_->localCommitExchange(*m));
            },control_options);
          execution_task_lease_sub_=node_->create_subscription<ew::TaskExecutionLease>(
            "/d1max/live_planning/execution/task_lease",rclcpp::QoS(2).reliable(),
            [this](ew::TaskExecutionLease::ConstSharedPtr m){execution_validator_->taskLease(*m);},control_options);
        } else {
        execution_handoff_sub_=node_->create_subscription<ew::ExecutionHandoffGrant>(
            "/d1max/live_planning/execution/handoff_grant",rclcpp::QoS(2),
            [this](ew::ExecutionHandoffGrant::ConstSharedPtr m){execution_validator_->handoff(*m);},control_options);
        execution_prepared_sub_=node_->create_subscription<ew::PreparedMotionDemand>(
            "/d1max/live_planning/execution/prepared_demand",rclcpp::QoS(1),
            [this](ew::PreparedMotionDemand::ConstSharedPtr m){execution_validator_->preparedDemand(*m);},control_options);
        execution_commit_ack_sub_=node_->create_subscription<ew::ExecutionCommitAck>(
            "/d1max/live_planning/execution/commit_ack",rclcpp::QoS(8),
            [this](ew::ExecutionCommitAck::ConstSharedPtr m){execution_validator_->commitAck(*m);},control_options);
        // Archived v3 braking-volume re-entry only. task_follower_v1 reacts to
        // Native's actual committed-curve verdict and has no MotionValidation
        // inbox that could reintroduce a second collision gate.
        execution_blocked_entry_sub_=node_->create_subscription<ew::MotionValidation>(
            "/d1max/live_planning/execution/blocked_entry",rclcpp::QoS(4),
            [this](ew::MotionValidation::ConstSharedPtr m){execution_validator_->submitBlockedEntry(*m);},control_options);
        }
        execution_progress_sub_=node_->create_subscription<ew::TrackingProgress>(
            "planning/tracking_progress",rclcpp::QoS(1),
            [this](ew::TrackingProgress::ConstSharedPtr m){execution_validator_->progress(*m);},control_options);
        execution_demand_sub_=node_->create_subscription<ew::MotionDemand>(
            "/d1max/live_planning/execution/demand",rclcpp::QoS(1),
            [this](ew::MotionDemand::ConstSharedPtr m){execution_validator_->demand(*m);},control_options);
        execution_admission_sub_=node_->create_subscription<ew::TrajectoryAdmission>(
            "/d1max/live_planning/execution/admission",rclcpp::QoS(1),
            [this](ew::TrajectoryAdmission::ConstSharedPtr m){execution_validator_->submitCandidateEntryRejection(*m);},control_options);
      }
      execution_proposal_sub_=node_->create_subscription<ew::ReferenceProposal>(
          "/d1max/live_planning/execution/reference_proposal",local_follower_?rclcpp::QoS(8).reliable().transient_local():rclcpp::QoS(1),
          [this,mode](ew::ReferenceProposal::ConstSharedPtr m){
            dispatchOwnerInput(OwnerInput::Proposal,m->reference.generation,
              [this,m,mode]{receiveReferenceProposal(m,mode);});
          });
      // Publish immediately after the writer's fusion transaction. A second
      // independent 200 ms timer can add an entire stale-map cycle by phase.
      // The 20 Hz validator thread still only reads immutable snapshots.
      grid_map_writer_->setCollisionUpdateCallback([this]{
        // Runs only on the map writer group. Never read the manager's query
        // pointer here: its owner can simultaneously hold a private snapshot.
        map_validation_publication_pending_=true;
        // Admission interest is an advisory hint. Never take the validator's
        // ledger lock before publishing the fast current-curve snapshot.
        map_solver_publication_pending_=execution_validator_!=nullptr;
        publishMapSnapshots();
        // Compatibility has no separate map group. A formal writer must not
        // touch the FSM future, manager or adoption state on this thread.
        if(!map_writer_group_&&local_follower_&&solve_future_.valid()&&
           solve_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready)pollSolveWorker();
      });
      if(map_writer_group_) {
        // A previous complete fusion can miss both borrowed validator slots.
        // The 10 ms retry below shares this serial writer with the next long
        // integration, so give that pending delivery a chance BEFORE starting
        // it. Do not mark a new update or copy an already delivered head.
        grid_map_writer_->setCollisionPublicationRetryCallback([this]{
          if(map_validation_publication_pending_)publishMapSnapshots();
          return !map_validation_publication_pending_;
        });
        // All three bounded slots can briefly be borrowed at fusion completion.
        // Deliver that completed writer state when a slot is released; do not
        // wait for another sensor integration, renew its ray times or allocate
        // another map. Serial with fusion, so no partially written map is read.
        map_publication_timer_=node_->create_wall_timer(std::chrono::milliseconds(10),
          [this]{publishMapSnapshots();},map_writer_group_);
      }
    }
    // Zero keeps the upstream smoothness/collision/feasibility objective.
    // Optional reference attraction is a D1 extension, not a mode-3 requirement.
    /* callback */
    // Author main / pinned ROS2: 100 Hz FSM, independent 20 Hz collision
    // checks. This is a state/poll cadence, NOT a 100 Hz optimization worker.
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
          dispatchOwnerInput(OwnerInput::Odom,ownerSourceOrder(m->header.stamp),[this,m]{odometryCallback(m);});
        });
    if (nativeMotionSubscriptionsEnabled(planning_only_)) {
      tracking_progress_sub_=node_->create_subscription<d1max_planning_interfaces::msg::TrackingProgress>(
          "planning/tracking_progress",rclcpp::QoS(1).best_effort(),
          [this](ew::TrackingProgress::ConstSharedPtr m){
            dispatchOwnerInput(OwnerInput::Progress,ownerSourceOrder(m->header.stamp),[this,m]{trackingProgressCallback(m);});
          });
    }
    // Freshness comes from GridMap's integrated sensor sources. A raw cloud
    // receipt has no independent lease and needs no second FSM subscription.
    if (nativeMotionSubscriptionsEnabled(planning_only_)) {
      // The formal ledger carries the writer's exact holding/tracking state.
      // The old unversioned Go2 topic belongs only to compatibility sessions.
      if (!execution_validator_)
        go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "planning/go2_execution_frozen", 10,
            std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    }

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    tagged_bspline_pub_ = node_->create_publisher<d1max_planning_interfaces::msg::TaggedBspline>(
        "planning/tagged_bspline", local_follower_?rclcpp::QoS(1).reliable().transient_local():rclcpp::QoS(1).reliable());
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
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
            dispatchOwnerInput(OwnerInput::Goal,ownerSourceOrder(m->header.stamp),[this,m]{rvizGoalCallback(m);});
          });
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      if (require_tagged_reference_)
        typed_path_sub_ = node_->create_subscription<d1max_planning_interfaces::msg::ReferencePath>(
            "typed_initial_path", rclcpp::QoS(1).reliable(),
            [this](ew::ReferencePath::ConstSharedPtr m){
              // A formal cancellation uses only owner/ledger identity. It
              // must retire the task even when no query snapshot is available.
              // Compatibility may synthesize an emergency curve, so retains
              // its original map-scoped path.
              if(execution_validator_&&m->path.poses.empty()) {
                typedPathCallback(m);return;
              }
              dispatchOwnerInput(OwnerInput::TypedPath,m->generation,[this,m]{typedPathCallback(m);});
            });
      else
        path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "initial_path", 1,[this](nav_msgs::msg::Path::ConstSharedPtr m){
              dispatchOwnerInput(OwnerInput::Path,ownerSourceOrder(m->header.stamp),[this,m]{pathCallback(m);});
            });
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
    const int final_occ = map->getPlanningInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getPlanningInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
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

  std::optional<std::vector<Eigen::Vector3d>> SCANReplanFSM::prepareRollingReference(const ew::ReferencePath &msg) const
  {
    if(!reference_path_guidance_||!have_odom_||(node_->now()-last_odom_time_).seconds()>odom_timeout_||
       std::abs(reference_path_z_offset_)>1e-9)return {};
    if(msg.schema_version!=2||msg.point_reference!="body_center"||
       msg.path.header.frame_id!=self_inflation_frame_id_||msg.task_id.empty()||
       msg.route_id.empty()||msg.route_hash.empty()||msg.map_version_id.empty()||msg.segment_id.empty()||
       msg.point_segment_ids.size()!=msg.path.poses.size()||
       msg.point_segment_kinds.size()!=msg.path.poses.size()||
       msg.point_required_modes.size()!=msg.path.poses.size())return {};
    std::vector<Eigen::Vector3d> waypoints;
    for(const auto& pose:msg.path.poses) {
      if(!pose.header.frame_id.empty()&&pose.header.frame_id!=self_inflation_frame_id_)return {};
      const auto& p=pose.pose.position;waypoints.emplace_back(p.x,p.y,p.z);
    }
    try {
      waypoints=prepareReferenceWaypoints(waypoints,odom_pos_,reference_path_z_offset_,reference_start_tolerance_);
      DiscreteReference validated;setReferenceFromBody(validated,odom_pos_,waypoints);
    } catch(const std::invalid_argument& error) {
      const Eigen::Vector3d first=waypoints.empty()?Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN()):waypoints.front();
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "Reference geometry preflight rejected generation=%lu reason=%s distance=%.6f limit=%.6f body=(%.6f %.6f %.6f) first=(%.6f %.6f %.6f)",
        msg.generation,error.what(),(first-odom_pos_).norm(),reference_start_tolerance_,
        odom_pos_.x(),odom_pos_.y(),odom_pos_.z(),first.x(),first.y(),first.z());
      return {};
    }
    return waypoints;
  }

  void SCANReplanFSM::installRollingReference(const ew::ReferencePath &msg,const std::vector<Eigen::Vector3d> &waypoints)
  {
    // Owner callback is serialized. Geometry and body input were checked
    // before Native's exact predecessor CAS; there is no second admission
    // step after that CAS that could leave proposal and geometry split.
    reference_metadata_=msg;reference_generation_=msg.generation;have_reference_generation_=true;
    trigger_=true;planGlobalTrajByWaypoints(waypoints);
    changeFSMExecState(REPLAN_TRAJ,"ROLLING_REFERENCE");
  }

  bool SCANReplanFSM::rollingReferenceBusy()
  {
    return solveWorkPending()||pending_snapshot_request_.pending()||
      (execution_validator_&&execution_validator_->localCommitInFlight())||
      keepPendingSpliceForOrdinaryReplan();
  }

  void SCANReplanFSM::rejectDeferredRollingProposal(const std::string &reason,bool retry_priority)
  {
    if(!deferred_rolling_proposal_)return;
    const auto m=std::move(deferred_rolling_proposal_);
    const auto now=node_->now();
    ew::ReferenceReceipt ack;ack.version=m->version;ack.proposal_id=m->proposal_id;
    ack.expected_version=m->expected_version;ack.expected_trajectory_id=m->expected_trajectory_id;
    ack.source_stamp=now;ack.valid_until=now+rclcpp::Duration::from_seconds(.5);
    ack.transport_mode=execution_transport_mode_;ack.accepted=false;ack.reason=reason;
    const auto committed=execution_validator_->committedIdentity();
    RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
      "Reference rejected generation=%lu reason=%s expected_generation=%lu expected_curve=%ld committed_generation=%lu committed_curve=%ld",
      m->reference.generation,ack.reason.c_str(),m->expected_version.reference_generation,
      m->expected_trajectory_id,committed?committed->version.reference_generation:0UL,
      committed?committed->trajectory_id:-1L);
    execution_receipt_history_.remember(*m,ack,execution_proposal_?
      std::optional<ew::ExecutionVersion>(execution_proposal_->version):std::nullopt,committed?
      std::optional<ew::ExecutionVersion>(committed->version):std::nullopt);
    execution_receipt_pub_->publish(ack);
    // One upstream re-proposal interval, not a new source/curve lease. This
    // prevents immediately starting another old-window splice between a
    // rejected stale predecessor and its fresh 500 ms owner retry.
    if(retry_priority)rolling_retry_priority_until_=std::chrono::steady_clock::now()+
      std::chrono::milliseconds(650);
  }

  void SCANReplanFSM::processDeferredRollingProposal()
  {
    if(!deferred_rolling_proposal_)return;
    const auto now=node_->now();
    if(!(rclcpp::Time(deferred_rolling_proposal_->valid_until)>now)||
       (now-rclcpp::Time(deferred_rolling_proposal_->source_stamp)).seconds()>1.) {
      rejectDeferredRollingProposal("rolling_reference_original_deadline_elapsed",true);return;
    }
    if(!rollingReferenceExtension(reference_metadata_,deferred_rolling_proposal_->reference,fixed_stair_reference_continuity_)) {
      rejectDeferredRollingProposal("rolling_reference_owner_changed",false);return;
    }
    if(rollingReferenceBusy())return;
    const auto m=std::move(deferred_rolling_proposal_);
    receiveReferenceProposal(m,execution_transport_mode_);
  }

  void SCANReplanFSM::receiveReferenceProposal(ew::ReferenceProposal::ConstSharedPtr m,const std::string &mode)
  {
    const auto previous=execution_receipt_history_.lookup(*m);
    using ReceiptMatch=ReferenceReceiptHistory<ew::ReferenceProposal,ew::ReferenceReceipt>::Match;
    if(previous.match==ReceiptMatch::Exact) {
      // Recover the literal original receipt, not a new geometry admission.
      execution_receipt_pub_->publish(*previous.receipt);return;
    }
    if(previous.match==ReceiptMatch::PayloadConflict) {
      RCLCPP_WARN(node_->get_logger(),"Reject reference identity reused with different payload");return;
    }
    if(deferred_rolling_proposal_&&deferred_rolling_proposal_->version==m->version) {
      if(*deferred_rolling_proposal_!=*m)
        RCLCPP_WARN(node_->get_logger(),"Reject queued reference identity reused with different payload");
      return; // Original pending request/deadline remains unchanged.
    }
    if(m->reference.session_id==navigation_session_id_&&have_reference_generation_&&
       m->reference.generation<=reference_generation_) {
      RCLCPP_WARN(node_->get_logger(),"Ignore uncached historical reference revision");return;
    }
    const auto received_at=node_->now();
    ew::ReferenceReceipt ack;ack.version=m->version;ack.proposal_id=m->proposal_id;
    ack.expected_version=m->expected_version;ack.expected_trajectory_id=m->expected_trajectory_id;
    ack.source_stamp=received_at;ack.valid_until=received_at+rclcpp::Duration::from_seconds(.5);
    ack.transport_mode=mode;ack.reason="reference_contract_rejected";
    const double age=(received_at-rclcpp::Time(m->source_stamp)).seconds();
    if(m->transport_mode==mode&&!m->proposal_id.empty()&&age>=-.02&&age<=1.&&
       rclcpp::Time(m->valid_until)>received_at&&m->version==executionVersion(m->reference)&&
       acceptsReferenceGeneration(navigation_session_id_,m->reference.session_id,
         have_reference_generation_,reference_generation_,m->reference.generation)&&
       m->reference.point_reference=="body_center"&&m->reference.path.header.frame_id==self_inflation_frame_id_&&
       !m->reference.path.poses.empty()) {
      const bool rolling=local_follower_&&have_target_&&
        rollingReferenceExtension(reference_metadata_,m->reference,fixed_stair_reference_continuity_);
      if(rolling&&rollingReferenceBusy()) {
        rejectDeferredRollingProposal("rolling_reference_superseded",false);
        deferred_rolling_proposal_=m;return;
      }
      // Ordinary map corrections produce a new anchor, so they take the
      // non-rolling path too. Preflight EVERY candidate before Native's CAS
      // and typedPathCallback's intentional replacement/retirement effects.
      // Otherwise one malformed or temporarily stale reanchor clears the
      // incumbent search reference before its negative receipt is returned.
      const auto prepared=prepareRollingReference(m->reference);
      if(!prepared) {
        // No candidate, active geometry, generation or task mutation.
        ack.reason=rolling?"rolling_reference_input_not_ready":"reference_geometry_rejected";
      } else if(execution_validator_->replacePendingReference(*m,rolling,
          rolling&&fixed_stair_reference_continuity_&&reference_metadata_.segment_id!=m->reference.segment_id,
          rolling?std::optional<ew::ExecutionVersion>(executionVersion(reference_metadata_)):std::nullopt)) {
        rejectDeferredRollingProposal("rolling_reference_superseded",false);
        execution_proposal_=*m;
        if(rolling)installRollingReference(m->reference,*prepared);
        else replaceTypedReference(std::make_shared<ew::ReferencePath>(m->reference),&*prepared);
        ack.accepted=have_target_&&reference_generation_==m->reference.generation;
        ack.reason=ack.accepted?"pending_reference_received_not_motion_authority":"reference_geometry_rejected";
        if(ack.accepted)rolling_retry_priority_until_={};
      } else if(rolling) {
        ack.reason="rolling_reference_predecessor_changed_retry";
        rolling_retry_priority_until_=std::chrono::steady_clock::now()+std::chrono::milliseconds(650);
      }
    }
    const auto committed=execution_validator_->committedIdentity();
    if(!ack.accepted)RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
      "Reference rejected generation=%lu reason=%s expected_generation=%lu expected_curve=%ld committed_generation=%lu committed_curve=%ld",
      m->reference.generation,ack.reason.c_str(),m->expected_version.reference_generation,
      m->expected_trajectory_id,committed?committed->version.reference_generation:0UL,
      committed?committed->trajectory_id:-1L);
    execution_receipt_history_.remember(*m,ack,execution_proposal_?
      std::optional<ew::ExecutionVersion>(execution_proposal_->version):std::nullopt,committed?
      std::optional<ew::ExecutionVersion>(committed->version):std::nullopt);
    execution_receipt_pub_->publish(ack);
  }

  void SCANReplanFSM::typedPathCallback(
      const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr &msg)
  {
    replaceTypedReference(msg,nullptr);
  }

  void SCANReplanFSM::replaceTypedReference(
      const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr &msg,
      const std::vector<Eigen::Vector3d> *prepared)
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
      rejectDeferredRollingProposal("rolling_reference_cancelled",false);
      rolling_retry_priority_until_={};
      if(last_visualized_committed_)visualization_->clearOptimalTraj(0);
      last_visualized_committed_.reset();last_execution_debug_key_.reset();
    }
    if(local_follower_&&reference_path_guidance_&&have_target_&&
       rollingReferenceExtension(reference_metadata_,*msg,fixed_stair_reference_continuity_)) {
      // A moving window is new SEARCH geometry, not a new task/active curve.
      // Validate before mutation; all committed geometry/progress/limiter and
      // its collision readers retain their original identity and evidence.
      if(rollingReferenceBusy())return;
      const auto prepared=prepareRollingReference(*msg);if(!prepared)return;
      installRollingReference(*msg,*prepared);
      return;
    }
    rejectDeferredRollingProposal("rolling_reference_replaced",false);
    rolling_retry_priority_until_={};
    cancelSolveWorker();
    actual_body_seed_request_.reset();
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
    if(prepared) {
      // The serialized owner already admitted this exact body/geometry before
      // the native CAS. Do not re-read the wall clock after retiring the old
      // reference: crossing an odom-age boundary here must not split admission
      // from installation. Subsequent solve/motion still use fresh source data.
      installReferenceWaypoints(*prepared);
    } else pathCallback(std::make_shared<nav_msgs::msg::Path>(msg->path));
  }

  void SCANReplanFSM::publishTrajectory(const scan_planner_msgs::msg::Bspline &trajectory)
  {
    auto published=trajectory;
    const auto heading=planner_manager_->acceptedHeadingContract();
    if(!encodeFrozenHeading(published,heading)) {
      publishInvalidLocalPlanDebug("invalid_frozen_heading_trajectory",false);return;
    }
    bspline_pub_->publish(published);  // legacy/debug only; not a motion authority
    if (require_tagged_reference_ && have_reference_generation_)
    {
      d1max_planning_interfaces::msg::TaggedBspline tagged;
      tagged.session_id = navigation_session_id_;
      tagged.generation = reference_generation_;
      tagged.frame_id = self_inflation_frame_id_;
      tagged.trajectory = published;
      tagged.schema_version=reference_metadata_.schema_version;
      // This is the original geometry heading seed/lattice, NOT a replacement
      // for the acquired join_pose. Moving splices and low-speed replacements
      // must replay the same full-curve memory in native and controller.
      if(measuredHeadingEnabled(heading)) {
        auto velocity=planner_manager_->local_data_.position_traj_.getDerivative();
        double speed_bound=0.;const auto controls=velocity.getControlPoint();
        for(int i=0;i<controls.cols();++i)speed_bound=std::max(speed_bound,controls.col(i).norm());
        const auto n=bodyHeadingSamplingIntervals(planner_manager_->local_data_.position_traj_.getTimeSum(),
          speed_bound,planner_manager_->grid_map_->getResolution());
        if(!n||!validMeasuredHeadingContract(heading)) {
          publishInvalidLocalPlanDebug("invalid_frozen_body_heading_seed",false);return;
        }
        tagged.body_heading_seed_valid=true;tagged.body_heading_seed_yaw=heading.measured_yaw;
        tagged.body_heading_sampling_intervals=*n;
      }
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
      if(local_follower_)if(const auto& splice=planner_manager_->acceptedSplice())tagged.splice=splice->wire;
      if (!planning_only_) if (const auto &join=planner_manager_->acceptedJoinEvidence()) {
        tagged.valid_start_time=join->curve_time;
        tagged.valid_start_arc_length=join->arc_length;
        tagged.join_source_stamp=rclcpp::Time(
            static_cast<std::int64_t>(std::llround(join->measured.source_stamp*1e9)),
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
      // Formal candidates are published by Native's single admission selector.
      // Sending every solve here would let the next 100 ms result erase the
      // candidate currently undergoing its bounded native check/preparation.
      // Preview and the legacy transport retain their existing direct topic.
      if(!local_follower_)tagged_bspline_pub_->publish(tagged);
      if(execution_validator_&&execution_proposal_&&execution_proposal_->reference.generation==tagged.generation)
        execution_validator_->candidate(tagged,*execution_proposal_,
            heading,planning_only_?
              planner_manager_->acceptedPreviewGeometryEvidence():std::nullopt);
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
    const auto now=node_->now();
    const auto diagnostic=execution_validator_->committedDiagnosticAt(now.nanoseconds(),odom_timeout_);
    if(!diagnostic) {
      if(last_visualized_committed_)visualization_->clearOptimalTraj(0);
      last_visualized_committed_.reset();last_execution_debug_key_.reset();return false;
    }
    const auto key=ExecutionDebugKey{diagnostic->version,diagnostic->trajectory_id,diagnostic->phase};
    const auto published=std::chrono::steady_clock::now();
    // Same status is diagnostic telemetry at <=20 Hz. Identity/phase edges
    // bypass this throttle; collision proofs and control are never gated here.
    if(last_execution_debug_key_&&*last_execution_debug_key_==key&&
       published-last_execution_debug_publish_<std::chrono::milliseconds(50))return true;
    const auto view=execution_validator_->committedView();if(!view)return false;
    const auto& spline=view->spline;
    if(executionVersion(spline)!=diagnostic->version||spline.trajectory.traj_id!=diagnostic->trajectory_id)
      return true; // A real concurrent commit wins; retry its exact view next poll.
    const bool fresh=view->proof&&view->proof->valid&&
      view->proof->version==diagnostic->version&&view->proof->trajectory_id==diagnostic->trajectory_id&&
      (local_follower_?
        rclcpp::Time(view->proof->valid_until).nanoseconds()==0&&
        rclcpp::Time(view->proof->check_end).nanoseconds()>0&&
        rclcpp::Time(view->proof->check_end).nanoseconds()<=now.nanoseconds()+20000000LL:
        rclcpp::Time(view->proof->valid_until)>now&&
        (now-rclcpp::Time(view->proof->body_source_stamp)).seconds()<=odom_timeout_);
    const auto actual_phase=fresh&&view->debug?std::string("accepted"):
      view->proof&&!view->proof->valid?view->proof->reason:std::string("waiting_current_validation");
    if(actual_phase!=diagnostic->phase)return true; // Proof changed between the two bounded reads.
    last_execution_debug_key_=key;last_execution_debug_publish_=published;
    if(fresh&&view->debug) {
      auto debug=*view->debug;debug.header=localPlanDebugHeader();debug.phase="accepted";debug.valid=true;
      debug.checked_map_source_stamp_ns=std::min(rclcpp::Time(view->proof->front_ray_source_stamp).nanoseconds(),
                                                 rclcpp::Time(view->proof->rear_ray_source_stamp).nanoseconds());
      debug.checked_body_source_stamp_ns=rclcpp::Time(view->proof->body_source_stamp).nanoseconds();
      debug.checked_map_revision=view->proof->map_snapshot_revision;
      debug.checked_context_sequence=view->proof->version.context_sequence;
      local_plan_debug_pub_->publish(debug);
      const auto curve_identity=std::make_pair(diagnostic->version,diagnostic->trajectory_id);
      if(!last_visualized_committed_||*last_visualized_committed_!=curve_identity) {
        Eigen::MatrixXd points(3,spline.trajectory.pos_pts.size());Eigen::VectorXd knots(spline.trajectory.knots.size());
        for(std::size_t i=0;i<spline.trajectory.pos_pts.size();++i) {
          const auto& p=spline.trajectory.pos_pts[i];points.col(i)=Eigen::Vector3d(p.x,p.y,p.z);
        }
        for(std::size_t i=0;i<spline.trajectory.knots.size();++i)knots[i]=spline.trajectory.knots[i];
        UniformBspline curve(points,3,.1);curve.setKnot(knots);visualization_->displayOptimalTraj(curve,0);
        last_visualized_committed_=curve_identity;
      }
    } else {
      local_plan_debug_pub_->publish(makeInvalidLocalPlanDebug(localPlanDebugHeader(),spline.session_id,
        spline.generation,static_cast<std::uint64_t>(spline.trajectory.traj_id),
        view->proof&&!view->proof->valid?view->proof->reason:"waiting_current_validation"));
    }
    return true;
  }

  NoMotionPreviewProgress SCANReplanFSM::planningOnlyCommittedProgress() const
  {
    if(!planning_only_||!execution_validator_||!reference_path_guidance_||
       !require_tagged_reference_||!have_target_||!have_odom_||odom_source_history_.empty()||
       odom_source_history_.back().source_ns!=last_odom_time_.nanoseconds())return {};
    const auto map=planner_manager_->grid_map_;
    const auto now=node_->now().nanoseconds();
    if(!map->integratedCloudFreshAt(now))return {};
    return execution_validator_->planningOnlyProgress(executionVersion(reference_metadata_),
      planner_manager_->local_data_.traj_id_,map->localizationContextSequence(),
      MeasuredBodyPose{odom_pos_,odom_orient_,last_odom_time_.seconds(),odom_source_history_.back().frame},
      last_odom_time_.nanoseconds(),now,map->getResolution());
  }

  std::optional<double> SCANReplanFSM::planningOnlyCommittedCurveTime() const
  {
    const auto progress=planningOnlyCommittedProgress();
    return progress.connected()?std::optional<double>(progress.curve_time):std::nullopt;
  }

  bool SCANReplanFSM::tryReplanPlanningOnlyConnection(const NoMotionPreviewProgress& progress)
  {
    const double age=(node_->now()-last_odom_time_).seconds();
    if(!planning_only_||!reference_path_guidance_||!have_target_||!have_odom_||
       exec_state_!=EXEC_TRAJ||progress.status!=NoMotionPreviewStatus::ReplanFromMeasuredBody||
       age<-.02||age>.4||!planner_manager_->grid_map_->integratedCloudFreshAt(node_->now().nanoseconds())||
       !formalReseedSubmissionDue(std::chrono::steady_clock::now(),last_solve_submit_,solveWorkPending(),min_replan_interval_))return false;
    // Keep the same route/reference and original incumbent evidence. The
    // existing replan path takes measured odometry as its new entry boundary;
    // a projection failure can no longer strand EXEC_TRAJ indefinitely.
    last_attempt_failure_phase_="preview_connection_requires_measured_reseed";
    changeFSMExecState(REPLAN_TRAJ,"MEASURED_PREVIEW_CONNECTION_RESEED");
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
    installReferenceWaypoints(waypoints);
  }

  void SCANReplanFSM::installReferenceWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
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
    geometry_msgs::msg::Twist world_twist=msg->twist.twist;
    try
    {
      if (!position.allFinite()) throw std::invalid_argument("non-finite position");
      velocity = odometryVelocityInWorld(
          Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z), orientation, odom_twist_in_body_frame_);
      const auto angular=odometryVelocityInWorld(
          {world_twist.angular.x,world_twist.angular.y,world_twist.angular.z},
          orientation,odom_twist_in_body_frame_);
      world_twist.linear.x=velocity.x();world_twist.linear.y=velocity.y();world_twist.linear.z=velocity.z();
      world_twist.angular.x=angular.x();world_twist.angular.y=angular.y();world_twist.angular.z=angular.z();
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
    odom_source_history_.push_back({source_time.nanoseconds(),position,msg->header.frame_id});
    // Hard bounds apply even if input rate changes. Never extrapolate from the
    // newest sample, retain unbounded messages, or refresh a source stamp.
    while (odom_source_history_.size()>64 ||
           (!odom_source_history_.empty() && source_time.nanoseconds()-
            odom_source_history_.front().source_ns>1000000000LL))
      odom_source_history_.pop_front();
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_ = velocity;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_ = orientation;
    planner_manager_->setMeasuredBodyPose(
        MeasuredBodyPose{position,orientation,source_time.seconds(),msg->header.frame_id},
        self_inflation_frame_id_,odom_timeout_);
    planner_manager_->setMeasuredBodyVelocity(velocity);
    if(execution_validator_&&!local_follower_) execution_validator_->body(
      MeasuredBodyPose{position,orientation,source_time.seconds(),msg->header.frame_id},
      world_twist,source_time.nanoseconds());

    have_odom_ = true;
    finishOwnerBodyUpdate();
  }

  void SCANReplanFSM::finishOwnerBodyUpdate()
  {
    // Both owner ingress paths have already accepted ONE newer original
    // acquisition. Run the same pending/UI work without writing it back to
    // the independent inbox or granting a second source freshness lease.
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }
    if (pending_tracking_progress_) {
      const auto pending=pending_tracking_progress_;
      pending_tracking_progress_.reset();
      consumeTrackingProgress(pending);
    }
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
    const auto support=execution_validator_?
        execution_validator_->displayPlanningSupport(executionVersion(reference_metadata_)):nullptr;
    StairAttitudeScope geometry_scope(*planner_manager_->grid_map_,support,
        MeasuredBodyPose{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_});
    const auto shape=planner_manager_->grid_map_->queryBodyEnvelope(odom_pos_,true);
    try {validateBodyEnvelope(shape);}catch(const std::invalid_argument&){return;}
    const double yaw=getOdomYaw();
    Eigen::Vector2d pitch_roll=Eigen::Vector2d::Zero();
    if(shape.model==CollisionModel::OrientedBox&&support&&support->isStair()&&
        support->source.frame_id==self_inflation_frame_id_) {
      const double norm=odom_orient_.norm();
      if(!odom_orient_.coeffs().allFinite()||!std::isfinite(norm)||norm<=1e-9)return;
      const Eigen::Vector3d up=odom_orient_.normalized()*Eigen::Vector3d::UnitZ();
      if(up.z()<=0.)return;
      // At the exact measured body position StairAttitudeQuery's weight is
      // one. Decode that same acquired up vector without another terrain fit.
      const double nx=std::cos(yaw)*up.x()+std::sin(yaw)*up.y();
      const double ny=-std::sin(yaw)*up.x()+std::cos(yaw)*up.y();
      pitch_roll=Eigen::Vector2d(std::atan2(nx,up.z()),std::atan2(-ny,std::hypot(nx,up.z())));
    }
    // Pair shape and effective attitude from ONE original body acquisition. The map
    // writer's independent callback may already have received another sample.
    // This display payload cannot alter any collision query or certificate.
    if(grid_map_writer_)grid_map_writer_->setVisualizationBodyEnvelope(
        shape,yaw,pitch_roll.x(),pitch_roll.y(),last_odom_time_.nanoseconds(),self_inflation_frame_id_);
    const double radius=shape.radius;
    const double height=shape.below+shape.above;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = shape.model==CollisionModel::OrientedBox ?
        visualization_msgs::msg::Marker::CUBE:visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = shape.model==CollisionModel::OrientedBox ? shape.length:2.0*radius;
    marker.scale.y = shape.model==CollisionModel::OrientedBox ? shape.width:2.0*radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (shape.above-shape.below);

    if(shape.model==CollisionModel::OrientedBox) {
      const Eigen::Quaterniond orientation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(pitch_roll.x(),Eigen::Vector3d::UnitY())*
          Eigen::AngleAxisd(pitch_roll.y(),Eigen::Vector3d::UnitX());
      center=odom_pos_+orientation*Eigen::Vector3d(0.,0.,.5*(shape.above-shape.below));
      marker.id=0;
      marker.pose.orientation.x=orientation.x();marker.pose.orientation.y=orientation.y();
      marker.pose.orientation.z=orientation.z();marker.pose.orientation.w=orientation.w();
      marker.pose.position.x=center.x();marker.pose.position.y=center.y();marker.pose.position.z=center.z();
      self_inflation_pub_->publish(marker);
      // Retire the second legacy marker when replacing an earlier display.
      marker.id=1;marker.action=visualization_msgs::msg::Marker::DELETE;
      self_inflation_pub_->publish(marker);
      return;
    }

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + shape.offset * heading;
    Eigen::Vector3d rear = center - shape.offset * heading;

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
    // Lifecycle work needs no collision map. In particular a cancelled or
    // expired ready result cannot retain the single worker slot behind a
    // borrowed map. Successful values wait in one bounded slot for adoption.
    collectSolveCompletion();
    OwnerMapScope scope(*this);
    if(!scope) {
      // The independent collision/control readers continue. A busy snapshot
      // cannot clear the incumbent, adopt a completion or lose an input.
      publishExecutionLedgerDebug();return;
    }
    drainOwnerInputs();
    execFSMOwnerCallback();
  }

  void SCANReplanFSM::execFSMOwnerCallback()
  {
    refreshPlanningBodyFromInbox();
    processDeferredRollingProposal();
    if(execution_validator_) {
      const auto blocked=execution_validator_->consumeBlockedEntry();
      const auto collision=execution_validator_->consumeCommittedCollisionAt(node_->now().nanoseconds());
      const bool current_collision=collision&&have_target_&&
          sameExecutionTask(collision->version,executionVersion(reference_metadata_));
      if((blocked&&have_target_&&blocked->version==executionVersion(reference_metadata_))||current_collision) {
        // Strictly matched native command proof, not an arbitrary "blocked"
        // status. Retire only the pending candidate, never the active curve or
        // global task. The shared solve budget/rate limit still applies.
        const auto identity=current_collision?
          std::make_pair(collision->version,collision->trajectory_id):
          std::make_pair(blocked->version,blocked->trajectory_id);
        const bool same_replacement=collision_replan_incumbent_&&
          *collision_replan_incumbent_==identity&&
          (blocked_entry_replan_pending_||solveWorkPending()||pending_snapshot_request_.pending());
        // A genuine occupied -> clear -> occupied edge can arrive while the
        // same replacement is already solving. Keep the latest negative
        // execution verdict, but do not repeatedly abort that one worker.
        // Its final native adoption/proof still uses the latest environment.
        if(!same_replacement) {
          cancelSolveWorker();execution_validator_->cancelPending();
          force_visible_side_target_=blocked_entry_replan_pending_=true;
          blocked_entry_position_=odom_pos_;
        }
        collision_replan_incumbent_=identity;
        if(current_collision)RCLCPP_WARN(node_->get_logger(),
          "Confirmed committed curve occupied; requesting bounded local side corridor curve=%ld map_revision=%lu proof_sequence=%lu coalesced=%d; task and global route unchanged",
          collision->trajectory_id,collision->map_snapshot_revision,collision->sequence,same_replacement);
        else RCLCPP_WARN(node_->get_logger(),"Confirmed command entry blocked; requesting bounded local side corridor for curve %ld",blocked->trajectory_id);
      }
    }
    if(blocked_entry_replan_pending_) {
      // A confirmed collision changes the desired planning state immediately.
      // Only actual solver submission is rate-limited below; a periodic clock
      // or failed-attempt cooldown is not another collision response delay.
      // The cancelled worker still has to retire before a replacement starts.
      blocked_entry_replan_pending_=false;
      last_attempt_failure_phase_="blocked_executable_entry";
      changeFSMExecState(GEN_NEW_TRAJ,"COMMAND_ENTRY_BLOCKED");
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
    // A rolling reference's end_pt_ is only its finite search-window tail.
    // Reaching it must not retire the task's planner while its committed
    // original goal is still farther along the route (or on another floor).
    // The Native committed job, not a pending proposal, owns that goal.
    std::optional<Eigen::Vector3d> measured_task_goal=end_pt_;
    if(local_follower_)measured_task_goal=execution_validator_?
      execution_validator_->committedTaskGoal(executionVersion(reference_metadata_)):
      std::optional<Eigen::Vector3d>{};
    if (reference_path_guidance_ && have_target_ && have_odom_ &&
        (exec_state_==GEN_NEW_TRAJ || exec_state_==REPLAN_TRAJ ||
         exec_state_==EXEC_TRAJ || exec_state_==WAIT_ENVIRONMENT) &&
        measured_task_goal&&measuredReferenceGoalReached(odom_pos_, *measured_task_goal,
            reference_goal_xy_tolerance_, reference_goal_z_tolerance_)&&
        (!local_follower_||(execution_validator_&&execution_validator_->measuredGoalSettled(
            executionVersion(reference_metadata_),node_->now().nanoseconds())))) {
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
       formalReseedSubmissionDue(std::chrono::steady_clock::now(),last_solve_submit_,solveWorkPending(),min_replan_interval_)) {
      const auto reseed=execution_validator_->consumeFormalReseedAt(node_->now().nanoseconds(),
          planner_manager_->grid_map_->getResolution());
      if(reseed&&sameExecutionTask(reseed->version,executionVersion(reference_metadata_))&&
         (reseed->terminal||reseed->actual_body||reseed->version==executionVersion(reference_metadata_))) {
        // A stale join boundary is not an obstacle and does not justify a side
        // corridor or changing the task. Keep writer-applied geometry and its
        // real safety reader alive; only reseed the next bounded local solve.
        if(reseed->actual_body)actual_body_seed_request_=reseed->version;
        last_attempt_failure_phase_=reseed->reason;
        changeFSMExecState(GEN_NEW_TRAJ,reseed->terminal?"MEASURED_LOCAL_ENDPOINT":
          reseed->actual_body?"MEASURED_BODY_RESEED":"MEASURED_ENTRY_RESEED");
        RCLCPP_INFO(node_->get_logger(),"Formal local reseed curve=%ld reason=%s original_body_source_ns=%ld",
          reseed->trajectory_id,reseed->reason.c_str(),rclcpp::Time(reseed->body_source_stamp).nanoseconds());
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
        if(nativePlanningAttemptDeferred(solveWorkPending(),pending_snapshot_request_.pending(),
            last_attempt_failure_phase_))break;
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
        if(nativePlanningAttemptDeferred(solveWorkPending(),pending_snapshot_request_.pending(),
            last_attempt_failure_phase_))break;
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
      // Give the one queued tail (or the fresh predecessor retry after its
      // negative receipt) priority over starting another OLD-window solve.
      // The active finite curve/proofs continue unchanged; collision and
      // actual-entry/terminal reseeds above bypass this scheduling wait.
      if(!force_visible_side_target_&&!blocked_entry_replan_pending_&&
         (deferred_rolling_proposal_||std::chrono::steady_clock::now()<rolling_retry_priority_until_))
        break;
      // Both ordinary triggers (periodic AND executed-distance progress) must
      // give an immutable future splice its measured entry opportunity. A
      // periodic-only check lets the next non-periodic tick replace the same
      // tail, continually moving its seam ahead of the real follower.
      // Collision, cancellation, input replacement and actual entry/terminal
      // reseeds have already been handled before this nominal scheduling.
      if(keepPendingSpliceForOrdinaryReplan()) {
        if(periodicReplanDue(node_->now().seconds(),last_replan_time_.seconds(),max_replan_interval_))
          last_replan_time_=node_->now();
        break;
      }
      // Periodic solving advances a moving local horizon. Exact fresh applied
      // geometry near its beginning may be retained while the real body is
      // stationary; source and native collision readers remain independent.
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
      LocalTrajData committed_info;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      if (reference_path_guidance_) {
        if (execution_validator_&&!planning_only_) {
          // local_data_ can already contain a candidate that the follower has
          // not adopted. Only the immutable committed curve and its exact real
          // progress may determine the running horizon/replan trigger.
          const auto committed=execution_validator_->committedHorizonAt(
            executionVersion(reference_metadata_),time_now.nanoseconds());
          if(!committed)return;
          committed_info.position_traj_=committed->curve;
          committed_info.duration_=committed->duration;
          committed_info.start_pos_=committed->curve.evaluateDeBoorT(0.);
          committed_info.traj_id_=committed->trajectory_id;
          info=&committed_info;t_cur=committed->measured_time;
        } else if (planning_only_) {
          const auto measured=planningOnlyCommittedProgress();
          if(!measured.connected()) {
            tryReplanPlanningOnlyConnection(measured);
            return;
          }
          t_cur=measured.curve_time;
        } else if (!execution_validator_ && go2_execution_frozen_ && planner_manager_->hasPreviewHeadingContract())
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

      if(local_follower_&&execution_validator_&&reference_path_guidance_) {
        auto velocity=info->position_traj_.getDerivative();
        auto acceleration=velocity.getDerivative();
        const Eigen::Vector3d planned_velocity=velocity.evaluateDeBoorT(t_cur);
        const double speed=std::max(odom_vel_.head<2>().norm(),planned_velocity.head<2>().norm());
        // Planned acceleration is a conservative scheduling input, not a
        // claim of measured acceleration or a replacement control limit.
        const double positive_a=planned_velocity.norm()>1e-8?
          std::max(0.,acceleration.evaluateDeBoorT(t_cur).dot(planned_velocity.normalized())):0.;
        const double reserve=continuationPlanningDistance(speed,positive_a,
          planner_manager_->pp_.max_acc_,continuation_max_jerk_);
        const auto goal=execution_validator_->committedTaskGoal(executionVersion(reference_metadata_));
        const Eigen::Vector3d tail=info->position_traj_.evaluateDeBoorT(info->duration_);
        if(goal&&(tail-*goal).norm()>.05&&(tail-pos).norm()<=reserve) {
          // Chord sum is only an early-work trigger; it never certifies free
          // space. A fixed bound avoids growing callback cost with duration.
          Eigen::Vector3d previous=pos;double remaining=0.;
          for(int i=1;i<=128;++i) {
            const auto p=info->position_traj_.evaluateDeBoorT(t_cur+(info->duration_-t_cur)*i/128.);
            remaining+=(p-previous).norm();previous=p;
          }
          if(remaining<=reserve) {
            changeFSMExecState(REPLAN_TRAJ,"CONTINUATION_RESERVE");return;
          }
        }
      }

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

    const auto display_now=std::chrono::steady_clock::now();
    if(last_data_display_publish_==std::chrono::steady_clock::time_point{}||
       display_now-last_data_display_publish_>=std::chrono::milliseconds(50)) {
      data_disp_.header.stamp = node_->now();
      data_disp_pub_->publish(data_disp_);last_data_display_publish_=display_now;
    }
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
    if (nativePlanningAttemptDeferred(solveWorkPending(),pending_snapshot_request_.pending(),
        last_attempt_failure_phase_)) return;
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
                nativeResourceWait(reason) ? "bounded snapshot retry" :
                nativeNumericalReplanFailure(reason) ? "fresh planning evidence retry" : "changed environment", reason.c_str());
    changeFSMExecState(WAIT_ENVIRONMENT, "BOUNDED_FAILURE");
  }

  bool SCANReplanFSM::failedAttemptRetryReady(double monotonic_now)
  {
    if (!std::isfinite(monotonic_now)||monotonic_now<failed_replan_monotonic_) return false;
    if(local_follower_&&!planning_only_&&nativeLatestStateRetry(last_attempt_failure_phase_)) {
      const auto map=planner_manager_->grid_map_;
      const double age=(node_->now()-last_odom_time_).seconds();
      const bool fresh=have_odom_&&age>=0.&&age<=odom_timeout_&&
          map->integratedCloudFreshAt(node_->now().nanoseconds());
      const bool changed=last_odom_time_.nanoseconds()>failed_body_source_ns_||
          map->latestCloudStamp()>failed_map_stamp_||
          map->occupancyRevision()!=failed_environment_revision_;
      // One worker / latest input / existing submission cadence. No retry of
      // the same rejected sample, no artificial 200 ms after normal movement,
      // and no busy retry of a stale map merely because odometry is streaming.
      return fresh&&changed&&monotonic_now-failed_replan_monotonic_>=min_replan_interval_;
    }
    // An exclusive reader releasing the third slot need not change a voxel or
    // body pose. Re-attempt resources after the configured cooldown, then acquire through the
    // same arbiter; contention never bypasses ownership. Every new attempt gets
    // one ORIGINAL 400 ms deadline in callReboundReplan, not a continuation or
    // extension of the failed request. Existing actual-solve cooldown remains.
    const bool resource=nativeResourceWait(last_attempt_failure_phase_);
    const double cooldown=std::max(min_replan_interval_,failed_replan_cooldown_);
    if (monotonic_now-failed_replan_monotonic_<cooldown) return false;
    // A bounded preview connector miss is not an occupied-map verdict. The
    // next solve must use the current acquired body, even if it then stops and
    // the saturated map stays unchanged. Keep the SAME existing cooldown.
    const bool measured_reseed=planning_only_&&last_attempt_failure_phase_==
        candidateAcceptanceReason(CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed);
    return resource || measured_reseed || environmentChangedSinceFailure();
  }

  bool SCANReplanFSM::environmentChangedSinceFailure()
  {
    const auto map=planner_manager_->grid_map_;
    const auto& reason=last_attempt_failure_phase_;
    // A geometrically unchanged saturated map can acquire fresh FREE evidence.
    // Recheck/unknown failures must not wait forever for a changed occupancy
    // bit. Both real sensor watermarks must advance; receipt/timer/republication
    // alone cannot recover the attempt, and WAIT_ENVIRONMENT still enforces the
    // configured solve cooldown plus the one 400 ms shared solve budget.
    const bool evidence_dependent=nativeNumericalReplanFailure(reason)||
        reason=="waiting_sensor_map"||reason=="waiting_recheck"||
        reason=="waiting_observed_space"||reason=="failed_final_collision"||reason=="failed_splice_prefix_collision"||
        reason=="candidate_latest_geometry_rejected"||
        reason=="failed_current_validation"||reason=="blocked_executable_entry"||
        reason==candidateAcceptanceReason(CandidateAcceptanceOutcome::PreviewSourceExpired)||
        reason==candidateAcceptanceReason(CandidateAcceptanceOutcome::PreviewSourceChanged)||
        reason==candidateAcceptanceReason(CandidateAcceptanceOutcome::PreviewGeometryBlocked)||
        reason=="waiting_observed_side_target"||reason=="waiting_side_target"||reason=="visible_side_target_budget_exhausted";
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
    const bool body_recovered=new_body && (reason=="waiting_body_pose" || nativeConnectionWait(reason));
    const bool dynamics_recovered=new_body && last_attempt_failure_phase_=="failed_dynamics" &&
        failedDynamicsBoundaryChanged(failed_body_velocity_, odom_vel_, planner_manager_->pp_.max_vel_);
    const bool attitude_changed=new_body &&
        failedOrientationBoundaryChanged(failed_body_orientation_, odom_orient_);
    return new_supported_map || body_recovered || dynamics_recovered || attitude_changed ||
        map->occupancyRevision()!=failed_environment_revision_ ||
        (odom_pos_-failed_body_position_).norm()>=failed_replan_body_distance_;
  }

  bool SCANReplanFSM::keepPendingSpliceForOrdinaryReplan()
  {
    if(!execution_validator_||!local_follower_||!reference_path_guidance_||planning_only_||
       !have_target_||!have_odom_||blocked_entry_replan_pending_||force_visible_side_target_||
       solveWorkPending()||pending_snapshot_request_.pending())return false;
    return execution_validator_->pendingSpliceDispositionAt(executionVersion(reference_metadata_),
      node_->now().nanoseconds(),planner_manager_->grid_map_->getResolution()).keep;
  }

  bool SCANReplanFSM::keepPreviewIncumbentForPeriodicReplan()
  {
    if(planning_only_) {
      const auto measured_time=planningOnlyCommittedCurveTime();
      const auto& curve=planner_manager_->local_data_;
      const double advance=std::min(1.,planning_horizon_*.25);
      // Actual body speed and XYZ projection, never a tracker heartbeat or a
      // wall-clock spline endpoint. Only scheduling time may be postponed.
      return measured_time&&odom_vel_.allFinite()&&odom_vel_.squaredNorm()<.0001&&
        std::isfinite(advance)&&advance>0.&&curve.duration_>0.&&
        (odom_pos_-curve.start_pos_).norm()<advance&&*measured_time<curve.duration_*.25;
    }
    if(execution_validator_) {
      // Only postpone a nominal PERIODIC solve. Reference ingress, native
      // occupied events and measured reseeds are processed before this branch;
      // a replacement acquisition/worker must never be hidden by idle holding.
      if(!reference_path_guidance_||!have_target_||!have_odom_||
         blocked_entry_replan_pending_||force_visible_side_target_||
         solveWorkPending()||pending_snapshot_request_.pending())return false;
      return execution_validator_->keepCommittedIdleHorizonAt(
        executionVersion(reference_metadata_),node_->now().nanoseconds(),odom_timeout_,
        std::min(1.,planning_horizon_*.25));
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

  bool SCANReplanFSM::refreshPlanningBodyFromInbox()
  {
    if(!local_follower_||!execution_validator_)return false;
    const auto sample=execution_validator_->latestPlanningBodyAt(
      executionVersion(reference_metadata_),node_->now().nanoseconds(),odom_timeout_);
    if(!sample||sample->source_ns<=last_odom_time_.nanoseconds())return false;
    // Only this owner mutates FSM/manager state. Copy one paired ORIGINAL
    // acquisition from the independent inbox, never its callback/receipt time.
    // This avoids waiting behind map integration's queued odometry callbacks.
    odom_pos_=sample->pose.position;odom_orient_=sample->pose.orientation;
    const auto& v=sample->twist->linear;odom_vel_={v.x,v.y,v.z};
    last_odom_time_=rclcpp::Time(sample->source_ns,node_->get_clock()->get_clock_type());have_odom_=true;
    odom_source_history_.push_back({sample->source_ns,odom_pos_,sample->pose.frame});
    while(odom_source_history_.size()>64||(!odom_source_history_.empty()&&
        sample->source_ns-odom_source_history_.front().source_ns>1000000000LL))
      odom_source_history_.pop_front();
    planner_manager_->setMeasuredBodyPose(sample->pose,self_inflation_frame_id_,odom_timeout_);
    planner_manager_->setMeasuredBodyVelocity(odom_vel_);
    finishOwnerBodyUpdate();
    return true;
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
    refreshPlanningBodyFromInbox();
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    planner_manager_->setPlannedSplice({});
    if (planning_only_) return; // No controller derivatives or frozen-topic dependency.
    if (have_new_target_&&!local_follower_) return;

    if(execution_validator_) {
      if(actual_body_seed_request_&&!sameExecutionTask(*actual_body_seed_request_,
          executionVersion(reference_metadata_)))actual_body_seed_request_.reset();
      // The formal rolling follower always solves from the acquired body.
      // An incumbent is continuity/rollback geometry, never the next physical
      // start boundary. In particular do not substitute its future P/V/A and
      // then require a robot which left that curve to return to its seam.
      if(const auto boundary=execution_validator_->committedBoundary(
          odom_pos_,odom_vel_,last_odom_time_.nanoseconds(),node_->now().seconds())) {
        start_vel_=boundary->velocity;start_acc_=boundary->acceleration;
      }
      return;
    }

    if (go2_execution_frozen_) return;

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
    if (!msg || planning_only_) return;
    if(execution_validator_)execution_validator_->progress(*msg);
    consumeTrackingProgress(msg);
  }

  std::optional<Eigen::Vector3d> SCANReplanFSM::odometryPositionAtSource(
      std::int64_t source_ns,const std::string &frame) const
  {
    const auto next=std::lower_bound(odom_source_history_.begin(),odom_source_history_.end(),
      source_ns,[](const OdomSourceSample& sample,std::int64_t stamp){return sample.source_ns<stamp;});
    if (next==odom_source_history_.end() || next->frame!=frame) return {};
    if (next->source_ns==source_ns) return next->position;
    if (next==odom_source_history_.begin()) return {};
    const auto previous=std::prev(next);
    const auto gap=next->source_ns-previous->source_ns;
    // Only interpolate between nearby, actually acquired samples in the same
    // frame. A missing bracket is waiting evidence, not an invented pose.
    if (previous->frame!=frame || gap<=0 || gap>100000000LL) return {};
    const double alpha=static_cast<double>(source_ns-previous->source_ns)/static_cast<double>(gap);
    return (1.-alpha)*previous->position+alpha*next->position;
  }

  void SCANReplanFSM::consumeTrackingProgress(
      const d1max_planning_interfaces::msg::TrackingProgress::ConstSharedPtr &msg)
  {
    const auto &identity=reference_metadata_;
    const double age=(node_->now()-rclcpp::Time(msg->header.stamp)).seconds();
    if (!msg->valid && msg->session_id==navigation_session_id_ &&
        msg->generation==reference_generation_ && msg->trajectory_id==accepted_curve_id_) {
      tracking_progress_.reset();
      pending_tracking_progress_.reset();
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
    if (!measured.allFinite() || msg->curve_time>curve.duration_) return;
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
    const auto source_ns=rclcpp::Time(msg->header.stamp).nanoseconds();
    const auto paired=odometryPositionAtSource(source_ns,msg->header.frame_id);
    if (!paired) {
      // Progress may reach this callback before its corresponding odometry.
      // One latest pending message is retried when real odometry arrives.
      if (!pending_tracking_progress_ || source_ns>
          rclcpp::Time(pending_tracking_progress_->header.stamp).nanoseconds())
        pending_tracking_progress_=msg;
      return;
    }
    if ((measured-*paired).norm()>planner_manager_->grid_map_->getResolution()*.25) return;
    if (pending_tracking_progress_ && rclcpp::Time(pending_tracking_progress_->header.stamp)<=
        rclcpp::Time(msg->header.stamp)) pending_tracking_progress_.reset();
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
      // This callback rechecks SCAN trajectory geometry, not an SDK command.
      // Use the same explicit policy as target selection/search/optimization;
      // the independent command validator owns observed-free braking evidence.
      // Otherwise the compatibility path can accept an official candidate and
      // immediately veto it merely because raw voxels remain unobserved.
      if (map->getPlanningInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
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
    if (solveWorkPending()) return false;
    const auto submitted=std::chrono::steady_clock::now();
    if(pending_snapshot_request_.pending()&&!snapshotAcquisitionCurrent())return false;
    if ((solve_worker_||execution_validator_)&&
        !formalReseedSubmissionDue(submitted,last_solve_submit_,false,min_replan_interval_)) {
      last_attempt_failure_phase_="waiting_solve_budget";
      return false;
    }
    if(local_follower_)setStartStateFromOdomOrCurrentTraj();
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
          planner_manager_->grid_map_->planningRequiresObservedFree() &&
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
        const auto support=execution_validator_?
            execution_validator_->planningSupport(executionVersion(reference_metadata_)):nullptr;
        StairAttitudeScope diagnostic_attitude(*planner_manager_->grid_map_,support,
            MeasuredBodyPose{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_});
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
    const double predecessor_measured_time=planning_only_ ?
        planningOnlyCommittedCurveTime().value_or(planner_manager_->previewCurveProgressTime()):
        (tracking_progress_ && tracking_progress_->valid &&
        tracking_progress_->trajectory_id==preceding_id) ? tracking_progress_->curve_time :
        planner_manager_->previewCurveProgressTime();
    if (reference_path_guidance_ && require_tagged_reference_ && preceding_id > 0 &&
        accepted_curve_generation_ == preceding_generation &&
        planner_manager_->local_data_.traj_id_ == preceding_id)
      predecessor.emplace(planner_manager_->local_data_.position_traj_);

    // Both asynchronous and explicitly synchronous solves carry the same
    // semantic segment boundary into their distinct preview/motion admission.
    const auto &meta=reference_metadata_;
    const bool single_segment=meta.schema_version!=2 ||
        (!meta.point_segment_ids.empty() && std::all_of(
            meta.point_segment_ids.begin(),meta.point_segment_ids.end(),
            [&](const std::string &id){return id==meta.segment_id;}));
    // With one fixed actuator mode, a source-labelled stair/landing window
    // has no mode-switch seam. Native still checks its full literal support
    // and collision geometry; moving along it is not crossing an unconfirmed
    // locomotion transaction. Legacy/segment_modes keeps its original rule.
    planner_manager_->setMeasuredJoinSingleSegment(single_segment||
      (fixed_stair_reference_continuity_&&continuousStairReferenceWindow(meta)));
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
      // Target selection (especially a side search) and snapshot acquisition
      // may have overlapped new body samples. Rebuild target/reference ONCE
      // with that paired actual P/V before copying the worker inputs. Reuse
      // the same third-slot lease and original deadline; do not loop forever
      // chasing a 50 Hz state stream or allocate another solve budget.
      if(local_follower_) {
        const auto selected_body_source=last_odom_time_.nanoseconds();
        setStartStateFromOdomOrCurrentTraj();
        if(last_odom_time_.nanoseconds()>selected_body_source&&
           !getLocalTarget(request_budget,&snapshot)) {
          last_attempt_failure_phase_=request_budget->allowed()?
              local_target_query_debug_.reason:request_budget->reason();
          if(!request_budget->allowed())cancelSnapshotAcquisition();
          return false;
        }
      }
      // A slot miss did not start a solve. Do not add a submission cooldown to the
      // pending reader, otherwise a fair admission grant could strand solver
      // intent until its lease expires. Starts share one configured rate and worker.
      last_solve_submit_=std::chrono::steady_clock::now();
      solve_snapshot_bytes_=snapshot->collisionSnapshotBytes();
      // Bind the exact accepted reference version, including source-labelled
      // stair/landing regions, before preparing this independent map reader.
      const auto support=execution_validator_?
          execution_validator_->planningSupport(executionVersion(reference_metadata_)):nullptr;
      // One immutable support supplies BOTH body attitude and source height
      // throughout preparation/search/optimization. The exclusive snapshot
      // must not outlive this binding or retain it when returned to the pool.
      auto worker_attitude=std::make_unique<StairAttitudeScope>(*snapshot,support,
          MeasuredBodyPose{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_});
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
      const auto solve_body_source_ns=last_odom_time_.nanoseconds();
      const bool poly=have_new_target_ || flag_use_poly_init;
      have_new_target_=false;
      try {
        solve_future_=std::async(std::launch::async,
          [this,request_budget,submitted,start,velocity,acceleration,target,target_velocity,poly,flag_randomPolyTraj,solve_body_source_ns,
           worker_attitude=std::move(worker_attitude)]() mutable {
        SolveScopeExit release_snapshot([this,&worker_attitude]() noexcept {
          worker_attitude.reset();
          solve_worker_->releaseSolveSnapshot();
        });
        WorkerResult result;
        result.body_source_ns=solve_body_source_ns;
        const auto begun=std::chrono::steady_clock::now();
        result.queue_ms=std::chrono::duration<double,std::milli>(begun-submitted).count();
        try {
          if (solve_worker_->reboundReplan(start,velocity,acceleration,target,target_velocity,
                                          poly,flag_randomPolyTraj,request_budget))
            result.candidate=solve_worker_->takeSolvedCandidate();
          result.failure=solve_worker_->lastFailurePhase();
        } catch (const std::bad_alloc &) {
          throw; // Resource exhaustion is not a recoverable/safe planning result.
        } catch (...) {
          result.candidate.reset();
          result.failure="solve_exception";
        }
        result.finished_at=std::chrono::steady_clock::now();
        result.solve_ms=std::chrono::duration<double,std::milli>(result.finished_at-begun).count();
        return result;
      });
      actual_body_seed_request_.reset();
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
    actual_body_seed_request_.reset();
    // getLocalTarget's scope has already returned. Keep the same immutable
    // reference/body binding across synchronous search, optimization and
    // adoption, just as the worker and owner-adoption paths do separately.
    const auto synchronous_support=execution_validator_?
      execution_validator_->planningSupport(executionVersion(reference_metadata_)):nullptr;
    StairAttitudeScope synchronous_attitude(*planner_manager_->grid_map_,synchronous_support,
      MeasuredBodyPose{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_});
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
    if (plan_success && !planner_manager_->commitPreviewCandidate(request_budget)) {
      plan_success=false;
      last_attempt_failure_phase_=planning_only_?planner_manager_->lastFailurePhase():"waiting_sensor_map";
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
    if(completed_solve_&&completed_solve_->budget)completed_solve_->budget->cancel();
    completed_solve_.reset();retired_solve_failure_.reset();
    // A running worker still owns its captured inputs until it actually exits.
    if(!solve_future_.valid())solve_predecessor_.reset();
  }

  bool SCANReplanFSM::solveWorkPending() const
  {
    return solve_future_.valid()||completed_solve_.has_value()||retired_solve_failure_.has_value();
  }

  void SCANReplanFSM::collectSolveCompletion()
  {
    if(solve_future_.valid()&&
       solve_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      WorkerResult result;
      try {
        result=solve_future_.get(); // The worker already released its map/provider lease.
      } catch(const std::bad_alloc&) {
        if(active_solve_budget_)active_solve_budget_->cancel();
        active_solve_budget_.reset();solve_predecessor_.reset();
        throw; // Keep the existing fail-stop resource exhaustion contract.
      } catch(...) {
        result.failure="solve_exception";
      }
      const auto harvest=std::chrono::steady_clock::now();
      const double owner_delay_ms=result.finished_at==std::chrono::steady_clock::time_point{}?0.:
          std::chrono::duration<double,std::milli>(harvest-result.finished_at).count();
      RCLCPP_INFO(node_->get_logger(),
          "NativeSolve generation=%lu queue_ms=%.3f snapshot_ms=%.3f solve_ms=%.3f snapshot_bytes=%zu deadline=%s owner_delay_ms=%.3f body_source_ns=%ld body_age_at_harvest_ms=%.3f owner_snapshot_sequence=%lu owner_snapshot_revision=%lu owner_publication_misses=%lu",
          solve_generation_,result.queue_ms,snapshot_copy_ms_,result.solve_ms,
          solve_snapshot_bytes_,active_solve_budget_ ? active_solve_budget_->reason():"missing",
          owner_delay_ms,result.body_source_ns,result.body_source_ns>0?
            (node_->now().nanoseconds()-result.body_source_ns)*1e-6:NAN,
          owner_snapshot_timing_.publication_sequence,owner_snapshot_timing_.map_revision,
          owner_snapshot_timing_.publication_misses);
      completed_solve_=CompletedOwnerSolve<WorkerResult>{
        std::move(result),std::move(active_solve_budget_),solve_generation_};
    }
    if(retired_solve_failure_&&
       (retired_solve_failure_->generation!=reference_generation_||!have_target_))
      retired_solve_failure_.reset();
    if(!completed_solve_)return;
    const auto disposition=ownerSolveCompletionDisposition(completed_solve_->generation,
        reference_generation_,have_target_,completed_solve_->budget,
        completed_solve_->result.candidate.has_value());
    if(disposition==OwnerSolveCompletionDisposition::AwaitMap)return;
    if(disposition==OwnerSolveCompletionDisposition::Failed) {
      // Do not query a map, revalidate the incumbent, or publish a failure as a
      // motion verdict here. Only the later owner-map pass records the failed
      // environment and applies the unchanged retry policy, exactly once.
      planner_manager_->importAttemptDiagnostics(*solve_worker_);
      const auto& budget=completed_solve_->budget;
      retired_solve_failure_=RetiredOwnerSolveFailure{completed_solve_->generation,
        !budget->allowed()?budget->reason():completed_solve_->result.failure};
    }
    completed_solve_.reset();solve_predecessor_.reset();
  }

  void SCANReplanFSM::publishMapSnapshots()
  {
    if(!grid_map_writer_)return;
    if(map_validation_publication_pending_&&
       snapshot_pool_.publishValidation(*grid_map_writer_,node_->now().nanoseconds(),std::chrono::steady_clock::now())) {
      map_validation_publication_pending_=false;
      if(execution_validator_)execution_validator_->snapshotPublished();
    }
    // A successful head publication may have left the owner's other slot
    // borrowed. Finish that bounded delivery on the existing 10 ms writer
    // retry; identical completed sources/revision cause no additional copy.
    // A replica is the SAME completed source for the owner, not a new map.
    // The head publication already woke current-curve renewal. Waking it
    // again here adds a redundant full collision pass on the fusion path.
    snapshot_pool_.refreshOwnerReplica(*grid_map_writer_,node_->now().nanoseconds(),std::chrono::steady_clock::now());
    if(!execution_validator_) {
      map_solver_publication_pending_=false;return;
    }
    // A candidate may arrive after fusion found no admission work and cleared
    // its pending bit. Check demand on the existing bounded writer retry too;
    // it must deliver the last COMPLETE writer state, not wait for another
    // lidar frame while admission repeatedly borrows the solver's older input.
    const auto wanted=execution_validator_->trySlowSnapshotWanted();
    if(!wanted)return; // Busy ledger: keep the same bounded 10 ms retry.
    if(!*wanted) {map_solver_publication_pending_=false;return;}
    const auto publication=snapshot_pool_.refreshSolverPublication(*grid_map_writer_,
      node_->now().nanoseconds(),std::chrono::steady_clock::now());
    map_solver_publication_pending_=publication==CollisionSnapshotPool::SolverPublication::Deferred;
    if(publication==CollisionSnapshotPool::SolverPublication::Published) {
      execution_validator_->admissionSnapshotPublished();
    }
  }

  bool SCANReplanFSM::pollSolveWorker()
  {
    collectSolveCompletion();
    if(solve_future_.valid())return true;
    if(retired_solve_failure_) {
      last_attempt_failure_phase_=std::move(retired_solve_failure_->reason);
      retired_solve_failure_.reset();
      ++replan_fail_count_;publishAttemptDebug(solve_attempt_header_);
      waitForChangedEnvironment();return true;
    }
    if(!completed_solve_)return false;
    auto completion=std::move(*completed_solve_);completed_solve_.reset();
    const auto budget=std::move(completion.budget);
    auto result=std::move(completion.result);
    // Input replacement already revoked the old generation. No old diagnostics,
    // candidate or previous result may overwrite the replacement's state.
    if (completion.generation!=reference_generation_ || !have_target_ ||
        !budget || budget->stop()==SolveBudget::Stop::Cancelled) {
      solve_predecessor_.reset();
      return false;
    }
    planner_manager_->importAttemptDiagnostics(*solve_worker_);
    last_attempt_failure_phase_=result.failure;
    // The solver released its own map. Adoption queries the owner's newer
    // private reader, so it must bind the same source terrain explicitly;
    // otherwise a valid pitched stair candidate is rechecked as an upright
    // box here and never reaches the follower.
    refreshPlanningBodyFromInbox();
    const auto adoption_support=execution_validator_?
        execution_validator_->planningSupport(executionVersion(reference_metadata_)):nullptr;
    StairAttitudeScope adoption_attitude(*planner_manager_->grid_map_,adoption_support,
        MeasuredBodyPose{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_});
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
    // A short solve may still wait for the map owner to harvest it. Adoption
    // must observe the latest independently acquired body, not that owner's
    // last queued odometry callback. The candidate's original solve P/V/stamp
    // remain immutable; ordinary measured projection decides whether it joins.
    bool adopted=false;
    const bool acquired_body_fresh=have_odom_&&
        (node_->now()-last_odom_time_).seconds()>=-.02&&
        (node_->now()-last_odom_time_).seconds()<=odom_timeout_;
    const bool adoption_attempted=result.candidate&&acquired_body_fresh;
    if(local_follower_&&result.candidate&&result.candidate->splice) {
      auto splice=*result.candidate->splice;
      const auto current=execution_validator_->committedHorizonAtBody(
        executionVersion(reference_metadata_),node_->now().nanoseconds(),
        MeasuredBodyPose{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_},
        last_odom_time_.nanoseconds());
      if(current&&current->version==splice.wire.incumbent_version&&current->trajectory_id==splice.wire.incumbent_trajectory_id) {
        splice.measured_time=current->measured_time;splice.progress_source_ns=current->measured_source_ns;
        splice.committed_arc=current->committed_arc;
        planner_manager_->setPlannedSplice(std::move(splice));
      } else planner_manager_->setPlannedSplice({});
    }
    if (adoption_attempted)
      adopted=planner_manager_->adoptSolvedCandidate(std::move(*result.candidate),budget);
    if (!budget->allowed()) last_attempt_failure_phase_=budget->reason();
    else if (result.candidate && !adopted) last_attempt_failure_phase_=planning_only_?
        (adoption_attempted?planner_manager_->lastFailurePhase():
          candidateAcceptanceReason(CandidateAcceptanceOutcome::PreviewSourceExpired)):
        (adoption_attempted&&!planner_manager_->lastFailurePhase().empty()?
          planner_manager_->lastFailurePhase():"waiting_recheck");
    if(adoption_attempted&&planning_only_) {
      const auto& outcomes=planner_manager_->candidateAcceptanceDiagnostics();
      RCLCPP_INFO(node_->get_logger(),"PreviewCandidateAcceptance outcome=%s accepted=%lu reseed=%lu blocked=%lu source_changed=%lu original_curve_preserved=1 motion_join_evidence=0",
          candidateAcceptanceReason(outcomes.last),
          outcomes.counts[static_cast<std::size_t>(CandidateAcceptanceOutcome::PreviewGeometryAccepted)],
          outcomes.counts[static_cast<std::size_t>(CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed)],
          outcomes.counts[static_cast<std::size_t>(CandidateAcceptanceOutcome::PreviewGeometryBlocked)],
          outcomes.counts[static_cast<std::size_t>(CandidateAcceptanceOutcome::PreviewSourceChanged)]);
    }
    publishAttemptDebug(solve_attempt_header_);
    if (!adopted) {
      ++replan_fail_count_;
      if(planning_only_&&adoption_attempted&&budget->allowed()&&
         planner_manager_->candidateAcceptanceDiagnostics().last==CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed) {
        // Keep the task/reference and incumbent. REPLAN_TRAJ uses acquired
        // XYZ/velocity again; callReboundReplan retains the configured single-worker cadence.
        changeFSMExecState(REPLAN_TRAJ,"PREVIEW_CANDIDATE_MEASURED_RESEED");
        return true;
      }
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
    const auto source_support=execution_validator_?
        execution_validator_->planningSupport(executionVersion(reference_metadata_)):nullptr;
    if(reference_path_guidance_&&require_tagged_reference_&&
       !stairReferenceGeometryReady(reference_metadata_,executionVersion(reference_metadata_),
                                    source_support,self_inflation_frame_id_)) {
      // Do not turn asynchronous terrain delivery into an upright-box
      // collision verdict. Keep the incumbent and retry the pending reference
      // using its exact support when available, within the usual solve budget.
      local_target_query_debug_=ReferenceTargetResult{};
      local_target_query_debug_.reason=last_attempt_failure_phase_="waiting_reference_support";
      return false;
    }
    const MeasuredBodyPose attitude_body{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_};
    StairAttitudeScope attitude(*planner_manager_->grid_map_,source_support,attitude_body);
    const auto heading_policy=planner_manager_->configuredHeadingPolicy();
    double target_body_yaw=0.;
    if(heading_policy==BodyHeadingPolicy::HoldHeading) {
      const MeasuredBodyPose measured{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_};
      if(!measuredBodyYaw(measured,self_inflation_frame_id_,node_->now().seconds(),odom_timeout_,target_body_yaw)) {
        last_attempt_failure_phase_="waiting_measured_target_heading";return false;
      }
    }
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
      // Stair/platform A* joins the exact endpoint along a checked incident
      // edge. Its body need not point along the distant start-to-goal chord.
      // Keep the historical selector unchanged outside this stair scope;
      // route-tangent and complete candidate collision checks still apply.
      options.require_chord_heading=!planner_manager_->grid_map_->hasQuerySupportHeightProvider();
      if(local_follower_&&execution_validator_&&discrete_reference_.length()>0.&&
         discrete_progress_>=discrete_reference_.length()-1e-6) {
        const auto version=executionVersion(reference_metadata_);
        const auto goal=execution_validator_->committedTaskGoal(version);
        const auto support=execution_validator_->planningSupport(version);
        if(goal&&support&&support->valid&&support->source.frame_id==self_inflation_frame_id_) {
          const auto endpoint=discrete_reference_.sample(discrete_reference_.length());
          const auto actual_ground=support->nearest(start_pt_.x(),start_pt_.y(),
              start_pt_.z()-support->source.body_reference_height_m);
          const auto endpoint_ground=support->nearest(endpoint.x(),endpoint.y(),
              endpoint.z()-support->source.body_reference_height_m);
          bool supported=actual_ground&&endpoint_ground&&
            std::abs(start_pt_.z()-actual_ground->z-support->source.body_reference_height_m)<=reference_goal_z_tolerance_&&
            std::abs(endpoint.z()-endpoint_ground->z-support->source.body_reference_height_m)<=reference_goal_z_tolerance_;
          if(support->model()) {
            const auto actual_body=support->bodyPositionOnSupport(start_pt_);
            const auto endpoint_body=support->bodyPositionOnSupport(endpoint);
            supported=actual_body&&endpoint_body&&
              std::abs(start_pt_.z()-actual_body->z())<=reference_goal_z_tolerance_&&
              std::abs(endpoint.z()-endpoint_body->z())<=reference_goal_z_tolerance_;
          }
          options.final_endpoint_reseed=FinalEndpointReseed{*goal,reference_goal_xy_tolerance_,
            reference_goal_z_tolerance_,supported,execution_validator_->finalEndpointConnectionNeedsReseed(
              version,node_->now().nanoseconds()),NoMotionPreviewConnectionPolicy::max_connector_m};
        }
      }
      auto selection=selectReferenceTarget(discrete_reference_, start_pt_,
          discrete_progress_, planning_horizon_, options,
          [this,&budget,heading_policy,target_body_yaw](const Eigen::Vector3d &point, double yaw) {
            if (!solveAllowed(budget)) return -1;
            const auto map=planner_manager_->grid_map_;
            return map->getPlanningInflateOccupancy(point,
                heading_policy==BodyHeadingPolicy::HoldHeading?target_body_yaw:yaw);
          });
      std::vector<Eigen::Vector3d> visible_side_reference;
      const bool reference_exhausted=selection.reason=="waiting_reference_extension"||
          selection.reason=="waiting_goal_reached";
      if(force_visible_side_target_&&!reference_exhausted) {
        selection.valid=false;selection.reason="blocked_executable_entry";
      }
      if(!selection.valid&&!reference_exhausted&&execution_validator_&&solveAllowed(budget)) {
        const auto support=execution_validator_->planningSupport(executionVersion(reference_metadata_));
        double measured_yaw=0.;
        const MeasuredBodyPose measured{odom_pos_,odom_orient_,last_odom_time_.seconds(),self_inflation_frame_id_};
        if(support&&measuredBodyYaw(measured,self_inflation_frame_id_,node_->now().seconds(),odom_timeout_,measured_yaw)) {
          // A final measured-state refresh can rebuild selection using the
          // already reserved solver snapshot. Never compete with ourselves
          // for the exclusive third slot or restart the parent deadline.
          auto snapshot=reserved_snapshot?*reserved_snapshot:GridMap::Ptr{};
          if(!snapshot)snapshot=snapshot_pool_.acquireSolver(*planner_manager_->grid_map_,
              node_->now().nanoseconds(),budget->deadline());
          if(!snapshot){selection.reason="waiting_snapshot_slot";local_target_query_debug_=selection;return false;}
          // Waiting is part of the parent's ORIGINAL 400 ms, not the 80 ms
          // side computation quota. Allocate that sub-budget only after the
          // exclusive fair lease exists; it is capped by original remaining.
          const auto side_budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(80),
              []{return SolveBudget::Clock::now();},budget);
          StairAttitudeScope side_attitude(*snapshot,support,measured);
          auto visible=selectPlanningSideTarget(discrete_reference_,start_pt_,measured_yaw,discrete_progress_,
              planning_horizon_,*support,*snapshot,self_inflation_frame_id_,side_budget,1024,heading_policy);
          // A matched occupied command proof requests ONE bounded alternative,
          // not a permanent ban on the unchanged route. Consume only after the
          // real side search ran: resource acquisition above can still retry.
          // If a transient obstacle disappears while we are stationary, the
          // next changed-environment solve must again consider the normal
          // reference/A* entry. Independent command evidence still rejects a
          // persistent occupied or unknown sweep; this grants no motion lease.
          force_visible_side_target_=false;
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
      // slice() prepends a distinct projection only when actual_start differs
      // from it. A visible-side support path is already actual motion geometry,
      // so its first real segment must never be discarded as a projection.
      const bool has_start_projection_connector=visible_side_reference.empty()&&
          (local_debug_projection_-start_pt_).norm()>1e-6;
      local_debug_selected_reference_ = visible_side_reference.empty() ?
          discrete_reference_.slice(discrete_progress_, target_arc, start_pt_) : std::move(visible_side_reference);
      if (!selection.valid) {
        last_attempt_failure_phase_=selection.reason;
        planner_manager_->setLocalReference({});
        return false;
      }
      if (selection.reason!="reference_target_visible_side"&&target_arc < discrete_reference_.length() - 1e-4)
      {
        Eigen::Vector3d tangent = discrete_reference_.sample(std::min(target_arc + .05, discrete_reference_.length())) -
                                        discrete_reference_.sample(std::max(0.0, target_arc - .05));
        if(source_support&&source_support->source.frame_id==self_inflation_frame_id_) {
          const auto surface_tangent=source_support->tangentOnSurface(local_target_pt_,tangent);
          if(surface_tangent)tangent=*surface_tangent;
        }
        if (tangent.norm() > 1e-6) local_target_vel_ = tangent.normalized() * planner_manager_->pp_.max_vel_;
      }
      planner_manager_->setLocalReference(local_debug_selected_reference_,
        selection.reason=="reference_target_final_endpoint_reseed",has_start_projection_connector,source_support);
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
      return planner_manager_->grid_map_->getPlanningInflateOccupancy(pt,
          heading_policy==BodyHeadingPolicy::HoldHeading?target_body_yaw:estimateYawFromSegment(odom_pos_, pt));
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
