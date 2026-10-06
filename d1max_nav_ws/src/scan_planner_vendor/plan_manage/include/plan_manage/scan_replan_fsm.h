#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <optional>
#include <future>
#include <plan_env/collision_snapshot_pool.hpp>
#include <plan_env/pending_snapshot_request.hpp>
#include <plan_manage/execution_validator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <d1max_planning_interfaces/msg/reference_path.hpp>
#include <d1max_planning_interfaces/msg/tagged_bspline.hpp>
#include <d1max_planning_interfaces/msg/tracking_progress.hpp>
#include <d1max_planning_interfaces/msg/local_plan_debug.hpp>
#include <bspline_opt/reference_path.hpp>
#include <plan_manage/reference_target.hpp>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      WAIT_ENVIRONMENT
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    SCANPlannerManager::Ptr solve_worker_;
    CollisionSnapshotPool snapshot_pool_;
    std::unique_ptr<ExecutionValidator> execution_validator_;
    std::optional<ew::ReferenceProposal> execution_proposal_;
    rclcpp::TimerBase::SharedPtr validation_snapshot_timer_;
    rclcpp::Subscription<ew::ReferenceProposal>::SharedPtr execution_proposal_sub_;
    rclcpp::Subscription<ew::SupportReference>::SharedPtr execution_support_sub_;
    rclcpp::Subscription<ew::ExecutionPermit>::SharedPtr execution_permit_sub_;
    rclcpp::Subscription<ew::TrackingProgress>::SharedPtr execution_progress_sub_;
    rclcpp::Subscription<ew::MotionDemand>::SharedPtr execution_demand_sub_;
    rclcpp::Subscription<ew::MotionValidation>::SharedPtr execution_blocked_entry_sub_;
    rclcpp::Subscription<ew::TrajectoryAdmission>::SharedPtr execution_admission_sub_;
    rclcpp::Subscription<ew::ExecutionHandoffGrant>::SharedPtr execution_handoff_sub_;
    rclcpp::Subscription<ew::PreparedMotionDemand>::SharedPtr execution_prepared_sub_;
    rclcpp::Subscription<ew::ExecutionCommitAck>::SharedPtr execution_commit_ack_sub_;
    bool force_visible_side_target_{false},blocked_entry_replan_pending_{false};
    Eigen::Vector3d blocked_entry_position_{Eigen::Vector3d::Zero()};
    rclcpp::Publisher<ew::ReferenceReceipt>::SharedPtr execution_receipt_pub_;
    rclcpp::CallbackGroup::SharedPtr execution_control_group_;
    struct WorkerResult {
      std::optional<SCANPlannerManager::SolvedCandidate> candidate;
      std::string failure;
      double solve_ms{0.},queue_ms{0.};
    };
    std::future<WorkerResult> solve_future_;
    SolveBudget::Ptr active_solve_budget_;
    PendingSnapshotRequest pending_snapshot_request_;
    bool solve_worker_enabled_{true};
    std::uint64_t solve_generation_{0};
    std::chrono::steady_clock::time_point last_solve_submit_{};
    std_msgs::msg::Header solve_attempt_header_;
    std::optional<UniformBspline> solve_predecessor_;
    std::int64_t solve_predecessor_id_{0};
    double solve_predecessor_measured_time_{0.};
    double snapshot_copy_ms_{0.};
    d1max_planning_interfaces::msg::ReferencePath reference_metadata_;
    std::optional<d1max_planning_interfaces::msg::TrackingProgress> tracking_progress_;
    bool require_schema_v2_{false};
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    double reference_path_z_offset_{0.0};
    double reference_start_tolerance_{1.0};
    double odom_timeout_{0.5};
    bool strict_input_frames_{false};
    bool odom_twist_in_body_frame_{false};
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_cloud_time_, last_replan_time_;
    bool have_fresh_cloud_{false};
    double max_replan_interval_{0.0};
    double failed_replan_cooldown_{0.5}, failed_replan_body_distance_{0.15};
    double reference_goal_xy_tolerance_{.20}, reference_goal_z_tolerance_{.15};
    double reference_target_forward_margin_{1.0}, reference_target_backward_margin_{2.0};
    double reference_target_min_advance_{0.3};
    double reference_target_exit_margin_{0.2};
    double failed_replan_monotonic_{0.0};
    double failed_map_stamp_{0.0};
    std::uint64_t failed_environment_revision_{0};
    std::array<std::int64_t,2> failed_ray_stamps_{{0,0}};
    Eigen::Vector3d failed_body_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d failed_body_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond failed_body_orientation_{Eigen::Quaterniond::Identity()};
    std::int64_t failed_body_source_ns_{0};
    bool reference_path_guidance_{false};
    DiscreteReference discrete_reference_;
    double discrete_progress_{0.0};
    bool require_tagged_reference_{false};
    bool have_reference_generation_{false};
    std::string navigation_session_id_;
    uint64_t reference_generation_{0};
    std::string self_inflation_frame_id_;
    Eigen::Vector3d local_debug_projection_{Eigen::Vector3d::Zero()};
    double local_debug_target_arc_{0.0};
    std::vector<Eigen::Vector3d> local_debug_selected_reference_;
    ReferenceTargetResult local_target_query_debug_;
    std::chrono::steady_clock::time_point last_target_evidence_diagnostic_{};
    uint64_t last_local_debug_generation_{0};
    std::string last_local_debug_phase_;
    std::string last_attempt_failure_phase_;
    bool have_local_debug_state_{false};
    std::uint64_t accepted_curve_generation_{0};
    std::int64_t accepted_curve_id_{-1};
    std::uint64_t predecessor_id_{0};
    bool predecessor_safe_{false};
    builtin_interfaces::msg::Time predecessor_check_stamp_;
    std::optional<d1max_planning_interfaces::msg::LocalPlanDebug> accepted_preview_debug_;
    std::uint64_t accepted_curve_context_sequence_{0};
    enum class PreviewRecheck { Unavailable, Stale, Safe, Unsafe, Uncertified };

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr execution_body_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_health_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<d1max_planning_interfaces::msg::ReferencePath>::SharedPtr typed_path_sub_;
    rclcpp::Subscription<d1max_planning_interfaces::msg::TrackingProgress>::SharedPtr tracking_progress_sub_;
    rclcpp::Publisher<d1max_planning_interfaces::msg::TaggedBspline>::SharedPtr tagged_bspline_pub_;
    rclcpp::Publisher<d1max_planning_interfaces::msg::LocalPlanDebug>::SharedPtr local_plan_debug_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_attempt_debug_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool pollSolveWorker();
    void cancelSolveWorker();
    void cancelSnapshotAcquisition();
    bool snapshotAcquisitionCurrent();
    void publishCommittedTrajectory();
    void trackingProgressCallback(const d1max_planning_interfaces::msg::TrackingProgress::ConstSharedPtr &msg);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    bool getLocalTarget(const SolveBudget::Ptr &budget = {},GridMap::Ptr* reserved_snapshot=nullptr);
    void publishAcceptedLocalPlanDebug(std::uint64_t plan_id);
    bool publishExecutionLedgerDebug();
    PreviewRecheck revalidatePreviewIncumbent();
    void publishInvalidLocalPlanDebug(const std::string &phase, bool force = false);
    std_msgs::msg::Header localPlanDebugHeader();
    void publishAttemptDebug(const std_msgs::msg::Header &header, bool clear_only=false,
                             bool include_optimizer_diagnostics=true);
    void finishProcess();
    void waitForChangedEnvironment();
    bool failedAttemptRetryReady(double monotonic_now);
    bool environmentChangedSinceFailure();
    bool keepPreviewIncumbentForPeriodicReplan();
    bool shouldReplanUncertifiedPreview();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void typedPathCallback(const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr &msg);
    void publishTrajectory(const scan_planner_msgs::msg::Bspline &trajectory);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
      cancelSolveWorker();
      if (solve_future_.valid()) solve_future_.wait();
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
