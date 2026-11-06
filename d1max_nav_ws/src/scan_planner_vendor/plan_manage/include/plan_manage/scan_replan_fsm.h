#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <optional>
#include <future>
#include <deque>
#include <iterator>
#include <tuple>
#include <array>
#include <functional>
#include <plan_env/collision_snapshot_pool.hpp>
#include <plan_env/pending_snapshot_request.hpp>
#include <plan_manage/execution_validator.hpp>
#include <plan_manage/role_executors.hpp>
#include <plan_manage/owner_solve_completion.hpp>
#include <plan_manage/reference_receipt_history.hpp>
#include <plan_manage/rolling_reference.hpp>
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
    // The live GridMap has exactly one ROS writer group. Formal planning
    // borrows one exclusive immutable query view for each owner transaction;
    // no FSM callback reads live buffers while a ray batch is being fused.
    GridMap::Ptr grid_map_writer_;
    rclcpp::CallbackGroup::SharedPtr map_writer_group_;
    // Coalesced retry of an already completed fusion transaction. These are
    // owned by the writer callback group, never a queue of old map copies.
    bool map_validation_publication_pending_{false};
    bool map_solver_publication_pending_{false};
    rclcpp::TimerBase::SharedPtr map_publication_timer_;
    void publishMapSnapshots();
    unsigned owner_map_depth_{0};
    CollisionSnapshotPool::PublicationTiming owner_snapshot_timing_{};
    enum class OwnerInput : std::size_t { Odom, Progress, Proposal, TypedPath, Path, Goal, Count };
    std::array<std::function<void()>,static_cast<std::size_t>(OwnerInput::Count)> owner_inputs_;
    std::array<std::uint64_t,static_cast<std::size_t>(OwnerInput::Count)> owner_input_orders_{};
    class OwnerMapScope {
      SCANReplanFSM& owner_;
      GridMap::Ptr previous_;
      bool entered_{false},borrowed_{false};
    public:
      explicit OwnerMapScope(SCANReplanFSM& owner);
      ~OwnerMapScope();
      explicit operator bool()const{return entered_;}
      OwnerMapScope(const OwnerMapScope&)=delete;
      OwnerMapScope& operator=(const OwnerMapScope&)=delete;
    };
    void deferOwnerInput(OwnerInput input,std::uint64_t order,std::function<void()> callback);
    void dispatchOwnerInput(OwnerInput input,std::uint64_t order,std::function<void()> callback);
    void drainOwnerInputs();
    std::unique_ptr<ExecutionValidator> execution_validator_;
    std::optional<ew::ReferenceProposal> execution_proposal_;
    // Scheduling intent only. Keep through snapshot/rate waits; consume when
    // a solve actually starts. Never reuse an old task's acquired-body intent.
    std::optional<ew::ExecutionVersion> actual_body_seed_request_;
    // One original request, never a renewed source/deadline. A compatible
    // rolling tail may wait for a reserved splice; cancellations never wait.
    ew::ReferenceProposal::ConstSharedPtr deferred_rolling_proposal_;
    std::chrono::steady_clock::time_point rolling_retry_priority_until_{};
    std::string execution_transport_mode_;
    double continuation_max_jerk_{.7};
    ReferenceReceiptHistory<ew::ReferenceProposal,ew::ReferenceReceipt> execution_receipt_history_;
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
    rclcpp::Subscription<ew::LocalTrajectoryCommit>::SharedPtr execution_local_commit_sub_;
    rclcpp::Publisher<ew::LocalTrajectoryCommit>::SharedPtr execution_local_commit_result_pub_;
    rclcpp::Subscription<ew::TaskExecutionLease>::SharedPtr execution_task_lease_sub_;
    bool force_visible_side_target_{false},blocked_entry_replan_pending_{false};
    std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> collision_replan_incumbent_;
    Eigen::Vector3d blocked_entry_position_{Eigen::Vector3d::Zero()};
    rclcpp::Publisher<ew::ReferenceReceipt>::SharedPtr execution_receipt_pub_;
    rclcpp::CallbackGroup::SharedPtr execution_control_group_;
    struct WorkerResult {
      std::optional<SCANPlannerManager::SolvedCandidate> candidate;
      std::string failure;
      double solve_ms{0.},queue_ms{0.};
      std::chrono::steady_clock::time_point finished_at{};
      std::int64_t body_source_ns{0};
    };
    std::future<WorkerResult> solve_future_;
    std::optional<CompletedOwnerSolve<WorkerResult>> completed_solve_;
    std::optional<RetiredOwnerSolveFailure> retired_solve_failure_;
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
    std::size_t solve_snapshot_bytes_{0};
    d1max_planning_interfaces::msg::ReferencePath reference_metadata_;
    std::optional<d1max_planning_interfaces::msg::TrackingProgress> tracking_progress_;
    // Both inputs retain acquisition time. Callback order is not a motion
    // residual: keep a bounded source history for progress/odometry pairing.
    struct OdomSourceSample {
      std::int64_t source_ns;
      Eigen::Vector3d position;
      std::string frame;
    };
    std::deque<OdomSourceSample> odom_source_history_;
    d1max_planning_interfaces::msg::TrackingProgress::ConstSharedPtr pending_tracking_progress_;
    bool require_schema_v2_{false};
    bool planning_only_{false};
    bool local_follower_{false};
    bool fixed_stair_reference_continuity_{false};
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
    double body_height_;
    double reference_path_z_offset_{0.0};
    double reference_start_tolerance_{1.0};
    double odom_timeout_{0.5};
    bool strict_input_frames_{false};
    bool odom_twist_in_body_frame_{false};
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_replan_time_;
    double max_replan_interval_{0.0};
    double min_replan_interval_{0.0};
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
    std::int64_t last_local_debug_stamp_ns_{0};
    uint64_t last_local_debug_generation_{0};
    std::string last_local_debug_phase_;
    std::string last_attempt_failure_phase_;
    bool have_local_debug_state_{false};
    using ExecutionDebugKey=std::tuple<ew::ExecutionVersion,std::int64_t,std::string>;
    std::optional<ExecutionDebugKey> last_execution_debug_key_;
    std::chrono::steady_clock::time_point last_execution_debug_publish_{},last_data_display_publish_{};
    std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_visualized_committed_;
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
    rclcpp::Subscription<ew::LocalNavigationState>::SharedPtr execution_local_body_sub_;
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
    bool solveWorkPending() const;
    void collectSolveCompletion();
    void cancelSolveWorker();
    void cancelSnapshotAcquisition();
    bool snapshotAcquisitionCurrent();
    void publishCommittedTrajectory();
    void trackingProgressCallback(const d1max_planning_interfaces::msg::TrackingProgress::ConstSharedPtr &msg);
    void consumeTrackingProgress(const d1max_planning_interfaces::msg::TrackingProgress::ConstSharedPtr &msg);
    std::optional<Eigen::Vector3d> odometryPositionAtSource(std::int64_t source_ns,
                                                         const std::string &frame) const;
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool refreshPlanningBodyFromInbox();
    void finishOwnerBodyUpdate();
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
    bool keepPendingSpliceForOrdinaryReplan();
    bool environmentChangedSinceFailure();
    bool keepPreviewIncumbentForPeriodicReplan();
    std::optional<double> planningOnlyCommittedCurveTime() const;
    NoMotionPreviewProgress planningOnlyCommittedProgress() const;
    bool tryReplanPlanningOnlyConnection(const NoMotionPreviewProgress& progress);
    bool shouldReplanUncertifiedPreview();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();

    /* ROS functions */
    void execFSMCallback();
    void execFSMOwnerCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void typedPathCallback(const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr &msg);
    void replaceTypedReference(const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr &msg,
                               const std::vector<Eigen::Vector3d> *prepared);
    void installReferenceWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    void receiveReferenceProposal(ew::ReferenceProposal::ConstSharedPtr msg,const std::string &mode);
    std::optional<std::vector<Eigen::Vector3d>> prepareRollingReference(const ew::ReferencePath &msg) const;
    void installRollingReference(const ew::ReferencePath &msg,const std::vector<Eigen::Vector3d> &waypoints);
    void processDeferredRollingProposal();
    bool rollingReferenceBusy();
    void rejectDeferredRollingProposal(const std::string &reason,bool retry_priority);
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
    std::vector<ExecutorRole> executorRoles()const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
