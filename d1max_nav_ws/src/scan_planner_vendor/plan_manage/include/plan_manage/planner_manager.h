#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <optional>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <plan_manage/candidate_adoption.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace scan_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class SCANPlannerManager
  {
    // SECTION stable
  public:
    SCANPlannerManager();
    ~SCANPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                       SolveBudget::Ptr budget = {});
    struct SolvedCandidate {
      UniformBspline curve;
      SplineHeadingContract heading;
      std::uint64_t context_sequence{0};
      MeasuredBodyPose solve_body;
      bool single_segment{false};
    };
    // No subscriptions, timers or writable live GridMap are constructed here.
    void initSolveWorker(const SCANPlannerManager &owner);
    void releaseOwnerSolver() { bspline_optimizer_rebound_.reset(); }
    void prepareSolveWorker(const SCANPlannerManager &owner, const GridMap::Ptr &snapshot);
    void releaseSolveSnapshot() noexcept;
    std::optional<SolvedCandidate> takeSolvedCandidate();
    bool adoptSolvedCandidate(SolvedCandidate candidate, const SolveBudget::Ptr &budget);
    void importAttemptDiagnostics(const SCANPlannerManager &worker);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);
    void setLocalReference(const std::vector<Eigen::Vector3d> &points) { local_reference_ = points; }
    void setMeasuredBodyPose(const MeasuredBodyPose &pose, const std::string &frame, double maximum_age) {
      measured_body_pose_=pose; measured_body_frame_=frame; measured_body_maximum_age_=maximum_age;
    }
    void setMeasuredBodyVelocity(const Eigen::Vector3d &velocity) { measured_body_velocity_=velocity; }
    void setMeasuredJoinSingleSegment(bool value) { measured_join_single_segment_=value; }
    const std::optional<CandidateJoinEvidence> &acceptedJoinEvidence() const { return accepted_join_; }
    const std::string &lastFailurePhase() const { return last_failure_phase_; }
    // This only cancels an existing solve; transport/worker ownership must
    // deliver cancellation independently before claiming bounded ROS cancel.
    void cancelSolve() {
      if (const auto budget=std::atomic_load(&solve_budget_)) budget->cancel();
    }
    const std::vector<Eigen::Vector3d> &attemptBlockedPoints() const { return attempt_blocked_points_; }
    const std::vector<Eigen::Vector3d> &attemptDetourSeed() const { return attempt_detour_seed_; }
    // Optional continuity proof has its own small budget. Failure never relaxes
    // collision checks or rejects an otherwise valid replacement trajectory.
    bool recheckPredecessor(UniformBspline &trajectory, double measured_curve_time) {
      return checkWholeTrajectoryCollision(trajectory, 5000, .008, false, measured_curve_time);
    }
    bool hasPreviewHeadingContract() const { return accepted_heading_contract_.preview_only_enabled; }
    double previewCurveProgressTime() const { return preview_curve_progress_time_; }
    CurveCheckEvidence recheckPreviewTrajectory() {
      // A manually moved robot in no-motion preview still has real measured
      // progress. Certify that geometric join; a wall clock is never progress.
      CurveCheckEvidence evidence=CurveCheckEvidence::Uncertified;
      const auto source=grid_map_->latestCloudStampNs();
      const auto context=grid_map_->localizationContextSequence();
      const auto revision=grid_map_->occupancyRevision();
      const auto body_source=measured_body_pose_.source_stamp;
      const auto projected=measuredPreviewCurveTime(local_data_.position_traj_,
          measured_body_pose_.position,preview_curve_progress_time_,grid_map_->getResolution());
      if (!projected) return evidence;
      checkWholeTrajectoryCollision(local_data_.position_traj_,50000,.03,false,*projected,
                                    accepted_heading_contract_,&evidence);
      if (!grid_map_->integratedCloudFreshAt(node_->now().nanoseconds()) ||
          source!=grid_map_->latestCloudStampNs() || context!=grid_map_->localizationContextSequence() ||
          revision!=grid_map_->occupancyRevision() || body_source!=measured_body_pose_.source_stamp)
        return CurveCheckEvidence::Uncertified;
      if (evidence==CurveCheckEvidence::Clear) preview_curve_progress_time_=*projected;
      return evidence;
    }
    // A preview candidate is not an incumbent until the caller's final source
    // gate succeeds. Failed/aged solves must not overwrite accepted geometry.
    bool commitPreviewCandidate();
    void discardPreviewCandidate() { pending_preview_candidate_.reset(); }

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    rclcpp::Node *node_{nullptr};
    SolveBudget::Ptr solve_budget_;
    bool worker_only_{false};
    bool reboundReplanImpl(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                          Eigen::Vector3d end_pt, Eigen::Vector3d end_vel,
                          bool flag_polyInit, bool flag_randomPolyTraj);
    MeasuredBodyPose measured_body_pose_;
    Eigen::Vector3d measured_body_velocity_{Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN())};
    bool measured_join_single_segment_{false};
    std::optional<CandidateJoinEvidence> accepted_join_;
    std::string measured_body_frame_;
    double measured_body_maximum_age_{0.};
    std::vector<Eigen::Vector3d> local_reference_;
    std::string last_failure_phase_{"failed_optimization"};
    std::vector<Eigen::Vector3d> attempt_blocked_points_, attempt_detour_seed_;
    double reference_detour_anchor_margin_{.5};
    SplineHeadingContract preview_heading_contract_;
    SplineHeadingContract attempt_heading_contract_, accepted_heading_contract_;
    double preview_curve_progress_time_{0.};
    struct PreviewCandidate {
      UniformBspline curve;
      SplineHeadingContract heading;
      rclcpp::Time solved_at;
      std::uint64_t context_sequence;
    };
    std::optional<PreviewCandidate> pending_preview_candidate_;
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continuous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);
    bool checkDynamicFeasibility(UniformBspline position_traj);
    bool checkWholeTrajectoryCollision(UniformBspline &position_traj,
        std::size_t query_budget=200000, double wall_budget_seconds=0., bool diagnostic=true,
        double measured_curve_time=0., const SplineHeadingContract &heading_contract={},
        CurveCheckEvidence *evidence=nullptr);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<SCANPlannerManager> Ptr;

    // !SECTION
  };
} // namespace scan_planner

#endif
