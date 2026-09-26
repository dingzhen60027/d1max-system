#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
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
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);
    void setLocalReference(const std::vector<Eigen::Vector3d> &points) { local_reference_ = points; }
    const std::string &lastFailurePhase() const { return last_failure_phase_; }
    const std::vector<Eigen::Vector3d> &attemptBlockedPoints() const { return attempt_blocked_points_; }
    const std::vector<Eigen::Vector3d> &attemptDetourSeed() const { return attempt_detour_seed_; }
    // Optional continuity proof has its own small budget. Failure never relaxes
    // collision checks or rejects an otherwise valid replacement trajectory.
    bool recheckPredecessor(UniformBspline &trajectory) {
      return checkWholeTrajectoryCollision(trajectory, 5000, .008, false);
    }

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    rclcpp::Node *node_{nullptr};
    std::vector<Eigen::Vector3d> local_reference_;
    std::string last_failure_phase_{"failed_optimization"};
    std::vector<Eigen::Vector3d> attempt_blocked_points_, attempt_detour_seed_;
    double reference_detour_anchor_margin_{.5};
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continuous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);
    bool checkDynamicFeasibility(UniformBspline position_traj);
    bool checkWholeTrajectoryCollision(UniformBspline &position_traj,
        std::size_t query_budget=200000, double wall_budget_seconds=0., bool diagnostic=true);

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
