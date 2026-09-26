#include "ele_planner/offline_ele_planner.h"

#include <stdexcept>

void OfflineElePlanner::InitMap(
    const double a_start_cost_threshold, const double safe_cost_margin,
    const double resolution, const int num_layers, const double step_cost_weight,
    ConstStridedMatrixRef cost_map, ConstStridedMatrixRef height_map,
    ConstStridedMatrixRef ceiling, ConstStridedMatrixRef ele_map,
    ConstStridedMatrixRef grad_x, ConstStridedMatrixRef grad_y) {
  const auto same_shape = [&cost_map](const auto& matrix) {
    return matrix.rows() == cost_map.rows() && matrix.cols() == cost_map.cols();
  };
  if (num_layers <= 0 || cost_map.rows() <= 0 || cost_map.cols() <= 0 ||
      cost_map.rows() % num_layers != 0 || !same_shape(height_map) ||
      !same_shape(ceiling) || !same_shape(ele_map) || !same_shape(grad_x) ||
      !same_shape(grad_y) || !std::isfinite(resolution) || resolution <= 0) {
    throw std::invalid_argument("Invalid planner map dimensions or resolution");
  }
  map_ = std::make_shared<DenseElevationMap>();
  map_->Init(resolution, num_layers, cost_map, ele_map, height_map, ceiling,
             grad_x, grad_y);
  path_finder_.InitShared(
      a_start_cost_threshold, num_layers, resolution, step_cost_weight,
      std::shared_ptr<const Eigen::MatrixXd>(map_, &map_->CostMatrix()),
      std::shared_ptr<const Eigen::MatrixXd>(map_, &map_->HeightMatrix()),
      std::shared_ptr<const Eigen::MatrixXd>(map_, &map_->GatewayMatrix()));
  trajectory_optimizer_ = GPMPOptimizerWnoa(safe_cost_margin, map_);
  trajectory_optimizer_wnoj_ =
      GPMPOptimizer(safe_cost_margin, max_heading_rate_, map_);
}

bool OfflineElePlanner::Plan(const Eigen::Vector3i& start,
                             const Eigen::Vector3i& goal, const bool optimize) {
  path_.clear();
  if (!path_finder_.Search(start, goal)) {
    printf("A star Failed!\n");
    return false;
  }

  if (optimize) {
    path_ = path_finder_.GetPathPoints();
    // A* omits the start node; same/adjacent cells cannot provide GPMP's two
    // required support states. Validate here rather than running A* twice in
    // Python (or aborting at path.front()/the optimizer's assertion).
    if (path_.size() < 2) {
      throw std::invalid_argument(
          "PCT endpoints are too close for native GPMP optimization");
    }
    path_.front().ref_v = 1;
    path_.back().ref_v = 1;

    bool success = false;
    if (use_quintic_) {
      success = trajectory_optimizer_wnoj_.GenerateTrajectory(path_, 200);
    } else {
      success = trajectory_optimizer_.GenerateTrajectory(path_, 200);
    }

    return success;
  }

  return true;
}
