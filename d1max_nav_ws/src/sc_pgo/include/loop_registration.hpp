#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <string>
#include <vector>

namespace sc_pgo {
using LoopCloud = pcl::PointCloud<pcl::PointXYZI>;
struct RegistrationOptions {
  double coarse_leaf = 0.40, fine_leaf = 0.20;
  double coarse_distance = 2.0, fine_distance = 0.5;
  double overlap_distance = 0.35, min_overlap = 0.65, min_reverse_overlap = 0.35;
  double max_inlier_mse = 0.03, radius = 30.0;
};
struct RegistrationResult {
  bool accepted = false;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  double overlap = 0, reverse_overlap = 0;
  double inlier_mse = std::numeric_limits<double>::infinity();
  std::string reason = "no_converged_hypothesis";
};
// source=current BODY cloud, target=historical BODY submap. Coordinates stay
// close to zero; the estimated transform is a relative pose, not a map shift.
RegistrationResult registerLoop(const LoopCloud::ConstPtr& source,
  const LoopCloud::ConstPtr& target, const std::vector<Eigen::Matrix4f>& seeds,
  const RegistrationOptions& options);
}  // namespace sc_pgo
