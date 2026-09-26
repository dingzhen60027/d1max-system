#include "loop_registration.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <cmath>
#include <stdexcept>

namespace sc_pgo {
namespace {
LoopCloud::Ptr downsample(const LoopCloud::ConstPtr& input, double leaf, double radius) {
  auto finite = std::make_shared<LoopCloud>();
  for (const auto& p : *input)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        p.getVector3fMap().norm() < radius) finite->push_back(p);
  auto output = std::make_shared<LoopCloud>();
  if (finite->empty()) return output;
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(leaf, leaf, leaf); voxel.setInputCloud(finite); voxel.filter(*output);
  return output;
}
std::pair<double,double> quality(const LoopCloud::ConstPtr& source,
                               const LoopCloud::ConstPtr& target, double distance) {
  if (source->empty() || target->empty()) return {0, std::numeric_limits<double>::infinity()};
  pcl::KdTreeFLANN<pcl::PointXYZI> tree; tree.setInputCloud(target);
  std::vector<int> ids(1); std::vector<float> ds(1);
  size_t n = 0; double squared_error = 0;
  for (const auto& p : *source) {
    if (tree.nearestKSearch(p, 1, ids, ds) && ds[0] <= distance*distance) {
      ++n; squared_error += ds[0];
    }
  }
  return {double(n)/source->size(), n ? squared_error/n : std::numeric_limits<double>::infinity()};
}
}
RegistrationResult registerLoop(const LoopCloud::ConstPtr& source,
  const LoopCloud::ConstPtr& target, const std::vector<Eigen::Matrix4f>& seeds,
  const RegistrationOptions& options) {
  RegistrationResult best;
  for (const double value : {options.coarse_leaf, options.fine_leaf, options.coarse_distance,
       options.fine_distance, options.overlap_distance, options.max_inlier_mse, options.radius})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid loop registration scale");
  if (!source || !target || !std::isfinite(options.min_overlap) || options.min_overlap <= 0 ||
      options.min_overlap > 1 || !std::isfinite(options.min_reverse_overlap) ||
      options.min_reverse_overlap <= 0 || options.min_reverse_overlap > 1)
    throw std::invalid_argument("Invalid loop cloud or overlap threshold");
  auto coarse_s = downsample(source, options.coarse_leaf, options.radius);
  auto coarse_t = downsample(target, options.coarse_leaf, options.radius);
  auto fine_s = downsample(source, options.fine_leaf, options.radius);
  auto fine_t = downsample(target, options.fine_leaf, options.radius);
  if (coarse_s->size() < 100 || coarse_t->size() < 100) { best.reason="sparse"; return best; }
  for (const auto& seed : seeds) {
    if (!seed.allFinite()) continue;
    pcl::IterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> coarse, fine;
    coarse.setInputSource(coarse_s); coarse.setInputTarget(coarse_t);
    coarse.setMaxCorrespondenceDistance(options.coarse_distance);
    coarse.setMaximumIterations(45); coarse.setTransformationEpsilon(1e-7);
    coarse.setEuclideanFitnessEpsilon(1e-6);
    LoopCloud scratch; coarse.align(scratch, seed);
    if (!coarse.hasConverged()) continue;
    fine.setInputSource(fine_s); fine.setInputTarget(fine_t);
    fine.setMaxCorrespondenceDistance(options.fine_distance);
    fine.setMaximumIterations(60); fine.setTransformationEpsilon(1e-8);
    fine.setEuclideanFitnessEpsilon(1e-7);
    auto aligned=std::make_shared<LoopCloud>();
    fine.align(*aligned, coarse.getFinalTransformation());
    if (!fine.hasConverged() || !fine.getFinalTransformation().allFinite()) continue;
    const auto q = quality(aligned, fine_t, options.overlap_distance);
    const auto reverse = quality(fine_t, aligned, options.overlap_distance);
    const bool passed=q.first>=options.min_overlap && reverse.first>=options.min_reverse_overlap &&
                      q.second<=options.max_inlier_mse;
    // Nearly equal coverage is common in enclosed scenes. Do not keep the
    // first seed just because both have 100% overlap: prefer its lower error.
    const bool better_quality = q.first > best.overlap + .01 ||
      (std::abs(q.first-best.overlap) <= .01 && q.second < best.inlier_mse);
    if ((passed && !best.accepted) || (passed==best.accepted && better_quality)) {
      best.accepted=passed; best.transform=fine.getFinalTransformation();
      best.overlap=q.first; best.reverse_overlap=reverse.first; best.inlier_mse=q.second;
      best.reason=passed ? "geometry_pass" : "overlap_or_residual";
    }
  }
  return best;
}
}  // namespace sc_pgo
