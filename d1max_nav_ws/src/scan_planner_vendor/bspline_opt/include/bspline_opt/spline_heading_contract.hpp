#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace scan_planner {

// Explicit no-motion preview only. Velocity is NOT changed: this says that a
// mm/s position derivative is not a trustworthy estimate of body orientation.
// 0.02 m/s is an engineering direction-confidence threshold above the observed
// 1--10 mm/s stationary jitter, NOT a calibrated stop/motion detection threshold.
struct SplineHeadingContract {
  bool preview_only_enabled{false};
  double measured_yaw{0.};
  double minimum_direction_speed{.02};
  double body_extent{.49};
};

inline bool validPreviewHeadingContract(const SplineHeadingContract &contract) {
  return std::isfinite(contract.measured_yaw) &&
      std::isfinite(contract.minimum_direction_speed) &&
      contract.minimum_direction_speed>0. && contract.minimum_direction_speed<=.05 &&
      std::isfinite(contract.body_extent) && contract.body_extent>0.;
}

inline double previewBodyHeading(const Eigen::Vector3d &velocity,double previous_yaw,
                                  const SplineHeadingContract &contract) {
  return velocity.head<2>().norm()<contract.minimum_direction_speed ? previous_yaw :
      std::atan2(velocity.y(),velocity.x());
}

// Full translation x rotation sweep, including BOTH endpoint positions and
// every intermediate heading. Changing direction confidence cannot jump over
// a colliding rotation. The caller owns the shared query/time budget.
template<class Query>
bool headingTransitionFree(const Eigen::Vector3d &from,const Eigen::Vector3d &to,
    double from_yaw,double to_yaw,double resolution,double body_extent,Query query) {
  if (!from.allFinite() || !to.allFinite() || !std::isfinite(from_yaw) ||
      !std::isfinite(to_yaw) || !std::isfinite(resolution) || resolution<=0. ||
      !std::isfinite(body_extent) || body_extent<=0.) return false;
  const double yaw_step=std::min(.05,resolution*.25/body_extent);
  const double turn=std::atan2(std::sin(to_yaw-from_yaw),std::cos(to_yaw-from_yaw));
  const double linear_count=std::ceil((to-from).norm()/(resolution*.25));
  const double angular_count=std::ceil(std::abs(turn)/yaw_step);
  if (!std::isfinite(linear_count) || !std::isfinite(angular_count) ||
      linear_count>50000. || angular_count>50000.) return false;
  const int translations=std::max(1,static_cast<int>(linear_count));
  const int turns=std::max(1,static_cast<int>(angular_count));
  for(int j=0;j<=translations;++j) {
    const Eigen::Vector3d p=from+(to-from)*(static_cast<double>(j)/translations);
    for(int k=0;k<=turns;++k)
      if (!query(p,from_yaw+turn*(static_cast<double>(k)/turns))) return false;
  }
  return true;
}

} // namespace scan_planner
