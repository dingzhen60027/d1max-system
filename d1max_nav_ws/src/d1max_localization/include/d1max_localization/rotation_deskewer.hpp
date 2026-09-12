#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace d1max_localization {

struct AngularVelocitySample {
  int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct LinearVelocitySample {
  int64_t stamp_ns{0};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};  // fixed tracking axes, sensor origin
};

class RotationDeskewer {
public:
  bool build(const std::deque<AngularVelocitySample> &imu_samples,
             int64_t scan_start_ns, int64_t scan_end_ns,
             const Eigen::Vector3d &gyro_bias,
             const Eigen::Matrix3d &rotation_lidar_from_imu,
             double max_imu_gap_sec, std::string *error);

  // Add MC translation in the same fixed basis after building gyro rotations.
  // Failure invalidates compensation; callers must drop the scan, never use zero velocity.
  bool addTranslation(const std::deque<LinearVelocitySample>& samples,
                      double max_gap_sec, std::string* error);

  Eigen::Vector3d compensate(const Eigen::Vector3d &point,
                             int64_t point_stamp_ns) const;

private:
  struct RotationKnot {
    int64_t stamp_ns;
    Eigen::Quaterniond rotation;
  };

  Eigen::Quaterniond rotationAt(int64_t stamp_ns) const;
  Eigen::Vector3d translationAt(int64_t stamp_ns) const;
  std::vector<RotationKnot> knots_;
  std::vector<Eigen::Vector3d> translations_;
};

}  // namespace d1max_localization
