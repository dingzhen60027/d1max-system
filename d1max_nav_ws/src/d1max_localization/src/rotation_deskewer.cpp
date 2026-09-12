#include "d1max_localization/rotation_deskewer.hpp"

#include <algorithm>
#include <cmath>

namespace d1max_localization {
namespace {

constexpr double kNanosecondsToSeconds = 1e-9;

Eigen::Quaterniond rotationFromVector(const Eigen::Vector3d &rotation_vector) {
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

}  // namespace

bool RotationDeskewer::build(
    const std::deque<AngularVelocitySample> &imu_samples,
    int64_t scan_start_ns, int64_t scan_end_ns,
    const Eigen::Vector3d &gyro_bias,
    const Eigen::Matrix3d &rotation_lidar_from_imu,
    double max_imu_gap_sec, std::string *error) {
  knots_.clear();
  translations_.clear();
  auto fail = [&](const std::string &message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  if (scan_end_ns <= scan_start_ns) {
    return fail("scan duration is not positive");
  }
  if (imu_samples.size() < 2) {
    return fail("fewer than two IMU samples");
  }
  if (max_imu_gap_sec <= 0.0) {
    return fail("max IMU gap must be positive");
  }
  if (!gyro_bias.allFinite() || !rotation_lidar_from_imu.allFinite()) {
    return fail("IMU calibration contains invalid values");
  }

  const auto first_after_start = std::upper_bound(
      imu_samples.begin(), imu_samples.end(), scan_start_ns,
      [](int64_t stamp, const AngularVelocitySample &sample) {
        return stamp < sample.stamp_ns;
      });
  if (first_after_start == imu_samples.begin()) {
    return fail("IMU does not cover scan start");
  }
  const auto first_at_or_after_end = std::lower_bound(
      imu_samples.begin(), imu_samples.end(), scan_end_ns,
      [](const AngularVelocitySample &sample, int64_t stamp) {
        return sample.stamp_ns < stamp;
      });
  if (first_at_or_after_end == imu_samples.end()) {
    return fail("IMU does not cover scan end");
  }

  const auto coverage_begin = std::prev(first_after_start);
  const int64_t max_gap_ns =
      static_cast<int64_t>(max_imu_gap_sec / kNanosecondsToSeconds);
  for (auto current = coverage_begin; current != first_at_or_after_end;
       ++current) {
    const auto next = std::next(current);
    if (!current->angular_velocity.allFinite() ||
        !next->angular_velocity.allFinite()) {
      return fail("IMU angular velocity contains invalid values");
    }
    const int64_t gap_ns = next->stamp_ns - current->stamp_ns;
    if (gap_ns <= 0) {
      return fail("IMU timestamps are not strictly increasing");
    }
    if (gap_ns > max_gap_ns) {
      return fail("IMU gap exceeds configured limit");
    }
  }

  // Materialize Eigen expressions before locals leave scope (ASan regression).
  auto angularVelocityAt = [&](int64_t stamp_ns) -> Eigen::Vector3d {
    const auto right = std::lower_bound(
        imu_samples.begin(), imu_samples.end(), stamp_ns,
        [](const AngularVelocitySample &sample, int64_t stamp) {
          return sample.stamp_ns < stamp;
        });
    if (right != imu_samples.end() && right->stamp_ns == stamp_ns) {
      return rotation_lidar_from_imu *
             (right->angular_velocity - gyro_bias);
    }
    const auto left = std::prev(right);
    const double ratio = static_cast<double>(stamp_ns - left->stamp_ns) /
                         static_cast<double>(right->stamp_ns - left->stamp_ns);
    const Eigen::Vector3d interpolated =
        left->angular_velocity +
        ratio * (right->angular_velocity - left->angular_velocity);
    return rotation_lidar_from_imu * (interpolated - gyro_bias);
  };

  std::vector<int64_t> knot_times;
  knot_times.reserve(
      static_cast<size_t>(std::distance(coverage_begin, first_at_or_after_end)) +
      2);
  knot_times.push_back(scan_start_ns);
  for (auto sample = first_after_start; sample != first_at_or_after_end;
       ++sample) {
    if (sample->stamp_ns > scan_start_ns && sample->stamp_ns < scan_end_ns) {
      knot_times.push_back(sample->stamp_ns);
    }
  }
  knot_times.push_back(scan_end_ns);

  knots_.reserve(knot_times.size());
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  knots_.push_back({scan_start_ns, rotation});
  for (size_t index = 1; index < knot_times.size(); ++index) {
    const int64_t previous_ns = knot_times[index - 1];
    const int64_t current_ns = knot_times[index];
    const double dt =
        static_cast<double>(current_ns - previous_ns) * kNanosecondsToSeconds;
    const Eigen::Vector3d average_velocity =
        0.5 * (angularVelocityAt(previous_ns) +
               angularVelocityAt(current_ns));
    rotation =
        (rotation * rotationFromVector(average_velocity * dt)).normalized();
    knots_.push_back({current_ns, rotation});
  }

  if (error != nullptr) {
    error->clear();
  }
  return true;
}

Eigen::Quaterniond RotationDeskewer::rotationAt(int64_t stamp_ns) const {
  if (stamp_ns <= knots_.front().stamp_ns) {
    return knots_.front().rotation;
  }
  if (stamp_ns >= knots_.back().stamp_ns) {
    return knots_.back().rotation;
  }
  const auto right = std::lower_bound(
      knots_.begin(), knots_.end(), stamp_ns,
      [](const RotationKnot &knot, int64_t stamp) {
        return knot.stamp_ns < stamp;
      });
  if (right->stamp_ns == stamp_ns) {
    return right->rotation;
  }
  const auto left = std::prev(right);
  const double ratio = static_cast<double>(stamp_ns - left->stamp_ns) /
                       static_cast<double>(right->stamp_ns - left->stamp_ns);
  return left->rotation.slerp(ratio, right->rotation).normalized();
}

Eigen::Vector3d RotationDeskewer::compensate(
    const Eigen::Vector3d &point, int64_t point_stamp_ns) const {
  if (knots_.empty()) {
    return point;
  }
  const Eigen::Quaterniond point_rotation = rotationAt(point_stamp_ns);
  const Eigen::Quaterniond end_rotation = knots_.back().rotation;
  return end_rotation.conjugate() * (point_rotation * point + translationAt(point_stamp_ns) - translationAt(knots_.back().stamp_ns));
}

bool RotationDeskewer::addTranslation(const std::deque<LinearVelocitySample>& samples,
                                     double max_gap_sec,std::string* error) {
  translations_.clear();
  auto fail=[&](const std::string& reason) {
    knots_.clear();
    if(error) *error=reason;
    return false;
  };
  if(knots_.empty() || samples.size()<2 || !std::isfinite(max_gap_sec) || max_gap_sec<=0)
    return fail("missing / invalid velocity coverage");
  const int64_t start=knots_.front().stamp_ns,end=knots_.back().stamp_ns;
  if(samples.front().stamp_ns>start || samples.back().stamp_ns<end)
    return fail("velocity does not cover entire scan");
  for(size_t i=1;i<samples.size();++i) {
    if(samples[i].stamp_ns<=samples[i-1].stamp_ns)
      return fail("velocity timestamps not increasing");
    if(samples[i].stamp_ns>=start && samples[i-1].stamp_ns<=end &&
       (!samples[i].linear_velocity.allFinite() || !samples[i-1].linear_velocity.allFinite() ||
        (samples[i].stamp_ns-samples[i-1].stamp_ns)*1e-9>max_gap_sec))
      return fail("velocity gap / invalid value");
  }
  auto velocityAt=[&](int64_t t)->Eigen::Vector3d {
    auto right=std::lower_bound(samples.begin(),samples.end(),t,
        [](const LinearVelocitySample& s,int64_t t){return s.stamp_ns<t;});
    if(right->stamp_ns==t) return right->linear_velocity;
    auto left=std::prev(right);
    double u=double(t-left->stamp_ns)/double(right->stamp_ns-left->stamp_ns);
    return (1-u)*left->linear_velocity+u*right->linear_velocity;
  };
  Eigen::Vector3d position=Eigen::Vector3d::Zero();
  translations_.push_back(position);
  for(size_t i=1;i<knots_.size();++i) {
    const auto& left=knots_[i-1];const auto& right=knots_[i];
    const double dt=(right.stamp_ns-left.stamp_ns)*1e-9;
    position += .5*dt*(left.rotation*velocityAt(left.stamp_ns)+right.rotation*velocityAt(right.stamp_ns));
    translations_.push_back(position);
  }
  if(error)error->clear();
  return true;
}

Eigen::Vector3d RotationDeskewer::translationAt(int64_t t) const {
  if(translations_.empty())return Eigen::Vector3d::Zero();
  if(t<=knots_.front().stamp_ns)return translations_.front();
  if(t>=knots_.back().stamp_ns)return translations_.back();
  auto right=std::lower_bound(knots_.begin(),knots_.end(),t,
      [](const RotationKnot& k,int64_t t){return k.stamp_ns<t;});
  size_t i=std::distance(knots_.begin(),right);
  double u=double(t-knots_[i-1].stamp_ns)/double(right->stamp_ns-knots_[i-1].stamp_ns);
  return (1-u)*translations_[i-1]+u*translations_[i];
}

}  // namespace d1max_localization
