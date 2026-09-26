#pragma once
#include <Eigen/Core>
#include <cmath>
#include <deque>

namespace faster_lio {
// Startup admission only. Never flattens a running trajectory or assumes z=0.
// A low-variance IMU cannot distinguish rest from constant velocity; callers
// must start at rest. Defaults leave existing localization behavior unchanged.
class StationaryInitialization {
 public:
  double duration = 2.0;
  double max_acc_std = 0.20;
  double max_gyro_std = 0.015;
  double max_gyro_mean = 0.05;
  struct Sample { double t; Eigen::Vector3d a, w; };
  std::deque<Sample> samples;
  void clear() { samples.clear(); }
  bool add(double t, const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
    if (!std::isfinite(t) || !a.allFinite() || !w.allFinite()) {
      clear(); return false;
    }
    if (!samples.empty()) {
      if (t <= samples.back().t) return false;
      if (t - samples.back().t > 0.05) clear();
    }
    samples.push_back({t, a, w});
    while (samples.size() > 2 && samples[1].t <= t - duration) samples.pop_front();
    return true;
  }
  bool ready(int minimum_samples) const {
    if (samples.size() < static_cast<size_t>(minimum_samples) ||
        samples.back().t - samples.front().t < duration) return false;
    Eigen::Vector3d a = Eigen::Vector3d::Zero(), w = a, va = a, vw = a;
    for (const auto& s : samples) { a += s.a; w += s.w; }
    a /= samples.size(); w /= samples.size();
    for (const auto& s : samples) {
      va += (s.a-a).cwiseAbs2(); vw += (s.w-w).cwiseAbs2();
    }
    return a.norm() > 8.0 && a.norm() < 12.0 &&
      std::sqrt(va.sum()/samples.size()) <= max_acc_std &&
      std::sqrt(vw.sum()/samples.size()) <= max_gyro_std && w.norm() <= max_gyro_mean;
  }
};
}  // namespace faster_lio
