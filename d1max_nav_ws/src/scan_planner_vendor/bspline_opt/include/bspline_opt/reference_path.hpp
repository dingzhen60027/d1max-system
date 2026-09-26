#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace scan_planner {

// Geometry-only reference: interpolation remains on the validated PCT polyline.
// No second unconstrained polynomial fit is allowed to invent a shortcut.
class DiscreteReference {
 public:
  std::vector<Eigen::Vector3d> points;
  std::vector<double> arc;

  void set(const std::vector<Eigen::Vector3d> &input) {
    points.clear(); arc.clear();
    for (const auto &point : input) {
      if (!point.allFinite()) throw std::invalid_argument("non-finite discrete reference");
      if (points.empty()) { points.push_back(point); arc.push_back(0.0); }
      else if ((point - points.back()).norm() > 1e-6) {
        arc.push_back(arc.back() + (point - points.back()).norm()); points.push_back(point);
      }
    }
    if (points.size() < 2) throw std::invalid_argument("reference needs a nonzero segment");
  }
  double length() const { return arc.empty() ? 0.0 : arc.back(); }
  Eigen::Vector3d sample(double distance) const {
    if (points.empty()) throw std::logic_error("reference not initialized");
    distance = std::clamp(distance, 0.0, length());
    auto it = std::upper_bound(arc.begin(), arc.end(), distance);
    if (it == arc.end()) return points.back();
    const auto i = static_cast<std::size_t>(it - arc.begin() - 1);
    const double ratio = (distance - arc[i]) / (arc[i + 1] - arc[i]);
    return (1.0 - ratio) * points[i] + ratio * points[i + 1];
  }
  // Bounded forward projection avoids jumping to a later crossing/corridor.
  double project(const Eigen::Vector3d &position, double lower, double upper) const {
    lower = std::clamp(lower, 0.0, length()); upper = std::clamp(upper, lower, length());
    double best = lower, best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      if (arc[i + 1] < lower || arc[i] > upper) continue;
      const double lo = std::max(lower, arc[i]), hi = std::min(upper, arc[i + 1]);
      const Eigen::Vector3d tangent = (points[i + 1] - points[i]) / (arc[i + 1] - arc[i]);
      const double s = std::clamp(arc[i] + (position - points[i]).dot(tangent), lo, hi);
      const double d2 = (sample(s) - position).squaredNorm();
      if (d2 + 1e-12 < best_d2) { best_d2 = d2; best = s; }
    }
    return best;
  }
  std::vector<Eigen::Vector3d> slice(double begin, double end, const Eigen::Vector3d &actual_start) const {
    std::vector<Eigen::Vector3d> out{actual_start};
    const auto append = [&out](const Eigen::Vector3d &p) {
      if ((p - out.back()).norm() > 1e-6) out.push_back(p);
    };
    append(sample(begin));
    for (std::size_t i = 1; i < points.size(); ++i)
      if (arc[i] > begin + 1e-6 && arc[i] < end - 1e-6) append(points[i]);
    append(sample(end));
    return out;
  }
};

struct ReferenceSeed {
  std::vector<Eigen::Vector3d> samples;
  double dt;
};

inline ReferenceSeed sampleReferenceSeed(const DiscreteReference &path, double max_speed,
                                        double acceleration, double start_speed, double end_speed,
                                        double point_spacing) {
  if (path.length() <= 1e-6 || max_speed <= 0 || acceleration <= 0 || point_spacing <= 0)
    throw std::invalid_argument("invalid reference seed limits");
  const double v0 = std::clamp(start_speed, 0.0, max_speed);
  const double v1 = std::clamp(end_speed, 0.0, max_speed);
  const double peak = std::min(max_speed, std::sqrt(acceleration * path.length() + .5 * (v0*v0 + v1*v1)));
  const double ta = std::max(0.0, (peak - v0) / acceleration);
  const double td = std::max(0.0, (peak - v1) / acceleration);
  const double sa = (v0 + peak) * ta * .5;
  const double sd = (v1 + peak) * td * .5;
  const double tc = std::max(0.0, (path.length() - sa - sd) / peak);
  const double total_time = ta + tc + td;
  const int segments = std::max(6, static_cast<int>(std::ceil(total_time * max_speed / point_spacing)));
  ReferenceSeed seed{{}, total_time / segments};
  for (int i = 0; i <= segments; ++i) {
    const double t = seed.dt * i;
    double s;
    if (t < ta) s = v0*t + .5*acceleration*t*t;
    else if (t < ta + tc) s = sa + peak*(t - ta);
    else { const double u = t - ta - tc; s = sa + peak*tc + peak*u - .5*acceleration*u*u; }
    seed.samples.push_back(path.sample(i == segments ? path.length() : s));
  }
  return seed;
}

// Correspondence cost at uniform cubic spline knots. The caller supplies the
// collision-checked local guidance seed; keep it during retiming/refinement,
// rather than pulling a valid detour back onto an obstructed global line.
inline double referenceSampleCost(const Eigen::MatrixXd &control,
                                  const std::vector<Eigen::Vector3d> &reference,
                                  Eigen::MatrixXd &gradient) {
  gradient = Eigen::MatrixXd::Zero(3, control.cols());
  if (reference.size() < 2 || control.cols() < 4) return 0.0;
  const int samples = control.cols() - 2;
  double cost = 0.0;
  for (int i = 0; i < samples; ++i) {
    const double index = static_cast<double>(i) * (reference.size() - 1) / (samples - 1);
    const auto a = std::min(reference.size() - 2, static_cast<std::size_t>(index));
    const double ratio = index - a;
    const Eigen::Vector3d target = (1.0 - ratio) * reference[a] + ratio * reference[a + 1];
    const Eigen::Vector3d error = (control.col(i) + 4.0 * control.col(i + 1) + control.col(i + 2)) / 6.0 - target;
    cost += error.squaredNorm();
    gradient.col(i) += error / 3.0;
    gradient.col(i + 1) += 4.0 * error / 3.0;
    gradient.col(i + 2) += error / 3.0;
  }
  return cost;
}

}  // namespace scan_planner
