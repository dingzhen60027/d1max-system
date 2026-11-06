#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

namespace scan_planner {

// A height graph supplied by the immutable support model, not a measured body
// height or permission to move vertically to avoid an obstacle.
struct SupportHeightSample {
  double height;
  Eigen::Vector2d gradient;
};
using SupportHeightSampler =
    std::function<std::optional<SupportHeightSample>(const Eigen::Vector3d&)>;

// SCAN optimizes XY. On one sloped support plane, freezing control-point Z
// instead of its height residual changes the physical support when XY moves.
// Parameterize only the unfixed controls by the plane's height increment. The
// first/last three columns (cubic PVA) and each initial height residual survive.
// A control point is not a body pose: bind the *actual seed curve*, never test a
// control point against a finite support polygon. A coupled output must still
// be sampled against this finite model by the optimizer before admission;
// native whole-curve checks retain obstacle and collision authority.
class SupportHeightCoupling {
 public:
  void reset() { bound_ = active_ = false; anchors_.resize(0, 0); }

  bool prepare(const Eigen::MatrixXd& controls, double dt,
               const SupportHeightSampler& sample,
               const std::function<bool()>& allowed = {}) {
    reset();
    if (!sample || controls.rows() != 3 || controls.cols() < 7 ||
        !controls.allFinite() || !std::isfinite(dt) || dt <= 0.) return false;
    std::optional<Eigen::Vector2d> gradient;
    double intercept = 0.;
    for (Eigen::Index i = 0; i < controls.cols(); ++i) {
      if (allowed && !allowed()) return false;
      // Uniform cubic Greville knot, including clamped first/last samples.
      // This is the actual curve value, not its control polygon, and is O(N)
      // overall instead of repeated linear knot searches inside de Boor.
      const Eigen::Index j = std::clamp<Eigen::Index>(i-1,0,controls.cols()-3);
      const Eigen::Vector3d p = (controls.col(j)+4.*controls.col(j+1)+controls.col(j+2))/6.;
      const auto support = sample(p);
      if (!support || !std::isfinite(support->height) ||
          !support->gradient.allFinite()) return false;
      const double b = support->height - support->gradient.dot(p.head<2>());
      if (!gradient) { gradient = support->gradient; intercept = b; }
      // A flight/landing junction is not one affine plane. Do not silently
      // apply the flight grade to a landing, another floor or another flight.
      // Such mixed seeds keep the existing implementation and final checks.
      else if ((support->gradient - *gradient).norm() > 1e-8 ||
               std::abs(b - intercept) > 1e-7) return false;
    }
    if (!gradient) return false;
    bound_ = true;
    intercept_ = intercept;
    if (gradient->squaredNorm() < 1e-24) return false;
    gradient_ = *gradient;
    anchors_ = controls;
    active_ = true;
    return true;
  }

  bool active() const { return active_; }
  bool bound() const { return bound_; }
  bool contains(const Eigen::Vector3d& p, const SupportHeightSampler& sample) const {
    if (!active_) return true;
    const auto support = sample ? sample(p) : std::nullopt;
    return support && std::isfinite(support->height) && support->gradient.allFinite() &&
        (support->gradient-gradient_).norm() <= 1e-8 &&
        std::abs(support->height-gradient_.dot(p.head<2>())-intercept_) <= 1e-7;
  }
  void applyProbe(const Eigen::Vector3d& center, Eigen::Vector3d& probe) const {
    if (active_) probe.z() += gradient_.dot(probe.head<2>() - center.head<2>());
  }

  void apply(Eigen::MatrixXd& controls) const {
    if (!active_) return;
    for (Eigen::Index i = 3; i < controls.cols() - 3; ++i)
      controls(2, i) = anchors_(2, i) + gradient_.dot(
          controls.col(i).head<2>() - anchors_.col(i).head<2>());
  }

  void pullback(Eigen::MatrixXd& gradient) const {
    if (active_)
      for (Eigen::Index i = 3; i < gradient.cols() - 3; ++i)
        gradient.col(i).head<2>() += gradient_ * gradient(2, i);
    // There is still no independent vertical optimization variable.
    gradient.row(2).setZero();
  }

 private:
  bool active_{false};
  bool bound_{false};
  Eigen::Vector2d gradient_{Eigen::Vector2d::Zero()};
  double intercept_{0.};
  Eigen::MatrixXd anchors_;
};
}  // namespace scan_planner
