// Standalone regression for the actual native source, without ROS or GTSAM.
#include "map_manager/dense_elevation_map.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void Require(const bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void Near(const double actual, const double expected, const std::string& label,
          const double tolerance = 1e-8) {
  Require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          label + ": actual=" + std::to_string(actual) +
              ", expected=" + std::to_string(expected));
}

DenseElevationMap Map(const Eigen::MatrixXd& cost, const int layers = 1,
                      const double upper_height = 0.0) {
  DenseElevationMap result;
  Eigen::MatrixXd height = Eigen::MatrixXd::Zero(cost.rows(), cost.cols());
  if (layers == 2) height.bottomRows(cost.rows() / 2).setConstant(upper_height);
  const Eigen::MatrixXd ceiling = height.array() + 3.0;
  // Deliberately unrelated cached gradients: this API differentiates its
  // bilinear scalar, not a rotated/stale gradient array supplied by the caller.
  result.Init(0.2, layers, cost, Eigen::MatrixXd::Zero(cost.rows(), cost.cols()),
              height, ceiling, Eigen::MatrixXd::Constant(cost.rows(), cost.cols(), 123.0),
              Eigen::MatrixXd::Constant(cost.rows(), cost.cols(), -456.0));
  return result;
}

double Lookup(DenseElevationMap& map, const bool safe, const double x,
              const double y, Eigen::Vector2d* grad = nullptr) {
  return safe ? map.GetValueBilinearSafe(0, x, y, 0.0, grad)
              : map.GetValueBilinear(0, x, y, grad);
}

Eigen::MatrixXd Plane(const int rows = 5, const int columns = 6) {
  Eigen::MatrixXd values(rows, columns);
  for (int row = 0; row < rows; ++row)
    for (int column = 0; column < columns; ++column)
      values(row, column) = 1.0 + 2.0 * column + 3.0 * row;
  return values;
}

void Constant(const bool safe) {
  auto map = Map(Eigen::MatrixXd::Constant(5, 6, 13.0));
  for (const double x : {-10.0, 0.0, 0.5, 1.25, 4.99, 5.0, 99.0})
    for (const double y : {-10.0, 0.0, 1.5, 3.75, 4.0, 99.0}) {
      Eigen::Vector2d grad;
      Near(Lookup(map, safe, x, y, &grad), 13.0, "constant scalar");
      Near(grad(0), 0.0, "constant dx");
      Near(grad(1), 0.0, "constant dy");
      Near(Lookup(map, safe, x, y), 13.0, "constant without gradient");
    }
}

void Planar(const bool safe) {
  auto map = Map(Plane());
  for (const double x : {0.0, 0.25, 0.5, 1.49, 1.5, 2.77, 5.0})
    for (const double y : {0.0, 0.5, 1.75, 2.49, 2.5, 4.0}) {
      Eigen::Vector2d grad;
      Near(Lookup(map, safe, x, y, &grad), 1 + 2 * x + 3 * y, "plane scalar");
      Near(grad(0), 2.0, "plane dx");
      Near(grad(1), 3.0, "plane dy");
    }
}

void Peak(const bool safe) {
  Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(5, 5);
  cost(2, 2) = 50.0;
  auto map = Map(cost);
  for (int row = 0; row < 5; ++row)
    for (int column = 0; column < 5; ++column)
      Near(Lookup(map, safe, column, row), cost(row, column), "integer cell centre");
  Near(Lookup(map, safe, 2.49, 2.0), 25.5, "peak right slope");
  Near(Lookup(map, safe, 2.5, 2.0), 25.0, "peak half-cell");
  Near(Lookup(map, safe, 1.49, 2.0), 24.5, "peak left slope");
  for (double x = 0.0; x <= 4.0; x += 0.125)
    for (double y = 0.0; y <= 4.0; y += 0.125) {
      const double result = Lookup(map, safe, x, y);
      Require(result >= 0 && result <= 50, "interpolation escaped scalar convex hull");
    }
}

void HalfCellContinuity(const bool safe) {
  Eigen::MatrixXd values(6, 6);
  for (int row = 0; row < 6; ++row)
    for (int column = 0; column < 6; ++column)
      values(row, column) = (3 * row + 7 * column + row * column) % 19;
  auto map = Map(values);
  const double epsilon = 1e-7;
  for (double boundary = 0.5; boundary < 5.0; boundary += 0.5)
    for (const double fixed : {0.23, 1.73, 3.19, 4.41}) {
      Near(Lookup(map, safe, boundary - epsilon, fixed),
           Lookup(map, safe, boundary + epsilon, fixed), "continuous x", 1e-5);
      Near(Lookup(map, safe, fixed, boundary - epsilon),
           Lookup(map, safe, fixed, boundary + epsilon), "continuous y", 1e-5);
    }
}

void Boundary(const bool safe) {
  auto map = Map(Plane());
  for (const double x : {-100.0, -0.1, 0.0, 0.25, 4.9, 5.0, 5.1, 100.0})
    for (const double y : {-100.0, -0.1, 0.0, 0.75, 3.9, 4.0, 4.1, 100.0}) {
      Eigen::Vector2d grad;
      const double cx = std::max(0.0, std::min(x, 5.0));
      const double cy = std::max(0.0, std::min(y, 4.0));
      Near(Lookup(map, safe, x, y, &grad), 1 + 2 * cx + 3 * cy, "clamped plane");
      Near(grad(0), (x < 0 || x > 5) ? 0.0 : 2.0, "clamped plane dx");
      Near(grad(1), (y < 0 || y > 4) ? 0.0 : 3.0, "clamped plane dy");
    }
  // The chosen boundary derivative is the interior one-sided derivative.
  const double h = 1e-6;
  Eigen::Vector2d grad;
  const double low = Lookup(map, safe, 0, 2, &grad);
  Near((Lookup(map, safe, h, 2) - low) / h, grad(0), "low one-sided dx", 1e-6);
  const double high = Lookup(map, safe, 5, 2, &grad);
  Near((high - Lookup(map, safe, 5 - h, 2)) / h, grad(0), "high one-sided dx", 1e-6);
  for (const auto& size : std::vector<std::pair<int, int>>{{1, 1}, {1, 6}, {5, 1}}) {
    auto narrow = Map(Plane(size.first, size.second));
    for (const double x : {-10., 0., 0.25, 10.})
      for (const double y : {-10., 0., 0.75, 10.}) {
        const double cx = std::max(0.0, std::min(x, double(size.second - 1)));
        const double cy = std::max(0.0, std::min(y, double(size.first - 1)));
        Near(Lookup(narrow, safe, x, y, &grad), 1 + 2 * cx + 3 * cy, "singleton dimension");
        if (size.second == 1) Near(grad(0), 0.0, "singleton dx");
        if (size.first == 1) Near(grad(1), 0.0, "singleton dy");
      }
  }
}

void Gradient(const bool safe) {
  Eigen::MatrixXd values(6, 7);
  for (int row = 0; row < values.rows(); ++row)
    for (int column = 0; column < values.cols(); ++column)
      values(row, column) = 6 + std::sin(0.7 * column) + std::cos(0.4 * row) +
                            0.05 * row * column;
  auto map = Map(values);
  const double h = 1e-6;
  for (const double x : {-0.8, 0.13, 1.23, 2.73, 4.41, 5.89, 6.8})
    for (const double y : {-0.9, 0.31, 1.57, 2.91, 4.63, 5.9}) {
      Eigen::Vector2d grad;
      Lookup(map, safe, x, y, &grad);
      Near(grad(0), (Lookup(map, safe, x + h, y) - Lookup(map, safe, x - h, y)) / (2 * h),
           "finite difference dx", 2e-6);
      Near(grad(1), (Lookup(map, safe, x, y + h) - Lookup(map, safe, x, y - h)) / (2 * h),
           "finite difference dy", 2e-6);
    }
}

void LayerSelection(const bool safe) {
  Eigen::MatrixXd values(10, 6);
  values.topRows(5) = Plane().array() + 20.0;
  values.bottomRows(5) = Plane();
  auto different_floor = Map(values, 2, 3.0);
  auto same_floor = Map(values, 2, 0.0);
  for (const double x : {0.0, 0.3, 1.5, 4.7, 5.0})
    for (const double y : {0.0, 0.7, 2.5, 4.0}) {
      Near(Lookup(different_floor, safe, x, y), 21 + 2 * x + 3 * y, "do not change floors");
      Near(Lookup(same_floor, safe, x, y), 1 + 2 * x + 3 * y, "existing same-floor cost selection");
    }
}

void Nonfinite(const bool safe) {
  auto map = Map(Plane());
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()})
    for (const bool on_x : {false, true}) {
      bool rejected = false;
      try { Lookup(map, safe, on_x ? bad : 1.0, on_x ? 1.0 : bad); }
      catch (const std::invalid_argument&) { rejected = true; }
      Require(rejected, "nonfinite coordinate was not rejected");
    }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Require(argc == 3, "usage: bilinear_probe CASE safe|nominal");
    const std::string name = argv[1];
    const std::string mode = argv[2];
    Require(mode == "safe" || mode == "nominal", "invalid API mode");
    const bool safe = mode == "safe";
    if (name == "constant") Constant(safe);
    else if (name == "plane") Planar(safe);
    else if (name == "peak") Peak(safe);
    else if (name == "continuity") HalfCellContinuity(safe);
    else if (name == "boundary") Boundary(safe);
    else if (name == "gradient") Gradient(safe);
    else if (name == "layers") LayerSelection(safe);
    else if (name == "nonfinite") Nonfinite(safe);
    else throw std::runtime_error("unknown case: " + name);
    std::cout << "PASS " << name << " " << mode << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
