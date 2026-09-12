#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
namespace d1max_localization {
using Information6 = Eigen::Matrix<double,6,6>;
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s;s<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return s;
}
// FastGICP uses a left perturbation [rotation, translation] about map origin.
// Recenter on the scanner and scale radians by a common scene radius. Per-axis
// diagonal whitening is deliberately forbidden: it hides unobservable axes.
inline double informationRatio(const Information6& h, const Eigen::Vector3d& center, double radius) {
  if(!h.allFinite()||!center.allFinite()||!std::isfinite(radius)||radius<=0)return 0.;
  Information6 change=Information6::Identity();
  change.block<3,3>(0,0)/=radius;
  change.block<3,3>(3,0)=skew(center)/radius;
  Information6 normalized=change.transpose()*h*change;
  Eigen::SelfAdjointEigenSolver<Information6> solver((normalized+normalized.transpose())*.5);
  if(solver.info()!=Eigen::Success || solver.eigenvalues().minCoeff()<=0)return 0.;
  return solver.eigenvalues().minCoeff()/std::max(1e-12,solver.eigenvalues().maxCoeff());
}
}
