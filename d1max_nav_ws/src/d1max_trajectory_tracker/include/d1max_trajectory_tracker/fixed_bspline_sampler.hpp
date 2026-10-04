#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

namespace d1max_trajectory_tracker {

// An allocation-free, fixed-3D execution of the vendor's exact De Boor
// recurrence. Prepared FROM the vendor's actual controls/knots/order (including
// its derivative curves), not an interpolated table or a second knot contract.
// Keep vendor construction, derivative bounds, geometry and source admission.
class FixedBsplineSampler final {
 public:
  static std::shared_ptr<const FixedBsplineSampler> fromVendor(scan_planner::UniformBspline& curve) {
    auto points=curve.getControlPoint();auto knots=curve.getKnot();const int degree=curve.getOrder();
    if(degree<1||degree>3||points.rows()!=3||points.cols()<=degree||points.cols()>10000||
       knots.size()!=points.cols()+degree+1||!points.allFinite()||!knots.allFinite())return {};
    // Production admits SCAN's strictly increasing, non-clamped knot vectors.
    // A repeated knot is not normalized here: the existing rejection survives.
    for(Eigen::Index i=1;i<knots.size();++i)if(knots[i]-knots[i-1]<=1e-9)return {};
    return std::shared_ptr<const FixedBsplineSampler>(new FixedBsplineSampler(
      std::move(points),std::move(knots),degree));
  }

  Eigen::Vector3d evaluateDeBoorT(const double& time)const {
    // Argument order matters for NaN and signed zero; mirror evaluateDeBoorT
    // and min(max(low,u),high) exactly. Native admission still rejects NaN.
    const double u=time+knots_[degree_];
    const double ub=std::min(std::max(knots_[degree_],u),knots_[knots_.size()-degree_-1]);
    const double* first=knots_.data()+degree_+1;
    const double* last=knots_.data()+points_.cols()+1;
    // Native chooses the LEFT span at an exact knot, not upper_bound's right.
    const int k=static_cast<int>(std::lower_bound(first,last,ub)-knots_.data())-1;
    std::array<Eigen::Vector3d,4> d;
    for(int i=0;i<=degree_;++i)d[i]=points_.col(k-degree_+i);
    for(int r=1;r<=degree_;++r)for(int i=degree_;i>=r;--i) {
      const double alpha=(ub-knots_[i+k-degree_])/(knots_[i+1+k-r]-knots_[i+k-degree_]);
      d[i]=(1-alpha)*d[i-1]+alpha*d[i];
    }
    return d[degree_];
  }

 private:
  FixedBsplineSampler(Eigen::MatrixXd points,Eigen::VectorXd knots,int degree):
    points_(std::move(points)),knots_(std::move(knots)),degree_(degree) {}
  const Eigen::MatrixXd points_;
  const Eigen::VectorXd knots_;
  const int degree_;
};
} // namespace d1max_trajectory_tracker
