#pragma once

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace scan_planner {

// A soft geometric comfort objective, not collision or motion permission.
// Uniform cubic B-spline derivatives use normalized span u, NOT seconds, so
// retiming cannot make a sharp route look geometrically straighter. The 1 cm
// regularizer has control-polygon length units, not a measured speed gate.
// Three-point Gauss integration per span is bounded O(number of controls).
inline double planarCurvatureCost(const Eigen::MatrixXd& control,double preferred_radius,
                                 Eigen::MatrixXd& gradient) {
  if(control.rows()!=3||control.cols()<4||!control.allFinite()||
     !std::isfinite(preferred_radius)||preferred_radius<=0.)
    throw std::invalid_argument("invalid planar curvature objective input");
  gradient=Eigen::MatrixXd::Zero(3,control.cols());
  constexpr double epsilon=.01;
  constexpr double nodes[]={.11270166537925831148,.5,.88729833462074168852};
  constexpr double weights[]={5./18.,4./9.,5./18.};
  const double preferred_curvature=1./preferred_radius;
  double cost=0.;
  for(Eigen::Index span=0;span+3<control.cols();++span) {
    for(int sample=0;sample<3;++sample) {
      const double u=nodes[sample],u2=u*u;
      const double first[]={-.5*(1.-u)*(1.-u),1.5*u2-2.*u,
                            -1.5*u2+u+.5,.5*u2};
      const double second[]={1.-u,3.*u-2.,1.-3.*u,u};
      Eigen::Vector2d velocity=Eigen::Vector2d::Zero(),acceleration=Eigen::Vector2d::Zero();
      for(int j=0;j<4;++j) {
        velocity+=first[j]*control.col(span+j).head<2>();
        acceleration+=second[j]*control.col(span+j).head<2>();
      }
      const double speed=velocity.norm();
      const double square=velocity.squaredNorm()+epsilon*epsilon;
      const double root=std::sqrt(square),inverse_cube=1./(square*root);
      const double cross=velocity.x()*acceleration.y()-velocity.y()*acceleration.x();
      const double curvature=cross*inverse_cube;
      const double excess=std::abs(curvature)-preferred_curvature;
      if(excess<=0.)continue;
      // excess>0 implies nonzero v: at v=0 the regularized curvature is 0.
      const double weight=weights[sample],sign=std::copysign(1.,curvature);
      cost+=weight*excess*excess*speed;
      const Eigen::Vector2d curvature_v=
          Eigen::Vector2d(acceleration.y(),-acceleration.x())*inverse_cube-
          3.*cross*inverse_cube/square*velocity;
      const Eigen::Vector2d curvature_a=
          Eigen::Vector2d(-velocity.y(),velocity.x())*inverse_cube;
      const Eigen::Vector2d gradient_v=weight*(2.*excess*sign*speed*curvature_v+
          excess*excess/speed*velocity);
      const Eigen::Vector2d gradient_a=weight*2.*excess*sign*speed*curvature_a;
      for(int j=0;j<4;++j)
        gradient.col(span+j).head<2>()+=first[j]*gradient_v+second[j]*gradient_a;
    }
  }
  return cost;
}

} // namespace scan_planner
