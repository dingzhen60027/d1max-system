#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <cmath>
#include <functional>

namespace scan_planner {
struct InitialMotionLimits {
  bool valid{false},recovering{false};
  double peak_speed{0.},nominal_speed_bound{0.},acceleration_bound{0.},recovery_end{0.};
  double recovery_limit{0.};
  const char* reason{"invalid_curve"};
};

// A measured v(0) is a boundary condition, not a commanded cruising speed.
// For a uniform cubic only Q0/Q1 of its quadratic velocity are fixed by the
// measured v(0),a(0). They disappear after two knot spans. All other velocity
// controls MUST remain inside the ordinary speed ball. The convex-hull property
// proves the whole suffix bound continuously (not just at sampled timestamps).
// Recovery must fit within twice the INITIAL constant-deceleration stopping
// time; increasing knot time cannot turn an initial exception into cruising.
// No position/velocity/acceleration, source timestamp or command limit changes.
inline InitialMotionLimits inspectInitialMotionLimits(UniformBspline curve,
    double nominal_speed,double maximum_acceleration,bool allow_recovery=true) {
  InitialMotionLimits out;
  const auto p=curve.getControlPoint();const auto knots=curve.getKnot();
  // Wire decoding sets the original knot vector after construction; the
  // vendor's cached interval is then not authoritative. Read original knots.
  const double dt=knots.size()>4?knots[4]-knots[3]:NAN;
  if(!std::isfinite(nominal_speed)||nominal_speed<=0.||
     !std::isfinite(maximum_acceleration)||maximum_acceleration<=0.||
     curve.getOrder()!=3||p.rows()!=3||p.cols()<4||!p.allFinite()||
     knots.size()!=p.cols()+4||!knots.allFinite()||!std::isfinite(dt)||dt<=0.)return out;
  for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return out;
  auto velocity=curve.getDerivative();auto acceleration=velocity.getDerivative();
  const auto q=velocity.getControlPoint(),a=acceleration.getControlPoint();
  if(!q.allFinite()||!a.allFinite())return out;
  const double initial_speed=velocity.evaluateDeBoorT(0.).norm();
  out.recovering=allow_recovery&&initial_speed>nominal_speed+1e-9;
  if(out.recovering) {
    if(p.cols()<6){out.reason="initial_speed_recovery_too_short";return out;}
    for(int i=1;i<knots.size();++i)if(std::abs((knots[i]-knots[i-1])-dt)>1e-7*std::max(1.,dt)) {
      out.reason="recovery_requires_uniform_knots";return out;
    }
    out.recovery_end=2.*dt;
    out.recovery_limit=2.*initial_speed/maximum_acceleration;
    if(out.recovery_end>out.recovery_limit+1e-9) {
      out.reason="initial_speed_recovery_too_slow";return out;
    }
  }
  for(int i=0;i<q.cols();++i) {
    const double speed=q.col(i).norm();out.peak_speed=std::max(out.peak_speed,speed);
    if(!out.recovering||i>=2)out.nominal_speed_bound=std::max(out.nominal_speed_bound,speed);
  }
  for(int i=0;i<a.cols();++i)out.acceleration_bound=std::max(out.acceleration_bound,a.col(i).norm());
  if(out.acceleration_bound>maximum_acceleration+1e-6){out.reason="acceleration_exceeds_limit";return out;}
  if(out.nominal_speed_bound>nominal_speed+1e-6){out.reason="nominal_speed_exceeds_limit";return out;}
  out.valid=true;out.reason="ok";return out;
}

// SCAN's optimization objective penalizes derivatives softly. For an acquired
// above-cruise boundary, uniform time scaling cannot enforce the first-boundary
// equality. Project ONLY free interior controls onto the existing convex speed
// and acceleration balls. Endpoint p/v/a controls remain byte-for-byte fixed.
// This changes geometry: the caller must still do its ordinary full collision
// validation. Failure leaves the supplied curve unchanged, within the SAME
// solve deadline; this is not another planner or an executable fallback.
inline bool projectInitialRecoveryInterior(UniformBspline& curve,double speed,double acceleration,
    const std::function<bool()>& allowed={}) {
  const auto initial=inspectInitialMotionLimits(curve,speed,acceleration);
  if(!initial.recovering)return initial.valid;
  if(initial.valid)return true;
  if(initial.recovery_end<=0.||initial.recovery_end>initial.recovery_limit+1e-9)return false;
  auto p=curve.getControlPoint();const int n=p.cols();const auto knots=curve.getKnot();
  const double dt=knots[4]-knots[3];
  const auto free=[&](int i){return i>=3&&i<n-3;};
  const auto permitted=[&]{return !allowed||allowed();};
  for(int pass=0;pass<128;++pass) {
    if(!permitted())return false;
    for(int i=2;i<n-1;++i) {
      const Eigen::Vector3d delta=p.col(i+1)-p.col(i);const double length=delta.norm();
      const double limit=speed*dt;
      if(length<=limit+1e-12)continue;
      const int a=free(i)?1:0,b=free(i+1)?1:0;if(!a&&!b)return false;
      const Eigen::Vector3d excess=delta*(1.-limit/length);
      if(a)p.col(i)+=excess/double(a+b);
      if(b)p.col(i+1)-=excess/double(a+b);
    }
    for(int i=0;i<n-2;++i) {
      const Eigen::Vector3d delta=p.col(i)-2.*p.col(i+1)+p.col(i+2);
      const double length=delta.norm(),limit=acceleration*dt*dt;
      if(length<=limit+1e-12)continue;
      const int coefficients[]={1,-2,1};double weights=0.;
      for(int k=0;k<3;++k)if(free(i+k))weights+=coefficients[k]*coefficients[k];
      if(weights==0.)return false;
      const Eigen::Vector3d excess=delta*(1.-limit/length);
      for(int k=0;k<3;++k)if(free(i+k))p.col(i+k)-=excess*(coefficients[k]/weights);
    }
    UniformBspline candidate(p,3,dt);const auto checked=inspectInitialMotionLimits(candidate,speed,acceleration);
    if(checked.valid&&checked.nominal_speed_bound<=speed+1e-9&&checked.acceleration_bound<=acceleration+1e-9) {
      if(!permitted())return false;curve=candidate;return true;
    }
  }
  return false;
}
} // namespace scan_planner
