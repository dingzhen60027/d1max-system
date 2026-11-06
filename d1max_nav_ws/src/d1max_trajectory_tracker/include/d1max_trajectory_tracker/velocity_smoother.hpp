#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Core>

namespace d1max_trajectory_tracker {
// Conservative jerk-limited command stopping distance. First shed any
// POSITIVE acceleration to zero; then use a triangular/trapezoidal S stop
// ending at both zero velocity and zero acceleration. Negative acceleration
// is not credited as extra free braking distance. This is not a plant bound.
inline double jerkStoppingDistance(double speed,double acceleration,double max_acceleration,double max_jerk) {
  speed=std::max(0.,speed);acceleration=std::clamp(acceleration,0.,max_acceleration);
  if(max_acceleration<=0.)return speed==0.&&acceleration==0.?0.:std::numeric_limits<double>::infinity();
  if(max_jerk<=0.)return speed*speed/(2.*max_acceleration);
  const double t=acceleration/max_jerk;
  const double initial=speed*t+acceleration*t*t/3.;
  speed+=acceleration*acceleration/(2.*max_jerk);
  const double stop_time=speed<=max_acceleration*max_acceleration/max_jerk?
    2.*std::sqrt(speed/max_jerk):speed/max_acceleration+max_acceleration/max_jerk;
  return initial+.5*speed*stop_time;
}
inline double jerkBrakingSpeedLimit(double distance,double acceleration,double max_acceleration,
    double max_jerk,double ceiling) {
  double low=0.,high=ceiling;
  for(int i=0;i<32;++i) {
    const double mid=.5*(low+high);
    if(jerkStoppingDistance(mid,acceleration,max_acceleration,max_jerk)<=distance)low=mid;
    else high=mid;
  }
  return low;
}
struct JerkStopStep {double velocity,acceleration;};
// Finite S stop (jerk down, optional constant deceleration, jerk up). Re-solving
// from the carried state yields the same time-based plan across timer jitter.
inline JerkStopStep jerkStopStep(double velocity,double acceleration,double max_acceleration,double max_jerk,double dt) {
  if(dt<=0.)return {velocity,acceleration};
  if(velocity<=1e-12)return {0.,0.};
  acceleration=std::clamp(acceleration,-max_acceleration,max_acceleration);
  if(acceleration<0.&&velocity<acceleration*acceleration/(2.*max_jerk)) {
    // A contracted safety envelope may already make a soft jerk stop
    // unreachable. Zero velocity has priority over running backwards.
    const double h=std::min(dt,-acceleration/max_jerk);
    const double next=velocity+acceleration*h+.5*max_jerk*h*h;
    return next<=0.?JerkStopStep{0.,0.}:JerkStopStep{next,acceleration+max_jerk*h};
  }
  const double peak=std::min(max_acceleration,std::sqrt(max_jerk*velocity+.5*acceleration*acceleration));
  const double down=std::max(0.,(acceleration+peak)/max_jerk);
  const double dwell=std::max(0.,(velocity+.5*acceleration*acceleration/max_jerk-peak*peak/max_jerk)/peak);
  const double up=peak/max_jerk;
  if(dt+1e-12>=down+dwell+up)return {0.,0.};
  double left=dt;
  const double h1=std::min(left,down);velocity+=acceleration*h1-.5*max_jerk*h1*h1;
  acceleration-=max_jerk*h1;left-=h1;
  const double h2=std::min(left,dwell);velocity-=peak*h2;left-=h2;
  if(left>0.){velocity+=acceleration*left+.5*max_jerk*left*left;acceleration+=max_jerk*left;}
  return {std::max(0.,velocity),acceleration};
}
// Stateful second-order command shaping, not a measured robot acceleration.
// Real elapsed time is subdivided for consistent 10--50 Hz/jittered callers.
// Collision speed/acceleration bounds are applied by the caller AFTER shaping;
// they take precedence if a newly contracted envelope cannot respect jerk.
template<int N>
Eigen::Matrix<double,N,1> smoothVelocityTarget(
    const Eigen::Matrix<double,N,1>& target,
    const Eigen::Matrix<double,N,1>& previous,
    Eigen::Matrix<double,N,1>& acceleration,
    double max_acceleration,double max_jerk,double dt) {
  if(dt<=0.)return previous;
  if(max_jerk<=0.)return target;
  if(target.norm()<=1e-12&&previous.norm()>1e-12) {
    const Eigen::Matrix<double,N,1> direction=previous/previous.norm();
    const auto stop=jerkStopStep(previous.norm(),acceleration.dot(direction),max_acceleration,max_jerk,dt);
    acceleration=direction*stop.acceleration;return direction*stop.velocity;
  }
  const double omega=max_jerk/max_acceleration;
  const int count=std::max(1,static_cast<int>(std::ceil(dt/.005)));
  const double h=dt/count;
  Eigen::Matrix<double,N,1> velocity=previous;
  for(int i=0;i<count;++i) {
    Eigen::Matrix<double,N,1> jerk=omega*omega*(target-velocity)-2.*omega*acceleration;
    if(jerk.norm()>max_jerk)jerk*=max_jerk/jerk.norm();
    Eigen::Matrix<double,N,1> next=acceleration+jerk*h;
    if(next.norm()>max_acceleration)next*=max_acceleration/next.norm();
    velocity+=(acceleration+next)*(.5*h);
    acceleration=next;
  }
  return velocity;
}
} // namespace d1max_trajectory_tracker
