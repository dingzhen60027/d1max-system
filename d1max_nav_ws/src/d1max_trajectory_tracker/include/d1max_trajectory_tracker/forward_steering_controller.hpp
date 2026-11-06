#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace d1max_trajectory_tracker {
// Stateless geometry policy; task ownership, curve time and command history
// belong to their existing owners.
inline double smoothUnitInterval(double value) {
  const double v=std::clamp(value,0.,1.);
  return v*v*v*(10.+v*(-15.+6.*v));
}
inline double headingTranslationWeight(double error,double threshold) {
  if(!std::isfinite(error)||!std::isfinite(threshold)||threshold<=0.)return 0.;
  return smoothUnitInterval((threshold-std::abs(error))/(.5*threshold));
}
// Remove only the small error itself. Its derivative is in [0,1]: unlike
// error * a narrow switching weight, this cannot amplify local feedback gain.
// No angular engage/release state or mandatory alignment interval.
inline double softTrackingError(double error,double scale) {
  return scale>0. ? error-scale*std::tanh(error/scale) : error;
}
// The odometry angular vector is expressed in the continuous world frame;
// its Z component is NOT the derivative of the body's horizontal heading
// while pitched. With the body-forward axis f=R*ex and f_dot=omega x f,
// d atan2(f.y,f.x)/dt = omega.z - f.z*(omega.xy dot f.xy)/|f.xy|^2.
// In particular, rolling about a tilted forward axis must not create a yaw
// correction. Keep the measured vector unchanged for diagnostics/rest checks.
inline double projectedBodyHeadingRate(const Eigen::Quaterniond& orientation,
    const Eigen::Vector3d& world_angular_velocity) {
  if(!orientation.coeffs().allFinite()||!world_angular_velocity.allFinite())
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector3d forward=orientation*Eigen::Vector3d::UnitX();
  // Preserve the existing arithmetic exactly for a level forward axis,
  // including roundoff from an otherwise level roll/yaw quaternion.
  if(std::abs(forward.z())<=1e-12)return world_angular_velocity.z();
  // Heading is singular for a vertical forward axis. Regularize only that
  // cone (horizontal projection below 0.1, |pitch| above about 84 degrees):
  // bounded, continuous and tending to the original world-Z damping, with
  // no new pose rejection, mode switch or motion gate. Normal stair attitudes
  // use the exact azimuth derivative above.
  const double horizontal_squared=forward.head<2>().squaredNorm();
  return world_angular_velocity.z()-forward.z()*
    world_angular_velocity.head<2>().dot(forward.head<2>())/
    std::max(horizontal_squared,.01);
}
// A direction behind a forward-only body has two valid turn branches. Choosing
// atan2's shortest branch independently on every new curve makes a rear goal
// alternate left/right near +/-pi. Remember only that turn choice, not an
// accumulated/unwrapped angle: once the direction is in front, ordinary signed
// feedback resumes. The error is therefore bounded by 3*pi/2 and can never
// accumulate a full revolution. This neither gates translation nor changes
// geometry/collision authority; the existing command conditioner owns motion.
class RearTurnIntent {
public:
  double resolve(double error) {
    if(!std::isfinite(error))return error;
    constexpr double pi=3.14159265358979323846;
    error=std::atan2(std::sin(error),std::cos(error));
    if(std::abs(error)<=pi/2.) {side_=0;return error;}
    if(side_==0)side_=error<0.?-1:1;
    if(error*side_<0.)error+=side_*2.*pi;
    return error;
  }
  void reset(){side_=0;}
private:
  int side_{0};
};
// Anticipate only the ACCEPTED reference's bending. The chord starts at the
// measured curve projection, not at the actual body: putting body cross-track
// error here would duplicate the separate soft feedback below. For a circle,
// 2 * det(unit tangent, chord) / |chord|^2 is exactly its signed curvature;
// for a straight reference it is zero regardless of the body's lateral error.
// No curve clock/state is advanced, and a finite tail with no chord retains
// its current geometry instead of introducing a new stop or turn.
inline double previewReferenceCurvature(const Eigen::Vector2d& tangent,
    const Eigen::Vector2d& chord,double current_curvature) {
  if(!tangent.allFinite()||!chord.allFinite()||!std::isfinite(current_curvature)||
     tangent.norm()<1e-9)return std::numeric_limits<double>::quiet_NaN();
  if(chord.squaredNorm()<1e-12)return current_curvature;
  const Eigen::Vector2d direction=tangent.normalized();
  return 2.*(direction.x()*chord.y()-direction.y()*chord.x())/chord.squaredNorm();
}
struct ForwardSteeringSettings {
  double lateral_preview_m=.5,heading_error_scale_rad=.04;
  double yaw_gain=1.2,yaw_damping=.15;
};
struct ForwardSteeringTarget {
  bool valid=false;
  double path_heading=0.,heading_error=0.,heading_weight=0.,forward=0.;
  double curvature=0.,yaw_feedback=0.,yaw_damping=0.,measured_yaw_rate=0.;
  double correction_scale=1.;
  double measured_forward=0.;
  // Preview anticipates the bend, whereas rate damping refers to the current
  // accepted heading derivative. Zero preserves the existing exact reference
  // (resolved entry / full flight / legacy callers).
  double damping_curvature_offset=0.;
  double curvatureYawAtSpeed(double moving_speed)const {
    const double feedforward=curvature*moving_speed;
    const double legacy=(1.+yaw_damping)*feedforward;
    return damping_curvature_offset==0.?legacy:
      legacy+yaw_damping*damping_curvature_offset*moving_speed;
  }
  // A normalized SDK response is not a calibrated SI velocity. Steering a
  // bend must account for how fast the acquired body is actually advancing,
  // rather than waiting for lateral error when it outruns its SI request.
  // The current accepted command supplies startup anticipation, not a future
  // trajectory target. A zero command has no curvature term; reverse MC
  // neither reverses the heading nor requests backward/sideways movement.
  double yawAtForward(double accepted_forward)const {
    // Use the SAME continuous forward-feasibility weight as translation.
    // Merely testing weight>0 kept the full acquired-speed curvature until
    // 89.999 degrees, then removed it at 90 degrees. With opposite curvature
    // and a residual positive command that could reverse the yaw target.
    // cos^2 fades with a zero derivative at the forward/rear boundary; a
    // behind-body path still cannot resurrect curvature while coasting.
    const double moving_speed=accepted_forward>0. ?
      std::max(accepted_forward,std::max(0.,measured_forward))*heading_weight : 0.;
    return curvatureYawAtSpeed(moving_speed)+
      correction_scale*(yaw_feedback-yaw_damping*measured_yaw_rate);
  }
  // A forward-only platform cannot retain full translation and independently
  // clip the turn that makes that translation follow a bend. Scale the whole
  // desired twist when its combined acquired-speed curvature + correction
  // demand exceeds yaw capability. This reduces forward demand continuously,
  // without an alignment mode, angle latch, or changed path geometry. Actual
  // acquired speed cannot be scaled retroactively: its curvature term still
  // applies while translation decelerates, and the existing yaw cap remains.
  // The one command conditioner owns every actual output derivative.
  ForwardSteeringTarget withTwistLimits(double forward_limit,double yaw_limit)const {
    auto result=*this;
    if(!valid||!std::isfinite(measured_forward)||!std::isfinite(forward_limit)||forward_limit<0.||
       !std::isfinite(yaw_limit)||yaw_limit<=0.) {result.valid=false;return result;}
    result.forward=std::min(result.forward,forward_limit);
    const double turn=result.yawAtForward(result.forward);
    if(!std::isfinite(turn)){result.valid=false;return result;}
    const double scale=std::abs(turn)>yaw_limit?yaw_limit/std::abs(turn):1.;
    result.forward*=scale;result.correction_scale*=scale;
    return result;
  }
};
// Unicycle path following at the measured, ordered curve projection. Spatial
// preview sets how gently to recover cross-track error; it is NOT a shortcut
// to a future point or a wall-clock trajectory clock. Curvature supplies the
// ongoing bend; feedback corrects deviation. Forward and turn act together.
inline ForwardSteeringTarget forwardSteeringTarget(const ForwardSteeringSettings& c,
    const Eigen::Vector2d& tangent,const Eigen::Vector2d& cross_error,
    double path_speed,double body_yaw,double body_yaw_rate,double curve_curvature) {
  ForwardSteeringTarget out;
  if(!tangent.allFinite()||tangent.norm()<1e-9||!cross_error.allFinite()||
     !std::isfinite(path_speed)||!std::isfinite(body_yaw)||!std::isfinite(body_yaw_rate)||
     !std::isfinite(curve_curvature)||!std::isfinite(c.lateral_preview_m)||c.lateral_preview_m<=0.||
     !std::isfinite(c.heading_error_scale_rad)||c.heading_error_scale_rad<0.||
     !std::isfinite(c.yaw_gain)||c.yaw_gain<=0.||!std::isfinite(c.yaw_damping)||c.yaw_damping<0.)return out;
  const Eigen::Vector2d direction=tangent.normalized();
  out.path_heading=std::atan2(direction.y(),direction.x());
  const double lateral=direction.x()*cross_error.y()-direction.y()*cross_error.x();
  const double correction=std::atan2(lateral,c.lateral_preview_m);
  const double delta=out.path_heading+correction-body_yaw;
  out.heading_error=std::atan2(std::sin(delta),std::cos(delta));
  const double useful_forward=std::max(0.,std::cos(out.heading_error));
  out.heading_weight=useful_forward*useful_forward;
  out.forward=std::max(0.,path_speed)*out.heading_weight;
  out.curvature=curve_curvature;
  out.yaw_feedback=c.yaw_gain*softTrackingError(out.heading_error,c.heading_error_scale_rad);
  out.yaw_damping=c.yaw_damping;out.measured_yaw_rate=body_yaw_rate;
  out.valid=true;return out;
}
} // namespace d1max_trajectory_tracker
