#pragma once
#include <bspline_opt/spline_heading_contract.hpp>

namespace scan_planner {
// Resolve heading only for a SHORT backwards acquired entry followed by
// forward geometry on this same curve. Position/velocity and curve time are
// never altered. Normal bends and a route genuinely behind the robot retain
// their tangent policy. The 0.4 m geometric horizon is shared by Native and
// control, not the controller's lateral-feedback gain or an execution clock.
// At most two passes over the EXISTING bounded Native sampling lattice;
// one immutable preparation table, no per-query allocation, recursive curve
// scan, new sampling timer or map.
template<class PositionAt,class VelocityAt,class Allowed>
bool resolveForwardEntryHeading(SplineHeadingContract& contract,double duration,
    std::uint32_t intervals,PositionAt position,VelocityAt velocity,Allowed allowed) {
  contract.forward_entry_end=-1.;contract.forward_entry_yaw=contract.measured_yaw;
  contract.forward_entry_profile.reset();
  if(!contract.align_then_forward||contract.policy!=BodyHeadingPolicy::PathTangent)return true;
  if(!validMeasuredHeadingContract(contract)||!std::isfinite(duration)||duration<=0.||
     duration>120.||!intervals||intervals>49999||!allowed())return false;
  const Eigen::Vector3d origin=position(0.),initial_velocity=velocity(0.);
  if(!origin.allFinite()||!initial_velocity.allFinite())return false;
  const Eigen::Vector2d head(std::cos(contract.measured_yaw),std::sin(contract.measured_yaw));
  if(initial_velocity.head<2>().norm()<contract.minimum_direction_speed||
     initial_velocity.head<2>().dot(head)>=0.)return true;
  constexpr double horizon=.4;
  auto profile=std::make_shared<ForwardEntryHeadingProfile>();
  profile->seed_yaw=contract.measured_yaw;
  // Most entries occupy only a few hundred nodes, even on a long curve.
  // Grow only as the bounded 0.4 m domain requires, not to the full trajectory.
  profile->knots.reserve(std::min<std::uint32_t>(intervals+1,256));
  profile->knots.push_back({0.,0.,initial_velocity.head<2>().norm(),contract.measured_yaw});
  Eigen::Vector3d previous=origin,forward_point=origin;
  double arc=0.,end_time=0.;
  for(std::uint32_t i=1;i<=intervals;++i) {
    if(!allowed())return false;
    const double time=duration*static_cast<double>(i)/intervals;
    const Eigen::Vector3d next=position(time);if(!next.allFinite())return false;
    const double ds=(next-previous).head<2>().norm();
    end_time=time;forward_point=next;
    if(arc+ds>=horizon) {
      const double previous_time=duration*static_cast<double>(i-1)/intervals;
      end_time=previous_time+(time-previous_time)*(horizon-arc)/ds;
      forward_point=position(end_time);if(!forward_point.allFinite())return false;
      arc+=(forward_point-previous).head<2>().norm();
      const Eigen::Vector3d v=velocity(end_time);if(!v.allFinite())return false;
      profile->knots.push_back({end_time,arc,v.head<2>().norm(),0.});
      break;
    }
    arc+=ds;previous=next;
    const Eigen::Vector3d v=velocity(time);if(!v.allFinite())return false;
    profile->knots.push_back({time,arc,v.head<2>().norm(),0.});
  }
  const Eigen::Vector2d direction=(forward_point-origin).head<2>();
  if(direction.norm()<.001||direction.dot(head)<=0.)return true;
  // A short local goal may stop before the spatial horizon. Its trailing
  // zero/low-speed derivatives have no new direction; do not let them erase
  // an already recovered forward entry. End the blend at the last reliable
  // derivative, then the ordinary shared low-speed heading memory retains it.
  // A genuinely backwards (nonzero) suffix is NOT trimmed or reinterpreted.
  while(profile->knots.size()>2&&
        profile->knots.back().arc_rate<contract.minimum_direction_speed)
    profile->knots.pop_back();
  end_time=profile->knots.back().time;
  // Preserve the measured head through the reverse/near-stationary part.
  // In particular, the first barely-positive vx with vy=.03 is not a new
  // measured head (it may point almost 90 degrees sideways). Blend only after
  // the final non-forward node, and rejoin the unchanged tangent by the SAME
  // existing spatial horizon. Nothing outside this exceptional prefix changes.
  std::size_t first_forward=1;
  double yaw=contract.measured_yaw;
  for(std::size_t i=0;i<profile->knots.size();++i) {
    if(!allowed())return false;
    const double time=profile->knots[i].time;
    const Eigen::Vector3d v=velocity(time);if(!v.allFinite())return false;
    if(v.head<2>().norm()>=contract.minimum_direction_speed&&
       v.head<2>().dot(direction)>0.&&v.head<2>().dot(head)>0.) {
      const double raw=std::atan2(v.y(),v.x());
      yaw+=std::atan2(std::sin(raw-yaw),std::cos(raw-yaw));
    } else first_forward=i+1;
    profile->knots[i].tangent_yaw=yaw;
  }
  if(first_forward>=profile->knots.size()-1)return true;
  profile->blend_start_arc=profile->knots[first_forward].arc;
  profile->blend_end_arc=profile->knots.back().arc;
  if(profile->blend_end_arc-profile->blend_start_arc<=1e-9)return true;
  // Fritsch-Carlson slope limiting on shared nodes keeps s(t) C1 and monotone
  // even across an acquired zero-speed cusp. A following interval may reduce
  // a shared slope further, which cannot break monotonicity of the prior one.
  for(std::size_t i=1;i<profile->knots.size();++i) {
    if(!allowed())return false;
    auto& a=profile->knots[i-1];auto& b=profile->knots[i];
    const double secant=(b.arc-a.arc)/(b.time-a.time);
    if(secant<=1e-15) {a.arc_rate=0.;b.arc_rate=0.;continue;}
    const double alpha=a.arc_rate/secant,beta=b.arc_rate/secant;
    const double radius=std::hypot(alpha,beta);
    if(radius>3.) {const double scale=3./radius;a.arc_rate*=scale;b.arc_rate*=scale;}
  }
  contract.forward_entry_end=end_time;
  contract.forward_entry_profile=std::move(profile);
  return true;
}
}  // namespace scan_planner
