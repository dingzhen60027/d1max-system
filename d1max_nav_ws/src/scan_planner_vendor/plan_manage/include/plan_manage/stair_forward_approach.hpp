#pragma once
#include <plan_manage/collision_reference.hpp>
#include <plan_manage/planning_support.hpp>

namespace scan_planner {

// On a narrow flight, the route projection is a progress coordinate, not a
// sideways driving goal. Shape only a source-labelled, horizontally straight
// prefix into a forward approach. Literal tread Z may be piecewise varying;
// it is sampled from the unchanged reference, not replaced by a fitted ramp.
// Actual P/V/A are still fixed by the cubic fit;
// this seed neither changes the route/support nor bypasses native collision.
inline std::vector<Eigen::Vector3d> measuredStairForwardApproach(
    const std::vector<Eigen::Vector3d>& reference,double resolution,
    const OccupancyQuery& occupied,const Eigen::Vector3d& velocity,double yaw,
    bool projection_connector,const PlanningSupport* support,const std::string& frame) {
  const std::size_t begin=projection_connector?1:0;
  if(!projection_connector||!support||!support->isStair()||support->source.frame_id!=frame||
     reference.size()<begin+2||!velocity.allFinite()||!std::isfinite(yaw)||
     !std::isfinite(resolution)||resolution<=0.)return reference;
  for(const auto& p:reference)if(!p.allFinite())return reference;
  const auto flight=support->stairFlightAt(reference.front());
  if(!flight||support->stairFlightAt(reference[begin])!=flight)return reference;
  const Eigen::Vector3d delta=reference[begin+1]-reference[begin];
  const double length=delta.head<2>().norm();
  if(length<1e-6)return reference;
  const Eigen::Vector2d tangent=delta.head<2>()/length;
  const Eigen::Vector2d heading(std::cos(yaw),std::sin(yaw));
  if(heading.dot(tangent)<=.5)return reference; // Not a new in-place-turn policy.
  const Eigen::Vector2d offset=(reference.front()-reference[begin]).head<2>();
  const double lateral=std::abs(offset.x()*tangent.y()-offset.y()*tangent.x());
  const double advance=std::clamp(.8+2.*lateral,.8,1.5);
  double walked=0.;Eigen::Vector3d join=reference[begin],exit_direction=delta/length;
  std::vector<Eigen::Vector3d> original_prefix{reference[begin]};
  std::vector<double> prefix_arc{0.};
  std::size_t next=begin+1;
  for(std::size_t i=begin+1;i<reference.size();++i) {
    const Eigen::Vector3d d=reference[i]-reference[i-1];const double xy=d.head<2>().norm();
    if(xy<1e-6||support->stairFlightAt(reference[i])!=flight||
       d.head<2>().dot(tangent)<xy*.995)break;
    const double take=std::min(xy,advance-walked);
    join=reference[i-1]+d*(take/xy);exit_direction=d/xy;
    walked+=take;next=i+(take>=xy-1e-9?1:0);
    original_prefix.push_back(join);prefix_arc.push_back(walked);
    if(walked>=advance-1e-9)break;
  }
  if(walked<.2)return reference;
  Eigen::Vector2d entry=.05*heading;const double speed=velocity.head<2>().norm();
  if(speed>1e-12)entry+=std::max(0.,velocity.head<2>().dot(heading))*velocity.head<2>()/speed;
  entry.normalize();
  const Eigen::Vector3d first(walked*entry.x(),walked*entry.y(),0.);
  const Eigen::Vector3d last(walked*exit_direction.x(),walked*exit_direction.y(),0.);
  const std::array<Eigen::Vector3d,6> controls{reference.front(),reference.front()+first/5.,
    reference.front()+2.*first/5.,join-2.*last/5.,join-last/5.,join};
  double bound=0.;for(std::size_t i=1;i<controls.size();++i)
    bound=std::max(bound,5.*(controls[i]-controls[i-1]).norm());
  const double required=std::ceil(bound/(resolution*.5));
  if(!std::isfinite(required)||required>2048.)return reference;
  const int count=std::max(2,static_cast<int>(required));
  std::vector<Eigen::Vector3d> planar{reference.front()};
  std::vector<double> planar_arc{0.};
  planar.reserve(count+1);planar_arc.reserve(count+1);
  for(int i=1;i<=count;++i) {
    const double u=double(i)/count;auto values=controls;
    for(std::size_t order=1;order<controls.size();++order)
      for(std::size_t j=0;j<controls.size()-order;++j)values[j]=(1.-u)*values[j]+u*values[j+1];
    const Eigen::Vector3d p=i==count?join:values[0];
    planar_arc.push_back(planar_arc.back()+(p-planar.back()).head<2>().norm());
    planar.push_back(p);
  }
  if(planar_arc.back()<1e-9)return reference;
  std::vector<Eigen::Vector3d> shaped{reference.front()};
  shaped.reserve(count+1+reference.size()-next);
  const double start_height_offset=reference.front().z()-original_prefix.front().z();
  for(int i=1;i<=count;++i) {
    const double fraction=planar_arc[i]/planar_arc.back();
    const double arc=walked*fraction;
    const auto upper=std::upper_bound(prefix_arc.begin(),prefix_arc.end(),arc);
    const std::size_t k=std::min(original_prefix.size()-2,
        static_cast<std::size_t>(upper-prefix_arc.begin()-1));
    const double ratio=std::clamp((arc-prefix_arc[k])/(prefix_arc[k+1]-prefix_arc[k]),0.,1.);
    Eigen::Vector3d p=planar[i];
    const double blend=1.-fraction*fraction*fraction*(10.+fraction*(-15.+6.*fraction));
    p.z()=(1.-ratio)*original_prefix[k].z()+ratio*original_prefix[k+1].z()+blend*start_height_offset;
    if(i==count)p=join;
    // Densify in FULL XYZ: a short XY tread edge can have appreciable height.
    // Every returned point and intervening source/occupancy query stays within
    // the same literal flight; neither missing support nor real walls vanish.
    const Eigen::Vector3d a=shaped.back();
    const int pieces=std::max(1,static_cast<int>(std::ceil((p-a).norm()/(resolution*.5))));
    if(shaped.size()+pieces>2049)return reference;
    for(int j=1;j<=pieces;++j) {
      const Eigen::Vector3d point=j==pieces?p:a+(p-a)*(double(j)/pieces);
      if(support->stairFlightAt(point)!=flight||
         !collisionFreeSegment(shaped.back(),point,resolution*.5,occupied))return reference;
      shaped.push_back(point);
    }
  }
  shaped.insert(shaped.end(),reference.begin()+next,reference.end());
  return shaped;
}

inline std::vector<Eigen::Vector3d> measuredReferenceForwardApproach(
    const std::vector<Eigen::Vector3d>& reference,double resolution,
    const OccupancyQuery& occupied,const Eigen::Vector3d& velocity,double yaw,
    bool projection_connector,const PlanningSupport* support,const std::string& frame) {
  const bool flight=projection_connector&&!reference.empty()&&support&&support->source.frame_id==frame&&
      support->stairFlightAt(reference.front()).has_value();
  return flight?measuredStairForwardApproach(reference,resolution,occupied,velocity,yaw,
      projection_connector,support,frame):measuredForwardApproach(reference,resolution,occupied,velocity,yaw,
      projection_connector);
}
} // namespace scan_planner
