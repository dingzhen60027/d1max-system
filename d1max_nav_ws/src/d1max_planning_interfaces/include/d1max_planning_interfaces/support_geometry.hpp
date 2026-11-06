#pragma once
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>

namespace d1max_planning_interfaces {
// Transport shape/identity only. Native SCAN is still the geometry authority;
// this cannot certify observations, collision clearance or physical ability.
inline bool hasParametricStairGeometry(const msg::SupportReference& s) {
  const auto digest=[](const std::string& h) {
    return h.size()==64&&std::all_of(h.begin(),h.end(),[](char c){return
      (c>='0'&&c<='9')||(c>='a'&&c<='f');});
  };
  const auto point=[](const auto& p) {return std::isfinite(p.x)&&std::isfinite(p.y)&&
    std::isfinite(p.z)&&std::max({std::abs(p.x),std::abs(p.y),std::abs(p.z)})<=1e5;};
  const auto pose=[&](const auto& p) {
    const auto& q=p.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return point(p.position)&&std::isfinite(norm)&&std::abs(norm-1.)<=1e-6;
  };
  if(!digest(s.stair_model_hash)||!digest(s.support_hash)||!digest(s.support_map_sha256)||
     s.frame_id.empty()||s.stair_model_surfaces.empty()||s.stair_model_surfaces.size()>64||
     s.stair_model_obstacles.size()>256)return false;
  std::set<std::string> ids;
  for(const auto& f:s.stair_model_surfaces) {
    if(f.surface_id.empty()||!ids.insert(f.surface_id).second||!pose(f.pose)||
       // Discrete step arrays are reserved by the wire schema, but are not
       // implemented by the current native surface query. Never silently
       // reinterpret a step sequence as its grade/intercept envelope.
       (f.kind!="continuous_support_envelope"&&f.kind!="landing")||
       f.polygon.size()<3||f.polygon.size()>64||!std::isfinite(f.support_grade)||
       std::abs(f.support_grade)>1.||!std::isfinite(f.support_intercept_m)||
       !std::isfinite(f.height_association_m)||f.height_association_m<=0.||f.height_association_m>.5||
       !std::isfinite(f.contact_tolerance_m)||f.contact_tolerance_m<0.||f.contact_tolerance_m>.05||
       (f.kind=="landing"&&f.support_grade!=0.))return false;
    double area=0.;
    for(std::size_t i=0;i<f.polygon.size();++i) {
      const auto& a=f.polygon[i];const auto& b=f.polygon[(i+1)%f.polygon.size()];
      if(!point(a)||std::abs(a.z)>1e-9||std::hypot(a.x-b.x,a.y-b.y)<1e-9)return false;
      area+=a.x*b.y-a.y*b.x;
      for(const auto& c:f.polygon)
        if((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)<-1e-8)return false;
    }
    if(!(area>1e-9))return false;
    if(!f.tread_stations_m.empty()||!f.tread_heights_m.empty())return false;
  }
  ids.clear();
  for(const auto& b:s.stair_model_obstacles) {
    if(b.structure_id.empty()||!ids.insert(b.structure_id).second||!pose(b.pose)||!point(b.half_extents)||
       std::min({b.half_extents.x,b.half_extents.y,b.half_extents.z})<=0.||
       std::max({b.half_extents.x,b.half_extents.y,b.half_extents.z})>10.)return false;
  }
  return true;
}
inline bool supportGeometryAvailable(const msg::SupportReference& s) {
  return s.verified||hasParametricStairGeometry(s);
}
} // namespace d1max_planning_interfaces
