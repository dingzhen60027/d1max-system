// Offline passive-snapshot replay. No ROS context, node, transport or commands.
// Reuses native observed-ray traversal, raw-state and exact-cylinder helpers.
// Repeating the same cloud only reveals permanently unseen cells. It is NOT
// additional real sensor evidence, a live-map dump, or motion authorization.
#include <plan_env/observed_ray.hpp>
#include <plan_env/voxel_collision.hpp>
#include <iostream>
#include <unordered_map>
#include <array>

using Cell=std::array<int,3>;
struct Hash {std::size_t operator()(const Cell &c) const {
  std::size_t h=0;for(int x:c) h^=std::hash<int>{}(x)+0x9e3779b9+(h<<6)+(h>>2);return h;
}};
struct Vote {int hit=0,miss=0;bool traversed=false;};
Cell key(const Eigen::Vector3i &v){return {v.x(),v.y(),v.z()};}
Cell key(const Eigen::Vector3d &v){return key(Eigen::Vector3i((v/.08).array().floor().cast<int>()));}

int main() {
  std::size_t count;Eigen::Vector3d origin,body;double yaw;
  if (!(std::cin>>count) || !count || count>250000) return 2;
  for(int i=0;i<3;++i) std::cin>>origin[i];
  for(int i=0;i<3;++i) std::cin>>body[i];
  std::cin>>yaw;
  if(!origin.allFinite()||!body.allFinite()||!std::isfinite(yaw)) return 2;
  std::unordered_map<Cell,Vote,Hash> votes;
  std::size_t budget=16000000;
  for(std::size_t i=0;i<count;++i) {
    Eigen::Vector3d p;std::cin>>p.x()>>p.y()>>p.z();
    if(!std::cin || !p.allFinite()) return 2;
    bool hit=true;
    if(!scan_planner::clipObservedRay(origin,{6.,6.,3.2},p,hit)) continue;
    const double length=(p-origin).norm();
    if(length>8.) {p=origin+(p-origin)*(8./length);hit=false;}
    auto &end=votes[key(p)];if(hit) ++end.hit;else ++end.miss;
    if(!scan_planner::visitObservedRay(origin,p,.08,!hit,budget,[&](const Eigen::Vector3i &cell){
      auto &v=votes[key(cell)];if(!v.traversed){v.traversed=true;++v.miss;}
    })) return 3;
  }
  const double minimum=std::log(.12/.88),maximum=std::log(.98/.02),occupied=std::log(.8/.2);
  const auto kernel=scan_planner::conservativeCylinderInflation(.08,.29,.45,.45);
  std::cout<<"{\"mode\":\"offline_repeated_static_snapshot_not_new_observations\",\"cloud_points\":"<<count
      <<",\"ray_steps\":"<<16000000-budget<<",\"touched_voxels\":"<<votes.size()<<",\"results\":[";
  bool comma=false;
  for(int repetitions:{1,3,10}) for(bool exact:{false,true}) {
    std::array<std::size_t,3> counts{{0,0,0}};
    Eigen::Vector3d first_occ=Eigen::Vector3d::Zero(),first_unknown=Eigen::Vector3d::Zero();
    const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
    for(int side:{-1,1}) {
      const Eigen::Vector3d center=body+side*.2*heading;
      const Eigen::Vector3i index=(center/.08).array().floor().cast<int>();
      const auto span=scan_planner::verticalVoxelSpan(center.z(),.45,.45,.08);
      for(const auto &offset:kernel) {
        const Eigen::Vector3i cell=index-offset;
        if(cell.z()<span.low||cell.z()>span.high) continue;
        if(exact&&!scan_planner::cylinderIntersectsVoxelXY(center,cell,.08,.29)) continue;
        auto it=votes.find(key(cell));double odds=minimum-.01;
        if(it!=votes.end()) {
          const double change=it->second.hit>=it->second.miss ? std::log(.85/.15):std::log(.3/.7);
          for(int r=0;r<repetitions;++r) odds=std::clamp(odds+change,minimum,maximum);
        }
        const int state=scan_planner::strictRawVoxelStatus(odds,minimum,occupied);
        if(state==1&&!counts[1]) first_occ=(cell.cast<double>().array()+.5)*.08;
        if(state==2&&!counts[2]) first_unknown=(cell.cast<double>().array()+.5)*.08;
        ++counts[state];
      }
    }
    if(comma)std::cout<<",";comma=true;
    std::cout<<"{\"repetitions\":"<<repetitions<<",\"exact_xy\":"<<(exact?"true":"false")
      <<",\"free\":"<<counts[0]<<",\"occupied\":"<<counts[1]<<",\"unknown\":"<<counts[2]
      <<",\"first_occupied\":["<<first_occ.x()<<","<<first_occ.y()<<","<<first_occ.z()<<"]"
      <<",\"first_unknown\":["<<first_unknown.x()<<","<<first_unknown.y()<<","<<first_unknown.z()<<"]}";
  }
  std::cout<<"]}\n";
}
