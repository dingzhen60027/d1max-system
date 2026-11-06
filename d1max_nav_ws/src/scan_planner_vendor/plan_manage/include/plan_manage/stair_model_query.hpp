#pragma once

#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <d1max_planning_interfaces/support_geometry.hpp>
#include <plan_env/voxel_collision.hpp>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace scan_planner {
// Explicit map-authoring prior, never an array of measured lidar returns.
// The continuous support envelope governs the body/search height and normal.
// It is deliberately NOT an occupancy/contact exemption below a sloping plane.
class StairModelGeometry {
public:
  struct Sample {
    Eigen::Vector3d ground,body,normal;
    std::string surface_id,segment_id,kind;
  };
private:
  struct Surface {
    std::string id,segment,kind;
    Eigen::Vector3d origin,normal;
    Eigen::Matrix3d rotation;
    std::vector<Eigen::Vector2d> polygon;
    double grade,intercept,height_association;
  };
  struct Obstacle {
    std::string id;
    Eigen::Vector3d center,half;
    Eigen::Matrix3d rotation;
  };
  std::vector<Surface> surfaces_;
  std::vector<Obstacle> obstacles_;
  bool present_{false},valid_{false};
  std::string frame_,model_hash_,support_hash_,support_reference_id_;
  double body_reference_height_{0.};
  static bool bounded(const Eigen::Vector3d& p) {
    return p.allFinite()&&(p.array().abs()<=1e5).all();
  }
  static bool hash(const std::string& s) {
    return s.size()==64&&std::all_of(s.begin(),s.end(),[](unsigned char c){return std::isxdigit(c);});
  }
  static bool pose(const geometry_msgs::msg::Pose& p,Eigen::Vector3d& origin,Eigen::Matrix3d& rotation) {
    origin={p.position.x,p.position.y,p.position.z};
    Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
    if(!bounded(origin)||!q.coeffs().allFinite()||std::abs(q.norm()-1.)>1e-6)return false;
    rotation=q.normalized().toRotationMatrix();return true;
  }
  static double cross(const Eigen::Vector2d& a,const Eigen::Vector2d& b) {return a.x()*b.y()-a.y()*b.x();}
  static bool onSegment(const Eigen::Vector2d& p,const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    return std::abs(cross(b-a,p-a))<=1e-10&& (p-a).dot(p-b)<=1e-10;
  }
  static bool contains(const std::vector<Eigen::Vector2d>& polygon,const Eigen::Vector2d& point) {
    bool inside=false;
    for(std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
      const auto& a=polygon[j];const auto& b=polygon[i];
      if(onSegment(point,a,b))return true;
      if((a.y()>point.y())!=(b.y()>point.y())&&
          point.x()<(b.x()-a.x())*(point.y()-a.y())/(b.y()-a.y())+a.x())inside=!inside;
    }
    return inside;
  }
  static bool polygonValid(const std::vector<Eigen::Vector2d>& polygon) {
    double area=0.;
    for(std::size_t i=0;i<polygon.size();++i) {
      const auto& a=polygon[i];const auto& b=polygon[(i+1)%polygon.size()];
      if((a-b).squaredNorm()<1e-16)return false;
      area+=cross(a,b);
      for(std::size_t j=i+1;j<polygon.size();++j) {
        if(j==i+1||(i==0&&j+1==polygon.size()))continue;
        const auto& c=polygon[j];const auto& d=polygon[(j+1)%polygon.size()];
        if(onSegment(a,c,d)||onSegment(b,c,d)||onSegment(c,a,b)||onSegment(d,a,b)||
          (cross(b-a,c-a)*cross(b-a,d-a)<0.&&cross(d-c,a-c)*cross(d-c,b-c)<0.))return false;
      }
    }
    return std::isfinite(area)&&std::abs(area)>1e-10;
  }
  static bool boxesIntersect(const Eigen::Vector3d& a,const Eigen::Matrix3d& ra,const Eigen::Vector3d& ha,
      const Eigen::Vector3d& b,const Eigen::Matrix3d& rb,const Eigen::Vector3d& hb) {
    const auto delta=b-a;
    const auto separated=[&](const Eigen::Vector3d& axis) {
      return std::abs(delta.dot(axis))>
        (ra.transpose()*axis).cwiseAbs().dot(ha)+(rb.transpose()*axis).cwiseAbs().dot(hb)+1e-12;
    };
    for(int i=0;i<3;++i)if(separated(ra.col(i))||separated(rb.col(i)))return false;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)if(separated(ra.col(i).cross(rb.col(j))))return false;
    return true;
  }
public:
  // Cap by bytes without cutting a valid UTF-8 code point. Malformed source
  // bytes are handled by diagnosticDump below; labels must never turn a
  // collision-reporting path into a serialization exception.
  static std::string boundedDiagnosticId(const std::string& value) {
    std::size_t end=std::min<std::size_t>(value.size(),256);
    if(end<value.size())while(end>0 &&
        (static_cast<unsigned char>(value[end])&0xc0U)==0x80U)--end;
    return value.substr(0,end);
  }
  static std::string diagnosticDump(const nlohmann::json& value) {
    return value.dump(-1,' ',false,nlohmann::json::error_handler_t::replace);
  }
  explicit StairModelGeometry(const d1max_planning_interfaces::msg::SupportReference& source) {
    frame_=source.frame_id;model_hash_=source.stair_model_hash;support_hash_=source.support_hash;
    support_reference_id_=source.support_reference_id;body_reference_height_=source.body_reference_height_m;
    present_=!source.stair_model_hash.empty()||!source.stair_model_surfaces.empty()||!source.stair_model_obstacles.empty();
    if(!present_)return;
    if(!d1max_planning_interfaces::hasParametricStairGeometry(source))return;
    if(!hash(source.stair_model_hash)||!hash(source.support_hash)||!hash(source.support_map_sha256)||
        source.frame_id.empty()||source.support_reference_id.empty()||
        !std::isfinite(source.body_reference_height_m)||source.body_reference_height_m<.2||source.body_reference_height_m>.8||
        source.stair_model_surfaces.empty()||source.stair_model_surfaces.size()>64||source.stair_model_obstacles.size()>256)return;
    std::set<std::string> ids;
    for(const auto& s:source.stair_model_surfaces) {
      Surface surface;surface.id=s.surface_id;surface.segment=s.segment_id;surface.kind=s.kind;
      if(s.surface_id.empty()||s.segment_id.empty()||!ids.insert(s.surface_id).second||
          (s.kind!="standard_steps"&&s.kind!="continuous_support_envelope"&&s.kind!="landing")||
          !pose(s.pose,surface.origin,surface.rotation)||s.polygon.size()<3||s.polygon.size()>64||
          !std::isfinite(s.support_grade)||!std::isfinite(s.support_intercept_m)||
          !std::isfinite(s.contact_tolerance_m)||s.contact_tolerance_m<0.||s.contact_tolerance_m>.05||
          !std::isfinite(s.height_association_m)||s.height_association_m<=0.||s.height_association_m>.5||
          std::abs(s.support_grade)>1.||std::abs(s.support_intercept_m)>1e5)return;
      if(s.kind=="landing"&&s.support_grade!=0.)return;
      if(s.kind=="standard_steps") {
        if(s.tread_heights_m.empty()||s.tread_heights_m.size()>255||s.tread_stations_m.size()!=s.tread_heights_m.size()+1)return;
        for(std::size_t i=0;i<s.tread_stations_m.size();++i)
          if(!std::isfinite(s.tread_stations_m[i])||std::abs(s.tread_stations_m[i])>1e5||
              (i&&s.tread_stations_m[i]<=s.tread_stations_m[i-1]))return;
        for(const auto h:s.tread_heights_m)if(!std::isfinite(h)||std::abs(h)>1e5)return;
      } else if(!s.tread_heights_m.empty()||!s.tread_stations_m.empty())return;
      for(const auto& p:s.polygon) {
        if(!bounded({p.x,p.y,p.z})||p.z!=0.)return;
        surface.polygon.emplace_back(p.x,p.y);
      }
      if(!polygonValid(surface.polygon))return;
      surface.grade=s.support_grade;surface.intercept=s.support_intercept_m;
      surface.height_association=s.height_association_m;
      surface.normal=surface.rotation*Eigen::Vector3d(-surface.grade,0.,1.).normalized();
      if(surface.normal.z()<=1e-6)return; // not a support graph in this gravity frame
      surfaces_.push_back(std::move(surface));
    }
    ids.clear();
    for(const auto& s:source.stair_model_obstacles) {
      Obstacle obstacle;obstacle.id=s.structure_id;
      obstacle.half={s.half_extents.x,s.half_extents.y,s.half_extents.z};
      if(s.structure_id.empty()||!ids.insert(s.structure_id).second||!pose(s.pose,obstacle.center,obstacle.rotation)||
          !bounded(obstacle.half)||(obstacle.half.array()<=0.).any())return;
      obstacles_.push_back(std::move(obstacle));
    }
    valid_=true;
  }
  bool present()const{return present_;}
  bool valid()const{return valid_;}
  bool hasObstacles()const{return valid_&&!obstacles_.empty();}
private:
  // Project the body plane vertically in the query frame. Its offset from
  // ground follows the model's transformed map-up, not an invented odom-Z.
  // Overlapping floors are selected by expected height. Equal-distance,
  // different-height alternatives are ambiguous, never extrapolated together.
  std::optional<Sample> sampleShifted(const Eigen::Vector3d& expected_body,double height)const {
    if(!valid_||!bounded(expected_body)||!std::isfinite(height)||height<0.||height>1.)return {};
    std::optional<Sample> result;double distance=INFINITY;bool ambiguous=false;
    for(const auto& s:surfaces_) {
      const Eigen::Vector3d offset=s.rotation.col(2)*height;
      const Eigen::Vector3d expected_ground=expected_body-offset;
      const Eigen::Vector3d n=s.rotation*Eigen::Vector3d(-s.grade,0.,1.);
      // Association is explicit map geometry, in model-up height units. A
      // finite XY patch must not pull another storey onto its support plane.
      if(std::abs(n.dot(expected_ground-s.origin)-s.intercept)>s.height_association)continue;
      Eigen::Vector3d ground=expected_ground;
      ground.z()=(s.intercept+n.dot(s.origin)-n.x()*ground.x()-n.y()*ground.y())/n.z();
      const Eigen::Vector3d local=s.rotation.transpose()*(ground-s.origin);
      if(!contains(s.polygon,local.head<2>()))continue;
      const Eigen::Vector3d body=ground+offset;
      const double d=std::abs(body.z()-expected_body.z());
      if(!result||d<distance-1e-10) {
        result=Sample{ground,body,s.normal,s.id,s.segment,s.kind};distance=d;ambiguous=false;
      } else if(std::abs(d-distance)<=1e-10) {
        if(std::abs(result->body.z()-body.z())>1e-8)ambiguous=true;
        else if(s.kind=="landing")result=Sample{ground,body,s.normal,s.id,s.segment,s.kind};
      }
    }
    return ambiguous?std::nullopt:result;
  }
public:
  std::optional<Sample> sample(const Eigen::Vector3d& expected_ground)const {
    return sampleShifted(expected_ground,0.);
  }
  std::optional<Sample> sampleBody(const Eigen::Vector3d& expected_body,double body_height)const {
    return sampleShifted(expected_body,body_height);
  }
  bool hardCollision(const Eigen::Vector3d& position,double yaw,double pitch,double roll,const BodyEnvelope& body)const {
    if(!valid_||obstacles_.empty())return false;
    validateBodyEnvelope(body);
    if(body.model!=CollisionModel::OrientedBox||!bounded(position)||!std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(roll))return true;
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d center=position+rotation.col(2)*(body.above-body.below)*.5;
    const Eigen::Vector3d half(body.length*.5,body.width*.5,(body.above+body.below)*.5);
    for(const auto& obstacle:obstacles_)
      if(boxesIntersect(center,rotation,half,obstacle.center,obstacle.rotation,obstacle.half))return true;
    return false;
  }
  // Already-transformed geometry in the collision/query frame. Recording the
  // exact poses avoids needing an unrecorded live anchor to reproduce a model
  // intersection. Diagnostic only: no changed hard-boundary semantics.
  std::string describeHardCollision(const Eigen::Vector3d& position,double yaw,double pitch,double roll,
      const BodyEnvelope& body)const {
    using Json=nlohmann::json;
    const auto v=[](const Eigen::Vector3d& p){return Json::array({p.x(),p.y(),p.z()});};
    const auto matrix=[](const Eigen::Matrix3d& r){return Json::array({
      Json::array({r(0,0),r(0,1),r(0,2)}),Json::array({r(1,0),r(1,1),r(1,2)}),
      Json::array({r(2,0),r(2,1),r(2,2)})});};
    bool truncated=false;
    const auto id=[&](const std::string& value){truncated=truncated||value.size()>256;return boundedDiagnosticId(value);};
    Json result={{"schema",1},{"valid",valid_},{"support_frame",id(frame_)},
      {"model_hash",id(model_hash_)},{"support_hash",id(support_hash_)},
      {"support_reference_id",id(support_reference_id_)},{"body_reference_height",body_reference_height_}};
    if(!valid_||!bounded(position)||!std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(roll))
      return diagnosticDump(result);
    validateBodyEnvelope(body);
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d center=position+rotation.col(2)*(body.above-body.below)*.5;
    const Eigen::Vector3d half(body.length*.5,body.width*.5,(body.above+body.below)*.5);
    result["query"]={{"xyz",v(position)},{"yaw_pitch_roll",{yaw,pitch,roll}},
      {"body_length_width_below_above",{body.length,body.width,body.below,body.above}},
      {"center",v(center)},{"half_extents",v(half)},{"rotation_rows",matrix(rotation)}};
    result["structures"]=Json::array();std::size_t matches=0;
    constexpr std::size_t max_structures=4;
    for(const auto& obstacle:obstacles_) {
      if(!boxesIntersect(center,rotation,half,obstacle.center,obstacle.rotation,obstacle.half))continue;
      if(matches++<max_structures)result["structures"].push_back({{"structure_id",id(obstacle.id)},
        {"center",v(obstacle.center)},{"half_extents",v(obstacle.half)},
        {"rotation_rows",matrix(obstacle.rotation)}});
    }
    result["structures_total"]=obstacles_.size();result["intersections_total"]=matches;
    result["intersections_omitted"]=matches-result["structures"].size();
    const auto selected=sampleBody(position,body_reference_height_);
    result["support_sample"]=nullptr;
    if(selected)for(const auto& s:surfaces_)if(s.id==selected->surface_id) {
      Json polygon=Json::array();for(const auto& p:s.polygon)polygon.push_back({p.x(),p.y()});
      result["support_sample"]={{"surface_id",id(s.id)},{"segment_id",id(s.segment)},{"kind",id(s.kind)},
        {"origin",v(s.origin)},{"rotation_rows",matrix(s.rotation)},{"normal",v(s.normal)},
        {"grade",s.grade},{"intercept",s.intercept},{"height_association",s.height_association},
        {"polygon",std::move(polygon)},{"ground",v(selected->ground)},{"body",v(selected->body)}};
      break; // One selected finite surface; at most the validated 64 vertices.
    }
    result["ids_truncated"]=truncated;return diagnosticDump(result);
  }
};
} // namespace scan_planner
