#pragma once

#include <plan_manage/planning_support.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <plan_manage/stair_contact_query.hpp>
#include <plan_manage/stair_riser_contact.hpp>
#include <plan_env/grid_map.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <unordered_map>

namespace scan_planner {
// One candidate/reader owns this small cache. The immutable source support may
// be shared, but a solver and a collision reader never mutate the same cache.
// This predicts body attitude, NOT a new ground surface or a free-space mask.
class StairAttitudeQuery {
  struct Key {int x,y,z;bool operator==(const Key& b)const{return x==b.x&&y==b.y&&z==b.z;}};
  struct Hash {std::size_t operator()(const Key& k)const{return std::hash<int>{}(k.x)^
    (std::hash<int>{}(k.y)<<1)^(std::hash<int>{}(k.z)<<2);}};
  std::shared_ptr<const PlanningSupport> support_;
  MeasuredBodyPose measured_;
  bool measured_valid_{false};
  Eigen::Vector3d measured_up_{Eigen::Vector3d::UnitZ()};
  std::unordered_map<Key,std::optional<Eigen::Vector3d>,Hash> normals_;
public:
  StairAttitudeQuery(std::shared_ptr<const PlanningSupport> support,const MeasuredBodyPose& measured)
      :support_(std::move(support)),measured_(measured) {
    measured_valid_=support_&&measured.frame==support_->source.frame_id&&
      measured.position.allFinite()&&measured.orientation.coeffs().allFinite()&&
      std::isfinite(measured.orientation.norm())&&measured.orientation.norm()>1e-9;
    if(measured_valid_) {
      measured_up_=measured.orientation.normalized()*Eigen::Vector3d::UnitZ();
      measured_valid_=measured_up_.z()>0.;
    }
  }
  std::optional<Eigen::Vector2d> operator()(const Eigen::Vector3d& body,double yaw) {
    if(!support_||!support_->isStair())return Eigen::Vector2d::Zero();
    if(!body.allFinite()||!std::isfinite(yaw)||(body.array().abs()>1e5).any())return {};
    std::optional<Eigen::Vector3d> normal;
    if(support_->model()) {
      // An explicit finite model has an analytic normal. Evaluate the actual
      // location, not a cache-cell centre that may belong to another flight or
      // landing. This does not create any observed terrain/contact evidence.
      const auto fit=support_->terrainFit(body);
      if(fit.valid)normal=fit.normal;
    } else {
    constexpr double cell=.05;
    const Key key{int(std::floor(body.x()/cell)),int(std::floor(body.y()/cell)),int(std::floor(body.z()/cell))};
    auto it=normals_.find(key);
    if(it==normals_.end()) {
      // Fixed grid centres make the prediction independent of query ordering.
      const Eigen::Vector3d center((key.x+.5)*cell,(key.y+.5)*cell,(key.z+.5)*cell);
      const auto fit=support_->terrainFit(center);
      std::optional<Eigen::Vector3d> normal=fit.valid?std::optional<Eigen::Vector3d>(fit.normal):std::nullopt;
      if(normals_.size()>=4096)normals_.clear();
      it=normals_.emplace(key,std::move(normal)).first;
    }
    normal=it->second;
    if(!normal) {
      // The cell centre is only a cache representative: quantisation may move
      // it outside the same source-fit bounds while this actual query remains
      // supported. Recheck the literal position with unchanged fit criteria.
      // Do not cache this result for the whole cell: a neighbouring query can
      // legitimately lack support, independent of which one was queried first.
      const auto fit=support_->terrainFit(body);
      if(fit.valid)normal=fit.normal;
    }
    }
    // Insufficient actual stair context retains upright geometry; the fit
    // predicts attitude only, never FREE space or a replacement ground point.
    Eigen::Vector3d up=normal.value_or(Eigen::Vector3d::UnitZ());
    if(measured_valid_) {
      // At the robot use its actual acquired attitude, smoothly joining the
      // terrain prediction over half a metre. No pose/height is moved.
      const double weight=std::clamp(1.-(body-measured_.position).norm()/.5,0.,1.);
      up=((1.-weight)*up+weight*measured_up_).normalized();
    }
    const double nx=std::cos(yaw)*up.x()+std::sin(yaw)*up.y();
    const double ny=-std::sin(yaw)*up.x()+std::cos(yaw)*up.y();
    return Eigen::Vector2d(std::atan2(nx,up.z()),std::atan2(-ny,std::hypot(nx,up.z())));
  }
};

inline GridMap::QueryAttitudeProvider stairAttitudeProvider(
    std::shared_ptr<const PlanningSupport> support,const MeasuredBodyPose& measured,
    bool oriented_box=true) {
  if(!oriented_box||!support||!support->isStair())return {};
  auto query=std::make_shared<StairAttitudeQuery>(std::move(support),measured);
  return [query=std::move(query)](const Eigen::Vector3d& p,double yaw){return (*query)(p,yaw);};
}

// Snapshots are exclusively borrowed by each lane. Always restore a nested
// binding so a flat task/new version cannot inherit a previous stair model.
class StairAttitudeScope {
  GridMap& map_;
  using ContactKey=std::array<double,6>;
  struct ContactHash {
    std::size_t operator()(const ContactKey& key)const {
      std::size_t result=0;
      for(const auto value:key)result^=std::hash<double>{}(value)+0x9e3779b9U+(result<<6)+(result>>2);
      return result;
    }
  };
  // The literal and explicit-model geometric providers must enter and leave
  // together in the map's frame; accepting only clearance's
  // frame would still let foreign terrain tilt the box or move search height.
  static std::shared_ptr<const PlanningSupport> localSupport(
      const GridMap& map,std::shared_ptr<const PlanningSupport> support) {
    return support&&support->isStair()&&support->source.frame_id==map.collisionFrame()
      ?std::move(support):std::shared_ptr<const PlanningSupport>{};
  }
  std::shared_ptr<const PlanningSupport> support_;
  GridMap::QueryAttitudeProvider previous_;
  GridMap::QuerySupportHeightProvider previous_height_;
  GridMap::QueryStairClearanceProvider previous_clearance_;
  GridMap::QueryStairContactProvider previous_contact_;
  GridMap::QueryHardBoundaryProvider previous_boundary_;
  GridMap::QueryHardBoundaryDiagnosticProvider previous_boundary_diagnostic_;
public:
  StairAttitudeScope(GridMap& map,std::shared_ptr<const PlanningSupport> support,
      const MeasuredBodyPose& measured):map_(map),support_(localSupport(map,std::move(support))),previous_(
      map.exchangeQueryAttitudeProvider(stairAttitudeProvider(support_,measured,
          map.planningBodyEnvelope().model==CollisionModel::OrientedBox))),previous_height_(
      map.exchangeQuerySupportHeightProvider(support_?
        GridMap::QuerySupportHeightProvider([support=support_](const Eigen::Vector3d& body)->std::optional<double>{
          const auto fit=support->terrainFit(body);
          return fit.valid?std::optional<double>(body.z()+fit.center_residual):std::nullopt;
        }):GridMap::QuerySupportHeightProvider{})),previous_clearance_(
      map.exchangeQueryStairClearanceProvider((map.stairClearanceConfigured()||map.stairSubvoxelConfigured())&&
        support_?
        GridMap::QueryStairClearanceProvider([support=support_](const Eigen::Vector3d& body){
          return support->stairClearanceAllowed(body);
        }):GridMap::QueryStairClearanceProvider{})) {
    GridMap::QueryStairContactProvider contact;
    if(support_&&map.stairContactConfigured()) {
      auto query=std::make_shared<StairContactQuery>(support_,map.collisionFrame(),map.collisionResolution());
      auto riser=std::make_shared<StairRiserContactQuery>(support_,map.collisionFrame(),map.collisionResolution());
      auto answers=std::make_shared<std::unordered_map<ContactKey,bool,ContactHash>>();
      if(query->valid())contact=[query,riser,answers](const Eigen::Vector3d& lo,const Eigen::Vector3d& hi){
        // One exclusively owned reader/scope, immutable source, exact ALL-hit
        // bounds. An endpoint extending the union changes this key, even in
        // the same octant. An interior endpoint was already covered by the
        // whole-AABB proof. Different curves/readers never share answers.
        const ContactKey key{{lo.x(),lo.y(),lo.z(),hi.x(),hi.y(),hi.z()}};
        const auto found=answers->find(key);
        if(found!=answers->end())return found->second;
        const bool matched=query->matchesObservedBounds(lo,hi)||riser->matchesObservedBounds(lo,hi);
        if(answers->size()>=4096)answers->clear();
        answers->emplace(key,matched);return matched;
      };
    }
    previous_contact_=map.exchangeQueryStairContactProvider(std::move(contact));
    GridMap::QueryHardBoundaryProvider boundaries;
    if(support_&&support_->model())boundaries=[support=support_](const Eigen::Vector3d& p,
        double yaw,double pitch,double roll,const BodyEnvelope& body){
      return support->model()->hardCollision(p,yaw,pitch,roll,body);
    };
    previous_boundary_=map.exchangeQueryHardBoundaryProvider(std::move(boundaries));
    GridMap::QueryHardBoundaryDiagnosticProvider diagnostic;
    if(support_&&support_->model())diagnostic=[support=support_,frame=map.collisionFrame()](const Eigen::Vector3d& p,
        double yaw,double pitch,double roll,const BodyEnvelope& body){
      auto context=nlohmann::json::parse(support->model()->describeHardCollision(p,yaw,pitch,roll,body));
      context["query_frame"]=StairModelGeometry::boundedDiagnosticId(frame);
      return StairModelGeometry::diagnosticDump(context);
    };
    previous_boundary_diagnostic_=map.exchangeQueryHardBoundaryDiagnosticProvider(std::move(diagnostic));
  }
  ~StairAttitudeScope(){
    map_.exchangeQueryHardBoundaryDiagnosticProvider(std::move(previous_boundary_diagnostic_));
    map_.exchangeQueryHardBoundaryProvider(std::move(previous_boundary_));
    map_.exchangeQueryStairContactProvider(std::move(previous_contact_));
    map_.exchangeQueryStairClearanceProvider(std::move(previous_clearance_));
    map_.exchangeQuerySupportHeightProvider(std::move(previous_height_));
    map_.exchangeQueryAttitudeProvider(std::move(previous_));
  }
  StairAttitudeScope(const StairAttitudeScope&)=delete;
  StairAttitudeScope& operator=(const StairAttitudeScope&)=delete;
};
} // namespace scan_planner
