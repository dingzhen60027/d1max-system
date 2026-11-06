#pragma once
#include <plan_manage/motion_sweep.hpp>
#include <d1max_planning_interfaces/support_regions.hpp>
#include <plan_manage/stair_model_query.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace scan_planner {
struct TerrainFit {
  bool valid{false};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  std::size_t samples{0};
  double rms{NAN},minor_variance{NAN},center_residual{NAN};
};
// Source geometry for SCAN candidate construction, NOT a motion/capability
// certificate. The historical MotionSupport remains a flat, observed-motion
// contract; stair provenance must not be squeezed into its slope/step limits.
class PlanningSupport {
  using Point=geometry_msgs::msg::Point;
  struct Key {
    int x,y,z;
    bool operator==(const Key& b)const {return x==b.x&&y==b.y&&z==b.z;}
  };
  struct Hash {
    std::size_t operator()(const Key& k)const {
      return std::hash<int>{}(k.x)^(std::hash<int>{}(k.y)<<1)^(std::hash<int>{}(k.z)<<2);
    }
  };
  std::unordered_map<Key,std::vector<Point>,Hash> bins_;
  std::unordered_map<Key,std::vector<std::size_t>,Hash> terrain_bins_;
  std::vector<bool> stair_owned_;
  std::optional<MotionSupport> flat_;
  std::shared_ptr<const StairModelGeometry> model_;
  bool literal_valid_{false};
  static Key key(double x,double y,double z) {
    return {int(std::floor(x/.1)),int(std::floor(y/.1)),int(std::floor(z/.1))};
  }
  static bool hash(const std::string& value) {
    return value.size()==64&&std::all_of(value.begin(),value.end(),[](unsigned char c){return std::isxdigit(c);});
  }
  static bool validRiserObservations(const d1max_planning_interfaces::msg::SupportReference& s) {
    if(s.stair_riser_xyz.size()!=s.stair_riser_owner.size()||s.stair_riser_xyz.size()>20000)return false;
    for(std::size_t i=0;i<s.stair_riser_xyz.size();++i) {
      const auto& p=s.stair_riser_xyz[i];
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
          std::abs(p.x)>1e5||std::abs(p.y)>1e5||std::abs(p.z)>1e5)return false;
      const auto owner=s.stair_riser_owner[i];
      if(s.regions.empty()) {
        if(owner!=0||s.required_mode!="stair"||
            (s.segment_kind!="stair_up"&&s.segment_kind!="stair_down"))return false;
      } else {
        if(owner>=s.regions.size())return false;
        const auto& region=s.regions[owner];
        if(region.required_mode!="stair"||
            (region.segment_kind!="stair_up"&&region.segment_kind!="stair_down"))return false;
      }
    }
    return true;
  }
public:
  d1max_planning_interfaces::msg::SupportReference source;
  bool valid{false};
private:
  void initializeLiteral() {
    const auto& s=source;
    // Independently observed original risers never enter the ground fit. An
    // empty extension preserves the historical flat/stair contract exactly.
    if(!validRiserObservations(s))return;
    if(s.regions.empty()&&s.required_mode=="general") {
      flat_.emplace(s);valid=flat_->valid;return;
    }
    const bool single_stair=s.regions.empty()&&s.required_mode=="stair"&&
      (s.floor_id=="stairs"||s.floor_id=="stair_lower"||s.floor_id=="stair_upper")&&
      (s.segment_kind=="stair_up"||s.segment_kind=="stair_down"||s.segment_kind=="landing")&&
      s.reason=="original_stair_surface_returns_associated";
    valid=s.verified&&(single_stair||d1max_planning_interfaces::validStairTransitionRegions(s))&&
      !s.frame_id.empty()&&
      !s.support_reference_id.empty()&&hash(s.support_hash)&&hash(s.support_map_sha256)&&
      s.support_ground_xyz.size()>=2&&s.support_ground_xyz.size()<=20000&&
      std::isfinite(s.support_xy_radius_m)&&s.support_xy_radius_m>0.&&s.support_xy_radius_m<=.5&&
      std::isfinite(s.body_reference_height_m)&&s.body_reference_height_m>=.2&&s.body_reference_height_m<=.8&&
      s.max_support_slope_rad==0.&&s.max_support_step_m==0.;
    if(!valid)return;
    stair_owned_.assign(s.support_ground_xyz.size(),single_stair);
    for(const auto& region:s.regions) {
      const bool stair=region.required_mode=="stair" &&
        (region.segment_kind=="stair_up"||region.segment_kind=="stair_down"||region.segment_kind=="landing");
      for(std::size_t i=region.begin_index;i<region.end_index;++i)stair_owned_[i]=stair;
    }
    for(std::size_t i=0;i<s.support_ground_xyz.size();++i) {
      const auto& p=s.support_ground_xyz[i];
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
         std::abs(p.x)>1e5||std::abs(p.y)>1e5||std::abs(p.z)>1e5) {valid=false;bins_.clear();return;}
      bins_[key(p.x,p.y,p.z)].push_back(p);
      auto xy=key(p.x,p.y,0.);terrain_bins_[xy].push_back(i);
    }
  }
public:
  explicit PlanningSupport(const d1max_planning_interfaces::msg::SupportReference& s):source(s) {
    initializeLiteral();literal_valid_=valid;
    auto model=std::make_shared<const StairModelGeometry>(s);
    if(!model->present())return; // byte-identical legacy interpretation
    if(!model->valid()) {valid=false;return;}
    model_=std::move(model);valid=true;
  }
  const MotionSupport* flatSupport()const {return flat_&&!model_?&*flat_:nullptr;}
  bool isStair()const {return valid&&(model_||!flat_);}
  bool literalStairValid()const {return valid&&literal_valid_&&!flat_;}
  const StairModelGeometry* model()const {return model_.get();}
  std::optional<Eigen::Vector3d> modeledBodyPosition(const Eigen::Vector3d& body)const {
    if(!valid||!model_)return {};
    const auto sample=model_->sampleBody(body,source.body_reference_height_m);
    return sample?std::optional<Eigen::Vector3d>(sample->body):std::nullopt;
  }
  std::optional<Eigen::Vector3d> bodyPositionOnSupport(const Eigen::Vector3d& body)const {
    if(const auto modeled=modeledBodyPosition(body))return modeled;
    const auto literal=nearestLiteral(body.x(),body.y(),body.z()-source.body_reference_height_m);
    return literal?std::optional<Eigen::Vector3d>(Eigen::Vector3d(body.x(),body.y(),
      literal->z+source.body_reference_height_m)):std::nullopt;
  }
  // Literal, same-level ownership for seed shaping. A landing or a tie across
  // two flights is not a flight: an approach may not cut across that boundary.
  std::optional<std::string> stairFlightAt(const Eigen::Vector3d& body)const {
    if(!isStair()||!body.allFinite()||(body.array().abs()>1e5).any())return {};
    const Eigen::Vector3d ground=body-Eigen::Vector3d(0.,0.,source.body_reference_height_m);
    if(model_)if(const auto sample=model_->sampleBody(body,source.body_reference_height_m))
      return sample->kind=="landing"?std::nullopt:std::optional<std::string>(sample->segment_id);
    if(!literalStairValid())return {};
    const auto center=key(ground.x(),ground.y(),0.);
    const int extent=int(std::ceil(source.support_xy_radius_m/.1));
    double best=source.support_xy_radius_m*source.support_xy_radius_m;
    bool found=false,ambiguous=false;std::optional<std::string> flight;
    const auto owner=[&](std::size_t index)->std::optional<std::string> {
      if(source.regions.empty())return source.segment_kind=="stair_up"||source.segment_kind=="stair_down"?
          std::optional<std::string>(source.version.segment_id):std::nullopt;
      for(const auto& region:source.regions)
        if(index>=region.begin_index&&index<region.end_index)
          return region.required_mode=="stair"&&
            (region.segment_kind=="stair_up"||region.segment_kind=="stair_down")?
              std::optional<std::string>(region.segment_id):std::nullopt;
      return {};
    };
    for(int x=-extent;x<=extent;++x)for(int y=-extent;y<=extent;++y) {
      const auto it=terrain_bins_.find({center.x+x,center.y+y,0});if(it==terrain_bins_.end())continue;
      for(const auto index:it->second) {
        const auto& p=source.support_ground_xyz[index];
        const double distance=(Eigen::Vector3d(p.x,p.y,p.z)-ground).squaredNorm();
        if(distance>source.support_xy_radius_m*source.support_xy_radius_m)continue;
        if(!found||distance<best-1e-12) {
          best=distance;found=true;ambiguous=false;flight=owner(index);
        } else if(std::abs(distance-best)<=1e-12&&flight!=owner(index))ambiguous=true;
      }
    }
    return found&&!ambiguous?flight:std::nullopt;
  }
  // A transition window may contain both ordinary floor and stair regions.
  // Select clearance by the nearest literal, same-level source support point,
  // not the window's overall stair flag or the fitted plane's extrapolation.
  // Missing support and exact ties with floor support retain the normal box.
  bool stairClearanceAllowed(const Eigen::Vector3d& body)const {
    if(!isStair()||!body.allFinite()||(body.array().abs()>1e5).any())return false;
    const Eigen::Vector3d ground=body-Eigen::Vector3d(0.,0.,source.body_reference_height_m);
    if(model_&&model_->sampleBody(body,source.body_reference_height_m))return true;
    if(!literalStairValid())return false;
    const auto center=key(ground.x(),ground.y(),0.);
    const int extent=int(std::ceil(source.support_xy_radius_m/.1));
    double best=source.support_xy_radius_m*source.support_xy_radius_m;
    bool found=false,stair=false;
    for(int x=-extent;x<=extent;++x)for(int y=-extent;y<=extent;++y) {
      const auto it=terrain_bins_.find({center.x+x,center.y+y,0});if(it==terrain_bins_.end())continue;
      for(const auto i:it->second) {
        const auto& p=source.support_ground_xyz[i];
        const double distance=(Eigen::Vector3d(p.x,p.y,p.z)-ground).squaredNorm();
        if(distance>source.support_xy_radius_m*source.support_xy_radius_m)continue;
        if(!found||distance<best-1e-12) {best=distance;found=true;stair=stair_owned_[i];}
        else if(std::abs(distance-best)<=1e-12)stair=stair&&stair_owned_[i];
      }
    }
    return found&&stair;
  }
  // Two-dimensional callers retain the exact historical flat behavior. A
  // stair query without its expected level is ambiguous and never guesses.
  std::optional<Point> nearest(double x,double y)const {
    return valid&&flat_?flat_->nearest(x,y):std::nullopt;
  }
  std::optional<Point> nearest(double x,double y,double expected_ground_z)const {
    if(!valid||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(expected_ground_z)||
       std::abs(x)>1e5||std::abs(y)>1e5||std::abs(expected_ground_z)>1e5)return {};
    if(model_)if(const auto sample=model_->sample({x,y,expected_ground_z})) {
      Point point;point.x=sample->ground.x();point.y=sample->ground.y();point.z=sample->ground.z();return point;
    }
    return nearestLiteral(x,y,expected_ground_z);
  }
  std::optional<Point> nearestLiteral(double x,double y,double expected_ground_z)const {
    if(!valid||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(expected_ground_z)||
       std::abs(x)>1e5||std::abs(y)>1e5||std::abs(expected_ground_z)>1e5)return {};
    if(!literal_valid_)return {};
    if(flat_)return flat_->nearest(x,y);
    const auto center=key(x,y,expected_ground_z);
    const int extent=int(std::ceil(source.support_xy_radius_m/.1));
    double best=source.support_xy_radius_m*source.support_xy_radius_m;
    std::optional<Point> result;
    for(int dx=-extent;dx<=extent;++dx)for(int dy=-extent;dy<=extent;++dy)for(int dz=-extent;dz<=extent;++dz) {
      const auto it=bins_.find({center.x+dx,center.y+dy,center.z+dz});if(it==bins_.end())continue;
      for(const auto& p:it->second) {
        const double distance=(p.x-x)*(p.x-x)+(p.y-y)*(p.y-y)+
          (p.z-expected_ground_z)*(p.z-expected_ground_z);
        if(distance<=best) {best=distance;result=p;}
      }
    }
    return result;
  }
  // Fit the local mean stair surface, not individual tread discontinuities or
  // the optimized curve's instantaneous vertical derivative. All inputs are
  // literal classified support returns. There is no mutable shared cache;
  // each native map-query scope owns its bounded normal cache independently.
  TerrainFit terrainFit(const Eigen::Vector3d& body)const {
    TerrainFit result;
    if(!valid||!body.allFinite())return result;
    if(model_)if(const auto sample=model_->sampleBody(body,source.body_reference_height_m)) {
      result.valid=true;result.normal=sample->normal;result.samples=0;
      // This is a model prior, not a successful point-cloud fit. Keep its
      // residual/variance fields NaN rather than fabricating lidar statistics.
      result.center_residual=sample->body.z()-body.z();return result;
    }
    if(!literal_valid_)return result;
    if(flat_) {result.valid=true;result.rms=result.center_residual=0.;return result;}
    const Eigen::Vector3d center=body-Eigen::Vector3d(0.,0.,source.body_reference_height_m);
    if(!nearest(center.x(),center.y(),center.z()))return result;
    const auto cell=key(center.x(),center.y(),0.);
    std::vector<Eigen::Vector3d> points;std::vector<double> initial,weights;
    for(int x=-6;x<=6;++x)for(int y=-6;y<=6;++y) {
      const auto found=terrain_bins_.find({cell.x+x,cell.y+y,0});if(found==terrain_bins_.end())continue;
      for(const auto index:found->second) {
        const auto& p=source.support_ground_xyz[index];const Eigen::Vector3d delta=Eigen::Vector3d(p.x,p.y,p.z)-center;
        if(delta.head<2>().squaredNorm()>.36||std::abs(delta.z())>.5)continue;
        points.push_back(delta);initial.push_back(std::exp(-delta.head<2>().squaredNorm()/.16));
      }
    }
    result.samples=points.size();if(points.size()<12)return result;
    weights=initial;Eigen::Vector3d coefficients=Eigen::Vector3d::Zero();
    // A fixed three robust least-squares passes limit the influence of tread
    // edges without removing points or fitting an arbitrary support plane.
    for(int iteration=0;iteration<3;++iteration) {
      Eigen::Matrix3d matrix=Eigen::Matrix3d::Zero();Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
      for(std::size_t i=0;i<points.size();++i) {
        const Eigen::Vector3d feature(points[i].x(),points[i].y(),1.);
        matrix.noalias()+=weights[i]*feature*feature.transpose();rhs.noalias()+=weights[i]*feature*points[i].z();
      }
      const Eigen::LDLT<Eigen::Matrix3d> solve(matrix);
      if(solve.info()!=Eigen::Success||!solve.isPositive())return result;
      coefficients=solve.solve(rhs);if(!coefficients.allFinite())return result;
      for(std::size_t i=0;i<points.size();++i) {
        const double residual=points[i].z()-coefficients.dot(Eigen::Vector3d(points[i].x(),points[i].y(),1.));
        weights[i]=initial[i]*std::min(1.,.06/std::max(std::abs(residual),1e-12));
      }
    }
    double weight_sum=0.,square_error=0.;Eigen::Vector2d mean=Eigen::Vector2d::Zero();
    for(std::size_t i=0;i<points.size();++i) {
      weight_sum+=weights[i];mean+=weights[i]*points[i].head<2>();
      const double error=points[i].z()-coefficients.dot(Eigen::Vector3d(points[i].x(),points[i].y(),1.));
      square_error+=weights[i]*error*error;
    }
    if(weight_sum<=0.)return result;
    mean/=weight_sum;
    Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
    for(std::size_t i=0;i<points.size();++i) {
      const Eigen::Vector2d delta=points[i].head<2>()-mean;covariance.noalias()+=weights[i]*delta*delta.transpose();
    }
    covariance/=weight_sum;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covariance);
    if(eig.info()!=Eigen::Success)return result;
    result.rms=std::sqrt(square_error/weight_sum);result.minor_variance=eig.eigenvalues()[0];
    result.center_residual=coefficients.z();
    result.normal=Eigen::Vector3d(-coefficients.x(),-coefficients.y(),1.).normalized();
    // These are geometric fit/conditioning limits, not stair-gait capability.
    // Insufficient or mixed-height evidence returns no attitude, never FREE.
    result.valid=result.rms<=.08&&result.minor_variance>=.0025&&
      std::abs(result.center_residual)<=.15&&coefficients.head<2>().norm()<=1.;
    return result;
  }
  static std::optional<Eigen::Vector2d> attitudeFromNormal(const Eigen::Vector3d& normal,double yaw) {
    if(!normal.allFinite()||normal.norm()<1e-9||normal.z()<=0.||!std::isfinite(yaw))return {};
    const Eigen::Vector3d n=normal.normalized();
    const double nx=std::cos(yaw)*n.x()+std::sin(yaw)*n.y();
    const double ny=-std::sin(yaw)*n.x()+std::cos(yaw)*n.y();
    // ROS Rz(yaw)*Ry(pitch)*Rx(roll): forward uphill means negative pitch.
    return Eigen::Vector2d(std::atan2(nx,n.z()),std::asin(std::clamp(-ny,-1.,1.)));
  }
  std::optional<Eigen::Vector2d> terrainAttitude(const Eigen::Vector3d& body,double yaw)const {
    const auto fit=terrainFit(body);return fit.valid?attitudeFromNormal(fit.normal,yaw):std::nullopt;
  }
  // The body traverses a mean stair surface; it does not execute the vertical
  // riser derivative of a 5 cm reference chord. Keep the route's XY direction
  // and lift it onto the SAME source-backed surface used for body attitude.
  // This changes a future velocity boundary only, never the measured start,
  // target position, support points or collision geometry. No usable fit means
  // no replacement; flat sessions retain their exact existing boundary.
  std::optional<Eigen::Vector3d> tangentOnSurface(const Eigen::Vector3d& body,
                                                 const Eigen::Vector3d& tangent)const {
    if(!isStair()||!body.allFinite()||!tangent.allFinite()||
       tangent.head<2>().norm()<=1e-6)return {};
    const auto fit=terrainFit(body);
    if(!fit.valid||!fit.normal.allFinite()||fit.normal.z()<=1e-6)return {};
    Eigen::Vector3d result=tangent;
    result.z()=-fit.normal.head<2>().dot(tangent.head<2>())/fit.normal.z();
    return result.allFinite()?std::optional<Eigen::Vector3d>(result):std::nullopt;
  }
};

// Reference and static support travel independently. A small stair reference
// can arrive first, but its search must not temporarily use an upright/flat
// body just because its terrain geometry is still being delivered. This is
// candidate-input readiness only: it does not retire or invalidate the already
// committed curve. Flat and legacy references retain their original behavior.
template<class Reference,class Version>
inline bool stairReferenceGeometryReady(const Reference& reference,const Version& version,
    const std::shared_ptr<const PlanningSupport>& support,const std::string& frame) {
  const bool stair=reference.schema_version==2 &&
    (reference.required_mode=="stair" || std::any_of(reference.point_required_modes.begin(),
        reference.point_required_modes.end(),[](const auto& mode){return mode=="stair";}));
  return !stair || (support&&support->isStair()&&support->source.version==version&&
                   support->source.frame_id==frame);
}
} // namespace scan_planner
