#pragma once
#include <plan_env/grid_map.h>
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Geometry>
#include <unordered_map>
#include <optional>

namespace scan_planner {
inline bool motionSourceFresh(std::int64_t source,std::int64_t now) {
  return source>0&&now>=source&&now-source<=100000000LL;
}
inline std::int64_t motionEvidenceDeadline(std::int64_t demand_source,std::int64_t demand_body,
    std::int64_t actual_body,std::int64_t original_deadline,std::int64_t permit_deadline,
    std::int64_t curve_deadline,std::int64_t raw_deadline,std::int64_t front_source,std::int64_t rear_source,
    std::int64_t sensor_max_age_ns=500000000LL) {
  if(sensor_max_age_ns<1000000LL||sensor_max_age_ns>600000000LL)return 0;
  return std::min<std::int64_t>({demand_source+100000000LL,demand_body+100000000LL,
    actual_body+100000000LL,original_deadline,permit_deadline,curve_deadline,raw_deadline,
    front_source+sensor_max_age_ns,rear_source+sensor_max_age_ns});
}
// These are measured upper bounds, not a fitted deceleration model. A fixture
// record cannot be used by the live transport. The original bytes are hashed.
struct BrakingModel {
  double max_speed{0.},max_yaw{0.},reaction{0.},distance{0.},yaw{0.},latency{0.},
    tracking_error{0.},heading_error{0.};
  std::string sha256;
  double sensor_source_age{.5};
  std::int64_t raySourceAgeNs()const {return static_cast<std::int64_t>(sensor_source_age*1e9);}
  static BrakingModel load(const std::string& file,const std::string& sha,const std::string& transport);
  bool valid() const {
    for(double n:{max_speed,max_yaw,reaction,distance,yaw,latency,tracking_error,heading_error})
      if(!std::isfinite(n)||n<=0.)return false;
    return max_speed<=.3&&max_yaw<=.5&&reaction<=1.&&distance<=1.&&yaw<=1.&&
      latency<=3.&&tracking_error<=.25&&heading_error<=.5&&sha256.size()==64&&
      std::isfinite(sensor_source_age)&&sensor_source_age>=.001&&sensor_source_age<=.6;
  }
};

class MotionSupport {
  struct Key {int x,y;bool operator==(const Key& b)const{return x==b.x&&y==b.y;}};
  struct Hash {std::size_t operator()(const Key& k)const{return std::hash<int>{}(k.x)^(std::hash<int>{}(k.y)<<1);}};
  using Point=geometry_msgs::msg::Point;
  std::unordered_map<Key,std::vector<Point>,Hash> bins_;
public:
  d1max_planning_interfaces::msg::SupportReference source;
  bool valid{false};
  explicit MotionSupport(const d1max_planning_interfaces::msg::SupportReference& s):source(s) {
    valid=s.verified&&s.floor_id=="floor1"&&s.segment_kind=="floor"&&s.required_mode=="general"&&
      s.support_hash.size()==64&&s.support_map_sha256.size()==64&&s.support_ground_xyz.size()>=2&&
      s.support_ground_xyz.size()<=20000&&std::isfinite(s.support_xy_radius_m)&&s.support_xy_radius_m>0.&&
      s.support_xy_radius_m<=.5&&std::isfinite(s.body_reference_height_m)&&s.body_reference_height_m>=.2&&
      s.body_reference_height_m<=.8&&std::isfinite(s.max_support_slope_rad)&&s.max_support_slope_rad>0.&&
      s.max_support_slope_rad<=.176&&std::isfinite(s.max_support_step_m)&&s.max_support_step_m>0.&&s.max_support_step_m<=.08;
    if(!valid)return;
    for(const auto& p:s.support_ground_xyz) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::abs(p.x)>1e5||std::abs(p.y)>1e5){valid=false;return;}
      bins_[{int(std::floor(p.x/.1)),int(std::floor(p.y/.1))}].push_back(p);
    }
  }
  std::optional<Point> nearest(double x,double y)const {
    if(!valid||!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>1e5||std::abs(y)>1e5)return {};
    const Key key{int(std::floor(x/.1)),int(std::floor(y/.1))};
    const int n=int(std::ceil(source.support_xy_radius_m/.1))+1;
    double best=source.support_xy_radius_m*source.support_xy_radius_m;std::optional<Point> result;
    for(int a=-n;a<=n;++a)for(int b=-n;b<=n;++b) {
      auto it=bins_.find({key.x+a,key.y+b});if(it==bins_.end())continue;
      for(const auto& p:it->second){const double d=(p.x-x)*(p.x-x)+(p.y-y)*(p.y-y);if(d<=best){best=d;result=p;}}
    }
    return result;
  }
};

struct MotionSweepResult {bool valid{false};std::string reason;std::size_t unique_voxels{0};
  double length{0.},heading_bound{0.},min_z{0.},max_z{0.};};

// A reachable set, not just the ideal vx/wz arc: every possible center lies
// within this body-frame rectangle throughout reaction and braking, while
// turning each of the two cylinders is enclosed by a radius expansion. The
// original GridMap evidence (including never-seen and insufficient cells) is
// queried once per intersecting voxel. No free-space substitute is constructed.
inline MotionSweepResult validateMotionSweep(GridMap& map,const MotionSupport& support,
    const BrakingModel& model,const Eigen::Vector3d& body,double body_yaw,
    const geometry_msgs::msg::Twist& measured_world,const geometry_msgs::msg::Twist& demand,
    const std::string& frame,double budget_s=.025,double planning_straight_distance=0.) {
  MotionSweepResult result;result.reason="invalid_motion_model_or_input";
  const auto begin=std::chrono::steady_clock::now();
  const auto expired=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>budget_s;};
  if(!model.valid()||!body.allFinite()||!std::isfinite(body_yaw)||!support.valid||support.source.frame_id!=frame||
      !std::isfinite(budget_s)||budget_s<=0.||budget_s>.1||
      !std::isfinite(planning_straight_distance)||planning_straight_distance<0.||
      planning_straight_distance>2.)return result;
  for(double v:{demand.linear.x,demand.linear.y,demand.linear.z,demand.angular.x,demand.angular.y,demand.angular.z,
      measured_world.linear.x,measured_world.linear.y,measured_world.linear.z,measured_world.angular.z})if(!std::isfinite(v))return result;
  if(demand.linear.x<0.||demand.linear.x>model.max_speed||std::abs(demand.angular.z)>model.max_yaw||
      std::abs(demand.linear.y)+std::abs(demand.linear.z)+std::abs(demand.angular.x)+std::abs(demand.angular.y)>1e-9||
      std::hypot(measured_world.linear.x,measured_world.linear.y)>model.max_speed+1e-6||
      std::abs(measured_world.angular.z)>model.max_yaw+1e-6){result.reason="command_or_measured_speed_outside_record";return result;}
  const double c=std::cos(body_yaw),s=std::sin(body_yaw);
  const double vx=c*measured_world.linear.x+s*measured_world.linear.y;
  const double vy=-s*measured_world.linear.x+c*measured_world.linear.y;
  const double speed=std::max(demand.linear.x,std::hypot(measured_world.linear.x,measured_world.linear.y));
  const double turn=std::max(std::abs(demand.angular.z),std::abs(measured_world.angular.z))*model.reaction+
    model.yaw+model.heading_error;
  const double length=speed*model.reaction+model.distance;
  const double lateral=length*(turn<1.570796326794897?std::sin(turn):1.)+
    std::abs(vy)*model.reaction+model.tracking_error;
  const double xmin=turn>=1.570796326794897?-length-model.tracking_error:
    std::min(0.,vx)*model.reaction-(vx<0.?model.distance:0.)-model.tracking_error;
  // For candidate selection only, the union of this SAME reachable set along
  // a straight reference extends its longitudinal interval. It never shrinks
  // the actual command envelope or changes its evidence semantics. This is
  // not a command certificate: real commands still query the measured pose.
  const double xmax=length+model.tracking_error+planning_straight_distance;
  const auto shape=map.bodyEnvelope();
  if(!std::isfinite(shape.radius)||shape.radius<=0.||!std::isfinite(shape.offset)||shape.offset<0.||
      !std::isfinite(shape.above)||!std::isfinite(shape.below)||shape.above<0.||shape.below<0.)return result;
  const double radius=shape.radius+2.*shape.offset*std::sin(std::min(turn,3.141592653589793)*.5);
  result.length=length;result.heading_bound=turn;
  const auto anchor=support.nearest(body.x(),body.y());
  result.reason="braking_sweep_support_unverified";
  if(!anchor||std::abs(body.z()-anchor->z-support.source.body_reference_height_m)>.15)return result;
  double zlow=anchor->z,zhigh=anchor->z;
  const double step=std::min(.025,map.getResolution()*.5);
  if(!std::isfinite(step)||step<=0.)return result;
  const int nx=std::max(1,int(std::ceil((xmax-xmin)/step))),ny=std::max(1,int(std::ceil(2*lateral/step)));
  if(nx>200||ny>200)return result;
  for(int ix=0;ix<=nx;++ix)for(int iy=0;iy<=ny;++iy) {
    if(expired()){result.reason="motion_check_budget_exhausted";return result;}
    const double x=xmin+(xmax-xmin)*ix/nx,y=-lateral+2*lateral*iy/ny;
    const double wx=body.x()+c*x-s*y,wy=body.y()+s*x+c*y;
    const auto p=support.nearest(wx,wy);if(!p)return result;
    if(std::abs(p->z-anchor->z)>support.source.max_support_step_m+
        std::tan(support.source.max_support_slope_rad)*std::hypot(p->x-anchor->x,p->y-anchor->y))return result;
    zlow=std::min(zlow,p->z);zhigh=std::max(zhigh,p->z);
  }
  // Sample support bounds conservatively between sample centers as well; the
  // source map's checked local slope/step bound supplies the interpolation
  // uncertainty. No artificial flattening of the original floor is performed.
  const double zpad=std::tan(support.source.max_support_slope_rad)*step+model.tracking_error+
    std::abs(measured_world.linear.z)*model.reaction;
  result.min_z=body.z()+zlow-anchor->z-zpad-shape.below;
  result.max_z=body.z()+zhigh-anchor->z+zpad+shape.above;
  Eigen::Vector3d lower=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),upper=-lower;
  for(double x:{xmin-shape.offset-radius,xmax+shape.offset+radius})for(double y:{-lateral-radius,lateral+radius}) {
    Eigen::Vector3d p(body.x()+c*x-s*y,body.y()+s*x+c*y,0.);
    lower=lower.cwiseMin(p);upper=upper.cwiseMax(p);
  }
  lower.z()=result.min_z;upper.z()=result.max_z;
  Eigen::Vector3i low,high;map.posToIndex(lower,low);map.posToIndex(upper,high);
  const Eigen::Matrix<std::int64_t,3,1> size=(high-low+Eigen::Vector3i::Ones()).cast<std::int64_t>();
  if((size.array()<=0).any()||size.prod()>200000){result.reason="motion_volume_budget_exhausted";return result;}
  map.beginObservedProof();
  // Circumscribed XY cell disc means corner-intersecting voxels are included.
  const double padded_radius=radius+map.getResolution()*std::sqrt(.5);
  for(int x=low.x();x<=high.x();++x)for(int y=low.y();y<=high.y();++y) {
    Eigen::Vector3d center;map.indexToPos({x,y,low.z()},center);
    const double dx=center.x()-body.x(),dy=center.y()-body.y(),u=c*dx+s*dy,v=-s*dx+c*dy;
    bool inside=false;
    for(double offset:{-shape.offset,shape.offset}) {
      const double qx=std::max({xmin+offset-u,0.,u-(xmax+offset)}),qy=std::max(0.,std::abs(v)-lateral);
      inside=inside||(qx*qx+qy*qy<=padded_radius*padded_radius);
    }
    if(!inside)continue;
    for(int z=low.z();z<=high.z();++z) {
      if(expired()){result.reason="motion_check_budget_exhausted";return result;}
      ++result.unique_voxels;const int state=map.observedRawSnapshotStatus({x,y,z});
      if(state!=0){result.reason=state==1?"motion_sweep_occupied":state<0?"motion_sweep_outside_map":"motion_sweep_unknown_or_expired";return result;}
    }
  }
  result.valid=result.unique_voxels>0;result.reason=result.valid?"observed_free_motion_sweep":"empty_motion_sweep";return result;
}

// Conservative, bounded reference screening. The underlying production map,
// original support and measured braking model are identical to command checks.
// Full SCAN curve admission and fresh actual-body command validation remain
// mandatory; a straight corridor result does not authorize a future curve.
inline MotionSweepResult validateStraightBrakingCorridor(GridMap& map,const MotionSupport& support,
    const BrakingModel& model,const Eigen::Vector3d& start,const Eigen::Vector3d& target,
    const std::string& frame,double budget_s=.08) {
  const Eigen::Vector2d delta=(target-start).head<2>();
  if(!start.allFinite()||!target.allFinite()||delta.norm()<1e-6||delta.norm()>2.)
    return {false,"invalid_planning_corridor"};
  const double yaw=std::atan2(delta.y(),delta.x());
  geometry_msgs::msg::Twist demand,measured;
  demand.linear.x=model.max_speed;
  measured.linear.x=model.max_speed*std::cos(yaw);measured.linear.y=model.max_speed*std::sin(yaw);
  return validateMotionSweep(map,support,model,start,yaw,measured,demand,frame,budget_s,delta.norm());
}
} // namespace scan_planner
