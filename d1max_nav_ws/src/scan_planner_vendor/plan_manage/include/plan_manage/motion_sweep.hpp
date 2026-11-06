#pragma once
#include <plan_env/grid_map.h>
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Geometry>
#include <array>
#include <unordered_map>
#include <optional>

namespace scan_planner {
enum class KinematicProfile { ForwardYaw, HolonomicXYYaw };
inline KinematicProfile parseKinematicProfile(const std::string& value) {
  if(value=="forward_yaw")return KinematicProfile::ForwardYaw;
  if(value=="holonomic_xyyaw")return KinematicProfile::HolonomicXYYaw;
  throw std::invalid_argument("unsupported kinematic profile");
}
inline bool planarDemandIsZero(const geometry_msgs::msg::Twist& demand) {
  return std::abs(demand.linear.x)+std::abs(demand.linear.y)+std::abs(demand.angular.z)<1e-12;
}
inline bool motionSourceFresh(std::int64_t source,std::int64_t now,std::int64_t maximum_ns=100000000LL) {
  return maximum_ns>=100000000LL&&maximum_ns<=500000000LL&&source>0&&now>=source&&now-source<=maximum_ns;
}
inline std::int64_t motionEvidenceDeadline(std::int64_t demand_source,std::int64_t demand_body,
    std::int64_t actual_body,std::int64_t original_deadline,std::int64_t permit_deadline,
    std::int64_t curve_deadline,std::int64_t raw_deadline,std::int64_t front_source,std::int64_t rear_source,
    std::int64_t sensor_max_age_ns=500000000LL,std::int64_t command_pipeline_ns=100000000LL) {
  if(sensor_max_age_ns<1000000LL||sensor_max_age_ns>600000000LL||
      command_pipeline_ns<100000000LL||command_pipeline_ns>500000000LL)return 0;
  return std::min<std::int64_t>({demand_source+command_pipeline_ns,demand_body+command_pipeline_ns,
    actual_body+100000000LL,original_deadline,permit_deadline,curve_deadline,raw_deadline,
    front_source+sensor_max_age_ns,rear_source+sensor_max_age_ns});
}
// Bound numerical parameters, not a fitted deceleration model. A legacy
// physical record and an explicitly configured runtime profile have distinct
// provenance; runtime values do not assert measured upper bounds. A fixture
// cannot be used live. The original bytes are hashed in every case.
struct BrakingModel {
  double max_speed{0.},max_yaw{0.},reaction{0.},distance{0.},yaw{0.},latency{0.},
    tracking_error{0.},heading_error{0.};
  std::string sha256;
  double sensor_source_age{.5};
  double command_pipeline_bound{.1};
  // Legacy records cover forward+yaw only. A lateral braking bound cannot
  // be inferred from the forward distance, or granted by a runtime flag.
  KinematicProfile kinematic_profile{KinematicProfile::ForwardYaw};
  double lateral_distance{0.};
  std::string record_kind{},parameter_basis{};
  bool physical_acceptance_verified{false};
  std::int64_t raySourceAgeNs()const {return static_cast<std::int64_t>(sensor_source_age*1e9);}
  std::int64_t commandPipelineNs()const {return static_cast<std::int64_t>(std::llround(command_pipeline_bound*1e9));}
  static BrakingModel load(const std::string& file,const std::string& sha,const std::string& transport);
  bool valid() const {
    for(double n:{max_speed,max_yaw,reaction,distance,yaw,latency,tracking_error,heading_error})
      if(!std::isfinite(n)||n<=0.)return false;
    const bool profile_valid=kinematic_profile==KinematicProfile::ForwardYaw ||
      (kinematic_profile==KinematicProfile::HolonomicXYYaw&&
       std::isfinite(lateral_distance)&&lateral_distance>0.&&lateral_distance<=1.);
    return profile_valid&&max_speed<=.3&&max_yaw<=.5&&reaction<=1.&&distance<=1.&&yaw<=1.&&
      latency<=3.&&tracking_error<=.25&&heading_error<=.5&&sha256.size()==64&&
      std::isfinite(sensor_source_age)&&sensor_source_age>=.001&&sensor_source_age<=.6&&
      std::isfinite(command_pipeline_bound)&&command_pipeline_bound>=.1&&command_pipeline_bound<=.5;
  }
};

// Component extrema for a vector rotated through the complete bounded yaw
// interval. Endpoints alone miss an intermediate alignment with a cell/axis.
inline std::array<double,4> rotatedPlanarVelocityBounds(double vx,double vy,double turn) {
  const double speed=std::hypot(vx,vy);
  if(turn>=3.141592653589793)return {-speed,speed,-speed,speed};
  const double bearing=std::atan2(vy,vx);
  std::array<double,4> bounds{{std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()}};
  const auto sample=[&](double angle) {
    const double x=speed*std::cos(angle),y=speed*std::sin(angle);
    bounds[0]=std::min(bounds[0],x);bounds[1]=std::max(bounds[1],x);
    bounds[2]=std::min(bounds[2],y);bounds[3]=std::max(bounds[3],y);
  };
  sample(bearing-turn);sample(bearing+turn);
  for(int k=-4;k<=4;++k) {
    const double angle=k*1.570796326794897;
    if(angle>=bearing-turn&&angle<=bearing+turn)sample(angle);
  }
  return bounds;
}

struct PlanarReachableBounds {double xmin,xmax,ymin,ymax,length;};
// The record supplies longitudinal AND lateral stopping-displacement bounds
// for this profile, including simultaneous XY+yaw. Their source is either
// the legacy physical record or configured runtime parameters, never inferred.
// Rotate that stopping rectangle, not just the ideal commanded center arc.
inline PlanarReachableBounds holonomicReachableBounds(const BrakingModel& model,
    double measured_vx,double measured_vy,double command_vx,double command_vy,double turn,
    double planning_straight_distance=0.) {
  const auto actual=rotatedPlanarVelocityBounds(measured_vx,measured_vy,turn);
  const auto desired=rotatedPlanarVelocityBounds(command_vx,command_vy,turn);
  const double ax=std::min(turn,std::atan2(model.lateral_distance,model.distance));
  const double ay=std::min(turn,std::atan2(model.distance,model.lateral_distance));
  const double stop_x=model.distance*std::cos(ax)+model.lateral_distance*std::sin(ax);
  const double stop_y=model.lateral_distance*std::cos(ay)+model.distance*std::sin(ay);
  const double pad=model.tracking_error;
  return {std::min({0.,actual[0],desired[0]})*model.reaction-stop_x-pad,
    std::max({0.,actual[1],desired[1]})*model.reaction+stop_x+pad+planning_straight_distance,
    std::min({0.,actual[2],desired[2]})*model.reaction-stop_y-pad,
    std::max({0.,actual[3],desired[3]})*model.reaction+stop_y+pad,
    std::max(std::hypot(command_vx,command_vy),std::hypot(measured_vx,measured_vy))*model.reaction+
      std::hypot(model.distance,model.lateral_distance)};
}

class MotionSupport {
  struct Key {int x,y;bool operator==(const Key& b)const{return x==b.x&&y==b.y;}};
  struct Hash {std::size_t operator()(const Key& k)const{return std::hash<int>{}(k.x)^(std::hash<int>{}(k.y)<<1);}};
  using Point=geometry_msgs::msg::Point;
  std::unordered_map<Key,std::vector<Point>,Hash> bins_;
public:
  d1max_planning_interfaces::msg::SupportReference source;
  bool valid{false};
  explicit MotionSupport(const d1max_planning_interfaces::msg::SupportReference& s):source(s) {
    valid=s.verified&&(s.floor_id=="floor1"||s.floor_id=="floor2")&&(s.segment_kind=="floor"||s.segment_kind=="landing")&&s.required_mode=="general"&&
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

enum class MotionSweepCompletion { Uncertified, Clear, Blocked, BudgetExhausted };
struct MotionSweepResult {bool valid{false};std::string reason;std::size_t unique_voxels{0};
  MotionSweepCompletion completion{MotionSweepCompletion::Uncertified};
  double length{0.},heading_bound{0.},min_z{0.},max_z{0.};
  double center_xmin{0.},center_xmax{0.},center_ymin{0.},center_ymax{0.};
  // Unique raw evidence actually visited: free, occupied, unknown, outside.
  // Occupied and deadline exits may be partial; never label them full-volume
  // counts or turn partial free evidence into a command certificate.
  std::array<std::size_t,4> state_counts{{0,0,0,0}};
  bool query_complete{false};};

// A reachable set, not just the ideal vx/wz arc: every possible center lies
// within this body-frame rectangle throughout reaction and braking, while
// the configured body shape covers every heading in the braking bound. The
// original GridMap evidence is queried once per intersecting voxel under the
// SAME selected SCAN policy. Unknown/weak evidence remains in the diagnostics;
// official mode does not veto it, strict comparison mode requires fresh FREE.
// No free-space substitute or near-body clearing is constructed.
inline MotionSweepResult validateMotionSweep(GridMap& map,const MotionSupport& support,
    const BrakingModel& model,const Eigen::Vector3d& body,double body_yaw,
    const geometry_msgs::msg::Twist& measured_world,const geometry_msgs::msg::Twist& demand,
    const std::string& frame,double budget_s=.025,double planning_straight_distance=0.,
    std::optional<Eigen::Vector2d> planning_body_translation={}) {
  MotionSweepResult result;result.reason="invalid_motion_model_or_input";
  const auto begin=std::chrono::steady_clock::now();
  const auto expired=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>budget_s;};
  if(!model.valid()||!body.allFinite()||!std::isfinite(body_yaw)||!support.valid||support.source.frame_id!=frame||
      !std::isfinite(budget_s)||budget_s<=0.||budget_s>.1||
      !std::isfinite(planning_straight_distance)||planning_straight_distance<0.||
      planning_straight_distance>2.)return result;
  if(planning_body_translation&&(!planning_body_translation->allFinite()||
     planning_body_translation->norm()>2.||planning_straight_distance!=0.||
     model.kinematic_profile!=KinematicProfile::HolonomicXYYaw))return result;
  if(!map.collisionSnapshotSourceFresh()) {
    result.reason="motion_sensor_source_expired";return result;
  }
  for(double v:{demand.linear.x,demand.linear.y,demand.linear.z,demand.angular.x,demand.angular.y,demand.angular.z,
      measured_world.linear.x,measured_world.linear.y,measured_world.linear.z,measured_world.angular.z})if(!std::isfinite(v))return result;
  const bool holonomic=model.kinematic_profile==KinematicProfile::HolonomicXYYaw;
  const bool translation_invalid=holonomic?std::hypot(demand.linear.x,demand.linear.y)>model.max_speed+1e-12:
    (demand.linear.x<0.||demand.linear.x>model.max_speed||std::abs(demand.linear.y)>1e-9);
  if(translation_invalid||std::abs(demand.angular.z)>model.max_yaw||
      std::abs(demand.linear.z)+std::abs(demand.angular.x)+std::abs(demand.angular.y)>1e-9||
      std::hypot(measured_world.linear.x,measured_world.linear.y)>model.max_speed+1e-6||
      std::abs(measured_world.angular.z)>model.max_yaw+1e-6){result.reason="command_or_measured_speed_outside_record";return result;}
  const double c=std::cos(body_yaw),s=std::sin(body_yaw);
  const double vx=c*measured_world.linear.x+s*measured_world.linear.y;
  const double vy=-s*measured_world.linear.x+c*measured_world.linear.y;
  const double speed=std::max(demand.linear.x,std::hypot(measured_world.linear.x,measured_world.linear.y));
  const double turn=std::max(std::abs(demand.angular.z),std::abs(measured_world.angular.z))*model.reaction+
    model.yaw+model.heading_error;
  double length=speed*model.reaction+model.distance;
  const double lateral=length*(turn<1.570796326794897?std::sin(turn):1.)+
    std::abs(vy)*model.reaction+model.tracking_error;
  double xmin=turn>=1.570796326794897?-length-model.tracking_error:
    std::min(0.,vx)*model.reaction-(vx<0.?model.distance:0.)-model.tracking_error;
  // For candidate selection only, the union of this SAME reachable set along
  // a straight reference extends its longitudinal interval. It never shrinks
  // the actual command envelope or changes its evidence semantics. This is
  // not a command certificate: real commands still query the measured pose.
  double xmax=length+model.tracking_error+planning_straight_distance;
  double ymin=-lateral,ymax=lateral;
  if(holonomic) {
    const auto reach=holonomicReachableBounds(model,vx,vy,demand.linear.x,demand.linear.y,turn,
        planning_straight_distance);
    xmin=reach.xmin;xmax=reach.xmax;ymin=reach.ymin;ymax=reach.ymax;length=reach.length;
    if(planning_body_translation) {
      xmin+=std::min(0.,planning_body_translation->x());xmax+=std::max(0.,planning_body_translation->x());
      ymin+=std::min(0.,planning_body_translation->y());ymax+=std::max(0.,planning_body_translation->y());
    }
  }
  result.center_xmin=xmin;result.center_xmax=xmax;result.center_ymin=ymin;result.center_ymax=ymax;
  const auto shape=map.bodyEnvelope();
  try {validateBodyEnvelope(shape);}catch(const std::invalid_argument&){return result;}
  const bool oriented_box=shape.model==CollisionModel::OrientedBox;
  double extent_x=0.,extent_y=0.,radius=0.;
  if(oriented_box) {
    // The maximum projections of a rotated box occur at a corner bearing.
    // This contains ALL headings in +/-turn, not just the intended arc or its
    // end pose. Translating this same shape over the reachable-center rectangle
    // gives a conservative OBB sweep without replacing the body by cylinders.
    const double hx=.5*shape.length,hy=.5*shape.width;
    const double angle_x=std::min(turn,std::atan2(hy,hx));
    const double angle_y=std::min(turn,std::atan2(hx,hy));
    extent_x=hx*std::cos(angle_x)+hy*std::sin(angle_x);
    extent_y=hy*std::cos(angle_y)+hx*std::sin(angle_y);
  } else {
    radius=shape.radius+2.*shape.offset*std::sin(std::min(turn,3.141592653589793)*.5);
    extent_x=shape.offset+radius;extent_y=radius;
  }
  result.length=length;result.heading_bound=turn;
  const auto anchor=support.nearest(body.x(),body.y());
  result.reason="braking_sweep_support_unverified";
  if(!anchor||std::abs(body.z()-anchor->z-support.source.body_reference_height_m)>.15)return result;
  double zlow=anchor->z,zhigh=anchor->z;
  const double step=std::min(.025,map.getResolution()*.5);
  if(!std::isfinite(step)||step<=0.)return result;
  const int nx=std::max(1,int(std::ceil((xmax-xmin)/step))),ny=std::max(1,int(std::ceil((ymax-ymin)/step)));
  if(nx>200||ny>200)return result;
  for(int ix=0;ix<=nx;++ix)for(int iy=0;iy<=ny;++iy) {
    if(expired()){result.reason="motion_check_budget_exhausted";result.completion=MotionSweepCompletion::BudgetExhausted;return result;}
    const double x=xmin+(xmax-xmin)*ix/nx,y=ymin+(ymax-ymin)*iy/ny;
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
  for(double x:{xmin-extent_x,xmax+extent_x})for(double y:{ymin-extent_y,ymax+extent_y}) {
    Eigen::Vector3d p(body.x()+c*x-s*y,body.y()+s*x+c*y,0.);
    lower=lower.cwiseMin(p);upper=upper.cwiseMax(p);
  }
  lower.z()=result.min_z;upper.z()=result.max_z;
  Eigen::Vector3i low,high;
  if(oriented_box) {
    // Match the native OBB query's face-contact convention. floor(lower)
    // alone would omit the voxel immediately below an exact boundary.
    const Eigen::Vector3d middle=(lower+upper)*.5,half=(upper-lower)*.5;
    try {
      for(int axis=0;axis<3;++axis) {
        const auto span=verticalVoxelSpan(middle[axis],half[axis],half[axis],map.getResolution());
        low[axis]=span.low;high[axis]=span.high;
      }
    } catch(const std::invalid_argument&) {return result;}
  } else {map.posToIndex(lower,low);map.posToIndex(upper,high);}
  const Eigen::Matrix<std::int64_t,3,1> size=(high-low+Eigen::Vector3i::Ones()).cast<std::int64_t>();
  if((size.array()<=0).any())return result;
  if(size.prod()>200000){result.reason="motion_volume_budget_exhausted";result.completion=MotionSweepCompletion::BudgetExhausted;return result;}
  map.beginCollisionSnapshotProof();
  int blocked_state=0;
  // Circumscribed XY cell disc means corner-intersecting voxels are included.
  // The OBB branch instead uses the same exact rectangle/cell SAT as GridMap.
  const double padded_radius=radius+map.getResolution()*std::sqrt(.5);
  const Eigen::Vector3d sweep_center(body.x()+c*(xmin+xmax)*.5-s*(ymin+ymax)*.5,
                                    body.y()+s*(xmin+xmax)*.5+c*(ymin+ymax)*.5,body.z());
  for(int x=low.x();x<=high.x();++x)for(int y=low.y();y<=high.y();++y) {
    Eigen::Vector3d center;map.indexToPos({x,y,low.z()},center);
    const double dx=center.x()-body.x(),dy=center.y()-body.y(),u=c*dx+s*dy,v=-s*dx+c*dy;
    bool inside=oriented_box && orientedBoxIntersectsVoxelXY(sweep_center,body_yaw,Eigen::Vector3i(x,y,low.z()),
        map.getResolution(),xmax-xmin+2.*extent_x,ymax-ymin+2.*extent_y);
    if(!oriented_box)for(double offset:{-shape.offset,shape.offset}) {
      const double qx=std::max({xmin+offset-u,0.,u-(xmax+offset)}),qy=std::max({ymin-v,0.,v-ymax});
      inside=inside||(qx*qx+qy*qy<=padded_radius*padded_radius);
    }
    if(!inside)continue;
    for(int z=low.z();z<=high.z();++z) {
      if(expired()){result.reason="motion_check_budget_exhausted";result.completion=MotionSweepCompletion::BudgetExhausted;return result;}
      ++result.unique_voxels;int raw=2;const int state=map.collisionSnapshotStatus({x,y,z},&raw);
      ++result.state_counts[raw==0?0:raw==1?1:raw<0?3:2];
      // An earlier unknown/outside cell cannot hide a definite occupied voxel
      // and suppress the native blocked-entry replan event. Continue only the
      // SAME bounded sweep; an occupied hit still exits immediately. Time and
      // volume budgets remain unchanged, including the blocked timeout exit.
      blocked_state=mergeCollisionStatus(blocked_state,state);
      if(blocked_state==1){result.reason="motion_sweep_occupied";result.completion=MotionSweepCompletion::Blocked;return result;}
    }
  }
  result.query_complete=true;
  if(blocked_state!=0){result.reason=blocked_state<0?"motion_sweep_outside_map":"motion_sweep_unknown_or_expired";return result;}
  if(!map.collisionSnapshotSourceFresh()) {result.reason="motion_sensor_source_expired";return result;}
  result.valid=result.unique_voxels>0;
  if(result.valid)result.completion=MotionSweepCompletion::Clear;
  result.reason=result.valid?(map.planningRequiresObservedFree()?"observed_free_motion_sweep":
      "official_occupied_motion_sweep"):"empty_motion_sweep";return result;
}

// Conservative, bounded reference screening. The underlying production map,
// original support and measured braking model are identical to command checks.
// Full SCAN curve admission and fresh actual-body command validation remain
// mandatory; a straight corridor result does not authorize a future curve.
inline MotionSweepResult validateStraightBrakingCorridor(GridMap& map,const MotionSupport& support,
    const BrakingModel& model,const Eigen::Vector3d& start,const Eigen::Vector3d& target,
    const std::string& frame,double budget_s=.08,std::optional<double> frozen_body_yaw={}) {
  const Eigen::Vector2d delta=(target-start).head<2>();
  if(!start.allFinite()||!target.allFinite()||delta.norm()<1e-6||delta.norm()>2.)
    return {false,"invalid_planning_corridor"};
  const double direction=std::atan2(delta.y(),delta.x());
  const double yaw=frozen_body_yaw.value_or(direction);
  if(frozen_body_yaw&&(!std::isfinite(yaw)||model.kinematic_profile!=KinematicProfile::HolonomicXYYaw))
    return {false,"planning_heading_profile_mismatch"};
  geometry_msgs::msg::Twist demand,measured;
  demand.linear.x=model.max_speed*std::cos(direction-yaw);
  demand.linear.y=frozen_body_yaw?model.max_speed*std::sin(direction-yaw):0.;
  measured.linear.x=model.max_speed*std::cos(direction);measured.linear.y=model.max_speed*std::sin(direction);
  if(frozen_body_yaw)
    return validateMotionSweep(map,support,model,start,yaw,measured,demand,frame,budget_s,0.,
        Eigen::Rotation2Dd(-yaw)*delta);
  return validateMotionSweep(map,support,model,start,yaw,measured,demand,frame,budget_s,delta.norm());
}
} // namespace scan_planner
