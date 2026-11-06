#include <gtest/gtest.h>
#include <plan_manage/stair_forward_approach.hpp>
#include <plan_manage/stair_attitude_scope.hpp>
#include <bspline_opt/trajectory_timing.hpp>

using namespace scan_planner;
using Point=Eigen::Vector3d;

// Production GridMap, no ROS graph/SDK. This is a controlled narrow-flight
// reconstruction, NOT a claim that the complete 10-26 scan was recorded.
struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto map=std::make_shared<GridMap>();auto& p=map->mp_;auto& d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={160,64,100};p.map_origin_idx_.setZero();
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.box_nominal_length_=.93;p.box_nominal_width_=.30;p.stair_planar_clearance_=.025;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.double_cylinder_radius_=.2;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;p.map_sliding_en_=false;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.planning_collision_policy_="official";p.require_observed_free_=false;p.frame_id_="odom";
    map->updateMapBoundaryFromIndex();const auto n=160U*64U*100U;
    d.occupancy_buffer_.assign(n,-1.);d.occupancy_buffer_inflate_.assign(n,0);
    d.occupancy_buffer_inflate_cnt_.assign(n,0);d.planning_occupancy_buffer_inflate_.assign(n,0);
    d.planning_occupancy_buffer_inflate_cnt_.assign(n,0);map->rebuildInflationOffsets();return map;
  }
};
namespace {
std::shared_ptr<const PlanningSupport> source(double grade,bool landing=false) {
  d1max_planning_interfaces::msg::SupportReference s;
  s.support_reference_id="flight";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.frame_id="odom";s.version.segment_id="flight";s.floor_id="stair_lower";
  s.segment_kind=landing?"landing":grade>0.?"stair_up":"stair_down";s.required_mode="stair";
  s.reason="original_stair_surface_returns_associated";s.verified=true;
  s.support_xy_radius_m=.2;s.body_reference_height_m=.55;
  for(int x=-15;x<=60;++x)for(int y=-12;y<=12;++y) {
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;p.z=.6+grade*p.x;s.support_ground_xyz.push_back(p);
  }
  return std::make_shared<const PlanningSupport>(s);
}
std::vector<Point> route(double grade) {
  return {{0.,.12,1.15},{0.,0.,1.15},{.5,0.,1.15+.5*grade},
          {1.,0.,1.15+grade},{2.,0.,1.15+2.*grade}};
}
std::vector<Point> approach(const std::vector<Point>& r,const OccupancyQuery& q,
                           const PlanningSupport* s,const std::string& frame="odom") {
#if defined(D1MAX_STAIR_FORWARD_CONSTANT_GRADE_BASELINE)
  return constantGradeStairForwardApproach(r,.05,q,Point::Zero(),0.,true,s,frame);
#elif defined(D1MAX_STAIR_APPROACH_BASELINE)
  (void)s;(void)frame;
  return measuredForwardApproach(r,.05,q,Point::Zero(),0.,true);
#else
  return measuredStairForwardApproach(r,.05,q,Point::Zero(),0.,true,s,frame);
#endif
}
void same(const std::vector<Point>& a,const std::vector<Point>& b) {
  ASSERT_EQ(a.size(),b.size());for(std::size_t i=0;i<a.size();++i)EXPECT_TRUE(a[i].isApprox(b[i]));
}
}

TEST(StairForwardApproach,NativeNarrowFlightUsesForwardEntryInsteadOfSidewaysProjectionBothDirections) {
  for(double grade:{.35,-.35}) {
    auto map=GridMapTestAccess::make();auto support=source(grade);ASSERT_TRUE(support->valid);
    for(int x=-12;x<=65;++x)for(int z=5;z<=44;++z)for(double y:{-.475,.475})
      map->setOccupied({x*.05,y,z*.05});
    // Literal step returns remain occupied; the body tilts with the scoped
    // source-supported flight instead of requiring an empty/flattened floor.
    for(int x=-12;x<=65;++x)for(int y=-9;y<=9;++y)
      map->setOccupied({x*.05,y*.05,.6+grade*.3*std::floor(x*.05/.3)});
    const auto r=route(grade);
    MeasuredBodyPose body{r.front(),Eigen::Quaterniond(Eigen::AngleAxisd(-std::atan(grade),Point::UnitY())),100.,"odom"};
    StairAttitudeScope scope(*map,support,body);
    const auto q=[&](const Point& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
    ASSERT_EQ(q(r.front(),0.),0);
    ASSERT_EQ(q(r.front(),-M_PI/2.),1); // Unneeded lateral connector rotates the long box into a wall.
    const auto shaped=approach(r,q,support.get());
    ASSERT_GT(shaped.size(),r.size());
    EXPECT_TRUE(shaped.front().isApprox(r.front()));EXPECT_TRUE(shaped.back().isApprox(r.back()));
    const auto entry=shaped[1]-shaped[0];EXPECT_GT(entry.x(),0.);EXPECT_LT(std::abs(std::atan2(entry.y(),entry.x())),.03);
    for(std::size_t i=1;i<shaped.size();++i)
      EXPECT_TRUE(collisionFreeSegment(shaped[i-1],shaped[i],.025,q))<<i;
    DiscreteReference local;local.set(shaped);
    const auto seed=sampleReferenceSeed(local,.3,.35,0.,.3,.2);
    const Point end_velocity=.3*Point(1.,0.,grade).normalized();
    const CubicMotionBoundary boundary{r.front(),Point::Zero(),Point::Zero(),r.back(),end_velocity,Point::Zero()};
    const auto controls=fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
    UniformBspline curve(controls,3,seed.dt);
    SplineHeadingContract heading;heading.measured_heading_enabled=true;
    heading.measured_yaw=0.;heading.body_extent=map->queryBodyExtent();heading.align_then_forward=true;
    const auto collision=checkWholeSplineCollision(curve,.05,q,50000,heading);
    EXPECT_EQ(collision.state,SplineCollisionState::Clear);
    EXPECT_TRUE(curve.evaluateDeBoorT(0.).isApprox(r.front()));
    EXPECT_TRUE(curve.getDerivative().evaluateDeBoorT(0.).isZero(1e-9));
  }
}
TEST(StairForwardApproach,RealBlockingWallIsNotClearedOrAccepted) {
  auto s=source(.35);const auto r=route(.35);auto map=GridMapTestAccess::make();
  for(int y=-12;y<=12;++y)for(int z=15;z<=40;++z)map->setOccupied({.425,y*.05,z*.05});
  MeasuredBodyPose body{r.front(),Eigen::Quaterniond::Identity(),100.,"odom"};StairAttitudeScope scope(*map,s,body);
  const auto q=[&](const Point& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
  same(approach(r,q,s.get()),r);EXPECT_EQ(q({.425,0.,1.3},0.),1);
}
TEST(StairForwardApproach,MissingForeignAndLandingSupportDoNotReshape) {
  const auto r=route(.35);const OccupancyQuery free=[](const Point&,double){return 0;};auto s=source(.35);
  same(approach(r,free,nullptr),r);same(approach(r,free,s.get(),"other"),r);
  auto landing=source(.35,true);same(approach(r,free,landing.get()),r);
}
TEST(StairForwardApproach,DoesNotCutTheNextCornerOrFlattenItsHeight) {
  auto s=source(.35);const OccupancyQuery free=[](const Point&,double){return 0;};
  const std::vector<Point> r{{0.,.12,1.15},{0.,0.,1.15},{.5,0.,1.325},{.5,.4,1.4},{.5,.8,1.4}};
  const auto shaped=approach(r,free,s.get());ASSERT_GT(shaped.size(),r.size());
  ASSERT_GE(shaped.size(),3U);
  EXPECT_TRUE(shaped[shaped.size()-3].isApprox(r[2]));
  EXPECT_TRUE(shaped[shaped.size()-2].isApprox(r[3]));EXPECT_TRUE(shaped.back().isApprox(r.back()));
}
TEST(StairForwardApproach,FlatSupportAndNoProjectionConnectorKeepExistingGeometry) {
  auto wire=source(.0)->source;wire.floor_id="floor1";wire.segment_kind="floor";wire.required_mode="general";
  wire.max_support_slope_rad=.15;wire.max_support_step_m=.05;PlanningSupport s(wire);ASSERT_TRUE(s.valid);
  const auto r=route(0.);const OccupancyQuery free=[](const Point&,double){return 0;};
  same(measuredStairForwardApproach(r,.05,free,Point::Zero(),0.,true,&s,"odom"),r);
  same(measuredReferenceForwardApproach(r,.05,free,Point::Zero(),0.,true,&s,"odom"),
       measuredForwardApproach(r,.05,free,Point::Zero(),0.,true));
  auto slope=source(.35);same(measuredStairForwardApproach(route(.35),.05,free,Point::Zero(),0.,false,slope.get(),"odom"),route(.35));
}
TEST(StairForwardApproach,LandingOwnershipStopsShapingBeforeThePlatform) {
  auto wire=source(.35)->source;wire.reason="original_transition_surface_returns_associated";
  d1max_planning_interfaces::msg::SupportRegion flight,landing;
  flight.segment_id="flight";flight.floor_id="stair_lower";flight.segment_kind="stair_up";flight.required_mode="stair";
  flight.begin_index=0;flight.end_index=26U*25U;
  landing=flight;landing.segment_id="landing";landing.segment_kind="landing";
  landing.begin_index=flight.end_index;landing.end_index=wire.support_ground_xyz.size();
  wire.regions={flight,landing};PlanningSupport support(wire);ASSERT_TRUE(support.valid);
  EXPECT_EQ(support.stairFlightAt({.5,0.,1.325}),std::optional<std::string>("flight"));
  EXPECT_FALSE(support.stairFlightAt({.55,0.,1.3425}));
  const std::vector<Point> r{{0.,.12,1.15},{0.,0.,1.15},{.5,0.,1.325},{1.,0.,1.5},{2.,0.,1.85}};
  const OccupancyQuery free=[](const Point&,double){return 0;};
  const auto shaped=approach(r,free,&support);ASSERT_GT(shaped.size(),r.size());
  EXPECT_TRUE(shaped[shaped.size()-3].isApprox(r[2]));
  EXPECT_TRUE(shaped[shaped.size()-2].isApprox(r[3]));EXPECT_TRUE(shaped.back().isApprox(r.back()));
}

namespace {
double treadHeight(double x,double direction) {return .6+direction*.12*std::floor((x+1e-9)/.3);}
std::shared_ptr<const PlanningSupport> treadSource(double direction,bool gap=false) {
  auto wire=source(direction*.4)->source;wire.support_ground_xyz.clear();
  for(int x=-15;x<=60;++x)for(int y=-12;y<=12;++y) {
    if(gap&&x>=2&&x<=15)continue;
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;p.z=treadHeight(p.x,direction);
    wire.support_ground_xyz.push_back(p);
  }
  return std::make_shared<const PlanningSupport>(wire);
}
std::vector<Point> treadRoute(double direction) {
  std::vector<Point> r{{0.,.12,1.15},{0.,0.,1.15}};
  for(double x:{.2,.3,.5,.6,.8,.9,1.1,1.2,1.4,1.5,1.7,1.8,2.})
    r.emplace_back(x,0.,treadHeight(x,direction)+.55);
  return r;
}
}

TEST(StairForwardApproach,LiteralFlatTreadsAndRisersHaveForwardEntryBothDirections) {
  for(double direction:{1.,-1.}) {
    auto map=GridMapTestAccess::make();const auto support=treadSource(direction);ASSERT_TRUE(support->valid);
    for(int x=-12;x<=65;++x)for(int z=-20;z<=60;++z)for(double y:{-.475,.475})
      map->setOccupied({x*.05,y,z*.05});
    for(int x=-12;x<=65;++x)for(int y=-9;y<=9;++y)
      map->setOccupied({x*.05,y*.05,treadHeight(x*.05,direction)});
    const auto r=treadRoute(direction);
    MeasuredBodyPose body{r.front(),Eigen::Quaterniond::Identity(),100.,"odom"};
    StairAttitudeScope scope(*map,support,body);
    const auto q=[&](const Point& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
    ASSERT_EQ(q(r.front(),0.),0);ASSERT_EQ(q(r.front(),-M_PI/2.),1);
    const auto shaped=approach(r,q,support.get());ASSERT_GT(shaped.size(),r.size());
    EXPECT_TRUE(shaped.front().isApprox(r.front()));EXPECT_TRUE(shaped.back().isApprox(r.back()));
    const auto entry=shaped[1]-shaped[0];EXPECT_GT(entry.x(),0.);
    EXPECT_LT(std::abs(std::atan2(entry.y(),entry.x())),.03);
    bool has_level=false,has_rise=false;
    for(std::size_t i=1;i<shaped.size();++i) {
      EXPECT_TRUE(collisionFreeSegment(shaped[i-1],shaped[i],.025,q));
      if(shaped[i].x()>.8)continue;
      EXPECT_EQ(support->stairFlightAt(shaped[i]),std::optional<std::string>("flight"));
      const auto d=shaped[i]-shaped[i-1];
      has_level|=d.head<2>().norm()>.005&&std::abs(d.z())<1e-10;
      has_rise|=direction*d.z()>.01;
      EXPECT_GE(direction*d.z(),-1e-10);
    }
    EXPECT_TRUE(has_level);EXPECT_TRUE(has_rise); // Not a replacement ramp.
    DiscreteReference discrete;discrete.set(shaped);
    const auto seed=sampleReferenceSeed(discrete,.3,.35,0.,.3,.2);
    const CubicMotionBoundary boundary{r.front(),Point::Zero(),Point::Zero(),r.back(),Point(.3,0.,0.),Point::Zero()};
    const auto controls=fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
    UniformBspline curve(controls,3,seed.dt);
    SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.measured_yaw=0.;
    heading.body_extent=map->queryBodyExtent();heading.align_then_forward=true;
    EXPECT_EQ(checkWholeSplineCollision(curve,.05,q,50000,heading).state,SplineCollisionState::Clear);
    EXPECT_TRUE(curve.evaluateDeBoorT(0.).isApprox(r.front()));
    EXPECT_TRUE(curve.getDerivative().evaluateDeBoorT(0.).isZero(1e-9));
  }
}

TEST(StairForwardApproach,MeasuredStartHeightIsNotSnappedAndNoMissingSupportBridge) {
  const OccupancyQuery free=[](const Point&,double){return 0;};auto s=treadSource(1.);
  auto r=treadRoute(1.);r.front().z()+=.025;
  const auto shaped=approach(r,free,s.get());ASSERT_GT(shaped.size(),r.size());
  EXPECT_TRUE(shaped.front().isApprox(r.front()));EXPECT_TRUE(shaped.back().isApprox(r.back()));
  auto gap=treadSource(1.,true);const auto bounded=approach(r,free,gap.get());
  // The first 20 cm can remain associated with real support. Shaping that
  // prefix is allowed; the missing-support region and original tail must not
  // be bridged by an invented approach.
  ASSERT_GE(bounded.size(),r.size());
  const auto tail=bounded.size()-(r.size()-3);
  for(std::size_t i=0;i<tail;++i) {
    EXPECT_LE(bounded[i].x(),.2+1e-10);
    EXPECT_TRUE(gap->stairFlightAt(bounded[i]).has_value());
  }
  for(std::size_t i=3;i<r.size();++i)EXPECT_TRUE(bounded[tail+i-3].isApprox(r[i]));
  const OccupancyQuery wall=[](const Point& p,double){return p.x()>.35&&p.x()<.5?1:0;};
  same(approach(r,wall,s.get()),r);
}

TEST(StairForwardApproach,MixedStairWindowActualFloorUsesOriginalFlatInitializerOnce) {
  auto wire=source(0.)->source;wire.reason="original_transition_surface_returns_associated";
  wire.floor_id="floor1";wire.segment_kind="floor";wire.required_mode="general";
  wire.version.segment_id="floor";
  d1max_planning_interfaces::msg::SupportRegion floor,stair;
  floor.segment_id="floor";floor.floor_id="floor1";floor.segment_kind="floor";floor.required_mode="general";
  floor.begin_index=0;floor.end_index=26U*25U;
  stair=floor;stair.segment_id="flight";stair.floor_id="stair_lower";
  stair.segment_kind="stair_up";stair.required_mode="stair";
  stair.begin_index=floor.end_index;stair.end_index=wire.support_ground_xyz.size();wire.regions={floor,stair};
  PlanningSupport mixed(wire);ASSERT_TRUE(mixed.valid);ASSERT_TRUE(mixed.isStair());
  const auto r=route(0.);const OccupancyQuery free=[](const Point&,double){return 0;};
  ASSERT_FALSE(mixed.stairFlightAt(r.front()));
  same(measuredReferenceForwardApproach(r,.05,free,Point::Zero(),0.,true,&mixed,"odom"),
       measuredForwardApproach(r,.05,free,Point::Zero(),0.,true));
  auto stairs=treadSource(1.);const auto t=treadRoute(1.);
  same(measuredReferenceForwardApproach(t,.05,free,Point::Zero(),0.,true,stairs.get(),"odom"),
       measuredStairForwardApproach(t,.05,free,Point::Zero(),0.,true,stairs.get(),"odom"));
}
