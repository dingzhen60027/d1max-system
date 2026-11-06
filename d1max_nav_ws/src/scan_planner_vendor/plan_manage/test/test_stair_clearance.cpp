#include <gtest/gtest.h>
#include <plan_manage/stair_attitude_scope.hpp>
#include <plan_manage/reference_target.hpp>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/trajectory_timing.hpp>

// The fixture owns the production GridMap buffers. It does not substitute a
// FREE callback, delete an obstacle or raise the acquired starting position.
struct GridMapTestAccess {
  static void configure(GridMap& map,double stair_clearance) {
    auto& p=map.mp_;auto& d=map.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={100,80,80};
    p.map_origin_idx_.setZero();p.map_size_=p.map_voxel_num_.cast<double>()*.05;
    p.map_sliding_en_=false;map.updateMapBoundaryFromIndex();
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.40;
    p.box_nominal_length_=.93;p.box_nominal_width_=.30;p.stair_planar_clearance_=stair_clearance;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.double_cylinder_radius_=.2;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.planning_collision_policy_="official";p.require_observed_free_=false;p.frame_id_="odom";
    const auto n=100U*80U*80U;d.occupancy_buffer_.assign(n,-1.);
    d.occupancy_buffer_inflate_.assign(n,0);d.occupancy_buffer_inflate_cnt_.assign(n,0);
    d.planning_occupancy_buffer_inflate_.assign(n,0);d.planning_occupancy_buffer_inflate_cnt_.assign(n,0);
    map.rebuildInflationOffsets();
  }
  static auto raw(const GridMap& map){return map.md_.occupancy_buffer_;}
};

namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void configure(BsplineOptimizer& optimizer,const GridMap::Ptr& map) {
    optimizer.order_=3;optimizer.lambda1_=1.;optimizer.lambda2_=1.;optimizer.lambda3_=.1;
    optimizer.lambda4_=1.;optimizer.lambda_reference_=1.;optimizer.lambda_planar_curvature_=.2;
    optimizer.preferred_turn_radius_=1.;optimizer.dist0_=.28;
    optimizer.max_vel_=.3;optimizer.max_acc_=.35;optimizer.setEnvironment(map);
    optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMapFromEnvironment(map);
  }
};
}

namespace {
using namespace scan_planner;
using Point=Eigen::Vector3d;
constexpr double kSlope=.2;
double treadHeight(double x){return .05*std::floor(x/.25+1e-8);}
double alongFlight(double x,double y){return (x+y)/std::sqrt(2.);}

// Synthetic five-centimetre steps in an intentionally tight straight flight.
// This proves a software contract, not clearance of the user's real staircase.
std::shared_ptr<const PlanningSupport> stairSupport(bool flat=false) {
  d1max_planning_interfaces::msg::SupportReference message;
  message.support_reference_id="classified_source";message.support_hash=std::string(64,'a');
  message.support_map_sha256=std::string(64,'b');message.frame_id="odom";
  message.floor_id=flat?"floor1":"stair_lower";message.segment_kind=flat?"floor":"stair_up";
  message.required_mode=flat?"general":"stair";message.verified=true;
  message.reason=flat?"original_floor_support_classified":"original_stair_surface_returns_associated";
  message.support_xy_radius_m=.2;message.body_reference_height_m=.5;
  message.max_support_step_m=flat?.05:0.;message.max_support_slope_rad=flat?.15:0.;
  for(int x=-38;x<=38;++x)for(int y=-38;y<=38;++y) {
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;
    p.z=flat?0.:treadHeight(alongFlight(p.x,p.y));
    message.support_ground_xyz.push_back(p);
  }
  return std::make_shared<const PlanningSupport>(message);
}
GridMap::Ptr flightMap(double clearance,bool walls=true) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map,clearance);
  for(int x=-38;x<=38;++x) {
    for(int y=-38;y<=38;++y)
      map->setOccupied({x*.05,y*.05,treadHeight(alongFlight(x*.05,y*.05))});
    // The 45-degree corridor deliberately retains exact 5 cm voxel centres.
    // Its two stepped wall fronts are 35.355 cm apart: a software boundary
    // case between the unchanged 40 cm and stair-only 35 cm query widths.
    if(walls)for(int z=-10;z<=30;++z)for(int side:{-6,6})
      map->setOccupied({(x+.5)*.05,(x+side+.5)*.05,(z+.5)*.05});
  }
  return map;
}
MeasuredBodyPose acquired(const Point& body) {
  return {body,Eigen::Quaterniond(Eigen::AngleAxisd(std::acos(-1.)*.25,Point::UnitZ())*
    Eigen::AngleAxisd(-std::atan(kSlope),Point::UnitY())),1.,"odom"};
}
ReferenceTargetOptions productionEndpointOptions(const GridMap& map) {
  ReferenceTargetOptions result;result.forward_margin=result.backward_margin=0.;
  result.exit_margin=.2;result.sample_step=.025;
  result.require_chord_heading=!map.hasQuerySupportHeightProvider();return result;
}
DiscreteReference reference(const Point& start,const Point& goal) {
  DiscreteReference path;path.set({start,goal});return path;
}
}

TEST(StairClearance,NativeTightFlightChangesOnlyExtraXYMarginAndOptimizesWholeCurve) {
  // Both acquired travel directions must traverse the same literal stair
  // map. Reversing the route does not raise the start or remove tread points.
  for(const bool descending:{false,true}) {
  SCOPED_TRACE(descending?"down":"up");
  const double direction=descending?-1.:1.;
  const Point start(-.975*direction,-.975*direction,
                    .5+kSlope*alongFlight(-.975*direction,-.975*direction));
  const Point goal(.975*direction,.975*direction,
                   .5+kSlope*alongFlight(.975*direction,.975*direction));
  const double yaw=std::acos(-1.)*(descending?1.25:.25);
  auto pose=acquired(start);
  if(descending)pose.orientation=pose.orientation*
    Eigen::Quaterniond(Eigen::AngleAxisd(std::acos(-1.),Point::UnitZ()));
  const auto support=stairSupport();ASSERT_TRUE(support->valid);
  auto old=flightMap(.05),current=flightMap(.025);
  const auto before=GridMapTestAccess::raw(*current);
  ASSERT_EQ(GridMapTestAccess::raw(*old),before);
  {
    StairAttitudeScope scope(*old,support,pose);
    EXPECT_EQ(old->getPlanningInflateOccupancy(start,yaw),1);
    const auto path=reference(start,goal);
    EXPECT_FALSE(selectReferenceTarget(path,start,0.,path.length(),productionEndpointOptions(*old),
      [&](const Point& p,double yaw){return old->getPlanningInflateOccupancy(p,yaw);}).valid);
    AStar search;search.initGridMapFromEnvironment(old);
    search.setSolveBudget(std::make_shared<SolveBudget>(std::chrono::milliseconds(400)));
    EXPECT_EQ(search.AstarSearch(.05,start,goal,false,true,{true,true}),ASTAR_RET::INIT_START_OCCUPIED);
  }
  {
    StairAttitudeScope scope(*current,support,pose);
    const auto envelope=current->queryBodyEnvelope(start);
    EXPECT_DOUBLE_EQ(envelope.length,.98);EXPECT_DOUBLE_EQ(envelope.width,.35);
    EXPECT_DOUBLE_EQ(envelope.below,.25);EXPECT_DOUBLE_EQ(envelope.above,.25);
    ASSERT_EQ(current->getPlanningInflateOccupancy(start,yaw),0);
    const auto path=reference(start,goal);
    const auto budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
    const auto occupied=[&](const Point& p,double yaw) {
      return budget->allowed()?current->getPlanningInflateOccupancy(p,yaw):-1;
    };
    const auto target=selectReferenceTarget(path,start,0.,path.length(),productionEndpointOptions(*current),occupied);
    ASSERT_TRUE(target.valid);EXPECT_TRUE(target.point.isApprox(goal,0.));
    AStar search;search.initGridMapFromEnvironment(current);search.setSolveBudget(budget);
    ASSERT_EQ(search.AstarSearch(.05,start,goal,false,true,{true,true}),ASTAR_RET::SUCCESS)<<search.lastSearchReason();
    ASSERT_FALSE(search.getPath().empty());
    EXPECT_TRUE(search.getPath().front().isApprox(start,0.));EXPECT_TRUE(search.getPath().back().isApprox(goal,0.));
    BsplineOptimizer optimizer;BsplineOptimizerTestAccess::configure(optimizer,current);optimizer.setSolveBudget(budget);
    SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
    heading.measured_yaw=yaw;heading.body_extent=current->queryBodyExtent();optimizer.setPreviewHeadingContract(heading);
    const auto seed=sampleReferenceSeed(path,.3,.35,0.,0.,.2);
    const CubicMotionBoundary boundary{start,Point::Zero(),Point::Zero(),goal,Point::Zero(),Point::Zero()};
    auto controls=fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
    optimizer.reference_path_samples_=seed.samples;optimizer.initControlPointsWithSpline(controls,seed.dt);
    ASSERT_TRUE(optimizer.controlPointsInitialized())<<optimizer.lastFailurePhase();
    Eigen::MatrixXd solved;ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(solved,seed.dt))<<optimizer.lastFailurePhase();
    UniformBspline curve(solved,3,seed.dt);
    const auto timing=refineTimingWithFixedBoundary(curve,boundary,.3,.35,
      [&](Eigen::MatrixXd& q,double dt) {
        UniformBspline proposed(q,3,dt);optimizer.ref_pts_.clear();
        for(int i=0;i<=q.cols()-3;++i)optimizer.ref_pts_.push_back(proposed.evaluateDeBoorT(i*dt));
        Eigen::MatrixXd next;const bool accepted=optimizer.BsplineOptimizeTrajRefine(q,dt,next);
        if(accepted)q=next;
        return accepted;
      },8,std::nullopt,[&]{return budget->allowed();},true);
    ASSERT_TRUE(timing.success)<<timing.reason<<" phase="<<optimizer.lastFailurePhase();
    EXPECT_TRUE(cubicBoundaryMatches(curve,boundary));
    EXPECT_EQ(checkWholeSplineCollision(curve,.05,occupied,50000,heading).state,SplineCollisionState::Clear);
    EXPECT_TRUE(budget->allowed());
  }
  EXPECT_EQ(before,GridMapTestAccess::raw(*current));EXPECT_EQ(before,GridMapTestAccess::raw(*old));
  EXPECT_DOUBLE_EQ(current->queryBodyEnvelope(start).width,.40); // scope retired
  }
}

TEST(StairClearance,TrueWallHighStepAndCeilingRemainOccupiedAndBlockTheCurve) {
  const Point body(0.,0.,.5),goal(.6,.6,.5+kSlope*alongFlight(.6,.6));
  const double yaw=std::acos(-1.)*.25;
  const Eigen::Matrix3d rotation=acquired(body).orientation.toRotationMatrix();
  for(const Point& local:{Point(0.,.125,0.),Point(.2,0.,-.225),Point(.2,0.,.225)}) {
    auto map=flightMap(.025,false);
    {StairAttitudeScope baseline(*map,stairSupport(),acquired(body));
      ASSERT_EQ(map->getPlanningInflateOccupancy(body,yaw),0);}
    const Point obstacle=body+rotation*local;map->setOccupied(obstacle);
    const auto raw=GridMapTestAccess::raw(*map);
    StairAttitudeScope scope(*map,stairSupport(),acquired(body));
    ASSERT_EQ(map->getPlanningInflateOccupancy(body,yaw),1)<<local.transpose();
    // A* reference anchors may choose another incident heading. Its seed is
    // not an acquired-pose/rotation proof: the complete curve below must
    // still reject the actual occupied body, even if an A* edge can escape.
    const auto seed=sampleReferenceSeed(reference(body,goal),.3,.35,0.,0.,.2);
    const CubicMotionBoundary boundary{body,Point::Zero(),Point::Zero(),goal,Point::Zero(),Point::Zero()};
    UniformBspline curve(fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary),3,seed.dt);
    SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
    heading.measured_yaw=yaw;heading.body_extent=map->queryBodyExtent();
    const auto hit=checkWholeSplineCollision(curve,.05,[&](const Point& p,double yaw) {
      return map->getPlanningInflateOccupancy(p,yaw);
    },50000,heading);
    EXPECT_EQ(hit.state,SplineCollisionState::Collision);EXPECT_EQ(hit.occupancy,1);
    EXPECT_EQ(raw,GridMapTestAccess::raw(*map));
  }
}

TEST(StairClearance,FlatAndAbsentSupportKeepHistoricalBodyAndOccupancy) {
  const Point body(0.,0.,.5);auto old=flightMap(.05),current=flightMap(.025);
  for(const auto& support:{std::shared_ptr<const PlanningSupport>{},stairSupport(true)}) {
    StairAttitudeScope old_scope(*old,support,acquired(body));
    StairAttitudeScope new_scope(*current,support,acquired(body));
    EXPECT_DOUBLE_EQ(current->queryBodyEnvelope(body).length,1.03);
    EXPECT_DOUBLE_EQ(current->queryBodyEnvelope(body).width,.40);
    for(double yaw:{0.,.25,.5,1.,1.5})
      EXPECT_EQ(old->getPlanningInflateOccupancy(body,yaw),current->getPlanningInflateOccupancy(body,yaw));
  }
  EXPECT_EQ(GridMapTestAccess::raw(*old),GridMapTestAccess::raw(*current));
}

TEST(StairClearance,ActualTurningCornerRemainsCollisionDespiteClearEndpointHeadings) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map,.025);
  auto source=stairSupport()->source;
  for(auto& p:source.support_ground_xyz)p.z=0.;
  const auto support=std::make_shared<const PlanningSupport>(source);
  const Point start(0.,0.,.5),goal(0.,.7,.5);map->setOccupied({.325,.325,.5});
  const auto raw=GridMapTestAccess::raw(*map);
  MeasuredBodyPose pose{start,Eigen::Quaterniond::Identity(),1.,"odom"};
  StairAttitudeScope scope(*map,support,pose);
  const auto occupied=[&](const Point& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
  ASSERT_EQ(occupied(start,0.),0);ASSERT_EQ(occupied(start,std::acos(-1.)*.5),0);
  ASSERT_EQ(occupied(start,std::acos(-1.)*.25),1);
  const auto seed=sampleReferenceSeed(reference(start,goal),.3,.35,0.,0.,.2);
  const CubicMotionBoundary boundary{start,Point::Zero(),Point::Zero(),goal,Point::Zero(),Point::Zero()};
  UniformBspline curve(fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary),3,seed.dt);
  SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
  heading.measured_yaw=0.;heading.body_extent=map->queryBodyExtent();
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,occupied,50000,heading).state,SplineCollisionState::Collision);
  EXPECT_EQ(raw,GridMapTestAccess::raw(*map));
}
