#include <gtest/gtest.h>
#include <plan_manage/stair_attitude_scope.hpp>
#include <plan_manage/reference_target.hpp>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/trajectory_timing.hpp>

// Production GridMap/search/support geometry. No ROS node or mock occupancy.
struct GridMapTestAccess {
  static void configure(GridMap& m) {
    auto& p=m.mp_;auto& d=m.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={100,100,80};
    p.map_origin_idx_={0,0,0};p.map_size_=p.map_voxel_num_.cast<double>()*.05;
    p.map_sliding_en_=false;m.updateMapBoundaryFromIndex();
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.double_cylinder_radius_=.2;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.planning_collision_policy_="official";p.require_observed_free_=false;p.frame_id_="odom";
    const auto n=100U*100U*80U;d.occupancy_buffer_.assign(n,-1.);
    d.occupancy_buffer_inflate_.assign(n,0);d.occupancy_buffer_inflate_cnt_.assign(n,0);
    d.planning_occupancy_buffer_inflate_.assign(n,0);d.planning_occupancy_buffer_inflate_cnt_.assign(n,0);
    m.rebuildInflationOffsets();
  }
  static auto raw(const GridMap& m){return m.md_.occupancy_buffer_;}
};

namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void setup(BsplineOptimizer& o,const GridMap::Ptr& map) {
    o.order_=3;o.lambda1_=1.;o.lambda2_=1.;o.lambda3_=.1;o.lambda4_=1.;
    o.lambda_reference_=1.;o.lambda_planar_curvature_=.2;o.preferred_turn_radius_=1.;
    o.dist0_=.28;o.max_vel_=.3;o.max_acc_=.35;o.setEnvironment(map);
    o.a_star_=std::make_shared<AStar>();o.a_star_->initGridMapFromEnvironment(map);
  }
};
}

namespace {
using namespace scan_planner;
using P=Eigen::Vector3d;
std::shared_ptr<const PlanningSupport> surface(bool flat=false,bool upper=false) {
  d1max_planning_interfaces::msg::SupportReference s;
  s.support_reference_id="source";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.frame_id="odom";s.floor_id=flat?"floor1":"stair_lower";s.segment_kind=flat?"floor":"stair_up";
  s.required_mode=flat?"general":"stair";
  s.reason=flat?"original_floor_support_classified":"original_stair_surface_returns_associated";
  s.verified=true;s.support_xy_radius_m=.2;s.body_reference_height_m=.5;
  s.max_support_step_m=flat?.05:0.;s.max_support_slope_rad=flat?.15:0.;
  for(int level=0;level<=(upper?1:0);++level)
    for(int x=-30;x<=30;++x)for(int y=-15;y<=15;++y) {
      geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;p.z=level*1.5+(flat?0.:.2*p.x);
      s.support_ground_xyz.push_back(p);
    }
  return std::make_shared<const PlanningSupport>(s);
}
MeasuredBodyPose measured(){return {P(0.,0.,.5),Eigen::Quaterniond::Identity(),1.,"odom"};}

// A narrow flight entrance after an open turning landing. This is a bounded
// native geometry counterexample, not a measurement of the user's stairs.
std::shared_ptr<const PlanningSupport> landingSupport() {
  auto s=surface()->source;s.support_ground_xyz.clear();
  for(int x=-48;x<=48;++x)for(int y=-48;y<=48;++y) {
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;p.z=0.;
    s.support_ground_xyz.push_back(p);
  }
  return std::make_shared<const PlanningSupport>(s);
}
GridMap::Ptr turningLanding() {
  auto m=std::make_shared<GridMap>();GridMapTestAccess::configure(*m);
  for(int y=4;y<49;++y)for(double x:{-.325,.325})
    m->setOccupied({x,(y+.5)*.05,.5});
  return m;
}
ReferenceTargetOptions endpointOnlyOptions() {
  ReferenceTargetOptions o;o.forward_margin=o.backward_margin=0.;
  o.exit_margin=.1;o.sample_step=.025;return o;
}
ReferenceTargetOptions sourceBackedEndpointOptions(const GridMap& map) {
  auto o=endpointOnlyOptions();o.require_chord_heading=!map.hasQuerySupportHeightProvider();return o;
}
DiscreteReference landingReference(const P& start,const P& end) {
  DiscreteReference path;path.set({start,P(end.x(),start.y(),start.z()),end});return path;
}
std::shared_ptr<const PlanningSupport> finiteAnchorSupport() {
  auto s=surface()->source;s.support_ground_xyz.clear();s.verified=false;
  s.stair_model_hash=std::string(64,'c');
  d1max_planning_interfaces::msg::StairSurfaceModel m;
  m.surface_id="finite-rebound-flight";m.segment_id="lower";m.kind="continuous_support_envelope";
  m.pose.orientation.w=1.;m.support_grade=.2;m.support_intercept_m=0.;
  m.contact_tolerance_m=.02;m.height_association_m=.25;
  for(const auto& xy:std::vector<Eigen::Vector2d>{{-1.5,-.75},{1.5,-.75},{1.5,.75},{-1.5,.75}}) {
    geometry_msgs::msg::Point p;p.x=xy.x();p.y=xy.y();m.polygon.push_back(p);
  }
  s.stair_model_surfaces.push_back(m);return std::make_shared<const PlanningSupport>(s);
}
GridMap::Ptr reboundStairMap() {
  auto m=std::make_shared<GridMap>();GridMapTestAccess::configure(*m);
  for(int x=-49;x<49;++x)for(int y=-49;y<49;++y)
    m->setOccupied({(x+.5)*.05,(y+.5)*.05,.1+.2*(x+.5)*.05});
  return m;
}
}

TEST(StairSearchSupport,SameXYKeepsSourceLevelAndDoesNotManufactureMissingSupport) {
  GridMap m;GridMapTestAccess::configure(m);const auto raw=GridMapTestAccess::raw(m);
  StairAttitudeScope scope(m,surface(false,true),measured());
  ASSERT_TRUE(m.hasQuerySupportHeightProvider());
  const auto lower=m.querySupportBodyHeight({.5,0.,.62});
  const auto upper=m.querySupportBodyHeight({.5,0.,2.12});
  ASSERT_TRUE(lower&&upper);EXPECT_NEAR(*lower,.6,1e-10);EXPECT_NEAR(*upper,2.1,1e-10);
  EXPECT_FALSE(m.querySupportBodyHeight({.5,0.,1.3}));
  EXPECT_FALSE(m.querySupportBodyHeight({10.,0.,.5}));
  EXPECT_EQ(raw,GridMapTestAccess::raw(m));
}

TEST(StairSearchSupport,FlatAndNestedScopesAndNewSnapshotsRestoreOnlyOwnSource) {
  GridMap m,copy;GridMapTestAccess::configure(m);GridMapTestAccess::configure(copy);
  EXPECT_FALSE(m.hasQuerySupportHeightProvider());
  {
    StairAttitudeScope stairs(m,surface(),measured());EXPECT_TRUE(m.hasQuerySupportHeightProvider());
    {StairAttitudeScope flat(m,surface(true),measured());EXPECT_FALSE(m.hasQuerySupportHeightProvider());}
    EXPECT_TRUE(m.hasQuerySupportHeightProvider());
    m.copyCollisionSnapshotTo(copy,1000000000LL,std::chrono::steady_clock::now());
    EXPECT_FALSE(copy.hasQuerySupportHeightProvider());
  }
  EXPECT_FALSE(m.hasQuerySupportHeightProvider());
}

TEST(StairSearchSupport,UnavailableSourceKeepsExactNativeFlatSearchAndActualEndpoints) {
  auto m=std::make_shared<GridMap>();GridMapTestAccess::configure(*m);m->setOccupied({0.,0.,.5});
  const P start(-1.2,0.,.5),end(1.2,0.,.5);
  AStar search;search.initGridMapFromEnvironment(m);
  ASSERT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::SUCCESS);
  const auto before=search.getPath();const auto raw=GridMapTestAccess::raw(*m);
  m->exchangeQuerySupportHeightProvider([](const auto&)->std::optional<double>{return {};});
  ASSERT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::SUCCESS);
  const auto after=search.getPath();ASSERT_EQ(before.size(),after.size());
  for(std::size_t i=0;i<before.size();++i)EXPECT_TRUE(before[i].isApprox(after[i],0.));
  EXPECT_TRUE(after.front().isApprox(start,0.));EXPECT_TRUE(after.back().isApprox(end,0.));
  m->setOccupied(end);
  EXPECT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::INIT_TARGET_OCCUPIED);
  EXPECT_TRUE(search.getPath().empty());
}

TEST(StairSearchSupport,LegacyTargetChordRejectsLegalNativeTurningLandingEndpoint) {
  auto m=turningLanding();const auto raw=GridMapTestAccess::raw(*m);
  const P start(-1.2,-1.2,.5),end(.025,1.225,.5);
  auto acquired=measured();acquired.position=start;
  StairAttitudeScope scope(*m,landingSupport(),acquired);
  ASSERT_TRUE(m->hasQuerySupportHeightProvider());
  const double tangent=std::acos(-1.)*.5;
  const double chord=std::atan2(end.y()-start.y(),end.x()-start.x());
  ASSERT_EQ(m->getPlanningInflateOccupancy(end,tangent),0);
  ASSERT_EQ(m->getPlanningInflateOccupancy(end,chord),1);
  DiscreteReference reference;reference.set({start,P(end.x(),start.y(),.5),end});
  const auto selected=selectReferenceTarget(reference,start,0.,reference.length(),
    endpointOnlyOptions(),[&](const P& point,double yaw){return m->getPlanningInflateOccupancy(point,yaw);});
  EXPECT_FALSE(selected.valid);EXPECT_GT(selected.occupied_queries,0U);
  AStar search;search.initGridMapFromEnvironment(m);
  search.setSolveBudget(std::make_shared<SolveBudget>(std::chrono::milliseconds(400)));
  ASSERT_EQ(search.AstarSearch(.05,start,end,false,true,{true,true}),ASTAR_RET::SUCCESS)
      <<search.lastSearchReason();
  const auto path=search.getPath();ASSERT_GE(path.size(),2U);
  EXPECT_TRUE(path.front().isApprox(start,0.));EXPECT_TRUE(path.back().isApprox(end,0.));
  for(std::size_t i=1;i<path.size();++i) {
    const P delta=path[i]-path[i-1];const double yaw=std::atan2(delta.y(),delta.x());
    const int count=std::max(1,int(std::ceil(delta.norm()/.00625)));
    for(int k=0;k<=count;++k)
      ASSERT_EQ(m->getPlanningInflateOccupancy(path[i-1]+delta*(double(k)/count),yaw),0)
          <<"edge="<<i<<" sample="<<k;
  }
  EXPECT_EQ(raw,GridMapTestAccess::raw(*m));
}

TEST(StairSearchSupport,SourceBackedLandingSelectsExactEndpointAndOptimizesWholeCurve) {
  auto m=turningLanding();const auto raw=GridMapTestAccess::raw(*m);
  const P start(-1.2,-1.2,.5),end(.025,1.225,.5);
  auto acquired=measured();acquired.position=start;
  StairAttitudeScope scope(*m,landingSupport(),acquired);
  auto reference=landingReference(start,end);
  const auto budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
  const auto occupied=[&](const P& point,double yaw) {
    return budget->allowed()?m->getPlanningInflateOccupancy(point,yaw):-1;
  };
  const auto selected=selectReferenceTarget(reference,start,0.,reference.length(),
      sourceBackedEndpointOptions(*m),occupied);
  ASSERT_TRUE(selected.valid);EXPECT_TRUE(selected.point.isApprox(end,0.));
  BsplineOptimizer optimizer;BsplineOptimizerTestAccess::setup(optimizer,m);optimizer.setSolveBudget(budget);
  SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
  heading.measured_yaw=0.;heading.body_extent=m->queryBodyExtent();optimizer.setPreviewHeadingContract(heading);
  const auto seed=sampleReferenceSeed(reference,.3,.35,0.,0.,.2);
  const CubicMotionBoundary boundary{start,P::Zero(),P::Zero(),end,P::Zero(),P::Zero()};
  auto controls=fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
  optimizer.reference_path_samples_=seed.samples;optimizer.initControlPointsWithSpline(controls,seed.dt);
  ASSERT_TRUE(optimizer.controlPointsInitialized())<<optimizer.lastFailurePhase();
  Eigen::MatrixXd solved;ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(solved,seed.dt))<<optimizer.lastFailurePhase();
  UniformBspline curve(solved,3,seed.dt);
  const auto timing=refineTimingWithFixedBoundary(curve,boundary,.3,.35,
    [&](Eigen::MatrixXd& q,double dt) {
      UniformBspline proposed(q,3,dt);optimizer.ref_pts_.clear();
      for(int i=0;i<=q.cols()-3;++i)optimizer.ref_pts_.push_back(proposed.evaluateDeBoorT(i*dt));
      Eigen::MatrixXd next;const bool ok=optimizer.BsplineOptimizeTrajRefine(q,dt,next);
      if(ok)q=next;
      return ok;
    },8,std::nullopt,[&]{return budget->allowed();},true);
  ASSERT_TRUE(timing.success)<<timing.reason<<" optimizer="<<optimizer.lastFailurePhase();
  EXPECT_TRUE(cubicBoundaryMatches(curve,boundary));
  const auto check=checkWholeSplineCollision(curve,m->getResolution(),occupied,50000,heading);
  EXPECT_EQ(check.state,SplineCollisionState::Clear)<<"time="<<check.time<<" occupancy="<<check.occupancy;
  EXPECT_TRUE(budget->allowed());EXPECT_EQ(raw,GridMapTestAccess::raw(*m));
}

TEST(StairSearchSupport,FlatTargetSelectionRetainsExactHistoricalChordPolicy) {
  auto m=turningLanding();const P start(-1.2,-1.2,.5),end(.025,1.225,.5);
  auto acquired=measured();acquired.position=start;
  StairAttitudeScope scope(*m,surface(true),acquired);
  ASSERT_FALSE(m->hasQuerySupportHeightProvider());
  auto reference=landingReference(start,end);
  const auto occupied=[&](const P& point,double yaw){return m->getPlanningInflateOccupancy(point,yaw);};
  const auto old=selectReferenceTarget(reference,start,0.,reference.length(),endpointOnlyOptions(),occupied);
  const auto current=selectReferenceTarget(reference,start,0.,reference.length(),sourceBackedEndpointOptions(*m),occupied);
  EXPECT_FALSE(current.valid);EXPECT_EQ(old.valid,current.valid);EXPECT_EQ(old.reason,current.reason);
  EXPECT_EQ(old.queries,current.queries);EXPECT_EQ(old.occupied_queries,current.occupied_queries);
  EXPECT_TRUE(old.point.isApprox(current.point,0.));
}

TEST(StairSearchSupport,StairEndpointSelectionCannotHideTrueTangentOccupancyOrSealedPassage) {
  const P start(-1.2,-1.2,.5),end(.025,1.225,.5);
  for(bool blocked_endpoint:{false,true}) {
    auto m=turningLanding();
    if(blocked_endpoint)m->setOccupied(end);
    else for(int x=-6;x<=6;++x)m->setOccupied({(x+.5)*.05,.475,.5});
    const auto raw=GridMapTestAccess::raw(*m);auto acquired=measured();acquired.position=start;
    StairAttitudeScope scope(*m,landingSupport(),acquired);auto reference=landingReference(start,end);
    const auto selected=selectReferenceTarget(reference,start,0.,reference.length(),
      sourceBackedEndpointOptions(*m),[&](const P& point,double yaw){return m->getPlanningInflateOccupancy(point,yaw);});
    if(blocked_endpoint)EXPECT_FALSE(selected.valid);
    else EXPECT_TRUE(selected.valid); // An endpoint alone is deliberately not a path proof.
    AStar search;search.initGridMapFromEnvironment(m);
    search.setSolveBudget(std::make_shared<SolveBudget>(std::chrono::milliseconds(400)));
    EXPECT_NE(search.AstarSearch(.05,start,end,false,true,{true,true}),ASTAR_RET::SUCCESS);
    EXPECT_TRUE(search.getPath().empty());EXPECT_EQ(raw,GridMapTestAccess::raw(*m));
  }
}

TEST(StairSearchSupport,RelaxedEndpointHeadingStillChecksRealRotationOnWholeCurve) {
  auto m=std::make_shared<GridMap>();GridMapTestAccess::configure(*m);
  const P start(0.,0.,.5),end(0.,1.,.5);m->setOccupied({.325,.325,.5});
  const auto raw=GridMapTestAccess::raw(*m);
  StairAttitudeScope scope(*m,landingSupport(),measured());
  DiscreteReference reference;reference.set({start,end});
  const auto occupied=[&](const P& point,double yaw){return m->getPlanningInflateOccupancy(point,yaw);};
  const auto selected=selectReferenceTarget(reference,start,0.,reference.length(),
      sourceBackedEndpointOptions(*m),occupied);
  ASSERT_TRUE(selected.valid);
  ASSERT_EQ(occupied(start,0.),0);ASSERT_EQ(occupied(start,std::acos(-1.)*.5),0);
  ASSERT_EQ(occupied(start,std::acos(-1.)*.25),1); // Only the swept corner hits.
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,6);controls.row(2).setConstant(.5);
  for(int i=0;i<controls.cols();++i)controls(1,i)=(i-1)/3.;
  UniformBspline curve(controls,3,1.);
  EXPECT_TRUE(curve.evaluateDeBoorT(0.).isApprox(start,1e-12));
  EXPECT_TRUE(curve.evaluateDeBoorT(curve.getTimeSum()).isApprox(end,1e-12));
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,occupied).state,SplineCollisionState::Clear);
  SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
  heading.measured_yaw=0.;heading.body_extent=m->queryBodyExtent();
  const auto actual=checkWholeSplineCollision(curve,.05,occupied,50000,heading);
  EXPECT_EQ(actual.state,SplineCollisionState::Collision);EXPECT_EQ(actual.occupancy,1);
  EXPECT_EQ(raw,GridMapTestAccess::raw(*m));
}

TEST(StairSearchSupport,StairFinalEndpointReseedRetainsSameGoalAndTangentCheck) {
  auto m=turningLanding();const P start(-1.2,-1.2,.5),end(.025,1.225,.5);
  auto acquired=measured();acquired.position=start;
  StairAttitudeScope scope(*m,landingSupport(),acquired);auto reference=landingReference(start,end);
  auto options=sourceBackedEndpointOptions(*m);
  options.final_endpoint_reseed=FinalEndpointReseed{end,.2,.15,true};
  const auto occupied=[&](const P& point,double yaw){return m->getPlanningInflateOccupancy(point,yaw);};
  const auto selected=selectReferenceTarget(reference,start,reference.length(),.5,options,occupied);
  ASSERT_TRUE(selected.valid);EXPECT_EQ(selected.reason,"reference_target_final_endpoint_reseed");
  EXPECT_TRUE(selected.point.isApprox(end,0.));
  auto old=options;old.require_chord_heading=true;
  EXPECT_FALSE(selectReferenceTarget(reference,start,reference.length(),.5,old,occupied).valid);
  m->setOccupied(end);
  EXPECT_FALSE(selectReferenceTarget(reference,start,reference.length(),.5,options,occupied).valid);
}

TEST(StairSearchSupport,InternalReboundAnchorsUseSameSourceHeightInBothDirections) {
  // The recorded internal optimizer chord descended 14 cm in just 20 cm of
  // XY, unlike the finite stair surface. It is not an actual robot pose.
  // Keep that measured chord delta; the raw bounded surface here is a native
  // geometric counterexample, not a replay of unavailable live raw returns.
  const P in(0.,0.,.53),out(.115878,-.164802,.389894);
  for(bool modeled:{false,true})for(bool reverse:{false,true}) {
    auto m=reboundStairMap();
    const auto raw=GridMapTestAccess::raw(*m);
    StairAttitudeScope scope(*m,modeled?finiteAnchorSupport():surface(),measured());
    ASSERT_TRUE(m->querySupportBodyHeight(out));
    ASSERT_EQ(m->getPlanningInflateOccupancy(in,-.957968),0);
    ASSERT_EQ(m->getPlanningInflateOccupancy(out,-.957968),1);
    AStar search;search.initGridMapFromEnvironment(m);
    search.setSolveBudget(std::make_shared<SolveBudget>(std::chrono::milliseconds(400)));
    ASSERT_EQ(search.AstarSearch(.05,reverse?out:in,reverse?in:out,true),ASTAR_RET::SUCCESS)
        <<search.lastSearchReason();
    const auto path=search.getPath();ASSERT_GE(path.size(),2U);
    // Both XY and Z are rounded by the unchanged map-aligned native lattice.
    for(const auto& p:path)EXPECT_NEAR(p.z(),.5+.2*p.x(),.025*(1.+.2)+1e-12);
    for(std::size_t i=1;i<path.size();++i) {
      const P delta=path[i]-path[i-1];const double yaw=std::atan2(delta.y(),delta.x());
      const int count=std::max(1,int(std::ceil(delta.norm()/.00625)));
      for(int k=0;k<=count;++k)
        ASSERT_EQ(m->getPlanningInflateOccupancy(path[i-1]+delta*(double(k)/count),yaw),0);
    }
    EXPECT_EQ(raw,GridMapTestAccess::raw(*m));
  }
}

TEST(StairSearchSupport,InternalHeightGuideNeverMovesOccupiedMeasuredReferenceEndpoints) {
  const P clear(0.,0.,.53),blocked(.115878,-.164802,.389894);
  for(bool reverse:{false,true}) {
    auto m=reboundStairMap();const auto raw=GridMapTestAccess::raw(*m);
    StairAttitudeScope scope(*m,finiteAnchorSupport(),measured());
    AStar search;search.initGridMapFromEnvironment(m);
    const auto result=search.AstarSearch(.05,reverse?blocked:clear,reverse?clear:blocked,false,true);
    EXPECT_EQ(result,reverse?ASTAR_RET::INIT_START_OCCUPIED:ASTAR_RET::INIT_TARGET_OCCUPIED);
    EXPECT_TRUE(search.getPath().empty());EXPECT_EQ(raw,GridMapTestAccess::raw(*m));
  }
}

TEST(StairSearchSupport,SupportedInternalHelpersDoNotCrossNativeOccupiedWall) {
  auto m=reboundStairMap();
  for(int x=-49;x<49;++x)for(int z=-39;z<39;++z)
    m->setOccupied({(x+.5)*.05,-.025,(z+.5)*.05});
  const auto raw=GridMapTestAccess::raw(*m);
  StairAttitudeScope scope(*m,finiteAnchorSupport(),measured());
  AStar search;search.initGridMapFromEnvironment(m);
  search.setSolveBudget(std::make_shared<SolveBudget>(std::chrono::milliseconds(400)));
  EXPECT_NE(search.AstarSearch(.05,{-.2,.7,.46},{.2,-.7,.40},true),ASTAR_RET::SUCCESS);
  EXPECT_TRUE(search.getPath().empty());EXPECT_EQ(raw,GridMapTestAccess::raw(*m));
}

TEST(StairSearchSupport,MissingInternalSupportRetainsExactLegacySearchAndOriginalDeadline) {
  auto m=std::make_shared<GridMap>();GridMapTestAccess::configure(*m);
  m->setOccupied({0.,0.,.5});
  const P a(-1.2,0.,.5),b(1.2,0.,.5);
  AStar search;search.initGridMapFromEnvironment(m);
  ASSERT_EQ(search.AstarSearch(.05,a,b,true),ASTAR_RET::SUCCESS);
  const auto legacy=search.getPath();
  m->exchangeQuerySupportHeightProvider([](const auto&)->std::optional<double>{return {};});
  ASSERT_EQ(search.AstarSearch(.05,a,b,true),ASTAR_RET::SUCCESS);
  const auto unsupported=search.getPath();ASSERT_EQ(legacy.size(),unsupported.size());
  for(std::size_t i=0;i<legacy.size();++i)EXPECT_TRUE(legacy[i].isApprox(unsupported[i],0.));
  StairAttitudeScope scope(*m,finiteAnchorSupport(),measured());
  EXPECT_FALSE(m->querySupportBodyHeight({0.,0.,2.})); // Another level is not borrowed.
  auto now=SolveBudget::Clock::now();
  const auto budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(1),[&]{return now;});
  now+=std::chrono::milliseconds(1);
  search.setSolveBudget(budget);
  EXPECT_EQ(search.AstarSearch(.05,a,b,true),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(search.lastSearchTimedOut());EXPECT_TRUE(search.getPath().empty());
}
