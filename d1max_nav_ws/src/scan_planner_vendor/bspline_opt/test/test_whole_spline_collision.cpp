#include <gtest/gtest.h>
#include <bspline_opt/whole_spline_collision.hpp>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <limits>

using scan_planner::UniformBspline;
using scan_planner::SplineCollisionState;
using scan_planner::checkWholeSplineCollision;
using Point=Eigen::Vector3d;

TEST(FrozenBodyHeadingSampling, PreservesProductionSpatialAndTimeLatticeAndRejectsUnboundedInputs) {
  using scan_planner::bodyHeadingSamplingIntervals;
  EXPECT_EQ(bodyHeadingSamplingIntervals(1.,.1,.05),50U);
  EXPECT_EQ(bodyHeadingSamplingIntervals(1.,1.,.05),80U);
  EXPECT_FALSE(bodyHeadingSamplingIntervals(0.,1.,.05));
  EXPECT_FALSE(bodyHeadingSamplingIntervals(1.,-1.,.05));
  EXPECT_FALSE(bodyHeadingSamplingIntervals(1.,1.,0.));
  EXPECT_FALSE(bodyHeadingSamplingIntervals(1000.,1000.,.05));
  EXPECT_FALSE(bodyHeadingSamplingIntervals(1.,std::numeric_limits<double>::quiet_NaN(),.05));
}
TEST(FrozenBodyHeadingSampling, SameOriginalSeedAndLatticeReproduceLateEntryAndBacktracking) {
  scan_planner::SplineHeadingContract c;c.measured_heading_enabled=true;c.measured_yaw=.7;
  const auto velocity=[](double t)->Point {
    if(t>=.2&&t<.3)return {0.,.1,0.};
    if(t>=.4&&t<.5)return {.1,0.,0.};
    return {.001,-.001,0.};
  };
  const auto at=[&](double time,unsigned n=10){return scan_planner::sampledBodyHeadingAt(time,1.,n,c,velocity,[]{return true;});};
  ASSERT_TRUE(at(.35));EXPECT_DOUBLE_EQ(*at(.35),std::atan2(.1,0.));
  ASSERT_TRUE(at(.75));EXPECT_DOUBLE_EQ(*at(.75),0.);
  EXPECT_DOUBLE_EQ(*at(.19),.7); // Earlier actual progress replays the original seed, not latest yaw.
  EXPECT_DOUBLE_EQ(*at(.22),std::atan2(.1,0.)); // Exact entry derivative remains part of the contract.
  EXPECT_DOUBLE_EQ(*at(.75,1),.7); // A different consumer lattice really changes low-speed memory.
}
TEST(FrozenBodyHeadingSampling, HeldHeadingUsesOriginalSeedNotLateralVelocity) {
  scan_planner::SplineHeadingContract c;c.measured_heading_enabled=true;
  c.policy=scan_planner::BodyHeadingPolicy::HoldHeading;c.measured_yaw=-1.1;
  unsigned reads=0;
  const auto yaw=scan_planner::sampledBodyHeadingAt(.75,1.,10,c,[&](double)->Point{++reads;return {0.,1.,0.};},[]{return true;});
  ASSERT_TRUE(yaw);EXPECT_DOUBLE_EQ(*yaw,-1.1);EXPECT_EQ(reads,0U);
}
TEST(FrozenBodyHeadingSampling, CancelInvalidTimeAndNonFiniteDerivativeDoNotProduceHeading) {
  scan_planner::SplineHeadingContract c;c.measured_heading_enabled=true;
  const auto finite=[](double)->Point{return {.1,0.,0.};};
  EXPECT_FALSE(scan_planner::sampledBodyHeadingAt(.2,1.,10,c,finite,[]{return false;}));
  EXPECT_FALSE(scan_planner::sampledBodyHeadingAt(-.01,1.,10,c,finite,[]{return true;}));
  EXPECT_FALSE(scan_planner::sampledBodyHeadingAt(1.01,1.,10,c,finite,[]{return true;}));
  EXPECT_FALSE(scan_planner::sampledBodyHeadingAt(.2,1.,0,c,finite,[]{return true;}));
  EXPECT_FALSE(scan_planner::sampledBodyHeadingAt(.2,1.,50000,c,finite,[]{return true;}));
  EXPECT_FALSE(scan_planner::sampledBodyHeadingAt(.2,1.,10,c,[](double)->Point{
    return {std::numeric_limits<double>::quiet_NaN(),0.,0.};},[]{return true;}));
  unsigned reads=0;
  EXPECT_FALSE(scan_planner::sampledBodyHeadingAt(.8,1.,10,c,finite,[&]{return ++reads<4;}));
}

// Production GridMap/inflation and UniformBspline, without ROS initialization.
struct GridMapTestAccess {
  static GridMap::Ptr make(bool observed_free=false) {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={160,80,64};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.require_observed_free_=observed_free;p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;
    const std::size_t size=160U*80U*64U;
    d.occupancy_buffer_.assign(size,observed_free?-1.:-1.01);d.occupancy_buffer_inflate_.assign(size,0);
    d.occupancy_buffer_inflate_cnt_.assign(size,0);map->rebuildInflationOffsets();return map;
  }
  static void unknown(GridMap &map,const Point &point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);
    map.md_.occupancy_buffer_[map.toAddress(cell)]=-1.01;
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
  static void box(GridMap& map) {
    auto& p=map.mp_;p.collision_model_="oriented_box";p.box_length_=.94;p.box_width_=.30;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    map.rebuildInflationOffsets();
  }
};

namespace {
Eigen::MatrixXd straightPoints() {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,12);
  for(int i=0;i<points.cols();++i) points(0,i)=i*.3;
  points.row(2).setConstant(.55);return points;
}
auto query(GridMap &map) {
  return [&map](const Point &position,double yaw) {return map.getInflateOccupancy(position,yaw);};
}
UniformBspline slowCurve(const Point &velocity) {
  Eigen::MatrixXd points(3,7);
  for(int i=0;i<points.cols();++i)
    points.col(i)=Point(.025,.025,.55)+velocity*((i-1)*.25);
  return UniformBspline(points,3,.25);
}
scan_planner::SplineHeadingContract previewContract(double yaw=0.) {
  scan_planner::SplineHeadingContract contract;
  contract.preview_only_enabled=true;contract.measured_yaw=yaw;return contract;
}
scan_planner::SplineHeadingContract formalContract(bool writer_enabled,double yaw=0.) {
  scan_planner::SplineHeadingContract contract;
  contract.measured_heading_enabled=scan_planner::measuredHeadingGeometryContract(
      false,false,true,writer_enabled,true,false);
  contract.measured_yaw=yaw;
  return contract;
}
} // namespace

TEST(WholeSplineCollision, RealLowObstacleOnlyInLastThirdTriggersCollision) {
  auto map=GridMapTestAccess::make();UniformBspline curve(straightPoints(),3,1.);
  map->setOccupied({2.925,.025,.225}); // Actual occupied 20 cm obstacle-top cell.
  const double duration=curve.getTimeSum();
  for(int i=0;i<=200;++i)
    ASSERT_EQ(map->getInflateOccupancy(curve.evaluateDeBoorT(duration*(2./3.)*i/200.),0.),0);
  const auto result=checkWholeSplineCollision(curve,.05,query(*map));
  EXPECT_EQ(result.state,SplineCollisionState::Collision);
  EXPECT_EQ(result.occupancy,1);
  EXPECT_GT(result.time,duration*2./3.);
  EXPECT_LE(result.time,duration);
  EXPECT_EQ(map->getOccupancy(Point(2.925,.025,.225)),1);
}

TEST(WholeSplineCollision, ClosedLoopDoesNotUseZeroEndpointDistanceAsAStep) {
  auto map=GridMapTestAccess::make();Eigen::MatrixXd points(3,9);
  points << 0,0,0,1,1,0,0,0,0,
            0,0,0,0,1,1,0,0,0,
            .55,.55,.55,.55,.55,.55,.55,.55,.55;
  UniformBspline curve(points,3,1.);
  ASSERT_LT((curve.evaluateDeBoorT(0.)-curve.evaluateDeBoorT(curve.getTimeSum())).norm(),1e-12);
  const Point obstruction=curve.evaluateDeBoorT(2.5);
  map->setOccupied(obstruction);
  ASSERT_EQ(map->getInflateOccupancy(curve.evaluateDeBoorT(0.),0.),0);
  const auto result=checkWholeSplineCollision(curve,.05,query(*map));
  EXPECT_EQ(result.state,SplineCollisionState::Collision);
  EXPECT_EQ(result.occupancy,1);EXPECT_GT(result.queries,1U);
}

TEST(WholeSplineCollision, ClearCurveChecksBothEndpointsBoundedSpacingAndTrueTangent) {
  auto map=GridMapTestAccess::make();Eigen::MatrixXd points=straightPoints();
  for(int i=0;i<points.cols();++i) points(1,i)=.3*std::sin(i*.5);
  UniformBspline curve(points,3,.7);
  std::vector<Point> positions;std::vector<double> yaws;
  const auto result=checkWholeSplineCollision(curve,.05,[&](const Point &p,double yaw) {
    positions.push_back(p);yaws.push_back(yaw);return map->getInflateOccupancy(p,yaw);
  });
  ASSERT_EQ(result.state,SplineCollisionState::Clear);ASSERT_GE(positions.size(),2U);
  EXPECT_EQ(result.queries,positions.size());
  EXPECT_TRUE(positions.front().isApprox(curve.evaluateDeBoorT(0.),1e-12));
  EXPECT_TRUE(positions.back().isApprox(curve.evaluateDeBoorT(curve.getTimeSum()),1e-12));
  auto velocity=curve.getDerivative();
  for(std::size_t i=0;i<positions.size();++i) {
    if(i) EXPECT_LE((positions[i]-positions[i-1]).norm(),.05*.25+1e-12);
    const Point tangent=velocity.evaluateDeBoorT(curve.getTimeSum()*i/(positions.size()-1));
    ASSERT_GT(tangent.head<2>().norm(),1e-8);
    EXPECT_NEAR(yaws[i],std::atan2(tangent.y(),tangent.x()),1e-12);
  }
}

TEST(WholeSplineCollision, StationaryCurveHasFiniteFallbackAndStillChecksOccupancy) {
  auto map=GridMapTestAccess::make();Eigen::MatrixXd points(3,6);
  for(int i=0;i<points.cols();++i) points.col(i)=Point(.1,.2,.55);
  UniformBspline curve(points,3,1.);
  const auto clear=checkWholeSplineCollision(curve,.05,[&](const Point &p,double yaw) {
    EXPECT_TRUE(std::isfinite(yaw));return map->getInflateOccupancy(p,yaw);
  });
  EXPECT_EQ(clear.state,SplineCollisionState::Clear);EXPECT_EQ(clear.queries,2U);
  map->setOccupied({.1,.2,.55});
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map)).state,SplineCollisionState::Collision);
}
TEST(WholeSplineCollision,HoldHeadingDoesNotInferBodyYawFromLateralTranslation) {
  auto curve=slowCurve({0.,.2,0.});
  auto heading=formalContract(true,.0);heading.policy=scan_planner::BodyHeadingPolicy::HoldHeading;
  ASSERT_TRUE(scan_planner::validMeasuredHeadingContract(heading));
  const auto held=checkWholeSplineCollision(curve,.05,[&](const Point& p,double yaw) {
    EXPECT_DOUBLE_EQ(yaw,0.);
    return p.y()>.1&&yaw>1.3?1:0;
  },50000,heading);
  EXPECT_EQ(held.state,SplineCollisionState::Clear);
  heading.policy=scan_planner::BodyHeadingPolicy::PathTangent;
  const auto tangent=checkWholeSplineCollision(curve,.05,[](const Point& p,double yaw) {
    return p.y()>.1&&yaw>1.3?1:0;
  },50000,heading);
  EXPECT_EQ(tangent.state,SplineCollisionState::Collision);
  // Fixed yaw is not a smaller footprint or obstacle suppression: an obstacle
  // intersecting that very orientation is still rejected by the same checker.
  heading.policy=scan_planner::BodyHeadingPolicy::HoldHeading;
  const auto occupied=checkWholeSplineCollision(curve,.05,[](const Point& p,double yaw) {
    return p.y()>.1&&std::abs(yaw)<.01?1:0;
  },50000,heading);
  EXPECT_EQ(occupied.state,SplineCollisionState::Collision);
}
TEST(WholeSplineCollision,InvalidOrUnmeasuredHeldHeadingCannotBeCertified) {
  auto curve=slowCurve({0.,.2,0.});
  auto heading=formalContract(true);heading.policy=scan_planner::BodyHeadingPolicy::HoldHeading;
  heading.measured_yaw=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(scan_planner::validMeasuredHeadingContract(heading));
  EXPECT_NE(checkWholeSplineCollision(curve,.05,[](const Point&,double){return 0;},50000,heading).state,
      SplineCollisionState::Clear);
  heading.measured_yaw=.2;heading.measured_heading_enabled=false;
  EXPECT_FALSE(scan_planner::validMeasuredHeadingContract(heading));
  EXPECT_NE(checkWholeSplineCollision(curve,.05,[](const Point&,double){return 0;},50000,heading).state,
      SplineCollisionState::Clear);
  EXPECT_THROW(scan_planner::parseBodyHeadingPolicy("lateral_velocity_is_yaw"),std::invalid_argument);
}
TEST(WholeSplineCollision,OptimizerResetsNativeSearchHeadingFromEachFrozenSolveContract) {
  auto map=GridMapTestAccess::make(true);GridMapTestAccess::box(*map);
  map->setOccupied({.025,.325,.55});
  scan_planner::BsplineOptimizer optimizer;optimizer.a_star_=std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map,{64,64,32});
  auto heading=formalContract(true,0.);heading.policy=scan_planner::BodyHeadingPolicy::HoldHeading;
  optimizer.setPreviewHeadingContract(heading);
  EXPECT_EQ(optimizer.a_star_->AstarSearch(.05,{0.,0.,.55},{0.,-.8,.55},false),ASTAR_RET::SUCCESS);
  heading.policy=scan_planner::BodyHeadingPolicy::PathTangent;
  optimizer.setPreviewHeadingContract(heading);
  EXPECT_EQ(optimizer.a_star_->AstarSearch(.05,{0.,0.,.55},{0.,-.8,.55},false),ASTAR_RET::INIT_START_OCCUPIED);
}

TEST(WholeSplineCollision, BudgetExhaustionRejectsBeforeAnyPartialCertificate) {
  auto map=GridMapTestAccess::make();UniformBspline curve(straightPoints(),3,1.);
  std::size_t queries=0;
  const auto result=checkWholeSplineCollision(curve,.05,[&](const Point &p,double yaw) {
    ++queries;return map->getInflateOccupancy(p,yaw);
  },2);
  EXPECT_EQ(result.state,SplineCollisionState::SampleBudgetExceeded);
  EXPECT_EQ(queries,0U);EXPECT_EQ(result.queries,0U);
}

TEST(WholeSplineCollision, EmptyAndInvalidSplineInputsRejectWithoutEvaluation) {
  const auto forbidden_query=[](const Point &,double) {ADD_FAILURE()<<"invalid curve queried map";return 0;};
  UniformBspline empty;
  EXPECT_EQ(checkWholeSplineCollision(empty,.05,forbidden_query).state,SplineCollisionState::InvalidInput);
  auto points=straightPoints();UniformBspline wrong_order(points,2,1.);
  EXPECT_EQ(checkWholeSplineCollision(wrong_order,.05,forbidden_query).state,SplineCollisionState::InvalidInput);
  UniformBspline zero_interval(points,3,0.);
  EXPECT_EQ(checkWholeSplineCollision(zero_interval,.05,forbidden_query).state,SplineCollisionState::InvalidInput);
  points(0,4)=std::numeric_limits<double>::quiet_NaN();UniformBspline invalid_points(points,3,1.);
  EXPECT_EQ(checkWholeSplineCollision(invalid_points,.05,forbidden_query).state,SplineCollisionState::InvalidInput);
  UniformBspline bad_knots(straightPoints(),3,1.);auto knots=bad_knots.getKnot();knots[4]=knots[3];bad_knots.setKnot(knots);
  EXPECT_EQ(checkWholeSplineCollision(bad_knots,.05,forbidden_query).state,SplineCollisionState::InvalidInput);
  UniformBspline valid(straightPoints(),3,1.);
  for(double resolution:{0.,-1.,std::numeric_limits<double>::quiet_NaN()})
    EXPECT_EQ(checkWholeSplineCollision(valid,resolution,forbidden_query).state,SplineCollisionState::InvalidInput);
  EXPECT_EQ(checkWholeSplineCollision(valid,.05,forbidden_query,0).state,SplineCollisionState::InvalidInput);
}

TEST(PreviewSplineHeading, StationaryNoiseDoesNotChangeBodyHeadingOrActualDerivatives) {
  auto map=GridMapTestAccess::make();map->setOccupied({.025,.425,.225});
  ASSERT_EQ(map->getInflateOccupancy({.025,.025,.55},0.),0);
  ASSERT_EQ(map->getInflateOccupancy({.025,.025,.55},std::acos(-1.)/2.),1);
  for(const Point velocity : std::vector<Point>{{.009,0.,0.},{-.009,0.,0.},
      {0.,.009,0.},{0.,-.009,0.},{.005,-.005,.001},{0.,.019,0.}}) {
    auto curve=slowCurve(velocity);const auto original=curve.getControlPoint();
    const auto result=checkWholeSplineCollision(curve,.05,[&](const Point &p,double yaw) {
      EXPECT_NEAR(yaw,0.,1e-12);return map->getInflateOccupancy(p,yaw);
    },50000,previewContract());
    EXPECT_EQ(result.state,SplineCollisionState::Clear)<<velocity.transpose();
    EXPECT_TRUE(curve.getControlPoint().isApprox(original,0.));
    EXPECT_TRUE(curve.getDerivative().evaluateDeBoorT(0.).isApprox(velocity,1e-10));
  }
  auto noisy=slowCurve({0.,.009,0.});
  // Default/disabled behavior is deliberately unchanged, not silently opted in.
  EXPECT_EQ(checkWholeSplineCollision(noisy,.05,query(*map)).state,SplineCollisionState::Collision);
}

TEST(PreviewSplineHeading, FormalBtPreviewUsesWriterContractInsteadOfLegacyMapLabel) {
  using scan_planner::noMotionMeasuredHeadingContract;
  EXPECT_TRUE(noMotionMeasuredHeadingContract(false,false,true,false,true,false));
  // Strict observed-space planning still checks the same unknown evidence;
  // using measured orientation does not require downgrading its map policy.
  EXPECT_TRUE(noMotionMeasuredHeadingContract(false,false,true,false,true,true));
  EXPECT_FALSE(noMotionMeasuredHeadingContract(false,false,true,true,true,false));
  EXPECT_FALSE(noMotionMeasuredHeadingContract(false,false,false,true,true,false));
  EXPECT_TRUE(noMotionMeasuredHeadingContract(true,true,false,false,true,false));
  EXPECT_TRUE(noMotionMeasuredHeadingContract(true,true,false,true,true,false));
  EXPECT_THROW(noMotionMeasuredHeadingContract(true,true,true,true,true,false),std::invalid_argument);
  EXPECT_THROW(noMotionMeasuredHeadingContract(false,false,true,false,false,false),std::invalid_argument);
  EXPECT_THROW(noMotionMeasuredHeadingContract(true,false,false,false,true,false),std::invalid_argument);
}

TEST(PreviewSplineHeading, RecordedMillimeterVelocityCannotRotateMeasuredStandingFootprint) {
  auto map=GridMapTestAccess::make();
  // Recorded 2026-10-09 standing input: forward axis near +Y, but the 5/-15
  // mm/s derivative has an unreliable course near -Y. The native dual circle
  // footprint is yaw dependent even though its front/rear centers are symmetric.
  const Point velocity(.005,-.015,0.);const double measured_yaw=1.567123;
  auto curve=slowCurve(velocity);
  const double noise_yaw=std::atan2(velocity.y(),velocity.x());
  const Point obstruction(.325,-.225,.225);
  map->setOccupied(obstruction);
  ASSERT_EQ(map->getInflateOccupancy(curve.evaluateDeBoorT(0.),measured_yaw),0);
  ASSERT_EQ(map->getInflateOccupancy(curve.evaluateDeBoorT(0.),noise_yaw),1);
  const auto original=curve.getControlPoint();
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map)).state,SplineCollisionState::Collision);
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,
      previewContract(measured_yaw)).state,SplineCollisionState::Clear);
  EXPECT_TRUE(curve.getControlPoint().isApprox(original,0.));
  EXPECT_TRUE(curve.getDerivative().evaluateDeBoorT(0.).isApprox(velocity,1e-10));
  // The fix changes the queried orientation, never the map or the velocity.
  EXPECT_EQ(map->getOccupancy(obstruction),1);
  map->setOccupied(curve.evaluateDeBoorT(0.));
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,
      previewContract(measured_yaw)).state,SplineCollisionState::Collision);
}

TEST(PreviewSplineHeading, FormalStrictPlanningDoesNotOpenUnknownSpace) {
  auto map=GridMapTestAccess::make(true);auto curve=slowCurve({.005,-.015,0.});
  auto contract=previewContract(1.567123);
  contract.preview_only_enabled=scan_planner::noMotionMeasuredHeadingContract(
      false,false,true,false,true,true);
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,contract).state,
      SplineCollisionState::Clear);
  GridMapTestAccess::unknown(*map,curve.evaluateDeBoorT(0.));
  const auto denied=checkWholeSplineCollision(curve,.05,query(*map),50000,contract);
  EXPECT_EQ(denied.state,SplineCollisionState::Collision);
  EXPECT_EQ(denied.occupancy,2);
}

TEST(PreviewSplineHeading, ActualMeasuredFootprintStillRejectsAtZeroSpeed) {
  auto map=GridMapTestAccess::make();map->setOccupied({.025,.025,.225});
  auto curve=slowCurve(Point::Zero());
  const auto result=checkWholeSplineCollision(curve,.05,query(*map),50000,previewContract());
  EXPECT_EQ(result.state,SplineCollisionState::Collision);
  EXPECT_EQ(result.queries,1U);EXPECT_DOUBLE_EQ(result.yaw,0.);EXPECT_DOUBLE_EQ(result.time,0.);
}

TEST(PreviewSplineHeading, AboveThresholdRequiresMeasuredToCourseRotationSweep) {
  auto map=GridMapTestAccess::make();map->setOccupied({.325,.325,.225});
  const Point origin(.025,.025,.55);const double pi=std::acos(-1.);
  ASSERT_EQ(map->getInflateOccupancy(origin,0.),0);
  ASSERT_EQ(map->getInflateOccupancy(origin,pi/2.),0);
  ASSERT_EQ(map->getInflateOccupancy(origin,pi/4.),1);
  auto low=slowCurve({0.,.019,0.});
  EXPECT_EQ(checkWholeSplineCollision(low,.05,query(*map),50000,previewContract()).state,
            SplineCollisionState::Clear);
  auto high=slowCurve({0.,.021,0.});std::vector<double> checked;
  const auto result=checkWholeSplineCollision(high,.05,[&](const Point &p,double yaw) {
    checked.push_back(yaw);return map->getInflateOccupancy(p,yaw);
  },50000,previewContract());
  ASSERT_FALSE(checked.empty());EXPECT_DOUBLE_EQ(checked.front(),0.);
  EXPECT_EQ(result.state,SplineCollisionState::Collision);
  EXPECT_GT(result.yaw,0.);EXPECT_LT(result.yaw,pi/2.);EXPECT_DOUBLE_EQ(result.time,0.);
}

TEST(PreviewSplineHeading, LaterDirectionConfidenceCrossingAlsoChecksRotation) {
  auto map=GridMapTestAccess::make();map->setOccupied({.325,.325,.225});
  auto base=slowCurve({0.,.009,0.});auto points=base.getControlPoint();
  // Preserve p/v/a at t=0; a later control point raises direction confidence.
  for(int i=3;i<points.cols();++i) points(1,i)+=.02*(i-2);
  UniformBspline curve(points,3,.25);
  ASSERT_NEAR(curve.getDerivative().evaluateDeBoorT(0.).y(),.009,1e-10);
  const auto result=checkWholeSplineCollision(curve,.05,query(*map),50000,previewContract());
  EXPECT_EQ(result.state,SplineCollisionState::Collision);EXPECT_GT(result.time,0.);
}

TEST(PreviewSplineHeading, ClearCourseAboveThresholdAndShortArcRemainBounded) {
  auto map=GridMapTestAccess::make();auto curve=slowCurve({0.,.021,0.});
  const double pi=std::acos(-1.);bool saw_course=false;std::size_t queries=0;
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,[&](const Point &p,double yaw) {
    if(queries++==0) EXPECT_DOUBLE_EQ(yaw,0.);
    if(std::abs(yaw-pi/2.)<1e-10) saw_course=true;
    return map->getInflateOccupancy(p,yaw);
  },50000,previewContract()).state,SplineCollisionState::Clear);
  EXPECT_TRUE(saw_course);
  std::size_t arc_queries=0;
  EXPECT_TRUE(scan_planner::headingTransitionFree(Point(.025,.025,.55),Point(.025,.025,.55),
      pi-.01,-pi+.01,.05,.49,[&](const Point &p,double yaw) {
        ++arc_queries;EXPECT_LT(std::abs(yaw-pi),.011);
        return map->getInflateOccupancy(p,yaw)==0;
      }));
  EXPECT_LE(arc_queries,6U);
}

TEST(PreviewSplineHeading, RearBodyEntryCoversOppositeRetainedTurnWithRealRectangularVoxelQuery) {
  const double pi=std::acos(-1.),extent=std::hypot(1.03,.40)*.5;
  const Point body(0.,0.,.55);const Eigen::Vector3i obstacle(-8,-7,11);
  const auto free=[&](const Point& p,double yaw) {
    return !scan_planner::orientedBoxIntersectsVoxelXY(p,yaw,obstacle,.05,1.03,.40);
  };
  // The -100 deg shortest arc is clear, but a previously selected positive
  // turn to the SAME orientation passes through this real occupied cell.
  ASSERT_TRUE(scan_planner::headingTransitionFree(body,body,0.,-100.*pi/180.,.05,extent,free));
  EXPECT_FALSE(scan_planner::bodyEntryHeadingTransitionFree(body,body,0.,-100.*pi/180.,.05,extent,free));
  EXPECT_FALSE(scan_planner::headingSweepFree(body,body,0.,260.*pi/180.,.05,extent,free));
}

TEST(PreviewSplineHeading, RearBodyEntryCoversBothPiBranchesButForwardKeepsShortestSweep) {
  const double pi=std::acos(-1.);const Point body(0.,0.,.55);
  for(double degrees:{-180.,-179.,179.,180.}) {
    double highest=0.;std::size_t count=0;bool exact_target=false;
    EXPECT_TRUE(scan_planner::bodyEntryHeadingTransitionFree(body,body,0.,degrees*pi/180.,.05,.55,
      [&](const Point&,double yaw){highest=std::max(highest,yaw);exact_target|=yaw==degrees*pi/180.;++count;return true;}));
    EXPECT_NEAR(highest,2.*pi,1e-12);EXPECT_GT(count,500U);EXPECT_LT(count,600U);
    EXPECT_TRUE(exact_target);
  }
  for(double degrees:{-90.,-30.,0.,30.,90.}) {
    std::vector<double> entry,ordinary;
    EXPECT_TRUE(scan_planner::bodyEntryHeadingTransitionFree(body,body,0.,degrees*pi/180.,.05,.55,
      [&](const Point&,double yaw){entry.push_back(yaw);return true;}));
    EXPECT_TRUE(scan_planner::headingTransitionFree(body,body,0.,degrees*pi/180.,.05,.55,
      [&](const Point&,double yaw){ordinary.push_back(yaw);return true;}));
    EXPECT_EQ(entry,ordinary);
  }
  EXPECT_FALSE(scan_planner::rearBodyHeadingTransition(0.,pi/2.));
  EXPECT_TRUE(scan_planner::rearBodyHeadingTransition(0.,pi/2.+1e-6));
}

TEST(PreviewSplineHeading, RearBodyEntryCannotCertifyAnIncompleteOrInvalidSweep) {
  const Point body(0.,0.,.55);const double pi=std::acos(-1.);std::size_t count=0;
  EXPECT_FALSE(scan_planner::bodyEntryHeadingTransitionFree(body,body,0.,pi,.05,.55,
    [&](const Point&,double){return ++count<=50;}));
  EXPECT_EQ(count,51U);
  EXPECT_FALSE(scan_planner::bodyEntryHeadingTransitionFree(body,body,0.,pi,.05,.55,
    [](const Point&,double){return false;})); // Same caller deadline/cancel rejection.
  EXPECT_FALSE(scan_planner::bodyEntryHeadingTransitionFree(body,body,NAN,pi,.05,.55,
    [](const Point&,double){return true;}));
}

TEST(PreviewSplineHeading, InvalidContractAndExhaustedSweepNeverReturnClear) {
  auto map=GridMapTestAccess::make();auto curve=slowCurve({0.,.021,0.});
  auto invalid=previewContract();invalid.measured_yaw=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,invalid).state,
            SplineCollisionState::InvalidInput);
  const auto result=checkWholeSplineCollision(curve,.05,query(*map),60,previewContract());
  EXPECT_EQ(result.state,SplineCollisionState::SampleBudgetExceeded);
  EXPECT_LE(result.queries,60U);
}

TEST(MeasuredSplineHeading, MillimetersPerSecondEntryJitterDoesNotInventOppositeRotation) {
  // Same immutable cubic and its real derivative, not a substituted heading
  // trace. This entry initially drifts backwards 1.225 mm before +X motion.
  Eigen::MatrixXd points(3,9);
  for(int i=0;i<9;++i) {
    const double t=i-1.;points.col(i)=Point(-.007*t+.01*t*t,0.,.55);
  }
  UniformBspline curve(points,3,1.);auto heading=formalContract(true);
  heading.align_then_forward=true;
  const auto yaw_sensitive=[](const Point&,double yaw){return std::abs(yaw)>.2?1:0;};
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,yaw_sensitive,50000,heading).state,
    SplineCollisionState::Clear);
  // Choosing a reliable near direction does NOT skip the actual-body yaw or
  // its swept connector. An occupied measured orientation still rejects.
  heading.measured_yaw=.5;
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,yaw_sensitive,50000,heading).state,
    SplineCollisionState::Collision);
}

TEST(MeasuredSplineHeading, FormalGeometryDoesNotDependOnWriterAuthorization) {
  using scan_planner::measuredHeadingGeometryContract;
  for(bool writer:{false,true}) {
    EXPECT_TRUE(measuredHeadingGeometryContract(false,false,true,writer,true,false));
    EXPECT_TRUE(measuredHeadingGeometryContract(false,false,true,writer,true,true));
    EXPECT_THROW(measuredHeadingGeometryContract(false,false,true,writer,false,false),
                 std::invalid_argument);
  }
  EXPECT_FALSE(measuredHeadingGeometryContract(false,false,false,true,true,false));
  EXPECT_TRUE(measuredHeadingGeometryContract(true,true,false,false,true,false));
  EXPECT_THROW(measuredHeadingGeometryContract(true,false,false,true,true,false),
               std::invalid_argument);
}

TEST(MeasuredSplineHeading, WriterAndPreviewPreserveActualLowSpeedBoundaryAndMap) {
  auto map=GridMapTestAccess::make();
  const Point obstruction(.325,-.225,.225);
  map->setOccupied(obstruction);
  const Point velocity(.005,-.015,0.);const double measured_yaw=1.567123;
  auto curve=slowCurve(velocity);const auto controls=curve.getControlPoint();
  ASSERT_EQ(map->getInflateOccupancy(curve.evaluateDeBoorT(0.),measured_yaw),0);
  ASSERT_EQ(map->getInflateOccupancy(curve.evaluateDeBoorT(0.),
      std::atan2(velocity.y(),velocity.x())),1);
  for(bool writer:{false,true}) {
    auto contract=formalContract(writer,measured_yaw);
    ASSERT_FALSE(contract.preview_only_enabled);
    EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,contract).state,
              SplineCollisionState::Clear);
    EXPECT_TRUE(curve.getControlPoint().isApprox(controls,0.));
    EXPECT_TRUE(curve.getDerivative().evaluateDeBoorT(0.).isApprox(velocity,1e-10));
    EXPECT_EQ(map->getOccupancy(obstruction),1);
  }
  map->setOccupied(curve.evaluateDeBoorT(0.));
  for(bool writer:{false,true})
    EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,
        formalContract(writer,measured_yaw)).state,SplineCollisionState::Collision);
}

TEST(MeasuredSplineHeading, WriterAndPreviewCheckWholeInitialRotationNotJustEndpoints) {
  auto map=GridMapTestAccess::make();map->setOccupied({.325,.325,.225});
  const Point origin(.025,.025,.55);const double pi=std::acos(-1.);
  ASSERT_EQ(map->getInflateOccupancy(origin,0.),0);
  ASSERT_EQ(map->getInflateOccupancy(origin,pi/2.),0);
  ASSERT_EQ(map->getInflateOccupancy(origin,pi/4.),1);
  auto curve=slowCurve({0.,.021,0.});
  for(bool writer:{false,true}) {
    auto result=checkWholeSplineCollision(curve,.05,query(*map),50000,formalContract(writer));
    EXPECT_EQ(result.state,SplineCollisionState::Collision);
    EXPECT_DOUBLE_EQ(result.time,0.);
    EXPECT_GT(result.yaw,0.);EXPECT_LT(result.yaw,pi/2.);
    EXPECT_EQ(map->getOccupancy(Point(.325,.325,.225)),1);
  }
}

TEST(MeasuredSplineHeading, WriterEnabledGeometryDoesNotChangeUnknownEvidence) {
  auto map=GridMapTestAccess::make(true);auto curve=slowCurve({.005,-.015,0.});
  auto contract=formalContract(true,1.567123);
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,contract).state,
            SplineCollisionState::Clear);
  GridMapTestAccess::unknown(*map,curve.evaluateDeBoorT(0.));
  const auto result=checkWholeSplineCollision(curve,.05,query(*map),50000,contract);
  EXPECT_EQ(result.state,SplineCollisionState::Collision);EXPECT_EQ(result.occupancy,2);
}

TEST(MeasuredSplineHeading, InvalidMeasuredContractCannotProducePartialCertificate) {
  auto map=GridMapTestAccess::make();auto curve=slowCurve({0.,.021,0.});
  auto invalid=formalContract(true);
  invalid.measured_yaw=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,invalid).state,
            SplineCollisionState::InvalidInput);
  const auto result=checkWholeSplineCollision(curve,.05,query(*map),60,formalContract(true));
  EXPECT_EQ(result.state,SplineCollisionState::SampleBudgetExceeded);
  EXPECT_LE(result.queries,60U);
}
