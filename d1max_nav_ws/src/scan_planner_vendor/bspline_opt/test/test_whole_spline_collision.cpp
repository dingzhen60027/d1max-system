#include <gtest/gtest.h>
#include <bspline_opt/whole_spline_collision.hpp>
#include <plan_env/grid_map.h>
#include <limits>

using scan_planner::UniformBspline;
using scan_planner::SplineCollisionState;
using scan_planner::checkWholeSplineCollision;
using Point=Eigen::Vector3d;

// Production GridMap/inflation and UniformBspline, without ROS initialization.
struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={160,80,64};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.require_observed_free_=false;p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;
    const std::size_t size=160U*80U*64U;
    d.occupancy_buffer_.assign(size,-1.01);d.occupancy_buffer_inflate_.assign(size,0);
    d.occupancy_buffer_inflate_cnt_.assign(size,0);map->rebuildInflationOffsets();return map;
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

TEST(PreviewSplineHeading, InvalidContractAndExhaustedSweepNeverReturnClear) {
  auto map=GridMapTestAccess::make();auto curve=slowCurve({0.,.021,0.});
  auto invalid=previewContract();invalid.measured_yaw=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,query(*map),50000,invalid).state,
            SplineCollisionState::InvalidInput);
  const auto result=checkWholeSplineCollision(curve,.05,query(*map),60,previewContract());
  EXPECT_EQ(result.state,SplineCollisionState::SampleBudgetExceeded);
  EXPECT_LE(result.queries,60U);
}
