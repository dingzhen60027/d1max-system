// Synthetic inputs to the actual production GridMap + B-spline validator.
// No ROS init, publisher, SDK or map-clearing production path is used.
#include <gtest/gtest.h>
#include <plan_env/grid_map.h>
#include <plan_manage/trajectory_collision.hpp>

struct GridMapTestAccess {
  static GridMap::Ptr observedFree() {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.08;p.resolution_inv_=12.5;p.map_voxel_num_={64,64,32};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=.45;p.obstacles_inflation_z_down=.45;
    d.occupancy_buffer_.assign(64*64*32,-1.);
    d.occupancy_buffer_inflate_.assign(64*64*32,0);
    d.occupancy_buffer_inflate_cnt_.assign(64*64*32,0);
    map->rebuildInflationOffsets();return map;
  }
  static void corner(GridMap &map,int state) {
    Eigen::Vector3i cell;map.posToIndex(Eigen::Vector3d(.35,.35,.5),cell);
    map.md_.occupancy_buffer_[map.toAddress(cell)]=state==1 ? 2. : -1.01;
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
  static void unknown(GridMap &map,const Eigen::Vector3d &point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);
    map.md_.occupancy_buffer_[map.toAddress(cell)]=-1.01;
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
  static void standingBox(GridMap &map) {
    auto &p=map.mp_;
    p.collision_model_="oriented_box";p.box_length_=.93;p.box_width_=.30;
    p.body_below_=p.body_above_=.25;
    p.planning_body_below_=p.planning_body_above_=.25;
    map.rebuildInflationOffsets();
  }
  static GridMap::Ptr officialPreview() {
    auto map=observedFree();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;map->updateMapBoundaryFromIndex();
    p.require_observed_free_=false;
    d.occupancy_buffer_.assign(64*64*32,-1.01);
    map->rebuildInflationOffsets();return map;
  }
};

static bool validate(GridMap &map) {
  Eigen::MatrixXd points(3,23);
  for(int i=0;i<23;++i)points.col(i)=Eigen::Vector3d((i-1)*.05,0.,.55);
  scan_planner::UniformBspline curve(points,3,.2);
  auto query=[&](const Eigen::Vector3d &p,double yaw){return map.getInflateOccupancy(p,yaw);};
#ifdef FROZEN_BEFORE_INITIAL_YAW
  return scan_planner::wholeCurveCollisionFree(curve,.08,.49,query);
#else
  scan_planner::MeasuredBodyPose measured{Eigen::Vector3d(0.,0.,.55),
    Eigen::Quaterniond(Eigen::AngleAxisd(std::acos(-1.)/2.,Eigen::Vector3d::UnitZ())),10.,"map"};
  return scan_planner::wholeCurveCollisionFree(curve,.08,
      scan_planner::bodyExtent(map.bodyEnvelope()),query,measured,"map",10.,.5);
#endif
}

TEST(InitialYawNative, FullyObservedTurnWithoutObstaclesRemainsAdmissible) {
  auto map=GridMapTestAccess::observedFree();const bool accepted=validate(*map);
  RecordProperty("accepted",accepted ? "true":"false");EXPECT_TRUE(accepted);
}

TEST(InitialYawNative, StandingBoxFinalCurveChecksIntermediateTurnWithUnifiedExtent) {
  auto map=GridMapTestAccess::observedFree();GridMapTestAccess::standingBox(*map);
  const auto strict=map->bodyEnvelope(),planning=map->planningBodyEnvelope();
  ASSERT_EQ(strict.model,scan_planner::CollisionModel::OrientedBox);
  EXPECT_DOUBLE_EQ(strict.length,.93);EXPECT_DOUBLE_EQ(strict.width,.30);
  EXPECT_DOUBLE_EQ(strict.below,.25);EXPECT_DOUBLE_EQ(strict.above,.25);
  EXPECT_DOUBLE_EQ(planning.length,strict.length);EXPECT_DOUBLE_EQ(planning.width,strict.width);
  EXPECT_DOUBLE_EQ(planning.below,strict.below);EXPECT_DOUBLE_EQ(planning.above,strict.above);
  EXPECT_NEAR(scan_planner::bodyExtent(strict),std::hypot(.465,.15),1e-12);
  const auto optimizer_check=[&]() {
    Eigen::MatrixXd points(3,23);
    for(int i=0;i<23;++i)points.col(i)=Eigen::Vector3d((i-1)*.05,0.,.55);
    scan_planner::UniformBspline curve(points,3,.2);
    scan_planner::SplineHeadingContract heading;
    heading.measured_heading_enabled=true;heading.measured_yaw=std::acos(-1.)*.5;
    heading.body_extent=scan_planner::bodyExtent(planning);
    return scan_planner::checkWholeSplineCollision(curve,map->getResolution(),
      [&](const Eigen::Vector3d &p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},
      50000,heading).state;
  };
  ASSERT_EQ(optimizer_check(),scan_planner::SplineCollisionState::Clear);
  ASSERT_TRUE(validate(*map));
  map->setOccupied({.28,.28,.55});
  const Eigen::Vector3d body(0.,0.,.55);
  const double pi=std::acos(-1.);
  ASSERT_EQ(map->getPlanningInflateOccupancy(body,0.),0);
  ASSERT_EQ(map->getPlanningInflateOccupancy(body,pi*.5),0);
  ASSERT_EQ(map->getPlanningInflateOccupancy(body,pi*.25),1);
  EXPECT_EQ(optimizer_check(),scan_planner::SplineCollisionState::Collision);
  EXPECT_FALSE(validate(*map));
}

TEST(InitialYawNative, FreeEndpointsCannotHideOccupiedIntermediateFootprint) {
  auto map=GridMapTestAccess::observedFree();GridMapTestAccess::corner(*map,1);
  ASSERT_EQ(map->getInflateOccupancy({0.,0.,.55},0.),0);
  ASSERT_EQ(map->getInflateOccupancy({0.,0.,.55},std::acos(-1.)/2.),0);
  ASSERT_NE(map->getInflateOccupancy({0.,0.,.55},std::acos(-1.)/4.),0);
  const bool accepted=validate(*map);RecordProperty("accepted",accepted ? "true":"false");
#ifdef FROZEN_BEFORE_INITIAL_YAW
  EXPECT_TRUE(accepted); // Confirm the original unsafe acceptance, not a corrected result.
#else
  EXPECT_FALSE(accepted);
#endif
}

TEST(InitialYawNative, UnobservedIntermediateFootprintMustNotBeAcceptedEither) {
  auto map=GridMapTestAccess::observedFree();GridMapTestAccess::corner(*map,2);
  ASSERT_EQ(map->getInflateOccupancy({0.,0.,.55},0.),0);
  ASSERT_EQ(map->getInflateOccupancy({0.,0.,.55},std::acos(-1.)/2.),0);
  ASSERT_NE(map->getInflateOccupancy({0.,0.,.55},std::acos(-1.)/4.),0);
  const bool accepted=validate(*map);RecordProperty("accepted",accepted ? "true":"false");
#ifdef FROZEN_BEFORE_INITIAL_YAW
  EXPECT_TRUE(accepted);
#else
  EXPECT_FALSE(accepted);
#endif
}

namespace {
scan_planner::UniformBspline previewCurve(double vy) {
  Eigen::MatrixXd points(3,7);
  for(int i=0;i<points.cols();++i)
    points.col(i)=Eigen::Vector3d(.025,.025+vy*((i-1)*.25),.55);
  return scan_planner::UniformBspline(points,3,.25);
}
scan_planner::SplineHeadingContract previewHeading() {
  scan_planner::SplineHeadingContract contract;contract.preview_only_enabled=true;return contract;
}
scan_planner::MeasuredBodyPose previewBody() {
  return {{.025,.025,.55},Eigen::Quaterniond::Identity(),10.,"map"};
}
bool validatePreview(GridMap &map,scan_planner::UniformBspline &curve,
    const scan_planner::MeasuredBodyPose &body,std::size_t budget=50000) {
  return scan_planner::wholeCurveCollisionFree(curve,.05,.49,
      [&](const Eigen::Vector3d &p,double yaw){return map.getInflateOccupancy(p,yaw);},
      body,"map",10.,.5,budget,0.,0.,previewHeading());
}
} // namespace

TEST(InitialYawNative, PreviewInternalFinalAndRepeatedFullChecksShareOneContract) {
  auto map=GridMapTestAccess::officialPreview();map->setOccupied({.025,.425,.225});
  for(double velocity:{-.019,-.009,0.,.009,.019}) {
    auto curve=previewCurve(velocity);
    const auto inner=scan_planner::checkWholeSplineCollision(curve,.05,
        [&](const Eigen::Vector3d &p,double yaw){return map->getInflateOccupancy(p,yaw);},
        50000,previewHeading());
    ASSERT_EQ(inner.state,scan_planner::SplineCollisionState::Clear);
    EXPECT_TRUE(validatePreview(*map,curve,previewBody()));
    // Continuous no-motion rechecks use the same complete curve and real pose.
    EXPECT_TRUE(validatePreview(*map,curve,previewBody()));
  }
}

TEST(InitialYawNative, PreviewThresholdCannotSkipActualRotatingBodyCollision) {
  auto map=GridMapTestAccess::officialPreview();map->setOccupied({.325,.325,.225});
  auto slow=previewCurve(.019),moving=previewCurve(.021);
  EXPECT_TRUE(validatePreview(*map,slow,previewBody()));
  EXPECT_FALSE(validatePreview(*map,moving,previewBody()));
  // A new measured yaw is checked, never replaced with a remembered free yaw.
  auto turned=previewBody();turned.orientation=Eigen::AngleAxisd(std::acos(-1.)/4.,Eigen::Vector3d::UnitZ());
  EXPECT_FALSE(validatePreview(*map,slow,turned));
}

TEST(InitialYawNative, PreviewSourceLeaseJoinDistanceAndSweepBudgetRemainMandatory) {
  auto map=GridMapTestAccess::officialPreview();auto curve=previewCurve(.021);
  for(int failure=0;failure<4;++failure) {
    auto body=previewBody();
    if(failure==0) body.source_stamp=9.;
    if(failure==1) body.frame="wrong";
    if(failure==2) body.orientation.coeffs().setZero();
    if(failure==3) body.position.x()+=.02;
    EXPECT_FALSE(validatePreview(*map,curve,body))<<failure;
  }
  EXPECT_FALSE(validatePreview(*map,curve,previewBody(),60));
  EXPECT_TRUE(validatePreview(*map,curve,previewBody()));
  map->setOccupied({.025,.025,.225});
  EXPECT_FALSE(validatePreview(*map,curve,previewBody()));
}

TEST(InitialYawNative, IncompleteProofIsNotAClaimOfOccupiedGeometry) {
  auto map=GridMapTestAccess::officialPreview();auto curve=previewCurve(.021);
  const auto inspect=[&](const scan_planner::MeasuredBodyPose &body,std::size_t budget) {
    bool occupied_witness=false;
    const bool clear=scan_planner::wholeCurveCollisionFree(curve,.05,.49,
        [&](const Eigen::Vector3d &p,double yaw) {
          const int value=map->getInflateOccupancy(p,yaw);
          occupied_witness=occupied_witness || value==1;
          return value;
        },body,"map",10.,.5,budget,0.,0.,previewHeading());
    return scan_planner::curveCheckEvidence(clear,occupied_witness);
  };
  using Evidence=scan_planner::CurveCheckEvidence;
  EXPECT_EQ(inspect(previewBody(),50000),Evidence::Clear);
  EXPECT_EQ(inspect(previewBody(),1),Evidence::Uncertified);
  auto moved=previewBody();moved.position.x()+=.02;
  EXPECT_EQ(inspect(moved,50000),Evidence::Uncertified);
  auto stale=previewBody();stale.source_stamp=9.;
  EXPECT_EQ(inspect(stale,50000),Evidence::Uncertified);
  // No input weakening: retry with sufficient budget/fresh real pose must
  // actually complete the production GridMap whole-curve check.
  EXPECT_EQ(inspect(previewBody(),50000),Evidence::Clear);
  map->setOccupied({.025,.025,.225});
  EXPECT_EQ(inspect(previewBody(),50000),Evidence::Occupied);
  EXPECT_EQ(inspect(previewBody(),1),Evidence::Uncertified);
}

TEST(InitialYawNative, UnknownEvidenceIsNotAnOccupiedWitness) {
  auto map=GridMapTestAccess::observedFree();auto curve=previewCurve(0.);
  GridMapTestAccess::unknown(*map,{.025,.025,.55});
  ASSERT_EQ(map->getInflateOccupancy({.025,.025,.55},0.),2);
  const auto inspect=[&]() {
    bool occupied_witness=false;
    const bool clear=scan_planner::wholeCurveCollisionFree(curve,.05,.49,
        [&](const Eigen::Vector3d &p,double yaw) {
          const int value=map->getInflateOccupancy(p,yaw);
          occupied_witness=occupied_witness || value==1;return value;
        },previewBody(),"map",10.,.5,50000,0.,0.,previewHeading());
    return scan_planner::curveCheckEvidence(clear,occupied_witness);
  };
  EXPECT_EQ(inspect(),scan_planner::CurveCheckEvidence::Uncertified);
  EXPECT_TRUE(map->isUnknown(Eigen::Vector3d(.025,.025,.55)));
}

TEST(InitialYawNative,AlignPolicyChecksZeroDerivativeEntryTurnAtActualBodyBeforeTranslation) {
  auto map=GridMapTestAccess::observedFree();GridMapTestAccess::standingBox(*map);
  Eigen::MatrixXd points(3,23);
  for(int i=0;i<23;++i){const double t=(i-1)*.2;
    points.col(i)=Eigen::Vector3d(0.,.05*(t*t-.04/3.),.55);}
  scan_planner::UniformBspline curve(points,3,.2);
  ASSERT_LT(curve.getDerivative().evaluateDeBoorT(0.).head<2>().norm(),1e-12);
  const auto origin=curve.evaluateDeBoorT(0.);
  scan_planner::MeasuredBodyPose body{origin,Eigen::Quaterniond::Identity(),10.,"map"};
  scan_planner::SplineHeadingContract heading;heading.measured_heading_enabled=true;
  heading.body_extent=scan_planner::bodyExtent(map->planningBodyEnvelope());
  heading.align_then_forward=true;
  const auto free_query=[&](const Eigen::Vector3d&p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
  ASSERT_EQ(scan_planner::checkWholeSplineCollision(curve,.05,free_query,50000,heading).state,
    scan_planner::SplineCollisionState::Clear);
  ASSERT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,heading.body_extent,free_query,
    body,"map",10.,.4,50000,.050,0.,heading));
  map->setOccupied({.28,.28,.55});
  ASSERT_EQ(map->getPlanningInflateOccupancy(origin,0.),0);
  ASSERT_EQ(map->getPlanningInflateOccupancy(origin,std::acos(-1.)*.5),0);
  ASSERT_EQ(map->getPlanningInflateOccupancy(origin,std::acos(-1.)*.25),1);
  unsigned actual_entry_corner_queries=0;
  const auto corner=[&](const Eigen::Vector3d&p,double yaw){
    const int state=map->getPlanningInflateOccupancy(p,yaw);
    if((p-origin).norm()<1e-10&&state==1)++actual_entry_corner_queries;
    return state;
  };
  const auto inner=scan_planner::checkWholeSplineCollision(curve,.05,corner,50000,heading);
  EXPECT_EQ(inner.state,scan_planner::SplineCollisionState::Collision);
  EXPECT_DOUBLE_EQ(inner.time,0.);EXPECT_GT(actual_entry_corner_queries,0U);
  actual_entry_corner_queries=0;
  scan_planner::CurveCheckCompletion completion;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,heading.body_extent,corner,
    body,"map",10.,.4,50000,.050,0.,heading,0.,scan_planner::MeasuredConnectionPolicy::CandidateAdmission,
    .15,&completion));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::Blocked);
  EXPECT_GT(actual_entry_corner_queries,0U);
}
TEST(InitialYawNative,ExplicitAlignStrategyDoesNotRotateHoldHeadingOrLegacyLowSpeedPreview) {
  auto curve=previewCurve(.009);
  auto heading=previewHeading();heading.align_then_forward=true;
  heading.policy=scan_planner::BodyHeadingPolicy::HoldHeading;
  const auto held=scan_planner::checkWholeSplineCollision(curve,.05,
    [](const Eigen::Vector3d&,double yaw){EXPECT_NEAR(yaw,0.,1e-12);return 0;},50000,heading);
  EXPECT_EQ(held.state,scan_planner::SplineCollisionState::Clear);
  heading.policy=scan_planner::BodyHeadingPolicy::PathTangent;heading.align_then_forward=false;
  const auto legacy=scan_planner::checkWholeSplineCollision(curve,.05,
    [](const Eigen::Vector3d&,double yaw){EXPECT_NEAR(yaw,0.,1e-12);return 0;},50000,heading);
  EXPECT_EQ(legacy.state,scan_planner::SplineCollisionState::Clear);
}
