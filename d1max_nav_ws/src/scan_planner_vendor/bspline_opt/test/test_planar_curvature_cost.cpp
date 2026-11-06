#include <gtest/gtest.h>
#include <bspline_opt/planar_curvature_cost.hpp>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/whole_spline_collision.hpp>
#include <chrono>
#include <iostream>

namespace {
Eigen::MatrixXd bend(double sign=1.) {
  Eigen::MatrixXd q=Eigen::MatrixXd::Zero(3,14);
  for(int i=0;i<q.cols();++i)q.col(i)=Eigen::Vector3d(.5+.2*i,1.+sign*.18*std::sin(.8*i),.55+.01*i);
  return q;
}
struct GeometryMetrics {
  double peak{0.},arc_p95{0.},interior_peak{0.},endpoint_peak{0.};
  double turn{0.},energy{0.},length{0.};
};
GeometryMetrics geometryMetrics(const Eigen::MatrixXd& q,double weight) {
  scan_planner::UniformBspline curve(q,3,1.);auto v=curve.getDerivative();auto a=v.getDerivative();
  GeometryMetrics result;std::vector<std::pair<double,double>> samples;
  Eigen::Vector3d previous=curve.evaluateDeBoorT(0.);
  for(double t:{0.,curve.getTimeSum()}) {
    const Eigen::Vector3d velocity=v.evaluateDeBoorT(t),acc=a.evaluateDeBoorT(t);
    const double speed=velocity.head<2>().norm();
    if(speed>1e-9)result.endpoint_peak=std::max(result.endpoint_peak,
      std::abs(velocity.x()*acc.y()-velocity.y()*acc.x())/(speed*speed*speed));
  }
  result.peak=result.endpoint_peak;
  for(int i=1;i<=2000;++i) {
    const double t=curve.getTimeSum()*i/2000.;const Eigen::Vector3d p=curve.evaluateDeBoorT(t);
    const Eigen::Vector3d velocity=v.evaluateDeBoorT(t),acc=a.evaluateDeBoorT(t);
    const double ds=(p-previous).head<2>().norm(),speed=velocity.head<2>().norm();previous=p;result.length+=ds;
    if(speed<1e-9)continue;
    const double k=std::abs(velocity.x()*acc.y()-velocity.y()*acc.x())/(speed*speed*speed);
    result.peak=std::max(result.peak,k);result.turn+=k*ds;result.energy+=k*k*ds;samples.emplace_back(k,ds);
    if(t>=1.&&t<=curve.getTimeSum()-1.)result.interior_peak=std::max(result.interior_peak,k);
  }
  std::sort(samples.begin(),samples.end());
  double accumulated=0.;for(const auto& sample:samples) {
    accumulated+=sample.second;result.arc_p95=sample.first;if(accumulated>=.95*result.length)break;
  }
  std::cout<<"geometry,weight="<<weight<<",peak="<<result.peak<<",arc_p95="<<result.arc_p95
      <<",interior_peak="<<result.interior_peak<<",fixed_endpoint_peak="<<result.endpoint_peak
      <<",turn="<<result.turn<<",energy="<<result.energy<<",length="<<result.length<<'\n';
  return result;
}
scan_planner::SplineHeadingContract headingFor(const Eigen::MatrixXd& q) {
  scan_planner::UniformBspline curve(q,3,1.);
  const Eigen::Vector3d initial=curve.getDerivative().evaluateDeBoorT(0.);
  scan_planner::SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
  heading.measured_yaw=std::atan2(initial.y(),initial.x());heading.body_extent=std::hypot(.93,.30)*.5;
  return heading;
}
}

namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void configure(BsplineOptimizer& o,const Eigen::MatrixXd& q,double weight) {
    o.order_=3;o.lambda1_=1.;o.lambda2_=o.new_lambda2_=1.;o.lambda3_=.1;o.lambda4_=1.;
    o.lambda_planar_curvature_=weight;o.preferred_turn_radius_=1.;o.dist0_=.28;
    o.max_vel_=.30;o.max_acc_=.35;o.bspline_interval_=1.;o.iter_num_=0;
    o.setPreviewHeadingContract(headingFor(q));
    o.cps_.resize(q.cols());o.cps_.points=q;o.cps_.clearance=.28;
    o.ref_pts_.clear();for(int i=1;i+1<q.cols();++i)o.ref_pts_.push_back((q.col(i-1)+4.*q.col(i)+q.col(i+1))/6.);
  }
  static double cost(BsplineOptimizer& o,bool refine,Eigen::VectorXd& gradient) {
    const int n=3*(o.cps_.size-6);Eigen::VectorXd variables(n);gradient.resize(n);
    variables=Eigen::Map<Eigen::VectorXd>(o.cps_.points.data()+9,n);
    double value=0.;
    if(refine)o.combineCostRefine(variables.data(),gradient.data(),value,n);
    else o.combineCostRebound(variables.data(),gradient.data(),value,n);
    return value;
  }
};
}

struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto m=std::make_shared<GridMap>();auto& p=m->mp_;auto& d=m->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={160,80,64};
    p.map_origin_idx_.setZero();m->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.require_observed_free_=false;p.planning_collision_policy_="official";
    p.collision_model_="oriented_box";p.box_length_=.93;p.box_width_=.30;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    const std::size_t size=160U*80U*64U;d.occupancy_buffer_.assign(size,-1.);
    d.occupancy_buffer_inflate_.assign(size,0);d.occupancy_buffer_inflate_cnt_.assign(size,0);
    m->rebuildInflationOffsets();return m;
  }
};

TEST(PlanarCurvatureCost, StraightAndRepeatedPointsStayZeroFinite) {
  Eigen::MatrixXd q=Eigen::MatrixXd::Zero(3,12),g;
  EXPECT_DOUBLE_EQ(scan_planner::planarCurvatureCost(q,1.,g),0.);EXPECT_TRUE(g.isZero());
  for(int i=0;i<q.cols();++i)q.col(i)=Eigen::Vector3d(.2*i,0.,i*.1);
  EXPECT_DOUBLE_EQ(scan_planner::planarCurvatureCost(q,1.,g),0.);EXPECT_TRUE(g.isZero());
  q(0,2)=NAN;EXPECT_THROW(scan_planner::planarCurvatureCost(q,1.,g),std::invalid_argument);
  EXPECT_THROW(scan_planner::planarCurvatureCost(bend(),0.,g),std::invalid_argument);
}
TEST(PlanarCurvatureCost, AnalyticGradientMatchesBothTurnSignsAndDoesNotModifyHeight) {
  for(double sign:{-1.,1.}) {
    auto q=bend(sign);Eigen::MatrixXd gradient;
    ASSERT_GT(scan_planner::planarCurvatureCost(q,1.,gradient),0.);
    EXPECT_TRUE(gradient.row(2).isZero());
    for(int dim=0;dim<3;++dim)for(int i=0;i<q.cols();++i) {
      constexpr double h=1e-7;auto plus=q,minus=q;plus(dim,i)+=h;minus(dim,i)-=h;
      Eigen::MatrixXd unused;
      const double numeric=(scan_planner::planarCurvatureCost(plus,1.,unused)-
                            scan_planner::planarCurvatureCost(minus,1.,unused))/(2.*h);
      EXPECT_NEAR(numeric,gradient(dim,i),2e-5*(1.+std::abs(numeric)))<<dim<<','<<i;
    }
  }
}
TEST(PlanarCurvatureCost, RigidTransformHeightAndRetimingDoNotChangeGeometricObjective) {
  const auto q=bend();Eigen::MatrixXd g;const double original=scan_planner::planarCurvatureCost(q,1.,g);
  auto transformed=q;const double angle=1.7;
  for(int i=0;i<q.cols();++i)transformed.col(i)=Eigen::Vector3d(
    std::cos(angle)*q(0,i)-std::sin(angle)*q(1,i)+50.,
    std::sin(angle)*q(0,i)+std::cos(angle)*q(1,i)-30.,100.+i*.2);
  EXPECT_NEAR(scan_planner::planarCurvatureCost(transformed,1.,g),original,1e-9);
  scan_planner::UniformBspline fast(q,3,.2),slow(q,3,1.3);
  EXPECT_DOUBLE_EQ(scan_planner::planarCurvatureCost(fast.getControlPoint(),1.,g),
                   scan_planner::planarCurvatureCost(slow.getControlPoint(),1.,g));
}
TEST(PlanarCurvatureCost, BothProductionCostFunctionsAddTheSameSoftTermOnly) {
  const auto q=bend();Eigen::MatrixXd g;const double expected=.02*scan_planner::planarCurvatureCost(q,1.,g);
  for(bool refine:{false,true}) {
    scan_planner::BsplineOptimizer off,on;scan_planner::BsplineOptimizerTestAccess::configure(off,q,0.);
    scan_planner::BsplineOptimizerTestAccess::configure(on,q,.02);Eigen::VectorXd before,after;
    const double baseline=scan_planner::BsplineOptimizerTestAccess::cost(off,refine,before);
    const double changed=scan_planner::BsplineOptimizerTestAccess::cost(on,refine,after);
    EXPECT_NEAR(changed-baseline,expected,1e-12);
    const auto expected_gradient=.02*Eigen::Map<Eigen::VectorXd>(g.data()+9,3*(q.cols()-6));
    EXPECT_TRUE((after-before).isApprox(expected_gradient,1e-11));
  }
}
TEST(PlanarCurvatureCost, NativeOccupiedBoundaryStillRejectsAndCancellationStillApplies) {
  auto q=bend();auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::configure(optimizer,q,.2);
  optimizer.setEnvironment(map);optimizer.a_star_=std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map,{80,40,32});auto budget=std::make_shared<scan_planner::SolveBudget>();
  optimizer.setSolveBudget(budget);optimizer.initControlPoints(q);ASSERT_TRUE(optimizer.controlPointsInitialized());
  scan_planner::UniformBspline curve(q,3,1.);const Eigen::Vector3d occupied=curve.evaluateDeBoorT(0.);
  map->setOccupied(occupied);Eigen::MatrixXd out;
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(out,1.));EXPECT_EQ(map->getOccupancy(occupied),1);
  budget->cancel();EXPECT_FALSE(optimizer.BsplineOptimizeTrajRefine(q,1.,out));
}

// Both sides use the same production optimizer/GridMap, D1 rigid box and
// measured-body orientation sweep. These deterministic geometry fixtures are
// not physical calibration or a promise that every local optimum improves.
TEST(PlanarCurvatureCost, D1ProductionPairedCurvesImproveAndRetainCollisionAndBoundary) {
  for(int shape=0;shape<4;++shape) {
    GeometryMetrics before,after;
    for(double weight:{0.,.2}) {
    auto q=bend();
    if(shape==0)for(int i=0;i<q.cols();++i) {
      const double u=std::clamp((i-2.)/9.,0.,1.);
      q(1,i)=1.+.6*(1.-(3.*u*u-2.*u*u*u));
    }
    if(shape==1)for(int i=0;i<q.cols();++i) {
      q(0,i)=.5+.2*std::min(i,6);q(1,i)=.2*std::max(0,i-6);
    }
    auto map=GridMapTestAccess::make();
    if(shape==2) {q.row(1).setConstant(1.);map->setOccupied(Eigen::Vector3d(1.8,1.,.55));}
    scan_planner::BsplineOptimizer optimizer;scan_planner::BsplineOptimizerTestAccess::configure(optimizer,q,weight);
    optimizer.setEnvironment(map);optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,{80,40,32});
    auto budget=std::make_shared<scan_planner::SolveBudget>();optimizer.setSolveBudget(budget);
    const auto source=q;const auto heading=headingFor(source);
    const auto start=std::chrono::steady_clock::now();optimizer.initControlPoints(q);
    Eigen::MatrixXd result;const bool solved=optimizer.controlPointsInitialized()&&optimizer.BsplineOptimizeTrajRebound(result,1.);
    const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"scenario="<<shape<<",weight="<<weight<<",solved="<<solved<<",reason="<<optimizer.lastFailurePhase()
        <<",elapsed_ms="<<elapsed<<'\n';
    ASSERT_TRUE(solved)<<shape<<','<<weight<<','<<optimizer.lastFailurePhase();
    EXPECT_LT(elapsed,400.);
      EXPECT_TRUE(result.leftCols(3).isApprox(source.leftCols(3),0.));
      EXPECT_TRUE(result.rightCols(3).isApprox(source.rightCols(3),0.));
      EXPECT_TRUE(result.row(2).isApprox(source.row(2),0.));
      scan_planner::UniformBspline curve(result,3,1.),original(source,3,1.);
      // Explicit p/v/a equality, not just an unchanged goal coordinate.
      for(int derivative=0;derivative<3;++derivative) {
        for(double t:{0.,original.getTimeSum()})
          EXPECT_TRUE(curve.evaluateDeBoorT(t).isApprox(original.evaluateDeBoorT(t),1e-14));
        curve=curve.getDerivative();original=original.getDerivative();
      }
      curve=scan_planner::UniformBspline(result,3,1.);
      const auto collision=scan_planner::checkWholeSplineCollision(curve,.05,
        [&](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},50000,heading);
      EXPECT_EQ(collision.state,scan_planner::SplineCollisionState::Clear);
      EXPECT_GT(collision.queries,0U);
      if(weight==0.)before=geometryMetrics(result,weight);else after=geometryMetrics(result,weight);
    }
    // A fixed endpoint's P/V/A can fix the full-curve peak. The objective
    // must not claim to remove that immutable boundary curvature.
    EXPECT_LE(after.peak,before.peak)<<shape;
    EXPECT_LT(after.interior_peak,before.interior_peak)<<shape;
    EXPECT_LT(after.arc_p95,before.arc_p95)<<shape;
    EXPECT_LT(after.energy,before.energy)<<shape;
    EXPECT_LE(after.turn,before.turn+1e-4)<<shape;
    EXPECT_DOUBLE_EQ(after.endpoint_peak,before.endpoint_peak)<<shape;
    if(shape==1||shape==2) {
      EXPECT_LT(after.peak,.8*before.peak);
      EXPECT_LT(after.energy,.8*before.energy);
      EXPECT_GT(after.peak,1.); // preferred radius is a soft objective, NOT a new rejection gate.
    }
  }
}

TEST(PlanarCurvatureCost, D1CornerObstacleStillBlocksMeasuredRotationSweep) {
  auto q=bend();q.row(1).setConstant(0.);auto map=GridMapTestAccess::make();
  scan_planner::UniformBspline curve(q,3,1.);
  const Eigen::Vector3d origin=curve.evaluateDeBoorT(0.);
  // This voxel is outside the box at both endpoint headings, but inside its
  // corner during the initial 90-degree alignment. A centerline-only query
  // or an omitted orientation sweep would falsely report clear.
  const Eigen::Vector3d occupied=origin+Eigen::Vector3d(.30,.30,0.);
  map->setOccupied(occupied);
  EXPECT_EQ(map->getPlanningInflateOccupancy(origin,0.),0);
  EXPECT_EQ(map->getPlanningInflateOccupancy(origin,M_PI_2),0);
  auto heading=headingFor(q);heading.measured_yaw=M_PI_2;
  const auto collision=scan_planner::checkWholeSplineCollision(curve,.05,
      [&](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},50000,heading);
  EXPECT_EQ(collision.state,scan_planner::SplineCollisionState::Collision);
  EXPECT_EQ(map->getOccupancy(occupied),1);
}
