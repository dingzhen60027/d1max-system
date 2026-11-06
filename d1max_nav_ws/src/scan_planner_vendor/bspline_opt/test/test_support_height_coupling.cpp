#include <gtest/gtest.h>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/reference_path.hpp>
#include <bspline_opt/trajectory_timing.hpp>
#include <bspline_opt/whole_spline_collision.hpp>
#include <cstring>

struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto m=std::make_shared<GridMap>();auto& p=m->mp_;auto& d=m->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={200,100,160};
    p.map_origin_idx_.setZero();m->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.require_observed_free_=false;p.planning_collision_policy_="official";
    p.collision_model_="oriented_box";p.box_length_=.98;p.box_width_=.35;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    const std::size_t n=200U*100U*160U;
    d.occupancy_buffer_.assign(n,-1.);d.occupancy_buffer_inflate_.assign(n,0);
    d.occupancy_buffer_inflate_cnt_.assign(n,0);
    d.planning_occupancy_buffer_inflate_.assign(n,0);d.planning_occupancy_buffer_inflate_cnt_.assign(n,0);
    m->rebuildInflationOffsets();return m;
  }
  static auto raw(const GridMap& m) {return m.md_.occupancy_buffer_;}
};

namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void setup(BsplineOptimizer& o,const GridMap::Ptr& m) {
    o.order_=3;o.lambda1_=1.;o.lambda2_=o.new_lambda2_=1.;o.lambda3_=.1;o.lambda4_=1.;
    o.lambda_reference_=1.;o.lambda_planar_curvature_=.2;o.preferred_turn_radius_=1.;
    o.dist0_=.28;o.max_vel_=.3;o.max_acc_=.35;o.iter_num_=0;
    o.setEnvironment(m);o.a_star_=std::make_shared<AStar>();o.a_star_->initGridMap(m,{100,50,80});
  }
  static void bind(BsplineOptimizer& o,const Eigen::MatrixXd& q,double dt) {
    o.cps_.resize(q.cols());o.cps_.points=q;o.cps_.clearance=.28;o.bspline_interval_=dt;
    o.support_height_coupling_.prepare(q,dt,o.support_height_sampler_);
  }
  static double evaluate(BsplineOptimizer& o,const Eigen::VectorXd& x,Eigen::VectorXd& g,bool refine) {
    double cost=0.;g.resize(x.size());
    if(refine)o.combineCostRefine(x.data(),g.data(),cost,x.size());
    else o.combineCostRebound(x.data(),g.data(),cost,x.size());
    return cost;
  }
  static Eigen::MatrixXd restore(BsplineOptimizer& o,const Eigen::VectorXd& x) {
    o.restoreOptimizationVariables(x.data(),x.size());return o.cps_.points;
  }
  static bool supported(BsplineOptimizer& o,UniformBspline curve) {
    return o.coupledCurveWithinSupport(curve,o.support_height_coupling_,{});
  }
};
}
namespace {
using namespace scan_planner;
SupportHeightSampler plane(Eigen::Vector2d grade,double b=.55) {
  return [=](const Eigen::Vector3d& p)->std::optional<SupportHeightSample>{
    return SupportHeightSample{b+grade.dot(p.head<2>()),grade};};
}
struct Fixture {
  Eigen::MatrixXd q;double dt;CubicMotionBoundary boundary;
  std::vector<Eigen::Vector3d> samples;
};
Fixture fixture(Eigen::Vector2d grade,double residual=0.) {
  std::vector<Eigen::Vector3d> reference;
  for(int i=0;i<=60;++i) {
    const Eigen::Vector2d xy(.05*i,.12*std::sin(M_PI*i/60.));
    reference.emplace_back(xy.x(),xy.y(),.55+grade.dot(xy)+residual);
  }
  DiscreteReference path;path.set(reference);
  const Eigen::Vector3d v0=(reference[1]-reference[0]).normalized()*.25;
  const Eigen::Vector3d v1=(reference.back()-reference[59]).normalized()*.25;
  const Eigen::Vector3d a0(.025,.01,grade.dot(Eigen::Vector2d(.025,.01)));
  const Eigen::Vector3d a1(-.02,.01,grade.dot(Eigen::Vector2d(-.02,.01)));
  CubicMotionBoundary b{reference.front(),v0,a0,reference.back(),v1,a1};
  auto seed=sampleReferenceSeed(path,.3,.35,v0.norm(),v1.norm(),.2);
  return {fitCubicWithFixedBoundary(seed.samples,seed.dt,b),seed.dt,b,seed.samples};
}
double error(UniformBspline curve,Eigen::Vector2d grade,double residual=0.) {
  double result=0.;
  for(int i=0;i<=500;++i) {
    const auto p=curve.evaluateDeBoorT(curve.getTimeSum()*i/500.);
    result=std::max(result,std::abs(p.z()-.55-grade.dot(p.head<2>())-residual));
  }return result;
}
bool equalBytes(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b) {
  return a.rows()==b.rows()&&a.cols()==b.cols()&&
      std::memcmp(a.data(),b.data(),a.size()*sizeof(double))==0;
}
void refs(BsplineOptimizer& o,UniformBspline curve) {
  o.ref_pts_.clear();
  for(int i=0;i<=curve.getControlPoint().cols()-3;++i)
    o.ref_pts_.push_back(curve.evaluateDeBoorT(i*curve.getInterval()));
}
}

TEST(SupportHeightCoupling, ActualCurveSamplingPreservesResidualAndFixedNonzeroPVA) {
  for(double slope:{-.8,-.3,.3,.8}) {
    const auto f=fixture({slope,.13},-.07);SupportHeightCoupling coupling;
    int calls=0;const auto sampler=plane({slope,.13});
    ASSERT_TRUE(coupling.prepare(f.q,f.dt,[&](const auto& p){++calls;return sampler(p);}));
    EXPECT_EQ(calls,f.q.cols());
    auto q=f.q;
    for(int i=3;i<q.cols()-3;++i){q(0,i)+=.16*std::sin(i*.7);q(1,i)+=.04*std::cos(i*.4);}
    coupling.apply(q);
    EXPECT_TRUE(equalBytes(q.leftCols(3),f.q.leftCols(3)));
    EXPECT_TRUE(equalBytes(q.rightCols(3),f.q.rightCols(3)));
    EXPECT_TRUE(cubicBoundaryMatches(UniformBspline(q,3,f.dt),f.boundary));
    EXPECT_LT(error(UniformBspline(q,3,f.dt),{slope,.13},-.07),1e-11);
    const auto once=q;coupling.apply(q);EXPECT_TRUE(equalBytes(q,once));
    EXPECT_EQ(calls,f.q.cols()); // no iteration-time sampling or repeated offset
  }
}

TEST(SupportHeightCoupling, LandingMixedSlopesMissingSupportAndOtherFloorKeepOriginal) {
  const auto f=fixture({.4,0.});SupportHeightCoupling coupling;
  EXPECT_FALSE(coupling.prepare(f.q,f.dt,[](const Eigen::Vector3d& p)->std::optional<SupportHeightSample>{
    return p.x()<1.5?SupportHeightSample{.55+.4*p.x(),{.4,0.}}:
      SupportHeightSample{1.15,{0.,0.}};}));
  EXPECT_FALSE(coupling.bound());auto q=f.q;coupling.apply(q);EXPECT_TRUE(equalBytes(q,f.q));
  EXPECT_FALSE(coupling.prepare(f.q,f.dt,[](const Eigen::Vector3d& p)->std::optional<SupportHeightSample>{
    return SupportHeightSample{.55+.4*p.x()+(p.x()>1.5?3.:0.),{.4,0.}};}));
  EXPECT_FALSE(coupling.prepare(f.q,f.dt,[](const auto&)->std::optional<SupportHeightSample>{return {};}));
  EXPECT_FALSE(coupling.prepare(f.q,f.dt,{}));
  const auto flat=fixture({0.,0.});EXPECT_FALSE(coupling.prepare(flat.q,flat.dt,plane({0.,0.})));
  EXPECT_TRUE(coupling.bound());q=flat.q;coupling.apply(q);EXPECT_TRUE(equalBytes(q,flat.q));
}

TEST(SupportHeightCoupling, ProductionReboundAndRefineGradientsMatchFiniteDifferenceInTiltedOdom) {
  // normal is not aligned with world X: yaw/roll transformed support plane.
  const Eigen::Vector3d normal=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.16,Eigen::Vector3d::UnitX())*Eigen::Vector3d(-.6,0.,1.)).normalized();
  const Eigen::Vector2d grade=-normal.head<2>()/normal.z();const auto f=fixture(grade,.03);
  BsplineOptimizer o;BsplineOptimizerTestAccess::setup(o,GridMapTestAccess::make());
  o.setSupportHeightSampler(plane(grade));o.reference_path_samples_=f.samples;
  refs(o,UniformBspline(f.q,3,f.dt));BsplineOptimizerTestAccess::bind(o,f.q,f.dt);
  Eigen::VectorXd x=Eigen::Map<const Eigen::VectorXd>(f.q.data()+9,3*(f.q.cols()-6));
  x[6]+=.03;x[10]-=.01;
  for(bool refine:{false,true}) {
    Eigen::VectorXd gradient;BsplineOptimizerTestAccess::evaluate(o,x,gradient,refine);
    for(int i=0;i<x.size();++i) {
      auto plus=x,minus=x;plus[i]+=1e-6;minus[i]-=1e-6;Eigen::VectorXd unused;
      const double finite=(BsplineOptimizerTestAccess::evaluate(o,plus,unused,refine)-
          BsplineOptimizerTestAccess::evaluate(o,minus,unused,refine))/2e-6;
      EXPECT_NEAR(gradient[i],finite,2e-5*std::max(1.,std::abs(finite)))<<i<<" refine="<<refine;
    }
    const auto restored=BsplineOptimizerTestAccess::restore(o,x);
    EXPECT_LT(error(UniformBspline(restored,3,f.dt),grade,.03),1e-11);
    EXPECT_TRUE(cubicBoundaryMatches(UniformBspline(restored,3,f.dt),f.boundary));
  }
}

TEST(SupportHeightCoupling, ProductionOptimizeAndRetimeRemoveSlopeDriftNotMeasuredResidual) {
  for(double slope:{-.6,-.3,.3,.6}) {
    const Eigen::Vector2d grade(slope,0.);const auto f=fixture(grade,-.04);
    auto map=GridMapTestAccess::make();map->exchangeQuerySupportHeightProvider(
        [=](const Eigen::Vector3d& p){return std::optional<double>(.55+slope*p.x());});
    const auto raw=GridMapTestAccess::raw(*map);BsplineOptimizer o;BsplineOptimizerTestAccess::setup(o,map);
    o.setSupportHeightSampler(plane(grade));o.reference_path_samples_=f.samples;
    auto q=f.q;o.initControlPointsWithSpline(q,f.dt,true);
    ASSERT_TRUE(o.BsplineOptimizeTrajRebound(q,f.dt))<<slope;
    EXPECT_LT(error(UniformBspline(q,3,f.dt),grade,-.04),1e-10);
    UniformBspline curve(q,3,f.dt);
    const auto refine=[&](Eigen::MatrixXd& controls,double dt) {
      refs(o,UniformBspline(controls,3,dt));Eigen::MatrixXd out;
      const bool ok=o.BsplineOptimizeTrajRefine(controls,dt,out);if(ok)controls=out;return ok;
    };
    const auto timed=refineTimingWithFixedBoundary(curve,f.boundary,.3,.35,refine,8,std::nullopt,{},true);
    ASSERT_TRUE(timed.success)<<timed.reason;
    EXPECT_TRUE(cubicBoundaryMatches(curve,f.boundary));EXPECT_LT(error(curve,grade,-.04),1e-9);
    EXPECT_EQ(GridMapTestAccess::raw(*map),raw);
  }
}

TEST(SupportHeightCoupling, FlatAndClearedWorkerProduceByteIdenticalProductionOutput) {
  const auto f=fixture({0.,0.});Eigen::MatrixXd baseline;
  for(int mode=0;mode<3;++mode) {
    BsplineOptimizer o;BsplineOptimizerTestAccess::setup(o,GridMapTestAccess::make());
    if(mode==1)o.setSupportHeightSampler(plane({0.,0.}));
    if(mode==2){o.setSupportHeightSampler(plane({.6,0.}));o.setSupportHeightSampler({});}
    o.reference_path_samples_=f.samples;auto q=f.q;o.initControlPointsWithSpline(q,f.dt,true);
    ASSERT_TRUE(o.BsplineOptimizeTrajRebound(q,f.dt));refs(o,UniformBspline(q,3,f.dt));
    Eigen::MatrixXd refined;ASSERT_TRUE(o.BsplineOptimizeTrajRefine(q,f.dt,refined));
    if(mode==0)baseline=refined;else EXPECT_TRUE(equalBytes(baseline,refined));
  }
}

TEST(SupportHeightCoupling, ActualBodyObstacleRemainsOccupiedAndRawVoxelsUnchanged) {
  const auto f=fixture({.5,0.});auto q=f.q;SupportHeightCoupling coupling;
  ASSERT_TRUE(coupling.prepare(q,f.dt,plane({.5,0.})));
  q(0,q.cols()/2)+=.1;coupling.apply(q);UniformBspline curve(q,3,f.dt);
  auto map=GridMapTestAccess::make();const Eigen::Vector3d p=curve.evaluateDeBoorT(curve.getTimeSum()*.5);
  map->setOccupied(p);const auto raw=GridMapTestAccess::raw(*map);
  const auto result=checkWholeSplineCollision(curve,.05,[&](const auto& point,double yaw){
    return map->getPlanningInflateOccupancy(point,yaw);});
  EXPECT_EQ(result.state,SplineCollisionState::Collision);EXPECT_EQ(GridMapTestAccess::raw(*map),raw);
}

TEST(SupportHeightCoupling, ActualCoupledCurveCannotExtrapolateIntoLandingMissingSupportOrOtherFloor) {
  const auto f=fixture({.4,0.});
  for(int outside:{0,1,2}) {
    const SupportHeightSampler bounded=[=](const Eigen::Vector3d& p)->std::optional<SupportHeightSample> {
      if(std::abs(p.y())<=.3)return SupportHeightSample{.55+.4*p.x(),{.4,0.}};
      if(outside==0)return {};
      if(outside==1)return SupportHeightSample{1.15,{0.,0.}};
      return SupportHeightSample{3.55+.4*p.x(),{.4,0.}};
    };
    BsplineOptimizer o;auto map=GridMapTestAccess::make();BsplineOptimizerTestAccess::setup(o,map);
    o.setSupportHeightSampler(bounded);BsplineOptimizerTestAccess::bind(o,f.q,f.dt);
    ASSERT_TRUE(BsplineOptimizerTestAccess::supported(o,UniformBspline(f.q,3,f.dt)));
    auto changed=f.q;changed(1,changed.cols()/2)+=1.;
    Eigen::VectorXd x=Eigen::Map<const Eigen::VectorXd>(changed.data()+9,3*(changed.cols()-6));
    changed=BsplineOptimizerTestAccess::restore(o,x);
    UniformBspline changed_curve(changed,3,f.dt);
    const auto collision=checkWholeSplineCollision(changed_curve,.05,
        [&](const auto& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);});
    EXPECT_EQ(collision.state,SplineCollisionState::Clear); // no fabricated obstacle
    EXPECT_FALSE(BsplineOptimizerTestAccess::supported(o,UniformBspline(changed,3,f.dt)));
  }
}

TEST(SupportHeightCoupling, SeedSamplingMatchesCubicAndSharesCancellationBudget) {
  const auto f=fixture({.4,.17});UniformBspline curve(f.q,3,f.dt);int index=0;
  SupportHeightCoupling coupling;
  ASSERT_TRUE(coupling.prepare(f.q,f.dt,[&](const Eigen::Vector3d& p)->std::optional<SupportHeightSample> {
    const auto actual=curve.evaluateDeBoorT(std::clamp((index++-1)*f.dt,0.,curve.getTimeSum()));
    EXPECT_NEAR((p-actual).norm(),0.,1e-12);return plane({.4,.17})(p);
  }));
  int remaining=3,calls=0;
  EXPECT_FALSE(coupling.prepare(f.q,f.dt,[&](const auto& p){++calls;return plane({.4,.17})(p);},
      [&]{return remaining-->0;}));
  EXPECT_EQ(calls,3);EXPECT_FALSE(coupling.active());EXPECT_FALSE(coupling.bound());
}
