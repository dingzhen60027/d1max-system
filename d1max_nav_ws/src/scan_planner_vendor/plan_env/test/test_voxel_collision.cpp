#include <gtest/gtest.h>
#include <plan_env/voxel_collision.hpp>
#include <plan_env/observed_ray.hpp>
#include <set>
#include <tuple>

using scan_planner::conservativeCylinderInflation;
using scan_planner::observedCylinderStatus;
using Point=Eigen::Vector3d;
using Cell=Eigen::Vector3i;

namespace {
std::set<std::tuple<int,int,int>> keys(const std::vector<Cell> &kernel) {
  std::set<std::tuple<int,int,int>> result;
  for (const auto &p:kernel) result.emplace(p.x(),p.y(),p.z());
  return result;
}
Cell cell(const Point &p,double resolution=.08) {
  return (p.array()/resolution).floor().cast<int>();
}
}  // namespace

TEST(VoxelCollision, CapturesMeasuredFrontCornerThatCenterOnlyKernelMisses) {
  // Reproduced accepted synthetic trajectory's front cylinder: analytic box
  // corner lies 5.5 mm inside r=.29, but its occupied grid center is too far.
  const Point body_center(1.53995,.681681,.55), corner(1.5,.399999,.55);
  EXPECT_LT((body_center-corner).head<2>().norm(),.29);
  const Cell delta=cell(body_center)-cell(corner);
  EXPECT_GT(delta.head<2>().cast<double>().norm()*.08,.29);
  const auto kernel=keys(conservativeCylinderInflation(.08,.29,.15,.45));
  EXPECT_EQ(kernel.count({delta.x(),delta.y(),delta.z()}),1U);
}

TEST(VoxelCollision, EveryAnalyticCollisionAcrossVoxelPhasesIsContained) {
  const double res=.08,r=.29,below=.15,above=.45;
  const auto kernel=keys(conservativeCylinderInflation(res,r,below,above));
  int collisions=0;
  for (double ox : {.001,.039,.079}) for (double oy : {.001,.039,.079})
    for (double oz : {.001,.039,.079})
      for (double angle : {0.,.3,.8,1.7,2.5,3.3,4.5,5.9})
        for (double distance : {0.,.15,.289,.29})
          for (double dz : {-.45,-.08,0.,.14,.15}) {
            const Point obstacle(ox,oy,oz);
            // q-o lies in the reflected body Minkowski envelope.
            const Point query=obstacle+Point(distance*std::cos(angle),distance*std::sin(angle),dz);
            const Cell delta=cell(query)-cell(obstacle);
            ASSERT_EQ(kernel.count({delta.x(),delta.y(),delta.z()}),1U)
                << "obstacle="<<obstacle.transpose()<<" query="<<query.transpose();
            ++collisions;
          }
  EXPECT_GT(collisions,4000);
}

TEST(VoxelCollision, ConservativeKernelIsBoundedAndExcludesDistantSpace) {
  const auto kernel=keys(conservativeCylinderInflation(.08,.29,.15,.45));
  EXPECT_LT(kernel.size(),2000U);
  EXPECT_EQ(kernel.count({20,0,0}),0U);
  EXPECT_EQ(kernel.count({0,0,20}),0U);
  EXPECT_THROW(conservativeCylinderInflation(.00001,10.,10.,10.),std::invalid_argument);
  EXPECT_THROW(conservativeCylinderInflation(.08,-.1,.1,.2),std::invalid_argument);
}

TEST(VoxelCollision, UnknownBodyCornerBlocksEvenWhenCenterIsMeasuredFree) {
  const auto kernel=conservativeCylinderInflation(.08,.29,.15,.45);
  const Cell center(0,0,0), unknown(3,0,2);
  const int state=observedCylinderStatus(center,kernel,[&](const Cell &query) {
    return query==unknown ? 2:0;
  });
  EXPECT_EQ(state,2);
}

TEST(VoxelCollision, ReflectedBodyVerticalExtentIsNotSwapped) {
  const auto kernel=conservativeCylinderInflation(.08,.29,.15,.45);
  // Actual body reaches +.45 above the query; query-cell quantization makes
  // cell +6 intersect it. The lower body is only .15, so cell -6 is outside.
  EXPECT_EQ(observedCylinderStatus(Cell::Zero(),kernel,
      [](const Cell &q){return q==Cell(0,0,6) ? 2:0;}),2);
  EXPECT_EQ(observedCylinderStatus(Cell::Zero(),kernel,
      [](const Cell &q){return q==Cell(0,0,-6) ? 2:0;}),0);
}

TEST(VoxelCollision, ActualHeightEnvelopeDetectsLowObstaclesWithoutBlockingGround) {
  const double res=.08, below=.45, above=.45;
  const auto kernel=conservativeCylinderInflation(res,.29,below,above);
  for (double ground:{-2.037,-.8,-.079,-.04,0.,.039,.079,3.141}) {
    const Point body(.01,.02,ground+.55);
    const auto span=scan_planner::verticalVoxelSpan(body.z(),below,above,res);
    const Cell floor=cell(Point(.01,.02,ground));
    EXPECT_EQ(scan_planner::observedCylinderStatusAtHeight(cell(body),span,kernel,
        [&](const Cell &c){return c==floor ? 1:0;}),0);
    for (double height:{.101,.12,.20,.34,.55,.9}) {
      const Cell obstacle=cell(Point(.01,.02,ground+height));
      EXPECT_EQ(scan_planner::observedCylinderStatusAtHeight(cell(body),span,kernel,
          [&](const Cell &c){return c==obstacle ? 1:0;}),1) << ground<<","<<height;
      EXPECT_EQ(scan_planner::observedCylinderStatusAtHeight(cell(body),span,kernel,
          [&](const Cell &c){return c==obstacle ? 2:0;}),2);
    }
  }
}

TEST(VoxelCollision, HeightCacheCannotReuseAFreeResultForADifferentVerticalSpan) {
  scan_planner::VoxelStatusCache cache(4);
  int calls=0;
  EXPECT_EQ(cache.get(12,1,10,[&](){++calls;return 0;}),0);
  EXPECT_EQ(cache.get(12,0,10,[&](){++calls;return 1;}),1);
  EXPECT_EQ(cache.get(12,1,10,[&](){++calls;return 2;}),0);
  EXPECT_EQ(calls,2);
  cache.clear();
  EXPECT_EQ(cache.get(12,1,10,[](){return 2;}),2);
}

TEST(VoxelCollision, ActualHeightEnvelopeConservativelyIncludesTouchingVoxel) {
  const auto span=scan_planner::verticalVoxelSpan(.56,.40,.40,.08);
  EXPECT_EQ(span.low,1); // voxel [.08,.16] touches the lower face .16
  EXPECT_EQ(span.high,12); // voxel [.96,1.04] touches upper face .96
  EXPECT_THROW(scan_planner::verticalVoxelSpan(.5,-.1,.4,.08),std::invalid_argument);
  EXPECT_THROW(scan_planner::verticalVoxelSpan(INFINITY,.4,.4,.08),std::invalid_argument);
}

TEST(VoxelCollision, CompleteMeasuredEnvelopePassesAndOutOfMapFails) {
  const auto kernel=conservativeCylinderInflation(.08,.29,.15,.45);
  EXPECT_EQ(observedCylinderStatus(Cell::Zero(),kernel,[](const Cell &){return 0;}),0);
  EXPECT_EQ(observedCylinderStatus(Cell::Zero(),kernel,
      [](const Cell &q){return q.x()<0 ? -1:0;}),-1);
  EXPECT_EQ(observedCylinderStatus(Cell::Zero(),kernel,
      [](const Cell &q){return q==Cell(1,0,0) ? 1:0;}),1);
}

TEST(VoxelCollision, SameCellCacheReusesOnlyCurrentSnapshotAndCannotGrowUnbounded) {
  scan_planner::VoxelStatusCache cache(4);
  int calls=0,state=0;
  auto query=[&](){++calls;return state;};
  EXPECT_EQ(cache.get(10,query),0);
  EXPECT_EQ(cache.get(10,query),0);
  EXPECT_EQ(calls,1);
  state=2;cache.clear(); // New cloud / ring-buffer slide / reset.
  EXPECT_EQ(cache.get(10,query),2);
  EXPECT_EQ(calls,2);
  for (int i=0;i<100;++i) {
    EXPECT_EQ(cache.get(i,query),2);
    EXPECT_LE(cache.size(),4U);
  }
}

TEST(VoxelCollision, HiddenSolidCannotBecomeFreeWithoutNewMeasurement) {
  const auto kernel=conservativeCylinderInflation(.08,.29,.15,.45);
  const Cell center=cell(Point(2.097429,.249686,.55));
  const auto front_only=[](const Cell &c) { return c.x()>=19 ? 2:0; };
  EXPECT_EQ(observedCylinderStatus(center,kernel,front_only),2);
}

TEST(ObservedRay, LocalWindowPreservesNearbyEvidenceWithoutInventingHit) {
  const Point origin(1.,2.,3.), extent(6.,6.,3.2);
  Point end=origin+Point(7.,0.,0.); bool hit=true;
  ASSERT_TRUE(scan_planner::clipObservedRay(origin,extent,end,hit));
  EXPECT_FALSE(hit);
  EXPECT_NEAR(end.x(),7.,2e-6);
  EXPECT_LT(end.x(),7.);
  std::size_t budget=1000; std::vector<Eigen::Vector3i> cells;
  ASSERT_TRUE(scan_planner::visitObservedRay(origin,end,.1,true,budget,
    [&](const Eigen::Vector3i &cell){cells.push_back(cell);}));
  EXPECT_GT(cells.size(),50U);
  end=origin+Point(2.,1.,1.);hit=true;
  ASSERT_TRUE(scan_planner::clipObservedRay(origin,extent,end,hit));
  EXPECT_TRUE(hit);
  EXPECT_TRUE(end.isApprox(origin+Point(2.,1.,1.)));
}

TEST(ObservedRay, LocalWindowClipsNegativeAndVerticalDirectionsOnSameRay) {
  const Point origin(0.,0.,0.), extent(6.,6.,3.2);
  Point end(-7.,1.,4.);bool hit=true;
  ASSERT_TRUE(scan_planner::clipObservedRay(origin,extent,end,hit));
  EXPECT_FALSE(hit);
  EXPECT_LT(end.z(),3.2);
  EXPECT_LT(end.x(),-5.5);
  EXPECT_NEAR(end.y()/end.z(),.25,1e-9);
}

TEST(ObservedRay, ExactFloatDirectionHitEndpointAndOriginEvidence) {
  std::set<std::tuple<int,int,int>> visited;
  std::size_t budget=100;
  ASSERT_TRUE(scan_planner::visitObservedRay(Point(.1,.1,.1),Point(2.99,1.01,.1),1.,false,budget,
      [&](const Cell &c){visited.emplace(c.x(),c.y(),c.z());}));
  EXPECT_EQ(visited.count({0,0,0}),1U);
  EXPECT_EQ(visited.count({2,0,0}),1U); // True floating ray, not integer-cell slope.
  EXPECT_EQ(visited.count({2,1,0}),0U); // Occupied endpoint never contributes miss.
  EXPECT_EQ(visited.count({1,1,0}),0U); // Not crossed by this actual ray.
}

TEST(ObservedRay, SharedCellsDoNotTerminateNewRayCoverage) {
  std::set<std::tuple<int,int,int>> visited;
  for(const Point &end:{Point(4.01,2.99,.1),Point(4.99,2.01,.1)}) {
    std::size_t budget=100;
    ASSERT_TRUE(scan_planner::visitObservedRay(Point(.1,.1,.1),end,1.,false,budget,
      [&](const Cell &c){visited.emplace(c.x(),c.y(),c.z());}));
  }
  EXPECT_EQ(visited.count({3,1,0}),1U);
  EXPECT_EQ(visited.count({2,2,0}),1U);
}

TEST(ObservedRay, CompleteDenseMeasuredRaysCoverBodyWithoutFillingHiddenCells) {
  std::set<std::tuple<int,int,int>> visited;
  const Point origin(0,0,.55);
  // Deterministic real rays through every query-body voxel to a farther plane.
  // No cell not intersecting one of these rays is labelled observed free.
  const auto kernel=conservativeCylinderInflation(.08,.29,.15,.45);
  const Cell center=cell(Point(2.,0,.55));
  for(const Cell &offset:kernel) {
    const Point middle=(center-offset).cast<double>()*.08+Point::Constant(.04);
    const Point endpoint=origin+3.*(middle-origin);
    std::size_t budget=1000;
    ASSERT_TRUE(scan_planner::visitObservedRay(origin,endpoint,.08,false,budget,
      [&](const Cell &c){visited.emplace(c.x(),c.y(),c.z());}));
  }
  EXPECT_EQ(observedCylinderStatus(center,kernel,[&](const Cell &c){
    return visited.count({c.x(),c.y(),c.z()}) ? 0:2;
  }),0);
  EXPECT_EQ(visited.count({200,200,200}),0U);
}

TEST(ObservedRay, BudgetFailureDoesNotManufactureRemainingEvidence) {
  std::size_t budget=2;int count=0;
  EXPECT_FALSE(scan_planner::visitObservedRay(Point(.1,.1,.1),Point(9.1,.1,.1),1.,false,budget,
    [&](const Cell&){++count;}));
  EXPECT_EQ(count,2);EXPECT_EQ(budget,0U);
}

TEST(ObservedRay, AxisTiesAndNegativeDirectionsDoNotVisitUncrossedCells) {
  std::set<std::tuple<int,int,int>> visited;
  std::size_t budget=100;
  ASSERT_TRUE(scan_planner::visitObservedRay(Point(2.5,2.5,.5),Point(-1.5,-1.5,.5),1.,false,budget,
    [&](const Cell&c){visited.emplace(c.x(),c.y(),c.z());EXPECT_EQ(c.x(),c.y());}));
  EXPECT_EQ(visited.size(),4U);
}

TEST(VoxelCollision, LowPriorPlusFirstHitIsUncertainNotObservedFree) {
  const double minimum=std::log(.12/.88),occupied=std::log(.8/.2);
  EXPECT_EQ(scan_planner::strictRawVoxelStatus(minimum-.01,minimum,occupied),2);
  EXPECT_EQ(scan_planner::strictRawVoxelStatus(minimum,minimum,occupied),0);
  EXPECT_EQ(scan_planner::strictRawVoxelStatus(minimum-.01+std::log(.85/.15),minimum,occupied),2);
  EXPECT_EQ(scan_planner::strictRawVoxelStatus(occupied+.01,minimum,occupied),1);
}

TEST(ObservedRay, OppositeSignedExactEndpointFacesTerminateWithoutOvershooting) {
  // Actual straight-corridor fixture point 63630 previously made complete=false.
  const Point origin(0,0,.55),end(2.,-2.,.0227178);
  std::size_t budget=2048;std::vector<Cell> visited;
  ASSERT_TRUE(scan_planner::visitObservedRay(origin,end,.08,false,budget,
    [&](const Cell&c){visited.push_back(c);}));
  ASSERT_FALSE(visited.empty());
  for (const auto&c:visited) {EXPECT_GE(c.y(),-25);EXPECT_LE(c.x(),25);}
  EXPECT_GT(budget,1900U);
}

TEST(VoxelCollision, ExactCenterAvoidsQueryVoxelDoubleExpansionButKeepsContacts) {
  const Point center(.001,.04,.04);
  const Cell outside(4,0,0), touching(3,0,0);
  EXPECT_FALSE(scan_planner::cylinderIntersectsVoxelXY(center,outside,.08,.29));
  EXPECT_TRUE(scan_planner::cylinderIntersectsVoxelXY(center,touching,.08,.29));
  EXPECT_TRUE(scan_planner::cylinderIntersectsVoxelXY(Point(.03,.04,.04),outside,.08,.29));
  const auto kernel=scan_planner::conservativeCylinderInflation(.08,.29,.45,.45);
  const auto span=scan_planner::verticalVoxelSpan(center.z(),.45,.45,.08);
  for (int state:{1,2,-1}) {
    const auto query=[&](const Cell &c){return c==outside ? state:0;};
    EXPECT_EQ(scan_planner::observedCylinderStatusAtHeight(Cell(0,0,0),span,kernel,query),state);
    EXPECT_EQ(scan_planner::observedCylinderStatusAtPosition(center,Cell(0,0,0),span,kernel,.08,.29,query),0);
    const auto inside=[&](const Cell &c){return c==touching ? state:0;};
    EXPECT_EQ(scan_planner::observedCylinderStatusAtPosition(center,Cell(0,0,0),span,kernel,.08,.29,inside),state);
  }
}

TEST(VoxelCollision, ExactQueryCacheDoesNotReuseDifferentSubcellXY) {
  scan_planner::VoxelStatusCache cache(8);
  int calls=0;
  EXPECT_EQ(cache.getExact(1,-2,2,.001,.04,[&](){++calls;return 0;}),0);
  EXPECT_EQ(cache.getExact(1,-2,2,.03,.04,[&](){++calls;return 1;}),1);
  EXPECT_EQ(cache.getExact(1,-2,2,.001,.04,[&](){++calls;return 2;}),0);
  EXPECT_EQ(calls,2);
  cache.clear();
  EXPECT_EQ(cache.getExact(1,-2,2,.001,.04,[&](){++calls;return 2;}),2);
}

TEST(VoxelCollision, NonfiniteExactGeometryCannotSkipAllVoxelsAsFree) {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(scan_planner::cylinderIntersectsVoxelXY(Point(nan,0,0),Cell::Zero(),.08,.29),std::invalid_argument);
  EXPECT_THROW(scan_planner::cylinderIntersectsVoxelXY(Point::Zero(),Cell::Zero(),0.,.29),std::invalid_argument);
  EXPECT_THROW(scan_planner::cylinderIntersectsVoxelXY(Point::Zero(),Cell::Zero(),.08,nan),std::invalid_argument);
}

TEST(VoxelCollision, UnknownCannotHideOccupiedInEndpointDiagnostics) {
  scan_planner::CollisionEvidence evidence;
  evidence.counts[2]=100;
  EXPECT_EQ(evidence.state(),2);
  evidence.counts[1]=1;
  EXPECT_EQ(evidence.state(),1);
}

TEST(VoxelCollision, LiveSnapshotGroundVoxelStillBlocksRatherThanBeingSilentlyCleared) {
  // 2026-09-26 16:43 passive live snapshot; observed ground at z=-.46
  // occupies [-.48,-.40]. The body bottom is -.42566: the full voxel DOES
  // intersect even though its actual measured hit lies below the envelope.
  const Point body(-.27866238,1.5908151,.0243375);
  const Point center=body-.2*Point(std::cos(1.318),std::sin(1.318),0.);
  const Cell ground(-8,15,-6);
  const auto span=scan_planner::verticalVoxelSpan(center.z(),.45,.45,.08);
  EXPECT_LE(span.low,ground.z()); EXPECT_GE(span.high,ground.z());
  EXPECT_TRUE(scan_planner::cylinderIntersectsVoxelXY(center,ground,.08,.29));
  const auto kernel=scan_planner::conservativeCylinderInflation(.08,.29,.45,.45);
  const Cell cell=(center/.08).array().floor().cast<int>();
  EXPECT_EQ(scan_planner::observedCylinderStatusAtPosition(center,cell,span,kernel,.08,.29,
      [&](const Cell &c){return c==ground ? 1:0;}),1);
}
