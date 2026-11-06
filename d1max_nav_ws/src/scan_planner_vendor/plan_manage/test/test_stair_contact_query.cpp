#include <gtest/gtest.h>
#include <plan_manage/stair_contact_query.hpp>

namespace {
using namespace scan_planner;
using Vec=Eigen::Vector3d;
using Source=d1max_planning_interfaces::msg::SupportReference;
Source source() {
  Source s;s.support_reference_id="literal-stairs";s.support_hash=std::string(64,'a');
  s.support_map_sha256=std::string(64,'b');s.frame_id="odom";
  s.floor_id="stair_lower";s.segment_kind="stair_up";s.required_mode="stair";
  s.reason="original_stair_surface_returns_associated";s.verified=true;
  s.support_xy_radius_m=.12;s.body_reference_height_m=.50;s.version.segment_id="lower";
  for(const auto& p:{Vec(.005,.005,.001),Vec(.055,.005,.001),Vec(.105,.005,.171),
                    Vec(.155,.005,.171),Vec(-.005,-.005,-.001)}) {
    geometry_msgs::msg::Point q;q.x=p.x();q.y=p.y();q.z=p.z();s.support_ground_xyz.push_back(q);
  }
  s.stair_contact_indices={0,1,2,3,4};
  return s;
}
std::shared_ptr<const PlanningSupport> support(Source s=source()) {
  return std::make_shared<const PlanningSupport>(s);
}
Source mixed() {
  auto s=source();s.version.segment_id="floor";s.floor_id="floor1";
  s.segment_kind="floor";s.required_mode="general";
  s.reason="original_transition_surface_returns_associated";
  d1max_planning_interfaces::msg::SupportRegion floor,stair;
  floor.segment_id="floor";floor.floor_id="floor1";floor.segment_kind="floor";
  floor.required_mode="general";floor.begin_index=0;floor.end_index=1;
  stair.segment_id="lower";stair.floor_id="stair_lower";stair.segment_kind="stair_up";
  stair.required_mode="stair";stair.begin_index=1;stair.end_index=s.support_ground_xyz.size();
  s.regions={floor,stair};s.stair_contact_indices={1,2,3,4};return s;
}
Source tread(double height=0.,double grade_x=0.,double grade_y=0.) {
  auto s=source();s.support_ground_xyz.clear();s.stair_contact_indices.clear();
  for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y) {
    geometry_msgs::msg::Point p;p.x=x*.02;p.y=y*.02;
    p.z=height+grade_x*p.x+grade_y*p.y;
    s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
  }
  return s;
}
}

TEST(StairContactQuery,OnlyLiteralReturnsMatchWithHalfFineCell3DBound) {
  StairContactQuery query(support(),"odom",.05);ASSERT_TRUE(query.valid());
  EXPECT_DOUBLE_EQ(query.matchRadius(),.0125);
  EXPECT_EQ(query.sourcePointCount(),5U);
  EXPECT_TRUE(query.matchesObservedEndpoint({.005,.005,.001}));
  EXPECT_TRUE(query.matchesObservedEndpoint({.0174,.005,.001}));
  EXPECT_FALSE(query.matchesObservedEndpoint({.0176,.005,.001}));
  EXPECT_FALSE(query.matchesObservedEndpoint({.017,.017,.001}));
  EXPECT_FALSE(query.matchesObservedEndpoint({.005,.005,.026}));
  EXPECT_FALSE(query.matchesObservedEndpoint({.005,.005,.501}));
}
TEST(StairContactQuery,DoesNotInventRisersOrFillGapsFromAHeightPlane) {
  StairContactQuery query(support(),"odom",.05);
  EXPECT_TRUE(query.matchesObservedEndpoint({.105,.005,.171}));
  EXPECT_FALSE(query.matchesObservedEndpoint({.105,.005,.085}));
  EXPECT_FALSE(query.matchesObservedEndpoint({.080,.005,.086}));
  // The coarse route association radius is 12 cm, never this classifier's.
  EXPECT_FALSE(query.matchesObservedEndpoint({.105,.105,.171}));
}
TEST(StairContactQuery,SameSubcellMixedBoxHitOverridesMatchingTread) {
  StairContactQuery query(support(),"odom",.05);
  const Vec tread(.005,.005,.001),box(.005,.005,.022);
  ASSERT_EQ(hitSubvoxelBit(tread,{0,0,0},.05),hitSubvoxelBit(box,{0,0,0},.05));
  const auto clear=query.classifyObservedEndpoints({0,0,0},{tread});
  EXPECT_EQ(clear.observed,1);EXPECT_EQ(clear.contactMask(),1);
  const auto mixed=query.classifyObservedEndpoints({0,0,0},{tread,box});
  EXPECT_EQ(mixed.observed,1);EXPECT_EQ(mixed.matched,1);EXPECT_EQ(mixed.unmatched,1);
  EXPECT_EQ(mixed.contactMask(),0);
  const auto reverse=query.classifyObservedEndpoints({0,0,0},{box,tread});
  EXPECT_EQ(reverse.contactMask(),0);
}
TEST(StairContactQuery,SeparateOccupiedOctantsAreNotExcusedByOneSupportPoint) {
  StairContactQuery query(support(),"odom",.05);
  const auto result=query.classifyObservedEndpoints({0,0,0},{{.005,.005,.001},{.005,.005,.03}});
  EXPECT_EQ(result.observed,3);EXPECT_EQ(result.matched,1);EXPECT_EQ(result.unmatched,2);
  EXPECT_EQ(result.contactMask(),1);
}
TEST(StairContactQuery,FloorOwnershipAndSameXYOtherFloorStayOrdinaryObstacles) {
  auto s=mixed();auto p=s.support_ground_xyz[1];p.z=.501;s.support_ground_xyz[0]=p;
  StairContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
  EXPECT_TRUE(query.matchesObservedEndpoint({.055,.005,.001}));
  EXPECT_FALSE(query.matchesObservedEndpoint({.055,.005,.501}));
  // Exact coordinate tie with a floor point cannot become stair contact.
  s.support_ground_xyz[0]=s.support_ground_xyz[1];
  StairContactQuery tied(support(s),"odom",.05);ASSERT_TRUE(tied.valid());
  EXPECT_FALSE(tied.matchesObservedEndpoint({.055,.005,.001}));
}
TEST(StairContactQuery,NoStairSourceForeignFrameAndMalformedPacketsDoNotClassify) {
  StairContactQuery absent({},"odom",.05);EXPECT_FALSE(absent.valid());
  EXPECT_FALSE(absent.matchesObservedEndpoint({.005,.005,.001}));
  StairContactQuery foreign(support(),"map",.05);EXPECT_FALSE(foreign.valid());
  for(const double resolution:{0.,-.05,static_cast<double>(NAN),1.1}) {
    StairContactQuery invalid(support(),"odom",resolution);EXPECT_FALSE(invalid.valid());
  }
  auto s=source();s.verified=false;
  StairContactQuery unverified(support(s),"odom",.05);EXPECT_FALSE(unverified.valid());
  s=source();s.frame_id="";
  StairContactQuery no_frame(support(s),"",.05);EXPECT_FALSE(no_frame.valid());
}
TEST(StairContactQuery,ExplicitQualifiedIndicesAreRequiredAndMalformedSetsDisableContact) {
  auto s=source();s.stair_contact_indices.clear();
  StairContactQuery missing(support(s),"odom",.05);EXPECT_FALSE(missing.valid());
  s=source();s.stair_contact_indices={1};
  StairContactQuery subset(support(s),"odom",.05);ASSERT_TRUE(subset.valid());
  EXPECT_EQ(subset.qualifiedPointCount(),1U);
  EXPECT_FALSE(subset.matchesObservedEndpoint({.005,.005,.001}));
  EXPECT_TRUE(subset.matchesObservedEndpoint({.055,.005,.001}));
  for(const std::vector<std::uint32_t>& indices: {std::vector<std::uint32_t>{0,0},{0,99},{1,0}}) {
    s=source();s.stair_contact_indices=indices;
    StairContactQuery malformed(support(s),"odom",.05);EXPECT_FALSE(malformed.valid());
  }
  s=mixed();s.stair_contact_indices={0,1};
  StairContactQuery floor_index(support(s),"odom",.05);EXPECT_FALSE(floor_index.valid());
}
TEST(StairContactQuery,LiteralBoundsNeedOneQualifiedBallContainingEveryAccumulatedHit) {
  StairContactQuery query(support(),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({.0048,.0048,.0008},{.0052,.0052,.0012}));
  EXPECT_FALSE(query.matchesObservedBounds({.0048,.0048,.0008},{.0052,.0052,.022}));
  EXPECT_FALSE(query.matchesObservedBounds({.0048,.0048,.0498},{.0052,.0052,.0502}));
  // Separate qualified endpoints do not justify a large empty space between.
  EXPECT_FALSE(query.matchesObservedBounds({.005,.005,.001},{.055,.005,.001}));
  auto s=mixed();s.support_ground_xyz[0]=s.support_ground_xyz[1];
  StairContactQuery tied(support(s),"odom",.05);
  EXPECT_FALSE(tied.matchesObservedBounds({.0548,.0048,.0008},{.0552,.0052,.0012}));
  StairContactQuery foreign(support(),"map",.05);
  EXPECT_FALSE(foreign.matchesObservedBounds({.0048,.0048,.0008},{.0052,.0052,.0012}));
}
TEST(StairContactQuery,WholeQuantizedEndpointBoundsMatchObservedTreadBothDirections) {
  StairContactQuery flat(support(tread(.17)),"odom",.05);
  EXPECT_TRUE(flat.matchesObservedBounds({-.0201,-.0201,.1698},{.0201,.0201,.1702}));
  // The classifier has no travel-direction dependency. Small measured local
  // tread grade and translated heights do not turn into a stair-wide ramp.
  for(double grade:{-.08,.08}) {
    StairContactQuery slope(support(tread(-.17,grade,-grade*.5)),"odom",.05);
    EXPECT_TRUE(slope.matchesObservedBounds({-.02,-.02,-.173},{.02,.02,-.167}));
  }
}
TEST(StairContactQuery,BoundsContainingLowBoxOrMixedEndpointsRemainOccupied) {
  StairContactQuery query(support(tread()),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({-.02,-.02,-.0002},{.02,.02,.0002}));
  EXPECT_FALSE(query.matchesObservedBounds({-.02,-.02,0.},{.02,.02,.05}));
  EXPECT_FALSE(query.matchesObservedBounds({-.02,-.02,.0498},{.02,.02,.0502}));
  EXPECT_FALSE(query.matchesObservedBounds({-.02,-.02,0.},{.02,.02,.022}));
  // A single matching low endpoint cannot excuse the accumulated box extent.
  EXPECT_FALSE(query.matchesObservedBounds({0.,0.,0.},{.001,.001,.0499}));
}
TEST(StairContactQuery,NoExtrapolationOverTreadEdgeOrAcrossMeasuredSupportHole) {
  auto s=tread();
  StairContactQuery edge(support(s),"odom",.05);
  EXPECT_FALSE(edge.matchesObservedBounds({.085,-.02,-.0001},{.115,.02,.0001}));
  s.support_ground_xyz.erase(std::remove_if(s.support_ground_xyz.begin(),s.support_ground_xyz.end(),
    [](const auto& p){return std::hypot(p.x,p.y)<.045;}),s.support_ground_xyz.end());
  s.stair_contact_indices.clear();for(std::size_t i=0;i<s.support_ground_xyz.size();++i)
    s.stair_contact_indices.push_back(i);
  StairContactQuery hole(support(s),"odom",.05);
  EXPECT_FALSE(hole.matchesObservedBounds({-.005,-.005,-.0001},{.005,.005,.0001}));
}
TEST(StairContactQuery,ObservedWallRiserAndSparseLineCannotBecomeTreadPlane) {
  auto s=tread();for(auto& p:s.support_ground_xyz)std::swap(p.x,p.z);
  StairContactQuery wall(support(s),"odom",.05);
  EXPECT_FALSE(wall.matchesObservedBounds({-.001,-.02,-.02},{.001,.02,.02}));
  s=tread(0.,.5);
  StairContactQuery steep(support(s),"odom",.05);
  EXPECT_FALSE(steep.matchesObservedBounds({-.01,-.01,-.005},{.01,.01,.005}));
  s=tread();for(auto& p:s.support_ground_xyz)p.y=0.;
  StairContactQuery line(support(s),"odom",.05);
  EXPECT_FALSE(line.matchesObservedBounds({-.025,0.,0.},{.025,0.,0.}));
}
TEST(StairContactQuery,OtherLayerAndUnobservedRiserDoNotBorrowTreadEvidence) {
  auto s=tread();const auto second=tread(.17);
  const auto count=s.support_ground_xyz.size();
  s.support_ground_xyz.insert(s.support_ground_xyz.end(),second.support_ground_xyz.begin(),second.support_ground_xyz.end());
  for(std::size_t i=0;i<second.support_ground_xyz.size();++i)s.stair_contact_indices.push_back(count+i);
  StairContactQuery query(support(s),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({-.01,-.01,-.0001},{.01,.01,.0001}));
  EXPECT_TRUE(query.matchesObservedBounds({-.01,-.01,.1699},{.01,.01,.1701}));
  EXPECT_FALSE(query.matchesObservedBounds({-.01,-.01,.08},{.01,.01,.09}));
  EXPECT_FALSE(query.matchesObservedBounds({-.01,-.01,.49},{.01,.01,.50}));
}
TEST(StairContactQuery,NearbyForeignRegionCannotBorrowQualifiedPatch) {
  auto s=tread();s.reason="original_transition_surface_returns_associated";
  d1max_planning_interfaces::msg::SupportRegion lower,landing;
  lower.segment_id="lower";lower.floor_id="stair_lower";lower.segment_kind="stair_up";
  lower.required_mode="stair";lower.begin_index=0;lower.end_index=s.support_ground_xyz.size();
  landing=lower;landing.segment_id="platform";landing.segment_kind="landing";
  landing.begin_index=lower.end_index;landing.end_index=landing.begin_index+1;
  geometry_msgs::msg::Point p;p.x=.001;p.y=.001;p.z=0.;s.support_ground_xyz.push_back(p);
  s.regions={lower,landing};
  StairContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
  EXPECT_FALSE(query.matchesObservedBounds({-.01,-.01,-.0001},{.01,.01,.0001}));
}
TEST(StairContactQuery,NoisyOrOversizedIncompleteBoundsFailConservatively) {
  auto s=tread();for(std::size_t i=0;i<s.support_ground_xyz.size();++i)
    s.support_ground_xyz[i].z=(i%2?.009:-.009);
  StairContactQuery noisy(support(s),"odom",.05);
  EXPECT_FALSE(noisy.matchesObservedBounds({-.02,-.02,-.0001},{.02,.02,.0001}));
  StairContactQuery query(support(tread()),"odom",.05);
  EXPECT_FALSE(query.matchesObservedBounds({-.03,-.01,0.},{.03,.01,0.}));
  EXPECT_FALSE(query.matchesObservedBounds({.01,0.,0.},{-.01,0.,0.}));
  EXPECT_FALSE(query.matchesObservedBounds({NAN,0.,0.},{0.,0.,0.}));
  EXPECT_FALSE(query.matchesObservedBounds({0.,0.,0.},{0.,0.,INFINITY}));
}
TEST(StairContactQuery,PlaneCannotOverrideCloserUnqualifiedLiteralInSameRegion) {
  auto s=tread();geometry_msgs::msg::Point unqualified;
  unqualified.x=.001;unqualified.y=.001;unqualified.z=.010;
  s.support_ground_xyz.push_back(unqualified);
  StairContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
  EXPECT_FALSE(query.matchesObservedEndpoint({.001,.001,.01}));
  EXPECT_FALSE(query.matchesObservedBounds({.0008,.0008,.0098},{.0012,.0012,.0102}));
  // An unrelated unqualified source outside the tiny actual contact footprint
  // does not remove the observed tread's independent interpolation evidence.
  EXPECT_TRUE(query.matchesObservedBounds({-.04,-.04,-.0001},{-.02,-.02,.0001}));
}
TEST(StairContactQuery,WholeBoundsVetoKnownUnqualifiedInteriorAndBoundaryBeforeInterpolation) {
  for(const Vec& point:{Vec(.0125,.0125,.005),Vec(.0125,.0125,0.)}) {
    auto s=tread();geometry_msgs::msg::Point p;p.x=point.x();p.y=point.y();p.z=point.z();
    s.support_ground_xyz.push_back(p);
    StairContactQuery query(support(s),"odom",.05);
    EXPECT_FALSE(query.matchesObservedBounds({0.,0.,0.},{.05,.05,.01}));
  }
  // Eight qualified corners make the literal single-ball branch possible;
  // it must not bypass a ninth explicitly unqualified interior return.
  auto s=source();s.support_ground_xyz.clear();s.stair_contact_indices.clear();
  for(int corner=0;corner<8;++corner) {
    geometry_msgs::msg::Point p;p.x=corner&4?.002:0.;p.y=corner&2?.002:0.;p.z=corner&1?.002:0.;
    s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
  }
  StairContactQuery original(support(s),"odom",.05);
  ASSERT_TRUE(original.matchesObservedBounds({0.,0.,0.},{.002,.002,.002}));
  geometry_msgs::msg::Point p;p.x=.001;p.y=.001;p.z=.001;s.support_ground_xyz.push_back(p);
  StairContactQuery contradictory(support(s),"odom",.05);
  EXPECT_FALSE(contradictory.matchesObservedBounds({0.,0.,0.},{.002,.002,.002}));
}
TEST(StairContactQuery,MeasuredTrianglesCoverLegitimateRectangularSamplingButNotHoles) {
  std::vector<Eigen::Vector2d> points;
  for(int x=-3;x<=3;++x)for(int y=-3;y<=3;++y)points.emplace_back(x*.04,y*.05);
  EXPECT_TRUE(stairObservedRectangleCovered(points,{-.024,-.024},{.024,.024},.1));
  auto hole=points;
  hole.erase(std::remove_if(hole.begin(),hole.end(),[](const auto& p){return p.norm()<.061;}),hole.end());
  EXPECT_FALSE(stairObservedRectangleCovered(hole,{-.024,-.024},{.024,.024},.1));
  EXPECT_FALSE(stairObservedRectangleCovered(points,{.10,-.024},{.15,.024},.1));
  auto s=tread();s.support_ground_xyz.clear();s.stair_contact_indices.clear();
  for(const auto& xy:points) {geometry_msgs::msg::Point p;p.x=xy.x();p.y=xy.y();p.z=.34;
    s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);}
  StairContactQuery query(support(s),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({-.024,-.024,.3398},{.024,.024,.3402}));
}
TEST(StairContactQuery,NegativeLatticeBoundariesAreExactAndInvalidHitHistoryFailsClosed) {
  StairContactQuery query(support(),"odom",.05);
  const auto negative=query.classifyObservedEndpoints({-1,-1,-1},{{-.005,-.005,-.001}});
  EXPECT_TRUE(negative.complete);EXPECT_EQ(negative.contactMask(),128);
  EXPECT_EQ(query.classifyObservedEndpoints({0,0,0},{}).contactMask(),0);
  EXPECT_EQ(query.classifyObservedEndpoints({0,0,0},{{.005,.005,.001},{NAN,0.,0.}}).contactMask(),0);
  EXPECT_EQ(query.classifyObservedEndpoints({1,0,0},{{.005,.005,.001}}).contactMask(),0);
  EXPECT_FALSE(query.matchesObservedEndpoint({INFINITY,0.,0.}));
}
TEST(StairContactQuery,IndependentNewSourceCannotInheritPreviousTaskContact) {
  const auto first=support();StairContactQuery old(first,"odom",.05);
  auto next=source();next.version.task_id="new-task";
  for(auto& p:next.support_ground_xyz)p.z+=1.;
  StairContactQuery newer(support(next),"odom",.05);
  EXPECT_TRUE(old.matchesObservedEndpoint({.005,.005,.001}));
  EXPECT_FALSE(newer.matchesObservedEndpoint({.005,.005,.001}));
  EXPECT_TRUE(newer.matchesObservedEndpoint({.005,.005,1.001}));
}
