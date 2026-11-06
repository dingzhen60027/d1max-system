#include <gtest/gtest.h>
#include <plan_manage/stair_riser_contact.hpp>

namespace {
using namespace scan_planner;
using Source=d1max_planning_interfaces::msg::SupportReference;
Source steps(double rise=.17,double gap=0.,double yaw=0.) {
  Source s;s.support_reference_id="two-observed-treads";s.support_hash=std::string(64,'a');
  s.support_map_sha256=std::string(64,'b');s.frame_id="odom";s.floor_id="stair_lower";
  s.segment_kind="stair_up";s.required_mode="stair";s.version.segment_id="flight";
  s.reason="original_stair_surface_returns_associated";s.verified=true;
  s.support_xy_radius_m=.12;s.body_reference_height_m=.5;
  for(int level=0;level<2;++level)for(int x=0;x<9;++x)for(int y=-10;y<=10;++y) {
    const double longitudinal=(level?1.:-1.)*(gap*.5+x*.02),lateral=y*.02;
    geometry_msgs::msg::Point p;p.x=std::cos(yaw)*longitudinal-std::sin(yaw)*lateral;
    p.y=std::sin(yaw)*longitudinal+std::cos(yaw)*lateral;p.z=level*rise;
    s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
  }
  return s;
}
std::shared_ptr<const PlanningSupport> support(Source s=steps()) {
  return std::make_shared<const PlanningSupport>(s);
}
void regions(Source& s,const std::string& upper_kind,const std::string& upper_mode="stair") {
  s.reason="original_transition_surface_returns_associated";
  d1max_planning_interfaces::msg::SupportRegion lower,upper;
  lower.segment_id="flight";lower.floor_id="stair_lower";lower.segment_kind="stair_up";
  lower.required_mode="stair";lower.begin_index=0;lower.end_index=s.support_ground_xyz.size()/2;
  upper=lower;upper.segment_id="another";upper.floor_id=upper_mode=="stair"?"stair_upper":"floor2";
  upper.segment_kind=upper_kind;upper.required_mode=upper_mode;
  upper.begin_index=lower.end_index;upper.end_index=s.support_ground_xyz.size();s.regions={lower,upper};
  if(upper_mode!="stair")s.stair_contact_indices.resize(lower.end_index);
}
void originalRiser(Source& s,double face=0.,double yaw=0.,double bottom=0.,double top=.17) {
  for(int y=-10;y<=10;++y)for(int z=0;z<=17;++z) {
    geometry_msgs::msg::Point p;const double side=y*.02;
    p.x=std::cos(yaw)*face-std::sin(yaw)*side;p.y=std::sin(yaw)*face+std::cos(yaw)*side;
    p.z=bottom+(top-bottom)*z/17.;s.stair_riser_xyz.push_back(p);s.stair_riser_owner.push_back(0);
  }
}
}
TEST(StairRiserContact,TwoObservedTreadEdgesExplainThinActualRiserInBothDirections) {
  for(const std::string kind:{"stair_up","stair_down"}) {
    auto s=steps();s.segment_kind=kind;
    StairRiserContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
    EXPECT_TRUE(query.matchesObservedBounds({-.0001,-.02,.06},{.0001,.02,.11}));
    EXPECT_TRUE(query.matchesObservedBounds({-.0001,-.02,0.},{.0001,.02,.04}));
    EXPECT_TRUE(query.matchesObservedBounds({-.0001,-.02,.13},{.0001,.02,.17}));
  }
}
TEST(StairRiserContact,RotatedFlightUsesMeasuredEdgesRatherThanGlobalAxis) {
  StairRiserContactQuery query(support(steps(.17,0.,.7)),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({-.00005,-.00005,.07},{.00005,.00005,.10}));
  EXPECT_FALSE(query.matchesObservedBounds({-.12,-.12,.07},{-.10,-.10,.10}));
}
TEST(StairRiserContact,NewLowBoxOnTreadAndProtrudingMixedBoundsRemainBlocked) {
  StairRiserContactQuery query(support(),"odom",.05);
  EXPECT_FALSE(query.matchesObservedBounds({-.12,-.02,.04},{-.08,.02,.05}));
  EXPECT_FALSE(query.matchesObservedBounds({.08,-.02,.21},{.12,.02,.22}));
  EXPECT_FALSE(query.matchesObservedBounds({-.02,-.02,.06},{.02,.02,.11}));
}
TEST(StairRiserContact,ObservedRiserAndTopTreadCornerUseOnlyExistingStepSolid) {
  StairRiserContactQuery query(support(),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({-.0001,-.02,.132},{.035,.02,.1701}));
  // Above the real upper surface is not part of its supporting solid.
  EXPECT_FALSE(query.matchesObservedBounds({-.0001,-.02,.132},{.035,.02,.2201}));
  EXPECT_FALSE(query.matchesObservedBounds({-.0001,-.02,.142},{.035,.02,.1801}));
  // A new thin box behind the edge but not reaching the actual top surface
  // cannot borrow this special mixed-face explanation.
  EXPECT_FALSE(query.matchesObservedBounds({-.0001,-.02,.07},{.035,.02,.11}));
  // The observed upper patch cannot be extrapolated outside its lateral hull.
  EXPECT_FALSE(query.matchesObservedBounds({-.0001,.19,.132},{.035,.23,.1701}));
}
TEST(StairRiserContact,UnequalTreadSamplingDoesNotRotateTheObservedStraightEdge) {
  auto s=steps();s.support_ground_xyz.clear();s.stair_contact_indices.clear();
  // The actual staircase source uses 4 cm longitudinal / 5 cm lateral
  // spacing. A round query neighbourhood contains unequal sample counts on
  // its two treads, so their centroid difference is NOT the edge normal.
  for(int level=0;level<2;++level)for(int x=0;x<8;++x)for(int y=-10;y<=10;++y) {
    geometry_msgs::msg::Point p;p.x=(level?.28:0.)+.04*x;p.y=.05*y;p.z=level?.34:.17;
    s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
  }
  StairRiserContactQuery query(support(s),"odom",.05);
  for(const auto& range:std::vector<std::pair<double,double>>{{-.197254902,-.151372549},
      {-.094705882,-.055882353},{0.,.04372549},{.151372549,.197254902}}) {
    EXPECT_TRUE(query.matchesObservedBounds({.28,range.first,.30294117647058827},
        {.31254901960784317,range.second,.3401960784313726}));
    EXPECT_FALSE(query.matchesObservedBounds({.28,range.first,.31294117647058827},
        {.31254901960784317,range.second,.3501960784313726}));
  }
}
TEST(StairRiserContact,WallRailAndOutsideLateralOverlapDoNotBecomeRisers) {
  StairRiserContactQuery query(support(),"odom",.05);
  EXPECT_FALSE(query.matchesObservedBounds({-.0002,-.02,.17},{.0002,.02,.22}));
  EXPECT_FALSE(query.matchesObservedBounds({-.0002,.25,.06},{.0002,.28,.11}));
  EXPECT_FALSE(query.matchesObservedBounds({-.0002,-.02,-.06},{.0002,.02,-.02}));
}
TEST(StairRiserContact,SingleTreadMissingLevelOrLargeGapCannotInventAStair) {
  auto s=steps();s.support_ground_xyz.resize(s.support_ground_xyz.size()/2);
  s.stair_contact_indices.resize(s.support_ground_xyz.size());
  StairRiserContactQuery one(support(s),"odom",.05);
  EXPECT_FALSE(one.matchesObservedBounds({-.0002,-.02,.06},{.0002,.02,.11}));
  StairRiserContactQuery gap(support(steps(.17,.12)),"odom",.05);
  EXPECT_FALSE(gap.matchesObservedBounds({-.0002,-.02,.06},{.0002,.02,.11}));
  StairRiserContactQuery missing_step(support(steps(.34)),"odom",.05);
  EXPECT_FALSE(missing_step.matchesObservedBounds({-.0002,-.02,.13},{.0002,.02,.17}));
}
TEST(StairRiserContact,CrossFlightLandingAndFloorBoundariesCannotBeJoined) {
  for(const std::string kind:{"stair_up","stair_down","landing"}) {
    auto s=steps();regions(s,kind);StairRiserContactQuery query(support(s),"odom",.05);
    EXPECT_FALSE(query.matchesObservedBounds({-.0002,-.02,.06},{.0002,.02,.11}));
  }
  auto s=steps();regions(s,"floor","general");StairRiserContactQuery query(support(s),"odom",.05);
  EXPECT_FALSE(query.matchesObservedBounds({-.0002,-.02,.06},{.0002,.02,.11}));
}
TEST(StairRiserContact,UnqualifiedLiteralRiserEvidenceIsNotOverriddenByTwoTreads) {
  auto s=steps();geometry_msgs::msg::Point p;p.x=0.;p.y=0.;p.z=.08;s.support_ground_xyz.push_back(p);
  StairRiserContactQuery query(support(s),"odom",.05);
  EXPECT_FALSE(query.matchesObservedBounds({-.0002,-.02,.06},{.0002,.02,.11}));
}
TEST(StairRiserContact,MalformedFrameQualificationAndBoundsRemainUnclassified) {
  StairRiserContactQuery foreign(support(),"map",.05);EXPECT_FALSE(foreign.valid());
  auto s=steps();s.stair_contact_indices.clear();StairRiserContactQuery missing(support(s),"odom",.05);
  EXPECT_FALSE(missing.valid());
  StairRiserContactQuery query(support(),"odom",.05);
  EXPECT_FALSE(query.matchesObservedBounds({NAN,0.,0.},{0.,0.,0.}));
  EXPECT_FALSE(query.matchesObservedBounds({0.,0.,.17},{0.,0.,.06}));
  EXPECT_FALSE(query.matchesObservedBounds({-.04,-.02,.06},{.04,.02,.11}));
}
TEST(StairRiserContact,TreadSidesNeedTrue2DMeasuredExtentNotVerticalLine) {
  auto s=steps();for(auto& p:s.support_ground_xyz)p.y=0.;
  StairRiserContactQuery line(support(s),"odom",.05);
  EXPECT_FALSE(line.matchesObservedBounds({-.0002,0.,.06},{.0002,0.,.11}));
  s=steps();for(auto& p:s.support_ground_xyz)p.z+=.5*p.x;
  StairRiserContactQuery steep(support(s),"odom",.05);
  EXPECT_FALSE(steep.matchesObservedBounds({-.0002,-.02,.06},{.0002,.02,.11}));
}
TEST(StairRiserContact,NonAdjacentTreadsCannotExcuseAnObjectAboveTheMiddleTread) {
  for(const bool middle_has_plane:{true,false})for(const bool middle_qualified:{true,false}) {
    auto s=steps();s.support_ground_xyz.clear();s.stair_contact_indices.clear();
    const double begin[3]={-.11,-.03,.03},spacing[3]={.02,.015,.02};
    for(int level=0;level<3;++level)for(int x=0;x<5;++x)for(int y=-8;y<=8;++y) {
      if(level==1&&!middle_has_plane&&(x!=2||y!=0))continue;
      geometry_msgs::msg::Point p;p.x=begin[level]+spacing[level]*x;p.y=y*.02;p.z=.1*level;
      if(level!=1||middle_qualified)s.stair_contact_indices.push_back(s.support_ground_xyz.size());
      s.support_ground_xyz.push_back(p);
    }
    StairRiserContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
    // The observed middle elevation blocks borrowing non-adjacent low/high
    // treads, even if that middle cluster is too sparse to fit a plane.
    EXPECT_FALSE(query.matchesObservedBounds({-.0002,-.01,.1499},{.0002,.01,.1501}));
    if(middle_has_plane&&middle_qualified) {
      EXPECT_TRUE(query.matchesObservedBounds({.0299,-.01,.1499},{.0301,.01,.1501}));
      EXPECT_TRUE(query.matchesObservedBounds({-.0301,-.01,.0499},{-.0299,.01,.0501}));
    }
  }
}
TEST(StairRiserContact,ForeignUnqualifiedMiddleSurfaceStillSeparatesObservedTreads) {
  for(const std::string kind:{"stair_up","landing","floor"}) {
    auto s=steps();s.support_ground_xyz.clear();s.stair_contact_indices.clear();
    // The same flight owns low/high points; a separately owned actual middle
    // surface is appended without contact qualification. It is still evidence
    // against inventing a single low-to-high riser across it.
    for(int level:{0,2})for(int x=0;x<5;++x)for(int y=-8;y<=8;++y) {
      geometry_msgs::msg::Point p;p.x=(level==0?-.14:.04)+.02*x;p.y=.02*y;p.z=.1*level;
      s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
    }
    const auto end=s.support_ground_xyz.size();
    for(int x=0;x<5;++x)for(int y=-8;y<=8;++y) {
      geometry_msgs::msg::Point p;p.x=-.03+.015*x;p.y=.02*y;p.z=.1;s.support_ground_xyz.push_back(p);
    }
    s.reason="original_transition_surface_returns_associated";
    d1max_planning_interfaces::msg::SupportRegion current,other;
    current.segment_id="flight";current.floor_id="stair_lower";current.segment_kind="stair_up";
    current.required_mode="stair";current.begin_index=0;current.end_index=end;
    other.segment_id="other";other.floor_id=kind=="floor"?"floor1":"stair_upper";
    other.segment_kind=kind;other.required_mode=kind=="floor"?"general":"stair";
    other.begin_index=end;other.end_index=s.support_ground_xyz.size();s.regions={current,other};
    StairRiserContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
    EXPECT_FALSE(query.matchesObservedBounds({-.0002,-.01,.1499},{.0002,.01,.1501}));
  }
}
TEST(StairRiserContact,UnqualifiedCoClusterEvidenceCannotHideHeightOrExtendAnInventedGap) {
  for(const double middle_z:{0.,.015,.020,.024}) {
    auto s=steps();s.support_ground_xyz.clear();s.stair_contact_indices.clear();
    for(int level:{0,2})for(int x=0;x<5;++x)for(int y=-8;y<=8;++y) {
      geometry_msgs::msg::Point p;p.x=(level==0?-.14:.04)+.02*x;p.y=.02*y;p.z=.1*level;
      s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
    }
    for(int x=0;x<5;++x)for(int y=-8;y<=8;++y) {
      geometry_msgs::msg::Point p;p.x=-.03+.015*x;p.y=.02*y;p.z=middle_z;s.support_ground_xyz.push_back(p);
    }
    StairRiserContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
    for(const double x:{-.015,0.,.015,.03})
      EXPECT_FALSE(query.matchesObservedBounds({x-.0002,-.01,middle_z+.0499},
          {x+.0002,.01,middle_z+.0501}));
    // The unqualified extension cannot repair the 10 cm qualified-edge gap,
    // even where the currently observed thin hit might be a real riser.
    EXPECT_FALSE(query.matchesObservedBounds({.0348,-.01,.0499},{.0352,.01,.0501}));
  }
}
TEST(StairRiserContact,UnobservedGapCannotLicenseThinNewObjectOrAnInferredRiser) {
  for(const double gap:{.0004,.001,.01,.04,.10}) {
    StairRiserContactQuery query(support(steps(.17,gap)),"odom",.05);
    ASSERT_TRUE(query.valid());
    EXPECT_FALSE(query.matchesObservedBounds({-.00005,-.01,.07},{.00005,.01,.09}))<<gap;
    EXPECT_FALSE(query.matchesObservedBounds({.0002,.0002,.0602},{.0102,.0102,.0992}))<<gap;
  }
  // Only the representation's outward quantisation, not a free-space search
  // radius, may bridge the independently observed tread fronts.
  StairRiserContactQuery quantized(support(steps(.17,.05/255.)),"odom",.05);
  EXPECT_TRUE(quantized.matchesObservedBounds({0.,-.01,.07},{0.,.01,.09}));
}
TEST(StairRiserContact,UnqualifiedGeometryCannotRepairMissingQualifiedGapOrBackingEvidence) {
  // Actual extra points may constrain the candidates but never supply the
  // missing permission required by the original qualified geometry.
  auto wide=steps(.17,.14);
  for(int x=0;x<4;++x)for(int y=-5;y<=5;++y) {
    geometry_msgs::msg::Point p;p.x=-.06+.015*x;p.y=.02*y;p.z=0.;wide.support_ground_xyz.push_back(p);
  }
  StairRiserContactQuery gap(support(wide),"odom",.05);
  EXPECT_FALSE(gap.matchesObservedBounds({-.0002,-.01,.07},{.0002,.01,.09}));
  auto shallow=steps();shallow.support_ground_xyz.clear();shallow.stair_contact_indices.clear();
  for(int level=0;level<2;++level)for(int x=0;x<3;++x)for(int y=-8;y<=8;++y) {
    geometry_msgs::msg::Point p;p.x=(level?1.:-1.)*(.02+.01*x);p.y=.02*y;p.z=.17*level;
    shallow.stair_contact_indices.push_back(shallow.support_ground_xyz.size());shallow.support_ground_xyz.push_back(p);
  }
  for(int level=0;level<2;++level)for(int y=-8;y<=8;++y) {
    geometry_msgs::msg::Point p;p.x=level?.14:-.14;p.y=.02*y;p.z=.17*level;
    shallow.support_ground_xyz.push_back(p);
  }
  StairRiserContactQuery backing(support(shallow),"odom",.05);
  EXPECT_FALSE(backing.matchesObservedBounds({-.0002,-.01,.07},{.0002,.01,.09}));
}

TEST(StairRiserContact,IndependentOriginalRiserCompletesMeasuredGapWithoutChangingGroundFit) {
  for(const double yaw:{0.,.7,-.7,3.141592653589793})for(const std::string kind:{"stair_up","stair_down"}) {
    auto s=steps(.17,.04,yaw);s.segment_kind=kind;const auto before=support(s);
    originalRiser(s,0.,yaw);const auto after=support(s);ASSERT_TRUE(after->valid);
    EXPECT_EQ(after->source.support_ground_xyz,before->source.support_ground_xyz);
    const auto a=before->terrainFit({0.,0.,.585}),b=after->terrainFit({0.,0.,.585});
    EXPECT_EQ(a.valid,b.valid);EXPECT_EQ(a.samples,b.samples);EXPECT_EQ(a.normal,b.normal);
    StairRiserContactQuery query(after,"odom",.05);
    EXPECT_TRUE(query.matchesObservedBounds({-.00005,-.00005,.07},{.00005,.00005,.09}))<<yaw<<" "<<kind;
  }
  auto s=steps(.17,.04);originalRiser(s);
  StairRiserContactQuery query(support(s),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({0.,-.02,.132},{.035,.02,.1701}));
  EXPECT_FALSE(query.matchesObservedBounds({0.,-.02,.132},{.035,.02,.2201}));
}

TEST(StairRiserContact,CurrentShortObjectOrWholeHeightPlateCannotCertifyItself) {
  auto s=steps(.17,.04);
  StairRiserContactQuery absent(support(s),"odom",.05);
  EXPECT_FALSE(absent.matchesObservedBounds({-.00005,-.005,.06},{.00005,.005,.10}));
  for(int z=0;z<4;++z)
    EXPECT_FALSE(absent.matchesObservedBounds({-.00005,-.005,z*.04},{.00005,.005,z*.04+.04}));
  // The actual original riser is at the upper edge x=.02, not where the new
  // current plate stands at x=0. Whole-height current rays are still no source.
  originalRiser(s,.02);StairRiserContactQuery different(support(s),"odom",.05);
  EXPECT_FALSE(different.matchesObservedBounds({-.00005,-.005,.06},{.00005,.005,.10}));
  for(int z=0;z<4;++z)
    EXPECT_FALSE(different.matchesObservedBounds({-.00005,-.005,z*.04},{.00005,.005,z*.04+.04}));
  EXPECT_TRUE(different.matchesObservedBounds({.01995,-.005,.06},{.02005,.005,.10}));
  // Coherence tolerance for fitting the original plane is not unobserved
  // space in front of it. Even a 5 mm new plate differs from exact source.
  auto exact=steps(.17,.04);originalRiser(exact);
  StairRiserContactQuery narrow(support(exact),"odom",.05);
  for(const double x:{.005,.01})
    EXPECT_FALSE(narrow.matchesObservedBounds({x-.00005,-.005,.06},{x+.00005,.005,.10}));
}

TEST(StairRiserContact,OriginalFaceNeedsFullHeightWidthAndCorrectOwner) {
  auto partial=steps(.17,.04);originalRiser(partial,0.,0.,.05,.12);
  StairRiserContactQuery short_face(support(partial),"odom",.05);
  EXPECT_FALSE(short_face.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
  auto hole=steps(.17,.04);originalRiser(hole);
  for(std::size_t i=hole.stair_riser_xyz.size();i-->0;)if(std::abs(hole.stair_riser_xyz[i].y)<.06) {
    hole.stair_riser_xyz.erase(hole.stair_riser_xyz.begin()+i);hole.stair_riser_owner.erase(hole.stair_riser_owner.begin()+i);
  }
  StairRiserContactQuery missing(support(hole),"odom",.05);
  EXPECT_FALSE(missing.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
  auto cross=steps(.17,.04);regions(cross,"stair_up");originalRiser(cross);
  StairRiserContactQuery other(support(cross),"odom",.05);
  EXPECT_FALSE(other.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
}

TEST(StairRiserContact,OriginalWallCannotBeCroppedToAnIndependentStepFace) {
  auto s=steps(.17,.04);originalRiser(s);
  StairRiserContactQuery step(support(s),"odom",.05);
  ASSERT_TRUE(step.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
  for(int y=-3;y<=3;++y)for(int z=19;z<=29;++z) {
    geometry_msgs::msg::Point p;p.x=0.;p.y=y*.02;p.z=z*.01;
    s.stair_riser_xyz.push_back(p);s.stair_riser_owner.push_back(0);
  }
  StairRiserContactQuery wall(support(s),"odom",.05);
  EXPECT_FALSE(wall.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
  // A different, farther original face is not the local riser contradiction.
  for(std::size_t i=21*18;i<s.stair_riser_xyz.size();++i)s.stair_riser_xyz[i].x=.15;
  StairRiserContactQuery other_wall(support(s),"odom",.05);
  EXPECT_TRUE(other_wall.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
}

TEST(StairRiserContact,OriginalCandidatesCanContainBothHorizontalTreadsAndTheVerticalFace) {
  auto s=steps(.17,.04);originalRiser(s);
  // Original candidates are a literal scene subset, not pre-classified riser
  // points. Tread endpoints do not lie on the vertical face and must not be
  // fitted into it; the actual independently sampled top/bottom edge remains.
  for(int level=0;level<2;++level)for(int x=0;x<=4;++x)for(int y=-10;y<=10;++y) {
    geometry_msgs::msg::Point p;p.x=(level?1.:-1.)*.005*x;p.y=.02*y;p.z=.17*level;
    s.stair_riser_xyz.push_back(p);s.stair_riser_owner.push_back(0);
  }
  StairRiserContactQuery query(support(s),"odom",.05);
  EXPECT_TRUE(query.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
  EXPECT_TRUE(query.matchesObservedBounds({0.,-.02,.132},{.035,.02,.1701}));
  EXPECT_FALSE(query.matchesObservedBounds({.00495,-.005,.07},{.00505,.005,.09}));
}

TEST(StairRiserContact,FloatPcdEndpointsUseOnlyExistingQuantisationNotMissingCoverage) {
  auto s=steps(.17,.04);originalRiser(s);
  for(auto& p:s.stair_riser_xyz) {p.x=static_cast<float>(p.x);p.y=static_cast<float>(p.y);p.z=static_cast<float>(p.z);}
  StairRiserContactQuery stored(support(s),"odom",.05);
  EXPECT_TRUE(stored.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
  // A genuinely missing bottom strip, rather than scalar representation,
  // cannot be filled by the endpoint-alignment rule.
  for(std::size_t i=s.stair_riser_xyz.size();i-->0;)if(s.stair_riser_xyz[i].z<.001) {
    s.stair_riser_xyz.erase(s.stair_riser_xyz.begin()+i);s.stair_riser_owner.erase(s.stair_riser_owner.begin()+i);
  }
  StairRiserContactQuery gap(support(s),"odom",.05);
  EXPECT_FALSE(gap.matchesObservedBounds({-.00005,-.005,.07},{.00005,.005,.09}));
}

TEST(StairRiserSourceContract,MalformedIndependentObservationsAreRejected) {
  auto good=steps();originalRiser(good);ASSERT_TRUE(support(good)->valid);
  auto s=good;s.stair_riser_owner.pop_back();EXPECT_FALSE(support(s)->valid);
  s=good;s.stair_riser_owner[0]=UINT32_MAX;EXPECT_FALSE(support(s)->valid);
  s=good;s.stair_riser_xyz[0].x=NAN;EXPECT_FALSE(support(s)->valid);
  s=good;s.stair_riser_xyz[0].z=100001.;EXPECT_FALSE(support(s)->valid);
  s=good;s.segment_kind="landing";EXPECT_FALSE(support(s)->valid);
  s=good;s.required_mode="general";EXPECT_FALSE(support(s)->valid);
  s=good;s.stair_riser_xyz.resize(20001);s.stair_riser_owner.resize(20001);EXPECT_FALSE(support(s)->valid);
  s=good;regions(s,"landing");s.stair_riser_owner.assign(s.stair_riser_xyz.size(),1);EXPECT_FALSE(support(s)->valid);
}

TEST(StairRiserContact,CommonEdgeRequiresActualLocalFrontAndNotHorizontalBacking) {
  for(const bool hole:{false,true})for(const double yaw:{0.,.7,-.7,3.141592653589793})
      for(const std::string kind:{"stair_up","stair_down"}) {
    auto s=steps();s.support_ground_xyz.clear();s.stair_contact_indices.clear();s.segment_kind=kind;
    for(int level=0;level<2;++level)for(int x=0;x<9;++x)for(int y=-10;y<=10;++y) {
      geometry_msgs::msg::Point p;const double front=hole&&std::abs(y)<=2?.01:0.;
      const double n=level?front+.02*x:-.02*x,side=.02*y;
      p.x=std::cos(yaw)*n-std::sin(yaw)*side;p.y=std::sin(yaw)*n+std::cos(yaw)*side;p.z=.17*level;
      s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
    }
    StairRiserContactQuery query(support(s),"odom",.05);ASSERT_TRUE(query.valid());
    EXPECT_EQ(query.matchesObservedBounds({-.00005,-.00005,.07},{.00005,.00005,.09}),!hole)<<yaw<<" "<<kind;
    if(yaw==0.) {
      EXPECT_EQ(query.matchesObservedBounds({-.0001,-.005,.132},{.035,.005,.1701}),!hole);
      EXPECT_FALSE(query.matchesObservedBounds({-.0001,-.005,.132},{.035,.005,.2201}));
    }
  }
}
