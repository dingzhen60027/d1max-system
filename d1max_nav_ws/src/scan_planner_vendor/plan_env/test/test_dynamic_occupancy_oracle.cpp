#include <gtest/gtest.h>
#include <plan_env/dynamic_occupancy_oracle.hpp>

namespace {
using Oracle=scan_planner::DynamicOccupancyOracle;
using Json=nlohmann::json;
class DynamicOracleTest:public ::testing::Test {
 protected:
  Oracle oracle;
  Oracle::Context context{"sim-session","sim-seed",1,3};
  Json packet;
  static Json sphere(const std::array<double,3>& center,double radius,int state=2) {
    Json lo=Json::array(),hi=Json::array();
    for(const auto value:center){lo.push_back(value-radius);hi.push_back(value+radius);}
    return {{"state",state},{"min",lo},{"max",hi},{"enclosure","sphere_v1"},{"center",center},{"radius",radius}};
  }
  void SetUp() override {
    oracle.configure(std::string(64,'a'),"odom",{"person","cart"},.05);
    packet={{"schema",1},{"kind","isaac_dynamic_occupancy_oracle_v1"},
      {"session_id","sim-session"},{"seed_id","sim-seed"},{"epoch",1},{"context_sequence",3},
      {"sequence",1},{"frame_id","odom"},{"registry_sha256",std::string(64,'a')},
      {"source_stamp_ns",1000000000LL},{"valid_until_ns",1300000000LL},{"complete",true},
      {"reachable_horizon_ns",6000000000LL},{"reachable_until_ns",7000000000LL},
      {"actors",Json::array({{{"actor_id","person"},{"regions",Json::array({
        {{"state",2},{"min",{0.,0.,0.}},{"max",{.10,.10,2.}}}})}},
        {{"actor_id","cart"},{"regions",Json::array({
        {{"state",1},{"min",{2.,2.,0.}},{"max",{3.,3.,1.}}}})}}})}};
  }
};
TEST_F(DynamicOracleTest, ClosedVoxelContactVetoesEvenWithoutAnyLaserHit) {
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
  EXPECT_TRUE(oracle.live(context,1100000000LL,2100000000LL));
  EXPECT_EQ(oracle.status({-1,0,0}),2); // Closed cell touches actor at x=0.
  EXPECT_EQ(oracle.status({2,0,0}),2);  // Closed cell touches x=.10.
  EXPECT_EQ(oracle.status({40,40,0}),1);
  EXPECT_EQ(oracle.status({10,10,0}),0); // Absence of a veto is NOT FREE evidence.
}
TEST_F(DynamicOracleTest, SortedActorOrdinalQueriesAllRegionsAndFullZWithoutConferringFree) {
  EXPECT_EQ(oracle.actorIds(),(std::vector<std::string>{"cart","person"}));EXPECT_EQ(oracle.actorCount(),2);
  packet["actors"][0]["regions"].push_back(sphere({.5,.5,2.},.2));
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
  EXPECT_EQ(oracle.actorStatus(1,{40,40,0}),1); // cart is first in the registry, regardless of packet ordering.
  EXPECT_EQ(oracle.actorStatus(2,{40,40,0}),0);
  EXPECT_EQ(oracle.actorStatus(2,{10,10,40}),2); // All person regions, including upper Z.
  EXPECT_EQ(oracle.actorStatus(2,{10,10,34}),0);
  EXPECT_EQ(oracle.actorStatus(0,{100,100,100}),2);EXPECT_EQ(oracle.actorStatus(3,{100,100,100}),2);
  const Oracle copy=oracle;oracle.revoke();EXPECT_EQ(oracle.actorStatus(2,{100,100,100}),2);
  EXPECT_EQ(copy.actorStatus(2,{100,100,100}),0);
  EXPECT_FALSE(copy.live(context,1200000000LL,2100000000LL)); // Per-actor geometry cannot renew the lease.
}
TEST_F(DynamicOracleTest, SphereNarrowPhaseKeepsClosedFullVoxelFacesAndRemovesOnlyCubeCorners) {
  packet["actors"][0]["regions"][0]=sphere({0.,0.,0.},1.);
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
  for(const auto& cell:std::vector<std::array<int,3>>{{20,0,0},{-21,0,0},{0,0,20},{0,0,-21},{0,0,19}})
    EXPECT_EQ(oracle.status(cell),2);
  EXPECT_EQ(oracle.status({19,19,19}),0);
  EXPECT_FALSE(oracle.intersectsColumnXY(19,19));
  EXPECT_TRUE(oracle.intersectsColumnXY(20,0));
  EXPECT_TRUE(oracle.intersectsColumnXY(0,0));
  EXPECT_EQ(oracle.status({40,40,0}),1); // Legacy actor OCC still wins.
}
TEST_F(DynamicOracleTest, NegativeCoordinatesAndFullSixSecondV31SphereKeepFutureAndElevatedVetoes) {
  packet["actors"][0]["regions"][0]=sphere({0.,-12.808781623840332,0.},7.166098359546661);
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
  EXPECT_EQ(oracle.status({-144,-113,-1}),0); // v31 reported cube corner, >10.11 m away.
  EXPECT_EQ(oracle.status({-120,-240,-1}),2);
  EXPECT_EQ(oracle.status({0,-257,140}),2); // Full upper Z is part of the same unchanged sphere.
  EXPECT_EQ(oracle.status({0,-257,145}),0);
  EXPECT_EQ(oracle.reachableUntilNs(),7000000000LL);
  EXPECT_FALSE(oracle.live(context,1200000000LL,2100000000LL)); // Reachability never renews lease.
  packet["sequence"]=2;packet["source_stamp_ns"]=1100000000LL;packet["valid_until_ns"]=1400000000LL;
  packet["reachable_horizon_ns"]=8000000001LL;packet["reachable_until_ns"]=9100000001LL;
  EXPECT_FALSE(oracle.apply(packet.dump(),context,2100000000LL));
}
TEST_F(DynamicOracleTest, MalformedOrUnboundSphereRevokesEvenWhenTheQueriedCellIsDistant) {
  const auto original=packet;
  for(int variant=0;variant<12;++variant) {
    oracle.configure(std::string(64,'a'),"odom",{"person","cart"},.05);
    packet=original;ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
    packet["sequence"]=2;packet["source_stamp_ns"]=1100000000LL;packet["valid_until_ns"]=1400000000LL;
    packet["reachable_until_ns"]=7100000000LL;
    auto& r=packet["actors"][0]["regions"][0];r=sphere({0.,0.,0.},1.);
    if(variant==0)r.erase("center");if(variant==1)r.erase("radius");
    if(variant==2)r["enclosure"]=nullptr;if(variant==3)r["enclosure"]="unproved_shape";
    if(variant==4)r["radius"]=0.;if(variant==5)r["radius"]=std::numeric_limits<double>::infinity();
    if(variant==6)r["radius"]=true;if(variant==7)r["center"]={0.,0.};
    if(variant==8)r["max"][0]=1.001;if(variant==9)r.erase("enclosure");
    if(variant==10)r["center"][0]=std::numeric_limits<double>::quiet_NaN();
    if(variant==11)r["state"]=0;
    EXPECT_FALSE(oracle.apply(packet.dump(),context,2100000000LL))<<variant;
    EXPECT_EQ(oracle.status({100,100,100}),2)<<variant;
    EXPECT_TRUE(oracle.intersectsColumnXY(100,100))<<variant;
    EXPECT_FALSE(oracle.live(context,1100000000LL,2100000000LL))<<variant;
  }
}
TEST_F(DynamicOracleTest, SphereSnapshotStillNeedsExactSourceContextAndOriginalReceiptLease) {
  packet["actors"][0]["regions"][0]=sphere({0.,0.,0.},1.);
  const auto payload=packet.dump();ASSERT_TRUE(oracle.apply(payload,context,2000000000LL));
  const Oracle copy=oracle;
  ASSERT_TRUE(oracle.apply(payload,context,2199999999LL));
  EXPECT_EQ(oracle.receiptDeadlineNs(),2200000000LL);
  EXPECT_FALSE(copy.live(context,1100000000LL,2200000000LL));
  auto wrong=context;++wrong.sequence;
  EXPECT_FALSE(copy.live(wrong,1100000000LL,2100000000LL));
  EXPECT_FALSE(copy.live(context,1200000000LL,2100000000LL));
  packet["sequence"]=2;packet["source_stamp_ns"]=1100000000LL;packet["valid_until_ns"]=1400000000LL;
  packet["reachable_until_ns"]=7100000000LL;
  packet["actors"][1]["regions"][0]=sphere({0.,0.,0.},.5,1);
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2100000000LL));
  EXPECT_EQ(oracle.status({0,0,0}),1);
  EXPECT_EQ(copy.status({0,0,0}),2);
}
TEST_F(DynamicOracleTest, BothClocksAndCopiedSnapshotKeepOriginalFiniteLease) {
  const auto payload=packet.dump();ASSERT_TRUE(oracle.apply(payload,context,2000000000LL));
  Oracle snapshot=oracle;
  EXPECT_FALSE(oracle.live(context,1200000000LL,2050000000LL));
  EXPECT_FALSE(oracle.live(context,1100000000LL,2200000000LL));
  EXPECT_FALSE(oracle.live(context,999999999LL,2050000000LL));
  EXPECT_FALSE(oracle.live(context,1100000000LL,1999999999LL));
  ASSERT_TRUE(oracle.apply(payload,context,2199999999LL));
  EXPECT_EQ(oracle.receiptDeadlineNs(),2200000000LL); // Duplicate cannot renew receipt.
  packet["sequence"]=2;packet["source_stamp_ns"]=1100000000LL;packet["valid_until_ns"]=1400000000LL;packet["reachable_until_ns"]=7100000000LL;
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2100000000LL));
  EXPECT_FALSE(snapshot.live(context,1200000000LL,2100000000LL));
  EXPECT_EQ(snapshot.sourceNs(),1000000000LL);
}
TEST_F(DynamicOracleTest, MissingNewDuplicateAndMalformedActorsRevokeWholeVolume) {
  const auto original=packet;
  for(int variant=0;variant<6;++variant) {
    packet=original;oracle.configure(std::string(64,'a'),"odom",{"person","cart"},.05);
    ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
    packet["sequence"]=2;packet["source_stamp_ns"]=1100000000LL;packet["valid_until_ns"]=1400000000LL;packet["reachable_until_ns"]=7100000000LL;
    if(variant==0)packet["actors"].erase(0);
    if(variant==1)packet["actors"][0]["actor_id"]="unregistered-newborn";
    if(variant==2)packet["actors"][0]["actor_id"]="cart";
    if(variant==3)packet["actors"][0]["regions"]=Json::array();
    if(variant==4)packet["actors"][0]["regions"][0]["state"]=0;
    if(variant==5)packet["actors"][0]["regions"][0]["min"][0]=std::numeric_limits<double>::infinity();
    EXPECT_FALSE(oracle.apply(packet.dump(),context,2100000000LL))<<variant;
    EXPECT_FALSE(oracle.live(context,1100000000LL,2100000000LL))<<variant;
    EXPECT_EQ(oracle.status({100,100,100}),2)<<variant;
  }
}
TEST_F(DynamicOracleTest, ContextGeometryAndFiniteHorizonMustMatchExactly) {
  const auto original=packet;
  for(const auto &field:{"session_id","seed_id","frame_id","registry_sha256"}) {
    packet=original;packet[field]="wrong";EXPECT_FALSE(oracle.apply(packet.dump(),context,2000000000LL));
  }
  packet=original;packet["epoch"]=2;EXPECT_FALSE(oracle.apply(packet.dump(),context,2000000000LL));
  packet=original;packet["context_sequence"]=4;EXPECT_FALSE(oracle.apply(packet.dump(),context,2000000000LL));
  packet=original;packet["valid_until_ns"]=1300000001LL;EXPECT_FALSE(oracle.apply(packet.dump(),context,2000000000LL));
  packet=original;packet["actors"][0]["regions"][0]["max"][0]=1e30;
  EXPECT_FALSE(oracle.apply(packet.dump(),context,2000000000LL));
}
TEST_F(DynamicOracleTest, RejectedOwnSequenceCannotReplayToRestoreAuthorization) {
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
  packet["sequence"]=2;packet["source_stamp_ns"]=1100000000LL;packet["valid_until_ns"]=1400000000LL;packet["reachable_until_ns"]=7100000000LL;
  const auto valid=packet;packet["actors"].erase(0);
  EXPECT_FALSE(oracle.apply(packet.dump(),context,2100000000LL));
  EXPECT_FALSE(oracle.apply(valid.dump(),context,2100000001LL));
  packet=valid;packet["sequence"]=3;packet["source_stamp_ns"]=1200000000LL;packet["valid_until_ns"]=1500000000LL;packet["reachable_until_ns"]=7200000000LL;
  EXPECT_TRUE(oracle.apply(packet.dump(),context,2200000000LL));
  auto next=context;next.epoch=2;next.sequence=4;
  packet["epoch"]=2;packet["context_sequence"]=4;packet["sequence"]=1;
  EXPECT_TRUE(oracle.apply(packet.dump(),next,2200000001LL));
  EXPECT_FALSE(oracle.live(context,1200000000LL,2200000001LL));
}
TEST_F(DynamicOracleTest, ExplicitEmptyRegistryStillRequiresFiniteSameSourceCompleteHeartbeat) {
  oracle.configure(std::string(64,'a'),"odom",{},.05);packet["actors"]=Json::array();
  EXPECT_TRUE(oracle.enabled());EXPECT_FALSE(oracle.live(context,1000000000LL,2000000000LL));
  ASSERT_TRUE(oracle.apply(packet.dump(),context,2000000000LL));
  EXPECT_TRUE(oracle.live(context,1100000000LL,2100000000LL));EXPECT_EQ(oracle.status({0,0,0}),0);
  EXPECT_FALSE(oracle.live(context,1200000000LL,2100000000LL));
  packet["sequence"]=2;packet["source_stamp_ns"]=1100000000LL;packet["valid_until_ns"]=1400000000LL;
  packet["reachable_until_ns"]=7100000000LL;packet["actors"].push_back({{"actor_id","unregistered-newborn"},{"regions",Json::array()}});
  EXPECT_FALSE(oracle.apply(packet.dump(),context,2100000000LL));EXPECT_EQ(oracle.status({0,0,0}),2);
}
} // namespace
