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
