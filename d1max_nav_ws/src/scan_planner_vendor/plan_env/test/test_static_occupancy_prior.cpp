#include <gtest/gtest.h>
#include <plan_env/static_occupancy_prior.hpp>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {
using Json=nlohmann::json;
using Prior=scan_planner::StaticOccupancyPrior;

// The actual checked loader, without a ROS context, node or transport. The
// asymmetric shape and negative world origin expose axis/order mistakes.
class StaticOccupancyPriorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto pattern=(std::filesystem::temp_directory_path()/"d1max-static-prior-XXXXXX").string();
    std::vector<char> writable(pattern.begin(),pattern.end());writable.push_back('\0');
    const auto created=::mkdtemp(writable.data());
    ASSERT_NE(created,nullptr);directory_=created;manifest_path_=directory_/"prior.json";
    data_.assign(24,2);
    data_[1]=1;data_[2]=0;data_[4]=0;data_[10]=1;data_[12]=1;data_[23]=0;
    expected_.geometry_sha256=digest("frozen external colliders");
    expected_.map_version="scene-v1";expected_.frame_id="map";
    expected_.odom_frame_id="odom";expected_.resolution=.05;
    manifest_={{"schema",1},{"kind","certified_static_occupancy_prior"},
      {"provenance","isaac_closed_collision_geometry_v1"},
      {"frame_id","map"},{"map_version","scene-v1"},
      {"dtype","uint8"},{"meters_per_unit",1.},{"up_axis","Z"},
      {"storage_order","C_xyz_z_fastest"},
      {"state_codes",{{"free",0},{"occupied",1},{"unknown",2}}},
      {"voxel_resolution",.05},{"scene_sha256",digest("frozen scene")},
      {"spec_sha256",digest("frozen specification")},
      {"collider_sha256",expected_.geometry_sha256},
      {"map_from_odom",{{"transform_contract","fixed_identity_map_from_odom_v1"},
        {"from_frame","odom"},{"to_frame","map"},
        {"translation",{0.,0.,0.}},{"rotation_xyzw",{0.,0.,0.,1.}}}},
      {"closed_world_bounds",{{"semantics","whole_closed_cell_strictly_inside"},
        {"min",{-1.,-1.,-1.}},{"max",{1.,1.,1.}}}},
      {"geometry_margin_m",.001},{"origin_index",{-3,2,-1}},{"shape",{2,3,4}},
      {"data_file","states.u8"}};
    commit();
  }
  void TearDown() override {
    if(!directory_.empty()) {
      std::error_code ignored;std::filesystem::remove_all(directory_,ignored);
    }
  }
  static std::string digest(const std::string &value) {
    return Prior::sha256(value.data(),value.size());
  }
  void writeData() {
    std::ofstream output(directory_/"states.u8",std::ios::binary|std::ios::trunc);
    output.write(reinterpret_cast<const char*>(data_.data()),data_.size());
    ASSERT_TRUE(output.good());
  }
  void writeManifest() {
    const auto text=manifest_.dump();
    std::ofstream output(manifest_path_,std::ios::binary|std::ios::trunc);
    output.write(text.data(),text.size());ASSERT_TRUE(output.good());
    expected_.manifest_sha256=Prior::sha256(text.data(),text.size());
  }
  void commit() {
    writeData();manifest_["data_sha256"]=Prior::sha256(data_.data(),data_.size());
    manifest_["data_size_bytes"]=data_.size();writeManifest();
  }
  std::shared_ptr<const Prior> load() const {
    return Prior::load(manifest_path_.string(),expected_);
  }
  Json context() const {
    return {{"map_version",expected_.map_version},
      {"static_prior_manifest_sha256",expected_.manifest_sha256},
      {"map_from_odom_contract",expected_.transform_contract},
      {"map_from_odom_translation",{0.,0.,0.}},
      {"map_from_odom_rotation_xyzw",{0.,0.,0.,1.}}};
  }
  void supportContact() {
    expected_.allow_floor_contact=true;expected_.support_floor_z=0.;expected_.support_penetration_m=.02;
    expected_.dynamic_registry_sha256=digest("complete actors");
    manifest_["state_codes"]["support_contact"]=3;
    manifest_["closed_world_bounds"]["min"][2]=0.;
    manifest_["flat_support_contact"]={{"schema",1},{"kind","flat_plane_support_contact_v1"},
      {"floor_path","/World/GroundPlane/collisionPlane"},{"floor_z",0.},{"penetration_allowance_m",.02},
      {"semantics","whole_closed_cell_floor_intersection_inside_authorized_xy_and_disjoint_from_nonfloor_static_solids_plus_margin"}};
    manifest_["dynamic_actor_registry_sha256"]=expected_.dynamic_registry_sha256;
    manifest_["collision_geometry"]={{"flat_support_contact",manifest_["flat_support_contact"]},
      {"closed_world_bounds",manifest_["closed_world_bounds"]},
      {"dynamic_actor_registry_sha256",expected_.dynamic_registry_sha256},
      {"dynamic_actor_registry",Json::array({{{"id","registered-person"}}})},
      {"floor",{{"path","/World/GroundPlane/collisionPlane"},{"type","Plane"},{"axis","Z"},{"extent","infinite"},{"z",0.}}},
      {"boxes",Json::array()}};
    data_.assign(24,2);data_[1]=3;commit();
  }
  std::filesystem::path directory_,manifest_path_;
  std::vector<std::uint8_t> data_;
  Json manifest_;
  Prior::Expected expected_;
};

TEST_F(StaticOccupancyPriorTest, ExactWorldIntegerIndexUsesZFastestWithoutInterpolation) {
  const auto prior=load();ASSERT_NE(prior,nullptr);EXPECT_EQ(prior->bytes(),24U);
  EXPECT_EQ(prior->state({-3,2,-1}),2);
  EXPECT_EQ(prior->state({-3,2,0}),1); // Flat byte 1: adjacent z.
  EXPECT_EQ(prior->state({-3,2,1}),0); // Flat byte 2.
  EXPECT_EQ(prior->state({-3,3,-1}),0); // Flat byte 4: next y.
  EXPECT_EQ(prior->state({-3,4,1}),1); // Flat byte 10.
  EXPECT_EQ(prior->state({-2,2,-1}),1); // Flat byte 12: next x.
  EXPECT_EQ(prior->state({-2,4,2}),0); // Flat byte 23: upper included index.
  EXPECT_EQ(prior->manifestSha256(),expected_.manifest_sha256);
  EXPECT_EQ(prior->geometrySha256(),expected_.geometry_sha256);
  EXPECT_EQ(prior->mapVersion(),"scene-v1");
}

TEST_F(StaticOccupancyPriorTest, OutsideStoredIndicesAlwaysRemainsUnknown) {
  const auto prior=load();
  for(const std::array<int,3> cell:std::vector<std::array<int,3>>{
      {-4,2,-1},{-1,2,-1},{-3,1,-1},{-3,5,-1},{-3,2,-2},{-3,2,3},
      {std::numeric_limits<int>::min(),2,-1},{std::numeric_limits<int>::max(),2,-1}}) {
    SCOPED_TRACE(::testing::Message()<<cell[0]<<","<<cell[1]<<","<<cell[2]);
    EXPECT_EQ(prior->state(cell),2);
  }
}
TEST_F(StaticOccupancyPriorTest, FloorContactIsOptInIndependentStateAndNeverDefaultFree) {
  supportContact();const auto prior=load();EXPECT_TRUE(prior->floorContactEnabled());
  EXPECT_DOUBLE_EQ(prior->floorZ(),0.);EXPECT_EQ(prior->state({-3,2,0}),3);
  EXPECT_TRUE(prior->dynamicRegistryMatches(expected_.dynamic_registry_sha256,{"registered-person"}));
  EXPECT_FALSE(prior->dynamicRegistryMatches(expected_.dynamic_registry_sha256,{}));
  EXPECT_FALSE(prior->dynamicRegistryMatches(expected_.dynamic_registry_sha256,{"registered-person","registered-person"}));
  expected_.allow_floor_contact=false;EXPECT_THROW(load(),std::invalid_argument);
}
TEST_F(StaticOccupancyPriorTest, ExactEmptyDynamicRegistryStillBindsItsDistinctDigest) {
  supportContact();manifest_["collision_geometry"]["dynamic_actor_registry"]=Json::array();commit();
  const auto prior=load();EXPECT_TRUE(prior->dynamicRegistryMatches(expected_.dynamic_registry_sha256,{}));
  EXPECT_FALSE(prior->dynamicRegistryMatches("",{}));
  EXPECT_FALSE(prior->dynamicRegistryMatches(expected_.dynamic_registry_sha256,{"newborn"}));
}
TEST_F(StaticOccupancyPriorTest, SupportCellMustMeetExactFloorAndAuthorizedWholeXY) {
  supportContact();data_[1]=2;data_[2]=3;commit(); // z=[.05,.10] cannot contact z=0.
  EXPECT_THROW(load(),std::invalid_argument);
  supportContact();manifest_["closed_world_bounds"]["min"][0]=-.149;writeManifest();
  EXPECT_THROW(load(),std::invalid_argument);
}
TEST_F(StaticOccupancyPriorTest, FloorContactCannotEraseSharedLowObstacleOrMarginCell) {
  supportContact();manifest_["collision_geometry"]["boxes"].push_back({{"type","Cube"},
    {"min",{-.149,.101,0.}},{"max",{-.11,.14,.02}}});writeManifest();
  EXPECT_THROW(load(),std::invalid_argument);
  data_[1]=1;commit();const auto prior=load();EXPECT_EQ(prior->state({-3,2,0}),1);
}
TEST_F(StaticOccupancyPriorTest, FloorContactRequiresSimulationAndCompleteBoundedRegistry) {
  supportContact();manifest_["provenance"]="certified_prebuilt_static_volume_v1";writeManifest();
  EXPECT_THROW(load(),std::invalid_argument);
  supportContact();manifest_["collision_geometry"]["dynamic_actor_registry"]=nullptr;writeManifest();
  EXPECT_THROW(load(),std::invalid_argument);
  supportContact();expected_.support_penetration_m=.021;EXPECT_THROW(load(),std::invalid_argument);
}
TEST_F(StaticOccupancyPriorTest, EndpointNumericalBoundIsExplicitExactAndNeverDefaultProduction) {
  supportContact();EXPECT_DOUBLE_EQ(load()->floorEndpointErrorBoundM(),1e-5);
  expected_.support_endpoint_error_bound_m=.0002;
  EXPECT_THROW(load(),std::invalid_argument); // Legacy absence cannot authorize a wider bound.
  manifest_["flat_support_contact"]["floor_endpoint_error_bound_m"] = .0002;
  manifest_["collision_geometry"]["flat_support_contact"] = manifest_["flat_support_contact"];commit();
  EXPECT_DOUBLE_EQ(load()->floorEndpointErrorBoundM(),.0002);
  expected_.support_endpoint_error_bound_m=1e-5;EXPECT_THROW(load(),std::invalid_argument);
  expected_.support_endpoint_error_bound_m=.0002;expected_.allow_floor_contact=false;
  EXPECT_THROW(load(),std::invalid_argument);
}
TEST_F(StaticOccupancyPriorTest, EndpointNumericalBoundRejectsOversizeResolutionCapAndGeometryMismatch) {
  supportContact();expected_.support_endpoint_error_bound_m=.0002;
  manifest_["flat_support_contact"]["floor_endpoint_error_bound_m"] = .0002;
  manifest_["collision_geometry"]["flat_support_contact"] = manifest_["flat_support_contact"];commit();
  manifest_["collision_geometry"]["flat_support_contact"]["floor_endpoint_error_bound_m"] = .0001;commit();
  EXPECT_THROW(load(),std::invalid_argument);
  manifest_["collision_geometry"]["flat_support_contact"] = manifest_["flat_support_contact"];
  for(const auto invalid:{0.,-1e-5,.0010001,std::numeric_limits<double>::infinity()}) {
    expected_.support_endpoint_error_bound_m=invalid;
    manifest_["flat_support_contact"]["floor_endpoint_error_bound_m"]=invalid;
    manifest_["collision_geometry"]["flat_support_contact"]=manifest_["flat_support_contact"];commit();
    EXPECT_THROW(load(),std::invalid_argument);
  }
  expected_.support_endpoint_error_bound_m=.0002;expected_.resolution=.001;
  manifest_["voxel_resolution"]=.001;
  manifest_["flat_support_contact"]["floor_endpoint_error_bound_m"]=.0002;
  manifest_["collision_geometry"]["flat_support_contact"]=manifest_["flat_support_contact"];commit();
  EXPECT_THROW(load(),std::invalid_argument); // .2mm exceeds this grid's resolution/20.
}

TEST_F(StaticOccupancyPriorTest, UnsignedWorldOriginCannotWrapIntoNegativeValidIndex) {
  manifest_["origin_index"][0]=std::numeric_limits<std::uint64_t>::max();writeManifest();
  EXPECT_THROW(load(),std::invalid_argument);
}

TEST_F(StaticOccupancyPriorTest, EncodedThinWallAndUnknownAreNeverBridgedByNeighboringFree) {
  // These bytes represent a producer-certified subcell wall overlap. This
  // loader test does not claim to test the producer's collider rasterization.
  data_.assign(24,2);data_[1]=0;data_[5]=1;commit();
  const auto prior=load();
  EXPECT_EQ(prior->state({-3,2,0}),0);
  EXPECT_EQ(prior->state({-3,3,0}),1);
  EXPECT_EQ(prior->state({-3,4,0}),2);
  EXPECT_EQ(prior->state({-2,3,0}),2);
}

TEST_F(StaticOccupancyPriorTest, ManifestMutationCannotUsePreviousAuthorizationHash) {
  const auto authorized=expected_;
  manifest_["map_version"]="other-scene";writeManifest();
  EXPECT_THROW(Prior::load(manifest_path_.string(),authorized),std::invalid_argument);
}

TEST_F(StaticOccupancyPriorTest, ActualDataMutationCannotUseManifestDataHash) {
  data_[1]=0;writeData(); // Do not alter the authorized manifest or its data hash.
  EXPECT_THROW(load(),std::invalid_argument);
}

TEST_F(StaticOccupancyPriorTest, ValidlyHashedManifestStillRejectsWrongDataDigest) {
  manifest_["data_sha256"]=digest("different bytes");writeManifest();
  EXPECT_THROW(load(),std::invalid_argument);
}

TEST_F(StaticOccupancyPriorTest, ByteCountMustEqualDeclaredVolumeAndBeNonempty) {
  const auto original=data_;
  for(std::size_t size:{0U,23U,25U}) {
    SCOPED_TRACE(size);data_=original;data_.resize(size,2);commit();
    EXPECT_THROW(load(),std::invalid_argument);
  }
}

TEST_F(StaticOccupancyPriorTest, DeclaredByteCountMustBeExactUnsignedVolumeSize) {
  for(const Json size:std::vector<Json>{0,23,25,-1,24.5}) {
    SCOPED_TRACE(size.dump());manifest_["data_size_bytes"]=size;writeManifest();
    EXPECT_THROW(load(),std::invalid_argument);
  }
}

TEST_F(StaticOccupancyPriorTest, AuthenticatedDimensionsCannotOverflowOrChangeType) {
  const auto original=manifest_;
  for(const Json shape:std::vector<Json>{{0,3,4},{-1,3,4},{2,3},{2,3,4.5},
      {100001,1,1},{100000,100000,1}}) {
    SCOPED_TRACE(shape.dump());manifest_=original;manifest_["shape"]=shape;writeManifest();
    EXPECT_THROW(load(),std::invalid_argument);
  }
  for(const Json origin:std::vector<Json>{{-3,2},{-3.5,2,-1},
      {std::numeric_limits<int>::max(),2,-1},
      {static_cast<std::int64_t>(std::numeric_limits<int>::min())-1,2,-1}}) {
    SCOPED_TRACE(origin.dump());manifest_=original;manifest_["origin_index"]=origin;writeManifest();
    EXPECT_THROW(load(),std::invalid_argument);
  }
}

TEST_F(StaticOccupancyPriorTest, AuthenticatedWrongFrameMapGeometryOrResolutionIsRejected) {
  const auto original=manifest_;
  for(const auto &mutation:std::vector<std::pair<std::string,Json>>{
      {"frame_id","other-map"},{"map_version","scene-v2"},
      {"collider_sha256",digest("other geometry")},{"voxel_resolution",.1}}) {
    SCOPED_TRACE(mutation.first);manifest_=original;manifest_[mutation.first]=mutation.second;writeManifest();
    EXPECT_THROW(load(),std::invalid_argument);
  }
}

TEST_F(StaticOccupancyPriorTest, AuthenticatedTransformMustMatchExactDeclaredIdentity) {
  const auto original=manifest_;
  for(const auto &mutation:std::vector<std::pair<std::string,Json>>{
      {"from_frame","other-odom"},{"to_frame","other-map"},
      {"transform_contract","estimated_map_from_odom"},
      {"translation",{1e-12,0.,0.}},{"translation",{0.,0.}},
      {"rotation_xyzw",{0.,0.,0.,-1.}},{"rotation_xyzw",{0.,0.,.01,1.}}}) {
    SCOPED_TRACE(::testing::Message()<<mutation.first<<"="<<mutation.second.dump());
    manifest_=original;manifest_["map_from_odom"][mutation.first]=mutation.second;writeManifest();
    EXPECT_THROW(load(),std::invalid_argument);
  }
}

TEST_F(StaticOccupancyPriorTest, UnsupportedCertificationEncodingAndPathAreRejected) {
  const auto original=manifest_;
  for(const auto &mutation:std::vector<std::pair<std::string,Json>>{
      {"schema",2},{"kind","point_cloud_free_prior"},{"provenance","no_obstacle_samples"},
      {"storage_order","C_zyx_x_fastest"},
      {"dtype","float32"},{"meters_per_unit",.01},{"up_axis","Y"},
      {"state_codes",{{"free",2},{"occupied",1},{"unknown",0}}},
      {"data_file","../states.u8"},{"data_file","/tmp/states.u8"},
      {"geometry_margin_m",-.01},{"geometry_margin_m",.051}}) {
    SCOPED_TRACE(::testing::Message()<<mutation.first<<"="<<mutation.second.dump());
    manifest_=original;manifest_[mutation.first]=mutation.second;writeManifest();
    EXPECT_THROW(load(),std::invalid_argument);
  }
}

TEST_F(StaticOccupancyPriorTest, AuthenticatedByteCodesOutsideContractAreRejected) {
  for(std::uint8_t code:{3,127,255}) {
    SCOPED_TRACE(unsigned(code));data_[1]=code;commit();
    EXPECT_THROW(load(),std::invalid_argument);
  }
}

TEST_F(StaticOccupancyPriorTest, WholeFreeCellCannotTouchAuthorizationBoundaryOrMargin) {
  const auto original=manifest_;
  data_.assign(24,2);data_[18]=0; // World cell {-2,3,1}.
  const std::array<double,3> lo{{-.1,.15,.05}},hi{{-.05,.2,.1}};
  for(std::size_t axis=0;axis<3;++axis) {
    for(bool upper:{false,true}) {
      SCOPED_TRACE(::testing::Message()<<"axis="<<axis<<" upper="<<upper);
      manifest_=original;
      manifest_["closed_world_bounds"][upper?"max":"min"][axis]=upper?hi[axis]:lo[axis];
      commit();EXPECT_THROW(load(),std::invalid_argument);
    }
    manifest_=original;
    manifest_["closed_world_bounds"]["min"][axis]=lo[axis]-.0005;
    commit();EXPECT_THROW(load(),std::invalid_argument); // Inside geometric box, inside uncertainty margin.
  }
}

TEST_F(StaticOccupancyPriorTest, BoundaryOccupiedAndUnknownDoNotClaimUnauthorizedFree) {
  data_.assign(24,2);data_[0]=1;
  manifest_["closed_world_bounds"]["min"]={0.,0.,0.};
  manifest_["closed_world_bounds"]["max"]={1.,1.,1.};commit();
  const auto prior=load();EXPECT_EQ(prior->state({-3,2,-1}),1);
  EXPECT_EQ(prior->state({-3,2,0}),2);
}

TEST_F(StaticOccupancyPriorTest, LoadedVolumeAndIdentityStayImmutableAfterDiskReplacement) {
  const auto prior=load();const auto original_hash=expected_.manifest_sha256;
  const auto original_context=context();
  data_.assign(24,0);manifest_["map_version"]="scene-v2";
  expected_.map_version="scene-v2";commit();
  const auto replacement=load();
  EXPECT_EQ(prior->state({-3,2,0}),1);EXPECT_EQ(prior->state({-3,2,-1}),2);
  EXPECT_EQ(prior->manifestSha256(),original_hash);EXPECT_EQ(prior->mapVersion(),"scene-v1");
  EXPECT_TRUE(prior->matchesContext(original_context));EXPECT_FALSE(prior->matchesContext(context()));
  EXPECT_EQ(replacement->state({-3,2,0}),0);EXPECT_TRUE(replacement->matchesContext(context()));
}

TEST_F(StaticOccupancyPriorTest, ContextRequiresExactVersionManifestAndIdentity) {
  const auto prior=load();const auto good=context();ASSERT_TRUE(prior->matchesContext(good));
  for(const auto &mutation:std::vector<std::pair<std::string,Json>>{
      {"map_version","scene-v2"},{"static_prior_manifest_sha256",digest("other manifest")},
      {"map_from_odom_contract","estimated_map_from_odom"},
      {"map_from_odom_translation",{0.,1e-12,0.}},
      {"map_from_odom_rotation_xyzw",{0.,0.,0.,-1.}},
      {"map_from_odom_rotation_xyzw",{0.,0.,0.}}}) {
    SCOPED_TRACE(mutation.first);auto bad=good;bad[mutation.first]=mutation.second;
    EXPECT_FALSE(prior->matchesContext(bad));
  }
  for(auto entry=good.begin();entry!=good.end();++entry) {
    SCOPED_TRACE(entry.key());auto missing=good;missing.erase(entry.key());
    EXPECT_FALSE(prior->matchesContext(missing));
  }
  EXPECT_FALSE(prior->matchesContext(nullptr));EXPECT_FALSE(prior->matchesContext(Json::array()));
}

TEST_F(StaticOccupancyPriorTest, ExpectedAuthorizationCannotBeEmptyOrMalformed) {
  for(const std::string digest_value:{std::string{},std::string(63,'a'),std::string(64,'G')}) {
    SCOPED_TRACE(digest_value);auto bad=expected_;bad.manifest_sha256=digest_value;
    EXPECT_THROW(Prior::load(manifest_path_.string(),bad),std::invalid_argument);
    bad=expected_;bad.geometry_sha256=digest_value;
    EXPECT_THROW(Prior::load(manifest_path_.string(),bad),std::invalid_argument);
  }
  auto bad=expected_;bad.frame_id.clear();
  EXPECT_THROW(Prior::load(manifest_path_.string(),bad),std::invalid_argument);
  bad=expected_;bad.map_version.clear();
  EXPECT_THROW(Prior::load(manifest_path_.string(),bad),std::invalid_argument);
  bad=expected_;bad.resolution=0.;
  EXPECT_THROW(Prior::load(manifest_path_.string(),bad),std::invalid_argument);
}
} // namespace
