#include <gtest/gtest.h>
#include <d1max_planning_interfaces/isolated_reference_model.hpp>
#include <cstdio>
#include <unistd.h>
using d1max_planning_interfaces::IsolatedReferenceModel;
using Json=nlohmann::json;
namespace {
Json record() {
  return {{"schema_version",3},{"transport_mode","isolated_mock"},{"fixture_only",true},
    {"model","reaction_braking_reachable_v1"},
    {"measurements",{{"max_speed_mps",.6},{"max_yaw_radps",.8}}},
    {"isolated_platform_model",{{"schema",1},{"kind","official_spot_physx"},
      {"source_scope","isolated_simulation_physx_measured_model"},
      {"command_max_speed_mps",.15},{"command_max_yaw_radps",.3},
      {"reachable_max_speed_mps",.6},{"reachable_max_yaw_radps",.8}}},
    {"isolated_full_xyz_reference_model",{{"schema",1},{"kind","official_spot_physx"},
      {"source_scope","isolated_simulation_physx_measured_model"},
      {"reference_max_speed_mps",.5},{"measured_travel_max_speed_mps",.5},
      {"observed_max_full_xyz_speed_mps",.48905959685208217},{"evidence_sha256",std::string(64,'b')}}}};
}
auto parse(const Json& j,const std::string& mode="isolated_mock") {
  return IsolatedReferenceModel::parse(j,std::string(64,'a'),mode);
}
Json recordWithDomain(double speed,double observed) {
  auto j=record();
  j["measurements"]["max_speed_mps"]=speed;
  j["isolated_platform_model"]["reachable_max_speed_mps"]=speed;
  auto& reference=j["isolated_full_xyz_reference_model"];
  reference["reference_max_speed_mps"]=speed;
  reference["measured_travel_max_speed_mps"]=speed;
  reference["observed_max_full_xyz_speed_mps"]=observed;
  return j;
}
}
TEST(IsolatedReferenceModel, SeparateReferenceTravelAndCommandFields) {
  const auto m=parse(record());ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->referenceSpeed(),.5);EXPECT_DOUBLE_EQ(m->measuredTravelSpeed(),.5);
  EXPECT_DOUBLE_EQ(m->commandSpeed(),.15);EXPECT_DOUBLE_EQ(m->commandYaw(),.3);
  EXPECT_DOUBLE_EQ(m->observedSpeed(),.48905959685208217);
  auto j=record();j["isolated_full_xyz_reference_model"]["measured_travel_max_speed_mps"]=.49;
  const auto narrower=parse(j);ASSERT_TRUE(narrower);EXPECT_DOUBLE_EQ(narrower->measuredTravelSpeed(),.49);
}
TEST(IsolatedReferenceModel, AbsentMarkerKeepsLegacyInsteadOfPlanarReachInference) {
  auto j=record();j.erase("isolated_full_xyz_reference_model");
  EXPECT_FALSE(parse(j));EXPECT_FALSE(parse(j,"live"));
  EXPECT_FALSE(IsolatedReferenceModel::load("","","live"));
}
TEST(IsolatedReferenceModel, LiveOrUnsealedFixtureCannotEnableModel) {
  EXPECT_ANY_THROW(parse(record(),"live"));
  for(const char* key:{"fixture_only","transport_mode","schema_version","model","isolated_platform_model"}) {
    auto j=record();j.erase(key);EXPECT_ANY_THROW(parse(j))<<key;
  }
  auto j=record();j["fixture_only"]=false;EXPECT_ANY_THROW(parse(j));
  j=record();j["fixture_only"]=1;EXPECT_ANY_THROW(parse(j));
  j=record();j["schema_version"]=3.0;EXPECT_ANY_THROW(parse(j));
}
TEST(IsolatedReferenceModel, MarkersRequireExactKindScopeIntegerSchemaAndKeys) {
  for(const char* name:{"isolated_platform_model","isolated_full_xyz_reference_model"}) {
    auto j=record();j[name]["extra"]=true;EXPECT_ANY_THROW(parse(j));
    j=record();j[name]["kind"]="another_quadruped";EXPECT_ANY_THROW(parse(j));
    j=record();j[name]["schema"]=1.0;EXPECT_ANY_THROW(parse(j));
    j=record();j[name]["source_scope"]="live";EXPECT_ANY_THROW(parse(j));
  }
  for(const char* key:{"schema","kind","source_scope","reference_max_speed_mps",
      "measured_travel_max_speed_mps","observed_max_full_xyz_speed_mps","evidence_sha256"}) {
    auto j=record();j["isolated_full_xyz_reference_model"].erase(key);EXPECT_ANY_THROW(parse(j))<<key;
  }
}
TEST(IsolatedReferenceModel, ExplicitPointSixAdmissionKeepsCommandAuthoritySeparate) {
  for(const double observed:{.508039,.532,.6}) {
    auto j=record();auto& reference=j["isolated_full_xyz_reference_model"];
    reference["reference_max_speed_mps"]=.6;reference["measured_travel_max_speed_mps"]=.6;
    reference["observed_max_full_xyz_speed_mps"]=observed;
    const auto m=parse(j);ASSERT_TRUE(m);
    EXPECT_DOUBLE_EQ(m->referenceSpeed(),.6);EXPECT_DOUBLE_EQ(m->measuredTravelSpeed(),.6);
    EXPECT_DOUBLE_EQ(m->observedSpeed(),observed);
    EXPECT_DOUBLE_EQ(m->commandSpeed(),.15);EXPECT_DOUBLE_EQ(m->commandYaw(),.3);
  }
}
TEST(IsolatedReferenceModel, MatchedPointSixFiveDomainAcceptsMeasuredPeakAndExactBoundary) {
  for(const double observed:{.616161,.65}) {
    const auto m=parse(recordWithDomain(.65,observed));ASSERT_TRUE(m);
    EXPECT_DOUBLE_EQ(m->referenceSpeed(),.65);EXPECT_DOUBLE_EQ(m->measuredTravelSpeed(),.65);
    EXPECT_DOUBLE_EQ(m->observedSpeed(),observed);
    EXPECT_DOUBLE_EQ(m->commandSpeed(),.15);EXPECT_DOUBLE_EQ(m->commandYaw(),.3);
  }
  EXPECT_ANY_THROW(parse(recordWithDomain(.6500001,.616161)));
  for(const char* key:{"reference_max_speed_mps","measured_travel_max_speed_mps","observed_max_full_xyz_speed_mps"}) {
    auto j=recordWithDomain(.65,.616161);
    j["isolated_full_xyz_reference_model"][key]=.6500001;
    EXPECT_ANY_THROW(parse(j))<<key;
  }
}
TEST(IsolatedReferenceModel, OldPointSixAndIndependentNarrowerDomainsStillRejectNewMeasuredPeak) {
  EXPECT_ANY_THROW(parse(recordWithDomain(.6,.616161)));
  for(const char* key:{"reference_max_speed_mps","measured_travel_max_speed_mps"}) {
    auto j=recordWithDomain(.65,.616161);
    j["isolated_full_xyz_reference_model"][key]=.6;
    EXPECT_ANY_THROW(parse(j))<<key;
  }
}
TEST(IsolatedReferenceModel, PointSixFiveRequiresSameIsolationEvidenceAndAuthorityClosure) {
  const auto valid=recordWithDomain(.65,.616161);
  EXPECT_ANY_THROW(parse(valid,"live"));
  EXPECT_ANY_THROW(IsolatedReferenceModel::parse(valid,std::string(64,'x'),"isolated_mock"));
  auto j=valid;j["fixture_only"]=false;EXPECT_ANY_THROW(parse(j));
  j=valid;j["transport_mode"]="live";EXPECT_ANY_THROW(parse(j));
  j=valid;j["isolated_full_xyz_reference_model"]["evidence_sha256"]="unsealed";EXPECT_ANY_THROW(parse(j));
  j=valid;j["measurements"]["max_speed_mps"]=.6;EXPECT_ANY_THROW(parse(j));
  j=valid;j["isolated_platform_model"]["command_max_speed_mps"]=.3000001;EXPECT_ANY_THROW(parse(j));
  j=valid;j["isolated_platform_model"]["command_max_yaw_radps"]=.5000001;EXPECT_ANY_THROW(parse(j));
  j=valid;j["isolated_platform_model"]["reachable_max_yaw_radps"]=.8000001;
  j["measurements"]["max_yaw_radps"]=.8000001;EXPECT_ANY_THROW(parse(j));
}
TEST(IsolatedReferenceModel, ReferenceTravelAndObservedMustFitSameSealedPlatformReach) {
  for(const char* key:{"reference_max_speed_mps","measured_travel_max_speed_mps","observed_max_full_xyz_speed_mps"}) {
    auto j=record();j["isolated_platform_model"]["reachable_max_speed_mps"]=.49;
    j["measurements"]["max_speed_mps"]=.49;
    j["isolated_full_xyz_reference_model"]["reference_max_speed_mps"]=.49;
    j["isolated_full_xyz_reference_model"]["measured_travel_max_speed_mps"]=.49;
    j["isolated_full_xyz_reference_model"][key]=.5;
    EXPECT_ANY_THROW(parse(j))<<key;
  }
  auto j=record();j["isolated_platform_model"]["reachable_max_speed_mps"]=.55;
  j["measurements"]["max_speed_mps"]=.55;
  j["isolated_full_xyz_reference_model"]["reference_max_speed_mps"]=.6;
  j["isolated_full_xyz_reference_model"]["measured_travel_max_speed_mps"]=.6;
  j["isolated_full_xyz_reference_model"]["observed_max_full_xyz_speed_mps"]=.532;
  EXPECT_ANY_THROW(parse(j));
}
TEST(IsolatedReferenceModel, AdmissionDomainRejectsAboveReachAndMalformedNumbers) {
  for(const char* key:{"reference_max_speed_mps","measured_travel_max_speed_mps","observed_max_full_xyz_speed_mps"})
    for(const Json& value:{Json(.600001),Json(.61),Json(0),Json(-1),Json(true),Json(".6"),Json(nullptr)}) {
      auto j=record();j["isolated_full_xyz_reference_model"][key]=value;EXPECT_ANY_THROW(parse(j))<<key;
    }
}
TEST(IsolatedReferenceModel, ObservedEvidenceMustFitBothIndependentDomains) {
  for(const char* key:{"reference_max_speed_mps","measured_travel_max_speed_mps"}) {
    auto j=record();j["isolated_full_xyz_reference_model"][key]=.48;EXPECT_ANY_THROW(parse(j));
  }
  auto j=record();j["isolated_full_xyz_reference_model"]["evidence_sha256"]="unsealed";EXPECT_ANY_THROW(parse(j));
  EXPECT_ANY_THROW(IsolatedReferenceModel::parse(record(),std::string(64,'x'),"isolated_mock"));
}
TEST(IsolatedReferenceModel, CommandMustFitBothDomainsEvenWhenObservedEvidenceIsLower) {
  for(const char* key:{"reference_max_speed_mps","measured_travel_max_speed_mps"}) {
    auto j=record();j["isolated_full_xyz_reference_model"]["observed_max_full_xyz_speed_mps"]=.1;
    j["isolated_full_xyz_reference_model"][key]=.14;EXPECT_ANY_THROW(parse(j))<<key;
  }
}
TEST(IsolatedReferenceModel, OriginalPlatformMeasurementAndCommandClosureRemainExact) {
  auto j=record();j["measurements"]["max_speed_mps"]=.59;EXPECT_ANY_THROW(parse(j));
  j=record();j["isolated_platform_model"]["command_max_speed_mps"]=.31;EXPECT_ANY_THROW(parse(j));
  j=record();j["isolated_platform_model"]["command_max_yaw_radps"]=.51;EXPECT_ANY_THROW(parse(j));
}
TEST(IsolatedReferenceModel, LoaderChecksOriginalBytesHashAndBoundedAbsolutePath) {
  const std::string bytes=record().dump();unsigned char digest[EVP_MAX_MD_SIZE];unsigned count=0;
  ASSERT_TRUE(EVP_Digest(bytes.data(),bytes.size(),digest,&count,EVP_sha256(),nullptr));
  std::ostringstream hex;for(unsigned i=0;i<count;++i)hex<<std::hex<<std::setfill('0')<<std::setw(2)<<int(digest[i]);
  const auto file="/tmp/d1max_reference_model_test_"+std::to_string(getpid())+".json";
  {std::ofstream stream(file);stream<<bytes;}
  const auto m=IsolatedReferenceModel::load(file,hex.str(),"isolated_mock");ASSERT_TRUE(m);
  EXPECT_EQ(m->recordSha(),hex.str());
  EXPECT_ANY_THROW(IsolatedReferenceModel::load(file,std::string(64,'a'),"isolated_mock"));
  EXPECT_ANY_THROW(IsolatedReferenceModel::load(file,hex.str(),"live"));
  EXPECT_ANY_THROW(IsolatedReferenceModel::load("relative.json",hex.str(),"isolated_mock"));
  {std::ofstream stream(file,std::ios::app);stream<<' ';}
  EXPECT_ANY_THROW(IsolatedReferenceModel::load(file,hex.str(),"isolated_mock"));
  std::remove(file.c_str());
}
TEST(IsolatedReferenceModel, PointSixFiveLoaderRequiresMatchedOriginalByteHashAndRejectsLive) {
  const std::string bytes=recordWithDomain(.65,.616161).dump();
  unsigned char digest[EVP_MAX_MD_SIZE];unsigned count=0;
  ASSERT_TRUE(EVP_Digest(bytes.data(),bytes.size(),digest,&count,EVP_sha256(),nullptr));
  std::ostringstream hex;for(unsigned i=0;i<count;++i)hex<<std::hex<<std::setfill('0')<<std::setw(2)<<int(digest[i]);
  const auto file="/tmp/d1max_reference_model_point_six_five_test_"+std::to_string(getpid())+".json";
  {std::ofstream stream(file,std::ios::binary);stream<<bytes;}
  const auto m=IsolatedReferenceModel::load(file,hex.str(),"isolated_mock");ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->referenceSpeed(),.65);EXPECT_DOUBLE_EQ(m->measuredTravelSpeed(),.65);
  EXPECT_DOUBLE_EQ(m->observedSpeed(),.616161);EXPECT_EQ(m->recordSha(),hex.str());
  EXPECT_EQ(m->evidenceSha(),std::string(64,'b'));
  EXPECT_ANY_THROW(IsolatedReferenceModel::load(file,std::string(64,'a'),"isolated_mock"));
  EXPECT_ANY_THROW(IsolatedReferenceModel::load(file,hex.str(),"live"));
  {std::ofstream stream(file,std::ios::app);stream<<' ';}
  EXPECT_ANY_THROW(IsolatedReferenceModel::load(file,hex.str(),"isolated_mock"));
  std::remove(file.c_str());
}
