#include "execution_acceptance.hpp"
#include <cassert>
#include <cstdlib>
#include <iostream>
using namespace d1monitor::execution3;
int main(){
 char directory[]="/tmp/d1max-acceptance-test-XXXXXX";const auto*created=mkdtemp(directory);assert(created);
 const std::filesystem::path dir(created),evidence=dir/"evidence.txt",record=dir/"record.json";
 {std::ofstream out(evidence);out<<"Synthetic validator fixture, not physical acceptance.";}
 const std::string cal(64,'a'),profile(64,'b');
 nlohmann::json j={{"schema_version",3},{"evidence_id","TEST_ONLY"},{"robot_id","robot"},{"sdk_version","test"},
 {"profile","general_low_speed"},{"calibration_sha256",cal},{"robot_profile_sha256",profile},
 {"speed_mapping_verified",true},{"stop_timing_verified",true},{"body_envelope_verified",true},{"raw_ray_safety_verified",true},
 {"measurements",{{"forward_scale_mps",1.},{"yaw_scale_radps",1.5},{"max_speed_mps",.3},{"max_yaw_radps",.5},
 {"model","reaction_braking_reachable_v1"},{"reaction_bound_s",.8},{"stopping_distance_m",.1},{"stopping_yaw_rad",.1},
 {"stop_latency_bound_s",.3},{"tracking_error_bound_m",.02},{"heading_error_bound_rad",.05},{"body_height_m",.5},
 {"mc_clock_basis","source_delta_host_anchor_approximate"},{"mc_delay_bound_s",.1}}},{"records",nlohmann::json::array()}};
 j["execution_timing"]={{"sensor_source_age_bound_s",.5},{"command_pipeline_bound_s",.1},
   {"writer_period_s",.05},{"source_time_uncertainty_s",.1}};
 j["stationary_evidence"]={{"profile","general_low_speed"},{"linear_threshold_mps",.03},{"angular_threshold_radps",.05},
   {"stationary_duration_s",1.},{"reentry_duration_s",.6},{"minimum_new_samples",3},
   {"measured_static_linear_bound_mps",.01},{"measured_static_angular_bound_radps",.02},
   {"mc_expected_hz",50.},{"mc_min_hz",40.},{"mc_max_hz",60.}};
 for(const auto*kind:{"geometry","speed","braking","mc_time","raw_ray"})j["records"].push_back({{"path",evidence.string()},{"sha256",fileSha256(evidence)},{"kind",kind}});
 auto check=[&](const nlohmann::json&item){ {std::ofstream out(record);out<<item;}return validateAcceptance(record.string(),"robot","test",cal,profile);};
 assert(check(j).valid&&check(j).physically_verified);assert(check(j).mc_delay_bound==.1);auto bad=j;bad.erase("records");assert(!check(bad).valid);
 // Old records remain forward-only; full XY motion requires explicit,
 // measured lateral mapping and braking, never a vendor-table default.
 assert(check(j).kinematic_profile=="forward_yaw"&&check(j).lateral_scale==0.);
 bad=j;bad["kinematic_profile"]="unknown";assert(!check(bad).valid);
 auto omni=j;omni["kinematic_profile"]="holonomic_xyyaw";
 assert(!check(omni).valid);omni["measurements"]["lateral_metres_per_fraction"]=.6;
 assert(!check(omni).valid);omni["measurements"]["stopping_lateral_distance_m"]=.12;
 auto accepted=check(omni);assert(accepted.valid&&accepted.kinematic_profile=="holonomic_xyyaw"&&accepted.lateral_scale==.6);
 for(const auto*key:{"lateral_metres_per_fraction","stopping_lateral_distance_m"}) {
   bad=omni;bad["measurements"].erase(key);assert(!check(bad).valid);
   for(double value:{0.,-.1}){bad=omni;bad["measurements"][key]=value;assert(!check(bad).valid);}
 }
 bad=omni;bad["measurements"]["lateral_metres_per_fraction"]=.29;assert(!check(bad).valid);
 const auto fractions=sdkMoveFractions(-.2,.15,-.3,"holonomic_xyyaw",.8,.6,1.2);
 assert(std::abs(fractions.left_right-.25)<1e-12&&std::abs(fractions.forward_back+.25)<1e-12&&std::abs(fractions.yaw+.25)<1e-12);
 const auto opposite=sdkMoveFractions(.16,-.12,.24,"holonomic_xyyaw",.8,.6,1.2);
 assert(std::abs(opposite.left_right+.2)<1e-12&&std::abs(opposite.forward_back-.2)<1e-12&&std::abs(opposite.yaw-.2)<1e-12);
 const auto zero=sdkMoveFractions(0.,0.,0.,"forward_yaw",1.,0.,1.5);
 assert(zero.left_right==0.&&zero.forward_back==0.&&zero.yaw==0.);
 auto rejected=[](double vx,double vy,const std::string&profile,double lateral){try{
   sdkMoveFractions(vx,vy,0.,profile,1.,lateral,1.5);return false;
 }catch(const std::invalid_argument&){return true;}};
 assert(rejected(0.,.1,"forward_yaw",.6)&&rejected(-.1,0.,"forward_yaw",0.));
 assert(rejected(0.,.1,"holonomic_xyyaw",0.)&&rejected(0.,.7,"holonomic_xyyaw",.6));
 assert(rejected(0.,0.,"unknown",.6));
 bad=j;bad.erase("measurements");assert(!check(bad).valid);bad=j;bad["robot_profile_sha256"]=cal;assert(!check(bad).valid);
 bad=j;bad["robot_id"]="other";assert(!check(bad).valid);bad=j;bad["measurements"]["max_speed_mps"]=1.5;assert(!check(bad).valid);
 bad=j;bad["measurements"]["mc_clock_basis"]="ptp_verified";assert(!check(bad).valid);
 bad=j;bad["measurements"].erase("mc_delay_bound_s");assert(!check(bad).valid);
 bad=j;bad["measurements"]["mc_delay_bound_s"]=0.;assert(!check(bad).valid);
 bad=j;bad["measurements"]["mc_delay_bound_s"]=.251;assert(!check(bad).valid);
 bad=j;bad.erase("execution_timing");assert(!check(bad).valid);
 bad=j;bad.erase("stationary_evidence");assert(!check(bad).valid);
 bad=j;bad["execution_timing"]["writer_period_s"]=.1;assert(!check(bad).valid);
 bad=j;bad["execution_timing"]["command_pipeline_bound_s"]=.01;assert(!check(bad).valid);
 bad=j;bad["measurements"]["reaction_bound_s"]=.7;assert(!check(bad).valid);
 bad=j;bad["stationary_evidence"]["measured_static_linear_bound_mps"]=.038;assert(!check(bad).valid);
 bad["stationary_evidence"]["linear_threshold_mps"]=.04;assert(check(bad).valid);
 bad=j;bad["stationary_evidence"]["profile"]="stair";assert(!check(bad).valid);
 bad=j;bad["stationary_evidence"]["velocity_metric"]="ignore_z";assert(!check(bad).valid);
 bad=j;bad["stationary_evidence"]["velocity_metric"]="planar_xy_with_vertical_guard_v1";assert(check(bad).valid);
 bad=j;bad["stationary_evidence"]["stationary_duration_s"]=.99;assert(!check(bad).valid);
 bad=j;bad["stationary_evidence"]["minimum_new_samples"]=2;assert(!check(bad).valid);
 bad=j;bad["stationary_evidence"]["minimum_new_samples"]=3.5;assert(!check(bad).valid);
 bad=j;bad["stationary_evidence"]["minimum_new_samples"]=4294967299ULL;assert(!check(bad).valid);
 bad=j;bad["stationary_evidence"]["mc_min_hz"]=80.;assert(!check(bad).valid);
 bad=j;bad["records"].erase(4);assert(!check(bad).valid);bad=j;bad["raw_ray_safety_verified"]=false;assert(!check(bad).valid);
 bad=j;bad["records"][0]["sha256"]=cal;assert(!check(bad).valid);
 for(const auto*key:{"model","stopping_yaw_rad","tracking_error_bound_m","heading_error_bound_rad"}){bad=j;bad["measurements"].erase(key);assert(!check(bad).valid);}
 bad=j;bad["fixture_only"]=true;assert(!check(bad).valid);
 bad=j;bad["measurements"]["tracking_error_bound_m"]=.27;assert(!check(bad).valid);
 bad["measurements"]["tracking_error_bound_m"]=.25;assert(check(bad).valid);
 assert(check(j).valid);assert(loadBrakingModel(record.string(),fileSha256(record),"live").valid);
 assert(!loadBrakingModel(record.string(),cal,"live").valid);
 assert(!loadBrakingModel(record.string(),fileSha256(record),"isolated_mock").valid);
 auto mock=j;mock["fixture_only"]=true;mock["transport_mode"]="isolated_mock";check(mock);
 assert(loadBrakingModel(record.string(),fileSha256(record),"isolated_mock").valid);
 assert(!loadBrakingModel(record.string(),fileSha256(record),"live").valid);
 mock=omni;mock["fixture_only"]=true;mock["transport_mode"]="isolated_mock";check(mock);
 auto model=loadBrakingModel(record.string(),fileSha256(record),"isolated_mock");
 assert(model.valid&&model.kinematic_profile=="holonomic_xyyaw"&&model.stopping_lateral_distance==.12&&model.lateral_scale==.6);
 for(const auto*key:{"lateral_metres_per_fraction","stopping_lateral_distance_m"}) {
   bad=mock;bad["measurements"].erase(key);check(bad);
   assert(!loadBrakingModel(record.string(),fileSha256(record),"isolated_mock").valid);
 }
 bad=mock;bad["kinematic_profile"]="unknown";check(bad);
 assert(!loadBrakingModel(record.string(),fileSha256(record),"isolated_mock").valid);
 // Runtime profiles configure the same bounded model without claiming that
 // vendor/engineering numbers have undergone physical-evidence acceptance.
 auto runtime=omni;runtime["record_kind"]="runtime_motion_profile";
 runtime["parameter_basis"]="vendor_nominal_and_engineering";
 runtime["fixture_only"]=false;runtime["physical_acceptance_verified"]=false;
 for(const char*key:{"evidence_id","records","speed_mapping_verified","stop_timing_verified",
     "body_envelope_verified","raw_ray_safety_verified"})runtime.erase(key);
 runtime["measurements"]["forward_scale_mps"]=1.;runtime["measurements"]["lateral_metres_per_fraction"]=1.;
 runtime["measurements"]["yaw_scale_radps"]=2.;
 runtime["stationary_evidence"].erase("measured_static_linear_bound_mps");
 runtime["stationary_evidence"].erase("measured_static_angular_bound_radps");
 runtime["stationary_evidence"]["configured_static_linear_bound_mps"]=.01;
 runtime["stationary_evidence"]["configured_static_angular_bound_radps"]=.02;
 auto configured=check(runtime);assert(configured.valid&&!configured.physically_verified);
 assert(configured.parameter_basis=="vendor_nominal_and_engineering");
 assert(configured.reason=="runtime_motion_profile_configured_not_physically_verified");
 assert(configured.forward_scale==1.&&configured.lateral_scale==1.&&configured.yaw_scale==2.);
 assert(loadBrakingModel(record.string(),fileSha256(record),"live").valid);
 assert(!loadBrakingModel(record.string(),fileSha256(record),"isolated_mock").valid);
 for(const char*key:{"parameter_basis","fixture_only","physical_acceptance_verified","robot_id",
     "sdk_version","calibration_sha256","robot_profile_sha256","execution_timing","stationary_evidence"}){
   bad=runtime;bad.erase(key);assert(!check(bad).valid&&!check(bad).physically_verified);
 }
 for(const char*key:{"fixture_only","physical_acceptance_verified"})for(auto value:{nlohmann::json(true),nlohmann::json(0)}){
   bad=runtime;bad[key]=value;assert(!check(bad).valid);
   assert(!loadBrakingModel(record.string(),fileSha256(record),"live").valid);
 }
 bad=runtime;bad["parameter_basis"]="measured";assert(!check(bad).valid);
 bad=runtime;bad["record_kind"]="unknown_profile";assert(!check(bad).valid);
 bad=runtime;bad["transport_mode"]="isolated_mock";assert(!check(bad).valid);
 bad=runtime;bad["robot_id"]="foreign";assert(!check(bad).valid);
 bad=runtime;bad["profile"]="stairs";assert(!check(bad).valid);
 for(const char*key:{"forward_scale_mps","lateral_metres_per_fraction","yaw_scale_radps","stopping_distance_m",
     "stopping_lateral_distance_m","stopping_yaw_rad","reaction_bound_s","mc_delay_bound_s"}){
   bad=runtime;bad["measurements"].erase(key);assert(!check(bad).valid);
   bad=runtime;bad["measurements"][key]=0.;assert(!check(bad).valid);
 }
 bad=runtime;bad["measurements"]["max_speed_mps"]=.31;assert(!check(bad).valid);
 bad=runtime;bad["measurements"]["reaction_bound_s"]=.7;assert(!check(bad).valid);
 bad=runtime;bad["execution_timing"]["writer_period_s"]=.1;assert(!check(bad).valid);
 bad=runtime;bad["stationary_evidence"]["minimum_new_samples"]=2;assert(!check(bad).valid);
 for(const char*key:{"configured_static_linear_bound_mps","configured_static_angular_bound_radps"}){
   bad=runtime;bad["stationary_evidence"].erase(key);assert(!check(bad).valid);
   bad=runtime;bad["stationary_evidence"][key]=.2;assert(!check(bad).valid);
 }
 bad=runtime;bad["stationary_evidence"]["measured_static_linear_bound_mps"]=.01;assert(!check(bad).valid);
 // A runtime declaration cannot turn into evidence merely by deleting its
 // kind. The unchanged legacy contract still needs all evidence and flags.
 bad=runtime;bad.erase("record_kind");assert(!check(bad).valid);
 // Operator input observations select an explicitly versioned engineering
 // mapping. Legacy readers reject the new basis instead of ignoring fields.
 auto observed=runtime;observed["parameter_basis"]=operatorMappingBasis;
 SdkInputMapping mapping;mapping.enabled=true;observed["sdk_input_mapping"]=sdkInputMappingJson(mapping);
 auto configured_mapping=check(observed);
 assert(configured_mapping.valid&&!configured_mapping.physically_verified&&configured_mapping.sdk_input_mapping.enabled);
 assert(configured_mapping.parameter_basis==operatorMappingBasis);
 assert(sdkInputMappingJson(configured_mapping.sdk_input_mapping)==observed["sdk_input_mapping"]);
 assert(loadBrakingModel(record.string(),fileSha256(record),"live").valid);
 auto responsive=mapping;responsive.component_cap=true;
 responsive.planar={.30,.40,.10,.60,.60};responsive.yaw=responsive.planar;
 auto responsive_record=observed;responsive_record["sdk_input_mapping"]=sdkInputMappingJson(responsive);
 const auto responsive_checked=check(responsive_record);
 assert(responsive_checked.valid&&responsive_checked.sdk_input_mapping.component_cap);
 assert(!responsive_checked.physically_verified);
 assert(sdkInputMappingJson(responsive_checked.sdk_input_mapping)==responsive_record["sdk_input_mapping"]);
 auto incompatible=responsive_record;incompatible["sdk_input_mapping"]["schema_version"]=1;
 assert(!check(incompatible).valid);
 // V3 is an explicitly distinct engineering policy, never a speed or
 // physical-acceptance certificate. Its wire kind/schema select direct
 // breakaway; neither V1 nor V2 silently acquires that behavior.
 auto direct=responsive;direct.direct_breakaway=true;
 auto direct_record=observed;direct_record["sdk_input_mapping"]=sdkInputMappingJson(direct);
 const auto direct_json=nlohmann::json::parse(direct_record["sdk_input_mapping"].dump());
 assert(direct_json["schema_version"].is_number_integer()&&direct_json["schema_version"]==3);
 assert(direct_json["kind"]==sdkInputMappingKindV3&&direct_json["normalization"]==sdkInputMappingNormalizationV2);
 const auto direct_checked=check(direct_record);
 assert(direct_checked.valid&&!direct_checked.physically_verified);
 assert(direct_checked.parameter_basis==operatorMappingBasis);
 assert(direct_checked.sdk_input_mapping.enabled&&direct_checked.sdk_input_mapping.component_cap&&
   direct_checked.sdk_input_mapping.direct_breakaway);
 assert(std::string(sdkInputMappingKindFor(direct_checked.sdk_input_mapping))==sdkInputMappingKindV3);
 assert(sdkInputMappingJson(direct_checked.sdk_input_mapping)==direct_json);
 assert(sdkInputMappingJson(sdkInputMapping(direct_json))==direct_json);
 auto smoothed=direct;smoothed.startup_transition_s=1.;smoothed.fraction_acceleration_per_s2=2.;
 auto smoothed_record=direct_record;smoothed_record["sdk_input_mapping"]=sdkInputMappingJson(smoothed);
 const auto smoothed_checked=check(smoothed_record);
 assert(smoothed_checked.valid&&!smoothed_checked.physically_verified&&smoothed_checked.sdk_input_mapping.smoothTransitions());
 assert(sdkInputMappingJson(smoothed_checked.sdk_input_mapping)==smoothed_record["sdk_input_mapping"]);
 // V4 has only the fraction response parameters actually used by the writer.
 // It cannot silently retain a discarded startup/rate/acceleration knob.
 auto continuous=responsive;continuous.continuous_command_ratio=true;
 auto continuous_record=observed;continuous_record["sdk_input_mapping"]=sdkInputMappingJson(continuous);
 const auto continuous_json=continuous_record["sdk_input_mapping"];
 assert(continuous_json["schema_version"]==4&&continuous_json["kind"]==sdkInputMappingKindV4);
 assert(continuous_json["normalization"]==sdkInputMappingNormalizationV4);
 assert(!continuous_json.contains("smoothing")&&continuous_json["planar"].size()==3&&continuous_json["yaw"].size()==3);
 const auto continuous_checked=check(continuous_record);
 assert(continuous_checked.valid&&!continuous_checked.physically_verified&&continuous_checked.sdk_input_mapping.continuous_command_ratio);
 assert(!continuous_checked.sdk_input_mapping.direct_breakaway&&!continuous_checked.sdk_input_mapping.smoothTransitions());
 assert(sdkInputMappingJson(continuous_checked.sdk_input_mapping)==continuous_json);
 assert(sdkInputMappingJson(sdkInputMapping(continuous_json))==continuous_json);
 assert(loadBrakingModel(record.string(),fileSha256(record),"live").valid);
 for(const char*axis:{"planar","yaw"})for(const char*unused:{"rise_rate_per_s","fall_rate_per_s"}) {
   bad=continuous_record;bad["sdk_input_mapping"][axis][unused]=.6;assert(!check(bad).valid);
 }
 bad=continuous_record;bad["sdk_input_mapping"]["smoothing"]={{"startup_transition_s",1.},{"fraction_acceleration_per_s2",2.}};
 assert(!check(bad).valid);
 for(int schema:{1,2,3}){bad=continuous_record;bad["sdk_input_mapping"]["schema_version"]=schema;assert(!check(bad).valid);}
 for(const char*kind:{sdkInputMappingKind,sdkInputMappingKindV2,sdkInputMappingKindV3}) {
   bad=continuous_record;bad["sdk_input_mapping"]["kind"]=kind;assert(!check(bad).valid);
 }
 for(const auto&value:{nlohmann::json(true),nlohmann::json(4.0),nlohmann::json("4")}) {
   bad=continuous_record;bad["sdk_input_mapping"]["schema_version"]=value;assert(!check(bad).valid);
 }
 bad=continuous_record;bad["sdk_input_mapping"]["normalization"]=sdkInputMappingNormalization;assert(!check(bad).valid);
 bad=continuous_record;bad["sdk_input_mapping"]["normalization"]=sdkInputMappingNormalizationV2;assert(!check(bad).valid);
 bad=continuous_record;bad["physical_acceptance_verified"]=true;assert(!check(bad).valid);
 auto fifty_record=continuous_record;fifty_record["sdk_input_mapping"]["planar"]["maximum_fraction"]=.50;
 const auto fifty=check(fifty_record);
 assert(fifty.valid&&!fifty.physically_verified&&fifty.max_speed==continuous_checked.max_speed);
 assert(fifty.sdk_input_mapping.planar.maximum_fraction==.50);
 assert(sdkInputMappingJson(fifty.sdk_input_mapping)==fifty_record["sdk_input_mapping"]);
 bad=fifty_record;bad["sdk_input_mapping"]["planar"]["maximum_fraction"]=.500001;assert(!check(bad).valid);
 bad=fifty_record;bad["sdk_input_mapping"]["yaw"]["maximum_fraction"]=.400001;assert(!check(bad).valid);
 for(const auto&value:{nlohmann::json(true),nlohmann::json("1"),nlohmann::json(0.),nlohmann::json(-1.),nlohmann::json(3.)}) {
   bad=smoothed_record;bad["sdk_input_mapping"]["smoothing"]["startup_transition_s"]=value;assert(!check(bad).valid);
 }
 bad=smoothed_record;bad["sdk_input_mapping"]["smoothing"]["startup_transition_s"]=.1;assert(!check(bad).valid);
 bad=smoothed_record;bad["sdk_input_mapping"]["smoothing"]["fraction_acceleration_per_s2"]=1.;assert(!check(bad).valid);
 bad=smoothed_record;bad["sdk_input_mapping"]["smoothing"]["extra"]=1.;assert(!check(bad).valid);
 bad=smoothed_record;bad["sdk_input_mapping"]["smoothing"].erase("startup_transition_s");assert(!check(bad).valid);
 for(int schema:{1,2}) {
   bad=smoothed_record;bad["sdk_input_mapping"]["schema_version"]=schema;
   bad["sdk_input_mapping"]["kind"]=schema==1?sdkInputMappingKind:sdkInputMappingKindV2;
   bad["sdk_input_mapping"]["normalization"]=schema==1?sdkInputMappingNormalization:sdkInputMappingNormalizationV2;
   assert(!check(bad).valid);
 }
 assert(check(direct_record).valid); // Restore the valid file after malformed variants.
 assert(loadBrakingModel(record.string(),fileSha256(record),"live").valid);
 assert(!loadBrakingModel(record.string(),fileSha256(record),"isolated_mock").valid);
 assert(!configured_mapping.sdk_input_mapping.component_cap&&!configured_mapping.sdk_input_mapping.direct_breakaway);
 assert(responsive_checked.sdk_input_mapping.component_cap&&!responsive_checked.sdk_input_mapping.direct_breakaway);
 assert(std::string(sdkInputMappingKindFor(configured_mapping.sdk_input_mapping))==sdkInputMappingKind);
 assert(std::string(sdkInputMappingKindFor(responsive_checked.sdk_input_mapping))==sdkInputMappingKindV2);
 // Exhaustive mixed-version metadata rejects instead of downgrading the
 // policy or interpreting V3 as a legacy smooth ramp. V1 parameters are used
 // here so a cap violation cannot mask an invalid metadata combination.
 const char* kinds[]={sdkInputMappingKind,sdkInputMappingKindV2,sdkInputMappingKindV3};
 const char* normalizations[]={sdkInputMappingNormalization,sdkInputMappingNormalizationV2};
 for(int schema=1;schema<=3;++schema)for(int kind=0;kind<3;++kind)for(int normalization=0;normalization<2;++normalization) {
   auto variant=observed;auto& wire=variant["sdk_input_mapping"];
   wire["schema_version"]=schema;wire["kind"]=kinds[kind];wire["normalization"]=normalizations[normalization];
   const bool expected=kind==schema-1&&normalization==(schema==1?0:1);
   const auto parsed=check(variant);assert(parsed.valid==expected);
   if(expected) {
     assert(parsed.sdk_input_mapping.component_cap==(schema>=2));
     assert(parsed.sdk_input_mapping.direct_breakaway==(schema==3));
     assert(sdkInputMappingJson(parsed.sdk_input_mapping)==wire);
     assert(!parsed.physically_verified);
   }
 }
 for(const auto& value:{nlohmann::json(true),nlohmann::json(3.0),nlohmann::json("3"),nlohmann::json(4)}) {
   bad=direct_record;bad["sdk_input_mapping"]["schema_version"]=value;assert(!check(bad).valid);
 }
 for(const char* field:{"direct_breakaway","component_cap"}) {
   bad=direct_record;bad["sdk_input_mapping"][field]=true;assert(!check(bad).valid); // Not out-of-band policy switches.
 }
 for(const char* axis:{"planar","yaw"}) {
   bad=direct_record;bad["sdk_input_mapping"][axis]["maximum_fraction"]=.400001;assert(!check(bad).valid);
   bad=direct_record;bad["sdk_input_mapping"][axis]["effective_fraction"]=.400001;assert(!check(bad).valid);
   bad=direct_record;bad["sdk_input_mapping"][axis]["rise_rate_per_s"]=0.;assert(!check(bad).valid);
   bad=direct_record;bad["sdk_input_mapping"][axis]["fall_rate_per_s"]=1.01;assert(!check(bad).valid);
 }
 bad=direct_record;bad["physical_acceptance_verified"]=true;assert(!check(bad).valid);
 bad=direct_record;bad["parameter_basis"]="vendor_nominal_and_engineering";assert(!check(bad).valid);
 bad=direct_record;bad["fixture_only"]=true;assert(!check(bad).valid);
 bad=observed;bad.erase("sdk_input_mapping");assert(!check(bad).valid);
 bad=runtime;bad["sdk_input_mapping"]=observed["sdk_input_mapping"];assert(!check(bad).valid);
 bad=j;bad["sdk_input_mapping"]=observed["sdk_input_mapping"];assert(!check(bad).valid);
 for(const auto*field:{"schema_version","kind","normalization","planar","yaw"}) {
   bad=observed;bad["sdk_input_mapping"].erase(field);assert(!check(bad).valid);
 }
 for(const auto*field:{"kind","normalization"}) {
   bad=observed;bad["sdk_input_mapping"][field]="unknown";assert(!check(bad).valid);
 }
 for(const auto&value:{nlohmann::json(true),nlohmann::json(1.0),nlohmann::json(2)}) {
   bad=observed;bad["sdk_input_mapping"]["schema_version"]=value;assert(!check(bad).valid);
 }
 bad=observed;bad["sdk_input_mapping"]["extra"]=1;assert(!check(bad).valid);
 for(const auto*axis:{"planar","yaw"})for(const auto*field:{"effective_fraction","maximum_fraction","blend_demand_ratio","rise_rate_per_s","fall_rate_per_s"}) {
   bad=observed;bad["sdk_input_mapping"][axis].erase(field);assert(!check(bad).valid);
   for(const auto&value:{nlohmann::json(true),nlohmann::json(".2"),nlohmann::json(0.),nlohmann::json(-.1),nlohmann::json(nullptr)}) {
     bad=observed;bad["sdk_input_mapping"][axis][field]=value;assert(!check(bad).valid);
   }
   bad=observed;bad["sdk_input_mapping"][axis]["extra"]=1;assert(!check(bad).valid);
 }
 bad=observed;bad["sdk_input_mapping"]["planar"]["maximum_fraction"]=.31;assert(!check(bad).valid);
 bad=observed;bad["sdk_input_mapping"]["yaw"]["effective_fraction"]=.31;assert(!check(bad).valid);
 bad=observed;bad["sdk_input_mapping"]["yaw"]["blend_demand_ratio"]=1.;assert(!check(bad).valid);
 bad=observed;bad["sdk_input_mapping"]["yaw"]["rise_rate_per_s"]=1.01;assert(!check(bad).valid);
 bad=observed;bad["physical_acceptance_verified"]=true;assert(!check(bad).valid);
 bad=observed;bad["speed_mapping_verified"]=true;assert(!check(bad).valid);
 bad=observed;bad["parameter_basis"]="operator_measured_physical_mapping";assert(!check(bad).valid);
 {std::ofstream out(evidence);out<<"changed";}assert(!check(j).valid);
 assert(!validateAcceptance("relative.json","robot","test",cal,profile).valid);
 std::filesystem::remove(record);std::filesystem::remove(evidence);std::filesystem::remove(dir);
 std::cout<<"legacy physical evidence, separate runtime provenance, SDK V1/V2/V3 legacy and V4 static response roundtrip/version policy assertions passed\n";
}
