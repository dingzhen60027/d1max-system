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
 assert(check(j).valid);assert(check(j).mc_delay_bound==.1);auto bad=j;bad.erase("records");assert(!check(bad).valid);
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
 // Actual articulated-gait speed is not the writer's command authority.
 auto spot=mock;spot["measurements"]["max_speed_mps"]=.6;spot["measurements"]["max_yaw_radps"]=.8;
 spot["measurements"]["stopping_distance_m"]=.5;spot["measurements"]["stop_latency_bound_s"]=3.;
 spot["isolated_platform_model"]={{"schema",1},{"kind","official_spot_physx"},
   {"command_max_speed_mps",.15},{"command_max_yaw_radps",.3},
   {"reachable_max_speed_mps",.6},{"reachable_max_yaw_radps",.8},
   {"source_scope","isolated_simulation_physx_measured_model"}};
 auto loadMock=[&](const nlohmann::json& item,const std::string& mode="isolated_mock"){
   {std::ofstream out(record);out<<item;}
   return loadBrakingModel(record.string(),fileSha256(record),mode);
 };
 const auto plant=loadMock(spot);assert(plant.valid&&plant.isolated_spot_model);
 assert(plant.max_speed==.6&&plant.max_yaw==.8&&plant.command_max_speed==.15&&plant.command_max_yaw==.3);
 assert(plant.stop_latency==3.&&plant.stopping_distance==.5);
 assert(!check(spot).valid);assert(!loadMock(spot,"live").valid);
 // A stripped older marker cannot turn a new isolated reference record into
 // physical acceptance or a live braking calibration.
 bad=mock;bad["isolated_full_xyz_reference_model"]={{"schema",1},{"kind","official_spot_physx"}};
 assert(!check(bad).valid);assert(!loadMock(bad,"live").valid);
 bad=spot;bad["fixture_only"]=false;bad["transport_mode"]="live";
 assert(!check(bad).valid);assert(!loadMock(bad,"live").valid);
 bad=spot;bad.erase("isolated_platform_model");assert(!loadMock(bad).valid);
 for(const auto& pair:std::vector<std::pair<std::string,nlohmann::json>>{
     {"schema",true},{"schema",1.},{"schema",2},{"kind","official_go2_physx"},{"source_scope","live"},
     {"command_max_speed_mps",.301},{"command_max_yaw_radps",.501},
     {"command_max_speed_mps",0.},{"command_max_yaw_radps",true},
     {"reachable_max_speed_mps",.601},{"reachable_max_yaw_radps",.801},
     {"reachable_max_speed_mps",.59},{"reachable_max_yaw_radps","0.8"}}) {
   bad=spot;bad["isolated_platform_model"][pair.first]=pair.second;assert(!loadMock(bad).valid);
 }
 for(const auto*field:{"schema","kind","command_max_speed_mps","command_max_yaw_radps",
     "reachable_max_speed_mps","reachable_max_yaw_radps","source_scope"}) {
   bad=spot;bad["isolated_platform_model"].erase(field);assert(!loadMock(bad).valid);
 }
 bad=spot;bad["isolated_platform_model"]["extra_permission"]=true;assert(!loadMock(bad).valid);
 for(const auto& pair:std::vector<std::pair<std::string,double>>{
     {"stop_latency_bound_s",3.01},{"stopping_distance_m",1.01},{"reaction_bound_s",1.01}}) {
   bad=spot;bad["measurements"][pair.first]=pair.second;assert(!loadMock(bad).valid);
 }
 {std::ofstream out(evidence);out<<"changed";}assert(!check(j).valid);
 assert(!validateAcceptance("relative.json","robot","test",cal,profile).valid);
 std::filesystem::remove(record);std::filesystem::remove(evidence);std::filesystem::remove(dir);
 std::cout<<"physical evidence, measured stop/rate policy and full reaction budget assertions passed\n";
}
