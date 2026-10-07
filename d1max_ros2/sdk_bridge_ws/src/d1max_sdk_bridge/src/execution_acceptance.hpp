#pragma once
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include "execution_policy.hpp"
namespace d1monitor::execution3 {
struct BrakingModel {
 bool valid=false;std::string sha256,reason;
 double max_speed=.3,max_yaw=.5,reaction_bound=0,stopping_distance=0,stopping_yaw=0,stop_latency=0,tracking_error=0,heading_error=0;
 double command_max_speed=.3,command_max_yaw=.5;
 bool isolated_spot_model=false;
 ExecutionPolicy policy;
};
struct Acceptance {bool valid=false;std::string reason;double forward_scale=1.,yaw_scale=1.5,max_speed=.3,max_yaw=.5,mc_delay_bound=0.,reaction_bound=0.;ExecutionPolicy policy;};
inline ExecutionPolicy executionPolicy(const nlohmann::json&j,double reaction_bound) {
 const auto&t=j.at("execution_timing");const auto&s=j.at("stationary_evidence");ExecutionPolicy p;
 p.sensor_source_age_bound_s=t.at("sensor_source_age_bound_s").get<double>();
 p.command_pipeline_bound_s=t.at("command_pipeline_bound_s").get<double>();
 p.writer_period_s=t.at("writer_period_s").get<double>();
 p.source_time_uncertainty_s=t.at("source_time_uncertainty_s").get<double>();
 p.profile=s.at("profile").get<std::string>();
 p.linear_threshold_mps=s.at("linear_threshold_mps").get<double>();
 p.angular_threshold_radps=s.at("angular_threshold_radps").get<double>();
 p.stationary_duration_s=s.at("stationary_duration_s").get<double>();
 p.reentry_duration_s=s.at("reentry_duration_s").get<double>();
 if(!s.at("minimum_new_samples").is_number_integer()||s.at("minimum_new_samples").get<double>()<3.||
    s.at("minimum_new_samples").get<double>()>512.)throw std::invalid_argument("stationary_sample_count_must_be_bounded_integer");
 p.minimum_new_samples=s.at("minimum_new_samples").get<uint32_t>();
 p.measured_static_linear_bound_mps=s.at("measured_static_linear_bound_mps").get<double>();
 p.measured_static_angular_bound_radps=s.at("measured_static_angular_bound_radps").get<double>();
 p.mc_expected_hz=s.at("mc_expected_hz").get<double>();p.mc_min_hz=s.at("mc_min_hz").get<double>();
 p.mc_max_hz=s.at("mc_max_hz").get<double>();p.validate(reaction_bound);return p;
}
inline std::string fileSha256(const std::filesystem::path&p) {
  std::ifstream f(p,std::ios::binary);if(!f)throw std::runtime_error("evidence_file_unreadable");
  SHA256_CTX ctx;SHA256_Init(&ctx);char data[16384];while(f){f.read(data,sizeof(data));SHA256_Update(&ctx,data,f.gcount());}
  if(f.bad())throw std::runtime_error("evidence_file_read_failed");
  unsigned char hash[SHA256_DIGEST_LENGTH];SHA256_Final(hash,&ctx);std::ostringstream out;
  for(auto c:hash)out<<std::hex<<std::setfill('0')<<std::setw(2)<<static_cast<int>(c);return out.str();
}
inline BrakingModel brakingMeasurements(const nlohmann::json&m,double speed_cap=.3,double yaw_cap=.5) {
 BrakingModel r;
 if(m.at("model")!="reaction_braking_reachable_v1")throw std::runtime_error("braking_model_semantics_missing");
 auto number=[&](const char*k,double cap){const double v=m.at(k).get<double>();
   if(!std::isfinite(v)||v<=0||v>cap)throw std::runtime_error(std::string("invalid_braking_measurement:")+k);return v;};
 r.max_speed=number("max_speed_mps",speed_cap);r.max_yaw=number("max_yaw_radps",yaw_cap);
 r.command_max_speed=r.max_speed;r.command_max_yaw=r.max_yaw;
 r.reaction_bound=number("reaction_bound_s",1.);r.stopping_distance=number("stopping_distance_m",1.);
 r.stopping_yaw=number("stopping_yaw_rad",1.);r.stop_latency=number("stop_latency_bound_s",3.);
 r.tracking_error=number("tracking_error_bound_m",.25);r.heading_error=number("heading_error_bound_rad",.5);
 r.valid=true;return r;
}
// Live is additionally accepted only by validateAcceptance below. A synthetic
// fixture may exercise this exact model in private transport, never in live.
inline BrakingModel loadBrakingModel(const std::string&path,const std::string&expected,const std::string&mode) {
 BrakingModel r;try {
  if(!std::filesystem::path(path).is_absolute()||expected.size()!=64||fileSha256(path)!=expected)
    throw std::runtime_error("braking_record_hash_mismatch");
  std::ifstream f(path);nlohmann::json j;f>>j;
  if(j.at("schema_version")!=3)throw std::runtime_error("braking_record_schema");
  if((j.contains("isolated_platform_model")||j.contains("isolated_full_xyz_reference_model"))&&mode!="isolated_mock")
    throw std::runtime_error("isolated_platform_cannot_authorize_real_robot");
  if(mode=="isolated_mock") {
   if(j.at("transport_mode")!="isolated_mock"||!j.at("fixture_only").get<bool>())throw std::runtime_error("explicit_mock_braking_fixture_required");
  } else if(mode!="live"||j.value("fixture_only",false)||j.at("profile")!="general_low_speed")
    throw std::runtime_error("live_braking_record_cannot_be_fixture");
  if(j.contains("isolated_platform_model")) {
   const auto&p=j.at("isolated_platform_model");
   if(!j.at("fixture_only").is_boolean()||j.at("fixture_only")!=true||!p.is_object()||p.size()!=7||
      !p.at("schema").is_number_integer()||p.at("schema")!=1||p.at("kind")!="official_spot_physx"||
      p.at("source_scope")!="isolated_simulation_physx_measured_model")
     throw std::runtime_error("invalid_isolated_platform_model");
   auto bounded=[&](const char*name,double cap){const auto&field=p.at(name);
     if(!field.is_number())throw std::runtime_error("nonnumeric_isolated_platform_bound");
     const double v=field.get<double>();
     if(!std::isfinite(v)||v<=0.||v>cap)throw std::runtime_error("invalid_isolated_platform_bound");
     return v;};
   r=brakingMeasurements(j.at("measurements"),.65,.8);
   r.command_max_speed=bounded("command_max_speed_mps",.3);
   r.command_max_yaw=bounded("command_max_yaw_radps",.5);
   if(bounded("reachable_max_speed_mps",.65)!=r.max_speed||bounded("reachable_max_yaw_radps",.8)!=r.max_yaw||
      r.command_max_speed>r.max_speed||r.command_max_yaw>r.max_yaw)
     throw std::runtime_error("isolated_platform_measurement_mismatch");
   r.isolated_spot_model=true;
  } else r=brakingMeasurements(j.at("measurements"));
  r.policy=executionPolicy(j,r.reaction_bound);r.sha256=expected;r.reason="braking_model_verified";
 }catch(const std::exception&e){r=BrakingModel{};r.reason=e.what();}return r;
}
inline Acceptance validateAcceptance(const std::string&path,const std::string&robot,const std::string&sdk,const std::string&calibration,const std::string&profile) {
  Acceptance result;try{
    const auto sha=[](const std::string&s){return s.size()==64&&s.find_first_not_of("0123456789abcdef")==std::string::npos;};
    if(path.empty()||robot.empty()||sdk.empty()||!sha(calibration)||!sha(profile)||!std::filesystem::path(path).is_absolute())
      throw std::runtime_error("explicit_acceptance_identity_and_absolute_record_required");
    std::ifstream f(path);nlohmann::json j;f>>j;
    if(j.contains("isolated_platform_model")||j.contains("isolated_full_xyz_reference_model"))throw std::runtime_error("isolated_platform_is_not_physical_acceptance");
    if(j.at("schema_version")!=3||j.at("robot_id")!=robot||j.at("sdk_version")!=sdk||j.at("profile")!="general_low_speed"||
       j.at("calibration_sha256")!=calibration||j.at("robot_profile_sha256")!=profile||j.at("evidence_id").get<std::string>().empty())throw std::runtime_error("acceptance_identity_mismatch");
    for(const auto*k:{"speed_mapping_verified","stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"})
      if(!j.at(k).is_boolean()||!j.at(k).get<bool>())throw std::runtime_error(std::string("acceptance_pending:")+k);
    const auto&m=j.at("measurements");
    auto number=[&](const char*k,double cap){double v=m.at(k).get<double>();if(!std::isfinite(v)||v<=0||v>cap)throw std::runtime_error(std::string("invalid_measurement:")+k);return v;};
    result.forward_scale=number("forward_scale_mps",2.);result.yaw_scale=number("yaw_scale_radps",3.);
    result.max_speed=number("max_speed_mps",.3);result.max_yaw=number("max_yaw_radps",.5);
    const auto braking=brakingMeasurements(m);result.reaction_bound=braking.reaction_bound;result.policy=executionPolicy(j,braking.reaction_bound);
    if(j.value("fixture_only",false))throw std::runtime_error("fixture_is_not_physical_acceptance");
    number("body_height_m",1.);result.mc_delay_bound=number("mc_delay_bound_s",.25);
    if(m.at("mc_clock_basis")!="source_delta_host_anchor_approximate")throw std::runtime_error("mc_clock_basis_mismatch");
    if(result.max_speed>result.forward_scale||result.max_yaw>result.yaw_scale)throw std::runtime_error("validated_speed_exceeds_sdk_fraction");
    std::set<std::string>kinds;
    for(const auto&r:j.at("records")) {
      const auto p=r.at("path").get<std::string>();const auto hash=r.at("sha256").get<std::string>();
      if(!std::filesystem::path(p).is_absolute()||hash.size()!=64||!std::filesystem::is_regular_file(p)||fileSha256(p)!=hash)
        throw std::runtime_error("evidence_file_hash_mismatch");
      kinds.insert(r.at("kind").get<std::string>());
    }
    for(const auto*k:{"geometry","speed","braking","mc_time","raw_ray"})if(!kinds.count(k))throw std::runtime_error(std::string("missing_evidence:")+k);
    result.valid=true;result.reason="accepted_record_verified";
  }catch(const std::exception&e){result=Acceptance{};result.reason=e.what();}return result;
}
}
