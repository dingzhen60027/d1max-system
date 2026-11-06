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
#include <limits>
#include "execution_policy.hpp"
#include "execution_kinematics.hpp"
#include "sdk_input_mapping.hpp"
namespace d1monitor::execution3 {
struct BrakingModel {
 bool valid=false;std::string sha256,reason,kinematic_profile="forward_yaw";
 double max_speed=.3,max_yaw=.5,reaction_bound=0,stopping_distance=0,stopping_lateral_distance=0,lateral_scale=0,stopping_yaw=0,stop_latency=0,tracking_error=0,heading_error=0;
 ExecutionPolicy policy;
};
// Runtime parameter validity is not a certificate of physical verification.
struct Acceptance {bool valid=false,physically_verified=false;std::string reason,parameter_basis,kinematic_profile="forward_yaw";double forward_scale=1.,lateral_scale=0.,yaw_scale=1.5,max_speed=.3,max_yaw=.5,mc_delay_bound=0.,reaction_bound=0.;ExecutionPolicy policy;SdkInputMapping sdk_input_mapping;};
inline constexpr const char*operatorMappingBasis="vendor_nominal_engineering_and_operator_observation";
inline constexpr const char*sdkInputMappingKind="smooth_deadzone_ratio_v1";
inline constexpr const char*sdkInputMappingNormalization="dominant_axis_direction_preserving_norm_cap";
inline constexpr const char*sdkInputMappingKindV2="smooth_deadzone_ratio_v2";
inline constexpr const char*sdkInputMappingNormalizationV2="dominant_axis_direction_preserving_component_cap";
inline constexpr const char*sdkInputMappingKindV3="direct_breakaway_ratio_v3";
inline constexpr const char*sdkInputMappingKindV4="continuous_amplitude_ratio_v4";
inline constexpr const char*sdkInputMappingNormalizationV4="rounded_dominant_pnorm16_direction_preserving_component_cap";
inline const char*sdkInputMappingKindFor(const SdkInputMapping&m) {
 if(m.continuous_command_ratio)return sdkInputMappingKindV4;
 return m.direct_breakaway?sdkInputMappingKindV3:(m.component_cap?sdkInputMappingKindV2:sdkInputMappingKind);
}
inline void exactMappingKeys(const nlohmann::json&j,const std::set<std::string>&keys) {
 if(!j.is_object()||j.size()!=keys.size())throw std::invalid_argument("sdk_input_mapping_fields_invalid");
 for(const auto&item:j.items())if(!keys.count(item.key()))throw std::invalid_argument("sdk_input_mapping_unknown_field");
}
inline SdkInputAxisMapping sdkInputAxisMapping(const nlohmann::json&j) {
 exactMappingKeys(j,{"effective_fraction","maximum_fraction","blend_demand_ratio","rise_rate_per_s","fall_rate_per_s"});
 const auto number=[&](const char*key){const auto&v=j.at(key);
   if(!v.is_number()||v.is_boolean())throw std::invalid_argument("sdk_input_mapping_number_required");
   return v.get<double>();};
 SdkInputAxisMapping r;r.effective_fraction=number("effective_fraction");r.maximum_fraction=number("maximum_fraction");
 r.blend_demand_ratio=number("blend_demand_ratio");r.rise_rate_per_s=number("rise_rate_per_s");r.fall_rate_per_s=number("fall_rate_per_s");r.validate();return r;
}
inline SdkInputAxisMapping sdkInputAxisMappingV4(const nlohmann::json&j,double fraction_cap=.40) {
 exactMappingKeys(j,{"effective_fraction","maximum_fraction","blend_demand_ratio"});
 const auto number=[&](const char*key){const auto&v=j.at(key);
   if(!v.is_number()||v.is_boolean())throw std::invalid_argument("sdk_input_mapping_number_required");
   return v.get<double>();};
 SdkInputAxisMapping r;r.effective_fraction=number("effective_fraction");
 r.maximum_fraction=number("maximum_fraction");r.blend_demand_ratio=number("blend_demand_ratio");
 // Rise/fall defaults belong to the legacy in-memory representation only.
 // They are neither configurable nor used by this static V4 mapping.
 r.validate(fraction_cap);return r;
}
inline SdkInputMapping sdkInputMapping(const nlohmann::json&j) {
 auto keys=std::set<std::string>{"schema_version","kind","normalization","planar","yaw"};
 if(j.contains("smoothing"))keys.insert("smoothing");
 exactMappingKeys(j,keys);
 const bool v1=j.at("schema_version")==1&&j.at("kind")==sdkInputMappingKind&&j.at("normalization")==sdkInputMappingNormalization;
 const bool v2=j.at("schema_version")==2&&j.at("kind")==sdkInputMappingKindV2&&j.at("normalization")==sdkInputMappingNormalizationV2;
 const bool v3=j.at("schema_version")==3&&j.at("kind")==sdkInputMappingKindV3&&j.at("normalization")==sdkInputMappingNormalizationV2;
 const bool v4=j.at("schema_version")==4&&j.at("kind")==sdkInputMappingKindV4&&j.at("normalization")==sdkInputMappingNormalizationV4;
 if(!j.at("schema_version").is_number_integer()||(!v1&&!v2&&!v3&&!v4))
   throw std::invalid_argument("sdk_input_mapping_schema_or_kind_invalid");
 SdkInputMapping r;r.enabled=true;r.component_cap=v2||v3||v4;r.direct_breakaway=v3;r.continuous_command_ratio=v4;
 r.planar=v4?sdkInputAxisMappingV4(j.at("planar"),.50):sdkInputAxisMapping(j.at("planar"));
 r.yaw=v4?sdkInputAxisMappingV4(j.at("yaw")):sdkInputAxisMapping(j.at("yaw"));
 if(j.contains("smoothing")) {
   if(!v3)throw std::invalid_argument("sdk_input_smoothing_requires_v3");
   const auto&s=j.at("smoothing");exactMappingKeys(s,{"startup_transition_s","fraction_acceleration_per_s2"});
   for(const auto&item:s.items())if(!item.value().is_number()||item.value().is_boolean())
     throw std::invalid_argument("sdk_input_smoothing_number_required");
   r.startup_transition_s=s.at("startup_transition_s").get<double>();
   r.fraction_acceleration_per_s2=s.at("fraction_acceleration_per_s2").get<double>();
   if(!r.smoothTransitions())throw std::invalid_argument("sdk_input_smoothing_disabled_field");
 }
 r.validate();return r;
}
inline nlohmann::json sdkInputMappingJson(const SdkInputMapping&mapping) {
 const auto axis=[&](const SdkInputAxisMapping&r){nlohmann::json result={{"effective_fraction",r.effective_fraction},
   {"maximum_fraction",r.maximum_fraction},{"blend_demand_ratio",r.blend_demand_ratio}};
   if(!mapping.continuous_command_ratio){result["rise_rate_per_s"]=r.rise_rate_per_s;result["fall_rate_per_s"]=r.fall_rate_per_s;}
   return result;};
 nlohmann::json result={{"schema_version",mapping.continuous_command_ratio?4:(mapping.direct_breakaway?3:(mapping.component_cap?2:1))},{"kind",sdkInputMappingKindFor(mapping)},
   {"normalization",mapping.continuous_command_ratio?sdkInputMappingNormalizationV4:
      (mapping.component_cap?sdkInputMappingNormalizationV2:sdkInputMappingNormalization)},
   {"planar",axis(mapping.planar)},{"yaw",axis(mapping.yaw)}};
 if(mapping.smoothTransitions())result["smoothing"]={{"startup_transition_s",mapping.startup_transition_s},
   {"fraction_acceleration_per_s2",mapping.fraction_acceleration_per_s2}};
 return result;
}
inline bool runtimeMotionProfile(const nlohmann::json&j) {
 if(!j.contains("record_kind"))return false; // Legacy physical-evidence contract.
 if(j.at("record_kind")!="runtime_motion_profile")throw std::runtime_error("runtime_profile_kind_invalid");
 const auto&basis=j.at("parameter_basis");
 if((basis!="vendor_nominal_and_engineering"&&basis!=operatorMappingBasis)||
    !j.at("fixture_only").is_boolean()||j.at("fixture_only").get<bool>()||
    !j.at("physical_acceptance_verified").is_boolean()||j.at("physical_acceptance_verified").get<bool>()||
    (j.contains("transport_mode")&&j.at("transport_mode")!="live"))
   throw std::runtime_error("runtime_profile_provenance_invalid");
 if(basis==operatorMappingBasis)sdkInputMapping(j.at("sdk_input_mapping"));
 else if(j.contains("sdk_input_mapping"))throw std::runtime_error("sdk_input_mapping_requires_operator_basis");
 for(const auto*key:{"speed_mapping_verified","stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"})
   if(j.contains(key)&&(!j.at(key).is_boolean()||j.at(key).get<bool>()))throw std::runtime_error("runtime_profile_cannot_claim_verification");
 return true;
}
inline ExecutionPolicy executionPolicy(const nlohmann::json&j,double reaction_bound) {
 const auto&t=j.at("execution_timing");const auto&s=j.at("stationary_evidence");ExecutionPolicy p;
 const bool runtime=runtimeMotionProfile(j);
 p.sensor_source_age_bound_s=t.at("sensor_source_age_bound_s").get<double>();
 p.command_pipeline_bound_s=t.at("command_pipeline_bound_s").get<double>();
 p.writer_period_s=t.at("writer_period_s").get<double>();
 p.source_time_uncertainty_s=t.at("source_time_uncertainty_s").get<double>();
 p.profile=s.at("profile").get<std::string>();
 p.velocity_metric=s.value("velocity_metric",std::string("spatial_norm_3d"));
 p.linear_threshold_mps=s.at("linear_threshold_mps").get<double>();
 p.angular_threshold_radps=s.at("angular_threshold_radps").get<double>();
 p.stationary_duration_s=s.at("stationary_duration_s").get<double>();
 p.reentry_duration_s=s.at("reentry_duration_s").get<double>();
 if(!s.at("minimum_new_samples").is_number_integer()||s.at("minimum_new_samples").get<double>()<3.||
    s.at("minimum_new_samples").get<double>()>512.)throw std::invalid_argument("stationary_sample_count_must_be_bounded_integer");
 p.minimum_new_samples=s.at("minimum_new_samples").get<uint32_t>();
 if(runtime&&(s.contains("measured_static_linear_bound_mps")||s.contains("measured_static_angular_bound_radps")))
   throw std::invalid_argument("runtime_static_noise_must_be_labelled_configured");
 p.measured_static_linear_bound_mps=s.at(runtime?"configured_static_linear_bound_mps":"measured_static_linear_bound_mps").get<double>();
 p.measured_static_angular_bound_radps=s.at(runtime?"configured_static_angular_bound_radps":"measured_static_angular_bound_radps").get<double>();
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
inline BrakingModel brakingMeasurements(const nlohmann::json&m,const std::string&kinematic_profile="forward_yaw") {
 BrakingModel r;validateKinematicProfile(kinematic_profile);r.kinematic_profile=kinematic_profile;
 if(m.at("model")!="reaction_braking_reachable_v1")throw std::runtime_error("braking_model_semantics_missing");
 auto number=[&](const char*k,double cap){const double v=m.at(k).get<double>();
   if(!std::isfinite(v)||v<=0||v>cap)throw std::runtime_error(std::string("invalid_braking_measurement:")+k);return v;};
 r.max_speed=number("max_speed_mps",.3);r.max_yaw=number("max_yaw_radps",.5);
 r.reaction_bound=number("reaction_bound_s",1.);r.stopping_distance=number("stopping_distance_m",1.);
 r.stopping_yaw=number("stopping_yaw_rad",1.);r.stop_latency=number("stop_latency_bound_s",3.);
 r.tracking_error=number("tracking_error_bound_m",.25);r.heading_error=number("heading_error_bound_rad",.5);
 if(kinematic_profile=="holonomic_xyyaw") {
   r.lateral_scale=number("lateral_metres_per_fraction",std::numeric_limits<double>::max());
   r.stopping_lateral_distance=number("stopping_lateral_distance_m",1.);
   if(r.max_speed>r.lateral_scale)throw std::runtime_error("validated_speed_exceeds_sdk_fraction");
 }
 r.valid=true;return r;
}
// Live runtime parameters are additionally validated by validateAcceptance.
// A synthetic fixture exercises this model in private transport, never live.
inline BrakingModel loadBrakingModel(const std::string&path,const std::string&expected,const std::string&mode) {
 BrakingModel r;try {
  if(!std::filesystem::path(path).is_absolute()||expected.size()!=64||fileSha256(path)!=expected)
    throw std::runtime_error("braking_record_hash_mismatch");
  std::ifstream f(path);nlohmann::json j;f>>j;
  if(j.at("schema_version")!=3)throw std::runtime_error("braking_record_schema");
  if(mode=="isolated_mock") {
   if(j.contains("record_kind")||j.contains("sdk_input_mapping"))throw std::runtime_error("runtime_motion_profile_requires_live_transport");
   if(j.at("transport_mode")!="isolated_mock"||!j.at("fixture_only").get<bool>())throw std::runtime_error("explicit_mock_braking_fixture_required");
  } else if(mode!="live"||j.value("fixture_only",false)||j.at("profile")!="general_low_speed")
    throw std::runtime_error("live_braking_record_cannot_be_fixture");
  const bool runtime=mode=="live"&&runtimeMotionProfile(j);
  if(!runtime&&j.contains("sdk_input_mapping"))throw std::runtime_error("sdk_input_mapping_requires_runtime_profile");
  r=brakingMeasurements(j.at("measurements"),j.value("kinematic_profile",std::string("forward_yaw")));r.policy=executionPolicy(j,r.reaction_bound);r.sha256=expected;
  r.reason=runtime?"runtime_braking_model_configured":"braking_model_verified";
 }catch(const std::exception&e){r=BrakingModel{};r.reason=e.what();}return r;
}
inline Acceptance validateAcceptance(const std::string&path,const std::string&robot,const std::string&sdk,const std::string&calibration,const std::string&profile) {
  Acceptance result;try{
    const auto sha=[](const std::string&s){return s.size()==64&&s.find_first_not_of("0123456789abcdef")==std::string::npos;};
    if(path.empty()||robot.empty()||sdk.empty()||!sha(calibration)||!sha(profile)||!std::filesystem::path(path).is_absolute())
      throw std::runtime_error("explicit_motion_profile_identity_and_absolute_record_required");
    std::ifstream f(path);nlohmann::json j;f>>j;
    const bool runtime=runtimeMotionProfile(j);
    if(!runtime&&j.contains("sdk_input_mapping"))throw std::runtime_error("sdk_input_mapping_requires_runtime_profile");
    if(j.at("schema_version")!=3||j.at("robot_id")!=robot||j.at("sdk_version")!=sdk||j.at("profile")!="general_low_speed"||
       j.at("calibration_sha256")!=calibration||j.at("robot_profile_sha256")!=profile)throw std::runtime_error("motion_profile_identity_mismatch");
    if(!runtime) {
      if(j.at("evidence_id").get<std::string>().empty())throw std::runtime_error("acceptance_identity_mismatch");
      for(const auto*k:{"speed_mapping_verified","stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"})
        if(!j.at(k).is_boolean()||!j.at(k).get<bool>())throw std::runtime_error(std::string("acceptance_pending:")+k);
    }
    result.kinematic_profile=j.value("kinematic_profile",std::string("forward_yaw"));validateKinematicProfile(result.kinematic_profile);
    if(runtime&&j.at("parameter_basis")==operatorMappingBasis)result.sdk_input_mapping=sdkInputMapping(j.at("sdk_input_mapping"));
    const auto&m=j.at("measurements");
    auto number=[&](const char*k,double cap){double v=m.at(k).get<double>();if(!std::isfinite(v)||v<=0||v>cap)throw std::runtime_error(std::string("invalid_measurement:")+k);return v;};
    result.forward_scale=number("forward_scale_mps",2.);result.yaw_scale=number("yaw_scale_radps",3.);
    result.max_speed=number("max_speed_mps",.3);result.max_yaw=number("max_yaw_radps",.5);
    const auto braking=brakingMeasurements(m,result.kinematic_profile);result.lateral_scale=braking.lateral_scale;
    result.reaction_bound=braking.reaction_bound;result.policy=executionPolicy(j,braking.reaction_bound);
    if(j.value("fixture_only",false))throw std::runtime_error("fixture_is_not_physical_acceptance");
    number("body_height_m",1.);result.mc_delay_bound=number("mc_delay_bound_s",.25);
    if(m.at("mc_clock_basis")!="source_delta_host_anchor_approximate")throw std::runtime_error("mc_clock_basis_mismatch");
    if(result.max_speed>result.forward_scale||result.max_yaw>result.yaw_scale)throw std::runtime_error("validated_speed_exceeds_sdk_fraction");
    if(!runtime) {
      std::set<std::string>kinds;
      for(const auto&r:j.at("records")) {
        const auto p=r.at("path").get<std::string>();const auto hash=r.at("sha256").get<std::string>();
        if(!std::filesystem::path(p).is_absolute()||hash.size()!=64||!std::filesystem::is_regular_file(p)||fileSha256(p)!=hash)
          throw std::runtime_error("evidence_file_hash_mismatch");
        kinds.insert(r.at("kind").get<std::string>());
      }
      for(const auto*k:{"geometry","speed","braking","mc_time","raw_ray"})if(!kinds.count(k))throw std::runtime_error(std::string("missing_evidence:")+k);
    }
    result.valid=true;result.physically_verified=!runtime;
    result.parameter_basis=runtime?j.at("parameter_basis").get<std::string>():"physical_evidence";
    result.reason=runtime?"runtime_motion_profile_configured_not_physically_verified":"accepted_record_verified";
  }catch(const std::exception&e){result=Acceptance{};result.reason=e.what();}return result;
}
}
