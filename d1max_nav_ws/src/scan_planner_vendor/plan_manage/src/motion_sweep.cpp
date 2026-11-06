#include <plan_manage/motion_sweep.hpp>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>
namespace scan_planner {
BrakingModel BrakingModel::load(const std::string& file,const std::string& sha,const std::string& transport) {
  if(file.empty()||file.front()!='/'||sha.size()!=64)throw std::invalid_argument("absolute hashed braking record required");
  std::ifstream stream(file,std::ios::binary);if(!stream)throw std::invalid_argument("braking record not readable");
  std::string bytes((std::istreambuf_iterator<char>(stream)),{});
  if(bytes.empty()||bytes.size()>1024*1024)throw std::invalid_argument("braking record size invalid");
  unsigned char digest[EVP_MAX_MD_SIZE];unsigned int count=0;
  if(!EVP_Digest(bytes.data(),bytes.size(),digest,&count,EVP_sha256(),nullptr)||count!=32)
    throw std::runtime_error("braking record SHA failure");
  std::ostringstream hex;for(unsigned int i=0;i<count;++i)hex<<std::hex<<std::setfill('0')<<std::setw(2)<<int(digest[i]);
  if(hex.str()!=sha)throw std::invalid_argument("braking record hash mismatch");
  const auto j=nlohmann::json::parse(bytes);
  const bool runtime=j.value("record_kind","")=="runtime_motion_profile";
  if(transport=="isolated_mock") {
    if(runtime)throw std::invalid_argument("runtime profile requires live transport");
    if(j.value("schema_version",0)!=3||j.value("transport_mode","")!=transport||!j.value("fixture_only",false)||
       j.value("model","")!="reaction_braking_reachable_v1")throw std::invalid_argument("fixture braking record required");
  } else if(transport=="live") {
    if(j.value("schema_version",0)!=3||j.value("profile","")!="general_low_speed"||
       j.value("fixture_only",false)||j.value("transport_mode","")=="isolated_mock"||
       j.at("measurements").value("model","")!="reaction_braking_reachable_v1")
      throw std::invalid_argument("live braking parameter record required");
    if(runtime) {
      if(!j.at("schema_version").is_number_integer()||
          (j.value("parameter_basis","")!="vendor_nominal_and_engineering"&&
           j.value("parameter_basis","")!="vendor_nominal_engineering_and_operator_observation")||
          !j.contains("fixture_only")||!j.at("fixture_only").is_boolean()||j.at("fixture_only").get<bool>()||
          !j.contains("physical_acceptance_verified")||!j.at("physical_acceptance_verified").is_boolean()||
          j.at("physical_acceptance_verified").get<bool>()||j.value("transport_mode","")!="live")
        throw std::invalid_argument("runtime parameter provenance invalid");
      for(const char* key:{"speed_mapping_verified","stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"})
        if(j.contains(key)&&(!j.at(key).is_boolean()||j.at(key).get<bool>()))
          throw std::invalid_argument("runtime profile cannot claim physical verification");
      for(const char* key:{"robot_id","sdk_version","calibration_sha256","robot_profile_sha256"})
        if(j.value(key,"").empty())throw std::invalid_argument("runtime parameter identity missing");
      for(const char* key:{"calibration_sha256","robot_profile_sha256"}) {
        const auto value=j.at(key).get<std::string>();
        if(value.size()!=64||value.find_first_not_of("0123456789abcdef")!=std::string::npos)
          throw std::invalid_argument("runtime parameter identity hash invalid");
      }
      // The stationary policy uses configured bounds for this record kind.
      // No physical noise or braking evidence is manufactured by loading it.
      const auto& policy=j.at("stationary_evidence");
      if(policy.value("profile","")!="general_low_speed")
        throw std::invalid_argument("runtime stationary profile invalid");
      const auto bounded=[&](const char* key,double lo,double hi) {
        const auto& n=policy.at(key);
        if(!n.is_number())throw std::invalid_argument("runtime stationary bound non-numeric");
        const double value=n.get<double>();
        if(!std::isfinite(value)||value<lo||value>hi)
          throw std::invalid_argument("runtime stationary bound invalid");
        return value;
      };
      const double linear=bounded("linear_threshold_mps",.001,.05),angular=bounded("angular_threshold_radps",.001,.1);
      if(bounded("configured_static_linear_bound_mps",0.,.05)>linear||
          bounded("configured_static_angular_bound_radps",0.,.1)>angular)
        throw std::invalid_argument("runtime stationary bound outside threshold");
      for(const auto& names:{std::pair<const char*,const char*>{"measured_static_linear_bound_mps","configured_static_linear_bound_mps"},
          {"measured_static_angular_bound_radps","configured_static_angular_bound_radps"}})
        if(policy.contains(names.first)&&policy.at(names.first)!=policy.at(names.second))
          throw std::invalid_argument("runtime stationary compatibility bound mismatch");
      bounded("stationary_duration_s",1.,5.);bounded("reentry_duration_s",.6,5.);
      const double expected=bounded("mc_expected_hz",10.,200.),minimum=bounded("mc_min_hz",10.,200.),
        maximum=bounded("mc_max_hz",10.,300.);
      const auto& samples=policy.at("minimum_new_samples");
      if(!samples.is_number_integer()||samples.get<int>()<3||samples.get<int>()>512||
          expected<minimum||expected>maximum)
        throw std::invalid_argument("runtime stationary frequency or samples invalid");
    } else {
      for(const char* key:{"speed_mapping_verified","stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"})
        if(!j.value(key,false))throw std::invalid_argument("physical braking verification missing");
      for(const char* key:{"robot_id","sdk_version","calibration_sha256","robot_profile_sha256","evidence_id"})
        if(j.value(key,"").empty())throw std::invalid_argument("physical braking identity missing");
    }
    // Final robot/profile identity is independently checked by the SDK writer.
    // A configured runtime record is not physical acceptance; this geometric
    // proof never grants motion authority for either record kind.
  } else throw std::invalid_argument("explicit braking transport required");
  const auto& m=j.at("measurements");
  const auto number=[&](const char* name){const auto& n=m.at(name);if(!n.is_number())throw std::invalid_argument("non-numeric braking bound");return n.get<double>();};
  BrakingModel out{number("max_speed_mps"),number("max_yaw_radps"),number("reaction_bound_s"),
    number("stopping_distance_m"),number("stopping_yaw_rad"),number("stop_latency_bound_s"),
    number("tracking_error_bound_m"),number("heading_error_bound_rad"),sha};
  out.kinematic_profile=parseKinematicProfile(j.value("kinematic_profile",std::string("forward_yaw")));
  if(runtime) {
    out.record_kind="runtime_motion_profile";
    // Actuator fractions are owned by the SDK writer. This legacy geometric
    // model retains SI units and must never label operator tuning as measured.
    out.parameter_basis=j.at("parameter_basis").get<std::string>();
    // Keep false even though configured values are structurally usable live.
    out.physical_acceptance_verified=false;
    if(number("forward_scale_mps")<out.max_speed||number("yaw_scale_radps")<out.max_yaw||
        !std::isfinite(number("forward_scale_mps"))||!std::isfinite(number("yaw_scale_radps")))
      throw std::invalid_argument("runtime speed mapping invalid");
  }
  if(out.kinematic_profile==KinematicProfile::HolonomicXYYaw)
    out.lateral_distance=number("stopping_lateral_distance_m");
  if(runtime&&out.kinematic_profile==KinematicProfile::HolonomicXYYaw&&
      (!std::isfinite(number("lateral_metres_per_fraction"))||number("lateral_metres_per_fraction")<out.max_speed))
    throw std::invalid_argument("runtime lateral speed mapping invalid");
  // Match the SDK writer and Python safety contract. A geometrically correct
  // sweep is insufficient if its reaction bound omits queued sensor age.
  const auto& timing=j.at("execution_timing");
  double required=0.;
  for(const char* key:{"sensor_source_age_bound_s","command_pipeline_bound_s","writer_period_s","source_time_uncertainty_s"}) {
    const auto& field=timing.at(key);
    if(!field.is_number())throw std::invalid_argument("non-numeric execution timing bound");
    const double value=field.get<double>();
    if(!std::isfinite(value)||value<0.||value>1.)throw std::invalid_argument("execution timing bound invalid");
    required+=value;
  }
  out.sensor_source_age=timing.at("sensor_source_age_bound_s").get<double>();
  out.command_pipeline_bound=timing.at("command_pipeline_bound_s").get<double>();
  if(out.sensor_source_age<=0.||timing.at("command_pipeline_bound_s").get<double>()<=0.||
      out.sensor_source_age<.001||out.sensor_source_age>.6||
      timing.at("command_pipeline_bound_s").get<double>()<.1||timing.at("command_pipeline_bound_s").get<double>()>.5||
      timing.at("source_time_uncertainty_s").get<double>()>.25||
      std::abs(timing.at("writer_period_s").get<double>()-.05)>1e-9||out.reaction+1e-12<required)
    throw std::invalid_argument("reaction bound does not cover execution pipeline");
  if(!out.valid())throw std::invalid_argument("motion bounds invalid or outside configured low-speed limits");
  return out;
}
} // namespace scan_planner
