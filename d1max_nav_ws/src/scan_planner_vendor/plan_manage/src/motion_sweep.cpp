#include <plan_manage/motion_sweep.hpp>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <fstream>
#include <iomanip>
#include <sstream>
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
  if(j.contains("isolated_platform_model")&&transport!="isolated_mock")
    throw std::invalid_argument("isolated platform cannot authorize real robot");
  if(transport=="isolated_mock") {
    if(j.value("schema_version",0)!=3||j.value("transport_mode","")!=transport||!j.value("fixture_only",false)||
       j.value("model","")!="reaction_braking_reachable_v1")throw std::invalid_argument("fixture braking record required");
  } else if(transport=="live") {
    if(j.value("schema_version",0)!=3||j.value("profile","")!="general_low_speed"||
       j.value("fixture_only",false)||j.value("transport_mode","")=="isolated_mock"||
       j.at("measurements").value("model","")!="reaction_braking_reachable_v1")
      throw std::invalid_argument("physical braking record required");
    for(const char* key:{"speed_mapping_verified","stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"})
      if(!j.value(key,false))throw std::invalid_argument("physical braking verification missing");
    for(const char* key:{"robot_id","sdk_version","calibration_sha256","robot_profile_sha256","evidence_id"})
      if(j.value(key,"").empty())throw std::invalid_argument("physical braking identity missing");
    // Final robot/profile/physical-acceptance identity is independently checked
    // by the SDK writer. This geometric proof never grants motion authority.
  } else throw std::invalid_argument("explicit braking transport required");
  const auto& m=j.at("measurements");
  const auto number=[&](const char* name){const auto& n=m.at(name);if(!n.is_number())throw std::invalid_argument("non-numeric braking bound");return n.get<double>();};
  BrakingModel out{number("max_speed_mps"),number("max_yaw_radps"),number("reaction_bound_s"),
    number("stopping_distance_m"),number("stopping_yaw_rad"),number("stop_latency_bound_s"),
    number("tracking_error_bound_m"),number("heading_error_bound_rad"),sha};
  if(j.contains("isolated_platform_model")) {
    const auto& p=j.at("isolated_platform_model");
    if(!j.at("fixture_only").is_boolean()||j.at("fixture_only")!=true||!p.is_object()||p.size()!=7||
        !p.at("schema").is_number_integer()||p.at("schema")!=1||p.at("kind")!="official_spot_physx"||
        p.at("source_scope")!="isolated_simulation_physx_measured_model")
      throw std::invalid_argument("invalid isolated platform model");
    const auto bounded=[&](const char* name,double cap) {
      const auto& field=p.at(name);if(!field.is_number())throw std::invalid_argument("nonnumeric isolated platform bound");
      const double value=field.get<double>();
      if(!std::isfinite(value)||value<=0.||value>cap)throw std::invalid_argument("invalid isolated platform bound");
      return value;
    };
    out.command_max_speed=bounded("command_max_speed_mps",.3);
    out.command_max_yaw=bounded("command_max_yaw_radps",.5);
    if(bounded("reachable_max_speed_mps",.6)!=out.max_speed||bounded("reachable_max_yaw_radps",.8)!=out.max_yaw)
      throw std::invalid_argument("isolated platform measurement mismatch");
    out.isolated_spot_model=true;
  }
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
  if(out.sensor_source_age<=0.||timing.at("command_pipeline_bound_s").get<double>()<=0.||
      out.sensor_source_age<.001||out.sensor_source_age>.6||
      timing.at("command_pipeline_bound_s").get<double>()<.1||timing.at("command_pipeline_bound_s").get<double>()>.5||
      timing.at("source_time_uncertainty_s").get<double>()>.25||
      std::abs(timing.at("writer_period_s").get<double>()-.05)>1e-9||out.reaction+1e-12<required)
    throw std::invalid_argument("reaction bound does not cover execution pipeline");
  if(!out.valid())throw std::invalid_argument("braking bounds invalid or outside first-acceptance limits");
  return out;
}
} // namespace scan_planner
