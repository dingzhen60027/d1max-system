#pragma once

// An isolated reference/measurement domain, never command or braking authority.
// Both native planning and tracking consume the SAME hashed record. Missing
// metadata keeps the historical bounds; malformed metadata must fail closed.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>

namespace d1max_planning_interfaces {
class IsolatedReferenceModel {
  double reference_speed_,travel_speed_,observed_speed_,command_speed_,command_yaw_;
  std::string record_sha_,evidence_sha_;
  IsolatedReferenceModel(double reference,double travel,double observed,double command,double yaw,
      std::string record_sha,std::string evidence_sha):reference_speed_(reference),travel_speed_(travel),
      observed_speed_(observed),command_speed_(command),command_yaw_(yaw),
      record_sha_(std::move(record_sha)),evidence_sha_(std::move(evidence_sha)) {}
 public:
  double referenceSpeed()const{return reference_speed_;}
  double measuredTravelSpeed()const{return travel_speed_;}
  double observedSpeed()const{return observed_speed_;}
  double commandSpeed()const{return command_speed_;}
  double commandYaw()const{return command_yaw_;}
  const std::string& recordSha()const{return record_sha_;}
  const std::string& evidenceSha()const{return evidence_sha_;}
  static bool shaValid(const std::string& sha) {
    if(sha.size()!=64)return false;
    for(const auto c:sha)if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))return false;
    return true;
  }
  static std::optional<IsolatedReferenceModel> parse(const nlohmann::json& record,
      const std::string& sha,const std::string& mode) {
    if(!record.contains("isolated_full_xyz_reference_model"))return {};
    if(!shaValid(sha)||mode!="isolated_mock"||record.at("transport_mode")!="isolated_mock"||
        !record.at("schema_version").is_number_integer()||record.at("schema_version")!=3||
        !record.at("fixture_only").is_boolean()||record.at("fixture_only")!=true||
        record.at("model")!="reaction_braking_reachable_v1")
      throw std::invalid_argument("isolated_reference_fixture_required");
    const auto& platform=record.at("isolated_platform_model");
    const auto& reference=record.at("isolated_full_xyz_reference_model");
    const auto marker=[](const nlohmann::json& p) {
      return p.is_object()&&p.size()==7&&p.at("schema").is_number_integer()&&p.at("schema")==1&&
        p.at("kind")=="official_spot_physx"&&
        p.at("source_scope")=="isolated_simulation_physx_measured_model";
    };
    if(!marker(platform)||!marker(reference))throw std::invalid_argument("invalid_isolated_reference_marker");
    const auto bound=[](const nlohmann::json& p,const char* key,double maximum) {
      const auto& field=p.at(key);
      if(!field.is_number())throw std::invalid_argument("nonnumeric_isolated_reference_bound");
      const double value=field.get<double>();
      if(!std::isfinite(value)||value<=0.||value>maximum)
        throw std::invalid_argument("invalid_isolated_reference_bound");
      return value;
    };
    const double command=bound(platform,"command_max_speed_mps",.3);
    const double yaw=bound(platform,"command_max_yaw_radps",.5);
    const double reach=bound(platform,"reachable_max_speed_mps",.6);
    const double reach_yaw=bound(platform,"reachable_max_yaw_radps",.8);
    if(command>reach||yaw>reach_yaw||record.at("measurements").at("max_speed_mps")!=reach||
        record.at("measurements").at("max_yaw_radps")!=reach_yaw)
      throw std::invalid_argument("isolated_reference_platform_mismatch");
    const double speed=bound(reference,"reference_max_speed_mps",.5);
    const double travel=bound(reference,"measured_travel_max_speed_mps",.5);
    const double observed=bound(reference,"observed_max_full_xyz_speed_mps",.5);
    const auto& evidence=reference.at("evidence_sha256");
    if(!evidence.is_string()||!shaValid(evidence.get<std::string>())||
        observed>speed||observed>travel||command>speed||command>travel)
      throw std::invalid_argument("isolated_reference_evidence_domain_invalid");
    return IsolatedReferenceModel(speed,travel,observed,command,yaw,sha,evidence.get<std::string>());
  }
  static std::optional<IsolatedReferenceModel> load(const std::string& file,
      const std::string& sha,const std::string& mode) {
    if(file.empty()&&sha.empty())return {};
    if(file.empty()||file.front()!='/'||!shaValid(sha))
      throw std::invalid_argument("absolute_hashed_reference_record_required");
    std::ifstream stream(file,std::ios::binary);
    if(!stream)throw std::invalid_argument("reference_record_not_readable");
    std::string bytes;bytes.resize(1024*1024+1);
    stream.read(bytes.data(),bytes.size());bytes.resize(static_cast<std::size_t>(stream.gcount()));
    if(bytes.empty()||bytes.size()>1024*1024)throw std::invalid_argument("reference_record_size_invalid");
    unsigned char digest[EVP_MAX_MD_SIZE];unsigned int count=0;
    if(!EVP_Digest(bytes.data(),bytes.size(),digest,&count,EVP_sha256(),nullptr)||count!=32)
      throw std::runtime_error("reference_record_sha_failure");
    std::ostringstream hex;
    for(unsigned i=0;i<count;++i)hex<<std::hex<<std::setfill('0')<<std::setw(2)<<int(digest[i]);
    if(hex.str()!=sha)throw std::invalid_argument("reference_record_hash_mismatch");
    return parse(nlohmann::json::parse(bytes),sha,mode);
  }
};
}  // namespace d1max_planning_interfaces
