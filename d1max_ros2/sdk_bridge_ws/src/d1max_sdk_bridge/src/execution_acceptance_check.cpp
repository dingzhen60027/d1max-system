#include "execution_acceptance.hpp"
#include <iostream>
int main(int argc,char**argv){if(argc!=6){std::cerr<<"usage: execution_acceptance_check RECORD ROBOT_ID SDK_VERSION CALIBRATION_SHA256 ROBOT_PROFILE_SHA256\n";return 2;}
 const auto r=d1monitor::execution3::validateAcceptance(argv[1],argv[2],argv[3],argv[4],argv[5]);
 nlohmann::json out={{"valid",r.valid},{"reason",r.reason},{"max_speed_mps",r.max_speed},{"max_yaw_radps",r.max_yaw}};
 if(r.valid){std::ifstream f(argv[1]);nlohmann::json record;f>>record;out["measurements"]=record.at("measurements");
   out["execution_timing"]=record.at("execution_timing");out["stationary_evidence"]=record.at("stationary_evidence");
   out["braking_model_sha256"]=d1monitor::execution3::fileSha256(argv[1]);}
 std::cout<<out.dump()<<'\n';return r.valid?0:1;}
