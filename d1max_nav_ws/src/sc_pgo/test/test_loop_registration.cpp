#include "loop_registration.hpp"
#include "scancontext/Scancontext.h"
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <iostream>
#include <random>
#include <stdexcept>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  auto target=std::make_shared<sc_pgo::LoopCloud>();
  std::mt19937 rng(918);
  std::uniform_real_distribution<float> u(0,1);
  for (int i=0;i<16000;++i) {
    pcl::PointXYZI p;
    const int wall=i%5;
    p.x=12*u(rng)-5; p.y=9*u(rng)-3; p.z=4*u(rng)-1;
    if(wall==0)p.z=-1;
    if(wall==1)p.x=-5;
    if(wall==2)p.y=6;
    if(wall==3){p.x=2+u(rng);p.y=1;}
    if(wall==4){p.x=-1;p.y=-1+u(rng);}
    target->push_back(p);
  }
  Eigen::Matrix4f truth=Eigen::Matrix4f::Identity();
  truth.block<3,3>(0,0)=(Eigen::AngleAxisf(1.4f,Eigen::Vector3f::UnitZ())*
    Eigen::AngleAxisf(.04f,Eigen::Vector3f::UnitY())).toRotationMatrix();
  truth.block<3,1>(0,3)=Eigen::Vector3f(.25f,-.20f,.65f);
  auto source=std::make_shared<sc_pgo::LoopCloud>();
  pcl::transformPointCloud(*target,*source,truth.inverse());
  // Verify the sign of the actual ScanContext result, not an invented yaw seed.
  SCManager sc;
  sc.setMaximumRadius(30);
  Eigen::Matrix4f pure_yaw=Eigen::Matrix4f::Identity();
  pure_yaw.block<3,3>(0,0)=Eigen::AngleAxisf(1.4f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
  sc_pgo::LoopCloud rotated;
  pcl::transformPointCloud(*target,rotated,pure_yaw.inverse());
  auto current_descriptor=sc.makeScancontext(rotated);
  auto history_descriptor=sc.makeScancontext(*target);
  auto score=sc.distanceBtnScanContext(current_descriptor,history_descriptor);
  const double current_to_history_yaw=-score.second*sc.PC_UNIT_SECTORANGLE*M_PI/180.;
  require(std::abs(std::remainder(current_to_history_yaw-1.4,2*M_PI))<.11,
          "ScanContext yaw sign disagrees with registration convention");
  Eigen::Matrix4f drifted=truth; drifted(2,3)-=4.6f;
  Eigen::Matrix4f coarse=truth; coarse.block<3,1>(0,3).setZero();
  auto result=sc_pgo::registerLoop(source,target,{drifted,coarse},{});
  require(result.accepted,"valid large-drift closure rejected");
  require((result.transform-truth).norm()<.08,"incorrect relative pose/yaw convention");
  require(std::abs(result.transform(2,3)-.65)<.04,"height was flattened");
  auto unrelated=std::make_shared<sc_pgo::LoopCloud>();
  for(int i=0;i<10000;++i){pcl::PointXYZI p;p.x=20*u(rng)-10;p.y=20*u(rng)-10;p.z=10*u(rng);unrelated->push_back(p);}
  require(!sc_pgo::registerLoop(unrelated,target,{Eigen::Matrix4f::Identity()},{}).accepted,
          "unrelated geometry accepted");
  bool refused=false;
  try {sc_pgo::RegistrationOptions invalid; invalid.fine_leaf=0;
       sc_pgo::registerLoop(source,target,{coarse},invalid);}
  catch(const std::invalid_argument&){refused=true;}
  require(refused,"invalid registration configuration admitted");
  std::cout<<"PASS: large drift, yaw, nonzero relative height, unrelated-scene rejection\n";
}
