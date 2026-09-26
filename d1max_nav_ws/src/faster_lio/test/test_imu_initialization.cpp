#include <gtest/gtest.h>
#include "imu_processing.hpp"

namespace {
faster_lio::common::MeasureGroup measurements(double start, int count) {
  faster_lio::common::MeasureGroup result;
  for (int i=0; i<count; ++i) {
    auto imu=std::make_shared<sensor_msgs::msg::Imu>();
    const auto ns=static_cast<int64_t>(std::llround((start+i*.005)*1e9));
    imu->header.stamp.sec=ns/1000000000;
    imu->header.stamp.nanosec=ns%1000000000;
    imu->linear_acceleration.x=.15;
    imu->linear_acceleration.y=-.20;
    imu->linear_acceleration.z=9.8;
    imu->angular_velocity.x=.005;
    imu->angular_velocity.y=.002;
    imu->angular_velocity.z=-.001;
    result.imu_.push_back(imu);
  }
  return result;
}
}

TEST(ImuInitialization, StableWindowInitializesOnceAndResetDiscardsOldSamples) {
  using namespace faster_lio;
  ImuProcess process;
  process.stationary_initialization_enabled=true;
  process.gravity_aligned_world=true;
  process.initialization_samples=300;
  process.debug_file_enabled=false;
  process.SetAccCov(common::V3D::Constant(.1));
  process.SetGyrCov(common::V3D::Constant(.1));
  esekfom::esekf<state_ikfom,12,input_ikfom> filter;
  PointCloudType::Ptr cloud(new PointCloudType);
  process.Process(measurements(0.,400),filter,cloud);
  EXPECT_FALSE(process.Initialized());
  process.Process(measurements(2.,1),filter,cloud);
  ASSERT_TRUE(process.Initialized());
  EXPECT_GE(process.InitializationSamples(),300);
  const auto state=filter.get_x();
  const Eigen::Vector3d up=state.rot*Eigen::Vector3d(.15,-.20,9.8).normalized();
  EXPECT_LT((up-Eigen::Vector3d::UnitZ()).norm(),1e-9);
  EXPECT_NEAR(state.grav[0],0.,1e-9);
  EXPECT_NEAR(state.grav[1],0.,1e-9);
  EXPECT_LT(state.grav[2],-9.8);
  EXPECT_LT((state.bg-Eigen::Vector3d(.005,.002,-.001)).norm(),1e-9);
  EXPECT_LT((process.gravity_rotation_-Eigen::Matrix3d::Identity()).norm(),1e-9);
  process.Reset();
  EXPECT_FALSE(process.Initialized());
  EXPECT_TRUE(process.initialization_gate.samples.empty());
  process.Process(measurements(5.,200),filter,cloud);
  EXPECT_FALSE(process.Initialized());
  process.Process(measurements(6.,201),filter,cloud);
  EXPECT_TRUE(process.Initialized());
}

namespace {
using namespace faster_lio;
using Filter=esekfom::esekf<state_ikfom,12,input_ikfom>;
sensor_msgs::msg::Imu::SharedPtr endpoint(double t,double yaw_rate) {
  auto m=std::make_shared<sensor_msgs::msg::Imu>();
  const auto ns=static_cast<int64_t>(std::llround(t*1e9));
  m->header.stamp.sec=ns/1000000000; m->header.stamp.nanosec=ns%1000000000;
  m->angular_velocity.z=yaw_rate; m->linear_acceleration.z=9.81;
  return m;
}
faster_lio::common::MeasureGroup scan(double begin,double end,
    std::initializer_list<std::pair<double,double>> endpoints) {
  faster_lio::common::MeasureGroup m;
  m.lidar_bag_time_=begin; m.lidar_end_time_=end;
  for(const auto& item:endpoints)m.imu_.push_back(endpoint(item.first,item.second));
  for(int i=0;i<5;++i){
    PointType p; p.x=2.;p.y=0.;p.z=0.;
    p.curvature=(end-begin)*1000.*i/4.;m.lidar_->push_back(p);
  }
  return m;
}
void initialize_bounded(faster_lio::ImuProcess& p,Filter& f,double t=10.) {
  double eps[23];std::fill(eps,eps+23,.001);
  f.init_dyn_share(get_f,df_dx,df_dw,
      [](state_ikfom&,esekfom::dyn_share_datastruct<double>&){},3,eps);
  p.bounded_scan_endpoints=true;p.debug_file_enabled=false;p.initialization_samples=1;
  p.SetAccCov(faster_lio::common::V3D::Constant(.1));
  p.SetGyrCov(faster_lio::common::V3D::Constant(.1));
  PointCloudType::Ptr output(new PointCloudType);
  p.Process(scan(t-.005,t,{{t-.005,0.},{t,0.}}),f,output);
  ASSERT_TRUE(p.Initialized());
}
double yaw(const Filter& f) {
  // get_x() is not const in the vendor API.
  auto& mutable_f=const_cast<Filter&>(f);
  const auto r=mutable_f.get_x().rot.toRotationMatrix();
  return std::atan2(r(1,0),r(0,0));
}
}

TEST(BoundedImuIntegration, RealRightEndpointClipsAtScanEndAndContinuesNextFrame) {
  faster_lio::ImuProcess p;Filter f;initialize_bounded(p,f);
  PointCloudType::Ptr output(new PointCloudType);
  // True yaw rate increases linearly from 0 to 1rad/s over80ms. The scan
  // ends60ms in: integral is .5*(1/.08)*.06², not 0 or .04 plus abs(.02).
  p.Process(scan(10.,10.06,{{10.,0.},{10.08,1.}}),f,output);
  ASSERT_EQ(output->size(),5u);EXPECT_NEAR(yaw(f),.0225,1e-9);
  // Same physical interval is split across scans. Keeping the future10.08
  // sample as last_imu_ would reverse the next input order and fail this test.
  p.Process(scan(10.06,10.10,{{10.,0.},{10.08,1.},{10.12,1.5}}),f,output);
  ASSERT_EQ(output->size(),5u);EXPECT_NEAR(yaw(f),.0625,1e-9);
  p.Process(scan(10.10,10.12,{{10.08,1.},{10.12,1.5}}),f,output);
  ASSERT_EQ(output->size(),5u);EXPECT_NEAR(yaw(f),.09,1e-9);
}

TEST(BoundedImuIntegration, MissingRightEndpointCannotExtrapolateOrMutateFilter) {
  faster_lio::ImuProcess p;Filter f;initialize_bounded(p,f);
  PointCloudType::Ptr output(new PointCloudType);
  const auto before=f.get_P();
  p.Process(scan(10.,10.06,{{10.,0.},{10.04,1.}}),f,output);
  EXPECT_TRUE(output->empty());EXPECT_NEAR(yaw(f),0.,1e-12);
  EXPECT_EQ((f.get_P()-before).norm(),0.);
  // A real right endpoint arriving later permits retry, without reset.
  p.Process(scan(10.,10.06,{{10.,0.},{10.08,1.}}),f,output);
  EXPECT_EQ(output->size(),5u);EXPECT_NEAR(yaw(f),.0225,1e-9);
}

TEST(BoundedImuIntegration, NoiseScaleActuallyIncreasesPredictCovariance) {
  faster_lio::ImuProcess a,b;Filter fa,fb;initialize_bounded(a,fa);initialize_bounded(b,fb);
  a.integration_noise_scale=1.;b.integration_noise_scale=25.;
  PointCloudType::Ptr output(new PointCloudType);
  const auto m=scan(10.,10.06,{{10.,0.},{10.08,1.}});
  a.Process(m,fa,output);b.Process(m,fb,output);
  EXPECT_NEAR(yaw(fa),yaw(fb),1e-12);
  EXPECT_GT(fb.get_P().trace(),fa.get_P().trace());
  EXPECT_GT(fb.get_P()(3,3),fa.get_P()(3,3));
}

TEST(BoundedImuIntegration, UnixEpochPrecisionUsesSameClockAsRobustSync) {
  const double t=1790242500.;faster_lio::ImuProcess p;Filter f;initialize_bounded(p,f,t);
  PointCloudType::Ptr output(new PointCloudType);
  p.Process(scan(t,t+.06,{{t,0.},{t+.08,1.}}),f,output);
  ASSERT_EQ(output->size(),5u);EXPECT_NEAR(yaw(f),.0225,2e-6);
}

TEST(BoundedImuIntegration, OrdinaryMappingDoesNotEnableNewBranch) {
  faster_lio::ImuProcess p;EXPECT_FALSE(p.bounded_scan_endpoints);
}
