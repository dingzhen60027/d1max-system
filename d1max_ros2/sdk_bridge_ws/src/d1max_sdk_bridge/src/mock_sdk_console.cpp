// TEST-ONLY SDK replacement. No socket API, no vendor library, loopback argument
// mandatory. Linked only into the non-installed sdk_console_bridge_mock target.
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include "robot_sdk/sdk_client.hpp"
namespace robot_sdk {
class SDKClient::Impl {
 public:
  std::mutex mutex;std::shared_ptr<IDataCallback> data;std::shared_ptr<IControlCallback> ack;
  std::atomic<bool> connected{false},run{true};RobotState state{};std::thread worker;
  std::chrono::steady_clock::time_point stand_until{};
  Impl(){state.motion_status=static_cast<MotionStatus>(2);state.sport_mode=static_cast<SportMode>(1);state.control_source=static_cast<CtrlSource>(1);state.speed_level=static_cast<SpeedLevel>(1);state.software_emergency_status=static_cast<EmergencyStatus>(1);state.hardware_emergency_status=static_cast<EmergencyStatus>(1);
    worker=std::thread([this]{while(run){std::this_thread::sleep_for(std::chrono::milliseconds(200));if(!connected)continue;RobotState copy;std::shared_ptr<IDataCallback> cb;{std::lock_guard<std::mutex> lock(mutex);if(static_cast<int>(state.motion_status)==1&&std::chrono::steady_clock::now()>stand_until)state.motion_status=static_cast<MotionStatus>(static_cast<int>(state.sport_mode)+4);copy=state;cb=data;}if(cb)cb->OnRobotStateData(copy);}});
  }
  ~Impl(){run=false;if(worker.joinable())worker.join();}
};
SDKClient::SDKClient(ErrorHandler,ConnectionConfig,TransportProtocol):pImpl_(std::make_shared<Impl>()){}
SDKClient::~SDKClient()=default;
std::error_code SDKClient::Connect(std::string ip,std::string,bool,ConnectHandler){if(ip!="127.0.0.1")throw std::runtime_error("MOCK SDK: robot address forbidden");pImpl_->connected=true;return {};}
std::error_code SDKClient::Disconnect(bool,DisConnectHandler){pImpl_->connected=false;return {};}
bool SDKClient::IsConnected() const{return pImpl_->connected;}
void SDKClient::SetDataCallback(std::shared_ptr<IDataCallback> c){std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->data=std::move(c);}
void SDKClient::SetControlCallback(std::shared_ptr<IControlCallback> c){std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->ack=std::move(c);}
std::error_code SDKClient::TakeControl(int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.control_source=static_cast<CtrlSource>(2);cb=pImpl_->ack;}if(cb)cb->OnTakeControlAck(TakeControlAck{});return {};}
std::error_code SDKClient::ReleaseControl(int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.control_source=static_cast<CtrlSource>(1);cb=pImpl_->ack;}if(cb)cb->OnReleaseControlAck(ReleaseControlAck{});return {};}
std::error_code SDKClient::StandUp(int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.motion_status=static_cast<MotionStatus>(1);pImpl_->stand_until=std::chrono::steady_clock::now()+std::chrono::milliseconds(1200);cb=pImpl_->ack;}if(cb)cb->OnStandUp();return {};}
std::error_code SDKClient::LieDown(int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.motion_status=static_cast<MotionStatus>(2);cb=pImpl_->ack;}if(cb)cb->OnLieDown();return {};}
std::error_code SDKClient::Crawl(int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.motion_status=static_cast<MotionStatus>(3);cb=pImpl_->ack;}if(cb)cb->OnCrawl();return {};}
std::error_code SDKClient::SetMode(int m,int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.sport_mode=static_cast<SportMode>(m);pImpl_->state.motion_status=static_cast<MotionStatus>(m+4);cb=pImpl_->ack;}if(cb)cb->OnMode(m);return {};}
std::error_code SDKClient::SetSpeed(int s,int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.speed_level=static_cast<SpeedLevel>(s);cb=pImpl_->ack;}if(cb)cb->OnSpeed(s);return {};}
std::error_code SDKClient::SoftEmergencyStop(bool on,int,WriteHandler){std::shared_ptr<IControlCallback> cb;{std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.software_emergency_status=static_cast<EmergencyStatus>(on?2:1);pImpl_->state.speed={};cb=pImpl_->ack;}if(cb)cb->OnSoftEmergencyStop(on);return {};}
std::error_code SDKClient::Move(float y,float x,float yaw,int,WriteHandler){std::lock_guard<std::mutex> l(pImpl_->mutex);pImpl_->state.speed.line=x;pImpl_->state.speed.translation=y;pImpl_->state.speed.angle=yaw;return {};}
} // namespace robot_sdk
