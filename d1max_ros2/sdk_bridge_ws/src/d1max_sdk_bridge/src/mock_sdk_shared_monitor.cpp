// TEST ONLY. No socket API or vendor library. Any non-loopback robot argument
// aborts construction; commands only change this in-process robot substitute.
#include <robot_sdk/sdk_client.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
namespace robot_sdk {
using Clock=std::chrono::steady_clock;
class SDKClient::Impl {
public:
  std::mutex mutex;std::shared_ptr<IDataCallback>data;std::shared_ptr<IControlCallback>ack;
  std::atomic<bool>connected{false},run{true},mc{false};RobotState state{};std::thread worker;
  struct Write{Clock::time_point due;WriteHandler callback;};std::deque<Write>writes;
  unsigned tick=0;int delay=5,normal_delay=5,after_stand_delay=0;
  bool fail_post_estop_zero=false,estop_released=false;
  std::string mc_bias_file;int mc_bias=0;
  Impl(){
    if(!std::getenv("D1MAX_SHARED_MONITOR_TEST"))throw std::runtime_error("test-only SDK");
    if(const auto value=std::getenv("D1MAX_MOCK_MOVE_DELAY_MS"))delay=std::atoi(value);
    normal_delay=delay;
    if(const auto value=std::getenv("D1MAX_MOCK_DELAY_AFTER_STAND_MS"))after_stand_delay=std::atoi(value);
    fail_post_estop_zero=std::getenv("D1MAX_MOCK_FAIL_POST_ESTOP_ZERO")!=nullptr;
    if(const auto value=std::getenv("D1MAX_MOCK_MC_BIAS_FILE"))mc_bias_file=value;
    state.motion_status=static_cast<MotionStatus>(2);state.sport_mode=static_cast<SportMode>(1);
    state.control_source=static_cast<CtrlSource>(0);state.speed_level=static_cast<SpeedLevel>(1);
    state.software_emergency_status=state.hardware_emergency_status=static_cast<EmergencyStatus>(1);
    state.head_direction=static_cast<HeadDirection>(1);state.robot_mode=static_cast<RobotMode>(1);
    worker=std::thread([this]{while(run){
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      std::deque<WriteHandler>done;RobotState copy;std::shared_ptr<IDataCallback>cb;
      {std::lock_guard<std::mutex>lock(mutex);while(!writes.empty()&&writes.front().due<=Clock::now()){
        done.push_back(std::move(writes.front().callback));writes.pop_front();}copy=state;cb=data;}
      for(const auto&f:done)f({},1);
      if(!connected||!cb)continue;
      if(++tick%20==0)cb->OnRobotStateData(copy);
      if(tick%10==0&&!mc_bias_file.empty()){
        int value=0;std::ifstream input(mc_bias_file);input>>value;
        if(value!=mc_bias){mc_bias=value;record("MCBias",mc_bias);}
      }
      if(mc){MotionData m{};m.quat[0]=1;m.time_stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
        m.v_body[0]=copy.speed.line;m.v_body[1]=copy.speed.translation;m.omega_body[2]=copy.speed.angle;
        // Test-only sensor offset from the 2026-10-22 stationary capture.
        // It alters MC measurements, never Move acknowledgements/RobotState.
        if(mc_bias==1){m.v_body[0]-=.0061f;m.v_body[1]-=.0205f;m.v_body[2]-=.0233f;}
        // Later fixed-STAIR capture: each configured component is below
        // 0.03 m/s, but the combined norm is above it. Other values are
        // intentional unsafe-motion counterexamples, not sensor filtering.
        if(mc_bias==2){m.v_body[0]+=.0217f;m.v_body[2]+=.0287f;m.omega_body[2]+=.007f;}
        if(mc_bias==3)m.v_body[2]+=.031f;
        if(mc_bias==4)m.v_body[0]+=.031f;
        if(mc_bias==5)m.omega_body[2]+=.051f;
        cb->OnMcData(m);}
    }});
  }
  ~Impl(){run=false;if(worker.joinable())worker.join();}
  void record(const char*op,float a=0,float b=0,float c=0){
    fprintf(stderr,"MOCK_CMD %s %.6f %.6f %.6f %.9f\n",op,a,b,c,std::chrono::duration<double>(Clock::now().time_since_epoch()).count());
  }
};
SDKClient::SDKClient(ErrorHandler,ConnectionConfig,TransportProtocol):pImpl_(std::make_shared<Impl>()){}
SDKClient::~SDKClient()=default;
std::error_code SDKClient::Connect(std::string ip,std::string,bool,ConnectHandler cb){
  if(ip!="127.0.0.1")throw std::runtime_error("fake SDK refuses all robot addresses");
  pImpl_->record("Connect");pImpl_->connected=true;cb({});return {};
}
std::error_code SDKClient::Disconnect(bool,DisConnectHandler cb){pImpl_->record("Disconnect");pImpl_->connected=false;cb({});return {};}
bool SDKClient::IsConnected()const{return pImpl_->connected;}
ConnectionState SDKClient::GetConnectionState()const{return pImpl_->connected?ConnectionState::CONNECTED:ConnectionState::DISCONNECTED;}
const std::string& SDKClient::Version()const{static const std::string version="shared-monitor-test";return version;}
void SDKClient::SetDataCallback(std::shared_ptr<IDataCallback>cb){std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->data=std::move(cb);}
void SDKClient::SetControlCallback(std::shared_ptr<IControlCallback>cb){std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->ack=std::move(cb);}
std::error_code SDKClient::TakeControl(int,WriteHandler cb){
  std::shared_ptr<IControlCallback>ack;{std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->record("TakeControl");pImpl_->state.control_source=static_cast<CtrlSource>(2);ack=pImpl_->ack;}
  if(ack)ack->OnTakeControlAck(TakeControlAck{});cb({},1);return {};
}
std::error_code SDKClient::Move(float y,float x,float yaw,int,WriteHandler cb){
  std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->record("Move",y,x,yaw);
  if(pImpl_->fail_post_estop_zero&&pImpl_->estop_released&&y==0&&x==0&&yaw==0){
    pImpl_->record("MoveFailure");return std::make_error_code(std::errc::io_error);
  }
  pImpl_->state.speed.translation=y;pImpl_->state.speed.line=x;pImpl_->state.speed.angle=yaw;
  pImpl_->writes.push_back({Clock::now()+std::chrono::milliseconds(pImpl_->delay),std::move(cb)});return {};
}
std::error_code SDKClient::SetMode(int mode,int,WriteHandler cb){
  std::shared_ptr<IControlCallback>ack;{std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->record("SetMode",mode);pImpl_->state.sport_mode=static_cast<SportMode>(mode);pImpl_->state.motion_status=static_cast<MotionStatus>(mode+4);ack=pImpl_->ack;}
  if(ack)ack->OnMode(mode);cb({},1);return {};
}
std::error_code SDKClient::StandUp(int,WriteHandler cb){std::shared_ptr<IControlCallback>ack;
  {std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->record("StandUp");
    pImpl_->state.motion_status=static_cast<MotionStatus>(static_cast<int>(pImpl_->state.sport_mode)+4);
    if(pImpl_->after_stand_delay)pImpl_->delay=pImpl_->after_stand_delay;
    ack=pImpl_->ack;}
  if(ack)ack->OnStandUp();cb({},1);return {};
}
#define MOCK_POSTURE(NAME,STATUS,CALLBACK) \
std::error_code SDKClient::NAME(int,WriteHandler cb){std::shared_ptr<IControlCallback>ack; \
{std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->record(#NAME);pImpl_->state.motion_status=static_cast<MotionStatus>(STATUS);ack=pImpl_->ack;} \
if(ack)ack->CALLBACK();cb({},1);return {};}
MOCK_POSTURE(LieDown,2,OnLieDown)
MOCK_POSTURE(Crawl,3,OnCrawl)
MOCK_POSTURE(Locked,4,OnLocked)
#undef MOCK_POSTURE
std::error_code SDKClient::SetSpeed(int speed,int,WriteHandler cb){std::shared_ptr<IControlCallback>ack;
  {std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->record("SetSpeed",speed);pImpl_->state.speed_level=static_cast<SpeedLevel>(speed);ack=pImpl_->ack;}
  if(ack)ack->OnSpeed(speed);cb({},1);return {};}
std::error_code SDKClient::SoftEmergencyStop(bool on,int,WriteHandler cb){std::shared_ptr<IControlCallback>ack;
  {std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->record("SoftEmergencyStop",on);pImpl_->state.software_emergency_status=static_cast<EmergencyStatus>(on?2:1);pImpl_->state.speed={};pImpl_->estop_released=!on;if(!on)pImpl_->delay=pImpl_->normal_delay;ack=pImpl_->ack;}
  if(ack)ack->OnSoftEmergencyStop(on);cb({},1);return {};}
std::error_code SDKClient::SetMcConfig(bool on,int,WriteHandler cb){std::shared_ptr<IControlCallback>ack;
  {std::lock_guard<std::mutex>l(pImpl_->mutex);pImpl_->mc=on;ack=pImpl_->ack;}if(ack)ack->OnMcConfig(on);cb({},1);return {};}
std::error_code SDKClient::SetJointStateConfig(bool on,int,WriteHandler cb){std::shared_ptr<IControlCallback>ack;
  {std::lock_guard<std::mutex>l(pImpl_->mutex);ack=pImpl_->ack;}if(ack)ack->OnJointStateConfig(on);cb({},1);return {};}
}
