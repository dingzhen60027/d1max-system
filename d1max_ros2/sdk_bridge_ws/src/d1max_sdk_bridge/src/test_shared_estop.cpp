#include "shared_estop_core.hpp"
#include "shared_estop_ipc.hpp"
#include "monitor_estop_core.hpp"
#include <cassert>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

using namespace d1monitor::shared_estop;
static Json hello(){return {{"type","hello"},{"version",1},{"client","test-client"}};}
static Json command(uint64_t id,const std::string&op){return {{"type","command"},{"version",1},{"sdk_session","sdk"},{"request",std::to_string(id)},{"op",op}};}
struct FakeReceiver {
  std::mutex mutex;d1monitor::Safety safety;bool navigation_allowed=true;
  unsigned software_true_writes=0,software_false_writes=0;bool moved_after_veto=false;
  FakeReceiver(){safety.connected=true;safety.update(1,1,1.);}
  bool request(double now,bool on=true){std::lock_guard<std::mutex>lock(mutex);
    const bool send=safety.request(now,on);
    if(!safety.request_accepted)return false;
    if(on)navigation_allowed=false;
    if(send){if(on)++software_true_writes;else ++software_false_writes;}
    return true;
  }
  void writer(){std::lock_guard<std::mutex>lock(mutex);if(navigation_allowed){assert(safety.sent==0);}}
};
static int clientSocket(const std::string&path){
  const int fd=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);assert(fd>=0);
  sockaddr_un address{};address.sun_family=AF_UNIX;std::copy(path.begin(),path.end(),address.sun_path);
  assert(connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0);return fd;
}
static void sendLine(int fd,const Json&value){const auto s=value.dump()+"\n";assert(::send(fd,s.data(),s.size(),MSG_NOSIGNAL)==ssize_t(s.size()));}
static Json readLine(int fd,std::string&buffer){
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  for(;;){const auto n=buffer.find('\n');if(n!=std::string::npos){auto j=Json::parse(buffer.substr(0,n));buffer.erase(0,n+1);return j;}
    assert(std::chrono::steady_clock::now()<end);pollfd p{fd,POLLIN,0};const auto ready=poll(&p,1,100);assert(ready>=0);
    if(ready){char bytes[4096];const auto count=recv(fd,bytes,sizeof(bytes),0);assert(count>0);buffer.append(bytes,size_t(count));}
  }
}
int main(){unsigned cases=0;
  {
    Protocol protocol("sdk",true);assert(protocol.receive(1,hello()).kind==Protocol::Kind::Hello);
    // Force the timer to run after registration but before hello was queued.
    // It must not see a peer eligible for unsolicited state yet.
    std::vector<uint64_t> observed;
    std::thread broadcaster([&]{observed=protocol.clients();});broadcaster.join();
    assert(observed.empty());protocol.helloQueued(1);
    assert(protocol.clients()==std::vector<uint64_t>{1});
    protocol.disconnect(1);assert(protocol.clients().empty());
    protocol.helloQueued(1);assert(protocol.clients().empty());++cases;
  }
  {
    Protocol protocol("sdk");State state;state.present=true;state.connected=true;state.owned=true;state.received=10.;state.sequence=7;
    assert(protocol.receive(1,hello()).kind==Protocol::Kind::Hello);
    const auto h=protocol.hello(state);assert(h["capabilities"]==Json::array({"emergency","emergency_release"}));assert(h["state"]["received"]==10.);
    assert(h["state"]["estop_pending"]==false&&h["state"]["estop_transaction"]=="0");
    // Heartbeats cannot refresh the raw RobotState receipt or create a lease.
    assert(protocol.state(state)["state"]["received"]==10.);assert(protocol.state(state)["state"]["sequence"]=="7");++cases;
    uint64_t id=0;
    for(const auto*op:{"take","release","posture","speed","jog","heartbeat","recover","unknown"}){
      auto r=protocol.receive(1,command(++id,op));assert(r.kind==Protocol::Kind::Reject);assert(r.reason=="shared_session_emergency_only");}++cases;
    for(const auto*op:{"stop","release_jog","detach"})assert(protocol.receive(1,command(++id,op)).kind==Protocol::Kind::LocalOnly);
    ++cases;
    auto j=command(++id,"emergency");j["on"]=false;auto release=protocol.receive(1,j);
    assert(release.kind==Protocol::Kind::Emergency&&!release.on);
    assert(protocol.result(release,true,"written_not_confirmed")["on"]==false);
    j=command(++id,"emergency");j["on"]=1;assert(protocol.receive(1,j).kind==Protocol::Kind::Reject);
    j=command(++id,"emergency");j["on"]=true;assert(protocol.receive(1,j).kind==Protocol::Kind::Emergency);
    assert(protocol.receive(1,j).kind==Protocol::Kind::Reject);protocol.receive(1,hello());assert(protocol.receive(1,j).kind==Protocol::Kind::Reject);++cases;
    j=command(++id,"emergency");j["sdk_session"]="other";assert(protocol.receive(1,j).kind==Protocol::Kind::Reject);
    j["sdk_session"]="sdk";j["request"]="18446744073709551616";assert(protocol.receive(1,j).kind==Protocol::Kind::Reject);
    j["request"]=42;assert(protocol.receive(1,j).kind==Protocol::Kind::Reject);
    protocol.disconnect(1);assert(protocol.receive(1,command(++id,"emergency")).kind==Protocol::Kind::Reject);++cases;
  }
  {
    FakeReceiver receiver;std::vector<std::thread>threads;
    for(unsigned i=0;i<32;++i)threads.emplace_back([&receiver]{assert(receiver.request(1.1));receiver.writer();});
    for(auto&t:threads)t.join();
    assert(receiver.software_true_writes==1&&!receiver.navigation_allowed);
    receiver.safety.update(2,1,1.2);receiver.safety.update(2,1,1.3);assert(receiver.request(1.4));assert(receiver.software_true_writes==1);
    receiver.safety.connected=false;assert(!receiver.request(1.5));assert(receiver.software_true_writes==1);
    receiver.safety.connected=true;receiver.safety.replay=true;assert(!receiver.request(1.6));++cases;
    // A submitted write without a state ACK expires. No timer retries it or
    // restores the original navigation permission.
    FakeReceiver no_ack;assert(no_ack.request(1.1));no_ack.safety.tick(7.);
    assert(no_ack.safety.result=="UNCONFIRMED"&&no_ack.software_true_writes==1&&!no_ack.navigation_allowed);++cases;
    FakeReceiver toggle;assert(toggle.request(1.1));toggle.safety.update(2,1,1.2);toggle.safety.update(2,1,1.3);
    assert(toggle.request(1.4,false)&&toggle.software_false_writes==1&&!toggle.navigation_allowed);
    assert(toggle.request(1.5,false)&&toggle.software_false_writes==1); // Coalesced, not a second false.
    const auto old=toggle.safety.transaction;
    assert(toggle.request(1.6,true)&&toggle.software_true_writes==2&&!toggle.navigation_allowed);
    toggle.safety.failed("late_false_error",old);assert(toggle.safety.pending);
    toggle.safety.update(1,1,1.7);assert(toggle.safety.pending);toggle.writer();++cases;
  }
  {
    // Match the actual shared monitor contract, not standalone motion
    // recovery: retiring the old task/losing SDK ownership cannot require an
    // automatic TakeControl merely to submit explicit software OFF. State
    // source/ownership remain truthful and this protocol owns no motion API.
    Protocol protocol("sdk");State state;state.present=state.connected=true;
    state.owned=false;state.source=1;state.soft=2;state.hardware=1;state.received=100.;state.sequence=2;
    assert(protocol.receive(1,hello()).kind==Protocol::Kind::Hello);
    auto off=command(1,"emergency");off["on"]=false;const auto request=protocol.receive(1,off);
    assert(request.kind==Protocol::Kind::Emergency&&!request.on);
    d1monitor::Safety safety;safety.connected=true;safety.update(state.soft,state.hardware,state.received);
    assert(safety.request(100.1,request.on)&&safety.request_accepted);
    const auto before=state.json();assert(!before["owned"].get<bool>()&&before["source"]==1);
    safety.update(1,1,101.);assert(!safety.releaseConfirmed());
    safety.update(1,1,102.);assert(safety.releaseConfirmed()&&safety.sent==1);
    // No bool ACK or fabricated post-request heartbeat was used as feedback.
    assert(!safety.ack);assert(state.json()==before);++cases;
  }
  {
    WriteResult write;assert(!write.read().first);std::thread late([&]{write.finish({});});late.join();
    write.finish(std::make_error_code(std::errc::io_error));assert(write.read().first&&!write.read().second);++cases;
    auto inbox=std::make_shared<WriteInbox>();auto outcome=std::make_shared<WriteResult>();
    std::weak_ptr<WriteInbox>weak=inbox;
    std::function<void(const std::error_code&)>callback=[inbox,outcome](const std::error_code&e){inbox->finish(e);outcome->finish(e);};
    // Destroy the simulated Node's ownership before a delayed SDK completion.
    inbox.reset();outcome.reset();callback(std::make_error_code(std::errc::io_error));
    assert(weak.lock()->take()==std::make_error_code(std::errc::io_error));callback={};assert(weak.expired());++cases;
  }
  char directory[]="/tmp/d1max-shared-estop-test-XXXXXX";assert(mkdtemp(directory));
  const auto path=std::string(directory)+"/estop.sock";
  {
    bool unavailable=false;std::unique_ptr<IPC>optional;
    try{optional=std::make_unique<IPC>(std::string(directory)+"/absent/estop.sock",[](uint64_t,const Json&){},[](uint64_t){});optional->start();}
    catch(const std::exception&){optional.reset();unavailable=true;}
    FakeReceiver ros_only;assert(unavailable&&!optional&&ros_only.request(1.1)&&ros_only.software_true_writes==1);++cases;
    const int file=open(path.c_str(),O_CREAT|O_EXCL|O_WRONLY|O_CLOEXEC,0600);assert(file>=0);close(file);
    bool refused=false;try{IPC blocked(path,[](uint64_t,const Json&){},[](uint64_t){});}catch(const std::runtime_error&){refused=true;}
    struct stat unchanged{};assert(refused&&lstat(path.c_str(),&unchanged)==0&&S_ISREG(unchanged.st_mode));assert(unlink(path.c_str())==0);++cases;
  }
  {
    Protocol protocol("sdk");FakeReceiver receiver;State state;state.present=state.connected=state.owned=true;state.received=10.;state.sequence=1;
    IPC*active=nullptr;
    IPC server(path,[&](uint64_t c,const Json&j){
      const auto r=protocol.receive(c,j);
      if(r.kind==Protocol::Kind::Hello){active->send(c,protocol.hello(state));protocol.helloQueued(c);return;}
      if(r.kind==Protocol::Kind::Emergency){const bool ok=receiver.request(1.1,r.on);active->send(c,protocol.state(state));active->send(c,protocol.result(r,ok,"request_submitted_not_measured_stop"));}
      else if(r.kind==Protocol::Kind::Reject||r.kind==Protocol::Kind::LocalOnly)active->send(c,protocol.result(r,r.kind==Protocol::Kind::LocalOnly,r.reason));
    },[&](uint64_t c){protocol.disconnect(c);});active=&server;server.start();
    struct stat status{};assert(lstat(path.c_str(),&status)==0&&S_ISSOCK(status.st_mode)&&(status.st_mode&0777)==0600);
    bool refused=false;try{IPC duplicate(path,[](uint64_t,const Json&){},[](uint64_t){});}catch(const std::runtime_error&){refused=true;}assert(refused);++cases;
    const int fd=clientSocket(path);std::string buffer;sendLine(fd,hello());assert(readLine(fd,buffer)["type"]=="hello");
    sendLine(fd,command(1,"jog"));assert(!readLine(fd,buffer)["ok"].get<bool>());
    auto emergency=command(2,"emergency");emergency["on"]=true;sendLine(fd,emergency);
    assert(readLine(fd,buffer)["type"]=="state");const auto result=readLine(fd,buffer);assert(result["ok"].get<bool>()&&result["request"]=="2");
    assert(receiver.software_true_writes==1&&!receiver.navigation_allowed);++cases;
    sendLine(fd,command(3,"detach"));assert(readLine(fd,buffer)["ok"].get<bool>());close(fd);
    const int reattached=clientSocket(path);buffer.clear();sendLine(reattached,hello());assert(readLine(reattached,buffer)["state"]["received"]==10.);
    sendLine(reattached,command(1,"emergency"));assert(readLine(reattached,buffer)["type"]=="state");assert(readLine(reattached,buffer)["ok"].get<bool>());
    close(reattached);assert(receiver.software_true_writes==1&&!receiver.navigation_allowed);++cases;
  }
  struct stat gone{};assert(lstat(path.c_str(),&gone)!=0&&errno==ENOENT);assert(rmdir(directory)==0);
  std::cout<<"shared emergency-only IPC: "<<cases<<" cases passed; no SDK linked or robot connection\n";
}
