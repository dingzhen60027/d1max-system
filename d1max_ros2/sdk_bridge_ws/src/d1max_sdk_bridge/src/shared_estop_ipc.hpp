#pragma once
// Local-only NDJSON transport. Receive handlers run on the I/O thread and may
// submit a nonblocking emergency request; this class implements no SDK calls.
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace d1monitor::shared_estop {
class IPC {
public:
  using Json=nlohmann::json;
  using Receive=std::function<void(uint64_t,const Json&)>;
  using Disconnect=std::function<void(uint64_t)>;
  static std::string defaultPath(){return "/run/user/"+std::to_string(getuid())+"/d1max-sdk-manual.sock";}
  IPC(std::string path,Receive receive,Disconnect disconnect):path_(std::move(path)),receive_(std::move(receive)),disconnect_(std::move(disconnect)){
    if(path_.size()>=sizeof(sockaddr_un::sun_path))throw std::runtime_error("shared estop IPC path too long");
    struct stat parent{};const auto dir=path_.substr(0,path_.find_last_of('/'));
    if(stat(dir.c_str(),&parent)||!S_ISDIR(parent.st_mode)||parent.st_uid!=getuid()||(parent.st_mode&0077))
      throw std::runtime_error("shared estop IPC requires a private same-user runtime directory");
    struct stat old{};
    if(lstat(path_.c_str(),&old)==0){
      if(!S_ISSOCK(old.st_mode)||old.st_uid!=getuid())throw std::runtime_error("shared estop IPC existing path is not an owned socket");
      const int probe=socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);sockaddr_un address{};address.sun_family=AF_UNIX;
      std::copy(path_.begin(),path_.end(),address.sun_path);
      const int connected=probe>=0?connect(probe,reinterpret_cast<sockaddr*>(&address),sizeof(address)):-1;
      const int probe_error=errno;
      if(probe>=0)close(probe);
      // Only ECONNREFUSED proves a stale owned socket. A full backlog or any
      // uncertain probe must never unlink another live server's endpoint.
      if(connected==0||probe<0||probe_error!=ECONNREFUSED)throw std::runtime_error("shared estop IPC already active or unavailable");
      if(unlink(path_.c_str()))throw std::runtime_error("cannot remove stale shared estop IPC socket");
    }
    listener_=socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(listener_<0)throw std::runtime_error("cannot create shared estop IPC socket");
    sockaddr_un address{};address.sun_family=AF_UNIX;std::copy(path_.begin(),path_.end(),address.sun_path);
    if(bind(listener_,reinterpret_cast<sockaddr*>(&address),sizeof(address))){
      close(listener_);listener_=-1;throw std::runtime_error("cannot bind private shared estop IPC socket");}
    if(lstat(path_.c_str(),&inode_)){close(listener_);listener_=-1;throw std::runtime_error("cannot stat shared estop IPC socket");}
    if(chmod(path_.c_str(),0600)||listen(listener_,4)){
      close(listener_);listener_=-1;removeOwnedSocket();throw std::runtime_error("cannot initialize private shared estop IPC socket");}
  }
  // Start only once the owning Monitor has stored the complete server.
  void start(){if(worker_.joinable())throw std::logic_error("shared estop IPC already started");worker_=std::thread([this]{run();});}
  void stop(){stopping_.store(true);if(worker_.joinable())worker_.join();}
  ~IPC(){stop();if(listener_>=0)close(listener_);removeOwnedSocket();}
  IPC(const IPC&)=delete;IPC&operator=(const IPC&)=delete;
  void send(uint64_t client,const Json&value){
    auto line=value.dump()+"\n";if(line.size()>16384)return;
    std::lock_guard<std::mutex>lock(out_mutex_);
    // Coalesce diagnostics before the I/O thread drains them. Command replies
    // are never replaced by a state update.
    const bool state=value.value("type",std::string())=="state";
    if(state)for(auto it=out_.rbegin();it!=out_.rend();++it)if(it->client==client&&it->state){it->line=std::move(line);return;}
    if(out_.size()>=128){overflow_.store(true);return;}
    out_.push_back({client,std::move(line),state});
  }
private:
  void removeOwnedSocket(){struct stat actual{};if(inode_.st_ino&&lstat(path_.c_str(),&actual)==0&&actual.st_ino==inode_.st_ino&&actual.st_dev==inode_.st_dev)unlink(path_.c_str());}
  struct Packet{uint64_t client;std::string line;bool state;};
  struct Peer{int fd;uint64_t id;std::string input,output;bool drop=false;};
  void run()noexcept{
    std::map<int,Peer> peers;uint64_t next=0;
    try{while(!stopping_.load()){
      std::deque<Packet>out;{std::lock_guard<std::mutex>lock(out_mutex_);out.swap(out_);}
      for(const auto&message:out)for(auto&entry:peers)if(entry.second.id==message.client){
        auto&peer=entry.second;if(peer.output.size()+message.line.size()>65536)peer.drop=true;else peer.output+=message.line;}
      if(overflow_.exchange(false))for(auto&entry:peers)entry.second.drop=true;
      std::vector<pollfd>polls{{listener_,POLLIN,0}};
      for(const auto&p:peers)polls.push_back({p.first,short(POLLIN|(p.second.output.empty()?0:POLLOUT)),0});
      if(poll(polls.data(),polls.size(),20)<0&&errno!=EINTR)break;
      if(polls[0].revents&POLLIN){
        const int fd=accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
        if(fd>=0){ucred credentials{};socklen_t size=sizeof(credentials);
          if(peers.size()>=4||getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&credentials,&size)||credentials.uid!=getuid())close(fd);
          else peers.emplace(fd,Peer{fd,++next,{},{},false});}
      }
      for(size_t i=1;i<polls.size();++i){auto found=peers.find(polls[i].fd);if(found==peers.end())continue;auto&peer=found->second;
        if(polls[i].revents&(POLLERR|POLLHUP|POLLNVAL))peer.drop=true;
        if(!peer.drop&&(polls[i].revents&POLLIN)){
          char bytes[4096];const auto n=recv(peer.fd,bytes,sizeof(bytes),0);
          if(n==0)peer.drop=true;else if(n<0){if(errno!=EAGAIN&&errno!=EINTR)peer.drop=true;}
          else {peer.input.append(bytes,size_t(n));if(peer.input.size()>16384)peer.drop=true;
            size_t newline=0;unsigned count=0;while(!peer.drop&&(newline=peer.input.find('\n'))!=std::string::npos){
              if(++count>32){peer.drop=true;break;}const auto line=peer.input.substr(0,newline);peer.input.erase(0,newline+1);
              try{receive_(peer.id,Json::parse(line));}catch(const Json::exception&){peer.drop=true;}
            }}
        }
        if(!peer.drop&&(polls[i].revents&POLLOUT)&&!peer.output.empty()){
          const auto n=::send(peer.fd,peer.output.data(),peer.output.size(),MSG_NOSIGNAL);
          if(n>0)peer.output.erase(0,size_t(n));else if(n<0&&errno!=EAGAIN&&errno!=EINTR)peer.drop=true;
        }
      }
      for(auto it=peers.begin();it!=peers.end();)if(it->second.drop){const auto id=it->second.id;close(it->first);it=peers.erase(it);disconnect_(id);}else ++it;
    }}catch(...){/* Drop only panel connections; no SDK/session/permission side effect. */}
    for(const auto&p:peers){close(p.first);try{disconnect_(p.second.id);}catch(...){}}
  }
  std::string path_;Receive receive_;Disconnect disconnect_;int listener_=-1;struct stat inode_{};
  std::mutex out_mutex_;std::deque<Packet>out_;std::atomic<bool>stopping_{false},overflow_{false};std::thread worker_;
};
}
