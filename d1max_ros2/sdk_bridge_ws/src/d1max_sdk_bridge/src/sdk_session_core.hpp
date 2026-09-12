#pragma once
#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <unistd.h>

namespace d1monitor {
// Process-wide exclusion, including direct binary launches (ROS discovery alone
// has a check/start race). Never unlink the live lock inode.
class SessionLease {
 public:
  explicit SessionLease(const std::string& path) {
    fd_=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd_<0)throw std::runtime_error("Cannot open SDK session lock: "+path);
    if(flock(fd_,LOCK_EX|LOCK_NB)!=0){close(fd_);fd_=-1;throw std::runtime_error("SDK monitor already running; refusing duplicate connection");}
  }
  ~SessionLease(){if(fd_>=0)close(fd_);}
  SessionLease(const SessionLease&)=delete;
  SessionLease& operator=(const SessionLease&)=delete;
 private:int fd_=-1;
};

struct ConnectRetry {
  double last_attempt=-1e9,interval_sec=5.;
  unsigned attempts=0;
  // SDK enum DISCONNECTED=1. The vendor owns CONNECTING/HANDSHAKING/
  // RECONNECTING; never start a second operation while any is in progress.
  bool due(int state,double now) {
    if(state!=1||now-last_attempt<interval_sec)return false;
    last_attempt=now;++attempts;return true;
  }
};
}
