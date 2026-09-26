#include "sdk_session_core.hpp"
#include <cassert>
#include <iostream>
#include <sys/wait.h>
int main(){
  assert(d1monitor::SessionLease::default_path()==
    "/run/user/"+std::to_string(getuid())+"/d1max-sdk-monitor.lock");
  d1monitor::ConnectRetry retry;
  assert(retry.due(1,0.));assert(!retry.due(1,4.9));assert(retry.due(1,5.));
  for(int state:{0,2,3,4,5})assert(!retry.due(state,20.));
  assert(retry.due(1,20.));assert(retry.attempts==3);
  char directory[]="/tmp/d1max-sdk-lock-test-XXXXXX";
  assert(mkdtemp(directory));const std::string path=std::string(directory)+"/session.lock";
  {
    d1monitor::SessionLease first(path);bool refused=false;
    try{d1monitor::SessionLease second(path);}catch(const std::runtime_error&){refused=true;}
    assert(refused);
    // Every SDK entry point uses this lease before constructing its SDKClient.
    // A separate process must be excluded too, not only another object.
    const auto child=fork();assert(child>=0);
    if(child==0){
      try{d1monitor::SessionLease duplicate(path);_exit(1);}
      catch(const std::runtime_error&){_exit(0);}
    }
    int status=0;assert(waitpid(child,&status,0)==child);
    assert(WIFEXITED(status)&&WEXITSTATUS(status)==0);
  }
  {d1monitor::SessionLease after_exit(path);}
  assert(unlink(path.c_str())==0);assert(rmdir(directory)==0);
  std::cout<<"SDK session: retry serialization and process lease checks passed\n";
}
