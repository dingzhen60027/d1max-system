#include "sdk_session_core.hpp"
#include <cassert>
#include <iostream>
int main(){
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
  }
  {d1monitor::SessionLease after_exit(path);}
  assert(unlink(path.c_str())==0);assert(rmdir(directory)==0);
  std::cout<<"SDK session: retry serialization and process lease checks passed\n";
}
