"""Compile exact passive worker methods, no ROS/SDK/network in this regression."""
from pathlib import Path
import subprocess

import pytest

from test_geometry_exchange_wiring import body


@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    source=(Path(__file__).resolve().parents[1]/'src/local_follower_node.cpp').read_text()
    methods='\n'.join(body(source,s) for s in [
        'void startControlTraceWorker()', 'void runControlTraceWorker()', 'void stopControlTraceWorker()'])
    temp=tmp_path_factory.mktemp('actual_worker')
    cpp=temp/'probe.cpp'
    cpp.write_text(r'''
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <string>
#define RCLCPP_WARN(...) ++warnings
class Probe {
 public:
  std::atomic<int> warnings{0},flushes{0};std::mutex gate;std::condition_variable ready;
  bool entered=false,release=false,fail=false;
  void flushControlTrace(){++flushes;std::unique_lock<std::mutex> lock(gate);entered=true;ready.notify_all();
    if(fail)throw std::runtime_error("diagnostic only");ready.wait(lock,[this]{return release;});}
''' + methods + r'''
 private:
  std::mutex trace_wait_mutex_;std::condition_variable trace_wakeup_;
  bool trace_stop_{false};std::thread trace_worker_;
};
int main(int argc,char**argv){using clock=std::chrono::steady_clock;Probe p;
  const std::string mode=argv[1];p.fail=mode=="throw";p.release=mode=="wake";p.startControlTraceWorker();
  if(mode=="wake") {const auto start=clock::now();p.stopControlTraceWorker();
    if(clock::now()-start>std::chrono::milliseconds(150))return 1;
    if(p.flushes!=1)return 6;return 0;}
  {std::unique_lock<std::mutex> lock(p.gate);
    if(!p.ready.wait_for(lock,std::chrono::seconds(2),[&]{return p.entered;}))return 2;}
  int control_ticks=0;const auto start=clock::now();
  while(clock::now()-start<std::chrono::milliseconds(300)){
    ++control_ticks;std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  {std::lock_guard<std::mutex> lock(p.gate);p.release=true;}p.ready.notify_all();p.stopControlTraceWorker();
  if(control_ticks<20)return 3;
  if(mode=="throw"&&p.warnings!=1)return 4;
  if(mode=="slow"&&p.warnings!=0)return 5;
  if(mode=="slow"&&p.flushes!=2)return 7;
  std::cout<<"control_ticks="<<control_ticks<<"\n";return 0;
}
''')
    binary=temp/'probe'
    subprocess.run(['c++','-std=c++17','-pthread','-O2',str(cpp),'-o',str(binary)],check=True)
    return binary


@pytest.mark.parametrize('mode',['wake','slow','throw'])
def test_actual_worker_has_no_control_dependency(probe,mode):
    subprocess.run([str(probe),mode],check=True,timeout=5)
