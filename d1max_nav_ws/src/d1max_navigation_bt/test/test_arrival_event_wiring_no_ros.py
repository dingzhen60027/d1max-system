"""Compile the actual navigator completion callback/consumer, without ROS nodes.

Set D1MAX_BT_SOURCE_ROOT to a frozen repository root for same-input before/after.
Only transport, clock and live-state providers are passive substitutes; the
acceptance body and final-consumption method are extracted unchanged.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(os.environ.get('D1MAX_BT_SOURCE_ROOT', Path(__file__).resolve().parents[3]))


def block(text, marker):
    start = text.index(marker)
    opening = text.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


class ArrivalWiring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (ROOT/'src/d1max_navigation_bt/src/navigator_node.cpp').read_text()
        method = block(source, '  bool measuredGoalReached(').replace(' override', '')
        follow = block(source, '  void requestFollow(')
        begin = follow.index('      if (success) {')
        end = follow.index('      task->follow.result = {success ?', begin)
        acceptance = follow[begin:end]
        event = 'MeasuredArrivalEvent arrival;' in source
        fields = 'MeasuredArrivalEvent arrival;' if event else '''
          bool arrived=false;int64_t measured_arrival_stamp=0;
          Clock::time_point arrival_received_at{};'''
        helper = block(source, 'ArrivalIdentity arrivalIdentity(') if event else ''
        harness = r'''
#include "d1max_navigation_bt/runtime_policy.hpp"
#include <chrono>
#include <memory>
#include <iostream>
using namespace d1max_navigation_bt;
using Clock=std::chrono::steady_clock;
namespace BT {enum class NodeStatus{SUCCESS,FAILURE};}
struct TaskIdentity{std::string task_id="task",context_id="session";};
struct Result{BT::NodeStatus status=BT::NodeStatus::SUCCESS;std::string reason;};
struct Authority{bool stopped=true;bool goalStopped()const{return stopped;}};
struct Snapshot{std::string map_version_id="map",route_id="route",route_hash="hash";};
struct Task{
  TaskIdentity identity;struct{Snapshot snapshot;}route;
  int64_t epoch=7;std::string seed="seed";
  bool retired=false,finished=false,cancellation_requested=false;
  std::shared_ptr<Authority>follower=std::make_shared<Authority>(),execution;
  struct{Result result;}follow;
FIELDS
};
HELPER
namespace rclcpp{struct Time{
  int64_t ns;explicit Time(int64_t n):ns(n){}int64_t nanoseconds()const{return ns;}
  double seconds()const{return ns*1e-9;}
};}
struct PassiveNavigator{
  std::shared_ptr<Task>task=std::make_shared<Task>();
  int64_t wall=100100000000LL;double elapsed_time=0.,arrival_evidence_timeout_=4.;
  bool fresh=true,context=true,body=true;
  std::string session_="session";int local_state_=0;
  struct{bool local_control_ready=true,ready=true;}health_;
  std::shared_ptr<Task>lookup(const TaskIdentity&id){
    return id.task_id==task->identity.task_id&&id.context_id==task->identity.context_id?task:nullptr;}
  rclcpp::Time now()const{return rclcpp::Time(wall);}
  double elapsed(Clock::time_point)const{return elapsed_time;}
  bool healthFresh()const{return fresh;}
  struct Ready{bool ready;};
  Ready contextValid(const TaskIdentity&)const{return {context&&!task->retired&&!task->cancellation_requested};}
  template<class...T>bool localTaskInputsFresh(T...)const{return body;}
  void receive(int64_t stamp){
    struct Response{int64_t measured_arrival_stamp;};
    struct{std::shared_ptr<Response>result;}result{std::make_shared<Response>(Response{stamp})};
    const bool success=true;
ACCEPTANCE
  }
METHOD
};
int main(int argc,char**argv){
  if(argc!=2)return 2;std::string test=argv[1];PassiveNavigator n;
  if(test=="stale_at_receipt")n.wall=110000000000LL;
  if(test=="retired_before_receipt")n.task->retired=true;
  n.receive(100000000000LL);
  if(test=="pause_after_receipt"){n.wall=110000000000LL;n.elapsed_time=9.9;}
  if(test=="no_measured_stop")n.task->follower->stopped=false;
  if(test=="cancelled_after_receipt")n.task->cancellation_requested=true;
  if(test=="epoch_replaced"){n.task->epoch=8;n.context=false;}
  if(test=="not_fresh_yet")n.body=false;
  auto id=n.task->identity;if(test=="new_task")id.task_id="new-task";
  bool ok=n.measuredGoalReached(id);
  bool expected=test=="fresh"||test=="pause_after_receipt";
  std::cout<<test<<": accepted="<<ok<<" expected="<<expected<<"\n";
  return ok==expected?0:1;
}
'''.replace('FIELDS', fields).replace('HELPER', helper).replace('ACCEPTANCE', acceptance).replace('METHOD', method)
        cls.directory = tempfile.TemporaryDirectory(prefix='bt-arrival-event-')
        cpp = Path(cls.directory.name)/'probe.cpp'
        cpp.write_text(harness)
        cls.binary = Path(cls.directory.name)/'probe'
        subprocess.run(['c++', '-std=c++17', '-O0', '-I', str(ROOT/'src/d1max_navigation_bt/include'),
                        str(cpp), '-o', str(cls.binary)], check=True, capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def run_case(self, name):
        completed = subprocess.run([str(self.binary), name], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stdout+completed.stderr)


for _case in ('fresh', 'pause_after_receipt', 'stale_at_receipt', 'retired_before_receipt',
              'no_measured_stop', 'cancelled_after_receipt', 'epoch_replaced', 'not_fresh_yet', 'new_task'):
    setattr(ArrivalWiring, 'test_'+_case, lambda self, name=_case: self.run_case(name))


if __name__ == '__main__':
    unittest.main()
