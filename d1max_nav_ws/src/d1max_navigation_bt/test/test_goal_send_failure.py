"""Offline execution of the navigator's real send catches and retirement fence.

Only the transport and task storage are doubles. No ROS context, graph or SDK
is created. The extracted production bodies prevent a test-only retry policy
from silently disagreeing with navigator_node.cpp.
"""
from pathlib import Path
import os
import subprocess

import pytest


# Test-only override permits identical inputs against a frozen release source
# and its own headers; it never changes imports or production configuration.
ROOT = Path(os.environ.get('D1MAX_GOAL_SEND_SOURCE_ROOT',
                           Path(__file__).resolve().parents[1])).resolve()


def braced(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


@pytest.fixture(scope='module')
def send_failure_probe(tmp_path_factory):
    source = (ROOT / 'src/navigator_node.cpp').read_text()
    sends = {}
    for operation in ('compute', 'follow'):
        start = source.index(f'try {{ {operation}_client_->async_send_goal(goal, options); }}')
        catch = braced(source[start:], 'catch (const std::exception& e)')
        sends[operation] = source[start:source.index('catch (const std::exception& e)', start)] + catch
    fence = braced(source, 'bool softwareOperationsDone()')
    program = r'''
#include "d1max_navigation_bt/runtime_policy.hpp"
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
using namespace d1max_navigation_bt;
namespace BT { enum class NodeStatus { RUNNING, FAILURE }; }
struct Result { BT::NodeStatus status{BT::NodeStatus::RUNNING}; std::string reason; };
struct Operation : AsyncRequestState { Result result; };
struct Task {
  Operation compute, follow;
  bool retired{false}, authority_retired{true};
  double retired_at{0.};
  std::string retirement_error;
  bool pending() const { return compute.pending() || follow.pending(); }
};
bool authorityRetired(const Task& task) { return task.authority_retired; }
double elapsed(double age) { return age; }
struct Transport {
  void async_send_goal(int, int) { throw std::runtime_error("injected_send_failure"); }
};
struct NavigatorProbe {
  std::map<std::string, std::shared_ptr<Task>> tasks_;
  std::string worker_retirement_fault_;
  double cancellation_timeout_{3.};
  Transport transport;
  Transport* compute_client_{&transport};
  Transport* follow_client_{&transport};
  void clearResolvedStopError(Task&) {} // No physical execution in this test.
  FENCE
  void sendCompute(std::shared_ptr<Task> task) { int goal=0,options=0; COMPUTE }
  void sendFollow(std::shared_ptr<Task> task) { int goal=0,options=0; FOLLOW }
};
int main(int argc, char** argv) {
  const bool follow=std::string(argv[1])=="follow";
  NavigatorProbe node;
  auto task=std::make_shared<Task>(); node.tasks_["old-task"]=task;
  auto& operation=follow?task->follow:task->compute;
  task->authority_retired=!follow;
  operation.begin();
  if(follow)node.sendFollow(task);else node.sendCompute(task);
  std::cout << "failure=" << (operation.result.status==BT::NodeStatus::FAILURE) << '\n';
  std::cout << "pending_after_exception=" << operation.pending() << '\n';
  task->retired=true;
  for(double age:{0.,4.,100.}) {
    task->retired_at=age;
    const bool ready=node.softwareOperationsDone();
    std::cout << "ready_" << int(age) << '=' << ready << '\n';
    std::cout << "fault_" << int(age) << '=' << node.worker_retirement_fault_ << '\n';
  }
  // A late accepted handle must still get targeted cancellation. Its ACK
  // alone is not a terminal result and cannot allow the replacement task.
  std::cout << "cancel_late_handle=" << operation.shouldCancel(true) << '\n';
  operation.cancel_sent=true;
  std::cout << "pending_after_cancel_ack=" << operation.pending() << '\n';
  const bool accepted=operation.acceptTerminal(task->retired);
  task->authority_retired=true; // Exact matching worker retirement ACK.
  std::cout << "late_result_commits=" << accepted << '\n';
  std::cout << "ready_after_result=" << node.softwareOperationsDone() << '\n';
  std::cout << "fault_after_result=" << node.worker_retirement_fault_ << '\n';
  // A definite goal rejection has its own already-terminal proof. It does
  // not pass through an uncertain send failure or wait for a fictitious ACK.
  auto rejected=std::make_shared<Task>();rejected->retired=true;
  rejected->follow.begin();rejected->follow.terminal=true;
  node.tasks_.clear();node.tasks_["rejected-task"]=rejected;
  std::cout << "ready_after_reject=" << node.softwareOperationsDone() << '\n';
}
'''.replace('FENCE', fence).replace('COMPUTE', sends['compute']).replace('FOLLOW', sends['follow'])
    directory = tmp_path_factory.mktemp('goal-send-failure')
    cpp, binary = directory / 'probe.cpp', directory / 'probe'
    cpp.write_text(program)
    subprocess.run(['g++', '-std=c++17', '-I', str(ROOT / 'include'), str(cpp), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize('operation', ['compute', 'follow'])
def test_uncertain_send_keeps_retirement_fence_until_correlated_terminal(send_failure_probe, operation):
    output = subprocess.run([str(send_failure_probe), operation], check=True,
                            capture_output=True, text=True).stdout
    print(f'production_source={ROOT}; operation={operation}\n{output}', end='')
    facts = dict(line.split('=', 1) for line in output.splitlines())
    assert facts['failure'] == '1'
    assert facts['pending_after_exception'] == '1'
    assert facts['ready_0'] == facts['ready_4'] == facts['ready_100'] == '0'
    assert facts['fault_0'] == ''
    assert facts['fault_4'] == facts['fault_100'] == 'previous_worker_cancellation_timeout'
    assert facts['cancel_late_handle'] == facts['pending_after_cancel_ack'] == '1'
    assert facts['late_result_commits'] == '0'
    assert facts['ready_after_result'] == facts['ready_after_reject'] == '1'
    assert facts['fault_after_result'] == ''
