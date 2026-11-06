"""Compile actual diagnostic consumer; no ROS nodes, SDK or network."""
from pathlib import Path
import json
import subprocess

from test_geometry_exchange_wiring import body


def test_actual_tracker_drain_separates_task_retirement_and_idle(tmp_path):
    package = Path(__file__).resolve().parents[1]
    source = (package/'src/local_follower_node.cpp').read_text()
    method = body(source, 'void flushControlTrace()')
    cpp = tmp_path/'probe.cpp'
    cpp.write_text(r'''
#include "d1max_trajectory_tracker/control_trace_diagnostics.hpp"
#include "d1max_planning_interfaces/bounded_diagnostic_file.hpp"
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
using namespace d1max_trajectory_tracker;
class Probe {
 public:
  Probe(const char* path):trace_file_("tracker",path,4096,4096){}
  struct Clock{long nanoseconds()const{return 123456789;}};
  Clock now(){return {};}
  static double receipt(){return 2000.;}
  ControlTraceBuffer control_trace_;
  d1max_diagnostics::BoundedTaskDiagnosticFile trace_file_;
  bool trace_failure_reported_{false},trace_path_logged_{false};
''' + method + r'''
};
ControlTraceRecord record(const std::string& task,bool active,double time,int seq){
  ControlTraceRecord r;auto id=std::make_shared<ControlTraceIdentity>();
  id->session="s";id->task=task;id->id=seq;r.identity=id;r.task_active=active;
  r.tick.current.steady_s=time;r.tick.current.source_ns=123456789;r.tick.current.command_sequence=seq;
  return r;
}
int main(int argc,char**argv){Probe p(argv[1]);
  p.control_trace_.push(record("first",true,100.,1));
  p.control_trace_.push(record("first",false,100.1,2));
  p.control_trace_.push(record("second",true,100.2,3));
  p.flushControlTrace();
  for(int n=0;n<100;++n){p.control_trace_.push(record("second",false,1000.+n,n+4));p.flushControlTrace();}
  return p.trace_failure_reported_?1:0;
}
''')
    binary = tmp_path/'probe'
    subprocess.run(['c++', '-std=c++17', '-O2', '-pthread', '-I'+str(package/'include'),
                    '-I'+str(package.parent/'d1max_planning_interfaces/include'),
                    str(cpp), '-o', str(binary)], check=True)
    root = tmp_path/'traces'
    subprocess.run([str(binary), str(root)], check=True)
    component = next(root.iterdir())
    def ticks(bucket):
        return [tick[0] for file in component.glob(bucket+'-*/trace-*.jsonl')
                for line in file.read_text().splitlines()
                for tick in json.loads(line)['payload']['ticks']]
    assert ticks('task0') == [1, 2]
    assert ticks('task1') == [3]
    assert ticks('idle')
    assert len([path for path in component.glob('task*-*') if path.is_dir()]) == 2
