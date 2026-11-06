"""Passive real navigator dependency-callback and stage-selection regression."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_arrival_event_wiring_no_ros import block


ROOT = Path(os.environ.get('D1MAX_BT_SOURCE_ROOT', Path(__file__).resolve().parents[3]))


class DependencyWiring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source=(ROOT/'src/d1max_navigation_bt/src/navigator_node.cpp').read_text()
        callback=block(source[source.index('    dependency_sub_='):], '[this](')
        callback=callback[callback.index('{')+1:-1]
        phase=(block(source,'  bool globalDependencyRequired()') if '  bool globalDependencyRequired()' in source
               else 'bool globalDependencyRequired() const{return true;}')
        access=(block(source,'  std::string dependencyBlocker(') if '  std::string dependencyBlocker(' in source
                else 'std::string dependencyBlocker(double at){return dependency_health_.blocker(at);}')
        harness=r'''
#include "d1max_navigation_bt/dependency_health.hpp"
#include "d1max_navigation_bt/runtime_policy.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <iostream>
#include <memory>
#include <map>
using namespace d1max_navigation_bt;
namespace BT{enum class NodeStatus{RUNNING,SUCCESS};}
struct Task {struct Operation{struct{BT::NodeStatus status=BT::NodeStatus::SUCCESS;}result;
  bool work=false;bool pending()const{return work;}}compute;};
struct PassiveNavigator {
  DependencyHealthGuard dependency_health_{true};
  std::string session_="session",planning_scope_="navigation";
  std::shared_ptr<Task>current_=std::make_shared<Task>();
  std::map<std::string,std::shared_ptr<Task>>tasks_{{"task",current_}};
  double wall=100.,mono=10.;
  double wallTime()const{return wall;}double monotonic()const{return mono;}
  bool taskFollower()const{return true;}
PHASE
ACCESS
  void receive(const nlohmann::json&value){
    struct Message{std::string data;};auto m=std::make_shared<Message>(Message{value.dump()});
CALLBACK
  }
};
int main(int argc,char**argv){
  if(argc!=2)return 2;std::string scenario=argv[1];PassiveNavigator n;
  nlohmann::json parts=nlohmann::json::array();
  for(auto name:{"global","reference","tracker"})parts.push_back({{"name",name},{"required",true},
    {"seen",true},{"callback_age_s",0.},{"producer_source_time",100.},{"recovering",false}});
  if(scenario=="global_committed"||scenario=="global_pending"||scenario=="cancel_pending") {
    parts[0]["callback_age_s"]=2.;parts[0]["producer_source_time"]=98.;
  }
  if(scenario=="global_pending")n.current_->compute.result.status=BT::NodeStatus::RUNNING;
  if(scenario=="cancel_pending") {
    auto old=std::make_shared<Task>();old->compute.work=true;n.tasks_["old-cancelling"]=old;
  }
  if(scenario=="tracker_stalled") {parts[2]["callback_age_s"]=2.;parts[2]["producer_source_time"]=98.;}
  nlohmann::json v{{"schema",1U},{"sequence",1ULL},{"session_id","session"},{"ready",false},
    {"fatal",false},{"reason","component_executor_stalled:global"},{"observed_at_unix",100.},{"components",parts}};
  if(scenario=="fresh"||scenario=="monitor_died")v["ready"]=true;
  if(scenario=="fatal"){v["fatal"]=true;v["reason"]="process_died";}
  if(scenario=="foreign")v["session_id"]="foreign";
  n.receive(v);
  if(scenario=="monitor_died")n.mono=12.;
  auto reason=n.dependencyBlocker(n.mono);
  bool expected=scenario=="fresh"||scenario=="global_committed";
  std::cout<<scenario<<": ready="<<reason.empty()<<" expected="<<expected<<" reason="<<reason<<"\n";
  return reason.empty()==expected?0:1;
}
'''.replace('PHASE',phase).replace('ACCESS',access).replace('CALLBACK',callback)
        cls.directory=tempfile.TemporaryDirectory(prefix='bt-dependency-')
        cpp=Path(cls.directory.name)/'probe.cpp';cpp.write_text(harness)
        cls.binary=Path(cls.directory.name)/'probe'
        subprocess.run(['c++','-std=c++17','-O0','-I',str(ROOT/'src/d1max_navigation_bt/include'),
            str(cpp),'-o',str(cls.binary)],check=True,capture_output=True,text=True)

    @classmethod
    def tearDownClass(cls):cls.directory.cleanup()

    def run_case(self,name):
        result=subprocess.run([str(self.binary),name],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)


for _case in ('fresh','global_committed','global_pending','cancel_pending','tracker_stalled','fatal','foreign','monitor_died'):
    setattr(DependencyWiring,'test_'+_case,lambda self,name=_case:self.run_case(name))


if __name__=='__main__':unittest.main()
