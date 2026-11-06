#!/usr/bin/env python3
"""Compile production diagnostic/OptimalTraj methods with passive I/O only.

Real messages and UniformBspline are used. No ROS context, node, network or SDK
is created. RMW late-subscriber delivery itself is deliberately not claimed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess


HARNESS = r'''
#include <plan_manage/execution_validator.hpp>
#include <plan_manage/local_plan_debug.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/parameter.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <map>
#include <tuple>
#include <iostream>
template<class M> struct PassivePublisher {
  std::vector<M> messages;bool durable=false;
  size_t get_subscription_count()const{return 0;}
  void publish(const M& m){messages.push_back(m);}
};
struct PassiveNode {
  bool has_parameter(const std::string&)const{return false;}
  rclcpp::Parameter get_parameter(const std::string&)const{return rclcpp::Parameter("frame","odom");}
  rclcpp::Time now()const{return rclcpp::Time(100000000000LL,RCL_ROS_TIME);}
  template<class M> auto create_publisher(const std::string&,const rclcpp::QoS& q) {
    auto p=std::make_shared<PassivePublisher<M>>();
    p->durable=q.get_rmw_qos_profile().durability==RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;return p;
  }
};
namespace scan_planner {
class PlanningVisualization {
public:
  using MarkerPublisher=PassivePublisher<visualization_msgs::msg::Marker>;
  PassiveNode* node_;std::string frame_id_="odom";
  std::shared_ptr<MarkerPublisher> goal_point_pub,global_list_pub,init_list_pub,optimal_list_pub,a_star_list_pub;
  explicit PlanningVisualization(PassiveNode*);
  void displayOptimalTraj(UniformBspline,int);void clearOptimalTraj(int);
};
__VISUAL_METHODS__
struct PassiveLedger {
  std::optional<ExecutionValidator::CommittedView> view;
  unsigned key_reads=0,geometry_reads=0;
  auto committedDiagnosticAt(std::int64_t,double) {
    ++key_reads;
    std::optional<ExecutionValidator::CommittedDiagnosticKey> result;
    if(view) {
      const bool accepted=view->proof&&view->proof->valid&&view->debug.has_value();
      result=ExecutionValidator::CommittedDiagnosticKey{executionVersion(view->spline),
        view->spline.trajectory.traj_id,accepted,accepted?"accepted":view->proof&&!view->proof->valid?
          view->proof->reason:"waiting_current_validation"};
    }return result;
  }
  auto committedView(){++geometry_reads;return view;}
};
class SCANReplanFSM {
public:
  PassiveNode* node_;std::shared_ptr<PassiveLedger> execution_validator_=std::make_shared<PassiveLedger>();
  std::shared_ptr<PassivePublisher<ew::LocalPlanDebug>> local_plan_debug_pub_=std::make_shared<PassivePublisher<ew::LocalPlanDebug>>();
  std::shared_ptr<PlanningVisualization> visualization_;
  using ExecutionDebugKey=std::tuple<ew::ExecutionVersion,std::int64_t,std::string>;
  std::optional<ExecutionDebugKey> last_execution_debug_key_;
  std::chrono::steady_clock::time_point last_execution_debug_publish_{};
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_visualized_committed_;
  double odom_timeout_=.4;
  explicit SCANReplanFSM(PassiveNode* n):node_(n),visualization_(std::make_shared<PlanningVisualization>(n)){}
  std_msgs::msg::Header localPlanDebugHeader(){std_msgs::msg::Header h;h.frame_id="odom";h.stamp=node_->now();return h;}
  bool publishExecutionLedgerDebug();
};
__DIAGNOSTIC_METHOD__
}
int main() {
  using namespace scan_planner;
  unsigned checks=0,failures=0;
  const auto expect=[&](bool ok,const char* name){++checks;failures+=!ok;std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";};
  PassiveNode node;PlanningVisualization visual(&node);
  Eigen::MatrixXd controls(3,6);for(int i=0;i<6;++i)controls.col(i)=Eigen::Vector3d(.1*i,0.,.55);
  UniformBspline curve(controls,3,.1);
  visual.displayOptimalTraj(curve,0);
  const auto& m=visual.optimal_list_pub->messages;
  expect(visual.optimal_list_pub->durable&&m.size()==2,
    "production_durable_publisher_caches_curve_even_without_current_subscriber");
  expect(m.size()==2&&m[0].id==0&&m[1].id==1000&&
    m[0].lifetime.sec==0&&m[0].lifetime.nanosec==0&&m[1].lifetime.sec==0&&m[1].lifetime.nanosec==0,
    "immutable_curve_uses_stable_ids_and_explicit_infinite_lifetime");
  visual.clearOptimalTraj(0);
  expect(m.size()==4&&m[2].id==0&&m[3].id==1000&&
    m[2].action==visualization_msgs::msg::Marker::DELETE&&m[3].action==visualization_msgs::msg::Marker::DELETE,
    "clear_retires_only_two_local_curve_ids_on_same_durable_publisher");
  SCANReplanFSM f(&node);ExecutionValidator::CommittedView v;
  v.spline.schema_version=2;v.spline.session_id="session";v.spline.task_id="task";v.spline.route_id="route";
  v.spline.route_hash=std::string(64,'a');v.spline.map_version_id="map";v.spline.generation=1;
  v.spline.trajectory.order=3;v.spline.trajectory.traj_id=1;
  const auto knots=curve.getKnot();
  for(int i=0;i<controls.cols();++i){geometry_msgs::msg::Point p;p.x=controls(0,i);p.y=controls(1,i);p.z=controls(2,i);v.spline.trajectory.pos_pts.push_back(p);}
  for(int i=0;i<knots.size();++i)v.spline.trajectory.knots.push_back(knots[i]);
  v.debug=ew::LocalPlanDebug();v.proof=ew::TrajectoryValidation();v.proof->valid=true;
  v.proof->valid_until=rclcpp::Time(100200000000LL);v.proof->body_source_stamp=node.now();
  f.execution_validator_->view=v;
  expect(f.publishExecutionLedgerDebug(),"first_committed_status_publishes");
  const auto drawn=f.visualization_->optimal_list_pub->messages.size();
  for(unsigned i=0;i<100;++i)f.publishExecutionLedgerDebug();
  expect(f.execution_validator_->geometry_reads==1&&f.local_plan_debug_pub_->messages.size()==1&&
    drawn==2&&f.visualization_->optimal_list_pub->messages.size()==2,
    "unchanged_100Hz_polls_use_light_key_not_geometry_copy_or_curve_redraw");
  f.last_execution_debug_publish_=std::chrono::steady_clock::now()-std::chrono::milliseconds(51);
  f.publishExecutionLedgerDebug();
  expect(f.local_plan_debug_pub_->messages.size()==2&&f.visualization_->optimal_list_pub->messages.size()==2,
    "diagnostic_20Hz_heartbeat_does_not_replot_immutable_committed_curve");
  f.execution_validator_->view->proof->valid=false;f.execution_validator_->view->proof->reason="occupied";
  f.publishExecutionLedgerDebug();
  expect(f.local_plan_debug_pub_->messages.size()==3&&!f.local_plan_debug_pub_->messages.back().valid&&
    f.local_plan_debug_pub_->messages.back().phase=="occupied"&&f.visualization_->optimal_list_pub->messages.size()==2,
    "true_negative_status_bypasses_visual_throttle_without_deleting_old_geometry");
  f.execution_validator_->view->proof->valid=true;f.publishExecutionLedgerDebug();
  expect(f.local_plan_debug_pub_->messages.size()==4&&f.visualization_->optimal_list_pub->messages.size()==2,
    "same_curve_positive_return_does_not_reconstruct_curve_or_renew_its_geometry");
  f.execution_validator_->view->spline.trajectory.traj_id=2;f.publishExecutionLedgerDebug();
  expect(f.visualization_->optimal_list_pub->messages.size()==4,
    "new_actual_committed_curve_identity_updates_same_stable_marker_ids");
  f.execution_validator_->view.reset();f.publishExecutionLedgerDebug();f.publishExecutionLedgerDebug();
  const auto& cleared=f.visualization_->optimal_list_pub->messages;
  expect(cleared.size()==6&&cleared[4].action==visualization_msgs::msg::Marker::DELETE&&
    cleared[5].action==visualization_msgs::msg::Marker::DELETE,
    "retirement_clears_local_curve_once_and_does_not_clear_fixed_global_route");
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";return failures?1:0;
}
'''


def method(source, signature):
    start=source.index(signature)
    opened=source.index('{',start)
    depth=1
    end=opened+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}')
        end+=1
    return source[start:end]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    package=Path(__file__).resolve().parents[1]
    parser.add_argument('--source',type=Path,default=package/'src/scan_replan_fsm.cpp')
    parser.add_argument('--native-build',type=Path,required=True)
    parser.add_argument('--build-dir',type=Path,required=True)
    args=parser.parse_args()
    visual=(package.parent/'traj_utils/src/planning_visualization.cpp').read_text()
    visual_methods='\n'.join(method(visual,s) for s in
      ('  PlanningVisualization::PlanningVisualization(',
       '  void PlanningVisualization::displayOptimalTraj(',
       '  void PlanningVisualization::clearOptimalTraj('))
    visual_methods=visual_methods.replace('rclcpp::Node *node','PassiveNode *node',1)
    diagnostic=method(args.source.read_text(),'  bool SCANReplanFSM::publishExecutionLedgerDebug()')
    args.build_dir.mkdir(parents=True,exist_ok=True)
    cpp=args.build_dir/'visual_diagnostics.cpp'
    cpp.write_text(HARNESS.replace('__VISUAL_METHODS__',visual_methods).replace('__DIAGNOSTIC_METHOD__',diagnostic))
    metadata=args.native_build/'CMakeFiles/scan_planner_node.dir'
    variables=dict(re.findall(r'^(CXX_\w+) = (.*)$',(metadata/'flags.make').read_text(),re.M))
    flags=['-std=c++17','-I'+str(package/'include')]
    for name in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS'):
        flags+=shlex.split(variables[name])
    obj=args.build_dir/'visual_diagnostics.o'
    subprocess.run(['/usr/bin/c++',*flags,'-c',str(cpp),'-o',str(obj)],check=True)
    original=shlex.split((metadata/'link.txt').read_text())
    executable=args.build_dir/'visual_diagnostics'
    subprocess.run([original[0],str(obj),'-o',str(executable),*original[original.index('-o')+2:]],check=True)
    result=subprocess.run([str(executable)],capture_output=True,text=True)
    print(result.stdout+result.stderr,end='')
    (args.build_dir/'result.json').write_text(json.dumps(dict(returncode=result.returncode,
      stdout=result.stdout,stderr=result.stderr,source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
      production_methods_sha256=hashlib.sha256((visual_methods+diagnostic).encode()).hexdigest(),
      ros_initialized=False,scope='production diagnostic/marker functions; passive publisher, node clock and diagnostic ledger seam; actual RMW late delivery untested'),indent=2)+'\n')
    return result.returncode


if __name__=='__main__':
    raise SystemExit(main())
