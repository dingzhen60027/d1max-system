#!/usr/bin/env python3
"""Compile the actual manager curve-check method against production GridMap.

Only the node clock/logging is passive. No ROS init, graph, transport or SDK;
the native map, inflated query, spline and entire collision checker are real.
--source can point at the frozen pre-change method for same-input evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tempfile

HARNESS = r'''
#include <plan_env/grid_map.h>
#include <plan_manage/trajectory_collision.hpp>
#include <plan_env/solve_budget.hpp>
#include <iostream>
#undef RCLCPP_WARN
#define RCLCPP_WARN(...) ((void)0)
struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={160,80,64};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.planning_collision_policy_="observed_free";
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;
    d.occupancy_buffer_.assign(160*80*64,-1.);
    d.occupancy_buffer_inflate_.assign(160*80*64,0);
    d.occupancy_buffer_inflate_cnt_.assign(160*80*64,0);
    map->rebuildInflationOffsets();return map;
  }
  static void unknown(GridMap &map,const Eigen::Vector3d &point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);
    map.md_.occupancy_buffer_[map.toAddress(cell)]=-1.01;
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
};
struct PassiveStamp {double seconds()const{return 10.;}};
struct PassiveNode {PassiveStamp now()const{return {};}};
namespace scan_planner {
class SCANPlannerManager {
public:
  PassiveNode clock;PassiveNode *node_=&clock;
  GridMap::Ptr grid_map_=GridMapTestAccess::make();
  SplineHeadingContract preview_heading_contract_;
  SolveBudget::Ptr solve_budget_;
  MeasuredBodyPose measured_body_pose_{{.3,0.,.55},Eigen::Quaterniond::Identity(),10.,"odom"};
  std::string measured_body_frame_="odom";double measured_body_maximum_age_=.5;
  bool checkWholeTrajectoryCollision(UniformBspline &position_traj,
    std::size_t query_budget,double wall_budget_seconds,bool diagnostic,double measured_curve_time,
    const SplineHeadingContract &heading_contract,CurveCheckEvidence *evidence
#ifndef FROZEN_CURVE_EVIDENCE
    ,int *query_failure_state
#endif
  );
};
__METHOD__
}
int main() {
  using namespace scan_planner;
  int failures=0;std::cout<<"[";
  for(int scenario=0;scenario<4;++scenario) {
    SCANPlannerManager manager;auto &map=*manager.grid_map_;
    if(scenario==1||scenario==3)GridMapTestAccess::unknown(map,{.775,.025,.55});
    if(scenario==2||scenario==3)map.setOccupied({.025,.025,.55});
    const int actual=map.getPlanningInflateOccupancy({.3,0.,.55},0.);
    map.resetCollisionDiagnostics();
    Eigen::MatrixXd controls(3,7);
    for(int i=0;i<controls.cols();++i)controls.col(i)=Eigen::Vector3d(.3+(i-1)*.05,0.,.55);
    UniformBspline curve(controls,3,.5);CurveCheckEvidence evidence;
    int state=0;
    const bool safe=manager.checkWholeTrajectoryCollision(curve,50000,0.,false,0.,{},&evidence
#ifndef FROZEN_CURVE_EVIDENCE
      ,&state
#endif
    );
    const auto unknown=map.unknownCollisionQueries();
#ifdef FROZEN_CURVE_EVIDENCE
    const std::string phase=safe?"clear":(unknown>0?"waiting_observed_space":"failed_final_collision");
    const bool expected=(scenario==0?safe:!safe);
#else
    const std::string phase=safe?"clear":(state==2?"waiting_observed_space":
      (state==1?"failed_final_collision":"failed_curve_validation"));
    const auto expected_evidence=scenario==0?CurveCheckEvidence::Clear:
      (scenario==1?CurveCheckEvidence::Uncertified:CurveCheckEvidence::Occupied);
    const bool expected=(scenario==0?safe:!safe)&&evidence==expected_evidence&&state==actual;
#endif
    failures+=!expected;
    if(scenario)std::cout<<",";
    std::cout<<"{\"scenario\":"<<scenario<<",\"actual_query\":"<<actual<<
      ",\"safe\":"<<(safe?"true":"false")<<",\"evidence\":"<<int(evidence)<<
      ",\"captured_state\":"<<state<<",\"unknown_diagnostic_count\":"<<unknown<<
      ",\"phase\":\""<<phase<<"\"}";
  }
  std::cout<<"]\n";return failures?1:0;
}
'''


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'src/planner_manager.cpp')
    parser.add_argument('--native-build', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('  bool SCANPlannerManager::checkWholeTrajectoryCollision(')
    end = source.index('  bool SCANPlannerManager::commitPreviewCandidate()', start)
    method = source[start:end]
    frozen = 'int *query_failure_state' not in method
    # Release builds intentionally omit test executables. In that case reuse
    # the actual node's generated compiler/link inputs, not guessed libraries.
    target = 'test_initial_yaw_native'
    if not (args.native_build/('CMakeFiles/'+target+'.dir/flags.make')).is_file():
        target = 'scan_planner_node'
    flags_file = args.native_build/('CMakeFiles/'+target+'.dir/flags.make')
    flags = {}
    for line in flags_file.read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    link = shlex.split((args.native_build/('CMakeFiles/'+target+'.dir/link.txt')).read_text())
    with tempfile.TemporaryDirectory(prefix='scan-manager-curve-evidence-') as directory:
        scratch = Path(directory)
        cpp, obj, binary = scratch/'method.cpp', scratch/'method.o', scratch/'method'
        cpp.write_text(HARNESS.replace('__METHOD__', method))
        compile_command = ['/usr/bin/c++'] + flags['CXX_DEFINES'] + flags['CXX_INCLUDES'] + flags['CXX_FLAGS']
        if frozen:
            compile_command.append('-DFROZEN_CURVE_EVIDENCE')
        subprocess.run(compile_command + ['-c', str(cpp), '-o', str(obj)], check=True)
        # Link only the extracted method harness; never include the production
        # main/FSM objects or launch a node. Retain its real installed libraries.
        objects = [i for i, value in enumerate(link) if value.endswith('.o')]
        if not objects:
            raise ValueError('native_generated_link_has_no_object_inputs')
        link = [str(obj) if i == objects[0] else value for i, value in enumerate(link)
                if i not in objects[1:]]
        for i, argument in enumerate(link):
            if i and link[i-1] == '-o':
                link[i] = str(binary)
        subprocess.run(link, cwd=args.native_build, check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True, check=False)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if args.report:
            args.report.write_text(json.dumps(dict(source=str(args.source),
                source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                method_sha256=hashlib.sha256(method.encode()).hexdigest(),
                frozen_method=frozen,returncode=result.returncode,
                generated_native_target=target,
                native_scenarios=json.loads(result.stdout),stderr=result.stderr,
                source_scope='actual manager method + production GridMap/UniformBspline/whole checker; passive clock/logging only',
                ros_initialized=False,robot_connected=False),indent=2)+'\n')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
