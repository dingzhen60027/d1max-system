#!/usr/bin/env python3
"""Compile the production heading initialization with passive parameter storage.

No ROS init, map replacement, SDK or graph. Collision geometry is exercised by
the separate native GridMap tests; this checks the startup declaration order.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <bspline_opt/spline_heading_contract.hpp>
#include <map>
#include <string>
#include <variant>
#include <iostream>
struct Value {
  std::variant<bool,double> value;
  template<class T>T get()const{return std::get<T>(value);}
  bool as_bool()const{return get<bool>();}
  double as_double()const{return get<double>();}
};
struct Parameters {
  std::map<std::string,Value> overrides;
  const auto& get_parameter_overrides()const{return overrides;}
};
struct Node {
  std::map<std::string,Value> declared;
  Parameters startup;
  Node() {declared={{"grid_map.preview_only",{false}},
      {"grid_map.use_projected_rays",{true}},
      {"grid_map.double_cylinder_radius",{.29}},
      {"grid_map.double_cylinder_offset",{.2}}};}
  bool has_parameter(const std::string&n)const{return declared.count(n);}
  const Value& get_parameter(const std::string&n)const{return declared.at(n);}
  Parameters* get_node_parameters_interface(){return &startup;}
  template<class T>void declare_parameter(const std::string&n,T fallback) {
    auto it=startup.overrides.find(n);
    declared.emplace(n,it==startup.overrides.end()?Value{fallback}:it->second);
  }
};
struct Map {bool strict=false;bool planningRequiresObservedFree()const{return strict;}};
bool initialize(Node*node_,Map*grid_map_) {
  Node*node=node_;
  scan_planner::SplineHeadingContract preview_heading_contract_;
  const auto get_double=[&](const std::string&name,double fallback) {
    if(!node_->has_parameter(name))node_->declare_parameter<double>(name,fallback);
    return node_->get_parameter(name).as_double();
  };
  using namespace scan_planner;
__PRODUCTION__
  return preview_heading_contract_.preview_only_enabled;
}
int main() {
  int failures=0;
  auto check=[&](bool value,const char*name){
    std::cout<<(value?"PASS ":"FAIL ")<<name<<"\n";failures+=!value;};
  Map map;
  Node formal;formal.startup.overrides={{"fsm.execution_protocol",{true}},
      {"fsm.writer_handoff_enabled",{false}}};
  check(initialize(&formal,&map),"undeclared_formal_preview_overrides_enable_measured_yaw");
  check(!formal.has_parameter("fsm.execution_protocol") &&
      !formal.has_parameter("fsm.writer_handoff_enabled"),"reader_does_not_change_fsm_declaration_order");
  Node motion;motion.startup.overrides={{"fsm.execution_protocol",{true}},
      {"fsm.writer_handoff_enabled",{true}}};
  check(!initialize(&motion,&map),"motion_writer_path_remains_unchanged");
  Node missing;missing.startup.overrides={{"fsm.execution_protocol",{true}}};
  check(!initialize(&missing,&map),"missing_writer_override_not_assumed_preview");
  Node declared;declared.startup.overrides=formal.startup.overrides;
  declared.declared["fsm.execution_protocol"]={true};
  declared.declared["fsm.writer_handoff_enabled"]={true};
  check(!initialize(&declared,&map),"declared_effective_writer_takes_precedence");
  Node legacy;legacy.declared["grid_map.preview_only"]={true};
  legacy.startup.overrides["manager.preview_body_heading_contract"]={true};
  check(initialize(&legacy,&map),"legacy_explicit_map_preview_still_supported_without_fsm_flags");
  Node strict;strict.startup.overrides=formal.startup.overrides;map.strict=true;
  check(initialize(&strict,&map),"strict_query_policy_is_not_changed_to_enable_heading");
  return failures?1:0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('    if (!node_->has_parameter("manager.preview_body_heading_contract"))')
    end = source.index('    bspline_optimizer_rebound_.reset(new BsplineOptimizer);', start)
    include = args.source.parents[2] / 'bspline_opt/include'
    with tempfile.TemporaryDirectory(prefix='scan-heading-startup-') as temporary:
        directory = args.build_dir or Path(temporary)
        directory.mkdir(parents=True, exist_ok=True)
        cpp, binary = directory / 'startup.cpp', directory / 'startup'
        cpp.write_text(HARNESS.replace('__PRODUCTION__', source[start:end]))
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra',
                        '-I' + str(include), '-I/usr/include/eigen3',
                        str(cpp), '-o', str(binary)], check=True)
        return subprocess.run([str(binary)], check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
