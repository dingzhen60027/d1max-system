#!/usr/bin/env python3
"""Compile the real publication branch; passive sinks, no ROS or planner substitute."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / "src/scan_replan_fsm.cpp"
text = source.read_text()
begin = text.index("      if(!local_follower_)tagged_bspline_pub_->publish(tagged);")
end = text.index("\n    }\n  }", begin)
branch = text[begin:end]
harness = r'''
#include <memory>
#include <optional>
#include <stdexcept>
#include <iostream>
struct Tag {int generation=1;};
struct Proposal {struct {int generation=1;} reference;};
struct Publisher {int calls=0;void publish(const Tag&){++calls;}};
struct Native {int calls=0;void candidate(const Tag&,const Proposal&,int,std::optional<int>){++calls;}};
struct Manager {std::optional<int> acceptedPreviewGeometryEvidence(){return 1;}};
struct Fixture {
  bool local_follower_=true,planning_only_=false;Tag tagged;int heading=0;
  std::shared_ptr<Publisher> tagged_bspline_pub_=std::make_shared<Publisher>();
  std::shared_ptr<Native> execution_validator_=std::make_shared<Native>();
  std::optional<Proposal> execution_proposal_=Proposal{};
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  void publish() {__PRODUCTION_BRANCH__}
};
void expect(bool ok){if(!ok)throw std::runtime_error("candidate publication regression");}
int main(){
  Fixture formal;formal.publish();expect(formal.tagged_bspline_pub_->calls==0&&formal.execution_validator_->calls==1);
  Fixture preview;preview.local_follower_=false;preview.planning_only_=true;preview.publish();
  expect(preview.tagged_bspline_pub_->calls==1&&preview.execution_validator_->calls==1);
  Fixture legacy;legacy.local_follower_=false;legacy.publish();expect(legacy.tagged_bspline_pub_->calls==1);
  Fixture mismatch;mismatch.tagged.generation=2;mismatch.publish();
  expect(mismatch.tagged_bspline_pub_->calls==0&&mismatch.execution_validator_->calls==0);
  Fixture missing;missing.execution_proposal_.reset();missing.publish();
  expect(missing.tagged_bspline_pub_->calls==0&&missing.execution_validator_->calls==0);
  std::cout<<"5 production publication branch checks passed\n";
}
'''
with tempfile.TemporaryDirectory(prefix="candidate-publication-") as directory:
    root = Path(directory)
    cpp, binary = root / "publication.cpp", root / "publication"
    cpp.write_text(harness.replace("__PRODUCTION_BRANCH__", branch))
    subprocess.run(["c++", "-std=c++17", str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
