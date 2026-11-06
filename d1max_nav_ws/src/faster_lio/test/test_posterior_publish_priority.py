"""Run the actual production publication tail with a blocked legacy writer.

No ROS or robot. Only publisher behavior is replaced; extracted statements are
the unchanged production dispatch. This proves ordering, not live LIO latency.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE=Path(__file__).resolve().parents[1]/'src/laser_mapping.cc'
CPP=r'''
#include <atomic>
#include <memory>
#include <stdexcept>
#include <nlohmann/json.hpp>
using json=nlohmann::json;
struct LocalizationPipelineTrace {enum Stage {LegacyOdomBegin,LegacyOdomEnd};};
struct Fixture;
struct Publisher {Fixture* owner;void publish(const json& value);};
struct Fixture {
  bool robust_mode_=true,publish_tf_=false,fail_legacy=false;
  std::atomic<bool> robust_fault_{false},robust_imu_clock_reset_{false};
  double lidar_end_time_=100.1,wall=100.15,legacy_delay=.4;
  json odom_aft_mapped_={{"stamp_ns",100100000000LL},{"epoch",7},
    {"pose",{1.,2.,.5}},{"velocity",{.3,0.,0.}},{"covariance",42}};
  json events=json::array();
  std::shared_ptr<Publisher> pub_odom_aft_mapped=std::make_shared<Publisher>(Publisher{this});
  int64_t BeginRobustPhase(LocalizationPipelineTrace::Stage,double){return 0;}
  void EndRobustPhase(LocalizationPipelineTrace::Stage,double,int64_t){}
  void PublishRobustSample(bool valid){events.push_back({{"name","formal"},{"at",wall},
    {"valid",valid},{"payload",odom_aft_mapped_}});}
  void dispatch(){
@@BODY@@
  }
};
void Publisher::publish(const json& value){
  owner->wall+=owner->legacy_delay;
  if(owner->fail_legacy)throw std::runtime_error("legacy failure");
  owner->events.push_back({{"name","legacy"},{"at",owner->wall},{"payload",value}});
}
int main(){json out;
  for(const char* name:{"normal","slow","fault","clock","mapping","legacy_failure"}){
    Fixture f;
    if(std::string(name)=="normal")f.legacy_delay=0;
    if(std::string(name)=="fault")f.robust_fault_=true;
    if(std::string(name)=="clock")f.robust_imu_clock_reset_=true;
    if(std::string(name)=="mapping")f.robust_mode_=false;
    if(std::string(name)=="legacy_failure")f.fail_legacy=true;
    bool threw=false;try{f.dispatch();}catch(const std::exception&){threw=true;}
    out[name]={{"events",f.events},{"threw",threw}};
  }
  std::cout<<out.dump();
}
'''


def extracted_tail(path):
    text=path.read_text()
    begin=text.index('void LaserMapping::PublishOdometry(')
    start=text.index('odom_aft_mapped_.twist.covariance[6*i+j]=twist_cov(i,j);',begin)
    start=text.index('\n',start)+1
    end=text.index('    if(!publish_tf_)return;',start)+len('    if(!publish_tf_)return;')
    return text[start:end]


def evaluate(path):
    with tempfile.TemporaryDirectory(prefix='lio_posterior_order_') as directory:
        cpp=Path(directory)/'main.cpp';binary=Path(directory)/'test'
        cpp.write_text('#include <iostream>\n'+CPP.replace('@@BODY@@',extracted_tail(path)))
        subprocess.run(['g++','-std=c++17','-O2',str(cpp),'-o',str(binary)],check=True,capture_output=True)
        return json.loads(subprocess.check_output([str(binary)],text=True))


class PosteriorPriority(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.result=evaluate(SOURCE)

    def test_slow_compatibility_writer_cannot_delay_original_posterior(self):
        case=self.result['slow']['events'];formal=next(x for x in case if x['name']=='formal')
        self.assertAlmostEqual(formal['at'],100.15)
        self.assertLess(formal['at']-formal['payload']['stamp_ns']*1e-9,.35)

    def test_original_sample_and_legacy_payload_remain_identical(self):
        for name in ('normal','slow','fault','clock'):
            events=self.result[name]['events'];self.assertEqual(len(events),2)
            self.assertEqual(events[0]['payload'],events[1]['payload'])
            self.assertEqual(events[0]['payload']['stamp_ns'],100100000000)

    def test_faults_never_become_valid_by_reordering(self):
        for name in ('fault','clock'):
            self.assertFalse(next(x for x in self.result[name]['events'] if x['name']=='formal')['valid'])

    def test_nonrobust_mapping_compatibility_is_unchanged(self):
        self.assertEqual([x['name'] for x in self.result['mapping']['events']],['legacy'])

    def test_completed_posterior_survives_legacy_publisher_exception(self):
        case=self.result['legacy_failure'];self.assertTrue(case['threw'])
        self.assertEqual([x['name'] for x in case['events']],['formal'])


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,default=SOURCE)
    args,rest=parser.parse_known_args();SOURCE=args.source
    unittest.main(argv=['test_posterior_publish_priority.py']+rest)
