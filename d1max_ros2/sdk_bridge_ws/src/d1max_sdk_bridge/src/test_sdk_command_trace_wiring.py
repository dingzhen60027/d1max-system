#!/usr/bin/env python3
"""SDK-free checks of production trace-worker isolation and shutdown ordering."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE=Path(__file__).with_name('sdk_monitor_bridge.cpp')

def method(text,signature):
    begin=text.index(signature);start=text.index('{',begin);depth=0
    # These selected methods contain no braces inside string literals.
    for i in range(start,len(text)):
        if text[i]=='{':depth+=1
        elif text[i]=='}':
            depth-=1
            if not depth:return text[begin:i+1]
    raise AssertionError('unterminated method: '+signature)

class Wiring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.text=SOURCE.read_text()
    def test_tick_and_estop_have_no_new_trace_io_or_wait(self):
        for signature in ('void tick(){','EstopSubmission request_soft_estop(',
                          'void shared_estop_tick(){','std::optional<double> execution_tick('):
            body=method(self.text,signature)
            for forbidden in ('execution_trace_file_','execution_trace_.beginDrain','execution_trace_.preparePage',
                              'execution_trace_.commitPage','execution_trace_worker_',
                              'execution_trace_wait_mutex_','execution_trace_wake_'):
                self.assertNotIn(forbidden,body)
    def test_task_feedback_has_independent_latest_publication_and_shutdown(self):
        writer=method(self.text,'std::optional<double> execution_tick(')
        telemetry=method(self.text,'void work_telemetry()')
        start=method(self.text,'void start_execution_workers()')
        self.assertIn('execution_feedback_worker_->put',writer)
        self.assertNotIn('execution_task_feedback_->publish',writer)
        self.assertNotIn('task_feedback',telemetry)
        self.assertIn('TaskFeedbackPublication',start)
        self.assertIn('execution_task_feedback_->publish',start)
        self.assertLess(start.index('TaskFeedbackPublication'),start.index('execution_writer_=std::thread'))
        close=method(self.text,'~Monitor()')
        self.assertLess(close.index('stop_execution_workers()'),close.index('execution_feedback_worker_->close()'))
        self.assertLess(close.index('sdk_->Disconnect(true)'),close.index('execution_feedback_worker_->close()'))
        mock=SOURCE.with_name('sdk_execution_mock.cpp').read_text()
        self.assertIn('TaskFeedbackPublication',mock)
        self.assertIn('feedback_worker_->put(*fact)',mock)
        self.assertNotIn('feedback_->publish',mock)
    def test_only_passive_worker_owns_file_sink(self):
        body=method(self.text,'void work_command_trace()')
        self.assertEqual(self.text.count('execution_trace_file_.write('),1)
        self.assertEqual(self.text.count('execution_trace_.beginDrain('),1)
        self.assertEqual(self.text.count('execution_trace_.preparePage('),1)
        self.assertEqual(self.text.count('execution_trace_.commitPage('),1)
        self.assertIn('execution_trace_file_.write(',body)
        self.assertIn('if(written)execution_trace_.commitPage()',body)
        self.assertIn('flush(1)',body);self.assertIn('flush(2)',body)
        for forbidden in ('sdk_->','execution3_','navigation_events_','raise_navigation_event',
                          'WorkerStopped','request_soft_estop','execution_writes_'):
            self.assertNotIn(forbidden,body)
        self.assertIn('catch(...)',body)
    def test_start_failure_is_diagnostic_only(self):
        body=method(self.text,'void start_command_trace_worker()')
        self.assertIn('std::thread',body);self.assertIn('catch(...)',body)
        self.assertNotIn('raise_navigation_event',body);self.assertNotIn('execution3_',body)
        launch=method(self.text,'void start_execution_workers()')
        self.assertGreater(launch.index('start_command_trace_worker();'),launch.index('execution_writer_=std::thread'))
    def test_stop_precedes_io_join(self):
        body=method(self.text,'~Monitor()')
        request=body.index('execution_trace_stopping_.store(true)')
        stop=body.index('shutdown_execution();')
        disconnect=body.index('sdk_->Disconnect(true)')
        destroyed=body.index('sdk_.reset()')
        join=body.index('execution_trace_worker_.join()')
        self.assertLess(body.index('stop_execution_workers()'),stop)
        self.assertLess(stop,disconnect);self.assertLess(disconnect,destroyed)
        self.assertLess(destroyed,request);self.assertLess(request,join)
        self.assertNotIn('execution_trace_worker_',body[:stop])
    def test_exact_worker_with_blocked_and_throwing_sink(self):
        methods=method(self.text,'void start_command_trace_worker()')+'\n'+method(self.text,'void work_command_trace()')
        header=SOURCE.with_name('sdk_command_trace.hpp')
        code=r'''
#include HEADER
#include <cassert>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <thread>
#include <vector>
#define RCLCPP_INFO(...) do {} while(false)
#define RCLCPP_WARN(...) do {} while(false)
struct Sink {
  std::promise<void> entered,release;std::shared_future<void> gate=release.get_future().share();
  bool fail=false,reject_once=false,entered_once=false;std::filesystem::path path{"unused-no-file-created"};
  std::vector<std::string> attempts,accepted;
  bool write(std::string_view page){
    attempts.emplace_back(page);if(!entered_once){entered_once=true;entered.set_value();}
    gate.wait();if(fail)throw std::runtime_error("slow disk failed");
    if(reject_once){reject_once=false;return false;}
    accepted.emplace_back(page);return true;
  }
  const std::filesystem::path& directory()const{return path;}
  uint64_t failedWrites()const{return 0;}uint64_t rejectedBatches()const{return 0;}
};
struct Harness {
  d1monitor::command_trace::Trace execution_trace_;
  Sink execution_trace_file_;
  bool execution_trace_location_logged_=false,execution_trace_failure_logged_=false;
  std::atomic<bool>execution_trace_stopping_{false};
  std::mutex execution_trace_wait_mutex_;std::condition_variable execution_trace_wake_;
  std::thread execution_trace_worker_;
METHODS
};
int main(){
  using namespace std::chrono_literals;
  for(int failure:{0,1,2}){
    Harness h;h.execution_trace_file_.fail=failure==1;h.execution_trace_file_.reject_once=failure==2;
    d1monitor::command_trace::Record r;r.selected_ns=1;r.begin_ns=2;r.end_ns=3;
    h.execution_trace_.push(r);auto entered=h.execution_trace_file_.entered.get_future();h.start_command_trace_worker();
    assert(entered.wait_for(3s)==std::future_status::ready);
    // The production worker is blocked in the sink. Caller-side urgent work
    // and producer capture do not share its wait/file mutex or executor.
    std::atomic<bool>urgent{false};auto control=std::async(std::launch::async,[&]{
      urgent.store(true);for(int i=0;i<20;++i)h.execution_trace_.push(r);
      h.execution_trace_stopping_.store(true);h.execution_trace_wake_.notify_all();});
    assert(control.wait_for(200ms)==std::future_status::ready);control.get();assert(urgent.load());
    // Real Monitor's static ordering check requires stop/disconnect before
    // this join. Do not claim a blocked filesystem gives bounded process exit.
    h.execution_trace_file_.release.set_value();h.execution_trace_worker_.join();
    if(failure==1){assert(h.execution_trace_.pending()&&h.execution_trace_file_.accepted.empty());}
    else {
      size_t moves=0;for(const auto& page:h.execution_trace_file_.accepted){const auto parsed=nlohmann::json::parse(page);
        for(const auto& e:parsed["events"])if(e[0]=="M")++moves;}
      assert(moves==21&&!h.execution_trace_.pending());
      if(failure==2)assert(h.execution_trace_file_.attempts.size()>=3&&
        h.execution_trace_file_.attempts[0]==h.execution_trace_file_.attempts[1]);
    }
  }
  Harness idle;idle.start_command_trace_worker();const auto at=std::chrono::steady_clock::now();
  idle.execution_trace_stopping_.store(true);idle.execution_trace_wake_.notify_all();idle.execution_trace_worker_.join();
  assert(std::chrono::steady_clock::now()-at<1s);
  // Normal shutdown before the first one-second wake still drains captured
  // events. The real destructor orders the stop flag after SDK destruction.
  Harness tail;d1monitor::command_trace::Record r;r.selected_ns=1;r.begin_ns=2;r.end_ns=3;
  tail.execution_trace_.push(r);tail.execution_trace_file_.release.set_value();tail.start_command_trace_worker();
  tail.execution_trace_stopping_.store(true);tail.execution_trace_wake_.notify_all();tail.execution_trace_worker_.join();
  assert(tail.execution_trace_file_.accepted.size()==1&&!tail.execution_trace_.pending());
}
'''.replace('HEADER',json.dumps(str(header),ensure_ascii=False)).replace('METHODS',methods)
        with tempfile.TemporaryDirectory(prefix='sdk-trace-worker-') as directory:
            source=Path(directory)/'worker.cpp';source.write_text(code)
            executable=Path(directory)/'worker'
            subprocess.run(['g++','-std=c++17','-O2','-pthread','-Wall','-Wextra','-Werror',str(source),'-o',str(executable)],check=True,timeout=30)
            subprocess.run([str(executable)],check=True,timeout=10)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,default=SOURCE)
    args,rest=parser.parse_known_args();SOURCE=args.source
    unittest.main(argv=[__file__]+rest)
