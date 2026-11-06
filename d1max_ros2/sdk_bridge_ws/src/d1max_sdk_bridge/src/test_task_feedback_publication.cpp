#include "task_feedback_publication.hpp"
#include <cassert>
#include <chrono>
#include <future>
#include <iostream>
#include <vector>
using namespace d1monitor::execution3;
using namespace std::chrono_literals;
int main() {
  // The diagnostic publisher is blocked indefinitely until this fixture
  // releases it. The real independent feedback helper still delivers.
  std::promise<void> diagnostic_entered,diagnostic_release,feedback_seen;
  auto release_diagnostic=diagnostic_release.get_future();
  auto diagnostic=std::async(std::launch::async,[&]{diagnostic_entered.set_value();release_diagnostic.wait();});
  diagnostic_entered.get_future().wait();
  TaskFeedback fact;fact.write_sequence=12;fact.discontinuity_write_sequence=11;
  TaskFeedbackPublication feedback([&](const auto& f){assert(f.write_sequence==12);
    assert(f.discontinuity_write_sequence==11);feedback_seen.set_value();});
  assert(feedback.put(fact));assert(feedback_seen.get_future().wait_for(1s)==std::future_status::ready);
  feedback.close();assert(!feedback.put(fact));diagnostic_release.set_value();diagnostic.get();

  // Its own blocked middleware cannot hold the writer's mailbox mutex or
  // grow a FIFO. The next callback receives only the latest sticky record.
  std::promise<void> entered,release,latest_seen;auto unblock=release.get_future();
  std::vector<std::uint64_t> delivered;
  TaskFeedbackPublication latest([&](const auto& f){delivered.push_back(f.write_sequence);
    if(f.write_sequence==1){entered.set_value();unblock.wait();}
    else {assert(f.write_sequence==100&&f.discontinuity_write_sequence==2);latest_seen.set_value();}});
  fact.write_sequence=1;fact.discontinuity_write_sequence=0;assert(latest.put(fact));entered.get_future().wait();
  for(unsigned i=2;i<=100;++i){fact.write_sequence=i;fact.discontinuity_write_sequence=2;assert(latest.put(fact));}
  release.set_value();assert(latest_seen.get_future().wait_for(1s)==std::future_status::ready);
  latest.close();assert((delivered==std::vector<std::uint64_t>{1,100}));
  std::promise<void> failed;
  TaskFeedbackPublication failure([](const auto&){throw std::runtime_error("blocked publisher failed");},[&]{failed.set_value();});
  assert(failure.put(fact));assert(failed.get_future().wait_for(1s)==std::future_status::ready);
  failure.close();assert(!failure.put(fact));
  {TaskFeedbackPublication idle([](const auto&){assert(false);});} // RAII wakes an idle consumer.

  // Production write_sequence restarts on every grant; arm generation does
  // not. The first fact of task 2 cannot wait for task 1's old write count.
  TaskFeedback old_task;old_task.sdk_session="sdk";old_task.sdk_arm_generation=1;
  old_task.execution_id="task1:execution";old_task.control_epoch=1;
  old_task.version.session_id="nav1";old_task.version.task_id="task1";
  old_task.write_sequence=572;old_task.source_stamp.sec=100;
  auto new_task=old_task;new_task.sdk_arm_generation=2;new_task.execution_id="task2:execution";
  new_task.control_epoch=2;new_task.version.task_id="task2";new_task.write_sequence=1;
  // Wall-clock ordering is not a new gate: the writer's arm is authoritative.
  new_task.source_stamp.sec=99;
  std::promise<void> old_seen,new_seen,geometry_seen,new_navigation_seen;
  TaskFeedbackPublication domains([&](const auto& f){
    if(f.sdk_arm_generation==1)old_seen.set_value();
    else if(f.sdk_arm_generation==3){assert(f.control_epoch==1);new_navigation_seen.set_value();}
    else if(f.write_sequence==1)new_seen.set_value();
    else {assert(f.write_sequence==2&&f.version.reference_generation==123);geometry_seen.set_value();}
  });
  assert(domains.put(old_task));assert(old_seen.get_future().wait_for(1s)==std::future_status::ready);
  assert(domains.put(new_task));assert(new_seen.get_future().wait_for(1s)==std::future_status::ready);
  assert(!domains.put(new_task)); // duplicate
  auto late_old=old_task;late_old.write_sequence=10000;assert(!domains.put(late_old));
  auto wrong_task=new_task;wrong_task.write_sequence=2;wrong_task.version.task_id="task1";
  assert(!domains.put(wrong_task));
  auto wrong_execution=new_task;wrong_execution.write_sequence=2;wrong_execution.execution_id="old";
  assert(!domains.put(wrong_execution));
  auto wrong_session=new_task;wrong_session.sdk_session="retired_sdk";wrong_session.sdk_arm_generation=100;
  assert(!domains.put(wrong_session));
  auto geometry=new_task;geometry.write_sequence=2;geometry.version.reference_generation=123;
  geometry.version.anchor_id="new_curve_anchor";assert(domains.put(geometry));
  assert(geometry_seen.get_future().wait_for(1s)==std::future_status::ready);
  auto navigation=new_task;navigation.sdk_arm_generation=3;navigation.control_epoch=1;
  navigation.version.session_id="nav2";navigation.version.task_id="nav2.1";
  navigation.execution_id="nav2.1:execution";assert(domains.put(navigation));
  assert(new_navigation_seen.get_future().wait_for(1s)==std::future_status::ready);
  assert(!domains.put(geometry)); // prior navigation's late callback
  domains.close();geometry.write_sequence=3;assert(!domains.put(geometry));

  // Cross-task latest replacement remains bounded while old middleware is
  // blocked; an old-domain late fact cannot replace the new task's write 1.
  std::promise<void> domain_entered,domain_release,domain_latest_seen;
  auto domain_unblock=domain_release.get_future();std::vector<std::uint64_t> domain_arms;
  TaskFeedbackPublication domain_latest([&](const auto& f){domain_arms.push_back(f.sdk_arm_generation);
    if(f.sdk_arm_generation==1){domain_entered.set_value();domain_unblock.wait();}
    else {assert(f.write_sequence==1);domain_latest_seen.set_value();}});
  assert(domain_latest.put(old_task));domain_entered.get_future().wait();
  old_task.write_sequence=600;assert(domain_latest.put(old_task));
  assert(domain_latest.put(new_task));assert(!domain_latest.put(late_old));
  domain_release.set_value();assert(domain_latest_seen.get_future().wait_for(1s)==std::future_status::ready);
  domain_latest.close();assert((domain_arms==std::vector<std::uint64_t>{1,2}));
  std::cout<<"task feedback publication: 6 cases passed\n";
}
