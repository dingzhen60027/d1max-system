#include "execution_shutdown_core.hpp"
#include <cassert>
#include <iostream>
using d1monitor::execution3::ShutdownStopTransaction;
int main(){
 {ShutdownStopTransaction t(10.,false);assert(t.done(10.));assert(!t.due(10.,true));assert(t.attempts()==0);}
 {ShutdownStopTransaction t(10.,true);assert(t.due(10.,true));assert(!t.due(10.01,true));
  t.acknowledge(true);assert(t.done(10.02));assert(t.submitted());assert(t.attempts()==1);
  assert(t.reason()=="shutdown_zero_submitted_stop_unconfirmed");assert(!t.due(10.1,true));}
 {ShutdownStopTransaction t(10.,true);assert(t.due(10.,true));assert(t.due(10.06,true));
  assert(t.due(10.12,true));assert(!t.due(10.18,true));assert(t.done(10.301));
  assert(t.attempts()==3);assert(!t.submitted());t.acknowledge(true);assert(!t.submitted());}
 {ShutdownStopTransaction t(10.,true);assert(!t.due(10.,false));assert(t.done(10.01));
  assert(t.attempts()==0);assert(!t.due(10.1,true));}
 {ShutdownStopTransaction t(10.,true);assert(t.due(10.,true));t.acknowledge(false);
  assert(t.done(10.01));assert(!t.submitted());assert(!t.due(10.1,true));}
 {ShutdownStopTransaction t(10.,true);assert(t.done(9.));assert(!t.due(10.,true));}
 std::cout<<"finite shutdown zero transaction positive/negative assertions passed\n";
}
