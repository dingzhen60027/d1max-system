#pragma once
#include "execution_policy.hpp"
// Explicit synthetic policy for pure unit tests only. Never a live record.
inline d1monitor::execution3::ExecutionPolicy testExecutionPolicy(){
  d1monitor::execution3::ExecutionPolicy p;p.profile="general_low_speed";
  p.sensor_source_age_bound_s=.5;p.command_pipeline_bound_s=.1;p.writer_period_s=.05;p.source_time_uncertainty_s=.1;
  p.linear_threshold_mps=.03;p.angular_threshold_radps=.05;p.stationary_duration_s=1.;p.reentry_duration_s=.6;
  p.measured_static_linear_bound_mps=.01;p.measured_static_angular_bound_radps=.02;p.minimum_new_samples=3;
  p.mc_expected_hz=50.;p.mc_min_hz=40.;p.mc_max_hz=60.;p.validate(.8);return p;
}
