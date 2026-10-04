#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace d1monitor::execution3 {
// Measurements, not ROS knobs. The acceptance record hash binds this policy
// to geometry, firmware, braking evidence and the general-low-speed mode.
struct ExecutionPolicy {
  double sensor_source_age_bound_s=0,command_pipeline_bound_s=0,writer_period_s=0,source_time_uncertainty_s=0;
  double linear_threshold_mps=0,angular_threshold_radps=0,stationary_duration_s=0,reentry_duration_s=0;
  double measured_static_linear_bound_mps=0,measured_static_angular_bound_radps=0;
  double mc_expected_hz=0,mc_min_hz=0,mc_max_hz=0;
  uint32_t minimum_new_samples=0;
  std::string profile;
  double reactionBudget()const {
    return sensor_source_age_bound_s+command_pipeline_bound_s+writer_period_s+source_time_uncertainty_s;
  }
  void validate(double reaction_bound)const {
    auto bounded=[](double v,double lo,double hi){return std::isfinite(v)&&v>=lo&&v<=hi;};
    if(profile!="general_low_speed"||!bounded(sensor_source_age_bound_s,.001,.6)||
       !bounded(command_pipeline_bound_s,.1,.5)||!std::isfinite(writer_period_s)||std::abs(writer_period_s-.05)>1e-9||
       !bounded(source_time_uncertainty_s,0.,.25)||!bounded(reaction_bound,.001,1.)||reactionBudget()>reaction_bound+1e-12)
      throw std::invalid_argument("execution_reaction_budget_not_bound");
    if(!bounded(linear_threshold_mps,.001,.05)||!bounded(angular_threshold_radps,.001,.1)||
       !bounded(measured_static_linear_bound_mps,0.,linear_threshold_mps)||
       !bounded(measured_static_angular_bound_radps,0.,angular_threshold_radps)||
       !bounded(stationary_duration_s,1.,5.)||!bounded(reentry_duration_s,.6,5.)||
       minimum_new_samples<3||minimum_new_samples>512||!bounded(mc_expected_hz,10.,200.)||
       !bounded(mc_min_hz,10.,mc_expected_hz)||!bounded(mc_max_hz,mc_expected_hz,300.))
      throw std::invalid_argument("stationary_evidence_measurements_not_bound");
  }
};
}
