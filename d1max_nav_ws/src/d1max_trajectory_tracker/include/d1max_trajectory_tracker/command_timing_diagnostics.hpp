#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

namespace d1max_trajectory_tracker {

// Passive differences at the control owner's actual monotonic timestamps.
// These are command derivatives, not measured robot acceleration or SDK send
// timing. No allocation, command substitution, reset or control decision.
class CommandTimingDiagnostics {
 public:
  struct Window {
    std::uint64_t samples{0}, intervals{0}, normal_intervals{0},
      nonmonotonic_samples{0}, stop_edges{0}, constraint_overrides{0};
    double max_interval_s{0.}, max_forward_acceleration{0.}, max_yaw_acceleration{0.},
      max_forward_jerk{0.}, max_yaw_jerk{0.},
      normal_max_forward_acceleration{0.}, normal_max_yaw_acceleration{0.},
      normal_max_forward_jerk{0.}, normal_max_yaw_jerk{0.};
  };

  void observe(double stamp, double forward, double yaw, bool normal, bool constraint_override = false) {
    if (!std::isfinite(stamp) || !std::isfinite(forward) || !std::isfinite(yaw)) return;
    ++window_.samples;
    if (constraint_override) ++window_.constraint_overrides;
    if (have_sample_) {
      const double dt = stamp - stamp_;
      if (dt <= 0.) { ++window_.nonmonotonic_samples; return; }
      ++window_.intervals;
      window_.max_interval_s = std::max(window_.max_interval_s, dt);
      const bool stopped = forward == 0. && yaw == 0.;
      if (stopped && (forward_ != 0. || yaw_ != 0.)) ++window_.stop_edges;
      const double a = (forward - forward_) / dt;
      const double yaw_a = (yaw - yaw_) / dt;
      peak(window_.max_forward_acceleration, a);
      peak(window_.max_yaw_acceleration, yaw_a);
      const bool normal_interval = normal && normal_;
      if (normal_interval) {
        ++window_.normal_intervals;
        peak(window_.normal_max_forward_acceleration, a);
        peak(window_.normal_max_yaw_acceleration, yaw_a);
      }
      if (have_acceleration_) {
        const double jerk = (a - acceleration_) / dt;
        const double yaw_jerk = (yaw_a - yaw_acceleration_) / dt;
        peak(window_.max_forward_jerk, jerk);
        peak(window_.max_yaw_jerk, yaw_jerk);
        if (normal_interval && normal_acceleration_) {
          peak(window_.normal_max_forward_jerk, jerk);
          peak(window_.normal_max_yaw_jerk, yaw_jerk);
        }
      }
      acceleration_ = a; yaw_acceleration_ = yaw_a;
      have_acceleration_ = true; normal_acceleration_ = normal_interval;
    }
    stamp_ = stamp; forward_ = forward; yaw_ = yaw; normal_ = normal; have_sample_ = true;
  }

  Window takeWindow() {
    const Window out = window_;
    window_ = Window{};
    // Retain differences across a report or ordinary curve replacement.
    return out;
  }

 private:
  static void peak(double& current, double value) { current = std::max(current, std::abs(value)); }
  Window window_;
  double stamp_{0.}, forward_{0.}, yaw_{0.}, acceleration_{0.}, yaw_acceleration_{0.};
  bool have_sample_{false}, have_acceleration_{false}, normal_{false}, normal_acceleration_{false};
};

// Detect demand changes, never decide or modify motion. Source/identity fields
// are copied for correlation; derivatives use only the real steady tick clock.
class CommandTransitionDiagnostics {
 public:
  enum Kind : unsigned { Zero = 1U, TranslationZero = 2U, SpeedDrop = 4U };
  struct Sample {
    double steady_s{0.},forward{0.},lateral{0.},yaw{0.};
    std::int64_t source_ns{0},body_source_ns{0},curve_id{-1};
    std::uint64_t command_sequence{0},reference_generation{0};
  };
  struct Transition {Sample previous,current;double interval_s{0.};unsigned kind{0};};

  std::optional<Transition> observeTick(const Sample& sample) {
    if(!finite(sample)){++invalid_samples_;return {};}
    if(!previous_){previous_=sample;return Transition{sample,sample,0.,0U};}
    const double interval=sample.steady_s-previous_->steady_s;
    if(interval<=0.){++nonmonotonic_samples_;return {};}
    const double before=std::hypot(previous_->forward,previous_->lateral);
    const double after=std::hypot(sample.forward,sample.lateral);
    unsigned kind=0;
    if(after==0.&&sample.yaw==0.&&(before!=0.||previous_->yaw!=0.))kind|=Zero;
    if(after==0.&&before!=0.)kind|=TranslationZero;
    // Logging sensitivity only: a >= 0.05 m/s AND >= 20% one-tick drop.
    // These constants are not a speed/acceleration cap or control gate.
    if(before-after>=.05-1e-12&&before-after>=.2*before)kind|=SpeedDrop;
    const Transition result{*previous_,sample,interval,kind};previous_=sample;
    if(kind)++events_;
    return result;
  }
  std::optional<Transition> observe(const Sample& sample) {
    auto result=observeTick(sample);
    return result&&result->kind?result:std::optional<Transition>{};
  }
  std::uint64_t events()const{return events_;}
  std::uint64_t invalidSamples()const{return invalid_samples_;}
  std::uint64_t nonmonotonicSamples()const{return nonmonotonic_samples_;}

 private:
  static bool finite(const Sample& s){return std::isfinite(s.steady_s)&&std::isfinite(s.forward)&&
    std::isfinite(s.lateral)&&std::isfinite(s.yaw);}
  std::optional<Sample> previous_;
  std::uint64_t events_{0},invalid_samples_{0},nonmonotonic_samples_{0};
};

// Single-owner diagnostic storage; no new worker or lock. Overflow is visible,
// oldest unreported events are retained and no queue grows with run duration.
template<class Event,std::size_t Capacity>
class BoundedDiagnosticEvents {
 public:
  static_assert(Capacity>0,"diagnostic_capacity_must_be_nonzero");
  bool push(Event event){
    if(size_==Capacity){++dropped_;return false;}
    events_[(begin_+size_)%Capacity]=std::move(event);++size_;return true;
  }
  std::optional<Event> pop(){
    if(!size_)return {};
    auto result=std::move(events_[begin_]);events_[begin_].reset();
    begin_=(begin_+1)%Capacity;--size_;return result;
  }
  std::size_t size()const{return size_;}
  std::uint64_t dropped()const{return dropped_;}
 private:
  std::array<std::optional<Event>,Capacity> events_;
  std::size_t begin_{0},size_{0};std::uint64_t dropped_{0};
};
}  // namespace d1max_trajectory_tracker
