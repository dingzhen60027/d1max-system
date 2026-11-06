#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace d1max_trajectory_tracker {

// A description of the existing SDK V4 static response, not another dynamics
// model or a physical velocity calibration. reference == 0 disables it.
struct CommandResponseCurve {
  double reference=0.;
  double effective_fraction=0.;
  double maximum_fraction=0.;
  double blend_ratio=.10;

  bool enabled() const { return reference>0.; }
  void validate() const {
    for (double value : {reference,effective_fraction,maximum_fraction,blend_ratio})
      if (!std::isfinite(value))
        throw std::invalid_argument("command_response_curve_nonfinite");
    if (reference<0. || (enabled() &&
        (effective_fraction<=0. || effective_fraction>maximum_fraction ||
         maximum_fraction>1. || blend_ratio<=0. || blend_ratio>=1.)))
      throw std::invalid_argument("command_response_curve_bounds_invalid");
  }
  double map(double demand) const {
    if (!std::isfinite(demand))
      throw std::invalid_argument("command_response_demand_nonfinite");
    if (!enabled()) return demand;
    const double ratio=std::clamp(std::abs(demand)/reference,0.,1.);
    const auto smooth=[](double x) { return x*x*x*(10.+x*(-15.+6.*x)); };
    const double magnitude=ratio<=blend_ratio ?
      effective_fraction*smooth(ratio/blend_ratio) :
      effective_fraction+(maximum_fraction-effective_fraction)*
        smooth((ratio-blend_ratio)/(1.-blend_ratio));
    return demand==0. ? 0. : std::copysign(magnitude,demand);
  }
  double inverse(double fraction) const {
    if (!std::isfinite(fraction))
      throw std::invalid_argument("command_response_fraction_nonfinite");
    if (!enabled()) return fraction;
    if (fraction==0.) return 0.;
    const double magnitude=std::min(std::abs(fraction),maximum_fraction);
    // Explicit endpoints avoid a numerical tail after an actual stop.
    if (magnitude>=maximum_fraction)
      return std::copysign(effective_fraction==maximum_fraction ? reference*blend_ratio : reference,fraction);
    if (magnitude==effective_fraction)
      return std::copysign(reference*blend_ratio,fraction);
    double lower=0.,upper=reference;
    for (int i=0;i<48;++i) {
      const double middle=lower+(upper-lower)*.5;
      if (map(middle)<magnitude) lower=middle;
      else upper=middle;
    }
    return std::copysign(lower+(upper-lower)*.5,fraction);
  }
};

struct ResponseCommandStep {
  double value=0.;
  double mapped_fraction=0.;
  double mapped_rate=0.;
  bool limited=false;
  bool priority_override=false;
};

// This is a constraint of the controller's ONE SI command conditioner. It
// selects an SI output whose static SDK image has bounded discrete first and
// second differences; the SDK writer still maps and sends that same command.
// It does not delay/replay a command, own a task, or change its source time.
class ResponseCommandLimiter {
 public:
  void reset() { previous_rate_=0.; }
  ResponseCommandStep step(double target_si,double previous_si,double lower_si,double upper_si,
      const CommandResponseCurve& curve,double max_fraction_rate,double max_fraction_acceleration,
      double dt) {
    curve.validate();
    for (double value : {target_si,previous_si,lower_si,upper_si,dt})
      if (!std::isfinite(value))
        throw std::invalid_argument("response_command_step_nonfinite");
    if (dt<=0. || lower_si>upper_si)
      throw std::invalid_argument("response_command_step_bounds_invalid");
    ResponseCommandStep result;
    result.value=std::clamp(target_si,lower_si,upper_si);
    if (!curve.enabled()) {
      previous_rate_=0.;
      result.limited=result.value!=target_si;
      return result;
    }
    if (!std::isfinite(max_fraction_rate) || !std::isfinite(max_fraction_acceleration) ||
        max_fraction_rate<=0. || max_fraction_acceleration<=0.)
      throw std::invalid_argument("response_command_derivative_bounds_invalid");

    const double previous=curve.map(previous_si);
    const double rate_lower=std::clamp(previous_rate_-max_fraction_acceleration*dt,
      -max_fraction_rate,max_fraction_rate);
    const double rate_upper=std::clamp(previous_rate_+max_fraction_acceleration*dt,
      -max_fraction_rate,max_fraction_rate);
    const double response_lower=previous+rate_lower*dt;
    const double response_upper=previous+rate_upper*dt;
    const double allowed_lower=std::max(curve.map(lower_si),response_lower);
    const double allowed_upper=std::min(curve.map(upper_si),response_upper);
    if (allowed_lower<=allowed_upper) {
      const double error=curve.map(result.value)-previous;
      const double distance=std::abs(error);
      // Reserve this tick's travel AND the following rate-braking distance.
      // The algebraically equivalent rational form avoids cancellation near
      // zero. This is a command-space arrival profile, not a robot inertia
      // estimate. A suddenly moved target may still be overshot while the
      // carried rate is being shed; a stationary reachable target is not.
      const double adt=max_fraction_acceleration*dt;
      const double safe_rate=distance==0. ? 0. :
        2.*max_fraction_acceleration*distance/
          (std::sqrt(adt*adt+2.*max_fraction_acceleration*distance)+adt);
      const double requested_rate=std::copysign(std::min(max_fraction_rate,safe_rate),error);
      const double rate=std::clamp(requested_rate,rate_lower,rate_upper);
      const double fraction=std::clamp(previous+rate*dt,allowed_lower,allowed_upper);
      // A plateau has many valid SI preimages. Preserve the already selected
      // SI target when it has this exact SDK image; choosing the smallest
      // inverse instead would manufacture an unrelated demand/limit jump.
      if (curve.map(result.value)!=fraction)
        result.value=std::clamp(curve.inverse(fraction),lower_si,upper_si);
    } else {
      // Existing trajectory/SI acceleration bounds have priority. Report the
      // conflict instead of freezing navigation or silently claiming the
      // command-domain derivative bound still held.
      result.priority_override=true;
    }
    result.mapped_fraction=curve.map(result.value);
    result.mapped_rate=(result.mapped_fraction-previous)/dt;
    previous_rate_=result.mapped_rate;
    result.limited=result.value!=target_si;
    return result;
  }
 private:
  double previous_rate_=0.;
};

}  // namespace d1max_trajectory_tracker
