#pragma once

#include <bspline_opt/spline_heading_contract.hpp>
#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/support_height_coupling.hpp>
#include <plan_env/solve_budget.hpp>
#include <functional>
#include <string>

namespace scan_planner {

// Optional trajectory quality, never a new collision/authorization policy.
// The input has already passed the normal SCAN pipeline. Failure leaves it
// byte-for-byte unchanged. Distances are native feasible lateral translations
// of the complete hard body envelope, not Euclidean point/robot clearance.
struct SoftClearanceOptions {
  double distance{0.};
  int work_budget_ms{30};
  double preferred_turn_radius{1.};
};
struct SoftClearanceReport {
  bool attempted{false},changed{false};
  std::string reason{"disabled"};
  std::size_t queries{0},sections{0},candidates{0};
  std::size_t curvature_rejections{0},clearance_rejections{0},admission_rejections{0};
  double initial_penalty{0.},final_penalty{0.},maximum_shift{0.},elapsed_ms{0.};
  double initial_minimum_side_margin{0.},final_minimum_side_margin{0.};
  double initial_curvature_penalty{0.},final_curvature_penalty{0.};
};
using SoftClearanceQuery=std::function<int(const Eigen::Vector3d&,double)>;
using SoftClearanceAdmission=std::function<bool(UniformBspline&,const SolveBudget::Ptr&)>;
inline constexpr double soft_clearance_publication_reserve_s=.020;
struct SoftClearanceWorkContext {
  // Include the owner's existing adoption budget when it still has to run.
  double completion_reserve_s{soft_clearance_publication_reserve_s};
  // Remaining duration in the SAME source clock as the original solve pose.
  // Never convert an absolute ROS timestamp into a steady deadline. Paused
  // clocks are permitted; invalid/backwards reads stop optional quality only.
  std::function<double()> remaining_source;
};

SoftClearanceReport improveSoftClearance(UniformBspline& accepted,
    double resolution,const SoftClearanceOptions& options,
    const SplineHeadingContract& heading,const SolveBudget::Ptr& parent,
    const SoftClearanceQuery& query,const SoftClearanceAdmission& admission,
    const SoftClearanceWorkContext& context={},
    const SupportHeightCoupling* support_height=nullptr);

}  // namespace scan_planner
