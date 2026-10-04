#pragma once
#include <stdexcept>

namespace d1max_trajectory_tracker {
inline void requireIntegratedControlContract() {
  // V2 curve identity and source-paired body state transport exist. They are
  // evidence, not execution permission. The task owner, anchor revision handoff,
  // clearance revalidation and physical mode/stop acceptance are not integrated.
  throw std::invalid_argument(
      "motion architecture incomplete: control_task_permission_owner_not_integrated; "
      "anchor_revision_handoff_and_clearance_revalidation_not_integrated; "
      "physical_mode_and_stop_acceptance_pending");
}
}  // namespace d1max_trajectory_tracker
