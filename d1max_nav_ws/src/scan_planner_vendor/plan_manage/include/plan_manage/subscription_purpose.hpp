#pragma once

namespace scan_planner {

// A planning-only process has geometry evidence and commit inputs, but no
// controller/writer inboxes. Execution uses typed writer/tracker inputs; any
// unversioned compatibility inbox additionally requires absence of the ledger.
inline constexpr bool nativeMotionSubscriptionsEnabled(bool planning_only) {
  return !planning_only;
}

}  // namespace scan_planner
