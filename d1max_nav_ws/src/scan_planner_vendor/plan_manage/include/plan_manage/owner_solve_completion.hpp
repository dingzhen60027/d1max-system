#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <plan_env/solve_budget.hpp>

namespace scan_planner {

// Owner-only, fixed-capacity completion storage. Finishing computation is not
// geometry adoption: the original budget and generation travel with the value.
template<class Result> struct CompletedOwnerSolve {
  Result result;
  SolveBudget::Ptr budget;
  std::uint64_t generation{0};
};

struct RetiredOwnerSolveFailure {
  std::uint64_t generation{0};
  std::string reason;
};

enum class OwnerSolveCompletionDisposition { Discard, Failed, AwaitMap };

inline OwnerSolveCompletionDisposition ownerSolveCompletionDisposition(
    std::uint64_t generation, std::uint64_t current_generation, bool have_target,
    const SolveBudget::Ptr& budget, bool has_candidate) {
  if (generation != current_generation || !have_target || !budget ||
      budget->stop() == SolveBudget::Stop::Cancelled)
    return OwnerSolveCompletionDisposition::Discard;
  if (!budget->allowed() || !has_candidate)
    return OwnerSolveCompletionDisposition::Failed;
  return OwnerSolveCompletionDisposition::AwaitMap;
}

}  // namespace scan_planner
