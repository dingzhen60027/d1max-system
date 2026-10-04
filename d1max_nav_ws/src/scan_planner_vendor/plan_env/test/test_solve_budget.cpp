#include <gtest/gtest.h>
#include <plan_env/solve_budget.hpp>
#include <thread>

using scan_planner::SolveBudget;
TEST(SolveBudget, SharedAbsoluteDeadlineDoesNotRenewAtEachStage) {
  auto now = SolveBudget::Time{};
  auto budget = std::make_shared<SolveBudget>(std::chrono::milliseconds(400), [&] { return now; });
  const auto search = budget, optimizer = budget, validation = budget;
  now += std::chrono::milliseconds(200);
  EXPECT_TRUE(search->allowed());
  now += std::chrono::milliseconds(199);
  EXPECT_TRUE(optimizer->allowed());
  now += std::chrono::milliseconds(1);
  EXPECT_FALSE(validation->allowed());
  EXPECT_STREQ(validation->reason(), "solve_deadline_exceeded");
}
TEST(SolveBudget, CancelIsVisibleAcrossOwnersAndDoesNotAffectNextSolve) {
  auto budget = std::make_shared<SolveBudget>();
  std::thread cancel([budget] { budget->cancel(); });
  cancel.join();
  EXPECT_FALSE(budget->allowed());
  EXPECT_STREQ(budget->reason(), "solve_cancelled");
  EXPECT_TRUE(SolveBudget().allowed());
}
TEST(SolveBudget, InvalidOrExtendedBudgetCannotBeConfigured) {
  EXPECT_THROW(SolveBudget(std::chrono::milliseconds(0)), std::invalid_argument);
  EXPECT_THROW(SolveBudget(std::chrono::milliseconds(401)), std::invalid_argument);
}
TEST(SolveBudget, SideSelectionEightyMillisecondSubBudgetCannotRenewParentDeadline) {
  auto now=SolveBudget::Time{};
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
  auto side=std::make_shared<SolveBudget>(std::chrono::milliseconds(80),[&]{return now;},parent);
  now+=std::chrono::milliseconds(80);
  EXPECT_FALSE(side->allowed());EXPECT_TRUE(parent->allowed());
  now+=std::chrono::milliseconds(300);
  auto late=std::make_shared<SolveBudget>(std::chrono::milliseconds(80),[&]{return now;},parent);
  EXPECT_EQ(late->deadline(),parent->deadline());
  now+=std::chrono::milliseconds(20);
  EXPECT_FALSE(late->allowed());EXPECT_FALSE(parent->allowed());
}
TEST(SolveBudget, SideSelectionSeesParentCancellationBeforeItsOwnDeadline) {
  auto now=SolveBudget::Time{};
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
  auto side=std::make_shared<SolveBudget>(std::chrono::milliseconds(80),[&]{return now;},parent);
  parent->cancel();EXPECT_FALSE(side->allowed());EXPECT_STREQ(side->reason(),"solve_cancelled");
}
