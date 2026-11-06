#include <gtest/gtest.h>
#include <plan_env/blocked_voxel_watch.hpp>
#include <cstring>
#include <atomic>
#include <thread>

namespace {
using namespace scan_planner;
constexpr BlockedVoxelKey kCell{{-666, 646, -13}};
BlockedVoxelWatchRequest request(std::uint64_t sequence = 1, BlockedVoxelKey cell = kCell) {
  BlockedVoxelWatchRequest value;
  value.context_generation = 7;
  value.request_sequence = sequence;
  value.count = 1;
  value.cells[0] = cell;
  return value;
}
BlockedVoxelFusion fusion(std::uint64_t sequence = 1) {
  return {sequence, {{1000, 1010}}, {{11, 12}}};
}
BlockedVoxelRawState occupied(double odds = 1.4) { return {odds, true, true}; }
BlockedVoxelRawState freeState(double odds = -.1) { return {odds, true, false}; }
const BlockedVoxelWatchCell* find(const BlockedVoxelWatchSnapshot& snapshot, BlockedVoxelKey key) {
  for (const auto& cell : snapshot.cells)
    if (cell.context_generation == 7 && cell.key == key) return &cell;
  return nullptr;
}
void setup(BlockedVoxelWatch& watch) {
  watch.setContext(7);
  ASSERT_TRUE(watch.acceptRequest(request()));
}
}  // namespace

TEST(BlockedVoxelWatch, SnapshotIsValueOnlyAndStrictlyBounded) {
  EXPECT_TRUE(std::is_trivially_copyable<BlockedVoxelWatchSnapshot>::value);
  EXPECT_LE(sizeof(BlockedVoxelWatchSnapshot), 16384U);
  BlockedVoxelWatch watch;
  const auto before = watch.snapshot();
  EXPECT_EQ(before.history_revision, 0U);
  EXPECT_EQ(before.context_generation, 0U);
}

TEST(BlockedVoxelWatch, MailboxOnlyKeepsLatestRequestAndDoesNotTrustProducerSequence) {
  BlockedVoxelWatchMailbox mailbox;
  auto a = request(9000), b = request(1, {{4, 5, 6}});
  ASSERT_TRUE(mailbox.submit(a));
  ASSERT_TRUE(mailbox.submit(b));
  BlockedVoxelWatchRequest taken;
  ASSERT_TRUE(mailbox.tryTakeLatest(taken));
  EXPECT_EQ(taken.request_sequence, 2U);
  EXPECT_EQ(taken.cells[0], b.cells[0]);
  EXPECT_FALSE(mailbox.tryTakeLatest(taken));
  b.count = 5;
  EXPECT_FALSE(mailbox.submit(b));
  b.count = 0;
  EXPECT_FALSE(mailbox.submit(b));
  b.count = 1; b.context_generation = 0;
  EXPECT_FALSE(mailbox.submit(b));
}

TEST(BlockedVoxelWatch, ContextAndRequestOrderCannotResurrectOldIdentity) {
  BlockedVoxelWatch watch;
  setup(watch);
  EXPECT_FALSE(watch.acceptRequest(request()));
  auto wrong = request(2); wrong.context_generation = 6;
  EXPECT_FALSE(watch.acceptRequest(wrong));
  EXPECT_TRUE(watch.acceptRequest(request(2)));
  EXPECT_FALSE(watch.acceptRequest(request(1)));
  auto zero = request(0);
  EXPECT_FALSE(watch.acceptRequest(zero));
}

TEST(BlockedVoxelWatch, ConcurrentMailboxNeverProducesTornCellsOrUnboundedQueue) {
  BlockedVoxelWatchMailbox mailbox;
  std::atomic<bool> complete{false};
  std::thread producer([&] {
    for (int i = 1; i <= 10000; ++i) {
      auto value = request(i, {{i, i + 1, i + 2}});
      value.count = 4;
      value.cells.fill(value.cells[0]);
      mailbox.tryOffer(value);
    }
    complete.store(true);
  });
  std::uint64_t last_sequence = 0;
  do {
    BlockedVoxelWatchRequest value;
    if (!mailbox.tryTake(value)) continue;
    EXPECT_GT(value.request_sequence, last_sequence);
    last_sequence = value.request_sequence;
    EXPECT_EQ(value.count, 4U);
    for (const auto& cell : value.cells) {
      EXPECT_EQ(cell, value.cells[0]);
      EXPECT_EQ(cell[1], cell[0] + 1);
      EXPECT_EQ(cell[2], cell[0] + 2);
    }
  } while (!complete.load());
  producer.join();
}

TEST(BlockedVoxelWatch, NoObservationDoesNotPretendToClearOrRefreshAnObservation) {
  BlockedVoxelWatch watch;
  setup(watch);
  auto sample = [](const auto&) { return occupied(); };
  ASSERT_TRUE(watch.beginFusion(fusion(), sample));
  ASSERT_TRUE(watch.finishFusion(sample));
  auto snapshot = watch.snapshot();
  const auto* record = find(snapshot, kCell);
  ASSERT_NE(record, nullptr);
  ASSERT_NE(record->latest(), nullptr);
  EXPECT_EQ(record->latest()->outcome, BlockedVoxelUpdate::NoObservation);
  EXPECT_TRUE(record->latest()->after.occupied);
  EXPECT_EQ(record->latest()->hit_votes, 0U);
  EXPECT_EQ(record->latest()->miss_votes, 0U);
  EXPECT_EQ(record->latest()->fusion.source_stamps_ns[0], 1000);
  EXPECT_EQ(record->latest()->fusion.projection_sequences[1], 12U);
  const auto revision = snapshot.change_revision;
  ASSERT_TRUE(watch.beginFusion(fusion(2), sample));
  ASSERT_TRUE(watch.finishFusion(sample));
  snapshot = watch.snapshot();
  EXPECT_EQ(snapshot.change_revision, revision);
  EXPECT_GT(snapshot.history_revision, 1U);
}

TEST(BlockedVoxelWatch, ActualMissClearingRemainsRecordedWithoutAnotherFailureQuery) {
  BlockedVoxelWatch watch;
  setup(watch);
  ASSERT_TRUE(watch.beginFusion(fusion(), [](const auto&) { return occupied(); }));
  ASSERT_TRUE(watch.recordUpdate(kCell, occupied(), freeState(), 0, 1, BlockedVoxelUpdate::Applied));
  ASSERT_TRUE(watch.finishFusion([](const auto&) { return freeState(); }));
  const auto cleared = watch.snapshot();
  const auto* record = find(cleared, kCell);
  ASSERT_NE(record, nullptr);
  EXPECT_TRUE(record->active);
  EXPECT_EQ(record->latest()->transition, BlockedVoxelTransition::Cleared);
  EXPECT_EQ(record->latest()->miss_votes, 1U);
  EXPECT_DOUBLE_EQ(record->latest()->before.log_odds, 1.4);
  EXPECT_DOUBLE_EQ(record->latest()->after.log_odds, -.1);
  ASSERT_TRUE(watch.beginFusion(fusion(2), [](const auto&) { return freeState(); }));
  ASSERT_TRUE(watch.finishFusion([](const auto&) { return freeState(); }));
  const auto snapshot = watch.snapshot();
  EXPECT_EQ(find(snapshot, kCell)->count, 2);
  EXPECT_EQ(find(snapshot, kCell)->samples[0].transition, BlockedVoxelTransition::Cleared);
  // The copied collision snapshot does not mutate as the writer advances.
  EXPECT_EQ(find(cleared, kCell)->count, 1);
}

TEST(BlockedVoxelWatch, AllProductionDrainOutcomesRemainDistinctAndVotesAreNotRecounted) {
  for (auto outcome : {BlockedVoxelUpdate::Applied, BlockedVoxelUpdate::HitSaturated,
                       BlockedVoxelUpdate::MissSaturated, BlockedVoxelUpdate::OutsideLocalUpdate,
                       BlockedVoxelUpdate::NotProcessed}) {
    BlockedVoxelWatch watch;
    setup(watch);
    ASSERT_TRUE(watch.beginFusion(fusion(), [](const auto&) { return occupied(); }));
    ASSERT_TRUE(watch.recordUpdate(kCell, occupied(), occupied(), 1, 1, outcome));
    EXPECT_FALSE(watch.recordUpdate(kCell, occupied(), freeState(), 0, 10, BlockedVoxelUpdate::Applied));
    ASSERT_TRUE(watch.finishFusion([](const auto&) { return occupied(); }));
    const auto snapshot = watch.snapshot();
    const auto* sample = find(snapshot, kCell)->latest();
    EXPECT_EQ(sample->outcome, outcome);
    EXPECT_EQ(sample->hit_votes, 1U);
    EXPECT_EQ(sample->miss_votes, 1U);
    EXPECT_EQ(sample->transition, BlockedVoxelTransition::Unchanged);
  }
}

TEST(BlockedVoxelWatch, SlidOutIsTerminalAndNeverConfusedWithObservedFree) {
  BlockedVoxelWatch watch;
  setup(watch);
  ASSERT_TRUE(watch.beginFusion(fusion(), [](const auto&) { return occupied(); }));
  ASSERT_TRUE(watch.finishFusion([](const auto&) { return BlockedVoxelRawState{}; }));
  auto snapshot = watch.snapshot();
  const auto* record = find(snapshot, kCell);
  EXPECT_FALSE(record->active);
  EXPECT_EQ(record->latest()->outcome, BlockedVoxelUpdate::SlidOut);
  EXPECT_EQ(record->latest()->transition, BlockedVoxelTransition::SlidOut);
  EXPECT_FALSE(record->latest()->after.in_map);
  auto revision = snapshot.history_revision;
  ASSERT_TRUE(watch.beginFusion(fusion(2), [](const auto&) { return occupied(); }));
  ASSERT_TRUE(watch.finishFusion([](const auto&) { return occupied(); }));
  EXPECT_EQ(watch.snapshot().history_revision, revision);
}

TEST(BlockedVoxelWatch, ResetRetainsTerminalReasonWithoutFollowingOldContextCell) {
  BlockedVoxelWatch watch;
  setup(watch);
  ASSERT_TRUE(watch.beginFusion(fusion(), [](const auto&) { return occupied(); }));
  ASSERT_TRUE(watch.finishFusion([](const auto&) { return occupied(); }));
  watch.setContext(8, fusion(2));
  const auto snapshot = watch.snapshot();
  const auto* record = find(snapshot, kCell);
  ASSERT_NE(record, nullptr);
  EXPECT_EQ(snapshot.context_generation, 8U);
  EXPECT_FALSE(record->active);
  EXPECT_EQ(record->latest()->outcome, BlockedVoxelUpdate::Reset);
  EXPECT_EQ(record->latest()->transition, BlockedVoxelTransition::Reset);
  EXPECT_TRUE(record->latest()->after.occupied);  // no invented free observation
  EXPECT_FALSE(watch.acceptRequest(request(3)));
}

TEST(BlockedVoxelWatch, DuplicateRequestedCellsAndOverflowCannotGrowMemory) {
  BlockedVoxelWatch watch;
  watch.setContext(7);
  auto initial = request(); initial.count = 4;
  initial.cells.fill(kCell);
  ASSERT_TRUE(watch.acceptRequest(initial));
  auto snapshot = watch.snapshot();
  unsigned active = 0;
  for (const auto& cell : snapshot.cells) active += cell.active;
  EXPECT_EQ(active, 1U);
  for (int i = 1; i <= 20; ++i) EXPECT_TRUE(watch.acceptRequest(request(i + 1, {{i, i, i}})));
  snapshot = watch.snapshot(); active = 0;
  for (const auto& cell : snapshot.cells) active += cell.active;
  EXPECT_EQ(active, kBlockedVoxelWatchCells);
  EXPECT_EQ(find(snapshot, kCell), nullptr);
}

TEST(BlockedVoxelWatch, HistoryWrapsAtSixteenCompletedSamples) {
  BlockedVoxelWatch watch;
  setup(watch);
  for (unsigned sequence = 1; sequence <= 40; ++sequence) {
    ASSERT_TRUE(watch.beginFusion(fusion(sequence), [](const auto&) { return occupied(); }));
    ASSERT_TRUE(watch.finishFusion([](const auto&) { return occupied(); }));
  }
  const auto snapshot = watch.snapshot();
  const auto* record = find(snapshot, kCell);
  EXPECT_EQ(record->count, kBlockedVoxelWatchHistory);
  EXPECT_EQ(record->latest()->fusion.sequence, 40U);
  EXPECT_EQ(record->samples[record->next].fusion.sequence, 25U);
}

TEST(BlockedVoxelWatch, ReentrantOrPartialFusionCannotReplaceAnOpenSample) {
  BlockedVoxelWatch watch;
  setup(watch);
  EXPECT_FALSE(watch.finishFusion([](const auto&) { return occupied(); }));
  EXPECT_FALSE(watch.recordUpdate(kCell, occupied(), freeState(), 0, 1, BlockedVoxelUpdate::Applied));
  EXPECT_FALSE(watch.beginFusion(fusion(0), [](const auto&) { return occupied(); }));
  ASSERT_TRUE(watch.beginFusion(fusion(), [](const auto&) { return occupied(); }));
  EXPECT_FALSE(watch.beginFusion(fusion(2), [](const auto&) { return freeState(); }));
  EXPECT_FALSE(watch.acceptRequest(request(2, {{9, 9, 9}})));
  ASSERT_TRUE(watch.finishFusion([](const auto&) { return occupied(); }));
  EXPECT_EQ(find(watch.snapshot(), kCell)->latest()->fusion.sequence, 1U);
}
