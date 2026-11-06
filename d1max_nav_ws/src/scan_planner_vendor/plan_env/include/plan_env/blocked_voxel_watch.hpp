#pragma once

// Bounded, diagnostic-only history for the few raw voxels which actually
// blocked a native collision query. This is not an occupancy map, a decay
// policy, a collision cache or an authorization input. The caller reports the
// production writer's already-decided updates; this class never decides one.
//
// Reader -> writer requests use a separate latest-value mailbox. The writer
// takes it at most once per fusion. No ray traversal calls lock or allocate.
// Completed snapshots are small value objects with no shared mutable state.
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <type_traits>

namespace scan_planner {

using BlockedVoxelKey = std::array<std::int32_t, 3>;
constexpr std::size_t kBlockedVoxelWatchCells = 4;
constexpr std::size_t kBlockedVoxelWatchHistory = 16;

struct BlockedVoxelWatchRequest {
  std::uint64_t context_generation{0};
  std::uint64_t request_sequence{0};
  std::array<BlockedVoxelKey, kBlockedVoxelWatchCells> cells{};
  std::uint8_t count{0};
};

// These are fusion-batch identities, NOT proof that either physical source
// hit/missed a particular watched voxel. Per-cell votes below are precisely
// the production cache's deduplicated votes, not raw ray overlap counts.
struct BlockedVoxelFusion {
  std::uint64_t sequence{0};
  std::array<std::int64_t, 2> source_stamps_ns{};
  std::array<std::uint64_t, 2> projection_sequences{};
};

struct BlockedVoxelRawState {
  double log_odds{0.};
  bool in_map{false};
  bool occupied{false};
};

enum class BlockedVoxelUpdate : std::uint8_t {
  NotProcessed,
  Applied,
  HitSaturated,
  MissSaturated,
  OutsideLocalUpdate,
  NoObservation,
  SlidOut,
  Reset
};

inline const char* blockedVoxelUpdateName(BlockedVoxelUpdate value) noexcept {
  switch (value) {
    case BlockedVoxelUpdate::Applied: return "applied";
    case BlockedVoxelUpdate::HitSaturated: return "hit_saturated";
    case BlockedVoxelUpdate::MissSaturated: return "miss_saturated";
    case BlockedVoxelUpdate::OutsideLocalUpdate: return "outside_local_update";
    case BlockedVoxelUpdate::NoObservation: return "no_observation";
    case BlockedVoxelUpdate::SlidOut: return "slid_out";
    case BlockedVoxelUpdate::Reset: return "reset";
    default: return "not_processed";
  }
}

enum class BlockedVoxelTransition : std::uint8_t {
  Unchanged,
  BecameOccupied,
  Cleared,
  SlidOut,
  Reset
};

inline const char* blockedVoxelTransitionName(BlockedVoxelTransition value) noexcept {
  switch (value) {
    case BlockedVoxelTransition::BecameOccupied: return "became_occupied";
    case BlockedVoxelTransition::Cleared: return "cleared";
    case BlockedVoxelTransition::SlidOut: return "slid_out";
    case BlockedVoxelTransition::Reset: return "reset";
    default: return "unchanged";
  }
}

struct BlockedVoxelSample {
  BlockedVoxelFusion fusion{};
  BlockedVoxelRawState before{}, after{};
  std::uint32_t hit_votes{0}, miss_votes{0};
  BlockedVoxelUpdate outcome{BlockedVoxelUpdate::NotProcessed};
  BlockedVoxelTransition transition{BlockedVoxelTransition::Unchanged};
};

struct BlockedVoxelWatchCell {
  BlockedVoxelKey key{};
  std::uint64_t context_generation{0}, last_request_sequence{0};
  std::array<BlockedVoxelSample, kBlockedVoxelWatchHistory> samples{};
  std::uint8_t count{0}, next{0};
  bool active{false};

  const BlockedVoxelSample* latest() const noexcept {
    return count == 0 ? nullptr : &samples[(next + kBlockedVoxelWatchHistory - 1) %
                                          kBlockedVoxelWatchHistory];
  }
};

struct BlockedVoxelWatchSnapshot {
  std::uint64_t context_generation{0}, history_revision{0}, change_revision{0};
  std::array<BlockedVoxelWatchCell, kBlockedVoxelWatchCells> cells{};
};

static_assert(std::is_trivially_copyable<BlockedVoxelWatchSnapshot>::value,
              "collision snapshots must own a plain diagnostic value copy");
static_assert(sizeof(BlockedVoxelWatchSnapshot) <= 16384,
              "blocked voxel diagnostic must remain strictly bounded");

// Never put this mailbox inside a copied collision snapshot. Only one writer
// consumes it. Failed tryTakeLatest leaves the request pending for next fusion.
class BlockedVoxelWatchMailbox {
 public:
  bool tryOffer(const BlockedVoxelWatchRequest& request) {
    if (request.context_generation == 0 || request.count == 0 ||
        request.count > kBlockedVoxelWatchCells) return false;
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    if (!guard.owns_lock()) return false;
    pending_ = request;
    pending_.request_sequence = ++sequence_;
    has_pending_ = true;
    return true;
  }

  bool submit(const BlockedVoxelWatchRequest& request) { return tryOffer(request); }

  bool tryTakeLatest(BlockedVoxelWatchRequest& request) {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    if (!guard.owns_lock() || !has_pending_) return false;
    request = pending_;
    has_pending_ = false;
    return true;
  }

  bool tryTake(BlockedVoxelWatchRequest& request) { return tryTakeLatest(request); }

 private:
  std::mutex mutex_;
  BlockedVoxelWatchRequest pending_{};
  std::uint64_t sequence_{0};
  bool has_pending_{false};
};

// Writer owned. All public operations except snapshot() are called only by the
// existing serialized production map writer, never by a query/solver thread.
class BlockedVoxelWatch {
 public:
  // Reset is a distinct terminal diagnostic, never reported as observed free.
  // The old record remains visible until a later request reuses its slot.
  void setContext(std::uint64_t context_generation,
                  const BlockedVoxelFusion& fusion = {}) noexcept {
    if (context_generation == snapshot_.context_generation) return;
    fusion_open_ = false;
    for (auto& cell : snapshot_.cells) {
      if (!cell.active) continue;
      BlockedVoxelSample sample;
      sample.fusion = fusion;
      if (const auto* previous = cell.latest()) sample.before = previous->after;
      sample.after = sample.before;
      sample.outcome = BlockedVoxelUpdate::Reset;
      sample.transition = BlockedVoxelTransition::Reset;
      append(cell, sample);
      cell.active = false;
    }
    snapshot_.context_generation = context_generation;
    last_request_sequence_ = 0;
    ++snapshot_.change_revision;
  }

  // Merge rather than replace: once a failed voxel clears, its history remains
  // observable even though the collision query no longer reports that voxel.
  bool acceptRequest(const BlockedVoxelWatchRequest& request) noexcept {
    if (fusion_open_ || request.context_generation == 0 ||
        request.context_generation != snapshot_.context_generation ||
        request.request_sequence == 0 || request.request_sequence <= last_request_sequence_ ||
        request.count == 0 || request.count > kBlockedVoxelWatchCells) return false;
    last_request_sequence_ = request.request_sequence;
    for (std::size_t i = 0; i < request.count; ++i) {
      auto* cell = find(request.cells[i]);
      if (!cell) {
        cell = replacementSlot();
        *cell = BlockedVoxelWatchCell{};
        cell->key = request.cells[i];
        cell->context_generation = request.context_generation;
        cell->active = true;
        ++snapshot_.change_revision;
      }
      cell->last_request_sequence = request.request_sequence;
    }
    return true;
  }

  // sampler(key) returns the production map's current raw state; no new
  // collision query is necessary. At most four reads at each fusion boundary.
  template <class Sampler>
  bool beginFusion(const BlockedVoxelFusion& fusion, Sampler&& sampler) {
    if (fusion_open_ || fusion.sequence == 0) return false;
    fusion_open_ = true;
    touched_.fill(false);
    for (std::size_t i = 0; i < snapshot_.cells.size(); ++i) {
      if (!snapshot_.cells[i].active) continue;
      working_[i] = BlockedVoxelSample{};
      working_[i].fusion = fusion;
      working_[i].before = sampler(snapshot_.cells[i].key);
      working_[i].after = working_[i].before;
    }
    return true;
  }

  // Called in the existing cache-drain path AFTER production decides its
  // update branch. Duplicate reporting is rejected, not counted twice.
  bool recordUpdate(const BlockedVoxelKey& key, const BlockedVoxelRawState& before,
                    const BlockedVoxelRawState& after, std::uint32_t hit_votes,
                    std::uint32_t miss_votes, BlockedVoxelUpdate outcome) noexcept {
    if (!fusion_open_) return false;
    for (std::size_t i = 0; i < snapshot_.cells.size(); ++i) {
      const auto& cell = snapshot_.cells[i];
      if (!cell.active || cell.key != key || touched_[i]) continue;
      auto& sample = working_[i];
      sample.before = before;
      sample.after = after;
      sample.hit_votes = hit_votes;
      sample.miss_votes = miss_votes;
      sample.outcome = outcome;
      touched_[i] = true;
      return true;
    }
    return false;
  }

  template <class Sampler>
  bool finishFusion(Sampler&& sampler) {
    if (!fusion_open_) return false;
    for (std::size_t i = 0; i < snapshot_.cells.size(); ++i) {
      auto& cell = snapshot_.cells[i];
      if (!cell.active) continue;
      auto& sample = working_[i];
      const auto current = sampler(cell.key);
      sample.after = current;
      if (!current.in_map) {
        sample.outcome = BlockedVoxelUpdate::SlidOut;
        sample.transition = BlockedVoxelTransition::SlidOut;
        cell.active = false;
      } else {
        if (!touched_[i]) sample.outcome = BlockedVoxelUpdate::NoObservation;
        if (sample.before.in_map && sample.before.occupied && !current.occupied)
          sample.transition = BlockedVoxelTransition::Cleared;
        else if (sample.before.in_map && !sample.before.occupied && current.occupied)
          sample.transition = BlockedVoxelTransition::BecameOccupied;
      }
      append(cell, sample);
    }
    fusion_open_ = false;
    return true;
  }

  BlockedVoxelWatchSnapshot snapshot() const noexcept { return snapshot_; }

 private:
  BlockedVoxelWatchCell* find(const BlockedVoxelKey& key) noexcept {
    for (auto& cell : snapshot_.cells)
      if (cell.active && cell.context_generation == snapshot_.context_generation && cell.key == key)
        return &cell;
    return nullptr;
  }

  BlockedVoxelWatchCell* replacementSlot() noexcept {
    for (auto& cell : snapshot_.cells) if (!cell.active) return &cell;
    auto* oldest = &snapshot_.cells[0];
    for (auto& cell : snapshot_.cells)
      if (cell.last_request_sequence < oldest->last_request_sequence) oldest = &cell;
    return oldest;
  }

  static bool sameState(const BlockedVoxelRawState& a, const BlockedVoxelRawState& b) noexcept {
    return a.log_odds == b.log_odds && a.in_map == b.in_map && a.occupied == b.occupied;
  }

  void append(BlockedVoxelWatchCell& cell, const BlockedVoxelSample& sample) noexcept {
    const auto* previous = cell.latest();
    if (!previous || previous->outcome != sample.outcome || previous->transition != sample.transition ||
        previous->hit_votes != sample.hit_votes || previous->miss_votes != sample.miss_votes ||
        !sameState(previous->before, sample.before) || !sameState(previous->after, sample.after))
      ++snapshot_.change_revision;
    cell.samples[cell.next] = sample;
    cell.next = (cell.next + 1) % kBlockedVoxelWatchHistory;
    if (cell.count < kBlockedVoxelWatchHistory) ++cell.count;
    ++snapshot_.history_revision;
  }

  BlockedVoxelWatchSnapshot snapshot_{};
  std::array<BlockedVoxelSample, kBlockedVoxelWatchCells> working_{};
  std::array<bool, kBlockedVoxelWatchCells> touched_{};
  std::uint64_t last_request_sequence_{0};
  bool fusion_open_{false};
};

}  // namespace scan_planner
