#pragma once

#include <plan_env/voxel_collision.hpp>
#include <algorithm>
#include <cstdint>
#include <limits>

namespace scan_planner {
// Preserve live slot/entry correspondence while skipping billions of clears.
// Old stale tags remain intact so the real wrap path must invalidate them.
// Shared by the native cache and GridMap tests; never included by production.
struct VoxelStatusCacheTestAccess {
  static void forceNextSlotGenerationWrap(VoxelStatusCache& cache) {
    const auto current=cache.generation_;
    for(auto& slot:cache.slots_)if(slot.generation==current)
      slot.generation=std::numeric_limits<std::uint32_t>::max();
    cache.generation_=std::numeric_limits<std::uint32_t>::max();
  }
  static std::uint32_t slotGeneration(const VoxelStatusCache& cache) {
    return cache.generation_;
  }
  static void forceNextSerialSaturation(VoxelStatusCache& cache) {
    cache.clear_serial_=std::numeric_limits<std::uint64_t>::max()-1;
  }
  static std::size_t slotsTagged(const VoxelStatusCache& cache,std::uint32_t generation) {
    return std::count_if(cache.slots_.begin(),cache.slots_.end(),
      [generation](const auto& slot){return slot.generation==generation;});
  }
};
}  // namespace scan_planner
