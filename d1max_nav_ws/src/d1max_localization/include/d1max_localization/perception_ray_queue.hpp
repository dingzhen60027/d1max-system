#pragma once

#include <array>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

#include "d1max_localization/perception_rays.hpp"

namespace d1max_localization::perception_rays
{
// At most one pending cloud per source. Overload drops old metadata rather
// than blocking the LIO callback or accumulating an unbounded work queue.
class LatestQueue
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  struct Item {Sensor sensor; Cloud cloud;};
  enum class Submit {Queued, Replaced, Rejected};

  Submit submit(Sensor sensor, Cloud cloud)
  {
    const auto index = static_cast<size_t>(sensor);
    if (index >= slots_.size() || !cloud) {return Submit::Rejected;}
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) {return Submit::Rejected;}
    const auto result = slots_[index] ? Submit::Replaced : Submit::Queued;
    slots_[index] = std::move(cloud);
    condition_.notify_one();
    return result;
  }

  std::optional<Item> wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] {return stopped_ || slots_[0] || slots_[1];});
    if (stopped_) {return std::nullopt;}
    const size_t index = slots_[next_] ? next_ : 1 - next_;
    next_ = 1 - index;
    return Item{static_cast<Sensor>(index), std::exchange(slots_[index], nullptr)};
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    slots_ = {};
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::array<Cloud, 2> slots_{};
  size_t next_{0};
  bool stopped_{false};
};
}  // namespace d1max_localization::perception_rays
