#pragma once
#include <plan_env/observed_ray.hpp>
#include <algorithm>
#include <array>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace scan_planner {
// Speculative DDA only: no map, clock, ROS object, endpoint vote or occupancy
// update is reachable from the worker. The writer replays visits in ray order.
// Two persistent helpers plus the calling writer are exactly three compute lanes.
// All public methods belong to ONE serialized caller. run() is synchronous and
// non-reentrant; rays() must not be mutated until run() has returned.
class ParallelObservedRayVisits {
 public:
  static constexpr std::size_t kMaxRays=200000;
  static constexpr std::size_t kMaxCells=8000000;
  static constexpr std::size_t kMaxSteps=16000000;
  static constexpr unsigned kLanes=3;
  struct Ray {
    Eigen::Vector3d origin,end;
    unsigned char source_mask{0};
    bool include_end{false};
  };
  ParallelObservedRayVisits() : ParallelObservedRayVisits(
      [](auto work) { return std::thread(std::move(work)); }) {}
  ~ParallelObservedRayVisits() {
    stopWorkers();
  }
  ParallelObservedRayVisits(const ParallelObservedRayVisits&)=delete;
  ParallelObservedRayVisits& operator=(const ParallelObservedRayVisits&)=delete;
  std::vector<Ray>& rays() {return rays_;}

  // Conservative L1+2 bound includes zero-length/end/tied crossings. All
  // lanes' event capacities SUM to this one original global budget, never
  // separate 16M allowances. Unsafe input leaves the caller untouched.
  bool run(const Eigen::Vector3i& minimum,const Eigen::Vector3i& size,
           double resolution,std::size_t budget) {
    complete_ = false;
    steps_ = 0;
    if(rays_.size()<2||rays_.size()>kMaxRays||budget<2||budget>kMaxSteps||
       !std::isfinite(resolution)||resolution<=0.||(size.array()<=0).any()) {
      return false;
    }
    std::uint64_t cells=1;
    for(int axis=0;axis<3;++axis) {
      if(static_cast<std::uint64_t>(size[axis])>kMaxCells/cells) {
        return false;
      }
      cells*=static_cast<std::uint64_t>(size[axis]);
      const auto maximum=static_cast<std::int64_t>(minimum[axis])+size[axis]-1;
      if(maximum>std::numeric_limits<int>::max()) {
        return false;
      }
    }
    prefix_.resize(rays_.size()+1);
    prefix_[0] = 0;
    for(std::size_t i=0;i<rays_.size();++i) {
      const auto& ray=rays_[i];
      if(!ray.origin.allFinite()||!ray.end.allFinite()||
         (ray.source_mask!=1&&ray.source_mask!=2)) {
        return false;
      }
      const Eigen::Vector3d first=(ray.origin/resolution).array().floor();
      const Eigen::Vector3d last=(ray.end/resolution).array().floor();
      std::size_t upper=2;
      for(int axis=0;axis<3;++axis) {
        const double maximum=static_cast<double>(minimum[axis])+size[axis]-1;
        if(first[axis]<minimum[axis]||first[axis]>maximum||
           last[axis]<minimum[axis]||last[axis]>maximum) {
          return false;
        }
        const auto difference=static_cast<std::size_t>(std::abs(last[axis]-first[axis]));
        if(difference>budget||upper>budget-difference) {
          return false;
        }
        upper+=difference;
      }
      if(upper>budget||prefix_[i]>budget-upper) {
        return false;
      }
      prefix_[i+1]=prefix_[i]+upper;
    }
    minimum_ = minimum;
    size_ = size;
    resolution_ = resolution;
    for(int axis=0;axis<3;++axis) {
      minimum_local_[axis]=minimum_[axis]%size_[axis];
      if(minimum_local_[axis]<0) {
        minimum_local_[axis]+=size_[axis];
      }
    }
    boundaries_.front()=0;
    boundaries_.back()=rays_.size();
    for(unsigned lane=1;lane<kLanes;++lane) {
      const auto target=prefix_.back()*lane/kLanes;
      boundaries_[lane]=std::lower_bound(prefix_.begin(),prefix_.end(),target)-prefix_.begin();
    }
    // A single address arena bounds TOTAL retained capacity across alternating
    // unbalanced jobs. Independently growing vectors could each retain a
    // previous large part and silently multiply the global event-memory bound.
    if(event_capacity_<prefix_.back()) {
      events_.reset();
      event_capacity_ = 0;
      events_.reset(new int[prefix_.back()]);
      event_capacity_ = prefix_.back();
    }
    for(unsigned lane=0;lane<kLanes;++lane) {
      auto& part = parts_[lane];
      part.begin = boundaries_[lane];
      part.end = boundaries_[lane+1];
      part.limit = prefix_[part.end]-prefix_[part.begin];
      part.steps = 0;
      part.complete = false;
      part.visited.assign(static_cast<std::size_t>(cells),0);
      part.base = prefix_[part.begin];
      part.count = 0;
      part.offsets.resize(part.end-part.begin+1);
    }
    // All potentially allocating preparation above precedes job publication.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      remaining_workers_ = workers_.size();
      pending_.fill(true);
    }
    wake_.notify_all();
    try {
      runPart(0);
    } catch(...) {
      parts_[0].complete = false;
    }
    try {
      std::unique_lock<std::mutex> lock(mutex_);
      finished_.wait(lock, [this] { return remaining_workers_==0; });
    } catch(...) {
      // A synchronization failure after dispatch cannot safely fall back and
      // reuse scratch while the helper may still be running. Fail closed;
      // unlike an allocation/preflight failure, do NOT escape to the caller.
      std::terminate();
    }
    complete_=true;
    for(const auto& part:parts_) {
      steps_+=part.steps;
      complete_=complete_&&part.complete;
    }
    complete_=complete_&&steps_<=budget;
    return complete_;
  }
  std::size_t steps() const {return steps_;}
  // Only successful, fully joined work may be committed. First source visits
  // within a lane retain DDA order; duplicates across lanes are deliberately
  // retained for the writer's original source-bit visitor to reject.
  template<class Visitor> void replay(std::size_t ray_index,Visitor visitor) const {
    if(!complete_||ray_index>=rays_.size()) {
      throw std::logic_error("uncommitted DDA scratch");
    }
    unsigned lane=0;
    while(lane+1<kLanes&&ray_index>=boundaries_[lane+1]) {
      ++lane;
    }
    const auto& part = parts_[lane];
    const auto local = ray_index-part.begin;
    for(auto index=part.offsets[local];index<part.offsets[local+1];++index) {
      const int address=events_[part.base+index];
      int remaining = address;
      Eigen::Vector3i cell;
      cell.z() = remaining%size_.z();
      remaining /= size_.z();
      cell.y() = remaining%size_.y();
      cell.x() = remaining/size_.y();
      for(int axis=0;axis<3;++axis) {
        int offset = cell[axis]-minimum_local_[axis];
        if(offset<0) {
          offset+=size_[axis];
        }
        cell[axis]=minimum_[axis]+offset;
      }
      visitor(cell,address);
    }
  }

 private:
  friend struct ParallelObservedRayVisitsTestAccess;
  // The private factory seam permits deterministic construction-failure tests;
  // production has only the default two-helper construction policy above.
  template<class StartWorker>
  explicit ParallelObservedRayVisits(StartWorker start_worker) {
    try {
      for(unsigned i=0;i<workers_.size();++i) {
        workers_[i]=start_worker([this,i] { workerLoop(i+1); });
      }
    } catch(...) {
      // In particular, failure to construct helper 2 must retire helper 1
      // before unwinding any joinable std::thread member. No job exists yet.
      stopWorkers();
      throw;
    }
  }
  void stopWorkers() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    wake_.notify_all();
    for(auto& worker:workers_) {
      if(worker.joinable()) {
        worker.join();
      }
    }
  }
  struct Part {
    std::vector<unsigned char> visited;
    std::vector<std::size_t> offsets;
    std::size_t begin{0},end{0},limit{0},steps{0},base{0},count{0};
    bool complete{false};
  };
  void runPart(unsigned lane) {
    auto& part = parts_[lane];
    auto remaining = part.limit;
    bool valid = true;
    // Each lane exclusively owns this storage until run() joins all lanes.
    // Copy invariants into the visitor: its unsigned-char writes may otherwise
    // alias the ray/part objects and force their reload on every voxel visit.
    auto* const visited = part.visited.data();
    const auto visited_size = part.visited.size();
    auto* const events = events_.get()+part.base;
    const auto limit = part.limit;
    auto count = part.count;
    for(auto i=part.begin;i<part.end;++i) {
      part.offsets[i-part.begin] = count;
      const auto& ray = rays_[i];
      const auto source_mask = ray.source_mask;
      const bool complete=visitObservedRayIndexed(ray.origin,ray.end,resolution_,ray.include_end,
          remaining,size_,[&,source_mask,visited,visited_size,events,limit](const Eigen::Vector3i&,int address) {
            if(address<0||static_cast<std::size_t>(address)>=visited_size) {
              valid = false;
              return;
            }
            auto& bits = visited[address];
            if(bits&source_mask) {
              return;
            }
            bits|=source_mask;
            if(count>=limit) {
              valid = false;
              return;
            }
            events[count++]=address;
          });
      part.count = count;
      if(!complete||!valid) {
        part.steps = part.limit-remaining;
        return;
      }
    }
    part.offsets.back() = part.count;
    part.steps = part.limit-remaining;
    part.complete = true;
  }
  void workerLoop(unsigned lane) {
    for(;;) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this,lane] { return stop_||pending_[lane-1]; });
        if(stop_) {
          return;
        }
        pending_[lane-1] = false;
      }
      try {
        runPart(lane);
      } catch(...) {
        parts_[lane].complete = false;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        --remaining_workers_;
      }
      finished_.notify_one();
    }
  }
  std::vector<Ray> rays_;
  std::vector<std::size_t> prefix_;
  std::array<Part,kLanes> parts_;
  std::unique_ptr<int[]> events_;
  std::size_t event_capacity_{0};
  Eigen::Vector3i minimum_,size_,minimum_local_;
  double resolution_{0.};
  std::array<std::size_t,kLanes+1> boundaries_{};
  std::size_t steps_{0};
  bool complete_{false};
  std::mutex mutex_;
  std::condition_variable wake_,finished_;
  bool stop_{false};
  std::array<bool,kLanes-1> pending_{};
  unsigned remaining_workers_{0};
  std::array<std::thread,kLanes-1> workers_;
};
} // namespace scan_planner
