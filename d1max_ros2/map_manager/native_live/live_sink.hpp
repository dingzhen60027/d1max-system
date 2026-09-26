// Display-only observer of native MOLA outputs. No changes to estimation or SDK.
#pragma once
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mrpt/poses/Lie/SO.h>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace d1max {
class LiveSink {
  using Pose = mola::LocalizationSourceBase::LocalizationUpdate;
  using Update = mola::MapSourceBase::MapUpdate;
  struct Key {
    int x,y,z;
    bool operator==(const Key& b) const { return x==b.x && y==b.y && z==b.z; }
  };
  struct Hash {
    size_t operator()(const Key& k) const {
      return std::hash<int>{}(k.x) ^ (std::hash<int>{}(k.y)*19349663u) ^ (std::hash<int>{}(k.z)*83492791u);
    }
  };
  struct Item { mrpt::maps::CPointsMap::ConstPtr points; Pose pose; };
  std::filesystem::path root_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Item> queue_;
  std::optional<Pose> pose_, last_;
  bool stopping_=false;
  std::thread thread_;
  std::unordered_map<Key,std::array<float,3>,Hash> voxels_;
  std::vector<Pose> path_;
  std::atomic<size_t> dropped_{0};
  double voxel_=0.15, period_=1.;
  size_t max_points_=1000000, updates_=0;
  std::chrono::steady_clock::time_point last_write_{};
  Key key(const std::array<float,3>& p) const {
    return {int(std::floor(p[0]/voxel_)),int(std::floor(p[1]/voxel_)),int(std::floor(p[2]/voxel_))};
  }
  void snapshot() {
    if(path_.empty()) return;
    // A single atomic payload: count, source timestamp, followed by global XYZ.
    const auto tmp=root_/"global.bin.tmp", out=root_/"global.bin";
    std::ofstream s(tmp,std::ios::binary|std::ios::trunc);
    s.exceptions(std::ios::failbit|std::ios::badbit);
    const uint64_t count=voxels_.size();
    double stamp=mrpt::Clock::toDouble(path_.back().timestamp);
    {std::lock_guard<std::mutex> l(mutex_);if(pose_) stamp=mrpt::Clock::toDouble(pose_->timestamp);}
    s.write("D1MG0001",8); s.write(reinterpret_cast<const char*>(&count),8);
    s.write(reinterpret_cast<const char*>(&stamp),8);
    for(const auto& [k,p]:voxels_) s.write(reinterpret_cast<const char*>(p.data()),12);
    s.close(); std::filesystem::rename(tmp,out);
    std::ofstream t(root_/"trajectory.txt.tmp");
    t.exceptions(std::ios::failbit|std::ios::badbit); t<<std::setprecision(16);
    for(const auto& u:path_) t<<mrpt::Clock::toDouble(u.timestamp)<<' '<<u.pose.x<<' '<<u.pose.y<<' '<<u.pose.z<<' '<<u.pose.yaw<<' '<<u.pose.pitch<<' '<<u.pose.roll<<'\n';
    t.close();std::filesystem::rename(root_/"trajectory.txt.tmp",root_/"trajectory.txt");
    std::ofstream j(root_/"status.json.tmp");
    j.exceptions(std::ios::failbit|std::ios::badbit);
    j<<std::setprecision(16)<<"{\"source\":\"native_mola_deskewed_global\",\"frame\":\"d1max_loc_map\",\"stamp\":"<<stamp
     <<",\"points\":"<<count<<",\"voxel_m\":"<<voxel_<<",\"integrated_scans\":"<<updates_
     <<",\"queue_dropped\":"<<dropped_.load()<<",\"final_pcd_loaded\":false}";
    j.close();std::filesystem::rename(root_/"status.json.tmp",root_/"status.json");
    last_write_=std::chrono::steady_clock::now();
  }
  void run() noexcept {
    try {
      for(;;) {
        Item item;
        { std::unique_lock<std::mutex> l(mutex_);cv_.wait_for(l,std::chrono::duration<double>(period_),[&]{return stopping_||!queue_.empty();});
          if(queue_.empty()) {if(stopping_) break;l.unlock();snapshot();continue;}
          item=std::move(queue_.front());queue_.pop_front(); }
        for(size_t i=0;i<item.points->size();++i) {
          std::array<float,3> p;item.points->getPointFast(i,p[0],p[1],p[2]);
          if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||
             std::abs(p[0])>100000||std::abs(p[1])>100000||std::abs(p[2])>100000) continue;
          voxels_.try_emplace(key(p),p); // Already MAP coordinates: never apply pose twice.
        }
        while(voxels_.size()>max_points_) {
          voxel_*=1.25; decltype(voxels_) coarse;
          for(const auto& [k,p]:voxels_) coarse.try_emplace(key(p),p);
          voxels_.swap(coarse); // Preserve whole-map coverage, not a rolling local window.
        }
        path_.push_back(item.pose);++updates_;
        if(std::chrono::duration<double>(std::chrono::steady_clock::now()-last_write_).count()>=period_) snapshot();
      }
      snapshot();
    } catch(const std::exception& e) {
      std::ofstream(root_/"error.txt")<<e.what();
      // Display failure must not alter the estimator or block the sensor callback.
    }
  }
public:
  explicit LiveSink(const std::filesystem::path& root):root_(root) {
    std::filesystem::create_directories(root_);
    if(const char* v=std::getenv("D1MAX_MOLA_LIVE_VOXEL")) voxel_=std::stod(v);
    if(const char* v=std::getenv("D1MAX_MOLA_LIVE_PERIOD")) period_=std::stod(v);
    if(const char* v=std::getenv("D1MAX_MOLA_LIVE_MAX_POINTS")) max_points_=std::stoul(v);
    if(voxel_<.05||voxel_>1||period_<.2||period_>10||max_points_<10000||max_points_>2000000) throw std::runtime_error("Invalid live display limits");
    thread_=std::thread([this]{run();});
  }
  ~LiveSink(){finish();}
  void finish() {
    {std::lock_guard<std::mutex> l(mutex_);stopping_=true;}cv_.notify_one();
    if(thread_.joinable()) thread_.join();
  }
  void pose(const Pose& p) {std::lock_guard<std::mutex> l(mutex_);pose_=p;}
  void cloud(const Update& u) {
    if(u.map_name!="deskewed_scan"||u.reference_frame!="d1max_loc_map") return;
    auto points=std::dynamic_pointer_cast<const mrpt::maps::CPointsMap>(u.map);
    if(!points) return;
    std::lock_guard<std::mutex> l(mutex_);
    if(stopping_||!pose_||pose_->timestamp!=u.timestamp) return;
    if(last_) {
      const auto delta=mrpt::poses::CPose3D(pose_->pose)-mrpt::poses::CPose3D(last_->pose);
      if(delta.translation().norm()<.3 && mrpt::poses::Lie::SO<3>::log(delta.getRotationMatrix()).norm()<mrpt::DEG2RAD(10.)) return;
    }
    if(queue_.size()>=8) {++dropped_;return;}
    queue_.push_back({points,*pose_});last_=pose_;cv_.notify_one();
  }
};
inline std::shared_ptr<LiveSink> attach_live(const mola::LidarOdometry::Ptr& lio) {
  const char* root=std::getenv("D1MAX_MOLA_LIVE_DIR");
  if(!root||!*root) return {};
  auto s=std::make_shared<LiveSink>(root);
  lio->subscribeToLocalizationUpdates([s](const auto& p){s->pose(p);});
  lio->subscribeToMapUpdates([s](const auto& m){s->cloud(m);});
  return s;
}
} // namespace d1max
