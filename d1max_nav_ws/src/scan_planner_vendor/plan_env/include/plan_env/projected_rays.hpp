#pragma once
#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace scan_planner {
// An integration result is a lease update, not merely a slow diagnostics
// heartbeat. Publish a changed completed source stamp/validity/context on the
// same occupancy timer callback; unchanged state remains bounded to 5 Hz.
// Repeating a heartbeat never invents a newer sensor/source timestamp.
class ProjectedRayStatusSchedule {
public:
  using Clock=std::chrono::steady_clock;
  bool due(Clock::time_point now,std::int64_t source_stamp,bool valid,std::uint64_t context) const {
    return !published_ || source_stamp!=source_stamp_ || valid!=valid_ || context!=context_ ||
        std::chrono::duration<double>(now-last_).count()>=.2;
  }
  void markPublished(Clock::time_point now,std::int64_t source_stamp,bool valid,std::uint64_t context) {
    published_=true;last_=now;source_stamp_=source_stamp;valid_=valid;context_=context;
  }
private:
  bool published_{false},valid_{false};
  Clock::time_point last_{};
  std::int64_t source_stamp_{0};
  std::uint64_t context_{0};
};
// An independent acquisition, never a fused XYZ cloud with an assumed origin.
struct ProjectedRayBatch {
  std::vector<Eigen::Vector3d> endpoints, origins;
  std::int64_t stamp_ns{0};
  std::uint16_t sensor_id{0};
};
inline bool rayStampFresh(std::int64_t stamp, std::int64_t now, double maximum_age) {
  if (stamp<=0 || now<=0 || !std::isfinite(maximum_age) || maximum_age<=0.) return false;
  const double age=(static_cast<double>(now)-static_cast<double>(stamp))*1e-9;
  return age>=-.1 && age<=maximum_age;
}
// Fixed schema is intentional: do not accidentally reinterpret XYZ-only clouds
// as independently sourced evidence. All bytes are bounded before reading.
inline std::optional<ProjectedRayBatch> decodeProjectedRays(
    const sensor_msgs::msg::PointCloud2 &cloud, const std::string &frame) {
  using F=sensor_msgs::msg::PointField;
  const std::uint64_t count=static_cast<std::uint64_t>(cloud.width)*cloud.height;
  if (cloud.header.frame_id!=frame || cloud.is_bigendian || cloud.point_step!=64 ||
      !count || count>100000 || static_cast<std::uint64_t>(cloud.row_step)*cloud.height!=cloud.data.size() ||
      static_cast<std::uint64_t>(cloud.width)*64>cloud.row_step || cloud.data.size()>8U*1024U*1024U ||
      cloud.header.stamp.sec<0 || cloud.header.stamp.nanosec>=1000000000U) return std::nullopt;
  struct Field {const char *name; std::uint32_t offset; std::uint8_t type;};
  const Field required[]={{"x",0,F::FLOAT32},{"y",4,F::FLOAT32},{"z",8,F::FLOAT32},
    {"origin_x",16,F::FLOAT32},{"origin_y",20,F::FLOAT32},{"origin_z",24,F::FLOAT32},
    {"sensor_id",28,F::UINT16}};
  for (const auto &wanted:required) {
    unsigned matches=0;
    for (const auto &field:cloud.fields) if (field.name==wanted.name) {
      if (field.offset!=wanted.offset || field.datatype!=wanted.type || field.count!=1) return std::nullopt;
      ++matches;
    }
    if (matches!=1) return std::nullopt;
  }
  ProjectedRayBatch batch;
  batch.stamp_ns=static_cast<std::int64_t>(cloud.header.stamp.sec)*1000000000LL+cloud.header.stamp.nanosec;
  batch.endpoints.reserve(count); batch.origins.reserve(count);
  for (std::uint32_t row=0;row<cloud.height;++row) for (std::uint32_t column=0;column<cloud.width;++column) {
    const auto *bytes=cloud.data.data()+static_cast<std::size_t>(row)*cloud.row_step+column*64;
    float xyz[3],origin[3]; std::uint16_t sensor;
    std::memcpy(xyz,bytes,12);std::memcpy(origin,bytes+16,12);std::memcpy(&sensor,bytes+28,2);
    if (sensor>1 || (!batch.origins.empty() && sensor!=batch.sensor_id)) return std::nullopt;
    batch.sensor_id=sensor;
    const Eigen::Vector3d point(xyz[0],xyz[1],xyz[2]), start(origin[0],origin[1],origin[2]);
    if (!point.allFinite() || !start.allFinite() || point.cwiseAbs().maxCoeff()>1000000. ||
        start.cwiseAbs().maxCoeff()>1000000. || (point-start).squaredNorm()<1e-12) return std::nullopt;
    batch.endpoints.push_back(point);batch.origins.push_back(start);
  }
  return batch;
}
} // namespace scan_planner
