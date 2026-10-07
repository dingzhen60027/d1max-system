#pragma once
#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <plan_env/near_field_diagnostics.hpp>

namespace scan_planner {
// Match the explicit KEEP_LAST depth of the mixed-source ROS subscription.
// Draining before integration exposes queued acquisitions to the existing
// independently validated latest-per-source slots. Never loop until empty:
// arrivals may continue while decoding, and mapping must still make progress.
inline constexpr std::size_t kProjectedRayIngressDepth=5;
inline constexpr std::size_t kMaxProjectedAcquisitionPoints=100000;
struct ProjectedRayIngressDrain {
  std::size_t taken{0},accepted{0};
};
template<class Message, class Take, class Accept>
ProjectedRayIngressDrain drainProjectedRayIngress(Take take, Accept accept) {
  ProjectedRayIngressDrain result;
  Message message;
  while(result.taken<kProjectedRayIngressDepth && take(message)) {
    ++result.taken;
    if(accept(message)) ++result.accepted;
  }
  return result;
}

// This flag scopes a diagnostic preview lease; it never authorizes execution.
// Execution entry points must independently reject preview-only configurations.
inline void validateCloudPoseTiming(double wait, double age, bool preview_only,
                                   bool require_observed_free, bool use_projected_rays) {
  if (preview_only && (require_observed_free || !use_projected_rays))
    throw std::invalid_argument("preview-only timing requires official collision policy and projected rays");
  const double maximum_age=preview_only ? .75 : .5;
  if (!std::isfinite(wait) || wait<=0. || wait>.25 ||
      !std::isfinite(age) || age<=0. || age>maximum_age)
    throw std::invalid_argument("cloud/pose timing exceeds 250ms wait / 500ms source age (750ms only in explicit official projected-ray preview)");
}

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
  // Zero is unattributed/static. Only the explicit sealed simulation decoder
  // accepts nonzero sorted-registry ordinals; legacy 64-byte rays stay zero.
  std::vector<std::uint16_t> actor_ids;
  std::int64_t stamp_ns{0};
  std::uint16_t sensor_id{0};
  std::vector<RayDiagnosticMetadata> diagnostics;
};
inline bool rayStampFresh(std::int64_t stamp, std::int64_t now, double maximum_age) {
  if (stamp<=0 || now<=0 || !std::isfinite(maximum_age) || maximum_age<=0.) return false;
  const double age=(static_cast<double>(now)-static_cast<double>(stamp))*1e-9;
  return age>=-.1 && age<=maximum_age;
}
// Fixed schema is intentional: do not accidentally reinterpret XYZ-only clouds
// as independently sourced evidence. All bytes are bounded before reading.
inline std::optional<ProjectedRayBatch> decodeProjectedRays(
    const sensor_msgs::msg::PointCloud2 &cloud, const std::string &frame,bool diagnostics=false,
    std::uint16_t max_actor_id=0) {
  using F=sensor_msgs::msg::PointField;
  const std::uint64_t count=static_cast<std::uint64_t>(cloud.width)*cloud.height;
  const bool actor_schema=cloud.point_step==72&&max_actor_id>0;
  if (cloud.header.frame_id!=frame || cloud.is_bigendian || (cloud.point_step!=64&&!actor_schema) ||
      !count || count>kMaxProjectedAcquisitionPoints || static_cast<std::uint64_t>(cloud.row_step)*cloud.height!=cloud.data.size() ||
      static_cast<std::uint64_t>(cloud.width)*cloud.point_step>cloud.row_step || cloud.data.size()>8U*1024U*1024U ||
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
  unsigned actor_fields=0;
  for(const auto& field:cloud.fields)if(field.name=="isaac_actor_id") {
    ++actor_fields;
    if(!actor_schema||field.offset!=64||field.datatype!=F::UINT16||field.count!=1)return std::nullopt;
  }
  if(actor_fields!=(actor_schema?1U:0U))return std::nullopt;
  ProjectedRayBatch batch;
  // Optional fields are witnesses, not new admission rules. A missing or bad
  // diagnostic schema must not change which clouds the production map accepts.
  bool metadata_ok=diagnostics;
  const Field optional[]={{"ring",30,F::UINT16},{"offset_time",32,F::UINT32},
    {"source_index",36,F::UINT32},{"timestamp",40,F::FLOAT64},
    {"source_timestamp",48,F::FLOAT64},{"raw_timestamp",56,F::FLOAT64}};
  if(diagnostics) for(const auto &wanted:optional) {
    unsigned matches=0;
    for(const auto &field:cloud.fields) if(field.name==wanted.name) {
      ++matches;
      metadata_ok=metadata_ok && field.offset==wanted.offset && field.datatype==wanted.type && field.count==1;
    }
    metadata_ok=metadata_ok && matches==1;
  }
  batch.stamp_ns=static_cast<std::int64_t>(cloud.header.stamp.sec)*1000000000LL+cloud.header.stamp.nanosec;
  batch.endpoints.reserve(count); batch.origins.reserve(count);batch.actor_ids.reserve(count);
  if(diagnostics) batch.diagnostics.reserve(count);
  for (std::uint32_t row=0;row<cloud.height;++row) for (std::uint32_t column=0;column<cloud.width;++column) {
    const auto *bytes=cloud.data.data()+static_cast<std::size_t>(row)*cloud.row_step+column*cloud.point_step;
    float xyz[3],origin[3]; std::uint16_t sensor;
    std::memcpy(xyz,bytes,12);std::memcpy(origin,bytes+16,12);std::memcpy(&sensor,bytes+28,2);
    if (sensor>1 || (!batch.origins.empty() && sensor!=batch.sensor_id)) return std::nullopt;
    batch.sensor_id=sensor;
    const Eigen::Vector3d point(xyz[0],xyz[1],xyz[2]), start(origin[0],origin[1],origin[2]);
    if (!point.allFinite() || !start.allFinite() || point.cwiseAbs().maxCoeff()>1000000. ||
        start.cwiseAbs().maxCoeff()>1000000. || (point-start).squaredNorm()<1e-12) return std::nullopt;
    std::uint16_t actor=0;
    if(actor_schema)std::memcpy(&actor,bytes+64,2);
    if(actor>max_actor_id)return std::nullopt;
    batch.endpoints.push_back(point);batch.origins.push_back(start);batch.actor_ids.push_back(actor);
    if(diagnostics) {
      RayDiagnosticMetadata meta;meta.sensor_id=sensor;meta.scan_stamp_ns=batch.stamp_ns;
      if(metadata_ok) {
        std::memcpy(&meta.ring,bytes+30,2);std::memcpy(&meta.offset_time_ns,bytes+32,4);
        std::memcpy(&meta.source_index,bytes+36,4);std::memcpy(&meta.timestamp,bytes+40,8);
        std::memcpy(&meta.source_timestamp,bytes+48,8);std::memcpy(&meta.raw_timestamp,bytes+56,8);
        meta.source_fields_available=std::isfinite(meta.timestamp) && std::isfinite(meta.source_timestamp) &&
            std::isfinite(meta.raw_timestamp);
      }
      batch.diagnostics.push_back(meta);
    }
  }
  return batch;
}
} // namespace scan_planner
