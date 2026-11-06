#include <gtest/gtest.h>
#include <plan_env/projected_rays.hpp>
#include <limits>

namespace {
using F=sensor_msgs::msg::PointField;
sensor_msgs::msg::PointCloud2 wire(bool extended,std::size_t count=2) {
  sensor_msgs::msg::PointCloud2 c;c.header.frame_id="odom";c.header.stamp.sec=100;
  c.width=count;c.height=1;c.point_step=extended?68:64;c.row_step=c.width*c.point_step;
  c.data.resize(c.row_step);c.is_dense=true;
  const auto field=[&](const char* name,unsigned offset,unsigned type) {
    F f;f.name=name;f.offset=offset;f.datatype=type;f.count=1;c.fields.push_back(f);
  };
  field("x",0,F::FLOAT32);field("y",4,F::FLOAT32);field("z",8,F::FLOAT32);
  field("intensity",12,F::FLOAT32);field("origin_x",16,F::FLOAT32);
  field("origin_y",20,F::FLOAT32);field("origin_z",24,F::FLOAT32);
  field("sensor_id",28,F::UINT16);field("ring",30,F::UINT16);
  field("offset_time",32,F::UINT32);field("source_index",36,F::UINT32);
  field("timestamp",40,F::FLOAT64);field("source_timestamp",48,F::FLOAT64);
  field("raw_timestamp",56,F::FLOAT64);if(extended)field("ray_kind",64,F::UINT8);
  for(std::size_t i=0;i<count;++i) {
    auto* b=c.data.data()+i*c.point_step;
    const float end[]={1.f+static_cast<float>(i),.2f,.3f},origin[]={.1f,.2f,.3f};
    const std::uint16_t sensor=1,ring=31;const std::uint32_t offset=10000*i,index=7*i;
    const double time=100.+i*.00001,source=80.+i*.00001,raw=source*1e6;
    std::memcpy(b,end,12);std::memcpy(b+16,origin,12);std::memcpy(b+28,&sensor,2);
    std::memcpy(b+30,&ring,2);std::memcpy(b+32,&offset,4);std::memcpy(b+36,&index,4);
    std::memcpy(b+40,&time,8);std::memcpy(b+48,&source,8);std::memcpy(b+56,&raw,8);
    if(extended)b[64]=i%2;
  }
  return c;
}
}

TEST(ProjectedRaysWire, LegacyAndCensoredKeepOriginalGeometryAndSourceMetadata) {
  auto legacy=scan_planner::decodeProjectedRays(wire(false),"odom",true);
  auto extended=scan_planner::decodeProjectedRays(wire(true),"odom",true);
  ASSERT_TRUE(legacy);ASSERT_TRUE(extended);
  EXPECT_EQ(legacy->ray_kinds,(std::vector<std::uint8_t>{0,0}));
  EXPECT_EQ(extended->ray_kinds,(std::vector<std::uint8_t>{0,1}));
  EXPECT_EQ(extended->stamp_ns,100000000000LL);EXPECT_EQ(extended->sensor_id,1);
  for(unsigned i=0;i<2;++i) {
    EXPECT_EQ(legacy->endpoints[i],extended->endpoints[i]);
    EXPECT_EQ(legacy->origins[i],extended->origins[i]);
    const auto& a=legacy->diagnostics[i];const auto& b=extended->diagnostics[i];
    EXPECT_EQ(a.source_index,b.source_index);EXPECT_EQ(a.offset_time_ns,b.offset_time_ns);
    EXPECT_EQ(a.timestamp,b.timestamp);EXPECT_EQ(a.source_timestamp,b.source_timestamp);
    EXPECT_EQ(a.raw_timestamp,b.raw_timestamp);EXPECT_TRUE(b.source_fields_available);
  }
}

TEST(ProjectedRaysWire, MalformedExtensionNeverBecomesFreeOrLegacyHit) {
  for(unsigned fault=0;fault<14;++fault) {
    auto c=wire(true);
    switch(fault) {
      case 0:c.data[64]=2;break;
      case 1:c.data[64]=255;break;
      case 2:c.data[65]=1;break;
      case 3:c.data[66]=1;break;
      case 4:c.data[67]=1;break;
      case 5:c.fields.pop_back();break;
      case 6:c.fields.push_back(c.fields.back());break;
      case 7:c.fields.back().datatype=F::INT8;break;
      case 8:c.fields.back().offset=65;break;
      case 9:c.fields.back().count=2;break;
      case 10:c.fields[11].name="not_timestamp";break;
      case 11:c.fields[11].offset=64;break;
      case 12:c.point_step=66;break;
      case 13:c.data.pop_back();break;
    }
    EXPECT_FALSE(scan_planner::decodeProjectedRays(c,"odom"))<<fault;
  }
  auto legacy=wire(false);auto f=wire(true).fields.back();legacy.fields.push_back(f);
  EXPECT_FALSE(scan_planner::decodeProjectedRays(legacy,"odom"));
}

TEST(ProjectedRaysWire, CensoringDoesNotSanitizeInvalidOrZeroLengthReturn) {
  auto c=wire(true);c.data[64]=1;
  float value=std::numeric_limits<float>::quiet_NaN();std::memcpy(c.data.data(),&value,4);
  EXPECT_FALSE(scan_planner::decodeProjectedRays(c,"odom"));
  c=wire(true);c.data[64]=1;std::memcpy(c.data.data(),c.data.data()+16,12);
  EXPECT_FALSE(scan_planner::decodeProjectedRays(c,"odom"));
}

TEST(ProjectedRaysWire, ReusedBatchResetsKindsAndKeepsBoundedAcquisitionLimit) {
  scan_planner::ProjectedRayBatch batch;
  ASSERT_TRUE(scan_planner::decodeProjectedRaysInto(wire(true),"odom",true,batch));
  ASSERT_TRUE(scan_planner::decodeProjectedRaysInto(wire(false,1),"odom",false,batch));
  EXPECT_EQ(batch.ray_kinds,(std::vector<std::uint8_t>{0}));EXPECT_TRUE(batch.diagnostics.empty());
  auto maximum=wire(true,scan_planner::kMaxProjectedAcquisitionPoints);
  EXPECT_TRUE(scan_planner::decodeProjectedRays(maximum,"odom"));
  EXPECT_LT(maximum.data.size(),8U*1024U*1024U);
  auto excess=wire(true,scan_planner::kMaxProjectedAcquisitionPoints+1);
  EXPECT_FALSE(scan_planner::decodeProjectedRays(excess,"odom"));
}

TEST(ProjectedRaysWire, ExtendedRowsUseDeclaredStrideWithoutReadingPadding) {
  const auto flat=wire(true,2);auto rows=flat;
  rows.width=1;rows.height=2;rows.row_step=72;rows.data.assign(144,0);
  std::copy_n(flat.data.begin(),68,rows.data.begin());
  std::copy_n(flat.data.begin()+68,68,rows.data.begin()+72);
  rows.data[68]=rows.data[140]=255;
  auto result=scan_planner::decodeProjectedRays(rows,"odom");ASSERT_TRUE(result);
  EXPECT_EQ(result->ray_kinds,(std::vector<std::uint8_t>{0,1}));
  EXPECT_DOUBLE_EQ(result->endpoints[1].x(),2.);
}
