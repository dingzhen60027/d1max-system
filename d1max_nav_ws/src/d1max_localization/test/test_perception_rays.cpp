#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <limits>

#include "d1max_localization/input_clock.hpp"
#include "d1max_localization/perception_ray_queue.hpp"

namespace rays = d1max_localization::perception_rays;
namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
constexpr double kEpoch = 1790412195.0;

Cloud inputCloud(uint32_t width = 4, uint32_t height = 1, uint32_t padding = 0)
{
  Cloud cloud;
  cloud.header.frame_id = "front_sensor";
  cloud.header.stamp.sec = static_cast<int32_t>(kEpoch);
  cloud.width = width;
  cloud.height = height;
  cloud.point_step = 32;
  cloud.row_step = cloud.point_step * cloud.width + padding;
  cloud.data.resize(static_cast<size_t>(cloud.row_step) * cloud.height, 0xA5);
  cloud.fields = {
    rays::detail::outputField("x", 0, Field::FLOAT32),
    rays::detail::outputField("y", 4, Field::FLOAT32),
    rays::detail::outputField("z", 8, Field::FLOAT32),
    rays::detail::outputField("intensity", 12, Field::FLOAT32),
    rays::detail::outputField("ring", 16, Field::UINT16),
    rays::detail::outputField("timestamp", 24, Field::FLOAT64)};
  for (uint32_t i = 0; i < width * height; ++i) {
    auto * point = cloud.data.data() + (i / width) * cloud.row_step +
      (i % width) * cloud.point_step;
    rays::detail::write<float>(point, 1.0F + i);
    rays::detail::write<float>(point + 4, 0.0F);
    rays::detail::write<float>(point + 8, -0.5F);
    rays::detail::write<float>(point + 12, 10.0F + i);
    rays::detail::write<uint16_t>(point + 16, static_cast<uint16_t>(92 + i % 4));
    rays::detail::write<double>(point + 24, 0.025 * (i % 4));
  }
  return cloud;
}

template<typename T>
T field(const Cloud & cloud, uint32_t index, const char * name)
{
  for (const auto & value : cloud.fields) {
    if (value.name == name) {
      return rays::detail::read<T>(cloud.data.data() + index * cloud.point_step + value.offset);
    }
  }
  ADD_FAILURE() << "missing field " << name;
  return {};
}

template<typename T>
void change(Cloud & cloud, uint32_t index, uint32_t offset, T value)
{
  rays::detail::write<T>(cloud.data.data() + (index / cloud.width) * cloud.row_step +
    (index % cloud.width) * cloud.point_step + offset, value);
}

rays::Result convert(const Cloud & cloud, double offset = 0.0, double now = kEpoch + 0.10,
  const rays::Options & options = {})
{
  return rays::convert(cloud, rays::Sensor::Front, "front_sensor", "tracking",
    tf2::Transform::getIdentity(), offset, now, options);
}
}  // namespace

TEST(PerceptionRays, VersionOneSchemaAndSourceOrderRoundTrip)
{
  const auto raw = inputCloud();
  const auto result = convert(raw);
  ASSERT_TRUE(result) << result.error;
  const auto & cloud = result.cloud;
  EXPECT_EQ(cloud.header.frame_id, "tracking");
  EXPECT_EQ(cloud.header.stamp.sec, static_cast<int32_t>(kEpoch));
  EXPECT_EQ(cloud.width, 4U);
  EXPECT_EQ(cloud.point_step, 64U);
  EXPECT_EQ(cloud.row_step, 256U);
  EXPECT_EQ(cloud.fields.size(), 14U);
  EXPECT_FALSE(cloud.is_bigendian);
  EXPECT_TRUE(cloud.is_dense);
  for (uint32_t i = 0; i < cloud.width; ++i) {
    EXPECT_FLOAT_EQ(field<float>(cloud, i, "x"), 1.0F + i);
    EXPECT_FLOAT_EQ(field<float>(cloud, i, "z"), -0.5F);
    EXPECT_FLOAT_EQ(field<float>(cloud, i, "intensity"), 10.0F + i);
    EXPECT_FLOAT_EQ(field<float>(cloud, i, "origin_x"), 0.0F);
    EXPECT_FLOAT_EQ(field<float>(cloud, i, "origin_y"), 0.0F);
    EXPECT_FLOAT_EQ(field<float>(cloud, i, "origin_z"), 0.0F);
    EXPECT_EQ(field<uint16_t>(cloud, i, "sensor_id"), 0U);
    EXPECT_EQ(field<uint16_t>(cloud, i, "ring"), 92U + i);
    EXPECT_EQ(field<uint32_t>(cloud, i, "source_index"), i);
    EXPECT_NEAR(field<uint32_t>(cloud, i, "offset_time"), i * 25000000., 150.0);
    EXPECT_DOUBLE_EQ(field<double>(cloud, i, "timestamp"), kEpoch + .025 * i);
    EXPECT_DOUBLE_EQ(field<double>(cloud, i, "source_timestamp"), kEpoch + .025 * i);
    EXPECT_DOUBLE_EQ(field<double>(cloud, i, "raw_timestamp"), .025 * i);
  }
  EXPECT_EQ(raw.data, inputCloud().data);  // converter never mutates original input
}

TEST(PerceptionRays, BothOriginsUseExactlyTheConfiguredExtrinsicComposition)
{
  auto front = inputCloud();
  auto rear = inputCloud();
  rear.header.frame_id = "rear_sensor";
  tf2::Quaternion rotation;
  rotation.setRPY(.2, -.3, .6);
  const tf2::Transform front_transform(rotation, tf2::Vector3(.4, .02, -.03));
  const tf2::Transform rear_to_front(tf2::Quaternion(1, 0, 0, 0), tf2::Vector3(0, 0, -.7323));
  const auto rear_transform = front_transform * rear_to_front;
  const auto a = rays::convert(front, rays::Sensor::Front, "front_sensor", "tracking",
    front_transform, 0.0, kEpoch + .1);
  const auto b = rays::convert(rear, rays::Sensor::Rear, "rear_sensor", "tracking",
    rear_transform, 0.0, kEpoch + .1);
  ASSERT_TRUE(a) << a.error;
  ASSERT_TRUE(b) << b.error;
  for (const auto * result : {&a, &b}) {
    const auto & transform = result == &a ? front_transform : rear_transform;
    const auto expected = transform * tf2::Vector3(1, 0, -.5);
    EXPECT_NEAR(field<float>(result->cloud, 0, "x"), expected.x(), 1e-6);
    EXPECT_NEAR(field<float>(result->cloud, 0, "y"), expected.y(), 1e-6);
    EXPECT_NEAR(field<float>(result->cloud, 0, "z"), expected.z(), 1e-6);
    const tf2::Vector3 origin(field<float>(result->cloud, 0, "origin_x"),
      field<float>(result->cloud, 0, "origin_y"), field<float>(result->cloud, 0, "origin_z"));
    EXPECT_NEAR((origin - transform.getOrigin()).length(), 0.0, 1e-6);
    EXPECT_NEAR((expected - origin).length(), std::sqrt(1.25), 1e-6);
  }
  EXPECT_EQ(field<uint16_t>(a.cloud, 0, "sensor_id"), 0U);
  EXPECT_EQ(field<uint16_t>(b.cloud, 0, "sensor_id"), 1U);
  EXPECT_EQ(field<uint16_t>(b.cloud, 0, "ring"), 92U);  // no 96 offset or Livox fold
}

TEST(PerceptionRays, RearOnlyMessageKeepsRearIdentityWithoutFrontPair)
{
  auto raw = inputCloud();
  raw.header.frame_id = "rear_sensor";
  const auto result = rays::convert(raw, rays::Sensor::Rear, "rear_sensor", "tracking",
    tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(-.73, 0, 0)),
    0.0, kEpoch + .10);
  ASSERT_TRUE(result);
  EXPECT_EQ(field<uint16_t>(result.cloud, 3, "sensor_id"), 1U);
  EXPECT_FLOAT_EQ(field<float>(result.cloud, 3, "origin_x"), -.73F);
}

TEST(PerceptionRays, OrganizedRowsHonorPaddingAndOriginalIndices)
{
  const auto result = convert(inputCloud(2, 2, 17));
  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.cloud.width, 4U);
  EXPECT_FLOAT_EQ(field<float>(result.cloud, 2, "x"), 3.0F);
  EXPECT_EQ(field<uint32_t>(result.cloud, 3, "source_index"), 3U);
}

TEST(PerceptionRays, NonmonotonicAcquisitionOrderDoesNotSortAwayProvenance)
{
  auto input = inputCloud();
  change<double>(input, 0, 24, .075);
  change<double>(input, 3, 24, 0.0);
  const auto result = convert(input);
  ASSERT_TRUE(result);
  EXPECT_EQ(field<uint32_t>(result.cloud, 0, "source_index"), 0U);
  EXPECT_NEAR(field<uint32_t>(result.cloud, 0, "offset_time"), 75000000., 150.);
  EXPECT_EQ(field<uint32_t>(result.cloud, 3, "offset_time"), 0U);
  EXPECT_DOUBLE_EQ(rays::stampSeconds(result.cloud.header.stamp), kEpoch);
}

TEST(PerceptionRays, SameInputClockOffsetAppliedToBothSensorTimes)
{
  d1max_localization::InputClock clock(true, 2, .1);
  EXPECT_FALSE(clock.observe(kEpoch, kEpoch + 20.01, 10.0));
  ASSERT_TRUE(clock.observe(kEpoch + .11, kEpoch + 20.12, 10.11));
  const auto shared = clock.offset(10.12);
  ASSERT_TRUE(shared);
  for (auto sensor : {rays::Sensor::Front, rays::Sensor::Rear}) {
    auto raw = inputCloud();
    raw.header.frame_id = sensor == rays::Sensor::Front ? "front_sensor" : "rear_sensor";
    const auto result = rays::convert(raw, sensor, raw.header.frame_id, "tracking",
      tf2::Transform::getIdentity(), *shared, kEpoch + 20.11);
    ASSERT_TRUE(result) << result.error;
    EXPECT_NEAR(rays::stampSeconds(result.cloud.header.stamp), kEpoch + *shared, 3e-7);
    for (uint32_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(field<double>(result.cloud, i, "timestamp"),
        field<double>(result.cloud, i, "source_timestamp") + *shared);
      EXPECT_NEAR(field<uint32_t>(result.cloud, i, "offset_time"), i * 25000000., 150.);
    }
  }
  EXPECT_FALSE(clock.offset(10.42));  // worker must not publish on expired IMU clock
}

TEST(PerceptionRays, AbsoluteTimeUnitsMatchExistingAiryDecoder)
{
  for (double scale : {1.0, 1e3, 1e6, 1e9}) {
    auto raw = inputCloud();
    for (uint32_t i = 0; i < 4; ++i) {change<double>(raw, i, 24, (kEpoch + .025 * i) * scale);}
    const auto result = convert(raw);
    ASSERT_TRUE(result) << result.error << " scale=" << scale;
    EXPECT_NEAR(field<double>(result.cloud, 2, "source_timestamp"), kEpoch + .05, 1e-6);
    EXPECT_DOUBLE_EQ(field<double>(result.cloud, 2, "raw_timestamp"), (kEpoch + .05) * scale);
  }
  EXPECT_DOUBLE_EQ(rays::decodePointTime(25000, kEpoch, 1e-6, .1), kEpoch + .025);
}

TEST(PerceptionRays, InvalidPointsAreNotInventedAndSourceIndicesRemainTraceable)
{
  auto raw = inputCloud(8);
  change<float>(raw, 1, 0, std::numeric_limits<float>::quiet_NaN());
  change<float>(raw, 2, 0, 1000.0F);
  change<double>(raw, 3, 24, std::numeric_limits<double>::quiet_NaN());
  const auto result = convert(raw);
  ASSERT_TRUE(result);
  EXPECT_EQ(result.cloud.width, 5U);
  EXPECT_EQ(result.invalid_geometry, 1U);
  EXPECT_EQ(result.outside_range, 1U);
  EXPECT_EQ(result.invalid_time, 1U);
  EXPECT_EQ(field<uint32_t>(result.cloud, 1, "source_index"), 4U);
}

TEST(PerceptionRays, RangeGateIsAboutSensorOriginNotTrackingOrigin)
{
  auto raw = inputCloud();
  rays::Options options;
  options.min_range = 2.5;
  const auto result = rays::convert(raw, rays::Sensor::Front, "front_sensor", "tracking",
    tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(-3, 0, 0)),
    0.0, kEpoch + .10, options);
  ASSERT_TRUE(result);
  EXPECT_EQ(result.outside_range, 2U);
  EXPECT_EQ(result.cloud.width, 2U);
  EXPECT_FLOAT_EQ(field<float>(result.cloud, 0, "x"), 0.0F);
  EXPECT_FLOAT_EQ(field<float>(result.cloud, 0, "origin_x"), -3.0F);
}

TEST(PerceptionRays, StaleFutureAndZeroDurationCloudsFailClosed)
{
  EXPECT_EQ(convert(inputCloud(), 0, kEpoch + 1).error, "invalid_or_stale_point_time");
  EXPECT_EQ(convert(inputCloud(), 0, kEpoch - 1).error, "invalid_or_stale_point_time");
  auto raw = inputCloud();
  for (uint32_t i = 0; i < 4; ++i) {change<double>(raw, i, 24, 0.0);}
  EXPECT_EQ(convert(raw).error, "invalid_or_stale_point_time");
  change<double>(raw, 3, 24, .19);
  EXPECT_EQ(convert(raw).error, "invalid_or_stale_point_time");
}

TEST(PerceptionRays, InvalidLayoutSchemaAndOversizeFailClosed)
{
  auto raw = inputCloud();
  raw.is_bigendian = true;
  EXPECT_EQ(convert(raw).error, "invalid_layout");
  raw = inputCloud(); raw.data.pop_back();
  EXPECT_EQ(convert(raw).error, "invalid_layout");
  raw = inputCloud(); --raw.row_step;
  EXPECT_EQ(convert(raw).error, "invalid_layout");
  raw = inputCloud(); raw.fields.back().offset = UINT32_MAX;
  EXPECT_EQ(convert(raw).error, "invalid_schema");
  raw = inputCloud(); raw.fields.back().datatype = Field::FLOAT32;
  EXPECT_EQ(convert(raw).error, "invalid_schema");
  raw = inputCloud(); raw.fields.push_back(raw.fields.back());
  EXPECT_EQ(convert(raw).error, "invalid_schema");
  raw = inputCloud(); raw.fields.back().count = 2;
  EXPECT_EQ(convert(raw).error, "invalid_schema");
  raw = inputCloud(); raw.width = UINT32_MAX;
  EXPECT_EQ(convert(raw).error, "point_count_limit");
  raw = inputCloud(); raw.width = 0;
  EXPECT_EQ(convert(raw).error, "point_count_limit");
}

TEST(PerceptionRays, SensorFrameTransformAndClockAreValidated)
{
  auto raw = inputCloud();
  raw.header.frame_id = "rear_sensor";
  EXPECT_EQ(convert(raw).error, "source_frame_mismatch");
  EXPECT_EQ(convert(inputCloud(), std::numeric_limits<double>::quiet_NaN()).error, "invalid_clock");
  raw = inputCloud(); raw.header.stamp.nanosec = 1000000000U;
  EXPECT_EQ(convert(raw).error, "invalid_clock");
  auto bad_transform = tf2::Transform::getIdentity();
  bad_transform.setOrigin(tf2::Vector3(std::numeric_limits<double>::quiet_NaN(), 0, 0));
  EXPECT_EQ(rays::convert(inputCloud(), rays::Sensor::Front, "front_sensor", "tracking",
    bad_transform, 0, kEpoch + .1).error, "invalid_transform");
  bad_transform = tf2::Transform::getIdentity();
  bad_transform.getBasis()[0][0] = 2;
  EXPECT_EQ(rays::convert(inputCloud(), rays::Sensor::Front, "front_sensor", "tracking",
    bad_transform, 0, kEpoch + .1).error, "invalid_transform");
  EXPECT_EQ(rays::convert(inputCloud(), static_cast<rays::Sensor>(5), "front_sensor", "tracking",
    tf2::Transform::getIdentity(), 0, kEpoch + .1).error, "invalid_sensor_id");
}

TEST(PerceptionRays, BoundsAreExplicitAndDoNotLowerExistingBlindRange)
{
  rays::Options options;
  EXPECT_TRUE(rays::validOptions(options));
  options.min_range = -1;
  EXPECT_FALSE(rays::validOptions(options));
  EXPECT_EQ(convert(inputCloud(), 0, kEpoch + .1, options).error, "invalid_options");
  options = {}; options.max_input_points = 0;
  EXPECT_FALSE(rays::validOptions(options));
  options = {}; options.relative_timestamp_scale = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(rays::validOptions(options));
}

TEST(PerceptionRayQueue, FrontAndRearHaveIndependentBoundedLatestSlots)
{
  rays::LatestQueue queue;
  auto a = std::make_shared<Cloud>(inputCloud());
  auto b = std::make_shared<Cloud>(inputCloud());
  auto c = std::make_shared<Cloud>(inputCloud());
  EXPECT_EQ(queue.submit(rays::Sensor::Front, a), rays::LatestQueue::Submit::Queued);
  EXPECT_EQ(queue.submit(rays::Sensor::Front, b), rays::LatestQueue::Submit::Replaced);
  EXPECT_EQ(queue.submit(rays::Sensor::Rear, c), rays::LatestQueue::Submit::Queued);
  auto first = queue.wait();
  ASSERT_TRUE(first);
  EXPECT_EQ(first->sensor, rays::Sensor::Front);
  EXPECT_EQ(first->cloud, b);
  EXPECT_EQ(queue.submit(rays::Sensor::Front, a), rays::LatestQueue::Submit::Queued);
  auto second = queue.wait();
  ASSERT_TRUE(second);
  EXPECT_EQ(second->sensor, rays::Sensor::Rear);  // busy front cannot starve rear
  EXPECT_EQ(second->cloud, c);
  queue.stop();
  EXPECT_FALSE(queue.wait());  // discard pending old front on shutdown
}

TEST(PerceptionRayQueue, StopWakesWorkerAndPreventsSubsequentSubmissions)
{
  rays::LatestQueue queue;
  auto waiting = std::async(std::launch::async, [&queue] {return queue.wait();});
  queue.stop();
  ASSERT_EQ(waiting.wait_for(std::chrono::milliseconds(200)), std::future_status::ready);
  EXPECT_FALSE(waiting.get());
  EXPECT_EQ(queue.submit(rays::Sensor::Front, std::make_shared<Cloud>(inputCloud())),
    rays::LatestQueue::Submit::Rejected);
  EXPECT_EQ(queue.submit(static_cast<rays::Sensor>(2), std::make_shared<Cloud>(inputCloud())),
    rays::LatestQueue::Submit::Rejected);
}
