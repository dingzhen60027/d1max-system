#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "pointcloud_preprocess.h"

namespace {
using faster_lio::LidarType;
using faster_lio::PointCloudPreprocess;

velodyne_ros::Point Point(float time, uint16_t ring = 0, float x = 1.F) {
    velodyne_ros::Point p{};
    p.x = x;
    p.y = .2F;
    p.z = .3F;
    p.intensity = 42.F;
    p.time = time;
    p.ring = ring;
    return p;
}

sensor_msgs::msg::PointCloud2::SharedPtr Cloud(const std::vector<velodyne_ros::Point>& points) {
    pcl::PointCloud<velodyne_ros::Point> pcl;
    for (const auto& p : points) pcl.push_back(p);
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pcl, *msg);
    return msg;
}

PointCloudPreprocess Processor(int stride = 1) {
    PointCloudPreprocess processor;
    processor.Set(LidarType::VELO32, .5, stride);
    processor.NumScans() = 192;
    processor.TimeScale() = 1.F;
    return processor;
}
}  // namespace

TEST(PointcloudPreprocess, ValidAdapterInputPreservesValuesAndStride) {
    auto processor = Processor(2);
    const auto input = Cloud({Point(0.F), Point(10.F), Point(20.F, 191, 2.F), Point(30.F)});
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(input, output);
    ASSERT_EQ(output->size(), 2U);
    EXPECT_FLOAT_EQ(output->at(0).x, 1.F);
    EXPECT_FLOAT_EQ(output->at(0).curvature, 0.F);
    EXPECT_FLOAT_EQ(output->at(1).x, 2.F);
    EXPECT_FLOAT_EQ(output->at(1).y, .2F);
    EXPECT_FLOAT_EQ(output->at(1).z, .3F);
    EXPECT_FLOAT_EQ(output->at(1).intensity, 42.F);
    EXPECT_FLOAT_EQ(output->at(1).curvature, 20.F);
}

TEST(PointcloudPreprocess, ValidDenseCloudMatchesLegacyTimestampedFilter) {
    std::vector<velodyne_ros::Point> points;
    for (int i = 0; i < 1000; ++i)
        points.push_back(Point(i * .1F, i % 192, .01F * (i % 300)));
    const auto cloud = Cloud(points);
    for (int stride : {1, 2, 3}) {
        auto processor = Processor(stride);
        processor.TimeScale() = .5F;
        PointCloudType::Ptr output(new PointCloudType);
        processor.Process(cloud, output);
        size_t index = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& p = points[i];
            if (i % stride != 0 || p.x*p.x + p.y*p.y + p.z*p.z <= .5*.5) continue;
            ASSERT_LT(index, output->size());
            const auto& q = output->at(index++);
            EXPECT_FLOAT_EQ(q.x, p.x);
            EXPECT_FLOAT_EQ(q.y, p.y);
            EXPECT_FLOAT_EQ(q.z, p.z);
            EXPECT_FLOAT_EQ(q.intensity, p.intensity);
            EXPECT_FLOAT_EQ(q.curvature, p.time * .5F);
        }
        EXPECT_EQ(output->size(), index);
    }
}

TEST(PointcloudPreprocess, EmptyCloudClearsPreviousOutput) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(Cloud({Point(0.F), Point(10.F)}), output);
    ASSERT_EQ(output->size(), 2U);
    processor.Process(Cloud({}), output);
    EXPECT_TRUE(output->empty());
}

TEST(PointcloudPreprocess, NullInputAndOutputAreSafe) {
    auto processor = Processor();
    PointCloudType::Ptr output;
    processor.Process(sensor_msgs::msg::PointCloud2::SharedPtr{}, output);
    ASSERT_TRUE(output);
    EXPECT_TRUE(output->empty());
    processor.Process(livox_ros_driver2::msg::CustomMsg::SharedPtr{}, output);
    EXPECT_TRUE(output->empty());
}

TEST(PointcloudPreprocess, RejectsNonfiniteCoordinatesIntensityAndInvalidRing) {
    auto processor = Processor();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    auto a = Point(1.F); a.x = nan;
    auto b = Point(2.F); b.y = inf;
    auto c = Point(3.F); c.z = nan;
    auto d = Point(4.F); d.intensity = inf;
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(Cloud({Point(0.F), a, b, c, d, Point(5.F, 192), Point(6.F, 65535), Point(10.F, 191)}), output);
    ASSERT_EQ(output->size(), 2U);
    EXPECT_FLOAT_EQ(output->front().curvature, 0.F);
    EXPECT_FLOAT_EQ(output->back().curvature, 10.F);
}

TEST(PointcloudPreprocess, InvalidTailDoesNotTriggerSyntheticTiming) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(Cloud({Point(0.F), Point(10.F), Point(20.F),
        Point(std::numeric_limits<float>::quiet_NaN())}), output);
    ASSERT_EQ(output->size(), 3U);
    EXPECT_FLOAT_EQ(output->front().curvature, 0.F);
    EXPECT_FLOAT_EQ(output->at(1).curvature, 10.F);
    EXPECT_FLOAT_EQ(output->back().curvature, 20.F);
}

TEST(PointcloudPreprocess, ValidTimeDetectionDoesNotDependOnDownsamplingStride) {
    auto processor = Processor(2);
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(Cloud({Point(0.F), Point(10.F)}), output);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_FLOAT_EQ(output->front().curvature, 0.F);
}

TEST(PointcloudPreprocess, RejectsIncompatibleExplicitTimeSchemaWithoutFallback) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    for (int defect = 0; defect < 4; ++defect) {
        const auto message = Cloud({Point(0.F), Point(10.F)});
        for (auto& field : message->fields) {
            if (field.name != "time") continue;
            if (defect == 0) field.datatype = sensor_msgs::msg::PointField::FLOAT64;
            if (defect == 1) field.count = 0;
            if (defect == 2) field.count = 2;
            if (defect == 3) field.offset = message->point_step;
        }
        processor.Process(message, output);
        EXPECT_TRUE(output->empty());
    }
}

TEST(PointcloudPreprocess, MissingTimeFieldRetainsLegacyTiming) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    auto second = Point(0.F); second.y = .1F;
    const auto message = Cloud({Point(0.F), second});
    message->fields.erase(std::remove_if(message->fields.begin(), message->fields.end(),
        [](const auto& field) { return field.name == "time"; }), message->fields.end());
    processor.Process(message, output);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_GT(output->front().curvature, 0.F);
}

TEST(PointcloudPreprocess, RejectsNegativeNonfiniteAndOverflowedTimes) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(Cloud({Point(-1.F), Point(std::numeric_limits<float>::infinity()),
        Point(std::numeric_limits<float>::quiet_NaN()), Point(10.F)}), output);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_FLOAT_EQ(output->front().curvature, 10.F);
    processor.TimeScale() = 2.F;
    processor.Process(Cloud({Point(std::numeric_limits<float>::max()), Point(10.F)}), output);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_FLOAT_EQ(output->front().curvature, 20.F);
}

TEST(PointcloudPreprocess, UnorderedValidTimesEndAtMaximumAndKeepTieOrder) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(Cloud({Point(20.F, 0, 2.F), Point(0.F), Point(20.F, 0, 3.F), Point(10.F)}), output);
    ASSERT_EQ(output->size(), 4U);
    EXPECT_FLOAT_EQ(output->front().curvature, 0.F);
    EXPECT_FLOAT_EQ(output->at(1).curvature, 10.F);
    EXPECT_FLOAT_EQ(output->at(2).x, 2.F);
    EXPECT_FLOAT_EQ(output->back().x, 3.F);
    EXPECT_FLOAT_EQ(output->back().curvature, 20.F);
}

TEST(PointcloudPreprocess, LegacyZeroTimesSkipInvalidRingBeforeIndexing) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    auto second = Point(0.F); second.y = .1F;
    processor.Process(Cloud({Point(0.F, 65535), Point(0.F), second}), output);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_TRUE(std::isfinite(output->front().curvature));
    EXPECT_GT(output->front().curvature, 0.F);
}

TEST(PointcloudPreprocess, InvalidConfigurationDoesNotDivideByZeroOrAllocateBadRingVectors) {
    PointCloudType::Ptr output(new PointCloudType);
    const auto cloud = Cloud({Point(10.F)});
    auto processor = Processor(0);
    processor.Process(cloud, output);
    EXPECT_TRUE(output->empty());
    processor = Processor(); processor.NumScans() = -1;
    processor.Process(cloud, output);
    EXPECT_TRUE(output->empty());
    processor = Processor(); processor.TimeScale() = std::numeric_limits<float>::quiet_NaN();
    processor.Process(cloud, output);
    EXPECT_TRUE(output->empty());
}

TEST(PointcloudPreprocess, UnknownTypeCannotReturnPreviousScan) {
    auto processor = Processor();
    PointCloudType::Ptr output(new PointCloudType);
    const auto cloud = Cloud({Point(10.F)});
    processor.Process(cloud, output);
    ASSERT_EQ(output->size(), 1U);
    processor.SetLidarType(static_cast<LidarType>(999));
    processor.Process(cloud, output);
    EXPECT_TRUE(output->empty());
}

TEST(PointcloudPreprocess, OusterRejectsInvalidGeometryAndRing) {
    PointCloudPreprocess processor;
    processor.Set(LidarType::OUST64, .5, 1);
    processor.NumScans() = 64;
    pcl::PointCloud<ouster_ros::Point> pcl;
    ouster_ros::Point good{};
    good.x = 1.F; good.y = .2F; good.z = .3F;
    good.intensity = 42.F; good.ring = 63; good.t = 10000000;
    pcl.push_back(good);
    auto bad = good; bad.x = std::numeric_limits<float>::quiet_NaN(); pcl.push_back(bad);
    bad = good; bad.intensity = std::numeric_limits<float>::infinity(); pcl.push_back(bad);
    bad = good; bad.ring = 64; pcl.push_back(bad);
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pcl, *message);
    PointCloudType::Ptr output(new PointCloudType);
    processor.Process(message, output);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_FLOAT_EQ(output->front().x, 1.F);
    EXPECT_FLOAT_EQ(output->front().curvature, 10.F);
}
