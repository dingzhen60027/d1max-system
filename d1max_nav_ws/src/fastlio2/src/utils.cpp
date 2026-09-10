#include "utils.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace
{
const sensor_msgs::msg::PointField *findField(
    const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
{
    const auto field = std::find_if(
        msg.fields.begin(), msg.fields.end(),
        [&name](const sensor_msgs::msg::PointField &candidate)
        {
            return candidate.name == name;
        });
    return field == msg.fields.end() ? nullptr : &*field;
}

std::size_t fieldSize(uint8_t datatype)
{
    switch (datatype)
    {
    case sensor_msgs::msg::PointField::UINT8:
    case sensor_msgs::msg::PointField::INT8:
        return 1;
    case sensor_msgs::msg::PointField::UINT16:
    case sensor_msgs::msg::PointField::INT16:
        return 2;
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::FLOAT32:
        return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
        return 8;
    default:
        return 0;
    }
}

template <typename T>
T readValue(const uint8_t *data)
{
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

bool readNumeric(
    const uint8_t *data, const sensor_msgs::msg::PointField &field, double &value)
{
    switch (field.datatype)
    {
    case sensor_msgs::msg::PointField::UINT8:
        value = readValue<uint8_t>(data + field.offset);
        return true;
    case sensor_msgs::msg::PointField::INT8:
        value = readValue<int8_t>(data + field.offset);
        return true;
    case sensor_msgs::msg::PointField::UINT16:
        value = readValue<uint16_t>(data + field.offset);
        return true;
    case sensor_msgs::msg::PointField::INT16:
        value = readValue<int16_t>(data + field.offset);
        return true;
    case sensor_msgs::msg::PointField::UINT32:
        value = readValue<uint32_t>(data + field.offset);
        return true;
    case sensor_msgs::msg::PointField::INT32:
        value = readValue<int32_t>(data + field.offset);
        return true;
    case sensor_msgs::msg::PointField::FLOAT32:
        value = readValue<float>(data + field.offset);
        return true;
    case sensor_msgs::msg::PointField::FLOAT64:
        value = readValue<double>(data + field.offset);
        return true;
    default:
        return false;
    }
}
} // namespace

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    if (!msg || filter_num <= 0)
        return cloud;
    const int point_num = std::min<int>(msg->point_num, msg->points.size());
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pointCloud2ToPCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    int filter_num,
    double point_time_scale_to_ms,
    double min_range,
    double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    if (!msg || msg->is_bigendian || filter_num <= 0 ||
        point_time_scale_to_ms <= 0.0)
    {
        return cloud;
    }

    const auto *x_field = findField(*msg, "x");
    const auto *y_field = findField(*msg, "y");
    const auto *z_field = findField(*msg, "z");
    const auto *intensity_field = findField(*msg, "intensity");
    const auto *time_field = findField(*msg, "time");
    if (!x_field || !y_field || !z_field || !intensity_field || !time_field)
    {
        return cloud;
    }

    const std::array<const sensor_msgs::msg::PointField *, 5> required_fields{
        x_field, y_field, z_field, intensity_field, time_field};
    for (const auto *field : required_fields)
    {
        const std::size_t size = fieldSize(field->datatype);
        if (size == 0 || field->count != 1 ||
            static_cast<std::size_t>(field->offset) + size > msg->point_step)
        {
            return cloud;
        }
    }

    const std::size_t point_count =
        static_cast<std::size_t>(msg->width) * msg->height;
    cloud->reserve(point_count / static_cast<std::size_t>(filter_num) + 1);
    const double min_range_squared = min_range * min_range;
    const double max_range_squared = max_range * max_range;
    std::size_t linear_index = 0;

    for (uint32_t row = 0; row < msg->height; ++row)
    {
        const std::size_t row_offset = static_cast<std::size_t>(row) * msg->row_step;
        for (uint32_t column = 0; column < msg->width; ++column, ++linear_index)
        {
            if (linear_index % static_cast<std::size_t>(filter_num) != 0)
            {
                continue;
            }
            const std::size_t point_offset =
                row_offset + static_cast<std::size_t>(column) * msg->point_step;
            if (point_offset + msg->point_step > msg->data.size())
            {
                return pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZINormal>);
            }

            const uint8_t *data = msg->data.data() + point_offset;
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double intensity = 0.0;
            double relative_time = 0.0;
            if (!readNumeric(data, *x_field, x) ||
                !readNumeric(data, *y_field, y) ||
                !readNumeric(data, *z_field, z) ||
                !readNumeric(data, *intensity_field, intensity) ||
                !readNumeric(data, *time_field, relative_time))
            {
                return pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZINormal>);
            }
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                !std::isfinite(intensity) || !std::isfinite(relative_time))
            {
                continue;
            }

            const double range_squared = x * x + y * y + z * z;
            if (range_squared < min_range_squared ||
                range_squared > max_range_squared)
            {
                continue;
            }

            pcl::PointXYZINormal point{};
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            point.z = static_cast<float>(z);
            point.intensity = static_cast<float>(intensity);
            point.curvature =
                static_cast<float>(relative_time * point_time_scale_to_ms);
            if (point.curvature >= 0.0F)
            {
                cloud->push_back(point);
            }
        }
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
