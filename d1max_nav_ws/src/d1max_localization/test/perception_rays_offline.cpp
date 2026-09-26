// Offline CDR batch converter. Deliberately NO rclcpp::init(), Node or executor.
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "d1max_localization/perception_rays.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: perception_rays_offline requests.jsonl\n";
    return 2;
  }
  std::ifstream requests(argv[1]);
  if (!requests) {std::cerr << "cannot read request manifest\n"; return 2;}
  namespace rays = d1max_localization::perception_rays;
  using Cloud = sensor_msgs::msg::PointCloud2;
  rclcpp::Serialization<Cloud> serializer;
  std::string line;
  size_t index = 0, failures = 0;
  while (std::getline(requests, line)) {
    nlohmann::json response{{"index", index++}, {"ok", false}};
    try {
      if (line.size() > 16384) {throw std::runtime_error("oversize request");}
      const auto request = nlohmann::json::parse(line);
      const std::filesystem::path input_path(request.at("input_cdr").get<std::string>());
      const std::filesystem::path output_path(request.at("output_cdr").get<std::string>());
      const auto bytes = std::filesystem::file_size(input_path);
      if (bytes == 0 || bytes > 64U*1024U*1024U || std::filesystem::exists(output_path)) {
        throw std::runtime_error("invalid input size or output already exists");
      }
      std::ifstream input(input_path, std::ios::binary);
      rclcpp::SerializedMessage encoded(bytes);
      auto & storage = encoded.get_rcl_serialized_message();
      input.read(reinterpret_cast<char *>(storage.buffer), bytes);
      if (!input || static_cast<uintmax_t>(input.gcount()) != bytes) {
        throw std::runtime_error("short CDR read");
      }
      storage.buffer_length = bytes;
      Cloud cloud;
      serializer.deserialize_message(&encoded, &cloud);
      const auto translation = request.at("translation").get<std::array<double, 3>>();
      const auto rotation = request.at("rotation").get<std::array<double, 4>>();
      tf2::Quaternion q(rotation[0], rotation[1], rotation[2], rotation[3]);
      if (!std::isfinite(q.length()) || std::abs(q.length()-1.) > .001) {
        throw std::runtime_error("invalid source calibration quaternion");
      }
      q.normalize();
      rays::Options options;
      options.min_range = request.value("min_range", .5);
      options.max_range = request.value("max_range", 60.);
      options.scan_period = request.value("scan_period", .1);
      options.relative_timestamp_scale = request.value("relative_timestamp_scale", 1.);
      options.max_input_points = request.value("max_input_points", 250000U);
      const auto sensor = request.at("sensor_id").get<int>();
      if (sensor != 0 && sensor != 1) {throw std::runtime_error("invalid sensor identity");}
      const auto result = rays::convert(cloud, static_cast<rays::Sensor>(sensor),
        request.at("source_frame").get<std::string>(), request.at("target_frame").get<std::string>(),
        tf2::Transform(q, tf2::Vector3(translation[0], translation[1], translation[2])),
        request.at("clock_offset").get<double>(), request.at("ros_now").get<double>(), options);
      response.update({{"reason", result.error}, {"input_points", result.input_points},
        {"invalid_geometry", result.invalid_geometry}, {"outside_range", result.outside_range},
        {"invalid_time", result.invalid_time}, {"output_points", result.cloud.width}});
      if (!result) {++failures;}
      else {
        rclcpp::SerializedMessage output;
        serializer.serialize_message(&result.cloud, &output);
        const auto & data = output.get_rcl_serialized_message();
        std::ofstream target(output_path, std::ios::binary);
        target.write(reinterpret_cast<const char *>(data.buffer), data.buffer_length);
        if (!target) {throw std::runtime_error("CDR output write failed");}
        response.update({{"ok", true}, {"header_stamp", rays::stampSeconds(result.cloud.header.stamp)},
          {"latest_timestamp", result.latest_timestamp}});
      }
    } catch (const std::exception & error) {
      ++failures;
      response["reason"] = error.what();
    }
    std::cout << response.dump() << '\n';
  }
  return failures ? 1 : 0;
}
