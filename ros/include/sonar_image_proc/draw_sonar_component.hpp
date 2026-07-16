// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2

#pragma once

#include <array>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"

#include "sonar_image_proc/ColorMaps.h"
#include "sonar_image_proc/SonarDrawer.h"

namespace draw_sonar {

class DrawSonarComponent : public rclcpp::Node {
 public:
  explicit DrawSonarComponent(const rclcpp::NodeOptions & options);
  virtual ~DrawSonarComponent() = default;

 private:
  void cvBridgeAndPublish(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
      const cv::Mat &mat,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub);

  void sonarImageCallback(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg);

  void setColorMap(const std::string &color_map_name);

  rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sub_sonar_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr osd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr old_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr histogram_pub_;

  sonar_image_proc::SonarDrawer sonar_drawer_;

  float max_range_;
  bool publish_old_api_, publish_timing_, publish_histogram_;

  float min_db_, max_db_;
  bool log_scale_;

  std::unique_ptr<sonar_image_proc::SonarColorMap> color_map_;

  // CUDA draw path (lib/GpuSonarDraw.cu): colormap LUT + bicubic fan remap
  // on the GPU for uint8 pings; visually equivalent, CPU path is the
  // fallback. lut_ is the active colormap evaluated per intensity — valid
  // only for maps that are pure functions of the uint8 intensity.
  bool use_gpu_{false};
  bool lut_valid_{false};
  std::array<uint8_t, 256 * 3> lut_{};
};

}  // namespace draw_sonar
