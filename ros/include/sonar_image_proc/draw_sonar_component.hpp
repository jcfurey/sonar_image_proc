// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2

#pragma once

#include <array>
#include <memory>
#include <string>

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sonar_image_proc/msg/fan_image_info.hpp"
#include "sonar_image_proc/msg/rectified_image_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

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

  void publishRectifiedProducts(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
      const sonar_image_proc::AbstractSonarInterface &interface,
      const cv::Mat &range_bearing_image);

  void publishFloorProjectedProducts(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
      const sonar_image_proc::AbstractSonarInterface &interface,
      const cv::Mat &range_bearing_image);

  void setColorMap(const std::string &color_map_name);

  rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sub_sonar_image_;
  // Operator product: Cartesian fan with baked-in range/bearing annotations.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  // Machine-vision product: identical fan pixels without annotations.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr clean_pub_;
  // Polar range x bearing inspection image. This used to be called "rect",
  // although no camera-style rectification is involved.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr polar_pub_;
  // Temporary compatibility alias for drawn_sonar_polar.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
  // Full-ping range-bearing product in a conventional forward-facing,
  // rectilinear rectangle (far up, near down). Generated before the fan remap.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rectified_pub_;
  rclcpp::Publisher<sonar_image_proc::msg::RectifiedImageInfo>::SharedPtr
      rectified_info_pub_;
  // Perspective virtual-camera view of the ping projected onto the stamped
  // sea-floor plane. Its vertical placement comes from TF/head pitch.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_projected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      floor_projected_info_pub_;
  // Orthographic pixel<->metre mapping of the Cartesian fan.
  rclcpp::Publisher<sonar_image_proc::msg::FanImageInfo>::SharedPtr fan_info_pub_;
  // Temporary non-pinhole CameraInfo compatibility contract.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr osd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr histogram_pub_;

  sonar_image_proc::SonarDrawer sonar_drawer_;

  float max_range_;
  bool publish_timing_, publish_histogram_;
  bool publish_legacy_camera_info_{true};
  bool publish_legacy_rect_topic_{true};
  int rectified_width_{0};
  int rectified_height_{0};
  float rectified_aspect_ratio_{16.0f / 9.0f};
  std::string floor_projection_sensor_frame_;
  std::string floor_projection_optical_frame_;
  std::string floor_projection_surface_frame_{"sea_floor_estimate"};
  double floor_projection_tf_timeout_{0.05};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  float min_db_, max_db_;
  bool log_scale_;
  std::string input_image_layout_;

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
