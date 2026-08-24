// Copyright 2026 ERDC

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "sonar_image_proc/draw_sonar_component.hpp"
#include "sonar_image_proc/msg/fan_image_info.hpp"
#include "sonar_image_proc/msg/rectified_image_info.hpp"

namespace {

using marine_acoustic_msgs::msg::ProjectedSonarImage;
using marine_acoustic_msgs::msg::SonarImageData;

ProjectedSonarImage makePing() {
  ProjectedSonarImage ping;
  ping.header.stamp.sec = 42;
  ping.header.stamp.nanosec = 123456789;
  ping.header.frame_id = "sonar_projection_frame";
  for (int range = 1; range <= 161; ++range)
    ping.ranges.push_back(0.02F * range);
  for (int beam = 0; beam < 33; ++beam) {
    const double azimuth = -0.5 + beam / 32.0;
    geometry_msgs::msg::Vector3 direction;
    direction.x = 0.0;
    direction.y = -std::sin(azimuth);
    direction.z = std::cos(azimuth);
    ping.beam_directions.push_back(direction);
  }
  ping.image.dtype = SonarImageData::DTYPE_UINT8;
  ping.image.beam_count = ping.beam_directions.size();
  // Current ProjectedSonarImage wire order is beam-major. A level head has
  // platform up along projection -x, so a 1-rad transmit aperture first sees
  // a floor 0.40 m below the sonar at r=0.40/sin(0.5)=0.83 m. Paint the broad
  // persistent return that the production detector expects, plus one thin
  // nearer ring it must reject.
  for (size_t beam = 0; beam < ping.beam_directions.size(); ++beam) {
    for (const float range : ping.ranges) {
      uint8_t intensity = range >= 0.40F / std::sin(0.5F) ? 220 : 10;
      if (std::abs(range - 0.50F) < 0.005F) intensity = 255;
      ping.image.data.push_back(intensity);
    }
  }
  ping.ping_info.tx_beamwidths.assign(ping.beam_directions.size(), 1.0F);
  ping.ping_info.rx_beamwidths.assign(ping.beam_directions.size(), 0.1F);
  return ping;
}

TEST(FanInfoContract, PublishesTruthfulFanAndRectifiedGeometry) {
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/fan_info_contract"});
  options.parameter_overrides({
      rclcpp::Parameter("use_gpu", false),
      rclcpp::Parameter("publish_legacy_camera_info", false),
      rclcpp::Parameter("publish_legacy_rect_topic", false),
      rclcpp::Parameter("rectified_width", 7),
      rclcpp::Parameter("rectified_height", 5),
      rclcpp::Parameter("floor_projection_optical_frame",
                        "sonar_optical_frame"),
      rclcpp::Parameter("floor_projection_reference_frame", "base_link"),
  });

  auto drawer = std::make_shared<draw_sonar::DrawSonarComponent>(options);
  auto harness = std::make_shared<rclcpp::Node>("fan_info_contract_harness");

  sensor_msgs::msg::Image::ConstSharedPtr clean_image;
  sensor_msgs::msg::Image::ConstSharedPtr polar_image;
  sensor_msgs::msg::Image::ConstSharedPtr rectified_image;
  sensor_msgs::msg::Image::ConstSharedPtr floor_projected_image;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr floor_projected_info;
  sonar_image_proc::msg::FanImageInfo::ConstSharedPtr fan_info;
  sonar_image_proc::msg::RectifiedImageInfo::ConstSharedPtr rectified_info;
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  auto clean_subscription = harness->create_subscription<sensor_msgs::msg::Image>(
      "/fan_info_contract/drawn_sonar_clean", reliable_qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr message) {
        clean_image = std::move(message);
      });
  auto polar_subscription = harness->create_subscription<sensor_msgs::msg::Image>(
      "/fan_info_contract/drawn_sonar_polar", reliable_qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr message) {
        polar_image = std::move(message);
      });
  auto info_subscription =
      harness->create_subscription<sonar_image_proc::msg::FanImageInfo>(
          "/fan_info_contract/fan_info", reliable_qos,
          [&](sonar_image_proc::msg::FanImageInfo::ConstSharedPtr message) {
            fan_info = std::move(message);
          });
  auto rectified_subscription =
      harness->create_subscription<sensor_msgs::msg::Image>(
          "/fan_info_contract/drawn_sonar_rectified", reliable_qos,
          [&](sensor_msgs::msg::Image::ConstSharedPtr message) {
            rectified_image = std::move(message);
          });
  auto rectified_info_subscription =
      harness->create_subscription<sonar_image_proc::msg::RectifiedImageInfo>(
          "/fan_info_contract/rectified_info", reliable_qos,
          [&](sonar_image_proc::msg::RectifiedImageInfo::ConstSharedPtr message) {
            rectified_info = std::move(message);
          });
  auto floor_projected_subscription =
      harness->create_subscription<sensor_msgs::msg::Image>(
          "/fan_info_contract/drawn_sonar_floor_projected", reliable_qos,
          [&](sensor_msgs::msg::Image::ConstSharedPtr message) {
            floor_projected_image = std::move(message);
          });
  auto floor_projected_info_subscription =
      harness->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/fan_info_contract/floor_projected_camera_info", reliable_qos,
          [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
            floor_projected_info = std::move(message);
          });
  auto ping_publisher = harness->create_publisher<ProjectedSonarImage>(
      "/fan_info_contract/sonar_image", rclcpp::SensorDataQoS());

  // The node-based constructor is available in both Jazzy and newer tf2_ros.
  // RequiredInterfaces was added after Jazzy, so using it here makes an
  // otherwise portable test fail to compile on the deployed ROS release.
  tf2_ros::StaticTransformBroadcaster static_tf(harness);
  geometry_msgs::msg::TransformStamped projection_from_optical;
  projection_from_optical.header.stamp = harness->now();
  projection_from_optical.header.frame_id = "sonar_projection_frame";
  projection_from_optical.child_frame_id = "sonar_optical_frame";
  projection_from_optical.transform.rotation.z = -std::sqrt(0.5);
  projection_from_optical.transform.rotation.w = std::sqrt(0.5);
  geometry_msgs::msg::TransformStamped projection_from_base;
  projection_from_base.header = projection_from_optical.header;
  projection_from_base.child_frame_id = "base_link";
  // base +z (platform up) -> projection -x for a level head. This is the
  // orientation half of the floor estimate; its translation is deliberately
  // irrelevant because standoff comes from the ping.
  projection_from_base.transform.rotation.y = -std::sqrt(0.5);
  projection_from_base.transform.rotation.w = std::sqrt(0.5);
  static_tf.sendTransform({projection_from_optical, projection_from_base});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(drawer);
  executor.add_node(harness);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(3);
  const auto ping = makePing();
  while ((!clean_image || !polar_image || !fan_info || !rectified_image ||
          !rectified_info || !floor_projected_image ||
          !floor_projected_info) &&
         std::chrono::steady_clock::now() < deadline) {
    ping_publisher->publish(ping);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(clean_image);
  ASSERT_TRUE(polar_image);
  ASSERT_TRUE(fan_info);
  ASSERT_TRUE(rectified_image);
  ASSERT_TRUE(rectified_info);
  ASSERT_TRUE(floor_projected_image);
  ASSERT_TRUE(floor_projected_info);
  EXPECT_EQ(clean_image->header, fan_info->header);
  EXPECT_EQ(clean_image->width, fan_info->width);
  EXPECT_EQ(clean_image->height, fan_info->height);
  EXPECT_DOUBLE_EQ(fan_info->origin_v,
                   static_cast<double>(fan_info->height));
  EXPECT_GT(fan_info->pixels_per_meter, 0.0);
  EXPECT_NEAR(fan_info->min_range, 0.02, 1e-6);
  EXPECT_NEAR(fan_info->max_range, 3.22, 1e-6);
  EXPECT_LT(fan_info->min_bearing, 0.0);
  EXPECT_GT(fan_info->max_bearing, 0.0);
  EXPECT_EQ(polar_image->height, ping.ranges.size());
  EXPECT_EQ(polar_image->width, ping.beam_directions.size());
  EXPECT_EQ(rectified_image->header, rectified_info->header);
  EXPECT_EQ(rectified_image->width, 7U);
  EXPECT_EQ(rectified_image->height, 5U);
  EXPECT_EQ(rectified_image->width, rectified_info->width);
  EXPECT_EQ(rectified_image->height, rectified_info->height);
  EXPECT_NEAR(rectified_info->min_range, 0.02, 1e-6);
  EXPECT_NEAR(rectified_info->max_range, 3.22, 1e-6);
  EXPECT_NEAR(rectified_info->meters_per_row, 0.8, 1e-6);
  EXPECT_NEAR(rectified_info->principal_point_u, 3.0, 1e-6);
  EXPECT_NEAR(
      std::atan((0.0 - rectified_info->principal_point_u) /
                rectified_info->horizontal_focal_length),
      rectified_info->min_bearing, 1e-6);
  EXPECT_NEAR(
      std::atan((6.0 - rectified_info->principal_point_u) /
                rectified_info->horizontal_focal_length),
      rectified_info->max_bearing, 1e-6);
  EXPECT_EQ(floor_projected_image->header,
            floor_projected_info->header);
  EXPECT_EQ(floor_projected_image->header.frame_id,
            "sonar_optical_frame");
  EXPECT_EQ(floor_projected_image->width, 7U);
  EXPECT_EQ(floor_projected_image->height, 5U);
  // The rectangular raster contains the measured spherical aperture. This
  // synthetic ping has +/-0.5 rad on both axes; the sec(azimuth) term makes
  // the vertical pinhole extent largest at the outer bearing beams.
  EXPECT_NEAR(floor_projected_info->k[0], 3.0 / std::tan(0.5), 1e-6);
  EXPECT_NEAR(floor_projected_info->k[4],
              2.0 * std::cos(0.5) / std::tan(0.5), 1e-6);
  EXPECT_DOUBLE_EQ(floor_projected_info->k[2], 3.0);
  EXPECT_DOUBLE_EQ(floor_projected_info->k[5], 2.0);
  const double corner_x =
      (0.0 - floor_projected_info->k[2]) / floor_projected_info->k[0];
  const double corner_horizontal_range = std::hypot(corner_x, 1.0);
  EXPECT_NEAR(std::atan(((0.0 - floor_projected_info->k[5]) /
                         floor_projected_info->k[4]) /
                        corner_horizontal_range),
              -0.5, 1e-6);
  EXPECT_NEAR(std::atan(((4.0 - floor_projected_info->k[5]) /
                         floor_projected_info->k[4]) /
                        corner_horizontal_range),
              0.5, 1e-6);
  const size_t row_bytes = floor_projected_image->step;
  const auto top_sum = std::accumulate(
      floor_projected_image->data.begin(),
      floor_projected_image->data.begin() + row_bytes, 0U);
  const auto bottom_sum = std::accumulate(
      floor_projected_image->data.end() - row_bytes,
      floor_projected_image->data.end(), 0U);
  EXPECT_EQ(top_sum, 0U);
  EXPECT_GT(bottom_sum, 0U);
  EXPECT_EQ(drawer->count_publishers("/fan_info_contract/camera_info"), 0U);
  EXPECT_EQ(drawer->count_publishers("/fan_info_contract/drawn_sonar_rect"),
            0U);

  executor.remove_node(harness);
  executor.remove_node(drawer);
  rclcpp::shutdown();
}

}  // namespace
