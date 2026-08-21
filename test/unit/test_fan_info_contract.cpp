// Copyright 2026 ERDC

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "sonar_image_proc/draw_sonar_component.hpp"
#include "sonar_image_proc/msg/fan_image_info.hpp"

namespace {

using marine_acoustic_msgs::msg::ProjectedSonarImage;
using marine_acoustic_msgs::msg::SonarImageData;

ProjectedSonarImage makePing() {
  ProjectedSonarImage ping;
  ping.header.stamp.sec = 42;
  ping.header.stamp.nanosec = 123456789;
  ping.header.frame_id = "sonar_projection_frame";
  ping.ranges = {1.0F, 2.0F, 3.0F};
  for (const double azimuth : {-0.5, 0.0, 0.5}) {
    geometry_msgs::msg::Vector3 beam;
    beam.x = 0.0;
    beam.y = -std::sin(azimuth);
    beam.z = std::cos(azimuth);
    ping.beam_directions.push_back(beam);
  }
  ping.image.dtype = SonarImageData::DTYPE_UINT8;
  ping.image.beam_count = 3;
  ping.image.data = {10, 20, 30, 40, 50, 60, 70, 80, 90};
  return ping;
}

TEST(FanInfoContract, PublishesStampedOrthographicGeometryAndPolarName) {
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/fan_info_contract"});
  options.parameter_overrides({
      rclcpp::Parameter("use_gpu", false),
      rclcpp::Parameter("publish_legacy_camera_info", false),
      rclcpp::Parameter("publish_legacy_rect_topic", false),
  });

  auto drawer = std::make_shared<draw_sonar::DrawSonarComponent>(options);
  auto harness = std::make_shared<rclcpp::Node>("fan_info_contract_harness");

  sensor_msgs::msg::Image::ConstSharedPtr clean_image;
  sensor_msgs::msg::Image::ConstSharedPtr polar_image;
  sonar_image_proc::msg::FanImageInfo::ConstSharedPtr fan_info;
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
  auto ping_publisher = harness->create_publisher<ProjectedSonarImage>(
      "/fan_info_contract/sonar_image", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(drawer);
  executor.add_node(harness);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(3);
  const auto ping = makePing();
  while ((!clean_image || !polar_image || !fan_info) &&
         std::chrono::steady_clock::now() < deadline) {
    ping_publisher->publish(ping);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(clean_image);
  ASSERT_TRUE(polar_image);
  ASSERT_TRUE(fan_info);
  EXPECT_EQ(clean_image->header, fan_info->header);
  EXPECT_EQ(clean_image->width, fan_info->width);
  EXPECT_EQ(clean_image->height, fan_info->height);
  EXPECT_DOUBLE_EQ(fan_info->origin_v,
                   static_cast<double>(fan_info->height));
  EXPECT_GT(fan_info->pixels_per_meter, 0.0);
  EXPECT_DOUBLE_EQ(fan_info->min_range, 1.0);
  EXPECT_DOUBLE_EQ(fan_info->max_range, 3.0);
  EXPECT_LT(fan_info->min_bearing, 0.0);
  EXPECT_GT(fan_info->max_bearing, 0.0);
  EXPECT_EQ(polar_image->height, ping.ranges.size());
  EXPECT_EQ(polar_image->width, ping.beam_directions.size());
  EXPECT_EQ(drawer->count_publishers("/fan_info_contract/camera_info"), 0U);
  EXPECT_EQ(drawer->count_publishers("/fan_info_contract/drawn_sonar_rect"),
            0U);

  executor.remove_node(harness);
  executor.remove_node(drawer);
  rclcpp::shutdown();
}

}  // namespace
