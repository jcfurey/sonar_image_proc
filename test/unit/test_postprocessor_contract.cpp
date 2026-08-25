// Copyright 2026 ERDC

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "sonar_image_proc/sonar_postprocessor_component.hpp"

namespace
{

using marine_acoustic_msgs::msg::ProjectedSonarImage;
using marine_acoustic_msgs::msg::SonarImageData;

void appendUint32(std::vector<std::uint8_t> & bytes, std::uint32_t value)
{
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    bytes.push_back(static_cast<std::uint8_t>(value >> shift));
  }
}

TEST(PostprocessorContract, LegacyInputProducesCanonicalBeamMajorOutput)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/postprocessor_contract"});
  options.parameter_overrides({
      rclcpp::Parameter("input_image_layout", "range_major"),
      rclcpp::Parameter("gain", 1.0),
      rclcpp::Parameter("gamma", 0.0),
  });
  auto processor =
    std::make_shared<sonar_postprocessor::SonarPostprocessorComponent>(options);
  auto harness = std::make_shared<rclcpp::Node>("postprocessor_contract_harness");

  ProjectedSonarImage::ConstSharedPtr output;
  auto subscription = harness->create_subscription<ProjectedSonarImage>(
      "/postprocessor_contract/sonar_image_postproc", rclcpp::QoS(10),
    [&](ProjectedSonarImage::ConstSharedPtr message) {
      output = std::move(message);
      });
  auto publisher = harness->create_publisher<ProjectedSonarImage>(
      "/postprocessor_contract/sonar_image", rclcpp::SensorDataQoS());

  ProjectedSonarImage ping;
  ping.header.frame_id = "sonar";
  ping.ranges = {1.0F, 2.0F, 3.0F};
  ping.beam_directions.resize(2);
  ping.beam_directions[0].z = 1.0;
  ping.beam_directions[1].y = 0.1;
  ping.beam_directions[1].z = 0.99;
  ping.image.dtype = SonarImageData::DTYPE_UINT32;
  ping.image.beam_count = 0;  // pre-a355c65 compatibility path
  // Range-major: [r0b0, r0b1, r1b0, r1b1, r2b0, r2b1].
  for (const std::uint32_t value :
    {1U, 65536U, 16U, 16777216U, 256U,
      std::numeric_limits<std::uint32_t>::max()})
  {
    appendUint32(ping.image.data, value);
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(processor);
  executor.add_node(harness);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::seconds(3);
  while (!output && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(ping);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(output);
  EXPECT_EQ(output->image.dtype, SonarImageData::DTYPE_UINT8);
  EXPECT_EQ(output->image.beam_count, 2U);
  // Beam-major: all three ranges for beam 0, then all three for beam 1.
  EXPECT_EQ(output->image.data,
    (std::vector<std::uint8_t>{0, 31, 63, 127, 191, 255}));

  executor.remove_node(harness);
  executor.remove_node(processor);
  rclcpp::shutdown();
}

}  // namespace
