// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2

#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"

namespace sonar_postprocessor {

class SonarPostprocessorComponent : public rclcpp::Node {
 public:
  explicit SonarPostprocessorComponent(const rclcpp::NodeOptions & options);
  virtual ~SonarPostprocessorComponent() = default;

 private:
  void sonarImageCallback(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg);

  rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sub_sonar_image_;
  rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr pub_sonar_image_;

  float gain_, gamma_;
  std::string input_image_layout_;
};

}  // namespace sonar_postprocessor
