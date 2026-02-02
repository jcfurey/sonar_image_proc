// Copyright 2021 University of Washington Applied Physics Laboratory
// Ported to ROS 2

#include "rclcpp/rclcpp.hpp"
#include "sonar_image_proc/sonar_postprocessor_component.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<sonar_postprocessor::SonarPostprocessorComponent>(options);
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
