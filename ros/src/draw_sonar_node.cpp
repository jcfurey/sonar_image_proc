// Copyright 2021 University of Washington Applied Physics Laboratory
// Ported to ROS 2

#include "rclcpp/rclcpp.hpp"
#include "sonar_image_proc/draw_sonar_component.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<draw_sonar::DrawSonarComponent>(options);
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
