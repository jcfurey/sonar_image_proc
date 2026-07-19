// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2

#include "sonar_image_proc/sonar_postprocessor_component.hpp"

#include "sonar_image_proc/sonar_image_msg_interface.h"

namespace sonar_postprocessor {

using marine_acoustic_msgs::msg::ProjectedSonarImage;
using sonar_image_proc::SonarImageMsgInterface;

SonarPostprocessorComponent::SonarPostprocessorComponent(const rclcpp::NodeOptions & options)
      : Node("sonar_postprocessor", options) {
    
    // Declare and get parameters
    this->declare_parameter("gain", 1.0);
    this->declare_parameter("gamma", 0.0);

    gain_ = this->get_parameter("gain").as_double();
    gamma_ = this->get_parameter("gamma").as_double();

    sub_sonar_image_ = this->create_subscription<ProjectedSonarImage>(
        "sonar_image", 10,
        std::bind(&SonarPostprocessorComponent::sonarImageCallback, 
                  this, std::placeholders::_1));

    pub_sonar_image_ = this->create_publisher<ProjectedSonarImage>(
        "sonar_image_postproc", 10);

    RCLCPP_DEBUG(this->get_logger(), "sonar_processor ready to run...");
  }

  void SonarPostprocessorComponent::sonarImageCallback(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg) {
    SonarImageMsgInterface interface(msg);

    // For now, only postprocess 32bit images
    if (msg->image.dtype != msg->image.DTYPE_UINT32) {
      pub_sonar_image_->publish(*msg);
      return;
    }

    // Validate the data buffer covers ranges*azimuths*4 bytes before the
    // intensity_uint32() reads below (mirror of draw_sonar_component's guard):
    // a short/malformed UINT32 buffer is otherwise read out of bounds.
    {
      const size_t need = static_cast<size_t>(interface.nRanges()) *
                          static_cast<size_t>(interface.nAzimuth()) * 4;
      if (msg->image.data.size() < need) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: %zu data bytes < %zu required",
            msg->image.data.size(), need);
        return;
      }
    }

    // Expect this will copy
    marine_acoustic_msgs::msg::ProjectedSonarImage out = *msg;

    // For now, only 8-bit output is supported
    out.image.dtype = out.image.DTYPE_UINT8;
    out.image.data.clear();
    out.image.data.reserve(interface.ranges().size() *
                           interface.azimuths().size());

    for (unsigned int r_idx = 0; r_idx < interface.nRanges(); ++r_idx) {
      for (unsigned int a_idx = 0; a_idx < interface.nAzimuth(); ++a_idx) {
        sonar_image_proc::AzimuthRangeIndices idx(a_idx, r_idx);

        // Avoid log(0); normalize to [0, 1] in log space
        auto intensity = interface.intensity_uint32(idx);
        auto v = log(std::max((uint)1, intensity)) / log(UINT32_MAX);

        // Apply gain then clamp
        v *= gain_;
        v = std::min(1.0, std::max(0.0, v));

        // Apply gamma correction (0 = disabled)
        if (gamma_ > 0.0) {
          v = pow(v, gamma_);
        }

        out.image.data.push_back(UINT8_MAX * v);
      }
    }

    pub_sonar_image_->publish(out);
  }

}  // namespace sonar_postprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sonar_postprocessor::SonarPostprocessorComponent)
