// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2

#include "sonar_image_proc/sonar_postprocessor_component.hpp"

#include "sonar_image_proc/ImageLayout.h"
#include "sonar_image_proc/sonar_image_msg_interface.h"

namespace sonar_postprocessor {

using marine_acoustic_msgs::msg::ProjectedSonarImage;
using sonar_image_proc::SonarImageMsgInterface;

SonarPostprocessorComponent::SonarPostprocessorComponent(const rclcpp::NodeOptions & options)
      : Node("sonar_postprocessor", options) {
    // Declare and get parameters
    this->declare_parameter("gain", 1.0);
    this->declare_parameter("gamma", 0.0);
    this->declare_parameter("input_image_layout", "beam_major");

    gain_ = this->get_parameter("gain").as_double();
    gamma_ = this->get_parameter("gamma").as_double();
    input_image_layout_ =
        this->get_parameter("input_image_layout").as_string();
    if (input_image_layout_ != "beam_major" &&
        input_image_layout_ != "range_major") {
      throw std::invalid_argument(
          "input_image_layout must be beam_major or range_major");
    }

    sub_sonar_image_ = this->create_subscription<ProjectedSonarImage>(
        "sonar_image", rclcpp::SensorDataQoS(),
        std::bind(&SonarPostprocessorComponent::sonarImageCallback,
                  this, std::placeholders::_1));

    pub_sonar_image_ = this->create_publisher<ProjectedSonarImage>(
        "sonar_image_postproc", 10);

    RCLCPP_DEBUG(this->get_logger(), "sonar_processor ready to run...");
  }

  void SonarPostprocessorComponent::sonarImageCallback(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg) {
    const std::size_t n_ranges = msg->ranges.size();
    const std::size_t n_beams = msg->beam_directions.size();
    if (msg->image.beam_count != 0 && msg->image.beam_count != n_beams) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Dropping sonar image: image.beam_count %u != %zu beam_directions",
          msg->image.beam_count, n_beams);
      return;
    }
    std::size_t bytes_per_cell = 0;
    if (msg->image.dtype == msg->image.DTYPE_UINT8) bytes_per_cell = 1;
    else if (msg->image.dtype == msg->image.DTYPE_UINT16) bytes_per_cell = 2;
    else if (msg->image.dtype == msg->image.DTYPE_UINT32) bytes_per_cell = 4;
    if (n_ranges == 0 || n_beams == 0 || bytes_per_cell == 0 ||
        n_ranges > msg->image.data.max_size() / n_beams ||
        n_ranges * n_beams >
            msg->image.data.max_size() / bytes_per_cell ||
        msg->image.data.size() != n_ranges * n_beams * bytes_per_cell) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Dropping malformed or unsupported sonar image (%zu ranges x %zu "
          "beams, dtype %u, %zu bytes)",
          n_ranges, n_beams, msg->image.dtype, msg->image.data.size());
      return;
    }

    // The inherited message interface is range-major. Canonicalize into that
    // private working order once, then always publish the standards-facing
    // output as beam-major.
    auto working = std::make_shared<ProjectedSonarImage>(*msg);
    working->image.beam_count = static_cast<std::uint32_t>(n_beams);
    if (input_image_layout_ == "beam_major" &&
        !sonar_image_proc::beamMajorToRangeMajor(
            msg->image.data, n_ranges, n_beams, bytes_per_cell,
            working->image.data)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not canonicalize beam-major sonar image");
      return;
    }
    SonarImageMsgInterface interface(working);

    // Expect this will copy
    marine_acoustic_msgs::msg::ProjectedSonarImage out = *msg;
    out.image.beam_count = static_cast<std::uint32_t>(n_beams);

    // Only UINT32 needs radiometric conversion. Still canonicalize every other
    // supported input so the output topic has one unambiguous wire contract.
    if (msg->image.dtype != msg->image.DTYPE_UINT32) {
      if (!sonar_image_proc::rangeMajorToBeamMajor(
              working->image.data, n_ranges, n_beams, bytes_per_cell,
              out.image.data)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Could not publish beam-major sonar image");
        return;
      }
      pub_sonar_image_->publish(out);
      return;
    }

    // For now, only 8-bit output is supported
    out.image.dtype = out.image.DTYPE_UINT8;
    out.image.is_bigendian = false;
    std::vector<std::uint8_t> range_major_output;
    range_major_output.reserve(interface.ranges().size() *
                               interface.azimuths().size());

    for (int r_idx = 0; r_idx < interface.nRanges(); ++r_idx) {
      for (int a_idx = 0; a_idx < interface.nAzimuth(); ++a_idx) {
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

        range_major_output.push_back(UINT8_MAX * v);
      }
    }

    if (!sonar_image_proc::rangeMajorToBeamMajor(
            range_major_output, n_ranges, n_beams, 1, out.image.data)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not publish processed beam-major sonar image");
      return;
    }

    pub_sonar_image_->publish(out);
  }

}  // namespace sonar_postprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sonar_postprocessor::SonarPostprocessorComponent)
