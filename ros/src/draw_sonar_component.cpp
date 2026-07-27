// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2 by GitHub Copilot

#include "sonar_image_proc/draw_sonar_component.hpp"

#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include <cv_bridge/cv_bridge.hpp>

#include "sonar_image_proc/ColorMaps.h"
#include "sonar_image_proc/DrawSonar.h"
#include "sonar_image_proc/HistogramGenerator.h"
#include "sonar_image_proc/ImageLayout.h"
#include "sonar_image_proc/SonarDrawer.h"
#include "sonar_image_proc/sonar_image_msg_interface.h"
#ifdef SONAR_IMAGE_PROC_WITH_CUDA
#include "sonar_image_proc/GpuSonarDraw.h"
#endif

// Subscribes to sonar message topic, draws using opencv then publishes result

namespace draw_sonar {

using namespace std;
using namespace cv;

using sonar_image_proc::SonarImageMsgInterface;
using std_msgs::msg::UInt32MultiArray;

using sonar_image_proc::HistogramGenerator;

using sonar_image_proc::InfernoColorMap;
using sonar_image_proc::InfernoSaturationColorMap;
using sonar_image_proc::SonarColorMap;

DrawSonarComponent::DrawSonarComponent(const rclcpp::NodeOptions & options)
      : Node("draw_sonar", options),
        max_range_(0.0),
        color_map_(new InfernoColorMap())  // Set a reasonable default
  {
    // Declare and get parameters
    this->declare_parameter("max_range", 0.0);
    this->declare_parameter("publish_old", false);
    this->declare_parameter("publish_timing", true);
    this->declare_parameter("publish_histogram", false);
    this->declare_parameter("color_map", "inferno");
    this->declare_parameter("range_spacing", 10.0);
    this->declare_parameter("bearing_spacing", 10.0);
    this->declare_parameter("line_alpha", 0.5);
    this->declare_parameter("line_thickness", 1);
    this->declare_parameter("log_scale", false);
    this->declare_parameter("min_db", -80.0);
    this->declare_parameter("max_db", 0.0);
    this->declare_parameter("pixels_per_meter", 100.0);
    // marine_acoustic_msgs is beam-major. The rendering library retains its
    // range-major working image; older recorded images can opt into the
    // compatibility layout through bringup.
    this->declare_parameter("input_image_layout", "beam_major");
    // draw on the GPU (colormap LUT + bicubic fan remap, visually equivalent;
    // see lib/GpuSonarDraw.cu). CPU path is the fallback for non-uint8 pings,
    // non-LUT colormaps, or any device failure.
    this->declare_parameter("use_gpu", false);

    max_range_ = this->get_parameter("max_range").as_double();
    use_gpu_ = this->get_parameter("use_gpu").as_bool();
    publish_old_api_ = this->get_parameter("publish_old").as_bool();
    publish_timing_ = this->get_parameter("publish_timing").as_bool();
    publish_histogram_ = this->get_parameter("publish_histogram").as_bool();
    input_image_layout_ =
        this->get_parameter("input_image_layout").as_string();

    std::string color_map_name = this->get_parameter("color_map").as_string();
    setColorMap(color_map_name);

    log_scale_ = this->get_parameter("log_scale").as_bool();
    min_db_ = this->get_parameter("min_db").as_double();
    max_db_ = this->get_parameter("max_db").as_double();

    // Set the pixels per meter scale factor for the sonar drawer
    float pixels_per_meter = this->get_parameter("pixels_per_meter").as_double();
    sonar_drawer_.setPixelsPerMeter(pixels_per_meter);
    sonar_drawer_.setMaxRange(max_range_);
    RCLCPP_INFO(this->get_logger(), "Using pixels_per_meter: %f", pixels_per_meter);

    // Configure sonar drawer overlay
    double range_spacing = this->get_parameter("range_spacing").as_double();
    double bearing_spacing = this->get_parameter("bearing_spacing").as_double();
    double line_alpha = this->get_parameter("line_alpha").as_double();
    int line_thickness = this->get_parameter("line_thickness").as_int();

    sonar_drawer_.overlayConfig()
        .setRangeSpacing(range_spacing)
        .setRadialSpacing(bearing_spacing)
        .setLineAlpha(line_alpha)
        .setLineThickness(line_thickness);

    if (max_range_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Only drawing to max range %f", max_range_);
    }

    // Create subscription. Sonar image publishers (OceanSim, the real Oculus
    // driver) use SensorDataQoS (best-effort); the plain-integer-depth overload
    // used here previously built rclcpp's default QoS (reliable), which a
    // best-effort publisher can never satisfy -- the subscription silently
    // never connected. Match sonar_proc's SonarProcessingNode, which already
    // subscribes to the same message type with SensorDataQoS() correctly.
    sub_sonar_image_ = this->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>(
        "sonar_image", rclcpp::SensorDataQoS(),
        std::bind(&DrawSonarComponent::sonarImageCallback, this, std::placeholders::_1));

    // Create publishers
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar", 10);
    osd_pub_ = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_osd", 10);
    rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_rect", 10);

    if (publish_old_api_)
      old_pub_ = this->create_publisher<sensor_msgs::msg::Image>("old_drawn_sonar", 10);

    if (publish_timing_)
      timing_pub_ = this->create_publisher<std_msgs::msg::String>("sonar_image_proc_timing", 10);

    if (publish_histogram_)
      histogram_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("histogram", 10);

    RCLCPP_DEBUG(this->get_logger(), "draw_sonar ready to run...");
  }

  void DrawSonarComponent::cvBridgeAndPublish(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
      const cv::Mat &mat,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub) {
    cv_bridge::CvImage img_bridge(msg->header, "rgb8", mat);

    auto output_msg = img_bridge.toImageMsg();
    pub->publish(*output_msg);
  }

  void DrawSonarComponent::sonarImageCallback(
      const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg) {
    if (!color_map_) {
      RCLCPP_FATAL(this->get_logger(), "Colormap is undefined, this shouldn't happen");
      return;
    }

    const std::size_t n_ranges = msg->ranges.size();
    const std::size_t n_bearings = msg->image.beam_count;
    std::size_t elem = 0;
    if (msg->image.dtype == msg->image.DTYPE_UINT8) elem = 1;
    else if (msg->image.dtype == msg->image.DTYPE_UINT16) elem = 2;
    else if (msg->image.dtype == msg->image.DTYPE_UINT32) elem = 4;

    auto working_msg = msg;
    if (input_image_layout_ == "beam_major") {
      auto range_major =
          std::make_shared<marine_acoustic_msgs::msg::ProjectedSonarImage>(*msg);
      if (!sonar_image_proc::beamMajorToRangeMajor(
              msg->image.data, n_ranges, n_bearings, elem,
              range_major->image.data)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: could not decode beam-major payload");
        return;
      }
      working_msg = std::move(range_major);
    } else if (input_image_layout_ != "range_major") {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Unknown input_image_layout '%s' (expected beam_major or range_major)",
          input_image_layout_.c_str());
      return;
    }

    SonarImageMsgInterface interface(working_msg);
    if (interface.nRanges() < 2 || interface.nBearings() < 2) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Dropping sonar image: rendering requires at least 2 ranges and "
          "2 bearings (got %d ranges x %d bearings)",
          interface.nRanges(), interface.nBearings());
      return;
    }
    // A data buffer shorter than ranges*bearings*elem was read out of bounds
    // by every consumer below (the CPU index() lookups and the GPU H2D copy
    // alike) — validate once, before any path touches it.
    {
      const size_t need = static_cast<size_t>(interface.nRanges()) *
                          static_cast<size_t>(interface.nBearings()) * elem;
      if (elem > 0 && working_msg->image.data.size() < need) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: %zu data bytes < %zu required "
            "(%d ranges x %d bearings)",
            working_msg->image.data.size(), need, interface.nRanges(),
            interface.nBearings());
        return;
      }
    }
    if (log_scale_) {
      interface.do_log_scale(min_db_, max_db_);
    }

    rclcpp::Duration old_api_elapsed(0, 0), rect_elapsed(0, 0);
    rclcpp::Duration map_elapsed(0, 0), histogram_elapsed(0, 0);

    if (publish_old_api_) {
      auto begin = this->get_clock()->now();

      // Used to be a configurable parameter, but now only meaningful
      // in the deprecated API
      const int pix_per_range_bin = 2;

      cv::Size sz = sonar_image_proc::old_api::calculateImageSize(
          interface, cv::Size(0, 0), pix_per_range_bin, max_range_);
      cv::Mat mat(sz, CV_8UC3);
      mat = sonar_image_proc::old_api::drawSonar(interface, mat, *color_map_,
                                                 max_range_);

      cvBridgeAndPublish(working_msg, mat, old_pub_);

      old_api_elapsed = this->get_clock()->now() - begin;
    }

    if (publish_histogram_) {
      auto begin = this->get_clock()->now();

      auto histogram_out = UInt32MultiArray();
      histogram_out.data = HistogramGenerator::Generate(interface);

      histogram_pub_->publish(histogram_out);

      histogram_elapsed = this->get_clock()->now() - begin;
    }

    {
      auto begin = this->get_clock()->now();

      cv::Mat rect_mat;
      cv::Mat sonar_mat;
      bool gpu_drawn = false;
#ifdef SONAR_IMAGE_PROC_WITH_CUDA
      // GPU draw: one shot produces both the rect image and the fan (see
      // GpuSonarDraw.h — LUT stage exact, remap visually equivalent). Any
      // failure or unsupported input falls through to the CPU path below.
      if (use_gpu_ && lut_valid_ &&
          working_msg->image.dtype ==
              marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8 &&
          sonar_image_proc::gpu::available()) {
        const int n_ranges = interface.nRanges();
        const int n_bearings = interface.nBearings();
        const auto az = interface.azimuthBounds();
        const float display_max_range =
            sonar_drawer_.effectiveMaxRange(interface);
        const auto geom = sonar_image_proc::gpu::fanGeometry(
            display_max_range, az.first, az.second,
            sonar_drawer_.pixelsPerMeter());
        if (n_ranges > 0 && n_bearings > 0 && geom.width > 0 &&
            geom.height > 0) {
          rect_mat.create(cv::Size(n_ranges, n_bearings), CV_8UC3);
          sonar_mat.create(cv::Size(geom.width, geom.height), CV_8UC3);
          gpu_drawn = sonar_image_proc::gpu::drawSonar(
              working_msg->image.data.data(), n_ranges, n_bearings,
              interface.minRange(), interface.maxRange(),
              interface.azimuths().data(), sonar_drawer_.pixelsPerMeter(),
              lut_.data(), rect_mat.data, geom, sonar_mat.data);
        }
      }
#endif
      if (!gpu_drawn)
        rect_mat = sonar_drawer_.drawRectSonarImage(interface, *color_map_);

      // Rotate rectangular image to the more expected format where zero range
      // is at the bottom of the image, with negative azimuth to the right
      // aka (rotated 90 degrees CCW)
      cv::Mat rotated_rect;
      cv::rotate(rect_mat, rotated_rect, cv::ROTATE_90_COUNTERCLOCKWISE);
      cvBridgeAndPublish(working_msg, rotated_rect, rect_pub_);

      rect_elapsed = this->get_clock()->now() - begin;
      begin = this->get_clock()->now();

      if (!gpu_drawn)
        sonar_mat = sonar_drawer_.remapRectSonarImage(interface, rect_mat);
      cvBridgeAndPublish(working_msg, sonar_mat, pub_);

      if (osd_pub_->get_subscription_count() > 0) {
        cv::Mat osd_mat = sonar_drawer_.drawOverlay(interface, sonar_mat);
        cvBridgeAndPublish(working_msg, osd_mat, osd_pub_);
      }

      map_elapsed = this->get_clock()->now() - begin;
    }

    if (publish_timing_) {
      ostringstream output;

      output << "{";
      output << "\"draw_total\" : " << (map_elapsed + rect_elapsed).seconds();
      output << ", \"rect\" : " << rect_elapsed.seconds();
      output << ", \"map\" : " << map_elapsed.seconds();

      if (publish_old_api_) output << ", \"old_api\" : " << old_api_elapsed.seconds();
      if (publish_histogram_)
        output << ", \"histogram\" : " << histogram_elapsed.seconds();

      output << "}";

      auto out_msg = std_msgs::msg::String();
      out_msg.data = output.str();

      timing_pub_->publish(out_msg);
    }
  }

  void DrawSonarComponent::setColorMap(const std::string &color_map_name) {
    if (color_map_name == "inferno_saturation") {
      color_map_.reset(new InfernoSaturationColorMap());
    } else if (color_map_name == "mitchell") {
      color_map_.reset(new sonar_image_proc::MitchellColorMap());
    } else {
      if (color_map_name != "inferno") {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown color_map '%s', using 'inferno'",
                    color_map_name.c_str());
      }
      color_map_.reset(new InfernoColorMap());
    }

    // GPU LUT: the active colormap evaluated per uint8 intensity, matching
    // the CPU lookup_cv8uc3 arithmetic (truncating float->uchar conversion,
    // NOT saturate_cast rounding — the CPU maps convert implicitly)
    lut_valid_ = false;
    if (color_map_name == "mitchell") {
      for (int i = 0; i < 256; ++i) {
        const float f = static_cast<float>(i) / UINT8_MAX;
        lut_[3 * i + 0] = static_cast<uchar>((1.0f - f) * 255.0f);
        lut_[3 * i + 1] = static_cast<uchar>(f * 255.0f);
        lut_[3 * i + 2] = static_cast<uchar>(f * 255.0f);
      }
      lut_valid_ = true;
    } else {  // inferno / inferno_saturation
      for (int i = 0; i < 256; ++i)
        for (int c = 0; c < 3; ++c)
          lut_[3 * i + c] = cv::saturate_cast<uchar>(
              InfernoColorMap::_inferno_data_uint8[i][c]);
      if (color_map_name == "inferno_saturation") {
        lut_[3 * 255 + 0] = 0;
        lut_[3 * 255 + 1] = 255;
        lut_[3 * 255 + 2] = 0;
      }
      lut_valid_ = true;
    }
  }

}  // namespace draw_sonar

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(draw_sonar::DrawSonarComponent)
