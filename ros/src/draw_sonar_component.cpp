// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2 by GitHub Copilot

#include "sonar_image_proc/draw_sonar_component.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <chrono>

#include <cv_bridge/cv_bridge.hpp>

#include "sonar_image_proc/ColorMaps.h"
#include "sonar_image_proc/DrawSonar.h"
#include "sonar_image_proc/HistogramGenerator.h"
#include "sonar_image_proc/SonarDrawer.h"
#include "sonar_image_proc/sonar_image_msg_interface.h"

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

    max_range_ = this->get_parameter("max_range").as_double();
    publish_old_api_ = this->get_parameter("publish_old").as_bool();
    publish_timing_ = this->get_parameter("publish_timing").as_bool();
    publish_histogram_ = this->get_parameter("publish_histogram").as_bool();
    
    std::string color_map_name = this->get_parameter("color_map").as_string();
    setColorMap(color_map_name);

    log_scale_ = this->get_parameter("log_scale").as_bool();
    min_db_ = this->get_parameter("min_db").as_double();
    max_db_ = this->get_parameter("max_db").as_double();

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

    // Create subscription
    sub_sonar_image_ = this->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>(
        "sonar_image", 10,
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

    SonarImageMsgInterface interface(msg);
    if (log_scale_) {
      interface.do_log_scale(min_db_, max_db_);
    }

    std::chrono::duration<double> old_api_elapsed, rect_elapsed, map_elapsed, histogram_elapsed;

    if (publish_old_api_) {
      auto begin = std::chrono::steady_clock::now();

      // Used to be a configurable parameter, but now only meaningful
      // in the deprecated API
      const int pix_per_range_bin = 2;

      cv::Size sz = sonar_image_proc::old_api::calculateImageSize(
          interface, cv::Size(0, 0), pix_per_range_bin, max_range_);
      cv::Mat mat(sz, CV_8UC3);
      mat = sonar_image_proc::old_api::drawSonar(interface, mat, *color_map_,
                                                 max_range_);

      cvBridgeAndPublish(msg, mat, old_pub_);

      old_api_elapsed = std::chrono::steady_clock::now() - begin;
    }

    if (publish_histogram_) {
      auto begin = std::chrono::steady_clock::now();

      auto histogram_out = UInt32MultiArray();
      histogram_out.data = HistogramGenerator::Generate(interface);

      histogram_pub_->publish(histogram_out);

      histogram_elapsed = std::chrono::steady_clock::now() - begin;
    }

    {
      auto begin = std::chrono::steady_clock::now();

      cv::Mat rect_mat = sonar_drawer_.drawRectSonarImage(interface, *color_map_);

      // Rotate rectangular image to the more expected format where zero range
      // is at the bottom of the image, with negative azimuth to the right
      // aka (rotated 90 degrees CCW)
      cv::Mat rotated_rect;
      cv::rotate(rect_mat, rotated_rect, cv::ROTATE_90_COUNTERCLOCKWISE);
      cvBridgeAndPublish(msg, rotated_rect, rect_pub_);

      rect_elapsed = std::chrono::steady_clock::now() - begin;
      begin = std::chrono::steady_clock::now();

      cv::Mat sonar_mat = sonar_drawer_.remapRectSonarImage(interface, rect_mat);
      cvBridgeAndPublish(msg, sonar_mat, pub_);

      if (osd_pub_->get_subscription_count() > 0) {
        cv::Mat osd_mat = sonar_drawer_.drawOverlay(interface, sonar_mat);
        cvBridgeAndPublish(msg, osd_mat, osd_pub_);
      }

      map_elapsed = std::chrono::steady_clock::now() - begin;
    }

    if (publish_timing_) {
      ostringstream output;

      output << "{";
      output << "\"draw_total\" : " << (map_elapsed + rect_elapsed).count();
      output << ", \"rect\" : " << rect_elapsed.count();
      output << ", \"map\" : " << map_elapsed.count();

      if (publish_old_api_) output << ", \"old_api\" : " << old_api_elapsed.count();
      if (publish_histogram_)
        output << ", \"histogram\" : " << histogram_elapsed.count();

      output << "}";

      auto out_msg = std_msgs::msg::String();
      out_msg.data = output.str();

      timing_pub_->publish(out_msg);
    }
  }

  void DrawSonarComponent::setColorMap(const std::string &color_map_name) {
    // TBD actually implement the parameter processing here...
    color_map_.reset(new InfernoSaturationColorMap());
  }

}  // namespace draw_sonar

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(draw_sonar::DrawSonarComponent)
