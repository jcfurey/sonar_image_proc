// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg
// Ported to ROS 2 by GitHub Copilot

#include "sonar_image_proc/draw_sonar_component.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "sonar_image_proc/ColorMaps.h"
#include "sonar_image_proc/CoordinateTable.h"
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

using sonar_image_proc::SonarImageMsgInterface;
using std_msgs::msg::UInt32MultiArray;

using sonar_image_proc::HistogramGenerator;

using sonar_image_proc::InfernoColorMap;
using sonar_image_proc::InfernoSaturationColorMap;
using sonar_image_proc::SonarColorMap;
using SteadyClock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

DrawSonarComponent::DrawSonarComponent(const rclcpp::NodeOptions & options)
      : Node("draw_sonar", options),
        max_range_(0.0),
        color_map_(new InfernoColorMap())  // Set a reasonable default
  {
    // Declare and get parameters
    this->declare_parameter("max_range", 0.0);
    this->declare_parameter("publish_timing", true);
    this->declare_parameter("publish_histogram", false);
    this->declare_parameter("color_map", "inferno");
    this->declare_parameter("range_spacing", 0.0);
    this->declare_parameter("bearing_spacing", 10.0);
    this->declare_parameter("bearing_at_zero", true);
    this->declare_parameter("font_scale", 0.8);
    this->declare_parameter("line_alpha", 0.5);
    this->declare_parameter("line_thickness", 1);
    this->declare_parameter("log_scale", false);
    this->declare_parameter("min_db", -80.0);
    this->declare_parameter("max_db", 0.0);
    // 0 (or negative) selects the native scale: one output pixel per range
    // bin, recomputed per ping. The sonar changes its range resolution with
    // the commanded range, so any fixed value is only correct at one range.
    this->declare_parameter("pixels_per_meter", 0.0);
    // Upper bound on the native scale. 0 = unbounded, which is safe because
    // the native output height is exactly nRanges.
    this->declare_parameter("max_pixels_per_meter", 0.0);
    // Rectangular range-bearing output. A zero height retains the ping's
    // native radial sample density; a zero width derives a 16:9 (configurable)
    // rectilinear raster from that height. Positive dimensions pin the output.
    this->declare_parameter("rectified_width", 0);
    this->declare_parameter("rectified_height", 0);
    this->declare_parameter("rectified_aspect_ratio", 16.0 / 9.0);
    // True virtual-camera projection of an image-derived floor return. TF
    // supplies the platform-up direction through the live pivot head; the
    // coherent return band in each ping supplies the plane standoff. Empty
    // optical frame derives sonar*/optical_frame from the ping's projection
    // frame; operational profiles pin it explicitly.
    this->declare_parameter("floor_projection_sensor_frame", "");
    this->declare_parameter("floor_projection_optical_frame", "");
    this->declare_parameter("floor_projection_reference_frame", "base_link");
    this->declare_parameter("floor_projection_tf_timeout", 0.05);
    this->declare_parameter("floor_projection_tf_max_delta", 0.02);
    this->declare_parameter("floor_detection_min_range", 0.2);
    this->declare_parameter("floor_detection_min_score", 0.05);
    this->declare_parameter("floor_detection_min_support_fraction", 0.40);
    this->declare_parameter("floor_detection_persistence_range", 0.50);
    this->declare_parameter("floor_detection_edge_window_bins", 5);
    // marine_acoustic_msgs is beam-major. The rendering library retains its
    // range-major working image; older recorded images can opt into the
    // compatibility layout through bringup.
    this->declare_parameter("input_image_layout", "beam_major");
    // draw on the GPU (colormap LUT + bicubic fan remap, visually equivalent;
    // see lib/GpuSonarDraw.cu). CPU path is the fallback for non-uint8 pings,
    // non-LUT colormaps, or any device failure.
    this->declare_parameter("use_gpu", false);
    // Compatibility outputs are kept on for one migration window. New
    // consumers use fan_info, rectified_info, and drawn_sonar_polar according
    // to the image geometry they actually consume.
    this->declare_parameter("publish_legacy_camera_info", true);
    this->declare_parameter("publish_legacy_rect_topic", true);

    max_range_ = this->get_parameter("max_range").as_double();
    use_gpu_ = this->get_parameter("use_gpu").as_bool();
    publish_timing_ = this->get_parameter("publish_timing").as_bool();
    publish_histogram_ = this->get_parameter("publish_histogram").as_bool();
    publish_legacy_camera_info_ =
        this->get_parameter("publish_legacy_camera_info").as_bool();
    publish_legacy_rect_topic_ =
        this->get_parameter("publish_legacy_rect_topic").as_bool();
    const int64_t configured_rectified_width =
        this->get_parameter("rectified_width").as_int();
    const int64_t configured_rectified_height =
        this->get_parameter("rectified_height").as_int();
    const double configured_rectified_aspect_ratio =
        this->get_parameter("rectified_aspect_ratio").as_double();
    floor_projection_sensor_frame_ =
        this->get_parameter("floor_projection_sensor_frame").as_string();
    floor_projection_optical_frame_ =
        this->get_parameter("floor_projection_optical_frame").as_string();
    floor_projection_reference_frame_ =
        this->get_parameter("floor_projection_reference_frame").as_string();
    floor_projection_tf_timeout_ =
        this->get_parameter("floor_projection_tf_timeout").as_double();
    floor_projection_tf_max_delta_ =
        this->get_parameter("floor_projection_tf_max_delta").as_double();
    floor_detection_config_.minimum_range = static_cast<float>(
        this->get_parameter("floor_detection_min_range").as_double());
    floor_detection_config_.minimum_score = static_cast<float>(
        this->get_parameter("floor_detection_min_score").as_double());
    floor_detection_config_.minimum_support_fraction = static_cast<float>(
        this->get_parameter("floor_detection_min_support_fraction")
            .as_double());
    floor_detection_config_.persistence_range = static_cast<float>(
        this->get_parameter("floor_detection_persistence_range").as_double());
    floor_detection_config_.edge_window_bins = static_cast<int>(
        this->get_parameter("floor_detection_edge_window_bins").as_int());
    input_image_layout_ =
        this->get_parameter("input_image_layout").as_string();

    constexpr int kMaxRectifiedDimension = 16384;
    if (configured_rectified_width < 0 || configured_rectified_height < 0 ||
        configured_rectified_width == 1 || configured_rectified_height == 1 ||
        configured_rectified_width > kMaxRectifiedDimension ||
        configured_rectified_height > kMaxRectifiedDimension ||
        !std::isfinite(configured_rectified_aspect_ratio) ||
        configured_rectified_aspect_ratio <= 0.0 ||
        configured_rectified_aspect_ratio > 10.0) {
      throw std::invalid_argument(
          "rectified_width/height must be 0 or 2..16384 and "
          "rectified_aspect_ratio must be finite in (0, 10]");
    }
    if (floor_projection_reference_frame_.empty() ||
        !std::isfinite(floor_projection_tf_timeout_) ||
        floor_projection_tf_timeout_ < 0.0 ||
        floor_projection_tf_timeout_ > 1.0 ||
        !std::isfinite(floor_projection_tf_max_delta_) ||
        floor_projection_tf_max_delta_ < 0.0 ||
        floor_projection_tf_max_delta_ > 1.0 ||
        !floor_detection_config_.valid()) {
      throw std::invalid_argument(
          "floor_projection_reference_frame must be nonempty, "
          "floor_projection_tf_timeout and floor_projection_tf_max_delta must "
          "be finite in [0, 1], and floor detection parameters must be finite "
          "and positive");
    }
    rectified_width_ = static_cast<int>(configured_rectified_width);
    rectified_height_ = static_cast<int>(configured_rectified_height);
    rectified_aspect_ratio_ =
        static_cast<float>(configured_rectified_aspect_ratio);

    // Read the log-scale window BEFORE setColorMap: the GPU LUT folds the log
    // transform into its own index, so it has to be built with these known.
    log_scale_ = this->get_parameter("log_scale").as_bool();
    min_db_ = this->get_parameter("min_db").as_double();
    max_db_ = this->get_parameter("max_db").as_double();

    std::string color_map_name = this->get_parameter("color_map").as_string();
    setColorMap(color_map_name);

    // Set the pixels per meter scale factor for the sonar drawer
    float pixels_per_meter = this->get_parameter("pixels_per_meter").as_double();
    float max_pixels_per_meter =
        this->get_parameter("max_pixels_per_meter").as_double();
    sonar_drawer_.setPixelsPerMeter(pixels_per_meter);
    sonar_drawer_.setMaxPixelsPerMeter(max_pixels_per_meter);
    sonar_drawer_.setMaxRange(max_range_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (pixels_per_meter > 0.0f) {
      RCLCPP_INFO(this->get_logger(), "Using fixed pixels_per_meter: %f",
                  pixels_per_meter);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Using native pixels_per_meter (one pixel per range bin)%s",
                  max_pixels_per_meter > 0.0f ? ", capped" : "");
    }

    // Configure sonar drawer overlay
    double range_spacing = this->get_parameter("range_spacing").as_double();
    double bearing_spacing = this->get_parameter("bearing_spacing").as_double();
    bool bearing_at_zero = this->get_parameter("bearing_at_zero").as_bool();
    double font_scale = this->get_parameter("font_scale").as_double();
    double line_alpha = this->get_parameter("line_alpha").as_double();
    int line_thickness = this->get_parameter("line_thickness").as_int();

    sonar_drawer_.overlayConfig()
        .setRangeSpacing(range_spacing)
        .setRadialSpacing(bearing_spacing)
        .setRadialAtZero(bearing_at_zero)
        .setFontScale(font_scale)
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
    // drawn_sonar is the human/operator contract: it is self-describing when
    // viewed in a generic image viewer or exported as a still. Algorithms
    // that must not track the grid subscribe to drawn_sonar_clean instead.
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar", 10);
    clean_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_clean", 10);
    // The fan is an orthographic metric remap, not a pinhole camera. Carry its
    // per-ping pixel geometry in a message with those actual semantics.
    fan_info_pub_ =
        this->create_publisher<sonar_image_proc::msg::FanImageInfo>("fan_info", 10);
    if (publish_legacy_camera_info_) {
      camera_info_pub_ =
          this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
      RCLCPP_WARN(this->get_logger(),
                  "Publishing deprecated non-pinhole camera_info compatibility "
                  "metadata; migrate consumers to fan_info");
    }
    osd_pub_ = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_osd", 10);
    polar_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_polar", 10);
    rectified_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "drawn_sonar_rectified", 10);
    rectified_info_pub_ =
        this->create_publisher<sonar_image_proc::msg::RectifiedImageInfo>(
            "rectified_info", 10);
    floor_projected_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "drawn_sonar_floor_projected", 10);
    floor_projected_info_pub_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "floor_projected_camera_info", 10);
    if (publish_legacy_rect_topic_) {
      rect_pub_ =
          this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_rect", 10);
      RCLCPP_WARN(this->get_logger(),
                  "drawn_sonar_rect is deprecated and was never a rectified "
                  "camera image; use drawn_sonar_polar");
    }

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

    try {
      const std::size_t n_ranges = msg->ranges.size();
      // beam_directions is what every consumer downstream indexes by
      // (interface nBearings(), the GPU stride); image.beam_count is the
      // producer's claim about the payload stride. A mismatch means the decode
      // below would silently skew the fan — drop instead. beam_count 0
      // (pre-a355c65 bags never populated it) defers to beam_directions.
      const std::size_t n_bearings = msg->beam_directions.size();
      if (msg->image.beam_count != 0 && msg->image.beam_count != n_bearings) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: image.beam_count %u != %zu beam_directions",
            msg->image.beam_count, n_bearings);
        return;
      }
      std::size_t elem = 0;
      if (msg->image.dtype == msg->image.DTYPE_UINT8) elem = 1;
      else if (msg->image.dtype == msg->image.DTYPE_UINT16) elem = 2;
      else if (msg->image.dtype == msg->image.DTYPE_UINT32) elem = 4;
      if (elem == 0) {
        // FLOAT32 (in the message contract, unsupported here) or garbage: the
        // interface reads every sample as 0, so this previously published an
        // all-black fan with no diagnostic on the range_major path.
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: unsupported image dtype %u",
            msg->image.dtype);
        return;
      }

      // Keep the size arithmetic and the legacy int-valued drawing API in
      // range before either a layout conversion or renderer sees the payload.
      // In particular, a wrapped product could otherwise make a malformed
      // range-major image appear large enough for the GPU copy below.
      if (n_ranges < 2 || n_bearings < 2 ||
          n_ranges > static_cast<std::size_t>(
                         std::numeric_limits<int>::max()) ||
          n_bearings > static_cast<std::size_t>(
                           std::numeric_limits<int>::max()) ||
          n_ranges > std::numeric_limits<std::size_t>::max() / n_bearings) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: invalid dimensions (%zu ranges x %zu "
            "bearings)",
            n_ranges, n_bearings);
        return;
      }
      const std::size_t cell_count = n_ranges * n_bearings;
      if (cell_count > std::numeric_limits<std::size_t>::max() / elem) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: byte count overflows for %zu ranges x "
            "%zu bearings x %zu bytes",
            n_ranges, n_bearings, elem);
        return;
      }
      const std::size_t required_bytes = cell_count * elem;
      if (msg->image.data.size() < required_bytes) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: %zu data bytes < %zu required "
            "(%zu ranges x %zu bearings)",
            msg->image.data.size(), required_bytes, n_ranges, n_bearings);
        return;
      }

      // Reject malformed coordinate tables before bounds calculation or the
      // fan remap's binary search.  A range table has one physical direction;
      // beam bearings may be ascending or descending, but must not reverse or
      // repeat within a ping.
      bool ranges_ascending = false;
      bool bearings_ascending = false;
      SonarImageMsgInterface input_geometry(msg);
      bool directions_valid = true;
      for (const auto &direction : msg->beam_directions) {
        const double norm_squared = direction.x * direction.x +
                                    direction.y * direction.y +
                                    direction.z * direction.z;
        if (!std::isfinite(norm_squared) || norm_squared <= 1e-12) {
          directions_valid = false;
          break;
        }
      }
      if (!directions_valid ||
          !sonar_image_proc::validateCoordinateTable(msg->ranges,
                                                     ranges_ascending) ||
          !ranges_ascending || msg->ranges.front() <= 0.0f ||
          !sonar_image_proc::validateCoordinateTable(input_geometry.azimuths(),
                                                     bearings_ascending)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Dropping sonar image: ranges must be finite, positive and "
            "strictly increasing; beam directions must produce one finite, "
            "strictly monotonic bearing table");
        return;
      }

      auto working_msg = msg;
      if (input_image_layout_ == "beam_major") {
        // Copy the metadata only: the copy constructor also duplicated
        // image.data (~0.5 MB per ping at 493x512x2) just for the transpose
        // below to overwrite it.
        auto range_major =
            std::make_shared<marine_acoustic_msgs::msg::ProjectedSonarImage>();
        range_major->header = msg->header;
        range_major->ping_info = msg->ping_info;
        range_major->beam_directions = msg->beam_directions;
        range_major->ranges = msg->ranges;
        range_major->image.is_bigendian = msg->image.is_bigendian;
        range_major->image.dtype = msg->image.dtype;
        range_major->image.beam_count = msg->image.beam_count;
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
      if (log_scale_) {
        interface.do_log_scale(min_db_, max_db_);
      }

      Seconds rect_elapsed = Seconds::zero();
      Seconds rectified_elapsed = Seconds::zero();
      Seconds floor_projected_elapsed = Seconds::zero();
      Seconds map_elapsed = Seconds::zero();
      Seconds histogram_elapsed = Seconds::zero();

      if (publish_histogram_) {
        auto begin = SteadyClock::now();

        auto histogram_out = UInt32MultiArray();
        histogram_out.data = HistogramGenerator::Generate(interface);

        histogram_pub_->publish(histogram_out);

        histogram_elapsed = SteadyClock::now() - begin;
      }

      {
        auto begin = SteadyClock::now();

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
          const float draw_ppm =
              sonar_drawer_.effectivePixelsPerMeter(interface);
          const auto geom = sonar_image_proc::gpu::fanGeometry(
              display_max_range, az.first, az.second, draw_ppm);
          if (n_ranges > 0 && n_bearings > 0 && geom.width > 0 &&
              geom.height > 0) {
            rect_mat.create(cv::Size(n_ranges, n_bearings), CV_8UC3);
            sonar_mat.create(cv::Size(geom.width, geom.height), CV_8UC3);
            gpu_drawn = sonar_image_proc::gpu::drawSonar(
                working_msg->image.data.data(), n_ranges, n_bearings,
                interface.ranges().data(), interface.azimuths().data(), draw_ppm,
                lut_.data(), rect_mat.data, geom, sonar_mat.data);
          }
        }
#endif
        if (!gpu_drawn)
          rect_mat = sonar_drawer_.drawRectSonarImage(interface, *color_map_);

        // Rotate the polar range x bearing image to the more expected format
        // where zero range
        // is at the bottom of the image, with negative azimuth to the right
        // aka (rotated 90 degrees CCW). Subscriber-gated like the OSD: both names
        // are inspection outputs, and nothing deployed consumes either one.
        const bool polar_wanted = polar_pub_->get_subscription_count() > 0;
        const bool legacy_rect_wanted =
            rect_pub_ && rect_pub_->get_subscription_count() > 0;
        if (polar_wanted || legacy_rect_wanted) {
          cv::Mat rotated_rect;
          cv::rotate(rect_mat, rotated_rect, cv::ROTATE_90_COUNTERCLOCKWISE);
          if (polar_wanted)
            cvBridgeAndPublish(working_msg, rotated_rect, polar_pub_);
          if (legacy_rect_wanted)
            cvBridgeAndPublish(working_msg, rotated_rect, rect_pub_);
        }

        rect_elapsed = SteadyClock::now() - begin;
        begin = SteadyClock::now();

        publishRectifiedProducts(working_msg, interface, rect_mat);

        rectified_elapsed = SteadyClock::now() - begin;
        begin = SteadyClock::now();

        publishFloorProjectedProducts(working_msg, interface, rect_mat);

        floor_projected_elapsed = SteadyClock::now() - begin;
        begin = SteadyClock::now();

        if (!gpu_drawn)
          sonar_mat = sonar_drawer_.remapRectSonarImage(interface, rect_mat);

        // Same stamp and frame as the image it describes. Consumers synchronize
        // fan_info and drawn_sonar_clean exactly by stamp; publishing first also
        // keeps compatibility with the legacy latch-based CameraInfo consumer.
        {
          const auto geom = sonar_drawer_.fanImageGeometry(interface);
          const auto azimuth = interface.azimuthBounds();
          sonar_image_proc::msg::FanImageInfo fan_info;
          fan_info.header = working_msg->header;
          fan_info.width = geom.width;
          fan_info.height = geom.height;
          fan_info.origin_u = geom.origin_x;
          fan_info.origin_v = geom.height;
          fan_info.pixels_per_meter = geom.pixels_per_meter;
          fan_info.min_range = interface.minRange();
          fan_info.max_range = sonar_drawer_.effectiveMaxRange(interface);
          fan_info.min_bearing = azimuth.first;
          fan_info.max_bearing = azimuth.second;
          fan_info_pub_->publish(fan_info);

          if (camera_info_pub_) {
            sensor_msgs::msg::CameraInfo info;
            info.header = working_msg->header;
            info.width = geom.width;
            info.height = geom.height;
            info.distortion_model = "";
            info.d.clear();
            info.k = {geom.pixels_per_meter, 0.0,
                      static_cast<double>(geom.origin_x), 0.0,
                      geom.pixels_per_meter, static_cast<double>(geom.height),
                      0.0, 0.0, 1.0};
            info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            info.p = {geom.pixels_per_meter, 0.0,
                      static_cast<double>(geom.origin_x), 0.0, 0.0,
                      geom.pixels_per_meter, static_cast<double>(geom.height),
                      0.0, 0.0, 0.0, 1.0, 0.0};
            camera_info_pub_->publish(info);
          }
        }
        if (clean_pub_->get_subscription_count() > 0)
          cvBridgeAndPublish(working_msg, sonar_mat, clean_pub_);

        const cv::Mat annotated_mat =
            sonar_drawer_.drawOverlay(interface, sonar_mat);
        cvBridgeAndPublish(working_msg, annotated_mat, pub_);

        // Compatibility alias for existing dashboards. New consumers should
        // use drawn_sonar (operator) or drawn_sonar_clean (machine vision).
        if (osd_pub_->get_subscription_count() > 0)
          cvBridgeAndPublish(working_msg, annotated_mat, osd_pub_);

        map_elapsed = SteadyClock::now() - begin;
      }

      if (publish_timing_) {
        std::ostringstream output;

        output << "{";
        output << "\"draw_total\" : "
               << (map_elapsed + rect_elapsed + rectified_elapsed +
                   floor_projected_elapsed)
                      .count();
        output << ", \"rect\" : " << rect_elapsed.count();
        output << ", \"rectified\" : " << rectified_elapsed.count();
        output << ", \"floor_projected\" : "
               << floor_projected_elapsed.count();
        output << ", \"map\" : " << map_elapsed.count();

        if (publish_histogram_)
          output << ", \"histogram\" : " << histogram_elapsed.count();

        output << "}";

        auto out_msg = std_msgs::msg::String();
        out_msg.data = output.str();

        timing_pub_->publish(out_msg);
      }
    } catch (const cv::Exception &error) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Dropping sonar image after OpenCV error: %s",
                            error.what());
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Dropping sonar image after processing error: %s",
                            error.what());
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
    // The GPU path indexes this LUT with the raw uint8 sample and never calls
    // the interface accessors, so log scaling has to be baked in here or it is
    // silently ignored on that path. Folding it into the index is exact,
    // because the transform is a monotonic function of the sample value alone.
    // Mirrors intensity_float_log() for 8-bit data: one LSB is 1/255, so the
    // full-scale bottom is 10*log10(1/255) = -24.07 dB.
    // Full-precision fraction, split from the quantized index: the Mitchell
    // LUT below is computed from the FRACTION on the CPU path
    // (intensity_float()), so building it from the 8-bit-quantized index here
    // diverged by up to one count per channel under log scaling.
    const auto log_fraction = [this](int i) -> float {
      if (!log_scale_) return static_cast<float>(i) / UINT8_MAX;
      constexpr float kLsb = 1.0f / UINT8_MAX;
      const float norm = std::max(static_cast<float>(i) / UINT8_MAX, kLsb);
      const float v = std::log10(norm) * 10.0f;
      const float full_min_db = std::log10(kLsb) * 10.0f;
      const float min_db = (min_db_ == 0 ? full_min_db : min_db_);
      // Span of the EFFECTIVE window. Computing it from the raw min_db_ made
      // min_db 0 (auto) with a nonzero max_db yield a NEGATIVE span, which
      // clamped the whole fan to black. A non-positive effective span
      // (misconfigured window) falls back to the full scale.
      float span = max_db_ - min_db;
      if (!(span > 0.0f)) span = -full_min_db;
      return std::min(1.0f, std::max(0.0f, (v - min_db) / span));
    };
    const auto log_index = [&log_fraction](int i) -> int {
      return static_cast<int>(log_fraction(i) * UINT8_MAX);
    };

    lut_valid_ = false;
    if (color_map_name == "mitchell") {
      for (int i = 0; i < 256; ++i) {
        const float f = log_fraction(i);
        lut_[3 * i + 0] = static_cast<uint8_t>((1.0f - f) * 255.0f);
        lut_[3 * i + 1] = static_cast<uint8_t>(f * 255.0f);
        lut_[3 * i + 2] = static_cast<uint8_t>(f * 255.0f);
      }
      lut_valid_ = true;
    } else {  // inferno / inferno_saturation
      for (int i = 0; i < 256; ++i)
        for (int c = 0; c < 3; ++c)
          lut_[3 * i + c] = cv::saturate_cast<uint8_t>(
              InfernoColorMap::_inferno_data_uint8[log_index(i)][c]);
      if (color_map_name == "inferno_saturation") {
        // The CPU keys the green marker off the POST-log intensity
        // (ColorMaps.h: intensity_uint8() == 255), so every raw value the log
        // window saturates must be green here too — overriding only raw 255
        // diverged for any max_db below 0.
        for (int i = 0; i < 256; ++i) {
          if (log_index(i) == UINT8_MAX) {
            lut_[3 * i + 0] = 0;
            lut_[3 * i + 1] = 255;
            lut_[3 * i + 2] = 0;
          }
        }
      }
      lut_valid_ = true;
    }
  }

}  // namespace draw_sonar

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(draw_sonar::DrawSonarComponent)
