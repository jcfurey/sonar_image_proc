// Copyright 2026 ERDC

#include "sonar_image_proc/draw_sonar_component.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <opencv2/core/core.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.hpp>

namespace draw_sonar {

namespace {

bool rigidTransformFromMsg(
    const geometry_msgs::msg::Transform &message,
    sonar_image_proc::SonarDrawer::RigidTransform &output) {
  tf2::Quaternion quaternion(message.rotation.x, message.rotation.y,
                             message.rotation.z, message.rotation.w);
  if (!std::isfinite(quaternion.length2()) || quaternion.length2() <= 1e-12)
    return false;
  quaternion.normalize();
  const tf2::Matrix3x3 rotation(quaternion);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column)
      output.rotation(row, column) = rotation[row][column];
  }
  output.translation =
      cv::Vec3f(message.translation.x, message.translation.y,
                message.translation.z);
  return cv::checkRange(cv::Mat(output.rotation), true, nullptr) &&
         cv::checkRange(cv::Mat(output.translation), true, nullptr);
}

std::string derivedOpticalFrame(const std::string &projection_frame) {
  constexpr const char *kProjection = "projection_frame";
  const size_t position = projection_frame.rfind(kProjection);
  if (position == std::string::npos) return std::string();
  std::string result = projection_frame;
  result.replace(position, std::char_traits<char>::length(kProjection),
                 "optical_frame");
  return result;
}

sensor_msgs::msg::CameraInfo makePinholeInfo(
    const std_msgs::msg::Header &header,
    const sonar_image_proc::SonarDrawer::PinholeGeometry &camera) {
  sensor_msgs::msg::CameraInfo info;
  info.header = header;
  info.width = camera.width;
  info.height = camera.height;
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = {camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy,
            0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};
  info.p = {camera.fx, 0.0, camera.cx, 0.0, 0.0, camera.fy,
            camera.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

bool setVerticalApertureGeometry(
    const std::vector<float> &azimuths,
    const std::vector<float> &elevation_beamwidths,
    sonar_image_proc::SonarDrawer::PinholeGeometry &camera) {
  if (azimuths.empty() ||
      (elevation_beamwidths.size() != 1 &&
       elevation_beamwidths.size() != azimuths.size())) {
    return false;
  }

  float maximum_projected_elevation = 0.0F;
  for (size_t index = 0; index < azimuths.size(); ++index) {
    const float azimuth = azimuths[index];
    const float beamwidth = elevation_beamwidths.size() == 1
                                ? elevation_beamwidths.front()
                                : elevation_beamwidths[index];
    if (!std::isfinite(beamwidth) || beamwidth <= 0.0F ||
        beamwidth >= static_cast<float>(M_PI) || !std::isfinite(azimuth) ||
        std::abs(azimuth) >= 0.5F * static_cast<float>(M_PI)) {
      return false;
    }
    // A sonar direction (azimuth a, elevation e) reaches the optical pinhole
    // at y/z = tan(e) / cos(a). The off-axis sec(a) factor matters for the
    // Oculus's very wide fan; omitting it crops the elevation aperture at the
    // outer bearings even when the on-axis +/-beamwidth/2 fits exactly.
    const float projected_elevation =
        std::tan(0.5F * beamwidth) / std::cos(azimuth);
    if (!std::isfinite(projected_elevation) ||
        !(projected_elevation > 0.0F)) {
      return false;
    }
    maximum_projected_elevation =
        std::max(maximum_projected_elevation, projected_elevation);
  }
  if (!(maximum_projected_elevation > 0.0F) || camera.height < 2)
    return false;

  // The ping's transmit beamwidth is the actual observable elevation FOV.
  // Using fx here made a 130-degree Oculus azimuth fan imply roughly 100
  // degrees vertically even though the head only insonifies 20 degrees. The
  // valid surface collapsed into a thin curved strip in a mostly black frame.
  // fx and fy are intentionally different: the configured rectangular raster
  // resamples the very different horizontal and vertical sonar apertures.
  camera.fy = 0.5F * (camera.height - 1) / maximum_projected_elevation;
  return camera.valid();
}

}  // namespace

void DrawSonarComponent::publishRectifiedProducts(
    const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
    const sonar_image_proc::AbstractSonarInterface &interface,
    const cv::Mat &range_bearing_image) {
  const bool image_wanted = rectified_pub_->get_subscription_count() > 0;
  const bool info_wanted = rectified_info_pub_->get_subscription_count() > 0;
  if (!image_wanted && !info_wanted) return;

  const auto geometry = sonar_drawer_.rectifiedImageGeometry(
      interface, rectified_width_, rectified_height_, rectified_aspect_ratio_);
  if (!geometry.valid()) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot rectify sonar ping: invalid range/bearing geometry");
    return;
  }

  sonar_image_proc::msg::RectifiedImageInfo info;
  info.header = msg->header;
  info.width = geometry.width;
  info.height = geometry.height;
  info.horizontal_focal_length = geometry.horizontal_focal_length;
  info.principal_point_u = geometry.principal_point_u;
  info.meters_per_row = geometry.meters_per_row;
  info.min_range = geometry.min_range;
  info.max_range = geometry.max_range;
  info.min_bearing = geometry.min_bearing;
  info.max_bearing = geometry.max_bearing;
  // Publish geometry first, matching fan_info and CameraPublisher ordering for
  // consumers that latch the latest metadata.
  rectified_info_pub_->publish(info);

  if (!image_wanted) return;
  const cv::Mat rectified = sonar_drawer_.rectifyRangeBearingImage(
      interface, range_bearing_image, geometry);
  if (rectified.empty()) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot rectify sonar ping: source raster does not match ping "
        "geometry");
    return;
  }
  cvBridgeAndPublish(msg, rectified, rectified_pub_);
}

void DrawSonarComponent::publishFloorProjectedProducts(
    const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
    const sonar_image_proc::AbstractSonarInterface &interface,
    const cv::Mat &range_bearing_image) {
  const bool image_wanted =
      floor_projected_pub_->get_subscription_count() > 0;
  const bool info_wanted =
      floor_projected_info_pub_->get_subscription_count() > 0;
  if (!image_wanted && !info_wanted) return;

  const auto range_geometry = sonar_drawer_.rectifiedImageGeometry(
      interface, rectified_width_, rectified_height_, rectified_aspect_ratio_);
  const std::string projection_frame =
      floor_projection_sensor_frame_.empty()
          ? msg->header.frame_id
          : floor_projection_sensor_frame_;
  std::string optical_frame = floor_projection_optical_frame_;
  if (optical_frame.empty())
    optical_frame = derivedOpticalFrame(projection_frame);
  if (!range_geometry.valid() || projection_frame.empty() ||
      optical_frame.empty()) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot create floor-projected camera view: invalid output "
        "geometry or no optical frame for '%s'",
        projection_frame.c_str());
    return;
  }

  if (msg->ping_info.tx_beamwidths.empty() ||
      (msg->ping_info.tx_beamwidths.size() != 1 &&
       msg->ping_info.tx_beamwidths.size() !=
           static_cast<size_t>(interface.nAzimuth()))) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot create floor-projected camera view: ping has no valid "
        "transmit/elevation aperture");
    return;
  }

  // Reuse the exact rectilinear horizontal geometry. Vertically, frame the
  // physical transmit aperture rather than deriving an unrelated FOV from the
  // output aspect ratio. Per-bearing aperture differences are still enforced
  // by projectOntoPlaneImage; the widest one defines the raster envelope.
  sonar_image_proc::SonarDrawer::PinholeGeometry camera;
  camera.width = range_geometry.width;
  camera.height = range_geometry.height;
  camera.fx = range_geometry.horizontal_focal_length;
  camera.cx = range_geometry.principal_point_u;
  camera.cy = 0.5f * (camera.height - 1);
  if (!setVerticalApertureGeometry(interface.azimuths(),
                                   msg->ping_info.tx_beamwidths, camera)) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot create floor-projected camera view: invalid "
        "transmit/elevation beamwidths");
    return;
  }

  std_msgs::msg::Header output_header = msg->header;
  output_header.frame_id = optical_frame;
  floor_projected_info_pub_->publish(makePinholeInfo(output_header, camera));
  if (!image_wanted) return;

  try {
    const rclcpp::Time stamp(msg->header.stamp);
    const auto timeout =
        rclcpp::Duration::from_seconds(floor_projection_tf_timeout_);
    const auto sensor_from_camera_msg = tf_buffer_->lookupTransform(
        projection_frame, optical_frame, stamp, timeout);
    const auto sensor_from_surface_msg = tf_buffer_->lookupTransform(
        projection_frame, floor_projection_surface_frame_, stamp, timeout);

    sonar_image_proc::SonarDrawer::RigidTransform sensor_from_camera;
    sonar_image_proc::SonarDrawer::RigidTransform sensor_from_surface;
    if (!rigidTransformFromMsg(sensor_from_camera_msg.transform,
                               sensor_from_camera) ||
        !rigidTransformFromMsg(sensor_from_surface_msg.transform,
                               sensor_from_surface)) {
      throw std::runtime_error("TF contains non-finite geometry");
    }

    sonar_image_proc::SonarDrawer::Plane floor_plane;
    floor_plane.normal =
        sensor_from_surface.rotation * cv::Vec3f(0.0f, 0.0f, 1.0f);
    floor_plane.offset =
        -floor_plane.normal.dot(sensor_from_surface.translation);

    const cv::Mat floor_projected = sonar_drawer_.projectOntoPlaneImage(
        interface, range_bearing_image, camera, sensor_from_camera, floor_plane,
        msg->ping_info.tx_beamwidths);
    if (floor_projected.empty()) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Cannot create floor-projected camera view: invalid plane, "
          "aperture, or source raster");
      return;
    }

    cv_bridge::CvImage image_bridge(output_header, "rgb8", floor_projected);
    floor_projected_pub_->publish(*image_bridge.toImageMsg());
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot create floor-projected camera view at ping stamp: %s",
        error.what());
  } catch (const std::exception &error) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot create floor-projected camera view: %s", error.what());
  }
}

}  // namespace draw_sonar
