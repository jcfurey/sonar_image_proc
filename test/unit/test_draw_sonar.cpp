// Copyright 2026 ERDC

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "sonar_image_proc/DrawSonar.h"
#include "sonar_image_proc/ImageLayout.h"

namespace {

TEST(TestDrawSonar, DecodesStandardBeamMajorPayload) {
  const std::vector<std::uint8_t> beam_major{
      10, 0, 11, 0, 12, 0, 20, 0, 21, 0, 22, 0};
  const std::vector<std::uint8_t> expected_range_major{
      10, 0, 20, 0, 11, 0, 21, 0, 12, 0, 22, 0};
  std::vector<std::uint8_t> range_major;

  ASSERT_TRUE(sonar_image_proc::beamMajorToRangeMajor(
      beam_major, 3, 2, 2, range_major));
  EXPECT_EQ(range_major, expected_range_major);
  EXPECT_FALSE(sonar_image_proc::beamMajorToRangeMajor(
      beam_major, 4, 2, 2, range_major));
}

class TestPing : public sonar_image_proc::AbstractSonarInterface {
 public:
  TestPing(std::vector<float> ranges, std::vector<float> azimuths)
      : ranges_(std::move(ranges)),
        azimuths_(std::move(azimuths)),
        intensities_(ranges_.size() * azimuths_.size(), 0.0f) {}

  DataType_t data_type() const override { return TYPE_FLOAT32; }

  const std::vector<float> &ranges() const override { return ranges_; }
  const std::vector<float> &azimuths() const override { return azimuths_; }

  float intensity_float(
      const sonar_image_proc::AzimuthRangeIndices &idx) const override {
    return intensities_.at(idx.range() * azimuths_.size() + idx.azimuth());
  }

  void setIntensity(size_t range, size_t azimuth, float value) {
    intensities_.at(range * azimuths_.size() + azimuth) = value;
  }

 private:
  std::vector<float> ranges_;
  std::vector<float> azimuths_;
  std::vector<float> intensities_;
};

TEST(TestDrawSonar, RectangularImagePreservesRangeMajorLayout) {
  TestPing ping({0.0f, 1.0f, 2.0f}, {-0.5f, 0.5f});
  ping.setIntensity(2, 1, 1.0f);

  sonar_image_proc::SonarDrawer drawer;
  const cv::Mat rect = drawer.drawRectSonarImage(
      ping, sonar_image_proc::SonarColorMap(),
      cv::Mat(0, 0, CV_32FC1));

  ASSERT_EQ(rect.rows, 2);
  ASSERT_EQ(rect.cols, 3);
  EXPECT_FLOAT_EQ(rect.at<float>(1, 2), 1.0f);
  EXPECT_FLOAT_EQ(rect.at<float>(0, 2), 0.0f);
}

TEST(TestDrawSonar, RectifiedGeometryIsNativeHeightAndRectilinearWidth) {
  const float edge = std::atan(1.0f);
  TestPing ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f}, {-edge, 0.0f, edge});

  sonar_image_proc::SonarDrawer drawer;
  const auto geometry = drawer.rectifiedImageGeometry(ping);

  ASSERT_TRUE(geometry.valid());
  EXPECT_EQ(geometry.height, 5);  // one row per native range bin
  EXPECT_EQ(geometry.width, 9);   // round(5 * 16/9)
  EXPECT_FLOAT_EQ(geometry.horizontal_focal_length, 4.0f);
  EXPECT_FLOAT_EQ(geometry.principal_point_u, 4.0f);
  EXPECT_FLOAT_EQ(geometry.meters_per_row, 1.0f);
  EXPECT_FLOAT_EQ(geometry.min_range, 0.0f);
  EXPECT_FLOAT_EQ(geometry.max_range, 4.0f);
}

TEST(TestDrawSonar, RectifiedImageHasCameraStyleOrientation) {
  const float edge = std::atan(1.0f);
  TestPing ping({0.0f, 1.0f, 2.0f}, {-edge, 0.0f, edge});
  cv::Mat source = cv::Mat::zeros(3, 3, CV_32FC1);
  source.at<float>(0, 0) = 0.25f;  // near, negative bearing
  source.at<float>(1, 1) = 0.50f;  // middle range, boresight
  source.at<float>(2, 2) = 1.00f;  // far, positive bearing

  sonar_image_proc::SonarDrawer drawer;
  const auto geometry = drawer.rectifiedImageGeometry(ping, 3, 3);
  const cv::Mat rectified =
      drawer.rectifyRangeBearingImage(ping, source, geometry);

  ASSERT_EQ(rectified.size(), cv::Size(3, 3));
  EXPECT_NEAR(rectified.at<float>(2, 0), 0.25f, 1e-6f);
  EXPECT_NEAR(rectified.at<float>(1, 1), 0.50f, 1e-6f);
  EXPECT_NEAR(rectified.at<float>(0, 2), 1.00f, 1e-6f);
}

TEST(TestDrawSonar, RectifiedImageUsesActualNonUniformRangeTable) {
  const float edge = std::atan(1.0f);
  TestPing ping({0.0f, 0.25f, 2.0f}, {-edge, 0.0f, edge});
  cv::Mat source(3, 3, CV_32FC1);
  for (int a = 0; a < source.rows; ++a) {
    source.at<float>(a, 0) = 0.0f;
    source.at<float>(a, 1) = 0.25f;
    source.at<float>(a, 2) = 2.0f;
  }

  sonar_image_proc::SonarDrawer drawer;
  const auto geometry = drawer.rectifiedImageGeometry(ping, 3, 5);
  const cv::Mat rectified =
      drawer.rectifyRangeBearingImage(ping, source, geometry);

  ASSERT_EQ(rectified.size(), cv::Size(3, 5));
  // Linear interpolation against the physical range table reproduces this
  // ramp. Treating the three input columns as uniformly spaced would not.
  EXPECT_NEAR(rectified.at<float>(1, 1), 1.5f, 0.03f);
  EXPECT_NEAR(rectified.at<float>(2, 1), 1.0f, 0.03f);
  EXPECT_NEAR(rectified.at<float>(3, 1), 0.5f, 0.03f);
}

TEST(TestDrawSonar, RectifiedCacheInvalidatesForChangedInteriorRange) {
  const float edge = std::atan(1.0f);
  TestPing first_ping({0.0f, 0.25f, 2.0f}, {-edge, 0.0f, edge});
  TestPing second_ping({0.0f, 1.75f, 2.0f}, {-edge, 0.0f, edge});
  cv::Mat source = cv::Mat::zeros(3, 3, CV_32FC1);
  source.col(1).setTo(1.0f);

  sonar_image_proc::SonarDrawer drawer;
  const auto first_geometry = drawer.rectifiedImageGeometry(first_ping, 3, 5);
  const auto second_geometry =
      drawer.rectifiedImageGeometry(second_ping, 3, 5);
  const cv::Mat first =
      drawer.rectifyRangeBearingImage(first_ping, source, first_geometry);
  const cv::Mat second =
      drawer.rectifyRangeBearingImage(second_ping, source, second_geometry);

  ASSERT_EQ(first.size(), second.size());
  EXPECT_GT(cv::norm(first, second, cv::NORM_INF), 0.2);
}

TEST(TestDrawSonar, RectifiedImageUsesActualNonUniformBearingTable) {
  const float edge = std::atan(1.0f);
  const std::vector<float> bearings{-edge, -0.2f, edge};
  TestPing ping({0.0f, 1.0f, 2.0f}, bearings);
  cv::Mat source(3, 3, CV_32FC1);
  for (int a = 0; a < source.rows; ++a)
    source.row(a).setTo(bearings[a]);

  sonar_image_proc::SonarDrawer drawer;
  const auto geometry = drawer.rectifiedImageGeometry(ping, 5, 3);
  const cv::Mat rectified =
      drawer.rectifyRangeBearingImage(ping, source, geometry);

  ASSERT_EQ(rectified.size(), cv::Size(5, 3));
  for (int u = 0; u < rectified.cols; ++u) {
    const float expected = std::atan(
        (u - geometry.principal_point_u) /
        geometry.horizontal_focal_length);
    EXPECT_NEAR(rectified.at<float>(1, u), expected, 0.03f) << "u=" << u;
  }
}

sonar_image_proc::SonarDrawer::RigidTransform sonarFromOptical() {
  sonar_image_proc::SonarDrawer::RigidTransform transform;
  // Camera optical: +x right, +y down, +z forward.
  // Sonar projection: +x down, +y left, +z forward.
  transform.rotation = cv::Matx33f(0.0f, 1.0f, 0.0f,
                                   -1.0f, 0.0f, 0.0f,
                                   0.0f, 0.0f, 1.0f);
  return transform;
}

sonar_image_proc::SonarDrawer::PinholeGeometry testPinhole(
    int width = 5, int height = 5) {
  sonar_image_proc::SonarDrawer::PinholeGeometry camera;
  camera.width = width;
  camera.height = height;
  camera.fx = 2.0f;
  camera.fy = 2.0f;
  camera.cx = 0.5f * (width - 1);
  camera.cy = 0.5f * (height - 1);
  return camera;
}

TestPing makeSyntheticFloorPing(const cv::Vec3f &platformUp,
                                float floorDistance, float beamwidth,
                                float floorBeamFraction = 1.0f,
                                bool addNearRing = false) {
  std::vector<float> ranges;
  for (int index = 0; index <= 200; ++index)
    ranges.push_back(0.02f * index);
  std::vector<float> azimuths;
  for (int index = 0; index < 65; ++index)
    azimuths.push_back(-0.6f + 1.2f * index / 64.0f);

  TestPing ping(std::move(ranges), std::move(azimuths));
  const size_t floorBeamCount = static_cast<size_t>(std::clamp(
      std::lround(floorBeamFraction * ping.azimuths().size()), 0L,
      static_cast<long>(ping.azimuths().size())));
  const size_t floorBeamBegin =
      (ping.azimuths().size() - floorBeamCount) / 2;
  const size_t floorBeamEnd = floorBeamBegin + floorBeamCount;
  constexpr int kElevationSamples = 128;
  for (size_t beam = 0; beam < ping.azimuths().size(); ++beam) {
    float qMinimum = std::numeric_limits<float>::infinity();
    float qMaximum = -std::numeric_limits<float>::infinity();
    const float azimuth = ping.azimuths()[beam];
    for (int sample = 0; sample <= kElevationSamples; ++sample) {
      const float elevation =
          -0.5f * beamwidth + beamwidth * sample / kElevationSamples;
      const float cosElevation = std::cos(elevation);
      const cv::Vec3f ray(std::sin(elevation),
                          -cosElevation * std::sin(azimuth),
                          cosElevation * std::cos(azimuth));
      const float q = -platformUp.dot(ray);
      qMinimum = std::min(qMinimum, q);
      qMaximum = std::max(qMaximum, q);
    }

    for (size_t range = 0; range < ping.ranges().size(); ++range) {
      const float r = ping.ranges()[range];
      const float q = r > 0.0f ? floorDistance / r
                               : std::numeric_limits<float>::infinity();
      const float positiveMinimum = std::max(0.0f, qMinimum);
      if (beam >= floorBeamBegin && beam < floorBeamEnd &&
          q > positiveMinimum && q <= qMaximum) {
        ping.setIntensity(range, beam, 1.0f);
      }
    }

    // A brighter but one-bin-thick rail is a deliberate distractor. A detector
    // that scores only the leading edge chooses this instead of the broad floor
    // reverberation band.
    ping.setIntensity(60, beam, 1.0f);

    // The field Oculus data also carries a broad-looking near-head/range ring.
    // It spans enough bins to fool a short persistence test, but not the 0.5 m
    // floor-reverberation window used by the deployed detector.
    if (addNearRing) {
      for (size_t range = 20; range <= 28; ++range)
        ping.setIntensity(range, beam, 1.0f);
    }
  }
  return ping;
}

sonar_image_proc::SonarDrawer::PinholeGeometry aperturePinhole(
    float azimuthHalfWidth, float elevationHalfWidth) {
  sonar_image_proc::SonarDrawer::PinholeGeometry camera;
  camera.width = 65;
  camera.height = 65;
  camera.cx = 32.0f;
  camera.cy = 32.0f;
  camera.fx = 32.0f / std::tan(azimuthHalfWidth);
  camera.fy = 32.0f / std::tan(elevationHalfWidth);
  return camera;
}

TEST(TestDrawSonar, FloorDetectorUsesPersistentReturnInsteadOfThinRail) {
  constexpr float kBeamwidth = 20.0f * static_cast<float>(M_PI) / 180.0f;
  constexpr float kFloorDistance = 0.50f;
  // A level head sees the floor only through the +elevation (image-bottom)
  // half of the transmit aperture. Projection-frame +x is down, so platform
  // up is -x.
  const cv::Vec3f platformUp(-1.0f, 0.0f, 0.0f);
  const TestPing ping =
      makeSyntheticFloorPing(platformUp, kFloorDistance, kBeamwidth);

  sonar_image_proc::SonarDrawer drawer;
  sonar_image_proc::SonarDrawer::FloorDetectionConfig config;
  config.minimum_range = 0.2f;
  config.minimum_score = 0.05f;
  config.persistence_range = 0.16f;
  config.edge_window_bins = 3;
  const auto estimate = drawer.estimateFloorPlaneFromImage(
      ping, platformUp, {kBeamwidth}, config);

  ASSERT_TRUE(estimate.detected);
  EXPECT_NEAR(estimate.distance, kFloorDistance, 0.02f);
  EXPECT_GT(estimate.score, config.minimum_score);
  EXPECT_GT(estimate.support_fraction, 0.95f);
  EXPECT_NEAR(cv::norm(estimate.plane.normal), 1.0f, 1e-6f);
  EXPECT_FLOAT_EQ(estimate.plane.offset, estimate.distance);
}

TEST(TestDrawSonar, FloorDetectorRejectsShortNearFieldReverberation) {
  constexpr float kBeamwidth = 20.0f * static_cast<float>(M_PI) / 180.0f;
  constexpr float kFloorDistance = 0.50f;
  const cv::Vec3f platformUp(-1.0f, 0.0f, 0.0f);
  const TestPing ping = makeSyntheticFloorPing(
      platformUp, kFloorDistance, kBeamwidth, 1.0f, true);

  sonar_image_proc::SonarDrawer drawer;
  sonar_image_proc::SonarDrawer::FloorDetectionConfig config;
  config.edge_window_bins = 3;
  const auto estimate = drawer.estimateFloorPlaneFromImage(
      ping, platformUp, {kBeamwidth}, config);

  ASSERT_TRUE(estimate.detected);
  EXPECT_NEAR(estimate.distance, kFloorDistance, 0.02f);
}

TEST(TestDrawSonar, FloorDetectorHonorsConfiguredPartialBeamSupport) {
  constexpr float kBeamwidth = 20.0f * static_cast<float>(M_PI) / 180.0f;
  constexpr float kFloorDistance = 0.50f;
  const cv::Vec3f platformUp(-1.0f, 0.0f, 0.0f);
  const TestPing ping = makeSyntheticFloorPing(
      platformUp, kFloorDistance, kBeamwidth, 0.55f);

  sonar_image_proc::SonarDrawer drawer;
  sonar_image_proc::SonarDrawer::FloorDetectionConfig config;
  config.minimum_support_fraction = 0.40f;
  config.edge_window_bins = 3;
  const auto estimate = drawer.estimateFloorPlaneFromImage(
      ping, platformUp, {kBeamwidth}, config);

  ASSERT_TRUE(estimate.detected);
  EXPECT_NEAR(estimate.distance, kFloorDistance, 0.02f);
  EXPECT_GE(estimate.support_fraction, config.minimum_support_fraction);
  EXPECT_LT(estimate.support_fraction, 0.80f);
}

TEST(TestDrawSonar, FloorDetectorRejectsWhenTiltPointsApertureAwayFromFloor) {
  constexpr float kBeamwidth = 20.0f * static_cast<float>(M_PI) / 180.0f;
  const TestPing ping = makeSyntheticFloorPing(
      cv::Vec3f(-1.0f, 0.0f, 0.0f), 0.5f, kBeamwidth);
  sonar_image_proc::SonarDrawer drawer;

  // +z is platform-up in this synthetic sensor pose, so every forward ray has
  // a negative floorward component. A bright image edge cannot override the
  // pivot-head geometry and invent a floor behind the sonar.
  sonar_image_proc::SonarDrawer::FloorDetectionConfig config;
  const auto estimate = drawer.estimateFloorPlaneFromImage(
      ping, cv::Vec3f(0.0f, 0.0f, 1.0f), {kBeamwidth}, config);
  EXPECT_FALSE(estimate.detected);
  EXPECT_FLOAT_EQ(estimate.support_fraction, 0.0f);
}

TEST(TestDrawSonar, HeadTiltSelectsTopOrBottomForSameFloorReturn) {
  constexpr float kBeamwidth = 20.0f * static_cast<float>(M_PI) / 180.0f;
  constexpr float kFloorDistance = 0.50f;
  const cv::Vec3f bottomFacingUp(-1.0f, 0.0f, 0.0f);
  const cv::Vec3f topFacingUp(1.0f, 0.0f, 0.0f);
  const TestPing ping =
      makeSyntheticFloorPing(bottomFacingUp, kFloorDistance, kBeamwidth);

  sonar_image_proc::SonarDrawer drawer;
  sonar_image_proc::SonarDrawer::FloorDetectionConfig config;
  config.minimum_score = 0.05f;
  config.edge_window_bins = 3;
  const auto bottom = drawer.estimateFloorPlaneFromImage(
      ping, bottomFacingUp, {kBeamwidth}, config);
  // A 2-D ping cannot distinguish +/-elevation. The pivot-head orientation is
  // what resolves the other physically valid branch, using the identical
  // image evidence here.
  const auto top = drawer.estimateFloorPlaneFromImage(
      ping, topFacingUp, {kBeamwidth}, config);
  ASSERT_TRUE(bottom.detected);
  ASSERT_TRUE(top.detected);
  EXPECT_NEAR(bottom.distance, top.distance, 0.01f);

  cv::Mat source = drawer.drawRectSonarImage(
      ping, sonar_image_proc::SonarColorMap(), cv::Mat(0, 0, CV_32FC1));
  const auto camera = aperturePinhole(0.6f, 0.5f * kBeamwidth);
  const cv::Mat bottomImage = drawer.projectOntoPlaneImage(
      ping, source, camera, sonarFromOptical(), bottom.plane, {kBeamwidth});
  const cv::Mat topImage = drawer.projectOntoPlaneImage(
      ping, source, camera, sonarFromOptical(), top.plane, {kBeamwidth});
  ASSERT_FALSE(bottomImage.empty());
  ASSERT_FALSE(topImage.empty());

  const cv::Rect upper(0, 0, camera.width, camera.height / 2);
  const cv::Rect lower(0, camera.height / 2 + 1, camera.width,
                       camera.height / 2);
  EXPECT_FLOAT_EQ(static_cast<float>(cv::sum(bottomImage(upper))[0]), 0.0f);
  EXPECT_GT(cv::sum(bottomImage(lower))[0], 1.0);
  EXPECT_GT(cv::sum(topImage(upper))[0], 1.0);
  EXPECT_FLOAT_EQ(static_cast<float>(cv::sum(topImage(lower))[0]), 0.0f);
}

TEST(TestDrawSonar, PlaneProjectionPlacesSurfaceOnEitherSideOfHorizon) {
  const float edge = std::atan(1.0f);
  TestPing ping({0.5f, 1.0f, std::sqrt(2.0f), 2.0f, 3.0f},
                {-edge, 0.0f, edge});
  const cv::Mat source = cv::Mat::ones(3, 5, CV_32FC1);
  const auto camera = testPinhole();
  const auto transform = sonarFromOptical();
  sonar_image_proc::SonarDrawer drawer;

  // +x is down in the sonar projection frame. x=+1 must therefore occupy
  // the lower image, while x=-1 must occupy the upper image. This is the
  // property a fixed "seafloor is at the bottom" warp cannot satisfy.
  sonar_image_proc::SonarDrawer::Plane below;
  below.normal = cv::Vec3f(1.0f, 0.0f, 0.0f);
  below.offset = -1.0f;
  const cv::Mat belowImage = drawer.projectOntoPlaneImage(
      ping, source, camera, transform, below, {2.4f});
  ASSERT_EQ(belowImage.size(), cv::Size(5, 5));
  EXPECT_GT(belowImage.at<float>(4, 2), 0.9f);
  EXPECT_FLOAT_EQ(belowImage.at<float>(0, 2), 0.0f);

  sonar_image_proc::SonarDrawer::Plane above = below;
  above.offset = 1.0f;
  const cv::Mat aboveImage = drawer.projectOntoPlaneImage(
      ping, source, camera, transform, above, {2.4f});
  ASSERT_EQ(aboveImage.size(), cv::Size(5, 5));
  EXPECT_GT(aboveImage.at<float>(0, 2), 0.9f);
  EXPECT_FLOAT_EQ(aboveImage.at<float>(4, 2), 0.0f);
}

TEST(TestDrawSonar, PlaneProjectionRespectsVerticalAperture) {
  const float edge = std::atan(1.0f);
  TestPing ping({0.5f, 1.0f, std::sqrt(2.0f), 2.0f, 3.0f},
                {-edge, 0.0f, edge});
  const cv::Mat source = cv::Mat::ones(3, 5, CV_32FC1);
  sonar_image_proc::SonarDrawer::Plane below;
  below.normal = cv::Vec3f(1.0f, 0.0f, 0.0f);
  below.offset = -1.0f;

  sonar_image_proc::SonarDrawer drawer;
  const cv::Mat projected = drawer.projectOntoPlaneImage(
      ping, source, testPinhole(), sonarFromOptical(), below,
      {20.0f * static_cast<float>(M_PI) / 180.0f});

  ASSERT_EQ(projected.size(), cv::Size(5, 5));
  // The x=z floor ray is 45 degrees below boresight and cannot have produced
  // an echo through a +/-10 degree transmit aperture.
  EXPECT_FLOAT_EQ(projected.at<float>(4, 2), 0.0f);
}

TEST(TestDrawSonar, PlaneProjectionUnbowsAFrontoParallelWall) {
  const float edge = std::atan(1.0f);
  const float edgeRange = std::sqrt(8.0f);
  TestPing ping({2.0f, edgeRange}, {-edge, 0.0f, edge});
  cv::Mat source = cv::Mat::zeros(3, 2, CV_32FC1);
  // A wall at sonar-forward z=2 appears as a bowed range trace in the native
  // raster: r=2 on boresight and r=sqrt(8) at +/-45 degrees.
  source.at<float>(0, 1) = 1.0f;
  source.at<float>(1, 0) = 1.0f;
  source.at<float>(2, 1) = 1.0f;

  sonar_image_proc::SonarDrawer::Plane wall;
  wall.normal = cv::Vec3f(0.0f, 0.0f, 1.0f);
  wall.offset = -2.0f;
  sonar_image_proc::SonarDrawer drawer;
  const cv::Mat projected = drawer.projectOntoPlaneImage(
      ping, source, testPinhole(5, 3), sonarFromOptical(), wall, {1.0f});

  ASSERT_EQ(projected.size(), cv::Size(5, 3));
  EXPECT_GT(projected.at<float>(1, 0), 0.9f);
  EXPECT_GT(projected.at<float>(1, 2), 0.9f);
  EXPECT_GT(projected.at<float>(1, 4), 0.9f);
}

TEST(TestDrawSonar, GoldenFanHashPinsTheWholeDrawChain) {
  // Golden regression over the full rect -> remap chain: a deterministic
  // ping with structure at known (range, azimuth) cells, drawn at a fixed
  // scale, must produce byte-identical fan pixels. Pins the remap maps,
  // bin-center conventions, y-axis direction and interpolation in one
  // number. The hash is specific to the pinned libopencv build (ROS
  // Jazzy); if a deliberate drawing change or an OpenCV upgrade moves it,
  // re-bless the constant from the printed value after eyeballing the fan.
  TestPing ping({0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f},
                {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f});
  for (size_t r = 0; r < 8; ++r)
    for (size_t a = 0; a < 5; ++a)
      ping.setIntensity(r, a,
                        0.1f * static_cast<float>(r) +
                        0.02f * static_cast<float>(a));
  ping.setIntensity(6, 2, 1.0f);  // a bright target on the boresight

  sonar_image_proc::SonarDrawer drawer;
  drawer.setPixelsPerMeter(20.0f);
  const cv::Mat rect = drawer.drawRectSonarImage(
      ping, sonar_image_proc::SonarColorMap(), cv::Mat(0, 0, CV_32FC1));
  const cv::Mat fan = drawer.remapRectSonarImage(ping, rect);
  ASSERT_FALSE(fan.empty());
  ASSERT_TRUE(fan.isContinuous());

  std::uint64_t h = 1469598103934665603ULL;
  const std::size_t n = fan.total() * fan.elemSize();
  for (std::size_t i = 0; i < n; ++i) {
    h ^= fan.data[i];
    h *= 1099511628211ULL;
  }
  const std::uint64_t kGolden = 0x8358b5cb5dba73f6ULL;
  EXPECT_EQ(h, kGolden) << "fan hash 0x" << std::hex << h << std::dec
                        << " (" << fan.rows << "x" << fan.cols << ")";
}

TEST(TestDrawSonar, MaxRangeClipsFanAndInvalidatesCachedMap) {
  TestPing ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
                {-static_cast<float>(M_PI) / 6.0f, 0.0f,
                 static_cast<float>(M_PI) / 6.0f});
  cv::Mat rect = cv::Mat::zeros(3, 5, CV_32FC1);

  sonar_image_proc::SonarDrawer drawer;
  drawer.setPixelsPerMeter(10.0f);

  const cv::Mat full = drawer.remapRectSonarImage(ping, rect);
  EXPECT_EQ(full.rows, 40);

  drawer.setMaxRange(2.0f);
  const cv::Mat clipped = drawer.remapRectSonarImage(ping, rect);
  EXPECT_EQ(clipped.rows, 20);
  EXPECT_LT(clipped.cols, full.cols);

  drawer.setMaxRange(0.0f);
  const cv::Mat restored = drawer.remapRectSonarImage(ping, rect);
  EXPECT_EQ(restored.size(), full.size());
}

TEST(TestDrawSonar, MaximumRangeMapsToLastSourceColumn) {
  TestPing ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
                {-static_cast<float>(M_PI) / 6.0f, 0.0f,
                 static_cast<float>(M_PI) / 6.0f});
  cv::Mat rect = cv::Mat::zeros(3, 5, CV_32FC1);
  rect.at<float>(1, 4) = 1.0f;

  sonar_image_proc::SonarDrawer drawer;
  drawer.setPixelsPerMeter(10.0f);
  const cv::Mat fan = drawer.remapRectSonarImage(ping, rect);

  const int origin_x =
      std::abs(static_cast<int>(std::floor(
          fan.rows * std::sin(ping.azimuths().front()))));
  ASSERT_LT(origin_x, fan.cols);
  EXPECT_GT(fan.at<float>(0, origin_x), 0.9f);
}

TEST(TestDrawSonar, RemapUsesActualNonUniformAzimuths) {
  const std::vector<float> azimuths{-0.5f, -0.4f, 0.5f};
  TestPing ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f}, azimuths);

  // Only the middle beam is bright. A uniform min/max interpolation would map
  // azimuth -0.4 near row 0; interpolation against the real bearing table maps
  // it to row 1.
  cv::Mat rect = cv::Mat::zeros(3, 5, CV_32FC1);
  rect.row(1).setTo(1.0f);

  sonar_image_proc::SonarDrawer drawer;
  constexpr float pixels_per_meter = 100.0f;
  drawer.setPixelsPerMeter(pixels_per_meter);
  const cv::Mat fan = drawer.remapRectSonarImage(ping, rect);

  const float radius = 2.0f * pixels_per_meter;
  const int origin_x =
      std::abs(static_cast<int>(std::floor(
          fan.rows * std::sin(azimuths.front()))));
  const int x =
      origin_x + static_cast<int>(std::round(radius * std::sin(-0.4f)));
  const int y =
      fan.rows - static_cast<int>(std::round(radius * std::cos(-0.4f)));

  ASSERT_GE(x, 0);
  ASSERT_LT(x, fan.cols);
  ASSERT_GE(y, 0);
  ASSERT_LT(y, fan.rows);
  EXPECT_GT(fan.at<float>(y, x), 0.8f);
}

TEST(TestDrawSonar, ChangedInteriorAzimuthInvalidatesCachedMap) {
  TestPing first_ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
                      {-0.5f, -0.4f, 0.5f});
  TestPing second_ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
                       {-0.5f, 0.4f, 0.5f});
  cv::Mat rect = cv::Mat::zeros(3, 5, CV_32FC1);
  rect.row(1).setTo(1.0f);

  sonar_image_proc::SonarDrawer drawer;
  drawer.setPixelsPerMeter(20.0f);
  const cv::Mat first = drawer.remapRectSonarImage(first_ping, rect);
  const cv::Mat second = drawer.remapRectSonarImage(second_ping, rect);

  ASSERT_EQ(first.size(), second.size());
  EXPECT_GT(cv::norm(first, second, cv::NORM_INF), 0.1);
}

// The sonar changes its range resolution with the commanded range, so the
// default scale is "native": one output pixel per range bin. These pin the
// behaviour the deployed draw_sonar.yaml now relies on.

TEST(TestDrawSonar, NativeScaleIsOnePixelPerRangeBin) {
  // 5 bins spanning 0..4 m -> native scale is 5/4 = 1.25 px/m.
  TestPing ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f}, {-0.5f, 0.0f, 0.5f});

  sonar_image_proc::SonarDrawer drawer;  // default: native
  EXPECT_FLOAT_EQ(drawer.effectivePixelsPerMeter(ping), 5.0f / 4.0f);
}

TEST(TestDrawSonar, NativeOutputHeightTracksRangeBinCount) {
  // The point of the native scale: height follows the ping's bin count rather
  // than the commanded range, so the fan does not shrink as the range pulls in.
  const std::vector<float> az{-0.5f, 0.0f, 0.5f};
  TestPing near_ping({0.0f, 0.25f, 0.5f, 0.75f, 1.0f}, az);  // 1 m, 5 bins
  TestPing far_ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f}, az);     // 4 m, 5 bins

  cv::Mat rect = cv::Mat::zeros(3, 5, CV_32FC1);
  rect.row(1).setTo(1.0f);

  sonar_image_proc::SonarDrawer drawer;
  const cv::Mat near_fan = drawer.remapRectSonarImage(near_ping, rect);
  const cv::Mat far_fan = drawer.remapRectSonarImage(far_ping, rect);

  ASSERT_FALSE(near_fan.empty());
  ASSERT_FALSE(far_fan.empty());
  // A fixed scale would have made the 1 m fan a quarter the height of the 4 m
  // one; native makes them the same, because both carry 5 range bins.
  EXPECT_EQ(near_fan.rows, far_fan.rows);
}

TEST(TestDrawSonar, ExplicitScaleOverridesNative) {
  TestPing ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f}, {-0.5f, 0.0f, 0.5f});

  sonar_image_proc::SonarDrawer drawer;
  drawer.setPixelsPerMeter(37.0f);
  EXPECT_FLOAT_EQ(drawer.effectivePixelsPerMeter(ping), 37.0f);

  // ... and an explicit value ignores the native cap entirely.
  drawer.setMaxPixelsPerMeter(5.0f);
  EXPECT_FLOAT_EQ(drawer.effectivePixelsPerMeter(ping), 37.0f);
}

TEST(TestDrawSonar, MaxPixelsPerMeterCapsTheNativeScale) {
  // 5 bins over 0.1 m would be 50 px/m natively.
  TestPing ping({0.0f, 0.025f, 0.05f, 0.075f, 0.1f}, {-0.5f, 0.0f, 0.5f});

  sonar_image_proc::SonarDrawer drawer;
  EXPECT_FLOAT_EQ(drawer.effectivePixelsPerMeter(ping), 50.0f);

  drawer.setMaxPixelsPerMeter(20.0f);
  EXPECT_FLOAT_EQ(drawer.effectivePixelsPerMeter(ping), 20.0f);
}

TEST(TestDrawSonar, DegeneratePingFallsBackRatherThanDividingByZero) {
  TestPing empty_ping({}, {-0.5f, 0.0f, 0.5f});
  TestPing single_bin({1.0f}, {-0.5f, 0.0f, 0.5f});

  sonar_image_proc::SonarDrawer drawer;
  EXPECT_FLOAT_EQ(drawer.effectivePixelsPerMeter(empty_ping), 100.0f);
  EXPECT_FLOAT_EQ(drawer.effectivePixelsPerMeter(single_bin), 100.0f);
}

TEST(TestDrawSonar, OverlayAnchorsBearingGridAtZero) {
  TestPing ping({0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
                {-65.0f * static_cast<float>(M_PI) / 180.0f, 0.0f,
                 65.0f * static_cast<float>(M_PI) / 180.0f});

  sonar_image_proc::SonarDrawer drawer;
  drawer.setPixelsPerMeter(125.0f);
  drawer.overlayConfig()
      .setRangeSpacing(100.0f)  // no internal arc through the test point
      .setRadialSpacing(10.0f)
      .setRadialAtZero(true)
      .setLineAlpha(1.0f)
      .setFontScale(0.5f);

  const auto geometry = drawer.fanImageGeometry(ping);
  ASSERT_GT(geometry.width, 0);
  ASSERT_GT(geometry.height, 0);
  const cv::Mat clean =
      cv::Mat::zeros(geometry.height, geometry.width, CV_8UC3);
  const cv::Mat annotated = drawer.drawOverlay(ping, clean);

  // The zero-bearing ray runs vertically from the sonar origin. This pixel is
  // well away from the outer range arc and all text, so a nonzero value pins
  // the boresight itself rather than merely the existence of an overlay.
  const cv::Vec3b boresight = annotated.at<cv::Vec3b>(
      geometry.height / 2, geometry.origin_x);
  EXPECT_GT(boresight[0] + boresight[1] + boresight[2], 0);
}

}  // namespace
