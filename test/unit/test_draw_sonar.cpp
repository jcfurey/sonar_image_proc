// Copyright 2026 ERDC

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
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

}  // namespace
