// Copyright 2026 ERDC

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "sonar_image_proc/DrawSonar.h"

namespace {

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

}  // namespace
