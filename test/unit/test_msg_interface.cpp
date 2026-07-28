// Copyright 2026 ERDC

// Log scaling used to be gated on DTYPE_UINT32, which made log_scale a silent
// no-op for the 8- and 16-bit pings the Oculus driver actually produces. On a
// real 16-bit ping that left 91.9% of samples mapping to pure black, with the
// brightest sample in the image reaching only 94/255 -- the fan could not
// contain white. These pin the dtype-agnostic behaviour.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "sonar_image_proc/sonar_image_msg_interface.h"

namespace {

using marine_acoustic_msgs::msg::ProjectedSonarImage;
using marine_acoustic_msgs::msg::SonarImageData;
using sonar_image_proc::SonarImageMsgInterface;

// One beam, N ranges, with the given raw sample values.
ProjectedSonarImage::SharedPtr makePing(uint8_t dtype,
                                        const std::vector<uint32_t> &samples) {
  auto ping = std::make_shared<ProjectedSonarImage>();
  ping->ranges.resize(samples.size());
  for (size_t i = 0; i < samples.size(); ++i) ping->ranges[i] = 0.1f * (i + 1);

  geometry_msgs::msg::Vector3 dir;
  dir.x = 1.0;
  dir.y = 0.0;
  dir.z = 0.0;
  ping->beam_directions.push_back(dir);

  ping->image.dtype = dtype;
  ping->image.beam_count = 1;
  const size_t elem = (dtype == SonarImageData::DTYPE_UINT8)    ? 1
                      : (dtype == SonarImageData::DTYPE_UINT16) ? 2
                                                                : 4;
  ping->image.data.resize(samples.size() * elem, 0);
  for (size_t i = 0; i < samples.size(); ++i)
    for (size_t b = 0; b < elem; ++b)
      ping->image.data[i * elem + b] =
          static_cast<uint8_t>((samples[i] >> (8 * b)) & 0xFF);
  return ping;
}

TEST(TestMsgInterface, LogScaleAppliesToUint16) {
  // 655 is ~1% of full scale: linearly that is 2/255, essentially black.
  auto ping = makePing(SonarImageData::DTYPE_UINT16, {655});
  SonarImageMsgInterface linear(ping);
  SonarImageMsgInterface logged(ping);
  logged.do_log_scale(-80.0f, 0.0f);

  const sonar_image_proc::AzimuthRangeIndices idx(0, 0);
  const uint8_t lin = linear.intensity_uint8(idx);
  const uint8_t lg = logged.intensity_uint8(idx);

  EXPECT_LT(lin, 5) << "linear should be near-black for a 1% sample";
  EXPECT_GT(lg, 100) << "log scaling must lift it well clear of black";
  EXPECT_NE(lin, lg) << "log_scale was a no-op for 16-bit data";
}

TEST(TestMsgInterface, LogScaleAppliesToUint8) {
  auto ping = makePing(SonarImageData::DTYPE_UINT8, {3});
  SonarImageMsgInterface linear(ping);
  SonarImageMsgInterface logged(ping);
  logged.do_log_scale(-80.0f, 0.0f);

  const sonar_image_proc::AzimuthRangeIndices idx(0, 0);
  EXPECT_EQ(linear.intensity_uint8(idx), 3);
  EXPECT_GT(logged.intensity_uint8(idx), 100)
      << "log_scale was a no-op for 8-bit data";
}

TEST(TestMsgInterface, LogScaleStillWorksForUint32) {
  // The one dtype that was never broken -- guard against regressing it.
  auto ping = makePing(SonarImageData::DTYPE_UINT32, {42949673u});  // ~1%
  SonarImageMsgInterface logged(ping);
  logged.do_log_scale(-80.0f, 0.0f);
  EXPECT_GT(logged.intensity_uint8(sonar_image_proc::AzimuthRangeIndices(0, 0)),
            100);
}

TEST(TestMsgInterface, LogScaleIsMonotonicAndBounded) {
  auto ping = makePing(SonarImageData::DTYPE_UINT16,
                       {0, 1, 100, 1000, 10000, 65535});
  SonarImageMsgInterface logged(ping);
  logged.do_log_scale(-80.0f, 0.0f);

  uint8_t prev = 0;
  for (int r = 0; r < 6; ++r) {
    const uint8_t v =
        logged.intensity_uint8(sonar_image_proc::AzimuthRangeIndices(0, r));
    EXPECT_GE(v, prev) << "log mapping must be non-decreasing at range " << r;
    prev = v;
  }
  EXPECT_EQ(prev, 255) << "a full-scale sample must reach full output";
}

TEST(TestMsgInterface, ZeroSampleDoesNotProduceNaN) {
  // log10(0) is -inf; the LSB floor is what keeps this finite.
  auto ping = makePing(SonarImageData::DTYPE_UINT16, {0});
  SonarImageMsgInterface logged(ping);
  logged.do_log_scale(-80.0f, 0.0f);
  const float f =
      logged.intensity_float(sonar_image_proc::AzimuthRangeIndices(0, 0));
  EXPECT_TRUE(std::isfinite(f));
  EXPECT_GE(f, 0.0f);
  EXPECT_LE(f, 1.0f);
}

TEST(TestMsgInterface, LinearPathUnchangedWithoutLogScale) {
  auto ping = makePing(SonarImageData::DTYPE_UINT16, {0x1234});
  SonarImageMsgInterface iface(ping);
  const sonar_image_proc::AzimuthRangeIndices idx(0, 0);
  EXPECT_EQ(iface.intensity_uint16(idx), 0x1234);
  EXPECT_EQ(iface.intensity_uint8(idx), 0x12);  // >> 8, as before
}

}  // namespace
