// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>

#include "sonar_image_proc/DrawSonar.h"
#include "sonar_image_proc/CoordinateTable.h"
#include "sonar_image_proc/OverlayImage.h"

namespace sonar_image_proc {

using sonar_image_proc::AbstractSonarInterface;

static float deg2radf(float deg) { return deg * M_PI / 180.0; }
static float rad2degf(float rad) { return rad * 180.0 / M_PI; }

// OpenCV's built-in fonts do not render a UTF-8 degree symbol reliably, so
// use an explicit unit. These labels are part of the operator image and must
// remain readable without a viewer-specific cursor readout or side channel.
static std::string rangeLabel(float range, float spacing) {
  int precision = spacing < 0.1f ? 2 : (spacing < 1.0f ? 1 : 0);
  if (precision == 0 && std::abs(range - std::round(range)) > 0.01f)
    precision = 1;
  if (precision == 1 &&
      std::abs(range * 10.0f - std::round(range * 10.0f)) > 0.01f)
    precision = 2;

  std::ostringstream text;
  text << std::fixed << std::setprecision(precision) << range << " m";
  return text.str();
}

static std::string bearingLabel(float bearing) {
  const float degrees = rad2degf(bearing);
  const bool integral = std::abs(degrees - std::round(degrees)) < 0.05f;

  std::ostringstream text;
  if (degrees > 0.05f) text << "+";
  text << std::fixed << std::setprecision(integral ? 0 : 1) << degrees
       << " deg";
  return text.str();
}

static void putOutlinedText(cv::Mat &image, const std::string &text,
                            const cv::Point2f &center, float fontScale,
                            const cv::Scalar &color, int lineThickness) {
  const int textThickness = std::max(1, lineThickness);
  int baseline = 0;
  const cv::Size textSize = cv::getTextSize(
      text, cv::FONT_HERSHEY_PLAIN, fontScale, textThickness, &baseline);

  // The OSD canvas is sized around every label before this is called. Do not
  // clamp here: clamping a label at an edge breaks its visual registration
  // with the arc/ray it annotates.
  const cv::Point origin(cvRound(center.x - textSize.width / 2.0f),
                         cvRound(center.y + textSize.height / 2.0f));

  // A dark halo keeps white annotations legible over strong returns without
  // hiding a rectangular patch of sonar data behind each label.
  cv::putText(image, text, origin, cv::FONT_HERSHEY_PLAIN, fontScale,
              cv::Scalar(0, 0, 0, 230), textThickness + 2, cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_PLAIN, fontScale, color,
              textThickness, cv::LINE_AA);
}

// Bounding rectangle of the pixels touched by putOutlinedText(), including
// the dark halo. It is used to grow the OSD canvas before labels are drawn,
// keeping their anchors exact instead of clamping them into the fan raster.
static cv::Rect textBounds(const std::string &text, const cv::Point2f &center,
                           float fontScale, int lineThickness) {
  const int textThickness = std::max(1, lineThickness);
  int baseline = 0;
  const cv::Size textSize = cv::getTextSize(
      text, cv::FONT_HERSHEY_PLAIN, fontScale, textThickness, &baseline);
  const cv::Point origin(cvRound(center.x - textSize.width / 2.0f),
                         cvRound(center.y + textSize.height / 2.0f));
  const int halo = textThickness + 2;
  return cv::Rect(origin.x - halo, origin.y - textSize.height - halo,
                  textSize.width + 2 * halo,
                  textSize.height + baseline + 2 * halo);
}

static float labelClearance(const std::string &text, float fontScale,
                            int lineThickness) {
  const cv::Rect bounds =
      textBounds(text, cv::Point2f(0.0f, 0.0f), fontScale, lineThickness);
  return 0.5f * std::hypot(static_cast<float>(bounds.width),
                            static_cast<float>(bounds.height)) +
         6.0f;
}

// Default to the native scale (see setPixelsPerMeter): the sonar's range
// resolution changes with the commanded range, so a fixed scale is only ever
// right at one range.
SonarDrawer::SonarDrawer()
    : pixels_per_meter_(0.0f), max_pixels_per_meter_(0.0f), max_range_(0.0f) {
  ;
}

float SonarDrawer::effectivePixelsPerMeter(
    const AbstractSonarInterface &ping) const {
  if (pixels_per_meter_ > 0.0f) return pixels_per_meter_;

  // Native: one output pixel per range bin along the radius. Falls back to the
  // historical fixed scale for a degenerate ping so a bad message cannot
  // produce a zero-sized or absurd image.
  constexpr float kFallbackPixelsPerMeter = 100.0f;
  const float max_range = effectiveMaxRange(ping);
  const int n_ranges = ping.nRanges();
  if (max_range <= 0.0f || n_ranges < 2) return kFallbackPixelsPerMeter;

  const float ppm = static_cast<float>(n_ranges) / max_range;
  if (max_pixels_per_meter_ > 0.0f)
    return std::min(ppm, max_pixels_per_meter_);
  return ppm;
}

float SonarDrawer::effectiveMaxRange(const AbstractSonarInterface &ping) const {
  const float ping_max_range = ping.maxRange();
  if (max_range_ <= 0.0f) return ping_max_range;
  return std::min(max_range_, ping_max_range);
}

SonarDrawer::FanGeometry SonarDrawer::fanImageGeometry(
    const AbstractSonarInterface &ping) const {
  // Mirrors CachedMap::Entry::create() below -- keep the two in step, they
  // describe the same image.
  FanGeometry g;
  g.pixels_per_meter = effectivePixelsPerMeter(ping);
  const float max_range = effectiveMaxRange(ping);
  if (max_range <= 0.0f || g.pixels_per_meter <= 0.0f) return g;

  const auto bounds = ping.azimuthBounds();
  g.height = static_cast<int>(ceil(max_range * g.pixels_per_meter));
  const int minus_width =
      static_cast<int>(floor(g.height * sin(bounds.first)));
  const int plus_width = static_cast<int>(ceil(g.height * sin(bounds.second)));
  g.width = plus_width - minus_width;
  g.origin_x = abs(minus_width);
  return g;
}

SonarDrawer::RectifiedGeometry SonarDrawer::rectifiedImageGeometry(
    const AbstractSonarInterface &ping, int outputWidth, int outputHeight,
    float aspectRatio) const {
  RectifiedGeometry g;
  if (ping.nRanges() < 2 || ping.nAzimuth() < 2 || outputWidth < 0 ||
      outputHeight < 0 || !std::isfinite(aspectRatio) ||
      aspectRatio <= 0.0f || aspectRatio > 10.0f) {
    return g;
  }

  const auto rangeBounds = ping.rangeBounds();
  const auto bearingBounds = ping.azimuthBounds();
  g.min_range = rangeBounds.first;
  g.max_range = effectiveMaxRange(ping);
  g.min_bearing = bearingBounds.first;
  g.max_bearing = bearingBounds.second;

  // A rectilinear horizontal coordinate is monotonic only inside the forward
  // pinhole hemisphere. Oculus heads are at most +/-65 degrees, but reject a
  // malformed message here rather than wrapping tan() through infinity.
  constexpr float kHalfPi = static_cast<float>(M_PI / 2.0);
  if (!std::isfinite(g.min_range) || !std::isfinite(g.max_range) ||
      !std::isfinite(g.min_bearing) || !std::isfinite(g.max_bearing) ||
      g.max_range <= g.min_range || g.min_bearing <= -kHalfPi ||
      g.max_bearing >= kHalfPi || g.max_bearing <= g.min_bearing) {
    return RectifiedGeometry();
  }

  if (outputHeight > 0) {
    g.height = outputHeight;
  } else {
    // Retain approximately one row per source range interval when max_range
    // clips a ping. ceil() avoids throwing away the partially represented last
    // interval; the unclipped result is exactly nRanges.
    const float fullSpan = rangeBounds.second - rangeBounds.first;
    if (!(fullSpan > 0.0f)) return RectifiedGeometry();
    const float represented =
        (g.max_range - g.min_range) / fullSpan * (ping.nRanges() - 1);
    g.height = static_cast<int>(std::ceil(represented)) + 1;
  }

  constexpr int kMaxOutputDimension = 16384;
  if (g.height < 2 || g.height > kMaxOutputDimension) {
    return RectifiedGeometry();
  }
  g.width = outputWidth > 0
                ? outputWidth
                : static_cast<int>(std::lround(g.height * aspectRatio));
  if (g.width < 2 || g.width > kMaxOutputDimension) {
    return RectifiedGeometry();
  }

  const float minTan = std::tan(g.min_bearing);
  const float maxTan = std::tan(g.max_bearing);
  const float tanSpan = maxTan - minTan;
  if (!std::isfinite(tanSpan) || !(tanSpan > 0.0f)) {
    return RectifiedGeometry();
  }

  g.horizontal_focal_length = (g.width - 1) / tanSpan;
  g.principal_point_u = -g.horizontal_focal_length * minTan;
  g.meters_per_row = (g.max_range - g.min_range) / (g.height - 1);
  if (!g.valid()) return RectifiedGeometry();
  return g;
}

cv::Mat SonarDrawer::rectifyRangeBearingImage(
    const AbstractSonarInterface &ping, const cv::Mat &rangeBearingImage,
    const RectifiedGeometry &geometry) {
  cv::Mat out;
  if (!geometry.valid() || rangeBearingImage.empty() ||
      rangeBearingImage.rows != ping.nAzimuth() ||
      rangeBearingImage.cols != ping.nRanges()) {
    return out;
  }

  const CachedRectifiedMap::MapPair maps(_rectified_map(ping, geometry));
  if (maps.first.empty() || maps.second.empty()) return out;

  cv::remap(rangeBearingImage, out, maps.first, maps.second, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar::all(0));
  return out;
}

// Fills the rectangular image one azimuth (one row) at a time.  Templated on
// the pixel type so the type dispatch happens once per image rather than
// once per pixel.
template <typename Pixel, typename LookupFn>
static void fillRectRows(cv::Mat &rect, int nRanges, int nAzimuth,
                         LookupFn lookup) {
  for (int b = 0; b < nAzimuth; b++) {
    // Walking along a row is contiguous in memory; the original
    // range-major traversal strode the full row pitch on every pixel.
    Pixel *const row = rect.ptr<Pixel>(b);
    for (int r = 0; r < nRanges; r++) {
      row[r] = lookup(AzimuthRangeIndices(b, r));
    }
  }
}

cv::Mat SonarDrawer::drawRectSonarImage(const AbstractSonarInterface &ping,
                                        const SonarColorMap &colorMap,
                                        const cv::Mat &rectIn) {
  cv::Mat rect(rectIn);

  const int nRanges = ping.nRanges();
  const int nAzimuth = ping.nAzimuth();
  const cv::Size imgSize(nRanges, nAzimuth);

  if ((rect.type() == CV_8UC3) || (rect.type() == CV_32FC3) ||
      (rect.type() == CV_32FC1)) {
    rect.create(imgSize, rect.type());
  } else {
    rect.create(imgSize, CV_8UC3);
  }

  if ((nRanges <= 0) || (nAzimuth <= 0)) return rect;

  switch (rect.type()) {
    case CV_8UC3:
      fillRectRows<cv::Vec3b>(rect, nRanges, nAzimuth,
                              [&](const AzimuthRangeIndices &loc) {
                                return colorMap.lookup_cv8uc3(ping, loc);
                              });
      break;
    case CV_32FC3:
      fillRectRows<cv::Vec3f>(rect, nRanges, nAzimuth,
                              [&](const AzimuthRangeIndices &loc) {
                                return colorMap.lookup_cv32fc3(ping, loc);
                              });
      break;
    case CV_32FC1:
      fillRectRows<float>(rect, nRanges, nAzimuth,
                          [&](const AzimuthRangeIndices &loc) {
                            return colorMap.lookup_cv32fc1(ping, loc);
                          });
      break;
    default:
      // assert() on a string literal is always true, so this never fired
      assert(false && "Should never get here.");
      break;
  }

  return rect;
}

cv::Mat SonarDrawer::remapRectSonarImage(const AbstractSonarInterface &ping,
                                         const cv::Mat &rectImage) {
  cv::Mat out;
  const CachedMap::MapPair maps(
      _map(ping, effectivePixelsPerMeter(ping), effectiveMaxRange(ping)));

  // The map is empty for a degenerate ping -- no ranges, fewer than two
  // beams, or a non-positive max range.  cv::remap asserts on an empty map,
  // and the resulting cv::Exception propagates out of the subscription
  // callback and takes the node down.
  if (maps.first.empty() || maps.second.empty()) return out;

  cv::remap(rectImage, out, maps.first, maps.second, cv::INTER_CUBIC,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  return out;
}

cv::Mat SonarDrawer::drawOverlay(const AbstractSonarInterface &ping,
                                 const cv::Mat &sonarImage) {
  // remapRectSonarImage() returns an empty Mat for a degenerate ping;
  // overlayImage's CV_Assert would abort on it.
  if (sonarImage.empty()) return cv::Mat();

  const cv::Mat &overlay =
      _overlay(ping, sonarImage, overlayConfig(), effectiveMaxRange(ping));

  // The annotated operator image has a display-only border for labels. Keep
  // the clean fan untouched inside that canvas; its original coordinate frame
  // continues to be described by FanImageInfo.
  cv::Mat background = cv::Mat::zeros(overlay.size(), sonarImage.type());
  sonarImage.copyTo(background(cv::Rect(_overlay.imageOrigin(),
                                      sonarImage.size())));

  // Alpha blend overlay onto the padded operator image.
  cv::Mat output;
  overlayImage<unsigned char>(background, overlay, output);

  return output;
}

cv::Mat SonarDrawer::drawSonar(const AbstractSonarInterface &ping,
                               const SonarColorMap &colorMap,
                               const cv::Mat &img, bool addOverlay) {
  cv::Mat rect = drawRectSonarImage(ping, colorMap, img);
  cv::Mat sonar = remapRectSonarImage(ping, rect);
  if (addOverlay) {
    return drawOverlay(ping, sonar);
  } else {
    return sonar;
  }
}

// ==== SonarDrawer::Cached ====

bool SonarDrawer::Cached::isValid(const AbstractSonarInterface &ping) const {
  // Check for cache invalidation...
  if ((_numAzimuth != ping.nAzimuth()) || (_numRanges != ping.nRanges()) ||
      (_rangeBounds != ping.rangeBounds() ||
       (_azimuthBounds != ping.azimuthBounds())))
    return false;

  return true;
}

// ==== SonarDrawer::CachedMap ====

SonarDrawer::CachedMap::MapPair SonarDrawer::CachedMap::operator()(
    const AbstractSonarInterface &ping, float pixelsPerMeter, float maxRange) {
  for (auto &entry : _entries) {
    if (entry.isValidFor(ping, pixelsPerMeter, maxRange))
      return std::make_pair(entry._scMap1, entry._scMap2);
  }

  // Miss -- rebuild into the slot that has gone longest without being built.
  Entry &entry = _entries[_nextEvict];
  _nextEvict = (_nextEvict + 1) % kNumEntries;

  entry.create(ping, pixelsPerMeter, maxRange);
  return std::make_pair(entry._scMap1, entry._scMap2);
}

//  **assumes** this structure for the rectImage:
//   * It has nBearings cols and nRanges rows
//
void SonarDrawer::CachedMap::Entry::create(const AbstractSonarInterface &ping,
                                           float pixelsPerMeter,
                                           float displayMaxRange) {
  // Invalidate FIRST: every early return below (degenerate geometry, fewer
  // than two beams) used to leave the slot's PREVIOUS maps in place, and
  // operator() returns them unconditionally — so a degenerate ping was
  // remapped through another geometry's tables instead of yielding the
  // documented empty Mat.
  _scMap1.release();
  _scMap2.release();
  cv::Mat newmap;

  bool rangesAscending = false;
  bool bearingsAscending = false;
  if (!validateCoordinateTable(ping.ranges(), rangesAscending) ||
      !rangesAscending ||
      !validateCoordinateTable(ping.azimuths(), bearingsAscending)) {
    return;
  }

  const auto azimuthBounds = ping.azimuthBounds();

  // Calculate image dimensions based on pixels per meter scale factor
  // Height represents maxRange since origin is at the bottom
  const int height = static_cast<int>(ceil(displayMaxRange * pixelsPerMeter));
  const int minusWidth =
      static_cast<int>(floor(height * sin(azimuthBounds.first)));
  const int plusWidth =
      static_cast<int>(ceil(height * sin(azimuthBounds.second)));
  const int width = plusWidth - minusWidth;

  const int originx = abs(minusWidth);

  const cv::Size imgSize(width, height);
  if ((width <= 0) || (height <= 0)) return;

  newmap.create(imgSize, CV_32FC2);

  // Real per-beam bearings (radians, monotonic). The Oculus beam table is
  // NON-UNIFORM (measured ~89% max spacing deviation, up to ~6 deg cumulative
  // error at the fan edges), so the old uniform (azimuth - min)/db mapping
  // placed pixels on the wrong beam and bowed straight walls into arcs (the
  // \todo below). Interpolate azimuth -> fractional beam index against the
  // actual bearings instead. GpuSonarDraw.cu:fan_cubic_kernel does the same.
  const auto &azimuths = ping.azimuths();
  const int nAz = static_cast<int>(azimuths.size());
  if (nAz < 2) return;
  const bool ascending = azimuths.back() >= azimuths.front();
  // fractional beam index for a query azimuth, or -1 outside the fan
  auto azToIndex = [&](float a) -> float {
    const float a0 = azimuths.front(), a1 = azimuths.back();
    const float lo_a = ascending ? a0 : a1, hi_a = ascending ? a1 : a0;
    if (a < lo_a - 1e-6f || a > hi_a + 1e-6f) return -1.0f;
    int lo = 0, hi = nAz - 1;
    while (hi - lo > 1) {
      const int mid = (lo + hi) / 2;
      const bool left = ascending ? (azimuths[mid] <= a) : (azimuths[mid] >= a);
      if (left) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    const float denom = azimuths[hi] - azimuths[lo];
    return lo + (std::abs(denom) > 1e-9f ? (a - azimuths[lo]) / denom : 0.0f);
  };

  // Loop invariants.  These were being recomputed inside the innermost loop,
  // where minRange()/maxRange() are virtual calls into the ping that also
  // re-check the cached-bounds state -- millions of times per map.
  const float minRange = ping.minRange();
  const float sourceMaxRange = ping.maxRange();
  const int rows = newmap.rows, cols = newmap.cols;

  // For cv::remap, a map is
  //
  //  dst = src( mapx(x,y), mapy(x,y) )
  //
  // That is, the map is the size of the dst array,
  // and contains the coords in the source image
  // for each pixel in the dst image.
  //
  // This map draws the sonar with range = 0
  // centered on the bottom edge of the resulting image
  // with increasing range along azimuth = 0 going
  // vertically upwards in the image
  //
  // Iterate row-major so the writes run contiguously through newmap.
  for (int y = 0; y < rows; y++) {
    cv::Vec2f *const row = newmap.ptr<cv::Vec2f>(y);

    // Constant across the row
    const float dy = rows - y;
    const float dySq = dy * dy;

    for (int x = 0; x < cols; x++) {
      // Calculate range and bearing of this pixel from origin
      const float dx = x - originx;

      const float range = sqrt(dx * dx + dySq);
      const float azimuth = atan2(dx, dy);

      // Map from pixel coordinates to data coordinates in the rect image
      // rangeInPixels is distance from origin in output image (origin = range
      // 0) Convert to actual range in meters, then to range bin index
      const float rangeInPixels = range;  // Distance in pixels from origin
      const float rangeInMeters =
          rangeInPixels / pixelsPerMeter;  // Convert to meters

      // Clamp to valid range and convert to bin index
      float xp;
      if (rangeInMeters < minRange || rangeInMeters > sourceMaxRange) {
        // Out of range - map to transparent/invalid
        xp = -1.0f;  // Will be clamped/handled by remap
      } else {
        xp = coordinateToIndex(ping.ranges(), rangeInMeters, true);
      }

      // Interpolate against the real (non-uniform) bearing table; -1 (outside
      // the fan) lands out-of-bounds so cv::remap's BORDER_CONSTANT blacks it.
      const float yp = azToIndex(azimuth);

      row[x] = cv::Vec2f(xp, yp);
    }
  }

  cv::convertMaps(newmap, cv::Mat(), _scMap1, _scMap2, CV_16SC2);

  // Save meta-information to check for cache expiry
  _numRanges = ping.nRanges();
  _numAzimuth = ping.nBearings();

  _rangeBounds = ping.rangeBounds();
  _azimuthBounds = ping.azimuthBounds();
  _pixelsPerMeter = pixelsPerMeter;
  _maxRange = displayMaxRange;
  _azimuths = azimuths;
  _ranges = ping.ranges();
}

bool SonarDrawer::CachedMap::Entry::isValidFor(
    const AbstractSonarInterface &ping, float pixelsPerMeter,
    float maxRange) const {
  if (_scMap1.empty() || _scMap2.empty()) return false;

  // Check if pixels per meter has changed
  if (_pixelsPerMeter != pixelsPerMeter) return false;
  if (_maxRange != maxRange) return false;
  if (_azimuths != ping.azimuths()) return false;
  if (_ranges != ping.ranges()) return false;

  return Cached::isValid(ping);
}

// ==== SonarDrawer::CachedRectifiedMap ====

namespace {

bool sameRectifiedGeometry(
    const SonarDrawer::RectifiedGeometry &lhs,
    const SonarDrawer::RectifiedGeometry &rhs) {
  return lhs.width == rhs.width && lhs.height == rhs.height &&
         lhs.horizontal_focal_length == rhs.horizontal_focal_length &&
         lhs.principal_point_u == rhs.principal_point_u &&
         lhs.meters_per_row == rhs.meters_per_row &&
         lhs.min_range == rhs.min_range && lhs.max_range == rhs.max_range &&
         lhs.min_bearing == rhs.min_bearing &&
         lhs.max_bearing == rhs.max_bearing;
}

bool finiteTransform(const SonarDrawer::RigidTransform &transform) {
  for (int row = 0; row < 3; ++row) {
    if (!std::isfinite(transform.translation[row])) return false;
    for (int col = 0; col < 3; ++col) {
      if (!std::isfinite(transform.rotation(row, col))) return false;
    }
  }
  return true;
}

float beamwidthAt(const std::vector<float> &beamwidths, float beamIndex,
                  int beamCount) {
  if (beamwidths.size() == 1) return beamwidths.front();
  if (beamwidths.size() != static_cast<size_t>(beamCount) ||
      !std::isfinite(beamIndex) || beamIndex < 0.0f ||
      beamIndex > beamCount - 1) {
    return -1.0f;
  }
  const int lower = static_cast<int>(std::floor(beamIndex));
  const int upper = std::min(lower + 1, beamCount - 1);
  const float fraction = beamIndex - lower;
  return beamwidths[lower] * (1.0f - fraction) +
         beamwidths[upper] * fraction;
}

float quantileInPlace(std::vector<float> &values, float fraction) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  const size_t index = static_cast<size_t>(std::floor(
      std::clamp(fraction, 0.0f, 1.0f) * (values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

struct FloorwardAperture {
  float maximum = -std::numeric_limits<float>::infinity();
  float minimum = std::numeric_limits<float>::infinity();
};

FloorwardAperture floorwardAperture(const cv::Vec3f &platformUp,
                                    float azimuth, float beamwidth) {
  FloorwardAperture aperture;
  // A small fixed angular grid is inexpensive (normally 512*65 evaluations
  // per ping) and avoids fragile branch-cut handling around the analytic
  // extrema of A*sin(e)+B*cos(e). The 20-degree Oculus aperture is sampled at
  // 0.3125 degree intervals, much finer than the range-domain detector.
  constexpr int kElevationIntervals = 64;
  for (int sample = 0; sample <= kElevationIntervals; ++sample) {
    const float elevation =
        -0.5f * beamwidth +
        beamwidth * static_cast<float>(sample) / kElevationIntervals;
    const float cosElevation = std::cos(elevation);
    const cv::Vec3f ray(std::sin(elevation),
                        -cosElevation * std::sin(azimuth),
                        cosElevation * std::cos(azimuth));
    // platformUp points away from the floor. A positive value therefore means
    // this ray travels toward it.
    const float floorward = -platformUp.dot(ray);
    aperture.maximum = std::max(aperture.maximum, floorward);
    aperture.minimum = std::min(aperture.minimum, floorward);
  }
  return aperture;
}

float columnMean(const std::vector<float> &prefix, int columns, int column,
                 int begin, int end) {
  if (begin < 0 || end <= begin) return 0.0f;
  return (prefix[static_cast<size_t>(end) * columns + column] -
          prefix[static_cast<size_t>(begin) * columns + column]) /
         static_cast<float>(end - begin);
}

}  // namespace

SonarDrawer::FloorEstimate SonarDrawer::estimateFloorPlaneFromImage(
    const AbstractSonarInterface &ping,
    const cv::Vec3f &platform_up_in_sensor,
    const std::vector<float> &elevationBeamwidths,
    const FloorDetectionConfig &config) const {
  FloorEstimate estimate;
  const int rangeCount = ping.nRanges();
  const int beamCount = ping.nAzimuth();
  if (!config.valid() || rangeCount < 8 || beamCount < 2 ||
      (elevationBeamwidths.size() != 1 &&
       elevationBeamwidths.size() != static_cast<size_t>(beamCount))) {
    return estimate;
  }

  const float normalMagnitude = cv::norm(platform_up_in_sensor);
  if (!std::isfinite(normalMagnitude) || normalMagnitude <= 1e-6f)
    return estimate;
  const cv::Vec3f platformUp = platform_up_in_sensor / normalMagnitude;

  bool rangesAscending = false;
  bool bearingsAscending = false;
  if (!validateCoordinateTable(ping.ranges(), rangesAscending) ||
      !rangesAscending ||
      !validateCoordinateTable(ping.azimuths(), bearingsAscending)) {
    return estimate;
  }

  for (const float beamwidth : elevationBeamwidths) {
    if (!std::isfinite(beamwidth) || beamwidth <= 0.0f ||
        beamwidth >= static_cast<float>(M_PI)) {
      return estimate;
    }
  }

  const int edgeWindow = config.edge_window_bins;
  if (rangeCount <= 2 * edgeWindow + 4) return estimate;
  const float rangeMinimum = std::max(
      config.minimum_range, ping.ranges()[static_cast<size_t>(edgeWindow)]);
  const float configuredMaximum =
      config.maximum_range > 0.0f ? config.maximum_range : ping.maxRange();
  const float rangeMaximum = std::min(
      configuredMaximum,
      ping.ranges()[static_cast<size_t>(rangeCount - edgeWindow - 1)]);
  if (!std::isfinite(rangeMinimum) || !std::isfinite(rangeMaximum) ||
      !(rangeMaximum > rangeMinimum)) {
    return estimate;
  }

  // Work in a monotone compressed intensity domain, then normalize each beam
  // independently. The vendor payload is highly skewed and individual receive
  // beams carry different gain; a global linear threshold makes the floor
  // disappear in quiet beams and lets one hot beam dominate the fit.
  cv::Mat normalized(rangeCount, beamCount, CV_32FC1);
  constexpr float kLogScale = 255.0f;
  const float inverseLogRange = 1.0f / std::log1p(kLogScale);
  for (int range = 0; range < rangeCount; ++range) {
    float *const row = normalized.ptr<float>(range);
    for (int beam = 0; beam < beamCount; ++beam) {
      float intensity = ping.intensity_float(
          AzimuthRangeIndices(static_cast<size_t>(beam),
                              static_cast<size_t>(range)));
      if (!std::isfinite(intensity)) intensity = 0.0f;
      intensity = std::clamp(intensity, 0.0f, 1.0f);
      row[beam] = std::log1p(kLogScale * intensity) * inverseLogRange;
    }
  }

  std::vector<float> samples(static_cast<size_t>(rangeCount));
  for (int beam = 0; beam < beamCount; ++beam) {
    for (int range = 0; range < rangeCount; ++range)
      samples[static_cast<size_t>(range)] =
          normalized.at<float>(range, beam);
    const float lower = quantileInPlace(samples, 0.10f);
    const float upper = quantileInPlace(samples, 0.95f);
    const float span = upper - lower;
    for (int range = 0; range < rangeCount; ++range) {
      float &value = normalized.at<float>(range, beam);
      value = span > 1e-6f
                  ? std::clamp((value - lower) / span, 0.0f, 1.0f)
                  : 0.0f;
    }
  }

  cv::Mat smoothed;
  cv::GaussianBlur(normalized, smoothed, cv::Size(), 3.0, 2.0,
                   cv::BORDER_REPLICATE);

  // Per-beam cumulative sums make the persistence and edge windows O(1) for
  // each plane candidate.
  std::vector<float> prefix(
      static_cast<size_t>(rangeCount + 1) * beamCount, 0.0f);
  for (int range = 0; range < rangeCount; ++range) {
    const float *const row = smoothed.ptr<float>(range);
    for (int beam = 0; beam < beamCount; ++beam) {
      prefix[static_cast<size_t>(range + 1) * beamCount + beam] =
          prefix[static_cast<size_t>(range) * beamCount + beam] + row[beam];
    }
  }

  std::vector<float> nearestFloorward(static_cast<size_t>(beamCount), -1.0f);
  std::vector<float> farthestFloorward(static_cast<size_t>(beamCount), 0.0f);
  float maximumFloorward = 0.0f;
  int reachableBeams = 0;
  for (int beam = 0; beam < beamCount; ++beam) {
    const float beamwidth = elevationBeamwidths.size() == 1
                                ? elevationBeamwidths.front()
                                : elevationBeamwidths[beam];
    const auto aperture =
        floorwardAperture(platformUp, ping.azimuth(beam), beamwidth);
    nearestFloorward[beam] = aperture.maximum;
    // When the aperture reaches or crosses the platform horizon, the floor
    // band has no finite far edge. This is expected for a level or shallow
    // head, not an error.
    farthestFloorward[beam] = aperture.minimum > 1e-5f
                                  ? aperture.minimum
                                  : 0.0f;
    if (aperture.maximum > 1e-5f) {
      maximumFloorward = std::max(maximumFloorward, aperture.maximum);
      ++reachableBeams;
    }
  }

  const int minimumSupport = std::max(
      2, static_cast<int>(std::ceil(config.minimum_support_fraction *
                                    static_cast<float>(beamCount))));
  if (reachableBeams < minimumSupport || !(maximumFloorward > 0.0f))
    return estimate;

  const float maximumDistance = rangeMaximum * maximumFloorward;
  if (!std::isfinite(maximumDistance) || !(maximumDistance > 0.0f))
    return estimate;

  float bestScore = -std::numeric_limits<float>::infinity();
  float bestDistance = 0.0f;
  float bestSupport = 0.0f;
  std::vector<float> beamScores;
  beamScores.reserve(static_cast<size_t>(beamCount));
  const float beforeExtent = 0.5f * config.persistence_range;
  const float afterInset = 0.125f * config.persistence_range;

  // One distance hypothesis per native range bin retains the sensor's radial
  // precision without turning this display path into an unbounded optimizer.
  for (int candidate = 1; candidate <= rangeCount; ++candidate) {
    const float distance = maximumDistance *
                           static_cast<float>(candidate) /
                           static_cast<float>(rangeCount);
    beamScores.clear();

    for (int beam = 0; beam < beamCount; ++beam) {
      const float qNear = nearestFloorward[beam];
      if (!(qNear > 1e-5f)) continue;
      const float entryRange = distance / qNear;
      if (!std::isfinite(entryRange) || entryRange < rangeMinimum ||
          entryRange > rangeMaximum) {
        continue;
      }

      float exitRange = rangeMaximum;
      const float qFar = farthestFloorward[beam];
      if (qFar > 1e-5f)
        exitRange = std::min(exitRange, distance / qFar);

      const float entryIndex = coordinateToIndex(
          ping.ranges(), entryRange, rangesAscending);
      if (!(entryIndex >= 0.0f)) continue;
      const int edgeIndex = static_cast<int>(std::lround(entryIndex));
      if (edgeIndex < edgeWindow ||
          edgeIndex + edgeWindow > rangeCount) {
        continue;
      }

      const float beforeStartRange =
          std::max(ping.minRange(), entryRange - beforeExtent);
      const float afterStartRange = entryRange + afterInset;
      const float afterEndRange = std::min(
          {exitRange, entryRange + afterInset + config.persistence_range,
           rangeMaximum});
      if (!(afterEndRange > afterStartRange)) continue;

      const float beforeStartIndex = coordinateToIndex(
          ping.ranges(), beforeStartRange, rangesAscending);
      const float afterStartIndex = coordinateToIndex(
          ping.ranges(), afterStartRange, rangesAscending);
      const float afterEndIndex = coordinateToIndex(
          ping.ranges(), afterEndRange, rangesAscending);
      if (beforeStartIndex < 0.0f || afterStartIndex < 0.0f ||
          afterEndIndex < 0.0f) {
        continue;
      }

      const int beforeBegin = std::clamp(
          static_cast<int>(std::floor(beforeStartIndex)), 0, rangeCount);
      const int beforeEnd = std::clamp(
          static_cast<int>(std::ceil(entryIndex)), 0, rangeCount);
      const int afterBegin = std::clamp(
          static_cast<int>(std::floor(afterStartIndex)), 0, rangeCount);
      const int afterEnd = std::clamp(
          static_cast<int>(std::ceil(afterEndIndex)), 0, rangeCount);
      if (beforeEnd - beforeBegin < 4 || afterEnd - afterBegin < 4)
        continue;

      const float edgeBefore = columnMean(
          prefix, beamCount, beam, edgeIndex - edgeWindow, edgeIndex);
      const float edgeAfter = columnMean(
          prefix, beamCount, beam, edgeIndex, edgeIndex + edgeWindow);
      const float before = columnMean(prefix, beamCount, beam, beforeBegin,
                                      beforeEnd);
      const float after = columnMean(prefix, beamCount, beam, afterBegin,
                                     afterEnd);
      const float edgeContrast = edgeAfter - edgeBefore;
      const float persistentContrast = after - before;
      beamScores.push_back(0.35f * edgeContrast +
                           0.65f * persistentContrast);
    }

    if (static_cast<int>(beamScores.size()) < minimumSupport) continue;
    const float geometricSupport =
        static_cast<float>(beamScores.size()) / beamCount;

    // Score the weakest member of the best `minimumSupport` beams. This makes
    // the configured support fraction mean what it says. The old fixed 20th
    // percentile required roughly 80% of every wide ping to agree even though
    // the deployed parameter promised 25%; real St. Louis floor returns cover
    // the central fan strongly while pilings occupy the outer beams.
    const auto supportRank =
        beamScores.begin() + (beamScores.size() - minimumSupport);
    std::nth_element(beamScores.begin(), supportRank, beamScores.end());
    const float robustContrast = *supportRank;
    // Agreement must mean support for this robust hypothesis, not merely a
    // value infinitesimally above zero. Thin rails and numerical interpolation
    // produce weak positive contrast in otherwise empty beams; counting those
    // made a 55%-wide floor report 92% support and defeated the configured
    // coherence gate. A quarter of the robust rank admits the weaker shoulders
    // of the same coherent lobe without counting those near-zero distractors.
    constexpr float kSupportContrastFraction = 0.25f;
    const float supportContrast =
        robustContrast * kSupportContrastFraction;
    const int supportingBeams = robustContrast > 0.0f ?
        static_cast<int>(std::count_if(
            beamScores.begin(), beamScores.end(),
            [supportContrast](float value) { return value >= supportContrast; })) :
        0;
    const float evidenceSupport =
        static_cast<float>(supportingBeams) / beamCount;
    const float score = robustContrast * std::sqrt(geometricSupport);
    if (std::isfinite(score) && score > bestScore) {
      bestScore = score;
      bestDistance = distance;
      bestSupport = evidenceSupport;
    }
  }

  if (!std::isfinite(bestScore)) return estimate;
  estimate.plane.normal = platformUp;
  estimate.plane.offset = bestDistance;
  estimate.distance = bestDistance;
  estimate.score = bestScore;
  estimate.support_fraction = bestSupport;
  estimate.detected = bestScore >= config.minimum_score &&
                      bestSupport >= config.minimum_support_fraction;
  return estimate;
}

cv::Mat SonarDrawer::projectOntoPlaneImage(
    const AbstractSonarInterface &ping, const cv::Mat &rangeBearingImage,
    const PinholeGeometry &camera,
    const RigidTransform &sensorFromCamera, const Plane &planeInSensor,
    const std::vector<float> &elevationBeamwidths) const {
  cv::Mat output;
  if (!camera.valid() || !finiteTransform(sensorFromCamera) ||
      rangeBearingImage.empty() ||
      rangeBearingImage.rows != ping.nAzimuth() ||
      rangeBearingImage.cols != ping.nRanges() || ping.nAzimuth() < 2 ||
      ping.nRanges() < 2 ||
      (elevationBeamwidths.size() != 1 &&
       elevationBeamwidths.size() != static_cast<size_t>(ping.nAzimuth()))) {
    return output;
  }

  const float normalMagnitude = cv::norm(planeInSensor.normal);
  if (!std::isfinite(normalMagnitude) || normalMagnitude <= 1e-6f ||
      !std::isfinite(planeInSensor.offset)) {
    return output;
  }
  const cv::Vec3f planeNormal = planeInSensor.normal / normalMagnitude;
  const float planeOffset = planeInSensor.offset / normalMagnitude;
  const float originSide =
      planeNormal.dot(sensorFromCamera.translation) + planeOffset;
  if (!std::isfinite(originSide)) return output;

  for (const float beamwidth : elevationBeamwidths) {
    if (!std::isfinite(beamwidth) || beamwidth <= 0.0f ||
        beamwidth >= static_cast<float>(M_PI)) {
      return output;
    }
  }

  cv::Mat rangeMap(camera.height, camera.width, CV_32FC1,
                   cv::Scalar(-1.0f));
  cv::Mat bearingMap(camera.height, camera.width, CV_32FC1,
                     cv::Scalar(-1.0f));
  const float maximumRange = effectiveMaxRange(ping);
  const auto rangeBounds = ping.rangeBounds();
  bool rangesAscending = false;
  bool bearingsAscending = false;
  if (!validateCoordinateTable(ping.ranges(), rangesAscending) ||
      !validateCoordinateTable(ping.azimuths(), bearingsAscending)) {
    return output;
  }

  cv::parallel_for_(cv::Range(0, camera.height), [&](const cv::Range &rows) {
    for (int v = rows.start; v < rows.end; ++v) {
      float *const rangeRow = rangeMap.ptr<float>(v);
      float *const bearingRow = bearingMap.ptr<float>(v);
      const float cameraY = (v - camera.cy) / camera.fy;
      for (int u = 0; u < camera.width; ++u) {
        const cv::Vec3f cameraRay((u - camera.cx) / camera.fx, cameraY,
                                  1.0f);
        const cv::Vec3f sensorRay =
            sensorFromCamera.rotation * cameraRay;
        const float denominator = planeNormal.dot(sensorRay);
        if (!std::isfinite(denominator) || std::abs(denominator) <= 1e-7f)
          continue;

        const float rayScale = -originSide / denominator;
        if (!std::isfinite(rayScale) || rayScale <= 0.0f) continue;
        const cv::Vec3f point =
            sensorFromCamera.translation + sensorRay * rayScale;
        const float range = cv::norm(point);
        if (!std::isfinite(range) || range < rangeBounds.first ||
            range > maximumRange) {
          continue;
        }

        // sonar projection frame convention: +x elevation/down, +y left,
        // +z forward. Bearing is therefore atan2(-y, z).
        const float horizontalRange = std::hypot(point[1], point[2]);
        const float bearing = std::atan2(-point[1], point[2]);
        const float elevation = std::atan2(point[0], horizontalRange);
        const float bearingIndex = coordinateToIndex(
            ping.azimuths(), bearing, bearingsAscending);
        if (bearingIndex < 0.0f) continue;
        const float beamwidth =
            beamwidthAt(elevationBeamwidths, bearingIndex, ping.nAzimuth());
        if (!(beamwidth > 0.0f) ||
            std::abs(elevation) > 0.5f * beamwidth + 1e-6f) {
          continue;
        }

        const float rangeIndex =
            coordinateToIndex(ping.ranges(), range, rangesAscending);
        if (rangeIndex < 0.0f) continue;
        rangeRow[u] = rangeIndex;
        bearingRow[u] = bearingIndex;
      }
    }
  });

  cv::remap(rangeBearingImage, output, rangeMap, bearingMap, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar::all(0));
  return output;
}

SonarDrawer::CachedRectifiedMap::MapPair
SonarDrawer::CachedRectifiedMap::operator()(
    const AbstractSonarInterface &ping,
    const RectifiedGeometry &geometry) {
  for (auto &entry : _entries) {
    if (entry.isValidFor(ping, geometry))
      return std::make_pair(entry._map1, entry._map2);
  }

  Entry &entry = _entries[_nextEvict];
  _nextEvict = (_nextEvict + 1) % kNumEntries;
  entry.create(ping, geometry);
  return std::make_pair(entry._map1, entry._map2);
}

bool SonarDrawer::CachedRectifiedMap::Entry::isValidFor(
    const AbstractSonarInterface &ping,
    const RectifiedGeometry &geometry) const {
  return !_map1.empty() && !_map2.empty() &&
         sameRectifiedGeometry(_geometry, geometry) &&
         _ranges == ping.ranges() && _azimuths == ping.azimuths();
}

void SonarDrawer::CachedRectifiedMap::Entry::create(
    const AbstractSonarInterface &ping,
    const RectifiedGeometry &geometry) {
  _map1.release();
  _map2.release();
  _geometry = RectifiedGeometry();
  _ranges.clear();
  _azimuths.clear();
  if (!geometry.valid() || ping.nRanges() < 2 || ping.nAzimuth() < 2)
    return;

  bool rangesAscending = false;
  bool bearingsAscending = false;
  if (!validateCoordinateTable(ping.ranges(), rangesAscending) ||
      !validateCoordinateTable(ping.azimuths(), bearingsAscending)) {
    return;
  }

  std::vector<float> rangeIndices(geometry.height, -1.0f);
  for (int v = 0; v < geometry.height; ++v) {
    const float range = geometry.max_range - v * geometry.meters_per_row;
    rangeIndices[v] =
        coordinateToIndex(ping.ranges(), range, rangesAscending);
  }

  std::vector<float> bearingIndices(geometry.width, -1.0f);
  for (int u = 0; u < geometry.width; ++u) {
    const float bearing = std::atan(
        (u - geometry.principal_point_u) /
        geometry.horizontal_focal_length);
    bearingIndices[u] =
        coordinateToIndex(ping.azimuths(), bearing, bearingsAscending);
  }

  // Source layout is [bearing row, range column]. cv::remap therefore wants
  // (source range index, source bearing index) at every destination pixel.
  cv::Mat floatingMap(geometry.height, geometry.width, CV_32FC2);
  for (int v = 0; v < geometry.height; ++v) {
    cv::Vec2f *const row = floatingMap.ptr<cv::Vec2f>(v);
    for (int u = 0; u < geometry.width; ++u)
      row[u] = cv::Vec2f(rangeIndices[v], bearingIndices[u]);
  }
  cv::convertMaps(floatingMap, cv::Mat(), _map1, _map2, CV_16SC2);

  _geometry = geometry;
  _ranges = ping.ranges();
  _azimuths = ping.azimuths();
}

// === SonarDrawer::CachedOverlay ===

bool SonarDrawer::CachedOverlay::isValidFor(const AbstractSonarInterface &ping,
                                            const cv::Mat &sonarImage,
                                            const OverlayConfig &config,
                                            float maxRange) const {
  if (sonarImage.size() != _source_size) return false;

  if (_config_used != config) return false;
  if (_maxRange != maxRange) return false;

  return Cached::isValid(ping);
}

const cv::Mat &SonarDrawer::CachedOverlay::operator()(
    const AbstractSonarInterface &ping, const cv::Mat &sonarImage,
    const OverlayConfig &config, float maxRange) {
  if (!isValidFor(ping, sonarImage, config, maxRange))
    create(ping, sonarImage, config, maxRange);

  return _overlay;
}

// Converts sonar bearing (with sonar "forward" at bearing 0) to image
// orientation with sonar "forward" point upward in the image, which is the -Y
// direction in image coordinates.
static float bearingToImage(float d) { return (-M_PI / 2) + d; }

void SonarDrawer::CachedOverlay::create(const AbstractSonarInterface &ping,
                                        const cv::Mat &sonarImage,
                                        const OverlayConfig &config,
                                        float maxRange) {
  const cv::Size sz(sonarImage.size());
  _overlay = cv::Mat::zeros(sz, CV_8UC4);
  _source_size = sz;
  _image_origin = cv::Point(0, 0);
  if (sz.width <= 0 || sz.height <= 0 || !std::isfinite(maxRange) ||
      maxRange <= 0.0f) {
    return;
  }

  // Same origin_x formula as CachedMap::Entry::create — the old width/2
  // coincides with it only for a symmetric fan, so the overlay arcs were
  // drawn about the wrong apex on any asymmetric crop.
  const int originx = abs(
      static_cast<int>(floor(sz.height * sin(ping.minAzimuth()))));
  const cv::Point2f sourceOrigin(originx, sz.height);

  const cv::Vec3b color(config.lineColor());
  const cv::Scalar textColor(color[0], color[1], color[2], 255);
  const cv::Vec4b lineColor(color[0], color[1], color[2],
                            cv::saturate_cast<uint8_t>(
                                std::clamp(config.lineAlpha(), 0.0f, 1.0f) *
                                255.0f));

  const float minAzimuth = ping.minAzimuth();
  const float maxAzimuth = ping.maxAzimuth();

  //== Draw arcs ==
  float arcSpacing = config.rangeSpacing();

  if (!std::isfinite(arcSpacing) || arcSpacing <= 0) {
    // Calculate automatically .. just a lame heuristic for now
    if (maxRange <= 2)
      arcSpacing = 0.5;
    else if (maxRange <= 5)
      arcSpacing = 1.0;
    else if (maxRange <= 10)
      arcSpacing = 2.0;
    else if (maxRange < 50)
      arcSpacing = 10.0;
    else
      arcSpacing = 20.0;
  }

  std::vector<float> arcRanges;
  for (float r = arcSpacing; r < maxRange; r += arcSpacing)
    arcRanges.push_back(r);
  // The displayed maximum is operationally the most important range value.
  // Draw and label it even when it is not an even multiple of the spacing.
  arcRanges.push_back(maxRange);

  //== Draw radials ==
  std::vector<float> radials;

  // Configuration ... move later
  const float radialSpacing = deg2radf(config.radialSpacing());

  radials.push_back(minAzimuth);
  radials.push_back(maxAzimuth);

  // If radialSpacing == 0. draw only the outline radial lines
  if (radialSpacing > 0) {
    if (config.radialAtZero()) {
      if (minAzimuth < 0.0f && maxAzimuth > 0.0f) radials.push_back(0.0f);
      for (float d = radialSpacing; d > minAzimuth && d < maxAzimuth;
           d += radialSpacing) {
        radials.push_back(d);
      }
      for (float d = -radialSpacing; d > minAzimuth && d < maxAzimuth;
           d -= radialSpacing) {
        radials.push_back(d);
      }
    } else {
      for (float d = radialSpacing / 2; d > minAzimuth && d < maxAzimuth;
           d += radialSpacing) {
        radials.push_back(d);
      }
      for (float d = -radialSpacing / 2; d > minAzimuth && d < maxAzimuth;
           d -= radialSpacing) {
        radials.push_back(d);
      }
    }
  }

  // Sort and unique
  std::sort(radials.begin(), radials.end());
  auto last = std::unique(radials.begin(), radials.end(),
                          [](float lhs, float rhs) {
                            return std::abs(lhs - rhs) < 1e-5f;
                          });
  radials.erase(last, radials.end());

  struct Label {
    std::string text;
    cv::Point2f center;
  };
  std::vector<Label> rangeLabels;
  std::vector<Label> bearingLabels;
  rangeLabels.reserve(arcRanges.size());
  bearingLabels.reserve(radials.size());

  // Range labels live just beyond the low-bearing edge of the fan. Their
  // anchor has the same radius as the arc it names, while a perpendicular
  // offset keeps every glyph outside the measured cone. The short exterior
  // tick makes that registration legible at a glance.
  const float rangeTheta = bearingToImage(minAzimuth);
  const cv::Point2f rangeRay(std::cos(rangeTheta), std::sin(rangeTheta));
  const cv::Point2f rangeOutward(std::sin(rangeTheta), -std::cos(rangeTheta));
  for (const float r : arcRanges) {
    const float radiusPix = (r / maxRange) * sz.height;
    const std::string text = rangeLabel(r, arcSpacing);
    const float clearance =
        labelClearance(text, config.fontScale(), config.lineThickness());
    rangeLabels.push_back(
        {text, sourceOrigin + radiusPix * rangeRay + clearance * rangeOutward});
  }

  // Bearing labels sit just past the outer range arc, exactly on the ray they
  // name. This preserves their degree-to-grid correspondence for both
  // symmetric and cropped/asymmetric fans.
  for (const auto b : radials) {
    const float theta = bearingToImage(b);
    const cv::Point2f ray(std::cos(theta), std::sin(theta));
    const std::string text = bearingLabel(b);
    const float clearance =
        labelClearance(text, config.fontScale(), config.lineThickness());
    bearingLabels.push_back({text, sourceOrigin + (sz.height + clearance) * ray});
  }

  // Find the tight display canvas that holds the original fan and every
  // external label. This avoids clipping (and therefore avoids moving labels
  // away from their grid markers) even for wide or asymmetric field-of-view
  // pings.
  cv::Rect bounds(0, 0, sz.width, sz.height);
  const auto includeLabel = [&](const Label &label) {
    bounds |= textBounds(label.text, label.center, config.fontScale(),
                         config.lineThickness());
  };
  for (const auto &label : rangeLabels) includeLabel(label);
  for (const auto &label : bearingLabels) includeLabel(label);

  constexpr int kCanvasPadding = 4;
  bounds.x -= kCanvasPadding;
  bounds.y -= kCanvasPadding;
  bounds.width += 2 * kCanvasPadding;
  bounds.height += 2 * kCanvasPadding;
  _image_origin = cv::Point(-bounds.x, -bounds.y);
  _overlay = cv::Mat::zeros(bounds.size(), CV_8UC4);
  const cv::Point2f origin = sourceOrigin +
                             cv::Point2f(_image_origin.x, _image_origin.y);

  const auto drawRangeArc = [&](float r) {
    const float radiusPix = (r / maxRange) * sz.height;
    cv::ellipse(_overlay, origin, cv::Size(radiusPix, radiusPix), 0,
                rad2degf(bearingToImage(minAzimuth)),
                rad2degf(bearingToImage(maxAzimuth)), lineColor,
                config.lineThickness());
  };
  for (const float r : arcRanges) drawRangeArc(r);

  constexpr float kLabelTickLength = 8.0f;
  for (const auto b : radials) {
    const float theta = bearingToImage(b);
    const cv::Point2f ray(std::cos(theta), std::sin(theta));
    const cv::Point2f end = origin + sz.height * ray;
    cv::line(_overlay, origin, end, lineColor, config.lineThickness());
    cv::line(_overlay, end, end + kLabelTickLength * ray, lineColor,
             config.lineThickness());
  }

  for (const auto &label : rangeLabels) {
    const cv::Point2f marker = sourceOrigin +
                               ((label.center - sourceOrigin).dot(rangeRay)) *
                                   rangeRay;
    const cv::Point2f tickEnd =
        marker + kLabelTickLength * rangeOutward +
        cv::Point2f(_image_origin.x, _image_origin.y);
    cv::line(_overlay, marker + cv::Point2f(_image_origin.x, _image_origin.y),
             tickEnd, lineColor, config.lineThickness());
    putOutlinedText(_overlay, label.text,
                    label.center + cv::Point2f(_image_origin.x,
                                                _image_origin.y),
                    config.fontScale(), textColor, config.lineThickness());
  }
  for (const auto &label : bearingLabels) {
    putOutlinedText(_overlay, label.text,
                    label.center + cv::Point2f(_image_origin.x,
                                                _image_origin.y),
                    config.fontScale(), textColor, config.lineThickness());
  }

  _config_used = config;
  _maxRange = maxRange;
  _numRanges = ping.nRanges();
  _numAzimuth = ping.nBearings();
  _rangeBounds = ping.rangeBounds();
  _azimuthBounds = ping.azimuthBounds();
}

}  // namespace sonar_image_proc
