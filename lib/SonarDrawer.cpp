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

  const int maxX = std::max(0, image.cols - textSize.width - 1);
  const int minY = std::min(image.rows - 1, textSize.height + 1);
  const int maxY = std::max(minY, image.rows - baseline - 1);
  const cv::Point origin(
      std::clamp(cvRound(center.x - textSize.width / 2.0f), 0, maxX),
      std::clamp(cvRound(center.y + textSize.height / 2.0f), minY, maxY));

  // A dark halo keeps white annotations legible over strong returns without
  // hiding a rectangular patch of sonar data behind each label.
  cv::putText(image, text, origin, cv::FONT_HERSHEY_PLAIN, fontScale,
              cv::Scalar(0, 0, 0, 230), textThickness + 2, cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_PLAIN, fontScale, color,
              textThickness, cv::LINE_AA);
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

  // Alpha blend overlay onto sonarImage
  cv::Mat output;
  overlayImage<unsigned char>(
      sonarImage,
      _overlay(ping, sonarImage, overlayConfig(), effectiveMaxRange(ping)),
      output);

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
  const float rangeSpan = sourceMaxRange - minRange;
  const float lastColumn = ping.nRanges() - 1;
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
        const float rangeFraction = (rangeInMeters - minRange) / rangeSpan;
        // The first and last range values correspond to source columns 0 and
        // n-1. Multiplying by n mapped maxRange one column past the image,
        // blackening the outer edge and shifting every intermediate sample.
        xp = rangeFraction * lastColumn;
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
}

bool SonarDrawer::CachedMap::Entry::isValidFor(
    const AbstractSonarInterface &ping, float pixelsPerMeter,
    float maxRange) const {
  if (_scMap1.empty() || _scMap2.empty()) return false;

  // Check if pixels per meter has changed
  if (_pixelsPerMeter != pixelsPerMeter) return false;
  if (_maxRange != maxRange) return false;
  if (_azimuths != ping.azimuths()) return false;

  return Cached::isValid(ping);
}

// === SonarDrawer::CachedOverlay ===

bool SonarDrawer::CachedOverlay::isValidFor(const AbstractSonarInterface &ping,
                                            const cv::Mat &sonarImage,
                                            const OverlayConfig &config,
                                            float maxRange) const {
  if (sonarImage.size() != _overlay.size()) return false;

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
  if (sz.width <= 0 || sz.height <= 0 || !std::isfinite(maxRange) ||
      maxRange <= 0.0f) {
    return;
  }

  // Same origin_x formula as CachedMap::Entry::create — the old width/2
  // coincides with it only for a symmetric fan, so the overlay arcs were
  // drawn about the wrong apex on any asymmetric crop.
  const int originx = abs(
      static_cast<int>(floor(sz.height * sin(ping.minAzimuth()))));
  const cv::Point2f origin(originx, sz.height);

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

  // Put range values on boresight whenever it is visible. Bearing values live
  // around the outer arc, so the two scales remain visually distinct. For a
  // cropped fan that excludes zero, use its angular midpoint instead.
  const float rangeLabelBearing =
      (minAzimuth <= 0.0f && maxAzimuth >= 0.0f)
          ? 0.0f
          : (minAzimuth + maxAzimuth) / 2.0f;
  const auto drawRangeArc = [&](float r) {
    const float radiusPix = (r / maxRange) * sonarImage.size().height;

    cv::ellipse(_overlay, origin, cv::Size(radiusPix, radiusPix), 0,
                rad2degf(bearingToImage(minAzimuth)),
                rad2degf(bearingToImage(maxAzimuth)), lineColor,
                config.lineThickness());

    // Inset range text from its arc enough to separate the maximum-range value
    // from the 0-degree label. Text placement is also clamped, which keeps it
    // visible on asymmetric/cropped fans.
    const float labelInset = std::max(32.0f, 45.0f * config.fontScale());
    const float labelRadius = std::max(0.0f, radiusPix - labelInset);
    const float theta = bearingToImage(rangeLabelBearing);
    const cv::Point2f labelCenter(
        labelRadius * cos(theta) + origin.x,
        labelRadius * sin(theta) + origin.y);
    putOutlinedText(_overlay, rangeLabel(r, arcSpacing), labelCenter,
                    config.fontScale(), textColor, config.lineThickness());
  };

  for (float r = arcSpacing; r < maxRange; r += arcSpacing) {
    drawRangeArc(r);
  }

  // The displayed maximum is operationally the most important range value.
  // Draw and label it even when it is not an even multiple of the spacing.
  drawRangeArc(maxRange);

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

  // Draw full bearing rays from the sonar origin, then label them just inside
  // the maximum-range arc. This makes the image self-describing even in a
  // generic image viewer or a screenshot.
  for (const auto b : radials) {
    const float theta = bearingToImage(b);
    const cv::Point2f end(sz.height * cos(theta) + origin.x,
                          sz.height * sin(theta) + origin.y);
    cv::line(_overlay, origin, end, lineColor, config.lineThickness());

    const float labelRadius = std::max(0.0f, sz.height - 18.0f);
    const cv::Point2f labelCenter(labelRadius * cos(theta) + origin.x,
                                  labelRadius * sin(theta) + origin.y);
    putOutlinedText(_overlay, bearingLabel(b), labelCenter,
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
