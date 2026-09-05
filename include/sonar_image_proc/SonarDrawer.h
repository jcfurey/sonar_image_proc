// Copyright 2021 University of Washington Applied Physics Laboratory
//
// This file contains the class-based API (which is more efficient)
// as it can store and reuse intermediate results.
//
// See "DrawSonar.h" for the function-based API

#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>

#include "sonar_image_proc/AbstractSonarInterface.h"
#include "sonar_image_proc/ColorMaps.h"

namespace sonar_image_proc {

using sonar_image_proc::AbstractSonarInterface;

// Function in lib/OverlayImage.cpp
void overlayImage(const cv::Mat &background, const cv::Mat &foreground,
                  cv::Mat &output);

#define OVERLAY_RW(var, tp, set, get) \
  OverlayConfig &set(tp i) {          \
    var = i;                          \
    return *this;                     \
  }                                   \
  tp get() const { return var; }

class SonarDrawer {
 public:
  struct OverlayConfig {
   public:
    int DEFAULT_LINE_THICKNESS = 1;
    float DEFAULT_LINE_ALPHA = 0.5;

    // range spacing of 0 means "calculate automatically"
    float DEFAULT_RANGE_SPACING = 0;

    float DEFAULT_RADIAL_SPACING = 20;  // degrees
    bool DEFAULT_RADIAL_AT_ZERO = true;

    float DEFAULT_FONT_SCALE = 1.0;

    OverlayConfig()
        : line_thickness_(DEFAULT_LINE_THICKNESS),
          line_alpha_(DEFAULT_LINE_ALPHA),
          range_spacing_m_(DEFAULT_RANGE_SPACING),
          radial_spacing_deg_(DEFAULT_RADIAL_SPACING),
          radial_at_zero_(DEFAULT_RADIAL_AT_ZERO),
          font_scale_(DEFAULT_FONT_SCALE),
          line_color_(255, 255, 255) {}

    OVERLAY_RW(line_alpha_, float, setLineAlpha, lineAlpha)
    OVERLAY_RW(line_thickness_, int, setLineThickness, lineThickness)
    OVERLAY_RW(range_spacing_m_, float, setRangeSpacing, rangeSpacing)

    OVERLAY_RW(radial_spacing_deg_, float, setRadialSpacing, radialSpacing)
    OVERLAY_RW(radial_at_zero_, bool, setRadialAtZero, radialAtZero)
    OVERLAY_RW(font_scale_, float, setFontScale, fontScale)

    OverlayConfig &setLineColor(const cv::Vec3b &v) {
      line_color_ = v;
      return *this;
    }
    cv::Vec3b lineColor() const { return line_color_; }

    bool operator!=(const OverlayConfig &other) const {
      return (lineThickness() != other.lineThickness()) ||
             (lineAlpha() != other.lineAlpha()) ||
             (rangeSpacing() != other.rangeSpacing()) ||
             (radialSpacing() != other.radialSpacing()) ||
             (radialAtZero() != other.radialAtZero()) ||
             (fontScale() != other.fontScale()) ||
             (lineColor() != other.lineColor());
    }

   private:
    int line_thickness_;
    float line_alpha_;
    float range_spacing_m_;
    float radial_spacing_deg_;
    bool radial_at_zero_;
    float font_scale_;

    cv::Vec3b line_color_;
  };

  SonarDrawer();

  // Set the scale factor for the output fan image, in pixels per meter.
  //
  // A non-positive value selects "native": scale each ping so one output pixel
  // along the radius corresponds to one range bin, i.e. ppm = nRanges /
  // maxRange. This is usually what you want, because the sonar changes its
  // range resolution with the commanded range -- an M3000d at a 1 m range
  // produces ~2 mm bins, and rendering those at a fixed 100 px/m discards
  // about 80% of the radial resolution, while at 5 m the bins are ~8 mm and
  // 100 px/m is about right. Native tracks it automatically.
  //
  // Note the output height in native mode is simply nRanges, so image size is
  // bounded by the ping rather than by the commanded range.
  void setPixelsPerMeter(float ppm) { pixels_per_meter_ = ppm; }
  float pixelsPerMeter() const { return pixels_per_meter_; }

  // Upper bound applied to the native scale, in pixels per meter.
  // Non-positive means unbounded. Only meaningful when pixelsPerMeter() is
  // non-positive; an explicit scale is always honoured as given.
  void setMaxPixelsPerMeter(float ppm) { max_pixels_per_meter_ = ppm; }
  float maxPixelsPerMeter() const { return max_pixels_per_meter_; }

  // The scale actually used for this ping: the explicit value when one is
  // set, otherwise the ping's native radial sampling subject to
  // maxPixelsPerMeter().
  float effectivePixelsPerMeter(const AbstractSonarInterface &ping) const;

  // Pixel geometry of the Cartesian fan for this ping. The sonar origin sits
  // at (origin_x, height) -- range zero is the bottom edge.
  //
  // Exposed so consumers can be told the pixel<->metre mapping instead of
  // assuming it. They used to assume: sonar_optical_flow carried its own
  // pixels_per_meter and divided pixel displacement by it, which was silently
  // wrong by up to 5x once the fan started being scaled per ping.
  struct FanGeometry {
    int width = 0;
    int height = 0;
    int origin_x = 0;
    float pixels_per_meter = 0.0f;
  };
  FanGeometry fanImageGeometry(const AbstractSonarInterface &ping) const;

  // Geometry of the loss-minimized rectangular range-bearing product.
  //
  // Horizontal pixels use a rectilinear (pinhole-style) angular axis:
  //
  //   u = horizontal_focal_length * tan(bearing) + principal_point_u
  //
  // Vertical pixels retain the sonar's measured range rather than inventing
  // an elevation that the sensor cannot observe:
  //
  //   range = max_range - v * meters_per_row
  //
  // This makes the image look and behave like a conventional forward-facing
  // rectangular raster while preserving every range bin. It is deliberately
  // not sensor_msgs/CameraInfo geometry: a camera's v coordinate is elevation,
  // whereas this image's v coordinate is range.
  struct RectifiedGeometry {
    int width = 0;
    int height = 0;
    float horizontal_focal_length = 0.0f;
    float principal_point_u = 0.0f;
    float meters_per_row = 0.0f;
    float min_range = 0.0f;
    float max_range = 0.0f;
    float min_bearing = 0.0f;
    float max_bearing = 0.0f;

    bool valid() const {
      return width >= 2 && height >= 2 &&
             horizontal_focal_length > 0.0f && meters_per_row > 0.0f;
    }
  };

  // A zero output dimension selects an automatic value. The automatic height
  // retains the native radial sample density (and follows maxRange()); the
  // automatic width is height * aspectRatio. The default is therefore a 16:9
  // operator/CV image with native radial sampling and a derived angular width.
  RectifiedGeometry rectifiedImageGeometry(
      const AbstractSonarInterface &ping, int outputWidth = 0,
      int outputHeight = 0, float aspectRatio = 16.0f / 9.0f) const;

  // Resample the unrotated range-bearing raster returned by
  // drawRectSonarImage() into RectifiedGeometry. This goes directly from the
  // source ping raster; it must not be fed the Cartesian fan, which would add
  // a lossy inverse remap.
  cv::Mat rectifyRangeBearingImage(
      const AbstractSonarInterface &ping, const cv::Mat &rangeBearingImage,
      const RectifiedGeometry &geometry);

  // A genuine pinhole raster used for surface-aware projection. Unlike
  // RectifiedGeometry, both image coordinates are angular camera coordinates:
  //
  //   u = fx * camera_x / camera_z + cx
  //   v = fy * camera_y / camera_z + cy
  struct PinholeGeometry {
    int width = 0;
    int height = 0;
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;

    bool valid() const {
      return width >= 2 && height >= 2 && std::isfinite(fx) &&
             std::isfinite(fy) && std::isfinite(cx) && std::isfinite(cy) &&
             fx > 0.0f && fy > 0.0f;
    }
  };

  // sensor_from_camera: point_sensor = rotation * point_camera + translation.
  // Keeping this ROS-independent makes the projection geometry testable
  // without a TF graph.
  struct RigidTransform {
    cv::Matx33f rotation = cv::Matx33f::eye();
    cv::Vec3f translation{0.0f, 0.0f, 0.0f};
  };

  // Plane equation in the sonar projection frame:
  //
  //   normal.dot(point) + offset = 0
  struct Plane {
    cv::Vec3f normal{0.0f, 0.0f, 0.0f};
    float offset = 0.0f;
  };

  // Image-domain floor detector. The caller supplies only the orientation of
  // the platform-up normal in the sonar projection frame; the plane standoff
  // is measured from the ping itself. This deliberately separates the two
  // observables:
  //
  //   * pivot-head/TF geometry determines which elevation half can see down;
  //   * the coherent floor-reverberation onset determines plane distance.
  //
  // persistence_range rejects thin rails, walls and electronic rings that can
  // make a strong edge but do not remain bright through the range band swept
  // out by the transmitted elevation aperture.
  struct FloorDetectionConfig {
    float minimum_range = 0.2f;
    float maximum_range = 0.0f;  // non-positive uses the complete ping
    float minimum_score = 0.05f;
    float minimum_support_fraction = 0.40f;
    float persistence_range = 0.50f;
    int edge_window_bins = 5;

    bool valid() const {
      return std::isfinite(minimum_range) && minimum_range >= 0.0f &&
             std::isfinite(maximum_range) &&
             std::isfinite(minimum_score) && minimum_score >= 0.0f &&
             std::isfinite(minimum_support_fraction) &&
             minimum_support_fraction > 0.0f &&
             minimum_support_fraction <= 1.0f &&
             std::isfinite(persistence_range) && persistence_range > 0.0f &&
             edge_window_bins >= 1;
    }
  };

  struct FloorEstimate {
    Plane plane;
    float distance = 0.0f;
    float score = 0.0f;
    float support_fraction = 0.0f;
    bool detected = false;
  };

  // Detect the near edge of the persistent floor-reverberation band predicted
  // by platform_up_in_sensor and each receive beam's transmit/elevation
  // aperture. Returned plane normal is unit length and its positive offset is
  // the image-derived perpendicular sensor-to-floor distance. No DVL altitude
  // or DVL surface frame participates.
  FloorEstimate estimateFloorPlaneFromImage(
      const AbstractSonarInterface &ping,
      const cv::Vec3f &platform_up_in_sensor,
      const std::vector<float> &elevationBeamwidths,
      const FloorDetectionConfig &config) const;

  // Inverse-project every destination camera ray onto a measured/assumed
  // surface plane, then sample the source ping at the resulting physical
  // range and bearing. The candidate point is accepted only when its inferred
  // elevation lies inside that beam's transmitted vertical aperture.
  //
  // This is a real perspective projection and removes the circular bow caused
  // by using slant range as an image row. Its semantics are explicitly tied to
  // the supplied plane: callers must not present it as scene-wide elevation
  // truth when returns may stand above or below that surface.
  cv::Mat projectOntoPlaneImage(
      const AbstractSonarInterface &ping,
      const cv::Mat &rangeBearingImage, const PinholeGeometry &camera,
      const RigidTransform &sensorFromCamera, const Plane &planeInSensor,
      const std::vector<float> &elevationBeamwidths) const;

  // Limit the Cartesian fan to this range in meters. A non-positive value
  // uses the full range reported by the ping. The rectangular source image is
  // intentionally unaffected.
  void setMaxRange(float max_range) { max_range_ = max_range; }
  float maxRange() const { return max_range_; }
  float effectiveMaxRange(const AbstractSonarInterface &ping) const;

  // Calls drawRectSonarImage followed by remapRectSonarImage inline
  // The intermediate rectangular image is not returned, if required,
  // use the two functions individually...
  cv::Mat drawSonar(const AbstractSonarInterface &ping,
                    const SonarColorMap &colorMap = InfernoColorMap(),
                    const cv::Mat &image = cv::Mat(0, 0, CV_8UC3),
                    bool addOverlay = false);

  // Maps the sonar ping to an RGB image.
  // rectImage is reshaped to be numBearings rows x numRanges columns.
  //
  // If rectImage is either 8UC3 or 32FC3, it retains that type, otherwise
  // rectImage is converted to 8UC3
  //
  // Cell (0,0) is the color mapping of the data with the smallest range and
  // smallest (typically, most negative) bearing in the ping.
  //
  // Cell (0,nRange-1) is the data at the max range, most negative bearing.
  //
  // Cell (nBearing-1,nRange-1) is the data at the max range, most positive
  // bearing.
  //
  cv::Mat drawRectSonarImage(const AbstractSonarInterface &ping,
                             const SonarColorMap &colorMap = InfernoColorMap(),
                             const cv::Mat &rectImage = cv::Mat(0, 0, CV_8UC3));

  cv::Mat remapRectSonarImage(const AbstractSonarInterface &ping,
                              const cv::Mat &rectImage);

  // Creates an operator-display copy of sonarImage with the graphical overlay
  // using the configuration in overlayConfig. The returned image includes a
  // small black OSD border: range labels sit beyond their arc markers and
  // bearing labels beyond their ray markers, rather than obscuring returns in
  // the fan. Use the clean fan image together with FanImageInfo for
  // pixel-to-metre work; the OSD image deliberately has a larger canvas.
  cv::Mat drawOverlay(const AbstractSonarInterface &ping,
                      const cv::Mat &sonarImage);

  OverlayConfig &overlayConfig() { return overlay_config_; }

 private:
  OverlayConfig overlay_config_;
  float pixels_per_meter_;
  float max_pixels_per_meter_;
  float max_range_;

  // Utility class which can generate and store the two cv::Mats
  // required for the cv::remap() function
  //
  // Also stores meta-information to determine if the map is
  // invalid and needs to be regenerated.
  struct Cached {
   public:
    Cached()
        : _rangeBounds(UnsetBounds),
          _azimuthBounds(UnsetBounds),
          _numRanges(0),
          _numAzimuth(0) {
      ;
    }

   protected:
    virtual bool isValid(const AbstractSonarInterface &ping) const;

    // Meta-information to validate map
    std::pair<float, float> _rangeBounds, _azimuthBounds;
    int _numRanges, _numAzimuth;
  };

  struct CachedMap {
   public:
    CachedMap() : _nextEvict(0) { ; }
    typedef std::pair<cv::Mat, cv::Mat> MapPair;

    MapPair operator()(const AbstractSonarInterface &ping, float pixelsPerMeter,
                       float maxRange);

   private:
    // The remap tables for one ping geometry.
    struct Entry : public Cached {
      Entry() : Cached(), _pixelsPerMeter(0.0f), _maxRange(0.0f) { ; }

      bool isValidFor(const AbstractSonarInterface &ping, float pixelsPerMeter,
                      float maxRange) const;
      void create(const AbstractSonarInterface &ping, float pixelsPerMeter,
                  float maxRange);

      cv::Mat _scMap1, _scMap2;
      float _pixelsPerMeter;
      float _maxRange;
      std::vector<float> _azimuths;
      std::vector<float> _ranges;
    };

    // Building a remap table is by far the most expensive thing here -- tens
    // of milliseconds at a wide-field-of-view ping size -- and a
    // dual-frequency head (e.g. the Oculus M3000d, 130 degrees at 1.2MHz vs
    // 40 degrees at 3.0MHz) alternates between exactly two geometries.
    // Holding both means switching frequency costs nothing after the first
    // ping in each mode.
    static const size_t kNumEntries = 2;

    std::array<Entry, kNumEntries> _entries;

    // Round-robin eviction.  With kNumEntries slots and kNumEntries
    // geometries in rotation, every ping after the first in each mode hits.
    size_t _nextEvict;
  } _map;

  struct CachedRectifiedMap {
   public:
    CachedRectifiedMap() : _nextEvict(0) { ; }
    typedef std::pair<cv::Mat, cv::Mat> MapPair;

    MapPair operator()(const AbstractSonarInterface &ping,
                       const RectifiedGeometry &geometry);

   private:
    struct Entry {
      bool isValidFor(const AbstractSonarInterface &ping,
                      const RectifiedGeometry &geometry) const;
      void create(const AbstractSonarInterface &ping,
                  const RectifiedGeometry &geometry);

      cv::Mat _map1, _map2;
      RectifiedGeometry _geometry;
      std::vector<float> _ranges;
      std::vector<float> _azimuths;
    };

    static const size_t kNumEntries = 2;
    std::array<Entry, kNumEntries> _entries;
    size_t _nextEvict;
  } _rectified_map;

  struct CachedOverlay : public Cached {
   public:
    CachedOverlay()
        : Cached(),
          _maxRange(0.0f),
          _source_size(0, 0),
          _image_origin(0, 0) { ; }

    const cv::Mat &operator()(const AbstractSonarInterface &ping,
                              const cv::Mat &sonarImage,
                              const OverlayConfig &config, float maxRange);

    // Location of the unannotated fan in the padded OSD canvas returned by
    // operator(). It is intentionally internal to SonarDrawer: external
    // consumers use drawn_sonar_clean + FanImageInfo, whose coordinates are
    // not changed by this display-only border.
    cv::Point imageOrigin() const { return _image_origin; }

   private:
    bool isValidFor(const AbstractSonarInterface &ping,
                    const cv::Mat &sonarImage, const OverlayConfig &config,
                    float maxRange) const;

    void create(const AbstractSonarInterface &ping, const cv::Mat &sonarImage,
                const OverlayConfig &config, float maxRange);

    cv::Mat _overlay;
    OverlayConfig _config_used;
    float _maxRange;
    cv::Size _source_size;
    cv::Point _image_origin;
  } _overlay;
};  // class SonarDrawer

}  // namespace sonar_image_proc
