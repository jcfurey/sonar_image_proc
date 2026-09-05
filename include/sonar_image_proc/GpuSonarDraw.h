// CUDA path for the sonar draw pipeline (DrawSonarComponent): the per-cell
// colormap over the polar image and the bicubic polar->Cartesian fan remap —
// per ping, the two costs of drawing (the CPU twin is a scalar Mat::at +
// virtual-lookup loop followed by cv::remap INTER_CUBIC).
//
// Contract: VISUALLY equivalent, not bit-parity — this is an operator-view
// image with no downstream numeric consumer. The colormap stage is exact
// (same 256-entry LUT the CPU maps evaluate); the remap differs from
// cv::remap only in sub-pixel precision (cv::remap quantizes fractional
// coordinates to 1/32 via its converted 16SC2 maps; the kernel evaluates the
// polar mapping per output pixel in full float — strictly finer). Same cubic
// kernel (A = -0.75) and BORDER_CONSTANT(0) semantics.
//
// Any device failure returns false; the caller runs the CPU path.
#pragma once

#include <cstdint>

namespace sonar_image_proc {
namespace gpu {

// built with CUDA, device present, SONAR_IMAGE_PROC_FORCE_CPU unset (cached)
bool available();

// fan canvas geometry — the same ceil/floor/sin math as
// SonarDrawer::CachedMap::create, host-only (no CUDA needed)
struct FanGeometry {
  int width = 0;
  int height = 0;
  int originx = 0;
};
FanGeometry fanGeometry(float max_range, float azimuth_min, float azimuth_max,
                        float pixels_per_meter);

// image: bin-major uint8 [range_bin * n_bearings + bearing] (the msg layout).
// ranges: n_ranges strictly increasing physical ranges (metres).
// azimuths: n_bearings per-beam bearings (radians, monotonic) — the REAL,
//           non-uniform Oculus beam table. The remap interpolates azimuth ->
//           fractional beam index against this (a uniform (az-min)/db mapping
//           bows straight walls into arcs; matches SonarDrawer.cpp).
// lut_rgb: 256 x 3 bytes, the colormap evaluated per intensity.
// rect_out: n_bearings rows x n_ranges cols x 3 (the CPU drawRectSonarImage
//           layout, ready for the rotate + rect publish).
// fan_out: geom.height rows x geom.width cols x 3, range 0 at the bottom
//          edge, azimuth 0 up (the CPU remapRectSonarImage output).
bool drawSonar(const uint8_t* image, int n_ranges, int n_bearings,
               const float* ranges, const float* azimuths,
               float pixels_per_meter, const uint8_t* lut_rgb,
               uint8_t* rect_out, const FanGeometry& geom, uint8_t* fan_out);

}  // namespace gpu
}  // namespace sonar_image_proc
