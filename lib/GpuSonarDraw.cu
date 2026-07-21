// CUDA sonar draw — see GpuSonarDraw.h for the contract and SonarDrawer.cpp
// for the CPU twin.
//
// Two kernels:
//   rect_lut_kernel — one thread per polar cell: rect RGB = LUT[raw byte].
//     Exact vs the CPU colormap loop (which is itself a 256-entry table
//     lookup for the deployed maps).
//   fan_cubic_kernel — one thread per output fan pixel: evaluates the
//     polar mapping (range/azimuth -> rect coords) inline — the same math
//     SonarDrawer::CachedMap::create bakes into its remap maps, minus the
//     1/32 fixed-point quantization of cv::convertMaps — then samples the
//     rect image bicubically (A = -0.75, cv::remap's kernel) with
//     BORDER_CONSTANT(0).
#include "sonar_image_proc/GpuSonarDraw.h"

#include <cuda_runtime.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace sonar_image_proc {
namespace gpu {

namespace {

bool check(cudaError_t err, const char* what)
{
  if (err == cudaSuccess) return true;
  std::fprintf(stderr,
               "sonar_image_proc CUDA error in %s: %s (falling back to CPU)\n",
               what, cudaGetErrorString(err));
  return false;
}

// grow-only device allocation reused across pings; never freed (the CUDA
// context reclaims at process exit)
class DeviceBuffer
{
public:
  bool ensure(std::size_t bytes, const char* what)
  {
    if (bytes <= capacity_) return true;
    if (ptr_ != nullptr) {
      cudaFree(ptr_);
      ptr_ = nullptr;
      capacity_ = 0;
    }
    if (!check(cudaMalloc(&ptr_, bytes), what)) {
      ptr_ = nullptr;
      return false;
    }
    capacity_ = bytes;
    return true;
  }

  template <typename T>
  T* as() const
  {
    return static_cast<T*>(ptr_);
  }

private:
  void* ptr_ = nullptr;
  std::size_t capacity_ = 0;
};

__constant__ std::uint8_t c_lut[256 * 3];

// rect layout matches drawRectSonarImage: row = bearing, col = range bin
__global__ void rect_lut_kernel(const std::uint8_t* __restrict__ img,
                                int n_ranges, int n_bearings,
                                std::uint8_t* __restrict__ rect)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_ranges * n_bearings) return;
  const int b = idx / n_ranges;   // bearing (rect row)
  const int r = idx - b * n_ranges;  // range bin (rect col)
  const std::uint8_t v = img[r * n_bearings + b];
  rect[3 * idx + 0] = c_lut[3 * v + 0];
  rect[3 * idx + 1] = c_lut[3 * v + 1];
  rect[3 * idx + 2] = c_lut[3 * v + 2];
}

// cv::remap's bicubic kernel (Catmull-Rom-like, A = -0.75)
__device__ inline void cubic_coeffs(float t, float* w)
{
  const float A = -0.75f;
  w[0] = ((A * (t + 1.f) - 5.f * A) * (t + 1.f) + 8.f * A) * (t + 1.f) - 4.f * A;
  w[1] = ((A + 2.f) * t - (A + 3.f)) * t * t + 1.f;
  w[2] = ((A + 2.f) * (1.f - t) - (A + 3.f)) * (1.f - t) * (1.f - t) + 1.f;
  w[3] = 1.f - w[0] - w[1] - w[2];
}

__global__ void fan_cubic_kernel(const std::uint8_t* __restrict__ rect,
                                 int n_ranges, int n_bearings, int out_w,
                                 int out_h, int originx, float ppm,
                                 float min_range, float max_range,
                                 const float* __restrict__ azimuths,
                                 std::uint8_t* __restrict__ fan)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= out_w * out_h) return;
  const int y = idx / out_w;
  const int x = idx - y * out_w;

  // the same pixel->polar math as SonarDrawer::CachedMap::create
  const float dx = x - originx;
  const float dy = out_h - y;
  const float range_px = sqrtf(dx * dx + dy * dy);
  const float azimuth = atan2f(dx, dy);
  const float range_m = range_px / ppm;

  std::uint8_t* out = fan + 3 * idx;
  if (range_m < min_range || range_m > max_range) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return;
  }
  const float xp = (range_m - min_range) / (max_range - min_range) * n_ranges;

  // azimuth -> fractional beam index against the REAL (non-uniform) bearing
  // table (matches SonarDrawer::CachedMap::create). A uniform (az-min)/db
  // mapping mislocates beams and bows straight walls into arcs.
  const float a0 = azimuths[0], a1 = azimuths[n_bearings - 1];
  const bool asc = a1 >= a0;
  const float lo_a = asc ? a0 : a1, hi_a = asc ? a1 : a0;
  if (azimuth < lo_a - 1e-6f || azimuth > hi_a + 1e-6f) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return;
  }
  int blo = 0, bhi = n_bearings - 1;
  while (bhi - blo > 1) {
    const int mid = (blo + bhi) >> 1;
    const bool left = asc ? (azimuths[mid] <= azimuth) : (azimuths[mid] >= azimuth);
    if (left) blo = mid; else bhi = mid;
  }
  const float denom = azimuths[bhi] - azimuths[blo];
  const float yp =
      blo + (fabsf(denom) > 1e-9f ? (azimuth - azimuths[blo]) / denom : 0.f);

  const int x0 = static_cast<int>(floorf(xp));
  const int y0 = static_cast<int>(floorf(yp));
  const float fx = xp - x0;
  const float fy = yp - y0;
  float wx[4], wy[4];
  cubic_coeffs(fx, wx);
  cubic_coeffs(fy, wy);

  float acc[3] = {0.f, 0.f, 0.f};
  for (int j = 0; j < 4; ++j) {
    const int sy = y0 - 1 + j;
    if (sy < 0 || sy >= n_bearings) continue;  // border constant 0
    for (int i = 0; i < 4; ++i) {
      const int sx = x0 - 1 + i;
      if (sx < 0 || sx >= n_ranges) continue;
      const float w = wx[i] * wy[j];
      const std::uint8_t* px = rect + 3 * (sy * n_ranges + sx);
      acc[0] += w * px[0];
      acc[1] += w * px[1];
      acc[2] += w * px[2];
    }
  }
  // cubic weights overshoot; saturate like cv::saturate_cast<uchar>
  for (int c = 0; c < 3; ++c) {
    const float v = nearbyintf(acc[c]);
    out[c] = static_cast<std::uint8_t>(v < 0.f ? 0.f : (v > 255.f ? 255.f : v));
  }
}

}  // namespace

bool available()
{
  static const bool ok = [] {
    if (std::getenv("SONAR_IMAGE_PROC_FORCE_CPU") != nullptr) return false;
    int n = 0;
    return cudaGetDeviceCount(&n) == cudaSuccess && n > 0;
  }();
  return ok;
}

FanGeometry fanGeometry(float max_range, float azimuth_min, float azimuth_max,
                        float pixels_per_meter)
{
  // identical to SonarDrawer::CachedMap::create's canvas math
  FanGeometry g;
  g.height = static_cast<int>(std::ceil(max_range * pixels_per_meter));
  const int minus_width =
    static_cast<int>(std::floor(g.height * std::sin(azimuth_min)));
  const int plus_width =
    static_cast<int>(std::ceil(g.height * std::sin(azimuth_max)));
  g.width = plus_width - minus_width;
  g.originx = std::abs(minus_width);
  return g;
}

bool drawSonar(const std::uint8_t* image, int n_ranges, int n_bearings,
               float min_range, float max_range, const float* azimuths,
               float pixels_per_meter, const std::uint8_t* lut_rgb,
               std::uint8_t* rect_out, const FanGeometry& geom,
               std::uint8_t* fan_out)
{
  if (n_ranges <= 0 || n_bearings <= 0 || azimuths == nullptr ||
      geom.width <= 0 || geom.height <= 0 || !(max_range > min_range))
    return false;

  static std::mutex mutex;
  static DeviceBuffer img_buf, rect_buf, fan_buf, az_buf;
  std::lock_guard<std::mutex> lock(mutex);

  const std::size_t n_cells = static_cast<std::size_t>(n_ranges) * n_bearings;
  const std::size_t n_fan = static_cast<std::size_t>(geom.width) * geom.height;
  if (!img_buf.ensure(n_cells, "gpu_draw image") ||
      !rect_buf.ensure(3 * n_cells, "gpu_draw rect") ||
      !fan_buf.ensure(3 * n_fan, "gpu_draw fan") ||
      !az_buf.ensure(static_cast<std::size_t>(n_bearings) * sizeof(float),
                     "gpu_draw azimuths"))
    return false;

  if (!check(cudaMemcpyToSymbol(c_lut, lut_rgb, 256 * 3), "gpu_draw lut") ||
      !check(cudaMemcpy(img_buf.as<std::uint8_t>(), image, n_cells,
                        cudaMemcpyHostToDevice),
             "gpu_draw image upload") ||
      !check(cudaMemcpy(az_buf.as<float>(), azimuths,
                        static_cast<std::size_t>(n_bearings) * sizeof(float),
                        cudaMemcpyHostToDevice),
             "gpu_draw azimuths upload"))
    return false;

  const int threads = 256;
  rect_lut_kernel<<<(static_cast<int>(n_cells) + threads - 1) / threads,
                    threads>>>(img_buf.as<std::uint8_t>(), n_ranges,
                               n_bearings, rect_buf.as<std::uint8_t>());

  fan_cubic_kernel<<<(static_cast<int>(n_fan) + threads - 1) / threads,
                     threads>>>(
    rect_buf.as<std::uint8_t>(), n_ranges, n_bearings, geom.width, geom.height,
    geom.originx, pixels_per_meter, min_range, max_range, az_buf.as<float>(),
    fan_buf.as<std::uint8_t>());
  if (!check(cudaGetLastError(), "gpu_draw launch")) return false;

  if (!check(cudaMemcpy(rect_out, rect_buf.as<std::uint8_t>(), 3 * n_cells,
                        cudaMemcpyDeviceToHost),
             "gpu_draw rect download") ||
      !check(cudaMemcpy(fan_out, fan_buf.as<std::uint8_t>(), 3 * n_fan,
                        cudaMemcpyDeviceToHost),
             "gpu_draw fan download"))
    return false;

  return true;
}

}  // namespace gpu
}  // namespace sonar_image_proc
