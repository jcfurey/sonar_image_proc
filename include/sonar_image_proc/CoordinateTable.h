// Copyright 2026 ERDC

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sonar_image_proc {

// Validate a physical coordinate table before any lower_bound/binary-search
// mapping consumes it. Both orders are supported because sonar producers use
// different beam conventions; duplicates, reversals and non-finite values are
// not, because they make interpolation ambiguous.
inline bool validateCoordinateTable(const std::vector<float> &samples,
                                    bool &ascending) {
  if (samples.size() < 2 || !std::isfinite(samples.front()) ||
      !std::isfinite(samples.back()) || samples.back() == samples.front()) {
    return false;
  }
  ascending = samples.back() > samples.front();
  for (std::size_t i = 1; i < samples.size(); ++i) {
    if (!std::isfinite(samples[i - 1]) || !std::isfinite(samples[i])) {
      return false;
    }
    if (ascending ? samples[i] <= samples[i - 1]
                  : samples[i] >= samples[i - 1]) {
      return false;
    }
  }
  return true;
}

// Interpolate a validated coordinate table without assuming uniform sampling.
inline float coordinateToIndex(const std::vector<float> &samples, float value,
                        bool ascending) {
  if (!std::isfinite(value)) return -1.0f;
  const float low = ascending ? samples.front() : samples.back();
  const float high = ascending ? samples.back() : samples.front();
  constexpr float kTolerance = 1e-6f;
  if (value < low - kTolerance || value > high + kTolerance) return -1.0f;
  value = std::clamp(value, low, high);

  size_t lo = 0;
  size_t hi = samples.size() - 1;
  while (hi - lo > 1) {
    const size_t mid = (lo + hi) / 2;
    const bool before =
        ascending ? samples[mid] <= value : samples[mid] >= value;
    if (before)
      lo = mid;
    else
      hi = mid;
  }

  const float span = samples[hi] - samples[lo];
  if (std::abs(span) <= std::numeric_limits<float>::epsilon()) return -1.0f;
  return static_cast<float>(lo) + (value - samples[lo]) / span;
}

}  // namespace sonar_image_proc
