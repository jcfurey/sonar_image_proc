// Copyright 2026 ERDC

#pragma once

#include <cmath>
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

}  // namespace sonar_image_proc
